"""Opt-in, read-only planar sensors. No chassis action or stop RPC lives here."""

import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import threading
import time
import urllib.request

from navigation_probe import (ENDPOINTS, MAX_RESPONSE_BYTES, NoRedirect,
                              describe, reject_constant, validate_base_url)


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("invalid_number")
    return float(value)


def positive(cfg, key, default=None):
    value = finite(cfg.get(key, default))
    if value <= 0:
        raise ValueError("invalid_" + key)
    return value


def frame(value):
    if not isinstance(value, str) or not value or value.startswith("/") or any(c.isspace() for c in value):
        raise ValueError("invalid_frame")
    return value


def midpoint(record, timeout_sec):
    keys = ("request_unix_ns", "receive_unix_ns", "request_monotonic_ns", "receive_monotonic_ns")
    if any(type(record.get(k)) is not int or record[k] <= 0 for k in keys):
        raise ValueError("request_time_unavailable")
    wall = record[keys[1]] - record[keys[0]]
    elapsed = record[keys[3]] - record[keys[2]]
    if elapsed < 0 or elapsed > timeout_sec * 1e9:
        raise ValueError("request_timeout")
    if wall < 0 or abs(wall - elapsed) > 5_000_000:
        raise ValueError("request_clock_jump")
    return (record[keys[0]] + record[keys[1]]) // 2


def covariance(cfg, key):
    values = cfg.get(key)
    if not cfg.get("covariance_source") or not isinstance(values, list) or len(values) != 6:
        raise ValueError("covariance_unavailable")
    result = [0.0] * 36
    for i, value in enumerate(values):
        result[i * 7] = finite(value)
        if result[i * 7] <= 0:
            raise ValueError("covariance_invalid")
    return result


class OdomConversion:
    def __init__(self, cfg):
        if cfg.get("odom_time_mode", "source") != "poll_estimate":
            raise ValueError("odom_source_time_unavailable")
        if cfg.get("actual_speed_verified") is not True or cfg.get("local_odom_verified") is not True:
            raise ValueError("odom_feedback_semantics_unverified")
        self.timeout = positive(cfg, "request_timeout_sec", 0.1)
        self.max_age = positive(cfg, "max_age_sec", 0.5)
        self.max_jump = positive(cfg, "max_pose_jump_m", 0.5)
        self.max_yaw_jump = positive(cfg, "max_yaw_jump_rad", 0.5)
        self.pose_cov = covariance(cfg, "pose_variance")
        self.twist_cov = covariance(cfg, "twist_variance")
        self.odom_frame = frame(cfg.get("odom_frame", "odom"))
        self.base_frame = frame(cfg.get("base_frame", "base_link"))
        if self.odom_frame == self.base_frame:
            raise ValueError("duplicate_frame")
        self.previous = None
        self.blocked = None

    def convert(self, pose, speed, now_ns):
        if self.blocked:
            raise ValueError(self.blocked)
        try:
            stamp = midpoint(pose, self.timeout)
            speed_stamp = midpoint(speed, self.timeout)
        except ValueError as exc:
            if str(exc) == "request_clock_jump":
                self.blocked = "odom_clock_jump_relocalize_required"
            raise
        if any(not 0 <= now_ns - s <= self.max_age * 1e9 for s in (stamp, speed_stamp)):
            raise ValueError("odom_read_stale_or_future")
        p = describe("odometry_pose", pose["payload"])["values"]
        v = describe("speed", speed["payload"])["values"]
        if abs(p["pitch"]) > 0.05 or abs(p["roll"]) > 0.05:
            raise ValueError("non_planar_odom")
        if self.previous:
            old_stamp, old = self.previous
            distance = math.hypot(p["x"] - old["x"], p["y"] - old["y"])
            yaw = abs(math.remainder(p["yaw"] - old["yaw"], 2 * math.pi))
            reset = (p["x"] == p["y"] == p["yaw"] == 0 and
                     (old["x"] != 0 or old["y"] != 0 or old["yaw"] != 0))
            if stamp <= old_stamp or distance > self.max_jump or yaw > self.max_yaw_jump or reset:
                self.blocked = "odom_discontinuity_relocalize_required"
                raise ValueError(self.blocked)
        self.previous = stamp, p
        cr, sr = math.cos(p["roll"] / 2), math.sin(p["roll"] / 2)
        cp, sp = math.cos(p["pitch"] / 2), math.sin(p["pitch"] / 2)
        cy, sy = math.cos(p["yaw"] / 2), math.sin(p["yaw"] / 2)
        return {"stamp_ns": stamp, "frame_id": self.odom_frame, "child_frame_id": self.base_frame,
                "position": [p["x"], p["y"], p["z"]],
                "orientation": [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                                cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy],
                "velocity": [v["vx"], v["vy"], v["omega"]],
                "pose_covariance": self.pose_cov, "twist_covariance": self.twist_cov,
                "timing": {"odom_time_mode": "poll_estimate", "source_age_sec": None,
                           "source_stamp_ns": None, "speed_estimate_stamp_ns": speed_stamp,
                           "pose_request": {k: pose[k] for k in pose if k != "payload"},
                           "speed_request": {k: speed[k] for k in speed if k != "payload"}}}


class ScanConversion:
    def __init__(self, cfg):
        if cfg.get("source_clock_domain") != "ros_system_time" or cfg.get("axes_verified") is not True:
            raise ValueError("scan_time_or_axes_unverified")
        self.frame = frame(cfg.get("laser_frame", "laser"))
        self.range_min = positive(cfg, "range_min_m")
        self.range_max = positive(cfg, "range_max_m")
        self.angle_min = finite(cfg.get("angle_min_rad", -math.pi))
        self.angle_max = finite(cfg.get("angle_max_rad", math.pi))
        self.increment = positive(cfg, "angle_increment_rad")
        self.max_age = positive(cfg, "max_age_sec", 0.5)
        if self.range_max <= self.range_min or not 0 < self.angle_max - self.angle_min <= 2 * math.pi:
            raise ValueError("invalid_scan_limits")
        self.count = math.floor((self.angle_max - self.angle_min) / self.increment) + 1
        if not 2 <= self.count <= 20000:
            raise ValueError("invalid_scan_bin_count")
        self.previous = None
        self.blocked = False

    def convert(self, record, now_ns):
        if self.blocked:
            raise ValueError("scan_clock_reset_restart_required")
        start, end = record.get("start_stamp_raw"), record.get("end_stamp_raw")
        if type(start) is not int or start <= 0 or type(end) is not int or end < 0:
            raise ValueError("scan_time_unavailable")
        if self.previous is not None:
            if start < self.previous:
                self.blocked = True
                raise ValueError("scan_clock_reset_restart_required")
            if start == self.previous:
                return None
        self.previous = start
        stamp = start * 1000
        if not 0 <= now_ns - stamp <= self.max_age * 1e9:
            raise ValueError("scan_source_stamp_stale_or_future")
        points = record.get("points_angle_distance_valid")
        if not isinstance(points, list) or not 2 <= len(points) <= 100000:
            raise ValueError("invalid_scan_point_count")
        ranges = [math.inf] * self.count
        valid = 0
        for point in points:
            if not isinstance(point, list) or len(point) != 3 or type(point[2]) is not bool:
                raise ValueError("invalid_scan_point")
            angle, distance = finite(point[0]), finite(point[1])
            if not -math.pi <= angle <= math.pi or distance < 0:
                raise ValueError("invalid_scan_angle_or_distance")
            if not point[2] or not self.range_min <= distance <= self.range_max:
                continue
            index = round((angle - self.angle_min) / self.increment)
            if 0 <= index < self.count and self.angle_min <= angle <= self.angle_max:
                ranges[index] = min(ranges[index], distance)
                valid += 1
        if valid < 2 or sum(math.isfinite(r) for r in ranges) < 2:
            raise ValueError("insufficient_scan_returns")
        # SDK points need not be equiangular or in acquisition order. Bin nearest
        # returns; do not invent per-ray timing after reordering them.
        return {"stamp_ns": stamp, "frame_id": self.frame, "angle_min": self.angle_min,
                "angle_max": self.angle_min + (self.count - 1) * self.increment,
                "angle_increment": self.increment, "ranges": ranges,
                "range_min": self.range_min, "range_max": self.range_max,
                "scan_time": 0.0, "time_increment": 0.0,
                "timing": {"source_age_sec": (now_ns - stamp) / 1e9,
                           "start_stamp_raw": start, "end_stamp_raw": end,
                           "end_before_start": end < start, "scan_duration_available": False}}


def read_http(opener, url, name, timeout):
    record = {"request_unix_ns": time.time_ns(), "request_monotonic_ns": time.monotonic_ns()}
    with opener.open(urllib.request.Request(url + ENDPOINTS[name], method="GET"), timeout=timeout) as response:
        raw = response.read(MAX_RESPONSE_BYTES + 1)
    record.update(receive_unix_ns=time.time_ns(), receive_monotonic_ns=time.monotonic_ns())
    record["request_duration_ms"] = (record["receive_monotonic_ns"] - record["request_monotonic_ns"]) / 1e6
    midpoint(record, timeout)
    if len(raw) > MAX_RESPONSE_BYTES:
        raise ValueError("response_too_large")
    record["payload"] = json.loads(raw, parse_constant=reject_constant)
    describe(name, record["payload"])
    return record


class SdkScanReader:
    """Read-only SDK executable. Bounded lines, no shell, kill/reap on stop."""
    def __init__(self, cfg):
        executable = Path(cfg["sdk_reader_path"])
        if not executable.is_absolute() or not executable.is_file() or not os.access(executable, os.X_OK):
            raise ValueError("sdk_reader_unavailable")
        port = cfg.get("sdk_port", 1445)
        if type(port) is not int or not 1 <= port <= 65535 or not cfg.get("sdk_host"):
            raise ValueError("sdk_endpoint_unavailable")
        self.process = subprocess.Popen([str(executable), cfg["sdk_host"], str(port), "--stream-scan"],
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        self.buffer = b""

    def read(self, timeout=0.1):
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self.selector.select(remaining):
                raise TimeoutError("scan_read_timeout")
            data = os.read(self.process.stdout.fileno(), 65536)
            if not data:
                raise EOFError("sdk_reader_exited")
            self.buffer += data
            if len(self.buffer) > MAX_RESPONSE_BYTES:
                raise ValueError("scan_response_too_large")
        line, self.buffer = self.buffer.split(b"\n", 1)
        record = json.loads(line, parse_constant=reject_constant)
        if not isinstance(record, dict) or record.get("endpoint") != "raw_scan" or "error" in record:
            raise ValueError("sdk_read_failed")
        return record

    def close(self):
        self.selector.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(0.5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process.wait()
        self.process.stdout.close()


class RosOutput:
    def __init__(self, kind, cfg):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

        rclpy.init(domain_id=cfg.get("ros_domain_id", 42))
        self.node = Node("tianyi_" + kind)
        self.kind = kind
        self.message_type = LaserScan if kind == "navigation_lidar_2d" else Odometry
        self.pub = self.node.create_publisher(self.message_type, cfg["topic"],
                    QoSProfile(depth=2 if kind == "navigation_lidar_2d" else 5,
                               reliability=ReliabilityPolicy.RELIABLE))
        self.status_pub = self.node.create_publisher(String, cfg["topic"] + "/status", 1)
        self.tf = TransformBroadcaster(self.node) if kind == "navigation_odom" else None
        self.static = None
        if kind == "navigation_lidar_2d":
            self.static = StaticTransformBroadcaster(self.node, qos=QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            transform = cfg["base_to_laser"]
            msg = self.transform(time.time_ns(), cfg.get("base_frame", "base_link"),
                                 cfg.get("laser_frame", "laser"), transform["translation"], transform["rotation_xyzw"])
            self.static.sendTransform(msg)

    @staticmethod
    def transform(stamp, parent, child, position, rotation):
        from geometry_msgs.msg import TransformStamped
        msg = TransformStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, position)
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = map(float, rotation)
        return msg

    def publish(self, data):
        msg = self.message_type()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(data["stamp_ns"], 1_000_000_000)
        msg.header.frame_id = data["frame_id"]
        if self.kind == "navigation_lidar_2d":
            for key in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max",
                        "scan_time", "time_increment", "ranges"):
                setattr(msg, key, data[key])
        else:
            msg.child_frame_id = data["child_frame_id"]
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = data["position"]
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = data["orientation"]
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = data["velocity"]
            msg.pose.covariance, msg.twist.covariance = data["pose_covariance"], data["twist_covariance"]
            self.tf.sendTransform(self.transform(data["stamp_ns"], data["frame_id"], data["child_frame_id"],
                                                  data["position"], data["orientation"]))
        self.pub.publish(msg)

    def status(self, status):
        from std_msgs.msg import String
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def close(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()


def validate_config(kind, cfg):
    if kind not in ("navigation_odom", "navigation_lidar_2d"):
        raise ValueError("unknown_sensor")
    if positive(cfg, "poll_hz", 20) > 100 or positive(cfg, "max_age_sec", 0.5) > 2:
        raise ValueError("sensor_rate_or_age_out_of_range")
    if positive(cfg, "request_timeout_sec", 0.1) > 1:
        raise ValueError("request_timeout_out_of_range")
    if kind == "navigation_odom":
        validate_base_url(cfg["base_url"])
        return OdomConversion(cfg)
    conversion = ScanConversion(cfg)
    transform = cfg.get("base_to_laser")
    if not isinstance(transform, dict) or not transform.get("source"):
        raise ValueError("laser_extrinsics_unavailable")
    p, q = transform.get("translation"), transform.get("rotation_xyzw")
    if not isinstance(p, list) or len(p) != 3 or not isinstance(q, list) or len(q) != 4:
        raise ValueError("laser_extrinsics_invalid")
    values = [finite(v) for v in p + q]
    if abs(sum(v*v for v in values[3:]) - 1) > 1e-6:
        raise ValueError("laser_rotation_not_unit")
    if frame(cfg.get("base_frame", "base_link")) == conversion.frame:
        raise ValueError("duplicate_frame")
    return conversion


def worker(kind, cfg):
    from common import logsafe
    logsafe.install(check_fd=False)
    # Fresh interpreter, before importing rclpy; never inherits vendor FastDDS profile.
    profile = cfg.get("dds_profile", "/opt/phanthy-motus/dds-local.xml")
    if not Path(profile).is_file():
        raise ValueError("dds_profile_unavailable")
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = profile
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    parent = os.getppid()
    output = reader = None
    status = {"ready": False, "reason": "starting", "published": 0, "duplicates": 0, "errors": 0,
              "poll_cycles": 0, "successful_reads": 0, "timeouts": 0,
              "odom_time_mode": cfg.get("odom_time_mode", "source"), "source_age_sec": None}
    last = last_source = None
    try:
        conversion = validate_config(kind, cfg)
        output = RosOutput(kind, cfg)
        actual_qos = output.pub.qos_profile
        status["publisher_qos"] = {"depth": actual_qos.depth, "reliability": actual_qos.reliability.name,
                                   "durability": actual_qos.durability.name}
        if kind == "navigation_lidar_2d":
            reader = SdkScanReader(cfg)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
        period = 1 / positive(cfg, "poll_hz", 20)
        while not stop.is_set() and os.getppid() == parent:
            cycle = time.monotonic()
            status["poll_cycles"] += 1
            try:
                if reader:
                    record = reader.read()
                    status["successful_reads"] += 1
                    data = None if record is None else conversion.convert(record, time.time_ns())
                    if data is None and record is not None:
                        status["duplicates"] += 1
                else:
                    pose = read_http(opener, cfg["base_url"], "odometry_pose", conversion.timeout)
                    status["successful_reads"] += 1
                    speed = read_http(opener, cfg["base_url"], "speed", conversion.timeout)
                    status["successful_reads"] += 1
                    data = conversion.convert(pose, speed, time.time_ns())
                if data is not None and not stop.is_set():
                    output.publish(data)
                    last = time.monotonic()
                    last_source = data["stamp_ns"]
                    status.update(data["timing"])
                    status.update(ready=True, reason="ok", published=status["published"] + 1)
            except (ValueError, OSError) as exc:
                reason = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
                status.update(ready=False, reason=reason, errors=status["errors"] + 1)
                if isinstance(exc, TimeoutError) or reason == "request_timeout":
                    status["timeouts"] += 1
                if not reader and (isinstance(exc, OSError) or reason in ("request_clock_jump", "request_timeout")):
                    # Timeout/reconnect can hide an odometry service reset.
                    conversion.blocked = "odom_read_interrupted_relocalize_required"
            age = None if last is None else time.monotonic() - last
            status["output_age_sec"] = age
            if reader and last_source is not None:
                status["source_age_sec"] = (time.time_ns() - last_source) / 1e9
                if not 0 <= status["source_age_sec"] <= conversion.max_age:
                    status.update(ready=False, reason="scan_source_stamp_stale_or_future")
            if age is None or age > conversion.max_age:
                status["ready"] = False
                if status["reason"] == "ok":
                    status["reason"] = "sensor_stale"
            status["report_monotonic"] = time.monotonic()
            output.status(status)
            print(json.dumps(status, allow_nan=False), flush=True)
            stop.wait(max(0, period - (time.monotonic() - cycle)))
    finally:
        if reader:
            reader.close()
        if output:
            output.close()


class NavigationSensorPlugin:
    """One independently stoppable subprocess per tool; MCP info is read-only."""
    def __init__(self, kind, cfg, namespace):
        self.kind, self.cfg = kind, dict(cfg)
        suffix = "scan" if kind == "navigation_lidar_2d" else "odom"
        self.cfg.setdefault("topic", f"/{namespace}/navigation/{suffix}")
        self.process = None
        self.monitor = None
        self.lock = threading.RLock()
        self.snapshot = {"ready": False, "reason": "idle"}

    def topics(self):
        scan = self.kind == "navigation_lidar_2d"
        return [{"topic": self.cfg["topic"], "port": "scan" if scan else "odom",
                 "ros_type": "sensor_msgs/msg/LaserScan" if scan else "nav_msgs/msg/Odometry",
                 "format": "sensor/lidar" if scan else "data/json",
                 "qos": f"RELIABLE + KEEP_LAST(depth={2 if scan else 5}) + VOLATILE"},
                {"topic": self.cfg["topic"] + "/status", "port": "status", "format": "data/json",
                 "ros_type": "std_msgs/msg/String", "qos": "RELIABLE + KEEP_LAST(depth=1) + VOLATILE"}]

    def get_tool(self):
        return {"name": self.kind, "type": "sensor", "description": "二维导航只读传感器；未核实时间/外参时拒绝启动",
                "inputSchema": {"type": "object", "properties": {"action": {
                    "type": "string", "enum": ["start", "stop", "info"]}},
                    "x-action-params": {a: {"params": []} for a in ("start", "stop", "info")}},
                "topic_out": self.topics()}

    def start(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return
            self.stop()
            validate_config(self.kind, self.cfg)
            encoded = json.dumps(self.cfg, allow_nan=False).encode() + b"\n"
            if len(encoded) > 65535:
                raise ValueError("sensor_config_too_large")
            self.snapshot = {"ready": False, "reason": "starting"}
            self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), self.kind],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                start_new_session=True)
            try:
                self.process.stdin.write(encoded)
                self.process.stdin.close()
            except OSError:
                self.stop()
                raise
            self.monitor = threading.Thread(target=self._monitor, args=(self.process,), daemon=True)
            self.monitor.start()

    def _monitor(self, process):
        # Only this thread writes snapshots; replacing a dict is atomic. No lock
        # while stop joins this thread, so lifecycle operations cannot deadlock.
        for line in process.stdout:
            try:
                value = json.loads(line)
                if isinstance(value, dict) and "ready" in value and self.process is process:
                    self.snapshot = value
            except (ValueError, UnicodeError):
                continue
        process.stdout.close()

    def stop(self):
        with self.lock:
            process = self.process
            if process is not None:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(2)
                # Also reap a reader if the worker crashed before its finally.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                if self.monitor:
                    self.monitor.join(2)
                    if self.monitor.is_alive():
                        raise RuntimeError("sensor_stop_pending")
            self.process = None
            self.snapshot = {"ready": False, "reason": "idle"}

    close = stop

    def dispatch(self, action, args):
        try:
            if action == "start":
                self.start()
            elif action == "stop":
                self.stop()
            elif action != "info":
                raise ValueError("unknown_action")
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            return {"state": "error", "ready": False, "error": str(exc), "topic_out": self.topics()}
        with self.lock:
            value = dict(self.snapshot)
            running = self.process is not None and self.process.poll() is None
            if self.process is not None and not running:
                value.update(ready=False, reason="sensor_worker_exited")
            elif running and time.monotonic() - value.get("report_monotonic", 0) > 0.5:
                value.update(ready=False, reason="sensor_worker_stale")
            return dict(value, state="running" if running else "idle", topic_out=self.topics())


if __name__ == "__main__":
    try:
        worker(sys.argv[1], json.loads(sys.stdin.buffer.readline(65536)))
    except Exception as exc:
        print(json.dumps({"ready": False, "reason": "sensor_worker_failed", "error_type": type(exc).__name__}), flush=True)
        raise SystemExit(1)
