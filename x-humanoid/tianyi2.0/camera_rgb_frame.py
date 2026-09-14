"""Read-only Orbbec RGB frames. Never starts services or commands head joints."""

import hashlib
import json
import math
import struct
import threading
import time

SCHEMA = "phanthy.sensor.camera_rgb_frame.v1"
FORMAT = "application/vnd.phanthy.sensor-envelope.v1"


def stamp_ns(header):
    sec, ns = header.stamp.sec, header.stamp.nanosec
    return sec * 1_000_000_000 + ns if sec >= 0 and 0 <= ns < 1_000_000_000 and (sec or ns) else None


def calibration(info, image):
    unavailable = {"available": False, "calibration_id": None,
                   "reason": "camera_info_unavailable", "base_to_camera": None}
    if info is None:
        return unavailable
    if (info.width, info.height, info.header.frame_id) != (image.width, image.height, image.header.frame_id):
        return dict(unavailable, reason="camera_info_mismatch")
    result = {"width": info.width, "height": info.height,
              "frame_id": info.header.frame_id, "distortion_model": info.distortion_model,
              "k": list(info.k), "d": list(info.d), "r": list(info.r), "p": list(info.p)}
    if (not info.distortion_model or len(result["k"]) != 9 or len(result["r"]) != 9
            or len(result["p"]) != 12 or not all(math.isfinite(v) for key in ("k", "d", "r", "p") for v in result[key])
            or result["k"][0] <= 0 or result["k"][4] <= 0 or result["k"][8] != 1):
        return dict(unavailable, reason="camera_info_invalid")
    digest = hashlib.sha256(json.dumps(result, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return dict(result, available=True, calibration_id="camera-info-sha256:" + digest,
                source="sensor_msgs/CameraInfo", base_to_camera=None)


def encode_frame(image, info, received_ns, clock_domain):
    """Preserve the upstream stamp, not an invented exposure timestamp."""
    import cv2
    import numpy as np

    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(image.encoding)
    if (channels is None or not image.header.frame_id or image.width < 1 or image.height < 1
            or image.width * image.height > 16_000_000
            or image.step < image.width * channels or len(image.data) != image.height * image.step):
        raise ValueError("invalid_image_layout")
    pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
    pixels = pixels[:, :image.width * channels].reshape(image.height, image.width, channels)
    if image.encoding == "rgb8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    ok, jpeg = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise ValueError("jpeg_encoding_failed")
    jpeg = jpeg.tobytes()
    source_ns = stamp_ns(image.header)
    meta = {"schema": SCHEMA,
            "header": {"stamp_ns": source_ns, "frame_id": image.header.frame_id},
            "timing": {"source_stamp_ns": source_ns, "driver_receive_stamp_ns": received_ns,
                       "timestamp_source": "upstream_ros_header", "clock_domain": clock_domain,
                       "available": source_ns is not None and clock_domain == "ros_system_time"},
            "image": {"encoding": "jpeg", "width": image.width, "height": image.height,
                      "payload_size": len(jpeg)}, "calibration": calibration(info, image)}
    metadata = json.dumps(meta, separators=(",", ":"), allow_nan=False).encode()
    if len(metadata) > 65536 or len(jpeg) + len(metadata) + 12 > 8 * 1024 * 1024:
        raise ValueError("frame_too_large")
    return struct.pack("<4sII", b"PSE1", len(metadata), len(jpeg)) + metadata + jpeg, meta


class CameraRgbFramePlugin:
    """Independent opt-in stream; existing camera_head remains untouched.

    Persistent subscriptions avoid the existing Tianyi re-subscription failure.
    stop joins the encoder, drops pending work and disconnects its bridge socket.
    Subscriptions are inert while stopped and destroyed on bundle shutdown.
    """

    def __init__(self, cfg, namespace, ros2):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from rclpy.validate_full_topic_name import validate_full_topic_name
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import UInt8MultiArray
        from bridged_publisher import BridgedPublisher

        self._clock_domain = cfg.get("source_clock_domain", "unverified")
        if self._clock_domain not in ("unverified", "ros_system_time"):
            raise ValueError("source_clock_domain must be unverified or ros_system_time")
        self._topic = cfg.get("topic", f"/{namespace}/camera/rgb_frame")
        image_topic = cfg.get("image_topic", "/ob_camera_head/color/image_raw")
        info_topic = cfg.get("camera_info_topic", "/ob_camera_head/color/camera_info")
        for topic in (self._topic, image_topic, info_topic):
            validate_full_topic_name(topic)
        self._max_age = float(cfg.get("max_age_sec", 0.5))
        if not math.isfinite(self._max_age) or not 0.05 <= self._max_age <= 2:
            raise ValueError("max_age_sec must be within 0.05..2")
        self._ros2 = ros2
        self._message_type = UInt8MultiArray
        self._lock = threading.Lock()
        self._lifecycle = threading.RLock()
        self._wake = threading.Event()
        self._running = False
        self._closed = False
        self._thread = None
        self._pending = None
        self._info = None
        self._last_stamp = None
        self._last_receive = None
        self._meta = None
        self._reason = "idle"
        self._counts = {"received": 0, "coalesced": 0, "submitted": 0, "rejected": 0, "errors": 0}
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                         depth=1, durability=DurabilityPolicy.VOLATILE)
        self._node = Node("tianyi_camera_rgb_frame", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._node)
        self._pub = BridgedPublisher(self._node, UInt8MultiArray, self._topic, qos, bounded_frame=True)
        self._subscriptions = [self._node.create_subscription(Image, image_topic, self._on_image, qos),
                               self._node.create_subscription(CameraInfo, info_topic, self._on_info, qos)]

    def _topics(self):
        return [{"topic": self._topic, "format": FORMAT, "ros_type": "std_msgs/msg/UInt8MultiArray",
                 "schema": SCHEMA, "qos": "RELIABLE + KEEP_LAST(depth=1) + VOLATILE"}]

    def get_tool(self):
        return {"name": "camera_rgb_frame", "type": "sensor",
                "description": "Orbbec RGB 自描述帧；保留源 header，未知时间/标定明确诊断，不启动相机服务",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info"]}}},
                "topic_out": self._topics()}

    def _on_info(self, msg):
        with self._lock:
            if self._running:
                self._info = msg

    def _on_image(self, msg):
        received_ns, mono = time.time_ns(), time.monotonic()
        with self._lock:
            if not self._running:
                return
            self._counts["received"] += 1
            source = stamp_ns(msg.header)
            if source is not None and self._last_stamp is not None and source <= self._last_stamp:
                self._counts["rejected"] += 1
                self._reason = "source_stamp_not_increasing"
                return
            if source is not None:
                self._last_stamp = source
            self._last_receive = mono
            if self._pending is not None:
                self._counts["coalesced"] += 1
            self._pending = (msg, self._info, received_ns, mono)
            self._wake.set()

    def start(self):
        with self._lifecycle:
            with self._lock:
                if self._closed:
                    raise RuntimeError("camera_frame_closed")
                if self._running:
                    return
                if self._thread is not None and self._thread.is_alive():
                    raise RuntimeError("camera_frame_stop_pending")
                self._running = True
                self._pending = self._info = self._last_stamp = self._meta = self._last_receive = None
                self._reason = "awaiting_image"
                self._wake.clear()
            self._thread = threading.Thread(target=self._encode_loop, name="tianyi-rgb-frame", daemon=True)
            self._thread.start()

    def stop(self):
        with self._lifecycle:
            with self._lock:
                self._running = False
                self._pending = self._info = self._meta = None
                self._wake.set()
            if self._thread is not None:
                self._thread.join(timeout=2)
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("camera_frame_stop_pending")
            self._pub.destroy()
            with self._lock:
                self._reason = "idle"

    def close(self):
        with self._lifecycle:
            self.stop()
            if not self._closed:
                self._closed = True
                self._ros2.executor_tianyi.remove_node(self._node)
                self._node.destroy_node()

    def _encode_loop(self):
        while True:
            self._wake.wait(0.1)
            with self._lock:
                if not self._running:
                    return
                item, self._pending = self._pending, None
                self._wake.clear()
            if item is None:
                continue
            try:
                msg, info, received, mono = item
                raw, meta = encode_frame(msg, info, received, self._clock_domain)
                now = time.time_ns()
                source = meta["header"]["stamp_ns"]
                if (time.monotonic() - mono > self._max_age
                        or (self._clock_domain == "ros_system_time" and source is not None
                            and not -50_000_000 <= now - source <= self._max_age * 1e9)):
                    raise ValueError("image_stale_or_future")
                with self._lock:
                    if not self._running:
                        return
                outgoing = self._message_type()
                outgoing.data = list(raw)
                self._pub.publish(outgoing)
                with self._lock:
                    self._counts["submitted"] += 1
                    self._meta = meta
                    self._reason = ("source_time_unavailable" if not meta["timing"]["available"] else
                                    "camera_info_unavailable" if not meta["calibration"]["available"] else "ok")
            except Exception as exc:
                with self._lock:
                    self._counts["errors"] += 1
                    self._reason = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
                    self._meta = None
                    count = self._counts["errors"]
                if count == 1 or count % 100 == 0:
                    print(f"[camera_rgb_frame] frame rejected ({type(exc).__name__}), errors={count}", flush=True)

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        elif action != "info":
            return {"state": "error", "error": "unknown_action"}
        with self._lock:
            age = None if self._last_receive is None else time.monotonic() - self._last_receive
            reason = self._reason
            if self._running and age is not None and age > self._max_age:
                reason = "image_stale"
            return {"state": "running" if self._running else "idle", "ready": self._running and reason == "ok",
                    "reason": reason, "receive_age_ms": None if age is None else round(age * 1000),
                    "counters": dict(self._counts), "base_to_camera_available": False,
                    "topic_out": self._topics()}
