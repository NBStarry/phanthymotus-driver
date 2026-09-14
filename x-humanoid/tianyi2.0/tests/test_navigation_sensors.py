"""Read-only conversion/HTTP checks; optional real ROS subprocess lifecycle test."""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import navigation_sensors as sensors


def odom_config():
    return {"odom_time_mode": "poll_estimate", "actual_speed_verified": True,
            "local_odom_verified": True, "covariance_source": "synthetic-test-only",
            "pose_variance": [0.1] * 6, "twist_variance": [0.2] * 6}


def scan_config():
    return {"source_clock_domain": "ros_system_time", "axes_verified": True,
            "range_min_m": 0.1, "range_max_m": 20.0, "angle_increment_rad": math.pi / 4,
            "base_to_laser": {"source": "synthetic-test-only", "translation": [0.2, 0.0, 0.3],
                              "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]}}


def record(payload, start=1_000_000_000, elapsed=10_000_000):
    return {"payload": payload, "request_unix_ns": start, "receive_unix_ns": start + elapsed,
            "request_monotonic_ns": start, "receive_monotonic_ns": start + elapsed}


def pose(x=0.1):
    return {"x": x, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}


SPEED = {"vx": 0.1, "vy": 0.0, "omega": 0.0}


class ConversionTests(unittest.TestCase):
    def test_midpoint_is_explicit_estimate_and_not_stop_feedback(self):
        conv = sensors.OdomConversion(odom_config())
        value = conv.convert(record(pose()), record(SPEED, start=1_020_000_000), 1_040_000_000)
        self.assertEqual(value["stamp_ns"], 1_005_000_000)
        self.assertEqual(value["timing"]["odom_time_mode"], "poll_estimate")
        self.assertIsNone(value["timing"]["source_age_sec"])
        self.assertIsNone(value["timing"]["source_stamp_ns"])
        self.assertNotIn("stop_confirmed", value)
        self.assertEqual(value["pose_covariance"][7], 0.1)

    def test_missing_sources_never_become_ready(self):
        for key in ("actual_speed_verified", "local_odom_verified", "covariance_source", "pose_variance"):
            cfg = odom_config()
            cfg.pop(key)
            with self.assertRaises(ValueError):
                sensors.OdomConversion(cfg)
        cfg = odom_config()
        cfg.pop("odom_time_mode")
        with self.assertRaisesRegex(ValueError, "source_time_unavailable"):
            sensors.OdomConversion(cfg)

    def test_timeout_and_wall_jump(self):
        with self.assertRaisesRegex(ValueError, "request_timeout"):
            sensors.midpoint(record({}, elapsed=101_000_000), 0.1)
        row = record({})
        row["receive_unix_ns"] += 100_000_000
        with self.assertRaisesRegex(ValueError, "clock_jump"):
            sensors.midpoint(row, 0.1)

    def test_reset_and_backward_stamp_latch_until_restart(self):
        for next_pose, start in ((pose(0), 1_050_000_000), (pose(2), 1_050_000_000), (pose(), 1_000_000_000)):
            conv = sensors.OdomConversion(odom_config())
            conv.convert(record(pose()), record(SPEED), 1_020_000_000)
            with self.assertRaisesRegex(ValueError, "relocalize_required"):
                conv.convert(record(next_pose, start), record(SPEED, start), start + 20_000_000)
            with self.assertRaisesRegex(ValueError, "relocalize_required"):
                conv.convert(record(pose(), 1_100_000_000), record(SPEED, 1_100_000_000), 1_120_000_000)

    def test_unchanged_pose_is_not_rejected_as_cache(self):
        conv = sensors.OdomConversion(odom_config())
        for start in (1_000_000_000, 1_050_000_000):
            value = conv.convert(record(pose(), start), record(SPEED, start), start + 20_000_000)
            self.assertIsNone(value["timing"]["source_age_sec"])

    def scan(self):
        return {"start_stamp_raw": 1_000_000, "end_stamp_raw": 999_999,
                "points_angle_distance_valid": [[0.0, 2.0, True], [0.01, 1.0, True],
                                                [math.pi / 2, 3.0, True]]}

    def test_scan_units_bins_negative_duration_and_dedup(self):
        conv = sensors.ScanConversion(scan_config())
        value = conv.convert(self.scan(), 1_010_000_000)
        self.assertEqual(value["stamp_ns"], 1_000_000_000)
        self.assertEqual(value["ranges"][4], 1.0)
        self.assertTrue(math.isinf(value["ranges"][0]))
        self.assertEqual(value["time_increment"], 0)
        self.assertEqual(value["scan_time"], 0)
        self.assertTrue(value["timing"]["end_before_start"])
        self.assertIsNone(conv.convert(self.scan(), 2_000_000_000))

    def test_scan_rejects_bad_layout_and_clock_reset(self):
        for points in ([[0, 1, True]], [[8, 1, True], [0, 1, True]],
                       [[0, float("nan"), True], [1, 1, True]], [[0, 1, True], [0, 2, True]]):
            scan = self.scan()
            scan["points_angle_distance_valid"] = points
            with self.assertRaises(ValueError):
                sensors.ScanConversion(scan_config()).convert(scan, 1_010_000_000)
        conv = sensors.ScanConversion(scan_config())
        conv.convert(self.scan(), 1_010_000_000)
        old = self.scan()
        old["start_stamp_raw"] -= 1
        with self.assertRaisesRegex(ValueError, "restart_required"):
            conv.convert(old, 1_010_000_000)
        with self.assertRaisesRegex(ValueError, "restart_required"):
            conv.convert(self.scan(), 1_010_000_000)

    def test_no_unverified_tf_or_clock(self):
        cfg = scan_config()
        cfg["base_to_laser"] = None
        with self.assertRaisesRegex(ValueError, "extrinsics_unavailable"):
            sensors.validate_config("navigation_lidar_2d", cfg)
        cfg = scan_config()
        cfg["source_clock_domain"] = "not_ros_system_time"
        with self.assertRaises(ValueError):
            sensors.validate_config("navigation_lidar_2d", cfg)

    def test_tool_contract_and_missing_config_start(self):
        for kind in ("navigation_lidar_2d", "navigation_odom"):
            plugin = sensors.NavigationSensorPlugin(kind, {}, "ubuntu")
            tool = plugin.get_tool()
            self.assertEqual(set(tool["inputSchema"]["x-action-params"]), {"start", "stop", "info"})
            self.assertEqual(plugin.dispatch("info", {})["state"], "idle")
            self.assertFalse(plugin.dispatch("start", {})["ready"])
            self.assertIsNone(plugin.process)
            plugin.stop()
            plugin.stop()


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.requests = []
        self.delay = 0
        self.code = 200
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                owner.requests.append(self.path)
                time.sleep(owner.delay)
                body = pose() if self.path == sensors.ENDPOINTS["odometry_pose"] else SPEED
                encoded = json.dumps(body).encode()
                self.send_response(owner.code)
                self.send_header("Location", "/must-not-follow")
                self.end_headers()
                try:
                    self.wfile.write(encoded)
                except BrokenPipeError:
                    pass
            def log_message(self, *args):
                pass
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = "http://127.0.0.1:" + str(self.server.server_port)
        self.opener = sensors.urllib.request.build_opener(sensors.urllib.request.ProxyHandler({}), sensors.NoRedirect())

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_actual_http_read_is_bounded_and_readonly(self):
        p = sensors.read_http(self.opener, self.url, "odometry_pose", 0.1)
        s = sensors.read_http(self.opener, self.url, "speed", 0.1)
        value = sensors.OdomConversion(odom_config()).convert(p, s, time.time_ns())
        self.assertEqual(value["position"][0], 0.1)
        self.assertEqual(self.requests, [sensors.ENDPOINTS["odometry_pose"], sensors.ENDPOINTS["speed"]])

    def test_http_timeout_and_redirect_fail(self):
        self.code = 302
        with self.assertRaises(OSError):
            sensors.read_http(self.opener, self.url, "speed", 0.1)
        self.assertEqual(len(self.requests), 1)
        self.code, self.delay = 200, 0.15
        with self.assertRaises((OSError, ValueError)):
            sensors.read_http(self.opener, self.url, "speed", 0.03)

    @unittest.skipUnless(os.environ.get("TIANYI_ROS_TEST") == "1", "explicit isolated ROS environment required")
    def test_real_ros_odom_tf_and_restart(self):
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        rclpy.init(domain_id=93)
        node = Node("navigation_sensor_test")
        messages, transforms, statuses = [], [], []
        node.create_subscription(Odometry, "/fixture/navigation/odom", messages.append, 5)
        node.create_subscription(TFMessage, "/tf", transforms.append, 10)
        node.create_subscription(String, "/fixture/navigation/odom/status", statuses.append, 1)
        cfg = dict(odom_config(), base_url=self.url, ros_domain_id=93,
                   dds_profile=os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"])
        plugin = sensors.NavigationSensorPlugin("navigation_odom", cfg, "fixture")
        def wait_for(predicate):
            deadline = time.monotonic() + 6
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(predicate(), plugin.dispatch("info", {}))
        try:
            for _ in range(2):
                start_count = len(messages)
                plugin.dispatch("start", {})
                wait_for(lambda: len(messages) >= start_count + 3 and len(transforms) > 0)
                self.assertEqual(messages[-1].pose.pose.position.x, 0.1)
                self.assertEqual(messages[-1].header.frame_id, "odom")
                self.assertEqual(messages[-1].child_frame_id, "base_link")
                wait_for(lambda: bool(statuses))
                self.assertIsNone(json.loads(statuses[-1].data)["source_age_sec"])
                process = plugin.process
                plugin.dispatch("stop", {})
                self.assertIsNotNone(process.poll())
                self.assertFalse(plugin.monitor.is_alive())
                self.assertEqual(plugin.dispatch("info", {})["state"], "idle")
            # Shutdown the producer: no fresh data or optimistic readiness.
            plugin.start()
            wait_for(lambda: plugin.dispatch("info", {}).get("ready", False))
            self.delay = 0.2
            wait_for(lambda: not plugin.dispatch("info", {}).get("ready", False))
        finally:
            plugin.stop()
            node.destroy_node()
            rclpy.shutdown()


@unittest.skipUnless(os.environ.get("TIANYI_ROS_TEST") == "1", "explicit isolated ROS environment required")
class ScanRosTests(unittest.TestCase):
    def test_sdk_pipe_scan_qos_late_static_tf_and_restart(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        rclpy.init(domain_id=94)
        node = Node("navigation_scan_test")
        scans, transforms = [], []
        node.create_subscription(LaserScan, "/fixture/navigation/scan", scans.append, 2)
        cfg = dict(scan_config(), ros_domain_id=94, sdk_host="synthetic",
                   sdk_reader_path=str(Path(__file__).parent / "scan_sdk_fixture.py"),
                   dds_profile=os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"])
        plugin = sensors.NavigationSensorPlugin("navigation_lidar_2d", cfg, "fixture")
        def wait_for(predicate):
            deadline = time.monotonic() + 6
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(predicate(), plugin.dispatch("info", {}))
        try:
            plugin.start()
            wait_for(lambda: len(scans) >= 3)
            # Join after publication: static TF must be transient local.
            node.create_subscription(TFMessage, "/tf_static", transforms.append, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            wait_for(lambda: bool(transforms) and plugin.dispatch("info", {}).get("duplicates", 0) > 0)
            tf = transforms[-1].transforms[0]
            self.assertEqual((tf.header.frame_id, tf.child_frame_id), ("base_link", "laser"))
            self.assertAlmostEqual(tf.transform.translation.x, 0.2)
            endpoints = node.get_publishers_info_by_topic("/fixture/navigation/scan")
            self.assertTrue(any(e.qos_profile.reliability == ReliabilityPolicy.RELIABLE for e in endpoints))
            # FastDDS discovery does not expose remote history depth reliably;
            # verify it from the actual publisher handle, not the advertised tool.
            self.assertEqual(plugin.dispatch("info", {})["publisher_qos"],
                             {"depth": 2, "reliability": "RELIABLE", "durability": "VOLATILE"})
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in scans]
            self.assertEqual(len(stamps), len(set(stamps)))
            self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
            self.assertTrue(all(abs(r - 2) < 0.001 for r in scans[-1].ranges if math.isfinite(r)))
            process = plugin.process
            plugin.stop()
            self.assertIsNotNone(process.poll())
            with self.assertRaises(ProcessLookupError):
                os.killpg(process.pid, 0)
            count = len(scans)
            plugin.start()
            wait_for(lambda: len(scans) >= count + 3 and plugin.dispatch("info", {}).get("ready", False))
        finally:
            plugin.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
