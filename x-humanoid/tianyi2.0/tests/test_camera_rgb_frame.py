"""Run in ROS Humble with cv2/numpy; all messages are synthetic, no robot."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import UInt8MultiArray
except ImportError as exc:
    raise unittest.SkipTest("Real wire tests require ROS Humble, cv2 and numpy") from exc
import camera_rgb_frame as frame
from bridged_publisher import BridgedPublisher
from socket_bridge import SocketBridgeServer, TopicHandler


def sample(source=None):
    image = Image()
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(
        time.time_ns() if source is None else source, 1_000_000_000)
    image.header.frame_id = "camera_color_optical_frame"
    image.width, image.height, image.step, image.encoding = 8, 4, 26, "rgb8"
    # Red pixels and two padding bytes per row.
    image.data = list((bytes([255, 0, 0]) * 8 + b"\xff\xff") * 4)
    info = CameraInfo()
    info.header.frame_id = image.header.frame_id
    info.width, info.height = image.width, image.height
    info.distortion_model = "plumb_bob"
    info.k = [100., 0., 4., 0., 101., 2., 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [100., 0., 4., 0., 0., 101., 2., 0., 0., 0., 1., 0.]
    return image, info


def decode(raw):
    magic, metadata_size, image_size = struct.unpack_from("<4sII", raw)
    assert magic == b"PSE1" and len(raw) == 12 + metadata_size + image_size
    meta = json.loads(raw[12:12 + metadata_size])
    assert meta["image"]["payload_size"] == image_size
    pixels = cv2.imdecode(np.frombuffer(raw[12 + metadata_size:], dtype=np.uint8), cv2.IMREAD_COLOR)
    assert pixels.shape[:2] == (meta["image"]["height"], meta["image"]["width"])
    return meta, pixels


class FrameTests(unittest.TestCase):
    def test_wire_and_missing_metadata(self):
        image, info = sample()
        raw, meta = frame.encode_frame(image, info, 123, "ros_system_time")
        decoded, pixels = decode(raw)
        self.assertEqual(decoded, meta)
        self.assertGreater(int(pixels[0, 0, 2]), 240)
        self.assertLess(int(pixels[0, 0, 0]), 10)
        self.assertEqual(meta["header"]["stamp_ns"], frame.stamp_ns(image.header))
        self.assertEqual(meta["timing"]["driver_receive_stamp_ns"], 123)
        self.assertEqual(meta["calibration"]["k"], list(info.k))
        self.assertIsNone(meta["calibration"]["base_to_camera"])
        old_id = meta["calibration"]["calibration_id"]
        info.k[0] = 102.
        self.assertNotEqual(frame.calibration(info, image)["calibration_id"], old_id)
        self.assertFalse(frame.calibration(None, image)["available"])
        info.width += 1
        self.assertEqual(frame.calibration(info, image)["reason"], "camera_info_mismatch")
        image, info = sample(0)
        _, meta = frame.encode_frame(image, info, 123, "unverified")
        self.assertIsNone(meta["header"]["stamp_ns"])
        self.assertFalse(meta["timing"]["available"])
        image, info = sample()
        _, meta = frame.encode_frame(image, info, 123, "unverified")
        self.assertFalse(meta["timing"]["available"])
        info.k[0] = float("nan")
        self.assertEqual(frame.calibration(info, image)["reason"], "camera_info_invalid")
        image.step = 23
        with self.assertRaisesRegex(ValueError, "invalid_image_layout"):
            frame.encode_frame(image, info, 123, "ros_system_time")

    def test_ros_cross_domain_and_lifecycle(self):
        # Never share the robot's socket, services or network. CI runs --network none.
        with tempfile.TemporaryDirectory(prefix="tianyi-frame-test-") as directory, \
                patch.object(SocketBridgeServer, "SOCKET_DIR", directory), \
                patch.object(BridgedPublisher, "SOCKET_DIR", directory):
            bridge = SocketBridgeServer()
            bridge.start()
            ctx = Context()
            rclpy.init(context=ctx, domain_id=0)
            executor = SingleThreadedExecutor(context=ctx)
            source = Node("test_camera_source", context=ctx)
            executor.add_node(source)
            plugin = frame.CameraRgbFramePlugin({"source_clock_domain": "ros_system_time"},
                "test", SimpleNamespace(ctx_tianyi=ctx, executor_tianyi=executor))
            sink = Node("test_camera_sink", context=bridge.ctx)
            bridge.executor.add_node(sink)
            received = []
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            image_pub = source.create_publisher(Image, "/ob_camera_head/color/image_raw", qos)
            info_pub = source.create_publisher(CameraInfo, "/ob_camera_head/color/camera_info", qos)
            sub = sink.create_subscription(UInt8MultiArray, "/test/camera/rgb_frame",
                                           lambda msg: received.append(bytes(msg.data)), qos)

            def wait_until(predicate, tick=lambda: None, seconds=5):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if predicate():
                        return
                    tick()
                    executor.spin_once(timeout_sec=0.01)
                self.fail("condition timed out: " + str(plugin.dispatch("info", {})))

            try:
                plugin.dispatch("start", {})
                wait_until(lambda: info_pub.get_subscription_count() == 1)
                _, info = sample()
                wait_until(lambda: plugin._info is not None, lambda: info_pub.publish(info))
                wait_until(lambda: bool(received), lambda: image_pub.publish(sample()[0]))
                meta, _ = decode(received[-1])
                self.assertEqual(meta["schema"], frame.SCHEMA)
                self.assertTrue(meta["calibration"]["available"])
                self.assertTrue(plugin.dispatch("info", {})["ready"])
                actual_qos = bridge.handlers[plugin._topic].pub.qos_profile
                self.assertEqual(actual_qos.depth, 1)
                self.assertEqual(actual_qos.reliability, ReliabilityPolicy.RELIABLE)
                self.assertEqual(actual_qos.durability, DurabilityPolicy.VOLATILE)
                # Old bridge clients retain their original QoS.
                legacy = TopicHandler("/test/legacy", "std_msgs/msg/UInt8MultiArray", bridge.ctx, bridge.executor)
                self.assertEqual(legacy.pub.qos_profile.depth, 200)
                self.assertEqual(legacy.pub.qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
                bridge.executor.remove_node(legacy.node)
                legacy.node.destroy_node()

                plugin.dispatch("stop", {})
                self.assertFalse(plugin._thread.is_alive())
                count = plugin._counts["submitted"]
                plugin._on_image(sample()[0])
                self.assertEqual(plugin._counts["submitted"], count)
                plugin.dispatch("start", {})
                plugin._on_info(info)
                # One in-flight encoder and one latest pending frame, not a FIFO.
                entered, release = threading.Event(), threading.Event()
                original = frame.encode_frame

                def slow_encode(*args):
                    entered.set()
                    release.wait(1)
                    return original(*args)

                with patch.object(frame, "encode_frame", slow_encode):
                    plugin._on_image(sample()[0])
                    self.assertTrue(entered.wait(1))
                    for _ in range(20):
                        plugin._on_image(sample()[0])
                    latest = plugin._last_stamp
                    self.assertGreaterEqual(plugin._counts["coalesced"], 19)
                    release.set()
                    wait_until(lambda: plugin._meta is not None and plugin._meta["header"]["stamp_ns"] == latest)
                plugin._on_image(sample(latest)[0])
                plugin._on_image(sample(latest - 1)[0])
                self.assertEqual(plugin.dispatch("info", {})["reason"], "source_stamp_not_increasing")
                wait_until(lambda: plugin.dispatch("info", {})["reason"] == "image_stale", seconds=2)
                # Restart clears the epoch watermark but never repairs an old stamp with now().
                plugin.dispatch("stop", {})
                plugin.dispatch("start", {})
                plugin._on_image(sample(time.time_ns() - 2_000_000_000)[0])
                wait_until(lambda: plugin._reason == "image_stale_or_future")
                plugin._on_image(sample()[0])
                wait_until(lambda: plugin._reason == "camera_info_unavailable")
                plugin._on_info(info)
                plugin._pub.destroy()
                with patch.object(plugin._pub, "socket_path", directory + "/missing.sock"):
                    plugin._on_image(sample()[0])
                    wait_until(lambda: plugin._reason == "ConnectionError")
                previous = len(received)
                wait_until(lambda: len(received) > previous, lambda: image_pub.publish(sample()[0]))
                wait_until(lambda: bridge.handlers[plugin._topic].msg_count == plugin._counts["submitted"])
                plugin.close()
                with self.assertRaisesRegex(RuntimeError, "camera_frame_closed"):
                    plugin.start()
            finally:
                plugin.close()
                executor.remove_node(source)
                source.destroy_node()
                executor.shutdown()
                rclpy.shutdown(context=ctx)
                sink.destroy_subscription(sub)
                bridge.executor.remove_node(sink)
                sink.destroy_node()
                bridge.stop()


if __name__ == "__main__":
    unittest.main()
