"""Exercise the actual HTTP probe without ROS or robot access."""

import contextlib
import importlib.util
import io
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location("navigation_probe", Path(__file__).parents[1] / "navigation_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.requests = []
        self.status = 200
        self.bodies = {
            "scan": {"pose": {}, "laser_points": [{"angle": 0.0, "distance": 1.0, "valid": True}]},
            "odometry_pose": dict.fromkeys(("x", "y", "z", "yaw", "pitch", "roll"), 0.0),
            "speed": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
        }
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                owner.requests.append((self.command, self.path))
                name = next(k for k, v in probe.ENDPOINTS.items() if v == self.path)
                body = owner.bodies[name]
                encoded = body if isinstance(body, bytes) else json.dumps(body).encode()
                self.send_response(owner.status)
                if owner.status == 302:
                    self.send_header("Location", "/must-not-follow")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = "http://127.0.0.1:" + str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_repeated_payload_does_not_prove_zero_speed_or_freshness(self):
        report = probe.collect(self.url, samples=2)
        self.assertEqual(report["probe_status"], "collected")
        self.assertFalse(report["navigation_contract_verified"])
        self.assertEqual(len(self.requests), 6)
        self.assertTrue(all(method == "GET" and path in probe.ENDPOINTS.values() for method, path in self.requests))
        for name in probe.ENDPOINTS:
            self.assertEqual(report["summary"][name]["distinct_payloads"], 1)
            self.assertIsNone(report["summary"][name]["source_update_hz"])
            self.assertIsNone(report["summary"][name]["source_age_ms"])
        self.assertTrue(all("source_stamp_ns" not in r and "stop_confirmed" not in r for r in report["records"]))

    def test_source_looking_key_does_not_make_clock_verified(self):
        self.bodies["scan"]["timestamp"] = 123
        report = probe.collect(self.url, samples=1)
        self.assertIn("timestamp", report["records"][0]["response_keys"])
        self.assertFalse(report["navigation_contract_verified"])

    def test_invalid_scan_and_non_finite_values_are_errors(self):
        self.bodies["scan"] = {"laser_points": []}
        self.bodies["speed"] = b'{"vx":NaN,"vy":0,"omega":0}'
        report = probe.collect(self.url, samples=1)
        self.assertEqual(report["probe_status"], "read_errors")
        self.assertEqual(report["summary"]["scan"]["successful_reads"], 0)
        self.assertEqual(report["summary"]["speed"]["successful_reads"], 0)

    def test_http_failure_and_redirect_are_not_followed(self):
        for status in (503, 302):
            self.status = status
            report = probe.collect(self.url, samples=1)
            self.assertTrue(all(r["http_status"] == status and "error" in r for r in report["records"]))
        self.assertEqual(len(self.requests), 6)

    def test_response_size_is_bounded(self):
        with patch.object(probe, "MAX_RESPONSE_BYTES", 8):
            report = probe.collect(self.url, samples=1)
        self.assertTrue(all("error" in r for r in report["records"]))

    def test_timeout_never_becomes_success(self):
        with patch.object(probe.urllib.request.OpenerDirector, "open", side_effect=TimeoutError):
            report = probe.collect(self.url, samples=1)
        self.assertTrue(all(r["error"] == "TimeoutError" for r in report["records"]))

    def test_bad_arguments_fail_before_network(self):
        for url in ("file:///tmp/data", "http://user:secret@localhost", self.url + "/actions", self.url + "?x=1"):
            with self.assertRaises(ValueError):
                probe.collect(url, samples=1)
        for options in ({"samples": 0}, {"samples": 101}, {"interval": 0}, {"interval": float("nan")}, {"timeout": 0}):
            with self.assertRaises(ValueError):
                probe.collect(self.url, **options)
        self.assertEqual(self.requests, [])

    def test_cli_success_and_failure_exit_codes(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(probe.main(["--base-url", self.url, "--samples", "1"]), 0)
        self.assertEqual(json.loads(output.getvalue())["probe_status"], "collected")
        self.status = 503
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(probe.main(["--base-url", self.url, "--samples", "1"]), 2)


if __name__ == "__main__":
    unittest.main()
