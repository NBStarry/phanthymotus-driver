"""Read-only Slamtec REST evidence probe; never a navigation data producer."""

import argparse
import hashlib
import json
import math
import time
import urllib.error
import urllib.parse
import urllib.request


# Fixed GET allowlist: no actions, cancellation, configuration or heartbeat writes.
ENDPOINTS = {
    "scan": "/api/core/system/v1/laserscan",
    "odometry_pose": "/api/core/slam/v1/localization/odopose",
    "speed": "/api/core/motion/v1/speed",
}
MAX_RESPONSE_BYTES = 2 * 1024 * 1024


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def validate_base_url(value):
    parts = urllib.parse.urlsplit(value)
    if (parts.scheme not in ("http", "https") or not parts.hostname
            or parts.username or parts.password or parts.query or parts.fragment
            or parts.path not in ("", "/")):
        raise ValueError("base URL must be an HTTP(S) origin without credentials")
    _ = parts.port  # Reject malformed/out-of-range ports before making requests.
    return value.rstrip("/")


def reject_constant(value):
    raise ValueError("non-finite JSON number")


def describe(name, payload):
    """Keep structural evidence, not a synthetic source stamp or freshness flag."""
    if not isinstance(payload, dict) or "error" in payload:
        raise ValueError("unexpected response")
    if name == "scan":
        points = payload.get("laser_points")
        if not isinstance(points, list) or not points:
            raise ValueError("empty or invalid scan")
        for point in points:
            if not isinstance(point, dict) or not isinstance(point.get("valid"), bool):
                raise ValueError("invalid laser point")
            for key in ("angle", "distance"):
                value = point.get(key)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("invalid laser number")
        return {"point_count": len(points),
                "valid_point_count": sum(p["valid"] for p in points),
                "point_keys": sorted(set().union(*(p.keys() for p in points)))}
    keys = ("x", "y", "z", "yaw", "pitch", "roll") if name == "odometry_pose" else ("vx", "vy", "omega")
    result = {}
    for key in keys:
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("invalid pose or speed")
        result[key] = value
    return {"values": result}


def collect(base_url, samples=10, interval=0.2, timeout=2.0):
    base_url = validate_base_url(base_url)
    if (isinstance(samples, bool) or not isinstance(samples, int) or not 1 <= samples <= 100
            or not math.isfinite(interval) or not 0.2 <= interval <= 10
            or not math.isfinite(timeout) or not 0 < timeout <= 5):
        raise ValueError("samples: 1..100; interval: 0.2..10 s; timeout: (0,5] s")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    records = []
    for sample in range(samples):
        cycle_started = time.monotonic()
        for name, path in ENDPOINTS.items():
            started = time.monotonic_ns()
            record = {"endpoint": name, "poll_index": sample,
                      "request_unix_ns": time.time_ns(), "request_monotonic_ns": started}
            try:
                request = urllib.request.Request(base_url + path, method="GET")
                with opener.open(request, timeout=timeout) as response:
                    record["http_status"] = response.status
                    raw = response.read(MAX_RESPONSE_BYTES + 1)
                    record["receive_unix_ns"] = time.time_ns()
                    record["request_duration_ms"] = round((time.monotonic_ns() - started) / 1e6, 3)
                if len(raw) > MAX_RESPONSE_BYTES:
                    raise ValueError("response_too_large")
                payload = json.loads(raw, parse_constant=reject_constant)
                details = describe(name, payload)
                canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
                record.update(details, response_keys=sorted(payload),
                              content_sha256=hashlib.sha256(canonical.encode()).hexdigest())
            except (OSError, ValueError, urllib.error.URLError) as exc:
                # Do not echo server error bodies, credentials or private URLs.
                record["error"] = type(exc).__name__
                if isinstance(exc, urllib.error.HTTPError):
                    record["http_status"] = exc.code
                    exc.close()
            record.setdefault("receive_unix_ns", time.time_ns())
            record.setdefault("request_duration_ms", round((time.monotonic_ns() - started) / 1e6, 3))
            records.append(record)
        if sample + 1 < samples:
            time.sleep(max(0, interval - (time.monotonic() - cycle_started)))
    summary = {}
    for name in ENDPOINTS:
        rows = [r for r in records if r["endpoint"] == name]
        good = [r for r in rows if "error" not in r]
        summary[name] = {"polls": len(rows), "successful_reads": len(good),
                         "distinct_payloads": len({r["content_sha256"] for r in good}),
                         "source_update_hz": None, "source_age_ms": None}
    return {"probe_status": "read_errors" if any("error" in r for r in records) else "collected",
            "navigation_contract_verified": False,
            "note": "Request/receive times and distinct payloads do not prove acquisition time or freshness.",
            "summary": summary, "records": records}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", required=True)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args(argv)
    try:
        report = collect(args.base_url, args.samples, args.interval, args.timeout)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report["probe_status"] == "collected" else 2


if __name__ == "__main__":
    raise SystemExit(main())
