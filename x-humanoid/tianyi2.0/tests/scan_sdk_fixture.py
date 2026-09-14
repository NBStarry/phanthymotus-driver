#!/usr/bin/env python3
"""Synthetic reader for isolated tests only; no device/network access."""
import json
import math
import sys
import time

if sys.argv[1:] != ["synthetic", "1445", "--stream-scan"]:
    raise SystemExit(2)
while True:
    stamp = time.time_ns() // 1000
    row = {"endpoint": "raw_scan", "start_stamp_raw": stamp, "end_stamp_raw": stamp - 100,
           "points_angle_distance_valid": [[-math.pi + i * 2 * math.pi / 2999, 2.0, True]
                                           for i in range(3000)]}
    # Repeat one frame: consumer must not publish it twice or refresh its age.
    for _ in range(2):
        print(json.dumps(row), flush=True)
        time.sleep(0.05)
