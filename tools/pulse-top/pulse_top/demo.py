"""Demo log writer: a fake robot graph with a scripted incident, for GIFs and dev.

Writes real probe-schema jsonl (same shapes test_model.py pins) to a temp file on a
background thread. The story loops: steady traffic, a /scan stall that fires a
topic_gap warn, /cmd_vel drifting under its min rate, /localization going missing.
"""

from __future__ import annotations

import json
import math
import random
import tempfile
import threading
import time


def _window(i: int) -> dict:
    stall = i % 24 in (10, 11)          # /scan stalls two windows per loop
    cmd_low = 8 <= i % 24 <= 14         # /cmd_vel sags mid-loop
    loc_missing = i % 24 >= 18          # /localization dies late in the loop
    j = lambda base, amp=0.15: round(base + random.uniform(-amp, amp), 6)

    topics = [
        {"topic": "/scan", "pub_inter_hz": j(4.0) if stall else j(19.8),
         "pub_intra_hz": 0.0, "recv_inter_hz": j(4.0) if stall else j(19.8),
         "recv_intra_hz": 0.0, "recv_endpoint_seen": True,
         **({"pub_max_dt_ms": round(700 + random.uniform(0, 200), 3)} if stall else {})},
        {"topic": "/points", "pub_inter_hz": j(10.0), "pub_intra_hz": j(10.0),
         "recv_inter_hz": j(10.0), "recv_intra_hz": j(10.0), "recv_endpoint_seen": True},
        {"topic": "/imu", "pub_inter_hz": j(200.0, 0.5), "pub_intra_hz": 0.0,
         "recv_inter_hz": j(200.0, 0.5), "recv_intra_hz": 0.0, "recv_endpoint_seen": True},
        {"topic": "/camera/image_raw", "pub_inter_hz": j(29.9), "pub_intra_hz": j(29.9),
         "recv_inter_hz": j(29.9), "recv_intra_hz": j(29.9), "recv_endpoint_seen": True},
        {"topic": "/tf", "pub_inter_hz": j(99.8, 0.4), "pub_intra_hz": 0.0,
         "recv_inter_hz": j(99.7, 0.4), "recv_intra_hz": 0.0, "recv_endpoint_seen": True},
        {"topic": "/cmd_vel",
         # sag well under the 19 Hz min: 14.5-17.5 Hz, so every warn is visibly out of range
         "pub_inter_hz": j(16.0 - 1.5 * math.sin(i / 3.0)) if cmd_low else j(20.0),
         "pub_intra_hz": 0.0, "recv_inter_hz": j(16.0) if cmd_low else j(20.0),
         "recv_intra_hz": 0.0, "recv_endpoint_seen": True},
    ]
    warns = []
    if stall:
        warns.append({"kind": "topic_gap", "topic": "/scan",
                      "max_dt_ms": topics[0]["pub_max_dt_ms"], "max_gap_ms": 250})
    if cmd_low:
        warns.append({"kind": "topic_rate", "topic": "/cmd_vel",
                      "hz": topics[5]["pub_inter_hz"], "min_hz": 19.0, "max_hz": 22.0})
    if loc_missing:
        warns.append({"kind": "node_missing", "node": "/localization"})

    nodes = ["/perception", "/planner", "/controller"]
    if not loc_missing:
        nodes.append("/localization")
    return {"ts_ns": str(1782887150000000000 + i * 1_000_000_000), "window_s": 1.0,
            "topics": topics, "nodes": nodes, "warns": warns}


def start_demo_writer(period_s: float = 1.0) -> str:
    f = tempfile.NamedTemporaryFile(
        mode="w", prefix="pulse_top_demo.", suffix=".jsonl", delete=False
    )

    def run() -> None:
        i = 0
        while True:
            f.write(json.dumps(_window(i)) + "\n")
            f.flush()
            i += 1
            time.sleep(period_s)

    threading.Thread(target=run, daemon=True).start()
    return f.name
