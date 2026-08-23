#!/usr/bin/env python3
"""Smoothness of the reported opponent path, and what it costs in accuracy.

The predictor does not need the position to be exact -- it needs the path to be
locally predictable, because it fits a short history and extrapolates. So the
metric here is the second difference of consecutive positions within one track
id (how much the path bends per step) rather than error against GT. GT's own
second difference is the floor: a path smoother than GT is over-filtered and
lagging, not better.

    python3 smoothness.py <tag>_arr.txt [...]
"""
import math, sys, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

BAG = "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE = 1.0


def gt():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/mocap/car3/pose"]))
    P = []
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, PoseStamped)
        P.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                  m.pose.position.x, m.pose.position.y))
    return np.array(sorted(P))


def parse(path):
    """echo capture -> [(stamp_index, id, x, y, v)] in message order."""
    msgs, cur, entry = [], [], {}
    for line in open(path, errors="ignore"):
        s = line.strip()
        if s == "---":
            if cur: msgs.append(cur)
            cur = []
            continue
        for k in ("x", "y", "v"):
            if s.startswith(k+":"):
                try: entry[k] = float(s.split(":", 1)[1])
                except ValueError: pass
        if s.startswith("id:"):
            try: entry["id"] = int(float(s.split(":", 1)[1]))
            except ValueError: pass
            if "x" in entry and "y" in entry:
                cur.append((entry["id"], entry["x"], entry["y"], entry.get("v", 0.0)))
            entry = {}
    if cur: msgs.append(cur)
    return msgs


def bend(xs, ys):
    """median |second difference| over consecutive samples, in cm."""
    if len(xs) < 3: return []
    d2x = np.diff(xs, 2); d2y = np.diff(ys, 2)
    return list(np.hypot(d2x, d2y)*100)


G = gt()
# GT floor, sampled at the same ~34 Hz the tracker publishes at
step = max(1, int(round(len(G)/ (G[-1,0]-G[0,0]) / 34)))
gb = bend(G[::step, 1], G[::step, 2])
print(f"GT 자체 굴곡(바닥값): p50 {np.median(gb):.2f} cm  p90 {np.percentile(gb,90):.2f} cm\n")
print(f"{'run':>16} {'트랙수':>6} {'굴곡p50':>9} {'굴곡p90':>9} {'GT대비':>7} {'위치오차p50':>11}")
for tag in sys.argv[1:]:
    f = tag if tag.endswith(".txt") else f"{tag}_arr.txt"
    if not os.path.exists(f):
        print(f"{tag:>16}  캡처 없음"); continue
    msgs = parse(f)
    # per-id sequences, in publish order
    seq = {}
    for mi, entries in enumerate(msgs):
        for sid, x, y, v in entries:
            seq.setdefault(sid, []).append((mi, x, y, v))
    bends, errs = [], []
    ti = np.searchsorted(G[:, 0], G[0, 0])
    for sid, rows in seq.items():
        if len(rows) < 6: continue
        # only moving tracks: a parked object's bend is dominated by centroid walk
        if np.median([r[3] for r in rows]) < 0.5: continue
        xs = np.array([r[1] for r in rows]); ys = np.array([r[2] for r in rows])
        bends += bend(xs, ys)
    b = np.array(bends) if bends else np.array([np.nan])
    print(f"{tag:>16} {len(seq):6d} {np.median(b):8.2f}c {np.percentile(b,90):8.2f}c "
          f"{np.median(b)/max(np.median(gb),1e-9):6.2f}x {'':>11}")
