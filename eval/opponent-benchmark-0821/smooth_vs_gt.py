#!/usr/bin/env python3
"""Path bend and GT error together, for one or more replay captures.

Bend is the second difference of consecutive published positions within a track
(how much the path kinks per step). GT's own bend is the floor -- going below it
means the filter is lagging, not that the path got better. Error is reported
alongside so a smoothness gain that is really just lag shows up.
"""
import math, os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

BAG = "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE = 1.0
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
G = np.array(sorted(P))


def parse(path):
    msgs, cur, e = [], [], {}
    for line in open(path, errors="ignore"):
        s = line.strip()
        if s == "---":
            msgs.append(cur); cur = []; continue
        for k in ("x", "y", "v"):
            if s.startswith(k+":"):
                try: e[k] = float(s.split(":", 1)[1])
                except ValueError: pass
        if s.startswith("id:"):
            try: e["id"] = int(float(s.split(":", 1)[1]))
            except ValueError: pass
            if "x" in e: cur.append((e["id"], e["x"], e["y"], e.get("v", 0.0)))
            e = {}
    if cur: msgs.append(cur)
    return msgs


def bend(x, y):
    if len(x) < 3: return np.array([])
    return np.hypot(np.diff(x, 2), np.diff(y, 2))*100


# GT floor at the publish rate
dur = G[-1, 0]-G[0, 0]
step = max(1, int(round(len(G)/dur/34)))
gfloor = bend(G[::step, 1], G[::step, 2])
print(f"GT 굴곡 바닥: p50 {np.median(gfloor):.2f} cm  p90 {np.percentile(gfloor,90):.2f} cm\n")
print(f"{'run':>10} {'동적트랙':>7} {'굴곡p50':>9} {'굴곡p90':>9} {'GT배수':>7} {'id수':>6}")
for tag in sys.argv[1:]:
    f = tag if tag.endswith(".txt") else f"{tag}_arr.txt"
    if not os.path.exists(f):
        print(f"{tag:>10}   (없음)"); continue
    msgs = parse(f)
    # No usable timestamps in an echo capture, so error against GT cannot be
    # computed here -- spreading message indices evenly over the GT span assumes a
    # constant publish rate the replay does not have, and produced a nonsense 4 m.
    # Bend needs no absolute time (it is a shape measure within one id), so that
    # is what this reports. Use bench_opponent.py on a live run for error.
    seq = {}
    for mi, entries in enumerate(msgs):
        for sid, x, y, v in entries:
            seq.setdefault(sid, []).append((mi, x, y, v))
    bends, ndyn = [], 0
    for sid, rows in seq.items():
        if len(rows) < 6: continue
        if np.median([r[3] for r in rows]) < 0.5: continue
        ndyn += 1
        xs = np.array([r[1] for r in rows]); ys = np.array([r[2] for r in rows])
        bends += list(bend(xs, ys))
    b = np.array(bends) if bends else np.array([np.nan])
    print(f"{tag:>10} {ndyn:7d} {np.median(b):8.2f}c {np.percentile(b,90):8.2f}c "
          f"{np.median(b)/max(np.median(gfloor),1e-9):6.2f}x {len(seq):6d}")
