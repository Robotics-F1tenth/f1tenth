#!/usr/bin/env python3
"""Would a consistency gate let the two-point seed fire on the opponent but not on
a stationary object?

The ungated seed helped moving tracks and cost 4 real landmarks on a static bag,
because one frame pair's position noise reads as motion. Two consecutive
displacements separate the cases: on a real mover they agree in direction and
magnitude, on a stationary object they are random. This replays both bags'
published positions and reports what the gate would have done.
"""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

OPP = "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
STATIC = "/home/a/Downloads/f1tenth_bag/0820/rosbag2_2026_08_20-04_08_16"


def load(bag, topic="/f1/perception/object/obstacles/arr"):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, F1stateArr)
        t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        out.append((t, [(int(s.id), s.x, s.y, abs(s.v)) for s in m.f1_state_arr]))
    out.sort()
    return out


def seeds(arr, vmin, agree_deg):
    """Per new track, the seed the gate would produce on its 3rd measurement."""
    hist = {}
    fired, blocked = [], 0
    seen = set()
    for t, st in arr:
        for sid, x, y, v in st:
            h = hist.setdefault(sid, [])
            h.append((t, x, y))
            if len(h) == 3 and sid not in seen:
                seen.add(sid)
                (t1, x1, y1), (t2, x2, y2), (t3, x3, y3) = h
                d1, d2 = t2-t1, t3-t2
                if d1 < 1e-3 or d2 < 1e-3:
                    blocked += 1; continue
                v1 = ((x2-x1)/d1, (y2-y1)/d1)
                v2 = ((x3-x2)/d2, (y3-y2)/d2)
                s1, s2 = math.hypot(*v1), math.hypot(*v2)
                if s1 < vmin or s2 < vmin:
                    blocked += 1; continue
                dot = (v1[0]*v2[0]+v1[1]*v2[1])/(s1*s2)
                ang = math.degrees(math.acos(max(-1, min(1, dot))))
                if ang > agree_deg:
                    blocked += 1; continue
                fired.append(0.5*(s1+s2))
            if len(h) > 3:
                h.pop(0)
    return fired, blocked, len(seen)


print("게이트: 연속 두 변위가 모두 vmin 이상 + 방향차 agree 이내일 때만 시드\n")
print(f"{'vmin':>5} {'agree':>6} | {'상대차 bag':>26} | {'정적 bag':>24}")
print(f"{'':>5} {'':>6} | {'발동':>6} {'차단':>6} {'시드p50':>9} | {'발동':>6} {'차단':>6} {'시드p50':>7}")
ao = load(OPP)
astat = load(STATIC)
for vmin, agree in [(0.0, 180), (0.5, 60), (1.0, 45), (1.0, 30), (1.5, 30)]:
    fo, bo, no = seeds(ao, vmin, agree)
    fs, bs, ns = seeds(astat, vmin, agree)
    po = np.median(fo) if fo else float("nan")
    ps = np.median(fs) if fs else float("nan")
    print(f"{vmin:5.1f} {agree:6.0f} | {len(fo):6d} {bo:6d} {po:9.2f} | "
          f"{len(fs):6d} {bs:6d} {ps:7.2f}")
print("\n  상대차 bag: 발동이 많고 시드값이 실제 속도(약 2 m/s)에 가까울수록 좋다")
print("  정적 bag  : 발동이 적을수록 좋다 (발동 = 정지 물체에 가짜 속도 주입)")
