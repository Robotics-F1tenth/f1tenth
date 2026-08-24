#!/usr/bin/env python3
"""dynamic_speed_threshold: both sides of the trade, on the same quantity.

The threshold is compared against the PUBLISHED v (an EKF state), not against
raw centroid displacement -- those differ by a lot because the filter smooths
heavily. Everything here is measured on published v.

  benefit side: how fast the gate opens for a real moving opponent (0821)
  cost side   : how often a stationary obstacle reads as dynamic (0820)
"""
import math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

OPP = "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
STATIC = "/home/a/Downloads/f1tenth_bag/0820/rosbag2_2026_08_20-04_08_16"
GATE = 1.0


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = []
    while r.has_next():
        tn, d, _ = r.read_next()
        out.append((tn, d))
    return out


# ---- opponent: runs of continuous tracking, published v over time -----------
opp, arr = [], []
for tn, d in read(OPP, ["/mocap/car3/pose", "/f1/perception/object/obstacles/arr"]):
    if tn == "/mocap/car3/pose":
        m = deserialize_message(d, PoseStamped)
        opp.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    m.pose.position.x, m.pose.position.y))
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    [(int(s.id), s.x, s.y, abs(s.v)) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); arr.sort()
ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
runs, cur, prev = [], [], None
for i, (t, st) in enumerate(arr):
    hit = None
    if st:
        dd = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
        j = int(np.argmin(dd))
        if dd[j] <= GATE: hit = st[j]
    if hit is None or (prev is not None and hit[0] != prev):
        if len(cur) > 1: runs.append(cur)
        cur = []
    if hit is not None:
        cur.append((t, hit[3])); prev = hit[0]
    else:
        prev = None
if len(cur) > 1: runs.append(cur)
opp_v = np.array([v for run in runs for _, v in run])

# ---- static bag: published v on long-lived tracks ---------------------------
sarr = []
for tn, d in read(STATIC, ["/f1/perception/object/obstacles/arr"]):
    m = deserialize_message(d, F1stateArr)
    sarr.append([(int(s.id), abs(s.v)) for s in m.f1_state_arr])
life = {}
for st in sarr:
    for sid, _ in st: life[sid] = life.get(sid, 0) + 1
long_ids = {i for i, c in life.items() if c >= 40}
stat_v = np.array([v for st in sarr for sid, v in st if sid in long_ids])

print(f"상대차 추적구간 {len(runs)}개 / 프레임 {len(opp_v)}")
print(f"정적 bag 장수명 트랙 {len(long_ids)}개 / 프레임 {len(stat_v)}\n")
print(f"{'thresh':>7} | {'상대: 게이트 열림':>18} {'구간 중 못 여는 비율':>20} | "
      f"{'정적: 동적 오분류':>18}")
for th in [0.3, 0.5, 0.75, 1.0, 1.5, 2.0]:
    opened, never = [], 0
    for run in runs:
        t0 = run[0][0]
        tv = next((t-t0 for t, v in run if v >= th), None)
        if tv is None: never += 1
        else: opened.append(1000*tv)
    frac_open = 100*np.mean(opp_v >= th)
    fp = 100*np.mean(stat_v >= th)
    med = np.median(opened) if opened else float("nan")
    print(f"{th:7.2f} | p50 {med:6.0f} ms, 프레임 {frac_open:4.1f}% "
          f"{100*never/len(runs):17.0f}% | {fp:16.1f}%")
print("\n  상대: '못 여는 비율'이 낮을수록 좋다")
print("  정적: 낮을수록 좋다 (정지 물체를 동적으로 예측 -> 유령 궤적)")
