#!/usr/bin/env python3
"""After a re-acquisition, which predictor gate opens last?

The MDN needs kHistory=6 samples at kDt=0.04 -> only the last 0.20 s of history,
and it needs |v| >= dynamic_speed_threshold (1.0) to run at all. Whichever takes
longer is the one worth fixing.
"""
import math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

B = "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE, HIST_S, VTH = 1.0, 0.20, 1.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/mocap/car3/pose",
                                              "/f1/perception/object/obstacles/arr"]))
opp, arr = [], []
while r.has_next():
    tn, d, _ = r.read_next()
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

runs, cur = [], []
prev = None
for i, (t, st) in enumerate(arr):
    hit = None
    if st:
        d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
        j = int(np.argmin(d))
        if d[j] <= GATE: hit = st[j]
    if hit is None or (prev is not None and hit[0] != prev):
        if len(cur) > 1: runs.append(cur)
        cur = []
    if hit is not None:
        cur.append((t, hit[3]))
        prev = hit[0]
    else:
        prev = None
if len(cur) > 1: runs.append(cur)

hist_ms, vel_ms, never = [], [], 0
for run in runs:
    t0 = run[0][0]
    th = next((t-t0 for t, _ in run if t-t0 >= HIST_S), None)
    tv = next((t-t0 for t, v in run if v >= VTH), None)
    if th is not None: hist_ms.append(1000*th)
    if tv is not None: vel_ms.append(1000*tv)
    else: never += 1
print(f"연속 추적 구간 {len(runs)}개\n")
print(f"히스토리 0.20 s 를 채우기까지: n={len(hist_ms)}  "
      f"p50 {np.median(hist_ms):.0f} ms  p90 {np.percentile(hist_ms,90):.0f} ms")
print(f"|v| 가 {VTH} 를 넘기까지    : n={len(vel_ms)}  "
      f"p50 {np.median(vel_ms):.0f} ms  p90 {np.percentile(vel_ms,90):.0f} ms")
print(f"  구간이 끝날 때까지 {VTH} 를 못 넘김: {never}/{len(runs)} "
      f"({100*never/len(runs):.0f}%)")
tot = sum(len(x) for x in runs)
usable = 0
for run in runs:
    t0 = run[0][0]
    usable += sum(1 for t, v in run if t-t0 >= HIST_S and v >= VTH)
print(f"\n두 조건을 모두 만족하는 프레임: {usable}/{tot} = {100*usable/tot:.1f}%")
print("  -> 상대를 '보고 있는' 동안에도 이 비율에서만 MDN 예측이 돈다")
