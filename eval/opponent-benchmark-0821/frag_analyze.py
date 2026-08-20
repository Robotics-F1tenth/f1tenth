#!/usr/bin/env python3
"""Why does one opponent consume many track ids?

An id change can mean two different things and they need different fixes:
  gap    - the track died, then a new one was created later. Lifecycle/removal.
  swap   - a different id appears in the very next cycle. Association.
This separates them, and reports how long the opponent went unseen across a gap.
"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

B = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE = 1.0

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
T = ["/mocap/car3/pose", "/newmcl_pose", "/f1/perception/object/obstacles/arr"]
r.set_filter(rosbag2_py.StorageFilter(topics=T))
opp, ego, arr = [], [], []
while r.has_next():
    tn, d, _ = r.read_next()
    if tn in ("/mocap/car3/pose", "/newmcl_pose"):
        m = deserialize_message(d, PoseStamped)
        row = (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
               m.pose.position.x, m.pose.position.y)
        (opp if tn == "/mocap/car3/pose" else ego).append(row)
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                    [(int(s.id), s.x, s.y, abs(s.v)) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); E = np.array(sorted(ego)); arr.sort()
ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ex = np.interp(ts, E[:, 0], E[:, 1]); ey = np.interp(ts, E[:, 0], E[:, 2])
rng = np.hypot(ox-ex, oy-ey)

seq = []          # (cycle_index, t, id, range) where the opponent was matched
for i, (t, st) in enumerate(arr):
    if not st:
        continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] <= GATE:
        seq.append((i, t, st[j][0], rng[i], st[j][3]))

print(f"상대 매칭 {len(seq)}건")
gaps, swaps = [], 0
for k in range(1, len(seq)):
    if seq[k][2] == seq[k-1][2]:
        continue
    dcyc = seq[k][0] - seq[k-1][0]
    dt = seq[k][1] - seq[k-1][1]
    if dcyc <= 1:
        swaps += 1
    else:
        gaps.append((dcyc, dt, seq[k-1][3]))
print(f"\nid 전환 {swaps+len(gaps)}회")
print(f"  swap (바로 다음 사이클에 다른 id) : {swaps}회 -> 연관(association) 문제")
print(f"  gap  (사이 사이클에 상대 미검출)  : {len(gaps)}회 -> 검출/수명 문제")
if gaps:
    gc = np.array([g[0] for g in gaps]); gt = np.array([g[1] for g in gaps])
    print(f"    공백 길이  p50 {np.median(gc):.0f} 사이클 / {1000*np.median(gt):.0f} ms"
          f"   p90 {np.percentile(gc,90):.0f} / {1000*np.percentile(gt,90):.0f} ms")
    print(f"    공백 직전 거리 p50 {np.median([g[2] for g in gaps]):.2f} m")
# 재획득 직후 속도가 얼마나 낮은가
first_v, steady_v = [], []
prev = None
for k, (i, t, sid, rg, v) in enumerate(seq):
    if sid != prev:
        first_v.append(v); prev = sid
    else:
        steady_v.append(v)
if first_v and steady_v:
    print(f"\n재획득 직후 속도 p50 {np.median(first_v):.2f} m/s "
          f"vs 이어지는 프레임 p50 {np.median(steady_v):.2f} m/s   (GT p50 약 2.05)")
    print(f"  재획득 직후 |v|<0.5 (control 이 정지로 볼 값) 비율 "
          f"{100*np.mean(np.array(first_v)<0.5):.0f}%")
