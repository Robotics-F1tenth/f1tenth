#!/usr/bin/env python3
"""Would velocity from the track's OWN first two positions beat the EKF's warm-up?

This is not the warm-start rejected in workplan B-7 -- that inherited a DIFFERENT
object's velocity and risked dragging a genuinely static object out of the static
gate. This uses only the track's own measurements, so it can never import another
object's motion. Two positions and dt are exactly what velocity is defined by.
"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

B = "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE = 1.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/mocap/car3/pose", "/newmcl_pose",
                                              "/f1/perception/object/obstacles/arr"]))
opp, ego, arr = [], [], []
while r.has_next():
    tn, d, _ = r.read_next()
    if tn in ("/mocap/car3/pose", "/newmcl_pose"):
        m = deserialize_message(d, PoseStamped)
        (opp if tn == "/mocap/car3/pose" else ego).append(
            (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
             m.pose.position.x, m.pose.position.y))
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                    [(int(s.id), s.x, s.y, abs(s.v)) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); E = np.array(sorted(ego)); arr.sort()
ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
gsp = np.hypot(np.gradient(G[:, 1], G[:, 0]), np.gradient(G[:, 2], G[:, 0]))
gsp = np.convolve(gsp, np.ones(15)/15, mode="same")
ospd = np.interp(ts, G[:, 0], gsp)

prev = None   # (id, t, x, y)
age = 0
rows = []     # (age, ekf_v, twopt_v, gt_v)
for i, (t, st) in enumerate(arr):
    if not st:
        continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] > GATE:
        continue
    sid, sx, sy, v = st[j]
    if prev and prev[0] == sid:
        age += 1
        dt = t - prev[1]
        tp = math.hypot(sx-prev[2], sy-prev[3])/dt if dt > 1e-4 else float("nan")
    else:
        age, tp = 1, float("nan")
    rows.append((age, v, tp, ospd[i]))
    prev = (sid, t, sx, sy)

R = [r for r in rows if not math.isnan(r[2])]
print(f"{'프레임':>5} {'n':>4} {'EKF v':>7} {'2점 v':>7} {'GT v':>6} | "
      f"{'EKF 오차':>8} {'2점 오차':>8}  더 나은 쪽")
for a in range(2, 13):
    s = [r for r in R if r[0] == a]
    if len(s) < 8:
        continue
    ek = np.median([abs(r[1]-r[3]) for r in s])
    tw = np.median([abs(r[2]-r[3]) for r in s])
    print(f"{a:>5} {len(s):4d} {np.median([r[1] for r in s]):7.2f} "
          f"{np.median([r[2] for r in s]):7.2f} {np.median([r[3] for r in s]):6.2f} | "
          f"{ek:8.2f} {tw:8.2f}  {'2점' if tw < ek else 'EKF'}")
s = [r for r in R if r[0] >= 21]
if len(s) > 8:
    ek = np.median([abs(r[1]-r[3]) for r in s]); tw = np.median([abs(r[2]-r[3]) for r in s])
    print(f"{'21+':>5} {len(s):4d} {np.median([r[1] for r in s]):7.2f} "
          f"{np.median([r[2] for r in s]):7.2f} {np.median([r[3] for r in s]):6.2f} | "
          f"{ek:8.2f} {tw:8.2f}  {'2점' if tw < ek else 'EKF'}")
early = [r for r in R if r[0] <= 5]
print(f"\n초기 5프레임: 2점 속도가 0.5 이상으로 나오는 비율 "
      f"{100*np.mean([r[2] >= 0.5 for r in early]):.0f}%  (EKF 는 "
      f"{100*np.mean([r[1] >= 0.5 for r in early]):.0f}%)")
print(f"  2점 속도 노이즈: GT 대비 |오차| p90 {np.percentile([abs(r[2]-r[3]) for r in early],90):.2f} m/s")
