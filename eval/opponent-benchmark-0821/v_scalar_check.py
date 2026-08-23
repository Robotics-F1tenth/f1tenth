#!/usr/bin/env python3
"""Is scalar v really worse than its own components, or is that a correlation artifact?

Correlation measures explained variance, so a signal that swings widely scores
higher than a narrow one at the SAME absolute error. v_x/v_y swing through
positive and negative as the car goes round; |v| stays in a narrow band. Compare
on error terms, not on correlation.
"""
import math
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
                    [(int(s.id), s.x, s.y, s.v_x, s.v_y, s.v) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); arr.sort()
ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ker = np.ones(15)/15
gvx_ = np.convolve(np.gradient(G[:, 1], G[:, 0]), ker, mode="same")
gvy_ = np.convolve(np.gradient(G[:, 2], G[:, 0]), ker, mode="same")
gvx = np.interp(ts, G[:, 0], gvx_); gvy = np.interp(ts, G[:, 0], gvy_)
gsp = np.hypot(gvx, gvy)

rows, prev, age = [], None, 0
for i, (t, st) in enumerate(arr):
    if not st: prev = None; continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] > GATE: prev = None; continue
    sid, _, _, evx, evy, ev = st[j]
    age = 1 if sid != prev else age+1; prev = sid
    rows.append((age, evx, evy, ev, gvx[i], gvy[i], gsp[i]))
R = np.array(rows); st = R[:, 0] >= 21
E = R[st]
print(f"정상구간 n={len(E)}\n")
print(f"{'':>6} {'GT std':>8} {'MAE':>7} {'MAE/std':>8} {'상관':>7}")
for nm, ei, gi in [("v_x", 1, 4), ("v_y", 2, 5), ("v", 3, 6)]:
    e, g = E[:, ei], E[:, gi]
    mae = np.abs(e-g).mean(); sd = g.std()
    c = np.corrcoef(e, g)[0, 1]
    print(f"{nm:>6} {sd:8.2f} {mae:7.2f} {mae/sd:8.2f} {c:7.2f}")
print("\n=> GT 분산이 다르면 같은 오차라도 상관계수가 다르게 나온다.")

print("\n=== v 가 성분과 정합한가 (내부 일관성) ===")
recon = np.hypot(E[:, 1], E[:, 2])
print(f"  hypot(v_x,v_y) 와 발행된 v 의 차이: max {np.abs(recon-E[:,3]).max():.2e}"
      f"  -> {'동일한 값' if np.abs(recon-E[:,3]).max() < 1e-5 else '다름!'}")
print(f"  성분 오차로부터 기대되는 크기 오차: "
      f"sqrt(MAE_x^2+MAE_y^2) = {math.hypot(np.abs(E[:,1]-E[:,4]).mean(), np.abs(E[:,2]-E[:,5]).mean()):.2f} m/s")
print(f"  실제 v 의 MAE                     = {np.abs(E[:,3]-E[:,6]).mean():.2f} m/s")

print("\n=== 상대오차로 비교 (같은 축에 놓기) ===")
for nm, ei, gi in [("v_x", 1, 4), ("v_y", 2, 5), ("v", 3, 6)]:
    e, g = E[:, ei], E[:, gi]
    rng = np.percentile(g, 95) - np.percentile(g, 5)
    print(f"  {nm:>4}: MAE {np.abs(e-g).mean():.2f} / GT p5-p95 폭 {rng:.2f} = {100*np.abs(e-g).mean()/rng:.1f}%")

print("\n=== v 의 편향 (크기는 계통적으로 낮은가) ===")
bias = (E[:, 3]-E[:, 6])
print(f"  bias {bias.mean():+.2f} m/s   |bias|/GT p50 = {100*abs(bias.mean())/np.median(E[:,6]):.0f}%")
print(f"  성분 bias: v_x {(E[:,1]-E[:,4]).mean():+.2f}  v_y {(E[:,2]-E[:,5]).mean():+.2f}")
