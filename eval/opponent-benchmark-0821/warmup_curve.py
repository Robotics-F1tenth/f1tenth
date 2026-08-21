#!/usr/bin/env python3
"""How many frames does a re-acquired track need before its speed is usable?

Answers the concrete design question: if perception withholds (or flags) speed
for the first N frames of a track, what does N buy, and what does it cost?
"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

B = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE, STATIC_TH = 1.0, 0.5

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
gsp = np.hypot(np.gradient(G[:, 1], G[:, 0]), np.gradient(G[:, 2], G[:, 0]))
k = 15; gsp = np.convolve(gsp, np.ones(k)/k, mode="same")
ospd = np.interp(ts, G[:, 0], gsp)

# per-frame age within each matched id run
by_age = {}
prev_id, age = None, 0
for i, (t, st) in enumerate(arr):
    if not st:
        continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] > GATE:
        continue
    sid, _, _, v = st[j]
    age = 1 if sid != prev_id else age + 1
    prev_id = sid
    by_age.setdefault(min(age, 21), []).append((v, ospd[i],
                                                math.hypot(ox[i]-ex[i], oy[i]-ey[i])))

print(f"{'프레임':>5} {'n':>5} {'추정v p50':>10} {'GT v p50':>9} {'v<0.5 비율':>11} {'|오차| p50':>10}")
for a in sorted(by_age):
    rows = np.array(by_age[a])
    if len(rows) < 8:
        continue
    lab = f"{a}" if a < 21 else "21+"
    print(f"{lab:>5} {len(rows):5d} {np.median(rows[:,0]):10.2f} {np.median(rows[:,1]):9.2f} "
          f"{100*np.mean(rows[:,0] < STATIC_TH):10.0f}% {np.median(np.abs(rows[:,0]-rows[:,1])):10.2f}")

print("\n=== N 프레임 유예의 효과와 비용 ===")
allrows = [(a, v, g, rg) for a, L in by_age.items() for v, g, rg in L]
tot = len(allrows)
moving = [r for r in allrows if r[2] > 1.0]      # GT really moving
print(f"{'N':>3} {'억제되는 프레임':>13} {'그중 GT가 움직이는데 v<0.5':>26} {'억제 중 최근접거리':>18}")
for N in [0, 2, 3, 5, 8, 10]:
    supp = [r for r in allrows if r[0] <= N]
    bad = [r for r in supp if r[2] > 1.0 and r[1] < STATIC_TH]
    near = min([r[3] for r in supp], default=float("nan"))
    print(f"{N:>3} {len(supp):8d} ({100*len(supp)/tot:4.1f}%) {len(bad):20d} "
          f"({100*len(bad)/max(1,len(supp)):4.1f}%) {near:15.2f} m")
