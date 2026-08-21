#!/usr/bin/env python3
"""Detection rate over the cycles where detection was actually POSSIBLE.

The headline detection rate divides by every cycle, including the ones where the
opponent was occluded by a mapped wall or sat behind the lidar. Those are not
perception's to win, so they belong in the denominator only if the question is
"how often does control have the opponent" -- not if the question is "how good is
perception". This computes both.
"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from f1_msgs.msg import F1stateArr

B = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE, REAR = 1.0, -1.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=[
    "/mocap/car3/pose", "/newmcl_pose", "/map",
    "/f1/perception/object/obstacles/arr"]))
opp, ego, arr, grid, info = [], [], [], None, None
while r.has_next():
    tn, d, _ = r.read_next()
    if tn == "/map" and grid is None:
        m = deserialize_message(d, OccupancyGrid); info = m.info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
    elif tn in ("/mocap/car3/pose", "/newmcl_pose"):
        m = deserialize_message(d, PoseStamped); q = m.pose.orientation
        (opp if tn == "/mocap/car3/pose" else ego).append(
            (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
             m.pose.position.x, m.pose.position.y,
             math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                    [(s.x, s.y) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); E = np.array(sorted(ego)); arr.sort()
res, ox0, oy0 = info.resolution, info.origin.position.x, info.origin.position.y
W, H = info.width, info.height
occ = grid > 0

def blocked(x0, y0, x1, y1):
    d = math.hypot(x1-x0, y1-y0)
    n = max(2, int(d/(res*0.7)))
    for k in range(1, n):
        t = k/n
        mx = int((x0 + (x1-x0)*t - ox0)/res); my = int((y0 + (y1-y0)*t - oy0)/res)
        if 0 <= mx < W and 0 <= my < H and occ[my, mx]:
            return True
    return False

ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ex = np.interp(ts, E[:, 0], E[:, 1]); ey = np.interp(ts, E[:, 0], E[:, 2])
eyaw = np.interp(ts, E[:, 0], np.unwrap(E[:, 3]))

n_tot = len(arr)
n_occ = n_rear = n_vis = n_vis_hit = 0
rng_hit, rng_miss = [], []
for i, (t, st) in enumerate(arr):
    dx, dy = ox[i]-ex[i], oy[i]-ey[i]
    lx = math.cos(-eyaw[i])*dx - math.sin(-eyaw[i])*dy
    bl = blocked(ex[i], ey[i], ox[i], oy[i])
    rear = lx < REAR
    hit = bool(st) and min(math.hypot(s[0]-ox[i], s[1]-oy[i]) for s in st) <= GATE
    if bl:        n_occ += 1
    elif rear:    n_rear += 1
    else:
        n_vis += 1
        if hit:
            n_vis_hit += 1; rng_hit.append(math.hypot(dx, dy))
        else:
            rng_miss.append(math.hypot(dx, dy))

print(f"전체 사이클 {n_tot}")
print(f"  가려짐(맵 벽)      {n_occ:6d} ({100*n_occ/n_tot:5.1f}%)  <- 불가피")
print(f"  ego 뒤쪽(x<-1)     {n_rear:6d} ({100*n_rear/n_tot:5.1f}%)  <- 불가피")
print(f"  **관측 가능**      {n_vis:6d} ({100*n_vis/n_tot:5.1f}%)")
print()
print(f"=== 관측 가능한 {n_vis} 사이클 중 ===")
print(f"  트래킹 성공  {n_vis_hit:6d}  ({100*n_vis_hit/n_vis:5.1f}%)")
print(f"  놓침         {n_vis-n_vis_hit:6d}  ({100*(n_vis-n_vis_hit)/n_vis:5.1f}%)")
if rng_hit and rng_miss:
    h, m_ = np.array(rng_hit), np.array(rng_miss)
    print(f"\n  성공 시 거리 p50 {np.median(h):.2f}  p90 {np.percentile(h,90):.2f} m")
    print(f"  놓칠 시 거리 p50 {np.median(m_):.2f}  p90 {np.percentile(m_,90):.2f} m")
    print(f"\n  관측 가능 구간의 거리별 성공률:")
    for lo, hi in [(0,2),(2,4),(4,6),(6,8),(8,99)]:
        nh = int(((h>=lo)&(h<hi)).sum()); nm = int(((m_>=lo)&(m_<hi)).sum())
        if nh+nm < 20: continue
        print(f"    {lo:2d}-{hi:2d} m  n={nh+nm:5d}  성공 {100*nh/(nh+nm):5.1f}%")
