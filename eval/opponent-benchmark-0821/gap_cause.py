#!/usr/bin/env python3
"""Was the opponent invisible during a gap, or visible and dropped?

A detection gap has two very different explanations and only one of them is a
bug: the lidar physically could not see the car (occluded by a mapped wall, or
behind the sensor's usable field), or it could and perception lost it anyway.
This raycasts the map between ego and opponent for every cycle of every gap.
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
GATE = 1.0
REAR = -1.0          # extractObj drops clusters at x < -1 in the lidar frame

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
T = ["/mocap/car3/pose", "/newmcl_pose", "/map",
     "/f1/perception/object/obstacles/arr"]
r.set_filter(rosbag2_py.StorageFilter(topics=T))
opp, ego, arr, grid, info = [], [], [], None, None
while r.has_next():
    tn, d, _ = r.read_next()
    if tn == "/map" and grid is None:
        m = deserialize_message(d, OccupancyGrid)
        info = m.info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
    elif tn in ("/mocap/car3/pose", "/newmcl_pose"):
        m = deserialize_message(d, PoseStamped)
        q = m.pose.orientation
        row = (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
               m.pose.position.x, m.pose.position.y,
               math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        (opp if tn == "/mocap/car3/pose" else ego).append(row)
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                    [(int(s.id), s.x, s.y) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); E = np.array(sorted(ego)); arr.sort()
res, ox0, oy0 = info.resolution, info.origin.position.x, info.origin.position.y
W, H = info.width, info.height
occ = grid > 0

def blocked(x0, y0, x1, y1):
    """True if a mapped wall lies between the two points (coarse ray march)."""
    d = math.hypot(x1-x0, y1-y0)
    n = max(2, int(d/(res*0.7)))
    for k in range(1, n):   # skip both endpoints
        t = k/n
        mx = int((x0 + (x1-x0)*t - ox0)/res)
        my = int((y0 + (y1-y0)*t - oy0)/res)
        if 0 <= mx < W and 0 <= my < H and occ[my, mx]:
            return True
    return False

ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ex = np.interp(ts, E[:, 0], E[:, 1]); ey = np.interp(ts, E[:, 0], E[:, 2])
eyaw = np.interp(ts, E[:, 0], np.unwrap(E[:, 3]))

seen = np.zeros(len(arr), bool)
for i, (t, st) in enumerate(arr):
    if st and min(math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st) <= GATE:
        seen[i] = True

idx = np.nonzero(seen)[0]
gaps = [(idx[k-1]+1, idx[k]) for k in range(1, len(idx)) if idx[k]-idx[k-1] > 1]
print(f"공백 {len(gaps)}개, 총 {sum(b-a for a,b in gaps)} 사이클")

cats = {"가려짐(맵 벽)": 0, "뒤쪽(x<-1)": 0, "가려짐+뒤쪽": 0, "보이는데 놓침": 0}
vis_rng, vis_len = [], []
for a, b in gaps:
    nb = ncat = 0
    for i in range(a, b):
        dx, dy = ox[i]-ex[i], oy[i]-ey[i]
        lx = math.cos(-eyaw[i])*dx - math.sin(-eyaw[i])*dy   # lidar-frame x
        bl = blocked(ex[i], ey[i], ox[i], oy[i])
        rear = lx < REAR
        if bl and rear: cats["가려짐+뒤쪽"] += 1
        elif bl:        cats["가려짐(맵 벽)"] += 1
        elif rear:      cats["뒤쪽(x<-1)"] += 1
        else:
            cats["보이는데 놓침"] += 1
            vis_rng.append(math.hypot(dx, dy)); ncat += 1
    if ncat: vis_len.append(ncat)

tot = sum(cats.values())
print(f"\n공백 사이클 {tot}건의 원인:")
for k, v in sorted(cats.items(), key=lambda x: -x[1]):
    print(f"  {k:16} {v:6d} ({100*v/tot:5.1f}%)  {'#'*int(40*v/tot)}")
if vis_rng:
    vr = np.array(vis_rng)
    print(f"\n'보이는데 놓침' 구간 거리: p50 {np.median(vr):.2f}  p90 {np.percentile(vr,90):.2f} m")
    print(f"  <4m 비율 {100*np.mean(vr<4):.0f}%   <2m 비율 {100*np.mean(vr<2):.0f}%")
    vl = np.array(vis_len)
    print(f"  공백당 연속 미검출 p50 {np.median(vl):.0f} 사이클 ({25*np.median(vl):.0f} ms)")

# 재획득이 얼마나 가까이서 일어나나 (5프레임 유예의 안전성 판단용)
reacq = [math.hypot(ox[b]-ex[b], oy[b]-ey[b]) for a, b in gaps]
if reacq:
    ra = np.array(reacq)
    print(f"\n재획득 시점 거리: p50 {np.median(ra):.2f}  p10 {np.percentile(ra,10):.2f} m")
    print(f"  2m 이내 재획득 {100*np.mean(ra<2):.0f}%   3m 이내 {100*np.mean(ra<3):.0f}%")
