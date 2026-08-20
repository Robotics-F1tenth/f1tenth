import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr
B="/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B,storage_id="sqlite3"),rosbag2_py.ConverterOptions("",""))
T=["/mocap/car3/pose","/mocap/car4/pose","/newmcl_pose","/f1/perception/object/obstacles/arr"]
r.set_filter(rosbag2_py.StorageFilter(topics=T))
P={t:[] for t in T[:3]}; arr=[]
while r.has_next():
    tn,d,_=r.read_next()
    if tn in P:
        m=deserialize_message(d,PoseStamped)
        P[tn].append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.position.x,m.pose.position.y))
    else:
        m=deserialize_message(d,F1stateArr)
        arr.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    [(s.x,s.y) for s in m.f1_state_arr]))
for t in P: P[t]=np.array(sorted(P[t]))
arr.sort()
print("=== 시각 범위 (bag 시작 기준 초) ===")
t0=min(P[t][0,0] for t in P)
for t in P: print(f"  {t:22} {P[t][0,0]-t0:7.1f} ~ {P[t][-1,0]-t0:7.1f}  n={len(P[t])}")
print(f"  {'obstacles/arr':22} {arr[0][0]-t0:7.1f} ~ {arr[-1][0]-t0:7.1f}  n={len(arr)}")
ts=np.array([a[0] for a in arr])
ox=np.interp(ts,P["/mocap/car3/pose"][:,0],P["/mocap/car3/pose"][:,1])
oy=np.interp(ts,P["/mocap/car3/pose"][:,0],P["/mocap/car3/pose"][:,2])
ex=np.interp(ts,P["/newmcl_pose"][:,0],P["/newmcl_pose"][:,1])
ey=np.interp(ts,P["/newmcl_pose"][:,0],P["/newmcl_pose"][:,2])
rng=np.hypot(ox-ex,oy-ey)
print(f"\n=== ego -> 상대 실제 거리 ===")
print(f"  p10 {np.percentile(rng,10):.2f}  p50 {np.median(rng):.2f}  p90 {np.percentile(rng,90):.2f}  max {rng.max():.2f} m")
ntr=np.array([len(a[1]) for a in arr])
print(f"\n=== arr 트랙 수 ===  p50 {np.median(ntr):.0f}  mean {ntr.mean():.1f}  max {ntr.max()}  빈 사이클 {int((ntr==0).sum())}")
best=[]
for i,(t,st) in enumerate(arr):
    if not st: best.append(np.nan); continue
    best.append(min(math.hypot(x-ox[i],y-oy[i]) for x,y in st))
best=np.array(best); ok=~np.isnan(best)
print(f"\n=== 상대 GT 에 가장 가까운 트랙까지 거리 ===")
for g in [0.5,1.0,1.5,2.0,3.0]:
    print(f"  <{g:.1f}m : {100*np.mean(best[ok]<g):5.1f}%")
print(f"  p50 {np.median(best[ok]):.2f}  p90 {np.percentile(best[ok],90):.2f} m")
# 거리 구간별 매칭률
print(f"\n=== 실제거리 구간별, 1.0m 내 매칭률 ===")
for lo,hi in [(0,2),(2,4),(4,6),(6,8),(8,99)]:
    s=ok&(rng>=lo)&(rng<hi)
    if s.sum()<10: continue
    print(f"  {lo}-{hi}m: n={int(s.sum()):5d}  매칭 {100*np.mean(best[s]<1.0):5.1f}%")
