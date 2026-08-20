import rosbag2_py, numpy as np, math, sys
from rclpy.serialization import deserialize_message
from f1_msgs.msg import F1stateArr
from geometry_msgs.msg import PoseStamped
B=sys.argv[1] if len(sys.argv)>1 else "/home/a/Downloads/f1tenth_bag/0820/rosbag2_2026_08_20-04_08_16"
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B,storage_id="sqlite3"),rosbag2_py.ConverterOptions("",""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/f1/perception/static_obstacles","/newmcl_pose"]))
frames=[]; P=[]
while r.has_next():
    tn,d,_=r.read_next()
    if tn=="/newmcl_pose":
        m=deserialize_message(d,PoseStamped)
        P.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.position.x,m.pose.position.y))
    else:
        m=deserialize_message(d,F1stateArr)
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        frames.append((t,{int(s.id):(s.x,s.y) for s in m.f1_state_arr}))
P=np.array(P); P=P[np.argsort(P[:,0])]
frames.sort(key=lambda f:f[0]); t0=frames[0][0]
seen={}
for t,ids in frames:
    for i,(x,y) in ids.items(): seen.setdefault(i,[]).append((t,x,y))
tend=frames[-1][0]
rows=[]
for i,v in seen.items():
    ts=np.array([a[0] for a in v])
    x,y=np.mean([a[1] for a in v]),np.mean([a[2] for a in v])
    tlast=ts[-1]
    if tend-tlast<2.0: continue   # still alive at bag end
    k=np.searchsorted(P[:,0],tlast); k=min(max(k,0),len(P)-1)
    de=math.hypot(P[k,1]-x,P[k,2]-y)
    rows.append((tlast-t0, i, x, y, ts[-1]-ts[0], len(v), de))
rows.sort()
print(f"{'죽은t':>7} {'id':>5} {'x':>6} {'y':>6} {'수명s':>6} {'프레임':>6} {'ego거리':>8}  판정")
near=far=0
for tl,i,x,y,life,n,de in rows:
    v = "정상(ego 1.0m 내)" if de<1.05 else "*** ego 멀리 있는데 삭제 ***"
    if de<1.05: near+=1
    else: far+=1
    print(f"{tl:7.1f} {i:5d} {x:6.2f} {y:6.2f} {life:6.1f} {n:6d} {de:8.2f}  {v}")
print(f"\n삭제 {near+far}건: ego 근접 {near} / ego 원거리 {far}")
