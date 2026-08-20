import rosbag2_py, numpy as np, math
from rclpy.serialization import deserialize_message
from f1_msgs.msg import F1stateArr
from geometry_msgs.msg import PoseStamped
B="/home/a/Downloads/f1tenth_bag/0820/rosbag2_2026_08_20-04_08_16"
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B,storage_id="sqlite3"),rosbag2_py.ConverterOptions("",""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/f1/perception/static_obstacles","/newmcl_pose"]))
frames=[]; P=[]
while r.has_next():
    tn,d,_=r.read_next()
    if tn=="/newmcl_pose":
        m=deserialize_message(d,PoseStamped); q=m.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        P.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.position.x,m.pose.position.y,yaw))
    else:
        m=deserialize_message(d,F1stateArr)
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        frames.append((t,{int(s.id):(s.x,s.y) for s in m.f1_state_arr}))
P=np.array(P); P=P[np.argsort(P[:,0])]; frames.sort(key=lambda f:f[0])
seen={}
for t,ids in frames:
    for i,(x,y) in ids.items(): seen.setdefault(i,[]).append((t,x,y))
tend=frames[-1][0]
bear=[]
for i,v in seen.items():
    ts=np.array([a[0] for a in v]); tlast=ts[-1]
    if tend-tlast<2.0: continue
    x,y=np.mean([a[1] for a in v]),np.mean([a[2] for a in v])
    k=min(max(np.searchsorted(P[:,0],tlast),0),len(P)-1)
    ex,ey,eyaw=P[k,1],P[k,2],P[k,3]
    b=math.degrees(math.atan2(y-ey,x-ex)-eyaw)
    b=(b+180)%360-180
    bear.append(abs(b))
bear=np.array(bear)
print(f"삭제 시점 장애물 방위각 |bearing| (0=정면, 180=바로 뒤), n={len(bear)}")
for lo,hi,lab in [(0,45,"정면 ±45"),(45,90,"측면 45~90"),(90,135,"후측방 90~135"),(135,181,"후방 135~180")]:
    c=int(((bear>=lo)&(bear<hi)).sum())
    print(f"  {lab:16s} {c:3d}건 ({100*c/len(bear):5.1f}%)  {'#'*int(40*c/len(bear))}")
print(f"\n  |bearing| > 90도 (= ego 뒤쪽): {int((bear>90).sum())}/{len(bear)} = {100*(bear>90).mean():.1f}%")
