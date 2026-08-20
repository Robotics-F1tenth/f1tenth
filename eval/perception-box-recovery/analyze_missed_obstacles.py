import rosbag2_py, numpy as np, sys, math
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from f1_msgs.msg import F1stateArr
B=sys.argv[1]
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B,storage_id="sqlite3"),rosbag2_py.ConverterOptions("",""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/scan","/newmcl_pose","/map","/f1/perception/static_obstacles"]))
scans=[]; poses=[]; g=None; info=None; dets={}
while r.has_next():
    tn,d,_=r.read_next()
    if tn=="/map":
        m=deserialize_message(d,OccupancyGrid); info=m.info
        g=np.array(m.data,dtype=np.int16).reshape(info.height,info.width)
    elif tn=="/newmcl_pose":
        m=deserialize_message(d,PoseStamped)
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        q=m.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        poses.append((t,m.pose.position.x,m.pose.position.y,yaw))
    elif tn=="/scan":
        m=deserialize_message(d,LaserScan)
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        scans.append((t,np.array(m.ranges,dtype=np.float32),m.angle_min,m.angle_increment,m.range_max))
    else:
        m=deserialize_message(d,F1stateArr)
        for s in m.f1_state_arr: dets.setdefault(int(s.id),[]).append((s.x,s.y))
P=np.array(poses); P=P[np.argsort(P[:,0])]
res=info.resolution; ox=info.origin.position.x; oy=info.origin.position.y
W=info.width; H=info.height
# accumulate lidar hits in map frame, count per cell
acc=np.zeros((H,W),dtype=np.int32)
step=3
for k in range(0,len(scans),step):
    t,rg,amin,ainc,rmax=scans[k]
    i=np.searchsorted(P[:,0],t)
    if i<=0 or i>=len(P): continue
    px,py,yaw=P[i,1],P[i,2],P[i,3]
    ang=amin+np.arange(len(rg))*ainc
    ok=np.isfinite(rg)&(rg>0.1)&(rg<min(rmax,12.0))
    rr=rg[ok]; aa=ang[ok]+yaw
    X=px+rr*np.cos(aa); Y=py+rr*np.sin(aa)
    mx=((X-ox)/res).astype(int); my=((Y-oy)/res).astype(int)
    v=(mx>=0)&(my>=0)&(mx<W)&(my<H)
    np.add.at(acc,(my[v],mx[v]),1)
np.save("/tmp/acc.npy",acc); np.save("/tmp/mapg.npy",g)
import json
json.dump({"res":res,"ox":ox,"oy":oy,"W":W,"H":H},open("/tmp/mapinfo.json","w"))
det_xy=[]
for i,v in dets.items():
    a=np.array(v); det_xy.append((a[:,0].mean(),a[:,1].mean(),len(v)))
json.dump(det_xy,open("/tmp/dets.json","w"))
print(f"scans={len(scans)} poses={len(poses)} 누적셀={int((acc>0).sum())}")
