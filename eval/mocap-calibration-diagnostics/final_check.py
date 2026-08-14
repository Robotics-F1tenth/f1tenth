#!/usr/bin/env python3
"""최종 산출물(baked /mocap_pose1,2)에서 직진 구간 course vs heading 직접 확인."""
import math
import numpy as np
from chain_check import read, yaw_of
from geometry_msgs.msg import PoseStamped
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi

for BAG in ["/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38",
            "/home/a/Downloads/0730/rosbag2_2026_07_30-22_25_24"]:
    data = read(BAG, ["/mocap_pose1","/mocap_pose2"], {"/mocap_pose1":PoseStamped,"/mocap_pose2":PoseStamped})
    print(f"--- {BAG.split('-')[-1]} ---")
    for topic in ["/mocap_pose1","/mocap_pose2"]:
        A = np.array([[t, m.pose.position.x, m.pose.position.y,
                       yaw_of([m.pose.orientation.w,m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z])]
                      for t,m in data[topic]])
        if len(A) < 100: print(f"  {topic}: 데이터 부족"); continue
        T,P,Y = A[:,0], A[:,1:3], np.unwrap(A[:,3])
        st = np.r_[0, np.linalg.norm(np.diff(P,axis=0),axis=1)]
        k = st < 0.05; T,P,Y = T[k],P[k],Y[k]
        dt = np.diff(T); v = np.diff(P,axis=0)/np.maximum(dt,1e-6)[:,None]
        spd = np.linalg.norm(v,axis=1); om = np.diff(Y)/np.maximum(dt,1e-6)
        ym = 0.5*(Y[:-1]+Y[1:])
        dev = np.array([wrap(c-y) for c,y in zip(np.arctan2(v[:,1],v[:,0]), ym)])
        m = (spd>1.0)&(spd<8.0)&(np.abs(om)<0.15)&(dt>1e-4)
        d = dev[m]
        cm = math.atan2(np.sin(d).mean(), np.cos(d).mean())
        print(f"  {topic}: 직진표본 {m.sum():5d}  course-heading 편차 중앙값 {math.degrees(np.median(d)):+7.2f} deg  원형평균 {math.degrees(cm):+7.2f} deg  (0에 가까워야 정상)")
    print()
