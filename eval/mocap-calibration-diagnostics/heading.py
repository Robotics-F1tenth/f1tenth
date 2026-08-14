#!/usr/bin/env python3
"""직진 구간(|omega| 작음 -> 슬립 무시가능)만 써서 헤딩 오프셋 b를 깨끗하게 추정.
직진에서는 진행방향 == 차체 헤딩 이므로 b = mean(속도방향 - mocap yaw). 레버암은 omega~0이라 영향 없음."""
import math
import numpy as np
from chain_check import Cal, read, yaw_of, q_mul, mat2quat, BASE
from mocap4r2_msgs.msg import RigidBodies

BAG = "/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38"
data = read(BAG, ["/rigid_bodies"], {"/rigid_bodies": RigidBodies})
c1 = Cal(BASE+"params.yaml"); c2 = Cal(BASE+"params2.yaml")
qax = mat2quat(c1.Raxis)
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi

for body, cal in [("6", c1), ("7", c2)]:
    T,P,Y = [],[],[]
    for t,msg in data["/rigid_bodies"]:
        for rb in msg.rigidbodies:
            if rb.rigid_body_name==body:
                p = c1.Raxis @ np.array([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z])
                q = np.array([rb.pose.orientation.w, rb.pose.orientation.x, rb.pose.orientation.y, rb.pose.orientation.z])
                qa = q_mul(qax,q); qa/=np.linalg.norm(qa)
                T.append(t); P.append(p[:2]); Y.append(yaw_of(qa))
    T,P,Y = np.array(T), np.array(P), np.unwrap(np.array(Y))
    st = np.r_[0, np.linalg.norm(np.diff(P,axis=0),axis=1)]
    k = st<0.05; T,P,Y = T[k],P[k],Y[k]
    dt = np.diff(T); v = np.diff(P,axis=0)/dt[:,None]
    spd = np.linalg.norm(v,axis=1); om = np.diff(Y)/dt
    ym = 0.5*(Y[:-1]+Y[1:])
    course = np.arctan2(v[:,1], v[:,0])
    dev = np.array([wrap(cc-yy) for cc,yy in zip(course, ym)])
    # 직진 + 충분한 속도만
    m = (spd>1.0)&(spd<8.0)&(np.abs(om)<0.15)&(dt>1e-4)
    print(f"=== body '{body}' ===")
    print(f"  전체 속도: p50={np.percentile(spd,50):.2f} p95={np.percentile(spd,95):.2f} m/s   |omega| p50={np.percentile(np.abs(om),50):.2f} rad/s")
    print(f"  직진구간 표본 {m.sum()}개 (spd>1, |omega|<0.15)")
    if m.sum() > 50:
        d = dev[m]
        b_est = math.atan2(np.sin(d).mean(), np.cos(d).mean())
        cur_b = math.atan2(cal.Rpb[1,0], cal.Rpb[0,0])
        print(f"  진행방향-mocap yaw 편차: 중앙값 {math.degrees(np.median(d)):+7.3f} deg, 원형평균 {math.degrees(b_est):+7.3f} deg, 표준편차 {math.degrees(np.std(d)):.2f} deg")
        print(f"  -> 필요한 헤딩보정 b = {math.degrees(b_est):+7.3f} deg   |  현재 params b = {math.degrees(cur_b):+7.3f} deg   |  차이 = {math.degrees(wrap(b_est-cur_b)):+7.3f} deg")
    print()
