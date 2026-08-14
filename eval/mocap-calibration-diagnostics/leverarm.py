#!/usr/bin/env python3
"""비홀로노믹 구속만으로 T_point_base 추정 — MCL/map 불필요, mocap 단독.
후륜축(base_link)에서는 횡속도가 0이어야 한다는 물리 구속을 이용.
  v_P_lat = v_B_lat + omega*dx  ~=  omega*dx   -> dx는 v_lat vs omega 회귀로 관측됨
따라서 (dx,dy,b)를 스캔해 재구성된 base_link 궤적의 횡속도를 최소화."""
import sys, math
import numpy as np
from chain_check import Cal, read, yaw_of, q_mul, mat2quat, BASE
from mocap4r2_msgs.msg import RigidBodies

BAG = sys.argv[1] if len(sys.argv) > 1 else "/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38"
data = read(BAG, ["/rigid_bodies"], {"/rigid_bodies": RigidBodies})
c1 = Cal(BASE+"params.yaml"); c2 = Cal(BASE+"params2.yaml")
qax = mat2quat(c1.Raxis)

def extract(body):
    T, P, Y = [], [], []
    for t, msg in data["/rigid_bodies"]:
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == body:
                p = c1.Raxis @ np.array([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z])
                q = np.array([rb.pose.orientation.w, rb.pose.orientation.x, rb.pose.orientation.y, rb.pose.orientation.z])
                qa = q_mul(qax, q); qa /= np.linalg.norm(qa)
                T.append(t); P.append(p[:2]); Y.append(yaw_of(qa))
    return np.array(T), np.array(P), np.unwrap(np.array(Y))

def lat_cost(par, T, P, Y, ret=False):
    dx, dy, b = par
    c, s = np.cos(Y), np.sin(Y)
    B = np.c_[P[:,0] + c*dx - s*dy, P[:,1] + s*dx + c*dy]   # base_link 위치
    yb = Y + b                                               # base_link 헤딩
    dt = np.diff(T)
    v = np.diff(B, axis=0) / dt[:, None]
    spd = np.linalg.norm(v, axis=1)
    ym = 0.5*(yb[:-1] + yb[1:])
    v_lat = -np.sin(ym)*v[:,0] + np.cos(ym)*v[:,1]
    ok = (spd > 0.5) & (spd < 8.0) & (dt > 1e-4) & (np.abs(np.diff(Y)/dt) < 6.0)
    if ret: return v_lat[ok], spd[ok]
    return np.mean(v_lat[ok]**2)

def fit(T, P, Y):
    par = np.array([0.0, 0.0, 0.0])
    step = np.array([0.2, 0.2, 0.2])
    best = lat_cost(par, T, P, Y)
    for _ in range(300):
        improved = False
        for i in range(3):
            for sgn in (1, -1):
                cand = par.copy(); cand[i] += sgn*step[i]
                cst = lat_cost(cand, T, P, Y)
                if cst < best: best, par, improved = cst, cand, True
        if not improved:
            step *= 0.5
            if step.max() < 1e-6: break
    return par, best

for body, cal in [("6", c1), ("7", c2)]:
    T, P, Y = extract(body)
    # 글리치 제거
    step = np.r_[0, np.linalg.norm(np.diff(P, axis=0), axis=1)]
    keep = step < 0.05
    T, P, Y = T[keep], P[keep], Y[keep]
    par, cost = fit(T, P, Y)
    v_lat0, _ = lat_cost(np.zeros(3), T, P, Y, ret=True)
    v_lat1, spd = lat_cost(par, T, P, Y, ret=True)
    cur = np.array([cal.tpb[0], cal.tpb[1], math.atan2(cal.Rpb[1,0], cal.Rpb[0,0])])
    v_latc, _ = lat_cost(cur, T, P, Y, ret=True)
    print(f"=== body '{body}'  (n={len(T)}, 유효 {len(v_lat1)}) ===")
    print(f"  보정 없음(tracked point 그대로): 횡속도 RMS = {np.sqrt(np.mean(v_lat0**2)):.4f} m/s")
    print(f"  현재 params T_point_base       : 횡속도 RMS = {np.sqrt(np.mean(v_latc**2)):.4f} m/s"
          f"   [dx={cur[0]:+.4f}, dy={cur[1]:+.4f}, b={math.degrees(cur[2]):+.2f}deg]")
    print(f"  비홀로노믹 적합(mocap 단독)     : 횡속도 RMS = {np.sqrt(np.mean(v_lat1**2)):.4f} m/s"
          f"   [dx={par[0]:+.4f}, dy={par[1]:+.4f}, b={math.degrees(par[2]):+.2f}deg]")
    print()
