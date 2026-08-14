#!/usr/bin/env python3
"""bag 전 궤적(17k 샘플)으로 body6 캘리브레이션 재적합 -> 현재 params와 비교.
calibration_v2.py와 동일한 6-파라미터 모델 (a,tx,ty = T_map_mocap / b,bx,by = T_point_base)."""
import math
import numpy as np
from chain_check import Cal, read, yaw_of, BASE
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped

BAG = "/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38"
data = read(BAG, ["/rigid_bodies", "/newmcl_pose"],
            {"/rigid_bodies": RigidBodies, "/newmcl_pose": PoseStamped})
c1 = Cal(BASE+"params.yaml")

def wrap(a): return (a + math.pi) % (2*math.pi) - math.pi

# body6 raw (축변환만 적용 = T_mocap_point)
mt, mp = [], []
for t, msg in data["/rigid_bodies"]:
    for rb in msg.rigidbodies:
        if rb.rigid_body_name == "6":
            p = c1.Raxis @ np.array([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z])
            q = np.array([rb.pose.orientation.w, rb.pose.orientation.x, rb.pose.orientation.y, rb.pose.orientation.z])
            # 축변환 후 yaw
            from chain_check import q_mul, mat2quat
            qa = q_mul(mat2quat(c1.Raxis), q); qa /= np.linalg.norm(qa)
            mt.append(t); mp.append([p[0], p[1], yaw_of(qa)])
mt, mp = np.array(mt), np.array(mp)

ct = np.array([t for t, _ in data["/newmcl_pose"]])
cp = np.array([[m.pose.position.x, m.pose.position.y,
                yaw_of([m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z])]
               for _, m in data["/newmcl_pose"]])

# mocap을 MCL 시각에 최근접 매칭
idx = np.searchsorted(mt, ct).clip(1, len(mt)-1)
prev = np.abs(ct - mt[idx-1]) < np.abs(ct - mt[idx]); idx -= prev.astype(int)
ok = np.abs(ct - mt[idx]) < 0.01
MOC, MCL = mp[idx[ok]], cp[ok]
# mocap 글리치 제거 (연속 스텝 변위 과대)
step = np.r_[0, np.linalg.norm(np.diff(MOC[:, :2], axis=0), axis=1)]
good = step < 0.05
MOC, MCL = MOC[good], MCL[good]
print(f"적합 샘플: {len(MOC)}개 (calibration_v2.py는 손으로 3~수십개 찍음)")

def predict(par, moc):
    a, tx, ty, b, bx, by = par
    c, s = math.cos(a), math.sin(a)
    x1 = tx + c*moc[:,0] - s*moc[:,1]; y1 = ty + s*moc[:,0] + c*moc[:,1]
    yaw1 = moc[:,2] + a
    c1_, s1_ = np.cos(yaw1), np.sin(yaw1)
    return np.c_[x1 + c1_*bx - s1_*by, y1 + s1_*bx + c1_*by, yaw1 + b]

def resid(par, moc, mcl):
    pr = predict(par, moc)
    return np.concatenate([pr[:,0]-mcl[:,0], pr[:,1]-mcl[:,1],
                           np.vectorize(wrap)(pr[:,2]-mcl[:,2])])

def fit(moc, mcl, fix_a=None):
    g = np.array([wrap(m-o) for m, o in zip(mcl[:,2], moc[:,2])])
    a0 = math.atan2(np.sin(g).mean(), np.cos(g).mean()) if fix_a is None else fix_a
    R = np.array([[math.cos(a0), -math.sin(a0)],[math.sin(a0), math.cos(a0)]])
    t0 = (mcl[:,:2] - (R @ moc[:,:2].T).T).mean(0)
    par = np.array([a0, t0[0], t0[1], 0.0, 0.0, 0.0])
    free = [1,2,3,4,5] if fix_a is not None else [0,1,2,3,4,5]
    lam = 1e-3
    for _ in range(200):
        r = resid(par, moc, mcl); cost = 0.5*r@r
        J = np.zeros((len(r), len(free)))
        for k, fi in enumerate(free):
            p2 = par.copy(); p2[fi] += 1e-7
            J[:, k] = (resid(p2, moc, mcl) - r)/1e-7
        H = J.T@J + lam*np.eye(len(free))
        dp = -np.linalg.solve(H, J.T@r)
        cand = par.copy(); cand[free] += dp
        cand[0] = wrap(cand[0]); cand[3] = wrap(cand[3])
        if 0.5*resid(cand,moc,mcl)@resid(cand,moc,mcl) < cost:
            par = cand; lam = max(1e-7, lam*0.7)
            if np.linalg.norm(dp) < 1e-10: break
        else:
            lam = min(1e4, lam*2.5)
    return par

def report(tag, par, moc, mcl):
    pr = predict(par, moc)
    e = np.linalg.norm(pr[:,:2]-mcl[:,:2], axis=1)
    ey = np.abs([wrap(v) for v in pr[:,2]-mcl[:,2]])
    print(f"  {tag:34s} a={math.degrees(par[0]):+8.4f}deg t=[{par[1]:+.4f},{par[2]:+.4f}] "
          f"b={math.degrees(par[3]):+7.4f}deg | 잔차 p50={np.percentile(e,50)*100:5.2f}cm "
          f"p95={np.percentile(e,95)*100:5.2f}cm yaw={math.degrees(np.percentile(ey,50)):.2f}deg")

print("\n=== body6(ego) 를 MCL 기준으로 적합 ===")
cur = np.array([math.atan2(c1.R[1,0], c1.R[0,0]), c1.t[0], c1.t[1],
                math.atan2(c1.Rpb[1,0], c1.Rpb[0,0]), c1.tpb[0], c1.tpb[1]])
report("현재 params.yaml (그대로 평가)", cur, MOC, MCL)
best = fit(MOC, MCL)
report("전궤적 재적합 (6-DOF 자유)", best, MOC, MCL)
c2 = Cal(BASE+"params2.yaml")
a2 = math.atan2(c2.R[1,0], c2.R[0,0])
shared = fit(MOC, MCL, fix_a=a2)
report("car2의 a로 고정하고 나머지 적합", shared, MOC, MCL)

print("\n=== a(T_map_mocap yaw) 민감도 스캔 — 나머지 5개는 매번 재적합 ===")
print("   a[deg]    잔차p50[cm]   (car1=-89.95, car2=-91.38, 재적합최적=-90.71)")
for a_deg in [-88.5, -89.0, -89.5, -89.95, -90.5, -90.71, -91.0, -91.38, -92.0, -93.0]:
    p = fit(MOC, MCL, fix_a=math.radians(a_deg))
    pr = predict(p, MOC)
    e = np.linalg.norm(pr[:,:2]-MCL[:,:2], axis=1)
    mark = ""
    if abs(a_deg+89.95)<0.01: mark = "  <- car1 현재값"
    if abs(a_deg+91.38)<0.01: mark = "  <- car2 현재값"
    if abs(a_deg+90.71)<0.01: mark = "  <- 전궤적 최적"
    print(f"  {a_deg:+8.2f}   {np.percentile(e,50)*100:7.3f}{mark}")
