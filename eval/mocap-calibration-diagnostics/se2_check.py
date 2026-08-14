#!/usr/bin/env python3
"""recon(Aug2 params) vs baked(Jul30 params) 차이가 순수 SE(2)인지 검정.
순수 SE(2)면 -> 체인 구현 정확, 차이는 캘리브레이션 값 차이일 뿐."""
import math
import numpy as np
from chain_check import Cal, read, yaw_of, q_mul, q_rot, mat2quat, BASE
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped

BAG = "/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38"
data = read(BAG, ["/rigid_bodies", "/mocap_pose1", "/mocap_pose2"],
            {"/rigid_bodies": RigidBodies, "/mocap_pose1": PoseStamped, "/mocap_pose2": PoseStamped})
cals = {"6": Cal(BASE+"params.yaml"), "7": Cal(BASE+"params2.yaml")}

def umeyama_se2(A, B):
    """A(Nx2) -> B(Nx2) 최적 회전+평행이동"""
    ca, cb = A.mean(0), B.mean(0)
    H = (A-ca).T @ (B-cb)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    t = cb - R @ ca
    return R, t

for body, topic in [("6", "/mocap_pose1"), ("7", "/mocap_pose2")]:
    cal = cals[body]
    rec_t, rec_p = [], []
    for t, msg in data["/rigid_bodies"]:
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == body:
                p = np.array([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z])
                q = np.array([rb.pose.orientation.w, rb.pose.orientation.x, rb.pose.orientation.y, rb.pose.orientation.z])
                po, qo = cal.apply(p, q)
                rec_t.append(t); rec_p.append([po[0], po[1], yaw_of(qo)])
    rec_t = np.array(rec_t); rec_p = np.array(rec_p)
    bak_t = np.array([t for t, _ in data[topic]])
    bak_p = np.array([[m.pose.position.x, m.pose.position.y,
                       yaw_of([m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z])]
                      for _, m in data[topic]])
    # 시간 기준 최근접 매칭
    idx = np.searchsorted(rec_t, bak_t).clip(1, len(rec_t)-1)
    prev = np.abs(bak_t - rec_t[idx-1]) < np.abs(bak_t - rec_t[idx])
    idx = idx - prev.astype(int)
    dt = np.abs(bak_t - rec_t[idx])
    ok = dt < 0.005
    A = rec_p[idx[ok]][:, :2]; B = bak_p[ok][:, :2]
    R, t = umeyama_se2(A, B)
    resid = np.linalg.norm((A @ R.T + t) - B, axis=1)
    yaw_fit = math.atan2(R[1,0], R[0,0])
    print(f"=== body '{body}' ({topic}) : Aug2 params 재현 vs Jul30 baked ===")
    print(f"  매칭 {ok.sum()}개 (dt<5ms)")
    print(f"  최적 SE(2) 정합: yaw {math.degrees(yaw_fit):+.3f} deg, t=[{t[0]:+.3f}, {t[1]:+.3f}]")
    print(f"  정합 후 잔차: p50={np.percentile(resid,50)*100:.2f}cm p95={np.percentile(resid,95)*100:.2f}cm max={resid.max()*100:.2f}cm")
    print()
