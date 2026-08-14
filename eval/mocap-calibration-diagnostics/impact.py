#!/usr/bin/env python3
"""T_map_mocap 불일치가 실제 주행영역에서 만드는 좌표계 어긋남 정량화."""
import math
import numpy as np, yaml
from chain_check import Cal, read, yaw_of, BASE
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped

BAG = "/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38"
data = read(BAG, ["/rigid_bodies"], {"/rigid_bodies": RigidBodies})
c1, c2 = Cal(BASE+"params.yaml"), Cal(BASE+"params2.yaml")

# 두 T_map_mocap을 "같은" mocap 점들에 적용했을 때의 차이 (좌표계 어긋남 필드)
for body, cal_own in [("6", c1), ("7", c2)]:
    P = []
    for _, msg in data["/rigid_bodies"]:
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == body:
                P.append([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z])
    P = np.array(P)
    Pa = (c1.Raxis @ P.T).T           # 축 변환은 두 캘리 동일(y_up)
    m1 = (c1.R @ Pa.T).T + c1.t       # car1의 T_map_mocap 적용
    m2 = (c2.R @ Pa.T).T + c2.t       # car2의 T_map_mocap 적용
    d = np.linalg.norm(m1[:, :2] - m2[:, :2], axis=1)
    print(f"body '{body}' 주행영역에서 두 T_map_mocap 결과 차이:")
    print(f"   p50={np.percentile(d,50)*100:6.2f}cm  p95={np.percentile(d,95)*100:6.2f}cm  max={d.max()*100:6.2f}cm")

# 맵 프레임에서의 실제 주행 범위
P6 = np.array([[rb.pose.position.x, rb.pose.position.y, rb.pose.position.z]
               for _, msg in data["/rigid_bodies"] for rb in msg.rigidbodies if rb.rigid_body_name=="6"])
m6 = (c1.R @ (c1.Raxis @ P6.T)).T + c1.t
print(f"\nmap 프레임 주행범위: x[{m6[:,0].min():.2f},{m6[:,0].max():.2f}] y[{m6[:,1].min():.2f},{m6[:,1].max():.2f}]")

# 두 차 사이 거리는 SE(2)에 불변 -> 같은 프레임 쓰면 보존, 다른 프레임 쓰면 왜곡
pairs = []
for _, msg in data["/rigid_bodies"]:
    d = {rb.rigid_body_name: rb for rb in msg.rigidbodies}
    if "6" in d and "7" in d:
        p6 = np.array([d["6"].pose.position.x, d["6"].pose.position.y, d["6"].pose.position.z])
        p7 = np.array([d["7"].pose.position.x, d["7"].pose.position.y, d["7"].pose.position.z])
        pairs.append((p6, p7))
print(f"\n동시 관측 프레임 수: {len(pairs)}")
raw_d, cal_d = [], []
for p6, p7 in pairs:
    a6 = c1.Raxis @ p6; a7 = c1.Raxis @ p7
    raw_d.append(np.linalg.norm((a6-a7)[:2]))
    q6 = c1.R @ a6 + c1.t          # car1 캘리 적용
    q7 = c2.R @ a7 + c2.t          # car2 캘리 적용
    cal_d.append(np.linalg.norm((q6-q7)[:2]))
raw_d, cal_d = np.array(raw_d), np.array(cal_d)
err = cal_d - raw_d
print("두 차 간 거리 — mocap 원본 vs 각자 캘리 적용 후 (SE(2)면 보존돼야 함):")
print(f"   원본 거리 p50={np.percentile(raw_d,50):.3f}m")
print(f"   캘리 후 거리 오차: p50={np.percentile(np.abs(err),50)*100:.2f}cm  p95={np.percentile(np.abs(err),95)*100:.2f}cm  max={np.abs(err).max()*100:.2f}cm")
print(f"   (bias={err.mean()*100:+.2f}cm — 서로 다른 프레임을 쓴 탓에 생기는 계통 왜곡)")
