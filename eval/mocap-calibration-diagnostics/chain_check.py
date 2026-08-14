#!/usr/bin/env python3
"""mocap_localization 체인을 raw /rigid_bodies에서 재현하고 baked pose와 대조."""
import sys, math
from pathlib import Path
import numpy as np, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped

BAG = sys.argv[1] if len(sys.argv) > 1 else "/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38"
BASE = str(Path(__file__).resolve().parents[2] /
           "src/localization/mocap_localization/params") + "/"

def q_mul(a, b):
    w1,x1,y1,z1 = a; w2,x2,y2,z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])

def q_rot(q, v):
    w,x,y,z = q
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                  [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                  [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
    return R @ v

def mat2quat(R):
    w = math.sqrt(max(0, 1+R[0,0]+R[1,1]+R[2,2]))/2
    x = math.sqrt(max(0, 1+R[0,0]-R[1,1]-R[2,2]))/2
    y = math.sqrt(max(0, 1-R[0,0]+R[1,1]-R[2,2]))/2
    z = math.sqrt(max(0, 1-R[0,0]-R[1,1]+R[2,2]))/2
    x = math.copysign(x, R[2,1]-R[1,2]); y = math.copysign(y, R[0,2]-R[2,0]); z = math.copysign(z, R[1,0]-R[0,1])
    return np.array([w,x,y,z])

def yaw_of(q):
    w,x,y,z = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

class Cal:
    def __init__(self, path):
        d = yaml.safe_load(open(path))["/**"]["ros__parameters"]
        self.name = d["rigid_body_name"]; self.out = d["output_topic"]
        self.R = np.array(d["rotation"]).reshape(3,3)
        self.t = np.array(d["translation"])
        self.Rpb = np.array(d["tracked_point_to_base_rotation"]).reshape(3,3)
        self.tpb = np.array(d["tracked_point_to_base_translation"])
        self.axes = d.get("input_axes", "z_up")
        self.Raxis = (np.array([[1,0,0],[0,0,-1],[0,1,0]], float)
                      if self.axes == "y_up" else np.eye(3))
        self.qaxis = mat2quat(self.Raxis); self.qR = mat2quat(self.R); self.qpb = mat2quat(self.Rpb)

    def apply(self, p, q_in):
        p1 = self.R @ (self.Raxis @ p) + self.t
        q1 = q_mul(self.qR, q_mul(self.qaxis, q_in)); q1 /= np.linalg.norm(q1)
        p_out = p1 + q_rot(q1, self.tpb)
        q_out = q_mul(q1, self.qpb); q_out /= np.linalg.norm(q_out)
        return p_out, q_out

def read(bag, topics, types):
    so = rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions("", "")
    r = rosbag2_py.SequentialReader(); r.open(so, co)
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = {t: [] for t in topics}
    while r.has_next():
        tn, d, ts = r.read_next()
        out[tn].append((ts*1e-9, deserialize_message(d, types[tn])))
    return out

cals = {"6": Cal(BASE+"params.yaml"), "7": Cal(BASE+"params2.yaml")}
data = read(BAG, ["/rigid_bodies", "/mocap_pose1", "/mocap_pose2"],
            {"/rigid_bodies": RigidBodies, "/mocap_pose1": PoseStamped, "/mocap_pose2": PoseStamped})

print(f"raw /rigid_bodies: {len(data['/rigid_bodies'])}  baked pose1: {len(data['/mocap_pose1'])}  pose2: {len(data['/mocap_pose2'])}")

# raw 좌표 범위 (mocap 프레임) — yaw 오차의 레버암 계산용
for body in ["6", "7"]:
    P = []
    for _, msg in data["/rigid_bodies"]:
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == body:
                P.append([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z])
    P = np.array(P)
    r_xy = np.hypot(P[:,0], P[:,2] if cals[body].axes=="y_up" else P[:,1])
    print(f"body '{body}': n={len(P)}  mocap 원점からの거리 |p| p50={np.percentile(r_xy,50):.2f}m p95={np.percentile(r_xy,95):.2f}m max={r_xy.max():.2f}m")

# 재현 vs baked 대조
for body, topic in [("6", "/mocap_pose1"), ("7", "/mocap_pose2")]:
    cal = cals[body]
    recon = []
    for t, msg in data["/rigid_bodies"]:
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == body:
                p = np.array([rb.pose.position.x, rb.pose.position.y, rb.pose.position.z])
                q = np.array([rb.pose.orientation.w, rb.pose.orientation.x, rb.pose.orientation.y, rb.pose.orientation.z])
                po, qo = cal.apply(p, q)
                recon.append([t, po[0], po[1], yaw_of(qo)])
    recon = np.array(recon)
    baked = np.array([[t, m.pose.position.x, m.pose.position.y,
                       yaw_of([m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z])]
                      for t, m in data[topic]])
    # 시간축 정렬 (인덱스 기준 — stamp가 now()라 raw와 다름)
    n = min(len(recon), len(baked))
    d = np.hypot(recon[:n,1]-baked[:n,1], recon[:n,2]-baked[:n,2])
    print(f"\n재현 vs baked {topic}: n={n}  위치차 p50={np.percentile(d,50)*100:.2f}cm p95={np.percentile(d,95)*100:.2f}cm max={d.max()*100:.1f}cm")
