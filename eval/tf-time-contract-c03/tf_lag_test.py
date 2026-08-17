#!/usr/bin/env python3
"""C-03 검증: perception 이 스캔 시각으로 TF 를 조회할 때 실제로 몇 ms 지연된 자세를
받는지를, bag 의 baked /tf 와 mocap GT 만으로 직접 측정한다.

TF 를 조회 시각 T + shift 로 당겨서 뽑았을 때 mocap 대비 오차가 최소가 되는 shift 가
곧 TF 라벨이 앞서 있는 양이다. shift=0 이 최소면 지연이 없는 것."""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

BAG = sys.argv[1] if len(sys.argv) > 1 else "/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38"

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

so = rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3")
co = rosbag2_py.ConverterOptions("", "")
r = rosbag2_py.SequentialReader(); r.open(so, co)
r.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/mocap_pose1", "/scan"]))
tf, gt, scan = [], [], []
while r.has_next():
    tn, d, _ = r.read_next()
    if tn == "/tf":
        for t in deserialize_message(d, TFMessage).transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "base_link":
                s = t.header.stamp.sec + t.header.stamp.nanosec*1e-9
                tf.append([s, t.transform.translation.x, t.transform.translation.y,
                           yaw_of(t.transform.rotation)])
    elif tn == "/mocap_pose1":
        m = deserialize_message(d, PoseStamped)
        s = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        gt.append([s, m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)])
    elif tn == "/scan":
        m = deserialize_message(d, LaserScan)
        scan.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)

tf = np.array(sorted(tf, key=lambda v: v[0]))
gt = np.array(sorted(gt, key=lambda v: v[0]))
scan = np.array(sorted(scan))
print(f"tf(map->base_link) {len(tf)}  mocap {len(gt)}  scan {len(scan)}")

# mocap 글리치 구간 제외 (물리적으로 불가능한 스텝 주변)
dts = np.diff(gt[:, 0]); dist = np.hypot(np.diff(gt[:,1]), np.diff(gt[:,2]))
bad_t = gt[1:, 0][dist > 0.15 + 8.0*dts]
def clean(times):
    keep = np.ones(len(times), bool)
    for b in bad_t: keep &= np.abs(times - b) > 1.5
    return keep

# 스캔 시각 중 tf/gt 범위 안쪽만
lo = max(tf[0,0], gt[0,0]) + 2.0
hi = min(tf[-1,0], gt[-1,0]) - 2.0
T = scan[(scan > lo) & (scan < hi)]
T = T[clean(T)]
print(f"평가 스캔 {len(T)}개 (글리치 제외 후)\n")

gx = np.interp(T, gt[:,0], gt[:,1]); gy = np.interp(T, gt[:,0], gt[:,2])

print("TF 조회 시각을 T+shift 로 옮겼을 때 mocap GT 대비 위치오차")
print(f"{'shift[ms]':>10} {'p50[cm]':>9} {'p90[cm]':>9}")
best = None
for shift_ms in range(-20, 141, 10):
    s = shift_ms * 1e-3
    x = np.interp(T + s, tf[:,0], tf[:,1]); y = np.interp(T + s, tf[:,0], tf[:,2])
    e = np.hypot(x - gx, y - gy)
    p50 = np.percentile(e, 50)*100
    mark = ""
    if best is None or p50 < best[1]: best = (shift_ms, p50); mark = ""
    print(f"{shift_ms:>10} {p50:>9.2f} {np.percentile(e,90)*100:>9.2f}")
print(f"\n=> 오차 최소가 되는 shift = {best[0]} ms  (p50 {best[1]:.2f} cm)")
print("   이 값이 곧 'TF 라벨이 실제 유효시각보다 앞서 있는 양'")
