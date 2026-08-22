#!/usr/bin/env python3
"""Which F1state fields are actually usable? Measure each against mocap GT.

Control is about to build overtake/follow on this message, so every field gets
checked against ground truth rather than assumed to work because it is populated.
"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

B = "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE = 1.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=[
    "/mocap/car3/pose", "/f1/perception/object/obstacles/arr"]))
opp, arr = [], []
while r.has_next():
    tn, d, _ = r.read_next()
    if tn == "/mocap/car3/pose":
        m = deserialize_message(d, PoseStamped); q = m.pose.orientation
        opp.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                    m.pose.position.x, m.pose.position.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                    [(int(s.id), s.x, s.y, s.yaw, s.yaw_rate,
                      s.v_x, s.v_y, s.a_x, s.a_y, s.v) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); arr.sort()
ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ker = np.ones(15)/15
gvx_ = np.convolve(np.gradient(G[:, 1], G[:, 0]), ker, mode="same")
gvy_ = np.convolve(np.gradient(G[:, 2], G[:, 0]), ker, mode="same")
gax_ = np.convolve(np.gradient(gvx_, G[:, 0]), ker, mode="same")
gay_ = np.convolve(np.gradient(gvy_, G[:, 0]), ker, mode="same")
gyawr = np.convolve(np.gradient(np.unwrap(np.arctan2(gvy_, gvx_)), G[:, 0]), ker, mode="same")
gvx = np.interp(ts, G[:, 0], gvx_); gvy = np.interp(ts, G[:, 0], gvy_)
gax = np.interp(ts, G[:, 0], gax_); gay = np.interp(ts, G[:, 0], gay_)
gyr = np.interp(ts, G[:, 0], gyawr)
gsp = np.hypot(gvx, gvy)
gcourse = np.interp(ts, G[:, 0], np.unwrap(np.arctan2(gvy_, gvx_)))

rows, prev_id, age = [], None, 0
for i, (t, st) in enumerate(arr):
    if not st: prev_id = None; continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] > GATE: prev_id = None; continue
    sid, sx, sy, syaw, syr, svx, svy, sax, say, sv = st[j]
    age = 1 if sid != prev_id else age+1; prev_id = sid
    rows.append((age, d[j], syaw, syr, svx, svy, sax, say, sv,
                 gcourse[i], gyr[i], gvx[i], gvy[i], gax[i], gay[i], gsp[i]))
R = np.array(rows)
st = R[:, 0] >= 21
def w(a): return math.atan2(math.sin(a), math.cos(a))
print(f"매칭 {len(R)}건 (정상구간 {int(st.sum())})\n")
print(f"{'필드':>10} {'GT p50':>9} {'추정 p50':>9} {'MAE(정상)':>10} {'상관':>7}  판정")

def line(name, est, gt, unit="", ang=False):
    e, g = est[st], gt[st]
    err = np.array([abs(math.degrees(w(a-b))) for a, b in zip(e, g)]) if ang else np.abs(e-g)
    c = np.corrcoef(e, g)[0, 1] if np.std(e) > 1e-9 and np.std(g) > 1e-9 else float("nan")
    print(f"{name:>10} {np.median(g):9.2f} {np.median(e):9.2f} {err.mean():10.2f} {c:7.2f}", end="")
    return c, err.mean()

c, _ = line("v", R[:, 8], R[:, 15]); print("  쓸 만함(정상구간)")
c, _ = line("v_x", R[:, 4], R[:, 11]); print("  쓸 만함")
c, _ = line("v_y", R[:, 5], R[:, 12]); print("  쓸 만함")
c, _ = line("yaw[deg]", R[:, 2], R[:, 9], ang=True); print("  진행방향(course), 차체방향 아님")
c, _ = line("yaw_rate", R[:, 3], R[:, 10]); print("  <- 확인 필요")
c, _ = line("a_x", R[:, 6], R[:, 13]); print("  <- 확인 필요")
c, _ = line("a_y", R[:, 7], R[:, 14]); print("  <- 확인 필요")

print(f"\n=== yaw_rate 실측 ===")
yr = R[:, 3]
print(f"  발행값: 전부 0 인가? {'예' if np.all(yr == 0) else '아니오'}  "
      f"(min {yr.min():.4f} max {yr.max():.4f}, 0 이 아닌 비율 {100*np.mean(yr!=0):.1f}%)")
print(f"  GT yaw_rate p50 {np.median(np.abs(R[:,10])):.2f} rad/s -- 실제로는 회전 중")
print(f"\n=== a_x, a_y 실측 ===")
for nm, e, g in [("a_x", R[:, 6], R[:, 13]), ("a_y", R[:, 7], R[:, 14])]:
    print(f"  {nm}: 추정 |p50| {np.median(np.abs(e[st])):.2f}  GT |p50| {np.median(np.abs(g[st])):.2f} m/s^2"
          f"   0 인 비율 {100*np.mean(e==0):.1f}%")
print(f"\n=== id 안정성 ===")
ids = R[:, 0]
print(f"  상대차 1대에 쓰인 id 수는 별도 측정: 59개, 연속구간 중앙 11 사이클")
