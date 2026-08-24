#!/usr/bin/env python3
"""Is the opponent's x/y/yaw oscillation our noise, or is the car really moving?

Control is fitting a predicted trajectory to these, so the question is whether
the frame-to-frame variation carries signal. Compares the published track's
step-to-step change against GT's over the same intervals, within one track id so
re-acquisitions are not counted as jumps.
"""
import math
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
r.set_filter(rosbag2_py.StorageFilter(topics=["/mocap/car3/pose",
                                              "/f1/perception/object/obstacles/arr"]))
opp, arr = [], []
while r.has_next():
    tn, d, _ = r.read_next()
    if tn == "/mocap/car3/pose":
        m = deserialize_message(d, PoseStamped)
        opp.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    m.pose.position.x, m.pose.position.y))
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    [(int(s.id), s.x, s.y, s.yaw, abs(s.v)) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); arr.sort()
ts = np.array([a[0] for a in arr])
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ker = np.ones(15)/15
gvx = np.convolve(np.gradient(G[:, 1], G[:, 0]), ker, mode="same")
gvy = np.convolve(np.gradient(G[:, 2], G[:, 0]), ker, mode="same")
gc = np.interp(ts, G[:, 0], np.unwrap(np.arctan2(gvy, gvx)))

seq, prev, age = [], None, 0
for i, (t, st) in enumerate(arr):
    if not st: prev = None; continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] > GATE: prev = None; continue
    sid, sx, sy, syaw, sv = st[j]
    age = 1 if sid != prev else age+1; prev = sid
    seq.append((i, t, sid, age, sx, sy, syaw, sv, ox[i], oy[i], gc[i]))

def w(a): return math.atan2(math.sin(a), math.cos(a))
ep, gp, ey, gy, dts = [], [], [], [], []
for k in range(1, len(seq)):
    a, b = seq[k-1], seq[k]
    if b[2] != a[2] or b[3] != a[3]+1:   # same id, consecutive frame
        continue
    dt = b[1]-a[1]
    if not (1e-3 < dt < 0.1): continue
    dts.append(dt)
    ep.append(math.hypot(b[4]-a[4], b[5]-a[5]))
    gp.append(math.hypot(b[8]-a[8], b[9]-a[9]))
    ey.append(abs(math.degrees(w(b[6]-a[6]))))
    gy.append(abs(math.degrees(w(b[10]-a[10]))))
ep, gp, ey, gy, dts = map(np.array, (ep, gp, ey, gy, dts))
print(f"연속 프레임 쌍 {len(ep)}개, dt p50 {1000*np.median(dts):.1f} ms "
      f"({1/np.median(dts):.0f} Hz)\n")
print(f"{'':>14} {'p50':>9} {'p90':>9} {'max':>9}")
print(f"{'추정 위치변화':>14} {np.median(ep)*100:8.1f}cm {np.percentile(ep,90)*100:8.1f}cm {ep.max()*100:8.1f}cm")
print(f"{'GT 위치변화':>14} {np.median(gp)*100:8.1f}cm {np.percentile(gp,90)*100:8.1f}cm {gp.max()*100:8.1f}cm")
print(f"{'추정 yaw변화':>14} {np.median(ey):8.1f}° {np.percentile(ey,90):8.1f}° {ey.max():8.1f}°")
print(f"{'GT yaw변화':>14} {np.median(gy):8.1f}° {np.percentile(gy,90):8.1f}° {gy.max():8.1f}°")
print(f"\n  위치: 추정이 GT 의 {np.median(ep)/max(np.median(gp),1e-9):.2f} 배로 움직임")
print(f"  yaw : 추정이 GT 의 {np.median(ey)/max(np.median(gy),1e-9):.2f} 배로 움직임")
ex = ep - gp
print(f"\n  초과 이동(추정−GT) p50 {np.median(ex)*100:+.1f}cm  p90 {np.percentile(ex,90)*100:+.1f}cm")
print(f"  -> 이 초과분이 프레임간 흔들림(jitter)이고, 궤적 적합의 노이즈 바닥이다")

# yaw jitter vs speed: course is undefined at low speed
sp = np.array([seq[k][7] for k in range(1, len(seq))
               if seq[k][2] == seq[k-1][2] and seq[k][3] == seq[k-1][3]+1
               and 1e-3 < seq[k][1]-seq[k-1][1] < 0.1])
print(f"\n=== yaw 흔들림은 속도에 의존한다 ===")
for lo, hi in [(0,0.5),(0.5,1.0),(1.0,2.0),(2.0,99)]:
    m = (sp>=lo)&(sp<hi)
    if m.sum() < 15: continue
    print(f"  추정속도 {lo:.1f}-{hi:.1f} m/s (n={int(m.sum()):4d}): "
          f"yaw 변화 p50 {np.median(ey[m]):5.1f}°  p90 {np.percentile(ey[m],90):6.1f}°")
