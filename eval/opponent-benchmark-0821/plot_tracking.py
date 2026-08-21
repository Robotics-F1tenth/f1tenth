#!/usr/bin/env python3
"""Opponent tracking: error numbers plus estimate-vs-GT plots.

Also breaks yaw and speed error down by track age, because the aggregate MAE is
dominated by the frames right after a re-acquisition and quoting it alone
misrepresents steady-state accuracy.
"""
import sys, math, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

B = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15"
GATE = 1.0
OUT = "tracking_plots.png"


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=[
    "/mocap/car3/pose", "/newmcl_pose", "/f1/perception/object/obstacles/arr"]))
opp, ego, arr = [], [], []
while r.has_next():
    tn, d, _ = r.read_next()
    if tn in ("/mocap/car3/pose", "/newmcl_pose"):
        m = deserialize_message(d, PoseStamped)
        (opp if tn == "/mocap/car3/pose" else ego).append(
            (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
             m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)))
    else:
        m = deserialize_message(d, F1stateArr)
        arr.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                    [(int(s.id), s.x, s.y, s.yaw, abs(s.v)) for s in m.f1_state_arr]))
G = np.array(sorted(opp)); E = np.array(sorted(ego)); arr.sort()
ts = np.array([a[0] for a in arr])
t0 = ts[0]
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ex = np.interp(ts, E[:, 0], E[:, 1]); ey = np.interp(ts, E[:, 0], E[:, 2])
k = 15; ker = np.ones(k)/k
gvx = np.convolve(np.gradient(G[:, 1], G[:, 0]), ker, mode="same")
gvy = np.convolve(np.gradient(G[:, 2], G[:, 0]), ker, mode="same")
ospd = np.interp(ts, G[:, 0], np.hypot(gvx, gvy))
ocourse = np.interp(ts, G[:, 0], np.unwrap(np.arctan2(gvy, gvx)))

rows = []   # t, age, pos_err, yaw_err_deg, est_yaw, gt_course, est_v, gt_v, range
prev_id, age = None, 0
for i, (t, st) in enumerate(arr):
    if not st:
        continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] > GATE:
        continue
    sid, sx, sy, syaw, sv = st[j]
    age = 1 if sid != prev_id else age+1
    prev_id = sid
    rows.append((t-t0, age, d[j], abs(math.degrees(wrap(syaw-ocourse[i]))),
                 syaw, ocourse[i], sv, ospd[i], math.hypot(ox[i]-ex[i], oy[i]-ey[i])))
R = np.array(rows)
T, AGE, PE, YE, EY, GY, EV, GV, RG = (R[:, c] for c in range(9))

def p(a, q): return float(np.percentile(a, q))

print(f"매칭 {len(R)}건\n")
print("=== 전체 ===")
print(f"  위치   mean {PE.mean():.3f}  p50 {p(PE,50):.3f}  p90 {p(PE,90):.3f}  rmse {math.sqrt((PE**2).mean()):.3f} m")
print(f"  yaw    MAE {YE.mean():.1f}  p50 {p(YE,50):.1f}  p90 {p(YE,90):.1f} deg")
print(f"  속도   bias {(EV-GV).mean():+.2f}  MAE {np.abs(EV-GV).mean():.2f}  p90 {p(np.abs(EV-GV),90):.2f} m/s")

print("\n=== 트랙 나이별 (집계값이 왜 나쁜지) ===")
print(f"  {'구간':>12} {'n':>5} {'위치p50':>8} {'yawMAE':>8} {'yaw p50':>8} {'속도MAE':>8}")
for lo, hi, lab in [(1,5,"1-5 (재획득)"),(6,20,"6-20 (수렴중)"),(21,10**9,"21+ (정상)")]:
    m = (AGE >= lo) & (AGE <= hi)
    if m.sum() < 10: continue
    print(f"  {lab:>12} {int(m.sum()):5d} {p(PE[m],50):8.3f} {YE[m].mean():8.1f} "
          f"{p(YE[m],50):8.1f} {np.abs(EV-GV)[m].mean():8.2f}")

st = AGE >= 21
print(f"\n  정상구간만: yaw MAE {YE[st].mean():.1f}deg p50 {p(YE[st],50):.1f}  "
      f"속도 MAE {np.abs(EV-GV)[st].mean():.2f}  위치 p50 {p(PE[st],50):.3f} m")

print("\n=== vel_thresh 를 올리면 어떻게 되나 (정적 판정 = |v| < thresh) ===")
print(f"  상대차 GT 속도 p50 {np.median(GV):.2f} m/s -- 명백히 동적")
for th in [0.3, 0.5, 1.0, 2.0, 3.0]:
    frac = 100*np.mean(EV < th)
    frac_st = 100*np.mean(EV[st] < th)
    print(f"  thresh {th:.1f}: 상대차가 '정적'으로 분류되는 프레임 {frac:5.1f}% "
          f"(정상구간만 {frac_st:5.1f}%)")

fig, ax = plt.subplots(3, 2, figsize=(15, 11))
fig.suptitle(f"Opponent tracking vs mocap GT — {B.split('/')[-1]}  (n={len(R)})", fontsize=13)
a = ax[0, 0]
a.plot(T, GV, ".", ms=1.5, alpha=.5, label="GT speed", color="tab:green")
a.plot(T, EV, ".", ms=1.5, alpha=.5, label="estimate", color="tab:red")
a.axhline(0.5, ls="--", lw=.8, color="k", label="control static/dynamic 0.5")
a.set_xlabel("t [s]"); a.set_ylabel("speed [m/s]"); a.legend(fontsize=8); a.set_title("Speed: estimate vs GT")
a = ax[0, 1]
a.plot(T, np.degrees(np.unwrap(GY)) % 360, ".", ms=1.5, alpha=.5, label="GT course", color="tab:green")
a.plot(T, np.degrees(np.unwrap(EY)) % 360, ".", ms=1.5, alpha=.5, label="estimate", color="tab:red")
a.set_xlabel("t [s]"); a.set_ylabel("yaw [deg]"); a.legend(fontsize=8); a.set_title("Yaw: estimate vs GT course")
a = ax[1, 0]
a.hist(PE, bins=60, color="tab:blue"); a.axvline(p(PE,50), color="k", ls="--", lw=.8)
a.set_xlabel("position error [m]"); a.set_title(f"Position error (p50 {p(PE,50):.3f} m)")
a = ax[1, 1]
a.hist(YE, bins=60, color="tab:orange"); a.axvline(p(YE,50), color="k", ls="--", lw=.8)
a.set_xlabel("|yaw error| [deg]"); a.set_title(f"Yaw error (MAE {YE.mean():.1f}, p50 {p(YE,50):.1f} deg)")
a = ax[2, 0]
bins = [1,2,3,4,5,6,8,10,13,16,20,25,32,40,60,100,10**9]
xs, ys, zs = [], [], []
for lo, hi in zip(bins[:-1], bins[1:]):
    m = (AGE >= lo) & (AGE < hi)
    if m.sum() < 8: continue
    xs.append(lo); ys.append(np.median(EV[m])); zs.append(np.median(GV[m]))
a.plot(xs, ys, "o-", label="estimate p50", color="tab:red")
a.plot(xs, zs, "s-", label="GT p50", color="tab:green")
a.axhline(0.5, ls="--", lw=.8, color="k")
a.set_xscale("log"); a.set_xlabel("track age [frames]"); a.set_ylabel("speed [m/s]")
a.legend(fontsize=8); a.set_title("Speed convergence after re-acquisition")
a = ax[2, 1]
for lo, hi, lab, c in [(1,5,"age 1-5","tab:red"),(6,20,"age 6-20","tab:orange"),(21,10**9,"age 21+","tab:green")]:
    m = (AGE >= lo) & (AGE <= hi)
    if m.sum() < 10: continue
    a.hist(YE[m], bins=40, alpha=.55, label=f"{lab} (MAE {YE[m].mean():.0f}deg)", color=c)
a.set_xlabel("|yaw error| [deg]"); a.legend(fontsize=8); a.set_title("Yaw error by track age")
plt.tight_layout()
plt.savefig(OUT, dpi=110)
print(f"\n그래프 -> {OUT}")
