#!/usr/bin/env python3
"""Opponent tracking vs mocap GT: numbers, plus plots you can actually read.

The bag is 386s but the opponent is only tracked in short bursts, so one
full-width scatter shows nothing. Layout instead: a context strip over the whole
run, then the longest continuously-tracked runs zoomed, where GT and estimate
can be compared frame by frame.

GT is drawn as a line because it exists on every cycle. The estimate is drawn
only where a track was matched, with gaps left open -- those gaps are the
detection dropouts and hiding them would misrepresent coverage.
"""
import sys, math
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
GATE, OUT, NZOOM = 1.0, "tracking_plots.png", 3


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
ts = np.array([a[0] for a in arr]); t0 = ts[0]
ox = np.interp(ts, G[:, 0], G[:, 1]); oy = np.interp(ts, G[:, 0], G[:, 2])
ker = np.ones(15)/15
gvx = np.convolve(np.gradient(G[:, 1], G[:, 0]), ker, mode="same")
gvy = np.convolve(np.gradient(G[:, 2], G[:, 0]), ker, mode="same")
ospd = np.interp(ts, G[:, 0], np.hypot(gvx, gvy))
ocourse_u = np.unwrap(np.arctan2(gvy, gvx))
ocourse = np.interp(ts, G[:, 0], ocourse_u)

n = len(arr)
est_v = np.full(n, np.nan); est_y = np.full(n, np.nan)
pos_e = np.full(n, np.nan); age_a = np.full(n, np.nan)
prev_id, age = None, 0
for i, (t, st) in enumerate(arr):
    if not st:
        prev_id = None
        continue
    d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in st]
    j = int(np.argmin(d))
    if d[j] > GATE:
        prev_id = None
        continue
    sid, sx, sy, syaw, sv = st[j]
    age = 1 if sid != prev_id else age+1
    prev_id = sid
    est_v[i] = sv; est_y[i] = syaw; pos_e[i] = d[j]; age_a[i] = age
T = ts - t0
ok = ~np.isnan(est_v)

# ---- numbers (unchanged) --------------------------------------------------
PE, EV, GV = pos_e[ok], est_v[ok], ospd[ok]
YE = np.array([abs(math.degrees(wrap(a-b))) for a, b in zip(est_y[ok], ocourse[ok])])
AGE = age_a[ok]
p = lambda a, q: float(np.percentile(a, q))
print(f"매칭 {int(ok.sum())}건")
print(f"  위치 mean {PE.mean():.3f} p50 {p(PE,50):.3f} | yaw MAE {YE.mean():.1f} p50 {p(YE,50):.1f}"
      f" | 속도 bias {(EV-GV).mean():+.2f} MAE {np.abs(EV-GV).mean():.2f}")
st = AGE >= 21
print(f"  정상구간(21+): 위치 p50 {p(PE[st],50):.3f}  yaw MAE {YE[st].mean():.1f}  "
      f"속도 MAE {np.abs(EV-GV)[st].mean():.2f}")

# ---- longest continuously-tracked runs ------------------------------------
runs, s = [], None
for i in range(n):
    if ok[i] and s is None:
        s = i
    elif not ok[i] and s is not None:
        runs.append((s, i)); s = None
if s is not None:
    runs.append((s, n))
runs.sort(key=lambda ab: ab[0]-ab[1])
top = sorted(runs[:NZOOM])
print(f"\n가장 긴 연속 추적 구간 {NZOOM}개: " +
      ", ".join(f"{T[a]:.0f}~{T[b-1]:.0f}s({b-a}프레임)" for a, b in top))

fig = plt.figure(figsize=(16, 12))
gs = fig.add_gridspec(4, NZOOM, height_ratios=[1.1, 1.1, 1.5, 1.5], hspace=.45, wspace=.22)
fig.suptitle(f"Opponent tracking vs mocap GT — {B.split('/')[-1]}   "
             f"(matched {int(ok.sum())} / {n} cycles)", fontsize=13)

# context strips: whole run, GT as a line, tracked stretches marked
axc = fig.add_subplot(gs[0, :])
axc.plot(T, ospd, "-", lw=.7, color="tab:green", label="GT speed (always available)")
axc.plot(T[ok], est_v[ok], ".", ms=2.2, color="tab:red", label="estimate (only where tracked)")
for a, b in top:
    axc.axvspan(T[a], T[b-1], color="tab:blue", alpha=.13)
axc.axhline(0.5, ls="--", lw=.7, color="k")
axc.set_ylabel("speed [m/s]"); axc.legend(fontsize=8, loc="upper right")
axc.set_title("Whole run — shaded = zoomed below", fontsize=10)

# Yaw over the whole run is useless as an absolute angle: unwrapping accumulates
# to -17000 deg over 60+ laps and wrapping to 0-360 turns it into a solid band.
# The ERROR is the readable quantity at this zoom, so plot that instead and keep
# absolute yaw for the zoom panels where it means something.
axy = fig.add_subplot(gs[1, :], sharex=axc)
yaw_err_all = np.full(n, np.nan)
yaw_err_all[ok] = YE
axy.plot(T[ok], yaw_err_all[ok], ".", ms=2.2, color="tab:red", label="|yaw error|")
young = ok & (age_a <= 5)
axy.plot(T[young], yaw_err_all[young], "o", ms=4, mfc="none", mec="tab:purple",
         mew=.9, label="track age<=5")
axy.axhline(np.median(YE), ls="--", lw=.8, color="k",
            label=f"p50 {np.median(YE):.0f} deg")
axy.set_ylabel("|yaw error| [deg]"); axy.set_xlabel("t [s]")
axy.legend(fontsize=8, loc="upper right")
axy.set_title("Yaw error over the run — big errors cluster on fresh tracks", fontsize=10)

# zoomed panels
for c, (a, b) in enumerate(top):
    pad = max(1.0, 0.08*(T[b-1]-T[a]))
    lo, hi = T[a]-pad, T[b-1]+pad
    m = (T >= lo) & (T <= hi)
    ax = fig.add_subplot(gs[2, c])
    ax.plot(T[m], ospd[m], "-", lw=1.6, color="tab:green", label="GT")
    mm = m & ok
    ax.plot(T[mm], est_v[mm], "-o", lw=1.2, ms=2.5, color="tab:red", label="est")
    yng = mm & (age_a <= 5)
    ax.plot(T[yng], est_v[yng], "o", ms=5, mfc="none", mec="tab:purple", mew=1.2,
            label="age<=5")
    ax.axhline(0.5, ls="--", lw=.7, color="k")
    ax.set_title(f"speed  {T[a]:.0f}-{T[b-1]:.0f}s ({b-a}f)", fontsize=9)
    ax.set_xlabel("t [s]")
    if c == 0:
        ax.set_ylabel("speed [m/s]"); ax.legend(fontsize=7)
    ax2 = fig.add_subplot(gs[3, c])
    gcu = np.degrees(ocourse[m]); gcu -= 360*np.round((gcu-gcu[0])/360)
    ax2.plot(T[m], gcu, "-", lw=1.6, color="tab:green", label="GT course")
    ey = np.degrees(est_y[mm]); ey -= 360*np.round((ey-gcu[0])/360)
    ax2.plot(T[mm], ey, "-o", lw=1.2, ms=2.5, color="tab:red", label="est")
    ax2.set_title(f"yaw  {T[a]:.0f}-{T[b-1]:.0f}s", fontsize=9); ax2.set_xlabel("t [s]")
    if c == 0:
        ax2.set_ylabel("yaw [deg]"); ax2.legend(fontsize=7)

plt.savefig(OUT, dpi=110, bbox_inches="tight")
print(f"그래프 -> {OUT}")
