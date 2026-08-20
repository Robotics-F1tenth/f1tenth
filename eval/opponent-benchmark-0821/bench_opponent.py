#!/usr/bin/env python3
"""Opponent-tracking benchmark against two-car mocap GT.

Evaluates the perception output BAKED INTO THE BAG -- i.e. what actually ran on
the car -- so no replay is involved and the numbers are on-car, at race speed,
with a real opponent.

    python3 bench_opponent.py <bag_dir>

Ego is identified by matching /newmcl_pose against each mocap car; the other car
is the opponent. Tracks come from /f1/perception/object/obstacles/arr (map frame,
the per-cycle list control consumes).

Yaw is reported against two references on purpose. The tracker runs a constant-
acceleration model, whose yaw state is the COURSE (direction of travel), while
mocap yaw is the body heading. They differ whenever the car slips, so scoring
course-vs-heading would charge the tracker for a difference it never modelled.
"""
import sys, math, json
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

MOCAP = ["/mocap/car3/pose", "/mocap/car4/pose"]
MATCH_GATE = 1.0   # m, track<->opponent association for scoring


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(
        topics=MOCAP + ["/newmcl_pose", "/f1/perception/object/obstacles/arr"]))
    mc = {t: [] for t in MOCAP}
    ego, arr = [], []
    while r.has_next():
        tn, d, _ = r.read_next()
        if tn in MOCAP or tn == "/newmcl_pose":
            m = deserialize_message(d, PoseStamped)
            row = (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                   m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
            (ego if tn == "/newmcl_pose" else mc[tn]).append(row)
        else:
            m = deserialize_message(d, F1stateArr)
            t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
            arr.append((t, [(int(s.id), s.x, s.y, s.yaw, s.v, s.v_x, s.v_y)
                            for s in m.f1_state_arr]))
    srt = lambda a: np.array(sorted(a)) if a else np.zeros((0, 4))
    return srt(ego), {t: srt(v) for t, v in mc.items()}, sorted(arr)


def resample(gt, ts):
    """GT pose + velocity + course at the given times."""
    x = np.interp(ts, gt[:, 0], gt[:, 1])
    y = np.interp(ts, gt[:, 0], gt[:, 2])
    yaw = np.interp(ts, gt[:, 0], np.unwrap(gt[:, 3]))
    # velocity/course from mocap positions, lightly smoothed (70 Hz is noisy)
    k = 15
    ker = np.ones(k) / k
    gx = np.convolve(np.gradient(gt[:, 1], gt[:, 0]), ker, mode="same")
    gy = np.convolve(np.gradient(gt[:, 2], gt[:, 0]), ker, mode="same")
    vx = np.interp(ts, gt[:, 0], gx)
    vy = np.interp(ts, gt[:, 0], gy)
    return x, y, yaw, vx, vy


def main():
    bag = sys.argv[1].rstrip("/")
    ego, mc, arr = load(bag)
    if not arr:
        sys.exit("no obstacles/arr in this bag")

    # ego = the mocap car whose trajectory matches /newmcl_pose
    fit = {}
    for t, g in mc.items():
        if len(g) < 10:
            continue
        n = min(len(ego), 2000)
        idx = np.linspace(0, len(ego)-1, n).astype(int)
        ex, ey = ego[idx, 1], ego[idx, 2]
        gx = np.interp(ego[idx, 0], g[:, 0], g[:, 1])
        gy = np.interp(ego[idx, 0], g[:, 0], g[:, 2])
        fit[t] = float(np.median(np.hypot(ex-gx, ey-gy)))
    ego_topic = min(fit, key=fit.get)
    opp_topic = [t for t in mc if t != ego_topic][0]
    print(f"bag={bag.split('/')[-1]}")
    print(f"  ego  = {ego_topic}  (/newmcl_pose 와 중앙 {100*fit[ego_topic]:.1f} cm)")
    print(f"  상대 = {opp_topic}  ({100*fit[opp_topic]:.1f} cm -> 명확히 다른 차)")

    G = mc[opp_topic]
    ts = np.array([a[0] for a in arr])
    inb = (ts >= max(G[0, 0], ego[0, 0])) & (ts <= min(G[-1, 0], ego[-1, 0]))
    arr = [a for a, k in zip(arr, inb) if k]
    ts = ts[inb]
    ox, oy, oyaw, ovx, ovy = resample(G, ts)
    ex, ey, eyaw, _, _ = resample(ego, ts)
    # ego GT, to report the MCL error that floors every tracker number below
    egx, egy, _, _, _ = resample(mc[ego_topic], ts)
    ospd = np.hypot(ovx, ovy)
    ocourse = np.arctan2(ovy, ovx)

    rows, ids, matched_t = [], [], []
    for i, (t, states) in enumerate(arr):
        if not states:
            continue
        d = [math.hypot(s[1]-ox[i], s[2]-oy[i]) for s in states]
        j = int(np.argmin(d))
        if d[j] > MATCH_GATE:
            continue
        sid, sx, sy, syaw, sv, svx, svy = states[j]
        rows.append((t, d[j],
                     abs(wrap(syaw - ocourse[i])), abs(wrap(syaw - oyaw[i])),
                     sv - ospd[i], ospd[i],
                     math.hypot(ex[i]-ox[i], ey[i]-oy[i]),   # ego->opponent range
                     math.hypot(ex[i]-egx[i], ey[i]-egy[i])))  # ego MCL error
        ids.append(sid)
        matched_t.append(t)
    if not rows:
        sys.exit("opponent never matched a track")
    R = np.array(rows)
    dur = ts[-1] - ts[0]
    # availability: cycles where the opponent was matched, out of all cycles
    avail = len(rows) / len(arr)
    # fragmentation: distinct ids used for the single opponent
    uid = sorted(set(ids))
    # continuity: longest unbroken stretch on one id
    runs, cur, prev = [], 1, ids[0]
    for k in range(1, len(ids)):
        if ids[k] == prev:
            cur += 1
        else:
            runs.append(cur); cur = 1; prev = ids[k]
    runs.append(cur)

    def pct(a, p):
        return float(np.percentile(a, p))

    rng_all = np.hypot(ox-ex, oy-ey)
    nonempty = sum(1 for a in arr if a[1])
    print(f"\n  구간 {dur:.0f}s  |  arr 사이클 {len(arr)} (그중 트랙 0개 {len(arr)-nonempty}, "
          f"{100*(len(arr)-nonempty)/len(arr):.0f}%)")
    print(f"  상대 속도 GT p50 {np.median(ospd):.2f} / p95 {pct(ospd,95):.2f} m/s   "
          f"상대까지 실제거리 p50 {np.median(rng_all):.2f} / p90 {pct(rng_all,90):.2f} m")
    print(f"  자차 MCL 오차 p50 {100*pct(R[:,7],50):.1f} cm  <- 트래커 오차의 바닥")

    print(f"\n=== 전체 (매칭 {len(rows)}건, 게이트 {MATCH_GATE} m) ===")
    print(f"  위치   mean {R[:,1].mean():.3f}  p50 {pct(R[:,1],50):.3f}  p90 {pct(R[:,1],90):.3f}  "
          f"rmse {math.sqrt((R[:,1]**2).mean()):.3f} m")
    print(f"  yaw    course MAE {math.degrees(R[:,2].mean()):5.1f}  p50 {math.degrees(pct(R[:,2],50)):5.1f} deg"
          f"   |  heading MAE {math.degrees(R[:,3].mean()):5.1f}  p50 {math.degrees(pct(R[:,3],50)):5.1f} deg")
    print(f"  속도   bias {R[:,4].mean():+.2f}  MAE {np.abs(R[:,4]).mean():.2f}  p90 {pct(np.abs(R[:,4]),90):.2f} m/s")

    print(f"\n=== 거리 구간별 (핵심) ===")
    print(f"  {'거리':>7} {'사이클':>7} {'검출률':>7} {'위치p50':>8} {'yaw p50':>8} {'속도MAE':>8}")
    for lo, hi in [(0,2),(2,4),(4,6),(6,8),(8,99)]:
        sel_all = (rng_all>=lo) & (rng_all<hi)
        n_all = int(sel_all.sum())
        if n_all < 20:
            continue
        sel = (R[:,6]>=lo) & (R[:,6]<hi)
        n = int(sel.sum())
        det = 100.0*n/n_all
        if n >= 10:
            print(f"  {lo:2d}-{hi:2d} m {n_all:7d} {det:6.1f}% {pct(R[sel,1],50):8.3f} "
                  f"{math.degrees(pct(R[sel,2],50)):8.1f} {np.abs(R[sel,4]).mean():8.2f}")
        else:
            print(f"  {lo:2d}-{hi:2d} m {n_all:7d} {det:6.1f}% {'-':>8} {'-':>8} {'-':>8}")

    print(f"\n=== 연속성 ===")
    print(f"  상대 1대에 쓰인 track id {len(uid)}개  최장 연속 {max(runs)} 사이클  "
          f"연속구간 중앙 {int(np.median(runs))}")
    json.dump({"bag": bag, "ego_topic": ego_topic, "opp_topic": opp_topic,
               "dur": float(dur), "cycles": len(arr), "matched": len(rows),
               "avail": avail, "pos_mean": float(R[:,1].mean()),
               "pos_p50": pct(R[:,1],50), "pos_p90": pct(R[:,1],90),
               "pos_rmse": float(math.sqrt((R[:,1]**2).mean())),
               "yaw_course_mae_deg": math.degrees(R[:,2].mean()),
               "yaw_head_mae_deg": math.degrees(R[:,3].mean()),
               "spd_mae": float(np.abs(R[:,4]).mean()),
               "ids": len(uid), "run_max": int(max(runs)),
               "ego_mcl_p50_cm": 100*pct(R[:,7],50),
               "empty_cycles": len(arr)-nonempty,
               "opp_range_p50": float(np.median(rng_all))},
              open(f"result_{bag.split('/')[-1]}.json", "w"), indent=2)


if __name__ == "__main__":
    main()
