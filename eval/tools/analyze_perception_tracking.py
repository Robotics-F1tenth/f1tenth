#!/usr/bin/env python3
"""Offline ACC-yaw-quality analysis. Pure python+numpy, no ROS.

    python3 eval/tools/analyze_perception_tracking.py /tmp/eval_run5
    python3 eval/tools/analyze_perception_tracking.py --selftest

Focus: course-yaw accuracy of the tracker vs GT, on MATCHED frames only
(x,y is already good; availability is dominated by occlusion/FOV in this bag
and is out of scope). ID-agnostic per-frame nearest-position association.
Yaw compared as COURSE (direction of travel): tracker yaw = atan2(vy,vx),
GT course from GT *positions* (NOT mocap quaternion / body heading).
Low-speed frames are excluded — course is ill-defined for both there.
"""
import argparse, csv, math
import numpy as np

GATE = 0.5          # m, max track-GT distance to count as a match
COURSE_DT = 0.05    # s, centered window for GT course/speed
COURSE_VMIN = 0.30  # m/s, below this GT course is undefined
YAW_VMIN = 1.0      # m/s, yaw stats computed only above this GT speed
EVAL_HZ = 40.0      # uniform eval grid (~tracker rate)
SPEED_EDGES = [0.3, 1.0, 2.0, 3.0, 5.0, float("inf")]  # GT-speed buckets for yaw table
SEG_EDGES = [0.1, 0.3, 0.6, float("inf")]  # s since re-acquisition (transient vs steady)

# Opponent GT glitch exclusion (same criteria as analyze_localization.py): mocap step
# displacement physically impossible for a real car -> the rigid-body solve glitched,
# not the tracker. Matters more now that some bags used only 3 reflective markers
# per body (record5 used 5-6) -> opponent GT (mocap_pose2) is noisier than before.
GLITCH_D = 0.15      # m; GT step displacement floor considered impossible
GLITCH_VMAX = 8.0    # m/s; max honest car speed (scales floor over frame gaps)
GLITCH_PAD = 0.05     # s; padding added around the MEASURED contamination span
GLITCH_MAX_SPAN = 2.0 # s; safety cap on how long one glitch may be considered active


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def gt_glitch_windows(gt):
    """Merged [t0,t1] windows around physically impossible GT steps.

    The exclusion span is MEASURED per glitch, not a fixed margin: after the bad
    step, walk forward until the pose is back within reach of the pre-glitch
    position at an honest speed, then pad by GLITCH_PAD. A fixed +-1.5s margin was
    calibrated on record5, whose mis-solves were ~1.9m and really did stay wrong
    that long; the 3-marker 2026-07-30 session instead produces ~0.2m spikes that
    recover in ~30ms (p90 47ms), where a fixed 1.5s threw away 85-89% of the run
    for nothing. Measuring the span keeps both cases honest.
    """
    g = gt[gt[:, 0].argsort()]
    t, x, y = g[:, 0], g[:, 1], g[:, 2]
    dt = np.diff(t)
    dist = np.hypot(np.diff(x), np.diff(y))
    bad = np.where(dist > GLITCH_D + GLITCH_VMAX * dt)[0]
    wins = []
    for i in bad:
        t_end = t[i + 1]
        j = i + 1
        while j < len(t) and t[j] - t[i] <= GLITCH_MAX_SPAN:
            if np.hypot(x[j] - x[i], y[j] - y[i]) <= GLITCH_VMAX * (t[j] - t[i]) + GLITCH_D:
                break                      # trajectory is plausible again
            t_end = t[j]
            j += 1
        a, b = t[i] - GLITCH_PAD, t_end + GLITCH_PAD
        if wins and a <= wins[-1][1]:
            wins[-1][1] = max(wins[-1][1], b)
        else:
            wins.append([a, b])
    return wins


def load(path, cols):
    with open(path) as f:
        r = csv.reader(f); next(r)
        rows = [[float(v) for v in row] for row in r if row]
    return np.array(rows, float) if rows else np.empty((0, cols))


def gt_interp(gt):
    """pos(t)->(x,y), and course/speed(t) from centered diff of GT positions."""
    g = gt[gt[:, 0].argsort()]
    t, x, y = g[:, 0], g[:, 1], g[:, 2]

    def pos(ts):
        return np.interp(ts, t, x), np.interp(ts, t, y)

    def course_speed(ts):  # centered diff: unbiased heading on curved paths
        x0, y0 = pos(ts - COURSE_DT / 2); x1, y1 = pos(ts + COURSE_DT / 2)
        dx, dy = x1 - x0, y1 - y0
        disp = math.hypot(dx, dy)
        spd = disp / COURSE_DT
        course = math.atan2(dy, dx) if disp >= COURSE_VMIN * COURSE_DT else float("nan")
        return course, spd

    return pos, course_speed, (t[0], t[-1])


def analyze(tracks, gt, win=None, glitch_filter=True):
    pos, course_speed, (gt0, gt1) = gt_interp(gt)

    # group tracker rows into frames by identical timestamp
    frames = {}
    for r in tracks:
        frames.setdefault(round(r[0], 6), []).append(r)
    ftimes = np.array(sorted(frames))

    lo, hi = max(gt0, ftimes[0]), min(gt1, ftimes[-1])
    if win is not None:  # restrict to a racing window, seconds relative to gt start
        lo = max(lo, gt0 + win[0])
        hi = min(hi, gt0 + win[1])
    grid = np.arange(lo, hi, 1.0 / EVAL_HZ)
    tol = 0.5 / EVAL_HZ

    wins = gt_glitch_windows(gt) if glitch_filter else []
    if wins:
        span = sum(w1 - w0 for w0, w1 in wins)
        n_excl = int(sum((grid >= w0) & (grid <= w1) for w0, w1 in wins).sum())
        print(f"GT glitch windows ({len(wins)}, {span:.1f}s total): "
              + ", ".join(f"[+{w0 - gt0:.1f},+{w1 - gt0:.1f}]" for w0, w1 in wins)
              + f"  -> ~{n_excl}/{len(grid)} grid ticks excluded")

    # per frame: (t, matched, dist, yaw_err|nan, gt_speed, gt_course, trk_yaw)
    per = []
    for t in grid:
        if any(w0 <= t <= w1 for w0, w1 in wins):
            continue  # GT glitch window -> not evaluated (not even counted as a miss)
        j = int(np.searchsorted(ftimes, t))
        cand = [ftimes[k] for k in (j - 1, j) if 0 <= k < len(ftimes)]
        ft = min(cand, key=lambda f: abs(f - t)) if cand else None
        if ft is None or abs(ft - t) > tol:
            per.append((t, False, float("nan"), float("nan"), float("nan"),
                        float("nan"), float("nan")))
            continue
        gx, gy = pos(t)
        best, bestd = None, GATE
        for r in frames[ft]:
            d = math.hypot(r[2] - gx, r[3] - gy)
            if d <= bestd:
                best, bestd = r, d
        if best is None:
            per.append((t, False, float("nan"), float("nan"), float("nan"),
                        float("nan"), float("nan")))
            continue
        c, spd = course_speed(t)
        ye = abs(wrap(best[4] - c)) if not math.isnan(c) else float("nan")
        per.append((t, True, bestd, ye, spd, c, best[4]))

    return per, (gt0, gt1)


def report(per, span):
    n = len(per)
    matched = [p for p in per if p[1]]
    derr = np.array([p[2] for p in matched])
    drop = sum(1 for a, b in zip(per, per[1:]) if a[1] and not b[1])
    print(f"frames evaluated: {n}   matched: {len(matched)} ({len(matched)/n:.1%})   "
          f"dropouts: {drop}   [availability not the focus — see notes]")
    if len(derr):
        print(f"position err: mean {derr.mean():.3f} m  rmse {np.sqrt((derr**2).mean()):.3f} m")

    # yaw stats on matched frames above the speed gate (low speed = noisy course)
    yd = np.degrees([p[3] for p in matched
                     if not math.isnan(p[3]) and p[4] >= YAW_VMIN])
    print(f"\ncourse-yaw err (GT speed >= {YAW_VMIN} m/s, n={len(yd)}):")
    if len(yd):
        print(f"  MAE {yd.mean():.2f}  RMSE {np.sqrt((yd**2).mean()):.2f}  "
              f"median {np.median(yd):.2f}  p90 {np.percentile(yd, 90):.2f}  [deg]")

    # yaw error vs GT speed — shows whether error is just a low-speed artifact
    print("course-yaw MAE by GT speed:")
    for a, b in zip(SPEED_EDGES, SPEED_EDGES[1:]):
        bk = np.degrees([p[3] for p in matched
                         if not math.isnan(p[3]) and a <= p[4] < b])
        lbl = f"{a:.1f}-{b:.1f}" if b != float("inf") else f">{a:.1f}"
        print(f"  {lbl:>9} m/s: " + (f"{bk.mean():6.2f} deg (n={len(bk)})" if len(bk) else "  -"))

    # yaw error vs time since re-acquisition — separates transient from steady state.
    # seg_age resets every time tracking drops and re-acquires (id-independent).
    seg_start, ages = None, []
    for p in per:
        if not p[1]:
            seg_start = None; continue
        if seg_start is None:
            seg_start = p[0]
        if p[4] >= YAW_VMIN and not math.isnan(p[3]):
            ages.append((p[0] - seg_start, p[3], p[2]))  # seg_age, yaw_err, pos_err
    print("by time since re-acquisition (yaw lag vs association drift):")
    lo = 0.0
    for hi in SEG_EDGES:
        ye = np.degrees([e for a, e, _ in ages if lo <= a < hi])
        pe = np.array([d for a, _, d in ages if lo <= a < hi])
        lbl = f"{lo:.2f}-{hi:.2f}s" if hi != float("inf") else f">{lo:.2f}s"
        cell = f"yaw {ye.mean():6.2f} deg | pos {pe.mean():.3f} m (n={len(ye)})" if len(ye) else "  -"
        print(f"  {lbl:>12}: {cell}")
        lo = hi

    return {"match_rate": len(matched) / n,
            "yaw_mae_deg": yd.mean() if len(yd) else float("nan")}


def maybe_plot(per, prefix):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    t0 = per[0][0]
    m = [p for p in per if p[1] and not math.isnan(p[3])]
    t = [p[0] - t0 for p in m]
    gc = [math.degrees(p[5]) for p in m]
    ty = [math.degrees(p[6]) for p in m]
    ye = [math.degrees(p[3]) for p in m]
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    ax[0].plot(t, gc, ".", ms=3, label="GT course", color="k")
    ax[0].plot(t, ty, ".", ms=3, label="tracker yaw", color="tab:red", alpha=0.6)
    ax[0].set_ylabel("yaw [deg]"); ax[0].legend(loc="upper right")
    ax[1].plot(t, ye, ".", ms=3); ax[1].set_ylabel("|err| [deg]"); ax[1].set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(f"{prefix}_yaw.png", dpi=120)
    print(f"\nplot -> {prefix}_yaw.png")


def selftest():
    # GT: circle; tracker = GT course + 5deg offset, with an occlusion gap
    R, w = 3.0, 0.8                      # speed R*w = 2.4 m/s > YAW_VMIN
    tg = np.linspace(0, 10, 2900)
    gt = np.column_stack([tg, R*np.cos(w*tg), R*np.sin(w*tg), np.zeros_like(tg)])
    OFF = math.radians(5.0)
    tr = []
    for t in np.arange(0, 10, 0.025):
        if 4.0 <= t <= 5.0:              # occlusion gap -> ~90% availability
            continue
        x, y = R*math.cos(w*t), R*math.sin(w*t)
        course = math.atan2(R*w*math.cos(w*t), -R*w*math.sin(w*t))
        tr.append([t, 1, x, y, wrap(course + OFF), R*w])
    per, span = analyze(np.array(tr), gt)
    res = report(per, span)
    assert abs(res["yaw_mae_deg"] - 5.0) < 0.5, res["yaw_mae_deg"]
    assert 0.85 < res["match_rate"] < 0.95, res["match_rate"]
    print("\nselftest OK")


def report_id_persistence(tracks, top_n=5):
    # Raw track-ID lifespan (first->last observation), independent of GT matching.
    # The re-acquisition-age bucket in report() resets on ANY missed frame on the
    # uniform EVAL_HZ grid, so a track publishing on a sparse/gappy cadence (e.g.
    # perception only republishes when a fresh detection exists this scan, so
    # occlusion frames are skipped rather than coasted) looks like constant
    # re-acquisition there even if the SAME id persists for many seconds. This is
    # the more literal "how long did this id live" view, matching what rviz shows.
    by_id = {}
    for t, tid, *_ in tracks:
        by_id.setdefault(int(tid), []).append(t)
    spans = []
    for tid, ts in by_id.items():
        ts.sort()
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        spans.append((ts[-1] - ts[0], len(ts), max(gaps) if gaps else 0.0, tid))
    spans.sort(reverse=True)
    print(f"id lifespan (first->last obs, top {top_n} of {len(spans)}):")
    for span, n, maxgap, tid in spans[:top_n]:
        print(f"  id={tid:5d}  n={n:4d}  span={span:6.2f}s  max_gap={maxgap:.2f}s")
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix", nargs="?")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--t0", type=float, help="window start, s relative to GT start")
    ap.add_argument("--t1", type=float, help="window end, s relative to GT start")
    ap.add_argument("--no-glitch-filter", action="store_true",
                     help="disable GT glitch-window exclusion (default: on)")
    a = ap.parse_args()
    if a.selftest or not a.prefix:
        return selftest()
    tracks = load(f"{a.prefix}_tracks.csv", 6)
    gt = load(f"{a.prefix}_gt_opp.csv", 4)
    win = None
    if a.t0 is not None or a.t1 is not None:
        base = gt[:, 0].min()
        win = (a.t0 if a.t0 is not None else 0.0,
               a.t1 if a.t1 is not None else 1e9)
        lo, hi = base + win[0], base + win[1]
        tracks = tracks[(tracks[:, 0] >= lo) & (tracks[:, 0] <= hi)]  # window object-count too
        print(f"[window {win[0]:.0f}-{win[1]:.0f}s relative to GT start]")
    # object load (phantom-obstacle proxy): wall-stub suppression should lower these
    if len(tracks):
        nframes = len(set(np.round(tracks[:, 0], 6)))
        print(f"objects published: {len(tracks)} detections, "
              f"{len(set(tracks[:, 1].astype(int)))} distinct IDs, "
              f"{len(tracks)/max(nframes,1):.2f} obj/frame over {nframes} frames\n")
        report_id_persistence(tracks)
    per, _ = analyze(tracks, gt, win, glitch_filter=not a.no_glitch_filter)
    report(per, _)
    maybe_plot(per, a.prefix)


if __name__ == "__main__":
    main()
