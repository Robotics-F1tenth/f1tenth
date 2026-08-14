#!/usr/bin/env python3
"""Localization (MCL) accuracy vs mocap GT. Pure python+numpy, no ROS.

    python3 eval/tools/analyze_localization.py <mcl.csv> <gt_ego.csv> [--lane <centerline.csv>]
    python3 eval/tools/analyze_localization.py --selftest

CSVs: t,x,y,yaw (record_ros_topics.py's *_mcl.csv / *_gt_ego.csv format).
MCL estimates the BODY pose, so yaw here is compared quaternion-to-quaternion
(body vs body) — unlike the opponent-course eval, this is apples-to-apples.
Corner/straight split via centerline curvature at the GT position (>0.3 1/m).
The first SKIP_S seconds are also reported separately (initial convergence).

Latest criteria (2026-07 / yaw-lag-plan §8): mocap GT has physically impossible
jumps (~186cm in one 3.4ms step) that dominate tail metrics (p99/MAX). Detection
is by step DISPLACEMENT (dist > 0.15m + 8m/s*dt), not speed — logger timestamps
jitter to sub-ms, which makes any speed threshold fire on mm-level noise, and GT
has real 0.3s frame gaps where honest travel is large. Measured on record5: the
wrong-solution spans around each impossible step extend up to +-1.5s (mocap sits
on a bad solution, then snaps back — the snap is the detectable step), so the
exclusion margin is +-1.5s, windows merged (--no-glitch-filter to disable).
Robust percentiles (p50/p90/p95) are the primary metrics; the mean error vector
is also decomposed in the GT body frame (longitudinal/lateral constant bias —
calibration-type offsets show up there).
"""
import argparse, csv, math
from pathlib import Path
import numpy as np

SKIP_S = 2.0        # initial convergence window, reported separately
CURV_TH = 0.3       # 1/m; nearest-centerline curvature above this = corner
CURV_SMOOTH = 2     # +-idx for 3-point circumcircle curvature
GLITCH_D = 0.15     # m; GT step displacement floor considered impossible
GLITCH_VMAX = 8.0   # m/s; max honest car speed (scales floor over frame gaps)
GLITCH_PAD = 0.05     # s; padding added around the MEASURED contamination span
GLITCH_MAX_SPAN = 2.0 # s; safety cap on how long one glitch may be considered active


def load(path):
    with open(path) as f:
        r = csv.reader(f); next(r)
        rows = [[float(v) for v in row] for row in r if row]
    a = np.array(rows, float)
    return a[a[:, 0].argsort()]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def lane_curvature(lane):
    n = len(lane)
    k = np.zeros(n)
    for i in range(n):
        a, b, c = lane[(i - CURV_SMOOTH) % n], lane[i], lane[(i + CURV_SMOOTH) % n]
        ab, bc, ac = b - a, c - b, c - a
        d = np.hypot(*ab) * np.hypot(*bc) * np.hypot(*ac)
        if d > 1e-9:
            k[i] = 2.0 * abs(ab[0] * bc[1] - ab[1] * bc[0]) / d
    return k


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


def analyze(mcl, gt, lane=None, glitch_filter=True):
    t0 = gt[0, 0]
    gx = np.interp(mcl[:, 0], gt[:, 0], gt[:, 1])
    gy = np.interp(mcl[:, 0], gt[:, 0], gt[:, 2])
    # keep only samples inside GT time range
    m = (mcl[:, 0] >= gt[0, 0]) & (mcl[:, 0] <= gt[-1, 0])
    mcl, gx, gy = mcl[m], gx[m], gy[m]

    keep = np.ones(len(mcl), bool)
    if glitch_filter:
        wins = gt_glitch_windows(gt)
        for w0, w1 in wins:
            keep &= (mcl[:, 0] < w0) | (mcl[:, 0] > w1)
        if wins:
            span = sum(w1 - w0 for w0, w1 in wins)
            print(f"GT glitch windows ({len(wins)}, {span:.1f}s total): "
                  + ", ".join(f"[+{w0 - t0:.1f},+{w1 - t0:.1f}]" for w0, w1 in wins)
                  + f"  -> {int((~keep).sum())}/{len(keep)} samples excluded")
    mcl, gx, gy = mcl[keep], gx[keep], gy[keep]

    dx, dy = mcl[:, 1] - gx, mcl[:, 2] - gy
    perr = np.hypot(dx, dy)
    # body-yaw error (both are body headings)
    gyaw = np.interp(mcl[:, 0], gt[:, 0], np.unwrap(gt[:, 3]))
    yerr = np.degrees(np.abs([wrap(a - b) for a, b in zip(mcl[:, 3], gyaw)]))
    trel = mcl[:, 0] - t0

    corner = None
    if lane is not None:
        k = lane_curvature(lane)
        idx = [int(np.argmin((lane[:, 0] - x) ** 2 + (lane[:, 1] - y) ** 2))
               for x, y in zip(gx, gy)]
        corner = k[idx] > CURV_TH

    def stat(mask, label):
        if mask.sum() == 0:
            print(f"  {label:>22}: (no samples)"); return
        p, y = perr[mask], yerr[mask]
        q50, q90, q95 = np.percentile(p, [50, 90, 95])
        print(f"  {label:>22}: p50 {q50:.3f}  p90 {q90:.3f}  p95 {q95:.3f}  "
              f"MAE {p.mean():.3f}  RMSE {np.sqrt((p**2).mean()):.3f}  MAX {p.max():.3f} m"
              f"  | yaw MAE {y.mean():.2f} deg  (n={mask.sum()})")

    print(f"samples: {len(mcl)}  span: {trel[-1]:.1f}s")
    stat(np.ones(len(perr), bool), "overall")
    stat(trel < SKIP_S, f"first {SKIP_S:.0f}s (converging)")
    steady = trel >= SKIP_S
    stat(steady, f"after {SKIP_S:.0f}s (steady)")
    if corner is not None:
        stat(steady & corner, "steady & corner")
        stat(steady & ~corner, "steady & straight")
        # where is the steady-state max?
        if steady.sum():
            i = np.argmax(np.where(steady, perr, -1))
            print(f"  steady MAX at t=+{trel[i]:.1f}s pos=({gx[i]:.2f},{gy[i]:.2f}) "
                  f"{'corner' if corner[i] else 'straight'}")
    # constant bias in GT body frame (calibration-type offsets show up here)
    if steady.sum():
        c, s = np.cos(gyaw[steady]), np.sin(gyaw[steady])
        blong = (dx[steady] * c + dy[steady] * s).mean()
        blat = (-dx[steady] * s + dy[steady] * c).mean()
        print(f"  body-frame bias (steady): longitudinal {blong*100:+.1f} cm  "
              f"lateral {blat*100:+.1f} cm  (+long = ahead of GT)")
    return {"rmse": float(np.sqrt((perr[steady]**2).mean())) if steady.sum() else float("nan"),
            "max": float(perr[steady].max()) if steady.sum() else float("nan"),
            "p50": float(np.percentile(perr[steady], 50)) if steady.sum() else float("nan")}


def selftest():
    t = np.linspace(0, 10, 500)
    gt = np.column_stack([t, np.cos(t), np.sin(t), t * 0])
    mcl = gt.copy()
    mcl[:, 1] += 0.1          # constant 0.1m x-offset
    res = analyze(mcl, gt, lane=np.column_stack([np.cos(t), np.sin(t)]))
    assert abs(res["rmse"] - 0.1) < 0.01, res
    assert abs(res["max"] - 0.1) < 0.01, res
    # inject a mocap glitch: 2m out-and-back in one step -> must be excluded
    gt2 = gt.copy()
    gt2[250, 1] += 2.0
    res2 = analyze(mcl, gt2, lane=np.column_stack([np.cos(t), np.sin(t)]))
    assert abs(res2["max"] - 0.1) < 0.01, ("glitch not excluded", res2)
    res3 = analyze(mcl, gt2, glitch_filter=False)
    assert res3["max"] > 1.0, ("filter-off should keep glitch", res3)
    print("\nselftest OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mcl_csv", nargs="?")
    ap.add_argument("gt_csv", nargs="?")
    default_lane = Path(__file__).resolve().parents[2] / "src/perception/data/lane/map1_path.csv"
    ap.add_argument("--lane", default=str(default_lane))
    ap.add_argument("--no-glitch-filter", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest or not a.mcl_csv:
        return selftest()
    lane = None
    try:
        with open(a.lane) as f:
            lane = np.array([[float(v) for v in row[:2]] for row in csv.reader(f)
                             if row and not row[0].startswith("x")])
    except OSError:
        print(f"(lane file not found: {a.lane} -> no corner split)")
    analyze(load(a.mcl_csv), load(a.gt_csv), lane, glitch_filter=not a.no_glitch_filter)


if __name__ == "__main__":
    main()
