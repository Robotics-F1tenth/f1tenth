#!/usr/bin/env python3
"""EKF vs MCL-with-zero-order-hold, both sampled on the EKF's own clock.

Comparing MCL only at its own ~35Hz stamps flatters it: a controller running
faster than that does not get a fresh MCL pose, it gets the last one held. So
resample MCL with ZOH onto the EKF timestamps and score both against GT there.
That is the comparison a control consumer actually experiences.
"""
import bisect, csv, math, sys


def load(path):
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            out.append((float(row["t"]), float(row["x"]), float(row["y"]), float(row["yaw_body"])))
    out.sort(key=lambda r: r[0])
    return out


def nearest(rows, ts, t, tol):
    i = bisect.bisect_left(ts, t)
    best = None
    for j in (i - 1, i, i + 1):
        if 0 <= j < len(rows):
            d = abs(rows[j][0] - t)
            if best is None or d < best[0]:
                best = (d, rows[j])
    return best[1] if best and best[0] <= tol else None


def zoh(rows, ts, t):
    i = bisect.bisect_right(ts, t) - 1
    return rows[i] if i >= 0 else None


def pct(v, p):
    s = sorted(v)
    return s[min(int(len(s) * p / 100), len(s) - 1)]


def main():
    ekf, mcl, gt = (load(p) for p in sys.argv[1:4])
    gts = [r[0] for r in gt]
    mts = [r[0] for r in mcl]

    ekf_e, mcl_e, ekf_y, mcl_y = [], [], [], []
    for t, ex, ey, eyaw in ekf:
        g = nearest(gt, gts, t, 0.02)
        m = zoh(mcl, mts, t)
        if g is None or m is None:
            continue
        ekf_e.append(math.hypot(g[1] - ex, g[2] - ey))
        mcl_e.append(math.hypot(g[1] - m[1], g[2] - m[2]))
        for src, acc in ((eyaw, ekf_y), (m[3], mcl_y)):
            d = g[3] - src
            acc.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))

    n = len(ekf_e)
    print(f"  n={n} (EKF clock, MCL held between its own updates)")
    for name, e, y in (("MCL+ZOH", mcl_e, mcl_y), ("EKF    ", ekf_e, ekf_y)):
        print(f"  {name}: p50 {pct(e,50):.3f}  p90 {pct(e,90):.3f}  "
              f"MAE {sum(e)/n:.3f}  RMSE {math.sqrt(sum(v*v for v in e)/n):.3f} m"
              f"  | yaw MAE {sum(y)/n:.2f} deg")
    # Signed as change in error, so negative = EKF better, for both metrics.
    mae_m, mae_e = sum(mcl_e) / n, sum(ekf_e) / n
    rmse_m = math.sqrt(sum(v * v for v in mcl_e) / n)
    rmse_e = math.sqrt(sum(v * v for v in ekf_e) / n)
    yaw_m, yaw_e = sum(mcl_y) / n, sum(ekf_y) / n
    print(f"  -> EKF vs MCL+ZOH:  MAE {(mae_e-mae_m)/mae_m*100:+.1f}%  "
          f"RMSE {(rmse_e-rmse_m)/rmse_m*100:+.1f}%  "
          f"yaw {(yaw_e-yaw_m)/yaw_m*100:+.1f}%   (negative = EKF better)")


if __name__ == "__main__":
    main()
