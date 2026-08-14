#!/usr/bin/env python3
"""What yaw WOULD have been if March's extractYaw() result hadn't been discarded
(fonePerception.cpp:1291-1294 computes it, then overwrites with 0.0 before use).

Replicates extractYaw()'s exact algorithm: nearest centerline waypoint to the
object's position, tangent = atan2(next_wp - nearest_wp). Compared against GT
course (same course_speed() used everywhere else in this eval suite) using the
already-captured GT position CSVs in this directory -- no rerun needed.

    python3 centerline_tangent_check.py
"""
import math
import sys
from pathlib import Path
import numpy as np
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "eval" / "tools"))
from analyze_perception_tracking import gt_interp, gt_glitch_windows, load, EVAL_HZ

R = REPO_ROOT / "eval" / "perception-march-baseline-comparison"
LANE_DIR = REPO_ROOT / "src" / "perception" / "data" / "lane"
BAGS = [("22_25_24", "map7_path"), ("22_52_38", "map1_path"), ("22_48_25", "map1_path")]


def load_lane(name):
    return np.loadtxt(LANE_DIR / f"{name}.csv", delimiter=",")


def centerline_tangent(lane, x, y):
    """extractYaw()'s algorithm: nearest waypoint, tangent to the next one."""
    d2 = (lane[:, 0] - x) ** 2 + (lane[:, 1] - y) ** 2
    i = int(np.argmin(d2))
    j = (i + 1) % len(lane)
    return math.atan2(lane[j, 1] - lane[i, 1], lane[j, 0] - lane[i, 0])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    print(f"{'bag':<10} {'n':>5} {'평균|오차|':>10} {'중앙':>8} {'p90':>8} {'max':>8}   (도, extractYaw 결과 vs GT course)")
    for bag, lane_name in BAGS:
        lane = load_lane(lane_name)
        gt = load(f"{R}/march_{bag}_gt_opp.csv", 4)
        pos, course_speed, (gt0, gt1) = gt_interp(gt)
        wins = gt_glitch_windows(gt)
        grid = np.arange(gt0, gt1, 1.0 / EVAL_HZ)

        errs = []
        for t in grid:
            if any(w0 <= t <= w1 for w0, w1 in wins):
                continue
            course, spd = course_speed(t)
            if math.isnan(course) or spd < 0.30:  # same gate as analyze_perception_tracking
                continue
            x, y = pos(t)
            tangent = centerline_tangent(lane, x, y)
            errs.append(abs(math.degrees(wrap(tangent - course))))

        errs = np.array(errs)
        print(f"{bag:<10} {len(errs):>5} {errs.mean():>9.2f}° {np.median(errs):>7.2f}° "
              f"{np.percentile(errs, 90):>7.2f}° {errs.max():>7.2f}°")


if __name__ == "__main__":
    main()
