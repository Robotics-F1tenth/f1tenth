#!/usr/bin/env python3
"""Summarize a static_obstacles capture for the box-recall / off-ROI-FP question.

    python3 summarize_box.py <prefix> [box_x box_y box_radius]

Default box ROI center (-2.5, -9.5), radius 1.0m (covers box footprint + measured
tracking jitter -- see perception-workplan-0805.md B: max deviation p90 25-28cm,
plus the box itself is long so its centroid can wander along its length).
"""
import sys, math
sys.path.insert(0, "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30")
from parse_static_obstacles import parse

R = "/home/a/f1tenth/eval/perception-box-recovery"
BOX_X, BOX_Y, BOX_R = -2.5, -9.5, 1.0


def main():
    prefix = sys.argv[1]
    bx = float(sys.argv[2]) if len(sys.argv) > 2 else BOX_X
    by = float(sys.argv[3]) if len(sys.argv) > 3 else BOX_Y
    br = float(sys.argv[4]) if len(sys.argv) > 4 else BOX_R

    path = f"{R}/{prefix}_static_obstacles.txt"
    msgs = parse(path)
    by_id = {}
    for m in msgs:
        for id_, x, y, v in m:
            by_id.setdefault(id_, []).append((x, y))

    in_roi, out_roi = [], []
    for id_, pts in by_id.items():
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        d = math.hypot(mx - bx, my - by)
        (in_roi if d <= br else out_roi).append((id_, mx, my, len(pts), d))

    print(f"=== {prefix} === 확정 landmark 총 {len(by_id)}개")
    print(f"박스 ROI(({bx},{by}) 반경{br}m) 안: {len(in_roi)}개")
    for id_, mx, my, n, d in sorted(in_roi, key=lambda r: r[4]):
        print(f"    id={id_:.0f}  ({mx:.2f},{my:.2f})  n={n}  ROI중심거리={d:.2f}m  <- BOX HIT" )
    print(f"ROI 밖(= 잠재 FP 후보): {len(out_roi)}개")
    for id_, mx, my, n, d in sorted(out_roi, key=lambda r: -r[3])[:10]:
        print(f"    id={id_:.0f}  ({mx:.2f},{my:.2f})  n={n}")
    if len(out_roi) > 10:
        print(f"    ... 외 {len(out_roi)-10}개 더")


if __name__ == "__main__":
    main()
