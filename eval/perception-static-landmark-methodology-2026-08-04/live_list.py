#!/usr/bin/env python3
"""Size of the CONFIRMED-landmark list the controller actually receives, per lap.

lap_metrics.py counts distinct ids ever created, which conflates real
accumulation with erase-and-recreate churn. What the controller consumes is the
list published each cycle -- that is what this measures.

    python3 live_list.py <bag_key> <run_prefix> [run_prefix ...]
"""
import sys
sys.path.insert(0, "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30")
import numpy as np
from parse_static_obstacles import parse, parse_stamps
from lap_metrics import lap_bounds, merge_count

R = "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30"


def per_lap_live(prefix, laps, dur, t0):
    path = f"{R}/{prefix}_static_obstacles.txt"
    msgs = parse(path)
    stamps = parse_stamps(path)
    use_real = len(stamps) == len(msgs) and all(s is not None and s > 1e9 for s in stamps)
    n = max(len(msgs) - 1, 1)
    times = [(stamps[i] - t0) if use_real else dur * i / n for i in range(len(msgs))]
    out = []
    for li, lt in enumerate(laps):
        lo = laps[li - 1] if li else 0.0
        sizes = [len(m) for t, m in zip(times, msgs) if lo <= t < lt]
        objs = [merge_count([(p[1], p[2]) for p in m])
                for t, m in zip(times, msgs) if lo <= t < lt]
        out.append((np.median(sizes) if sizes else 0, np.median(objs) if objs else 0))
    return out


def main():
    bag_key, prefixes = sys.argv[1], sys.argv[2:]
    laps, dur, t0 = lap_bounds(bag_key)
    print(f"=== {bag_key} === 랩별 '살아있는 목록' 크기 중앙값 (id / 물리물체)")
    print(f"{'config':<22}" + "".join(f"{'L'+str(i+1):>13}" for i in range(len(laps))))
    for pfx in prefixes:
        cells = [f"{int(a):>5}/{int(b):<5}" for a, b in per_lap_live(pfx, laps, dur, t0)]
        print(f"{pfx:<22}" + "".join(f"{c:>13}" for c in cells))


if __name__ == "__main__":
    main()
