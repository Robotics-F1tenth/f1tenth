#!/usr/bin/env python3
"""Pseudo-GT for static obstacles via cross-bag consistency (no mocap GT exists
for static objects). The map1 group was recorded in this physical order:

    22_48_25 (obstacles PRESENT) -> 22_52_38 (PRESENT) -> 22_55_09 (REMOVED)

so a landmark position that shows up in 48 AND 52 but NOT in 55 is almost
certainly a real obstacle that was physically taken away before 55. One that
persists into 55 never moved -> fixed infrastructure or a standing false
positive. One that appears in a single bag is transient (a person) or spurious.

    python3 crossbag_gt.py [suffix]     # suffix picks which run set to read
"""
import sys, os
sys.path.insert(0, "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30")
from parse_static_obstacles import parse

R = "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30"
MATCH_R = 0.5   # m, cross-bag position match radius


def objects(prefix):
    """Landmark ids -> merged physical objects (centroid list) for one run."""
    path = f"{R}/{prefix}_static_obstacles.txt"
    if not os.path.exists(path):
        return None
    msgs = parse(path)
    cents = {}
    for m in msgs:
        for id_, x, y, v in m:
            cents.setdefault(id_, []).append((x, y))
    pos = {i: (sum(p[0] for p in v)/len(v), sum(p[1] for p in v)/len(v))
           for i, v in cents.items()}
    ids = sorted(pos)
    parent = {i: i for i in ids}
    def find(a):
        while parent[a] != a: a = parent[a]
        return a
    for i, a in enumerate(ids):
        for b in ids[i+1:]:
            if ((pos[a][0]-pos[b][0])**2 + (pos[a][1]-pos[b][1])**2) ** 0.5 < MATCH_R:
                ra, rb = find(a), find(b)
                if ra != rb: parent[ra] = rb
    groups = {}
    for i in ids:
        groups.setdefault(find(i), []).append(i)
    out = []
    for g in groups.values():
        xs = [pos[i][0] for i in g]; ys = [pos[i][1] for i in g]
        out.append((sum(xs)/len(xs), sum(ys)/len(ys), len(g)))
    return out


def near(objs, x, y, r=MATCH_R):
    for ox, oy, n in objs:
        if ((ox-x)**2 + (oy-y)**2) ** 0.5 < r:
            return (ox, oy, n)
    return None


def main():
    suf = sys.argv[1] if len(sys.argv) > 1 else ""
    a = objects(f"22_48_25{suf}"); b = objects(f"22_52_38{suf}"); c = objects(f"22_55_09{suf}")
    if a is None or b is None or c is None:
        print("missing run data", file=sys.stderr); sys.exit(1)
    print(f"physical objects per bag (ids merged at {MATCH_R}m): "
          f"48={len(a)}  52={len(b)}  55={len(c)}")

    real, infra, transient = [], [], []
    for x, y, n in a:
        inb, inc = near(b, x, y), near(c, x, y)
        if inb and not inc:
            real.append((x, y))
        elif inb and inc:
            infra.append((x, y))
        else:
            transient.append(("48only", x, y))
    for x, y, n in b:                       # things only 52 saw
        if not near(a, x, y):
            transient.append(("52only", x, y) if not near(c, x, y) else ("52+55", x, y))
    for x, y, n in c:                       # things only 55 saw (obstacles were gone)
        if not near(a, x, y) and not near(b, x, y):
            transient.append(("55only", x, y))

    print(f"\n[REAL obstacle proxy]  48+52, absent in 55 -> {len(real)}개")
    for x, y in real: print(f"    ({x:6.2f},{y:6.2f})")
    print(f"\n[INFRA / persistent FP]  present in all three -> {len(infra)}개")
    for x, y in infra: print(f"    ({x:6.2f},{y:6.2f})")
    print(f"\n[TRANSIENT / spurious]  single-bag or odd combos -> {len(transient)}개")
    for tag, x, y in transient: print(f"    {tag:8s} ({x:6.2f},{y:6.2f})")
    return real, infra


if __name__ == "__main__":
    main()
