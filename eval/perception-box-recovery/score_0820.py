#!/usr/bin/env python3
"""Score a replay against data-derived ground truth for the 0820 04_08_16 bag.

GT is not hand-labelled: it is the set of map-frame cells that returned lidar
hits persistently across the whole run while NOT being explained by the map
(occupancy dilated 0.25m to absorb MCL error). An obstacle the car drove past
repeatedly shows up there; a mapped wall does not.

A detection is credited if it lands within `MATCH` of such a cluster, and
counted as a false positive if it does not -- perception claiming an obstacle
where nothing was ever seen.
"""
import sys, json, numpy as np
from scipy import ndimage

MATCH = 0.6

# --gt <file>: use hand-clicked ground truth from annotate_bag.py instead of the
# auto-derived one. The auto GT cannot tell a real obstacle from an unmapped wall
# segment, so a scored miss is ambiguous; a clicked GT removes that ambiguity.
GT_FILE = None
if "--gt" in sys.argv:
    i = sys.argv.index("--gt")
    GT_FILE = sys.argv[i + 1]
    del sys.argv[i:i + 2]

acc = np.load("/tmp/acc.npy"); g = np.load("/tmp/mapg.npy"); E = np.load("/tmp/ego.npy")
mi = json.load(open("/tmp/mapinfo.json"))
res, ox, oy = mi["res"], mi["ox"], mi["oy"]

k = int(np.ceil(0.25 / res)); occ = (g > 0)
occ_d = ndimage.binary_dilation(occ, structure=np.ones((2 * k + 1, 2 * k + 1)))
lbl, n = ndimage.label((acc >= 40) & (~occ_d), structure=np.ones((3, 3)))
dist_wall = ndimage.distance_transform_edt(~occ, sampling=res)

gt = []
for i in range(1, n + 1):
    m = (lbl == i)
    if m.sum() < 3:
        continue
    ys, xs = np.nonzero(m)
    X, Y = xs.mean() * res + ox, ys.mean() * res + oy
    hits = int(acc[m].sum())
    ext = max(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1) * res
    de = float(np.hypot(E[:, 0] - X, E[:, 1] - Y).min())
    # in-track, obstacle-sized, and seen often enough to not be a stray return
    if hits >= 3000 and de <= 2.0 and ext <= 1.3:
        gt.append((X, Y, hits, ext, float(dist_wall[int(ys.mean()), int(xs.mean())]), de))
gt.sort(key=lambda r: -r[2])

if GT_FILE:
    import json as _json
    hand = _json.load(open(GT_FILE))["obstacles"]
    gt = [(o["x"], o["y"], 0, 0.0, o.get("wall_dist") or 0.0, 0.0) for o in hand]
    print(f"손으로 찍은 GT 사용: {GT_FILE} ({len(gt)}개)")

def load(tag):
    import re
    pts, cur = {}, None
    for line in open(f"{tag}_static_obstacles.txt", errors="ignore"):
        s = line.strip()
        if s.startswith("x:"): cur = [float(s[2:])]
        elif s.startswith("y:") and cur is not None: cur.append(float(s[2:]))
        elif s.startswith("id:") and cur is not None and len(cur) == 2:
            pts.setdefault(round(float(s[3:])), []).append(tuple(cur)); cur = None
    return np.array([[np.mean([p[0] for p in v]), np.mean([p[1] for p in v])]
                     for v in pts.values()]) if pts else np.zeros((0, 2))

print(f"데이터 기반 GT {len(gt)}개 (지속 미매핑 반사, 장애물 크기, 트랙 안)")
print(f"{'x':>7} {'y':>7} {'크기':>5} {'벽거리':>6} " + " ".join(f"{t:>10}" for t in sys.argv[1:]))
det = {t: load(t) for t in sys.argv[1:]}
found = {t: 0 for t in sys.argv[1:]}
for X, Y, hits, ext, dw, de in gt:
    marks = []
    for t in sys.argv[1:]:
        D = det[t]
        ok = len(D) and np.hypot(D[:, 0] - X, D[:, 1] - Y).min() < MATCH
        found[t] += bool(ok)
        marks.append("     O    " if ok else "     .    ")
    print(f"{X:7.2f} {Y:7.2f} {ext:5.2f} {dw:6.2f} " + " ".join(marks))
print()
for t in sys.argv[1:]:
    D = det[t]
    fp = sum(1 for d in D if not len(gt) or
             min(np.hypot(X - d[0], Y - d[1]) for X, Y, *_ in gt) >= MATCH)
    print(f"{t:>12}: 검출 {found[t]}/{len(gt)}  발행 landmark {len(D)}  GT밖(FP) {fp}")
