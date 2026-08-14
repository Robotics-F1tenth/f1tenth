#!/usr/bin/env python3
"""One-line metric summary for a perception run produced by run_bag_perception.sh.
Pulls together: static-landmark fragmentation, dynamic track counts, tracker
accuracy vs GT, and per-cycle compute time.

    python3 summarize.py <run_prefix_path> [label]
"""
import sys, os, re, subprocess
sys.path.insert(0, "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30")
from parse_static_obstacles import parse

CLUSTER_R = 0.5   # m; two landmark ids closer than this are treated as one physical object


def landmark_stats(txt):
    if not os.path.exists(txt):
        return None
    msgs = parse(txt)
    cents = {}
    for i, m in enumerate(msgs):
        for id_, x, y, v in m:
            cents.setdefault(id_, []).append((x, y))
    out = {}
    for id_, pts in cents.items():
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        out[id_] = (sum(xs)/len(xs), sum(ys)/len(ys),
                    ((max(xs)-min(xs))**2 + (max(ys)-min(ys))**2) ** 0.5)
    ids = sorted(out)
    parent = {i: i for i in ids}
    def find(a):
        while parent[a] != a: a = parent[a]
        return a
    for i, a in enumerate(ids):
        for b in ids[i+1:]:
            if ((out[a][0]-out[b][0])**2 + (out[a][1]-out[b][1])**2) ** 0.5 < CLUSTER_R:
                ra, rb = find(a), find(b)
                if ra != rb: parent[ra] = rb
    groups = {}
    for i in ids:
        groups.setdefault(find(i), []).append(i)
    frag = [g for g in groups.values() if len(g) > 1]
    return {"n_ids": len(ids), "n_clusters": len(groups), "n_frag": len(frag),
            "max_drift": max((out[i][2] for i in ids), default=0.0),
            "centroids": {i: (out[i][0], out[i][1]) for i in ids}}


def accuracy_stats(prefix):
    try:
        r = subprocess.run(["python3", "/home/a/f1tenth/eval/tools/analyze_perception_tracking.py", prefix],
                           capture_output=True, text=True, timeout=300)
    except Exception:
        return {}
    o = r.stdout
    d = {}
    m = re.search(r"(\d+) detections, (\d+) distinct IDs", o)
    if m: d["detections"], d["track_ids"] = int(m.group(1)), int(m.group(2))
    m = re.search(r"matched: (\d+) \(([\d.]+)%\)\s+dropouts: (\d+)", o)
    if m: d["matched"], d["match_pct"], d["dropouts"] = int(m.group(1)), float(m.group(2)), int(m.group(3))
    m = re.search(r"position err: mean ([\d.]+) m", o)
    if m: d["pos_err"] = float(m.group(1))
    m = re.search(r"MAE ([\d.]+)\s+RMSE ([\d.]+)\s+median ([\d.]+)", o)
    if m: d["yaw_mae"], d["yaw_median"] = float(m.group(1)), float(m.group(3))
    m = re.search(r"course-yaw err \(GT speed >= [\d.]+ m/s, n=(\d+)\)", o)
    if m: d["yaw_n"] = int(m.group(1))
    return d


def compute_stats(log):
    if not os.path.exists(log):
        return {}
    vals = [float(x) for x in re.findall(r"total=([\d.]+) µs", open(log, errors="ignore").read())]
    if not vals:
        return {}
    vals.sort()
    return {"cyc_n": len(vals), "cyc_med": vals[len(vals)//2],
            "cyc_p95": vals[int(len(vals)*0.95)], "cyc_max": vals[-1]}


def main():
    prefix = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(prefix)
    lm = landmark_stats(f"{prefix}_static_obstacles.txt")
    acc = accuracy_stats(prefix)
    cyc = compute_stats(f"{prefix}_perception.log")
    crash = 0
    plog = f"{prefix}_perception.log"
    if os.path.exists(plog):
        crash = open(plog, errors="ignore").read().count("process has died")
    print(f"{label:28s} "
          f"lm_ids={lm['n_ids'] if lm else -1:4d} clus={lm['n_clusters'] if lm else -1:3d} "
          f"frag={lm['n_frag'] if lm else -1:3d} drift={lm['max_drift'] if lm else -1:5.2f} | "
          f"trk_ids={acc.get('track_ids',-1):4d} match={acc.get('match_pct',-1):5.1f}% "
          f"pos={acc.get('pos_err',-1):.3f} yawMAE={acc.get('yaw_mae',-1):6.2f}(n={acc.get('yaw_n',-1)}) | "
          f"cyc_med={cyc.get('cyc_med',-1):6.1f}us p95={cyc.get('cyc_p95',-1):7.1f} | crash={crash}")

main()
