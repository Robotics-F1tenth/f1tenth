#!/usr/bin/env python3
"""Decompose the ~0.5 m position scatter of a *stationary* object into
  (a) within-pass detection noise  — how much one track's reported position
      wanders while the object stays visible during a single drive-by, and
  (b) across-pass drift            — how far the MEAN position moves between
      separate sightings (separate track ids, i.e. later laps).

Uses tracks.csv (accurate per-frame timestamps from eval_logger), not the
static-landmark capture, because the latter publishes with an unset header
stamp and would force a timing approximation.

A stationary object is identified empirically: a track whose own position span
is small (< STATIC_SPAN) and that lives long enough to measure.

    python3 scatter_decomp.py <run_prefix> [run_prefix ...]
"""
import sys, csv, os
import numpy as np

STATIC_SPAN = 0.5   # m, a track that moves less than this is treated as a static object
MIN_N = 30          # frames, minimum track length to measure
GROUP_R = 1.2       # m, track means closer than this are the same physical object


def load(prefix):
    p = f"{prefix}_tracks.csv"
    if not os.path.exists(p):
        return None
    rows = [r for r in csv.reader(open(p))][1:]
    a = np.array([[float(v) for v in r] for r in rows if r])
    return a if len(a) else None


def stationary_tracks(a):
    """-> list of dict(id, t0, t1, mean, pts)"""
    t, tid, x, y = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    out = []
    for i in np.unique(tid):
        m = tid == i
        if m.sum() < MIN_N:
            continue
        xs, ys = x[m], y[m]
        span = np.hypot(xs.max() - xs.min(), ys.max() - ys.min())
        if span >= STATIC_SPAN:
            continue
        out.append({"id": int(i), "t0": t[m].min(), "t1": t[m].max(),
                    "mean": (xs.mean(), ys.mean()),
                    "pts": np.column_stack([xs, ys]), "span": span})
    return out


def group(tracks):
    n = len(tracks)
    parent = list(range(n))
    def find(a):
        while parent[a] != a: a = parent[a]
        return a
    for i in range(n):
        for j in range(i + 1, n):
            d = np.hypot(tracks[i]["mean"][0] - tracks[j]["mean"][0],
                         tracks[i]["mean"][1] - tracks[j]["mean"][1])
            if d < GROUP_R:
                ra, rb = find(i), find(j)
                if ra != rb: parent[ra] = rb
    g = {}
    for i in range(n):
        g.setdefault(find(i), []).append(tracks[i])
    return list(g.values())


def main():
    for prefix in sys.argv[1:]:
        a = load(prefix)
        name = os.path.basename(prefix)
        if a is None:
            print(f"[{name}] 데이터 없음"); continue
        st = stationary_tracks(a)
        groups = group(st)
        multi = [g for g in groups if len(g) > 1]

        # (a) within-pass: RMS radial deviation from each track's own mean
        within = []
        for tr in st:
            d = np.hypot(tr["pts"][:, 0] - tr["mean"][0], tr["pts"][:, 1] - tr["mean"][1])
            within.append(np.sqrt((d ** 2).mean()))
        # (b) across-pass: distance between track means inside one physical group
        across = []
        for g in multi:
            for i in range(len(g)):
                for j in range(i + 1, len(g)):
                    across.append(np.hypot(g[i]["mean"][0] - g[j]["mean"][0],
                                           g[i]["mean"][1] - g[j]["mean"][1]))
        print(f"\n[{name}] 정지트랙 {len(st)}개 -> 물리 위치 {len(groups)}곳 "
              f"(재방문 2회+ 인 곳 {len(multi)}곳)")
        if within:
            w = np.array(within)
            print(f"  (a) 패스 내 산포(한 트랙이 보이는 동안 흔들리는 폭, RMS): "
                  f"중앙 {np.median(w)*100:5.1f} cm   p90 {np.percentile(w,90)*100:5.1f} cm")
        if across:
            ac = np.array(across)
            print(f"  (b) 패스 간 이동(재방문 시 평균위치가 옮겨간 거리):     "
                  f"중앙 {np.median(ac)*100:5.1f} cm   p90 {np.percentile(ac,90)*100:5.1f} cm  (n={len(ac)})")
            if within:
                print(f"  -> 비율 (b)/(a) = {np.median(ac)/np.median(w):.1f}배")
        else:
            print("  (b) 재방문 쌍 없음")


if __name__ == "__main__":
    main()
