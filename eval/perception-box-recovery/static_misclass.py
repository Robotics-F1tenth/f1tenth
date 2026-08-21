#!/usr/bin/env python3
"""How often does a genuinely STATIC obstacle read as moving?

The opponent-side sweep asked what raising vel_thresh does to a moving car. This
asks the other direction on a bag whose obstacles never move: for each confirmed
landmark position, what fraction of the tracker's frames there report |v| above
a given threshold. Those frames are the ones that would be rejected from
landmark candidacy, or handed to control as dynamic.

    python3 static_misclass.py <tag>_arr.txt [<tag>_static_obstacles.txt]
"""
import sys, math
import numpy as np

arr_file = sys.argv[1]
lm_file = sys.argv[2] if len(sys.argv) > 2 else arr_file.replace("_arr.txt",
                                                                "_static_obstacles.txt")

def parse(path, want_v=True):
    out, cur = [], {}
    for line in open(path, errors="ignore"):
        s = line.strip()
        for k in ("x", "y", "v"):
            if s.startswith(k + ":"):
                try: cur[k] = float(s.split(":", 1)[1])
                except ValueError: pass
        if s.startswith("id:"):
            try: cur["id"] = int(float(s.split(":", 1)[1]))
            except ValueError: pass
            if "x" in cur and "y" in cur:
                out.append((cur.get("id", -1), cur["x"], cur["y"], abs(cur.get("v", 0.0))))
            cur = {}
    return out

arr = parse(arr_file)
lm = parse(lm_file)
if not arr or not lm:
    sys.exit("empty capture")

# landmark positions = the static obstacles this run actually settled on
seen = {}
for i, x, y, _ in lm:
    seen.setdefault(i, []).append((x, y))
sites = [(np.mean([p[0] for p in v]), np.mean([p[1] for p in v]), len(v))
         for v in seen.values()]
sites = [s for s in sites if s[2] >= 30]     # ignore one-off ids
A = np.array([(x, y, v) for _, x, y, v in arr])
print(f"확정 랜드마크 지점 {len(sites)}곳, arr 트랙 항목 {len(A)}건\n")

print(f"{'thresh':>7} {'정적물체 프레임 중 |v|>=thresh':>28}  판정")
tot_frames, per_th = 0, {}
allv = []
for sx, sy, _ in sites:
    d = np.hypot(A[:, 0]-sx, A[:, 1]-sy)
    m = d < 0.5
    if m.sum() < 20:
        continue
    allv.append(A[m, 2])
    tot_frames += int(m.sum())
if not allv:
    sys.exit("no track frames near the landmark sites")
V = np.concatenate(allv)
print(f"  (정적 지점 0.5 m 내 트랙 프레임 {len(V)}건, |v| p50 {np.median(V):.2f} "
      f"p90 {np.percentile(V,90):.2f} m/s)\n")
for th in [0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 3.0]:
    frac = 100*np.mean(V >= th)
    tag = "  <- 현재" if abs(th-0.3) < 1e-9 else ""
    print(f"  {th:5.1f} {frac:26.1f}%{tag}")
print("\n  높을수록 정적 물체가 '움직인다'고 거부되어 랜드마크가 안 된다.")
print("  즉 vel_thresh 를 올리면 이쪽 오분류는 줄고, 상대차 오분류는 늘어난다.")
