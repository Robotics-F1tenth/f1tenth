#!/usr/bin/env python3
"""Score a bag's recorded /newmcl_pose by scan fit, with no ground truth.

Reimplements MCL's calculateCurrentPoseScanLikelihoodForCheck() -- the same
mean-log-p-per-beam the node logs as "MCL health: fit=" -- and evaluates it at
the poses the car actually published, rather than at poses from a fresh desktop
replay. That answers "was the on-car estimate sitting on the map correctly",
which is the only question ground truth would have settled.

Calibration from bags that did have mocap (docs: roadmap §0.7/§0.8):
    converged  fit p50 -2.3 to -2.6,  p5 about -3.3
    lost       fit p50 -4.5
    threshold  -3.5 separates them at 1.1% false alarm / 76.7% detection

Both the model and its constants are copied from MCL.h; see MODEL below. Keep
them in sync if the measurement model changes.

    python3 fit_from_bag.py <bagdir> [<bagdir> ...]
"""
import bisect
import glob
import math
import os
import sqlite3
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# MODEL -- mirrors als_ros/params/mcl_params.yaml and MCL.h lines 881-885.
Z_HIT, Z_RAND, Z_MAX = 0.9, 0.03, 0.04
VAR_HIT = 0.01
LAMBDA_UNKNOWN = 0.05
KNOWN_PRIOR = 0.7
SCAN_STEP = 10          # deployed value; fit is insensitive to it (0820: -2.30 vs -2.31)
MAX_PAIR_DT = 0.02      # s, scan-to-pose stamp match


def read(bagdir, topics):
    db = glob.glob(os.path.join(bagdir, "*.db3"))
    if not db:
        return {}
    con = sqlite3.connect(db[0])
    cur = con.cursor()
    types = {n: t for n, t in cur.execute("SELECT name,type FROM topics")}
    out = {t: [] for t in topics}
    for t in topics:
        if t not in types:
            continue
        cls = get_message(types[t])
        tid = next(cur.execute("SELECT id FROM topics WHERE name=?", (t,)))[0]
        for ts, d in cur.execute(
                f"SELECT timestamp,data FROM messages WHERE topic_id={tid} ORDER BY timestamp"):
            out[t].append((ts * 1e-9, deserialize_message(d, cls)))
    con.close()
    return out


def dist_map(grid):
    """Metres to the nearest occupied cell, as MCL builds it."""
    w, h, res = grid.info.width, grid.info.height, grid.info.resolution
    data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    binmap = np.where(data == 100, 0, 1).astype(np.uint8)
    dm = cv2.distanceTransform(binmap, cv2.DIST_L2, 5) * res
    return dm, res, grid.info.origin.position.x, grid.info.origin.position.y, w, h


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fits_for_bag(bagdir):
    msgs = read(bagdir, ["/map", "/scan", "/newmcl_pose", "/tf_static"])
    if not msgs.get("/map") or not msgs.get("/scan") or not msgs.get("/newmcl_pose"):
        return None
    dm, res, ox, oy, w, h = dist_map(msgs["/map"][0][1])

    lx = ly = lyaw = 0.0
    for _, m in msgs.get("/tf_static", []):
        for tr in m.transforms:
            if tr.child_frame_id.endswith("laser"):
                lx, ly = tr.transform.translation.x, tr.transform.translation.y
                lyaw = yaw_of(tr.transform.rotation)

    poses = [(t, m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
             for t, m in msgs["/newmcl_pose"]]
    poses.sort()
    pts = [p[0] for p in poses]

    norm_hit = 1.0 / math.sqrt(2.0 * VAR_HIT * math.pi)
    denom_hit = 1.0 / (2.0 * VAR_HIT)

    out = []
    for st, sc in msgs["/scan"]:
        stamp = sc.header.stamp.sec + sc.header.stamp.nanosec * 1e-9
        i = bisect.bisect_left(pts, stamp)
        best = None
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(poses):
                d = abs(poses[j][0] - stamp)
                if best is None or d < best[0]:
                    best = (d, poses[j])
        if best is None or best[0] > MAX_PAIR_DT:
            continue
        _, x, y, yaw = best[1]

        # base_link -> laser, exactly as the C++ does it
        sx = lx * math.cos(yaw) - ly * math.sin(yaw) + x
        sy = lx * math.sin(yaw) + ly * math.cos(yaw) + y
        syaw = lyaw + yaw

        r = np.asarray(sc.ranges[::SCAN_STEP], dtype=np.float64)
        idx = np.arange(0, len(sc.ranges), SCAN_STEP)
        ang = idx * sc.angle_increment + sc.angle_min + syaw
        ok = np.isfinite(r) & (r > sc.range_min) & (r < sc.range_max)
        if ok.sum() < 10:
            continue
        r, ang = r[ok], ang[ok]

        p_rand = 1.0 / (sc.range_max / res)
        mm_random = Z_RAND * p_rand
        unknown_denom = 1.0 - math.exp(-LAMBDA_UNKNOWN * sc.range_max)
        p_unknown = (LAMBDA_UNKNOWN * np.exp(-LAMBDA_UNKNOWN * r) / unknown_denom
                     * res * (1.0 - KNOWN_PRIOR))

        hx = r * np.cos(ang) + sx
        hy = r * np.sin(ang) + sy
        u = np.floor((hx - ox) / res).astype(int)
        v = np.floor((hy - oy) / res).astype(int)
        on = (u >= 0) & (u < w) & (v >= 0) & (v < h)

        p = p_unknown.copy()
        d = np.zeros_like(r)
        d[on] = dm[v[on], u[on]]
        p_hit = norm_hit * np.exp(-(d * d) * denom_hit) * res
        p[on] += (Z_HIT * p_hit[on] + mm_random) * KNOWN_PRIOR
        p[~on] += mm_random * KNOWN_PRIOR
        p = np.minimum(p, 1.0)

        out.append((stamp, float(np.mean(np.log(p))), int(ok.sum())))
    return out


def main():
    print(f"{'bag':>9} {'n':>5} {'fit p50':>8} {'p5':>7} {'min':>7} "
          f"{'<-3.5':>6} {'<-3.0':>6}  판정")
    for bagdir in sys.argv[1:]:
        name = os.path.basename(bagdir.rstrip("/"))[-8:]
        res = fits_for_bag(bagdir)
        if not res:
            print(f"{name:>9}     -- (scan/pose/map 부족)")
            continue
        f = np.array([x[1] for x in res])
        lost = (f < -3.5).mean() * 100
        warn = (f < -3.0).mean() * 100
        verdict = "정상" if lost < 1 and np.percentile(f, 50) > -3.0 else (
            "의심" if lost < 10 else "불량")
        print(f"{name:>9} {len(f):5d} {np.percentile(f,50):8.2f} {np.percentile(f,5):7.2f} "
              f"{f.min():7.2f} {lost:5.1f}% {warn:5.1f}%  {verdict}")


if __name__ == "__main__":
    main()
