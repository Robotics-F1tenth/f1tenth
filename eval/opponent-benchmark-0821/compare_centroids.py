#!/usr/bin/env python3
"""Score centroid estimators on captured cluster points.

    python3 compare_centroids.py [clusters_out]

For each frame, every estimator is run on the cluster nearest the opponent's GT
position, giving one path per method over the same frames. Reported per method:

  bend p50/p90   second difference of consecutive points [cm] -- the smoothness
                 the predictor actually consumes. GT's own bend is the floor.
  offset p50     distance from GT [m]. A method can be smooth simply by sitting
                 still in the wrong place, so this has to be read alongside.
  jump p90       largest single-step move [cm], the tail that breaks a curve fit.
"""
import math, sys, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import sensor_msgs_py.point_cloud2 as pc2
from centroid_methods import METHODS

BAG = sys.argv[1] if len(sys.argv) > 1 else "clusters_out"
MATCH = 1.0

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
gt, clouds, tfs, ego = [], [], [], []
while reader.has_next():
    tn, d, _ = reader.read_next()
    if tn == "/mocap/car3/pose":
        m = deserialize_message(d, PoseStamped)
        gt.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                   m.pose.position.x, m.pose.position.y))
    elif tn == "/newmcl_pose":
        m = deserialize_message(d, PoseStamped)
        ego.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    m.pose.position.x, m.pose.position.y))
    elif tn == "/obj_cluster":
        m = deserialize_message(d, PointCloud2)
        pts = np.array([[p[0], p[1], p[3]] for p in pc2.read_points(
            m, field_names=("x", "y", "z", "intensity"), skip_nans=True)])
        clouds.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9, pts))
    elif tn == "/tf":
        m = deserialize_message(d, TFMessage)
        for t in m.transforms:
            if t.child_frame_id in ("laser", "base_link"):
                q = t.transform.rotation
                tfs.append((t.header.stamp.sec+t.header.stamp.nanosec*1e-9,
                            t.transform.translation.x, t.transform.translation.y,
                            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                            t.child_frame_id))
G = np.array(sorted(gt)); E = np.array(sorted(ego))
clouds.sort(key=lambda c: c[0])
# /obj_cluster carries two entries per scan stamp (the empty-cloud branch also
# publishes), and duplicates halve the apparent step, inflating both the GT floor
# and every method's bend. Keep the last entry per stamp.
dedup = {}
for t, pts in clouds:
    dedup[round(t, 6)] = pts
clouds = [(t, dedup[t]) for t in sorted(dedup)]
print(f"deduped clusters -> {len(clouds)} frames")
T = np.array(sorted([(a, b, c, d) for a, b, c, d, _ in tfs])) if tfs else None
print(f"clusters {len(clouds)} frames, gt {len(G)}, ego {len(E)}, tf {0 if T is None else len(T)}")
if not clouds or len(G) < 10:
    sys.exit("capture incomplete")

paths = {k: [] for k in METHODS}
gtpath, idxs = [], []
for t, pts in clouds:
    if len(pts) == 0: continue
    gx = np.interp(t, G[:, 0], G[:, 1]); gy = np.interp(t, G[:, 0], G[:, 2])
    ex = np.interp(t, E[:, 0], E[:, 1]); ey = np.interp(t, E[:, 0], E[:, 2])
    if T is not None and len(T):
        px = np.interp(t, T[:, 0], T[:, 1]); py = np.interp(t, T[:, 0], T[:, 2])
        yaw = np.interp(t, T[:, 0], np.unwrap(T[:, 3]))
    else:
        px, py, yaw = ex, ey, 0.0
    # cluster points arrive in the lidar frame; lift to map with the same TF
    c, s = math.cos(yaw), math.sin(yaw)
    mx = px + c*pts[:, 0] - s*pts[:, 1]
    my = py + s*pts[:, 0] + c*pts[:, 1]
    ids = pts[:, 2].astype(int)
    best, bd = None, 1e9
    for cid in np.unique(ids):
        sel = ids == cid
        cxy = np.c_[mx[sel], my[sel]]
        d = np.hypot(cxy[:, 0].mean()-gx, cxy[:, 1].mean()-gy)
        if d < bd: bd, best = d, cxy
    if best is None or bd > MATCH: continue
    for name, fn in METHODS.items():
        paths[name].append(fn(best, ego=(px, py)))
    gtpath.append((gx, gy))

n = len(gtpath)
print(f"opponent matched in {n} frames\n")
if n < 20: sys.exit("too few matched frames")
GP = np.array(gtpath)


def bend(P):
    if len(P) < 3: return np.array([np.nan])
    return np.hypot(*np.diff(P, 2, axis=0).T)*100


def step(P):
    if len(P) < 2: return np.array([np.nan])
    return np.hypot(*np.diff(P, axis=0).T)*100


gb = bend(GP)  # GP is already one sample per kept cluster frame
print(f"GT 바닥: bend p50 {np.median(gb):.2f} cm  p90 {np.percentile(gb,90):.2f} cm\n")
print(f"{'method':>22} {'bend p50':>9} {'bend p90':>9} {'GT배수':>7} "
      f"{'offset p50':>11} {'jump p90':>9}")
for name in METHODS:
    P = np.array(paths[name])
    b = bend(P); off = np.hypot(P[:, 0]-GP[:, 0], P[:, 1]-GP[:, 1])
    print(f"{name:>22} {np.median(b):8.2f}c {np.percentile(b,90):8.2f}c "
          f"{np.median(b)/max(np.median(gb),1e-9):6.2f}x {np.median(off):10.3f}m "
          f"{np.percentile(step(P),90):8.2f}c")
