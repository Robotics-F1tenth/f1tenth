#!/usr/bin/env python3
"""Compare centroid estimators offline on captured cluster points.

The published path bends 2.1x more than GT, and raising the EKF's measurement
variance fivefold changed that by 1% -- so the jitter is in the measurement, not
the filter. The measurement is the trimmed AABB centre of the cluster, which
moves when the visible face changes with viewing angle even though the object
does not.

Each estimator here takes one frame's cluster points (map frame) and returns a
point. They are scored on how much the resulting path bends, against GT's own
bend as the floor, plus how far the point sits from GT (a stable-but-offset
estimator is not automatically better).
"""
import math
import numpy as np


def trimmed_aabb(pts, trim=0.1, **kw):
    """Current production behaviour: centre of the AABB after trimming extremes."""
    n = len(pts)
    k = max(0, int(trim*n))
    xs = np.sort(pts[:, 0]); ys = np.sort(pts[:, 1])
    return np.array([0.5*(xs[k]+xs[n-1-k]), 0.5*(ys[k]+ys[n-1-k])])


def mean_point(pts, **kw):
    """Plain mean. Follows point-density changes rather than extent changes."""
    return pts.mean(axis=0)


def median_point(pts, **kw):
    """Coordinate-wise median: robust to a few stray points, still density-driven."""
    return np.median(pts, axis=0)


def nearest_face_mid(pts, ego=None, **kw):
    """Midpoint of the visible face, i.e. of the points closest to the sensor.

    A 2D lidar only ever sees the near face, so this is the one part of the
    object that is genuinely observed. It is offset from the true centre by half
    the object depth, but that offset is roughly constant while the viewing angle
    holds -- which is what the predictor cares about.
    """
    if ego is None: return median_point(pts)
    d = np.hypot(pts[:, 0]-ego[0], pts[:, 1]-ego[1])
    sel = pts[d <= np.percentile(d, 35)]
    return sel.mean(axis=0) if len(sel) else median_point(pts)


def fixed_box_center(pts, ego=None, size=0.50, **kw):
    """Fit a fixed-size box to the visible face and report ITS centre.

    Assumes every obstacle is `size` across, which for cones and small boxes is
    close enough. The visible face gives one edge; the centre is half a box
    further along the sensor ray. Extent no longer has to be estimated per frame,
    so a changing silhouette cannot move the reported point.
    """
    if ego is None: return median_point(pts)
    face = nearest_face_mid(pts, ego)
    v = face - np.asarray(ego[:2])
    n = np.hypot(*v)
    if n < 1e-6: return face
    return face + (v/n)*(0.5*size)


def circle_fit(pts, **kw):
    """Algebraic circle fit (Kasa). The centre of a circular object is invariant
    to which arc is visible, unlike an AABB centre."""
    if len(pts) < 3: return median_point(pts)
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2*x, 2*y, np.ones(len(x))]
    b = x**2 + y**2
    try:
        c, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return median_point(pts)
    return np.array([c[0], c[1]])


METHODS = {
    "trimmed_aabb (현재)": trimmed_aabb,
    "mean": mean_point,
    "median": median_point,
    "nearest_face_mid": nearest_face_mid,
    "fixed_box_0.50": fixed_box_center,
    "circle_fit": circle_fit,
}
