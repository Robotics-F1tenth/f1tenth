#!/usr/bin/env python3
"""Estimators aimed at the close-range failure: the point COUNT jumping.

Measured on 0821 at 0-2 m with the sparse filter already on: frames in the worst
10% of path bend differ from the best 50% almost entirely by how much the point
count changed between frames (9.5 points vs 1), not by count, spacing or range.
A beam group appearing or leaving moves an extent-based centre even though the
object did not.

So each estimator here is built to be insensitive to how many points arrive:
take a fixed number of them, or weight by geometry rather than by membership.
"""
import math
import numpy as np


def trimmed_aabb(pts, ego=None, **kw):
    n = len(pts); k = max(0, int(0.1*n))
    xs = np.sort(pts[:, 0]); ys = np.sort(pts[:, 1])
    return np.array([0.5*(xs[k]+xs[n-1-k]), 0.5*(ys[k]+ys[n-1-k])])


def nearest_k(pts, ego=None, k=6, **kw):
    """Mean of the k points closest to the sensor. Fixed count by construction,
    so points joining or leaving the far side change nothing."""
    if ego is None or len(pts) <= k: return pts.mean(axis=0)
    d = np.hypot(pts[:, 0]-ego[0], pts[:, 1]-ego[1])
    return pts[np.argsort(d)[:k]].mean(axis=0)


def nearest_k_box(pts, ego=None, k=6, size=0.50, **kw):
    """nearest_k, then step half an object depth along the ray -- the near face
    is what is observed, the centre is inferred from an assumed size."""
    face = nearest_k(pts, ego, k)
    if ego is None: return face
    v = face - np.asarray(ego[:2]); n = np.hypot(*v)
    return face if n < 1e-6 else face + (v/n)*(0.5*size)


def closest_point(pts, ego=None, **kw):
    """The single closest return. Maximally insensitive to count, at the cost of
    riding on one beam's noise."""
    if ego is None: return pts.mean(axis=0)
    d = np.hypot(pts[:, 0]-ego[0], pts[:, 1]-ego[1])
    return pts[np.argmin(d)]


def range_weighted(pts, ego=None, tau=0.15, **kw):
    """Weight each point by exp(-(r - r_min)/tau): near-face points dominate and a
    far-side group arriving carries almost no weight. Continuous, so unlike a
    hard cut it cannot jump when a point crosses the boundary."""
    if ego is None: return pts.mean(axis=0)
    d = np.hypot(pts[:, 0]-ego[0], pts[:, 1]-ego[1])
    w = np.exp(-(d-d.min())/tau)
    return (pts[:, :2]*w[:, None]).sum(axis=0)/w.sum()


METHODS = {
    "trimmed_aabb (현재)": trimmed_aabb,
    "nearest_k(6)": nearest_k,
    "nearest_k_box(6,0.5)": nearest_k_box,
    "closest_point": closest_point,
    "range_weighted(0.15)": range_weighted,
}
