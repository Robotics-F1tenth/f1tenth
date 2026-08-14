#!/usr/bin/env python3
"""Lap-resolved static-landmark accounting.

The user's field note is the key: "the count at the end of lap 1 is the true
object count; it grows as you keep lapping." So the interesting quantity is not
a single end-of-run number but the CURVE: how many landmark ids exist after each
lap, versus how many distinct physical objects those ids actually represent
(ids merged at MERGE_R). A flat physical curve under a rising id curve is pure
fragmentation, which is exactly what the controller suffers from.

Lap boundaries come from the ego mocap pose re-crossing its start point.

Timing caveat: /f1/perception/static_obstacles is published with an unset header
stamp, so message index -> time is approximated by uniform division over the bag
duration. Lap resolution is 14-24 s, so this is good enough here, but it is an
approximation -- fixing the publisher's stamp would make it exact.

    python3 lap_metrics.py <bag_key> <run_prefix> [run_prefix ...]
"""
import sys, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30")
from parse_static_obstacles import parse, parse_stamps

R = "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30"
MERGE_R = 0.5      # m, ids closer than this are treated as one physical object
START_R = 0.7      # m, radius around the start point that counts as a lap crossing


def ego_track(bag_key):
    bag = f"/home/a/Downloads/0730/rosbag2_2026_07_30-{bag_key}"
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/mocap_pose1"]))
    t, x, y = [], [], []
    while rd.has_next():
        _, d, _ts = rd.read_next()
        m = deserialize_message(d, PoseStamped)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        x.append(m.pose.position.x); y.append(m.pose.position.y)
    t = np.array(t); t0 = t[0]; t -= t0
    return t, np.array(x), np.array(y), t0


def lap_bounds(bag_key):
    t, x, y, t0 = ego_track(bag_key)
    d0 = np.hypot(x - x[0], y - y[0])
    near = d0 < START_R
    cross = np.where((~near[:-1]) & (near[1:]))[0]
    laps = [t[i] for i in cross if t[i] > 5.0]
    return laps, t[-1], t0


def landmark_births(prefix, duration, t0=None):
    """id -> (birth_time, mean_x, mean_y). Uses the real header.stamp (relative to
    ego t0, same /clock epoch as the bag) when the capture has it (2026-08-05+);
    falls back to uniform-duration-over-message-index approximation otherwise
    (old captures, or when t0 isn't supplied).
    """
    path = f"{R}/{prefix}_static_obstacles.txt"
    if not os.path.exists(path):
        return None
    msgs = parse(path)
    stamps = parse_stamps(path) if t0 is not None else None
    # old captures (pre-2026-08-05) have stamp=0.0 (unset field, not absent) rather
    # than None -- an epoch sanity check (real bag times are ~1.7e9+) tells them apart.
    use_real = (stamps is not None and len(stamps) == len(msgs)
                and all(s is not None and s > 1e9 for s in stamps))
    n = max(len(msgs) - 1, 1)
    births, pos = {}, {}
    for i, m in enumerate(msgs):
        for id_, x, y, v in m:
            pos.setdefault(id_, []).append((x, y))
            if id_ not in births:
                births[id_] = (stamps[i] - t0) if use_real else duration * i / n
    return {i: (births[i],
                sum(p[0] for p in pos[i]) / len(pos[i]),
                sum(p[1] for p in pos[i]) / len(pos[i])) for i in births}


def merge_count(points):
    """How many distinct physical objects a set of (x,y) collapses to."""
    pts = list(points)
    if not pts:
        return 0
    parent = list(range(len(pts)))
    def find(a):
        while parent[a] != a: a = parent[a]
        return a
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            if np.hypot(pts[i][0]-pts[j][0], pts[i][1]-pts[j][1]) < MERGE_R:
                ra, rb = find(i), find(j)
                if ra != rb: parent[ra] = rb
    return len({find(i) for i in range(len(pts))})


def main():
    bag_key = sys.argv[1]
    prefixes = sys.argv[2:]
    laps, dur, t0 = lap_bounds(bag_key)
    print(f"=== {bag_key} ===  주행 {dur:.0f}s, 랩 경계 {[f'{v:.0f}s' for v in laps]}")
    print(f"{'config':<22}" + "".join(f"{'L'+str(i+1):>14}" for i in range(len(laps))))
    print(f"{'':<22}" + "".join(f"{'id/물체':>14}" for _ in laps))
    for pfx in prefixes:
        b = landmark_births(pfx, dur, t0)
        if b is None:
            print(f"{pfx:<22}(없음)"); continue
        cells = []
        for lt in laps:
            alive = [v for v in b.values() if v[0] <= lt]
            n_id = len(alive)
            n_obj = merge_count([(v[1], v[2]) for v in alive])
            cells.append(f"{n_id:>7}/{n_obj:<6}")
        print(f"{pfx:<22}" + "".join(f"{c:>14}" for c in cells))


if __name__ == "__main__":
    main()
