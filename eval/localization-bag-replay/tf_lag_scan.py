#!/usr/bin/env python3
"""How stale was /tf when it arrived, and did MCL's own pose topic share the problem?

MCL publishes /newmcl_pose and the map->base_link transform in the same cycle,
so the pose topic is the control: if /tf lags while pose does not, the delay is
in /tf's transport, not in the node or the recorder.

tf2's broadcaster queue is RELIABLE and 100 deep, so a stalled subscriber lets it
fill with depth/rate seconds of transforms -- 2.5 s at 40 Hz -- and every
subscriber then reads TF that stale. Lag near 2.5 s is the signature.

    python3 tf_lag_scan.py <bagdir> [<bagdir> ...]
"""
import glob
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STALE_S = 0.25          # what a 40 Hz consumer can still plausibly use
DEPTH_100_AT_40HZ = 2.5  # the full-queue signature


def lags(cur, types, topic):
    if topic not in types:
        return np.array([]), 0
    tid = next(cur.execute("SELECT id FROM topics WHERE name=?", (topic,)))[0]
    cls = get_message(types[topic])
    out, n = [], 0
    for ts, d in cur.execute(
            f"SELECT timestamp,data FROM messages WHERE topic_id={tid} ORDER BY timestamp"):
        m = deserialize_message(d, cls)
        n += 1
        if topic == "/tf":
            for tr in m.transforms:
                if tr.child_frame_id == "base_link":
                    out.append(ts * 1e-9 - (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9))
        else:
            h = m.header
            out.append(ts * 1e-9 - (h.stamp.sec + h.stamp.nanosec * 1e-9))
    return np.array(out), n


def main():
    print(f"{'bag':>9} {'tf p50':>8} {'tf p95':>8} {'tf max':>8} {'pose p50':>9} "
          f"{'>250ms':>7} {'~2.5s':>6} {'tf/pose':>8}  판정")
    hit = []
    for bagdir in sys.argv[1:]:
        name = os.path.basename(bagdir.rstrip("/"))[-8:]
        db = glob.glob(os.path.join(bagdir, "*.db3"))
        if not db:
            continue
        cur = sqlite3.connect(db[0]).cursor()
        types = {n: t for n, t in cur.execute("SELECT name,type FROM topics")}
        tl, _ = lags(cur, types, "/tf")
        pl, pn = lags(cur, types, "/newmcl_pose")
        if tl.size < 20 or pl.size < 20:
            print(f"{name:>9}     -- (표본 부족)")
            continue
        stale = (tl > STALE_S).mean() * 100
        full = (tl > DEPTH_100_AT_40HZ * 0.8).mean() * 100
        ratio = tl.size / pl.size
        bad = np.percentile(tl, 50) > STALE_S
        if bad:
            hit.append(name)
        verdict = "증상 발생" if bad else ("경미" if stale > 5 else "정상")
        print(f"{name:>9} {np.percentile(tl,50)*1000:7.0f}ms {np.percentile(tl,95)*1000:7.0f}ms "
              f"{tl.max()*1000:7.0f}ms {np.percentile(pl,50)*1000:8.0f}ms "
              f"{stale:6.1f}% {full:5.1f}% {ratio:7.2f}  {verdict}")
    print(f"\n증상 발생 bag: {hit if hit else '없음'}  ({len(hit)}/{len(sys.argv)-1})")


if __name__ == "__main__":
    main()
