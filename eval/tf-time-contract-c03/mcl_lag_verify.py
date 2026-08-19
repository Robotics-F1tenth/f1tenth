#!/usr/bin/env python3
"""Independent check of the "MCL lags GT by 160-180 ms" claim on the 0817 bag.

Different question from tf_lag_test.py: that one measures the TF *label* lead
(a stamping issue). This measures whether the /newmcl_pose *estimate itself*
trails GT in time -- shift the MCL series forward and see where error bottoms.
Also splits normal driving from the post-spin-stop segment the user flagged,
since a frozen offset there would otherwise contaminate the lag fit.

    python3 mcl_lag_verify.py [bag] [split_t]
"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

BAG = sys.argv[1] if len(sys.argv) > 1 else "/home/a/Downloads/f1tenth_bag/rosbag2_2026_08_17-23_44_03"
SPLIT_T = float(sys.argv[2]) if len(sys.argv) > 2 else 44.0
GT_TOPIC = "/mocap_cal/car3/base"
MCL_TOPIC = "/newmcl_pose"


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read():
    so = rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3")
    r = rosbag2_py.SequentialReader()
    r.open(so, rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[GT_TOPIC, MCL_TOPIC, "/odom"]))
    gt, mcl, odom = [], [], []
    while r.has_next():
        tn, d, _ = r.read_next()
        if tn in (GT_TOPIC, MCL_TOPIC):
            m = deserialize_message(d, PoseStamped)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            row = [s, m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)]
            (gt if tn == GT_TOPIC else mcl).append(row)
        elif tn == "/odom":
            m = deserialize_message(d, Odometry)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            odom.append([s, m.twist.twist.linear.x, m.twist.twist.angular.z])
    return np.array(gt), np.array(mcl), np.array(odom)


def err_at_shift(gt, mcl, shift, lo, hi, t0):
    """Position/yaw error when MCL sample at time t is compared to GT at t - shift.
    A positive best-shift means MCL is reporting a pose GT already passed = lag."""
    m = mcl[(mcl[:, 0] - t0 >= lo) & (mcl[:, 0] - t0 < hi)]
    if len(m) < 10:
        return None
    tq = m[:, 0] - shift
    gx = np.interp(tq, gt[:, 0], gt[:, 1])
    gy = np.interp(tq, gt[:, 0], gt[:, 2])
    gyaw = np.interp(tq, gt[:, 0], np.unwrap(gt[:, 3]))
    d = np.hypot(m[:, 1] - gx, m[:, 2] - gy)
    ye = np.array([wrap(a - b) for a, b in zip(m[:, 3], gyaw)])
    return np.median(d), np.degrees(np.mean(ye)), np.degrees(np.std(ye)), len(m)


def main():
    gt, mcl, odom = read()
    t0 = min(gt[0, 0], mcl[0, 0])
    dur = max(gt[-1, 0], mcl[-1, 0]) - t0
    print(f"bag={BAG.split('/')[-1]}  dur={dur:.1f}s  gt={len(gt)}  mcl={len(mcl)}")

    for label, lo, hi in [("전체", 0, 1e9), ("정상구간(0~%.0fs)" % SPLIT_T, 0, SPLIT_T),
                          ("급회전이후(>=%.0fs)" % SPLIT_T, SPLIT_T, 1e9)]:
        print(f"\n=== {label} ===")
        print(f"{'shift[ms]':>10} {'p50[cm]':>9} {'yaw_mean[deg]':>14} {'yaw_std[deg]':>13} {'n':>5}")
        best = None
        for sh_ms in range(0, 301, 20):
            r = err_at_shift(gt, mcl, sh_ms / 1000.0, lo, hi, t0)
            if r is None:
                continue
            p50, ym, ys, n = r
            mark = ""
            if best is None or p50 < best[1]:
                best = (sh_ms, p50); mark = ""
            print(f"{sh_ms:>10} {100*p50:>9.2f} {ym:>14.2f} {ys:>13.2f} {n:>5}")
        if best:
            print(f"  -> 최소 오차 shift = {best[0]} ms (p50 {100*best[1]:.2f} cm)")

    if len(odom):
        t = odom[:, 0] - t0
        sel = (t >= 43.0) & (t <= 48.5)
        print("\n=== /odom t=43~48.5s (급회전+정지 확인) ===")
        print(f"  |wz| max = {np.max(np.abs(odom[sel, 2])):.2f} rad/s"
              f" ({np.degrees(np.max(np.abs(odom[sel, 2]))):.0f} deg/s)")
        print(f"  vx: {odom[sel, 1].max():.2f} -> {odom[sel, 1][-1]:.2f} m/s")
        still = np.abs(odom[:, 1]) < 0.01
        if still.any():
            print(f"  vx≈0 최초 시각 = {t[still][0]:.1f}s, 이후 정지 유지 = {bool(still[t >= t[still][0]].all())}")


if __name__ == "__main__":
    main()
