#!/usr/bin/env python3
"""Where does MCL's ~180 ms lag come from, if a perception cycle is only ~0.6 ms?

The 180 ms is MCL's estimate vs GT, so perception's cycle time is irrelevant to
it. This decomposes the plausible contributors that ARE measurable from the bag:

  1. /newmcl_pose publish interval   -> sampling staleness (half a period, mean)
  2. /odom velocity lag vs GT        -> the motion model's input is already old
  3. scan stamp -> pose stamp        -> MCL's own compute latency

    python3 mcl_lag_budget.py [bag]
"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

BAG = sys.argv[1] if len(sys.argv) > 1 else "/home/a/Downloads/f1tenth_bag/rosbag2_2026_08_17-23_44_03"
GT_TOPIC = "/mocap_cal/car3/base"
SPLIT_T = 44.0  # ignore the post-spin frozen segment


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(
        topics=[GT_TOPIC, "/newmcl_pose", "/odom", "/scan"]))
    gt, mcl, odom, scan = [], [], [], []
    while r.has_next():
        tn, d, recv = r.read_next()
        if tn in (GT_TOPIC, "/newmcl_pose"):
            m = deserialize_message(d, PoseStamped)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            row = [s, m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation),
                   recv * 1e-9]
            (gt if tn == GT_TOPIC else mcl).append(row)
        elif tn == "/odom":
            m = deserialize_message(d, Odometry)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            odom.append([s, m.twist.twist.linear.x, m.twist.twist.angular.z])
        elif tn == "/scan":
            m = deserialize_message(d, LaserScan)
            scan.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return (np.array(gt), np.array(mcl), np.array(odom), np.array(scan))


def main():
    gt, mcl, odom, scan = read()
    t0 = gt[0, 0]
    print(f"bag={BAG.split('/')[-1]}")
    print(f"  gt={len(gt)} mcl={len(mcl)} odom={len(odom)} scan={len(scan)}")

    # --- 1. publish интервал -------------------------------------------------
    dt = np.diff(mcl[:, 0])
    dt = dt[dt > 0]
    print("\n=== 1. /newmcl_pose 발행 간격 ===")
    print(f"  평균 {1000*dt.mean():.1f} ms ({1/dt.mean():.1f} Hz), "
          f"p50 {1000*np.median(dt):.1f}, p95 {1000*np.percentile(dt,95):.1f} ms")
    print(f"  -> 샘플링만으로 평균 {1000*dt.mean()/2:.0f} ms 만큼 오래된 값을 보게 됨")
    if len(scan):
        sdt = np.diff(scan); sdt = sdt[sdt > 0]
        print(f"  (/scan 은 {1/sdt.mean():.1f} Hz)")

    # --- 2. odom velocity lag vs GT -----------------------------------------
    # GT velocity by centered difference of mocap positions.
    g = gt[gt[:, 0] - t0 < SPLIT_T]
    gt_t = g[:, 0]
    gvx = np.gradient(g[:, 1], gt_t)
    gvy = np.gradient(g[:, 2], gt_t)
    gspeed = np.hypot(gvx, gvy)
    gyaw = np.unwrap(g[:, 3])
    gwz = np.gradient(gyaw, gt_t)
    # light smoothing: mocap diff is noisy
    k = 9
    ker = np.ones(k) / k
    gspeed_s = np.convolve(gspeed, ker, mode="same")
    gwz_s = np.convolve(gwz, ker, mode="same")

    o = odom[odom[:, 0] - t0 < SPLIT_T]
    print("\n=== 2. /odom 속도가 GT 대비 얼마나 늦나 (shift 스윕) ===")
    print(f"{'shift[ms]':>10} {'vx_rmse':>9} {'wz_rmse':>9}")
    best_v = best_w = None
    for sh_ms in range(-40, 261, 20):
        sh = sh_ms / 1000.0
        # odom sample at t compared to GT at t - sh  (positive sh = odom is late)
        gq_v = np.interp(o[:, 0] - sh, gt_t, gspeed_s)
        gq_w = np.interp(o[:, 0] - sh, gt_t, gwz_s)
        rv = float(np.sqrt(np.mean((np.abs(o[:, 1]) - gq_v) ** 2)))
        rw = float(np.sqrt(np.mean((o[:, 2] - gq_w) ** 2)))
        if best_v is None or rv < best_v[1]:
            best_v = (sh_ms, rv)
        if best_w is None or rw < best_w[1]:
            best_w = (sh_ms, rw)
        print(f"{sh_ms:>10} {rv:>9.4f} {rw:>9.4f}")
    print(f"  -> vx 최적 shift {best_v[0]} ms, wz 최적 shift {best_w[0]} ms")
    print("     (양수 = odom 이 GT 보다 그만큼 늦음 = 모션모델 입력이 이미 낡음)")

    # --- 3. MCL compute latency: pose stamp vs nearest earlier scan stamp ----
    if len(scan):
        print("\n=== 3. MCL 계산 지연 (pose stamp - 직전 scan stamp) ===")
        idx = np.searchsorted(scan, mcl[:, 0]) - 1
        ok = idx >= 0
        lat = mcl[ok, 0] - scan[idx[ok]]
        lat = lat[(lat >= 0) & (lat < 0.5)]
        if len(lat):
            print(f"  p50 {1000*np.median(lat):.1f} ms, p95 {1000*np.percentile(lat,95):.1f} ms")
            print("  (pose 가 scan stamp 를 그대로 쓰면 0 에 가깝게 나옴)")

    # --- 4. receive-vs-stamp: how stale is the pose when it lands -----------
    if mcl.shape[1] > 4:
        age = mcl[:, 4] - mcl[:, 0]
        age = age[(age > -1) & (age < 1)]
        print("\n=== 4. /newmcl_pose 수신시각 - stamp (기록 시점 기준) ===")
        print(f"  p50 {1000*np.median(age):.1f} ms, p95 {1000*np.percentile(age,95):.1f} ms")


if __name__ == "__main__":
    main()
