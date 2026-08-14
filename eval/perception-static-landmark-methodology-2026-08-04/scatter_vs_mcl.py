#!/usr/bin/env python3
"""Test the field hypothesis: "a static object's reported position is worse right
after a corner, while MCL has not settled, than on a straight."

For every frame of a stationary track we have an accurate timestamp, so we can
line up three things at that instant:
  - the object's position deviation from its own track mean  (the thing we care about)
  - MCL error at that time                                   (/newmcl_pose vs /mocap_pose1)
  - ego yaw rate                                             (/ekf/imu/data, corner proxy)

GT frame drops are excluded: any sample whose neighbouring mocap gap exceeds
GAP_MAX is dropped, because interpolating GT across a 200 ms hole invents up to
60 cm of error at racing speed and would fake a correlation.

    python3 scatter_vs_mcl.py <bag_key> <run_prefix>
"""
import sys, csv, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

MIN_N, STATIC_SPAN = 30, 0.5
GAP_MAX = 0.02        # s, mocap gap beyond which GT interpolation is untrustworthy


def read(bag, topic, msgtype):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
    rd.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while rd.has_next():
        _, d, _ = rd.read_next()
        m = deserialize_message(d, msgtype)
        out.append(m)
    return out


def main(bag_key, prefix):
    bag = f"/home/a/Downloads/0730/rosbag2_2026_07_30-{bag_key}"
    gt = read(bag, "/mocap_pose1", PoseStamped)
    mcl = read(bag, "/newmcl_pose", PoseStamped)
    imu = read(bag, "/ekf/imu/data", Imu)

    def arr(ms):
        return (np.array([m.header.stamp.sec + m.header.stamp.nanosec*1e-9 for m in ms]),
                np.array([m.pose.position.x for m in ms]),
                np.array([m.pose.position.y for m in ms]))
    tg, xg, yg = arr(gt); tm, xm, ym = arr(mcl)
    ti = np.array([m.header.stamp.sec + m.header.stamp.nanosec*1e-9 for m in imu])
    wz = np.abs(np.array([m.angular_velocity.z for m in imu]))

    # MCL error on the MCL timeline, dropping samples adjacent to a GT gap
    gaps = np.diff(tg)
    ok = np.ones(len(tm), bool)
    for i in np.where(gaps > GAP_MAX)[0]:
        ok &= ~((tm >= tg[i]) & (tm <= tg[i+1]))
    ex, ey = np.interp(tm, tg, xg), np.interp(tm, tg, yg)
    err = np.hypot(xm - ex, ym - ey)
    tm_ok, err_ok = tm[ok], err[ok]

    rows = [r for r in csv.reader(open(f"{prefix}_tracks.csv"))][1:]
    a = np.array([[float(v) for v in r] for r in rows if r])
    t, tid, x, y = a[:, 0], a[:, 1], a[:, 2], a[:, 3]

    dev_all, mclerr_all, wz_all = [], [], []
    for i in np.unique(tid):
        m = tid == i
        if m.sum() < MIN_N: continue
        xs, ys = x[m], y[m]
        if np.hypot(xs.max()-xs.min(), ys.max()-ys.min()) >= STATIC_SPAN: continue
        d = np.hypot(xs - xs.mean(), ys - ys.mean())
        ts = t[m]
        dev_all.extend(d.tolist())
        mclerr_all.extend(np.interp(ts, tm_ok, err_ok).tolist())
        wz_all.extend(np.interp(ts, ti, wz).tolist())
    dev = np.array(dev_all); me = np.array(mclerr_all); w = np.array(wz_all)
    print(f"\n=== {bag_key} / {os.path.basename(prefix)} ===  정지트랙 프레임 {len(dev)}개")
    print(f"  위치 편차: 중앙 {np.median(dev)*100:.1f}cm  p90 {np.percentile(dev,90)*100:.1f}cm")
    print(f"  같은 시각 MCL 오차: 중앙 {np.median(me)*100:.1f}cm")

    r = np.corrcoef(dev, me)[0, 1]
    print(f"\n  [검정1] 편차 vs MCL 오차 상관계수 r = {r:+.3f}")
    hi = me > np.percentile(me, 75); lo = me < np.percentile(me, 25)
    print(f"     MCL 오차 상위25% 구간 편차 중앙 {np.median(dev[hi])*100:5.1f}cm")
    print(f"     MCL 오차 하위25% 구간 편차 중앙 {np.median(dev[lo])*100:5.1f}cm")

    r2 = np.corrcoef(dev, w)[0, 1]
    print(f"\n  [검정2] 편차 vs 자차 각속도(코너 여부) 상관계수 r = {r2:+.3f}")
    corner = w > 1.0; straight = w < 0.3
    if corner.sum() and straight.sum():
        print(f"     코너 구간(|yaw rate|>1.0) 편차 중앙 {np.median(dev[corner])*100:5.1f}cm (n={corner.sum()})")
        print(f"     직선 구간(|yaw rate|<0.3) 편차 중앙 {np.median(dev[straight])*100:5.1f}cm (n={straight.sum()})")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
