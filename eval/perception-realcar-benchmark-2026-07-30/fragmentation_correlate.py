#!/usr/bin/env python3
"""Correlate static-landmark ID births with ego yaw rate (|imu.angular_velocity.z|)
and with per-bag known duration (approx birth time = duration * msg_idx/total_msgs,
since the static_obstacles topic's own header stamp is never set by the publisher).

    python3 fragmentation_correlate.py <static_obstacles.txt> <bag_path> <duration_s>
"""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
import numpy as np
from parse_static_obstacles import parse


def load_yawrate(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/ekf/imu/data"]))
    ts, wz = [], []
    while reader.has_next():
        _, data, t = reader.read_next()
        msg = deserialize_message(data, Imu)
        ts.append(t / 1e9)
        wz.append(msg.angular_velocity.z)
    ts = np.array(ts); wz = np.array(wz)
    return ts, wz


def main(txt_path, bag_path, duration_s):
    msgs = parse(txt_path)
    n_total = len(msgs)
    by_id = {}
    for i, m in enumerate(msgs):
        for id_, x, y, v in m:
            by_id.setdefault(id_, []).append((i, x, y))

    ts, wz = load_yawrate(bag_path)
    t0 = ts.min()
    overall_absmean = np.abs(wz).mean()

    print(f"bag duration={duration_s}s  total /ekf/imu/data samples={len(ts)}  "
          f"overall mean|yaw_rate|={overall_absmean:.3f} rad/s")
    print(f"{'id':>6}{'birth_t':>9}{'|yawrate| @birth (±1s window mean)':>36}{'  ratio_vs_overall':>20}")
    rows = []
    for id_, pts in sorted(by_id.items(), key=lambda kv: kv[1][0][0]):
        idx0 = pts[0][0]
        birth_t = t0 + duration_s * (idx0 / max(n_total - 1, 1))
        win = (ts >= birth_t - 1.0) & (ts <= birth_t + 1.0)
        local = np.abs(wz[win]).mean() if win.sum() else float("nan")
        ratio = local / overall_absmean if overall_absmean > 0 else float("nan")
        rows.append((id_, birth_t - t0, local, ratio))
        print(f"{id_:6.0f}{birth_t - t0:9.1f}{local:36.3f}{ratio:20.2f}")

    ratios = np.array([r[3] for r in rows if not np.isnan(r[3])])
    print(f"\nmedian ratio (local/overall |yaw_rate| at birth) across {len(ratios)} ids: {np.median(ratios):.2f}")
    print(f"fraction of births with ratio>1.2 (elevated yaw rate): "
          f"{(ratios > 1.2).mean()*100:.0f}%")


if __name__ == "__main__":
    import sys
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]))
