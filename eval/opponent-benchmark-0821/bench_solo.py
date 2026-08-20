#!/usr/bin/env python3
"""Ego-side numbers for bags that have only one car in mocap.

Same bag family and same on-car build as bench_opponent.py, so these act as the
control: whatever the opponent bag shows has to be read against a solo run under
the same conditions, not against a different day's numbers.
"""
import sys, math, json
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

MOCAP = ["/mocap/car3/pose", "/mocap/car4/pose"]


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def main():
    bag = sys.argv[1].rstrip("/")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(
        topics=MOCAP + ["/newmcl_pose", "/f1/perception/object/obstacles/arr"]))
    mc = {t: [] for t in MOCAP}
    ego, arr = [], []
    while r.has_next():
        tn, d, _ = r.read_next()
        if tn in MOCAP or tn == "/newmcl_pose":
            m = deserialize_message(d, PoseStamped)
            row = (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                   m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
            (ego if tn == "/newmcl_pose" else mc[tn]).append(row)
        else:
            m = deserialize_message(d, F1stateArr)
            arr.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                        len(m.f1_state_arr)))
    ego = np.array(sorted(ego))
    arr.sort()
    name = bag.split("/")[-1]
    have = {t: np.array(sorted(v)) for t, v in mc.items() if v}
    out = {"bag": name, "cycles": len(arr)}

    ntr = np.array([a[1] for a in arr]) if arr else np.zeros(0)
    empty = float(np.mean(ntr == 0)) if len(ntr) else float("nan")
    dur = (arr[-1][0]-arr[0][0]) if arr else 0.0
    print(f"bag={name}  {dur:.0f}s  arr {len(arr)} 사이클")
    print(f"  트랙 0개 사이클 {100*empty:.0f}%   트랙수 mean {ntr.mean():.2f} max {int(ntr.max()) if len(ntr) else 0}")
    out.update(empty_frac=empty, tracks_mean=float(ntr.mean()) if len(ntr) else None)

    if have and len(ego) > 10:
        # ego = mocap car closest to /newmcl_pose
        fit = {}
        for t, g in have.items():
            idx = np.linspace(0, len(ego)-1, min(len(ego), 2000)).astype(int)
            gx = np.interp(ego[idx, 0], g[:, 0], g[:, 1])
            gy = np.interp(ego[idx, 0], g[:, 0], g[:, 2])
            fit[t] = float(np.median(np.hypot(ego[idx, 1]-gx, ego[idx, 2]-gy)))
        et = min(fit, key=fit.get)
        g = have[et]
        # GT_COVER: mocap can start late or drop out (car4 in 02_27_51 only begins at
        # 345s of a 633s bag). np.interp holds the endpoint value outside the data, so
        # comparing there scores the estimator against a frozen point and reports
        # metres of "error" that are entirely the interpolation. Score only inside the
        # window mocap actually covers.
        lo, hi = g[0, 0], g[-1, 0]
        inside = (ego[:, 0] >= lo) & (ego[:, 0] <= hi)
        cover = float(np.mean(inside))
        if cover < 0.999:
            print(f"  GT 커버리지 {100*cover:.0f}% "
                  f"(mocap {lo-ego[0,0]:.0f}~{hi-ego[0,0]:.0f}s) -- 그 구간만 채점")
        ego = ego[inside]
        idx = np.linspace(0, len(ego)-1, min(len(ego), 20000)).astype(int)
        gx = np.interp(ego[idx, 0], g[:, 0], g[:, 1])
        gy = np.interp(ego[idx, 0], g[:, 0], g[:, 2])
        gyaw = np.interp(ego[idx, 0], g[:, 0], np.unwrap(g[:, 3]))
        e = np.hypot(ego[idx, 1]-gx, ego[idx, 2]-gy)
        ye = np.array([math.atan2(math.sin(a-b), math.cos(a-b))
                       for a, b in zip(ego[idx, 3], gyaw)])
        spd = np.hypot(np.gradient(g[:, 1], g[:, 0]), np.gradient(g[:, 2], g[:, 0]))
        print(f"  ego = {et}")
        print(f"  MCL 위치 p50 {100*np.median(e):.1f} cm  p90 {100*np.percentile(e,90):.1f} cm"
              f"   yaw MAE {math.degrees(np.abs(ye).mean()):.2f} deg")
        print(f"  주행속도 GT p50 {np.median(spd):.2f} / p95 {np.percentile(spd,95):.2f} m/s")
        out.update(ego_topic=et, gt_cover=cover, mcl_p50_cm=100*float(np.median(e)),
                   mcl_p90_cm=100*float(np.percentile(e, 90)),
                   yaw_mae_deg=math.degrees(float(np.abs(ye).mean())),
                   speed_p50=float(np.median(spd)))
    else:
        print("  mocap 없음 -> ego 정확도 평가 불가")
    json.dump(out, open(f"solo_{name}.json", "w"), indent=2)


if __name__ == "__main__":
    main()
