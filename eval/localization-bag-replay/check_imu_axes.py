#!/usr/bin/env python3
"""Check whether /imu/data and /ekf/imu/data agree on axis convention.

They should. On car4 (2026-08-21) they did not: the driver applies its
imu_data_roll180 correction to /imu/data only, leaving the device filter's
/ekf/imu/data in NED (y and z still flipped). ekf_pose read the filter topic and
its yaw wandered +-60 degrees.

Needs no ground truth -- it compares the two topics against each other, so it
runs on the car. Rotate the vehicle (by hand is enough) while it samples.

    ros2 run ... or just: python3 check_imu_axes.py [seconds]
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

RAW, FILT = "/imu/data", "/ekf/imu/data"


class Check(Node):
    def __init__(self, secs):
        super().__init__("check_imu_axes")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.buf = {RAW: [], FILT: []}
        for t in (RAW, FILT):
            self.create_subscription(Imu, t, (lambda k: lambda m: self.cb(k, m))(t), qos)
        self.secs = secs
        self.done = False
        self.create_timer(secs, self.report)
        self.get_logger().info(f"{secs:.0f}초간 수집 -- 차를 좌우로 돌려 주세요")

    def cb(self, topic, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.buf[topic].append((t, msg.angular_velocity.z, msg.linear_acceleration.y,
                                msg.linear_acceleration.z))

    def report(self):
        raw, filt = self.buf[RAW], self.buf[FILT]
        if len(raw) < 50 or len(filt) < 50:
            self.get_logger().error(f"샘플 부족: {RAW}={len(raw)} {FILT}={len(filt)}")
            self.done = True
            return
        # nearest-time pairing; both come from the same device so stamps line up
        ft = [f[0] for f in filt]
        import bisect
        pairs = []
        for t, wz, ay, az in raw:
            i = bisect.bisect_left(ft, t)
            best = None
            for j in (i - 1, i, i + 1):
                if 0 <= j < len(filt):
                    d = abs(filt[j][0] - t)
                    if best is None or d < best[0]:
                        best = (d, filt[j])
            if best and best[0] < 0.02:
                pairs.append(((wz, ay, az), best[1][1:]))
        if len(pairs) < 30:
            self.get_logger().error(f"시간 정합 실패 (pairs={len(pairs)})")
            self.done = True
            return
        print(f"\n  쌍 {len(pairs)}개\n", flush=True)
        # 가속도 z 는 판정에서 제외한다: 필터 토픽은 중력보상돼 정지 시 0 근처인데
        # raw 토픽은 +9.8 이라, 축이 맞든 틀리든 기울기가 의미를 갖지 않는다.
        # 축 규약은 자이로 z 와 가속도 y 로 판정하고, z 는 참고로만 찍는다.
        ok = True
        for idx, name in enumerate(["자이로 z", "가속도 y"]):
            xs = [(a[idx], b[idx]) for a, b in pairs if abs(a[idx]) > 0.3]
            if len(xs) < 20:
                print(f"  {name:8s}: 변화 부족 -- 더 많이 움직여 주세요", flush=True)
                ok = False
                continue
            sxx = sum(a * a for a, _ in xs)
            slope = sum(a * b for a, b in xs) / sxx if sxx else 0.0
            verdict = "일치" if slope > 0.7 else ("반전!" if slope < -0.5 else "이상")
            if slope <= 0.7:
                ok = False
            print(f"  {name:8s}: {FILT} / {RAW} 기울기 {slope:+.3f}   → {verdict}", flush=True)
        raw_az = sum(a[2] for a, _ in pairs) / len(pairs)
        filt_az = sum(b[2] for _, b in pairs) / len(pairs)
        print(f"  (참고) 평균 a_z: {RAW} {raw_az:+.2f}, {FILT} {filt_az:+.2f}"
              f"  -- {FILT} 가 0 근처면 중력보상된 것", flush=True)
        print(f"\n  ==> {'통과 -- 두 토픽 축 규약 동일' if ok else '실패 -- 드라이버가 한쪽에만 보정을 적용하고 있음'}\n", flush=True)
        self.done = True


def main():
    rclpy.init()
    n = Check(float(sys.argv[1]) if len(sys.argv) > 1 else 20.0)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
