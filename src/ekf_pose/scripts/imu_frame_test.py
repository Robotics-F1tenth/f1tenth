#!/usr/bin/env python3
"""IMU 축 규약 검사 — 차 위에서 바로 돌리고 판정을 받는다. mocap 불필요.

    ros2 run ekf_pose imu_frame_test
    ros2 run ekf_pose imu_frame_test --ros-args -p imu_topic:=/ekf/imu/data

왜 필요한가
-----------
이 차량들의 /imu/data 는 가속도 x,y 가 차체 프레임 대비 뒤집혀 나온 이력이 있다
(2026-08-03 0730 bag + mocap GT 로 확인, 회귀 기울기 -1.1~-1.3).
`ekf_pose` 는 `imu_axis_yaw180` 로 소프트웨어 보정 중이고, `predict` 의 Coriolis
부호와 **반드시 세트**다 — 한쪽만 고치면 v_y 적분 오차가 8~10배로 튄다.
그래서 하드웨어/드라이버 설정을 만졌으면 이 스크립트로 재확인해야 한다.
배경: src/ekf_pose/docs/ekf-pose-analysis-2026-08.md (§2, §3)

손으로 기울이는 정지 테스트로는 판정할 수 없다
------------------------------------------------
정지 상태에서 기울이면 중력축(z)만 검증된다. 센서가 yaw 로 180도 돌아 있어도
기울임에 대한 반응은 똑같이 그럴듯하게 나온다 — "앞"이 어느 쪽인지는 실제로
움직여봐야(외부 기준이 있어야) 구분된다. 그래서 이 스크립트는 **주행 데이터**를 쓴다.

기준 신호는 /odom (휠 속도 + 조향 기반)이다. 크기는 부정확할 수 있어도
(코너에서 실제의 0.7~0.8배, 115ms 지연 — 같은 문서 §6) **부호는 신뢰할 수 있어서**
축 반전 판정에는 충분하다. mocap 이 없는 차(4호차 등)에서도 그대로 쓸 수 있다.

검사 순서 (뒤 단계가 앞 단계 결과에 의존하므로 순서가 중요)
  1) 자이로 z : /odom angular.z 와 부호 비교          -> 회전 부호 확정
  2) 횡 a_y   : 정상선회 항등식 a_y = v_x * omega     -> 1)에서 확정한 자이로 사용
  3) 종 a_x   : d(v_odom)/dt 와 비교
각 단계의 회귀 기울기가 +1 근처면 정상, -1 근처면 그 축이 반전된 것이다.

주행 방법: 8자 또는 좌우 코너를 섞어 30초 이상. 한쪽으로만 돌면 판정이 약해진다.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# 판정 기준
MIN_SAMPLES = 300        # 이보다 적으면 판정 보류
MIN_ABS_CORR = 0.30      # 상관이 이보다 낮으면 신호 부족 -> 더 주행 필요
MIN_SPEED = 0.5          # m/s, 정지 구간 제외
MIN_TURN_RATE = 0.15     # rad/s, a_y 검사는 실제로 돌 때만 의미 있음


def regress(x, y):
    """y = s*x + c 의 기울기 s 와 상관계수. 표본 부족/퇴화 시 (nan, nan)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if len(x) < 10 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
        return float("nan"), float("nan"), len(x)
    s = float(np.polyfit(x, y, 1)[0])
    c = float(np.corrcoef(x, y)[0, 1])
    return s, c, len(x)


def verdict(slope, corr, n, what):
    """기울기/상관/표본수 -> 사람이 읽는 판정."""
    if n < MIN_SAMPLES or not math.isfinite(slope):
        return f"  {what:<22} 판정보류 (표본 {n}개, 더 주행 필요)"
    if abs(corr) < MIN_ABS_CORR:
        return (f"  {what:<22} 판정보류 (상관 {corr:+.2f} 로 약함 — 코너를 더 "
                f"섞어서 재측정)  기울기 {slope:+.2f}, n={n}")
    if slope > 0:
        tag = "정상"
        if not (0.5 <= slope <= 2.0):
            tag = "정상(부호는 맞음, 크기는 어긋남)"
        return f"  {what:<22} {tag}   기울기 {slope:+.2f}  상관 {corr:+.2f}  n={n}"
    return (f"  {what:<22} ★반전★  기울기 {slope:+.2f}  상관 {corr:+.2f}  n={n}")


class ImuFrameTest(Node):
    def __init__(self):
        super().__init__("imu_frame_test")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("duration", 40.0)
        self.declare_parameter("gravity_compensated", False)
        self.imu_topic = self.get_parameter("imu_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.duration = float(self.get_parameter("duration").value)
        self.gravity_compensated = bool(
            self.get_parameter("gravity_compensated").value)

        # 드라이버가 RELIABLE 로 발행해도 BEST_EFFORT 구독은 호환된다.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        self.imu, self.odom = [], []
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"IMU: {self.imu_topic} | odom: {self.odom_topic} | {self.duration:.0f}초 수집\n"
            f"  >>> 지금부터 8자 또는 좌우 코너를 섞어서 주행하세요. "
            f"한쪽으로만 돌면 판정이 약해집니다. <<<")

    def _on_imu(self, m):
        t = rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.imu.append((t, m.linear_acceleration.x, m.linear_acceleration.y,
                         m.linear_acceleration.z, m.angular_velocity.z))

    def _on_odom(self, m):
        t = rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.odom.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))

    def _tick(self):
        el = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        if el < self.duration:
            if int(el) % 5 == 0:
                self.get_logger().info(
                    f"  수집중 {el:.0f}/{self.duration:.0f}s  "
                    f"imu={len(self.imu)} odom={len(self.odom)}")
            return
        self.report()
        rclpy.shutdown()

    def report(self):
        if len(self.imu) < 50 or len(self.odom) < 50:
            print(f"\n데이터 부족 (imu={len(self.imu)}, odom={len(self.odom)}). "
                  f"토픽 이름과 드라이버 상태를 확인하세요.")
            return

        imu_data = np.array(self.imu)       # t, ax, ay, az, wz
        odom_data = np.array(self.odom)     # t, vx, wz
        imu_data = imu_data[imu_data[:, 0].argsort()]
        odom_data = odom_data[odom_data[:, 0].argsort()]

        # odom 을 IMU 시각으로 보간
        v = np.interp(imu_data[:, 0], odom_data[:, 0], odom_data[:, 1])
        w_odom = np.interp(imu_data[:, 0], odom_data[:, 0], odom_data[:, 2])
        t, ax, ay, az, wz = (imu_data[:, 0], imu_data[:, 1], imu_data[:, 2],
                             imu_data[:, 3], imu_data[:, 4])

        moving = np.abs(v) > MIN_SPEED
        print("\n" + "=" * 72)
        print(f"IMU 축 규약 검사 결과   ({self.imu_topic})")
        print("=" * 72)
        az_rest = np.median(az[~moving]) if (~moving).sum() > 20 else float("nan")
        if self.gravity_compensated:
            az_hint = "중력보상 토픽: 정지 시 0 근처가 정상"
        else:
            az_hint = "+9.8 = z축 정상 / -9.8 = 상하 뒤집힘"
        print(f"  정지 시 a_z 중앙값 = {az_rest:+.2f} m/s^2   ({az_hint})")
        print(f"  주행 표본 {int(moving.sum())} / 전체 {len(imu_data)}\n")

        # 1) 자이로 부호 — /odom 의 각속도(조향 기반)와 비교. 크기는 안 믿고 부호만 본다.
        s1, c1, n1 = regress(w_odom[moving], wz[moving])
        print(verdict(s1, c1, n1, "자이로 z"))

        gyro_known = (math.isfinite(s1) and abs(c1) >= MIN_ABS_CORR
                      and n1 >= MIN_SAMPLES)

        # 2) 횡가속 — 정상선회 항등식 a_y = v_x * omega.
        # 자이로가 반전돼 있으면 먼저 부호를 바로잡아야, 자이로와 a_y가 동시에
        # 반전된 경우에도 두 오류가 상쇄되지 않고 a_y 반전을 검출할 수 있다.
        turning = moving & (np.abs(wz) > MIN_TURN_RATE)
        wz_vehicle = wz if not gyro_known or s1 > 0 else -wz
        s2, c2, n2 = regress((v * wz_vehicle)[turning], ay[turning])
        if gyro_known:
            print(verdict(s2, c2, n2, "횡가속 a_y"))
        else:
            print(f"  {'횡가속 a_y':<22} 판정보류 — 자이로 부호가 먼저 확정돼야 함 "
                  f"(참고 기울기 {s2:+.2f}, n={n2})")

        # 3) 종가속 — 휠속도 미분과 비교.
        dv = np.gradient(v, t)
        s3, c3, n3 = regress(dv[moving], ax[moving])
        print(verdict(s3, c3, n3, "종가속 a_x"))

        # 종합
        print("\n" + "-" * 72)

        def decisive(slope, corr, count):
            return (math.isfinite(slope) and abs(corr) >= MIN_ABS_CORR
                    and count >= MIN_SAMPLES)
        all_decisive = (decisive(s1, c1, n1) and decisive(s2, c2, n2)
                        and decisive(s3, c3, n3))
        inverted = [nm for nm, s, c, n in
                    (("자이로 z", s1, c1, n1), ("a_y", s2, c2, n2), ("a_x", s3, c3, n3))
                    if decisive(s, c, n) and s < 0]
        if not all_decisive:
            print("  판정: 판정보류 — 주행 표본 또는 신호 변화가 부족합니다.")
            print("  -> 이 결과로 IMU/소프트웨어 보정값을 변경하면 안 됩니다.")
        elif not inverted:
            print("  판정: 반전된 축 없음.")
            print("  -> ekf_pose 의 imu_axis_yaw180 을 false 로 내려야 합니다.")
            print("     ⚠ 단, Coriolis 부호와 세트입니다. 문서 §3 을 먼저 읽으세요.")
        elif set(inverted) == {"a_y", "a_x"}:
            print("  판정: a_x, a_y 둘 다 반전 / 자이로 정상  =  센서가 차체 대비 yaw 180도")
            print("  -> ekf_pose 의 imu_axis_yaw180: true 가 맞습니다 (현재 기본값).")
        else:
            print(f"  판정: 반전 축 = {', '.join(inverted)}")
            print("  -> 단순 yaw180 이 아닙니다. 드라이버의 sensor2vehicle 설정을 확인하세요:")
            print("     ros2 param get /microstrain_inertial_driver "
                  "filter_sensor2vehicle_frame_transformation_euler")
            print("     (identity 가 아니면 그 값이 원인일 수 있음. selector 도 같이 확인)")
        print("-" * 72)
        print("  참고: 이 검사는 /odom 을 기준으로 삼습니다. 조향 캘리브레이션이")
        print("        심하게 틀어져 있으면 기준 자체가 흔들릴 수 있습니다.")


def main():
    rclpy.init(args=sys.argv[1:])
    node = ImuFrameTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
