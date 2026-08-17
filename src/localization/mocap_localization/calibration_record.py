#!/usr/bin/env python3
"""캘리브레이션 표본을 주행 궤적 전체에서 기록한다 (차량 1대 = 파일 1개).

    ros2 run mocap_localization calibration_record --ros-args \
        -p mocap_topic:=/mocap_pose3 -p out:=/tmp/car3_samples.csv

왜 손으로 안 찍고 전 궤적을 쓰나
--------------------------------
calibration_v2.py 는 스페이스바로 표본을 찍고 최소 3개면 적합한다. 0730 bag 으로
같은 데이터를 전 궤적(2247 표본)으로 재적합해보면 잔차 p50 이 10.84 -> 7.05 cm 로
줄었다. 이 bag 의 MCL 자체 오차가 p50 7.6 cm 이므로 7.05 cm 는 사실상 노이즈 바닥
이고, 손으로 찍은 소수 표본이 그 위에 ~8 cm 의 캘리 오차를 얹고 있었다는 뜻이다.
상세: docs/mocap-calibration-analysis-2026-08.md §4

사용 순서
  1) 각 차량이 ego(MCL 구동) 역할일 때 한 번씩 이 노드로 표본을 기록한다.
  2) calibration_solve_joint.py 로 두 파일을 한꺼번에 푼다.
     T_map_mocap 은 방+맵의 성질이라 두 차가 공유해야 하고, 차량별로 따로 풀면
     서로 다른 맵 프레임에 살게 된다(실측 13 cm 어긋남). §2 참고.
"""
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CalibrationRecorder(Node):
    def __init__(self):
        super().__init__("calibration_record")
        self.declare_parameter("mcl_topic", "/newmcl_pose")
        self.declare_parameter("mocap_topic", "/mocap_pose3")
        self.declare_parameter("out", "/tmp/calib_samples.csv")
        # 두 토픽의 stamp 차이가 이보다 크면 짝으로 안 본다. MCL 은 scan 주기(~26 ms)
        # 로, mocap 은 수백 Hz 로 오므로 한쪽 주기의 절반 정도면 충분하다.
        self.declare_parameter("max_pair_dt", 0.015)

        self.mcl_topic = self.get_parameter("mcl_topic").value
        self.mocap_topic = self.get_parameter("mocap_topic").value
        self.max_pair_dt = float(self.get_parameter("max_pair_dt").value)
        out = self.get_parameter("out").value

        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_mcl", "x_mcl", "y_mcl", "yaw_mcl",
                         "t_moc", "x_moc", "y_moc", "yaw_moc"])
        self.n = 0
        self.latest_mocap = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        self.create_subscription(PoseStamped, self.mocap_topic, self._on_mocap, qos)
        self.create_subscription(PoseStamped, self.mcl_topic, self._on_mcl, 50)
        self.create_timer(5.0, self._tick)
        self.get_logger().info(
            f"기록 시작: mcl={self.mcl_topic} mocap={self.mocap_topic} -> {out}\n"
            f"  >>> 트랙을 한 바퀴 이상, 코너와 직선을 모두 지나도록 주행하세요. <<<\n"
            f"  (T_map_mocap 의 회전은 위치가 넓게 퍼져야 잘 관측됩니다)")

    def _on_mocap(self, msg):
        self.latest_mocap = msg

    def _on_mcl(self, msg):
        if self.latest_mocap is None:
            return
        t_mcl = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        t_moc = rclpy.time.Time.from_msg(
            self.latest_mocap.header.stamp).nanoseconds * 1e-9
        # mocap stamp 가 0 이면(발행자가 안 채운 경우) 시간 검사를 건너뛴다.
        if t_moc > 0.0 and abs(t_mcl - t_moc) > self.max_pair_dt:
            return
        p, m = msg.pose, self.latest_mocap.pose
        self.w.writerow([
            f"{t_mcl:.6f}", f"{p.position.x:.6f}", f"{p.position.y:.6f}",
            f"{quat_to_yaw(p.orientation):.6f}",
            f"{t_moc:.6f}", f"{m.position.x:.6f}", f"{m.position.y:.6f}",
            f"{quat_to_yaw(m.orientation):.6f}"])
        self.n += 1

    def _tick(self):
        self.get_logger().info(f"  표본 {self.n}개")
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CalibrationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"총 {node.n}개 표본 기록")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
