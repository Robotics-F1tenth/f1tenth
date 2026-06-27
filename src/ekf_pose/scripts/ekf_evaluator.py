#!/usr/bin/env python3
"""
EKF Pose Evaluator
------------------
/ekf_pose  (geometry_msgs/PoseStamped) vs
/mocap_pose1 (geometry_msgs/PoseStamped, GT)

메시지를 ApproximateTimeSynchronizer로 매칭하고
x, y, yaw 오차를 수집하여 RMSE / MAE / Max Error를 출력,
CSV 저장 및 matplotlib 플롯을 생성합니다.

실행 파라미터:
  ekf_topic      (default: /ekf_pose)
  gt_topic       (default: /mocap_pose1)
  output_dir     (default: /tmp/ekf_eval)
  sync_slop      (default: 0.05 [s])
"""

import os
import math
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import message_filters
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib
matplotlib.use("Agg")   # headless 환경에서도 동작
import matplotlib.pyplot as plt


def quat_to_yaw(q) -> float:
    """quaternion → yaw (rad)"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class EKFEvaluator(Node):
    def __init__(self):
        super().__init__("ekf_evaluator")

        self.declare_parameter("ekf_topic",   "/ekf_pose")
        self.declare_parameter("gt_topic",    "/mocap_pose1")
        self.declare_parameter("output_dir",  "/tmp/ekf_eval")
        self.declare_parameter("sync_slop",   0.05)

        ekf_topic  = self.get_parameter("ekf_topic").get_parameter_value().string_value
        gt_topic   = self.get_parameter("gt_topic").get_parameter_value().string_value
        self.out_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        slop       = self.get_parameter("sync_slop").get_parameter_value().double_value

        os.makedirs(self.out_dir, exist_ok=True)

        best_effort_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        ekf_sub = message_filters.Subscriber(self, PoseStamped, ekf_topic,  qos_profile=reliable_qos)
        gt_sub  = message_filters.Subscriber(self, PoseStamped, gt_topic,   qos_profile=reliable_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [ekf_sub, gt_sub], queue_size=100, slop=slop
        )
        self.sync.registerCallback(self.sync_callback)

        # 수집 버퍼
        self.timestamps: list[float] = []
        self.ex: list[float] = []   # x 오차
        self.ey: list[float] = []   # y 오차
        self.eyaw: list[float] = [] # yaw 오차
        self.pos_err: list[float] = []  # 2D Euclidean

        self.get_logger().info(
            f"[EKF Eval] 구독: EKF={ekf_topic}  GT={gt_topic}  slop={slop}s"
        )

    def sync_callback(self, ekf_msg: PoseStamped, gt_msg: PoseStamped):
        t = ekf_msg.header.stamp.sec + ekf_msg.header.stamp.nanosec * 1e-9

        dx   = ekf_msg.pose.position.x - gt_msg.pose.position.x
        dy   = ekf_msg.pose.position.y - gt_msg.pose.position.y
        dyaw = normalize_angle(
            quat_to_yaw(ekf_msg.pose.orientation) - quat_to_yaw(gt_msg.pose.orientation)
        )
        dist = math.hypot(dx, dy)

        self.timestamps.append(t)
        self.ex.append(dx)
        self.ey.append(dy)
        self.eyaw.append(dyaw)
        self.pos_err.append(dist)

    # ──────────────────────────────────────────────
    def print_stats(self):
        if not self.pos_err:
            self.get_logger().warn("[EKF Eval] 동기화된 메시지 없음 — 토픽 이름/slop 확인 필요")
            return

        n = len(self.pos_err)
        ex  = np.array(self.ex)
        ey  = np.array(self.ey)
        eyaw = np.array(self.eyaw)
        pe  = np.array(self.pos_err)

        sep = "=" * 55
        self.get_logger().info(sep)
        self.get_logger().info(f"  EKF 평가 결과  (N={n} pairs)")
        self.get_logger().info(sep)
        self.get_logger().info(f"  {'항목':<20} {'RMSE':>10} {'MAE':>10} {'Max':>10}")
        self.get_logger().info(f"  {'-'*50}")

        def fmt(arr):
            return f"{np.sqrt(np.mean(arr**2)):>10.4f} {np.mean(np.abs(arr)):>10.4f} {np.max(np.abs(arr)):>10.4f}"

        self.get_logger().info(f"  {'x  오차 [m]':<20} {fmt(ex)}")
        self.get_logger().info(f"  {'y  오차 [m]':<20} {fmt(ey)}")
        self.get_logger().info(f"  {'pos 2D 오차 [m]':<20} {fmt(pe)}")
        self.get_logger().info(f"  {'yaw 오차 [rad]':<20} {fmt(eyaw)}")
        self.get_logger().info(f"  {'yaw 오차 [deg]':<20} {fmt(np.degrees(eyaw))}")
        self.get_logger().info(sep)

        self._save_csv(n)
        self._save_plots()

    # ──────────────────────────────────────────────
    def _save_csv(self, n: int):
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.out_dir, f"ekf_eval_{ts_str}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_s", "ex_m", "ey_m", "pos_err_m", "eyaw_rad", "eyaw_deg"])
            t0 = self.timestamps[0]
            for i in range(n):
                w.writerow([
                    f"{self.timestamps[i] - t0:.4f}",
                    f"{self.ex[i]:.6f}",
                    f"{self.ey[i]:.6f}",
                    f"{self.pos_err[i]:.6f}",
                    f"{self.eyaw[i]:.6f}",
                    f"{math.degrees(self.eyaw[i]):.4f}",
                ])
        self.get_logger().info(f"[EKF Eval] CSV 저장: {path}")

    def _save_plots(self):
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        t = np.array(self.timestamps) - self.timestamps[0]
        pe  = np.array(self.pos_err)
        eyaw = np.degrees(np.array(self.eyaw))

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle("EKF Pose Error vs MoCap GT", fontsize=14)

        axes[0].plot(t, np.array(self.ex),  label="x error", color="tab:blue")
        axes[0].plot(t, np.array(self.ey),  label="y error", color="tab:orange")
        axes[0].axhline(0, color="k", linewidth=0.5)
        axes[0].set_ylabel("Error [m]")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(t, pe, color="tab:red", label="2D pos error")
        rmse_pos = float(np.sqrt(np.mean(pe**2)))
        axes[1].axhline(rmse_pos, color="tab:red", linestyle="--",
                        label=f"RMSE={rmse_pos:.4f}m")
        axes[1].set_ylabel("Pos Error [m]")
        axes[1].legend()
        axes[1].grid(True)

        axes[2].plot(t, eyaw, color="tab:green", label="yaw error")
        rmse_yaw = float(np.sqrt(np.mean(eyaw**2)))
        axes[2].axhline(0, color="k", linewidth=0.5)
        axes[2].axhline(rmse_yaw, color="tab:green", linestyle="--",
                        label=f"RMSE={rmse_yaw:.4f}°")
        axes[2].set_ylabel("Yaw Error [deg]")
        axes[2].set_xlabel("Time [s]")
        axes[2].legend()
        axes[2].grid(True)

        plt.tight_layout()
        plot_path = os.path.join(self.out_dir, f"ekf_eval_{ts_str}.png")
        plt.savefig(plot_path, dpi=150)
        plt.close()
        self.get_logger().info(f"[EKF Eval] 플롯 저장: {plot_path}")


def main(args=None):
    rclpy.init(args=args)
    node = EKFEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_stats()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
