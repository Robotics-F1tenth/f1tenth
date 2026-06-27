#!/usr/bin/env python3
"""
MCL localization evaluator.
Subscribes to MCL pose and ground truth, computes RMSE, saves to file.

Usage:
  ros2 run als_ros evaluate_localization.py \
    --ros-args \
    -p gt_topic:=/mocap_pose \
    -p duration:=30.0 \
    -p skip_seconds:=5.0 \
    -p result_file:=/tmp/mcl_eval_result.txt
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import time
import sys
import os


class LocalizationEvaluator(Node):
    def __init__(self):
        super().__init__('localization_evaluator')

        self.declare_parameter('gt_topic', '/mocap_pose')
        self.declare_parameter('mcl_topic', '/mcl_pose')
        self.declare_parameter('duration', 30.0)        # 평가 시간 (초)
        self.declare_parameter('skip_seconds', 5.0)     # 초기 수렴 대기 시간
        self.declare_parameter('result_file', '/tmp/mcl_eval_result.txt')

        gt_topic = self.get_parameter('gt_topic').value
        mcl_topic = self.get_parameter('mcl_topic').value
        self.duration = self.get_parameter('duration').value
        self.skip_seconds = self.get_parameter('skip_seconds').value
        self.result_file = self.get_parameter('result_file').value

        self.errors = []
        self.gt_pose = None
        self.mcl_pose = None
        self.start_time = None
        self.done = False

        # ground truth: PoseStamped
        self.gt_sub_pose = self.create_subscription(
            PoseStamped, gt_topic, self._gt_pose_cb, 10)

        self.mcl_sub = self.create_subscription(
            PoseStamped, mcl_topic, self._mcl_cb, 10)

        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'Evaluator started | GT: {gt_topic} | MCL: {mcl_topic} | '
            f'Duration: {self.duration}s (skip first {self.skip_seconds}s)')

    def _gt_pose_cb(self, msg: PoseStamped):
        self.gt_pose = (msg.pose.position.x, msg.pose.position.y)

    def _gt_odom_cb(self, msg: Odometry):
        self.gt_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _mcl_cb(self, msg: PoseStamped):
        self.mcl_pose = (msg.pose.position.x, msg.pose.position.y)
        if self.start_time is None:
            self.start_time = time.time()

    def _tick(self):
        if self.done:
            return

        if self.start_time is None:
            return

        elapsed = time.time() - self.start_time

        if elapsed > self.duration:
            self._save_result()
            self.done = True
            rclpy.shutdown()
            return

        # 초기 수렴 시간 건너뜀
        if elapsed < self.skip_seconds:
            return

        if self.gt_pose is not None and self.mcl_pose is not None:
            # MCL overflow/NaN 값 필터링 (맵 범위 밖이면 무효)
            if abs(self.mcl_pose[0]) > 1000 or abs(self.mcl_pose[1]) > 1000:
                return
            if not math.isfinite(self.mcl_pose[0]) or not math.isfinite(self.mcl_pose[1]):
                return
            dx = self.mcl_pose[0] - self.gt_pose[0]
            dy = self.mcl_pose[1] - self.gt_pose[1]
            err = math.sqrt(dx * dx + dy * dy)
            self.errors.append(err)

    def _save_result(self):
        if not self.errors:
            rmse = 9999.0
            mean_err = 9999.0
            max_err = 9999.0
        else:
            n = len(self.errors)
            rmse = math.sqrt(sum(e * e for e in self.errors) / n)
            mean_err = sum(self.errors) / n
            max_err = max(self.errors)

        result = f'rmse={rmse:.4f}\nmean={mean_err:.4f}\nmax={max_err:.4f}\nsamples={len(self.errors)}\n'
        with open(self.result_file, 'w') as f:
            f.write(result)

        self.get_logger().info(
            f'Evaluation done | RMSE={rmse:.4f}m | Mean={mean_err:.4f}m | '
            f'Max={max_err:.4f}m | Samples={len(self.errors)}')


def main():
    rclpy.init()
    node = LocalizationEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
