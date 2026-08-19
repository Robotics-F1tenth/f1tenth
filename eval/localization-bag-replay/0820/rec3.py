#!/usr/bin/env python3
"""Record MCL / EKF / GT pose topics to CSV. Topic names are args, because the
0820 car4 bag namespaces everything under /car4 and puts GT on /mocap_cal/car4/base.

usage: rec3.py <out_prefix> <mcl_topic> <gt_topic> [ekf_topic]
"""
import csv, math, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self, prefix, mcl_t, gt_t, ekf_t):
        super().__init__("rec3")
        self.files, self.w, self.n = {}, {}, {}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        for key, topic in (("mcl", mcl_t), ("gt_ego", gt_t), ("ekf", ekf_t)):
            if not topic:
                continue
            f = open(f"{prefix}_{key}.csv", "w", newline="")
            wr = csv.writer(f)
            wr.writerow(["t", "x", "y", "yaw_body"])
            self.files[key], self.w[key], self.n[key] = f, wr, 0
            self.create_subscription(
                PoseStamped, topic,
                (lambda k: (lambda m: self.cb(k, m)))(key), qos)
        self.create_timer(5.0, self.tick)

    def cb(self, key, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        self.w[key].writerow([f"{t:.6f}", f"{p.x:.6f}", f"{p.y:.6f}",
                              f"{yaw_of(msg.pose.orientation):.6f}"])
        self.n[key] += 1

    def tick(self):
        self.get_logger().info(" ".join(f"{k}={v}" for k, v in self.n.items()))
        for f in self.files.values():
            f.flush()

    def destroy_node(self):
        for f in self.files.values():
            f.close()
        super().destroy_node()


def main():
    prefix, mcl_t, gt_t = sys.argv[1], sys.argv[2], sys.argv[3]
    ekf_t = sys.argv[4] if len(sys.argv) > 4 else None
    rclpy.init()
    n = Rec(prefix, mcl_t, gt_t, ekf_t)
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.get_logger().info("final " + " ".join(f"{k}={v}" for k, v in n.n.items()))
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
