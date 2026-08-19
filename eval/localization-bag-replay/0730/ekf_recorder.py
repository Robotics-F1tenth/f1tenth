#!/usr/bin/env python3
"""Record /ekf_pose in the same t,x,y,yaw_body CSV shape analyze_localization.py wants."""
import csv, math, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self, out):
        super().__init__("ekf_recorder")
        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "x", "y", "yaw_body"])
        self.n = 0
        self.create_subscription(PoseStamped, "/ekf_pose", self.cb, 50)
        self.create_timer(5.0, lambda: (self.get_logger().info(f"ekf rows={self.n}"), self.f.flush()))

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        self.w.writerow([f"{t:.6f}", f"{p.x:.6f}", f"{p.y:.6f}", f"{yaw_of(msg.pose.orientation):.6f}"])
        self.n += 1

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    n = Rec(sys.argv[1])
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.get_logger().info(f"final ekf rows={n.n}")
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
