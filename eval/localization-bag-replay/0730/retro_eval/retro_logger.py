#!/usr/bin/env python3
"""ekf_pose retrodiction A/B: /ekf_pose vs mocap GT(/mocap_pose1) 로깅."""
import argparse, csv, atexit, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

class RetroLogger(Node):
    def __init__(self, out):
        super().__init__("retro_logger")
        self._f = {}; self._w = {}
        self._open("mcl", f"{out}_mcl.csv")
        self._open("gt", f"{out}_gt_ego.csv")
        qos = QoSProfile(depth=400, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, "/ekf_pose", lambda m: self._on("mcl", m), 200)
        self.create_subscription(PoseStamped, "/mocap_pose1", lambda m: self._on("gt", m), qos)
        atexit.register(self._close)

    def _open(self, k, path):
        f = open(path, "w", newline=""); w = csv.writer(f); w.writerow(["t","x","y","yaw"])
        self._f[k], self._w[k] = f, w

    def _on(self, k, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        p = msg.pose.position
        self._w[k].writerow([f"{t:.6f}", f"{p.x:.5f}", f"{p.y:.5f}", f"{yaw_from_quat(msg.pose.orientation):.6f}"])
        self._f[k].flush()

    def _close(self):
        for f in self._f.values():
            if not f.closed: f.close()

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default="/tmp/retro")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = RetroLogger(a.out)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node._close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
