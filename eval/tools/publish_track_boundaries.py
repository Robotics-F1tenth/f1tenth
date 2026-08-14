#!/usr/bin/env python3
"""Publish /center_path, /left_boundary, /right_boundary from map1_centerline.csv
so perception's extractObj boundary filter activates without running control.

Lets us test whether the corridor filter removes the persistent central-wall
structure blobs (which velocity/persistence can't).

    python3 eval/tools/publish_track_boundaries.py --ros-args -p use_sim_time:=true

csv columns: x_m,y_m,w_tr_left,w_tr_right,w_total,left_x,left_y,right_x,right_y
"""
import argparse, csv
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data      # match perception's SensorDataQoS sub
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

CSV = Path(__file__).resolve().parents[2] / \
    "src/control/smppi_cuda_controller/data/map1/map1_centerline.csv"
FRAME = "map"


def path_from(points, stamp, frame):
    p = Path()
    p.header.frame_id = frame
    p.header.stamp = stamp
    for x, y in points:
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


class BoundaryPublisher(Node):
    def __init__(self):
        super().__init__("boundary_publisher")
        center, left, right = [], [], []
        with open(CSV) as f:
            r = csv.reader(f); next(r)
            for row in r:
                if not row:
                    continue
                c = [float(v) for v in row]
                center.append((c[0], c[1]))
                left.append((c[5], c[6]))
                right.append((c[7], c[8]))
        self._center, self._left, self._right = center, left, right
        q = qos_profile_sensor_data
        self.pub_c = self.create_publisher(Path, "/center_path", q)
        self.pub_l = self.create_publisher(Path, "/left_boundary", q)
        self.pub_r = self.create_publisher(Path, "/right_boundary", q)
        self.create_timer(0.2, self._tick)  # 5 Hz, volatile QoS so keep publishing
        self.get_logger().info(
            f"publishing boundaries: center={len(center)} left={len(left)} right={len(right)}")

    def _tick(self):
        now = self.get_clock().now().to_msg()
        self.pub_c.publish(path_from(self._center, now, FRAME))
        self.pub_l.publish(path_from(self._left, now, FRAME))
        self.pub_r.publish(path_from(self._right, now, FRAME))


def main():
    argparse.ArgumentParser().parse_known_args()
    rclpy.init()
    node = BoundaryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
