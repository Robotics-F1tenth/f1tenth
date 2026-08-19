#!/usr/bin/env python3
"""One long-lived subscriber per topic, reporting counts per 10s bucket.

Repeatedly spawning `ros2 topic echo`/`hz` was giving false zeros (even /scan,
which perception doesn't touch, read 0), so this keeps a single node alive for
the whole run instead -- no repeated DDS participant churn.

    python3 rate_monitor.py <seconds>
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
from f1_msgs.msg import F1stateArr


class RateMonitor(Node):
    def __init__(self):
        super().__init__("rate_monitor")
        self.counts = {}
        self.marker_lens = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(MarkerArray, "/static_obstacle_markers",
                                 lambda m: self._hit("static_markers", len(m.markers)), qos)
        self.create_subscription(MarkerArray, "/bounding_box_markers",
                                 lambda m: self._hit("box_markers", len(m.markers)), qos)
        self.create_subscription(F1stateArr, "/f1/perception/object/obstacles/arr",
                                 lambda m: self._hit("obstacles_arr", len(m.f1_state_arr)), qos)
        self.create_subscription(LaserScan, "/scan",
                                 lambda m: self._hit("scan", 0), best)

    def _hit(self, key, n):
        self.counts[key] = self.counts.get(key, 0) + 1
        if n:
            self.marker_lens[key] = n


def main():
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 240
    rclpy.init()
    node = RateMonitor()
    t0 = time.time()
    last = dict()
    print("elapsed_s " + " ".join(
        f"{k}_hz" for k in ["scan", "static_markers", "box_markers", "obstacles_arr"])
        + " static_len box_len", flush=True)
    next_report = t0 + 10
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now >= next_report:
            row = [f"{now - t0:6.0f}"]
            for k in ["scan", "static_markers", "box_markers", "obstacles_arr"]:
                c = node.counts.get(k, 0)
                row.append(f"{(c - last.get(k, 0)) / 10.0:7.1f}")
                last[k] = c
            row.append(f"{node.marker_lens.get('static_markers', 0):5d}")
            row.append(f"{node.marker_lens.get('box_markers', 0):5d}")
            print(" ".join(row), flush=True)
            next_report += 10
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
