#!/usr/bin/env python3
"""Publish perception health as scalar topics so Foxglove can plot it live.

Perception's outputs are arrays and marker arrays -- there is nothing a Plot
panel can draw, so problems are only visible by staring at the 3D view or by
reading the 5-second log after the fact. This turns the same information into
scalars, on the same pattern control already uses for its own supervision.

    python3 perception_monitor.py --ros-args -p use_sim_time:=true

Works with no ground truth: everything here comes from perception's own output
plus the ego pose. Pair with opponent-benchmark-0821/live_gt_error.py when a
mocap bag is available and you want error against GT as well.

Published, all std_msgs/Float32 under /pmon:

  count/tracks           tracks in the per-cycle obstacle list
  count/landmarks        confirmed static landmarks
  count/tracks_to_control min(tracks, MAX_OBS) -- what control can actually take
  count/dropped_by_cap   tracks beyond MAX_OBS, silently discarded
  rate/arr_hz            obstacle-array publish rate
  rate/scan_hz           lidar rate, to tell perception stalls from sensor stalls
  age/arr_ms             time since the last obstacle array
  nearest/distance       ego to nearest reported obstacle
  nearest/bearing_deg    its bearing, |b|>90 means behind the lidar
  speed/max              fastest track in the list
  speed/n_above_static   tracks control would read as dynamic (|v| >= 0.5)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

MAX_OBS = 5          # cuda_mppi_core.hpp -- control takes at most this many
STATIC_TH = 0.5      # control's static/dynamic split


class PerceptionMonitor(Node):
    def __init__(self):
        super().__init__("perception_monitor")
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        # RELIABLE out: Foxglove's Plot panel drops BEST_EFFORT samples under
        # load and the curve goes ragged.
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=50)
        names = ["count/tracks", "count/landmarks", "count/tracks_to_control",
                 "count/dropped_by_cap", "rate/arr_hz", "rate/scan_hz",
                 "age/arr_ms", "nearest/distance", "nearest/bearing_deg",
                 "speed/max", "speed/n_above_static"]
        self.pub = {n: self.create_publisher(Float32, f"/pmon/{n}", pub_qos)
                    for n in names}
        self.ego = None
        self.arr_t, self.scan_t = [], []
        self.last_arr = None
        self.create_subscription(F1stateArr,
                                 "/f1/perception/object/obstacles/arr",
                                 self.on_arr, sub_qos)
        self.create_subscription(F1stateArr,
                                 "/f1/perception/static_obstacles",
                                 self.on_lm, sub_qos)
        self.create_subscription(PoseStamped, "/newmcl_pose", self.on_ego, sub_qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, sub_qos)
        # staleness has to be reported even when nothing arrives, so it cannot
        # be computed inside the array callback
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info("perception_monitor: publishing /pmon/*")

    def emit(self, n, v):
        m = Float32(); m.data = float(v); self.pub[n].publish(m)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _hz(stamps, now, window=2.0):
        while stamps and now - stamps[0] > window:
            stamps.pop(0)
        return len(stamps) / window if len(stamps) > 1 else 0.0

    def on_ego(self, msg):
        q = msg.pose.orientation
        self.ego = (msg.pose.position.x, msg.pose.position.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def on_scan(self, msg):
        self.scan_t.append(self.now_s())

    def on_lm(self, msg):
        self.emit("count/landmarks", len(msg.f1_state_arr))

    def on_arr(self, msg):
        t = self.now_s()
        self.arr_t.append(t)
        self.last_arr = t
        n = len(msg.f1_state_arr)
        self.emit("count/tracks", n)
        self.emit("count/tracks_to_control", min(n, MAX_OBS))
        self.emit("count/dropped_by_cap", max(0, n - MAX_OBS))
        speeds = [abs(s.v) for s in msg.f1_state_arr]
        self.emit("speed/max", max(speeds) if speeds else 0.0)
        self.emit("speed/n_above_static", sum(1 for v in speeds if v >= STATIC_TH))
        if self.ego and n:
            ex, ey, eyaw = self.ego
            best, bd = None, 1e9
            for s in msg.f1_state_arr:
                d = math.hypot(s.x-ex, s.y-ey)
                if d < bd:
                    bd, best = d, s
            b = math.atan2(best.y-ey, best.x-ex) - eyaw
            b = math.atan2(math.sin(b), math.cos(b))
            self.emit("nearest/distance", bd)
            self.emit("nearest/bearing_deg", math.degrees(b))

    def on_timer(self):
        now = self.now_s()
        self.emit("rate/arr_hz", self._hz(self.arr_t, now))
        self.emit("rate/scan_hz", self._hz(self.scan_t, now))
        self.emit("age/arr_ms",
                  1000.0*(now-self.last_arr) if self.last_arr else -1.0)


def main():
    rclpy.init()
    n = PerceptionMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
