#!/usr/bin/env python3
"""Publish opponent tracking error vs mocap GT as live topics, for Foxglove.

Run this next to `ros2 bag play` and Foxglove's Plot panel draws the same curves
as plot_tracking.py, live, without a post-processing step.

    ros2 bag play <bag> --clock          # terminal 1
    python3 live_gt_error.py             # terminal 2, --ros-args -p use_sim_time:=true

Published (all std_msgs/Float32, so any Foxglove Plot panel takes them):

    /bench/gt/speed          GT speed of the opponent          [m/s]
    /bench/est/speed         tracker estimate                  [m/s]
    /bench/err/speed         estimate - GT                     [m/s]
    /bench/gt/yaw            GT course                         [deg]
    /bench/est/yaw           tracker yaw state                 [deg]
    /bench/err/yaw           wrapped |error|                   [deg]
    /bench/err/position      distance to GT                    [m]
    /bench/track/age         frames since this id appeared
    /bench/track/id          current matched track id
    /bench/range             ego -> opponent distance          [m]

Nothing is published on cycles where the opponent is not matched, so gaps in the
Foxglove plot are exactly the detection gaps.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

GATE = 1.0
OPP_TOPIC = "/mocap/car3/pose"
EGO_TOPIC = "/newmcl_pose"


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LiveBench(Node):
    def __init__(self):
        super().__init__("live_gt_error")
        self.declare_parameter("opponent_topic", OPP_TOPIC)
        self.declare_parameter("ego_topic", EGO_TOPIC)
        self.declare_parameter("gate", GATE)
        self.gate = self.get_parameter("gate").value
        # Subscribe BEST_EFFORT so live sensor publishers match, but `ros2 bag play`
        # republishes RELIABLE by default and a BEST_EFFORT subscriber does match a
        # RELIABLE publisher, so one profile covers both. Publish RELIABLE: Foxglove's
        # Plot panel drops BEST_EFFORT samples under load and the plot goes ragged.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=50)
        names = ["gt/speed", "est/speed", "err/speed", "gt/yaw", "est/yaw",
                 "err/yaw", "err/position", "track/age", "track/id", "range"]
        self.pub = {n: self.create_publisher(Float32, f"/bench/{n}", pub_qos)
                    for n in names}
        self.opp = None          # (t, x, y)
        self.opp_prev = None
        self.gt_speed = 0.0
        self.gt_course = 0.0
        self.ego = None
        self.prev_id, self.age = None, 0
        self.create_subscription(
            PoseStamped, self.get_parameter("opponent_topic").value,
            self.on_opp, qos)
        self.create_subscription(
            PoseStamped, self.get_parameter("ego_topic").value, self.on_ego, qos)
        self.create_subscription(
            F1stateArr, "/f1/perception/object/obstacles/arr", self.on_arr, qos)
        self.get_logger().info(
            f"live bench: opponent={self.get_parameter('opponent_topic').value} "
            f"gate={self.gate} m -- publishing /bench/*")

    def emit(self, name, v):
        m = Float32(); m.data = float(v); self.pub[name].publish(m)

    def on_ego(self, msg):
        self.ego = (msg.pose.position.x, msg.pose.position.y)

    def on_opp(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        x, y = msg.pose.position.x, msg.pose.position.y
        if self.opp is not None:
            dt = t - self.opp[0]
            if dt > 1e-3:
                dx, dy = x-self.opp[1], y-self.opp[2]
                sp = math.hypot(dx, dy)/dt
                # mocap at ~70 Hz is noisy frame to frame; light EMA keeps the
                # reference readable without hiding real acceleration
                self.gt_speed = 0.8*self.gt_speed + 0.2*sp
                if sp > 0.15:
                    self.gt_course = math.atan2(dy, dx)
        self.opp = (t, x, y)

    def on_arr(self, msg):
        if self.opp is None or not msg.f1_state_arr:
            return
        _, ox, oy = self.opp
        best, bd = None, 1e9
        for s in msg.f1_state_arr:
            d = math.hypot(s.x-ox, s.y-oy)
            if d < bd:
                bd, best = d, s
        if best is None or bd > self.gate:
            return
        self.age = 1 if best.id != self.prev_id else self.age + 1
        self.prev_id = best.id
        ev = abs(best.v)
        self.emit("gt/speed", self.gt_speed)
        self.emit("est/speed", ev)
        self.emit("err/speed", ev - self.gt_speed)
        self.emit("gt/yaw", math.degrees(self.gt_course))
        self.emit("est/yaw", math.degrees(best.yaw))
        self.emit("err/yaw", abs(math.degrees(wrap(best.yaw - self.gt_course))))
        self.emit("err/position", bd)
        self.emit("track/age", self.age)
        self.emit("track/id", best.id)
        if self.ego:
            self.emit("range", math.hypot(ox-self.ego[0], oy-self.ego[1]))


def main():
    rclpy.init()
    n = LiveBench()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
