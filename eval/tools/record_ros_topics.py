#!/usr/bin/env python3
"""Record tracker, localization, and mocap topics to evaluation CSV files.

Run while replaying the bag (with --clock) and the perception node running:
    ros2 bag play <bag> --clock
    <launch perception node>            # publishes /f1/perception/object/obstacles/arr
    python3 eval/tools/record_ros_topics.py --out /tmp/eval_run5 \
      --ros-args -p use_sim_time:=true

Writes  <out>_tracks.csv  <out>_gt_opp.csv  <out>_gt_ego.csv
Then:   python3 eval/tools/analyze_perception_tracking.py /tmp/eval_run5
"""
import argparse, csv, atexit
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from f1_msgs.msg import F1stateArr

TRACK_TOPIC = "/f1/perception/object/obstacles/arr"
GT_OPP_TOPIC = "/mocap_pose2"   # opponent
GT_EGO_TOPIC = "/mocap_pose1"   # ego (logged for later relative-error analysis)
MCL_TOPIC = "/newmcl_pose"      # live MCL estimate (localization eval; absent -> empty csv)


def yaw_from_quat(q):
    # ponytail: planar yaw only, no roll/pitch needed for a ground vehicle
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EvalLogger(Node):
    def __init__(self, out):
        super().__init__("eval_logger")
        self._files, self._w = {}, {}
        self._open("tracks", f"{out}_tracks.csv", ["t", "id", "x", "y", "yaw", "v"])
        self._open("gt_opp", f"{out}_gt_opp.csv", ["t", "x", "y", "yaw_body"])
        self._open("gt_ego", f"{out}_gt_ego.csv", ["t", "x", "y", "yaw_body"])
        self._open("mcl", f"{out}_mcl.csv", ["t", "x", "y", "yaw_body"])
        # mocap_pose may be published reliable; bump depth so we don't drop at ~290Hz
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(F1stateArr, TRACK_TOPIC, self._on_tracks, 50)
        self.create_subscription(PoseStamped, GT_OPP_TOPIC, lambda m: self._on_gt("gt_opp", m), qos)
        self.create_subscription(PoseStamped, GT_EGO_TOPIC, lambda m: self._on_gt("gt_ego", m), qos)
        self.create_subscription(PoseStamped, MCL_TOPIC, lambda m: self._on_gt("mcl", m), qos)
        self.get_logger().info(f"logging -> {out}_*.csv  (Ctrl-C to stop)")
        atexit.register(self._close)

    def _open(self, key, path, header):
        f = open(path, "w", newline="")
        w = csv.writer(f); w.writerow(header)
        self._files[key], self._w[key] = f, w

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp(self, hdr):
        s = hdr.stamp.sec + hdr.stamp.nanosec * 1e-9
        return s if s > 0.0 else self._now()  # F1stateArr leaves header empty -> sim clock

    def _on_tracks(self, msg):
        t = self._stamp(msg.header)            # empty -> falls back to sim clock
        for o in msg.f1_state_arr:
            self._w["tracks"].writerow([f"{t:.6f}", int(o.id), f"{o.x:.5f}",
                                        f"{o.y:.5f}", f"{o.yaw:.6f}", f"{o.v:.5f}"])
        self._files["tracks"].flush()

    def _on_gt(self, key, msg):
        t = self._stamp(msg.header)
        p = msg.pose.position
        self._w[key].writerow([f"{t:.6f}", f"{p.x:.5f}", f"{p.y:.5f}",
                               f"{yaw_from_quat(msg.pose.orientation):.6f}"])
        self._files[key].flush()

    def _close(self):
        for f in self._files.values():
            if not f.closed:
                f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/eval_run")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = EvalLogger(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
