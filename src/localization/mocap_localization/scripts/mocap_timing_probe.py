#!/usr/bin/env python3
"""
Low-overhead live timing probe for mocap4r2_msgs/RigidBodies.

The upstream OptiTrack driver reconstructs header.stamp on the ROS host from
CameraMidExposureTimestamp. Its frame_number is a driver-local callback counter,
not Motive's native iFrame.
"""

from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from mocap4r2_msgs.msg import RigidBodies


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


class MocapTimingProbe(Node):
    def __init__(self):
        super().__init__("mocap_timing_probe")
        self.declare_parameter("topic", "/rigid_bodies")
        self.declare_parameter("report_period_sec", 1.0)
        self.declare_parameter("gap_threshold_ms", 60.0)

        topic = str(self.get_parameter("topic").value)
        report_period = float(self.get_parameter("report_period_sec").value)
        self.gap_threshold_ms = float(self.get_parameter("gap_threshold_ms").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            RigidBodies, topic, self.callback, qos)
        self.timer = self.create_timer(report_period, self.report)

        self.previous_receive_ns = None
        self.previous_header_ns = None
        self.previous_frame = None
        self.receive_dt_ms = []
        self.header_dt_ms = []
        self.measurement_age_ms = []
        self.frame_deltas = []

        self.get_logger().info(
            f"Probing {topic}: BEST_EFFORT/KEEP_LAST(1), "
            f"report={report_period:.1f}s, gap>{self.gap_threshold_ms:.1f}ms")

    def callback(self, msg):
        receive_ns = self.get_clock().now().nanoseconds
        header_ns = Time.from_msg(
            msg.header.stamp, clock_type=self.get_clock().clock_type).nanoseconds

        if self.previous_receive_ns is not None:
            self.receive_dt_ms.append((receive_ns - self.previous_receive_ns) * 1e-6)
        if self.previous_header_ns is not None:
            self.header_dt_ms.append((header_ns - self.previous_header_ns) * 1e-6)
        if self.previous_frame is not None:
            self.frame_deltas.append(int(msg.frame_number) - self.previous_frame)
        if header_ns != 0:
            self.measurement_age_ms.append((receive_ns - header_ns) * 1e-6)

        self.previous_receive_ns = receive_ns
        self.previous_header_ns = header_ns
        self.previous_frame = int(msg.frame_number)

    def report(self):
        receive_dt = self.receive_dt_ms
        header_dt = self.header_dt_ms
        ages = self.measurement_age_ms
        frame_deltas = self.frame_deltas
        self.receive_dt_ms = []
        self.header_dt_ms = []
        self.measurement_age_ms = []
        self.frame_deltas = []

        if not receive_dt:
            self.get_logger().warn("No /rigid_bodies samples in this window")
            return

        receive_hz = 1000.0 / median(receive_dt) if median(receive_dt) > 0.0 else 0.0
        header_hz = 1000.0 / median(header_dt) if header_dt and median(header_dt) > 0.0 else 0.0
        long_gaps = sum(dt > self.gap_threshold_ms for dt in receive_dt)
        missing_frames = sum(max(0, delta - 1) for delta in frame_deltas)

        age_text = "measurement_age=unavailable"
        if ages:
            age_text = (
                f"measurement_age_ms[p50/p95/max]="
                f"{percentile(ages, 0.50):.1f}/"
                f"{percentile(ages, 0.95):.1f}/{max(ages):.1f}")

        self.get_logger().info(
            f"samples={len(receive_dt) + 1} "
            f"header/receive~{header_hz:.1f}/{receive_hz:.1f}Hz "
            f"receive_dt_ms[p50/p95/max]="
            f"{percentile(receive_dt, 0.50):.1f}/"
            f"{percentile(receive_dt, 0.95):.1f}/{max(receive_dt):.1f} "
            f"gaps>{self.gap_threshold_ms:.0f}ms={long_gaps} "
            f"driver_sequence_gaps={missing_frames} {age_text}")


def main(args=None):
    rclpy.init(args=args)
    node = MocapTimingProbe()
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
