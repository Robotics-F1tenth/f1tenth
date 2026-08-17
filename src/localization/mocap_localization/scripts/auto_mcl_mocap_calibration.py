#!/usr/bin/env python3
"""
Estimate two-car mocap calibration from low-speed live MCL trajectories.

This tool intentionally writes no parameter file.  It accumulates raw OptiTrack
rigid-body poses and MCL poses while the cars are driven slowly, then prints a
candidate shared ``T_map_mocap`` and a per-vehicle tracked-point-to-base
correction.  The result is MCL-aligned operational calibration, not independent
ground truth for MCL evaluation.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def wrap_pi(angle: float) -> float:
    """Wrap an angle into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rot2d(angle: float) -> np.ndarray:
    """Return a planar rotation matrix."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return ROS planar yaw from a normalized or non-normalized quaternion."""
    scale = math.sqrt(x * x + y * y + z * z + w * w)
    if scale <= np.finfo(float).eps:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / scale, y / scale, z / scale, w / scale
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Multiply scalar-last quaternions."""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], dtype=np.float64)


def source_stamp_ns(message: PoseStamped | RigidBodies) -> int:
    """Return a nonzero source stamp in nanoseconds, or zero when unset."""
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class PlanarPose:
    """One planar source measurement."""

    stamp_ns: int
    receive_sec: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Pair:
    """A raw rigid-body pose matched with an MCL base pose."""

    raw: PlanarPose
    mcl: PlanarPose


@dataclass(frozen=True)
class VehicleFit:
    """Fitted per-vehicle correction and residual summary."""

    vehicle: str
    sample_count: int
    fit_count: int
    holdout_count: int
    offset_yaw: float
    offset_xy: np.ndarray
    fit_rmse: float
    holdout_rmse: float
    holdout_p95: float


class CalibrationError(RuntimeError):
    """Raised when a live trajectory is unsuitable for calibration."""


def raw_to_ros_planar(rigid_body, motive_up_axis: str, receive_sec: float) -> PlanarPose:
    """
    Convert one raw Motive pose into the ROS planar convention.

    Motive Y-up is converted by Rx(+90 deg): ``(x, y, z) -> (x, -z, y)``.
    This is exactly the conversion in ``mocap_localization.cpp``.
    """
    pose = rigid_body.pose
    point = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quaternion = np.array([
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    ], dtype=np.float64)
    if motive_up_axis == "y":
        point = np.array([point[0], -point[2], point[1]], dtype=np.float64)
        # scalar-last Rx(+90 degrees)
        quaternion = quat_multiply(
            np.array([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]), quaternion)
    if not np.all(np.isfinite(point)) or not np.all(np.isfinite(quaternion)):
        raise ValueError("non-finite rigid-body pose")
    return PlanarPose(
        stamp_ns=0,
        receive_sec=receive_sec,
        x=float(point[0]),
        y=float(point[1]),
        yaw=yaw_from_quaternion(*quaternion),
    )


def mcl_to_planar(message: PoseStamped, receive_sec: float) -> PlanarPose:
    """Convert an MCL PoseStamped into a planar pose."""
    pose = message.pose
    if not all(math.isfinite(value) for value in (
        pose.position.x, pose.position.y, pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    )):
        raise ValueError("non-finite MCL pose")
    return PlanarPose(
        stamp_ns=source_stamp_ns(message),
        receive_sec=receive_sec,
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=yaw_from_quaternion(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w),
    )


def fit_initial_shared(pairs: list[Pair]) -> np.ndarray:
    """Initialize shared yaw/translation from point origins, ignoring lever arms."""
    source = np.array([[pair.raw.x, pair.raw.y] for pair in pairs], dtype=np.float64)
    target = np.array([[pair.mcl.x, pair.mcl.y] for pair in pairs], dtype=np.float64)
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    translation = target_mean - rotation @ source_mean
    return np.array([yaw, translation[0], translation[1]], dtype=np.float64)


def model_pose(parameters: np.ndarray, raw: PlanarPose, vehicle_index: int) -> np.ndarray:
    """Apply shared map transform and one local tracked-point-to-base transform."""
    shared_yaw, tx, ty = parameters[:3]
    index = 3 + 3 * vehicle_index
    offset_yaw, bx, by = parameters[index:index + 3]
    shared_rotation = rot2d(shared_yaw)
    raw_rotation = rot2d(raw.yaw)
    position = (
        shared_rotation @ np.array([raw.x, raw.y], dtype=np.float64)
        + np.array([tx, ty], dtype=np.float64)
        + shared_rotation @ raw_rotation @ np.array([bx, by], dtype=np.float64)
    )
    return np.array([
        position[0], position[1], wrap_pi(shared_yaw + raw.yaw + offset_yaw),
    ], dtype=np.float64)


def residual_vector(parameters: np.ndarray, grouped_pairs: list[list[Pair]],
                    yaw_weight_m_per_rad: float) -> np.ndarray:
    """Return xy/yaw residuals for all selected trajectory samples."""
    residuals: list[float] = []
    for vehicle_index, pairs in enumerate(grouped_pairs):
        for pair in pairs:
            prediction = model_pose(parameters, pair.raw, vehicle_index)
            residuals.extend([
                prediction[0] - pair.mcl.x,
                prediction[1] - pair.mcl.y,
                yaw_weight_m_per_rad * wrap_pi(prediction[2] - pair.mcl.yaw),
            ])
    return np.asarray(residuals, dtype=np.float64)


def robust_weights(residuals: np.ndarray, huber_delta_m: float) -> np.ndarray:
    """Return one Huber weight per xyz/yaw residual block."""
    blocks = residuals.reshape((-1, 3))
    norms = np.linalg.norm(blocks, axis=1)
    block_weights = np.ones_like(norms)
    outside = norms > huber_delta_m
    block_weights[outside] = huber_delta_m / norms[outside]
    return np.repeat(np.sqrt(block_weights), 3)


def optimize(parameters: np.ndarray, grouped_pairs: list[list[Pair]],
             yaw_weight: float, huber_delta: float) -> np.ndarray:
    """Run robust numerical Levenberg-Marquardt on the SE(2) chain."""
    damping = 1.0e-3
    current = parameters.copy()
    for _ in range(45):
        residuals = residual_vector(current, grouped_pairs, yaw_weight)
        weights = robust_weights(residuals, huber_delta)
        weighted = residuals * weights
        jacobian = np.empty((residuals.size, current.size), dtype=np.float64)
        for column in range(current.size):
            perturbation = 1.0e-5
            candidate = current.copy()
            candidate[column] += perturbation
            delta = residual_vector(candidate, grouped_pairs, yaw_weight) - residuals
            jacobian[:, column] = delta / perturbation * weights
        hessian = jacobian.T @ jacobian + damping * np.eye(current.size)
        gradient = jacobian.T @ weighted
        step = -np.linalg.lstsq(hessian, gradient, rcond=None)[0]
        candidate = current + step
        candidate[0] = wrap_pi(candidate[0])
        for index in range(3, candidate.size, 3):
            candidate[index] = wrap_pi(candidate[index])
        new_residuals = residual_vector(candidate, grouped_pairs, yaw_weight)
        if np.sum((new_residuals * robust_weights(new_residuals, huber_delta)) ** 2) < \
                np.sum(weighted ** 2):
            current = candidate
            damping = max(1.0e-7, damping * 0.5)
            if np.linalg.norm(step) < 1.0e-7:
                break
        else:
            damping = min(1.0e5, damping * 4.0)
    return current


def pose_errors(parameters: np.ndarray, pairs: list[Pair], vehicle_index: int,
                yaw_weight: float) -> np.ndarray:
    """Return weighted 3D residual norms for a vehicle subset."""
    errors = []
    for pair in pairs:
        prediction = model_pose(parameters, pair.raw, vehicle_index)
        errors.append(math.sqrt(
            (prediction[0] - pair.mcl.x) ** 2
            + (prediction[1] - pair.mcl.y) ** 2
            + (yaw_weight * wrap_pi(prediction[2] - pair.mcl.yaw)) ** 2))
    return np.asarray(errors, dtype=np.float64)


def split_pairs(pairs: list[Pair], holdout_fraction: float) -> tuple[list[Pair], list[Pair]]:
    """Keep the final trajectory segment for a temporal holdout check."""
    holdout_count = max(20, int(len(pairs) * holdout_fraction))
    if len(pairs) < holdout_count + 80:
        raise CalibrationError("need at least 100 accepted samples per vehicle")
    return pairs[:-holdout_count], pairs[-holdout_count:]


class LiveAutoCalibrator(Node):
    """Collect low-speed raw/MCL pose pairs for car3 and car4."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("auto_mcl_mocap_calibration")
        self.args = args
        self.raw_by_vehicle: dict[str, deque[PlanarPose]] = {
            "car3": deque(maxlen=512), "car4": deque(maxlen=512),
        }
        self.pairs: dict[str, list[Pair]] = {"car3": [], "car4": []}
        self.last_mcl: dict[str, PlanarPose | None] = {"car3": None, "car4": None}
        self.last_accepted_time: dict[str, float] = {"car3": -math.inf, "car4": -math.inf}
        self.mcl_received: dict[str, int] = {"car3": 0, "car4": 0}
        self.too_fast: dict[str, int] = {"car3": 0, "car4": 0}
        self.unmatched: dict[str, int] = {"car3": 0, "car4": 0}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=64,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(RigidBodies, args.rigid_topic, self._raw_callback, qos)
        self.create_subscription(PoseStamped, args.car3_mcl_topic,
                                 lambda message: self._mcl_callback("car3", message), qos)
        self.create_subscription(PoseStamped, args.car4_mcl_topic,
                                 lambda message: self._mcl_callback("car4", message), qos)
        self.timer = self.create_timer(5.0, self._report_progress)
        self.get_logger().info(
            "Recording automatic low-speed calibration. Drive car3 and car4 "
            "sequentially, then Ctrl-C to fit. "
            f"Motive up axis={args.motive_up_axis}.")

    def _raw_callback(self, message: RigidBodies) -> None:
        receive_sec = time.monotonic()
        stamp = source_stamp_ns(message)
        for rigid_body in message.rigidbodies:
            vehicle = None
            if rigid_body.rigid_body_name == self.args.car3_body:
                vehicle = "car3"
            elif rigid_body.rigid_body_name == self.args.car4_body:
                vehicle = "car4"
            if vehicle is None:
                continue
            try:
                pose = raw_to_ros_planar(rigid_body, self.args.motive_up_axis, receive_sec)
            except ValueError:
                continue
            self.raw_by_vehicle[vehicle].append(PlanarPose(
                stamp_ns=stamp, receive_sec=pose.receive_sec,
                x=pose.x, y=pose.y, yaw=pose.yaw))

    def _mcl_callback(self, vehicle: str, message: PoseStamped) -> None:
        receive_sec = time.monotonic()
        self.mcl_received[vehicle] += 1
        try:
            mcl = mcl_to_planar(message, receive_sec)
        except ValueError:
            return
        previous = self.last_mcl[vehicle]
        self.last_mcl[vehicle] = mcl
        if previous is None:
            return
        elapsed = mcl.receive_sec - previous.receive_sec
        if elapsed <= 1.0e-4:
            return
        speed = math.hypot(mcl.x - previous.x, mcl.y - previous.y) / elapsed
        yaw_rate = abs(wrap_pi(mcl.yaw - previous.yaw)) / elapsed
        if speed > self.args.max_speed_mps or yaw_rate > self.args.max_yaw_rate_radps:
            self.too_fast[vehicle] += 1
            return
        if mcl.receive_sec - self.last_accepted_time[vehicle] < self.args.sample_period_sec:
            return
        raw_history = self.raw_by_vehicle[vehicle]
        if not raw_history:
            self.unmatched[vehicle] += 1
            return
        if mcl.stamp_ns and raw_history[-1].stamp_ns:
            raw = min(raw_history, key=lambda pose: abs(pose.stamp_ns - mcl.stamp_ns))
            difference_ms = abs(raw.stamp_ns - mcl.stamp_ns) * 1.0e-6
            if difference_ms > self.args.pair_tolerance_ms:
                self.unmatched[vehicle] += 1
                return
        else:
            raw = raw_history[-1]
        self.pairs[vehicle].append(Pair(raw=raw, mcl=mcl))
        self.last_accepted_time[vehicle] = mcl.receive_sec

    def _report_progress(self) -> None:
        self.get_logger().info(
            "accepted car3/car4=%d/%d, mcl=%d/%d, too_fast=%d/%d, unmatched=%d/%d" % (
                len(self.pairs["car3"]), len(self.pairs["car4"]),
                self.mcl_received["car3"], self.mcl_received["car4"],
                self.too_fast["car3"], self.too_fast["car4"],
                self.unmatched["car3"], self.unmatched["car4"],
            ))


def fit_and_report(pairs_by_vehicle: dict[str, list[Pair]], args: argparse.Namespace) -> dict:
    """Fit a joint shared transform and print Motive-ready correction values."""
    active = [vehicle for vehicle in ("car3", "car4") if len(pairs_by_vehicle[vehicle]) >= 100]
    if not active:
        raise CalibrationError("no vehicle has 100 accepted low-speed samples")
    train_sets: list[list[Pair]] = []
    holdout_sets: list[list[Pair]] = []
    for vehicle in active:
        train, holdout = split_pairs(pairs_by_vehicle[vehicle], args.holdout_fraction)
        train_sets.append(train)
        holdout_sets.append(holdout)
    initial = fit_initial_shared([pair for pairs in train_sets for pair in pairs])
    parameters = np.concatenate([initial, np.zeros(3 * len(active), dtype=np.float64)])
    parameters = optimize(parameters, train_sets, args.yaw_weight_m_per_rad, args.huber_delta_m)

    shared_yaw, shared_tx, shared_ty = parameters[:3]
    print("\n=== MCL-aligned automatic calibration candidate ===")
    print("This was fitted from low-speed MCL; do not use it to score MCL accuracy.")
    print("shared T_map_mocap (after the selected Y-up/Z-up conversion):")
    print(f"  yaw: {math.degrees(shared_yaw):+.3f} deg")
    print(f"  translation: x={shared_tx:+.3f} m, y={shared_ty:+.3f} m")
    print("\nVehicle rigid-body -> base_link corrections:")
    report: dict[str, object] = {
        "motive_up_axis": args.motive_up_axis,
        "shared_map_mocap": {
            "yaw_deg": math.degrees(shared_yaw), "translation_m": [shared_tx, shared_ty, 0.0],
        },
        "vehicles": {},
    }
    for vehicle_index, vehicle in enumerate(active):
        offset_yaw, bx, by = parameters[3 + 3 * vehicle_index:6 + 3 * vehicle_index]
        fit_errors = pose_errors(parameters, train_sets[vehicle_index], vehicle_index,
                                 args.yaw_weight_m_per_rad)
        holdout_errors = pose_errors(parameters, holdout_sets[vehicle_index], vehicle_index,
                                     args.yaw_weight_m_per_rad)
        print(f"\n  {vehicle}:")
        print(f"    tracked->base yaw: {math.degrees(offset_yaw):+.3f} deg")
        print(
            "    tracked->base horizontal offset (ROS): "
            f"x={bx * 100.0:+.1f} cm, y={by * 100.0:+.1f} cm")
        if args.motive_up_axis == "y":
            print("    Motive Y-up Builder pivot delta (current pivot -> base_link):")
            print(f"      X={bx * 100.0:+.1f} cm, Y=unobservable height, Z={-by * 100.0:+.1f} cm")
            print(f"      local-axis yaw about Motive +Y: {math.degrees(offset_yaw):+.3f} deg")
        else:
            print("    Motive Z-up Builder pivot delta (current pivot -> base_link):")
            print(f"      X={bx * 100.0:+.1f} cm, Y={by * 100.0:+.1f} cm, Z=unobservable height")
            print(f"      local-axis yaw about Motive +Z: {math.degrees(offset_yaw):+.3f} deg")
        print(f"    residual: fit RMSE={math.sqrt(np.mean(fit_errors ** 2)) * 100.0:.1f} cm, "
              f"holdout RMSE={math.sqrt(np.mean(holdout_errors ** 2)) * 100.0:.1f} cm, "
              f"holdout p95={np.percentile(holdout_errors, 95) * 100.0:.1f} cm")
        report["vehicles"][vehicle] = {
            "accepted_samples": len(pairs_by_vehicle[vehicle]),
            "tracked_to_base_yaw_deg": math.degrees(offset_yaw),
            "tracked_to_base_ros_xy_m": [bx, by],
            "motive_pivot_delta_m": (
                [bx, 0.0, -by] if args.motive_up_axis == "y" else [bx, by, 0.0]),
            "fit_rmse_m": math.sqrt(np.mean(fit_errors ** 2)),
            "holdout_rmse_m": math.sqrt(np.mean(holdout_errors ** 2)),
            "holdout_p95_m": float(np.percentile(holdout_errors, 95)),
        }
    print(
        "\nY/roll/pitch cannot be estimated from 2D lidar MCL; keep the "
        "measured height and verify with the 0/90/180/270 static test.")
    return report


def build_parser() -> argparse.ArgumentParser:
    """Create command-line arguments for the live auto-calibrator."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rigid-topic", default="/rigid_bodies")
    parser.add_argument("--car3-mcl-topic", default="/car3/newmcl_pose")
    parser.add_argument("--car4-mcl-topic", default="/car4/newmcl_pose")
    parser.add_argument("--car3-body", default="6")
    parser.add_argument("--car4-body", default="7")
    parser.add_argument("--motive-up-axis", choices=["y", "z"], required=True)
    parser.add_argument("--max-speed-mps", type=float, default=0.7)
    parser.add_argument("--max-yaw-rate-radps", type=float, default=0.7)
    parser.add_argument("--sample-period-sec", type=float, default=0.10)
    parser.add_argument("--pair-tolerance-ms", type=float, default=80.0)
    parser.add_argument("--yaw-weight-m-per-rad", type=float, default=0.35)
    parser.add_argument("--huber-delta-m", type=float, default=0.15)
    parser.add_argument("--holdout-fraction", type=float, default=0.20)
    parser.add_argument(
        "--report", type=Path,
        help="optional JSON report; source params are never written")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    """Reject unsafe numerical command-line settings."""
    for name in ("max_speed_mps", "max_yaw_rate_radps", "sample_period_sec",
                 "pair_tolerance_ms", "yaw_weight_m_per_rad", "huber_delta_m"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            raise CalibrationError(f"--{name.replace('_', '-')} must be positive")
    if not 0.05 <= args.holdout_fraction <= 0.4:
        raise CalibrationError("--holdout-fraction must be in [0.05, 0.4]")


def main(arguments: Sequence[str] | None = None) -> int:
    """Record until Ctrl-C, then fit and optionally save a report."""
    args = build_parser().parse_args(arguments)
    try:
        validate_args(args)
    except CalibrationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    rclpy.init(args=None)
    node = LiveAutoCalibrator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            report = fit_and_report(node.pairs, args)
            if args.report is not None:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
                print(f"\nSaved report: {args.report}")
        except CalibrationError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
