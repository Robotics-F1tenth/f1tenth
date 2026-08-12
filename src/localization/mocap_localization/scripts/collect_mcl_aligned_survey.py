#!/usr/bin/env python3
"""Collect timestamp-matched MCL and mocap pairs for a shared map fit."""

from __future__ import annotations

import argparse
import csv
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


CSV_FIELDS = [
    "point_id", "split", "mocap_x", "mocap_y", "map_x", "map_y",
    "vehicle", "pair_count", "pair_dt_ms_p50", "mocap_std_m", "mcl_std_m",
]


@dataclass(frozen=True)
class TimedPose:
    """Planar pose plus its measurement timestamp in nanoseconds."""

    stamp_ns: int
    x: float
    y: float


@dataclass(frozen=True)
class Capture:
    """Averaged MCL/mocap correspondence window."""

    mocap_x: float
    mocap_y: float
    map_x: float
    map_y: float
    count: int
    dt_p50_ms: float
    mocap_std_m: float
    mcl_std_m: float


def stamp_ns(message: PoseStamped) -> int:
    """Return a PoseStamped source timestamp, rejecting unset stamps."""
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if value == 0:
        raise ValueError("PoseStamped has an unset header.stamp")
    return value


def mean(values: list[float]) -> float:
    """Return an arithmetic mean for a nonempty list."""
    return sum(values) / len(values)


def planar_std(xs: list[float], ys: list[float]) -> float:
    """Return root-mean-square planar deviation around a sample mean."""
    center_x = mean(xs)
    center_y = mean(ys)
    return math.sqrt(mean([
        (x - center_x) ** 2 + (y - center_y) ** 2
        for x, y in zip(xs, ys)
    ]))


def summarize(samples: list[tuple[TimedPose, TimedPose]]) -> Capture:
    """Average a nonempty sequence of (mocap, MCL) timestamp pairs."""
    mocap_xs = [mocap.x for mocap, _ in samples]
    mocap_ys = [mocap.y for mocap, _ in samples]
    map_xs = [mcl.x for _, mcl in samples]
    map_ys = [mcl.y for _, mcl in samples]
    dt_ms = [abs(mocap.stamp_ns - mcl.stamp_ns) * 1.0e-6 for mocap, mcl in samples]
    return Capture(
        mocap_x=mean(mocap_xs),
        mocap_y=mean(mocap_ys),
        map_x=mean(map_xs),
        map_y=mean(map_ys),
        count=len(samples),
        dt_p50_ms=float(statistics.median(dt_ms)),
        mocap_std_m=planar_std(mocap_xs, mocap_ys),
        mcl_std_m=planar_std(map_xs, map_ys),
    )


def read_rows(path: Path) -> list[dict[str, str]]:
    """Read a compatible output CSV, or return no rows for a new file."""
    if not path.exists():
        return []
    required = set(CSV_FIELDS[:6])
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"existing CSV is incompatible: {path}")
        return list(reader)


def write_rows(path: Path, rows: list[dict[str, str]]) -> None:
    """Atomically write all completed rows to the CSV."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
    temporary.replace(path)


class MclAlignedCollector(Node):
    """Pair an MCL stream with the nearest recent mocap measurement."""

    def __init__(self, mocap_topic: str, mcl_topic: str, tolerance_sec: float):
        super().__init__("mcl_aligned_survey_collector")
        self.tolerance_ns = int(tolerance_sec * 1_000_000_000)
        self.recent_mocap: deque[TimedPose] = deque(maxlen=256)
        self.capture_active = False
        self.samples: list[tuple[TimedPose, TimedPose]] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PoseStamped, mocap_topic, self._mocap_callback, qos)
        self.create_subscription(PoseStamped, mcl_topic, self._mcl_callback, qos)
        self.get_logger().info(
            f"Pairing mocap={mocap_topic} with MCL={mcl_topic}; "
            f"maximum timestamp difference={tolerance_sec * 1000.0:.1f} ms.")

    @staticmethod
    def to_timed_pose(message: PoseStamped) -> TimedPose | None:
        """Extract a finite planar pose, ignoring malformed input."""
        position = message.pose.position
        if not math.isfinite(position.x) or not math.isfinite(position.y):
            return None
        try:
            return TimedPose(stamp_ns(message), float(position.x), float(position.y))
        except ValueError:
            return None

    def _mocap_callback(self, message: PoseStamped) -> None:
        pose = self.to_timed_pose(message)
        if pose is not None:
            self.recent_mocap.append(pose)

    def _mcl_callback(self, message: PoseStamped) -> None:
        if not self.capture_active or not self.recent_mocap:
            return
        mcl_pose = self.to_timed_pose(message)
        if mcl_pose is None:
            return
        mocap_pose = min(
            self.recent_mocap,
            key=lambda pose: abs(pose.stamp_ns - mcl_pose.stamp_ns))
        if abs(mocap_pose.stamp_ns - mcl_pose.stamp_ns) <= self.tolerance_ns:
            self.samples.append((mocap_pose, mcl_pose))

    def capture(self, duration_sec: float, min_pairs: int) -> Capture:
        """Collect a stable correspondence average during one stationary window."""
        self.samples = []
        self.capture_active = True
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.capture_active = False
        if len(self.samples) < min_pairs:
            raise RuntimeError(
                f"received only {len(self.samples)} matched pairs; need {min_pairs}. "
                "Check topic names, clocks, and timestamp tolerance.")
        return summarize(self.samples)


def make_row(point_id: str, split: str, vehicle: str, capture: Capture) -> dict[str, str]:
    """Create a calibrate_shared_map-compatible row from a capture."""
    return {
        "point_id": f"{point_id}_{vehicle}",
        "split": split,
        "mocap_x": f"{capture.mocap_x:.9f}",
        "mocap_y": f"{capture.mocap_y:.9f}",
        "map_x": f"{capture.map_x:.9f}",
        "map_y": f"{capture.map_y:.9f}",
        "vehicle": vehicle,
        "pair_count": str(capture.count),
        "pair_dt_ms_p50": f"{capture.dt_p50_ms:.3f}",
        "mocap_std_m": f"{capture.mocap_std_m:.6f}",
        "mcl_std_m": f"{capture.mcl_std_m:.6f}",
    }


def next_point_id(rows: list[dict[str, str]], vehicle: str, split: str) -> str:
    """Return the next automatic fit or holdout identifier for one vehicle."""
    prefix = "F" if split == "fit" else "H"
    suffix = f"_{vehicle}"
    used = {
        row["point_id"][:-len(suffix)]
        for row in rows
        if row["point_id"].endswith(suffix)
    }
    number = 1
    while f"{prefix}{number:02d}" in used:
        number += 1
    return f"{prefix}{number:02d}"


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Collect stationary MCL-aligned mocap survey rows. This is an "
            "operational map alignment tool, not independent MCL ground truth."))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vehicle", choices=["car3", "car4"], required=True)
    parser.add_argument("--mocap-topic", required=True)
    parser.add_argument("--mcl-topic", required=True)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--min-pairs", type=int, default=20)
    parser.add_argument("--time-tolerance-ms", type=float, default=50.0)
    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    """Run the interactive MCL-aligned sample collector."""
    args = build_parser().parse_args(arguments)
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        print("ERROR: --duration-sec must be positive", file=sys.stderr)
        return 2
    if args.min_pairs < 1:
        print("ERROR: --min-pairs must be at least 1", file=sys.stderr)
        return 2
    if not math.isfinite(args.time_tolerance_ms) or args.time_tolerance_ms <= 0.0:
        print("ERROR: --time-tolerance-ms must be positive", file=sys.stderr)
        return 2
    try:
        rows = read_rows(args.output)
        existing_ids = {row["point_id"] for row in rows}
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print("\nCommands: f = fit capture, h = holdout capture, q = finish")
    print("Optional explicit form: <point_id> <fit|holdout>")
    print("Each f/h immediately captures the next 3-second stationary window.")
    print("Type q to finish. Keep calibration and evaluation laps separate.\n")

    rclpy.init(args=None)
    collector = MclAlignedCollector(
        args.mocap_topic, args.mcl_topic, args.time_tolerance_ms * 1.0e-3)
    try:
        while rclpy.ok():
            command = input("mcl-survey> ").strip()
            if command.lower() in {"q", "quit", "exit"}:
                break
            if command.lower() in {"f", "fit"}:
                split = "fit"
                point_id = next_point_id(rows, args.vehicle, split)
            elif command.lower() in {"h", "hold", "holdout"}:
                split = "holdout"
                point_id = next_point_id(rows, args.vehicle, split)
            else:
                fields = command.split()
                if len(fields) != 2 or fields[1].lower() not in {"fit", "holdout"}:
                    print("Enter: f, h, <point_id> <fit|holdout>, or q")
                    continue
                point_id, split = fields[0], fields[1].lower()
            row_id = f"{point_id}_{args.vehicle}"
            if not point_id or row_id in existing_ids:
                print(f"ERROR: point id is empty or already recorded: {row_id}")
                continue
            try:
                print(
                    f"Capturing {row_id} for {args.duration_sec:.1f} s; "
                    f"keep {args.vehicle} still.")
                capture = collector.capture(args.duration_sec, args.min_pairs)
            except (KeyboardInterrupt, RuntimeError) as exc:
                print(f"Capture cancelled: {exc}")
                continue
            print(
                f"  pairs={capture.count}, dt_p50={capture.dt_p50_ms:.1f} ms, "
                f"mocap_std={capture.mocap_std_m:.3f} m, "
                f"mcl_std={capture.mcl_std_m:.3f} m")
            rows.append(make_row(point_id, split, args.vehicle, capture))
            try:
                write_rows(args.output, rows)
            except OSError as exc:
                print(f"ERROR: could not write {args.output}: {exc}", file=sys.stderr)
                return 2
            existing_ids.add(row_id)
            print(f"Saved {row_id} -> {args.output}\n")
    except KeyboardInterrupt:
        print("\nStopped; completed rows remain in the CSV.")
    finally:
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"Finished with {len(rows)} rows: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
