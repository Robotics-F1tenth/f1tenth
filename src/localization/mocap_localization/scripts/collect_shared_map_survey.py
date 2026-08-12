#!/usr/bin/env python3
"""Interactively collect paired car3/car4 survey rows for shared-map fitting."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


CSV_FIELDS = [
    "point_id", "split", "mocap_x", "mocap_y", "map_x", "map_y",
    "vehicle", "sample_count", "std_x_m", "std_y_m",
]


@dataclass(frozen=True)
class Capture:
    """One stable-position average from one vehicle."""

    x: float
    y: float
    count: int
    std_x: float
    std_y: float


def finite_float(text: str, label: str) -> float:
    """Parse one finite command-line coordinate."""
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError(f"{label} must be a number, got {text!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite, got {text!r}")
    return value


def sample_statistics(samples: list[tuple[float, float]]) -> Capture:
    """Return population mean and standard deviation for planar samples."""
    count = len(samples)
    mean_x = sum(sample[0] for sample in samples) / count
    mean_y = sum(sample[1] for sample in samples) / count
    std_x = math.sqrt(sum((sample[0] - mean_x) ** 2 for sample in samples) / count)
    std_y = math.sqrt(sum((sample[1] - mean_y) ** 2 for sample in samples) / count)
    return Capture(mean_x, mean_y, count, std_x, std_y)


def read_rows(path: Path) -> list[dict[str, str]]:
    """Read an existing compatible survey CSV, or return no rows for a new file."""
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not set(CSV_FIELDS[:6]).issubset(reader.fieldnames):
            raise ValueError(f"existing CSV is incompatible: {path}")
        return list(reader)


def write_rows(path: Path, rows: list[dict[str, str]]) -> None:
    """Atomically replace the survey CSV so a completed pair is never half-written."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
    temporary.replace(path)


class SurveyCollector(Node):
    """Collect stable pose averages from two PoseStamped topics."""

    def __init__(self, car3_topic: str, car4_topic: str):
        super().__init__("shared_map_survey_collector")
        self.active_vehicle: str | None = None
        self.samples: list[tuple[float, float]] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=32,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PoseStamped, car3_topic, self._car3_callback, qos)
        self.create_subscription(PoseStamped, car4_topic, self._car4_callback, qos)
        self.get_logger().info(
            f"Waiting for car3={car3_topic}, car4={car4_topic} "
            "(BEST_EFFORT/KEEP_LAST(32)).")

    def _car3_callback(self, message: PoseStamped) -> None:
        self._record("car3", message)

    def _car4_callback(self, message: PoseStamped) -> None:
        self._record("car4", message)

    def _record(self, vehicle: str, message: PoseStamped) -> None:
        if self.active_vehicle != vehicle:
            return
        position = message.pose.position
        if math.isfinite(position.x) and math.isfinite(position.y):
            self.samples.append((float(position.x), float(position.y)))

    def capture(self, vehicle: str, duration_sec: float, min_samples: int) -> Capture:
        """Capture only one vehicle's arrivals for a fixed wall-clock interval."""
        self.samples = []
        self.active_vehicle = vehicle
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.active_vehicle = None
        if len(self.samples) < min_samples:
            raise RuntimeError(
                f"{vehicle}: received only {len(self.samples)} valid samples; "
                f"need at least {min_samples}. Check the topic and tracking.")
        return sample_statistics(self.samples)


def prompt_capture(
    collector: SurveyCollector,
    vehicle: str,
    duration_sec: float,
    min_samples: int,
) -> Capture:
    """Wait for the operator before a single vehicle capture."""
    input(
        f"Place {vehicle} rear-axle center on the point, keep it still, "
        f"then press ENTER to capture {duration_sec:.1f} s: ")
    capture = collector.capture(vehicle, duration_sec, min_samples)
    print(
        f"  {vehicle}: n={capture.count}, mean=({capture.x:.6f}, {capture.y:.6f}), "
        f"std=({capture.std_x:.4f}, {capture.std_y:.4f}) m")
    return capture


def make_row(
    point_id: str, split: str, map_x: float, map_y: float, vehicle: str,
    capture: Capture,
) -> dict[str, str]:
    """Convert one capture to a calibrate_shared_map-compatible CSV row."""
    return {
        "point_id": f"{point_id}_{vehicle}",
        "split": split,
        "mocap_x": f"{capture.x:.9f}",
        "mocap_y": f"{capture.y:.9f}",
        "map_x": f"{map_x:.9f}",
        "map_y": f"{map_y:.9f}",
        "vehicle": vehicle,
        "sample_count": str(capture.count),
        "std_x_m": f"{capture.std_x:.9f}",
        "std_y_m": f"{capture.std_y:.9f}",
    }


def build_parser() -> argparse.ArgumentParser:
    """Create the CLI parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Interactively capture car3 then car4 at each surveyed map point. "
            "The output CSV can be passed directly to calibrate_shared_map."))
    parser.add_argument(
        "--output", type=Path, required=True, help="survey CSV to create or resume")
    parser.add_argument("--car3-topic", default="/mocap_cal/car3/base")
    parser.add_argument("--car4-topic", default="/mocap_cal/car4/base")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=30)
    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    """Run the interactive paired survey collector."""
    args = build_parser().parse_args(arguments)
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        print("ERROR: --duration-sec must be positive", file=sys.stderr)
        return 2
    if args.min_samples < 1:
        print("ERROR: --min-samples must be at least 1", file=sys.stderr)
        return 2
    try:
        rows = read_rows(args.output)
        existing_ids = {row["point_id"] for row in rows}
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print("\nCommands: <point_id> <fit|holdout> <map_x> <map_y>")
    print("Example:  P01 fit -1.250 0.700")
    print("For each point, the tool captures car3 first and car4 second. Type q to finish.\n")

    rclpy.init(args=None)
    collector = SurveyCollector(args.car3_topic, args.car4_topic)
    try:
        while rclpy.ok():
            command = input("survey> ").strip()
            if command.lower() in {"q", "quit", "exit"}:
                break
            fields = command.split()
            if len(fields) != 4:
                print("Enter: <point_id> <fit|holdout> <map_x> <map_y>, or q")
                continue
            point_id, split, map_x_text, map_y_text = fields
            split = split.lower()
            if not point_id or split not in {"fit", "holdout"}:
                print("split must be fit or holdout, and point_id must not be empty")
                continue
            try:
                map_x = finite_float(map_x_text, "map_x")
                map_y = finite_float(map_y_text, "map_y")
            except ValueError as exc:
                print(f"ERROR: {exc}")
                continue

            new_ids = {f"{point_id}_car3", f"{point_id}_car4"}
            collision = sorted(new_ids.intersection(existing_ids))
            if collision:
                print(f"ERROR: already recorded: {', '.join(collision)}")
                continue
            try:
                car3_capture = prompt_capture(
                    collector, "car3", args.duration_sec, args.min_samples)
                car4_capture = prompt_capture(
                    collector, "car4", args.duration_sec, args.min_samples)
            except (KeyboardInterrupt, RuntimeError) as exc:
                print(f"Capture cancelled: {exc}")
                continue

            rows.extend([
                make_row(point_id, split, map_x, map_y, "car3", car3_capture),
                make_row(point_id, split, map_x, map_y, "car4", car4_capture),
            ])
            try:
                write_rows(args.output, rows)
            except OSError as exc:
                print(f"ERROR: could not write {args.output}: {exc}", file=sys.stderr)
                return 2
            existing_ids.update(new_ids)
            print(f"Saved {point_id}: car3 + car4 -> {args.output}\n")
    except KeyboardInterrupt:
        print("\nStopped; all previously completed point pairs remain in the CSV.")
    finally:
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"Finished with {len(rows)} rows: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
