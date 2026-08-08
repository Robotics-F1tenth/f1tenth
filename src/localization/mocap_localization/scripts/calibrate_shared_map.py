#!/usr/bin/env python3
"""
Fit one shared, scale-free SE(2) map transform from surveyed points.

The input CSV contains base-corrected mocap positions and their independently
surveyed map positions.  Only ``rotation`` and ``translation`` are replaced in
the two generated candidate parameter files; vehicle-specific parameters are
preserved.  Source parameter files are never modified.
"""

from __future__ import annotations

import argparse
import copy
import csv
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Iterable, Mapping, Sequence

import numpy as np
import yaml


REQUIRED_COLUMNS = {
    "point_id",
    "split",
    "mocap_x",
    "mocap_y",
    "map_x",
    "map_y",
}
VALID_SPLITS = {"fit", "holdout"}
ROTATION_ATOL = 1.0e-10


class CalibrationError(RuntimeError):
    """Raised when inputs are unsafe or insufficient for calibration."""


@dataclass(frozen=True)
class SurveyPoint:
    point_id: str
    split: str
    mocap_xy: np.ndarray
    map_xy: np.ndarray


@dataclass(frozen=True)
class ResidualStats:
    count: int
    rmse_m: float
    mean_m: float
    median_m: float
    p95_m: float
    max_m: float


@dataclass(frozen=True)
class FitResult:
    rotation: np.ndarray
    translation: np.ndarray
    source_condition_ratio: float
    target_condition_ratio: float
    cross_condition_ratio: float
    fit_stats: ResidualStats
    holdout_stats: ResidualStats


def _finite_float(raw: str, column: str, row_number: int) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(
            f"CSV row {row_number}: {column} must be a number, got {raw!r}"
        ) from exc
    if not math.isfinite(value):
        raise CalibrationError(
            f"CSV row {row_number}: {column} must be finite, got {raw!r}"
        )
    return value


def load_survey_csv(path: Path) -> list[SurveyPoint]:
    if not path.is_file():
        raise CalibrationError(f"survey CSV not found: {path}")

    points: list[SurveyPoint] = []
    seen_ids: set[str] = set()
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise CalibrationError(f"survey CSV has no header: {path}")
        missing = REQUIRED_COLUMNS.difference(name.strip() for name in reader.fieldnames)
        if missing:
            raise CalibrationError(
                f"survey CSV is missing columns: {', '.join(sorted(missing))}"
            )

        for row_number, row in enumerate(reader, start=2):
            point_id = (row.get("point_id") or "").strip()
            split = (row.get("split") or "").strip().lower()
            if not point_id:
                raise CalibrationError(f"CSV row {row_number}: point_id is empty")
            if point_id in seen_ids:
                raise CalibrationError(f"CSV row {row_number}: duplicate point_id {point_id!r}")
            if split not in VALID_SPLITS:
                raise CalibrationError(
                    f"CSV row {row_number}: split must be fit or holdout, got {split!r}"
                )

            points.append(
                SurveyPoint(
                    point_id=point_id,
                    split=split,
                    mocap_xy=np.array(
                        [
                            _finite_float(row.get("mocap_x"), "mocap_x", row_number),
                            _finite_float(row.get("mocap_y"), "mocap_y", row_number),
                        ],
                        dtype=np.float64,
                    ),
                    map_xy=np.array(
                        [
                            _finite_float(row.get("map_x"), "map_x", row_number),
                            _finite_float(row.get("map_y"), "map_y", row_number),
                        ],
                        dtype=np.float64,
                    ),
                )
            )
            seen_ids.add(point_id)

    fit_count = sum(point.split == "fit" for point in points)
    holdout_count = sum(point.split == "holdout" for point in points)
    if fit_count < 3:
        raise CalibrationError(
            f"at least 3 fit points are required; found {fit_count}"
        )
    if holdout_count < 1:
        raise CalibrationError(
            "at least 1 holdout point is required for independent validation"
        )
    return points


def _points_for_split(
    points: Iterable[SurveyPoint], split: str
) -> tuple[np.ndarray, np.ndarray]:
    selected = [point for point in points if point.split == split]
    source = np.stack([point.mocap_xy for point in selected])
    target = np.stack([point.map_xy for point in selected])
    return source, target


def _condition_ratio(centered_points: np.ndarray, label: str) -> float:
    singular_values = np.linalg.svd(centered_points, compute_uv=False)
    if singular_values.shape != (2,) or singular_values[0] <= np.finfo(float).eps:
        raise CalibrationError(f"{label} fit points have zero spatial extent")
    return float(singular_values[1] / singular_values[0])


def _validate_geometry(
    source: np.ndarray,
    target: np.ndarray,
    min_condition_ratio: float,
) -> tuple[float, float]:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2:
        raise CalibrationError("fit point arrays must both have shape (N, 2)")
    if source.shape[0] < 3:
        raise CalibrationError("at least 3 fit points are required")
    if not 0.0 < min_condition_ratio <= 1.0:
        raise CalibrationError("min_condition_ratio must be in (0, 1]")

    source_ratio = _condition_ratio(source - source.mean(axis=0), "mocap")
    target_ratio = _condition_ratio(target - target.mean(axis=0), "map")
    if source_ratio < min_condition_ratio:
        raise CalibrationError(
            "mocap fit points are collinear or poorly conditioned: "
            f"ratio={source_ratio:.6g} < {min_condition_ratio:.6g}"
        )
    if target_ratio < min_condition_ratio:
        raise CalibrationError(
            "map fit points are collinear or poorly conditioned: "
            f"ratio={target_ratio:.6g} < {min_condition_ratio:.6g}"
        )
    return source_ratio, target_ratio


def _residual_stats(source: np.ndarray, target: np.ndarray, rotation: np.ndarray,
                    translation: np.ndarray) -> ResidualStats:
    predicted = (rotation @ source.T).T + translation
    errors = np.linalg.norm(predicted - target, axis=1)
    return ResidualStats(
        count=int(errors.size),
        rmse_m=float(np.sqrt(np.mean(errors * errors))),
        mean_m=float(np.mean(errors)),
        median_m=float(np.median(errors)),
        p95_m=float(np.percentile(errors, 95)),
        max_m=float(np.max(errors)),
    )


def fit_shared_se2(
    points: Sequence[SurveyPoint], min_condition_ratio: float
) -> FitResult:
    source, target = _points_for_split(points, "fit")
    holdout_source, holdout_target = _points_for_split(points, "holdout")
    source_ratio, target_ratio = _validate_geometry(
        source, target, min_condition_ratio
    )

    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    cross_covariance = source_centered.T @ target_centered

    u_matrix, singular_values, vt_matrix = np.linalg.svd(cross_covariance)
    if singular_values[0] <= np.finfo(float).eps:
        raise CalibrationError("cross-covariance has zero spatial extent")
    cross_ratio = float(singular_values[1] / singular_values[0])
    if cross_ratio < min_condition_ratio:
        raise CalibrationError(
            "fit correspondence cross-covariance is poorly conditioned: "
            f"ratio={cross_ratio:.6g} < {min_condition_ratio:.6g}"
        )

    # Kabsch fit with scale fixed to exactly one and reflection explicitly removed.
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_mean - rotation @ source_mean

    determinant = float(np.linalg.det(rotation))
    if abs(determinant - 1.0) > ROTATION_ATOL:
        raise CalibrationError(
            f"fitted transform is not a proper rotation: det={determinant:.12g}"
        )
    if not np.allclose(rotation.T @ rotation, np.eye(2), atol=ROTATION_ATOL, rtol=0.0):
        raise CalibrationError("fitted rotation failed the orthonormality check")

    return FitResult(
        rotation=rotation,
        translation=translation,
        source_condition_ratio=source_ratio,
        target_condition_ratio=target_ratio,
        cross_condition_ratio=cross_ratio,
        fit_stats=_residual_stats(source, target, rotation, translation),
        holdout_stats=_residual_stats(
            holdout_source, holdout_target, rotation, translation
        ),
    )


def _load_params(path: Path) -> dict:
    if not path.is_file():
        raise CalibrationError(f"params file not found: {path}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise CalibrationError(f"params YAML root must be a mapping: {path}")
    try:
        params = data["/**"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise CalibrationError(
            f"params YAML must contain /**/ros__parameters: {path}"
        ) from exc
    if not isinstance(params, dict):
        raise CalibrationError(f"ros__parameters must be a mapping: {path}")
    return data


def _shared_values(result: FitResult) -> tuple[list[float], list[float]]:
    rotation = [
        float(result.rotation[0, 0]), float(result.rotation[0, 1]), 0.0,
        float(result.rotation[1, 0]), float(result.rotation[1, 1]), 0.0,
        0.0, 0.0, 1.0,
    ]
    translation = [
        float(result.translation[0]),
        float(result.translation[1]),
        0.0,
    ]
    return rotation, translation


def _make_candidate(data: Mapping, rotation: Sequence[float],
                    translation: Sequence[float]) -> dict:
    candidate = copy.deepcopy(data)
    params = candidate["/**"]["ros__parameters"]
    params["rotation"] = list(rotation)
    params["translation"] = list(translation)
    return candidate


def _same_file_if_present(first: Path, second: Path) -> bool:
    try:
        return first.samefile(second)
    except FileNotFoundError:
        return False


def _preflight_output_paths(
    sources: Sequence[Path], outputs: Sequence[Path], overwrite: bool
) -> None:
    resolved_sources = [source.resolve(strict=True) for source in sources]
    if len(set(resolved_sources)) != len(resolved_sources):
        raise CalibrationError("the two source params paths must be different files")
    if len(set(output.resolve(strict=False) for output in outputs)) != len(outputs):
        raise CalibrationError("candidate output paths collide")

    for output in outputs:
        resolved_output = output.resolve(strict=False)
        if resolved_output in resolved_sources:
            raise CalibrationError(
                f"refusing to overwrite a source params file: {output}"
            )
        for source in sources:
            if _same_file_if_present(source, output):
                raise CalibrationError(
                    f"refusing to overwrite a source params file: {output}"
                )
        if output.exists() and not overwrite:
            raise CalibrationError(
                f"candidate already exists (use --overwrite explicitly): {output}"
            )
        if output.exists() and not output.is_file():
            raise CalibrationError(f"candidate path exists and is not a file: {output}")


def _yaml_text(data: Mapping) -> str:
    return yaml.safe_dump(
        data,
        sort_keys=False,
        default_flow_style=False,
        width=120,
        allow_unicode=True,
    )


def _stage_text(output: Path, text: str, mode: int) -> Path:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    return temporary


def write_candidates(
    params1_path: Path,
    params2_path: Path,
    output_dir: Path,
    result: FitResult,
    overwrite: bool,
) -> tuple[Path, Path]:
    sources = [params1_path, params2_path]
    if params1_path.name == params2_path.name:
        raise CalibrationError(
            "source params basenames are identical; candidate filenames would collide"
        )
    outputs = [output_dir / params1_path.name, output_dir / params2_path.name]
    _preflight_output_paths(sources, outputs, overwrite)

    data1 = _load_params(params1_path)
    data2 = _load_params(params2_path)
    rotation, translation = _shared_values(result)
    candidate1 = _make_candidate(data1, rotation, translation)
    candidate2 = _make_candidate(data2, rotation, translation)

    params1 = candidate1["/**"]["ros__parameters"]
    params2 = candidate2["/**"]["ros__parameters"]
    if params1["rotation"] != params2["rotation"]:
        raise CalibrationError("internal error: candidate rotations are not identical")
    if params1["translation"] != params2["translation"]:
        raise CalibrationError("internal error: candidate translations are not identical")

    output_dir.mkdir(parents=True, exist_ok=True)
    staged: list[Path] = []
    committed: list[Path] = []
    try:
        staged.append(
            _stage_text(outputs[0], _yaml_text(candidate1), params1_path.stat().st_mode & 0o777)
        )
        staged.append(
            _stage_text(outputs[1], _yaml_text(candidate2), params2_path.stat().st_mode & 0o777)
        )
        for temporary, output in zip(staged, outputs):
            if overwrite:
                os.replace(temporary, output)
            else:
                # Hard-link creation is atomic and fails instead of overwriting a race winner.
                os.link(temporary, output)
                temporary.unlink()
            committed.append(output)
    except Exception:
        for temporary in staged:
            temporary.unlink(missing_ok=True)
        if not overwrite:
            for output in committed:
                output.unlink(missing_ok=True)
        raise

    return outputs[0], outputs[1]


def _print_stats(label: str, stats: ResidualStats) -> None:
    print(
        f"{label}: n={stats.count} "
        f"rmse={stats.rmse_m:.6f} m mean={stats.mean_m:.6f} m "
        f"p50={stats.median_m:.6f} m p95={stats.p95_m:.6f} m "
        f"max={stats.max_m:.6f} m"
    )


def print_report(result: FitResult) -> None:
    yaw_deg = math.degrees(math.atan2(result.rotation[1, 0], result.rotation[0, 0]))
    print("Shared scale-free SE(2) fit")
    print(f"  yaw: {yaw_deg:.9f} deg")
    print(
        "  translation: "
        f"[{result.translation[0]:.12g}, {result.translation[1]:.12g}] m"
    )
    print(f"  determinant: {np.linalg.det(result.rotation):.12g}")
    print(
        "  conditioning ratios (larger is better): "
        f"mocap={result.source_condition_ratio:.6g} "
        f"map={result.target_condition_ratio:.6g} "
        f"cross={result.cross_condition_ratio:.6g}"
    )
    _print_stats("  fit residual", result.fit_stats)
    _print_stats("  holdout residual", result.holdout_stats)


def run_self_test() -> None:
    angle = math.radians(-37.0)
    expected_rotation = np.array(
        [[math.cos(angle), -math.sin(angle)],
         [math.sin(angle), math.cos(angle)]],
        dtype=np.float64,
    )
    expected_translation = np.array([1.25, -0.80], dtype=np.float64)
    fit_source = np.array(
        [[-1.2, -0.4], [0.0, 0.0], [2.1, 0.3], [-0.3, 1.8], [1.4, 2.2]],
        dtype=np.float64,
    )
    holdout_source = np.array([[0.7, -1.1], [2.4, 1.5]], dtype=np.float64)

    points: list[SurveyPoint] = []
    for index, source in enumerate(fit_source):
        points.append(
            SurveyPoint(
                point_id=f"fit_{index}",
                split="fit",
                mocap_xy=source,
                map_xy=expected_rotation @ source + expected_translation,
            )
        )
    for index, source in enumerate(holdout_source):
        points.append(
            SurveyPoint(
                point_id=f"holdout_{index}",
                split="holdout",
                mocap_xy=source,
                map_xy=expected_rotation @ source + expected_translation,
            )
        )

    result = fit_shared_se2(points, min_condition_ratio=1.0e-3)
    if not np.allclose(result.rotation, expected_rotation, atol=1.0e-12, rtol=0.0):
        raise CalibrationError("self-test failed to recover the synthetic rotation")
    if not np.allclose(result.translation, expected_translation, atol=1.0e-12, rtol=0.0):
        raise CalibrationError("self-test failed to recover the synthetic translation")
    if result.fit_stats.max_m > 1.0e-12 or result.holdout_stats.max_m > 1.0e-12:
        raise CalibrationError("self-test residual is unexpectedly nonzero")

    collinear = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    try:
        _validate_geometry(collinear, collinear.copy(), min_condition_ratio=1.0e-3)
    except CalibrationError:
        pass
    else:
        raise CalibrationError("self-test failed to reject collinear fit points")

    rotation, translation = _shared_values(result)
    base1 = {"/**": {"ros__parameters": {"rigid_body_name": "6", "keep": 1}}}
    base2 = {"/**": {"ros__parameters": {"rigid_body_name": "7", "keep": 2}}}
    candidate1 = _make_candidate(base1, rotation, translation)
    candidate2 = _make_candidate(base2, rotation, translation)
    p1 = candidate1["/**"]["ros__parameters"]
    p2 = candidate2["/**"]["ros__parameters"]
    if p1["rotation"] != p2["rotation"] or p1["translation"] != p2["translation"]:
        raise CalibrationError("self-test candidate shared transforms differ")
    if p1["keep"] != 1 or p2["keep"] != 2:
        raise CalibrationError("self-test modified a vehicle-specific parameter")

    print_report(result)
    print("SELF-TEST PASS")


def build_parser() -> argparse.ArgumentParser:
    try:
        from ament_index_python.packages import get_package_share_directory

        package_dir = Path(get_package_share_directory("mocap_localization"))
    except (ImportError, LookupError):
        # Source-tree fallback keeps --self-test and direct development use ROS-free.
        package_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Fit one shared map<-mocap SE(2) transform from base-corrected "
            "survey points and write two candidate params files."
        )
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run a synthetic fit and safety test",
    )
    parser.add_argument("--csv", type=Path, help="survey CSV with fit and holdout rows")
    parser.add_argument(
        "--params1",
        type=Path,
        default=package_dir / "params" / "params.yaml",
        help="first source params YAML (read-only)",
    )
    parser.add_argument(
        "--params2",
        type=Path,
        default=package_dir / "params" / "params2.yaml",
        help="second source params YAML (read-only)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="separate directory for candidate params.yaml and params2.yaml",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace existing candidate files; source params remain protected",
    )
    parser.add_argument(
        "--min-condition-ratio",
        type=float,
        default=1.0e-3,
        help="minimum minor/major singular-value ratio for fit geometry (default: 1e-3)",
    )
    parser.add_argument(
        "--max-fit-rmse-m",
        type=float,
        help="optional fit RMSE gate; no candidates are written when exceeded",
    )
    parser.add_argument(
        "--max-holdout-rmse-m",
        type=float,
        help="optional holdout RMSE gate; no candidates are written when exceeded",
    )
    return parser


def _validate_optional_limit(value: float | None, name: str) -> None:
    if value is not None and (not math.isfinite(value) or value < 0.0):
        raise CalibrationError(f"{name} must be a finite, non-negative number")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.self_test:
            run_self_test()
            return 0
        if args.csv is None:
            parser.error("--csv is required unless --self-test is used")
        if args.output_dir is None:
            parser.error("--output-dir is required unless --self-test is used")

        _validate_optional_limit(args.max_fit_rmse_m, "max_fit_rmse_m")
        _validate_optional_limit(args.max_holdout_rmse_m, "max_holdout_rmse_m")
        points = load_survey_csv(args.csv)
        result = fit_shared_se2(points, args.min_condition_ratio)
        print_report(result)

        if args.max_fit_rmse_m is not None and result.fit_stats.rmse_m > args.max_fit_rmse_m:
            raise CalibrationError(
                f"fit RMSE {result.fit_stats.rmse_m:.6f} m exceeds gate "
                f"{args.max_fit_rmse_m:.6f} m"
            )
        if (
            args.max_holdout_rmse_m is not None
            and result.holdout_stats.rmse_m > args.max_holdout_rmse_m
        ):
            raise CalibrationError(
                f"holdout RMSE {result.holdout_stats.rmse_m:.6f} m exceeds gate "
                f"{args.max_holdout_rmse_m:.6f} m"
            )

        output1, output2 = write_candidates(
            args.params1,
            args.params2,
            args.output_dir,
            result,
            args.overwrite,
        )
        print(f"candidate 1: {output1}")
        print(f"candidate 2: {output2}")
        print("source params were not modified")
        return 0
    except (CalibrationError, OSError, yaml.YAMLError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
