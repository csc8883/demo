from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from waypoint_planning.route_planner import plan_route_from_waypoints, validate_route_safety


DEFAULT_CLEARANCE_MATRIX: Tuple[Tuple[float, float, float, float], ...] = (
    (3.0, 5.0, 1.5, 2.5),
    (4.0, 6.0, 2.0, 3.0),
    (6.0, 10.0, 3.0, 5.0),
    (8.0, 12.0, 4.0, 6.0),
)


METRIC_COLUMNS = [
    "case_id",
    "repeat",
    "status",
    "passed",
    "tower_clearance_m",
    "wire_clearance_m",
    "task_tower_clearance_m",
    "task_wire_clearance_m",
    "elapsed_s",
    "total_length_m",
    "route_point_count",
    "task_point_count",
    "auxiliary_point_count",
    "detour_point_count",
    "task_corridor_auxiliary_count",
    "astar_segment_count",
    "astar_fallback_count",
    "min_tower_distance_m",
    "min_wire_distance_m",
    "min_conductor_no_fly_clearance_m",
    "conductor_no_fly_volume_count",
    "conductor_no_fly_violation_count",
    "violation_count",
    "task_violation_count",
    "auxiliary_violation_count",
    "segment_violation_count",
    "output_file",
    "error",
]


def _resolve_single_path(path_text: str | None, glob_pattern: str, label: str) -> Path:
    if path_text:
        path = Path(path_text)
        if not path.is_absolute():
            path = ROOT / path
        if not path.exists():
            raise FileNotFoundError(f"{label} not found: {path}")
        return path

    matches = sorted(ROOT.glob(glob_pattern))
    if not matches:
        raise FileNotFoundError(f"No {label} matched: {glob_pattern}")
    return matches[0]


def _parse_clearance(value: str) -> Tuple[float, float, float, float]:
    parts = [part.strip() for part in value.split(",") if part.strip()]
    if len(parts) not in (2, 4):
        raise argparse.ArgumentTypeError(
            "clearance must be tower,wire or tower,wire,task_tower,task_wire"
        )
    numbers = tuple(float(part) for part in parts)
    if any(number <= 0 for number in numbers):
        raise argparse.ArgumentTypeError("clearance values must be positive")
    if len(numbers) == 2:
        tower, wire = numbers
        return tower, wire, max(0.5, tower * 0.5), max(0.5, wire * 0.5)
    return numbers  # type: ignore[return-value]


def _format_float(value: Any) -> Any:
    if isinstance(value, float):
        if math.isinf(value) or math.isnan(value):
            return None
        return round(value, 3)
    return value


def _route_metric_row(
    case_id: str,
    repeat: int,
    clearances: Sequence[float],
    elapsed_s: float,
    output_path: Path,
    output_payload: Dict[str, Any],
    safety: Dict[str, Any],
) -> Dict[str, Any]:
    planning = output_payload.get("route_planning") or {}
    tower_clearance, wire_clearance, task_tower_clearance, task_wire_clearance = clearances
    row = {
        "case_id": case_id,
        "repeat": repeat,
        "status": "ok",
        "passed": bool(safety.get("passed")),
        "tower_clearance_m": tower_clearance,
        "wire_clearance_m": wire_clearance,
        "task_tower_clearance_m": task_tower_clearance,
        "task_wire_clearance_m": task_wire_clearance,
        "elapsed_s": round(elapsed_s, 3),
        "total_length_m": output_payload.get("totalLen"),
        "route_point_count": planning.get("route_point_count"),
        "task_point_count": planning.get("task_point_count"),
        "auxiliary_point_count": planning.get("auxiliary_point_count"),
        "detour_point_count": planning.get("detour_point_count"),
        "task_corridor_auxiliary_count": planning.get("task_corridor_auxiliary_count"),
        "astar_segment_count": planning.get("astar_segment_count"),
        "astar_fallback_count": planning.get("astar_fallback_count"),
        "min_tower_distance_m": safety.get("min_tower_distance_m"),
        "min_wire_distance_m": safety.get("min_wire_distance_m"),
        "min_conductor_no_fly_clearance_m": safety.get("min_conductor_no_fly_clearance_m"),
        "conductor_no_fly_volume_count": safety.get("conductor_no_fly_volume_count"),
        "conductor_no_fly_violation_count": safety.get("conductor_no_fly_violation_count"),
        "violation_count": safety.get("violation_count"),
        "task_violation_count": safety.get("task_violation_count"),
        "auxiliary_violation_count": safety.get("auxiliary_violation_count"),
        "segment_violation_count": safety.get("segment_violation_count"),
        "output_file": str(output_path.relative_to(ROOT)),
        "error": "",
    }
    return {key: _format_float(value) for key, value in row.items()}


def _failure_row(
    case_id: str,
    repeat: int,
    clearances: Sequence[float],
    elapsed_s: float,
    error: Exception,
) -> Dict[str, Any]:
    tower_clearance, wire_clearance, task_tower_clearance, task_wire_clearance = clearances
    row = {
        "case_id": case_id,
        "repeat": repeat,
        "status": "failed",
        "passed": False,
        "tower_clearance_m": tower_clearance,
        "wire_clearance_m": wire_clearance,
        "task_tower_clearance_m": task_tower_clearance,
        "task_wire_clearance_m": task_wire_clearance,
        "elapsed_s": round(elapsed_s, 3),
        "error": str(error),
    }
    for column in METRIC_COLUMNS:
        row.setdefault(column, "")
    return row


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=METRIC_COLUMNS)
        writer.writeheader()
        writer.writerows({column: row.get(column, "") for column in METRIC_COLUMNS} for row in rows)


def _write_markdown(path: Path, rows: Sequence[Dict[str, Any]], waypoint_path: Path, voxel_path: Path) -> None:
    columns = [
        "case_id",
        "repeat",
        "status",
        "passed",
        "tower_clearance_m",
        "wire_clearance_m",
        "total_length_m",
        "route_point_count",
        "detour_point_count",
        "astar_segment_count",
        "min_tower_distance_m",
        "min_wire_distance_m",
        "min_conductor_no_fly_clearance_m",
        "conductor_no_fly_violation_count",
        "violation_count",
        "elapsed_s",
        "error",
    ]
    lines = [
        "# Route Clearance Matrix Test",
        "",
        f"- waypoint: `{waypoint_path.relative_to(ROOT)}`",
        f"- voxel: `{voxel_path.relative_to(ROOT)}`",
        f"- generated_at: `{datetime.now().isoformat(timespec='seconds')}`",
        "",
        "|" + "|".join(columns) + "|",
        "|" + "|".join("---" for _ in columns) + "|",
    ]
    for row in rows:
        lines.append("|" + "|".join(str(row.get(column, "")) for column in columns) + "|")
    lines.append("")
    lines.append("Full JSON and CSV reports are in the same directory.")
    path.write_text("\n".join(lines), encoding="utf-8")


def run_matrix(
    waypoint_path: Path,
    voxel_path: Path,
    output_root: Path,
    clearances: Iterable[Tuple[float, float, float, float]],
    repeats: int,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    output_root.mkdir(parents=True, exist_ok=True)

    for case_index, clearance in enumerate(clearances, start=1):
        case_id = (
            f"c{case_index:02d}_tower{clearance[0]:g}_wire{clearance[1]:g}"
            f"_task{clearance[2]:g}_{clearance[3]:g}"
        )
        for repeat in range(1, repeats + 1):
            run_dir = output_root / case_id / f"repeat_{repeat:02d}"
            started = time.perf_counter()
            try:
                route_path, route_payload = plan_route_from_waypoints(
                    waypoint_path=waypoint_path,
                    output_dir=run_dir,
                    voxel_path=voxel_path,
                    clearance_m=clearance[0],
                    wire_clearance_m=clearance[1],
                    task_tower_clearance_m=clearance[2],
                    task_wire_clearance_m=clearance[3],
                )
                safety = validate_route_safety(
                    route_path=route_path,
                    voxel_path=voxel_path,
                    tower_clearance_m=clearance[0],
                    wire_clearance_m=clearance[1],
                    task_tower_clearance_m=clearance[2],
                    task_wire_clearance_m=clearance[3],
                )
                elapsed_s = time.perf_counter() - started
                rows.append(_route_metric_row(case_id, repeat, clearance, elapsed_s, route_path, route_payload, safety))
            except Exception as exc:
                elapsed_s = time.perf_counter() - started
                rows.append(_failure_row(case_id, repeat, clearance, elapsed_s, exc))
            latest = rows[-1]
            print(
                f"{latest['case_id']} repeat={repeat} status={latest['status']} "
                f"passed={latest['passed']} len={latest.get('total_length_m', '')} "
                f"points={latest.get('route_point_count', '')} violations={latest.get('violation_count', '')}"
            )
    return rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run repeated route planning tests for multiple safety distances.")
    parser.add_argument("--waypoint", help="Waypoint JSON path. Defaults to the first user waypoint JSON.")
    parser.add_argument("--voxel", help="Voxel NPZ path. Defaults to the matching #030 user voxel file.")
    parser.add_argument("--output-dir", default="userdata/user/test_reports/route_clearance_matrix")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument(
        "--fail-on-unsuccessful",
        action="store_true",
        help="Return exit code 1 when any run fails or violates the requested clearances.",
    )
    parser.add_argument(
        "--clearance",
        action="append",
        type=_parse_clearance,
        help="Add one clearance case: tower,wire or tower,wire,task_tower,task_wire. Can be repeated.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.repeats < 1:
        raise ValueError("--repeats must be >= 1")

    waypoint_path = _resolve_single_path(args.waypoint, "userdata/user/waypoint/*.json", "waypoint JSON")
    voxel_path = _resolve_single_path(args.voxel, "userdata/user/voxel/*030_voxel.npz", "voxel NPZ")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = ROOT / args.output_dir / timestamp
    clearances = tuple(args.clearance or DEFAULT_CLEARANCE_MATRIX)

    rows = run_matrix(waypoint_path, voxel_path, output_root, clearances, args.repeats)

    json_path = output_root / "metrics.json"
    csv_path = output_root / "metrics.csv"
    md_path = output_root / "metrics.md"
    json_path.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_csv(csv_path, rows)
    _write_markdown(md_path, rows, waypoint_path, voxel_path)

    passed = sum(1 for row in rows if row.get("passed") is True)
    print(f"reports: {json_path.relative_to(ROOT)}, {csv_path.relative_to(ROOT)}, {md_path.relative_to(ROOT)}")
    print(f"summary: {passed}/{len(rows)} runs passed")
    return 1 if args.fail_on_unsuccessful and passed != len(rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
