from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from pydantic import AliasChoices, BaseModel, ConfigDict, Field, field_validator, model_validator


def _read_json_with_fallback(json_path: Path) -> Dict[str, Any]:
    """Read a JSON file using the encodings seen in manual route exports."""
    raw = Path(json_path).read_bytes()
    last_error: Optional[Exception] = None
    for encoding in ("utf-8-sig", "utf-8", "gb18030", "gbk"):
        try:
            data = json.loads(raw.decode(encoding))
            if not isinstance(data, dict):
                raise ValueError("JSON root must be an object")
            return data
        except Exception as exc:
            last_error = exc
    raise ValueError(f"Unable to parse JSON file {json_path}: {last_error}")


def _normalize_xyz(value: Any) -> Optional[List[float]]:
    """Convert a coordinate-like value to a three-number list."""
    if value is None:
        return None
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
    else:
        parts = list(value)
    if len(parts) < 3:
        raise ValueError("coordinate must contain at least three values")
    return [float(part) for part in parts[:3]]


class ExtraAllowedModel(BaseModel):
    model_config = ConfigDict(extra="allow", populate_by_name=True)


class WaypointConstraints(ExtraAllowedModel):
    min_waypoints: Optional[int] = None
    max_waypoints: Optional[int] = None
    max_total_shots: Optional[int] = None
    max_shots_per_waypoint: Optional[int] = None
    safety_distance_m: float = 5.0
    safety_scope: str = "all_physical"
    conductor_no_fly_enabled: bool = True
    conductor_no_fly_extent_margin_m: float = 10.0
    conductor_no_fly_min_length_m: float = 80.0
    conductor_no_fly_boundary_tolerance_m: float = 0.3
    conductor_no_fly_clearance_m: Optional[float] = None
    manual_ratio_min: float = 0.70
    manual_ratio_max: float = 0.70
    single_layer_episodes: int = 10
    hierarchical_episodes: int = 10
    target_semantics: List[str] = Field(default_factory=list)
    exclude_semantics: List[str] = Field(default_factory=list)


class WaypointPlanningInput(ExtraAllowedModel):
    voxel_path: Path
    candidate_path: Path = Field(validation_alias=AliasChoices("candidate_path", "cand_path"))
    output_dir: Path
    planner_key: Optional[str] = None
    planner_name: Optional[str] = None
    status_key: str = "rl"
    manual_route_path: Optional[Path] = None
    constraints: WaypointConstraints = Field(default_factory=WaypointConstraints)

    @field_validator("voxel_path", "candidate_path", "manual_route_path")
    @classmethod
    def validate_existing_input_path(cls, value: Optional[Path]) -> Optional[Path]:
        """Require planner input files to exist before planning starts."""
        if value is None:
            return None
        path = Path(value)
        if not path.exists():
            raise ValueError(f"path does not exist: {path}")
        return path

    @field_validator("output_dir")
    @classmethod
    def normalize_output_dir(cls, value: Path) -> Path:
        """Normalize the planner output directory without creating it."""
        return Path(value)

    def to_summary(self) -> "PlanningInputSummary":
        """Build the compact input summary written to WaypointResult."""
        return PlanningInputSummary(
            voxel_filename=self.voxel_path.name,
            candidate_filename=self.candidate_path.name,
            output_dir=str(self.output_dir),
            planner_key=self.planner_key,
            planner_name=self.planner_name,
            status_key=self.status_key,
            manual_route_filename=self.manual_route_path.name if self.manual_route_path else None,
            constraints=self.constraints.model_dump(mode="json", exclude_none=True),
        )


class PlanningInputSummary(ExtraAllowedModel):
    voxel_filename: str
    candidate_filename: str
    output_dir: str
    planner_key: Optional[str] = None
    planner_name: Optional[str] = None
    status_key: str = "rl"
    manual_route_filename: Optional[str] = None
    constraints: Dict[str, Any] = Field(default_factory=dict)


class ManualRoutePoint(ExtraAllowedModel):
    SerialNumber: Optional[str] = None
    AimType: Optional[str] = None
    actionName: Optional[str] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    latitude_Aim: Optional[float] = None
    longitude_Aim: Optional[float] = None
    altitude_Aim: Optional[float] = None
    heading: Optional[float] = None
    pitch: Optional[float] = None
    Distance: Optional[float] = None
    isLast: Optional[int] = None
    linename: Optional[str] = None
    MatchImg: Optional[str] = None


class TowerContext(ExtraAllowedModel):
    towername: Optional[str] = None
    tower_latitude: Optional[float] = None
    tower_longitude: Optional[float] = None
    PlaneCenterPoint: Optional[str] = None
    PlanemTowerHight: Optional[float] = None
    PlaneAngle: Optional[float] = None
    PlaneLen: Optional[float] = None
    PlaneTowerR: Optional[float] = None
    PlaneWide: Optional[float] = None
    points: List[ManualRoutePoint] = Field(default_factory=list)

    def center_utm(self) -> Optional[List[float]]:
        """Return the tower center parsed from PlaneCenterPoint when present."""
        if not self.PlaneCenterPoint:
            return None
        return _normalize_xyz(self.PlaneCenterPoint)


class ManualRouteContext(ExtraAllowedModel):
    CoordinateSystem: Optional[str] = None
    FlyType: Optional[str] = None
    UTM: Optional[str] = None
    linename: Optional[str] = None
    sCameraType: Optional[str] = None
    tasktype: Optional[str] = None
    totalLen: Optional[float] = None
    towercount: Optional[int] = None
    vLevel: Optional[str] = None
    version: Optional[str] = None
    MinPts: List[Any] = Field(default_factory=list)
    towers: List[TowerContext] = Field(default_factory=list)
    points: List[ManualRoutePoint] = Field(default_factory=list)


class WaypointShot(ExtraAllowedModel):
    shot_id: Optional[str] = None
    yaw: Optional[float] = None
    pitch: Optional[float] = None
    focal_level: Optional[str] = None
    f_eq_mm: Optional[float] = None
    hfov_deg: Optional[float] = None
    vfov_deg: Optional[float] = None
    semantic_focus: Optional[str] = None


class Waypoint(ExtraAllowedModel):
    id: int
    position: Optional[List[float]] = None
    pos_utm: Optional[List[float]] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    focal_level: Optional[str] = None
    f_eq_mm: Optional[float] = None
    position_id: Optional[int] = None
    safety_distance_m: Optional[float] = None
    shot_count: Optional[int] = None
    action: Optional[str] = None
    shots: List[WaypointShot] = Field(default_factory=list)

    @field_validator("position", "pos_utm", mode="before")
    @classmethod
    def validate_coordinate(cls, value: Any) -> Optional[List[float]]:
        """Validate waypoint UTM coordinates."""
        return _normalize_xyz(value)

    @model_validator(mode="after")
    def sync_position_aliases(self) -> "Waypoint":
        """Keep legacy position and pos_utm fields available together."""
        if self.position is None and self.pos_utm is not None:
            self.position = list(self.pos_utm)
        if self.pos_utm is None and self.position is not None:
            self.pos_utm = list(self.position)
        if self.shot_count is None:
            self.shot_count = len(self.shots) if self.shots else 1
        return self


class WaypointStats(ExtraAllowedModel):
    coverage: Optional[float] = None
    count: Optional[int] = 0
    waypoint_count: Optional[int] = None
    shot_count: Optional[int] = 0
    path_length: Optional[float] = None
    avg_segment_length: Optional[float] = None
    altitude_avg: Optional[float] = None
    altitude_min: Optional[float] = None
    altitude_max: Optional[float] = None
    coverage_per_waypoint: Optional[float] = None
    coverage_per_shot: Optional[float] = None
    C_connection_attention: Optional[float] = None
    C_conductor_insulator_connection: Optional[float] = None
    C_wire_insulator_connection: Optional[float] = None
    C_insulator_tower_side_connection: Optional[float] = None
    C_ground_wire_tower_connection: Optional[float] = None
    C_tower_base_connection: Optional[float] = None
    manual_waypoint_count: Optional[int] = None
    manual_waypoint_min: Optional[int] = None
    manual_waypoint_max: Optional[int] = None
    manual_waypoint_ratio: Optional[float] = None
    manual_waypoint_limit_overridden: Optional[bool] = None
    tower_body_ring_count: Optional[int] = None
    safety_violation_count: Optional[int] = None
    min_safety_distance_m: Optional[float] = None
    conductor_no_fly_volume_count: Optional[int] = None
    conductor_no_fly_clearance_m: Optional[float] = None
    conductor_no_fly_waypoint_violation_count: Optional[int] = None
    conductor_no_fly_clearance_violation_count: Optional[int] = None
    min_conductor_no_fly_clearance_m: Optional[float] = None
    conductor_no_fly_source: Optional[str] = None
    focal_usage: Dict[str, int] = Field(default_factory=dict)
    supplement_counts: Dict[str, int] = Field(default_factory=dict)
    key_cluster_counts: Dict[str, Dict[str, int]] = Field(default_factory=dict)
    key_cluster_shortfalls: Dict[str, Dict[str, int]] = Field(default_factory=dict)

    @model_validator(mode="after")
    def sync_counts(self) -> "WaypointStats":
        """Keep legacy count and waypoint_count metrics in sync."""
        if self.count is None and self.waypoint_count is not None:
            self.count = self.waypoint_count
        if self.waypoint_count is None:
            self.waypoint_count = self.count
        return self


class TowerWaypointResult(ExtraAllowedModel):
    tower_index: int
    towername: Optional[str] = None
    tower_context: TowerContext
    waypoints: List[Waypoint] = Field(default_factory=list)


class WaypointResult(ExtraAllowedModel):
    algorithm: str
    method: str
    method_name: str
    status: str
    config_name: str
    stats: WaypointStats = Field(default_factory=WaypointStats)
    waypoints: List[Waypoint] = Field(default_factory=list)
    uncovered_summary: Dict[str, Any] = Field(default_factory=dict)
    uncovered_voxels: List[Dict[str, Any]] = Field(default_factory=list)
    planning_input: Optional[PlanningInputSummary] = None
    route_context: Optional[ManualRouteContext] = None
    tower_results: List[TowerWaypointResult] = Field(default_factory=list)


def load_manual_route_context(json_path: Path) -> ManualRouteContext:
    """Load a manual route JSON file into a loss-preserving Pydantic model."""
    return ManualRouteContext.model_validate(_read_json_with_fallback(Path(json_path)))


def assign_waypoints_to_towers(
    waypoints: Sequence[Waypoint],
    route_context: Optional[ManualRouteContext],
) -> List[TowerWaypointResult]:
    """Assign generated waypoints to manual-route towers by nearest tower center."""
    if not route_context or not route_context.towers:
        return []

    buckets: List[List[Waypoint]] = [[] for _ in route_context.towers]
    if len(route_context.towers) == 1:
        buckets[0].extend(waypoints)
    else:
        centers: List[Optional[List[float]]] = []
        for tower in route_context.towers:
            try:
                centers.append(tower.center_utm())
            except ValueError:
                centers.append(None)
        usable = [(idx, center) for idx, center in enumerate(centers) if center is not None]
        if usable:
            for waypoint in waypoints:
                position = waypoint.position or waypoint.pos_utm
                if position is None:
                    continue
                best_index = min(
                    usable,
                    key=lambda item: sum((float(position[i]) - float(item[1][i])) ** 2 for i in range(3)),
                )[0]
                buckets[best_index].append(waypoint)

    return [
        TowerWaypointResult(
            tower_index=index + 1,
            towername=tower.towername,
            tower_context=tower,
            waypoints=buckets[index],
        )
        for index, tower in enumerate(route_context.towers)
    ]


def build_waypoint_result(
    payload: Mapping[str, Any],
    planning_input: Optional[WaypointPlanningInput] = None,
    manual_route_path: Optional[Path] = None,
    route_context: Optional[ManualRouteContext] = None,
) -> WaypointResult:
    """Build the standard WaypointResult while keeping legacy payload keys."""
    data = dict(payload)
    if planning_input is not None:
        summary = planning_input.to_summary()
        if manual_route_path is not None and summary.manual_route_filename is None:
            summary.manual_route_filename = Path(manual_route_path).name
        data["planning_input"] = summary
        manual_route_path = manual_route_path or planning_input.manual_route_path
    if route_context is None and manual_route_path is not None:
        route_context = load_manual_route_context(Path(manual_route_path))
    if route_context is not None:
        data["route_context"] = route_context

    result = WaypointResult.model_validate(data)
    result.tower_results = assign_waypoints_to_towers(result.waypoints, result.route_context)
    return result
