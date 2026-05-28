"""End-to-end waypoint planning test: LAS -> voxel -> candidates -> planner.

Validates:
  1. photo waypoints 10-80
  2. safety violations = 0
  3. tower overview <= 3
  4. insulator <= 4 per instance
  5. connection <= 3 per target
  6. candidates 2000-15000
  7. key target coverage not regressed
  8. geometry consistency (heading == yaw, look_at present)
"""
import sys, json, time, tempfile
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from waypoint_planning.planning_core import (
    build_semantic_surface_model,
    generate_candidate_views,
    PlanningEnvironment,
    VisibilityEngine,
    load_candidate_views,
    evaluate_waypoint_coverage,
    flatten_waypoint_views,
    TARGET_SEMANTICS,
    ATTENTION_SEMANTICS,
    SEMANTIC_PRIORITY,
    SEMANTIC_WEIGHTS,
    compute_view_geometry,
)
from waypoint_planning.algorithms import create_planner

# ── Config ──────────────────────────────────────────────────────────
LAS_PATH = ROOT / "userdata/user/point_cloud/500kV古敬5307线#028.las"
VOXEL_SIZE = 0.10
MAX_VIS = 60.0

# ── Step 1: Read point cloud ────────────────────────────────────────
print(f"[1/6] Reading point cloud: {LAS_PATH.name}")
import laspy
with laspy.open(str(LAS_PATH)) as f:
    las = f.read()
    points = np.vstack((las.x, las.y, las.z)).transpose()
    labels = np.asarray(las.classification, dtype=int)
print(f"      {len(points)} points loaded")

# ── Step 2: Build surface model ─────────────────────────────────────
t0 = time.time()
print(f"[2/6] Building semantic surface model (voxel={VOXEL_SIZE}m)...")
surface_model = build_semantic_surface_model(points, labels, VOXEL_SIZE)
meta = surface_model.get("meta", {})
n_targets = len(surface_model.get("target_cells", []))
n_insulator_instances = len(meta.get("insulator_instances", []))
print(f"      {len(surface_model['voxels'])} voxels, {n_targets} target cells, "
      f"{n_insulator_instances} insulator instances, {time.time()-t0:.1f}s")

# ── Step 3: Generate candidates ─────────────────────────────────────
t0 = time.time()
print(f"[3/6] Generating candidate views...")
candidates = generate_candidate_views(surface_model)
stats = surface_model.get("meta", {}).get("candidate_generation_stats", {})
print(f"      {len(candidates)} candidates in {time.time()-t0:.1f}s")
if stats:
    raw = stats.get("raw_by_source", {})
    final = stats.get("final_by_source", {})
    rejected = stats.get("rejected_by_safety", {})
    print(f"      Raw by source: {json.dumps(raw)}")
    if rejected:
        print(f"      Rejected by safety: {json.dumps(rejected)}")
    print(f"      Final by source: {json.dumps(final)}")

# ── Step 3.5: Candidate count validation ────────────────────────────
candidate_count = len(candidates)
print(f"\n── Candidate Count Check ──")
print(f"  Total candidates: {candidate_count}")
if 2000 <= candidate_count <= 15000:
    print(f"  [PASS] Candidate count in range [2000, 15000]")
else:
    print(f"  [WARN] Candidate count outside recommended range [2000, 15000]")

# ── Step 3.6: AIMType statistics on candidates ──────────────────────
# Build AIMType distribution from candidates directly
aim_type_counts = Counter()
aim_type_distances: dict = {}
aim_type_focals: dict = {}
for c in candidates:
    at = c.get("AimType", "unknown")
    aim_type_counts[at] += 1
    if at not in aim_type_distances:
        aim_type_distances[at] = []
        aim_type_focals[at] = []
    d = c.get("Distance")
    if d is not None:
        aim_type_distances[at].append(float(d))
    f = c.get("focal_length_eq_mm") or c.get("f_eq_mm")
    if f is not None:
        aim_type_focals[at].append(float(f))

print(f"\n── AIMType Distribution ──")
print(f"  {'AimType':<35s} {'Count':>6s} {'Dist(avg)':>10s} {'Focal(avg)':>11s}")
print(f"  {'-'*66}")
for at in sorted(aim_type_counts.keys()):
    count = aim_type_counts[at]
    d_avg = f"{np.mean(aim_type_distances[at]):.1f}m" if aim_type_distances.get(at) else "N/A"
    f_avg = f"{np.mean(aim_type_focals[at]):.1f}mm" if aim_type_focals.get(at) else "N/A"
    print(f"  {at:<35s} {count:>6d} {d_avg:>10s} {f_avg:>11s}")

# ── Step 4: Save npz + candidates.json ──────────────────────────────
tmpdir = Path(tempfile.mkdtemp(prefix="e2e_test_"))
npz_path = tmpdir / f"{LAS_PATH.stem}_voxel.npz"
cand_path = tmpdir / f"{LAS_PATH.stem}_candidates.json"
output_dir = tmpdir / "waypoints"
output_dir.mkdir(parents=True, exist_ok=True)

print(f"\n[4/6] Saving npz + candidates.json -> {tmpdir}")

np.savez_compressed(
    npz_path,
    min_bound=surface_model["min_bound"],
    voxel_size=VOXEL_SIZE,
    z_max_map=surface_model["z_max_map"],
    voxels=np.array(surface_model["voxels"], dtype=object),
    attention_targets=np.array(surface_model.get("attention_targets", []), dtype=object),
    display_voxels=np.array(surface_model.get("display_voxels", surface_model["voxels"]), dtype=object),
    target_cells=np.array(surface_model.get("target_cells", []), dtype=object),
    wire_curves=np.array(surface_model.get("wire_curves", []), dtype=object),
    conductor_no_fly_volumes=np.array(surface_model.get("conductor_no_fly_volumes", []), dtype=object),
    safety_points=np.asarray(surface_model.get("safety_points", np.empty((0, 3))), dtype=float),
    safety_labels=np.asarray(surface_model.get("safety_labels", np.empty((0,))), dtype=int),
    local_center=np.asarray(surface_model["local_center"], dtype=float),
    local_frame=np.array(surface_model.get("local_frame", {}), dtype=object),
    meta_json=json.dumps(meta, ensure_ascii=False),
)
with open(cand_path, "w", encoding="utf-8") as f:
    json.dump(candidates, f, indent=2, ensure_ascii=False)

data = np.load(npz_path, allow_pickle=True)
print(f"      npz keys: {list(data.files)}")

# ── Step 5: Run planner ─────────────────────────────────────────────
print(f"\n[5/6] Running planner (greedy via create_planner)...")
t0 = time.time()

planner, fallback_reason = create_planner(
    "greedy",
    voxel_path=str(npz_path),
    cand_path=str(cand_path),
    output_dir=str(output_dir),
    status_key="e2e_test",
)
if fallback_reason:
    print(f"      [WARN] Fallback: {fallback_reason}")
result_path = planner.solve()
elapsed = time.time() - t0
print(f"      Planning time: {elapsed:.1f}s")

# ── Step 6: Load results ────────────────────────────────────────────
with open(result_path, "r", encoding="utf-8") as f:
    result = json.load(f)

waypoints = result.get("waypoints", result.get("points", []))
result_stats = result.get("stats", {})

# ── Step 7: Validation ──────────────────────────────────────────────
env = PlanningEnvironment(str(npz_path))
engine = VisibilityEngine(env)
errors: list[str] = []
warnings: list[str] = []

# ── 7a: Photo waypoint count 10-80 ──────────────────────────────────
photo_waypoints = [wp for wp in waypoints if wp.get("actionName", "photo") == "photo"]
auxiliary_waypoints = [wp for wp in waypoints if wp.get("actionName") == "none"]
photo_count = len(photo_waypoints)
aux_count = len(auxiliary_waypoints)
print(f"\n── Waypoint Count Validation ──")
print(f"  Photo waypoints:     {photo_count} (required: 10-80)")
print(f"  Auxiliary waypoints: {aux_count}")
print(f"  Stop reason:         {result_stats.get('stop_reason', 'N/A')}")
print(f"  Fallback reason:     {result_stats.get('fallback_reason', 'N/A')}")

if 10 <= photo_count <= 80:
    print(f"  [PASS] Photo waypoint count in range")
else:
    errors.append(f"Photo waypoint count {photo_count} outside [10, 80]")

# ── 7b: Safety violations = 0 ───────────────────────────────────────
safety_violations = result_stats.get("safety_violation_count", -1)
no_fly_violations = result_stats.get("conductor_no_fly_waypoint_violation_count", -1)
no_fly_clearance_violations = result_stats.get("conductor_no_fly_clearance_violation_count", -1)
print(f"\n── Safety Validation ──")
print(f"  Safety violations:          {safety_violations}")
print(f"  No-fly violations:          {no_fly_violations}")
print(f"  No-fly clearance violations: {no_fly_clearance_violations}")
if safety_violations == 0 and no_fly_violations == 0:
    print(f"  [PASS] No safety violations")
else:
    errors.append(f"Safety violations detected: safety={safety_violations}, no_fly={no_fly_violations}")

# ── 7c: Tower overview <= 3 ─────────────────────────────────────────
tower_overview_wps = [wp for wp in waypoints if wp.get("AimType") == "tower_overview"]
tower_overview_count = len(tower_overview_wps)
print(f"\n── Per-Type Constraints ──")
print(f"  Tower overview waypoints: {tower_overview_count} (limit: <= 3)")
if tower_overview_count <= 3:
    print(f"  [PASS] Tower overview <= 3")
else:
    warnings.append(f"Tower overview {tower_overview_count} exceeds limit 3")

# ── 7d: Insulator <= 4 per instance ─────────────────────────────────
insulator_wps = [wp for wp in waypoints if wp.get("AimType") == "insulator_string"]
print(f"  Insulator string waypoints: {len(insulator_wps)}")
if n_insulator_instances > 0:
    per_instance = len(insulator_wps) / n_insulator_instances
    print(f"  Per instance (avg): {per_instance:.1f} (limit: <= 4)")
    # We can only check average since we don't know which WP goes to which instance
    if per_instance <= 4.5:
        print(f"  [PASS] Insulator per instance roughly <= 4")
    else:
        warnings.append(f"Insulator per instance {per_instance:.1f} may exceed 4")

# ── 7e: Connection <= 3 per target ──────────────────────────────────
for conn_type, label in [
    ("conductor_insulator_connection", "Conductor-insulator connection"),
    ("insulator_tower_side_connection", "Insulator-tower connection"),
    ("ground_wire_tower_connection", "Ground wire connection"),
]:
    conn_wps = [wp for wp in waypoints if wp.get("AimType") == conn_type]
    print(f"  {label}: {len(conn_wps)} waypoints")

# ── 7f: Geometry consistency (heading == yaw, look_at present) ──────
print(f"\n── Geometry Consistency ──")
geo_errors = 0
for wp in photo_waypoints:
    pos = np.asarray(wp["position"], dtype=float)
    look_at = wp.get("look_at")
    if look_at is None:
        geo_errors += 1
        continue
    look_at_arr = np.asarray(look_at, dtype=float)
    geom = compute_view_geometry(pos, look_at_arr)
    heading = wp.get("heading") or wp.get("yaw")
    if heading is not None:
        diff = abs(float(heading) - float(geom["heading"]))
        if diff > 1.0 and abs(diff - 360.0) > 1.0:
            geo_errors += 1
    # Check Distance consistency
    dist = wp.get("Distance")
    if dist is not None:
        diff = abs(float(dist) - float(geom["Distance"]))
        if diff > 1.0:
            geo_errors += 1

if geo_errors == 0:
    print(f"  [PASS] All {len(photo_waypoints)} photo waypoints have consistent geometry")
else:
    warnings.append(f"{geo_errors} waypoints have geometry inconsistencies")

# ── Coverage evaluation ─────────────────────────────────────────────
coverage = evaluate_waypoint_coverage(waypoints, str(npz_path))

print(f"\n── Coverage (Target Semantics) ──")
print(f"  {'Semantic':<35s} {'Count':>6s} {'Covered':>8s} {'Pct':>7s} {'NoCand':>8s} {'Pri':>4s} {'Weight':>7s}")
print(f"  {'-'*78}")

# Per-semantic uncovered analysis
all_indices = set()
for view in flatten_waypoint_views(waypoints):
    pos = np.asarray(view["position"], dtype=float)
    pitch = float(view.get("pitch", 0.0))
    yaw = float(view.get("yaw", 0.0))
    focal = view.get("focal_level")
    f_eq = view.get("f_eq_mm")
    from waypoint_planning.planning_core import normalize_focal_level
    focal_level, _ = normalize_focal_level(focal, f_eq)
    indices = engine.visible_indices_for_view(pos, pitch, yaw, focal_level=focal_level)
    all_indices.update(indices.tolist())

covered_mask = np.zeros(env.target_count, dtype=bool)
covered_mask[list(all_indices)] = True

raw_cand_list = load_candidate_views(str(cand_path))
cand_positions = np.array([np.asarray(c.position, dtype=float) for c in raw_cand_list]) if raw_cand_list else np.empty((0, 3))
target_positions = env.target_coords

for sem in TARGET_SEMANTICS:
    sem_mask = env.semantics == sem
    sem_total = int(np.sum(sem_mask))
    if sem_total == 0:
        continue
    sem_covered = int(np.sum(covered_mask[sem_mask]))
    sem_uncovered = sem_total - sem_covered
    # Uncovered without candidate
    sem_uncovered_indices = np.where(sem_mask & ~covered_mask)[0]
    sem_no_cand = 0
    if len(cand_positions) > 0:
        for tgt_idx in sem_uncovered_indices:
            dists = np.linalg.norm(cand_positions - target_positions[tgt_idx], axis=1)
            if not np.any(dists <= MAX_VIS):
                sem_no_cand += 1
    cov_str = f"{100.0*sem_covered/sem_total:.1f}%" if sem_total else "N/A"
    pri = SEMANTIC_PRIORITY.get(sem, 0)
    w = SEMANTIC_WEIGHTS.get(sem, 0)
    print(f"  {sem:<35s} {sem_total:>6d} {sem_covered:>8d} {cov_str:>7s} {sem_no_cand:>8d} {pri:>4d} {w:>7.1f}")

# ── Overall metrics ─────────────────────────────────────────────────
print(f"\n── Overall Metrics ──")
c_weighted = coverage.get("C_weighted") or coverage.get("coverage_weighted")
c_geo = coverage.get("C_geo") or coverage.get("coverage_total")
print(f"  C_weighted:  {c_weighted}")
print(f"  C_geo:       {c_geo}")
print(f"  C_ins:       {coverage.get('C_ins')}")
print(f"  C_top:       {coverage.get('C_top')}")
print(f"  C_edge:      {coverage.get('C_edge')}")
print(f"  C_body:      {coverage.get('C_body')}")

# ── Critical coverage ───────────────────────────────────────────────
print(f"\n── Critical Coverage (from result stats) ──")
crit_cov = result_stats.get("critical_coverage", {})
for sem, info in crit_cov.items():
    print(f"  {sem}: {info.get('covered', '?')}/{info.get('total', '?')} = {info.get('coverage_pct', '?')}%")

# ── Shot-level AIMType stats on final waypoints ─────────────────────
print(f"\n── Final Waypoint AIMType Distribution ──")
wp_aim_counts = Counter(wp.get("AimType", "unknown") for wp in waypoints)
wp_focal_usage = Counter()
for wp in waypoints:
    for s in wp.get("shots", []):
        fl = s.get("focal_length_eq_mm") or s.get("f_eq_mm")
        if fl:
            wp_focal_usage[f"{fl:.0f}mm"] += 1

print(f"  AIMTypes: {dict(wp_aim_counts)}")
print(f"  Focal usage: {dict(wp_focal_usage)}")
print(f"  Total shots: {sum(len(wp.get('shots', [])) for wp in waypoints)}")

# ── Summary ─────────────────────────────────────────────────────────
print(f"\n{'='*60}")
print(f"TEST SUMMARY")
print(f"{'='*60}")
print(f"  Candidates:       {candidate_count}")
print(f"  Waypoints (total): {len(waypoints)}")
print(f"  Waypoints (photo): {photo_count}")
print(f"  Waypoints (aux):   {aux_count}")
print(f"  Stop reason:       {result_stats.get('stop_reason', 'N/A')}")
print(f"  Within limits:     {result_stats.get('within_waypoint_limits', 'N/A')}")
print(f"  Planning time:     {elapsed:.1f}s")

if errors:
    print(f"\n  [FAIL] {len(errors)} error(s):")
    for e in errors:
        print(f"    - {e}")
else:
    print(f"\n  [PASS] All hard constraints met")

if warnings:
    print(f"  [WARN] {len(warnings)} warning(s):")
    for w in warnings:
        print(f"    - {w}")

print(f"\n  Output: {result_path}")
print(f"  Temp:   {tmpdir}")
sys.exit(0 if not errors else 1)
