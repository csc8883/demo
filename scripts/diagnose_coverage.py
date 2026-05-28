"""Comprehensive diagnostic: per-semantic target/candidate/visibility/scoring analysis."""
import sys, json, tempfile
from collections import Counter, defaultdict
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
    CandidateViewpoint,
    TARGET_SEMANTICS,
    ATTENTION_SEMANTICS,
    SEMANTIC_PRIORITY,
    SEMANTIC_WEIGHTS,
    REQUIRED_RESOLUTION,
    AIMTYPE_VIEW_PROFILE,
)
import laspy

# ── Config ──────────────────────────────────────────────────────────
LAS_PATH = ROOT / "userdata/user/point_cloud/500kV古敬5307线#028.las"
VOXEL_SIZE = 0.10

# ── Load LAS ────────────────────────────────────────────────────────
print("=" * 70)
print("COMPREHENSIVE DIAGNOSTIC REPORT")
print("=" * 70)

with laspy.open(str(LAS_PATH)) as f:
    las = f.read()
    points = np.vstack((las.x, las.y, las.z)).transpose()
    labels = np.asarray(las.classification, dtype=int)

surface_model = build_semantic_surface_model(points, labels, VOXEL_SIZE)
candidates = generate_candidate_views(surface_model)
meta = surface_model.get("meta", {})

# ── Save npz ────────────────────────────────────────────────────────
tmpdir = Path(tempfile.mkdtemp(prefix="diag_"))
npz_path = tmpdir / "diag_voxel.npz"
cand_path = tmpdir / "diag_candidates.json"
output_dir = tmpdir / "waypoints"
output_dir.mkdir(parents=True, exist_ok=True)

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

# ── Environment ──────────────────────────────────────────────────────
env = PlanningEnvironment(str(npz_path))
engine = VisibilityEngine(env)
cv_list = load_candidate_views(str(cand_path))

print(f"\n{'='*70}")
print(f"1. TARGET AND CANDIDATE OVERVIEW")
print(f"{'='*70}")
print(f"  Total candidates:          {len(candidates)}")
print(f"  Total target cells:        {env.target_count}")
print(f"  Total attention targets:   {env.attention_count}")

# ── 2. Per-semantic target counts ───────────────────────────────────
print(f"\n{'='*70}")
print(f"2. PER-SEMANTIC TARGET COUNTS")
print(f"{'='*70}")
print(f"  {'Semantic':<35s} {'Targets':>8s} {'ReqRes':>8s} {'Priority':>8s} {'Weight':>8s}")
print(f"  {'-'*71}")

# Regular targets
sem_counts = Counter(env.semantics)
for sem in sorted(sem_counts.keys(), key=lambda s: sem_counts[s], reverse=True):
    req_res = REQUIRED_RESOLUTION.get(sem, 0.0)
    pri = SEMANTIC_PRIORITY.get(sem, 0)
    w = SEMANTIC_WEIGHTS.get(sem, 0)
    print(f"  {sem:<35s} {sem_counts[sem]:>8d} {req_res:>8.2f} {pri:>8d} {w:>8.1f}")

# Attention targets
print(f"\n  --- Attention targets ---")
att_sem_counts = Counter(env.attention_semantics)
for sem in sorted(att_sem_counts.keys(), key=lambda s: att_sem_counts[s], reverse=True):
    req_res = REQUIRED_RESOLUTION.get(sem, 0.0)
    pri = SEMANTIC_PRIORITY.get(sem, 0)
    w = SEMANTIC_WEIGHTS.get(sem, 0)
    print(f"  {sem:<35s} {att_sem_counts[sem]:>8d} {req_res:>8.2f} {pri:>8d} {w:>8.1f}")

# ── 3. Per-semantic candidate counts ───────────────────────────────
print(f"\n{'='*70}")
print(f"3. PER-AIMTYPE CANDIDATE COUNTS")
print(f"{'='*70}")

aim_counts = Counter(c.get("AimType", "unknown") for c in candidates)
aim_sem_map = {}
for c in candidates:
    at = c.get("AimType", "unknown")
    sf = c.get("semantic_focus", "unknown")
    if at not in aim_sem_map:
        aim_sem_map[at] = sf

print(f"  {'AimType':<35s} {'Cands':>6s} {'SemanticFocus':<25s}")
print(f"  {'-'*70}")
for at in sorted(aim_counts.keys(), key=lambda a: aim_counts[a], reverse=True):
    print(f"  {at:<35s} {aim_counts[at]:>6d} {aim_sem_map.get(at, '?'):<25s}")

# ── 4. Visibility statistics ───────────────────────────────────────
print(f"\n{'='*70}")
print(f"4. PER-AIMTYPE VISIBILITY STATISTICS")
print(f"{'='*70}")

by_aim = defaultdict(lambda: {"total": 0, "has_reg": 0, "has_att": 0, "has_any": 0,
                                "total_reg_vis": 0, "total_att_vis": 0,
                                "max_reg": 0, "max_att": 0})
for cv in cv_list:
    at = getattr(cv, "aim_type", "unknown")
    by_aim[at]["total"] += 1
    vis = engine.candidate_visible_indices(cv)
    att_vis = engine.candidate_visible_attention_indices(cv)
    if len(vis) > 0:
        by_aim[at]["has_reg"] += 1
        by_aim[at]["total_reg_vis"] += len(vis)
        by_aim[at]["max_reg"] = max(by_aim[at]["max_reg"], len(vis))
    if len(att_vis) > 0:
        by_aim[at]["has_att"] += 1
        by_aim[at]["total_att_vis"] += len(att_vis)
        by_aim[at]["max_att"] = max(by_aim[at]["max_att"], len(att_vis))
    if len(vis) > 0 or len(att_vis) > 0:
        by_aim[at]["has_any"] += 1

print(f"  {'AimType':<35s} {'Total':>5s} {'HasReg':>6s} {'HasAtt':>6s} {'HasAny':>6s} {'AvgReg':>6s} {'AvgAtt':>6s} {'MaxReg':>6s} {'MaxAtt':>6s}")
print(f"  {'-'*90}")
for at in sorted(by_aim.keys()):
    d = by_aim[at]
    avg_r = d["total_reg_vis"] / max(d["total"], 1)
    avg_a = d["total_att_vis"] / max(d["total"], 1)
    print(f"  {at:<35s} {d['total']:>5d} {d['has_reg']:>6d} {d['has_att']:>6d} {d['has_any']:>6d} {avg_r:>6.1f} {avg_a:>6.1f} {d['max_reg']:>6d} {d['max_att']:>6d}")

# ── 5. Per-semantic target reachability ─────────────────────────────
print(f"\n{'='*70}")
print(f"5. PER-SEMANTIC TARGET REACHABILITY")
print(f"{'='*70}")

# Build full coverage matrix from ALL candidates
all_covered_reg = np.zeros(env.target_count, dtype=bool)
all_covered_att = np.zeros(env.attention_count, dtype=bool)
for cv in cv_list:
    vis = engine.candidate_visible_indices(cv)
    att_vis = engine.candidate_visible_attention_indices(cv)
    all_covered_reg[vis] = True
    all_covered_att[att_vis] = True

print(f"  --- Regular targets ---")
print(f"  {'Semantic':<35s} {'Total':>6s} {'Reachable':>10s} {'Unreachable':>12s} {'ReachPct':>8s}")
print(f"  {'-'*75}")
for sem in sorted(sem_counts.keys(), key=lambda s: sem_counts[s], reverse=True):
    mask = env.semantics == sem
    total = int(np.sum(mask))
    reachable = int(np.sum(all_covered_reg[mask]))
    unreachable = total - reachable
    pct = f"{100*reachable/total:.1f}%" if total else "N/A"
    print(f"  {sem:<35s} {total:>6d} {reachable:>10d} {unreachable:>12d} {pct:>8s}")

print(f"\n  --- Attention targets ---")
print(f"  {'Semantic':<35s} {'Total':>6s} {'Reachable':>10s} {'Unreachable':>12s} {'ReachPct':>8s}")
print(f"  {'-'*75}")
for sem in sorted(att_sem_counts.keys(), key=lambda s: att_sem_counts[s], reverse=True):
    mask = env.attention_semantics == sem
    total = int(np.sum(mask))
    reachable = int(np.sum(all_covered_att[mask]))
    unreachable = total - reachable
    pct = f"{100*reachable/total:.1f}%" if total else "N/A"
    print(f"  {sem:<35s} {total:>6d} {reachable:>10d} {unreachable:>12d} {pct:>8s}")

# ── 6. Connection candidate deep dive ───────────────────────────────
print(f"\n{'='*70}")
print(f"6. CONNECTION CANDIDATE DEEP DIVE")
print(f"{'='*70}")

for conn_aim in ["conductor_insulator_connection", "insulator_tower_side_connection",
                  "ground_wire_tower_connection", "tower_base"]:
    conn_cvs = [cv for cv in cv_list if getattr(cv, "aim_type", None) == conn_aim]
    conn_cands = [c for c in candidates if c.get("AimType") == conn_aim]
    print(f"\n  --- {conn_aim} ---")
    print(f"  CandidateViewpoints: {len(conn_cvs)}, JSON candidates: {len(conn_cands)}")

    if not conn_cvs:
        print(f"  NO candidates found for this AIMType!")
        continue

    # Check visibility
    reg_vis_counts = [len(engine.candidate_visible_indices(cv)) for cv in conn_cvs]
    att_vis_counts = [len(engine.candidate_visible_attention_indices(cv)) for cv in conn_cvs]
    has_reg = sum(1 for v in reg_vis_counts if v > 0)
    has_att = sum(1 for v in att_vis_counts if v > 0)
    has_both = sum(1 for r, a in zip(reg_vis_counts, att_vis_counts) if r > 0 or a > 0)

    print(f"  Has regular visible:   {has_reg}/{len(conn_cvs)}")
    print(f"  Has attention visible: {has_att}/{len(conn_cvs)}")
    print(f"  Has any visible:       {has_both}/{len(conn_cvs)}")
    if reg_vis_counts:
        print(f"  Regular visible range: [{min(reg_vis_counts)}, {max(reg_vis_counts)}]")
    if att_vis_counts:
        print(f"  Attention visible range: [{min(att_vis_counts)}, {max(att_vis_counts)}]")

    # Check what attention targets they see and their semantics
    if has_att > 0:
        for cv in conn_cvs[:3]:
            att_vis = engine.candidate_visible_attention_indices(cv)
            if len(att_vis) > 0:
                seen_sems = Counter(env.attention_semantics[att_vis])
                print(f"  Example cv id={cv.id}: sees attention targets with semantics: {dict(seen_sems)}")
                # Check distance
                att_dists = np.linalg.norm(env.attention_coords[att_vis] - cv.position, axis=1)
                print(f"    distances: [{att_dists.min():.1f}m, {att_dists.max():.1f}m]")
                # Check if target center matches any attention coord
                tgt = cv.target_center
                nearest_att_idx = np.argmin(np.linalg.norm(env.attention_coords - tgt, axis=1))
                nearest_dist = np.linalg.norm(env.attention_coords[nearest_att_idx] - tgt)
                print(f"    target center nearest attention: idx={nearest_att_idx}, dist={nearest_dist:.3f}m")

    # Check what the first candidate looks like
    if conn_cands:
        c0 = conn_cands[0]
        print(f"  First candidate:")
        print(f"    pos={c0['utm_position']}")
        print(f"    target={c0['target_center']}")
        print(f"    distance={c0.get('Distance', '?')}m")
        print(f"    yaw={c0['yaw']}, pitch={c0['pitch']}")
        print(f"    semantic_focus={c0['semantic_focus']}")
        print(f"    AimType={c0.get('AimType', '?')}")
        print(f"    source={c0['source']}")
        print(f"    weight={c0.get('weight', '?')}")

# ── 7. Scoring analysis ─────────────────────────────────────────────
print(f"\n{'='*70}")
print(f"7. SCORING / PLANNER GAIN ANALYSIS")
print(f"{'='*70}")

# Simulate the planner's base_score computation
from waypoint_planning.planning_core import SEMANTIC_PRIORITY as SP

scored_by_aim = defaultdict(lambda: {"total": 0, "scores": [], "has_any": 0})
for cv in cv_list:
    at = getattr(cv, "aim_type", "unknown")
    scored_by_aim[at]["total"] += 1
    vis = engine.candidate_visible_indices(cv)
    att_vis = engine.candidate_visible_attention_indices(cv)
    if len(vis) == 0 and len(att_vis) == 0:
        continue
    scored_by_aim[at]["has_any"] += 1

    # Replicate base_score computation
    weights = float(np.sum(env.weights[vis]))
    quality_factor = 0.12 if at == "tower_overview" else 1.0
    semantic_bonus = 0.0
    for sem, pri in SP.items():
        if sem in ATTENTION_SEMANTICS:
            semantic_bonus += pri * float(np.sum(env.attention_semantics[att_vis] == sem)) / max(
                env.attention_totals.get(sem, 1), 1)
        else:
            semantic_bonus += pri * float(np.sum(env.semantics[vis] == sem)) / max(
                env.semantic_totals.get(sem, 1), 1)
    score = quality_factor * weights / max(env.total_weight, 1e-9) + 0.2 * semantic_bonus
    scored_by_aim[at]["scores"].append(score)

print(f"  {'AimType':<35s} {'Total':>5s} {'HasVis':>6s} {'MaxScore':>10s} {'AvgScore':>10s}")
print(f"  {'-'*70}")
for at in sorted(scored_by_aim.keys()):
    d = scored_by_aim[at]
    if d["scores"]:
        max_s = max(d["scores"])
        avg_s = np.mean(d["scores"])
    else:
        max_s = 0.0
        avg_s = 0.0
    print(f"  {at:<35s} {d['total']:>5d} {d['has_any']:>6d} {max_s:>10.6f} {avg_s:>10.6f}")

# ── 8. Attention target weight check ─────────────────────────────────
print(f"\n{'='*70}")
print(f"8. ATTENTION TARGET WEIGHT & COVERAGE CHECK")
print(f"{'='*70}")

print(f"  Attention SEMANTIC_PRIORITY values:")
for sem in ATTENTION_SEMANTICS:
    print(f"    {sem}: priority={SEMANTIC_PRIORITY.get(sem, 0)}, weight={SEMANTIC_WEIGHTS.get(sem, 0)}")

print(f"\n  Attention target semantic totals (for normalization):")
for sem in ATTENTION_SEMANTICS:
    total = env.attention_totals.get(sem, 0)
    print(f"    {sem}: {total}")

print(f"\n  Regular target semantic totals (for normalization):")
for sem in TARGET_SEMANTICS:
    total = env.semantic_totals.get(sem, 0)
    if total > 0:
        print(f"    {sem}: {total}")

# ── 8.5. Topology validation ──────────────────────────────────────────
print(f"\n{'='*70}")
print(f"8.5. TOPOLOGY VALIDATION")
print(f"{'='*70}")
topo = meta.get("topology_validation", {})
if topo:
    print(f"  Attention target counts:")
    for sem, count in topo.get("attention_target_counts", {}).items():
        print(f"    {sem}: {count}")
    print(f"\n  Ground-wire near insulator: {topo.get('ground_wire_near_insulator_count', '?')}")
    print(f"  Ground-wire/insulator misassociation: {topo.get('ground_wire_insulator_misassociation', '?')}")
    print(f"  Conductor near both insulator+ground_wire: {topo.get('conductor_near_both_insulator_and_ground_wire_count', '?')}")
    print(f"\n  Three key connection types present:")
    for sem, present in topo.get("three_key_connection_types_present", {}).items():
        status = "[OK]" if present else "[MISSING]"
        print(f"    {status} {sem}")
else:
    print(f"  [WARN] No topology_validation data in meta")

# ── 9. Recommendation summary ───────────────────────────────────────
print(f"\n{'='*70}")
print(f"9. SUMMARY & RECOMMENDATIONS")
print(f"{'='*70}")

# Which semantics have < 50% reachable?
for sem in sorted(sem_counts.keys(), key=lambda s: sem_counts[s], reverse=True):
    mask = env.semantics == sem
    total = int(np.sum(mask))
    reachable = int(np.sum(all_covered_reg[mask]))
    pct = 100 * reachable / total if total else 0
    if pct < 50:
        print(f"  [LOW REACH] {sem}: {reachable}/{total} = {pct:.1f}% reachable")
    else:
        print(f"  [OK]         {sem}: {reachable}/{total} = {pct:.1f}% reachable")

for sem in sorted(att_sem_counts.keys(), key=lambda s: att_sem_counts[s], reverse=True):
    mask = env.attention_semantics == sem
    total = int(np.sum(mask))
    reachable = int(np.sum(all_covered_att[mask]))
    pct = 100 * reachable / total if total else 0
    if pct < 50:
        print(f"  [LOW ATT]    {sem}: {reachable}/{total} = {pct:.1f}% (attention)")
    else:
        print(f"  [OK ATT]     {sem}: {reachable}/{total} = {pct:.1f}% (attention)")

# Check if ATTENTION_SEMANTICS are in SEMANTIC_PRIORITY
for sem in ATTENTION_SEMANTICS:
    if sem not in SEMANTIC_PRIORITY:
        print(f"  [BUG] {sem} is in ATTENTION_SEMANTICS but NOT in SEMANTIC_PRIORITY!")
    else:
        pri = SEMANTIC_PRIORITY[sem]
        # Compare with regular semantics
        print(f"  [INFO] {sem}: priority={pri}, vs insulator={SEMANTIC_PRIORITY.get('insulator', 0)}, tower_top={SEMANTIC_PRIORITY.get('tower_top', 0)}")

print(f"\nDone. Temp dir: {tmpdir}")
