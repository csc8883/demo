# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

"无人机巡检视点规划2.0" — a drone inspection waypoint/route planning system for power grid towers. Users upload LAS point clouds of transmission towers, the system voxelizes them, runs coverage-aware waypoint planning algorithms, and exports flight routes.

Repository name: `demo`.


# 全局语言规则

请始终使用简体中文与我交流。

## 必须使用中文的内容

- 任务理解
- 执行计划
- 中间进度说明
- 错误解释
- 修改总结
- 测试结果
- 风险提示
- TODO 列表

## 可以保留英文的内容

- 代码
- 命令
- 文件名
- 路径
- 变量名
- 函数名
- 类名
- Git 分支名
- 第三方库名称
- 原始报错日志

## 代码修改规则

- 修改代码前，先用中文说明修改计划。
- 修改代码后，用中文总结修改了哪些文件、为什么修改、如何验证。
- 不要用英文解释，除非我明确要求。
- 命令保持原样，命令解释使用中文。


## Run and test

```bash
# Copy and edit config, then start the dev server
cp .env.example .env
uvicorn main:app --host 127.0.0.1 --port 8000 --reload

# Run integration test scripts (run from repo root)
python scripts/test_conductor_no_fly_volume.py
python scripts/test_route_clearance_matrix.py
```

There is no test framework. The scripts under `scripts/` are standalone integration tests that import from `waypoint_planning.*` directly (they add the repo root to `sys.path`).

## Architecture

**Backend** — FastAPI monolith (`main.py`, ~1200 lines). Serves both a Jinja2-rendered SPA (`index.html`) and a REST API with custom header-based auth (`X-User-Name` + `X-Auth-Token`). User accounts live in `users.json` (HMAC-salted password hashing). There is no database — all state is files on disk under `userdata/{username}/{category}/`.

**Data pipeline** (5 user-data categories defined in `config.py`):
1. `point_cloud` — uploaded `.las` files
2. `voxel` — `.npz` voxel grids produced by `Pretreatment.run()`, plus `_candidates.json` view-candidate files
3. `waypoint` — JSON output from waypoint planning algorithms
4. `algorithm_route` — auto-generated flight routes from `route_planner.py`
5. `manual_route` — human-created route JSON files used as benchmarks

The UI composes these into a workflow: upload LAS → voxelize → run planner → plan route → export/validate.

**Waypoint planning engines** (`waypoint_planning/`):
- `planning_core.py` (111 KB) — voxelization, semantic surface modeling, candidate view generation, coverage evaluation, manual route parsing. Labels: tower, insulator, wire, ground_wire.
- `planning_solvers.py` (101 KB) — three planners registered in `PLANNER_REGISTRY`: `SemanticWeightedGreedyPlanner` (baseline greedy by coverage gain/safety), `SemanticSingleLayerRLPlanner` (single-layer RL), `HRLSolver` (hierarchical RL: high-level picks semantic regions, low-level picks safe waypoints/shots).
- `route_planner.py` (100 KB) — takes waypoints and produces flight routes with A* detour planning around no-fly zones, plus `validate_route_safety()` for clearance checking.
- `waypoint_models.py` — Pydantic models for planner input/constraints, handles multi-encoding JSON reads (utf-8-sig, utf-8, gb18030, gbk) for manual route compatibility.
- `algorithms.py` — thin wrappers re-exported for `main.py`, plus LAS reading helpers (`read_las_for_vis`), progress tracking dicts.

**Frontend** — single `index.html` with vanilla JS ES modules in `static/js/`. This is the only supported frontend; the retired Vue/Vite frontend and `static-vue` build output are not part of the app:
- `state.js` — global mutable state (user, coordinate offset, loaded assets)
- `api.js` — fetch wrapper that auto-attaches auth headers from state
- `scene.js` — Three.js r128 (CDN) 3D viewport, manages point cloud / voxel / route object groups, coordinate normalization via global offset
- `ui.js` — DOM management: layers panel, modals, forms, loading overlay
- `app.js` — wires everything together, binds functions to `window` for HTML `onclick` handlers

CSS: TailwindCSS (CDN) + custom `static/css/style.css` and `theme-extra.css`.

**Key patterns:**
- Per-user process status keys are `{username}_{task}` (e.g. `alice_voxelize`) to isolate concurrent background tasks across users.
- Coordinates are normalized in the frontend by subtracting the first loaded point cloud's center (`state.globalOffset`), so all 3D objects share a common origin.
- Route JSON files use multiple encodings — `waypoint_models._read_json_with_fallback()` tries utf-8-sig, utf-8, gb18030, gbk in order.
- `safe_filename()` strips path separators from user-provided filenames to prevent traversal.
