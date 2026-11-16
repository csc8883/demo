"""S0-S2 API acceptance on a real LAS in isolated temporary storage.

Run from repo root: python scripts/validate_inspection_baseline.py
Creates no user account and never replaces the user's original planning files.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from fastapi.testclient import TestClient
from backend.server import create_app
from backend.core.dependencies import get_current_user
from waypoint_planning.algorithms import process_status


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sample', type=Path)
    parser.add_argument('--report', type=Path, default=ROOT / 'validation' / 's0-s2-baseline.json')
    args = parser.parse_args()
    sample = args.sample or next((ROOT / 'userdata/user/point_cloud').glob('*028.las'))
    started = time.perf_counter()
    report = {'sample': sample.name, 'sha256': hashlib.sha256(sample.read_bytes()).hexdigest(), 'checks': {}}
    with tempfile.TemporaryDirectory(prefix='inspection_baseline_') as temp:
        with patch('backend.core.security.USER_DATA_DIR', Path(temp)):
            app = create_app()
            app.dependency_overrides[get_current_user] = lambda: 'baseline'
            with TestClient(app) as client:
                def success(response):
                    response.raise_for_status()
                    data = response.json()
                    assert data.get('status') == 'success', data
                    return data.get('data', {})
                with sample.open('rb') as stream:
                    success(client.post('/api/upload/point_cloud', files={'file': (sample.name, stream, 'application/octet-stream')}))
                report['checks']['upload'] = True
                voxel = success(client.post('/api/process/voxelize', data={'pc_filename': sample.name}))
                assert process_status['baseline:voxelize']['status'] == 'completed', process_status['baseline:voxelize']
                report['checks']['preprocess'] = True
                planners = success(client.get('/api/planners'))
                voxel_name = next((Path(temp) / 'baseline/voxel').glob('*_voxel.npz')).name
                success(client.post('/api/process/rl', data={'voxel_filename': voxel_name, 'planner': planners[0]['key']}))
                status = process_status['baseline:rl']
                assert status['status'] == 'completed', status
                output = next((Path(temp) / 'baseline/waypoint').glob('*.json'))
                raw = json.loads(output.read_text(encoding='utf-8'))
                display = success(client.get('/api/visualize/result', params={'type': 'waypoint', 'filename': output.name}))
                report['checks']['planning_and_visualization'] = True
                assert all(s.get('shot_id') and s.get('look_at') for w in display['waypoints'] for s in w['shots'])
                assert any(s.get('component_id') for w in display['waypoints'] for s in w['shots'])
                report['checks']['component_shot_identity'] = True
                exported = client.get('/api/export/waypoint', params={'filename': output.name})
                exported.raise_for_status()
                assert exported.json()['waypoints'] == raw['waypoints']
                assert display['waypoints'][0]['pos_utm'] == raw['waypoints'][0]['position']
                report['checks']['world_coordinates_preserved_in_export'] = True
                from backend.routers.routes import plan_route_from_waypoints
                route_started = Event()
                def observed_route(*a, **kw):
                    route_started.set()
                    return plan_route_from_waypoints(*a, **kw)
                with patch('backend.routers.routes.plan_route_from_waypoints', observed_route), ThreadPoolExecutor(max_workers=1) as pool:
                    pending = pool.submit(client.post, '/api/route/plan', data={'waypoint_filename': output.name})
                    assert route_started.wait(10), 'Route worker did not start'
                    before_status = time.perf_counter()
                    success(client.get('/api/planners'))
                    status_latency = time.perf_counter() - before_status
                    assert status_latency < 2, f'API blocked by route computation: {status_latency}s'
                    report['concurrent_status_latency_seconds'] = round(status_latency, 3)
                    route_data = success(pending.result())
                safety = success(client.post('/api/route/validate', data={'filename': route_data['filename']}))
                route_export = client.get('/api/export/route', params={'filename': route_data['filename']})
                route_export.raise_for_status()
                assert route_export.json().get('towers')
                report['checks']['route_plan_validate_export'] = True
                report['waypoint_count'] = len(raw['waypoints'])
                report['shot_count'] = sum(len(w.get('shots', [])) for w in raw['waypoints'])
                report['coverage'] = {k: raw.get('stats', {}).get(k) for k in ('C_geo', 'C_weighted', 'C_ins', 'stop_reason')}
                report['route'] = {k: route_data[k] for k in ('route_point_count', 'totalLen', 'detour_point_count', 'astar_segment_count', 'astar_fallback_count')}
                report['safety'] = {k: safety[k] for k in ('passed', 'violation_count', 'task_violation_count', 'auxiliary_violation_count', 'segment_violation_count', 'conductor_no_fly_violation_count')}
    report['elapsed_seconds'] = round(time.perf_counter() - started, 2)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=True, indent=2))


if __name__ == '__main__':
    main()
