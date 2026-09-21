"""Validated adapters for main.py trajectories and individual sweep cases."""
import hashlib
import os
import pickle
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent


def main_trajectory_signature(baseline):
    """Include the fine-search and main entry point as well as shared physics."""
    from sweep_2d import experiment_signature
    digest = hashlib.sha256(experiment_signature(baseline).encode())
    for name in ('main.py', 'two_stage_planner.py'):
        digest.update((ROOT / name).read_text(encoding='utf-8-sig').encode())
    return digest.hexdigest()


def load_trajectory_data(path=None, *, allow_raw=True):
    from sweep_2d import experiment_signature, load_case, SWEEP_SCHEMA_VERSION
    from trajectory_smoothing import SMOOTHING_VERSION, spline_curvature
    from scipy.interpolate import splev
    from fixed_wing_dynamics import FixedWingDynamics

    source = Path(path or os.environ.get('TRAJECTORY_FILE') or
                  ROOT / 'output_5zones' / 'planned_path.pkl').expanduser()
    with source.open('rb') as stream:
        data = pickle.load(stream)
    if not isinstance(data, dict) or 'baseline' not in data:
        raise ValueError('Trajectory lacks baseline provenance; regenerate it with the current program')
    baseline = data['baseline']
    if 'raw_path' in data and 'final_path' in data:
        weights = tuple(data.get('weights', ()))
        saved = load_case(source, experiment_signature(baseline), weights)
        row = None if saved is None else saved.get('row')
        if row is None or row.get('schema_version') != SWEEP_SCHEMA_VERSION or not row.get('平滑成功'):
            raise ValueError('Sweep case is incomplete or its source signature is stale')
        for column, key in (('飞行时间_s', 'T'), ('累积暴露_C', 'Crisk'),
                            ('燃油消耗_kg', 'F'), ('统一目标_J', 'J')):
            if not np.isclose(row[column], data['final_stats'][key], rtol=1e-9, atol=1e-8):
                raise ValueError('Sweep row and saved trajectory metrics disagree')
        mission = baseline['signature']['payload']
        data = dict(path=data['final_path'], path_raw=data['raw_path'],
                    stats=data['final_stats'], stats_raw=data['raw_stats'],
                    smooth_result=data['smooth'], speed_profile=data['speed'],
                    accel_profile=data['accel'], baseline=baseline,
                    start=mission['start'], goal=mission['goal'], L0=baseline['L0'],
                    w_time=weights[0], w_risk=weights[1], w_fuel=weights[2],
                    source_kind='sweep', run_signature=data['run_signature'])
    else:
        if data.get('trajectory_signature') != main_trajectory_signature(baseline):
            raise ValueError('Main trajectory signature is missing/stale; run current main.py first')
        data = dict(data, source_kind='main')

    points = np.asarray(data['path'], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Invalid trajectory coordinates')
    if not np.allclose(points[[0, -1]], [data['start'], data['goal']], atol=2., rtol=0):
        raise ValueError('Trajectory endpoints do not match its mission')
    if not all(np.isfinite(data['stats'][k]) for k in ('L', 'T', 'Crisk', 'F', 'J')):
        raise ValueError('Trajectory metrics must be finite')
    smooth = data.get('smooth_result')
    if smooth is None:
        if not allow_raw:
            raise ValueError('A validated final spline is required, not a raw A* path')
    else:
        if (not smooth.get('success') or not smooth.get('validation', {}).get('feasible')
                or smooth.get('smoothing_version') != SMOOTHING_VERSION):
            raise ValueError('Trajectory is not a successful current-version spline')
        curvature = spline_curvature(smooth['tck'], smooth['u_smooth'])
        spline_points = np.asarray(splev(smooth['u_smooth'], smooth['tck'])).T
        if points.shape != spline_points.shape or not np.allclose(points, spline_points, rtol=1e-10, atol=1e-7):
            raise ValueError('Saved points do not match their spline parameters')
        speeds = np.asarray(data['speed_profile'], dtype=float)
        accelerations = FixedWingDynamics().validate_speed_profile(points, speeds, curvature)
        if not np.allclose(accelerations, data['accel_profile'], atol=1e-8, rtol=1e-8):
            raise ValueError('Stored acceleration disagrees with the saved speed profile')
        time = np.sum(2 * np.linalg.norm(np.diff(points, axis=0), axis=1) / (speeds[:-1] + speeds[1:]))
        if not np.isclose(time, data['stats']['T'], atol=1e-6, rtol=1e-9):
            raise ValueError('Stored flight time disagrees with the saved speed profile')
        data['curvature_smooth'] = curvature
    data['source_file'] = str(source.resolve())
    print(f"Trajectory source: {data['source_file']} ({data['source_kind']})")
    return data


def trajectory_output_path(filename):
    """Separate explicitly chosen sweep-case figures from main.py figures."""
    selected = os.environ.get('TRAJECTORY_FILE')
    default = (ROOT / 'analysis_figures' / Path(selected).stem if selected
               else ROOT / 'output_5zones')
    directory = Path(os.environ.get('ANALYSIS_OUTDIR') or default)
    directory.mkdir(parents=True, exist_ok=True)
    return str(directory / Path(filename).name)
