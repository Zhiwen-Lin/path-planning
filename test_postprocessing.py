"""Postprocessing contract tests; no A* searches or production writes."""
import contextlib
import io
import os
import pickle
import runpy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import analysis_data_loader as loader
import trajectory_data_loader as trajectories
import trajectory_smoothing as smoothing
import sweep_2d as sweep
from baseline_manager import weighted_objective
from fixed_wing_dynamics import FixedWingDynamics
from test_regressions import selection_data, success_row

ROOT = Path(__file__).resolve().parent


def full_table():
    data = pd.concat([selection_data().iloc[:1]] * 72, ignore_index=True)
    data[['wt', 'wf']] = sweep.COMBOS
    x = np.arange(72.)
    data['T'], data['C'], data['F'] = 2500. + x, 30. + x * .2, 10. - x * .03
    data['L'], data['altm'] = 150. + x * .1, 1000. + x * 10.
    data['altkm'] = data['altm'] / 1000.
    data['Tm'], data['detour'] = data['T'] / 60., x * .1
    return data


def saved_case():
    raw = np.column_stack((np.linspace(2000., 22000., 10), np.full(10, 2000.), np.full(10, 1000.)))
    tck, _, _ = smoothing.fit_bspline(raw, 0.)
    points, u = smoothing.resample_by_arclength(tck, 100.)
    k = smoothing.spline_curvature(tck, u)
    dyn = FixedWingDynamics()
    _, v, _, a = dyn.compute_speed_profile(points, k, strict_feasibility=True)
    baseline = {'signature': {'sha256': 'fixture', 'payload': {'start': raw[0], 'goal': raw[-1]}},
                'T_min': 100., 'T_max': 500., 'C_min': 0., 'C_max': 100.,
                'F_min': 1., 'F_max': 20., 'L0': 20000.}
    stats = dict(L=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                 T=float(np.sum(2*np.linalg.norm(np.diff(points, axis=0), axis=1)/(v[:-1]+v[1:]))),
                 Crisk=12.5, F=4., min_turn_radius=float('inf'))
    stats.update(weighted_objective(stats, baseline, .3, 1., .2))
    smooth = dict(tck=tck, u_smooth=u, success=True, smoothed=True,
                  smoothing_version=smoothing.SMOOTHING_VERSION,
                  validation={'feasible': True})
    sig = sweep.experiment_signature(baseline)
    row = {**success_row(.3, .2), 'run_signature': sig}
    row.update({'飞行时间_s': stats['T'], '累积暴露_C': stats['Crisk'],
                '燃油消耗_kg': stats['F'], '统一目标_J': stats['J']})
    return dict(baseline=baseline, weights=(.3, 1., .2), run_signature=sig,
                raw_path=raw, raw_stats=stats, final_path=points, final_stats=stats,
                smooth=smooth, speed=v, accel=a, row=row)


class AnalysisContractTests(unittest.TestCase):
    def test_excel_reader_checks_current_source_signature(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'sweep.xlsx'
            full_table().to_excel(path, sheet_name='全部数据', index=False)
            with patch.object(loader, 'current_experiment_signature', return_value='test-signature'):
                self.assertEqual(len(loader.load_sweep_dataframe(str(path))[0]), 72)
            with patch.object(loader, 'current_experiment_signature', return_value='different-source'):
                with self.assertRaises(ValueError):
                    loader.load_sweep_dataframe(str(path))
    def test_formal_data_gate_rejects_partial_failed_and_mixed_data(self):
        self.assertEqual(len(loader.validate_sweep_dataframe(full_table())), 72)
        for field, value in [('run_signature', 'different'), ('run_signature', ''),
                             ('run_signature', np.nan), ('平滑版本', 'old'),
                             ('schema_version', 'old'), ('smoothOK', False), ('T', np.nan)]:
            data = full_table()
            data.loc[0, field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                loader.validate_sweep_dataframe(data)
        with self.assertRaises(ValueError):
            loader.validate_sweep_dataframe(full_table().iloc[:-1])

    def test_missing_current_workbook_does_not_fall_back_to_old_data(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {}, clear=True):
            old = Path(tmp) / 'sweep_2d_time_fuel.xlsx'
            old.touch()
            with patch.object(loader, 'DEFAULT_SWEEP', Path(tmp)/'new.xlsx'):
                with self.assertRaises(FileNotFoundError):
                    loader.resolve_sweep_file()
                self.assertEqual(loader.resolve_sweep_file(str(old)), old)

    def test_all_four_analysis_scripts_share_validated_table_and_selection(self):
        scripts = ('plot_heatmap_3obj_v3.py', 'analyze_core_tradeoffs_v2.py',
                   'analyze_altitude_strategy_v2.py', 'analyze_weight_sensitivity_v2.py')
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(os.environ, {'ANALYSIS_OUTDIR': tmp}))
            stack.enter_context(patch.object(loader, 'load_sweep_dataframe',
                side_effect=lambda: (full_table(), Path(tmp)/'fixture.xlsx')))
            stack.enter_context(patch('matplotlib.figure.Figure.savefig'))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            for script in scripts:
                with self.subTest(script=script):
                    namespace = runpy.run_path(str(ROOT/script), run_name='__main__')
                    if script == 'analyze_core_tradeoffs_v2.py':
                        from select_representative_weights import select_representatives
                        expected = select_representatives(full_table())
                        for name, row in namespace['representatives'].items():
                            self.assertEqual((row['wt'], row['wf']), (expected[name]['wt'], expected[name]['wf']))
                    plt.close('all')


class TrajectoryContractTests(unittest.TestCase):
    def load_case(self, case):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            path = Path(tmp)/'case.pkl'
            path.write_bytes(pickle.dumps(case))
            return trajectories.load_trajectory_data(path, allow_raw=False)

    def test_sweep_adapter_keeps_saved_metrics_and_profiles(self):
        case = saved_case()
        data = self.load_case(case)
        self.assertEqual(data['source_kind'], 'sweep')
        self.assertEqual(data['stats']['Crisk'], 12.5)
        np.testing.assert_array_equal(data['path'], case['final_path'])
        np.testing.assert_array_equal(data['speed_profile'], case['speed'])
        from visualize_speed_profile_english import build_profiles, _profile_time
        profiles = build_profiles({'path': data['path_raw']}, data, FixedWingDynamics())
        self.assertEqual(set(profiles), {'Coarse A*', 'Coarse + B-spline'})
        final = profiles['Coarse + B-spline']
        np.testing.assert_array_equal(final['v'], case['speed'])
        self.assertAlmostEqual(_profile_time(final['path'], final['v']), case['final_stats']['T'])

    def test_stale_signature_failed_spline_and_wrong_time_are_rejected(self):
        for error in ('signature', 'smooth', 'time', 'weights', 'points', 'row'):
            case = saved_case()
            if error == 'signature':
                case['run_signature'] = 'old'
            elif error == 'smooth':
                case['smooth']['success'] = False
            elif error == 'weights':
                case['weights'] = (.4, 1., .2)
            elif error == 'points':
                case['final_path'][10, 1] += 1.
            elif error == 'row':
                case['row']['累积暴露_C'] += 1.
            else:
                case['final_stats']['T'] += 1.
            with self.subTest(error=error), self.assertRaises(ValueError):
                self.load_case(case)

    def test_main_cache_signature_and_raw_vs_final_contract(self):
        case = saved_case()
        data = self.load_case(case)
        data.pop('run_signature')
        data['trajectory_signature'] = trajectories.main_trajectory_signature(data['baseline'])
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            path = Path(tmp)/'main.pkl'
            path.write_bytes(pickle.dumps(data))
            loaded = trajectories.load_trajectory_data(path, allow_raw=False)
            self.assertEqual(loaded['source_kind'], 'main')
            data.pop('smooth_result')
            path.write_bytes(pickle.dumps(data))
            trajectories.load_trajectory_data(path)
            with self.assertRaises(ValueError):
                trajectories.load_trajectory_data(path, allow_raw=False)
            data.pop('trajectory_signature')
            path.write_bytes(pickle.dumps(data))
            with self.assertRaises(ValueError):
                trajectories.load_trajectory_data(path)

    def test_radar_v2_annotation_uses_actual_exposure(self):
        data = self.load_case(saved_case())
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(os.environ, {'ANALYSIS_OUTDIR': tmp}))
            stack.enter_context(patch.object(trajectories, 'load_trajectory_data', return_value=data))
            stack.enter_context(patch('matplotlib.figure.Figure.savefig'))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            namespace = runpy.run_path(str(ROOT/'analyze_radar_interaction_v2.py'), run_name='__main__')
            self.assertIn('累积暴露: 12.50 s', namespace['info_text'])
            plt.close('all')


if __name__ == '__main__':
    unittest.main()
