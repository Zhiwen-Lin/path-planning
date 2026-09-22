"""Postprocessing contract tests; no A* searches or production writes."""
import contextlib
import ast
import io
import os
import pickle
import runpy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

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

    def test_all_analysis_scripts_share_validated_table_and_selection(self):
        scripts = ('analyze_core_tradeoffs_v2.py',
                   'analyze_altitude_strategy_v2.py', 'analyze_weight_sensitivity.py')
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
        from plot_flight_constraints import build_profiles, _profile_time
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

    def test_radar_annotation_uses_actual_exposure(self):
        data = self.load_case(saved_case())
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(os.environ, {'ANALYSIS_OUTDIR': tmp}))
            stack.enter_context(patch.object(trajectories, 'load_trajectory_data', return_value=data))
            stack.enter_context(patch('matplotlib.figure.Figure.savefig'))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            namespace = runpy.run_path(str(ROOT/'analyze_radar_interaction.py'))
            env = namespace['RadarEnvironment5Zones']()
            env.generate(seed=42)
            threat = namespace['EnvironmentThreatModel'](env, model='power', power_n=4.0)
            info_text = namespace['plot_penetration'](data, env, threat)
            self.assertIn('累积暴露: 12.50 s', info_text)
            plt.close('all')

    def test_merged_trajectory_entries_preserve_all_outputs_and_load_once(self):
        data = self.load_case(saved_case())
        expected = {
            'plot_trajectory.py': {'astar_3d_trajectory.png', 'astar_altitude_profile.png',
                'astar_topview.png', 'astar_altitude_changes.png', 'astar_3d_multi_view.png'},
            'plot_flight_constraints.py': {'speed_profile_three_way_english.png', 'turn_radius_analysis.png'},
            'analyze_radar_interaction.py': {'radar_interaction_analysis_sci.png', 'radar_interaction_overview.png'},
        }
        original = {key: np.array(data[key], copy=True)
                    for key in ('path', 'path_raw', 'speed_profile', 'accel_profile')}
        for script, outputs in expected.items():
            with self.subTest(script=script), tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
                stack.enter_context(patch.dict(os.environ, {'ANALYSIS_OUTDIR': tmp}))
                reader = stack.enter_context(patch.object(trajectories, 'load_trajectory_data', return_value=data))
                saved = {}
                def capture(fig, filename, **kwargs):
                    saved[Path(filename).name] = fig
                stack.enter_context(patch('matplotlib.figure.Figure.savefig', new=capture))
                stack.enter_context(patch.object(sys, 'argv', [script]))
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                runpy.run_path(str(ROOT/script), run_name='__main__')
                reader.assert_called_once()
                self.assertEqual(set(saved), outputs)
                if script == 'plot_trajectory.py':
                    distance = saved['astar_altitude_profile.png'].axes[0].lines[0].get_xdata()
                    self.assertAlmostEqual(distance[-1], data['stats']['L']/1000.)
                if script == 'plot_flight_constraints.py':
                    speed = saved['speed_profile_three_way_english.png'].axes[0].lines[1].get_ydata()
                    np.testing.assert_array_equal(speed, data['speed_profile']*3.6)
                for key, value in original.items():
                    np.testing.assert_array_equal(data[key], value)
                self.assertEqual(plt.get_fignums(), [])

    def test_main_speed_comparison_requires_matching_coarse_cache(self):
        import plot_flight_constraints as plots
        data = self.load_case(saved_case())
        data.update(source_kind='main', trajectory_signature='same-run')
        coarse = dict(data, path=data['path_raw'])
        profiles = plots.build_profiles(coarse, data, FixedWingDynamics())
        self.assertEqual(set(profiles), {'Coarse A*', 'Fine A* raw', 'Fine + B-spline'})
        np.testing.assert_array_equal(profiles['Fine + B-spline']['v'], data['speed_profile'])
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(os.environ, {'ANALYSIS_OUTDIR': tmp}))
            stack.enter_context(patch.object(plots, 'load_trajectory_data', return_value=coarse))
            stack.enter_context(patch('matplotlib.figure.Figure.savefig'))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            plots.plot_speed_profiles(data, FixedWingDynamics())
            coarse['w_time'] += .1
            with self.assertRaisesRegex(ValueError, 'different experiments'):
                plots.plot_speed_profiles(data, FixedWingDynamics())

    def test_altitude_axis_uses_3d_distance_on_climbing_path(self):
        import plot_trajectory as plots
        from radar_environment_5zones import RadarEnvironment5Zones
        points = np.array([[0., 0., 0.], [3000., 0., 4000.], [6000., 0., 0.]])
        data = dict(path=points, start=points[0], goal=points[-1],
                    stats={'L': 10000., 'Crisk': 0.}, L0=6000.)
        saved = {}
        def capture(fig, filename, **kwargs):
            saved[Path(filename).name] = fig
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(os.environ, {'ANALYSIS_OUTDIR': tmp}))
            stack.enter_context(patch('matplotlib.figure.Figure.savefig', new=capture))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            plots.plot_single_views(data, RadarEnvironment5Zones())
        distance = saved['astar_altitude_profile.png'].axes[0].lines[0].get_xdata()
        np.testing.assert_array_equal(distance, [0., 5., 10.])


class MergedEntryTests(unittest.TestCase):
    def test_imports_do_not_read_results_or_generate_figures(self):
        scripts = ('plot_trajectory.py', 'plot_flight_constraints.py',
                   'analyze_radar_interaction.py', 'analyze_weight_sensitivity.py')
        with patch.object(trajectories, 'load_trajectory_data', side_effect=AssertionError('read on import')), \
             patch.object(loader, 'load_sweep_dataframe', side_effect=AssertionError('read on import')), \
             patch('matplotlib.figure.Figure.savefig', side_effect=AssertionError('write on import')):
            for script in scripts:
                runpy.run_path(str(ROOT/script))
            self.assertEqual(plt.get_fignums(), [])

    def test_main_plot_commands_dispatch_only_requested_view(self):
        tree = ast.parse((ROOT/'main.py').read_text(encoding='utf-8-sig'))
        commands = [node.args[0] for node in ast.walk(tree) if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == 'subprocess' and node.func.attr == 'run']
        self.assertEqual(len(commands), 8)
        targets = {'single': 'plot_single_views', 'multi': 'plot_multi_view',
                   'speed': 'plot_speed_profiles', 'turn': 'plot_turn_radius',
                   'clearance': 'plot_clearance', 'penetration': 'plot_penetration'}
        with contextlib.redirect_stdout(io.StringIO()):
            for command in commands:
                script, *args = [ast.literal_eval(item) for item in command.elts[1:]]
                self.assertTrue((ROOT/script).is_file(), script)
                namespace = runpy.run_path(str(ROOT/script))
                globals_ = namespace['main'].__globals__
                with contextlib.ExitStack() as stack:
                    stack.enter_context(patch.dict(globals_, load_trajectory_data=lambda **kw: {}))
                    calls = {}
                    for name in targets.values():
                        if name in globals_:
                            calls[name] = Mock()
                    stack.enter_context(patch.dict(globals_, calls))
                    namespace['main'](args)
                    for name, call in calls.items():
                        self.assertEqual(call.call_count, int(name == targets[args[1]]))

    def test_heatmap_alias_uses_same_matrix_and_keeps_units(self):
        data = full_table()
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(os.environ, {'ANALYSIS_OUTDIR': tmp}))
            reader = stack.enter_context(patch.object(loader, 'load_sweep_dataframe', return_value=(data, Path(tmp)/'fixture.xlsx')))
            saved = {}
            def capture(fig, filename, **kwargs):
                saved[Path(filename).name] = fig
            stack.enter_context(patch('matplotlib.figure.Figure.savefig', new=capture))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            runpy.run_path(str(ROOT/'analyze_weight_sensitivity.py'), run_name='__main__')
            reader.assert_called_once()
            self.assertEqual(set(saved), {'weight_sensitivity_curves.png',
                'weight_interaction_heatmap.png', 'heatmap_3obj_v3.png'})
            heatmap = saved['weight_interaction_heatmap.png']
            self.assertIs(heatmap, saved['heatmap_3obj_v3.png'])
            expected = data.pivot(index='wf', columns='wt', values='C').sort_index(ascending=False)
            np.testing.assert_array_equal(heatmap.axes[1].images[0].get_array(), expected.values)
            self.assertIn('暴露时间 (s)', [ax.get_ylabel() for ax in heatmap.axes])
            self.assertEqual(plt.get_fignums(), [])


if __name__ == '__main__':
    unittest.main()
