"""Fast regressions for V5.3.1; no production files or long A* searches.

Run: python -m unittest -v test_regressions
"""
import contextlib
import io
import os
import pickle
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import pandas as pd

from astar_risk_planner import AStarRiskPlanner
from fixed_wing_dynamics import FixedWingDynamics
from radar_environment_5zones import RadarEnvironment5Zones
from select_representative_weights import select_representatives
import sweep_2d as sweep
import trajectory_smoothing as smoothing
from scipy.interpolate import splev


class SmoothingRepairTests(unittest.TestCase):
    def setUp(self):
        self.dyn = FixedWingDynamics()
        self.path = np.column_stack((np.linspace(0., 20000., 10),
                                     np.zeros(10), np.full(10, 1000.)))
        self.heading = np.array([np.cos(np.radians(20)), np.sin(np.radians(20))])

    def smooth(self, **kwargs):
        return smoothing.smooth_with_validation(
            self.path, self.dyn, s_param=0., grid_xy=2000.,
            max_smoothing_deviation=3000., ds_target=100., **kwargs)

    def test_heading_repair_preserves_endpoints_altitude_and_input(self):
        tck, _, _ = smoothing.fit_bspline(self.path, 0.)
        before = np.asarray(tck[1]).copy()
        fixed = smoothing.repair_terminal_heading(tck, self.heading)
        np.testing.assert_array_equal(tck[1], before)
        np.testing.assert_allclose(splev([0., 1.], fixed), splev([0., 1.], tck), atol=1e-10)
        np.testing.assert_array_equal(fixed[1][2], tck[1][2])
        tangent = np.asarray(splev(1., fixed, der=1))[:2]
        np.testing.assert_allclose(tangent / np.linalg.norm(tangent), self.heading, atol=1e-12)
        for bad in ([0., 0.], [np.nan, 1.], [1.], [1., 0., 0.]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                smoothing.repair_terminal_heading(tck, bad)

    def test_heading_failure_is_repaired_without_relaxing_constraints(self):
        path, result = self.smooth(terminal_heading=self.heading)
        self.assertTrue(result['success'], result['message'])
        self.assertTrue(result['terminal_heading_repaired'])
        self.assertEqual(result['total_candidate_evaluations'], 2)
        self.assertFalse(result['original_candidate_history'][0]['terminal_heading_ok'])
        self.assertLess(result['validation']['terminal_heading_error_deg'], 1e-5)
        self.assertEqual(result['validation']['terminal_heading_tolerance_deg'], 12.)
        self.assertTrue(result['validation']['feasible'])
        self.dyn.compute_speed_profile(path, strict_feasibility=True)

    def test_successful_original_candidate_is_not_repaired(self):
        with patch.object(smoothing, 'repair_terminal_heading', side_effect=AssertionError('unneeded repair')):
            _, result = self.smooth(terminal_heading=[1., 0.])
        self.assertTrue(result['success'])
        self.assertFalse(result['heading_repair_attempted'])
        self.assertEqual(result['total_candidate_evaluations'], 1)

    def test_heading_repair_cannot_bypass_other_hard_constraints(self):
        for checker in ('collision_checker', 'altitude_checker'):
            # Every sample collides, or every sample violates altitude.
            with self.subTest(checker=checker):
                _, result = self.smooth(terminal_heading=self.heading,
                    **{checker: lambda p: checker == 'collision_checker'})
                self.assertFalse(result['success'])
                self.assertTrue(result['heading_repair_attempted'])
                self.assertFalse(result['validation']['feasible'])

    def test_singular_and_vertical_splines_are_not_feasible(self):
        knots = np.array([0.] * 4 + [1.] * 4)
        # Constant cubic has no tangent; curvature=0 must not make it flyable.
        singular = (knots, [np.zeros(4)] * 3, 3)
        with self.assertRaisesRegex(ValueError, 'singular tangent'):
            smoothing.validate_spline(singular, np.zeros((4, 3)), np.linspace(0, 1, 4), self.dyn)
        vertical = (knots, [np.zeros(4), np.zeros(4), np.arange(4.) * 1000.], 3)
        raw = np.asarray(splev(np.linspace(0, 1, 4), vertical)).T
        result = smoothing.validate_spline(vertical, raw, np.linspace(0, 1, 4), self.dyn)
        self.assertEqual(result['max_climb_angle'], 90.)
        self.assertFalse(result['climb_angle_ok'])
        self.assertFalse(result['feasible'])


class ConstantThreat:
    def __init__(self, pd=1.0):
        self.pd = pd

    def check_collision(self, point):
        return False

    def calculate_risk(self, point):
        return self.pd


def planner_for_test(pd_value=1.0, samples=10):
    env = RadarEnvironment5Zones()
    env.radar_zones = []
    env.fire_zones = []
    dyn = FixedWingDynamics()
    planner = AStarRiskPlanner(env=env, threat=ConstantThreat(pd_value), dynamics=dyn,
                               n_edge_samples=samples)
    planner.set_mission([2000, 2000, 1000], [98000, 98000, 1000])
    return planner, dyn


class IntegrationTests(unittest.TestCase):
    def test_constant_probability_and_power_use_same_time(self):
        for speeds in [(33., 55.), (55., 33.), (55., 55.)]:
            for pd_value in [0., 0.25, 1.]:
                with self.subTest(speeds=speeds, pd=pd_value):
                    planner, dyn = planner_for_test(pd_value)
                    dyn.get_power_at_state = lambda *args: 43000.0
                    m = planner._segment_metrics([2000, 2000, 1000], [2100, 2000, 1000], *speeds)
                    expected = 200 / sum(speeds)
                    self.assertAlmostEqual(m['T'], expected, places=12)
                    self.assertAlmostEqual(m['Crisk'], pd_value * expected, places=12)
                    self.assertAlmostEqual(m['F'], 43000 * expected / dyn.fuel_heat_value_j_per_kg, places=12)

    def test_segment_split_and_quadrature_density(self):
        values = []
        for samples in [2, 10, 31]:
            p, d = planner_for_test(samples=samples)
            d.get_power_at_state = lambda *args: 43000.
            whole = p._segment_metrics([2000, 2000, 1000], [2100, 2000, 1000], 33, 55)
            middle_speed = np.sqrt((33**2 + 55**2) / 2)
            left = p._segment_metrics([2000, 2000, 1000], [2050, 2000, 1000], 33, middle_speed)
            right = p._segment_metrics([2050, 2000, 1000], [2100, 2000, 1000], middle_speed, 55)
            for key in ['T', 'Crisk', 'F']:
                self.assertAlmostEqual(whole[key], left[key] + right[key], places=12)
            values.append(whole['T'])
        np.testing.assert_allclose(values, values[0], rtol=1e-12)

    def test_raw_search_constant_speed_is_unchanged(self):
        p, d = planner_for_test(0.5)
        a, b = np.array([2000., 2000., 1000.]), np.array([4000., 2000., 1500.])
        m = p._segment_metrics(a, b)
        expected = d.get_segment_time(a, b, p.v_cruise)
        self.assertEqual(m['T'], expected)
        self.assertAlmostEqual(m['Crisk'], 0.5 * expected)

    def test_invalid_endpoint_speeds_are_rejected(self):
        p, _ = planner_for_test()
        for speeds in [(None, 33), (np.nan, 55), (0, 0), (-1, 55)]:
            with self.subTest(speeds=speeds), self.assertRaises(ValueError):
                p._segment_metrics([2000, 2000, 1000], [2100, 2000, 1000], *speeds)


class SpeedTests(unittest.TestCase):
    def setUp(self):
        self.dyn = FixedWingDynamics()
        self.path = np.array([[2000. + 100*i, 2000., 1000.] for i in range(20)])

    def test_airborne_start_and_acceleration_envelope(self):
        s, v, k, a = self.dyn.compute_speed_profile(self.path, np.zeros(20), strict_feasibility=True)
        self.assertEqual(v[0], self.dyn.v_stall)
        self.assertTrue(np.all(v >= self.dyn.v_stall))
        self.assertLessEqual(v.max(), self.dyn.cruise_speed)
        expected_a = np.diff(v**2) / (2*np.diff(s))
        np.testing.assert_allclose(a[:-1], expected_a, atol=1e-12)
        self.assertLessEqual(a.max(), self.dyn.max_horizontal_accel + 1e-9)
        self.assertGreaterEqual(a.min(), -self.dyn.max_decel - 1e-9)

    def test_backward_pass_brakes_for_turn(self):
        k = np.zeros(20)
        k[-3:] = 1.0 / 220.0
        _, v, _, a = self.dyn.compute_speed_profile(self.path, k, strict_feasibility=True)
        cap = np.sqrt(220*self.dyn.g*np.tan(np.radians(self.dyn.max_bank_angle_deg)))
        self.assertLessEqual(v[-1], cap + 1e-9)
        self.assertTrue(np.any(a < 0))
        self.assertGreaterEqual(a.min(), -self.dyn.max_decel - 1e-9)

    def test_first_point_is_subject_to_turn_constraint(self):
        k = np.zeros(20)
        k[0] = 1. / 100.
        with self.assertRaises(ValueError):
            self.dyn.compute_speed_profile(self.path, k, strict_feasibility=True)

    def test_invalid_input_is_rejected(self):
        for cap in [20, np.nan]:
            with self.assertRaises(ValueError):
                self.dyn.compute_speed_profile(self.path, v_ceiling=cap, strict_feasibility=True)
        k = np.zeros(20)
        k[5] = np.nan
        with self.assertRaises(ValueError):
            self.dyn.compute_speed_profile(self.path, k, strict_feasibility=True)
        bad_path = self.path.copy()
        bad_path[0, 0] = np.nan
        with self.assertRaises(ValueError):
            self.dyn.compute_speed_profile(bad_path, strict_feasibility=True)

    def test_external_speed_cannot_bypass_strict_evaluator(self):
        planner, _ = planner_for_test()
        for speed in [np.zeros(20), np.full(20, 90.), np.r_[33., np.full(19, 55.)]]:
            with self.subTest(speed=speed[:2]), self.assertRaises(ValueError):
                planner.evaluate_path(self.path, speed_profile=speed, curvatures=np.zeros(20),
                                      accel_profile=np.zeros(20), strict_feasibility=True)

    def test_final_evaluator_matches_speed_profile_time(self):
        planner, dyn = planner_for_test(0.25)
        _, v, k, a = dyn.compute_speed_profile(self.path, np.zeros(20), strict_feasibility=True)
        stats = planner.evaluate_path(self.path, speed_profile=v, curvatures=k,
                                      accel_profile=np.zeros(20), strict_feasibility=True)
        dt = 2*np.linalg.norm(np.diff(self.path, axis=0), axis=1)/(v[:-1]+v[1:])
        self.assertAlmostEqual(stats['T'], dt.sum(), places=10)
        self.assertAlmostEqual(stats['Crisk'], 0.25*stats['T'], places=10)
        self.assertAlmostEqual(stats['max_speed_change_rate'], np.abs(a).max())


def selection_data():
    data = pd.DataFrame([
        [0.0, 1., 0., 30., 0., 30.],
        [0.1, 1., 0., 10., 20., 25.],
        [0.2, 1., 0., 15., 6., 16.],
        [0.3, 1., 0., 25., 18., 10.],
    ], columns=['wt', 'wr', 'wf', 'T', 'C', 'F'])
    data['schema_version'] = sweep.SWEEP_SCHEMA_VERSION
    data['smoothOK'] = True
    data['高度包络满足'] = True
    return data


class SelectionTests(unittest.TestCase):
    def test_paper_objectives(self):
        selected = select_representatives(selection_data(), require_complete=False)
        for name, wt in [('绕飞', 0.), ('快穿', 0.1), ('平衡', 0.2), ('省油', 0.3)]:
            self.assertEqual(selected[name]['wt'], wt)

    def test_constant_objectives_and_ties_are_deterministic(self):
        data = selection_data()
        data[['T', 'C', 'F']] = [20., 0., 10.]
        selected = select_representatives(data.sample(frac=1, random_state=42), require_complete=False)
        for row in selected.values():
            self.assertEqual(row['wt'], 0.)
            self.assertEqual(row['ideal_distance'], 0.)

    def test_rejects_bad_incomplete_or_old_data(self):
        with self.assertRaises(ValueError):
            select_representatives(selection_data())
        for key, value in [('T', np.nan), ('F', np.inf), ('C', -1), ('smoothOK', False),
                           ('高度包络满足', False), ('schema_version', 'v5.3-old')]:
            data = selection_data()
            data.loc[0, key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                select_representatives(data, require_complete=False)
        data = pd.concat([selection_data(), selection_data().iloc[:1]])
        with self.assertRaises(ValueError):
            select_representatives(data, require_complete=False)

    def test_full_72_grid(self):
        data = pd.concat([selection_data().iloc[:1]]*72, ignore_index=True)
        data[['wt', 'wf']] = sweep.COMBOS
        self.assertEqual(len(select_representatives(data)), 4)

    def test_selection_cli_writes_all_four_cases(self):
        import select_representative_weights as selector
        data = pd.concat([selection_data().iloc[:1]]*72, ignore_index=True)
        data[['wt', 'wf']] = sweep.COMBOS
        data['L'], data['altm'], data['Tm'] = 150., 1000., data['T']/60
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(selector, 'load_sweep_dataframe', return_value=(data, Path(tmp)/'input.xlsx')))
            stack.enter_context(patch.object(selector, 'analysis_output_dir', return_value=Path(tmp)))
            stack.enter_context(patch('matplotlib.figure.Figure.savefig'))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            selector.main()
            result = pd.read_csv(Path(tmp)/'representative_weights.csv')
            self.assertEqual(set(result['用途']), {'绕飞', '快穿', '平衡', '省油'})
            self.assertTrue((result['理想点距离'] == 0).all())
            selector.plt.close('all')


def success_row(wt=0., wf=0.):
    return {'schema_version': sweep.SWEEP_SCHEMA_VERSION,
            'run_signature': 'test-signature', '权重_w_time': wt,
            '权重_w_risk': 1., '权重_w_fuel': wf, '平滑成功': True,
            '高度包络满足': True, '飞行时间_s': 100., '飞行时间_min': 100/60,
            '累积暴露_C': 10., '燃油消耗_kg': 1., '统一目标_J': 0.5}


class ResumeTests(unittest.TestCase):
    def test_72_case_export_load_select_and_resume_pipeline(self):
        from analysis_data_loader import load_sweep_dataframe
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            for key, value in [('OUT_DIR', tmp), ('CASE_DIR', os.path.join(tmp, 'cases')),
                               ('INTERIM_PKL', os.path.join(tmp, 'rows.pkl')),
                               ('EXCEL_FILE', os.path.join(tmp, 'sweep.xlsx'))]:
                stack.enter_context(patch.object(sweep, key, value))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            planner = Mock()
            baseline = dict.fromkeys(('T_min', 'T_max', 'C_min', 'C_max', 'F_min', 'F_max', 'L0', 'T0', 'C0'), 1.)
            planner.baseline_manager.ensure.return_value = baseline
            planner._run_astar.return_value = (np.zeros((2, 3)), {})
            stack.enter_context(patch.object(sweep, 'build_models', return_value=(None, None, None, planner)))
            stack.enter_context(patch.object(sweep, 'experiment_signature', return_value='test-signature'))
            finalise = stack.enter_context(patch.object(sweep, 'finalise_path', return_value=(np.zeros((2, 3)), {}, {}, [], [])))
            def make_row(wt, wf, *args):
                return {**success_row(wt, wf), '最小转弯半径_m': 1000., '平滑评价_Js': .1,
                        '平滑版本': smoothing.SMOOTHING_VERSION}
            stack.enter_context(patch.object(sweep, 'make_success_row', side_effect=make_row))
            rows, returned_baseline = sweep.run_sweep()
            self.assertEqual(planner._run_astar.call_count, 72)
            self.assertEqual(finalise.call_count, 72)
            self.assertEqual({(r['权重_w_time'], r['权重_w_fuel']) for r in rows}, set(sweep.COMBOS))
            sweep.export_excel(rows, returned_baseline)
            loaded, _ = load_sweep_dataframe(sweep.EXCEL_FILE)
            self.assertEqual(len(loaded), 72)
            self.assertEqual(set(loaded['平滑版本']), {smoothing.SMOOTHING_VERSION})
            self.assertEqual(set(loaded['run_signature']), {'test-signature'})
            self.assertEqual(len(select_representatives(loaded)), 4)
            rerun, _ = sweep.run_sweep()
            self.assertEqual(len(rerun), 72)
            self.assertEqual(planner._run_astar.call_count, 72)
            self.assertEqual(finalise.call_count, 72)

    def test_case_identity_and_complete_final_row_are_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'case.pkl'
            raw = {'run_signature': 'test-signature', 'weights': (0., 1., 0.),
                   'raw_path': np.zeros((2, 3)), 'raw_stats': {}, 'row': success_row()}
            sweep.atomic_pickle(path, raw)
            self.assertIsNone(sweep.load_case(path, 'test-signature', (0.1, 1., 0.)))
            self.assertIsNone(sweep.load_case(path, 'old-signature', (0., 1., 0.)))
            self.assertNotIn('row', sweep.load_case(path, 'test-signature', (0., 1., 0.)))
            complete = {**raw, **dict.fromkeys(('final_path', 'smooth', 'final_stats', 'speed', 'accel'), [])}
            sweep.atomic_pickle(path, complete)
            self.assertIn('row', sweep.load_case(path, 'test-signature', (0., 1., 0.)))
            complete['row'] = success_row(0.1)
            sweep.atomic_pickle(path, complete)
            recovered = sweep.load_case(path, 'test-signature', (0., 1., 0.))
            self.assertNotIn('row', recovered)
            self.assertIn('raw_path', recovered)

    def test_failed_and_nonfinite_rows_are_not_done(self):
        self.assertTrue(sweep.already_done([success_row()], 0, 0))
        for field, value in [('平滑成功', False), ('飞行时间_s', np.nan),
                             ('schema_version', 'old'), ('高度包络满足', False)]:
            row = success_row()
            row[field] = value
            self.assertFalse(sweep.already_done([row], 0, 0))

    def test_all_checkpoint_rows_are_checked(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(sweep, 'INTERIM_PKL', os.path.join(tmp, 'rows.pkl')):
            for corrupt in ['schema_version', 'run_signature']:
                rows = [success_row(), success_row(0.1)]
                rows[1][corrupt] = 'old'
                sweep.atomic_pickle(sweep.INTERIM_PKL, rows)
                with self.assertRaises(ValueError):
                    sweep.load_interim('test-signature')
            sweep.atomic_pickle(sweep.INTERIM_PKL, [success_row(), success_row()])
            with self.assertRaises(ValueError):
                sweep.load_interim('test-signature')

    def test_atomic_write_keeps_previous_checkpoint_on_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'rows.pkl'
            sweep.atomic_pickle(path, ['old'])
            with patch.object(sweep.os, 'replace', side_effect=OSError('interrupted')):
                with self.assertRaises(OSError):
                    sweep.atomic_pickle(path, ['new'])
            self.assertEqual(pickle.loads(path.read_bytes()), ['old'])

    def test_failed_smoothing_retries_without_astar_and_replaces_row(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.ExitStack() as stack:
            for key, value in [('OUT_DIR', tmp), ('INTERIM_PKL', os.path.join(tmp, 'rows.pkl')),
                               ('CASE_DIR', os.path.join(tmp, 'cases'))]:
                stack.enter_context(patch.object(sweep, key, value))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            planner = Mock()
            baseline = {'L0': 1.}
            planner.baseline_manager.ensure.return_value = baseline
            planner._run_astar.return_value = (np.zeros((2, 3)), {'L': 1.})
            stack.enter_context(patch.object(sweep, 'build_models', return_value=(None, None, None, planner)))
            stack.enter_context(patch.object(sweep, 'experiment_signature', return_value='test-signature'))
            rows = [success_row(wt, wf) for wt, wf in sweep.COMBOS if (wt, wf) != (0., 0.)]
            sweep.atomic_pickle(sweep.INTERIM_PKL, rows)
            with patch.object(sweep, 'finalise_path', side_effect=RuntimeError('test smoothing failure')):
                first, _ = sweep.run_sweep()
            self.assertEqual(len(first), 72)
            self.assertFalse(sweep.already_done(first, 0, 0))
            artifact = sweep.load_case(sweep.case_path(0, 0), 'test-signature')
            self.assertIn('raw_path', artifact)
            good = {**success_row(), '最小转弯半径_m': 1000., '平滑评价_Js': 0.1}
            with patch.object(sweep, 'finalise_path', return_value=(np.zeros((2, 3)), {}, {}, [], [])), \
                    patch.object(sweep, 'make_success_row', return_value=good):
                second, _ = sweep.run_sweep()
            self.assertEqual(len(second), 72)
            self.assertTrue(sweep.already_done(second, 0, 0))
            self.assertEqual(planner._run_astar.call_count, 1)
            third, _ = sweep.run_sweep()
            self.assertEqual(len(third), 72)
            self.assertEqual(planner._run_astar.call_count, 1)
            artifact = sweep.load_case(sweep.case_path(0, 0), 'test-signature')
            self.assertTrue({'final_path', 'smooth', 'speed', 'accel', 'row'} <= artifact.keys())
            # Lost aggregate update: recover the completed artifact directly.
            sweep.atomic_pickle(sweep.INTERIM_PKL, rows)
            with patch.object(sweep, 'finalise_path', side_effect=AssertionError('must not rerun')):
                recovered, _ = sweep.run_sweep()
            self.assertTrue(sweep.already_done(recovered, 0, 0))
            self.assertEqual(planner._run_astar.call_count, 1)

    def test_signature_changes_with_configuration(self):
        baseline = {'signature': {'sha256': 'abc'},
                    **dict.fromkeys(['T_min', 'T_max', 'C_min', 'C_max', 'F_min', 'F_max'], 1.)}
        sig = sweep.experiment_signature(baseline)
        with patch.object(sweep, 'FINAL_PATH_DS', 50.):
            self.assertNotEqual(sig, sweep.experiment_signature(baseline))

    def test_excel_export_can_be_retried_after_file_lock(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(sweep, 'EXCEL_FILE', os.path.join(tmp, 'test.xlsx')):
            baseline = dict.fromkeys(['T_min', 'T_max', 'C_min', 'C_max', 'F_min', 'F_max', 'L0', 'T0', 'C0'], 1.)
            with contextlib.redirect_stdout(io.StringIO()):
                sweep.export_excel([success_row()], baseline)
            original = Path(sweep.EXCEL_FILE).read_bytes()
            with patch.object(sweep.os, 'replace', side_effect=PermissionError('Excel lock')):
                with self.assertRaises(PermissionError):
                    sweep.export_excel([success_row()], baseline)
            self.assertEqual(original, Path(sweep.EXCEL_FILE).read_bytes())
            self.assertTrue(Path(sweep.EXCEL_FILE).with_suffix('.tmp.xlsx').exists())
            self.assertEqual(pd.read_excel(sweep.EXCEL_FILE, sheet_name='全部数据').shape[0], 1)


if __name__ == '__main__':
    unittest.main()
