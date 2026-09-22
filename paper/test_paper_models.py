"""Independent checks of manuscript equations against the existing implementation.

Run from the project root: python -m unittest -v paper.test_paper_models
No search, production data writes, or TeX-engine substitution is performed.
"""
import ast
import math
from pathlib import Path
import re
import unittest

import numpy as np

from baseline_manager import normalize_metrics, weighted_increment, weighted_objective
from fixed_wing_dynamics import FixedWingDynamics
from test_regressions import planner_for_test
from trajectory_smoothing import default_s_candidates, smoothing_objective


class PaperModelTests(unittest.TestCase):
    def test_power_formula_and_vertical_deadband(self):
        dyn = FixedWingDynamics()
        for height in (0., 3000., 10000.):
            for vh in (33., 55., 75.):
                rho = 1.225 * math.exp(-height/8500.)
                level = (.5*rho*dyn.cd0*dyn.wing_area*vh**3
                         + 2*(dyn.mass*dyn.g)**2/(rho*vh*dyn.wing_area
                         *math.pi*dyn.aspect_ratio*dyn.oswald_efficiency))
                for vz in (-12., 0., .01, .02, 10.):
                    climb = dyn.mass*dyn.g*vz/dyn.climb_efficiency if vz > .01 else 0.
                    expected = (level+climb)/dyn.propulsive_efficiency
                    self.assertAlmostEqual(dyn.get_power_at_state(vh, vz, height), expected)

    def test_search_time_uses_direction_specific_vertical_limit(self):
        dyn = FixedWingDynamics()
        start = np.array([0., 0., 3000.])
        for dz in (-1000., 0., 1000.):
            end = start + [4000., 0., dz]
            rate = dyn.max_ascend_speed if dz > 0 else dyn.max_descend_speed
            expected = max(np.linalg.norm(end-start)/dyn.cruise_speed, abs(dz)/rate)
            self.assertAlmostEqual(dyn.get_segment_time(start, end, dyn.cruise_speed), expected)

    def test_subsegment_time_and_exposure_use_same_accelerated_motion(self):
        planner, dyn = planner_for_test(pd_value=.4, samples=10)
        start = np.array([2000., 2000., 3000.])
        end = start + [900., 0., 0.]
        v0, v1 = 33., 55.
        length = np.linalg.norm(end-start)
        lam = np.linspace(0., 1., 10)
        speeds = np.sqrt((1-lam)*v0*v0+lam*v1*v1)
        dt = 2*length*np.diff(lam)/(speeds[:-1]+speeds[1:])
        metrics = planner._segment_metrics(start, end, v0, v1)
        self.assertAlmostEqual(dt.sum(), 2*length/(v0+v1))
        self.assertAlmostEqual(metrics['T'], dt.sum())
        self.assertAlmostEqual(metrics['Crisk'], .4*dt.sum())
        expected_fuel = sum(dyn.get_power_at_state(length/9/t, 0., 3000.)*t
                            /dyn.fuel_heat_value_j_per_kg for t in dt)
        self.assertAlmostEqual(metrics['F'], expected_fuel)

    def test_normalized_edge_sum_has_only_one_offset_and_is_unclipped(self):
        baseline = dict(T_min=100., T_max=400., C_min=10., C_max=50., F_min=2., F_max=5.)
        edges = [dict(T=60., Crisk=6., F=1.), dict(T=240., Crisk=54., F=5.)]
        total = {key: sum(e[key] for e in edges) for key in edges[0]}
        weights = (.3, 1., .2)
        offset = sum(w*b/d for w,b,d in zip(weights, (100., 10., 2.), (300., 40., 3.)))
        accumulated = sum(weighted_increment(e, baseline, *weights) for e in edges)
        self.assertAlmostEqual(accumulated-offset, weighted_objective(total, baseline, *weights)['J'])
        self.assertGreater(normalize_metrics(total, baseline)['C_n'], 1.)
        self.assertLess(normalize_metrics(dict(T=90., Crisk=5., F=1.), baseline)['T_n'], 0.)

    def test_default_speed_boundary_is_not_prescribed_at_goal(self):
        dyn = FixedWingDynamics()
        path = np.column_stack((np.linspace(0., 2000., 21), np.zeros(21), np.full(21, 3000.)))
        _, speed, _, _ = dyn.compute_speed_profile(path, curvatures_in=np.zeros(21), strict_feasibility=True)
        self.assertEqual(speed[0], dyn.v_stall)
        self.assertEqual(speed[-1], dyn.cruise_speed)
        self.assertLessEqual(speed.max(), min(dyn.cruise_speed, dyn.max_horizontal_speed))

    def test_refinement_score_against_analytic_parabola(self):
        # Exact cubic Bezier representation of q(u)=(1000u,200u^2,3000).
        # Compare the implemented sampled score with independently derived
        # analytic curvature, arc density and perpendicular line distance.
        dyn = FixedWingDynamics()
        tck = (np.array([0., 0., 0., 0., 1., 1., 1., 1.]),
               [np.array([0., 1000/3, 2000/3, 1000.]),
                np.array([0., 0., 200/3, 200.]), np.full(4, 3000.)], 3)
        raw = np.array([[0., 0., 3000.], [1000., 200., 3000.]])
        got = smoothing_objective(tck, raw, dyn, 200.)
        u = np.linspace(0., 1., 100001)
        density = np.sqrt(1e6+160000*u*u)
        curvature = 400000/density**3
        curvature_s = -3*400000*160000*u/density**6
        radius = max(dyn.min_turn_radius_floor,
                     dyn.v_stall**2/(dyn.g*math.tan(math.radians(dyn.max_bank_angle_deg))))
        length = np.trapezoid(density, u)
        bend = np.trapezoid((curvature*radius)**2*density, u)/length
        variation = radius**3*np.trapezoid(curvature_s**2*density, u)
        probes = np.linspace(0., 1., 4096)
        deviation = np.sqrt(np.mean((200*(probes**2-probes)/math.sqrt(1.04))**2))/200
        ratio = length/math.hypot(1000, 200)
        for key, expected in [('J_smooth', bend), ('J_curvature', variation),
                              ('J_deviation', deviation), ('J_length', ratio),
                              ('J_s', .4*bend+.3*variation+.2*deviation+.1*ratio)]:
            self.assertAlmostEqual(got[key], expected, delta=max(abs(expected)*2e-5, 1e-10))

    def test_backward_speed_pass_anticipates_a_curvature_bottleneck(self):
        dyn = FixedWingDynamics()
        path = np.column_stack((np.arange(6)*100., np.zeros(6), np.full(6, 3000.)))
        # Synthetic curvature inputs isolate speed propagation from curve fitting.
        curvature = np.array([0., 0., 0., 1/240., 0., 0.])
        _, speeds, _, _ = dyn.compute_speed_profile(path, curvatures_in=curvature,
                                                    strict_feasibility=True)
        bottleneck_sq = 240*dyn.g*math.tan(math.radians(dyn.max_bank_angle_deg))
        expected = np.sqrt([33**2, 33**2+600, bottleneck_sq+800,
                            bottleneck_sq, bottleneck_sq+600, bottleneck_sq+1200])
        np.testing.assert_allclose(speeds, expected, rtol=1e-12)
        self.assertLess(speeds[2], math.sqrt(33**2+1200))
        accel = np.diff(speeds**2)/200
        self.assertLessEqual(accel.max(), dyn.max_horizontal_accel+1e-10)
        self.assertGreaterEqual(accel.min(), -dyn.max_decel-1e-10)

    def test_documented_smoothing_budgets_match_implementation(self):
        source = Path(__file__).with_name('section3_method.tex').read_text(encoding='utf-8')
        block = source.split(r'\mathcal B=\{', 1)[1].split(r'\}', 1)[0]
        fractions = [float(x) for x in re.findall(r'\d+\.\d+', block)]
        self.assertEqual(len(fractions), 21)
        for count, spacing in ((71, 2000.), (703, 200.)):
            expected = [count*(spacing*f)**2 for f in fractions]
            np.testing.assert_allclose(default_s_candidates(count, spacing), expected)

    def test_planning_table_matches_entrypoint_configuration(self):
        root = Path(__file__).resolve().parents[1]
        module = ast.parse((root/'main.py').read_text(encoding='utf-8-sig'))
        values = {node.targets[0].id: ast.literal_eval(node.value)
                  for node in module.body if isinstance(node, ast.Assign)
                  and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
                  and isinstance(node.value, ast.Constant)}
        source = Path(__file__).with_name('section4_experiments.tex').read_text(encoding='utf-8')
        for row, expected in (
            ('Coarse horizontal / vertical spacing',
             [values['COARSE_GRID_XY'], values['COARSE_GRID_Z']]),
            ('Fine horizontal / vertical spacing',
             [values['FINE_GRID_XY'], values['FINE_GRID_Z']]),
            ('Three-dimensional fine-search corridor radius', [values['CORRIDOR_RADIUS']]),
            ('Final target sampling distance', [values['FINAL_PATH_DS']])):
            contents = next(line.split('&', 1)[1] for line in source.splitlines() if line.startswith(' '+row+' &'))
            actual = [float(x) for x in re.findall(r'\d+(?:\.\d+)?', contents)]
            self.assertEqual(actual, expected)
        from sweep_2d import SMOOTHING_TRUST_RADIUS
        self.assertIn(f'${SMOOTHING_TRUST_RADIUS:.2f}', source)
        self.assertFalse(values['ENFORCE_SEARCH_CORRIDOR_ON_SPLINE'])

    def test_integrated_manuscript_structure_and_section_placement(self):
        folder = Path(__file__).parent
        source = (folder/'manuscript_updated.tex').read_text(encoding='utf-8')
        for name in ('abstract_draft.tex', 'section1_introduction.tex',
                     'section2_revised.tex', 'section3_method.tex',
                     'section4_experiments.tex', 'section5_conclusions.tex',
                     'references.tex'):
            self.assertIn((folder/name).read_text(encoding='utf-8'), source)
        active = re.sub(r'(?m)(?<!\\)%.*$', '', source)
        labels = re.findall(r'\\label\{([^}]+)\}', active)
        refs = re.findall(r'\\(?:eqref|ref)\{([^}]+)\}', active)
        self.assertEqual(len(labels), len(set(labels)))
        self.assertFalse(set(refs)-set(labels))
        sections = re.split(r'\\section\{[^}]+\}', active)
        self.assertEqual(len(sections)-1, 5)
        self.assertEqual([len(re.findall(r'\\subsection\{', text)) for text in sections[1:]],
                         [3, 4, 5, 4, 0])
        self.assertIn(r'\label{pr:eq:fuelpower}', sections[2])
        self.assertIn(r'\label{pr:sec:objective}', sections[2])
        self.assertIn(r'\label{sec:method}', sections[3])
        self.assertIn(r'\label{sec:speed}', sections[3])
        self.assertNotIn(r'\includegraphics', active)
        self.assertNotIn(r'\input{', active)
        self.assertNotIn('lamport94', active)
        stack = []
        for kind, name in re.findall(r'\\(begin|end)\{([^}]+)\}', active):
            if kind == 'begin':
                stack.append(name)
            else:
                self.assertTrue(stack)
                self.assertEqual(stack.pop(), name)
        self.assertFalse(stack)
        depth = 0
        # A brace preceded by an even number of backslashes is not escaped.
        for match in re.finditer(r'(\\*)[{}]', active):
            if len(match.group(1)) % 2:
                continue
            depth += 1 if match.group()[-1] == '{' else -1
            self.assertGreaterEqual(depth, 0)
        self.assertEqual(depth, 0)

    def test_chapter4_contribution_structure_and_labels(self):
        source = Path(__file__).with_name('section4_experiments.tex').read_text(encoding='utf-8')
        active = re.sub(r'(?m)(?<!\\)%.*$', '', source)
        sections = re.split(r'\\subsection\{([^}]+)\}', active)
        self.assertEqual(sections[1::2], [
            'Simulation setup and evaluation protocols',
            'Overall planning performance and controlled comparisons',
            'Mission-preference sensitivity',
            'Representative mission preferences'])
        bodies = sections[2::2]
        self.assertEqual([len(re.findall(r'\\subsubsection\{', body)) for body in bodies],
                         [0, 3, 0, 0])
        # Preserve stable cross-reference labels when moving the experimental
        # material, and ensure each evidence group belongs to its contribution.
        expected_groups = [
            ('sec:setup', 'tab:aircraft', 'tab:planning'),
            ('sec:ablation', 'sec:objective_comparison',
             'sec:resolution_comparison', 'sec:refinement_comparison'),
            ('sec:sensitivity', 'ex:eq:weights'),
            ('sec:representative', 'ex:eq:extremecases', 'ex:eq:batchscale',
             'ex:eq:balancedcase', 'tab:representatives')]
        for body, names in zip(bodies, expected_groups):
            labels = re.findall(r'\\label\{([^}]+)\}', body)
            for name in names:
                self.assertIn(name, labels)

    def test_compact_chapter_figure_and_representative_table_contract(self):
        source = Path(__file__).with_name('section4_experiments.tex').read_text(encoding='utf-8')
        sensitivity, representative = source.split(r'\label{sec:sensitivity}', 1)[1].split(
            r'\subsection{Representative mission preferences}', 1)
        # All graphics remain commented placeholders; verify the agreed four
        # figure allocation without requiring missing artwork for compilation.
        figures = re.findall(r'\{figures/([^}]+)\}', sensitivity)
        self.assertEqual(figures, ['time_heatmap.pdf', 'exposure_heatmap.pdf',
                                  'fuel_heatmap.pdf', 'time_exposure_scatter.pdf'])
        self.assertEqual(re.findall(r'\{figures/([^}]+)\}', representative),
                         ['representative_preferences.pdf'])
        self.assertEqual(representative.count(r'\begin{table}'), 1)
        rows = [line.strip() for line in representative.splitlines()
                if line.strip().startswith(('Risk-dominant &', 'Time-dominant &', 'Balanced &'))]
        self.assertEqual(len(rows), 3)
        for line in rows:
            self.assertEqual(line.count('&'), 6)
            self.assertEqual(line.count('--'), 6)  # No invented experiment numbers.

    def test_citations_are_resolved_and_bibliography_is_used(self):
        folder = Path(__file__).parent
        source = (folder/'manuscript_updated.tex').read_text(encoding='utf-8')
        active = re.sub(r'(?m)(?<!\\)%.*$', '', source)
        keys = re.findall(r'\\bibitem\{([^}]+)\}', active)
        cited = {key.strip() for group in re.findall(r'\\cite\{([^}]+)\}', active)
                 for key in group.split(',')}
        self.assertTrue(cited)
        self.assertEqual(len(keys), len(set(keys)))
        self.assertEqual(cited, set(keys))
        entries = re.split(r'\\bibitem\{[^}]+\}', active)[1:]
        for entry in entries:
            self.assertRegex(entry, r'\b(?:19|20)\d{2}\b')
        dois = re.findall(r'\\url\{https://doi.org/([^}]+)\}', active)
        self.assertEqual(len(dois), len(set(dois)))
        self.assertTrue(dois)
        self.assertIn(r'\usepackage[hidelinks]{hyperref}', active)

    def test_frontmatter_and_preview_are_synchronized(self):
        folder = Path(__file__).parent
        main = (folder/'manuscript_updated.tex').read_text(encoding='utf-8')
        preview = (folder/'preview.tex').read_text(encoding='utf-8')
        title = re.search(r'(?m)^\\title\{([^}]+)\}', main).group(1)
        self.assertIn('\\title{'+title+'}', preview)
        inputs = re.findall(r'\\input\{([^}]+)\}', preview)
        expected = ['abstract_draft.tex', 'section1_introduction.tex',
                    'section2_revised.tex', 'section3_method.tex',
                    'section4_experiments.tex', 'section5_conclusions.tex',
                    'references.tex']
        self.assertEqual(inputs, expected)
        for name in inputs:
            self.assertTrue((folder/name).is_file())
            self.assertIn((folder/name).read_text(encoding='utf-8'), main)
        abstract = main.split(r'\begin{abstract}', 1)[1].split(r'\end{abstract}', 1)[0]
        self.assertEqual(abstract.strip(),
                         (folder/'abstract_draft.tex').read_text(encoding='utf-8').strip())
        self.assertNotIn('Abstract text.', main)
        self.assertNotIn('Research highlight 1', main)
        highlights = main.split(r'\begin{highlights}', 1)[1].split(r'\end{highlights}', 1)[0]
        items = re.findall(r'\\item ([^\n]+)', highlights)
        self.assertTrue(3 <= len(items) <= 5)
        for item in items:
            self.assertLessEqual(len(item), 85)

    def test_tex_structure_and_internal_references(self):
        path = Path(__file__).with_name('section2_revised.tex')
        source = path.read_text(encoding='utf-8')
        source = re.sub(r'(?m)(?<!\\)%.*$', '', source)
        labels = re.findall(r'\\label\{([^}]+)\}', source)
        refs = re.findall(r'\\(?:eqref|ref)\{([^}]+)\}', source)
        self.assertEqual(len(labels), len(set(labels)))
        self.assertFalse(set(refs)-set(labels))
        self.assertEqual(len(re.findall(r'\\subsection\{', source)), 4)
        stack = []
        for kind, name in re.findall(r'\\(begin|end)\{([^}]+)\}', source):
            if kind == 'begin':
                stack.append(name)
            else:
                self.assertTrue(stack)
                self.assertEqual(stack.pop(), name)
        self.assertFalse(stack)
        braces = 0
        for token in re.findall(r'(?<!\\)[{}]', source):
            braces += 1 if token == '{' else -1
            self.assertGreaterEqual(braces, 0)
        self.assertEqual(braces, 0)


if __name__ == '__main__':
    unittest.main()
