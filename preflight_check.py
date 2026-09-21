#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Fast regression checks to run before the expensive planning experiments."""
import inspect
import numpy as np

from baseline_manager import weighted_increment, weighted_objective
from altitude_constraints import (
    altitude_is_allowed, minimum_allowed_altitude,
    validate_transition_distance,
)
from astar_risk_planner import AStarRiskPlanner
from environment_threat_model import EnvironmentThreatModel
from radar_environment_5zones import RadarEnvironment5Zones
from fixed_wing_dynamics import FixedWingDynamics
from trajectory_smoothing import (
    SMOOTHING_VERSION,
    _distances_to_polyline,
    default_s_candidates,
    smooth_with_validation,
    repair_spline_to_altitude_floor,
    repair_spline_climb_angle,
    spline_altitude_floor_margin,
    spline_max_climb_angle,
    validate_spline,
)

print("=" * 78)
print("PRE-FLIGHT CHECK V5.3")
print("smoothing version:", SMOOTHING_VERSION)
print("=" * 78)

dyn = FixedWingDynamics()

# 0) API/version consistency.
sig = inspect.signature(dyn.compute_speed_profile)
assert "strict_feasibility" in sig.parameters
sig_smooth = inspect.signature(smooth_with_validation)
for required in (
    "max_smoothing_deviation", "enforce_search_corridor", "ds_target",
    "terminal_heading_tolerance_deg", "endpoint_tolerance_m", "altitude_checker", "altitude_floor",
):
    assert required in sig_smooth.parameters, required
assert SMOOTHING_VERSION.startswith("V5.3"), SMOOTHING_VERSION
assert hasattr(dyn, "fuel_heat_value_j_per_kg")
print("[0/11] version/API/fuel-unit consistency: PASS")

# 1) Geometry helper regression.
ref = np.array([[0., 0., 0.], [100., 0., 0.], [200., 0., 0.]])
q = np.array([[50., 30., 0.], [150., 0., 40.], [200., 0., 0.]])
d = _distances_to_polyline(q, ref)
assert np.allclose(d, [30., 40., 0.], atol=1e-8), d
print("[1/11] distance helper: PASS")

# 2) Exact paper normalization + additive-search equivalence.
baseline = {
    "T_min": 100., "T_max": 200.,
    "C_min": 2., "C_max": 10.,
    "F_min": 4., "F_max": 12.,
}
w = dict(w_time=0.3, w_risk=1.0, w_fuel=0.2)
metrics = {"T": 140., "Crisk": 6., "F": 8.}
o = weighted_objective(metrics, baseline, **w)
assert np.isclose(o["T_n"], 0.4)
assert np.isclose(o["C_n"], 0.5)
assert np.isclose(o["F_n"], 0.5)
assert np.isclose(o["J"], 0.3 * .4 + 1.0 * .5 + .2 * .5)
edge1 = {"T": 60., "Crisk": 2.5, "F": 3.}
edge2 = {"T": 80., "Crisk": 3.5, "F": 5.}
g = weighted_increment(edge1, baseline, **w) + weighted_increment(edge2, baseline, **w)
constant = (w["w_time"] * baseline["T_min"] / (baseline["T_max"] - baseline["T_min"])
            + w["w_risk"] * baseline["C_min"] / (baseline["C_max"] - baseline["C_min"])
            + w["w_fuel"] * baseline["F_min"] / (baseline["F_max"] - baseline["F_min"]))
assert np.isclose(g - o["J"], constant)
print("[2/11] normalized objective / additive A* equivalence: PASS")

# 3) Global B-spline candidate evaluation + terminal/end-point constraints.
path = []
for i in range(20):
    path.append(np.array([200.0 * i, 20.0 * (i % 2), min(10.0 * i, 100.0)]))
cands = default_s_candidates(len(path), 200.0)
smooth, result = smooth_with_validation(
    path, dyn,
    grid_xy=200.0,
    corridor_radius=None,
    collision_checker=lambda p: False,
    max_smoothing_deviation=200.0,
    ds_target=100.0,
    terminal_heading_tolerance_deg=30.0,
)
assert result["attempts"] == len(cands), (result["attempts"], len(cands))
assert len(result["candidate_history"]) == len(cands)
assert result["success"], result["message"]
val = result["validation"]
assert val["endpoints_ok"] and val["terminal_heading_ok"]
assert "curvature_variation_integral" in val
assert np.isfinite(result["selection_score"])
print("[3/11] all-s global selection / endpoint / terminal heading: PASS")

# 4) Search corridor remains diagnostic by default but can be strict.
straight = [np.array([0., 10., 0.]), np.array([250., 10., 0.]),
            np.array([500., 10., 0.]), np.array([750., 10., 0.]),
            np.array([1000., 10., 0.])]
coarse = [np.array([0., 0., 0.]), np.array([1000., 0., 0.])]
_, r1 = smooth_with_validation(
    straight, dyn, grid_xy=200.0, corridor_radius=1.0,
    corridor_reference=coarse, collision_checker=lambda p: False,
    max_smoothing_deviation=200.0, enforce_search_corridor=False,
    terminal_heading_tolerance_deg=30.0)
assert r1["success"], r1["message"]
assert not r1["validation"]["search_corridor_ok"]
_, r2 = smooth_with_validation(
    straight, dyn, grid_xy=200.0, corridor_radius=1.0,
    corridor_reference=coarse, collision_checker=lambda p: False,
    max_smoothing_deviation=200.0, enforce_search_corridor=True,
    terminal_heading_tolerance_deg=30.0)
assert not r2["success"], r2
print("[4/11] diagnostic-vs-strict search corridor: PASS")

# 5) Mission altitude envelope: endpoints stay unchanged; interior is 1-10 km.
start = np.array([2000.0, 2000.0, 0.0])
goal = np.array([98000.0, 98000.0, 0.0])
h_half = minimum_allowed_altitude(
    np.array([4500.0, 2000.0, 0.0]), start, goal,
    enroute_min_alt=1000.0, transition_distance=5000.0)
assert np.isclose(h_half, 500.0), h_half
assert np.isclose(minimum_allowed_altitude(start, start, goal, 1000.0, 5000.0), 0.0)
mid = np.array([50000.0, 50000.0, 1000.0])
assert np.isclose(minimum_allowed_altitude(mid, start, goal, 1000.0, 5000.0), 1000.0)
assert altitude_is_allowed(start, start, goal, 1000.0, 10000.0, 5000.0)
assert altitude_is_allowed(goal, start, goal, 1000.0, 10000.0, 5000.0)
assert altitude_is_allowed(mid, start, goal, 1000.0, 10000.0, 5000.0)
assert not altitude_is_allowed([50000., 50000., 999.], start, goal, 1000.0, 10000.0, 5000.0)
ok, required = validate_transition_distance(
    start, goal, 1000.0, 5000.0, dyn.max_climb_angle_deg)
assert ok and required < 5000.0, required

# A* must enforce the same envelope on nodes.
env = RadarEnvironment5Zones()
env.generate(seed=42)
threat = EnvironmentThreatModel(env, model="power", power_n=4.0)
planner = AStarRiskPlanner(
    env=env, threat=threat, dynamics=dyn,
    grid_res_xy=2000.0, grid_res_z=500.0,
    max_iterations=10, enroute_min_alt=1000.0,
    transition_distance=5000.0, max_flight_alt=10000.0)
planner.set_mission(start, goal)
assert planner._is_in_bounds(tuple(start))
assert planner._is_in_bounds(tuple(goal))
assert planner._is_in_bounds((50000.0, 50000.0, 1000.0))
assert not planner._is_in_bounds((50000.0, 50000.0, 0.0))
print(f"[5/11] altitude envelope / unchanged endpoints: PASS (minimum transition {required:.1f} m)")

# 6) B-spline validation must treat altitude-envelope failure as hard.
_, altitude_fail = smooth_with_validation(
    path, dyn,
    grid_xy=200.0,
    corridor_radius=None,
    collision_checker=lambda p: False,
    altitude_checker=lambda p: False,
    max_smoothing_deviation=200.0,
    ds_target=100.0,
    terminal_heading_tolerance_deg=30.0,
)
assert not altitude_fail["success"]
assert any(
    item.get("stage") != "fit" and item.get("altitude_ok") is False
    for item in altitude_fail["candidate_history"]
)
print("[6/11] B-spline altitude hard constraint: PASS")

# 7) Adaptive altitude repair must not undershoot a linearly rising ramp.
from scipy.interpolate import splprep
u_test = np.linspace(0.0, 1.0, 20)
x_test = 10000.0 * u_test
y_test = np.zeros_like(u_test)
z_test = np.minimum(1000.0, 0.20 * x_test)  # 0->1000 m over first 5 km
z_test[-1] = 0.0  # terminal endpoint, intentionally difficult shape
tck_test, _ = splprep([x_test, y_test, z_test], s=2.0e6, k=3)
raw_test = np.column_stack([x_test, y_test, z_test])

def _floor_test(q):
    x = float(q[0])
    return min(1000.0, max(0.0, 0.20 * min(x, 10000.0 - x)))

rtck, _, _ = repair_spline_to_altitude_floor(
    tck_test, raw_test, _floor_test, buffer_m=50.0)
margin = spline_altitude_floor_margin(rtck, _floor_test, n_probe=8192)
assert margin >= -1e-3, margin
print(f"[7/11] adaptive altitude-repair interpolation: PASS (margin={margin:.4f} m)")

# 8) Coarse-grid cubic overshoot must be repaired without relaxing 15 deg.
coarse_like = np.array([
    [0., 0., 0.],
    [2000., 0., 500.],
    [4000., 0., 1000.],
    [6000., 2000., 1000.],
    [8000., 4000., 1000.],
    [10000., 6000., 1000.],
    [12000., 8000., 1000.],
    [14000., 10000., 1000.],
    [16000., 12000., 1000.],
    [18000., 14000., 500.],
    [20000., 16000., 0.],
])
raw_edge_angles = np.degrees(np.arctan2(
    np.abs(np.diff(coarse_like[:, 2])),
    np.maximum(np.linalg.norm(np.diff(coarse_like[:, :2], axis=0), axis=1), 1e-12),
))
assert float(np.max(raw_edge_angles)) < dyn.max_climb_angle_deg

joint_start = coarse_like[0].copy()
joint_goal = coarse_like[-1].copy()
_joint_alt_checker = lambda p: altitude_is_allowed(
    p, joint_start, joint_goal, 1000.0, 10000.0, 5000.0)
_joint_alt_floor = lambda p: minimum_allowed_altitude(
    p, joint_start, joint_goal, 1000.0, 5000.0)

_, climb_result = smooth_with_validation(
    coarse_like, dyn,
    s_param=0.0,
    grid_xy=2000.0,
    corridor_radius=None,
    collision_checker=lambda p: False,
    altitude_checker=_joint_alt_checker,
    altitude_floor=_joint_alt_floor,
    max_smoothing_deviation=3200.0,
    ds_target=100.0,
    terminal_heading_tolerance_deg=60.0,
)
assert climb_result["success"], climb_result["message"]
assert climb_result.get("climb_repaired", False), climb_result
assert climb_result["validation"]["max_climb_angle"] <= dyn.max_climb_angle_deg + 1e-6
assert climb_result["validation"]["turn_radius_ok"]
assert climb_result["validation"]["altitude_ok"]
print(
    f"[8/11] joint altitude+climb B-spline repair: PASS "
    f"({climb_result['validation']['max_climb_angle']:.3f} deg)"
)

# 9) Sweep-specific coarse-grid trust region / schema / 72-case configuration.
import sweep_2d as sweep_cfg
expected_diag = np.sqrt(2.0 * sweep_cfg.GRID_RES_XY ** 2 + sweep_cfg.GRID_RES_Z ** 2)
assert np.isclose(sweep_cfg.COARSE_GRID_DIAGONAL, expected_diag)
assert np.isclose(
    sweep_cfg.SMOOTHING_TRUST_RADIUS,
    sweep_cfg.GRID_TRUST_SAFETY * expected_diag,
)
assert sweep_cfg.SMOOTHING_TRUST_RADIUS > 2512.4  # largest V5.2 locality failure
assert sweep_cfg.SMOOTHING_TRUST_RADIUS < 1.1 * expected_diag
assert len(sweep_cfg.COMBOS) == 72 and len(set(sweep_cfg.COMBOS)) == 72
assert str(sweep_cfg.SWEEP_SCHEMA_VERSION).startswith("v5.3")

# A gentle 2.5 km continuous bulge must be rejected by the old 2000 m rule but
# accepted by the new one-grid-diagonal trust region.
tt = np.linspace(0.0, 1.0, 81)
raw_ref = np.column_stack([
    100000.0 * tt,
    np.zeros_like(tt),
    np.full_like(tt, 1000.0),
])
bulge = raw_ref.copy()
bulge[:, 1] = 2500.0 * np.sin(np.pi * tt)
bulge_tck, bulge_u = splprep(
    [bulge[:, 0], bulge[:, 1], bulge[:, 2]], s=0.0, k=3)
old_trust_val = validate_spline(
    bulge_tck, raw_ref, bulge_u, dyn,
    max_smoothing_deviation=2000.0,
    terminal_heading=np.array([1.0, 0.0]),
    terminal_heading_tolerance_deg=10.0,
)
new_trust_val = validate_spline(
    bulge_tck, raw_ref, bulge_u, dyn,
    max_smoothing_deviation=sweep_cfg.SMOOTHING_TRUST_RADIUS,
    terminal_heading=np.array([1.0, 0.0]),
    terminal_heading_tolerance_deg=10.0,
)
assert not old_trust_val["smoothing_locality_ok"]
assert new_trust_val["smoothing_locality_ok"]
assert 2400.0 < new_trust_val["max_raw_deviation"] < 2600.0

print(
    f"[9/11] sweep trust/schema/72-combo configuration: PASS "
    f"(trust={sweep_cfg.SMOOTHING_TRUST_RADIUS:.1f} m, "
    f"2.5 km bulge accepted)"
)

# 10) Full sweep post-processing smoke test:
# coarse path -> B-spline repairs -> analytic curvature -> speed -> unified evaluator.
smoke_env = RadarEnvironment5Zones(
    map_size=30000.0, radar_radius=3000.0,
    min_flight_alt=1000.0, max_flight_alt=10000.0, cruise_alt=3000.0)
smoke_env.radar_zones = []
smoke_env.fire_zones = []
smoke_threat = EnvironmentThreatModel(smoke_env, model="power", power_n=4.0)
smoke_planner = AStarRiskPlanner(
    env=smoke_env, threat=smoke_threat, dynamics=dyn,
    grid_res_xy=sweep_cfg.GRID_RES_XY, grid_res_z=sweep_cfg.GRID_RES_Z,
    v_cruise=sweep_cfg.V_CRUISE,
    w_time=0.3, w_risk=1.0, w_fuel=0.2,
    n_edge_samples=sweep_cfg.N_EDGE_SAMPLES,
    max_iterations=10,
    enroute_min_alt=1000.0,
    transition_distance=5000.0,
    max_flight_alt=10000.0,
)
smoke_planner.set_mission(coarse_like[0], coarse_like[-1])
smoke_planner.set_baseline({
    "T_min": 300.0, "T_max": 800.0,
    "C_min": 0.0, "C_max": 100.0,
    "F_min": 1.0, "F_max": 10.0,
    "L0": 25000.0, "T0": 450.0, "C0": 0.0,
})
smoke_path, smoke_smooth, smoke_stats, smoke_speed, smoke_accel = sweep_cfg.finalise_path(
    coarse_like, smoke_planner, dyn, smoke_threat)
assert smoke_path is not None and smoke_smooth["success"]
assert smoke_smooth["validation"]["climb_angle_ok"]
assert smoke_smooth["validation"]["altitude_ok"]
assert smoke_smooth["validation"]["collision_free"]
assert np.all(np.isfinite(smoke_speed))
assert np.isfinite(smoke_stats["J"])
assert smoke_stats["max_climb_angle_deg"] <= dyn.max_climb_angle_deg + 1e-6
print(
    f"[10/11] full sweep post-processing pipeline: PASS "
    f"(J={smoke_stats['J']:.4f}, Rmin={smoke_stats['min_turn_radius']:.1f} m)"
)

# 11) Fuel model sanity: kg must be positive and finite.
P = dyn.get_power_at_state(dyn.cruise_speed, 0.0, 3000.0)
fuel_60s = P * 60.0 / dyn.fuel_heat_value_j_per_kg
assert np.isfinite(fuel_60s) and fuel_60s > 0.0 and fuel_60s < 100.0
print(f"[11/11] fuel units: PASS ({fuel_60s:.4f} kg for 60 s cruise reference)")

print("=" * 78)
print("ALL PRE-FLIGHT CHECKS PASSED")
print("Now run: python main.py")
print("=" * 78)
