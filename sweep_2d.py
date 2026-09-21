#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""2-D weight sweep for the final multi-objective planning pipeline.

Fixed ``w_risk=1`` and scan 12 time weights x 6 fuel weights = 72 cases.
Every successful case uses the same sequence as ``main.py``:

    A* -> B-spline global candidate selection -> analytic curvature
       -> adaptive fixed-wing speed -> unified T / Pd*dt / fuel evaluation

The normalization baseline is the shared ``output_5zones/baseline.pkl`` built
from the three single-objective anchors.  Primary Excel metrics therefore have
exactly the same meaning as the planner and ``main.py`` outputs.
"""

import argparse
import os
import pickle
import time
from datetime import datetime
from itertools import product

import numpy as np
import pandas as pd

from astar_risk_planner import AStarRiskPlanner
from baseline_manager import DEFAULT_BASELINE_PATH
from environment_threat_model import EnvironmentThreatModel
from fixed_wing_dynamics import FixedWingDynamics
from radar_environment_5zones import RadarEnvironment5Zones
from trajectory_smoothing import smooth_with_validation, spline_curvature
from altitude_constraints import DEFAULT_TRANSITION_DISTANCE_M

KM = 1000.0
SWEEP_SCHEMA_VERSION = "v5.3-grid-scaled-trust-climb-repair"

W_RISK_FIXED = 1.0
W_TIME_VALUES = np.round(np.arange(0.0, 1.1 + 1e-9, 0.1), 6).tolist()
W_FUEL_VALUES = [0.1, 0.0, 0.2, 0.3, 0.4, 0.5]
COMBOS = [(float(wt), float(wf))
          for wt, wf in product(W_TIME_VALUES, W_FUEL_VALUES)]

GRID_RES_XY = 2000.0
GRID_RES_Z = 500.0
FINAL_PATH_DS = 100.0
ALTITUDE_TRANSITION_DISTANCE = DEFAULT_TRANSITION_DISTANCE_M

# Smoothing trust region for the coarse-grid sweep.
# Distance-to-polyline is a 3-D Euclidean metric, so tie the allowed smoothing
# displacement to one maximum 26-neighbour coarse-grid step rather than only
# the horizontal cell width:
#
#   d_grid = sqrt((Δx)^2 + (Δy)^2 + (Δz)^2)
#          = sqrt(2*GRID_RES_XY^2 + GRID_RES_Z^2)
#
# A 5% numerical margin accommodates the continuous cubic interpolation while
# keeping the post-processed trajectory within about one coarse-grid step.
GRID_TRUST_SAFETY = 1.05
COARSE_GRID_DIAGONAL = float(
    np.sqrt(2.0 * GRID_RES_XY ** 2 + GRID_RES_Z ** 2)
)
SMOOTHING_TRUST_RADIUS = float(GRID_TRUST_SAFETY * COARSE_GRID_DIAGONAL)
V_CRUISE = 55.0
N_EDGE_SAMPLES = 10
EPSILON = 1.0
POWER_N = 4.0
SEED = 42

START = (2000.0, 2000.0, 0.0)
GOAL = (98000.0, 98000.0, 0.0)

OUT_DIR = "output_5zones"
INTERIM_PKL = os.path.join(OUT_DIR, "sweep2d_interim.pkl")
BASELINE_PKL = DEFAULT_BASELINE_PATH
EXCEL_FILE = os.path.join(OUT_DIR, "sweep_2d_time_fuel.xlsx")


def load_interim():
    if not os.path.exists(INTERIM_PKL):
        return []
    try:
        with open(INTERIM_PKL, "rb") as f:
            rows = pickle.load(f)
    except Exception as exc:
        print(f"  ! Cannot read old interim file: {exc}; starting clean")
        return []
    if not isinstance(rows, list):
        return []
    if rows and rows[0].get("schema_version") != SWEEP_SCHEMA_VERSION:
        print("  ! Old interim schema detected; it cannot supply the new metrics. Ignoring it.")
        return []
    print(f"  ✓ Resume file contains {len(rows)} completed cases")
    return rows


def already_done(rows, wt, wf, tol=1e-9):
    return any(
        abs(float(r.get("权重_w_time", 999)) - wt) < tol
        and abs(float(r.get("权重_w_fuel", 999)) - wf) < tol
        for r in rows
    )


def build_models():
    env = RadarEnvironment5Zones()
    env.generate(seed=SEED)
    threat = EnvironmentThreatModel(env, model="power", power_n=POWER_N)
    dynamics = FixedWingDynamics()
    planner = AStarRiskPlanner(
        env=env, threat=threat, dynamics=dynamics,
        grid_res_xy=GRID_RES_XY, grid_res_z=GRID_RES_Z,
        v_cruise=V_CRUISE,
        w_risk=W_RISK_FIXED, w_time=1.0, w_fuel=0.1,
        n_edge_samples=N_EDGE_SAMPLES,
        max_iterations=10_000_000,
        epsilon=EPSILON,
        baseline_path=BASELINE_PKL,
        enroute_min_alt=env.min_flight_alt,
        transition_distance=ALTITUDE_TRANSITION_DISTANCE,
        max_flight_alt=env.max_flight_alt,
    )
    return env, threat, dynamics, planner


def finalise_path(raw_path, planner, dynamics, threat):
    """Run B-spline -> curvature -> speed -> unified evaluator."""
    smooth_path, smooth = smooth_with_validation(
        raw_path,
        dynamics,
        grid_xy=GRID_RES_XY,
        corridor_radius=None,
        corridor_reference=None,
        collision_checker=threat.check_collision,
        altitude_checker=planner.altitude_allowed,
        altitude_floor=planner.minimum_allowed_altitude,
        max_smoothing_deviation=SMOOTHING_TRUST_RADIUS,
        enforce_search_corridor=False,
        ds_target=FINAL_PATH_DS,
    )
    if not smooth.get("success", False):
        return None, smooth, None, None, None

    P = np.asarray(smooth_path, dtype=float)
    u = np.asarray(smooth["u_smooth"], dtype=float)
    kappa = spline_curvature(smooth["tck"], u)
    _, speed, _, accel = dynamics.compute_speed_profile(
        P, curvatures_in=kappa, strict_feasibility=True)
    final_stats = planner.evaluate_path(
        P,
        curvatures=kappa,
        speed_profile=speed,
        accel_profile=accel,
        strict_feasibility=True,
    )
    return P, smooth, final_stats, speed, accel


def make_success_row(wt, wf, raw_path, raw_stats, final_path, smooth,
                     final_stats, elapsed, baseline):
    max_alt = float(np.max(final_path[:, 2]))
    L0 = float(baseline["L0"])
    val = smooth["validation"]
    return {
        "schema_version": SWEEP_SCHEMA_VERSION,
        "权重_w_time": round(float(wt), 6),
        "权重_w_risk": float(W_RISK_FIXED),
        "权重_w_fuel": round(float(wf), 6),
        # Final physical trajectory: these are the primary paper metrics.
        "飞行距离_km": final_stats["L"] / KM,
        "飞行时间_s": final_stats["T"],
        "飞行时间_min": final_stats["T"] / 60.0,
        "累积暴露_C": final_stats["Crisk"],
        "燃油消耗_kg": final_stats["F"],
        "归一化时间": final_stats["T_n"],
        "归一化暴露": final_stats["C_n"],
        "归一化燃油": final_stats["F_n"],
        "统一目标_J": final_stats["J"],
        "绕行增加_%": (final_stats["L"] / L0 - 1.0) * 100.0,
        "最大高度_m": max_alt,
        "最大高度_km": max_alt / KM,
        "航点数": int(len(final_path)),
        # New trajectory/dynamics metrics requested for the paper.
        "最小转弯半径_m": final_stats["min_turn_radius"],
        "最大曲率_1_m": final_stats["max_curvature"],
        "最大爬升角_deg": final_stats["max_climb_angle_deg"],
        "平滑评价_Js": smooth["selection_score"],
        "曲率变化积分_JkappaPrime": val["curvature_variation_integral"],
        "末端航向误差_deg": val["terminal_heading_error_deg"],
        "速度变化率最大_m_s2": final_stats["max_speed_change_rate"],
        "平滑成功": True,
        "高度包络满足": bool(val.get("altitude_ok", True)),
        "高度B样条修复": bool(smooth.get("altitude_repaired", False)),
        "高度修复缓冲_m": float(smooth.get("altitude_repair_buffer_m", 0.0)),
        "爬升角B样条修复": bool(smooth.get("climb_repaired", False)),
        "爬升修复斜率比例": float(smooth.get("climb_repair_fraction", 1.0)),
        "平滑最大偏离_m": float(val.get("max_raw_deviation", np.nan)),
        "平滑信赖域_m": float(SMOOTHING_TRUST_RADIUS),
        "粗网格3D对角_m": float(COARSE_GRID_DIAGONAL),
        "选中_s_m2": smooth["s_param_used"],
        "s候选数": smooth["attempts"],
        # Raw A* metrics retained for audit / before-after analysis.
        "A星原始距离_km": raw_stats["L"] / KM,
        "A星原始时间_s": raw_stats["T"],
        "A星原始暴露_C": raw_stats["Crisk"],
        "A星原始燃油_kg": raw_stats["F"],
        "A星原始J": raw_stats.get("J", np.nan),
        "A星原始航点数": int(len(raw_path)),
        "迭代次数": int(raw_stats.get("iterations", 0)),
        "扩展节点数": int(raw_stats.get("nodes_expanded", 0)),
        "计算耗时_s": float(elapsed),
        "时间戳": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }


def make_failure_row(wt, wf, elapsed, error, raw_stats=None, smooth=None):
    row = {
        "schema_version": SWEEP_SCHEMA_VERSION,
        "权重_w_time": round(float(wt), 6),
        "权重_w_risk": float(W_RISK_FIXED),
        "权重_w_fuel": round(float(wf), 6),
        "平滑成功": False,
        "计算耗时_s": float(elapsed),
        "错误": str(error),
        "时间戳": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    if raw_stats is not None:
        row.update({
            "A星原始距离_km": raw_stats.get("L", np.nan) / KM,
            "A星原始时间_s": raw_stats.get("T", np.nan),
            "A星原始暴露_C": raw_stats.get("Crisk", np.nan),
            "A星原始燃油_kg": raw_stats.get("F", np.nan),
            "A星原始J": raw_stats.get("J", np.nan),
        })
    if smooth is not None:
        row["平滑评价_Js"] = smooth.get("selection_score", np.nan)
        row["选中_s_m2"] = smooth.get("s_param_used", np.nan)
        row["s候选数"] = smooth.get("attempts", np.nan)
        row["高度B样条修复"] = bool(smooth.get("altitude_repaired", False))
        row["爬升角B样条修复"] = bool(smooth.get("climb_repaired", False))
        row["爬升修复斜率比例"] = float(smooth.get("climb_repair_fraction", 1.0))
        val = smooth.get("validation", {}) or {}
        row["失败候选最小转弯半径_m"] = val.get("min_turn_radius", np.nan)
        row["失败候选最大爬升角_deg"] = val.get("max_climb_angle", np.nan)
        row["失败候选最大偏离_m"] = val.get("max_raw_deviation", np.nan)
        row["失败候选高度包络满足"] = val.get("altitude_ok", np.nan)
        row["失败候选禁飞区安全"] = val.get("collision_free", np.nan)
        row["失败候选末端航向满足"] = val.get("terminal_heading_ok", np.nan)
    row["平滑信赖域_m"] = float(SMOOTHING_TRUST_RADIUS)
    row["粗网格3D对角_m"] = float(COARSE_GRID_DIAGONAL)
    return row


def run_sweep(resume=True, force_baseline=False):
    print("=" * 78)
    print("  2-D weight sweep: unified final-trajectory evaluation")
    print("=" * 78)
    print(f"  cases: {len(COMBOS)} | w_risk={W_RISK_FIXED:g}")
    print(f"  baseline: {BASELINE_PKL}")
    print(f"  altitude envelope: endpoints unchanged; "
          f"{1000:.0f}-{10000:.0f} m en-route; "
          f"{ALTITUDE_TRANSITION_DISTANCE/1000:.1f} km transitions")
    print(f"  coarse-grid 3D diagonal: {COARSE_GRID_DIAGONAL:.1f} m")
    print(f"  smoothing trust radius: {SMOOTHING_TRUST_RADIUS:.1f} m "
          f"({GRID_TRUST_SAFETY:.2f} × grid diagonal)")
    print(f"  output:   {EXCEL_FILE}")

    # Fast configuration checks before any expensive A* work.
    if len(COMBOS) != 72 or len(set(COMBOS)) != 72:
        raise RuntimeError(f"Expected 72 unique weight combinations, got {len(set(COMBOS))}")
    if GRID_RES_Z / GRID_RES_XY > np.tan(np.radians(FixedWingDynamics().max_climb_angle_deg)) + 1e-12:
        raise RuntimeError("Coarse-grid vertical/horizontal ratio violates climb-angle constraint")
    if SMOOTHING_TRUST_RADIUS < COARSE_GRID_DIAGONAL:
        raise RuntimeError("Smoothing trust radius must cover at least one 3-D coarse-grid neighbour step")

    os.makedirs(OUT_DIR, exist_ok=True)
    env, threat, dynamics, planner = build_models()
    baseline = planner.baseline_manager.ensure(
        planner, np.asarray(START, dtype=float), np.asarray(GOAL, dtype=float),
        force=force_baseline)
    planner.set_baseline(baseline)

    rows = load_interim() if resume else []
    combos = sorted(COMBOS, key=lambda p: (p[1], p[0]))
    n_todo = sum(not already_done(rows, wt, wf) for wt, wf in combos)
    done = 0

    start_arr = np.asarray(START, dtype=float)
    goal_arr = np.asarray(GOAL, dtype=float)

    for wt, wf in combos:
        if already_done(rows, wt, wf):
            continue
        done += 1
        print(f"\n[{done}/{n_todo}] w_T={wt:.1f}, w_R={W_RISK_FIXED:.1f}, w_F={wf:.1f}")
        planner.w_time = float(wt)
        planner.w_risk = float(W_RISK_FIXED)
        planner.w_fuel = float(wf)
        planner.set_baseline(baseline)

        t0 = time.time()
        raw_stats = None
        smooth = None
        try:
            raw_path, raw_stats = planner._run_astar(
                start_arr, goal_arr, compute_baseline=False)
            final_path, smooth, final_stats, _, _ = finalise_path(
                raw_path, planner, dynamics, threat)
            if final_path is None:
                raise RuntimeError(smooth.get("message", "B-spline smoothing failed"))
            elapsed = time.time() - t0
            row = make_success_row(
                wt, wf, raw_path, raw_stats, final_path, smooth,
                final_stats, elapsed, baseline)
            rows.append(row)
            repair_tags = []
            if row.get("高度B样条修复", False):
                repair_tags.append("alt-repair")
            if row.get("爬升角B样条修复", False):
                repair_tags.append("climb-repair")
            repair_note = f", repair={'+'.join(repair_tags)}" if repair_tags else ""
            print(
                f"  final: T={row['飞行时间_min']:.2f} min, "
                f"C={row['累积暴露_C']:.4f}, F={row['燃油消耗_kg']:.4f} kg, "
                f"J={row['统一目标_J']:.5f}, Rmin={row['最小转弯半径_m']:.1f} m, "
                f"Js={row['平滑评价_Js']:.5f}{repair_note}")
        except (RuntimeError, ValueError) as exc:
            elapsed = time.time() - t0
            print(f"  X failed after {elapsed:.1f}s: {exc}")
            rows.append(make_failure_row(
                wt, wf, elapsed, exc, raw_stats=raw_stats, smooth=smooth))

        with open(INTERIM_PKL, "wb") as f:
            pickle.dump(rows, f, protocol=pickle.HIGHEST_PROTOCOL)

    return rows, baseline


def export_excel(rows, baseline):
    df = pd.DataFrame(rows)
    valid = df.dropna(subset=["飞行时间_s"]) if "飞行时间_s" in df else df.iloc[0:0]

    preferred = [
        "权重_w_time", "权重_w_risk", "权重_w_fuel",
        "飞行距离_km", "飞行时间_min", "累积暴露_C", "燃油消耗_kg",
        "归一化时间", "归一化暴露", "归一化燃油", "统一目标_J",
        "绕行增加_%", "最大高度_km", "航点数",
        "最小转弯半径_m", "最大曲率_1_m", "最大爬升角_deg",
        "平滑评价_Js", "曲率变化积分_JkappaPrime", "末端航向误差_deg",
        "速度变化率最大_m_s2", "平滑成功",
        "高度包络满足", "高度B样条修复", "高度修复缓冲_m",
        "爬升角B样条修复", "爬升修复斜率比例",
        "平滑最大偏离_m", "平滑信赖域_m", "粗网格3D对角_m",
        "选中_s_m2", "s候选数",
        "A星原始距离_km", "A星原始时间_s", "A星原始暴露_C",
        "A星原始燃油_kg", "A星原始J", "A星原始航点数",
        "迭代次数", "扩展节点数", "计算耗时_s",
        "飞行时间_s", "最大高度_m", "错误", "时间戳", "schema_version",
    ]
    ordered = [c for c in preferred if c in df.columns]
    extras = [c for c in df.columns if c not in ordered]
    df_out = df[ordered + extras].sort_values(["权重_w_fuel", "权重_w_time"])

    with pd.ExcelWriter(EXCEL_FILE, engine="openpyxl") as writer:
        df_out.to_excel(writer, sheet_name="全部数据", index=False)
        pd.DataFrame([{
            "baseline_version": baseline.get("version"),
            "T_min": baseline["T_min"], "T_max": baseline["T_max"],
            "C_min": baseline["C_min"], "C_max": baseline["C_max"],
            "F_min": baseline["F_min"], "F_max": baseline["F_max"],
            "L0": baseline["L0"], "T0": baseline["T0"], "C0": baseline["C0"],
            "sweep_schema_version": SWEEP_SCHEMA_VERSION,
            "grid_xy_m": GRID_RES_XY, "grid_z_m": GRID_RES_Z,
            "coarse_grid_3d_diagonal_m": COARSE_GRID_DIAGONAL,
            "smoothing_trust_radius_m": SMOOTHING_TRUST_RADIUS,
        }]).to_excel(writer, sheet_name="归一化基准", index=False)

        for metric, sheet_name in [
            ("累积暴露_C", "透视_暴露"),
            ("飞行时间_min", "透视_时间_min"),
            ("燃油消耗_kg", "透视_油耗"),
            ("统一目标_J", "透视_J"),
            ("最小转弯半径_m", "透视_最小转弯半径"),
            ("平滑评价_Js", "透视_平滑评价"),
        ]:
            if metric not in valid.columns:
                continue
            pivot = valid.pivot_table(
                index="权重_w_time", columns="权重_w_fuel",
                values=metric, aggfunc="first")
            pivot.columns = [f"w_fuel={c:.1f}" for c in pivot.columns]
            pivot.index.name = "w_time \\ w_fuel"
            pivot.to_excel(writer, sheet_name=sheet_name[:31])

    print(f"\n✓ Excel saved: {EXCEL_FILE}")
    print(f"  rows={len(df_out)}, valid final trajectories={len(valid)}")


def print_summary(rows):
    df = pd.DataFrame(rows)
    if "飞行时间_s" not in df:
        return
    valid = df.dropna(subset=["飞行时间_s"])
    if valid.empty:
        return
    print("\n" + "=" * 78)
    print("  Sweep summary (final smoothed trajectories)")
    print("=" * 78)
    print(f"  valid: {len(valid)} / {len(df)}")
    print(f"  time: {valid['飞行时间_min'].min():.3f} .. {valid['飞行时间_min'].max():.3f} min")
    print(f"  risk: {valid['累积暴露_C'].min():.6f} .. {valid['累积暴露_C'].max():.6f} exposure-s")
    print(f"  fuel: {valid['燃油消耗_kg'].min():.6f} .. {valid['燃油消耗_kg'].max():.6f} kg")
    print(f"  min turn radius: {valid['最小转弯半径_m'].min():.2f} .. {valid['最小转弯半径_m'].max():.2f} m")
    print(f"  max climb angle: {valid['最大爬升角_deg'].min():.3f} .. {valid['最大爬升角_deg'].max():.3f} deg")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", action="store_true", default=True)
    parser.add_argument("--no-resume", dest="resume", action="store_false")
    parser.add_argument("--force-baseline", action="store_true",
                        help="recompute the three-anchor baseline.pkl")
    parser.add_argument("--fresh", action="store_true",
                        help="fresh V5.3 run: equivalent to --no-resume --force-baseline")
    args = parser.parse_args()

    if args.fresh:
        args.resume = False
        args.force_baseline = True

    rows, baseline = run_sweep(
        resume=args.resume, force_baseline=args.force_baseline)
    print_summary(rows)
    export_excel(rows, baseline)


if __name__ == "__main__":
    main()
