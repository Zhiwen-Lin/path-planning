#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Three-way speed/acceleration comparison
=======================================

Compares:
  1. Coarse A* polyline
  2. Fine A* raw polyline
  3. Fine A* + B-spline final trajectory

Important:
  - Raw polylines are densified ONLY for kinematic integration.
  - Their curvature constraint is evaluated at ORIGINAL A* vertices and then
    mapped onto the dense arclength grid. This avoids the old artifact where
    a 100 m linear interpolation changed the apparent corner radius.
  - The B-spline uses analytic curvature from spline derivatives.
"""

import os
import pickle
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fixed_wing_dynamics import FixedWingDynamics
from trajectory_smoothing import spline_curvature, resample_by_arclength
from trajectory_data_loader import load_trajectory_data, trajectory_output_path
from pathlib import Path

COARSE_PKL = "output_5zones/planned_path_coarse.pkl"
FINE_PKL = "output_5zones/planned_path.pkl"
OUTPUT = "output_5zones/speed_profile_three_way_english.png"
DS_PLOT = 100.0


def _deduplicate_polyline(path):
    P = np.asarray(path, dtype=float)
    if len(P) <= 1:
        return P

    keep = [0]
    for i in range(1, len(P)):
        if np.linalg.norm(P[i] - P[keep[-1]]) > 1e-9:
            keep.append(i)
    return P[keep]


def _densify_polyline_preserve_vertex_curvature(path, dyn, ds_target=100.0):
    """
    Densify a raw A* polyline without changing its curvature proxy.

    The old script first interpolated to 100 m and THEN applied Menger
    curvature. At a sharp polyline corner, that makes the radius depend on
    interpolation spacing and creates artificial stall-speed dips.

    Here:
      1) Menger curvature is computed on ORIGINAL A* vertices.
      2) The polyline is densified for forward/backward acceleration planning.
      3) Only the original vertices carry the original curvature constraints.
    """
    P = _deduplicate_polyline(path)
    n = len(P)

    if n < 2:
        return P, np.zeros(n)

    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s_orig = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s_orig[-1])

    k_orig = np.zeros(n)
    for i in range(1, n - 1):
        R = dyn.get_path_curvature_radius(P[i - 1], P[i], P[i + 1])
        if 1e-6 < R < float("inf"):
            k_orig[i] = 1.0 / R

    # Include every original vertex exactly, plus a regular arclength grid.
    regular = np.arange(0.0, total, max(ds_target, 1e-6))
    s_dense = np.unique(np.concatenate([regular, s_orig, [total]]))

    dense = np.column_stack([
        np.interp(s_dense, s_orig, P[:, j]) for j in range(3)
    ])

    k_dense = np.zeros(len(s_dense))
    for i in range(1, n - 1):
        idx = int(np.searchsorted(s_dense, s_orig[i]))
        if idx < len(s_dense) and abs(s_dense[idx] - s_orig[i]) < 1e-6:
            k_dense[idx] = k_orig[i]

    return dense, k_dense


def _profile_time(path, speeds):
    P = np.asarray(path, dtype=float)
    ds = np.linalg.norm(np.diff(P, axis=0), axis=1)
    v_avg = np.maximum((speeds[:-1] + speeds[1:]) / 2.0, 1e-3)
    return float(np.sum(ds / v_avg))


def _operating_min(s, v, dyn):
    return float(np.min(v))


def build_profiles(coarse_data, fine_data, dyn):
    profiles = {}

    # 1) Coarse raw
    coarse_dense, coarse_k = _densify_polyline_preserve_vertex_curvature(
        coarse_data["path"], dyn, DS_PLOT
    )
    s, v, k, a = dyn.compute_speed_profile(
        coarse_dense,
        curvatures_in=coarse_k,
        strict_feasibility=False
    )
    profiles["Coarse A*"] = {
        "path": coarse_dense, "s": s, "v": v, "k": k, "a": a,
        "physical_label": "polyline proxy"
    }

    is_sweep = fine_data.get('source_kind') == 'sweep'
    # main.py has a fine search; the coarse-only sweep has two stages to plot.
    if not is_sweep:
        fine_dense, fine_k = _densify_polyline_preserve_vertex_curvature(
            fine_data['path_raw'], dyn, DS_PLOT)
        s, v, k, a = dyn.compute_speed_profile(
            fine_dense, curvatures_in=fine_k, strict_feasibility=False)
        profiles["Fine A* raw"] = {
            "path": fine_dense, "s": s, "v": v, "k": k, "a": a,
            "physical_label": "polyline proxy"
        }

    # 3) Fine + B-spline
    tck = fine_data.get("smooth_result", {}).get("tck", None)
    if tck is None:
        raise RuntimeError(
            "Fine result does not contain a B-spline tck. "
            "Run the revised main.py first."
        )

    # Final figures use exactly the samples and speeds used for saved T/C/F.
    smooth_dense = np.asarray(fine_data['path'])
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(smooth_dense, axis=0), axis=1))]
    v = np.asarray(fine_data['speed_profile'])
    k = np.asarray(fine_data['curvature_smooth'])
    a = np.asarray(fine_data['accel_profile'])
    dyn.validate_speed_profile(smooth_dense, v, k)
    final_label = 'Coarse + B-spline' if is_sweep else 'Fine + B-spline'
    profiles[final_label] = {
        "path": smooth_dense, "s": s, "v": v, "k": k, "a": a,
        "physical_label": "final physical profile"
    }

    return profiles


def main():
    fine_data = load_trajectory_data(allow_raw=False)
    if fine_data['source_kind'] == 'sweep':
        coarse_data = {'path': fine_data['path_raw']}
    else:
        coarse_data = load_trajectory_data(Path(fine_data['source_file']).with_name('planned_path_coarse.pkl'))
        for key in ('trajectory_signature', 'w_time', 'w_risk', 'w_fuel', 'start', 'goal'):
            if not np.array_equal(coarse_data[key], fine_data[key]):
                raise ValueError(f'Coarse and fine caches belong to different experiments: {key}')

    dyn = FixedWingDynamics()
    profiles = build_profiles(coarse_data, fine_data, dyn)

    fig, axes = plt.subplots(3, 1, figsize=(14, 11))
    fig.patch.set_facecolor("white")

    # --------------------------------------------------------------
    # (a) Speed
    # --------------------------------------------------------------
    ax = axes[0]
    for name, p in profiles.items():
        ax.plot(p["s"] / 1000.0, p["v"] * 3.6, linewidth=2.2, label=name)

    ax.axhline(dyn.cruise_speed * 3.6, linestyle="--", linewidth=1.8,
               label=f"Cruise ({dyn.cruise_speed * 3.6:.0f} km/h)")
    ax.axhline(dyn.v_stall * 3.6, linestyle=":", linewidth=1.8,
               label=f"Stall ({dyn.v_stall * 3.6:.0f} km/h)")
    ax.set_ylabel("Speed (km/h)")
    ax.set_title("(a) Speed profile comparison")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.legend(ncol=2)

    # --------------------------------------------------------------
    # (b) Tangential acceleration
    # --------------------------------------------------------------
    ax = axes[1]
    for name, p in profiles.items():
        ax.plot(p["s"] / 1000.0, p["a"], linewidth=1.8, label=name)

    ax.axhline(dyn.max_horizontal_accel, linestyle="--", linewidth=1.5,
               label=f"+a limit ({dyn.max_horizontal_accel:.1f} m/s²)")
    ax.axhline(-dyn.max_decel, linestyle="--", linewidth=1.5,
               label=f"-a limit ({dyn.max_decel:.1f} m/s²)")
    ax.axhline(0.0, linewidth=1.0)
    ax.set_ylabel("Tangential acceleration (m/s²)")
    ax.set_title("(b) Forward-backward acceleration consistency")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.legend(ncol=2)

    # --------------------------------------------------------------
    # (c) Speed deficit from cruise -- makes differences visible
    # --------------------------------------------------------------
    ax = axes[2]
    for name, p in profiles.items():
        deficit = np.maximum(dyn.cruise_speed - p["v"], 0.0) * 3.6
        ax.plot(p["s"] / 1000.0, deficit, linewidth=2.0, label=name)

    ax.set_xlabel("Cumulative distance (km)")
    ax.set_ylabel("Cruise-speed deficit (km/h)")
    ax.set_title("(c) Where geometry / vertical-rate constraints force slowing")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.legend()

    fig.suptitle(
        "Speed Planning Comparison — " + fine_data['source_kind'],
        fontsize=15, fontweight="bold"
    )
    plt.tight_layout(rect=[0, 0, 1, 0.97])

    output = trajectory_output_path(Path(OUTPUT).name)
    plt.savefig(output, dpi=300, bbox_inches="tight", facecolor="white")
    plt.close(fig)

    # Console summary
    print("\n" + "=" * 92)
    print("Speed-profile summary")
    print("=" * 92)
    print(
        f"{'Stage':22s} {'N':>7s} {'L(km)':>9s} {'T(s)':>9s} "
        f"{'Vmin*':>9s} {'Vmean':>9s} {'|a|max':>9s}"
    )

    for name, p in profiles.items():
        L = p["s"][-1] if len(p["s"]) else 0.0
        T = _profile_time(p["path"], p["v"])
        vmin = _operating_min(p["s"], p["v"], dyn)
        vmaxa = float(np.max(np.abs(p["a"]))) if len(p["a"]) else 0.0
        print(
            f"{name:22s} {len(p['s']):7d} {L/1000:9.2f} {T:9.1f} "
            f"{vmin*3.6:9.1f} {np.mean(p['v'])*3.6:9.1f} {vmaxa:9.2f}"
        )

    print("\n* Vmin includes every point; the airborne start speed is the stall speed.")
    print("  Raw A* curves are polyline proxies; the B-spline curve is the final")
    print("  physically validated profile using analytic curvature.")
    print(f"\nSaved: {output}")


if __name__ == "__main__":
    main()
