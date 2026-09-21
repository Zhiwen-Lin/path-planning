#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Normalization baseline management for the multi-objective A* planner.

The normalization ranges are built from a three-anchor payoff table:
    time anchor : (w_T, w_R, w_F) = (1, 0, 0)
    risk anchor : (0, 1, 0)
    fuel anchor : (0, 0, 1)

For each objective, the corresponding anchor supplies the ideal (minimum)
value.  The maximum used for normalization is the largest value observed in
those three anchor solutions, i.e. a standard payoff-table approximation of
the nadir point.  This is reproducible and avoids the old semantic inversion
where C_min was numerically larger than C_max.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
import pickle
from dataclasses import asdict, is_dataclass
from typing import Any, Dict, Iterable, Optional

import numpy as np

BASELINE_VERSION = "three-anchor-payoff-v2-altitude-envelope"
DEFAULT_BASELINE_PATH = os.path.join("output_5zones", "baseline.pkl")


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if is_dataclass(value):
        return {k: _jsonable(v) for k, v in asdict(value).items()}
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)


def planner_signature(planner, start: Iterable[float], goal: Iterable[float]) -> Dict[str, Any]:
    """Return a deterministic configuration signature for cache validation."""
    env = planner.env
    threat = planner.threat
    dyn = planner.dynamics

    radars = [
        {
            "x": float(r.x), "y": float(r.y), "R": float(r.R),
            "rcs_factor": float(getattr(r, "rcs_factor", 1.0)),
            "name": str(getattr(r, "name", "Radar")),
        }
        for r in getattr(env, "radar_zones", [])
    ]
    fire = [
        {
            "x": float(f.x), "y": float(f.y),
            "horizontal_radius": float(f.horizontal_radius),
            "max_altitude": float(f.max_altitude),
            "name": str(getattr(f, "name", "FireZone")),
        }
        for f in getattr(env, "fire_zones", [])
    ]

    dyn_fields = {}
    if dyn is not None:
        # Only fields that affect time / fuel / feasibility / speed.
        for name in (
            "mass", "g", "wing_area", "v_stall", "cruise_speed",
            "max_horizontal_speed", "max_ascend_speed", "max_descend_speed",
            "max_horizontal_accel", "max_decel", "max_bank_angle_deg",
            "max_climb_angle_deg", "cd0", "aspect_ratio", "oswald_efficiency",
            "climb_efficiency", "descent_recovery", "propulsive_efficiency",
            "fuel_heat_value_j_per_kg",
        ):
            if hasattr(dyn, name):
                dyn_fields[name] = float(getattr(dyn, name))

    payload = {
        "baseline_version": BASELINE_VERSION,
        "start": [float(x) for x in start],
        "goal": [float(x) for x in goal],
        "grid_res_xy": float(planner.grid_res_xy),
        "grid_res_z": float(planner.grid_res_z),
        "v_cruise": float(planner.v_cruise),
        "n_edge_samples": int(planner.n_edge_samples),
        "epsilon": float(planner.epsilon),
        "bounds": [
            float(planner.x_min), float(planner.x_max),
            float(planner.y_min), float(planner.y_max),
            float(planner.z_min), float(planner.z_max),
        ],
        "altitude_envelope": {
            "enroute_min_alt": float(getattr(planner, "enroute_min_alt", 0.0)),
            "transition_distance": float(getattr(planner, "transition_distance", 0.0)),
            "max_alt": float(getattr(planner, "z_max", 0.0)),
        },
        "threat": {
            "model": str(getattr(threat, "model", "")),
            "edge_pd": float(getattr(threat, "edge_pd", 0.0)),
            "power_n": float(getattr(threat, "power_n", 0.0)),
            "R_inner_ratio": float(getattr(threat, "R_inner_ratio", 0.0)),
            "sigmoid_n": float(getattr(threat, "sigmoid_n", 0.0)),
        },
        "radars": radars,
        "fire_zones": fire,
        "dynamics": dyn_fields,
    }
    canonical = json.dumps(_jsonable(payload), sort_keys=True, separators=(",", ":"))
    return {
        "payload": payload,
        "sha256": hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
    }


def safe_range(lo: float, hi: float, name: str, eps: float = 1e-9) -> float:
    span = float(hi) - float(lo)
    if not np.isfinite(span) or span <= eps:
        raise ValueError(
            f"Normalization range for {name} is invalid: min={lo!r}, max={hi!r}. "
            "The three anchor solutions do not provide a usable payoff-table range."
        )
    return span


def objective_scales(baseline: Dict[str, Any]) -> Dict[str, float]:
    return {
        "T": safe_range(baseline["T_min"], baseline["T_max"], "time"),
        "C": safe_range(baseline["C_min"], baseline["C_max"], "risk"),
        "F": safe_range(baseline["F_min"], baseline["F_max"], "fuel"),
    }


def normalize_metrics(metrics: Dict[str, float], baseline: Dict[str, Any]) -> Dict[str, float]:
    """Normalize T, Crisk and F using the exact paper formula (no clipping)."""
    scale = objective_scales(baseline)
    return {
        "T_n": (float(metrics["T"]) - float(baseline["T_min"])) / scale["T"],
        "C_n": (float(metrics["Crisk"]) - float(baseline["C_min"])) / scale["C"],
        "F_n": (float(metrics["F"]) - float(baseline["F_min"])) / scale["F"],
    }


def weighted_objective(metrics: Dict[str, float], baseline: Dict[str, Any],
                       w_time: float, w_risk: float, w_fuel: float) -> Dict[str, float]:
    n = normalize_metrics(metrics, baseline)
    J = float(w_time) * n["T_n"] + float(w_risk) * n["C_n"] + float(w_fuel) * n["F_n"]
    return {**n, "J": float(J)}


def weighted_increment(edge_metrics: Dict[str, float], baseline: Dict[str, Any],
                       w_time: float, w_risk: float, w_fuel: float) -> float:
    """Additive A* edge cost equivalent to the normalized total objective.

    The offsets (-T_min, -C_min, -F_min) are path-independent constants and
    therefore must NOT be subtracted on every edge.  Omitting them preserves
    exactly the same path ordering as the reported total J while keeping A*
    additive.
    """
    scale = objective_scales(baseline)
    return float(
        float(w_time) * float(edge_metrics["T"]) / scale["T"]
        + float(w_risk) * float(edge_metrics["Crisk"]) / scale["C"]
        + float(w_fuel) * float(edge_metrics["F"]) / scale["F"]
    )


class BaselineManager:
    """Load, validate, compute and save the shared ``baseline.pkl``."""

    def __init__(self, path: str = DEFAULT_BASELINE_PATH):
        self.path = path

    def load(self) -> Optional[Dict[str, Any]]:
        if not os.path.exists(self.path):
            return None
        try:
            with open(self.path, "rb") as f:
                data = pickle.load(f)
        except Exception:
            return None
        if not isinstance(data, dict) or data.get("version") != BASELINE_VERSION:
            return None
        try:
            objective_scales(data)
        except Exception:
            return None
        return data

    def save(self, baseline: Dict[str, Any]) -> None:
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "wb") as f:
            pickle.dump(baseline, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, self.path)

    def matches(self, baseline: Dict[str, Any], planner, start, goal) -> bool:
        expected = planner_signature(planner, start, goal)["sha256"]
        return baseline.get("signature", {}).get("sha256") == expected

    @staticmethod
    def apply(planner, baseline: Dict[str, Any]) -> None:
        planner.baseline = baseline
        planner.T_min = float(baseline["T_min"])
        planner.T_max = float(baseline["T_max"])
        planner.C_min = float(baseline["C_min"])
        planner.C_max = float(baseline["C_max"])
        planner.F_min = float(baseline["F_min"])
        planner.F_max = float(baseline["F_max"])
        planner.L0 = float(baseline.get("L0", 0.0))
        planner.T0 = float(baseline.get("T0", 0.0))
        planner.C0 = float(baseline.get("C0", 0.0))
        planner.use_normalization = True

    def ensure(self, planner, start, goal, force: bool = False) -> Dict[str, Any]:
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)

        cached = None if force else self.load()
        if cached is not None and self.matches(cached, planner, start, goal):
            print(f"  ✓ Loading shared normalization baseline: {self.path}")
            self.apply(planner, cached)
            return cached

        baseline = self.compute(planner, start, goal)
        self.save(baseline)
        self.apply(planner, baseline)
        print(f"  ✓ Saved shared normalization baseline: {self.path}")
        return baseline

    def compute(self, planner, start: np.ndarray, goal: np.ndarray) -> Dict[str, Any]:
        print("\n" + "=" * 72)
        print("  Normalization baseline: three single-objective anchor paths")
        print("=" * 72)

        saved = {
            "w_time": planner.w_time,
            "w_risk": planner.w_risk,
            "w_fuel": planner.w_fuel,
            "use_normalization": planner.use_normalization,
            "baseline": getattr(planner, "baseline", None),
            "L0": planner.L0,
            "T0": planner.T0,
            "C0": planner.C0,
        }

        planner.use_normalization = False
        planner.baseline = None
        planner._compute_baseline(start, goal)
        L0, T0, C0 = planner.L0, planner.T0, planner.C0

        anchors = {}
        specs = (
            ("time", (1.0, 0.0, 0.0)),
            ("risk", (0.0, 1.0, 0.0)),
            ("fuel", (0.0, 0.0, 1.0)),
        )

        try:
            for idx, (name, weights) in enumerate(specs, 1):
                planner.w_time, planner.w_risk, planner.w_fuel = weights
                print(
                    f"\n[{idx}/3] {name}-optimal anchor "
                    f"(w_T,w_R,w_F)=({weights[0]:g},{weights[1]:g},{weights[2]:g})"
                )
                path, stats = planner._run_astar(start, goal, compute_baseline=False)
                anchors[name] = {
                    "weights": weights,
                    "stats": {
                        k: float(stats[k])
                        for k in ("L", "T", "Crisk", "F")
                    },
                    # Anchor paths make the baseline auditable and reproducible.
                    "path": [np.asarray(p, dtype=float) for p in path],
                }
                print(
                    f"  T={stats['T']:.2f} s, C={stats['Crisk']:.6f} exposure-s, "
                    f"F={stats['F']:.6f} kg"
                )
        finally:
            planner.w_time = saved["w_time"]
            planner.w_risk = saved["w_risk"]
            planner.w_fuel = saved["w_fuel"]
            planner.use_normalization = saved["use_normalization"]
            planner.baseline = saved["baseline"]
            planner.L0, planner.T0, planner.C0 = saved["L0"], saved["T0"], saved["C0"]

        rows = [anchors[k]["stats"] for k in ("time", "risk", "fuel")]
        baseline = {
            "version": BASELINE_VERSION,
            "method": "three_anchor_payoff_table_nadir_approximation",
            "signature": planner_signature(planner, start, goal),
            "T_min": float(anchors["time"]["stats"]["T"]),
            "C_min": float(anchors["risk"]["stats"]["Crisk"]),
            "F_min": float(anchors["fuel"]["stats"]["F"]),
            "T_max": float(max(r["T"] for r in rows)),
            "C_max": float(max(r["Crisk"] for r in rows)),
            "F_max": float(max(r["F"] for r in rows)),
            "L0": float(L0),
            "T0": float(T0),
            "C0": float(C0),
            "anchors": anchors,
        }
        # Fail early if any range is degenerate.
        objective_scales(baseline)

        print("\n[Payoff-table ranges]")
        print(f"  T: {baseline['T_min']:.3f} -> {baseline['T_max']:.3f} s")
        print(f"  C: {baseline['C_min']:.6f} -> {baseline['C_max']:.6f} exposure-s")
        print(f"  F: {baseline['F_min']:.6f} -> {baseline['F_max']:.6f} kg")
        print("=" * 72)
        return baseline
