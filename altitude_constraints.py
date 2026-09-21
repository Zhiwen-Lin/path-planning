#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mission altitude-envelope hard constraints.

The mission endpoints are allowed to remain at their original altitude (0 m in
the paper experiments).  Away from the departure/terminal transition zones, the
route must stay within the en-route altitude band.

For a point p, the local lower bound is

    h_min(p) = min(h_from_start, h_to_goal)

where each transition ramps linearly from the endpoint altitude to the en-route
minimum over ``transition_distance``.  This keeps take-off/landing endpoints
unchanged while preventing long ground-hugging paths in the mission interior.
"""

from __future__ import annotations

from typing import Sequence, Tuple
import numpy as np

DEFAULT_ENROUTE_MIN_ALT_M = 1000.0
DEFAULT_MAX_ALT_M = 10000.0
DEFAULT_TRANSITION_DISTANCE_M = 5000.0


def _as_point3(p: Sequence[float]) -> np.ndarray:
    a = np.asarray(p, dtype=float).reshape(-1)
    if a.size < 3:
        raise ValueError("Point must contain x, y, z")
    return a[:3]


def horizontal_distance(p: Sequence[float], q: Sequence[float]) -> float:
    p3 = _as_point3(p)
    q3 = _as_point3(q)
    return float(np.hypot(p3[0] - q3[0], p3[1] - q3[1]))


def minimum_allowed_altitude(
    point: Sequence[float],
    start: Sequence[float],
    goal: Sequence[float],
    enroute_min_alt: float = DEFAULT_ENROUTE_MIN_ALT_M,
    transition_distance: float = DEFAULT_TRANSITION_DISTANCE_M,
) -> float:
    """Return the local hard lower altitude bound in metres.

    The endpoint altitudes themselves are preserved.  With the default paper
    mission (start_z=goal_z=0 m), the lower bound ramps 0->1000 m over the
    first 5 km, stays at 1000 m in the mission interior, then ramps back to
    0 m over the final 5 km.
    """
    p = _as_point3(point)
    s = _as_point3(start)
    g = _as_point3(goal)

    h_cruise_min = float(enroute_min_alt)
    d_transition = float(transition_distance)
    if h_cruise_min < 0.0:
        raise ValueError("enroute_min_alt must be non-negative")
    if d_transition <= 0.0:
        raise ValueError("transition_distance must be positive")

    ds = horizontal_distance(p, s)
    dg = horizontal_distance(p, g)
    alpha_s = float(np.clip(ds / d_transition, 0.0, 1.0))
    alpha_g = float(np.clip(dg / d_transition, 0.0, 1.0))

    h_from_start = float(s[2] + (h_cruise_min - s[2]) * alpha_s)
    h_to_goal = float(g[2] + (h_cruise_min - g[2]) * alpha_g)

    # The lower of the two envelopes permits the departure/arrival transitions;
    # away from both endpoints both terms equal the en-route minimum.
    return float(min(h_from_start, h_to_goal, h_cruise_min))


def altitude_limits(
    point: Sequence[float],
    start: Sequence[float],
    goal: Sequence[float],
    enroute_min_alt: float = DEFAULT_ENROUTE_MIN_ALT_M,
    max_alt: float = DEFAULT_MAX_ALT_M,
    transition_distance: float = DEFAULT_TRANSITION_DISTANCE_M,
) -> Tuple[float, float]:
    return (
        minimum_allowed_altitude(
            point, start, goal,
            enroute_min_alt=enroute_min_alt,
            transition_distance=transition_distance,
        ),
        float(max_alt),
    )


def altitude_margin(
    point: Sequence[float],
    start: Sequence[float],
    goal: Sequence[float],
    enroute_min_alt: float = DEFAULT_ENROUTE_MIN_ALT_M,
    max_alt: float = DEFAULT_MAX_ALT_M,
    transition_distance: float = DEFAULT_TRANSITION_DISTANCE_M,
) -> float:
    """Signed clearance to the nearer altitude bound (m).

    Positive means feasible, zero means on a bound, negative means violated.
    """
    p = _as_point3(point)
    lo, hi = altitude_limits(
        p, start, goal,
        enroute_min_alt=enroute_min_alt,
        max_alt=max_alt,
        transition_distance=transition_distance,
    )
    return float(min(p[2] - lo, hi - p[2]))


def altitude_is_allowed(
    point: Sequence[float],
    start: Sequence[float],
    goal: Sequence[float],
    enroute_min_alt: float = DEFAULT_ENROUTE_MIN_ALT_M,
    max_alt: float = DEFAULT_MAX_ALT_M,
    transition_distance: float = DEFAULT_TRANSITION_DISTANCE_M,
    tolerance: float = 1e-6,
) -> bool:
    p = _as_point3(point)
    lo, hi = altitude_limits(
        p, start, goal,
        enroute_min_alt=enroute_min_alt,
        max_alt=max_alt,
        transition_distance=transition_distance,
    )
    tol = float(max(tolerance, 0.0))
    return bool((p[2] >= lo - tol) and (p[2] <= hi + tol))


def minimum_transition_distance(
    endpoint_alt: float,
    enroute_min_alt: float,
    max_climb_angle_deg: float,
) -> float:
    """Kinematic horizontal distance required to reach the en-route minimum."""
    dh = max(float(enroute_min_alt) - float(endpoint_alt), 0.0)
    angle = np.radians(float(max_climb_angle_deg))
    tan_angle = float(np.tan(angle))
    if dh <= 0.0:
        return 0.0
    if tan_angle <= 1e-12:
        return float("inf")
    return float(dh / tan_angle)


def validate_transition_distance(
    start: Sequence[float],
    goal: Sequence[float],
    enroute_min_alt: float,
    transition_distance: float,
    max_climb_angle_deg: float,
) -> Tuple[bool, float]:
    """Check that the chosen ramp is not steeper than the aircraft limit."""
    s = _as_point3(start)
    g = _as_point3(goal)
    required = max(
        minimum_transition_distance(s[2], enroute_min_alt, max_climb_angle_deg),
        minimum_transition_distance(g[2], enroute_min_alt, max_climb_angle_deg),
    )
    return bool(float(transition_distance) + 1e-9 >= required), float(required)
