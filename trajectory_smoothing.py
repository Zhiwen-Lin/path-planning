#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
trajectory_smoothing.py - 轨迹光滑处理模块
==========================================
使用三次 B 样条对 A* 规划的网格路径进行光滑处理。

在流程中的位置（同一次运行内完成，无需二次运行）:
    粗网格 A* → 通路 → 细网格 A* → 【B样条光滑】→ 速度剖面 → 可视化

为什么需要光滑:
    A* 在离散网格上搜索，输出的是网格对齐的折线。折线在拐点处曲率
    不连续（0 → ∞ → 0），而转弯限速依赖曲率，会因此产生锯齿状的
    速度剖面。三次 B 样条保证 C² 连续，曲率平滑变化，速度剖面才可信。

--- 三个关键设计决策 ---

1) 曲率一律用样条解析导数，不用 Menger 三点公式
       k(u) = |r'(u) × r''(u)| / |r'(u)|³
   三点公式在"点距 << 曲率半径"时，叉积退化为两个近平行向量相减，
   有效位被浮点相消吃掉，结果随采样密度发散：同一条样条曲线
   n=256→4096 时三点最小半径 351→57 m，而解析值稳定在 104 m。
   用三点公式做验证，会拒绝一条并不存在的尖角。

2) s_param 按量纲定标
   scipy 中 s 是残差平方和上界 Σ|P_i - C(u_i)|² ≤ s，量纲 m²。
   要允许平均每点偏离 δ 米，需 s ≈ n·δ²。旧版默认 n·0.5 不含任何
   长度量，对 512 点路径等于总偏差预算 256 m²（每点 0.3 m），
   在 200 m 网格上等于"不许动"，必然光滑不掉锯齿。

3) 违反约束时增大 s，不是减小
   s 越小越贴合原折线 → 曲率越大 → 越违反转弯半径约束。旧版
   `s *= 0.5` 的重试方向是反的，三次尝试单调走向更坏。
   本版改为在一组按量纲生成的候选 s 上升序搜索，取"最小的满足
   约束者"，在满足运动学的前提下把偏离原规划路径压到最小。

作者：Assistant
日期：2026-08-07（2026-08-08 修正 s 定标 / 重试方向 / 曲率算法）
"""

import numpy as np
from scipy.interpolate import splprep, splev
from typing import List, Tuple, Dict, Optional, Callable

from fixed_wing_dynamics import FixedWingDynamics

SMOOTHING_VERSION = "V5.3-adaptive-altitude-climb-envelope-terminal-heading"


# ============================================================================
# 样条解析曲率
# ============================================================================

def spline_curvature(tck, u_query: np.ndarray) -> np.ndarray:
    """
    用样条解析导数计算曲率（1/m）

        k(u) = |r'(u) × r''(u)| / |r'(u)|³

    这是曲率的定义式，与采样密度无关。务必用它替代三点公式：
    Menger 曲率在点距远小于曲率半径时会因浮点相消而发散。

    参数:
        tck: splprep 返回的样条表示 (knots, coefficients, degree)
        u_query: 参数采样点，形如 np.linspace(0, 1, n)

    返回:
        curvatures: np.ndarray, shape (len(u_query),), 单位 1/m
    """
    d1 = np.asarray(splev(u_query, tck, der=1), dtype=float).T   # r'  (n,3)
    d2 = np.asarray(splev(u_query, tck, der=2), dtype=float).T   # r'' (n,3)

    cross = np.cross(d1, d2)
    num = np.linalg.norm(cross, axis=1)
    den = np.linalg.norm(d1, axis=1) ** 3

    # |r'| 退化处（参数化奇点）视为直线，曲率 0
    return np.where(den > 1e-12, num / np.maximum(den, 1e-12), 0.0)


def spline_min_radius(tck, n_probe: int = 4096) -> float:
    """
    样条的最小曲率半径（米）。用密采样求解析曲率的极大值。

    n_probe 只影响"找极值找得准不准"，不影响曲率值本身
    （解析公式与采样密度无关），所以可以放心取大。
    """
    k = spline_curvature(tck, np.linspace(0.0, 1.0, n_probe))
    k_max = float(np.max(k))
    return float('inf') if k_max < 1e-15 else 1.0 / k_max



def spline_max_climb_angle(tck, n_probe: int = 4096) -> float:
    """Maximum absolute 3-D flight-path angle of a spline in degrees.

    The angle is evaluated from analytic B-spline derivatives:

        gamma = atan2(|dz/du|, sqrt((dx/du)^2 + (dy/du)^2))

    so the result is independent of output sampling spacing.
    """
    u = np.linspace(0.0, 1.0, max(int(n_probe), 256))
    d1 = np.asarray(splev(u, tck, der=1), dtype=float).T
    horizontal = np.linalg.norm(d1[:, :2], axis=1)
    angles = np.degrees(
        np.arctan2(np.abs(d1[:, 2]), np.maximum(horizontal, 1e-12))
    )
    return float(np.max(angles)) if len(angles) else 0.0


def curvature_variation_integral(tck, n_probe: int = 4096) -> float:
    """Return J_{kappa'} = integral (d kappa / ds)^2 ds (unit: 1/m^3)."""
    u = np.linspace(0.0, 1.0, int(n_probe))
    pts = np.asarray(splev(u, tck), dtype=float).T
    kappa = spline_curvature(tck, u)
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s_arc = np.concatenate([[0.0], np.cumsum(ds)])
    if len(s_arc) < 3 or s_arc[-1] < 1e-9:
        return 0.0
    # np.gradient accepts non-uniform coordinates and avoids dividing by a
    # nominal parameter u, which would give the wrong physical derivative.
    dk_ds = np.gradient(kappa, s_arc, edge_order=1)
    return float(np.trapz(dk_ds ** 2, s_arc))


def infer_terminal_heading(path: np.ndarray, lookback_distance: float = 1000.0) -> np.ndarray:
    """Infer a robust desired terminal *horizontal* heading from the raw path.

    A single final A* edge is grid-direction quantized and can cause the very
    terminal swing we are trying to remove.  We therefore use a look-back chord
    over approximately ``lookback_distance`` metres.
    """
    P = np.asarray(path, dtype=float)
    if len(P) < 2:
        return np.array([1.0, 0.0], dtype=float)
    goal = P[-1]
    acc = 0.0
    idx = len(P) - 2
    for i in range(len(P) - 2, -1, -1):
        acc += float(np.linalg.norm(P[i + 1] - P[i]))
        idx = i
        if acc >= float(lookback_distance):
            break
    v = goal[:2] - P[idx, :2]
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        v = goal[:2] - P[-2, :2]
        n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.array([1.0, 0.0], dtype=float)
    return v / n


def smoothing_objective(tck, raw_points: np.ndarray,
                        dynamics: FixedWingDynamics,
                        max_smoothing_deviation: Optional[float]) -> Dict[str, float]:
    """Evaluate one B-spline candidate for global ``s`` selection.

    The requested score is
        J_s = 0.4 J_smooth + 0.3 J_curvature
            + 0.2 J_deviation + 0.1 J_length.

    ``J_curvature`` is the dimensionless form of the explicit curvature
    continuity term J_{kappa'} = integral (d kappa / ds)^2 ds.  This directly
    penalizes sudden steering-rate changes rather than merely checking that the
    peak curvature is feasible.
    """
    u = np.linspace(0.0, 1.0, 4096)
    pts = np.asarray(splev(u, tck), dtype=float).T
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s_arc = np.concatenate([[0.0], np.cumsum(ds)])
    L = float(s_arc[-1]) if len(s_arc) else 0.0
    kappa = spline_curvature(tck, u)

    R_ref = max(float(dynamics.get_min_feasible_radius()), 1e-6)
    if L > 1e-9:
        # Average normalized bending energy: dimensionless and comparable
        # across candidate lengths.
        J_smooth = float(np.trapz((kappa * R_ref) ** 2, s_arc) / L)
        dk_ds = np.gradient(kappa, s_arc, edge_order=1)
        J_kappa_prime = float(np.trapz(dk_ds ** 2, s_arc))
    else:
        J_smooth = 0.0
        J_kappa_prime = 0.0

    # J_kappa_prime has units 1/m^3; multiply by R_ref^3 to make it
    # dimensionless while preserving its physical meaning.
    J_curvature = float(J_kappa_prime * R_ref ** 3)

    raw = np.asarray(raw_points, dtype=float)
    raw_length = float(np.sum(np.linalg.norm(np.diff(raw, axis=0), axis=1))) if len(raw) > 1 else L
    dev = _distances_to_polyline(pts, raw)
    rms_dev = float(np.sqrt(np.mean(dev ** 2))) if len(dev) else 0.0
    dev_scale = max(float(max_smoothing_deviation or 1.0), 1e-6)
    J_deviation = rms_dev / dev_scale
    J_length = L / max(raw_length, 1e-6)

    score = (0.4 * J_smooth + 0.3 * J_curvature
             + 0.2 * J_deviation + 0.1 * J_length)
    return {
        'J_s': float(score),
        'J_smooth': float(J_smooth),
        'J_curvature': float(J_curvature),
        'J_deviation': float(J_deviation),
        'J_length': float(J_length),
        'curvature_variation_integral': float(J_kappa_prime),
        'rms_raw_deviation': float(rms_dev),
        'spline_length': float(L),
        'raw_length': float(raw_length),
    }


# ============================================================================
# 弧长等距重采样
# ============================================================================

def resample_by_arclength(tck, ds_target: float,
                          n_probe: int = 8000) -> Tuple[np.ndarray, np.ndarray]:
    """
    沿样条按弧长等距采样

    为什么要等距: 均匀参数采样 u=linspace(0,1,n) 给出的点距并不均匀
    （B 样条参数化非弧长），会让速度剖面的 ds 忽大忽小。等弧长采样
    使 compute_speed_profile 的前向-后向扫描步长一致。

    参数:
        tck: 样条表示
        ds_target: 目标点距（米）
        n_probe: 弧长积分的探测点数

    返回:
        points: np.ndarray (m,3) - 采样点
        u_at:   np.ndarray (m,)  - 每个采样点对应的参数值
    """
    u_probe = np.linspace(0.0, 1.0, n_probe)
    pts = np.asarray(splev(u_probe, tck), dtype=float).T           # (n_probe,3)

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s_cum = np.concatenate([[0.0], np.cumsum(seg)])                # 累积弧长
    total = float(s_cum[-1])

    if total < 1e-6:
        return pts[:1], u_probe[:1]

    m = max(int(round(total / max(ds_target, 1e-6))) + 1, 2)
    s_want = np.linspace(0.0, total, m)

    # s_cum 单调递增 → 可直接线性插值反解参数 u
    u_at = np.interp(s_want, s_cum, u_probe)
    points = np.asarray(splev(u_at, tck), dtype=float).T

    return points, u_at


# ============================================================================
# B 样条拟合
# ============================================================================

def fit_bspline(path: List[np.ndarray], s_param: float, endpoint_weight: float = 1.0e5):
    """
    对 3D 路径拟合三次 B 样条，返回样条对象本身

    与旧版 smooth_trajectory_bspline 的区别: 返回 tck 而非点列。
    因为后续的曲率验证、速度剖面都需要解析导数，丢掉 tck 就只能
    退回三点差分，那正是发散的来源。

    参数:
        path: 原始路径 [(x,y,z), ...]（米）
        s_param: 残差平方和上界（m²），见模块头说明

    返回:
        tck: 样条表示
        u:   每个输入点对应的参数值
        P:   去重后的输入点数组 (n,3)

    异常:
        ValueError: 去重后点数 < 4（三次样条至少需要 k+1 = 4 个点）
        RuntimeError: splprep 拟合失败
    """
    P = np.asarray(path, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3:
        raise ValueError(f"路径应为 (n,3) 数组，当前 shape={P.shape}")

    # 去除连续重复点（splprep 遇到重复点会因参数化退化而失败）
    keep = [0]
    for i in range(1, len(P)):
        if np.linalg.norm(P[i] - P[keep[-1]]) > 1e-6:
            keep.append(i)
    P = P[keep]

    if len(P) < 4:
        raise ValueError(f"去重后路径点数不足 4 个（{len(P)} 个），无法做三次 B 样条")

    try:
        # splprep is retained.  Strong endpoint weights keep take-off and
        # terminal coordinates effectively fixed while still allowing all
        # interior points to participate in smoothing.
        weights = np.ones(len(P), dtype=float)
        weights[0] = weights[-1] = float(endpoint_weight)
        tck, u = splprep([P[:, 0], P[:, 1], P[:, 2]],
                         w=weights, s=float(s_param), k=3)
    except Exception as e:
        raise RuntimeError(f"B 样条拟合失败 (s={s_param:.3e}): {e}")

    return tck, u, P


def default_s_candidates(n_points: int,
                         grid_xy: float = 200.0) -> List[float]:
    """
    按量纲生成候选 s（升序），重点补足小 s 区间。

    scipy 的 s 满足：
        sum(|P_i - C(u_i)|^2) <= s

    因此 sqrt(s/n) 可理解为每个输入点允许的 RMS 拟合偏差尺度。
    旧版第一档是 grid/8；200 m 网格时就是 25 m。若细网格路径
    已经贴近 600 m 通路边界，这一档可能一开始就让样条外鼓出界。

    新版从 grid/50 开始，并在 4~40 m（对 200 m 网格）范围加密，
    优先找“刚好满足转弯/爬升，同时尽量贴近细网格原始路径”的最小平滑量。
    """
    n = max(int(n_points), 1)

    fractions = (
        0.02, 0.03, 0.04, 0.05,
        0.0625, 0.075, 0.09, 0.105,
        0.125, 0.15, 0.18, 0.22,
        0.25, 0.35, 0.50, 0.75,
        1.00, 1.50, 2.00, 3.00, 4.00,
    )

    values = [
        float(n) * (float(grid_xy) * f) ** 2
        for f in fractions
    ]
    return sorted(set(values))


# ============================================================================
# 3D 通路距离工具
# ============================================================================

def _distances_to_polyline(points: np.ndarray,
                           polyline: np.ndarray) -> np.ndarray:
    """
    批量计算 points 到 3D 折线 polyline 的最短欧氏距离。

    这是 validate_spline() 的基础几何函数。V2 打包时该函数被遗漏，
    导致运行期 NameError；V3 将其作为显式回归测试项。
    """
    Q = np.asarray(points, dtype=float)
    P = np.asarray(polyline, dtype=float)

    if Q.ndim != 2 or Q.shape[1] != 3:
        raise ValueError(f"points 应为 (n,3)，当前 shape={Q.shape}")
    if P.ndim != 2 or P.shape[1] != 3:
        raise ValueError(f"polyline 应为 (m,3)，当前 shape={P.shape}")
    if len(P) == 0:
        return np.full(len(Q), float("inf"), dtype=float)
    if len(P) == 1:
        return np.linalg.norm(Q - P[0], axis=1)

    min_d2 = np.full(len(Q), float("inf"), dtype=float)

    for i in range(len(P) - 1):
        a = P[i]
        b = P[i + 1]
        ab = b - a
        denom = float(np.dot(ab, ab))

        if denom < 1e-12:
            d2 = np.sum((Q - a) ** 2, axis=1)
        else:
            t = ((Q - a) @ ab) / denom
            t = np.clip(t, 0.0, 1.0)
            proj = a + t[:, None] * ab
            d2 = np.sum((Q - proj) ** 2, axis=1)

        min_d2 = np.minimum(min_d2, d2)

    return np.sqrt(np.maximum(min_d2, 0.0))


def _resample_polyline(path: np.ndarray,
                       ds_target: float = 50.0) -> Tuple[np.ndarray, np.ndarray]:
    """按弧长对普通折线重采样；用于通路诊断，不用于曲率计算。"""
    P = np.asarray(path, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3:
        raise ValueError(f"path 应为 (n,3)，当前 shape={P.shape}")
    if len(P) == 0:
        return np.empty((0, 3), dtype=float), np.empty(0, dtype=float)
    if len(P) == 1:
        return P.copy(), np.array([0.0])

    # 去除连续重复点，避免累计弧长出现重复横坐标
    keep = [0]
    for i in range(1, len(P)):
        if np.linalg.norm(P[i] - P[keep[-1]]) > 1e-9:
            keep.append(i)
    P = P[keep]

    if len(P) == 1:
        return P.copy(), np.array([0.0])

    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s_orig = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s_orig[-1])

    if total < 1e-9:
        return P[:1].copy(), np.array([0.0])

    ds_target = max(float(ds_target), 1e-6)
    s_regular = np.arange(0.0, total, ds_target)
    # 同时保留全部原始顶点，避免诊断漏掉顶点极值
    s_query = np.unique(np.concatenate([s_regular, s_orig, [total]]))

    Q = np.column_stack([
        np.interp(s_query, s_orig, P[:, j]) for j in range(3)
    ])
    return Q, s_query


def corridor_deviation_stats(path: np.ndarray,
                             corridor_reference: np.ndarray,
                             sample_spacing: float = 50.0) -> Dict:
    """
    诊断普通路径相对 3D 通路中心线的距离。

    返回 max/mean deviation、最大偏离位置和沿路径弧长。
    该函数主要用于在 B 样条之前回答：
        “600 m 通路本身还有多少余量？”
    """
    Q, s_query = _resample_polyline(path, sample_spacing)
    ref = np.asarray(corridor_reference, dtype=float)

    if len(Q) == 0:
        return {
            "max_deviation": 0.0,
            "mean_deviation": 0.0,
            "s_at_max": 0.0,
            "point_at_max": np.zeros(3, dtype=float),
        }

    dev = _distances_to_polyline(Q, ref)
    idx = int(np.argmax(dev))

    return {
        "max_deviation": float(dev[idx]),
        "mean_deviation": float(np.mean(dev)),
        "s_at_max": float(s_query[idx]),
        "point_at_max": Q[idx].copy(),
    }



# ============================================================================
# 垂向动力学修复：保持 x/y 平滑几何，限制 dz/ds_h
# ============================================================================

def repair_spline_climb_angle(
    tck,
    raw_points: np.ndarray,
    dynamics: FixedWingDynamics,
    altitude_floor: Optional[Callable[[np.ndarray], float]] = None,
    slope_fraction: float = 0.88,
    floor_buffer_m: float = 5.0,
    n_samples: Optional[int] = None,
):
    """Repair vertical B-spline shape while keeping the horizontal route.

    The raw A* grid edges already satisfy the fixed-wing climb-angle limit, but
    a cubic spline can overshoot between coarse 2 km / 500 m vertices.  This
    routine samples the current smooth x/y route, projects only its z-profile
    onto a Lipschitz envelope,

        |dz/ds_h| <= slope_fraction * tan(gamma_max),

    preserves the exact mission endpoints, optionally respects the mission
    altitude floor, and refits the corrected dense curve with ``splprep(s=0)``.

    The hard 15-degree limit is *not* relaxed.  ``slope_fraction < 1`` leaves
    numerical headroom for cubic interpolation between the projected samples.
    Every repaired spline is revalidated by the caller against all original
    hard constraints.
    """
    P = np.asarray(raw_points, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3 or len(P) < 4:
        raise ValueError("raw_points must be an (n,3) array with n>=4")

    if n_samples is None:
        n_samples = int(max(512, min(2048, 6 * max(len(P), 2))))
    n_samples = max(int(n_samples), 128)

    frac = float(slope_fraction)
    if not (0.0 < frac < 1.0):
        raise ValueError("slope_fraction must be between 0 and 1")

    u = np.linspace(0.0, 1.0, n_samples)
    Q = np.asarray(splev(u, tck), dtype=float).T

    dxy = np.linalg.norm(np.diff(Q[:, :2], axis=0), axis=1)
    s_h = np.concatenate([[0.0], np.cumsum(dxy)])
    total_h = float(s_h[-1])
    if total_h < 1e-6:
        raise RuntimeError("水平路径长度过小，无法执行爬升角修复")

    slope_limit = float(np.tan(np.radians(dynamics.max_climb_angle_deg)) * frac)

    # Lower bound: mission altitude floor + a tiny interior buffer.  The buffer
    # scales with the floor itself so it vanishes at 0 m mission endpoints.
    if altitude_floor is not None:
        floors = np.asarray([float(altitude_floor(q)) for q in Q], dtype=float)
        floor_peak = float(np.max(floors)) if len(floors) else 0.0
        if floor_peak > 1e-9:
            taper = np.clip(floors / floor_peak, 0.0, 1.0)
        else:
            taper = np.zeros_like(floors)
        lower = floors + float(max(floor_buffer_m, 0.0)) * taper
    else:
        lower = np.full(n_samples, -np.inf, dtype=float)

    z0 = float(P[0, 2])
    z1 = float(P[-1, 2])

    # Endpoint reachability cones.  A feasible z profile must be reachable from
    # both the start and the goal under the selected conservative slope.
    upper_from_endpoints = np.minimum(
        z0 + slope_limit * s_h,
        z1 + slope_limit * (total_h - s_h),
    )
    lower_from_endpoints = np.maximum(
        z0 - slope_limit * s_h,
        z1 - slope_limit * (total_h - s_h),
    )
    lower = np.maximum(lower, lower_from_endpoints)

    # Exact endpoints are mission invariants.
    lower[0] = z0
    lower[-1] = z1
    upper_from_endpoints[0] = z0
    upper_from_endpoints[-1] = z1

    if np.any(lower > upper_from_endpoints + 1e-6):
        raise RuntimeError(
            "当前保守爬升角目标与高度包络不兼容；无法在保持端点的同时修复"
        )

    z = np.clip(Q[:, 2], lower, upper_from_endpoints)
    z[0] = z0
    z[-1] = z1

    # Alternating forward/backward projection of adjacent slope constraints.
    # This is analogous to the forward/backward speed-profile limiter.
    for _ in range(30):
        previous = z.copy()

        for i in range(1, n_samples - 1):
            ds = max(float(dxy[i - 1]), 1e-9)
            lo = max(float(lower[i]), float(z[i - 1] - slope_limit * ds))
            hi = min(float(upper_from_endpoints[i]),
                     float(z[i - 1] + slope_limit * ds))
            if lo > hi + 1e-6:
                raise RuntimeError("爬升角前向投影出现不可行区间")
            z[i] = float(np.clip(z[i], lo, hi))

        for i in range(n_samples - 2, 0, -1):
            ds = max(float(dxy[i]), 1e-9)
            lo = max(float(lower[i]), float(z[i + 1] - slope_limit * ds))
            hi = min(float(upper_from_endpoints[i]),
                     float(z[i + 1] + slope_limit * ds))
            if lo > hi + 1e-6:
                raise RuntimeError("爬升角后向投影出现不可行区间")
            z[i] = float(np.clip(z[i], lo, hi))

        z[0] = z0
        z[-1] = z1
        if float(np.max(np.abs(z - previous))) < 1e-7:
            break

    # Discrete sanity check before cubic refit.
    dz = np.abs(np.diff(z))
    allowed = slope_limit * np.maximum(dxy, 1e-9)
    if np.any(dz > allowed + 1e-4):
        raise RuntimeError("爬升角投影未收敛到目标斜率")

    Q[:, 2] = z
    Q[0] = P[0]
    Q[-1] = P[-1]

    try:
        repaired_tck, repaired_u = splprep(
            [Q[:, 0], Q[:, 1], Q[:, 2]], s=0.0, k=3
        )
    except Exception as exc:
        raise RuntimeError(f"爬升角修复后的 B 样条重拟合失败: {exc}")

    return repaired_tck, repaired_u, Q


# ============================================================================
# 高度包络约束修复（仍保持 cubic B-spline / splprep）
# ============================================================================

def repair_spline_to_altitude_floor(
    tck,
    raw_points: np.ndarray,
    altitude_floor: Callable[[np.ndarray], float],
    buffer_m: float = 10.0,
    n_samples: Optional[int] = None,
):
    """Lift only the violating vertical component and refit a cubic B-spline.

    A large smoothing ``s`` can pull a long 1000 m plateau below the hard
    altitude floor even when the raw A* path is feasible.  Rejecting every such
    candidate makes the smoothing stage unnecessarily brittle.  This helper
    preserves the original smooth x/y geometry, projects only z upward to the
    mission floor, and then refits the corrected dense curve with ``splprep``
    at ``s=0``.  The returned object is therefore still a C2 cubic B-spline.

    The small interior buffer protects against interpolation undershoot between
    projection samples; it tapers to zero at the exact mission endpoints, so
    start/goal altitude is unchanged.
    """
    P = np.asarray(raw_points, dtype=float)
    if n_samples is None:
        # Dense projection is important near the departure/terminal ramps.
        n_samples = int(max(512, min(2048, 6 * max(len(P), 2))))
    n_samples = max(int(n_samples), 64)

    u = np.linspace(0.0, 1.0, n_samples)
    Q = np.asarray(splev(u, tck), dtype=float).T
    floors = np.asarray([float(altitude_floor(q)) for q in Q], dtype=float)

    # V5.2: use a floor-proportional *linear* taper rather than sin^2(u).
    # sin^2 has zero slope at the endpoints and can therefore let a cubic
    # interpolant sag below the linearly rising take-off/landing envelope.
    floor_peak = float(np.max(floors)) if len(floors) else 0.0
    if floor_peak > 1e-9:
        taper = np.clip(floors / floor_peak, 0.0, 1.0)
    else:
        taper = np.minimum(1.0, np.minimum(u, 1.0 - u) / 0.05)

    target_z = floors + float(max(buffer_m, 0.0)) * taper
    Q[:, 2] = np.maximum(Q[:, 2], target_z)

    # Preserve mission endpoints exactly.
    Q[0] = P[0]
    Q[-1] = P[-1]

    try:
        repaired_tck, repaired_u = splprep(
            [Q[:, 0], Q[:, 1], Q[:, 2]], s=0.0, k=3
        )
    except Exception as exc:
        raise RuntimeError(f"高度包络修复后的 B 样条重拟合失败: {exc}")

    return repaired_tck, repaired_u, Q


def spline_altitude_floor_margin(
    tck,
    altitude_floor: Callable[[np.ndarray], float],
    n_probe: int = 8192,
) -> float:
    """Minimum z - h_min(x,y) along a spline; negative means violation."""
    u = np.linspace(0.0, 1.0, max(int(n_probe), 256))
    Q = np.asarray(splev(u, tck), dtype=float).T
    margins = np.asarray(
        [float(q[2]) - float(altitude_floor(q)) for q in Q],
        dtype=float,
    )
    return float(np.min(margins)) if len(margins) else float('inf')


# ============================================================================
# 运动学 + 走廊约束验证
# ============================================================================

def validate_spline(tck, raw_points: np.ndarray, u_raw: np.ndarray,
                    dynamics: FixedWingDynamics,
                    corridor_radius: Optional[float] = None,
                    corridor_reference: Optional[np.ndarray] = None,
                    collision_checker: Optional[Callable[[np.ndarray], bool]] = None,
                    altitude_checker: Optional[Callable[[np.ndarray], bool]] = None,
                    max_smoothing_deviation: Optional[float] = None,
                    enforce_search_corridor: bool = False,
                    terminal_heading: Optional[np.ndarray] = None,
                    terminal_heading_tolerance_deg: float = 12.0,
                    endpoint_tolerance_m: float = 2.0,
                    ) -> Dict:
    """Validate a candidate final B-spline against physical hard constraints.

    Search-corridor membership remains diagnostic by default.  In addition to
    V4 constraints, V5 adds fixed endpoints, a terminal-heading constraint and
    explicitly reports curvature-continuity energy.
    """
    min_feasible_radius = float(dynamics.get_min_feasible_radius())
    raw_ref = np.asarray(raw_points, dtype=float)

    u_probe = np.linspace(0.0, 1.0, 4096)
    curve_probe = np.asarray(splev(u_probe, tck), dtype=float).T

    # 1) Curvature / turn radius.
    k = spline_curvature(tck, u_probe)
    k_max = float(np.max(k)) if len(k) else 0.0
    min_turn_radius = float('inf') if k_max < 1e-15 else 1.0 / k_max
    turn_radius_ok = bool(min_turn_radius >= min_feasible_radius - 1e-9)

    # Explicit steering smoothness term J_{kappa'}.
    ds_probe = np.linalg.norm(np.diff(curve_probe, axis=0), axis=1)
    s_arc = np.concatenate([[0.0], np.cumsum(ds_probe)])
    if len(s_arc) >= 3 and s_arc[-1] > 1e-9:
        dk_ds = np.gradient(k, s_arc, edge_order=1)
        curvature_variation = float(np.trapz(dk_ds ** 2, s_arc))
    else:
        curvature_variation = 0.0

    # 2) Analytic tangent -> climb angle.
    d1 = np.asarray(splev(u_probe, tck, der=1), dtype=float).T
    horiz = np.linalg.norm(d1[:, :2], axis=1)
    valid = horiz > 1e-9
    if np.any(valid):
        angles = np.degrees(np.arctan2(np.abs(d1[valid, 2]), horiz[valid]))
        max_climb_angle = float(np.max(angles))
    else:
        max_climb_angle = 0.0
    climb_angle_ok = bool(max_climb_angle <= dynamics.max_climb_angle_deg + 1e-9)

    # 3) Endpoints: smoothing must not move the mission start/goal.
    start_error = float(np.linalg.norm(curve_probe[0] - raw_ref[0]))
    end_error = float(np.linalg.norm(curve_probe[-1] - raw_ref[-1]))
    endpoints_ok = bool(
        start_error <= float(endpoint_tolerance_m) + 1e-9
        and end_error <= float(endpoint_tolerance_m) + 1e-9
    )

    # 4) Terminal horizontal heading.  This is what removes the C5-area
    # back-and-forth tail without imposing an arbitrary waypoint.
    terminal_heading_error = 0.0
    terminal_heading_ok = True
    if terminal_heading is not None:
        desired = np.asarray(terminal_heading, dtype=float).ravel()[:2]
        tangent = d1[-1, :2]
        nd = float(np.linalg.norm(desired))
        nt = float(np.linalg.norm(tangent))
        if nd < 1e-12 or nt < 1e-12:
            terminal_heading_error = 180.0
            terminal_heading_ok = False
        else:
            c = float(np.clip(np.dot(desired / nd, tangent / nt), -1.0, 1.0))
            terminal_heading_error = float(np.degrees(np.arccos(c)))
            terminal_heading_ok = bool(
                terminal_heading_error <= float(terminal_heading_tolerance_deg) + 1e-9
            )

    # 5) Coarse search corridor: diagnostic unless explicitly enforced.
    max_corridor_deviation = 0.0
    mean_corridor_deviation = 0.0
    search_corridor_ok = True
    if corridor_reference is not None:
        reference = np.asarray(corridor_reference, dtype=float)
        dev_corr = _distances_to_polyline(curve_probe, reference)
        max_corridor_deviation = float(np.max(dev_corr)) if len(dev_corr) else 0.0
        mean_corridor_deviation = float(np.mean(dev_corr)) if len(dev_corr) else 0.0
        if corridor_radius is not None:
            search_corridor_ok = bool(
                max_corridor_deviation <= float(corridor_radius) + 1e-9)

    # 6) Trust-region locality relative to the fine A* path.
    dev_raw = _distances_to_polyline(curve_probe, raw_ref)
    max_raw_deviation = float(np.max(dev_raw)) if len(dev_raw) else 0.0
    mean_raw_deviation = float(np.mean(dev_raw)) if len(dev_raw) else 0.0
    rms_raw_deviation = float(np.sqrt(np.mean(dev_raw ** 2))) if len(dev_raw) else 0.0
    smoothing_locality_ok = (
        True if max_smoothing_deviation is None
        else bool(max_raw_deviation <= float(max_smoothing_deviation) + 1e-9)
    )

    # 7) Mission altitude envelope.
    altitude_ok = True
    first_altitude_violation_index = None
    if altitude_checker is not None:
        for idx, pt in enumerate(curve_probe):
            if not bool(altitude_checker(pt)):
                altitude_ok = False
                first_altitude_violation_index = int(idx)
                break

    # 8) Hard no-fly collision.
    collision_free = True
    first_collision_index = None
    if collision_checker is not None:
        for idx, pt in enumerate(curve_probe):
            if bool(collision_checker(pt)):
                collision_free = False
                first_collision_index = int(idx)
                break

    corridor_hard_ok = search_corridor_ok if enforce_search_corridor else True
    feasible = bool(
        turn_radius_ok
        and climb_angle_ok
        and endpoints_ok
        and terminal_heading_ok
        and smoothing_locality_ok
        and altitude_ok
        and collision_free
        and corridor_hard_ok
    )

    return {
        'min_turn_radius': min_turn_radius,
        'max_curvature': k_max,
        'min_feasible_radius': min_feasible_radius,
        'turn_radius_ok': turn_radius_ok,
        'curvature_variation_integral': curvature_variation,
        'max_climb_angle': max_climb_angle,
        'climb_angle_ok': climb_angle_ok,
        'start_error': start_error,
        'end_error': end_error,
        'endpoint_tolerance_m': float(endpoint_tolerance_m),
        'endpoints_ok': endpoints_ok,
        'terminal_heading_error_deg': terminal_heading_error,
        'terminal_heading_tolerance_deg': float(terminal_heading_tolerance_deg),
        'terminal_heading_ok': bool(terminal_heading_ok),
        'max_corridor_deviation': max_corridor_deviation,
        'mean_corridor_deviation': mean_corridor_deviation,
        'search_corridor_ok': bool(search_corridor_ok),
        'search_corridor_enforced': bool(enforce_search_corridor),
        # Backward-compatible display keys.
        'max_deviation': max_corridor_deviation,
        'mean_deviation': mean_corridor_deviation,
        'corridor_ok': bool(search_corridor_ok),
        'max_raw_deviation': max_raw_deviation,
        'mean_raw_deviation': mean_raw_deviation,
        'rms_raw_deviation': rms_raw_deviation,
        'max_smoothing_deviation': max_smoothing_deviation,
        'smoothing_locality_ok': bool(smoothing_locality_ok),
        'altitude_ok': bool(altitude_ok),
        'first_altitude_violation_index': first_altitude_violation_index,
        'collision_free': bool(collision_free),
        'first_collision_index': first_collision_index,
        'feasible': feasible,
    }


def violation_score(val: Dict, dynamics: FixedWingDynamics,
                    corridor_radius: Optional[float],
                    max_smoothing_deviation: Optional[float] = None,
                    enforce_search_corridor: bool = False) -> float:
    """
    归一化违反度，仅用于无可行候选时选择最接近可行的诊断候选。

    V4 默认不把 coarse 搜索走廊超界计入违反度，因为它不是物理硬约束。
    """
    score = 0.0

    if val["min_turn_radius"] < val["min_feasible_radius"]:
        score += (
            val["min_feasible_radius"]
            / max(val["min_turn_radius"], 1e-6)
            - 1.0
        )

    if val["max_climb_angle"] > dynamics.max_climb_angle_deg:
        score += (
            val["max_climb_angle"]
            / max(dynamics.max_climb_angle_deg, 1e-6)
            - 1.0
        )

    if (
        max_smoothing_deviation is not None
        and val["max_raw_deviation"] > max_smoothing_deviation
    ):
        score += (
            val["max_raw_deviation"]
            / max(float(max_smoothing_deviation), 1e-6)
            - 1.0
        )

    if enforce_search_corridor and corridor_radius is not None:
        if val["max_corridor_deviation"] > corridor_radius:
            score += (
                val["max_corridor_deviation"]
                / max(float(corridor_radius), 1e-6)
                - 1.0
            )

    if not val.get("endpoints_ok", True):
        tol = max(float(val.get("endpoint_tolerance_m", 1.0)), 1e-6)
        score += max(float(val.get("start_error", 0.0)),
                     float(val.get("end_error", 0.0))) / tol

    if not val.get("terminal_heading_ok", True):
        tol = max(float(val.get("terminal_heading_tolerance_deg", 1.0)), 1e-6)
        score += float(val.get("terminal_heading_error_deg", 180.0)) / tol

    if not val.get("altitude_ok", True):
        score += 100.0

    if not val.get("collision_free", True):
        score += 100.0

    return float(score)


def validate_polyline(path: List[np.ndarray],
                      dynamics: FixedWingDynamics,
                      altitude_checker: Optional[Callable[[np.ndarray], bool]] = None) -> Dict:
    """
    折线路径的约束检查（仅在无法光滑、回退原路径时使用）

    折线在拐点处曲率数学上不存在（0 → ∞ → 0），这里用三点公式给出
    的只是一个网格尺度下的参考值，不代表可飞性 —— 折线本来就不可飞。
    """
    P = np.asarray(path, dtype=float)

    max_climb_angle = 0.0
    for i in range(len(P) - 1):
        d = P[i + 1] - P[i]
        horiz = float(np.hypot(d[0], d[1]))
        if horiz > 1e-6:
            max_climb_angle = max(
                max_climb_angle,
                float(np.degrees(np.arctan2(abs(d[2]), horiz))))

    min_turn_radius = float('inf')
    for i in range(1, len(P) - 1):
        R = dynamics.get_path_curvature_radius(P[i - 1], P[i], P[i + 1])
        if 1e-6 < R < float('inf'):
            min_turn_radius = min(min_turn_radius, R)

    altitude_ok = True
    first_altitude_violation_index = None
    if altitude_checker is not None:
        for idx, pt in enumerate(P):
            if not bool(altitude_checker(pt)):
                altitude_ok = False
                first_altitude_violation_index = int(idx)
                break

    min_feasible_radius = dynamics.get_min_feasible_radius()
    return {
        'min_turn_radius': min_turn_radius,
        'min_feasible_radius': min_feasible_radius,
        'turn_radius_ok': bool(min_turn_radius >= min_feasible_radius),
        'max_climb_angle': max_climb_angle,
        'climb_angle_ok': bool(max_climb_angle <= dynamics.max_climb_angle_deg),
        'max_deviation': 0.0,
        'mean_deviation': 0.0,
        'corridor_ok': True,
        'altitude_ok': bool(altitude_ok),
        'first_altitude_violation_index': first_altitude_violation_index,
        'feasible': bool(min_turn_radius >= min_feasible_radius
                         and max_climb_angle <= dynamics.max_climb_angle_deg
                         and altitude_ok),
    }


# ============================================================================
# 带验证的光滑（主接口）
# ============================================================================

def smooth_with_validation(path: List[np.ndarray],
                           dynamics: FixedWingDynamics,
                           s_param: Optional[float] = None,
                           grid_xy: float = 200.0,
                           corridor_radius: Optional[float] = 600.0,
                           corridor_reference: Optional[List[np.ndarray]] = None,
                           collision_checker: Optional[Callable[[np.ndarray], bool]] = None,
                           altitude_checker: Optional[Callable[[np.ndarray], bool]] = None,
                           altitude_floor: Optional[Callable[[np.ndarray], float]] = None,
                           ds_target: Optional[float] = None,
                           max_smoothing_deviation: Optional[float] = None,
                           enforce_search_corridor: bool = False,
                           terminal_heading: Optional[np.ndarray] = None,
                           terminal_heading_tolerance_deg: float = 12.0,
                           endpoint_tolerance_m: float = 2.0,
                           ) -> Tuple[List[np.ndarray], Dict]:
    """Globally select the best feasible cubic B-spline candidate.

    V5 deliberately evaluates *all* candidate ``s`` values.  The old
    "first feasible then break" rule was only a feasibility search; it could
    return a flyable but visibly poor curve.  Feasible candidates are now
    ranked by

        J_s = 0.4 J_smooth + 0.3 J_curvature
            + 0.2 J_deviation + 0.1 J_length.

    ``J_curvature`` contains the requested curvature-continuity energy
    ``integral (d kappa / ds)^2 ds``.  Endpoint, terminal-heading, turn-radius,
    climb-angle, locality and no-fly constraints remain hard constraints.
    """
    P0 = np.asarray(path, dtype=float)

    if len(P0) < 4:
        return list(path), {
            'tck': None, 'u_raw': None, 'u_smooth': None,
            'validation': validate_polyline(path, dynamics, altitude_checker=altitude_checker),
            's_param_used': 0.0, 'attempts': 0, 'candidate_history': [],
            'success': False, 'smoothed': False,
            'selection_score': float('inf'),
            'message': f'路径点数不足（{len(P0)} < 4），跳过光滑',
            'smoothing_version': SMOOTHING_VERSION,
        }

    if max_smoothing_deviation is None:
        max_smoothing_deviation = float(grid_xy)

    if ds_target is None:
        seg = np.linalg.norm(np.diff(P0, axis=0), axis=1)
        ds_target = float(np.mean(seg)) if len(seg) else float(grid_xy)

    if terminal_heading is None:
        terminal_heading = infer_terminal_heading(
            P0, lookback_distance=max(5.0 * float(grid_xy), 1000.0))
    terminal_heading = np.asarray(terminal_heading, dtype=float).ravel()[:2]

    candidates = ([float(s_param)] if s_param is not None
                  else default_s_candidates(len(P0), grid_xy))

    feasible_candidates = []
    fallback_candidates = []
    candidate_history = []
    last_error = None

    for candidate_index, s_val in enumerate(candidates, 1):
        try:
            tck, u_raw, P = fit_bspline(path, s_val)
        except (ValueError, RuntimeError) as e:
            last_error = str(e)
            candidate_history.append({
                'candidate_index': candidate_index, 's': float(s_val),
                'stage': 'fit', 'error': str(e), 'feasible': False,
            })
            continue

        try:
            def _validate(candidate_tck):
                return validate_spline(
                    candidate_tck, P, u_raw, dynamics,
                    corridor_radius=corridor_radius,
                    corridor_reference=corridor_reference,
                    collision_checker=collision_checker,
                    altitude_checker=altitude_checker,
                    max_smoothing_deviation=max_smoothing_deviation,
                    enforce_search_corridor=enforce_search_corridor,
                    terminal_heading=terminal_heading,
                    terminal_heading_tolerance_deg=terminal_heading_tolerance_deg,
                    endpoint_tolerance_m=endpoint_tolerance_m,
                )

            val = _validate(tck)
            altitude_repaired = False
            repair_buffer_m = 0.0
            climb_repaired = False
            climb_repair_fraction = 1.0

            # If the only geometric representation issue is that global
            # smoothing pulled z below the mission floor, repair the candidate
            # itself rather than relaxing the hard constraint.  Several small
            # buffers are tried; each repaired curve is revalidated against
            # every original hard constraint.
            if (
                altitude_floor is not None
                and altitude_checker is not None
                and not val.get('altitude_ok', True)
            ):
                repair_variants = []

                # V5.2 adaptive repair.  Start with a very small buffer and
                # increase it according to the *measured* post-refit floor
                # deficit.  This preserves the hard altitude constraint while
                # avoiding an arbitrary large lift of the whole trajectory.
                if max_smoothing_deviation is None:
                    _max_buffer = 200.0
                else:
                    _max_buffer = min(200.0, 0.90 * float(max_smoothing_deviation))
                _max_buffer = max(_max_buffer, 10.0)

                _buffer = 5.0
                _seen_buffers = set()
                for _repair_iter in range(12):
                    _buffer = float(min(_buffer, _max_buffer))
                    _key = round(_buffer, 6)
                    if _key in _seen_buffers:
                        break
                    _seen_buffers.add(_key)

                    try:
                        rtck, _, _ = repair_spline_to_altitude_floor(
                            tck, P, altitude_floor, buffer_m=_buffer)
                        _floor_margin = spline_altitude_floor_margin(
                            rtck, altitude_floor, n_probe=8192)
                        rval = _validate(rtck)
                        rquality = smoothing_objective(
                            rtck, P, dynamics, max_smoothing_deviation)
                        rviol = violation_score(
                            rval, dynamics, corridor_radius,
                            max_smoothing_deviation=max_smoothing_deviation,
                            enforce_search_corridor=enforce_search_corridor,
                        )
                        repair_variants.append(
                            (not bool(rval['feasible']), float(rviol),
                             float(rquality['J_s']), float(_buffer),
                             rtck, rval, rquality, float(_floor_margin))
                        )

                        # Once this candidate is fully feasible, a larger
                        # vertical lift cannot improve hard-constraint status.
                        if bool(rval['feasible']):
                            break

                        # If the repaired cubic still dips below the floor,
                        # increase by at least the measured deficit plus a
                        # numerical safety margin.  Otherwise grow moderately
                        # to see whether another hard constraint can be met.
                        if _floor_margin < -0.25:
                            _buffer = max(
                                _buffer * 1.45,
                                _buffer + (-_floor_margin) + 5.0,
                            )
                        else:
                            _buffer *= 1.45

                        if _buffer > _max_buffer + 1e-9:
                            if _key < round(_max_buffer, 6):
                                _buffer = _max_buffer
                            else:
                                break

                    except (ValueError, FloatingPointError, RuntimeError):
                        _buffer *= 1.6
                        if _buffer > _max_buffer + 1e-9:
                            break

                if repair_variants:
                    repair_variants.sort(key=lambda r: (r[0], r[1], r[2], r[3]))
                    _, _, _, repair_buffer_m, tck, val, quality, _floor_margin = repair_variants[0]
                    altitude_repaired = True
                    val['altitude_floor_margin_m'] = float(_floor_margin)
                else:
                    quality = smoothing_objective(
                        tck, P, dynamics, max_smoothing_deviation)
            else:
                quality = smoothing_objective(
                    tck, P, dynamics, max_smoothing_deviation)

            # V5.3: cubic interpolation of coarse 2 km / 500 m A* vertices can
            # overshoot the 15-degree climb limit even though every raw edge is
            # itself feasible (~14.04 deg for an axial 2000/500 step).
            # Repair only the vertical profile; x/y route geometry is retained.
            if not val.get('climb_angle_ok', True):
                _base_violation = violation_score(
                    val, dynamics, corridor_radius,
                    max_smoothing_deviation=max_smoothing_deviation,
                    enforce_search_corridor=enforce_search_corridor,
                )
                climb_variants = [
                    (not bool(val['feasible']), float(_base_violation),
                     float(quality['J_s']), -1.0,
                     tck, val, quality, 1.0, False)
                ]

                # Largest fraction first = smallest modification.  The last
                # value (0.76) still leaves enough slope for the configured
                # 1000 m / 5 km altitude transition (slope 0.20).
                for _fraction in (0.92, 0.88, 0.84, 0.80, 0.76):
                    try:
                        ctck, _, _ = repair_spline_climb_angle(
                            tck, P, dynamics,
                            altitude_floor=altitude_floor,
                            slope_fraction=_fraction,
                            floor_buffer_m=5.0,
                        )
                        cval = _validate(ctck)
                        cquality = smoothing_objective(
                            ctck, P, dynamics, max_smoothing_deviation)
                        cviol = violation_score(
                            cval, dynamics, corridor_radius,
                            max_smoothing_deviation=max_smoothing_deviation,
                            enforce_search_corridor=enforce_search_corridor,
                        )
                        climb_variants.append(
                            (not bool(cval['feasible']), float(cviol),
                             float(cquality['J_s']), -float(_fraction),
                             ctck, cval, cquality, float(_fraction), True)
                        )
                        # First fully feasible variant is the least aggressive
                        # slope repair, so no smaller fraction is necessary.
                        if bool(cval['feasible']):
                            break
                    except (ValueError, FloatingPointError, RuntimeError):
                        continue

                climb_variants.sort(key=lambda r: (r[0], r[1], r[2], r[3]))
                _, _, _, _, tck, val, quality, _chosen_fraction, _was_repaired = climb_variants[0]
                climb_repaired = bool(_was_repaired)
                climb_repair_fraction = float(_chosen_fraction)

        except (ValueError, FloatingPointError) as e:
            last_error = str(e)
            candidate_history.append({
                'candidate_index': candidate_index, 's': float(s_val),
                'stage': 'validate', 'error': str(e), 'feasible': False,
            })
            continue

        violation = violation_score(
            val, dynamics, corridor_radius,
            max_smoothing_deviation=max_smoothing_deviation,
            enforce_search_corridor=enforce_search_corridor,
        )

        item = {
            'candidate_index': candidate_index,
            's': float(s_val),
            'min_turn_radius': float(val['min_turn_radius']),
            'max_curvature': float(val['max_curvature']),
            'max_climb_angle': float(val['max_climb_angle']),
            'curvature_variation_integral': float(val['curvature_variation_integral']),
            'terminal_heading_error_deg': float(val['terminal_heading_error_deg']),
            'terminal_heading_ok': bool(val['terminal_heading_ok']),
            'start_error': float(val['start_error']),
            'end_error': float(val['end_error']),
            'endpoints_ok': bool(val['endpoints_ok']),
            'max_corridor_deviation': float(val['max_corridor_deviation']),
            'search_corridor_ok': bool(val['search_corridor_ok']),
            'max_raw_deviation': float(val['max_raw_deviation']),
            'rms_raw_deviation': float(val['rms_raw_deviation']),
            'smoothing_locality_ok': bool(val['smoothing_locality_ok']),
            'altitude_ok': bool(val.get('altitude_ok', True)),
            'first_altitude_violation_index': val.get('first_altitude_violation_index'),
            'altitude_repaired': bool(altitude_repaired),
            'altitude_repair_buffer_m': float(repair_buffer_m),
            'climb_repaired': bool(climb_repaired),
            'climb_repair_fraction': float(climb_repair_fraction),
            'turn_radius_ok': bool(val['turn_radius_ok']),
            'climb_angle_ok': bool(val['climb_angle_ok']),
            'collision_free': bool(val.get('collision_free', True)),
            'violation_score': float(violation),
            'feasible': bool(val['feasible']),
            **quality,
        }
        candidate_history.append(item)

        rec = (float(quality['J_s']), candidate_index, float(s_val),
               tck, u_raw, val, quality, float(violation))
        if val['feasible']:
            feasible_candidates.append(rec)
        else:
            fallback_candidates.append(rec)

    if not feasible_candidates and not fallback_candidates:
        return list(path), {
            'tck': None, 'u_raw': None, 'u_smooth': None,
            'validation': validate_polyline(path, dynamics, altitude_checker=altitude_checker),
            's_param_used': 0.0, 'attempts': len(candidates),
            'candidate_history': candidate_history,
            'success': False, 'smoothed': False,
            'selection_score': float('inf'),
            'message': ('B样条候选均发生数值/拟合失败'
                        f'（最后错误: {last_error}）'),
            'smoothing_version': SMOOTHING_VERSION,
        }

    if feasible_candidates:
        # Global score over every feasible candidate; no early break.
        chosen = min(feasible_candidates, key=lambda r: (r[0], r[1]))
        feasible = True
    else:
        # Diagnostic-only fallback: first minimize hard-constraint violation,
        # then the same smoothing score.
        chosen = min(fallback_candidates, key=lambda r: (r[7], r[0], r[1]))
        feasible = False

    score, selected_index, s_used, tck, u_raw, val, quality, _ = chosen
    selected_item = next(
        (item for item in candidate_history
         if item.get('candidate_index') == selected_index),
        {}
    )
    smooth_pts, u_smooth = resample_by_arclength(tck, ds_target)
    smooth_path = [np.asarray(pt, dtype=float) for pt in smooth_pts]

    if feasible:
        corridor_note = ''
        if (corridor_radius is not None and not val['search_corridor_ok']
                and not enforce_search_corridor):
            excess = val['max_corridor_deviation'] - float(corridor_radius)
            corridor_note = (
                f'；搜索走廊诊断超出 {excess:.1f} m（非最终物理硬约束）')
        altitude_note = ''
        if selected_item.get('altitude_repaired', False):
            altitude_note = (
                f'；高度包络B样条修复 '
                f'{selected_item.get("altitude_repair_buffer_m", 0.0):.0f} m'
            )
        climb_note = ''
        if selected_item.get('climb_repaired', False):
            climb_note = (
                f'；爬升角B样条修复 '
                f'{selected_item.get("climb_repair_fraction", 1.0):.2f}×tan(γmax)'
            )
        msg = (
            f'光滑成功（V5.3，全局评价 {len(candidates)} 个 s 候选；'
            f'选中第 {selected_index} 个，s={s_used:.3e} m²，'
            f'J_s={score:.5f}，R_min={val["min_turn_radius"]:.0f} m，'
            f'最大爬升角={val["max_climb_angle"]:.2f}°，'
            f'末端航向误差={val["terminal_heading_error_deg"]:.2f}°，'
            f'J_kappa_prime={val["curvature_variation_integral"]:.3e}'
            f'{altitude_note}{climb_note}{corridor_note}）')
    else:
        reasons = []
        if not val['turn_radius_ok']:
            reasons.append(
                f'R_min={val["min_turn_radius"]:.0f} < '
                f'{val["min_feasible_radius"]:.0f} m')
        if not val['climb_angle_ok']:
            reasons.append(
                f'爬升角={val["max_climb_angle"]:.2f}° > '
                f'{dynamics.max_climb_angle_deg:.2f}°')
        if not val['endpoints_ok']:
            reasons.append('端点偏移超限')
        if not val['terminal_heading_ok']:
            reasons.append(
                f'末端航向误差={val["terminal_heading_error_deg"]:.2f}°')
        if not val['smoothing_locality_ok']:
            reasons.append(
                f'细路径偏移={val["max_raw_deviation"]:.1f} m > '
                f'{float(max_smoothing_deviation):.1f} m')
        if not val.get('altitude_ok', True):
            reasons.append('违反任务高度包络')
        if not val.get('collision_free', True):
            reasons.append('进入硬禁飞/火力区')
        if enforce_search_corridor and not val['search_corridor_ok']:
            reasons.append('超出强制搜索走廊')
        msg = ('没有满足全部最终硬约束的B样条；仅返回最佳诊断候选（'
               + '；'.join(reasons) + '）')

    return smooth_path, {
        'tck': tck,
        'u_raw': u_raw,
        'u_smooth': u_smooth,
        'validation': val,
        's_param_used': float(s_used),
        'attempts': len(candidates),
        'selected_candidate_index': int(selected_index),
        'candidate_history': candidate_history,
        'selection_score': float(score),
        'selection_components': quality,
        'altitude_repaired': bool(selected_item.get('altitude_repaired', False)),
        'altitude_repair_buffer_m': float(selected_item.get('altitude_repair_buffer_m', 0.0)),
        'climb_repaired': bool(selected_item.get('climb_repaired', False)),
        'climb_repair_fraction': float(selected_item.get('climb_repair_fraction', 1.0)),
        'terminal_heading_target_xy': terminal_heading.copy(),
        'success': bool(feasible),
        'smoothed': True,
        'message': msg,
        'smoothing_version': SMOOTHING_VERSION,
    }


# ============================================================================
# 光滑效果统计（用于报告与对比）
# ============================================================================

def compare_paths(raw_path: List[np.ndarray],
                  smooth_path: List[np.ndarray],
                  dynamics: FixedWingDynamics,
                  tck=None) -> Dict:
    """
    对比光滑前后的路径几何特征

    注意两侧用的曲率算法不同，这是有意的:
        - 折线侧: 只能用三点公式（折线无解析导数）
        - 光滑侧: 用样条解析曲率（传入 tck）
    若不传 tck，光滑侧退回三点公式，其曲率值会随点距虚高，
    对比表将失真 —— 调用方应尽量传入 tck。

    参数:
        raw_path: 原始网格路径
        smooth_path: 光滑后路径
        dynamics: 固定翼动力学模型
        tck: 光滑轨迹的样条表示；None 时退回三点公式并置 curvature_exact=False
    """
    def path_length(p):
        arr = np.asarray(p, dtype=float)
        if len(arr) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))

    def curvature_stats_polyline(p):
        """三点曲率的 (最大, 平均)，单位 1/m"""
        P = np.asarray(p, dtype=float)
        if len(P) < 3:
            return 0.0, 0.0
        ks = []
        for i in range(1, len(P) - 1):
            R = dynamics.get_path_curvature_radius(P[i - 1], P[i], P[i + 1])
            ks.append(1.0 / R if 1e-6 < R < float('inf') else 0.0)
        if not ks:
            return 0.0, 0.0
        return float(np.max(ks)), float(np.mean(ks))

    L_raw = path_length(raw_path)
    L_smooth = path_length(smooth_path)

    k_max_raw, k_mean_raw = curvature_stats_polyline(raw_path)

    if tck is not None:
        k_sm = spline_curvature(tck, np.linspace(0.0, 1.0, 4096))
        k_max_smooth, k_mean_smooth = float(np.max(k_sm)), float(np.mean(k_sm))
        curvature_exact = True
    else:
        k_max_smooth, k_mean_smooth = curvature_stats_polyline(smooth_path)
        curvature_exact = False

    R_min_raw = 1.0 / k_max_raw if k_max_raw > 1e-12 else float('inf')
    R_min_smooth = 1.0 / k_max_smooth if k_max_smooth > 1e-12 else float('inf')

    return {
        'n_points_raw': len(raw_path),
        'n_points_smooth': len(smooth_path),
        'length_raw': L_raw,
        'length_smooth': L_smooth,
        'length_change_pct': (L_smooth / L_raw - 1.0) * 100.0 if L_raw > 1e-6 else 0.0,
        'curvature_max_raw': k_max_raw,
        'curvature_max_smooth': k_max_smooth,
        'curvature_mean_raw': k_mean_raw,
        'curvature_mean_smooth': k_mean_smooth,
        'R_min_raw': R_min_raw,
        'R_min_smooth': R_min_smooth,
        'curvature_max_reduction_pct': (
            (1.0 - k_max_smooth / k_max_raw) * 100.0 if k_max_raw > 1e-12 else 0.0),
        'curvature_exact': curvature_exact,
    }


# ============================================================================
# 向后兼容
# ============================================================================

def smooth_trajectory_bspline(path: List[np.ndarray],
                              s_param: float = None,
                              num_points: int = None) -> List[np.ndarray]:
    """
    [兼容接口] 只返回点列的旧签名。新代码请用 smooth_with_validation，
    它会一并返回 tck，供解析曲率使用。

    注意 s_param=None 时的默认值已按量纲修正为 n·(grid/4)²（原为 n·0.5，
    量纲错误，对 200 m 网格等于不做光滑）。
    """
    P = np.asarray(path, dtype=float)
    if s_param is None:
        s_param = default_s_candidates(len(P), grid_xy=200.0)[1]

    tck, _u, P_dedup = fit_bspline(path, s_param)

    if num_points is None:
        seg = np.linalg.norm(np.diff(P_dedup, axis=0), axis=1)
        ds = float(np.mean(seg)) if len(seg) else 200.0
        pts, _ = resample_by_arclength(tck, ds)
    else:
        pts = np.asarray(splev(np.linspace(0.0, 1.0, int(num_points)), tck),
                         dtype=float).T

    return [np.asarray(p, dtype=float) for p in pts]


def validate_smooth_path(smooth_path: List[np.ndarray],
                         dynamics: FixedWingDynamics) -> Dict:
    """
    [兼容接口] 折线版验证。等价于 validate_polyline。

    警告: 对 B 样条采样点调用本函数会得到随点距发散的转弯半径。
    有 tck 时请用 validate_spline。
    """
    return validate_polyline(smooth_path, dynamics)


# ============================================================================
# 自检测试
# ============================================================================

if __name__ == '__main__':
    print("=" * 78)
    print("  轨迹光滑模块 - 自检测试")
    print("=" * 78)

    dyn = FixedWingDynamics()
    print(f"\n[动力学参数]")
    print(f"  最大爬升角:        {dyn.max_climb_angle_deg}°")
    print(f"  最小可行转弯半径:  {dyn.get_min_feasible_radius():.0f} m  (失速+最大滚转)")
    print(f"  巡航最小转弯半径:  {dyn.get_min_turn_radius(dyn.cruise_speed):.0f} m")

    GRID = 200.0

    # ---- 测试 0: 三点曲率公式的正确性 ----
    print("\n[测试 0] get_path_curvature_radius 标定（已知半径圆）")
    R_true = 1000.0
    for dth in (0.3, 0.03, 0.003):
        th = np.array([-dth, 0.0, dth])
        C = np.stack([R_true * np.cos(th), R_true * np.sin(th), np.zeros(3)], axis=1)
        R = dyn.get_path_curvature_radius(C[0], C[1], C[2])
        print(f"  Δθ={dth:6.3f}  返回 {R:8.2f} m  (真实 {R_true:.0f})")
        assert abs(R / R_true - 1.0) < 1e-6, \
            "Menger 系数应为 2（k=2|cross|/abc），写成 4 会使半径减半"

    # ---- 测试 1: 模拟网格锯齿路径 ----
    # z 每步升 40 m、水平步长 200~283 m → 逐段爬升角 8.0~11.3°，
    # 相对 15° 限制留出余量（真实 A* 输出同样带余量）。
    # 这一点很重要: 光滑会抄近路缩短水平投影，在爬升量不变时抬高
    # 爬升角，若原路径已顶到 15° 则任何光滑都会越界。
    print("\n[测试 1] 200 m 网格锯齿路径的光滑")
    raw = []
    x = y = 0.0
    for i in range(120):
        if i % 3 == 0:
            x += GRID; y += GRID
        elif i % 3 == 1:
            x += GRID
        else:
            x += GRID; y -= GRID
        z = min(3200.0, i * 40.0) if i < 60 else max(0.0, (120 - i) * 40.0)
        raw.append(np.array([x, y, z]))

    raw_val = validate_polyline(raw, dyn)
    print(f"  原折线: R_min={raw_val['min_turn_radius']:.0f} m, "
          f"爬升角={raw_val['max_climb_angle']:.2f}°")

    sm, res = smooth_with_validation(raw, dyn, grid_xy=GRID, corridor_radius=600.0)
    print(f"  {res['message']}")
    v = res['validation']
    print(f"  点数 {len(raw)} → {len(sm)}")
    print(f"  R_min      {v['min_turn_radius']:8.0f} m  "
          f"(≥ {v['min_feasible_radius']:.0f}) {'OK' if v['turn_radius_ok'] else 'FAIL'}")
    print(f"  爬升角     {v['max_climb_angle']:8.2f}°  "
          f"(≤ {dyn.max_climb_angle_deg}) {'OK' if v['climb_angle_ok'] else 'FAIL'}")
    print(f"  相对原折线最大位移 {v['max_raw_deviation']:8.1f} m  "
          f"(≤ {GRID:.0f}) {'OK' if v['smoothing_locality_ok'] else 'FAIL'}")

    # ---- 测试 2: 解析曲率必须与采样密度无关 ----
    print("\n[测试 2] 解析曲率 vs Menger 三点（改变采样点距）")
    tck = res['tck']
    print(f"  {'ds(m)':>7} {'n':>6} {'解析 Rmin':>11} {'Menger Rmin':>13}")
    r_analytic = []
    for ds in (265.0, 130.0, 60.0, 30.0):
        P, u = resample_by_arclength(tck, ds)
        k = spline_curvature(tck, u)
        Ra = 1.0 / max(float(k.max()), 1e-15)
        rs = [dyn.get_path_curvature_radius(P[i - 1], P[i], P[i + 1])
              for i in range(1, len(P) - 1)]
        rs = [r for r in rs if 1e-6 < r < float('inf')]
        print(f"  {ds:7.0f} {len(P):6d} {Ra:11.0f} {min(rs):13.0f}")
        r_analytic.append(Ra)

    spread = (max(r_analytic) - min(r_analytic)) / np.mean(r_analytic)
    print(f"  解析值相对离散度 {spread*100:.1f}%")
    assert spread < 0.15, "解析曲率不应随采样密度显著变化"

    # ---- 测试 3: 速度剖面 ----
    print("\n[测试 3] 速度剖面对比")
    S = np.asarray(sm)
    _, u_s = resample_by_arclength(
        tck, float(np.mean(np.linalg.norm(np.diff(S, axis=0), axis=1))))
    k_s = spline_curvature(tck, u_s) if len(u_s) == len(S) else None

    _, sp_raw, _, ac_raw = dyn.compute_speed_profile(np.asarray(raw))
    _, sp_sm, _, ac_sm = dyn.compute_speed_profile(S, curvatures_in=k_s)

    def jitter(v):
        """相邻点速度变化的 RMS —— 衡量剖面锯齿, 与整体快慢无关。
        注意不能用 v.std(): 折线处处受曲率限速而'均匀地慢', std 反而低。"""
        return float(np.sqrt(np.mean(np.diff(v) ** 2))) if len(v) > 1 else 0.0

    print(f"  {'':10} {'v_mean':>8} {'v_max':>7} {'抖动RMS':>9} {'|a|max':>8}")
    print(f"  {'折线':10} {sp_raw.mean():8.1f} {sp_raw.max():7.1f} "
          f"{jitter(sp_raw):9.3f} {np.abs(ac_raw).max():8.2f}")
    print(f"  {'光滑':10} {sp_sm.mean():8.1f} {sp_sm.max():7.1f} "
          f"{jitter(sp_sm):9.3f} {np.abs(ac_sm).max():8.2f}")

    assert np.abs(ac_sm).max() <= max(dyn.max_decel,
                                      dyn.max_horizontal_accel) + 1e-6, \
        "加速度应在限内"
    assert sp_sm.max() <= dyn.cruise_speed + 1e-6, \
        "默认 v_ceiling=cruise, 速度不应超巡航"
    assert sp_sm.mean() > sp_raw.mean(), \
        "光滑消除了折线的虚假急转, 平均速度应提高"

    # ---- 测试 4: s 候选覆盖 ----
    print("\n[测试 4] 小 s 到大 s 的候选覆盖")
    radii = []
    candidates_test = default_s_candidates(len(raw), GRID)[:8]
    for s in candidates_test:
        t, _, _ = fit_bspline(raw, s)
        R = spline_min_radius(t)
        radii.append(R)
        print(f"  s={s:10.3e}  R_min={R:8.0f} m")
    assert len(candidates_test) >= 6
    assert candidates_test[0] < 1.0e5, "首个 s 应足够小，避免一开始就过度光滑"
    assert max(radii) > min(radii), "候选 s 应能产生不同程度的光滑"

    # ---- 测试 5: 边界情况 ----
    print("\n[测试 5] 边界情况")
    short = [np.array([0., 0., 0.]), np.array([100., 0., 0.]), np.array([200., 0., 0.])]
    _, r = smooth_with_validation(short, dyn, grid_xy=GRID)
    print(f"  3 点路径:      {r['message']}")
    assert not r['smoothed']

    dup = [np.array([0., 0., 0.])] * 3 + \
          [np.array([i * 100., 0., 0.]) for i in range(1, 10)]
    _, r = smooth_with_validation(dup, dyn, grid_xy=GRID)
    print(f"  含重复点:      {r['message'][:60]}")
    assert r['smoothed']

    line = [np.array([i * 200., 0., 0.]) for i in range(20)]
    _, r = smooth_with_validation(line, dyn, grid_xy=GRID)
    print(f"  纯直线:        {r['message'][:60]}")
    assert r['success'], "直线应能成功光滑"

    print("\n" + "=" * 78)
    print("  所有测试通过")
    print("=" * 78)
