#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A* 风险感知路径规划器
===================
替代 Lazy Theta* 的方案：A* + 样条平滑

核心思路：
1. A* 在3D网格上搜索，代价函数包含风险积分
2. 密集的路径点（50-100个）能精确表达连续风险分布
3. 后续用B样条平滑去除小拐点

与 Theta* 的对比：
- Theta*: 跳点长(10-20km) → LOS采样慢 → 路径点少(5-10个) → 风险评估不准
- A*:      逐格搜索(2km) → 边代价快 → 路径点多(50+个) → 精确跟随风险场

作者：
日期：2026-07-07
"""

import numpy as np
import heapq
import math
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Set, Optional

from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel
from baseline_manager import (BaselineManager, DEFAULT_BASELINE_PATH,
                              weighted_increment, weighted_objective)
from altitude_constraints import (
    DEFAULT_ENROUTE_MIN_ALT_M, DEFAULT_MAX_ALT_M,
    DEFAULT_TRANSITION_DISTANCE_M, altitude_is_allowed,
    minimum_allowed_altitude, altitude_margin, validate_transition_distance,
)

KM = 1000.0


# ============================================================================
# 搜索节点
# ============================================================================

@dataclass(order=True)
class Node:
    """A* 搜索节点"""
    f: float  # f = g + h (总代价估计)
    pos: Tuple[float, float, float] = field(compare=False)  # (x, y, z) 米
    g: float = field(compare=False, default=math.inf)  # 实际代价
    h: float = field(compare=False, default=0.0)  # 启发式估计
    secondary: float = field(compare=False, default=0.0)  # lexicographic path-length tie break
    parent: Optional['Node'] = field(compare=False, default=None, repr=False)


# ============================================================================
# A* 风险感知规划器
# ============================================================================

class AStarRiskPlanner:
    """
    A* 风险感知路径规划器

    特性：
    - 3D网格搜索（水平400m，垂直100m）
    - 统一目标：J = w_T T_n + w_R C_n + w_F F_n
    - 搜索边增量使用同一归一化尺度，保持 A* 可加性
    - 时间采用固定翼段时间模型；最终平滑轨迹可传入自适应速度剖面
    - 风险积分：∫ Pd dt，单位 exposure-s（概率×时间）
    - 油耗模型：升阻比功率模型，考虑爬升、高度、速度影响
    - 硬约束：禁飞区返回INF
    - 启发式：剩余时间下界（秒，与 g 同量纲，保证可采纳性）
    """

    def __init__(self,
                 env: RadarEnvironment5Zones,
                 threat: EnvironmentThreatModel,
                 dynamics=None,
                 grid_res_xy: float = 400.0,
                 grid_res_z: float = 100.0,
                 v_cruise: float = 55.0,
                 w_risk: float = 2.0,
                 w_time: float = 1.0,
                 w_fuel: float = 0.0,
                 n_edge_samples: int = 10,
                 max_iterations: int = 100000,
                 epsilon: float = 1.0,
                 baseline_path: str = DEFAULT_BASELINE_PATH,
                 enroute_min_alt: float = None,
                 transition_distance: float = DEFAULT_TRANSITION_DISTANCE_M,
                 max_flight_alt: float = None):
        """Create the fixed-wing, risk-aware multi-objective A* planner.

        In normalized mode the reported objective is exactly

            J = w_T T_n + w_R C_n + w_F F_n

        with ranges supplied by :mod:`baseline_manager`.  Search remains
        additive by applying the same denominators to each edge increment;
        the three ``-x_min`` offsets are path-independent constants and are
        therefore applied only when reporting the complete-path objective.
        """
        self.env = env
        self.threat = threat
        self.dynamics = dynamics
        self.grid_res_xy = float(grid_res_xy)
        self.grid_res_z = float(grid_res_z)
        self.v_cruise = float(v_cruise)
        self.w_risk = float(w_risk)
        self.w_time = float(w_time)
        self.w_fuel = float(w_fuel)
        self.n_edge_samples = max(int(n_edge_samples), 2)
        self.max_iterations = int(max_iterations)
        self.epsilon = float(epsilon)

        # Use the environment definition rather than a second hard-coded map.
        self.x_min, self.x_max = 0.0, float(getattr(env, 'map_size', 100000.0))
        self.y_min, self.y_max = 0.0, float(getattr(env, 'map_size', 100000.0))

        # Mission altitude envelope:
        #   endpoint altitude (0 m in the experiments)
        #        -> linear transition -> en-route >= 1000 m
        #        -> linear transition -> endpoint altitude.
        # The endpoints therefore remain unchanged, while the mission interior
        # cannot exploit z=0 as a free low-altitude route.
        self.z_min = 0.0  # global box floor; local floor is mission-dependent
        self.enroute_min_alt = float(
            getattr(env, 'min_flight_alt', DEFAULT_ENROUTE_MIN_ALT_M)
            if enroute_min_alt is None else enroute_min_alt
        )
        self.transition_distance = float(transition_distance)
        self.z_max = float(
            getattr(env, 'max_flight_alt', DEFAULT_MAX_ALT_M)
            if max_flight_alt is None else max_flight_alt
        )
        self.mission_start = None
        self.mission_goal = None

        # Straight-line display baseline.
        self.L0 = None
        self.T0 = None
        self.C0 = None

        # Three-anchor payoff-table normalization baseline.
        self.T_min = self.T_max = None
        self.C_min = self.C_max = None
        self.F_min = self.F_max = None
        self.baseline = None
        self.baseline_manager = BaselineManager(baseline_path)
        self.use_normalization = False

        self.iterations = 0
        self.nodes_expanded = 0

    # ------------------------------------------------------------------------
    # 坐标处理
    # ------------------------------------------------------------------------

    def _snap_to_grid(self, point: np.ndarray) -> Tuple[float, float, float]:
        """将点对齐到网格"""
        x = np.round(point[0] / self.grid_res_xy) * self.grid_res_xy
        y = np.round(point[1] / self.grid_res_xy) * self.grid_res_xy
        z = np.round(point[2] / self.grid_res_z) * self.grid_res_z

        # 高度限制
        z = np.clip(z, self.z_min, self.z_max)

        return (float(x), float(y), float(z))

    def set_mission(self, start, goal) -> None:
        """Set mission endpoints used by the spatial altitude envelope."""
        self.mission_start = np.asarray(start, dtype=float).reshape(-1)[:3].copy()
        self.mission_goal = np.asarray(goal, dtype=float).reshape(-1)[:3].copy()

        if self.mission_start.size != 3 or self.mission_goal.size != 3:
            raise ValueError("start/goal must be 3-D points")
        if self.mission_start[2] < self.z_min - 1e-9 or self.mission_start[2] > self.z_max + 1e-9:
            raise ValueError(f"Start altitude {self.mission_start[2]:.1f} m is outside [0, {self.z_max:.1f}] m")
        if self.mission_goal[2] < self.z_min - 1e-9 or self.mission_goal[2] > self.z_max + 1e-9:
            raise ValueError(f"Goal altitude {self.mission_goal[2]:.1f} m is outside [0, {self.z_max:.1f}] m")

        if self.dynamics is not None:
            ok, required = validate_transition_distance(
                self.mission_start, self.mission_goal,
                self.enroute_min_alt, self.transition_distance,
                self.dynamics.max_climb_angle_deg,
            )
            if not ok:
                raise ValueError(
                    f"Altitude transition distance {self.transition_distance:.1f} m is too short: "
                    f"at least {required:.1f} m is required by the "
                    f"{self.dynamics.max_climb_angle_deg:.1f} deg climb-angle limit."
                )

    def minimum_allowed_altitude(self, pos) -> float:
        """Local lower altitude bound for the current mission (m)."""
        if self.mission_start is None or self.mission_goal is None:
            return float(self.z_min)
        return float(minimum_allowed_altitude(
            pos, self.mission_start, self.mission_goal,
            enroute_min_alt=self.enroute_min_alt,
            transition_distance=self.transition_distance,
        ))

    def altitude_margin(self, pos) -> float:
        """Signed clearance to the mission altitude envelope (m)."""
        if self.mission_start is None or self.mission_goal is None:
            z = float(np.asarray(pos, dtype=float)[2])
            return float(min(z - self.z_min, self.z_max - z))
        return float(altitude_margin(
            pos, self.mission_start, self.mission_goal,
            enroute_min_alt=self.enroute_min_alt,
            max_alt=self.z_max,
            transition_distance=self.transition_distance,
        ))

    def altitude_allowed(self, pos) -> bool:
        """Whether a point satisfies the mission altitude hard constraint."""
        if self.mission_start is None or self.mission_goal is None:
            z = float(np.asarray(pos, dtype=float)[2])
            return bool(self.z_min - 1e-9 <= z <= self.z_max + 1e-9)
        return bool(altitude_is_allowed(
            pos, self.mission_start, self.mission_goal,
            enroute_min_alt=self.enroute_min_alt,
            max_alt=self.z_max,
            transition_distance=self.transition_distance,
            tolerance=1e-6,
        ))

    def _is_in_bounds(self, pos: Tuple[float, float, float]) -> bool:
        """Map box + mission altitude-envelope hard constraint."""
        x, y, z = pos
        return bool(
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
            and self.altitude_allowed(pos)
        )

    # ------------------------------------------------------------------------
    # 邻居生成（26邻接）
    # ------------------------------------------------------------------------

    def _get_neighbors(self, pos: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        生成26邻接的邻居

        dx, dy ∈ {-grid_res_xy, 0, +grid_res_xy}
        dz ∈ {-grid_res_z, 0, +grid_res_z}

        【关键改进】过滤掉违反固定翼爬升角约束的邻居
        """
        x, y, z = pos
        neighbors = []

        # 最大爬升角约束来自统一动力学模型，避免多个文件各写一个 15°。
        max_climb_angle_deg = (
            float(self.dynamics.max_climb_angle_deg)
            if self.dynamics is not None else 15.0
        )
        max_climb_angle_rad = np.radians(max_climb_angle_deg)

        for dx in [-self.grid_res_xy, 0, self.grid_res_xy]:
            for dy in [-self.grid_res_xy, 0, self.grid_res_xy]:
                for dz in [-self.grid_res_z, 0, self.grid_res_z]:
                    # 跳过自己
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    neighbor = (x + dx, y + dy, z + dz)

                    # 边界检查
                    if not self._is_in_bounds(neighbor):
                        continue

                    # 【爬升角约束检查】
                    horiz_dist = np.sqrt(dx**2 + dy**2)

                    # 纯垂直移动：固定翼不可能
                    if horiz_dist < 1e-6 and abs(dz) > 1e-6:
                        continue

                    # 有水平移动时，检查爬升角
                    if horiz_dist > 1e-6 and abs(dz) > 1e-6:
                        climb_angle = np.arctan2(abs(dz), horiz_dist)
                        if climb_angle > max_climb_angle_rad:
                            # 爬升角过大，跳过
                            continue

                    neighbors.append(neighbor)

        return neighbors

    # ------------------------------------------------------------------------
    # 碰撞检测（硬约束）
    # ------------------------------------------------------------------------

    def _collides(self, from_pos: Tuple[float, float, float],
                  to_pos: Tuple[float, float, float]) -> bool:
        """
        沿边采样检测禁飞区碰撞

        Returns:
            True: 碰撞
            False: 安全
        """
        from_arr = np.array(from_pos)
        to_arr = np.array(to_pos)

        # 沿边均匀采样
        for i in range(self.n_edge_samples):
            t = i / (self.n_edge_samples - 1) if self.n_edge_samples > 1 else 0
            sample_point = from_arr + t * (to_arr - from_arr)

            # Mission altitude envelope is a hard constraint on every sampled
            # point, not only on grid nodes.  This prevents an edge from cutting
            # below the departure/en-route/terminal lower-altitude envelope.
            if not self.altitude_allowed(sample_point):
                return True

            # 检测火力禁飞区
            if self.threat.check_collision(sample_point):
                return True

        return False

    # ------------------------------------------------------------------------
    # 边代价计算（核心！）
    # ------------------------------------------------------------------------

    def _fuel_heat_value(self) -> float:
        if self.dynamics is None:
            return 43.0e6
        return float(getattr(self.dynamics, 'fuel_heat_value_j_per_kg', 43.0e6))

    def _segment_metrics(self, from_pos, to_pos,
                         v_start: float = None, v_end: float = None) -> Dict[str, float]:
        """Evaluate one straight segment with the project's unified physics.

        The same routine is used by A* edge expansion and by complete-path
        statistics.  If ``v_start``/``v_end`` are supplied (final smoothed
        trajectory), segment time follows the adaptive speed profile.  If
        omitted (A* grid search), time follows ``get_segment_time``.

        Risk is always exposure-time, ``∫ Pd dt``.  Fuel is always kg, from
        shaft/fuel power divided by the fuel lower heating value.
        """
        p0 = np.asarray(from_pos, dtype=float)
        p1 = np.asarray(to_pos, dtype=float)
        delta = p1 - p0
        length = float(np.linalg.norm(delta))
        if length < 1e-12:
            return {'L': 0.0, 'T': 0.0, 'Crisk': 0.0, 'F': 0.0}

        if self._collides(tuple(p0), tuple(p1)):
            return {'L': length, 'T': float('inf'), 'Crisk': float('inf'), 'F': float('inf')}

        # Total segment time.  The adaptive profile returns path-speed
        # magnitudes, so trapezoidal average speed is appropriate here.
        if v_start is not None and v_end is not None:
            v0 = max(float(v_start), 0.0)
            v1 = max(float(v_end), 0.0)
            v_avg = max(0.5 * (v0 + v1), 1e-3)
            total_time = length / v_avg
        elif self.dynamics is not None:
            total_time = float(self.dynamics.get_segment_time(p0, p1, self.v_cruise))
        else:
            total_time = length / max(self.v_cruise, 1e-6)

        n_seg = max(self.n_edge_samples - 1, 1)
        risk = 0.0
        fuel = 0.0
        horiz_total = float(np.hypot(delta[0], delta[1]))
        dh_total = float(delta[2])

        for j in range(n_seg):
            a = j / n_seg
            b = (j + 1) / n_seg
            pa = p0 + a * delta
            pb = p0 + b * delta
            pm = 0.5 * (pa + pb)
            ds = float(np.linalg.norm(pb - pa))

            if v_start is not None and v_end is not None:
                va = max((1.0 - a) * float(v_start) + a * float(v_end), 0.0)
                vb = max((1.0 - b) * float(v_start) + b * float(v_end), 0.0)
                dt = ds / max(0.5 * (va + vb), 1e-3)
            else:
                # Search-stage motion is constant over a grid edge by
                # construction, so divide the dynamics edge time uniformly.
                dt = total_time / n_seg

            pd = float(self.threat.calculate_risk(pm))
            risk += pd * dt

            if self.dynamics is not None and dt > 1e-12:
                d = pb - pa
                v_horizontal = float(np.hypot(d[0], d[1])) / dt
                v_vertical = float(d[2]) / dt
                h_mid = float(pm[2])
                power = float(self.dynamics.get_power_at_state(
                    v_horizontal, v_vertical, h_mid))
                fuel += power * dt / self._fuel_heat_value()

        return {'L': length, 'T': total_time, 'Crisk': risk, 'F': fuel}

    def _edge_cost(self, from_pos: Tuple[float, float, float],
                   to_pos: Tuple[float, float, float]) -> float:
        """Return the additive edge contribution to the multi-objective A*."""
        m = self._segment_metrics(from_pos, to_pos)
        if not np.isfinite(m['T']):
            return float('inf')

        if self.use_normalization:
            if self.baseline is None:
                raise RuntimeError('Normalized A* requested without a loaded baseline')
            return weighted_increment(
                m, self.baseline, self.w_time, self.w_risk, self.w_fuel)

        # Single-objective anchor searches intentionally run unnormalized.
        return float(
            self.w_time * m['T']
            + self.w_risk * m['Crisk']
            + self.w_fuel * m['F']
        )

    # ------------------------------------------------------------------------
    # 启发式函数
    # ------------------------------------------------------------------------

    def _heuristic(self, pos: Tuple[float, float, float],
                   goal: Tuple[float, float, float]) -> float:
        """
        启发式函数：剩余时间下界

        根据是否启用归一化，返回两种量纲的值：
          - 不归一化: h = ε × w_time × (distance / v_cruise)  [单位：秒]
          - 归一化:   h = ε × w_time × (distance / v_cruise) / (T_max - T_min)  [无量纲]

        可采纳性证明（ε=1 时保证最优，ε>1 为加权A*）:

        【情况1：不归一化】
          真实剩余代价 = w_time·T_remain + w_risk·C_remain + w_fuel·F_remain
                      ≥ w_time·T_remain                        (后两项≥0)
                      ≥ w_time·(直线距离/v_cruise)             (三角不等式)
                      = h                                      ✓ 可采纳

        【情况2：归一化】
          真实剩余代价 = w_time·T_remain/(T_max-T_min)
                       + w_risk·C_remain/(C_max-C_min)
                       + w_fuel·F_remain/(F_max-F_min)
                      ≥ w_time·T_remain/(T_max-T_min)          (后两项≥0)
                      ≥ w_time·(直线距离/v_cruise)/(T_max-T_min)  (三角不等式)
                      = h_norm                                 ✓ 可采纳

          关键：边代价和启发式使用相同的归一化因子 (T_max-T_min)，
                保证 g 和 h 量纲一致（f = g + h 才有意义）。

        【ε 参数（加权A*系数）】
          ε = 1.0 : 标准A*，保证找到最优解（但搜索慢）
          ε > 1.0 : 加权A*，加速搜索，解质量 ≤ ε × 最优（牺牲最优性换速度）
          推荐: 复杂场景用 ε=1.0（最优），快速验证用 ε=1.5（接近最优且快2-3倍）
        """
        p1 = np.array(pos)
        p2 = np.array(goal)
        distance = np.linalg.norm(p2 - p1)

        # 剩余时间下界（原始秒）
        h_raw = self.epsilon * self.w_time * distance / self.v_cruise

        # 归一化模式：除以时间范围，使量纲与 g 一致
        if self.use_normalization and self.T_max is not None:
            h_norm = h_raw / max(self.T_max - self.T_min, 1e-6)
            return h_norm
        else:
            return h_raw

    # ------------------------------------------------------------------------
    # 路径重构
    # ------------------------------------------------------------------------

    def _reconstruct_path(self, node: Node) -> List[np.ndarray]:
        """从终点节点回溯路径"""
        path = []
        current = node

        while current is not None:
            path.append(np.array(current.pos))
            current = current.parent

        # 反转（从起点到终点）
        path.reverse()
        return path

    # ------------------------------------------------------------------------
    # 路径评估
    # ------------------------------------------------------------------------

    def evaluate_path(self, path: List[np.ndarray], *,
                      recompute_speed: bool = False,
                      curvatures: np.ndarray = None,
                      strict_feasibility: bool = False,
                      speed_profile: np.ndarray = None,
                      accel_profile: np.ndarray = None) -> Dict[str, float]:
        """Unified evaluator used by A*, ``main.py`` and sweep experiments.

        Parameters
        ----------
        path:
            Path samples in metres.
        recompute_speed:
            ``False`` for a raw A* grid path: every segment is evaluated with
            exactly the same motion model as the A* edge cost.  ``True`` for
            the final B-spline: first compute the adaptive fixed-wing speed
            profile, then integrate time/risk/fuel using that profile.
        curvatures:
            Analytic B-spline curvature samples (1/m).  Passing these avoids
            the unstable three-point curvature proxy on a dense spline.
        strict_feasibility:
            Forwarded to ``compute_speed_profile`` for the final trajectory.

        Returns the physical metrics plus normalized ``T_n/C_n/F_n/J`` when a
        shared baseline is available.
        """
        P = np.asarray(path, dtype=float)
        if P.ndim != 2 or (len(P) and P.shape[1] != 3):
            raise ValueError(f'path must have shape (n,3), got {P.shape}')
        if len(P) < 2:
            out = {'L': 0.0, 'T': 0.0, 'Crisk': 0.0, 'F': 0.0,
                   'min_turn_radius': float('inf'), 'max_curvature': 0.0,
                   'max_climb_angle_deg': 0.0, 'max_speed_change_rate': 0.0}
            if self.baseline is not None:
                out.update(weighted_objective(
                    out, self.baseline, self.w_time, self.w_risk, self.w_fuel))
            return out

        speeds = None
        accels = None
        k_used = None
        if speed_profile is not None:
            speeds = np.asarray(speed_profile, dtype=float).ravel()
            if len(speeds) != len(P):
                raise ValueError("speed_profile length must match path length")
            if accel_profile is not None:
                accels = np.asarray(accel_profile, dtype=float).ravel()
                if len(accels) != len(P):
                    raise ValueError("accel_profile length must match path length")
            k_used = (None if curvatures is None
                      else np.asarray(curvatures, dtype=float).ravel())
        elif recompute_speed and self.dynamics is not None:
            _, speeds, k_used, accels = self.dynamics.compute_speed_profile(
                P, curvatures_in=curvatures,
                strict_feasibility=strict_feasibility)

        totals = {'L': 0.0, 'T': 0.0, 'Crisk': 0.0, 'F': 0.0}
        for i in range(len(P) - 1):
            if speeds is None:
                m = self._segment_metrics(P[i], P[i + 1])
            else:
                m = self._segment_metrics(
                    P[i], P[i + 1], float(speeds[i]), float(speeds[i + 1]))
            if not np.isfinite(m['T']):
                raise ValueError(f'Path segment {i} violates a hard constraint (altitude envelope or no-fly zone)')
            for key in totals:
                totals[key] += float(m[key])

        # Geometry / dynamics metrics requested by the experiments.
        diffs = np.diff(P, axis=0)
        horiz = np.hypot(diffs[:, 0], diffs[:, 1])
        climb = np.degrees(np.arctan2(np.abs(diffs[:, 2]), np.maximum(horiz, 1e-12)))
        max_climb = float(np.max(climb)) if len(climb) else 0.0

        if curvatures is not None:
            k_geom = np.abs(np.asarray(curvatures, dtype=float).ravel())
        elif k_used is not None:
            k_geom = np.abs(np.asarray(k_used, dtype=float).ravel())
        elif self.dynamics is not None:
            k_geom = np.zeros(len(P), dtype=float)
            for i in range(1, len(P) - 1):
                R = self.dynamics.get_path_curvature_radius(P[i-1], P[i], P[i+1])
                if np.isfinite(R) and R > 1e-12:
                    k_geom[i] = 1.0 / R
        else:
            k_geom = np.zeros(len(P), dtype=float)

        max_k = float(np.max(k_geom)) if len(k_geom) else 0.0
        min_R = float('inf') if max_k <= 1e-15 else 1.0 / max_k
        max_speed_rate = (
            float(np.max(np.abs(accels)))
            if accels is not None and len(accels) else 0.0
        )

        out = {**totals,
               'min_turn_radius': min_R,
               'max_curvature': max_k,
               'max_climb_angle_deg': max_climb,
               'max_speed_change_rate': max_speed_rate,
               'speed_min': float(np.min(speeds)) if speeds is not None else float('nan'),
               'speed_max': float(np.max(speeds)) if speeds is not None else float('nan'),
               'speed_mean': float(np.mean(speeds)) if speeds is not None else float('nan')}

        if self.baseline is not None:
            out.update(weighted_objective(
                out, self.baseline, self.w_time, self.w_risk, self.w_fuel))
        else:
            out['J'] = float(
                self.w_time * out['T']
                + self.w_risk * out['Crisk']
                + self.w_fuel * out['F'])
        return out

    def _compute_path_stats(self, path: List[np.ndarray]) -> Dict[str, float]:
        """Backward-compatible wrapper: raw A* statistics equal search physics."""
        return self.evaluate_path(path, recompute_speed=False)

    # ------------------------------------------------------------------------
    # 基准值计算（仅用于显示）
    # ------------------------------------------------------------------------

    def _compute_baseline(self, start: np.ndarray, goal: np.ndarray):
        """Compute straight-line reference values for display only.

        Unlike the old implementation this never caps or rewrites the risk.
        ``C0`` is a direct numerical approximation of ``∫Pd dt`` on the
        straight chord, even if that chord is infeasible because of a hard
        zone.  Feasibility is handled by A*, not by this display reference.
        """
        self.L0 = float(np.linalg.norm(goal - start))
        self.T0 = self.L0 / max(self.v_cruise, 1e-6)
        n = 201
        t = np.linspace(0.0, 1.0, n)
        pts = start[None, :] + t[:, None] * (goal - start)[None, :]
        pd = np.asarray([self.threat.calculate_risk(p) for p in pts], dtype=float)
        # Uniform motion along the straight chord; trapezoidal integration.
        self.C0 = float(np.trapz(pd, t) * self.T0)
        print(f"  Straight reference: L0={self.L0/KM:.2f} km, "
              f"T0={self.T0:.2f} s, C0={self.C0:.6f} exposure-s")

    # ------------------------------------------------------------------------
    # 计算归一化基准
    # ------------------------------------------------------------------------

    def set_baseline(self, baseline: Dict) -> None:
        """Apply an already computed shared baseline to this planner."""
        BaselineManager.apply(self, baseline)

    def _compute_normalization_baseline(self, start: np.ndarray, goal: np.ndarray,
                                        force: bool = False):
        """Load or compute the shared three-anchor ``baseline.pkl``."""
        return self.baseline_manager.ensure(self, start, goal, force=force)

    # ------------------------------------------------------------------------
    # A* 主算法
    # ------------------------------------------------------------------------

    def plan(self, start: Tuple[float, float, float],
             goal: Tuple[float, float, float],
             use_normalization: bool = False) -> Tuple[List[np.ndarray], Dict]:
        """Plan one path; normalized runs all share ``baseline.pkl``."""
        start_arr = np.asarray(start, dtype=float)
        goal_arr = np.asarray(goal, dtype=float)
        self.set_mission(start_arr, goal_arr)

        if use_normalization:
            self._compute_normalization_baseline(start_arr, goal_arr)
            self.use_normalization = True
        else:
            self.use_normalization = False

        print(f"\n[Formal planning] weights: "
              f"w_time={self.w_time}, w_risk={self.w_risk}, w_fuel={self.w_fuel}")
        if self.use_normalization:
            print("  normalized objective: J = w_T*T_n + w_R*C_n + w_F*F_n")

        return self._run_astar(start_arr, goal_arr)

    def _run_astar(self, start: np.ndarray, goal: np.ndarray,
                   compute_baseline: bool = True) -> Tuple[List[np.ndarray], Dict]:
        """执行A*搜索的实际逻辑

        Args:
            start: 起点坐标(m)
            goal: 终点坐标(m)
            compute_baseline: 是否计算直线基准（预计算参考路径时设为False）

        Returns:
            path: 航点列表
            stats: 统计信息
        """
        start_time = time.time()

        # The altitude envelope depends on the original mission endpoints.
        # Set it before snapping or neighbor generation.
        self.set_mission(start, goal)

        # 1. 网格对齐
        start_grid = self._snap_to_grid(start)
        goal_grid = self._snap_to_grid(goal)

        # 只在正式规划时打印详细信息
        if compute_baseline:
            print(f"\n[A* 规划器]")
            print(f"  起点: ({start[0]/KM:.1f}, {start[1]/KM:.1f}, {start[2]/KM:.1f}) km")
            print(f"  终点: ({goal[0]/KM:.1f}, {goal[1]/KM:.1f}, {goal[2]/KM:.1f}) km")
            print(f"  栅格: {self.grid_res_xy/KM:.1f}km × {self.grid_res_z}m")
            print(f"  时间权重: w_time = {self.w_time}")
            print(f"  风险权重: w_risk = {self.w_risk}")
            print(f"  油耗权重: w_fuel = {self.w_fuel}")

        # 2. 计算归一化基准（直线路径，仅用于显示）
        if compute_baseline:
            self._compute_baseline(start, goal)

        # 3. 初始化
        open_set = []  # 优先队列
        closed_set: Set[Tuple[float, float, float]] = set()
        g_scores: Dict[Tuple[float, float, float], float] = {start_grid: 0.0}
        secondary_scores: Dict[Tuple[float, float, float], float] = {start_grid: 0.0}
        counter = 0  # 打破平局

        # 起始节点
        h_start = self._heuristic(start_grid, goal_grid)
        start_node = Node(f=h_start, pos=start_grid, g=0.0, h=h_start, secondary=0.0, parent=None)
        # Secondary geometric tie-break is important for risk-only anchors:
        # outside radar support many edges have exactly zero primary cost.
        # It never changes the primary optimum because it is consulted only
        # when f is exactly tied.
        tie_h = np.linalg.norm(np.asarray(start_grid) - np.asarray(goal_grid))
        heapq.heappush(open_set, (start_node.f, tie_h, counter, start_node))
        counter += 1

        self.iterations = 0
        self.nodes_expanded = 0

        print(f"\n[搜索] 开始A*搜索...")

        # 4. A* 主循环
        while open_set:
            self.iterations += 1

            # 进度输出
            if self.iterations % 2000 == 0:
                current_f = open_set[0][0]
                print(f"  迭代 {self.iterations}: open={len(open_set)}, "
                      f"closed={len(closed_set)}, f={current_f:.2f}")

            # 检查迭代限制
            if self.iterations > self.max_iterations:
                elapsed = time.time() - start_time
                raise RuntimeError(
                    f"超过最大迭代次数 {self.max_iterations}。"
                    f"已扩展 {self.nodes_expanded} 节点，耗时 {elapsed:.1f}s"
                )

            # 弹出f值最小的节点
            _, _, _, current = heapq.heappop(open_set)

            # 已访问
            if current.pos in closed_set:
                continue

            # 标记为已访问
            closed_set.add(current.pos)
            self.nodes_expanded += 1

            # 到达目标
            if current.pos == goal_grid:
                elapsed = time.time() - start_time

                print(f"\n[成功] 找到路径！")
                print(f"  迭代次数: {self.iterations}")
                print(f"  扩展节点: {self.nodes_expanded}")
                print(f"  计算耗时: {elapsed:.2f}s")

                # 重构路径
                path = self._reconstruct_path(current)
                stats = self._compute_path_stats(path)
                stats['iterations'] = self.iterations
                stats['nodes_expanded'] = self.nodes_expanded
                stats['time'] = elapsed
                stats['search_cost_additive'] = float(current.g)

                print(f"\n[路径统计]")
                print(f"  点数: {len(path)}")
                if self.L0 is not None:
                    print(f"  航程: {stats['L']/KM:.2f} km (绕行 {(stats['L']/self.L0-1)*100:+.1f}%)")
                else:
                    print(f"  航程: {stats['L']/KM:.2f} km")
                print(f"  时间: {stats['T']:.0f} s ({stats['T']/60:.1f} min)")
                if self.C0 is not None and abs(self.C0) > 1e-12:
                    print(f"  累积暴露: {stats['Crisk']:.6f} exposure-s "
                          f"(相对直线 {stats['Crisk']/self.C0:.2f})")
                else:
                    print(f"  累积暴露: {stats['Crisk']:.6f} exposure-s")
                print(f"  油耗: {stats.get('F', 0.0):.3f} kg")
                if self.baseline is not None:
                    print(f"  归一化: T_n={stats['T_n']:.4f}, "
                          f"C_n={stats['C_n']:.4f}, F_n={stats['F_n']:.4f}")
                    print(f"  统一目标 J={stats['J']:.6f}")

                return path, stats

            # 扩展邻居
            for neighbor_pos in self._get_neighbors(current.pos):
                # 已访问
                if neighbor_pos in closed_set:
                    continue

                # 计算边代价
                edge_cost = self._edge_cost(current.pos, neighbor_pos)

                # 碰撞（禁飞区）
                if math.isinf(edge_cost):
                    continue

                # Primary objective plus a *lexicographic* geometric
                # secondary objective.  This matters for pure-risk anchors,
                # where many zero-risk routes are exactly tied.
                tentative_g = current.g + edge_cost
                sec_edge = float(np.linalg.norm(
                    np.asarray(neighbor_pos) - np.asarray(current.pos)))
                tentative_secondary = current.secondary + sec_edge

                old_g = g_scores.get(neighbor_pos, math.inf)
                old_sec = secondary_scores.get(neighbor_pos, math.inf)
                primary_better = tentative_g < old_g - 1e-12
                exact_primary_tie = abs(tentative_g - old_g) <= 1e-12
                secondary_better = tentative_secondary < old_sec - 1e-9

                if primary_better or (exact_primary_tie and secondary_better):
                    g_scores[neighbor_pos] = tentative_g
                    secondary_scores[neighbor_pos] = tentative_secondary
                    h = self._heuristic(neighbor_pos, goal_grid)
                    f = tentative_g + h

                    neighbor_node = Node(
                        f=f, pos=neighbor_pos, g=tentative_g, h=h,
                        secondary=tentative_secondary, parent=current)

                    tie_h = tentative_secondary + float(np.linalg.norm(
                        np.asarray(neighbor_pos) - np.asarray(goal_grid)))
                    heapq.heappush(open_set, (f, tie_h, counter, neighbor_node))
                    counter += 1

        # 5. Open set为空，未找到路径
        elapsed = time.time() - start_time
        raise RuntimeError(
            f"A*未找到路径。已扩展 {self.nodes_expanded} 节点，"
            f"迭代 {self.iterations} 次，耗时 {elapsed:.1f}s"
        )


# ============================================================================
# 测试
# ============================================================================

if __name__ == '__main__':
    from fixed_wing_dynamics import FixedWingDynamics

    print("=" * 70)
    print("  A* 风险感知规划器 - 测试")
    print("=" * 70)

    # 生成环境
    print("\n[1/5] 生成环境...")
    env = RadarEnvironment5Zones()
    env.generate(seed=42)

    # 威胁模型
    print("[2/5] 建立威胁模型...")
    threat = EnvironmentThreatModel(env, model='power', power_n=4.0)

    # 固定翼动力学
    print("[3/5] 建立固定翼动力学...")
    dynamics = FixedWingDynamics()

    # A* 规划器（权重从 pickle 读取，与 main.py 一致；测试时用构造函数默认值）
    print("[4/5] 初始化规划器...")
    import pickle as _test_pkl, os as _test_os
    _P = "output_5zones/planned_path.pkl"
    if _test_os.path.exists(_P):
        with open(_P, 'rb') as _f:
            _d = _test_pkl.load(_f)
        _wr, _wt, _wf = _d['w_risk'], _d['w_time'], _d['w_fuel']
    else:
        _wr, _wt, _wf = 2.0, 1.0, 0.0  # 测试默认值
    planner = AStarRiskPlanner(
        env=env, threat=threat, dynamics=dynamics,
        grid_res_xy=400.0, grid_res_z=100.0, v_cruise=55.0,
        w_risk=_wr, w_time=_wt, w_fuel=_wf,
        n_edge_samples=10, max_iterations=10000000
    )

    # 规划
    print("[5/5] 开始规划...")
    start = (5000.0, 5000.0, 0.0)  # 起点高度0km
    goal = (95000.0, 95000.0, 0.0)  # 终点高度0km

    try:
        path, stats = planner.plan(start, goal)
        print("\n✓ 测试成功！")
    except RuntimeError as e:
        print(f"\n✗ 测试失败: {e}")
