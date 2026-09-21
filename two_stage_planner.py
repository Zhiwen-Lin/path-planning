#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
两阶段路径规划器 (Two-Stage Path Planner)
==========================================
通过粗网格获取初始路径，然后在通路内细化网格进行精细规划。

核心思路：
1. 粗网格 A* (2000m × 500m) → 获取粗路径
2. 基于粗路径构建 3D 通路（距离判定，半径 600m）
3. 细网格 A* (200m × 50m) → 仅在通路内搜索，获得精细路径

优势：
- 大幅减少搜索空间（从全地图缩减到通路）
- 细网格能更好地表达运动学约束（转弯半径、爬升角）
- 计算时间可控（通路限制了细网格规模）

作者：Assistant
日期：2026-08-03
"""

import numpy as np
import time
from typing import Tuple, List, Dict

from astar_risk_planner import AStarRiskPlanner
from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel
from fixed_wing_dynamics import FixedWingDynamics
from altitude_constraints import DEFAULT_TRANSITION_DISTANCE_M


# ============================================================================
# 3D 几何工具：点到路径的距离
# ============================================================================

def point_to_segment_distance_3d(point: np.ndarray,
                                  seg_start: np.ndarray,
                                  seg_end: np.ndarray) -> float:
    """
    计算 3D 点到 3D 线段的最短距离

    参数:
        point: np.array([x, y, z]) - 待检测点 (米)
        seg_start: np.array([x, y, z]) - 线段起点 (米)
        seg_end: np.array([x, y, z]) - 线段终点 (米)

    返回:
        float - 最短距离 (米)

    算法:
        1. 计算点在线段方向上的投影参数 t ∈ [0, 1]
        2. t=0: 最近点是起点; t=1: 最近点是终点; 0<t<1: 最近点在线段中间
        3. 计算点到最近点的欧氏距离
    """
    # 线段向量
    seg_vec = seg_end - seg_start
    seg_length_sq = np.dot(seg_vec, seg_vec)

    # 退化情况：线段长度为 0（起点=终点）
    if seg_length_sq < 1e-9:
        return np.linalg.norm(point - seg_start)

    # 点到线段起点的向量
    point_vec = point - seg_start

    # 投影参数 t = (point_vec · seg_vec) / |seg_vec|²
    # t ∈ [0, 1] 保证最近点在线段上
    t = np.dot(point_vec, seg_vec) / seg_length_sq
    t = np.clip(t, 0.0, 1.0)

    # 线段上的最近点
    closest_point = seg_start + t * seg_vec

    # 点到最近点的距离
    distance = np.linalg.norm(point - closest_point)

    return distance


def distance_to_path_3d(point: np.ndarray, path_3d: List[np.ndarray]) -> float:
    """
    计算点到 3D 路径的最短距离

    参数:
        point: np.array([x, y, z]) - 待检测点 (米)
        path_3d: list of np.ndarray - 路径的 3D 坐标序列 [(x1,y1,z1), ...]

    返回:
        float - 到路径的最短距离 (米)

    算法:
        遍历路径的所有线段，取到各线段距离的最小值
    """
    if len(path_3d) < 2:
        # 路径只有一个点，直接返回到该点的距离
        return np.linalg.norm(point - path_3d[0])

    min_distance = float('inf')

    # 遍历路径的每个线段
    for i in range(len(path_3d) - 1):
        seg_start = path_3d[i]
        seg_end = path_3d[i + 1]

        dist = point_to_segment_distance_3d(point, seg_start, seg_end)
        min_distance = min(min_distance, dist)

    return min_distance


def build_corridor_reference(coarse_path: List[np.ndarray],
                             actual_start: Tuple[float, float, float] = None,
                             actual_goal: Tuple[float, float, float] = None
                             ) -> List[np.ndarray]:
    """
    构造两阶段规划唯一的 3D 通路中心线定义：
        actual_start -> coarse_path -> actual_goal

    只有当实际起/终点与粗路径首尾相差 >1 m 时才补入，避免重复点。
    main.py 的 B 样条验证也调用本函数，防止“搜索通路”和“验证通路”
    使用不同定义。
    """
    ref = [np.asarray(p, dtype=float) for p in coarse_path]
    if not ref:
        raise ValueError("coarse_path 为空，无法构造 3D 通路")

    if actual_start is not None:
        s = np.asarray(actual_start, dtype=float)
        if np.linalg.norm(ref[0] - s) > 1.0:
            ref = [s] + ref

    if actual_goal is not None:
        g = np.asarray(actual_goal, dtype=float)
        if np.linalg.norm(ref[-1] - g) > 1.0:
            ref = ref + [g]

    return ref


def is_in_3d_corridor(point: Tuple[float, float, float],
                      coarse_path: List[np.ndarray],
                      corridor_radius: float) -> bool:
    """
    判断点是否在 3D 通路内

    参数:
        point: (x, y, z) - 待检测点 (米)
        coarse_path: list of np.ndarray - 粗路径的 3D 坐标
        corridor_radius: float - 通路半径 (米)

    返回:
        bool - True 表示在通路内，False 表示在通路外

    判定标准:
        点到粗路径的最短距离 ≤ corridor_radius
    """
    point_array = np.array(point)
    dist = distance_to_path_3d(point_array, coarse_path)
    return dist <= corridor_radius


# ============================================================================
# 通路内的细网格规划器
# ============================================================================

class Corridor3DFinePlanner(AStarRiskPlanner):
    """
    基于 3D 通路约束的细网格规划器

    继承自 AStarRiskPlanner，增加通路约束：
    - 节点与细网格边均必须位于距离粗路径 ≤ corridor_radius 的搜索通路内
    - 其他功能（威胁评估、动力学约束等）完全继承
    """

    def __init__(self,
                 coarse_path: List[np.ndarray],
                 corridor_radius: float = 600.0,
                 actual_start: Tuple[float, float, float] = None,
                 actual_goal: Tuple[float, float, float] = None,
                 *args, **kwargs):
        """
        参数:
            coarse_path: 粗路径的 3D 坐标列表（不修改）
            corridor_radius: 通路半径 (米)
            actual_start: 实际起点坐标，仅用于通路检测的端点延伸
            actual_goal: 实际终点坐标，仅用于通路检测的端点延伸
            *args, **kwargs: 传递给父类 AStarRiskPlanner 的参数

        说明:
            粗网格会将起终点对齐到粗网格节点（如 5000m → 4000m on 2000m grid），
            导致实际起终点距粗路径首尾节点可能超过 corridor_radius。
            解决方法：构建通路时，在粗路径首尾各加一段延伸线（到实际起终点），
            使通路自然覆盖起终点附近区域，粗路径数据本身不变。
        """
        super().__init__(*args, **kwargs)

        self.coarse_path = coarse_path
        self.corridor_radius = corridor_radius

        # 搜索与后处理共用同一个通路中心线定义
        corridor_ref = build_corridor_reference(
            coarse_path,
            actual_start=actual_start,
            actual_goal=actual_goal,
        )
        self.corridor_ref = corridor_ref

        # 外接立方体（基于通路参考路径，用于快速剔除）
        ref_array = np.array(corridor_ref)
        self.corridor_bounds = {
            'x_min': ref_array[:, 0].min() - corridor_radius,
            'x_max': ref_array[:, 0].max() + corridor_radius,
            'y_min': ref_array[:, 1].min() - corridor_radius,
            'y_max': ref_array[:, 1].max() + corridor_radius,
            'z_min': ref_array[:, 2].min() - corridor_radius,
            'z_max': ref_array[:, 2].max() + corridor_radius
        }

        print(f"\n[通路规划器初始化]")
        print(f"  粗路径节点数: {len(coarse_path)}")
        print(f"  通路参考路径节点数: {len(corridor_ref)} (含首尾延伸)")
        print(f"  通路半径: {corridor_radius} m")
        print(f"  通路范围:")
        print(f"    X: [{self.corridor_bounds['x_min']/1000:.1f}, {self.corridor_bounds['x_max']/1000:.1f}] km")
        print(f"    Y: [{self.corridor_bounds['y_min']/1000:.1f}, {self.corridor_bounds['y_max']/1000:.1f}] km")
        print(f"    Z: [{self.corridor_bounds['z_min']/1000:.1f}, {self.corridor_bounds['z_max']/1000:.1f}] km")

    def _is_in_bounds(self, pos: Tuple[float, float, float]) -> bool:
        """
        重载父类方法：增加通路约束

        检查顺序：
        1. 地图边界检查（父类逻辑）
        2. 通路外接立方体快速剔除
        3. 精确距离判定（基于含首尾延伸的通路参考路径）
        """
        if not super()._is_in_bounds(pos):
            return False

        x, y, z = pos

        # 外接立方体快速剔除
        if not (self.corridor_bounds['x_min'] <= x <= self.corridor_bounds['x_max'] and
                self.corridor_bounds['y_min'] <= y <= self.corridor_bounds['y_max'] and
                self.corridor_bounds['z_min'] <= z <= self.corridor_bounds['z_max']):
            return False

        # 精确距离判定（用含首尾延伸的参考路径）
        return is_in_3d_corridor(pos, self.corridor_ref, self.corridor_radius)


    def _edge_stays_in_corridor(self,
                                from_pos: Tuple[float, float, float],
                                to_pos: Tuple[float, float, float]) -> bool:
        """
        检查整条细网格边是否留在 3D 通路内。

        旧版只检查“邻居节点”是否在通路内；在弯曲通路处，两端都在管道内
        并不能严格保证连接线段中间也在管道内。这里沿边按 <=100 m 左右
        间距采样，补上这个硬约束。
        """
        a = np.asarray(from_pos, dtype=float)
        b = np.asarray(to_pos, dtype=float)
        length = float(np.linalg.norm(b - a))

        if length < 1e-9:
            return is_in_3d_corridor(
                tuple(a), self.corridor_ref, self.corridor_radius
            )

        spacing = min(
            max(self.grid_res_xy * 0.5, 25.0),
            max(self.corridor_radius * 0.25, 25.0),
        )
        n_samples = max(int(np.ceil(length / spacing)) + 1, 3)

        for t in np.linspace(0.0, 1.0, n_samples):
            pt = a + t * (b - a)
            if not is_in_3d_corridor(
                tuple(pt), self.corridor_ref, self.corridor_radius
            ):
                return False

        return True

    def _collides(self,
                  from_pos: Tuple[float, float, float],
                  to_pos: Tuple[float, float, float]) -> bool:
        """
        细网格阶段的硬约束：
          1. 整条边必须留在 3D 通路内；
          2. 再执行父类的火力禁飞区碰撞检测。
        """
        if not self._edge_stays_in_corridor(from_pos, to_pos):
            return True
        return super()._collides(from_pos, to_pos)


# ============================================================================
# 两阶段规划主流程
# ============================================================================

def two_stage_planning(env: RadarEnvironment5Zones,
                       threat: EnvironmentThreatModel,
                       dynamics: FixedWingDynamics,
                       start: Tuple[float, float, float],
                       goal: Tuple[float, float, float],
                       w_risk: float = 1.0,
                       w_time: float = 0.02,
                       w_fuel: float = 0.0,
                       corridor_radius: float = 600.0,
                       coarse_grid_xy: float = 2000.0,
                       coarse_grid_z: float = 500.0,
                       fine_grid_xy: float = 200.0,
                       fine_grid_z: float = 50.0,
                       use_normalization: bool = True,
                       enroute_min_alt: float = None,
                       transition_distance: float = DEFAULT_TRANSITION_DISTANCE_M,
                       max_flight_alt: float = None) -> Tuple[List[np.ndarray], Dict, List[np.ndarray], Dict]:
    """
    两阶段路径规划：粗网格 → 通路 → 细网格

    参数:
        env: 雷达环境
        threat: 威胁模型
        dynamics: 固定翼动力学模型
        start: (x, y, z) 起点坐标 (米)
        goal: (x, y, z) 终点坐标 (米)
        w_risk: 风险权重
        w_time: 时间权重
        w_fuel: 油耗权重
        corridor_radius: 通路半径 (米)
        coarse_grid_xy: 粗网格水平分辨率 (米)
        coarse_grid_z: 粗网格垂直分辨率 (米)
        fine_grid_xy: 细网格水平分辨率 (米)
        fine_grid_z: 细网格垂直分辨率 (米)
        use_normalization: 是否使用归一化
        enroute_min_alt: 航路中段最低高度 (m)，None 时使用环境值
        transition_distance: 起飞/进场高度过渡水平距离 (m)
        max_flight_alt: 最高高度 (m)，None 时使用环境值

    返回:
        fine_path: 细网格规划的最终路径
        fine_stats: 细路径的统计信息
        coarse_path: 粗网格规划的初始路径（用于对比和可视化）
        coarse_stats: 粗路径的真实统计（L/T/Crisk/F，来自 _compute_path_stats）
                      注意必须返回它: 调用方无法自行重算这些量，
                      否则只能填占位值（曾出现 Crisk 恒为 0 的假统计）
    """
    print("\n" + "=" * 80)
    print("  两阶段路径规划系统")
    print("=" * 80)

    # ========================================================================
    # 前置检查：网格比值 vs 爬升角约束
    # ========================================================================
    #  _get_neighbors() 会拒绝爬升角 > max_climb_angle 的邻居。
    #  单步轴向移动的爬升角 = arctan(grid_res_z / grid_res_xy)。
    #  若该角度超过限制，所有 dz≠0 的邻居全被拒绝，规划器被锁死在
    #  单一高度层，A* 无法爬升 → 搜索必然失败。
    max_climb_deg = dynamics.max_climb_angle_deg
    ratio_limit = np.tan(np.radians(max_climb_deg))

    print(f"\n[前置检查] 网格比值 vs 爬升角约束 (限制 {max_climb_deg}°, 比值上限 {ratio_limit:.3f})")
    for label, gxy, gz in [("粗网格", coarse_grid_xy, coarse_grid_z),
                           ("细网格", fine_grid_xy, fine_grid_z)]:
        ratio = gz / gxy
        angle = np.degrees(np.arctan2(gz, gxy))
        if ratio <= ratio_limit:
            print(f"  ✓ {label} {gxy:.0f}m × {gz:.0f}m: 比值 {ratio:.3f}, 轴向爬升角 {angle:.2f}°")
        else:
            gz_max = gxy * ratio_limit
            print(f"  ✗ {label} {gxy:.0f}m × {gz:.0f}m: 比值 {ratio:.3f}, 轴向爬升角 {angle:.2f}°")
            raise ValueError(
                f"{label}分辨率 {gxy:.0f}m × {gz:.0f}m 违反爬升角约束。\n"
                f"  单步轴向爬升角 {angle:.2f}° > 限制 {max_climb_deg}°，\n"
                f"  所有含高度变化的邻居 (dz≠0) 都会被 _get_neighbors() 拒绝，\n"
                f"  规划器将被锁死在单一高度层而无法爬升。\n"
                f"  修正方法（任选其一）:\n"
                f"    - 垂直分辨率降到 ≤ {gz_max:.0f}m (保持水平 {gxy:.0f}m)\n"
                f"    - 水平分辨率提高到 ≥ {gz/ratio_limit:.0f}m (保持垂直 {gz:.0f}m)"
            )

    # ========================================================================
    # 阶段 1：粗网格规划
    # ========================================================================
    print("\n" + "=" * 80)
    print("【阶段 1/2】粗网格规划")
    print("=" * 80)
    print(f"  网格分辨率: {coarse_grid_xy}m × {coarse_grid_xy}m × {coarse_grid_z}m")
    print(f"  起点: ({start[0]/1000:.1f}, {start[1]/1000:.1f}, {start[2]/1000:.1f}) km")
    print(f"  终点: ({goal[0]/1000:.1f}, {goal[1]/1000:.1f}, {goal[2]/1000:.1f}) km")

    coarse_start_time = time.time()

    # 创建粗网格规划器
    coarse_planner = AStarRiskPlanner(
        env=env,
        threat=threat,
        dynamics=dynamics,
        grid_res_xy=coarse_grid_xy,
        grid_res_z=coarse_grid_z,
        v_cruise=dynamics.cruise_speed,
        w_risk=w_risk,
        w_time=w_time,
        w_fuel=w_fuel,
        n_edge_samples=10,
        max_iterations=10000000,
        epsilon=1.0,
        enroute_min_alt=enroute_min_alt,
        transition_distance=transition_distance,
        max_flight_alt=max_flight_alt,
    )

    # 执行粗网格规划
    try:
        coarse_path, coarse_stats = coarse_planner.plan(start, goal, use_normalization=use_normalization)
    except RuntimeError as e:
        print(f"\n✗ 粗网格规划失败: {e}")
        raise

    coarse_time = time.time() - coarse_start_time

    print(f"\n✓ 粗网格规划完成")
    print(f"  路径节点数: {len(coarse_path)}")
    print(f"  路径航程: {coarse_stats['L']/1000:.1f} km")
    print(f"  飞行时间: {coarse_stats['T']:.0f} s ({coarse_stats['T']/60:.1f} min)")
    print(f"  累积风险: {coarse_stats['Crisk']:.2f}")
    print(f"  燃油消耗: {coarse_stats.get('F', 0.0):.2f} kg")
    print(f"  计算时间: {coarse_time:.2f} s")
    print(f"  迭代次数: {coarse_stats['iterations']:,}")

    # ========================================================================
    # 阶段 2：细网格规划（通路内）
    # ========================================================================
    print("\n" + "=" * 80)
    print("【阶段 2/2】细网格规划（通路内）")
    print("=" * 80)
    print(f"  通路半径: {corridor_radius} m")
    print(f"  网格分辨率: {fine_grid_xy}m × {fine_grid_xy}m × {fine_grid_z}m")
    print(f"  细化倍数: 水平 {coarse_grid_xy/fine_grid_xy:.0f}×, 垂直 {coarse_grid_z/fine_grid_z:.0f}×")

    fine_start_time = time.time()

    # 创建细网格规划器（带通路约束）
    fine_planner = Corridor3DFinePlanner(
        coarse_path=coarse_path,
        corridor_radius=corridor_radius,
        actual_start=start,
        actual_goal=goal,
        env=env,
        threat=threat,
        dynamics=dynamics,
        grid_res_xy=fine_grid_xy,
        grid_res_z=fine_grid_z,
        v_cruise=dynamics.cruise_speed,
        w_risk=w_risk,
        w_time=w_time,
        w_fuel=w_fuel,
        n_edge_samples=10,
        max_iterations=10000000,
        epsilon=1.0,
        enroute_min_alt=enroute_min_alt,
        transition_distance=transition_distance,
        max_flight_alt=max_flight_alt,
    )

    # 执行细网格规划
    # 关键：细网格复用粗网格的归一化基准（T_min, T_max, C_min, C_max, F_min, F_max）
    # 然后在通路内重新搜索最终路径
    try:
        if use_normalization:
            # 粗/细两阶段必须共享同一 payoff-table 尺度。细网格只改变
            # 搜索分辨率，不重新定义论文中的 T_n/C_n/F_n。
            if coarse_planner.baseline is None:
                raise RuntimeError("粗网格规划器缺少共享归一化 baseline")
            fine_planner.set_baseline(coarse_planner.baseline)

            print(f"\n[共享归一化基准]")
            print(f"    T: {fine_planner.T_min:.2f} -> {fine_planner.T_max:.2f} s")
            print(f"    C: {fine_planner.C_min:.6f} -> {fine_planner.C_max:.6f} exposure-s")
            print(f"    F: {fine_planner.F_min:.6f} -> {fine_planner.F_max:.6f} kg")

            # 直接搜索，避免 fine grid 因签名不同再次生成另一套 baseline。
            fine_path, fine_stats = fine_planner._run_astar(
                np.array(start), np.array(goal), compute_baseline=True
            )
        else:
            # 不使用归一化
            fine_path, fine_stats = fine_planner.plan(start, goal, use_normalization=False)

    except RuntimeError as e:
        print(f"\n✗ 细网格规划失败: {e}")
        print(f"  可能原因:")
        print(f"    - 通路半径过小，无法容纳运动学约束")
        print(f"    - 粗路径质量差，导致通路内无可行路径")
        print(f"  建议:")
        print(f"    - 增大 corridor_radius（当前 {corridor_radius}m）")
        print(f"    - 检查粗路径是否合理")
        raise

    fine_time = time.time() - fine_start_time

    print(f"\n✓ 细网格规划完成")
    print(f"  路径节点数: {len(fine_path)}")
    print(f"  路径航程: {fine_stats['L']/1000:.1f} km")
    print(f"  飞行时间: {fine_stats['T']:.0f} s ({fine_stats['T']/60:.1f} min)")
    print(f"  累积风险: {fine_stats['Crisk']:.2f}")
    print(f"  燃油消耗: {fine_stats.get('F', 0.0):.2f} kg")
    print(f"  计算时间: {fine_time:.2f} s")
    print(f"  迭代次数: {fine_stats['iterations']:,}")

    # ========================================================================
    # 结果对比
    # ========================================================================
    print("\n" + "=" * 80)
    print("【结果对比】")
    print("=" * 80)

    def _pct_change(new_value, old_value):
        old_value = float(old_value)
        if abs(old_value) < 1e-12:
            return None
        return (float(new_value) / old_value - 1.0) * 100.0

    print(f"\n路径质量:")

    for label, new_value, old_value, fmt, unit in [
        ("航程变化", fine_stats['L']/1000.0, coarse_stats['L']/1000.0, ".1f", " km"),
        ("时间变化", fine_stats['T'], coarse_stats['T'], ".0f", " s"),
        ("风险变化", fine_stats['Crisk'], coarse_stats['Crisk'], ".2f", ""),
        ("油耗变化", fine_stats.get('F', 0.0), coarse_stats.get('F', 0.0), ".2f", " kg"),
    ]:
        pct = _pct_change(new_value, old_value)
        pct_text = f"{pct:+.1f}%" if pct is not None else "N/A(基准为0)"
        print(
            f"  {label}: {format(old_value, fmt)} → {format(new_value, fmt)}{unit} "
            f"({pct_text})"
        )

    print(f"\n计算效率:")
    print(f"  粗网格: {coarse_time:.2f}s, {coarse_stats['iterations']:,} 迭代")
    print(f"  细网格: {fine_time:.2f}s, {fine_stats['iterations']:,} 迭代")
    print(f"  总耗时: {coarse_time + fine_time:.2f}s")

    print(f"\n分辨率提升:")
    print(f"  粗网格: {coarse_grid_xy}m × {coarse_grid_z}m")
    print(f"  细网格: {fine_grid_xy}m × {fine_grid_z}m")
    print(f"  网格体积比: 1 : {(coarse_grid_xy/fine_grid_xy) * (coarse_grid_z/fine_grid_z):.0f}")

    # 将统计信息也返回
    fine_stats['coarse_time'] = coarse_time
    fine_stats['fine_time'] = fine_time
    fine_stats['total_time'] = coarse_time + fine_time
    fine_stats['coarse_iterations'] = coarse_stats['iterations']
    fine_stats['fine_iterations'] = fine_stats['iterations']

    print("\n" + "=" * 80)
    print("  两阶段规划完成")
    print("=" * 80)

    # 粗网格的耗时也补进 coarse_stats, 与 fine_stats 对称
    coarse_stats['coarse_time'] = coarse_time
    if coarse_planner.baseline is not None:
        # Useful to main.py without recomputing or reopening a second cache.
        fine_stats['baseline'] = coarse_planner.baseline
        coarse_stats['baseline'] = coarse_planner.baseline

    return fine_path, fine_stats, coarse_path, coarse_stats


# ============================================================================
# 测试代码
# ============================================================================

if __name__ == '__main__':
    print("=" * 80)
    print("  两阶段路径规划器 - 测试")
    print("=" * 80)

    # 生成环境
    print("\n[1/4] 生成环境...")
    env = RadarEnvironment5Zones()
    env.generate(seed=42)
    print(f"  ✓ 地图: {env.map_size/1000:.0f}×{env.map_size/1000:.0f} km")
    print(f"  ✓ 雷达区: {len(env.radar_zones)}")
    print(f"  ✓ 火力区: {len(env.fire_zones)}")

    # 威胁模型
    print("\n[2/4] 建立威胁模型...")
    threat = EnvironmentThreatModel(env, model='power', power_n=4.0)
    print("  ✓ 威胁模型: Power-law (n=4.0)")

    # 固定翼动力学
    print("\n[3/4] 建立固定翼动力学...")
    dynamics = FixedWingDynamics()
    print(f"  ✓ 巡航速度: {dynamics.cruise_speed} m/s")
    print(f"  ✓ 最小转弯半径: {dynamics.get_min_turn_radius(dynamics.cruise_speed):.0f} m")

    # 两阶段规划
    print("\n[4/4] 执行两阶段规划...")
    # 起终点取粗细网格的公共节点 (2000 既是 2000m 也是 200m 的整数倍),
    # 否则粗网格会把 5000 对齐到 4000, 与细网格的 5000 差 1414 m。
    start = (2000.0, 2000.0, 0.0)
    goal = (98000.0, 98000.0, 0.0)

    try:
        fine_path, fine_stats, coarse_path, coarse_stats = two_stage_planning(
            env=env,
            threat=threat,
            dynamics=dynamics,
            start=start,
            goal=goal,
            w_risk=1.0,
            w_time=0.02,
            w_fuel=0.0,
            corridor_radius=600.0,
            coarse_grid_xy=2000.0,
            coarse_grid_z=500.0,
            fine_grid_xy=200.0,
            fine_grid_z=50.0,
            use_normalization=True
        )

        print("\n" + "=" * 80)
        print("  ✓ 测试成功！")
        print("=" * 80)
        print(f"\n最终路径:")
        print(f"  节点数: {len(fine_path)}")
        print(f"  航程: {fine_stats['L']/1000:.1f} km")
        print(f"  时间: {fine_stats['T']/60:.1f} min")

    except Exception as e:
        print("\n" + "=" * 80)
        print(f"  ✗ 测试失败: {e}")
        print("=" * 80)
        import traceback
        traceback.print_exc()
