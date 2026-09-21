#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
main.py - A*路径规划主程序（两阶段细化 + B样条光滑）
==========================================================
整合所有功能：规划、光滑、可视化、统计报告

流程：
1. 生成5区雷达环境
2. 【阶段1】粗网格A*规划 (2000m×500m) → 获取初始路径
3. 生成粗网格可视化（保留用于对比）
4. 【阶段2】基于粗路径构建3D通路（半径600m）
5. 细网格A*规划 (200m×50m) → 在通路内精细规划
6. 【B样条光滑】三次B样条光滑处理 → C²连续轨迹
7. 生成细网格可视化（最终结果，光滑后路径）
8. 输出统计报告（包含粗细对比 + 光滑效果）

用法：
    python main.py
"""

import numpy as np
import time
import os
import shutil
import sys
from datetime import datetime

from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel
from fixed_wing_dynamics import FixedWingDynamics
from trajectory_data_loader import main_trajectory_signature
from astar_risk_planner import AStarRiskPlanner
from two_stage_planner import two_stage_planning, build_corridor_reference
from trajectory_smoothing import (smooth_with_validation, compare_paths,
                                  spline_curvature, resample_by_arclength,
                                  corridor_deviation_stats, SMOOTHING_VERSION)
from altitude_constraints import (
    DEFAULT_TRANSITION_DISTANCE_M, altitude_is_allowed,
    minimum_allowed_altitude, validate_transition_distance,
)

print("=" * 80)
print("  A*固定翼路径规划系统（两阶段细化 + B样条光滑）")
print("=" * 80)
print(f"  运行时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
print(f"  轨迹光滑版本: {SMOOTHING_VERSION}")
print("\n说明：")
print("  - 先用粗网格快速规划，确定大致路线")
print("  - 再用细网格精细优化，提高运动学准确性")
print("  - B样条光滑处理，消除网格锯齿，生成C²连续轨迹")
print("  - 两套结果都会保存，方便对比分析")

# ============================================================================
# 权重配置（归一化模式）
# ============================================================================
#   归一化后，权重直接反映各目标的相对重要性。
#   研究者根据任务需求直接设计权重，无需通过二分法反解。
#
#   权重范围建议：
#     - w_time:  0.5 ~ 2.0  (时间重要性)
#     - w_risk:  0.5 ~ 2.0  (风险规避程度)
#     - w_fuel:  0.0 ~ 1.0  (油耗约束强度)
#
#   典型配置：
#     - 快速任务:   w_time=2.0, w_risk=0.5, w_fuel=0.0
#     - 隐蔽任务:   w_time=0.5, w_risk=2.0, w_fuel=0.0
#     - 长航时任务: w_time=1.0, w_risk=1.0, w_fuel=1.0

W_TIME = 0.3  # 时间权重
W_RISK = 1     # 风险权重
W_FUEL = 0.2     # 油耗权重

print("\n[权重配置]")
print(f"  w_time = {W_TIME}  (时间重要性)")
print(f"  w_risk = {W_RISK}  (风险规避程度)")
print(f"  w_fuel = {W_FUEL}  (油耗约束强度)")
print("=" * 80)

# ============================================================================
# 网格配置
# ============================================================================
CORRIDOR_RADIUS = 600.0   # 仅用于细网格A*搜索空间剪枝，不是最终飞行硬边界
COARSE_GRID_XY = 2000.0   # 粗网格水平分辨率 (米)
COARSE_GRID_Z = 500.0     # 粗网格垂直分辨率 (米)
FINE_GRID_XY = 200.0      # 细网格水平分辨率 (米)
FINE_GRID_Z = 50.0        # 细网格垂直分辨率 (米)

# V5: 最终B样条局部性约束——最多偏离细网格优化路径一个细网格尺度。
# 真正的硬安全约束仍由 threat.check_collision() 和固定翼运动学负责。
SMOOTHING_TRUST_RADIUS = FINE_GRID_XY
ENFORCE_SEARCH_CORRIDOR_ON_SPLINE = False
FINAL_PATH_DS = 100.0  # B样条最终路径/动力学统计统一按约100 m弧长采样

# 起终点高度保持论文设定不变（0 m）。仅对飞行过程施加空间高度包络：
# 0 m endpoint -> 5 km transition -> 1000~10000 m en-route
# -> 5 km transition -> 0 m endpoint.
ALTITUDE_TRANSITION_DISTANCE = DEFAULT_TRANSITION_DISTANCE_M
#
# 【重要】网格比值必须满足爬升角约束:
#   grid_res_z / grid_res_xy ≤ tan(max_climb_angle) = tan(15°) = 0.268
#
#   _get_neighbors() 会拒绝爬升角超过 15° 的邻居。若比值超限，
#   所有带高度变化的邻居 (dz≠0) 全被拒绝，规划器被锁死在单一高度层，
#   导致 A* 无法爬升而搜索失败。
#
#   当前配置:
#     粗网格 500/2000 = 0.250 ✓ (轴向爬升角 14.04°)
#     细网格  50/200  = 0.250 ✓ (轴向爬升角 14.04°)
#     网格体积比: 1 : 100（相比 100×25 的 1:400，搜索空间减少 4 倍）

# ============================================================================
# Step 1: 生成环境
# ============================================================================
print("\n[Step 1/7] 生成战场环境...")
env = RadarEnvironment5Zones()
env.generate(seed=42)
print("  ✓ 5区雷达环境生成完成")
print(f"    - 地图大小: {env.map_size/1000:.0f} × {env.map_size/1000:.0f} km")
print(f"    - 雷达区数: {len(env.radar_zones)}")
print(f"    - 火力区数: {len(env.fire_zones)}")
print(f"    - 航路高度范围: {env.min_flight_alt:.0f} ~ {env.max_flight_alt:.0f} m")
print(f"    - 起飞/进场高度过渡距离: {ALTITUDE_TRANSITION_DISTANCE/1000:.1f} km")

# ============================================================================
# Step 2: 建立威胁模型
# ============================================================================
print("\n[Step 2/7] 建立威胁模型...")
threat = EnvironmentThreatModel(env, model='power', power_n=4.0)
print("  ✓ 威胁模型建立完成")
print(f"    - 模型类型: Power-law (n=4.0)")

# ============================================================================
# Step 3: 建立固定翼动力学
# ============================================================================
print("\n[Step 3/7] 建立固定翼动力学...")
dynamics = FixedWingDynamics()
print("  ✓ 固定翼动力学模型建立完成")
print(f"    - 巡航速度: {dynamics.cruise_speed} m/s ({dynamics.cruise_speed*3.6:.0f} km/h)")
print(f"    - 最大加速度: {dynamics.max_horizontal_accel} m/s²")
print(f"    - 最大减速度: {dynamics.max_decel} m/s²")
print(f"    - 最大爬升率: {dynamics.max_ascend_speed} m/s")
print(f"    - 最小转弯半径(巡航): {dynamics.get_min_turn_radius(dynamics.cruise_speed):.0f} m")

# ============================================================================
# Step 4: 两阶段路径规划
# ============================================================================
print("\n[Step 4/7] 开始两阶段路径规划...")
start = (2000.0, 2000.0, 0.0)
goal = (98000.0, 98000.0, 0.0)

print(f"  起点: ({start[0]/1000:.1f}, {start[1]/1000:.1f}, {start[2]/1000:.1f}) km")
print(f"  终点: ({goal[0]/1000:.1f}, {goal[1]/1000:.1f}, {goal[2]/1000:.1f}) km")

_alt_ok, _alt_required = validate_transition_distance(
    start, goal, env.min_flight_alt, ALTITUDE_TRANSITION_DISTANCE,
    dynamics.max_climb_angle_deg,
)
if not _alt_ok:
    raise RuntimeError(
        f"高度过渡距离 {ALTITUDE_TRANSITION_DISTANCE:.1f} m 小于动力学最低需求 "
        f"{_alt_required:.1f} m。"
    )
print(f"  高度包络: 端点保持 {start[2]:.0f}/{goal[2]:.0f} m；"
      f"中段 {env.min_flight_alt:.0f}~{env.max_flight_alt:.0f} m；"
      f"过渡 {ALTITUDE_TRANSITION_DISTANCE/1000:.1f} km")

print(f"\n  网格配置:")
print(f"    - 粗网格: {COARSE_GRID_XY}m × {COARSE_GRID_Z}m")
print(f"    - 细网格: {FINE_GRID_XY}m × {FINE_GRID_Z}m")
print(f"    - 通路半径: {CORRIDOR_RADIUS}m")
print(f"    - 细化倍数: {(COARSE_GRID_XY/FINE_GRID_XY) * (COARSE_GRID_Z/FINE_GRID_Z):.0f}×")

planning_start = time.time()

path, stats, coarse_path, coarse_stats = two_stage_planning(
    env=env,
    threat=threat,
    dynamics=dynamics,
    start=start,
    goal=goal,
    w_risk=W_RISK,
    w_time=W_TIME,
    w_fuel=W_FUEL,
    corridor_radius=CORRIDOR_RADIUS,
    coarse_grid_xy=COARSE_GRID_XY,
    coarse_grid_z=COARSE_GRID_Z,
    fine_grid_xy=FINE_GRID_XY,
    fine_grid_z=FINE_GRID_Z,
    use_normalization=True,
    enroute_min_alt=env.min_flight_alt,
    transition_distance=ALTITUDE_TRANSITION_DISTANCE,
    max_flight_alt=env.max_flight_alt,
)

planning_time = time.time() - planning_start

print(f"\n  ✓ 路径规划完成")

# ============================================================================
# Step 4.5: B样条轨迹光滑处理
# ============================================================================
print("\n[Step 4.5/7] B样条轨迹光滑处理...")
print(f"  细网格原始路径点数: {len(path)}")

# 保存原始细网格路径（未光滑）
path_raw = path
stats_raw = stats.copy()

# 执行光滑
#   grid_xy 用于 s 参数的量纲定标 (s ≈ n·δ², δ 为允许的每点偏离)
#   corridor_radius 仅用于报告粗路径搜索走廊；最终硬约束见下方参数
smooth_start = time.time()

# 搜索阶段与后处理阶段使用完全相同的 3D 通路中心线
_corridor_reference = build_corridor_reference(
    coarse_path,
    actual_start=start,
    actual_goal=goal,
)

# 先诊断“细网格原始折线”离 600 m 通路边界还有多少余量。
# 这样若后续失败，可以区分：
#   A. 原始细路径已经贴边/越界 -> 通路本身或边约束问题
#   B. 原始路径余量充足，但某个 B 样条候选过度外鼓 -> 调 s
_raw_corr = corridor_deviation_stats(
    np.asarray(path_raw, dtype=float),
    np.asarray(_corridor_reference, dtype=float),
    sample_spacing=min(50.0, FINE_GRID_XY / 2.0),
)
_raw_margin = CORRIDOR_RADIUS - _raw_corr["max_deviation"]

print(
    f"  细网格原始路径最大通路偏离: {_raw_corr['max_deviation']:.1f} m "
    f"(通路 {CORRIDOR_RADIUS:.0f} m, 剩余余量 {_raw_margin:.1f} m)"
)

if _raw_corr["max_deviation"] > CORRIDOR_RADIUS + 1e-6:
    raise RuntimeError(
        "细网格原始路径本身已经离开 3D 通路："
        f"最大偏离 {_raw_corr['max_deviation']:.1f} m > "
        f"{CORRIDOR_RADIUS:.1f} m。"
        "这不是 B 样条参数问题，而是细网格边的通路硬约束没有生效。"
        "请确认正在使用与 V5.3 配套的 two_stage_planner.py，并从头重新运行 main.py。"
    )

path_smooth, smooth_result = smooth_with_validation(
    path_raw,
    dynamics,
    grid_xy=FINE_GRID_XY,

    # coarse corridor 仅作为搜索域/诊断量
    corridor_radius=CORRIDOR_RADIUS,
    corridor_reference=_corridor_reference,
    enforce_search_corridor=ENFORCE_SEARCH_CORRIDOR_ON_SPLINE,

    # 最终轨迹真正检查：动力学 + 硬禁飞 + 相对 fine path 的局部性
    collision_checker=threat.check_collision,
    altitude_checker=lambda p: altitude_is_allowed(
        p, start, goal,
        enroute_min_alt=env.min_flight_alt,
        max_alt=env.max_flight_alt,
        transition_distance=ALTITUDE_TRANSITION_DISTANCE,
    ),
    altitude_floor=lambda p: minimum_allowed_altitude(
        p, start, goal,
        enroute_min_alt=env.min_flight_alt,
        transition_distance=ALTITUDE_TRANSITION_DISTANCE,
    ),
    max_smoothing_deviation=SMOOTHING_TRUST_RADIUS,
    ds_target=FINAL_PATH_DS,
)
smooth_time = time.time() - smooth_start

print(f"  {smooth_result['message']}")
print(f"  光滑后路径点数: {len(path_smooth)}")

# 最终结果不允许悄悄使用违反“真实硬约束”的 fallback 样条
if not smooth_result["success"]:
    print("\n  [B样条候选诊断]")
    hist = smooth_result.get("candidate_history", [])

    if hist:
        print(
            "    s(m²)        Rmin(m)  climb(deg)  raw_dev(m)  "
            "search_corr(m)  collision  status"
        )

        for item in hist:
            if "error" in item:
                print(
                    f"    {item['s']:10.3e}  "
                    f"{item.get('stage', 'unknown'):>10s} ERROR  "
                    f"{item['error'][:80]}"
                )
                continue

            if item["feasible"]:
                status = "OK"
            elif not item["turn_radius_ok"]:
                status = "TURN"
            elif not item["climb_angle_ok"]:
                status = "CLIMB"
            elif not item.get("endpoints_ok", True):
                status = "ENDPOINT"
            elif not item.get("terminal_heading_ok", True):
                status = "HEADING"
            elif not item["smoothing_locality_ok"]:
                status = "LOCALITY"
            elif not item.get("collision_free", True):
                status = "COLLISION"
            elif (
                ENFORCE_SEARCH_CORRIDOR_ON_SPLINE
                and not item["search_corridor_ok"]
            ):
                status = "SEARCH_CORRIDOR"
            else:
                status = "UNKNOWN"

            print(
                f"    {item['s']:10.3e}  "
                f"{item['min_turn_radius']:9.1f}  "
                f"{item['max_climb_angle']:10.2f}  "
                f"{item['max_raw_deviation']:10.1f}  "
                f"{item['max_corridor_deviation']:14.1f}  "
                f"{str(item.get('collision_free', True)):>9s}  "
                f"{status}"
            )

    raise RuntimeError(
        "B样条没有找到满足最终真实硬约束的可行轨迹。"
        "请根据上表判断是 TURN / CLIMB / LOCALITY / COLLISION；"
        "注意 coarse 600 m 搜索走廊默认不再作为最终样条硬约束。\n"
        + smooth_result["message"]
    )

# 运动学验证结果
v = smooth_result['validation']
print(f"  最大爬升角: {v['max_climb_angle']:.2f}° "
      f"(限制 {dynamics.max_climb_angle_deg}°) "
      f"{'✓' if v['climb_angle_ok'] else '✗'}")
print(f"  最小转弯半径: {v['min_turn_radius']:.0f} m "
      f"(限制 {dynamics.get_min_feasible_radius():.0f} m) "
      f"{'✓' if v['turn_radius_ok'] else '✗'}")
print(f"  相对细网格原始路径最大位移: {v['max_raw_deviation']:.1f} m "
      f"(局部性限制 {SMOOTHING_TRUST_RADIUS:.0f} m) "
      f"{'✓' if v['smoothing_locality_ok'] else '✗'}")
print(f"  相对粗路径搜索走廊中心线: 最大 {v['max_corridor_deviation']:.1f} m "
      f"(搜索半径 {CORRIDOR_RADIUS:.0f} m；仅诊断) "
      f"{'✓' if v['search_corridor_ok'] else '△'}")
print(f"  硬禁飞/火力区碰撞: {'无 ✓' if v['collision_free'] else '有 ✗'}")
print(f"  端点保持: 起点误差 {v['start_error']:.3f} m, 终点误差 {v['end_error']:.3f} m "
      f"{'✓' if v['endpoints_ok'] else '✗'}")
print(f"  末端航向误差: {v['terminal_heading_error_deg']:.2f}° "
      f"(限制 {v['terminal_heading_tolerance_deg']:.1f}°) "
      f"{'✓' if v['terminal_heading_ok'] else '✗'}")
print(f"  曲率变化积分 J_kappa': {v['curvature_variation_integral']:.6e} 1/m³")
print(f"  全局平滑评价 J_s: {smooth_result.get('selection_score', float('nan')):.6f}")

# 光滑效果统计
if smooth_result['smoothed']:
    cmp = compare_paths(path_raw, path_smooth, dynamics, tck=smooth_result['tck'])
    print(f"\n  [光滑效果]")
    print(f"    航程: {cmp['length_raw']/1000:.2f} → {cmp['length_smooth']/1000:.2f} km "
          f"({cmp['length_change_pct']:+.2f}%)")
    print(f"    点数: {cmp['n_points_raw']} → {cmp['n_points_smooth']}")
    print(f"    最小转弯半径: {cmp['R_min_raw']:.0f} → {cmp['R_min_smooth']:.0f} m")
    print(f"    最大曲率: {cmp['curvature_max_raw']:.6f} → {cmp['curvature_max_smooth']:.6f} 1/m "
          f"(降低 {cmp['curvature_max_reduction_pct']:.1f}%)")
    print(f"  光滑耗时: {smooth_time:.2f} s")

# 最终流程固定为：B样条 -> 解析曲率 -> 自适应速度 -> 统一物理评价
print(f"\n  按最终流程重新评价光滑轨迹: B样条 → 曲率 → 速度 → T/C/F ...")
path_smooth_array = np.asarray(path_smooth, dtype=float)

if smooth_result['tck'] is None:
    raise RuntimeError("最终光滑路径缺少 B 样条 tck，无法进行解析曲率动力学验证。")
u_smooth = smooth_result.get('u_smooth')
if u_smooth is None:
    raise RuntimeError("B样条结果缺少 u_smooth；请确认使用 V5.3 配套文件。")

# 1) Curvature -- analytic B-spline derivative only.
curv_smooth = spline_curvature(smooth_result['tck'], np.asarray(u_smooth))
if len(curv_smooth) != len(path_smooth_array):
    raise RuntimeError(
        f"解析曲率点数 {len(curv_smooth)} 与最终路径点数 "
        f"{len(path_smooth_array)} 不一致；禁止退回三点曲率。")

# 2) Adaptive fixed-wing speed profile.
s_dense, speed_profile, curvatures_smooth, accel_smooth = dynamics.compute_speed_profile(
    path_smooth_array,
    curvatures_in=curv_smooth,
    strict_feasibility=True,
)

# 3) One evaluator for time, Pd×dt risk, kg fuel and normalized objective.
final_evaluator = AStarRiskPlanner(
    env=env, threat=threat, dynamics=dynamics,
    grid_res_xy=FINE_GRID_XY, grid_res_z=FINE_GRID_Z,
    v_cruise=dynamics.cruise_speed,
    w_risk=W_RISK, w_time=W_TIME, w_fuel=W_FUEL,
    n_edge_samples=10, max_iterations=1, epsilon=1.0,
    enroute_min_alt=env.min_flight_alt,
    transition_distance=ALTITUDE_TRANSITION_DISTANCE,
    max_flight_alt=env.max_flight_alt,
)
final_evaluator.set_mission(start, goal)
baseline = stats_raw.get('baseline')
if baseline is None:
    raise RuntimeError("two_stage_planning 未返回共享 baseline；请确认配套文件版本。")
final_evaluator.set_baseline(baseline)

stats_smooth = final_evaluator.evaluate_path(
    path_smooth_array,
    curvatures=curv_smooth,
    speed_profile=speed_profile,
    accel_profile=accel_smooth,
    strict_feasibility=True,
)
# Preserve search/runtime metadata while replacing only the final physical metrics.
stats_smooth.update({
    'iterations': stats_raw['iterations'],
    'nodes_expanded': stats_raw.get('nodes_expanded', 0),
    'time': stats_raw['time'],
    'coarse_time': stats_raw.get('coarse_time', 0),
    'fine_time': stats_raw.get('fine_time', 0),
    'total_time': stats_raw.get('total_time', 0),
    'coarse_iterations': stats_raw.get('coarse_iterations', 0),
    'fine_iterations': stats_raw.get('fine_iterations', 0),
    'smooth_time': smooth_time,
    'smoothed': True,
    'smoothing_score': smooth_result.get('selection_score', float('nan')),
    'curvature_variation_integral': v.get('curvature_variation_integral', float('nan')),
    'terminal_heading_error_deg': v.get('terminal_heading_error_deg', float('nan')),
    'baseline': baseline,
})

print(f"    速度: {speed_profile.min():.1f} ~ {speed_profile.max():.1f} m/s "
      f"(均值 {speed_profile.mean():.2f}, 标准差 {speed_profile.std():.2f})")
print(f"    最大速度变化率 |dv/dt|: {stats_smooth['max_speed_change_rate']:.3f} m/s²")
print(f"    航程: {stats_smooth['L']/1000:.3f} km")
print(f"    时间: {stats_smooth['T']:.2f} s ({stats_smooth['T']/60:.2f} min)")
print(f"    累积风险: {stats_smooth['Crisk']:.6f} exposure-s")
print(f"    燃油: {stats_smooth['F']:.6f} kg")
print(f"    T_n/C_n/F_n = {stats_smooth['T_n']:.5f} / "
      f"{stats_smooth['C_n']:.5f} / {stats_smooth['F_n']:.5f}")
print(f"    统一目标 J = {stats_smooth['J']:.6f}")

# 后续所有输出统一使用光滑后的最终物理轨迹。
path = path_smooth
stats = stats_smooth

# 保存路径到文件，供可视化脚本直接加载
import pickle
os.makedirs("output_5zones", exist_ok=True)

# 保存粗网格路径（用于对比）
#
# coarse_stats 直接用 two_stage_planning 返回的真实统计
# （规划器内部 _compute_path_stats 算的 L / T / Crisk / F）。
#
# 旧版在此处自行重造了一份统计: 'Crisk': 0 写死、'T' 用 L/v_cruise 恒速倒推，
# 于是 planned_path_coarse.pkl 里的累积风险恒为 0 —— 图上标注"风险 0"
# 而轨迹明显穿过雷达区，就是这个假统计导致的。
coarse_stats = dict(coarse_stats)
coarse_stats.setdefault('nodes_expanded', 0)
coarse_stats['time'] = coarse_stats.get('coarse_time',
                                        stats.get('coarse_time', 0))

print(f"\n  粗网格真实统计:")
print(f"    航程: {coarse_stats['L']/1000:.2f} km")
print(f"    时间: {coarse_stats['T']:.0f} s ({coarse_stats['T']/60:.1f} min)")
print(f"    风险: {coarse_stats['Crisk']:.2f}")
print(f"    油耗: {coarse_stats.get('F', 0.0):.2f} kg")

# ----------------------------------------------------------------------------
# Straight-display reference comes from the same shared baseline object.
# No second formula and no ad-hoc risk cap are allowed here.
# ----------------------------------------------------------------------------
L0_ref = float(baseline['L0'])
T0_ref = float(baseline['T0'])
C0_ref = float(baseline['C0'])

print(f"\n  共享 baseline.pkl:")
print(f"    L0 = {L0_ref/1000:.2f} km, T0 = {T0_ref:.2f} s, C0 = {C0_ref:.6f}")
print(f"    T range = [{baseline['T_min']:.3f}, {baseline['T_max']:.3f}] s")
print(f"    C range = [{baseline['C_min']:.6f}, {baseline['C_max']:.6f}] exposure-s")
print(f"    F range = [{baseline['F_min']:.6f}, {baseline['F_max']:.6f}] kg")

coarse_data = {
    'trajectory_signature': main_trajectory_signature(baseline),
    'path': coarse_path,
    'stats': coarse_stats,
    'grid_res_xy': COARSE_GRID_XY,
    'grid_res_z': COARSE_GRID_Z,
    'v_cruise': dynamics.cruise_speed,
    'w_risk': W_RISK,
    'w_time': W_TIME,
    'w_fuel': W_FUEL,
    'L0': L0_ref,
    'C0': C0_ref,
    'T0': T0_ref,
    'start': start,
    'goal': goal,
    'baseline': baseline,
}
with open("output_5zones/planned_path_coarse.pkl", 'wb') as f:
    pickle.dump(coarse_data, f)
print("  ✓ 粗网格路径已保存: output_5zones/planned_path_coarse.pkl")

# 保存细网格路径（最终结果 - 光滑后）
fine_data = {
    'trajectory_signature': main_trajectory_signature(baseline),
    'path': path,              # 光滑后路径
    'path_raw': path_raw,      # 原始细网格路径（未光滑）
    'stats': stats,            # 光滑后统计
    'stats_raw': stats_raw,    # 原始细网格统计
    'smooth_result': smooth_result,  # 光滑处理详情 (含 tck 样条对象)
    # 光滑轨迹的解析曲率 (1/m), 与 path 逐点对应。
    # 可视化/后处理请直接用它, 不要对 path 再做三点差分。
    'curvature_smooth': curvatures_smooth,
    'speed_profile': speed_profile,
    'accel_profile': accel_smooth,
    'smoothing_version': SMOOTHING_VERSION,
    'final_path_ds': FINAL_PATH_DS,
    'search_corridor_radius': CORRIDOR_RADIUS,
    'smoothing_trust_radius': SMOOTHING_TRUST_RADIUS,
    'search_corridor_enforced_on_spline': ENFORCE_SEARCH_CORRIDOR_ON_SPLINE,
    'grid_res_xy': FINE_GRID_XY,
    'grid_res_z': FINE_GRID_Z,
    'v_cruise': dynamics.cruise_speed,
    'w_risk': W_RISK,
    'w_time': W_TIME,
    'w_fuel': W_FUEL,
    # 直线基准（与 coarse pkl 同一套值，供绕行率等相对指标使用）
    'L0': L0_ref,
    'C0': C0_ref,
    'T0': T0_ref,
    'start': start,
    'goal': goal,
    'baseline': baseline,
}
with open("output_5zones/planned_path.pkl", 'wb') as f:
    pickle.dump(fine_data, f)
print("  ✓ 细网格路径已保存: output_5zones/planned_path.pkl (光滑后 + 原始细网格)")

# ============================================================================
# Step 5: 生成粗网格可视化（用于对比）
# ============================================================================
print("\n[Step 5/7] 生成粗网格可视化（用于对比）...")

print("\n  粗网格 - 4张独立图片...")
import subprocess

# main.py controls its own coarse/fine input and output directories.
plot_env = os.environ.copy()
for key in ('TRAJECTORY_FILE', 'ANALYSIS_OUTDIR'):
    plot_env.pop(key, None)

# 临时切换到粗网格数据
shutil.copy("output_5zones/planned_path_coarse.pkl", "output_5zones/planned_path_temp.pkl")
shutil.copy("output_5zones/planned_path_coarse.pkl", "output_5zones/planned_path.pkl")

result = subprocess.run([sys.executable, 'visualize_astar_separate.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    # 重命名为粗网格版本
    if os.path.exists("output_5zones/astar_3d_trajectory.png"):
        shutil.move("output_5zones/astar_3d_trajectory.png", "output_5zones/astar_3d_trajectory_coarse.png")
    if os.path.exists("output_5zones/astar_altitude_profile.png"):
        shutil.move("output_5zones/astar_altitude_profile.png", "output_5zones/astar_altitude_profile_coarse.png")
    if os.path.exists("output_5zones/astar_topview.png"):
        shutil.move("output_5zones/astar_topview.png", "output_5zones/astar_topview_coarse.png")
    if os.path.exists("output_5zones/astar_altitude_changes.png"):
        shutil.move("output_5zones/astar_altitude_changes.png", "output_5zones/astar_altitude_changes_coarse.png")
    print("    ✓ 粗网格 - 4张独立图片生成完成")
else:
    print("    ✗ 粗网格 - 生成失败")

print("\n  粗网格 - 多角度3D图...")
result = subprocess.run([sys.executable, 'visualize_astar_multi_angle.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    if os.path.exists("output_5zones/astar_3d_multi_view.png"):
        shutil.move("output_5zones/astar_3d_multi_view.png", "output_5zones/astar_3d_multi_view_coarse.png")
    print("    ✓ 粗网格 - 多角度3D图生成完成")
else:
    print("    ✗ 粗网格 - 生成失败")

print("\n  粗网格 - 速度剖面将在恢复细网格数据后做三组统一对比...")

print("\n  粗网格 - 雷达交互分析图...")
result = subprocess.run([sys.executable, 'analyze_radar_interaction.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    if os.path.exists("output_5zones/radar_interaction_analysis_sci.png"):
        shutil.move("output_5zones/radar_interaction_analysis_sci.png",
                   "output_5zones/radar_interaction_analysis_coarse.png")
    print("    ✓ 粗网格 - 雷达交互分析图生成完成")
else:
    print("    ✗ 粗网格 - 生成失败")

# ============================================================================
# Step 6: 生成细网格可视化（最终结果）
# ============================================================================
print("\n[Step 6/7] 生成细网格可视化（最终结果，光滑后路径）...")

# 恢复细网格数据
shutil.copy("output_5zones/planned_path_temp.pkl", "output_5zones/planned_path.pkl")
with open("output_5zones/planned_path.pkl", 'wb') as f:
    pickle.dump(fine_data, f)

print("\n  细网格 - 4张独立图片...")
result = subprocess.run([sys.executable, 'visualize_astar_separate.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    print("    ✓ 细网格 - 4张独立图片生成完成")
else:
    print("    ✗ 细网格 - 生成失败")

print("\n  细网格 - 多角度3D图...")
result = subprocess.run([sys.executable, 'visualize_astar_multi_angle.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    print("    ✓ 细网格 - 多角度3D图生成完成")
else:
    print("    ✗ 细网格 - 生成失败")

print("\n  三组速度-加速度对比图（粗网格 / 细网格原始 / B样条最终）...")
result = subprocess.run([sys.executable, 'visualize_speed_profile_english.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    print("    ✓ 三组速度-加速度对比图生成完成")
else:
    print("    ✗ 三组速度-加速度对比图生成失败")
    if result.stderr:
        print(result.stderr[-1500:])

print("\n  细网格 - 雷达交互分析图...")
result = subprocess.run([sys.executable, 'analyze_radar_interaction.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    print("    ✓ 细网格 - 雷达交互分析图生成完成")
else:
    print("    ✗ 细网格 - 生成失败")

print("\n  细网格 - 转弯半径分析图...")
result = subprocess.run([sys.executable, 'visualize_turn_radius.py'],
                       capture_output=True, text=True, env=plot_env)
if result.returncode == 0:
    print("    ✓ 细网格 - 转弯半径分析图生成完成")
else:
    print("    ✗ 细网格 - 生成失败")

# 清理临时文件
if os.path.exists("output_5zones/planned_path_temp.pkl"):
    os.remove("output_5zones/planned_path_temp.pkl")

# ============================================================================
# Step 7: 统计报告
# ============================================================================
print("\n" + "=" * 80)
print("  路径规划统计报告（两阶段 + B样条光滑）")
print("=" * 80)

print(f"\n【规划参数】")
print(f"  通路半径: {CORRIDOR_RADIUS}m")
print(f"  粗网格分辨率: {COARSE_GRID_XY}m × {COARSE_GRID_Z}m")
print(f"  细网格分辨率: {FINE_GRID_XY}m × {FINE_GRID_Z}m")
print(f"  网格细化倍数: {(COARSE_GRID_XY/FINE_GRID_XY) * (COARSE_GRID_Z/FINE_GRID_Z):.0f}×")

print(f"\n【粗网格结果】（初始路径）")
print(f"  路径点数: {len(coarse_path)}")
print(f"  实际航程: {coarse_stats['L']/1000:.2f} km")
print(f"  飞行时间: {coarse_stats['T']:.0f} s ({coarse_stats['T']/60:.1f} min)")
print(f"  计算耗时: {stats.get('coarse_time', 0):.2f} s")
print(f"  迭代次数: {stats.get('coarse_iterations', 0):,}")

print(f"\n【细网格结果】（最终路径，光滑后）")
print(f"  路径点数: {len(path)} (原始细网格: {len(path_raw)})")
print(f"  实际航程: {stats['L']/1000:.2f} km (原始: {stats_raw['L']/1000:.2f} km)")
print(f"  飞行时间: {stats['T']:.0f} s ({stats['T']/60:.1f} min) (原始: {stats_raw['T']:.0f} s)")
print(f"  累积风险: {stats['Crisk']:.2f} (原始: {stats_raw['Crisk']:.2f})")
print(f"  燃油消耗: {stats.get('F', 0.0):.2f} kg (原始: {stats_raw.get('F', 0.0):.2f} kg)")
print(f"  归一化 T_n/C_n/F_n: {stats['T_n']:.4f} / {stats['C_n']:.4f} / {stats['F_n']:.4f}")
print(f"  统一多目标 J: {stats['J']:.6f}")
print(f"  最小转弯半径: {stats['min_turn_radius']:.1f} m")
print(f"  最大曲率: {stats['max_curvature']:.6e} 1/m")
print(f"  最大爬升角: {stats['max_climb_angle_deg']:.3f}°")
print(f"  最大速度变化率 |dv/dt|: {stats['max_speed_change_rate']:.3f} m/s²")
print(f"  平滑评价 J_s: {stats['smoothing_score']:.6f}")
print(f"  曲率变化积分 J_kappa': {stats['curvature_variation_integral']:.6e} 1/m³")
print(f"  末端航向误差: {stats['terminal_heading_error_deg']:.3f}°")
print(f"  计算耗时: {stats.get('fine_time', 0):.2f} s")
print(f"  光滑耗时: {stats.get('smooth_time', 0):.2f} s")
print(f"  迭代次数: {stats.get('fine_iterations', 0):,}")

print(f"\n【对比分析】")
print(f"  航程变化: {coarse_stats['L']/1000:.2f} → {stats['L']/1000:.2f} km "
      f"({(stats['L']/coarse_stats['L']-1)*100:+.1f}%)")
print(f"  时间变化: {coarse_stats['T']:.0f} → {stats['T']:.0f} s "
      f"({(stats['T']/coarse_stats['T']-1)*100:+.1f}%)")
print(f"  总计算时间: {planning_time:.2f} s")
print(f"  路径点密度提升: {len(path)/len(coarse_path):.1f}×")

if smooth_result['smoothed']:
    print(f"\n【光滑效果】")
    print(f"  光滑成功: ✓")
    print(f"  s参数: {smooth_result['s_param_used']:.1f}")
    print(f"  候选评价数: {smooth_result.get('total_candidate_evaluations', smooth_result['attempts'])}（含航向修复重试）")
    print(f"  选中候选: #{smooth_result.get('selected_candidate_index', -1)}, J_s={smooth_result.get('selection_score', float('nan')):.6f}")
    print(f"  航程变化: {stats_raw['L']/1000:.2f} → {stats['L']/1000:.2f} km "
          f"({((stats['L']/stats_raw['L'])-1)*100:+.2f}%)")
    print(f"  时间变化: {stats_raw['T']:.0f} → {stats['T']:.0f} s "
          f"({((stats['T']/stats_raw['T'])-1)*100:+.2f}%)")
else:
    print(f"\n【光滑效果】")
    print(f"  光滑失败: 返回原始细网格路径")
    print(f"  原因: {smooth_result['message']}")

# 分析高度变化（细网格）
print(f"\n【高度分析】（基于细网格路径）")
path_array = np.array(path)
altitudes = path_array[:, 2] / 1000  # km
print(f"  起始高度: {altitudes[0]:.2f} km")
print(f"  最大高度: {altitudes.max():.2f} km")
print(f"  终止高度: {altitudes[-1]:.2f} km")

# 计算总爬升和下降
altitude_changes = np.diff(altitudes)
total_climb = altitude_changes[altitude_changes > 0].sum()
total_descent = -altitude_changes[altitude_changes < 0].sum()
print(f"  总爬升: {total_climb:.2f} km")
print(f"  总下降: {total_descent:.2f} km")

# 爬升角分析
max_climb_angle = 0
for i in range(len(path) - 1):
    p1, p2 = path[i], path[i+1]
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    horiz_dist = np.sqrt(dx**2 + dy**2)
    if horiz_dist > 1e-6:
        climb_angle = np.degrees(np.arctan2(abs(dz), horiz_dist))
        max_climb_angle = max(max_climb_angle, climb_angle)

print(f"  最大爬升角: {max_climb_angle:.2f}° (限制: {dynamics.max_climb_angle_deg}°)")

print(f"\n【输出文件】")
print(f"\n  粗网格结果（用于对比）:")
print(f"    - output_5zones/planned_path_coarse.pkl")
print(f"    - output_5zones/astar_3d_trajectory_coarse.png")
print(f"    - output_5zones/astar_altitude_profile_coarse.png")
print(f"    - output_5zones/astar_topview_coarse.png")
print(f"    - output_5zones/astar_altitude_changes_coarse.png")
print(f"    - output_5zones/astar_3d_multi_view_coarse.png")
print(f"    - output_5zones/radar_interaction_analysis_coarse.png")

print(f"\n  细网格结果（最终，光滑后路径）:")
print(f"    - output_5zones/planned_path.pkl (含光滑路径 + 原始细网格)")
print(f"    - output_5zones/baseline.pkl (三单目标锚点 payoff table)")
print(f"    - output_5zones/astar_3d_trajectory.png")
print(f"    - output_5zones/astar_altitude_profile.png")
print(f"    - output_5zones/astar_topview.png")
print(f"    - output_5zones/astar_altitude_changes.png")
print(f"    - output_5zones/astar_3d_multi_view.png")
print(f"    - output_5zones/speed_profile_three_way_english.png")
print(f"    - output_5zones/radar_interaction_analysis_sci.png")
print(f"    - output_5zones/turn_radius_analysis.png")

print("\n" + "=" * 80)
print("  ✓ 所有任务完成")
print("=" * 80)
print(f"\n说明：")
print(f"  - 粗网格结果带 '_coarse' 后缀，用于对比分析")
print(f"  - 细网格结果是B样条光滑后的最终路径，运动学约束和速度剖面更准确")
print(f"  - pkl 文件同时保存光滑路径和原始细网格路径，方便对比")
print(f"  - 可视化图片全部基于光滑后路径生成")
