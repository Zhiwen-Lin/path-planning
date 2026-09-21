# -*- coding: utf-8 -*-
"""
三维环境威胁模型 (Environment Threat Model)
============================================
为固定翼无人机路径规划提供代价函数输入。

系统最终多目标函数由 astar_risk_planner.py 统一为:
    J = w_T·T_n + w_R·C_n + w_F·F_n
本模块专门提供其中的 C_risk（累计雷达暴露），定义为 ∫Pd dt。

学术术语约定 (Terminology):
  - 被探测程度       : Detection_Probability (Pd) / Exposure_Risk, 取值 [0, 1]
  - 多雷达融合策略   : 最大暴露风险准则 (Max Exposure Risk Criterion)
                       P_total = max(Pd_1, Pd_2, ...), 严禁线性叠加
  - 火力拦截区       : 硬约束 (Hard Constraint), 禁飞区, 代价 = +inf
  - 雷达探测区       : 软约束 (Soft Constraint), 连续风险场, 贡献 C_risk

两类约束的角色:
  - 火力区 (FireZone)          -> check_collision() -> True / math.inf, 直接截断路径
  - 雷达区 (HemisphereRadarZone)-> calculate_risk() -> 连续 Pd, 参与积分

坐标/单位: 内部全部 SI 单位 (m)。可视化时换算为 km 显示。
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from dataclasses import dataclass
from typing import List, Tuple, Union, Optional, Sequence

from radar_environment_5zones import (
    RadarEnvironment5Zones, HemisphereRadarZone, FireZone
)

KM = 1000.0
Point = Union[Sequence[float], np.ndarray]   # (x, y, z) in meters


# ======================================================================
#  环境威胁模型
# ======================================================================

class EnvironmentThreatModel:
    """
    三维环境威胁模型: 硬约束(火力区) + 软约束(雷达探测风险场)。

    典型用法:
        env = RadarEnvironment5Zones(); env.generate(seed=42)
        threat = EnvironmentThreatModel(env, model='gaussian')

        # 硬约束
        blocked = threat.check_collision((x, y, z))          # -> bool
        pen     = threat.collision_penalty((x, y, z))        # -> 0.0 or math.inf

        # 软约束 (单点瞬时风险, 最大暴露准则)
        pd = threat.calculate_risk((x, y, z))                # -> [0,1]

        # 路径积分累积风险 C_risk = Σ Pd·Δt（单位：exposure-s）
        c_risk = threat.path_integration(path_points)        # -> float
    """

    # ------------------------------------------------------------------
    #  初始化
    # ------------------------------------------------------------------
    def __init__(self, env: RadarEnvironment5Zones,
                 model: str = 'power',
                 edge_pd: float = 0.05,
                 power_n: float = 2.0,
                 R_inner_ratio: float = 0.3,
                 sigmoid_n: float = 4.0):
        """
        Args:
            env      : 已生成的 RadarEnvironment5Zones (含 radar_zones, fire_zones)
            model    : 探测概率模型, 可选:
                       'gaussian' - Pd(d)=exp(-d²/(2σ²))            (默认, 山峰状, 光滑可导)
                       'power'    - Pd(d)={1, d≤Rinner; (d/Rmax)^n, Rinner<d<Rmax; 0, d≥Rmax}  (幂律衰减, 正确公式)
                       'sigmoid'  - Pd(d)=1/(1+(d/R_ref)^n)         (雷达方程启发, 内部≈1)
            edge_pd  : gaussian 模型中, 距离 d=R_max 处的 Pd 目标值 (用于反推 σ)
            power_n  : power 模型的衰减指数 n (推荐2.0, 产生连续梯度)
            R_inner_ratio: power 模型的内半径比例, R_inner = R_inner_ratio * R_max (默认0.3)
            sigmoid_n: sigmoid 模型的衰减指数 n
        """
        self.env = env
        self.model = model.lower()
        self.edge_pd = edge_pd
        self.power_n = power_n
        self.R_inner_ratio = R_inner_ratio
        self.sigmoid_n = sigmoid_n

        if self.model not in ('gaussian', 'power', 'sigmoid'):
            raise ValueError(f"未知探测模型: {model}")

    # ------------------------------------------------------------------
    #  雷达探测半径 (R_max)
    # ------------------------------------------------------------------
    @staticmethod
    def _detection_radius(radar: HemisphereRadarZone) -> float:
        """雷达最大探测半径 R_max = 半球半径 R。"""
        return radar.R

    def _gaussian_sigma(self, R_max: float) -> float:
        """
        由 "在 d=R_max 处 Pd=edge_pd" 反推高斯标准差 σ:
            edge_pd = exp(-R_max²/(2σ²))  =>  σ = R_max / sqrt(-2·ln(edge_pd))
        """
        return R_max / math.sqrt(-2.0 * math.log(self.edge_pd))

    # ==================================================================
    #  2. 火力拦截区 —— 硬约束 (Hard Constraint)
    # ==================================================================
    def check_collision(self, point: Point) -> bool:
        """
        硬约束碰撞检测: 点是否落入任一火力拦截区 (禁飞区)。

        火力区几何: 圆柱体 (拦截走廊), 底面圆心(x,y,0), 水平半径 6km, 射高 5km。
        (FireZone.is_point_inside 已实现该判定; 如需球体射程模型可扩展 FireZone。)

        Args:
            point: (x, y, z) 米
        Returns:
            True  -> 在禁飞区内 (路径应被截断, 代价视为 +inf)
            False -> 安全
        """
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        for fz in self.env.fire_zones:
            if fz.is_point_inside(x, y, z):
                return True
        return False

    def collision_penalty(self, point: Point) -> float:
        """
        硬约束代价: 在禁飞区内返回 math.inf, 否则 0.0。
        用于直接叠加进损失函数 J 以截断路径。
        """
        return math.inf if self.check_collision(point) else 0.0

    def segment_hits_fire_zone(self, p1: Point, p2: Point, n_check: int = 20) -> bool:
        """
        线段级硬约束检测: 采样 p1->p2 判断是否穿越任一火力区。
        用于路径积分时的截断判定。
        """
        p1 = np.asarray(p1, dtype=float)
        p2 = np.asarray(p2, dtype=float)
        for t in np.linspace(0.0, 1.0, n_check):
            if self.check_collision(p1 + t * (p2 - p1)):
                return True
        return False

    # ==================================================================
    #  3. 雷达探测区 —— 软约束 (Soft Constraint)
    # ==================================================================
    def _pd_single(self, point: Point, radar: HemisphereRadarZone) -> float:
        """
        单雷达探测概率 Pd (Exposure Risk), 取值 [0, 1]。

        d = 到雷达地面站 (rx, ry, 0) 的三维斜距 (slant range)。
        d 越小 -> Pd 越大 (距离越近, 被侦察程度越高)。
        超出探测半径 R_max 后 Pd 直接截为 0。
        """
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        dx, dy, dz = x - radar.x, y - radar.y, z - 0.0
        d = math.sqrt(dx * dx + dy * dy + dz * dz)   # slant range
        R_max = self._detection_radius(radar)

        # 探测半径之外无风险 (硬截断软约束的作用范围)
        if d >= R_max:
            return 0.0

        rcs = getattr(radar, 'rcs_factor', 1.0)   # RCS 调制因子 (可按目标散射截面缩放)

        if self.model == 'gaussian':
            # 高斯型: Pd(d) = exp(-d²/(2σ²)), 中心 Pd=1, 山峰状分布
            sigma = self._gaussian_sigma(R_max)
            pd = math.exp(-(d * d) / (2.0 * sigma * sigma))
        elif self.model == 'power':
            # 幂律型（衰减）: Pd随距离增大而减小
            # Pd(d) = {1, d≤Rinner; ((Rmax-d)/(Rmax-Rinner))^n, Rinner<d<Rmax; 0, d≥Rmax}
            R_inner = self.R_inner_ratio * R_max
            if d <= R_inner:
                pd = 1.0
            elif d < R_max:
                # 从内圈到外圈线性插值，再取幂次
                normalized_d = (d - R_inner) / (R_max - R_inner)
                pd = (1.0 - normalized_d) ** self.power_n
            else:
                pd = 0.0
        else:  # 'sigmoid'
            # 雷达方程启发: Pd(d) = 1 / (1 + (d/R_ref)^n), R_ref 取 R_max 的一半
            R_ref = 0.5 * R_max
            pd = 1.0 / (1.0 + (d / R_ref) ** self.sigmoid_n)

        return float(np.clip(pd * rcs, 0.0, 1.0))

    def calculate_risk(self, point: Point,
                       radar_list: Optional[List[HemisphereRadarZone]] = None) -> float:
        """
        多源融合瞬时风险 —— 最大暴露风险准则 (Max Exposure Risk Criterion)。

            P_total = max(Pd_1, Pd_2, ..., Pd_N)

        物理解释: 飞机只关心 "当前谁看得我最清楚", 由暴露最严重的单个雷达决定风险,
        避免多个弱雷达线性叠加导致风险虚高。

        Args:
            point     : (x, y, z) 米
            radar_list: 参与计算的雷达列表, 缺省用 env 全部雷达
        Returns:
            P_total ∈ [0, 1]
        """
        if radar_list is None:
            radar_list = self.env.radar_zones
        if not radar_list:
            return 0.0

        # 最大暴露准则: 取所有单雷达 Pd 的最大值 (严禁线性叠加)
        return max(self._pd_single(point, r) for r in radar_list)

    # ==================================================================
    #  4. 与总损失函数的接口
    # ==================================================================
    # ---- (1) 单点评估 (Point-wise Evaluation) ----
    def evaluate_point(self, point: Point) -> dict:
        """
        单点综合评估。用于实时避障 / 节点扩展代价估计。

        Returns dict:
            'Pd'        : 瞬时暴露风险 (最大暴露准则)      ∈ [0,1]
            'collision' : 是否落入火力区 (硬约束)          bool
            'penalty'   : 硬约束代价 (0 或 math.inf)
            'cost'      : 供 A*/RRT 直接使用的节点代价 (碰撞=inf, 否则=Pd)
        """
        collision = self.check_collision(point)
        pd = self.calculate_risk(point)
        return {
            'Pd': pd,
            'collision': collision,
            'penalty': math.inf if collision else 0.0,
            'cost': math.inf if collision else pd,
        }

    # ---- (2) 路径积分 (Path Integration) ----
    def path_integration(self, path_points: Sequence[Point],
                         truncate_on_collision: bool = True,
                         check_segment: bool = True,
                         integrate_by_time: bool = True,
                         speed: float = 55.0) -> float:
        """
        整条路径的累积被探测程度（暴露积分）。

        【重要】积分方式：
          - integrate_by_time=True（推荐）: C_risk = Σ Pd·Δt（按时间积分）
            物理含义：在辐射强度Pd的区域停留Δt时间，累积暴露 = Pd × Δt
          - integrate_by_time=False（旧版）: C_risk = Σ Pd·Δs（按距离积分）
            恒速假设：Δt = Δs/v，退化为按距离积分

        采用梯形积分: 相邻两点风险取均值, 乘以时间或距离。

        Args:
            path_points          : [(x,y,z), ...] 米
            truncate_on_collision: 路径若穿越火力区, 返回 math.inf (硬约束截断)
            check_segment        : True 则对线段做火力区穿越检测 (更严格)
            integrate_by_time    : True=按时间积分（物理正确），False=按距离（恒速近似）
            speed                : 飞行速度(m/s)，用于按时间积分时计算Δt

        Returns:
            C_risk (float)，单位：暴露秒（integrate_by_time=True）或 暴露米（False）
            或 math.inf (路径不可行)
        """
        pts = [np.asarray(p, dtype=float) for p in path_points]
        if len(pts) < 2:
            # 单点退化: 无路程, 累积风险为 0
            if len(pts) == 1 and truncate_on_collision and self.check_collision(pts[0]):
                return math.inf
            return 0.0

        # 端点硬约束
        if truncate_on_collision and self.check_collision(pts[0]):
            return math.inf

        c_risk = 0.0
        pd_prev = self.calculate_risk(pts[0])
        for i in range(1, len(pts)):
            p_prev, p_cur = pts[i - 1], pts[i]

            # 硬约束: 端点或线段穿越火力区 -> 截断
            if truncate_on_collision:
                if self.check_collision(p_cur):
                    return math.inf
                if check_segment and self.segment_hits_fire_zone(p_prev, p_cur):
                    return math.inf

            pd_cur = self.calculate_risk(p_cur)
            ds = float(np.linalg.norm(p_cur - p_prev))   # Δs 欧氏距离

            # 【修改】根据积分方式选择
            if integrate_by_time:
                dt = ds / speed if speed > 0 else 0.0    # Δt = Δs / v
                c_risk += 0.5 * (pd_prev + pd_cur) * dt  # 梯形积分（按时间）
            else:
                c_risk += 0.5 * (pd_prev + pd_cur) * ds  # 梯形积分（按距离，旧版）

            pd_prev = pd_cur

        return c_risk

    # 语义别名, 与需求文档措辞一致
    calculate_cumulative_risk = path_integration

    def total_loss_risk_term(self, path_points: Sequence[Point],
                             w4: float = 1.0) -> float:
        """
        计算总损失函数中的风险项 w4·C_risk。
        碰撞路径返回 math.inf (使该项主导 J, 表示任务失败)。
        """
        c_risk = self.path_integration(path_points)
        if math.isinf(c_risk):
            return math.inf
        return w4 * c_risk

    # ==================================================================
    #  网格采样 (供可视化 / 快速代价查表)
    # ==================================================================
    def risk_grid(self, z: float, n: int = 300
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        在固定高度 z 处生成风险场网格 (最大暴露准则)。
        Returns: (X, Y, Pd_grid) 均为 (n, n), 坐标单位米。
        """
        m = self.env.map_size
        xs = np.linspace(0, m, n)
        ys = np.linspace(0, m, n)
        X, Y = np.meshgrid(xs, ys)
        Pd = np.zeros_like(X)

        radars = self.env.radar_zones
        for i in range(n):
            for j in range(n):
                Pd[i, j] = self.calculate_risk((X[i, j], Y[i, j], z), radars)
        return X, Y, Pd


# ======================================================================
#  5. 可视化验证: X-Y 平面多高度切片热力图
# ======================================================================

def plot_threat_slices(threat: EnvironmentThreatModel,
                       z_slices_km: Sequence[float] = (3.0, 6.0, 9.0),
                       n: int = 260,
                       save_path: str = 'output_5zones/05_threat_field_slices.png'):
    """
    绘制多高度 X-Y 切片:
      - 热力图 (Heatmap) 表示雷达软约束风险场 Pd, 越红 Pd 越高 (山峰状威胁)
      - 黑色实心圆表示火力拦截区 (硬约束禁飞区)
      - 虚线圆表示雷达最大探测半径边界
    """
    n_slices = len(z_slices_km)
    fig, axes = plt.subplots(1, n_slices, figsize=(6.2 * n_slices, 6.0))
    if n_slices == 1:
        axes = [axes]

    m_km = threat.env.map_size / KM

    for ax, z_km in zip(axes, z_slices_km):
        z = z_km * KM
        X, Y, Pd = threat.risk_grid(z=z, n=n)

        # --- 软约束: 风险场热力图 ---
        im = ax.pcolormesh(X / KM, Y / KM, Pd,
                           cmap='hot_r', shading='auto', vmin=0.0, vmax=1.0)

        # --- 雷达探测半径边界 (虚线) ---
        for r in threat.env.radar_zones:
            # 该高度切片上, 半球在此 z 的水平截面半径 = sqrt(R² - z²)
            if r.R > z:
                r_slice = math.sqrt(r.R ** 2 - z ** 2) / KM
                ax.add_patch(Circle((r.x / KM, r.y / KM), r_slice,
                                    fill=False, edgecolor='#1f77b4',
                                    linewidth=1.2, linestyle='--', alpha=0.7))
            ax.plot(r.x / KM, r.y / KM, '^', color='#1f77b4',
                    markersize=7, markeredgecolor='white', markeredgewidth=0.8)

        # --- 硬约束: 火力区黑色实心圆 (仅在射高范围内绘制) ---
        for fz in threat.env.fire_zones:
            if z <= fz.max_altitude:
                ax.add_patch(Circle((fz.x / KM, fz.y / KM),
                                    fz.horizontal_radius / KM,
                                    facecolor='black', edgecolor='black',
                                    alpha=0.9, zorder=5))

        # --- 起点 / 终点 ---
        if threat.env.start is not None:
            ax.plot(threat.env.start[0] / KM, threat.env.start[1] / KM, 'o',
                    color='#2ca02c', markersize=9, markeredgecolor='white',
                    markeredgewidth=1.2, zorder=6)
        if threat.env.goal is not None:
            ax.plot(threat.env.goal[0] / KM, threat.env.goal[1] / KM, 's',
                    color='#00d4ff', markersize=9, markeredgecolor='white',
                    markeredgewidth=1.2, zorder=6)

        ax.set_xlim(0, m_km)
        ax.set_ylim(0, m_km)
        ax.set_aspect('equal')
        ax.set_xlabel('X (km)', fontsize=11, fontweight='bold')
        ax.set_ylabel('Y (km)', fontsize=11, fontweight='bold')
        ax.set_title(f'Exposure Risk Field  (Z = {z_km:.1f} km)',
                     fontsize=12, fontweight='bold')

        cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Detection Probability  $P_d$', fontsize=10)

    fig.suptitle('3D Environment Threat Model — Max Exposure Risk Criterion\n'
                 '(Heatmap: radar soft-constraint $P_d$   |   Black disks: fire-zone hard-constraint)',
                 fontsize=13, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig(save_path, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"✓ 威胁场切片图已保存: {save_path}")
    plt.close()


# ======================================================================
#  测试脚本
# ======================================================================

def _run_test():
    print("=" * 78)
    print("  三维环境威胁模型测试 — C_risk (被探测累计程度)")
    print("=" * 78)

    # 1) 生成环境
    env = RadarEnvironment5Zones()
    env.generate(seed=42)

    # 2) 建立威胁模型 (高斯型探测概率)
    threat = EnvironmentThreatModel(env, model='gaussian', edge_pd=0.05)

    # 3) 单点评估 (Point-wise Evaluation)
    print("\n[单点评估] Point-wise Evaluation:")
    c1 = env.radar_zones[0]
    test_points = [
        (c1.x, c1.y, 2 * KM, "雷达C1正上方(近)"),
        (c1.x + 8 * KM, c1.y, 2 * KM, "雷达C1外侧(远)"),
        (50 * KM, 50 * KM, 3 * KM, "地图中心(空旷)"),
        (c1.x, c1.y, 0.0, "火力区C1内部"),
    ]
    for x, y, z, label in test_points:
        info = threat.evaluate_point((x, y, z))
        pen = "inf" if math.isinf(info['penalty']) else f"{info['penalty']:.1f}"
        print(f"  {label:16s}: Pd={info['Pd']:.4f} | 碰撞={info['collision']!s:5s} "
              f"| 硬约束代价={pen}")

    # 4) 最大暴露准则演示: 相交区两雷达
    print("\n[最大暴露准则] Max Exposure Risk (取max, 非叠加):")
    c2 = env.radar_zones[1]
    mid = ((c1.x + c2.x) / 2, (c1.y + c2.y) / 2, 1 * KM)
    pd1 = threat._pd_single(mid, c1)
    pd2 = threat._pd_single(mid, c2)
    pd_max = threat.calculate_risk(mid)
    print(f"  C1↔C2 中点: Pd(C1)={pd1:.4f}, Pd(C2)={pd2:.4f} "
          f"-> P_total=max={pd_max:.4f} (线性叠加将虚高为 {pd1+pd2:.4f})")

    # 5) 路径积分 (Path Integration) C_risk = Σ Pd·Δt
    print("\n[路径积分] Cumulative Exposure Risk  C_risk = Σ Pd·Δt (exposure-s):")
    # 路径A: 直穿 C1 附近 (高风险)
    path_A = [(env.start[0], env.start[1], 2 * KM),
              (c1.x, c1.y, 2 * KM),
              (env.goal[0], env.goal[1], 3 * KM)]
    # 路径B: 绕开雷达 (贴地图边缘)
    path_B = [(env.start[0], env.start[1], 2 * KM),
              (5 * KM, 90 * KM, 3 * KM),
              (env.goal[0], env.goal[1], 3 * KM)]
    cA = threat.path_integration(path_A)
    cB = threat.path_integration(path_B)
    fmtA = "inf(穿火力区)" if math.isinf(cA) else f"{cA:.1f}"
    fmtB = "inf(穿火力区)" if math.isinf(cB) else f"{cB:.1f}"
    print(f"  路径A (直穿C1) : C_risk = {fmtA}")
    print(f"  路径B (绕行)   : C_risk = {fmtB}")
    if not math.isinf(cA) and not math.isinf(cB):
        print(f"  -> 绕行路径累积风险降低 {(1 - cB / max(cA,1e-9)) * 100:.1f}%")

    # 6) 可视化
    print("\n[可视化] 生成多高度切片热力图...")
    plot_threat_slices(threat, z_slices_km=(3.0, 6.0, 9.0))

    print("\n✓ 测试完成")
    print("=" * 78)


if __name__ == '__main__':
    _run_test()
