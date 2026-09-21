# -*- coding: utf-8 -*-
"""
5雷达动态布局威胁环境模块 (100km x 100km 场景)
==============================================
针对固定翼无人机突防场景，建立三维半球形雷达区建模。

场景配置：
  - 地图: 100km x 100km
  - 起点: S(5, 5)
  - 终点: T(95, 95)
  - 威胁模型: 半球形区域，统一半径 R = 15 km
  - 5个雷达: C1～C5，分三组战术布局

三组战术布局：
  1. 第一组 (C1, C2): 起点封锁区 - 剪刀差结构
     - 分布范围: 以(20,20)为中心，x,y ∈ [15, 30]
     - 几何约束: 15 < d(C1,C2) < 20
     - 目的: 制造起飞后首个博弈点

  2. 第二组 (C3, C4): 中场拦截区 - 独立分布
     - 分布范围: x ∈ [35, 75], y ∈ [25, 75]
     - 独立性约束: 相互间距 ≥ 40 (保证不相交且留出通道)
     - 目的: 防止退化为简单直线

  3. 第三组 (C5): 终点近卫区 - 安全距离限制
     - 分布范围: x,y ∈ [75, 90]
     - 硬性距离约束: d(C5,T) ≥ 20
     - 目的: 末端高威胁但不覆盖终点
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

KM = 1000.0  # 1 km = 1000 m


@dataclass
class HemisphereRadarZone:
    """
    半球形雷达区 (三维建模)

    威胁模型为以(x, y, 0)为球心，R为半径的半球体（z ≥ 0）。
    在原有三层结构基础上，现在统一使用 R = 15 km。

    后续可在此半球内部建立火力区等更复杂的子结构。
    """
    x: float          # 雷达站 x (m)
    y: float          # 雷达站 y (m)
    R: float          # 半球半径 (m)，统一为 15 km
    name: str = 'Radar'

    # 内部可选的分层参数（为后续火力区预留）
    # 当前阶段：整个半球统一作为探测/威胁区
    rcs_factor: float = 1.0

    def is_point_inside(self, x: float, y: float, z: float) -> bool:
        """判断点(x,y,z)是否在半球内部"""
        if z < 0:
            return False
        dx, dy = x - self.x, y - self.y
        dist_sq = dx*dx + dy*dy + z*z
        return dist_sq <= self.R * self.R

    def get_distance_to_surface(self, x: float, y: float, z: float) -> float:
        """
        计算点到半球表面的距离（负值表示在内部）
        注意：z<0时计算到地面圆边界的距离
        """
        dx, dy = x - self.x, y - self.y
        if z >= 0:
            dist_3d = np.sqrt(dx*dx + dy*dy + z*z)
            return dist_3d - self.R
        else:
            # z<0时，计算到地面圆周的距离
            dist_2d = np.sqrt(dx*dx + dy*dy)
            return dist_2d - self.R


@dataclass
class FireZone:
    """
    火力打击区 (类削顶半球体)

    几何模型：
      - 底面圆心: (x, y, 0)，与对应雷达圆心严格相同
      - 水平半径: 6 km
      - 垂直射高: 5 km
      - 形状: 类似削顶的半球体（圆柱体）

    约束类型: 硬约束禁飞区（进入即判定为失败）
    """
    x: float              # 圆心 x (m)，与雷达圆心相同
    y: float              # 圆心 y (m)，与雷达圆心相同
    horizontal_radius: float = 6.0 * 1000.0  # 水平半径 6 km
    max_altitude: float = 5.0 * 1000.0       # 最大射高 5 km
    name: str = 'FireZone'

    def is_point_inside(self, x: float, y: float, z: float) -> bool:
        """
        判断点(x,y,z)是否在火力打击区内

        几何条件：
          1. 高度: 0 ≤ z ≤ max_altitude
          2. 水平距离: sqrt((x-cx)^2 + (y-cy)^2) ≤ horizontal_radius
        """
        if z < 0 or z > self.max_altitude:
            return False

        dx = x - self.x
        dy = y - self.y
        horizontal_dist = np.sqrt(dx*dx + dy*dy)

        return horizontal_dist <= self.horizontal_radius

    def get_distance_to_boundary(self, x: float, y: float, z: float) -> float:
        """
        计算点到火力区边界的最小距离（负值表示在内部）
        """
        dx = x - self.x
        dy = y - self.y
        horizontal_dist = np.sqrt(dx*dx + dy*dy)

        # 到侧面圆柱的距离
        dist_to_cylinder = horizontal_dist - self.horizontal_radius

        # 如果在圆柱内
        if horizontal_dist <= self.horizontal_radius:
            if 0 <= z <= self.max_altitude:
                # 在火力区内部，返回到最近边界的负距离
                return -min(
                    self.horizontal_radius - horizontal_dist,  # 到侧面
                    z,                                          # 到底面
                    self.max_altitude - z                       # 到顶面
                )
            elif z > self.max_altitude:
                # 在圆柱上方
                return z - self.max_altitude
            else:
                # 在圆柱下方
                return -z
        else:
            # 在圆柱外
            if 0 <= z <= self.max_altitude:
                return dist_to_cylinder
            else:
                # 计算到边缘的欧氏距离
                dz = max(0, z - self.max_altitude, -z)
                return np.sqrt(dist_to_cylinder**2 + dz**2)


@dataclass
class RadarEnvironment5Zones:
    """
    5雷达动态布局威胁环境 (100km x 100km 场景)

    参数：
      - map_size: 地图尺寸 (m), 默认 100 km
      - radar_radius: 统一雷达半径 (m), 默认 15 km
      - min_flight_alt: 航路中段最低高度（端点过渡由 altitude_constraints 管理）
      - max_flight_alt: 航路最高高度
    """
    map_size: float = 100.0 * KM         # 100 km
    radar_radius: float = 15.0 * KM      # 统一半径 15 km
    min_flight_alt: float = 1000.0       # 最低飞行高度 (m)
    max_flight_alt: float = 10000.0      # 航路最高飞行高度 (m)
    cruise_alt: float = 3000.0           # 默认巡航高度 (m)

    radar_zones: List[HemisphereRadarZone] = field(default_factory=list)
    fire_zones: List[FireZone] = field(default_factory=list)  # 火力打击区列表
    start: np.ndarray = None
    goal: np.ndarray = None

    def generate(self, seed: int = 42, max_attempts: int = 500):
        """
        按照战术拓扑约束生成5个雷达区。

        每次运行使用seed控制随机性，保证可复现。
        每组雷达使用独立的seed，确保修改一组约束不影响其他组。
        max_attempts: 最大尝试次数，防止约束过严无解。
        """
        m = self.map_size
        R = self.radar_radius

        # 固定起点和终点
        self.start = np.array([5.0 * KM, 5.0 * KM, self.min_flight_alt])
        self.goal = np.array([95.0 * KM, 95.0 * KM, self.cruise_alt])

        # 为每组雷达创建独立的随机数生成器
        rng_group1 = np.random.RandomState(seed)      # 第一组: C1, C2
        rng_group2 = np.random.RandomState(seed + 1)  # 第二组: C3, C4
        rng_group3 = np.random.RandomState(seed + 2)  # 第三组: C5

        for attempt in range(max_attempts):
            try:
                radar_list = []

                # ============================================================
                # 第一组: C1, C2 - 起点封锁区（剪刀差结构）
                # ============================================================
                # 分布范围: 圆心位于 x,y ∈ [15, 30]
                # 几何约束: 17 < d(C1, C2) < 20  # 临时测试：从15改为17

                # 生成 C1
                c1_x = rng_group1.uniform(15.0 * KM, 30.0 * KM)
                c1_y = rng_group1.uniform(15.0 * KM, 30.0 * KM)

                # 生成 C2，确保满足距离约束
                for _ in range(100):
                    c2_x = rng_group1.uniform(15.0 * KM, 30.0 * KM)
                    c2_y = rng_group1.uniform(15.0 * KM, 30.0 * KM)
                    dist_c1c2 = np.sqrt((c1_x - c2_x)**2 + (c1_y - c2_y)**2)
                    if 17.0 * KM < dist_c1c2 < 20.0 * KM:  # 临时测试
                        break
                else:
                    continue  # 未找到合适的C2，重新尝试

                # 边界检查: 确保圆不超出地图左下边界
                if c1_x - R < 0 or c1_y - R < 0:
                    continue
                if c2_x - R < 0 or c2_y - R < 0:
                    continue

                radar_list.append(HemisphereRadarZone(c1_x, c1_y, R, 'C1'))
                radar_list.append(HemisphereRadarZone(c2_x, c2_y, R, 'C2'))

                # ============================================================
                # 第二组: C3, C4 - 中场拦截区（独立分布）
                # ============================================================
                # 分布范围: x ∈ [35, 75], y ∈ [25, 75]
                # 独立性约束: C3、C4之间 ≥ 40，与C1、C2之间 ≥ 25

                # 生成 C3
                for _ in range(100):
                    c3_x = rng_group2.uniform(35.0 * KM, 75.0 * KM)
                    c3_y = rng_group2.uniform(25.0 * KM, 75.0 * KM)

                    # 检查与 C1, C2 的距离
                    dist_c1c3 = np.sqrt((c1_x - c3_x)**2 + (c1_y - c3_y)**2)
                    dist_c2c3 = np.sqrt((c2_x - c3_x)**2 + (c2_y - c3_y)**2)

                    if dist_c1c3 >= 25.0 * KM and dist_c2c3 >= 25.0 * KM:
                        break
                else:
                    continue

                # 生成 C4
                for _ in range(100):
                    c4_x = rng_group2.uniform(35.0 * KM, 75.0 * KM)
                    c4_y = rng_group2.uniform(25.0 * KM, 75.0 * KM)

                    # 检查与 C1, C2, C3 的距离
                    dist_c1c4 = np.sqrt((c1_x - c4_x)**2 + (c1_y - c4_y)**2)
                    dist_c2c4 = np.sqrt((c2_x - c4_x)**2 + (c2_y - c4_y)**2)
                    dist_c3c4 = np.sqrt((c3_x - c4_x)**2 + (c3_y - c4_y)**2)

                    # C3和C4之间至少40km（保证不相交且留出通道）
                    if (dist_c1c4 >= 25.0 * KM and dist_c2c4 >= 25.0 * KM and
                        dist_c3c4 >= 40.0 * KM):
                        break
                else:
                    continue

                radar_list.append(HemisphereRadarZone(c3_x, c3_y, R, 'C3'))
                radar_list.append(HemisphereRadarZone(c4_x, c4_y, R, 'C4'))

                # ============================================================
                # 第三组: C5 - 终点近卫区（安全距离限制）
                # ============================================================
                # 分布范围: x,y ∈ [75, 90]
                # 硬性距离约束: d(C5, T) ≥ 20 km

                for _ in range(100):
                    c5_x = rng_group3.uniform(75.0 * KM, 90.0 * KM)
                    c5_y = rng_group3.uniform(75.0 * KM, 90.0 * KM)

                    # 计算到终点T(95, 95)的距离
                    dist_c5t = np.sqrt((c5_x - 95.0*KM)**2 + (c5_y - 95.0*KM)**2)

                    if dist_c5t >= 20.0 * KM:
                        break
                else:
                    continue

                radar_list.append(HemisphereRadarZone(c5_x, c5_y, R, 'C5'))

                # ============================================================
                # 所有约束满足，生成成功
                # ============================================================
                self.radar_zones = radar_list

                # 为每个雷达区生成对应的火力打击区
                self.fire_zones = []
                for rz in self.radar_zones:
                    fire_zone = FireZone(
                        x=rz.x,  # 与雷达圆心相同
                        y=rz.y,  # 与雷达圆心相同
                        name=f'Fire_{rz.name}'
                    )
                    self.fire_zones.append(fire_zone)

                # 可行性检查：起终点不在任何雷达半球内和火力区内
                if self._is_feasible():
                    print(f"✓ 雷达区和火力区生成成功 (尝试 {attempt + 1} 次)")
                    return self

            except Exception as e:
                continue

        raise RuntimeError(f"经过 {max_attempts} 次尝试，未能生成满足所有约束的雷达布局")

    def _is_feasible(self) -> bool:
        """
        可行性检查:
          1. 起点不在任何雷达半球内
          2. 终点不在任何雷达半球内
          3. 起点不在任何火力区内
          4. 终点不在任何火力区内
        """
        sx, sy, sz = self.start
        gx, gy, gz = self.goal

        for rz in self.radar_zones:
            if rz.is_point_inside(sx, sy, sz):
                return False
            if rz.is_point_inside(gx, gy, gz):
                return False

        for fz in self.fire_zones:
            if fz.is_point_inside(sx, sy, sz):
                return False
            if fz.is_point_inside(gx, gy, gz):
                return False

        return True

    # ------------------------------------------------------------------
    #  查询接口
    # ------------------------------------------------------------------

    def is_collision_free(self, x: float, y: float, z: float,
                          safety_margin: float = 0.0) -> bool:
        """
        碰撞检测: 点是否在任何雷达半球和火力区外（考虑安全裕度）

        safety_margin: 额外的安全距离 (m)
        """
        # 地图边界检查
        if x < 0 or x > self.map_size or y < 0 or y > self.map_size:
            return False

        # 高度边界检查
        if z < self.min_flight_alt or z > self.max_flight_alt:
            return False

        # 火力区碰撞检查（硬约束，优先级最高）
        for fz in self.fire_zones:
            if fz.is_point_inside(x, y, z):
                return False

        # 雷达半球碰撞检查
        for rz in self.radar_zones:
            dist = rz.get_distance_to_surface(x, y, z)
            if dist <= safety_margin:
                return False

        return True

    def is_segment_collision_free(self, p1: np.ndarray, p2: np.ndarray,
                                   n_check: int = 20,
                                   safety_margin: float = 0.0) -> bool:
        """线段碰撞检测"""
        for t in np.linspace(0, 1, n_check):
            pt = p1 + t * (p2 - p1)
            if not self.is_collision_free(pt[0], pt[1], pt[2], safety_margin):
                return False
        return True

    def get_threat_penalty(self, x: float, y: float, z: float) -> float:
        """
        三维威胁惩罚函数（连续）

        策略：根据点到雷达半球表面的距离计算惩罚
          - 在半球内: INF (硬约束)
          - 半球外: 根据距离衰减的连续惩罚

        衰减模型: penalty = A * exp(-alpha * d / R)
          其中 d 为到半球表面的距离
        """
        total_penalty = 0.0

        for rz in self.radar_zones:
            dist = rz.get_distance_to_surface(x, y, z)

            if dist <= 0:
                # 在半球内部，硬约束
                return float('inf')
            else:
                # 在半球外部，连续衰减惩罚
                # 影响范围约为 2R（超过2R后惩罚可忽略）
                if dist < 2.0 * rz.R:
                    alpha = 2.0  # 衰减系数
                    A = 100.0    # 基础惩罚幅度
                    penalty = A * np.exp(-alpha * dist / rz.R) * rz.rcs_factor
                    total_penalty += penalty

        return total_penalty

    def get_min_threat_distance(self, x: float, y: float, z: float) -> float:
        """
        返回点到最近雷达半球表面的距离 (m)
        负值表示在内部
        """
        min_dist = float('inf')
        for rz in self.radar_zones:
            dist = rz.get_distance_to_surface(x, y, z)
            min_dist = min(min_dist, dist)
        return min_dist

    def get_voronoi_sites(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        返回 Voronoi 骨架的站点和权重
        站点 = 所有雷达圆心 (2D)
        权重 = 雷达半径 R
        """
        centers = []
        weights = []
        for rz in self.radar_zones:
            centers.append([rz.x, rz.y])
            weights.append(rz.R)
        return np.array(centers), np.array(weights)

    # ------------------------------------------------------------------
    #  打印信息
    # ------------------------------------------------------------------

    def print_summary(self):
        """打印环境摘要"""
        m = self.map_size / KM
        print(f"\n{'='*70}")
        print(f"  5雷达动态布局威胁环境 (半球形建模)")
        print(f"{'='*70}")
        print(f"  地图尺寸: {m:.0f} km × {m:.0f} km")
        print(f"  雷达统一半径: {self.radar_radius/KM:.0f} km (半球形)")
        print(f"  起点 S: ({self.start[0]/KM:.1f}, {self.start[1]/KM:.1f}, {self.start[2]:.0f}m)")
        print(f"  终点 T: ({self.goal[0]/KM:.1f}, {self.goal[1]/KM:.1f}, {self.goal[2]:.0f}m)")
        print(f"  直线距离: {np.linalg.norm(self.goal[:2]-self.start[:2])/KM:.1f} km")
        print(f"\n  雷达布局 (5个):")

        # 第一组
        c1, c2 = self.radar_zones[0], self.radar_zones[1]
        d12 = np.sqrt((c1.x-c2.x)**2 + (c1.y-c2.y)**2) / KM
        print(f"\n  【第一组: 起点封锁区 - 剪刀差结构】")
        print(f"    {c1.name}: 圆心({c1.x/KM:.1f}, {c1.y/KM:.1f}) km, R={c1.R/KM:.0f} km")
        print(f"    {c2.name}: 圆心({c2.x/KM:.1f}, {c2.y/KM:.1f}) km, R={c2.R/KM:.0f} km")
        print(f"    圆心距 d(C1,C2) = {d12:.1f} km (约束: 15 < d < 20)")

        # 第二组
        c3, c4 = self.radar_zones[2], self.radar_zones[3]
        d34 = np.sqrt((c3.x-c4.x)**2 + (c3.y-c4.y)**2) / KM
        d13 = np.sqrt((c1.x-c3.x)**2 + (c1.y-c3.y)**2) / KM
        d23 = np.sqrt((c2.x-c3.x)**2 + (c2.y-c3.y)**2) / KM
        d14 = np.sqrt((c1.x-c4.x)**2 + (c1.y-c4.y)**2) / KM
        d24 = np.sqrt((c2.x-c4.x)**2 + (c2.y-c4.y)**2) / KM
        print(f"\n  【第二组: 中场拦截区 - 独立分布】")
        print(f"    {c3.name}: 圆心({c3.x/KM:.1f}, {c3.y/KM:.1f}) km, R={c3.R/KM:.0f} km")
        print(f"    {c4.name}: 圆心({c4.x/KM:.1f}, {c4.y/KM:.1f}) km, R={c4.R/KM:.0f} km")
        print(f"    圆心距 d(C3,C4) = {d34:.1f} km (约束: ≥ 40, 保证不相交)")
        print(f"    独立性检查: d(C1,C3)={d13:.1f}, d(C2,C3)={d23:.1f}, "
              f"d(C1,C4)={d14:.1f}, d(C2,C4)={d24:.1f} km")

        # 第三组
        c5 = self.radar_zones[4]
        d5t = np.sqrt((c5.x - self.goal[0])**2 + (c5.y - self.goal[1])**2) / KM
        print(f"\n  【第三组: 终点近卫区 - 安全距离限制】")
        print(f"    {c5.name}: 圆心({c5.x/KM:.1f}, {c5.y/KM:.1f}) km, R={c5.R/KM:.0f} km")
        print(f"    距终点 d(C5,T) = {d5t:.1f} km (约束: ≥ 20)")

        # 火力打击区
        print(f"\n  火力打击区 ({len(self.fire_zones)}个，与雷达圆心重合):")
        for fz in self.fire_zones:
            print(f"    {fz.name}: 圆心({fz.x/KM:.1f}, {fz.y/KM:.1f}) km, "
                  f"水平半径={fz.horizontal_radius/KM:.0f} km, "
                  f"射高={fz.max_altitude/KM:.1f} km")

        print(f"{'='*70}\n")


# ======================================================================
#  测试代码
# ======================================================================

if __name__ == '__main__':
    print("测试: 5雷达动态布局威胁环境生成\n")

    # 创建环境
    env = RadarEnvironment5Zones()

    # 生成雷达布局
    env.generate(seed=42)

    # 打印摘要
    env.print_summary()

    # 测试碰撞检测
    print("\n碰撞检测测试:")
    test_points = [
        (5*KM, 5*KM, 1000.0, "起点"),
        (95*KM, 95*KM, 3000.0, "终点"),
        (20*KM, 20*KM, 2000.0, "第一组雷达区域"),
        (50*KM, 50*KM, 3000.0, "中场区域"),
    ]

    for x, y, z, label in test_points:
        is_free = env.is_collision_free(x, y, z)
        min_dist = env.get_min_threat_distance(x, y, z)
        penalty = env.get_threat_penalty(x, y, z)
        status = "✓ 安全" if is_free else "✗ 碰撞"
        print(f"  {label:12s}: {status:6s}  最小距离={min_dist/KM:6.2f}km  "
              f"威胁惩罚={penalty:8.2f}")

    print("\n✓ 测试完成")
