# -*- coding: utf-8 -*-
"""
固定翼无人机飞行动力学模块 (Fixed-Wing UAV Dynamics)
=====================================================
替代原旋翼模型 uav_dynamics.py。参考机型: 翼龙-II 量级中低速战术固定翼无人机。

核心差异 (相对旋翼):
  1. 失速速度约束: 任意路径点速度必须 >= v_stall (不能悬停)
  2. 转弯半径基于滚转角物理模型: R = v^2 / (g * tan(phi)), 量级数百米
  3. 能耗采用升阻比模型: P = (m*g/(L/D)) * v + 0.5*rho*v^3*C_D0*S
  4. 爬升受最大爬升角约束 (gamma_max), 不能垂直爬升
  5. 最大航程硬约束 (max_range)

坐标/单位约定:
  - 内部所有计算使用 SI 单位 (m, s, m/s, J, W)
  - 地图坐标传入时若为 km, 由调用方负责换算
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class FixedWingDynamics:
    """
    固定翼无人机动力学约束参数
    参考机型: 翼龙-II 量级 (MALE 中空长航时无人机)
    """
    # 基本参数
    mass: float = 1100.0             # 起飞质量 (kg)
    g: float = 9.81                  # 重力加速度 (m/s^2)
    wing_area: float = 11.5          # 机翼参考面积 S_ref (m^2)
    air_density: float = 0.9         # 空气密度 (kg/m^3) @ ~3km 巡航高度

    # 速度约束 (m/s)
    v_stall: float = 33.0            # 失速速度 (~120 km/h) — 速度硬下限
    cruise_speed: float = 55.0       # 巡航速度 (~200 km/h)
    max_horizontal_speed: float = 83.0   # 最大平飞速度 (~300 km/h)
    max_ascend_speed: float = 12.0   # 最大上升率 (m/s)
    max_descend_speed: float = 15.0  # 最大下降率 (m/s)

    # 加速度约束 (m/s^2)
    max_horizontal_accel: float = 3.0    # 最大切向加速度
    max_decel: float = 4.0               # 最大减速度
    max_vertical_accel: float = 2.0      # 最大垂直加速度

    # 转弯约束 — 滚转角模型
    max_bank_angle_deg: float = 30.0     # 最大滚转角 (度)
    min_turn_radius_floor: float = 50.0  # 转弯半径硬下限 (m), 结构强度限制

    # 爬升约束
    max_climb_angle_deg: float = 15.0    # 最大爬升角 (度)

    # 能耗模型 (升阻比模型)
    lift_drag_ratio: float = 15.0        # 升阻比 L/D (用于参考/展示)
    cd0: float = 0.025                   # 零升阻力系数 C_D0
    aspect_ratio: float = 18.0           # 展弦比 AR (中空长航时大展弦比)
    oswald_efficiency: float = 0.85      # 奥斯瓦尔德效率因子 e
    climb_efficiency: float = 0.8        # 爬升能量效率
    descent_recovery: float = 0.0        # 下降势能回收比例 (固定翼通常不回收)

    # 航程与续航
    max_range: float = 4000000.0         # 最大航程 (m), 4000 km
    fuel_capacity_j: float = 5.0e9       # 燃油总能量 (J), 约对应数百kg航空燃油
    propulsive_efficiency: float = 0.30  # 推进系统总效率 (燃油->推进功)
    fuel_heat_value_j_per_kg: float = 43.0e6  # 航空燃油低位热值 (J/kg)

    # ------------------------------------------------------------------
    #  转弯半径模型 (滚转角物理模型)
#  ---------------------------------------------------------------
#  [物理原理]
#  水平转弯由升力水平分量提供向心力:
#    L·sin(φ) = m·v²/R  →  R = v²/(g·tan(φ))
#  φ 越大 → R 越小, 但 φ > 30° 固定翼容易失速, 故设限.
#
#  [与旋翼区别]
#  旋翼可悬停转弯 (R→0), 固定翼 R = v²/(g·tan(φ_max)) ≥ 数百米.
#  速度越快、转弯越急(φ大), 但 v 对 R 贡献平方级, 所以转弯必须先减速.
#
#  [在本文中的应用]
#  粗网格 A* 不直接调用转弯半径, 而由 compute_speed_profile 在
#  后处理中根据路径曲率计算"允许的最大转弯速度 v_turn_max":
#    v_turn_max = sqrt(R·g·tan(φ_max)) → 保证了"路径可飞":
#    - 曲率半径小 → v_turn_max 小 → 强制减速 → 不违反滚转角极限
#    - 受 v_stall 保底, 不会低到失速
    def get_turn_radius(self, speed: float, bank_angle_deg: float) -> float:
        """
        给定速度与滚转角的水平转弯半径。
          tan(phi) = v^2 / (R * g)  =>  R = v^2 / (g * tan(phi))
        固定翼 R 量级数百米, 远大于旋翼。
        """
        if abs(bank_angle_deg) < 1e-6:
            return float('inf')
        bank_rad = np.radians(min(abs(bank_angle_deg), 89.0))
        R = speed ** 2 / (self.g * np.tan(bank_rad))
        return max(R, self.min_turn_radius_floor)

    def get_min_turn_radius(self, speed: float) -> float:
        """给定速度下的最小转弯半径 (使用最大滚转角)。"""
        return self.get_turn_radius(speed, self.max_bank_angle_deg)

    def get_bank_angle_for_radius(self, speed: float, radius: float) -> float:
        """反算: 给定速度与期望半径所需滚转角 (度)。"""
        if radius < self.min_turn_radius_floor:
            return self.max_bank_angle_deg
        tan_phi = speed ** 2 / (radius * self.g)
        phi = np.degrees(np.arctan(tan_phi))
        return min(phi, self.max_bank_angle_deg)

    def get_turn_speed_for_radius(self, radius: float) -> float:
        """
        给定曲率半径, 计算允许的最大转弯速度。
          v_max = sqrt(R * g * tan(phi_max))
        固定翼受失速速度下限约束: 结果不得低于 v_stall。
        """
        v_sq = radius * self.g * np.tan(np.radians(self.max_bank_angle_deg))
        if v_sq <= 0:
            return self.v_stall
        v = np.sqrt(v_sq)
        v = min(v, self.max_horizontal_speed)
        return max(v, self.v_stall)

    def get_path_curvature_radius(self, p1: np.ndarray, p2: np.ndarray,
                                  p3: np.ndarray) -> float:
        """
        通过三点的曲率半径 (Menger 曲率倒数)。共线返回 inf。

        Menger 曲率:  k = 4A / (abc)
            A = 三角形面积 = 0.5·|d1 × d2|
            a, b, c = 三边长 = |d1|, |d2|, |d13|
        代入 A 得  k = 2·|d1 × d2| / (|d1|·|d2|·|d13|)

        注意: 系数是 2 而非 4。曾误写为 4·|cross|/(abc)，使返回半径
        恒为真实值的一半（半径 1000 m 的圆上取三点会返回 500 m），
        导致转弯限速被系统性收紧一倍。

        警告: 三点公式在"点距 << 曲率半径"时因浮点相消而失稳，
        不适用于密采样的光滑曲线。对 B 样条请用解析导数：
        trajectory_smoothing.spline_curvature()。
        """
        d1 = p2 - p1
        d2 = p3 - p2
        d13 = p3 - p1
        cross_norm = np.linalg.norm(np.cross(d1, d2))
        len_d1 = np.linalg.norm(d1)
        len_d2 = np.linalg.norm(d2)
        len_d13 = np.linalg.norm(d13)
        if cross_norm < 1e-9 or len_d1 < 1e-6 or len_d2 < 1e-6 or len_d13 < 1e-6:
            return float('inf')
        curvature = 2.0 * cross_norm / (len_d1 * len_d2 * len_d13)
        if curvature < 1e-9:
            return float('inf')
        return 1.0 / curvature

    def get_turn_penalty(self, turn_angle_deg: float, D_ref: float,
                         R_curv: float = None) -> float:
        """
        转弯在代价函数中的归一化惩罚。
          R_curv >= R_min: 可全速通过, 按弧长冗余轻惩罚
          R_curv <  R_min: 必须减速 (受 v_stall 约束, 固定翼可能无解) -> 重惩罚
        """
        if turn_angle_deg < 5.0:
            return 0.0
        R_min = self.get_min_turn_radius(self.cruise_speed)

        if R_curv is not None and R_curv < float('inf') and R_curv > 1e-6:
            if R_curv >= R_min:
                theta_rad = np.radians(turn_angle_deg)
                arc_len = R_curv * theta_rad
                chord_len = 2 * R_curv * np.sin(theta_rad / 2)
                extra_dist = max(0, arc_len - chord_len)
                margin_ratio = R_min / R_curv
                return (0.5 + 2.0 * margin_ratio) * (extra_dist / max(D_ref, 1.0))
            else:
                # 固定翼: 半径过小需减速, 但减速受 v_stall 限制
                v_allowed = self.get_turn_speed_for_radius(R_curv)
                R_phys = self.v_stall ** 2 / (
                    self.g * np.tan(np.radians(self.max_bank_angle_deg)))
                if R_curv < R_phys:
                    # 即使减到失速速度也转不过来 -> 物理不可行
                    return 1000.0
                theta_rad = np.radians(turn_angle_deg)
                arc_len = R_curv * theta_rad
                time_penalty = arc_len * (1.0 / v_allowed - 1.0 / self.cruise_speed)
                T_ref_est = D_ref / self.cruise_speed
                ratio = R_min / R_curv
                return 15.0 * ratio * time_penalty / max(T_ref_est, 0.1)

        w_turn = 3.0
        return w_turn * (R_min / max(D_ref, 1.0)) * (turn_angle_deg / 90.0)

    def get_min_feasible_radius(self) -> float:
        """物理上能转过的最小半径 (滚转角最大 + 减速到失速速度)。"""
        return self.v_stall ** 2 / (self.g * np.tan(np.radians(self.max_bank_angle_deg)))

    # ------------------------------------------------------------------
#  升阻比能耗模型 (论文中未启用, 备用)
#  ---------------------------------------------------------------
#  [公式]
#  固定翼平飞需用功率 = 寄生阻力功率 + 诱导阻力功率:
#    P_para = 0.5·ρ·v³·C_D0·S      [∝ v³, 高速主导 — 飞越快阻力越大]
#    P_ind  = 2·(m·g)² / (ρ·v·S·π·AR·e)  [∝ 1/v, 低速主导 — 飞越慢诱导阻力越大]
#  总功率为 U 型曲线, 最底端 = 最大航程速度.
#
#  [用途说明]
#  A* 与最终轨迹评估均通过 astar_risk_planner 的统一统计器调用
#  get_power_at_state，并用 fuel_heat_value_j_per_kg 转换为燃油质量。

    def get_air_density(self, h: float) -> float:
        """
        国际标准大气密度模型（ISA, International Standard Atmosphere）

        参数：
            h: 飞行高度 (m)，海平面为0

        返回：
            空气密度 (kg/m³)

        公式：
            ρ(h) = ρ0 × exp(-h / H)
            其中 ρ0 = 1.225 kg/m³（海平面密度）
                 H = 8500 m（对流层标高）
        """
        rho_0 = 1.225  # 海平面密度 (kg/m³)
        H = 8500.0     # 标高 (m)
        return rho_0 * np.exp(-h / H)

    def get_power_at_state(self, v_horizontal: float, v_vertical: float, h: float = 0.0) -> float:
        """
        给定飞行状态的燃油功率 (W)。

        参数：
            v_horizontal: 水平速度 (m/s)
            v_vertical: 垂直速度 (m/s)，正值上升、负值下降
            h: 飞行高度 (m)，默认0（海平面）

        固定翼平飞需用功率 = 阻力 x 速度, 阻力分两部分:
          - 寄生阻力功率: P_para = 0.5*rho*v^3*C_D0*S     (∝ v^3, 高速主导)
          - 诱导阻力功率: P_ind = 2*(m*g)^2/(rho*v*S*pi*AR*e)  (∝ 1/v, 低速主导)
        两者之和为 U 型曲线, 最小点即最大航程速度。
          - 爬升附加: P_climb = m*g*v_z / eta_climb (上升), 下降按回收比例
        """
        v = max(v_horizontal, self.v_stall * 0.5)

        # 【修改】使用高度相关的空气密度，替代固定值 self.air_density
        rho = self.get_air_density(h)

        # 寄生阻力功率 (∝ rho * v^3)
        P_para = 0.5 * rho * v ** 3 * self.cd0 * self.wing_area

        # 诱导阻力功率 (∝ 1/(rho*v))
        denom = (rho * v * self.wing_area *
                 np.pi * self.aspect_ratio * self.oswald_efficiency)
        P_ind = 2.0 * (self.mass * self.g) ** 2 / max(denom, 1e-6)

        P_level = P_para + P_ind

        # 爬升/下降附加功率
        if v_vertical > 0.01:
            P_climb = self.mass * self.g * v_vertical / self.climb_efficiency
        elif v_vertical < -0.01:
            P_climb = self.mass * self.g * v_vertical * self.descent_recovery
        else:
            P_climb = 0.0

        # 推进系统效率 (燃油功率 = 需用功率 / 效率)
        P_required = (P_level + P_climb) / self.propulsive_efficiency
        return max(P_required, P_level * 0.5 / self.propulsive_efficiency)

    def get_power_curve(self, speeds: np.ndarray) -> np.ndarray:
        """一组平飞速度下的需用功率曲线。"""
        return np.array([self.get_power_at_state(v, 0.0) for v in speeds])

    def get_optimal_cruise_speed(self) -> float:
        """
        最省油巡航速度 (最小化 P/v, 即最大航程速度)。
        理论上对应 P_induced = P_parasite 附近。
        """
        speeds = np.linspace(self.v_stall, self.max_horizontal_speed, 200)
        fuel_per_dist = np.array([self.get_power_at_state(v, 0.0) / v for v in speeds])
        return speeds[np.argmin(fuel_per_dist)]

    # ------------------------------------------------------------------
    #  飞行段时间与能量
#  ---------------------------------------------------------------
#  get_segment_time:  A* _edge_cost 直接调用, 这是核心接口
#  get_segment_energy: 能耗模型, 当前未使用, 供后续扩展
#  check_range_feasible: 航程硬约束检查, 当前未在 plan() 中调用
#
#  [时间 vs 距离的差异]
#  恒速模型: t = d/v (时间与距离成正比)
#  本模型:   t = max(d/v, dz/v_vert_max)
#  即: 爬升/下降受垂直速率限制 → 时间 = 距离/巡航 或 高度变化/爬升率, 取大者
#  这就打破了"时间与距离等价"的恒速假设, 使 w_time 成为独立维度
    def _clamp_speed(self, v: float) -> float:
        """将速度约束到 [v_stall, max_horizontal_speed]。"""
        return float(np.clip(v, self.v_stall, self.max_horizontal_speed))

    def get_segment_time(self, p1: np.ndarray, p2: np.ndarray, v_avg: float) -> float:
        """
        从 p1 到 p2 的飞行时间 (s)。
        固定翼: 速度不得低于 v_stall; 垂直分量受上升/下降率约束。
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        horiz_dist = np.sqrt(dx ** 2 + dy ** 2)
        total_dist = np.sqrt(horiz_dist ** 2 + dz ** 2)
        if total_dist < 1e-6:
            return 0.0

        v = self._clamp_speed(v_avg)
        t_3d = total_dist / v

        # 垂直速率硬约束
        if abs(dz) > 1e-6:
            v_vert_max = self.max_ascend_speed if dz > 0 else self.max_descend_speed
            t_vert = abs(dz) / v_vert_max
            t_3d = max(t_3d, t_vert)
        return t_3d

    def get_segment_energy(self, p1: np.ndarray, p2: np.ndarray, v_avg: float) -> float:
        """从 p1 到 p2 的能量消耗 (J), 基于升阻比功率模型。"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        horiz_dist = np.sqrt(dx ** 2 + dy ** 2)
        total_dist = np.sqrt(horiz_dist ** 2 + dz ** 2)
        if total_dist < 1e-6:
            return 0.0

        v = self._clamp_speed(v_avg)
        if horiz_dist < 1e-6:
            # 纯垂直机动对固定翼非物理, 退化为最小水平速度
            v_h = self.v_stall
            v_v = np.clip(dz / max(total_dist / v, 1e-6),
                          -self.max_descend_speed, self.max_ascend_speed)
        else:
            climb_angle = np.arctan2(abs(dz), horiz_dist)
            # 爬升角硬约束
            climb_angle = min(climb_angle, np.radians(self.max_climb_angle_deg))
            v_h = v * np.cos(climb_angle)
            v_v = v * np.sin(climb_angle) * (1 if dz >= 0 else -1)
            v_h = max(v_h, self.v_stall)
            if dz > 0:
                v_v = min(v_v, self.max_ascend_speed)
            else:
                v_v = max(v_v, -self.max_descend_speed)

        actual_speed = max(np.sqrt(v_h ** 2 + v_v ** 2), self.v_stall)
        flight_time = total_dist / actual_speed
        power = self.get_power_at_state(v_h, v_v)
        return power * flight_time

    def check_range_feasible(self, total_range: float) -> bool:
        """检查总航程是否在最大航程约束内。"""
        return total_range <= self.max_range

    def get_fuel_fraction(self, total_energy_j: float) -> float:
        """消耗能量占燃油总量的比例 (0~1+)。"""
        return total_energy_j / max(self.fuel_capacity_j, 1.0)

    # ------------------------------------------------------------------
    #  速度剖面计算 (含转弯半径 + 失速速度约束)
    # ------------------------------------------------------------------

    def validate_speed_profile(self, path, speeds, curvatures=None):
        """Validate supplied physical speeds; return segment accelerations.

        This also protects the final evaluator from bypassing strict checks
        by passing an already computed (or externally loaded) speed profile.
        """
        points = np.asarray(path, dtype=float)
        v = np.asarray(speeds, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or v.shape != (len(points),)
                or not np.all(np.isfinite(points)) or not np.all(np.isfinite(v))):
            raise ValueError('Path and speeds must be finite with matching shapes')
        if np.any(v < self.v_stall - 1e-6) or np.any(v > self.max_horizontal_speed + 1e-6):
            raise ValueError('Speed violates the stall/maximum speed envelope')
        if curvatures is not None:
            k = np.asarray(curvatures, dtype=float)
            if k.shape != v.shape or not np.all(np.isfinite(k)):
                raise ValueError('Curvatures must be finite and match speeds')
            if np.any(v**2 * np.abs(k) > self.g * np.tan(np.radians(self.max_bank_angle_deg)) + 1e-6):
                raise ValueError('Speed/curvature violates bank-angle constraint')
        d = np.diff(points, axis=0)
        ds = np.linalg.norm(d, axis=1)
        moving = ds > 1e-9
        if np.any(~moving & (np.abs(np.diff(v)) > 1e-6)):
            raise ValueError('Speed changes over a zero-length segment')
        dt = np.divide(2 * ds, v[:-1] + v[1:])
        accel = np.divide(np.diff(v), dt, out=np.zeros_like(ds), where=moving)
        if (np.any(accel > self.max_horizontal_accel + 1e-6)
                or np.any(accel < -self.max_decel - 1e-6)):
            raise ValueError('Speed profile violates acceleration/deceleration limits')
        slope = np.degrees(np.arctan2(np.abs(d[:, 2]), np.hypot(d[:, 0], d[:, 1])))
        if np.any(slope > self.max_climb_angle_deg + 1e-6):
            raise ValueError('Path violates climb-angle constraint')
        # For constant acceleration, the largest vertical speed on a straight
        # segment is at its faster endpoint, not necessarily at its midpoint.
        vz = np.divide(d[:, 2], ds, out=np.zeros_like(ds), where=moving)
        vz *= np.maximum(v[:-1], v[1:])
        if np.any(vz > self.max_ascend_speed + 1e-6) or np.any(vz < -self.max_descend_speed - 1e-6):
            raise ValueError('Speed profile violates vertical-rate constraints')
        return np.append(accel, 0.0) if len(v) else np.zeros(0)

    def compute_speed_profile(self, dense_path: np.ndarray,
                              curvatures_in: np.ndarray = None,
                              v_ceiling: float = None,
                              strict_feasibility: bool = False
                              ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        沿路径计算连续速度/曲率/加速度剖面（前向-后向算法）。

        与旧版相比，本版把 5 类约束统一放进同一个速度上限框架：
          1. 巡航/用户给定速度上限
          2. 转弯滚转角约束
          3. 失速速度下限
          4. 爬升角硬约束
          5. 最大上升/下降率约束
          6. 前向/后向切向加减速度约束

        注意：
          - 起点是已进入飞行段的边界，速度为 v_stall；不模拟地面滑跑。
            端点高度 0 m 保留为任务高度参考，不代表从静止起飞。
          - 对 B 样条必须尽量传入解析曲率 curvatures_in。
          - 对原始折线，Menger 曲率只能作为网格尺度下的“转弯代理量”。
          - strict_feasibility=True 时，一旦发现“低于失速速度仍无法满足”的
            转弯/垂直率约束，或路径爬升角本身超限，直接抛出 ValueError。
            最终 B 样条轨迹建议开启；粗/细原始折线对比可关闭并只作诊断。

        Returns
        -------
        s_dense, speeds, curvatures, accelerations
        """
        dense_path = np.asarray(dense_path, dtype=float)
        if dense_path.ndim != 2 or dense_path.shape[1] != 3 or not np.all(np.isfinite(dense_path)):
            raise ValueError('dense_path must be a finite (n,3) array')
        n = len(dense_path)

        if v_ceiling is None:
            v_ceiling = self.cruise_speed
        v_ceiling = float(min(v_ceiling, self.max_horizontal_speed))
        if not np.isfinite(v_ceiling) or v_ceiling < self.v_stall:
            raise ValueError('v_ceiling must be finite and at least v_stall')

        if n == 0:
            return (np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0))
        if n == 1:
            return (np.zeros(1), np.full(1, self.v_stall), np.zeros(1), np.zeros(1))

        # ====================================================================
        # Step 1: 路径几何
        # ====================================================================
        diffs = np.diff(dense_path, axis=0)
        seg_dists = np.linalg.norm(diffs, axis=1)

        s_dense = np.zeros(n)
        s_dense[1:] = np.cumsum(seg_dists)

        if curvatures_in is not None:
            curvatures = np.abs(np.asarray(curvatures_in, dtype=float)).ravel()
            if not np.all(np.isfinite(curvatures)):
                raise ValueError('curvatures_in must be finite')
            if len(curvatures) != n:
                raise ValueError(
                    f"curvatures_in 长度 {len(curvatures)} 与路径点数 {n} 不一致")
            turn_radii = np.where(
                curvatures > 1e-12,
                1.0 / np.maximum(curvatures, 1e-12),
                float('inf')
            )
        else:
            curvatures = np.zeros(n)
            turn_radii = np.full(n, float('inf'))
            for i in range(1, n - 1):
                R = self.get_path_curvature_radius(
                    dense_path[i - 1], dense_path[i], dense_path[i + 1])
                turn_radii[i] = R
                if 1e-6 < R < float('inf'):
                    curvatures[i] = 1.0 / R

        # ====================================================================
        # Step 2: 局部速度上限
        # ====================================================================
        v_max = np.full(n, v_ceiling, dtype=float)

        bank_tan = np.tan(np.radians(self.max_bank_angle_deg))
        turn_infeasible = []

        # 2.1 转弯约束
        for i in range(n):
            R = turn_radii[i]
            if R < float('inf'):
                # 不先用 get_turn_speed_for_radius()，因为后者会把低于失速的
                # 结果强行抬到 v_stall，从而掩盖“路径本身不可飞”。
                v_turn_physical = np.sqrt(max(R * self.g * bank_tan, 0.0))

                if v_turn_physical < self.v_stall - 1e-9:
                    turn_infeasible.append(i)

                # 非严格模式仍保留旧版可视化行为：最低画到失速速度；
                # 但会在 diagnostics / warning 中明确标记不可行。
                v_turn_for_profile = max(v_turn_physical, self.v_stall)
                v_max[i] = min(v_max[i], v_turn_for_profile)

        # 2.2 爬升角 + 垂直速度约束
        vertical_caps = np.full(n, float('inf'))
        slope_infeasible_segments = []

        for i in range(n - 1):
            ds = seg_dists[i]
            if ds < 1e-9:
                continue

            dx, dy, dz = diffs[i]
            horiz = float(np.hypot(dx, dy))

            climb_angle = np.degrees(np.arctan2(abs(dz), max(horiz, 1e-12)))
            if climb_angle > self.max_climb_angle_deg + 1e-9:
                slope_infeasible_segments.append(i)

            if abs(dz) > 1e-9:
                vz_limit = self.max_ascend_speed if dz > 0 else self.max_descend_speed

                # v_path * |dz|/ds <= vz_limit
                # => v_path <= vz_limit * ds / |dz|
                v_seg_vertical_max = vz_limit * ds / abs(dz)

                vertical_caps[i] = min(vertical_caps[i], v_seg_vertical_max)
                vertical_caps[i + 1] = min(vertical_caps[i + 1],
                                           v_seg_vertical_max)

        v_max = np.minimum(v_max, vertical_caps)

        # The stall constraint applies to every point, including the start.
        below_stall_cap = np.where(v_max < self.v_stall - 1e-9)[0]

        problems = []
        if turn_infeasible:
            problems.append(
                f"{len(turn_infeasible)} 个转弯点在 v_stall 下仍无法满足滚转角约束")
        if slope_infeasible_segments:
            problems.append(
                f"{len(slope_infeasible_segments)} 个路径段超过最大爬升角 "
                f"{self.max_climb_angle_deg:.1f}°")
        if len(below_stall_cap) > 0:
            problems.append(
                f"{len(below_stall_cap)} 个点的垂直率/局部速度上限低于 v_stall")

        if strict_feasibility and problems:
            raise ValueError("Path is dynamically infeasible: " + "；".join(problems))
        elif problems:
            print("[警告] compute_speed_profile: " + "；".join(problems))

        # 非严格模式为了能画出原始折线对比，仍以失速速度作为最低工作速度。
        v_max = np.maximum(v_max, self.v_stall)

        # Airborne boundary; no unmodelled ground roll below stall speed.
        v_max[0] = self.v_stall

        # 终点不高于巡航速度
        v_max[-1] = min(v_max[-1], self.cruise_speed)

        # ====================================================================
        # Step 3: Forward pass
        # ====================================================================
        v_fwd = np.zeros(n, dtype=float)
        v_fwd[0] = self.v_stall

        for i in range(n - 1):
            ds = seg_dists[i]
            v_achievable_sq = (
                v_fwd[i] ** 2 + 2.0 * self.max_horizontal_accel * ds
            )
            v_achievable = np.sqrt(max(0.0, v_achievable_sq))
            v_fwd[i + 1] = min(v_achievable, v_max[i + 1])

        # ====================================================================
        # Step 4: Backward pass
        # ====================================================================
        v_bwd = np.full(n, float('inf'), dtype=float)
        v_bwd[-1] = v_fwd[-1]

        for i in range(n - 2, -1, -1):
            ds = seg_dists[i]
            v_required_sq = (
                v_bwd[i + 1] ** 2 + 2.0 * self.max_decel * ds
            )
            v_required = np.sqrt(max(0.0, v_required_sq))
            v_bwd[i] = min(v_fwd[i], v_required)

        speeds = v_bwd.copy()
        if strict_feasibility:
            self.validate_speed_profile(dense_path, speeds, curvatures)

        # ====================================================================
        # Step 5: 从速度反算切向加速度
        # ====================================================================
        accelerations = np.zeros(n, dtype=float)
        segment_times = np.zeros(n - 1, dtype=float)
        vertical_rates = np.zeros(n - 1, dtype=float)

        for i in range(n - 1):
            ds = seg_dists[i]
            v_avg = max((speeds[i] + speeds[i + 1]) / 2.0, 1e-3)
            dt = ds / v_avg if ds > 1e-12 else 0.0
            segment_times[i] = dt

            dv = speeds[i + 1] - speeds[i]
            accelerations[i] = dv / dt if dt > 1e-12 else 0.0

            if dt > 1e-12:
                vertical_rates[i] = diffs[i, 2] / dt

        accelerations[-1] = 0.0

        # ====================================================================
        # Step 6: 数值验证 + diagnostics
        # ====================================================================
        accel_viol = float(np.max(accelerations - self.max_horizontal_accel))
        decel_viol = float(np.max(-self.max_decel - accelerations))

        if accel_viol > 1e-3:
            if strict_feasibility:
                raise ValueError('Tangential acceleration exceeds the limit')
            print(f"[警告] compute_speed_profile: 加速度超限 {accel_viol:.4f} m/s²")
        if decel_viol > 1e-3:
            if strict_feasibility:
                raise ValueError('Tangential deceleration exceeds the limit')
            print(f"[警告] compute_speed_profile: 减速度超限 {decel_viol:.4f} m/s²")

        n_stall = int(np.sum(speeds < self.v_stall - 1e-3))
        if n_stall:
            if strict_feasibility:
                raise ValueError('Speed falls below stall speed')
            print(f'[警告] compute_speed_profile: {n_stall} 点低于失速速度')

        climb_rate_viol = np.where(
            vertical_rates > self.max_ascend_speed + 1e-3
        )[0]
        descend_rate_viol = np.where(
            -vertical_rates > self.max_descend_speed + 1e-3
        )[0]

        if len(climb_rate_viol) > 0:
            msg = (
                f"{len(climb_rate_viol)} 段上升率超过 "
                f"{self.max_ascend_speed:.1f} m/s"
            )
            if strict_feasibility:
                raise ValueError("Path is dynamically infeasible: " + msg)
            print("[警告] compute_speed_profile: " + msg)

        if len(descend_rate_viol) > 0:
            msg = (
                f"{len(descend_rate_viol)} 段下降率超过 "
                f"{self.max_descend_speed:.1f} m/s"
            )
            if strict_feasibility:
                raise ValueError("Path is dynamically infeasible: " + msg)
            print("[警告] compute_speed_profile: " + msg)

        # 方便可视化和调试读取，不改变函数返回签名
        self.last_speed_profile_diagnostics = {
            'turn_infeasible_points': np.asarray(turn_infeasible, dtype=int),
            'slope_infeasible_segments': np.asarray(
                slope_infeasible_segments, dtype=int),
            'below_stall_cap_points': np.asarray(below_stall_cap, dtype=int),
            'vertical_rates': vertical_rates,
            'segment_times': segment_times,
            'n_stall_points': n_stall,
            'n_stall_after_takeoff': n_stall,  # compatibility: now checks every point
            'start_speed': float(speeds[0]),
            'strict_feasibility': bool(strict_feasibility),
        }

        return s_dense, speeds, curvatures, accelerations


# 向后兼容别名: 旧代码引用 UAVDynamics
UAVDynamics = FixedWingDynamics
