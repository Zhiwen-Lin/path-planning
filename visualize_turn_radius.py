#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
转弯半径分析可视化
==================
分析轨迹沿程转弯半径，验证固定翼运动学约束
展示B样条光滑的效果（消除曲率突变）
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pickle
import os

# 中文字体设置
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel
from fixed_wing_dynamics import FixedWingDynamics
from trajectory_smoothing import spline_curvature, resample_by_arclength

print("=" * 80)
print("  转弯半径分析")
print("=" * 80)

# ============================================================================
# 加载路径和动力学模型
# ============================================================================
PICKLE_PATH = "output_5zones/planned_path.pkl"
if not os.path.exists(PICKLE_PATH):
    print("\n未找到缓存路径，请先运行 main.py")
    import sys; sys.exit(1)

print("\n加载路径数据...")
with open(PICKLE_PATH, 'rb') as f:
    data = pickle.load(f)

path_smooth = np.array(data['path'])           # 光滑后路径
path_raw = np.array(data.get('path_raw', []))  # 原始细网格路径
smooth_result = data.get('smooth_result', {})
smoothed = smooth_result.get('smoothed', False)

print(f"  光滑后路径: {len(path_smooth)} 个航点")
if len(path_raw) > 0:
    print(f"  原始路径: {len(path_raw)} 个航点")
    has_raw = True
else:
    print(f"  原始路径: 未保存（老版本 pkl）")
    has_raw = False

dynamics = FixedWingDynamics()

# ============================================================================
# 计算转弯半径
# ============================================================================
print("\n计算转弯半径...")

def compute_turn_radii(path):
    """
    折线路径沿程转弯半径（三点 Menger 曲率）

    仅适用于折线。对 B 样条采样点用本函数会得到随点距发散的半径，
    光滑轨迹请用 compute_turn_radii_spline()。
    """
    if len(path) < 3:
        return np.array([]), np.array([])

    radii = []
    distances = [0]  # 累积距离

    for i in range(1, len(path) - 1):
        R = dynamics.get_path_curvature_radius(path[i-1], path[i], path[i+1])
        radii.append(R)

        # 累积距离（到当前点）
        d = np.linalg.norm(path[i] - path[i-1])
        distances.append(distances[-1] + d)

    # 最后一个点的累积距离
    distances.append(distances[-1] + np.linalg.norm(path[-1] - path[-2]))

    # 转弯半径对应的距离（去掉首尾点）
    distances_radii = np.array(distances[1:-1]) / 1000.0  # km
    radii = np.array(radii)

    # 处理无穷大（直线段）
    radii = np.clip(radii, 0, 50000)  # 限制最大值50km，便于可视化

    return distances_radii, radii


def compute_turn_radii_spline(tck, path):
    """
    B 样条轨迹沿程转弯半径（解析曲率 k = |r'×r''|/|r'|³）

    与采样密度无关。三点公式在点距远小于曲率半径时因浮点相消发散，
    会把一条 850 m 半径的平缓弯显示成几十米的尖角。
    """
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    total = float(np.sum(seg))
    ds = total / max(len(path) - 1, 1)

    pts, u = resample_by_arclength(tck, ds)
    k = spline_curvature(tck, u)
    radii = np.where(k > 1e-12, 1.0 / np.maximum(k, 1e-12), 50000.0)
    radii = np.clip(radii, 0, 50000)

    s = np.concatenate([[0.0],
                        np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    return s / 1000.0, radii

# 光滑后路径: 有样条则用解析曲率
_tck = smooth_result.get('tck', None)
if _tck is not None:
    dist_smooth, radii_smooth = compute_turn_radii_spline(_tck, path_smooth)
    print("  光滑轨迹曲率: B 样条解析导数")
else:
    dist_smooth, radii_smooth = compute_turn_radii(path_smooth)
    print("  光滑轨迹曲率: 三点公式（无样条，数值可能偏低）")

# 原始路径（如果有）
if has_raw and len(path_raw) >= 3:
    dist_raw, radii_raw = compute_turn_radii(path_raw)
else:
    dist_raw, radii_raw = np.array([]), np.array([])

# 统计
min_R_smooth = radii_smooth.min()
mean_R_smooth = radii_smooth.mean()
min_feasible = dynamics.get_min_feasible_radius()
min_cruise = dynamics.get_min_turn_radius(dynamics.cruise_speed)

print(f"\n[光滑后路径]")
print(f"  最小转弯半径: {min_R_smooth:.0f} m")
print(f"  平均转弯半径: {mean_R_smooth:.0f} m")
print(f"  最小可行半径: {min_feasible:.0f} m (失速速度 + 最大滚转角)")
print(f"  巡航最小半径: {min_cruise:.0f} m (巡航速度 + 最大滚转角)")
print(f"  约束满足: {'✓' if min_R_smooth >= min_feasible else '✗'}")

if len(radii_raw) > 0:
    min_R_raw = radii_raw.min()
    mean_R_raw = radii_raw.mean()
    print(f"\n[原始路径]")
    print(f"  最小转弯半径: {min_R_raw:.0f} m")
    print(f"  平均转弯半径: {mean_R_raw:.0f} m")
    print(f"  约束满足: {'✓' if min_R_raw >= min_feasible else '✗'}")

# ============================================================================
# 生成可视化
# ============================================================================
print("\n生成可视化...")

# 配色
COLOR_SMOOTH = '#00CED1'    # 青色 - 光滑后
COLOR_RAW = '#FFA07A'       # 浅橙色 - 原始
COLOR_LIMIT_1 = '#DC143C'   # 深红色 - 最小可行半径
COLOR_LIMIT_2 = '#FF1493'   # 深粉红 - 巡航最小半径

if has_raw and len(radii_raw) > 0:
    # 双子图：上图光滑后，下图对比
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.patch.set_facecolor('white')

    # ========================================================================
    # 子图1：光滑后路径转弯半径
    # ========================================================================
    ax1 = axes[0]
    ax1.set_facecolor('#F5F5F5')

    ax1.plot(dist_smooth, radii_smooth/1000, '-', color=COLOR_SMOOTH,
             linewidth=2.5, label='光滑后路径', alpha=1.0)

    # 约束参考线
    ax1.axhline(y=min_feasible/1000, color=COLOR_LIMIT_1, linestyle='--',
                linewidth=2.5, alpha=0.8,
                label=f'最小可行半径 ({min_feasible:.0f} m)')
    ax1.axhline(y=min_cruise/1000, color=COLOR_LIMIT_2, linestyle=':',
                linewidth=2.5, alpha=0.8,
                label=f'巡航最小半径 ({min_cruise:.0f} m)')

    # 填充安全区域
    ax1.axhspan(min_feasible/1000, 50, alpha=0.1, color='#66BB6A',
                label='安全区域')
    ax1.axhspan(0, min_feasible/1000, alpha=0.1, color='#EF5350',
                label='不可行区域')

    # 标注最小转弯半径点
    idx_min = np.argmin(radii_smooth)
    ax1.scatter([dist_smooth[idx_min]], [radii_smooth[idx_min]/1000],
                c='red', s=200, marker='*', edgecolors='black',
                linewidths=2, zorder=15, label=f'最小半径点 ({min_R_smooth:.0f} m)')
    ax1.annotate(f'{min_R_smooth:.0f} m',
                 xy=(dist_smooth[idx_min], radii_smooth[idx_min]/1000),
                 xytext=(dist_smooth[idx_min]+5, radii_smooth[idx_min]/1000+2),
                 fontsize=10, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.5', facecolor='#FFEB3B',
                          alpha=0.9, edgecolor='black', linewidth=1.5))

    ax1.set_xlabel('累积距离 (km)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('转弯半径 (km)', fontsize=12, fontweight='bold')
    ax1.set_title('(a) 光滑后路径转弯半径分析',
                  fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.legend(loc='upper right', fontsize=10, framealpha=0.95)
    ax1.set_ylim(0, min(20, radii_smooth.max()/1000 * 1.2))

    # ========================================================================
    # 子图2：光滑前后对比
    # ========================================================================
    ax2 = axes[1]
    ax2.set_facecolor('#F5F5F5')

    ax2.plot(dist_raw, radii_raw/1000, '-', color=COLOR_RAW,
             linewidth=2, label='原始路径（网格折线）', alpha=0.8)

    ax2.plot(dist_smooth, radii_smooth/1000, '-', color=COLOR_SMOOTH,
             linewidth=2.5, label='光滑后路径（B样条）', alpha=1.0)

    # 约束参考线
    ax2.axhline(y=min_feasible/1000, color=COLOR_LIMIT_1, linestyle='--',
                linewidth=2, alpha=0.7)
    ax2.axhline(y=min_cruise/1000, color=COLOR_LIMIT_2, linestyle=':',
                linewidth=2, alpha=0.7)

    # 填充区域
    ax2.axhspan(min_feasible/1000, 50, alpha=0.08, color='#66BB6A')
    ax2.axhspan(0, min_feasible/1000, alpha=0.08, color='#EF5350')

    ax2.set_xlabel('累积距离 (km)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('转弯半径 (km)', fontsize=12, fontweight='bold')
    ax2.set_title('(b) 光滑前后对比 - B样条消除曲率突变',
                  fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.legend(loc='upper right', fontsize=10, framealpha=0.95)
    ax2.set_ylim(0, min(20, max(radii_raw.max(), radii_smooth.max())/1000 * 1.2))

    # 统计信息框
    stats_text = f'光滑效果统计:\n'
    stats_text += f'最小半径: {min_R_raw:.0f} → {min_R_smooth:.0f} m\n'
    stats_text += f'平均半径: {mean_R_raw:.0f} → {mean_R_smooth:.0f} m\n'
    stats_text += f'改善: {((min_R_smooth/min_R_raw - 1)*100):+.1f}%'

    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='#FFEB3B',
                      alpha=0.9, edgecolor='black', linewidth=1.5))

    fig.suptitle('转弯半径分析 - 固定翼运动学约束验证',
                 fontsize=15, fontweight='bold', y=0.995)

else:
    # 单图：仅光滑后路径
    fig, ax = plt.subplots(figsize=(14, 8))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('#F5F5F5')

    ax.plot(dist_smooth, radii_smooth/1000, '-', color=COLOR_SMOOTH,
            linewidth=3, label='光滑后路径', alpha=1.0)

    # 约束参考线
    ax.axhline(y=min_feasible/1000, color=COLOR_LIMIT_1, linestyle='--',
               linewidth=2.5, alpha=0.8,
               label=f'最小可行半径 ({min_feasible:.0f} m)')
    ax.axhline(y=min_cruise/1000, color=COLOR_LIMIT_2, linestyle=':',
               linewidth=2.5, alpha=0.8,
               label=f'巡航最小半径 ({min_cruise:.0f} m)')

    # 填充区域
    ax.axhspan(min_feasible/1000, 50, alpha=0.1, color='#66BB6A',
               label='安全区域')
    ax.axhspan(0, min_feasible/1000, alpha=0.1, color='#EF5350',
               label='不可行区域')

    # 标注最小转弯半径点
    idx_min = np.argmin(radii_smooth)
    ax.scatter([dist_smooth[idx_min]], [radii_smooth[idx_min]/1000],
               c='red', s=250, marker='*', edgecolors='black',
               linewidths=2.5, zorder=15, label=f'最小半径点 ({min_R_smooth:.0f} m)')
    ax.annotate(f'{min_R_smooth:.0f} m',
                xy=(dist_smooth[idx_min], radii_smooth[idx_min]/1000),
                xytext=(dist_smooth[idx_min]+5, radii_smooth[idx_min]/1000+2),
                fontsize=11, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='#FFEB3B',
                         alpha=0.9, edgecolor='black', linewidth=1.5))

    ax.set_xlabel('累积距离 (km)', fontsize=13, fontweight='bold')
    ax.set_ylabel('转弯半径 (km)', fontsize=13, fontweight='bold')
    ax.set_title('转弯半径分析 - 固定翼运动学约束验证',
                 fontsize=14, fontweight='bold', pad=15)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='upper right', fontsize=11, framealpha=0.95)
    ax.set_ylim(0, min(20, radii_smooth.max()/1000 * 1.2))

plt.tight_layout()

output_path = "output_5zones/turn_radius_analysis.png"
plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
plt.close()

print(f"\n✓ 转弯半径分析图已保存: {output_path}")
print("=" * 80)
