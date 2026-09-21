#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
analyze_radar_interaction_v2.py - 轨迹-雷达交互分析（高质量重构版）
==================================================================
分析已规划轨迹与雷达区的交互情况

生成图表：
1. 轨迹-雷达交互总览图（带穿透点标注）
2. 各雷达区穿透统计（距离、暴露度）
"""

from trajectory_data_loader import load_trajectory_data, trajectory_output_path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.collections import LineCollection
import pickle
import os, warnings
warnings.filterwarnings('ignore')

# ── 字体配置 ──────────────────────────────────────────────────────────────────
matplotlib.rcParams.update({
    'font.sans-serif'   : ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS'],
    'font.family'       : 'sans-serif',
    'axes.unicode_minus': False,
    'font.size'         : 10,
    'axes.titlesize'    : 13,
    'axes.labelsize'    : 11,
    'xtick.labelsize'   : 10,
    'ytick.labelsize'   : 10,
    'legend.fontsize'   : 9,
    'figure.dpi'        : 120,
    'savefig.dpi'       : 600,
    'axes.linewidth'    : 1.0,
    'pdf.fonttype'      : 42,
    'ps.fonttype'       : 42,
})

# ── 配色方案 ──────────────────────────────────────────────────────────────────
COLOR_RADAR = '#ff9999'         # 雷达区填充（浅红）
COLOR_RADAR_EDGE = '#cc0000'    # 雷达区边界（深红）
COLOR_FIRE = '#ffcc99'          # 火力区填充（浅橙）
COLOR_FIRE_EDGE = '#ff6600'     # 火力区边界（深橙）
COLOR_PATH = '#3498db'          # 轨迹线（蓝）
COLOR_PATH_THICK = '#2874a6'    # 穿透段（深蓝）
COLOR_START = '#27ae60'         # 起点（绿）
COLOR_GOAL = '#e74c3c'          # 终点（红）
COLOR_PENETRATION = '#f39c12'   # 穿透点（橙）

OUTDIR = os.environ.get('ANALYSIS_OUTDIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analysis_figures'))
os.makedirs(OUTDIR, exist_ok=True)

print("=" * 80)
print("  轨迹-雷达交互分析（v2 高质量版）")
print("=" * 80)

# ── 加载环境和轨迹 ────────────────────────────────────────────────────────────
from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel

env = RadarEnvironment5Zones()
env.generate(seed=42)
threat = EnvironmentThreatModel(env, model='power', power_n=4.0)

data = load_trajectory_data()

path = np.array(data['path'])
stats = data['stats']
start = data['start']
goal = data['goal']
w_time = data.get('w_time', 0.0)
w_risk = data.get('w_risk', 1.0)
w_fuel = data.get('w_fuel', 0.0)

print(f"  ✓ 轨迹航点: {len(path)} 个")
print(f"  ✓ 航程: {stats['L']/1000:.1f} km")
print(f"  ✓ 权重: w_time={w_time:.2f}, w_risk={w_risk:.2f}, w_fuel={w_fuel:.2f}")

# ── 分析穿透情况 ──────────────────────────────────────────────────────────────
print("\n[2/2] 分析雷达穿透...")

KM = 1000.0
R_radar = env.radar_radius / KM

penetration_info = {}
penetration_points = []  # 所有穿透点

for i, radar in enumerate(env.radar_zones):
    cx, cy = radar.x / KM, radar.y / KM

    # 检查每段轨迹
    penetrations = []
    for j in range(len(path) - 1):
        p0 = np.asarray(path[j], dtype=float)
        p1 = np.asarray(path[j + 1], dtype=float)

        # 3-D midpoint: the radar region is a hemisphere, so evaluating at
        # z=0 (old code) falsely labels high-altitude segments as penetrations.
        pm = 0.5 * (p0 + p1)
        mx, my, mz = pm / KM
        slant_dist = np.sqrt((mx - cx)**2 + (my - cy)**2 + mz**2)

        if radar.is_point_inside(pm[0], pm[1], pm[2]):
            penetrations.append({
                'segment': j,
                'point': (mx, my),
                'altitude_km': mz,
                'dist_to_center': slant_dist,
                # EnvironmentThreatModel returns Pd / calculate_risk; there is
                # no "risk" key in evaluate_point.
                'exposure': float(threat.calculate_risk(pm))
            })
            penetration_points.append((mx, my, i))  # (x, y, radar_id)

    penetration_info[i] = {
        'center': (cx, cy),
        'penetrations': penetrations,
        'count': len(penetrations)
    }

print(f"  ✓ 雷达区总数: {len(env.radar_zones)}")
print(f"  ✓ 穿透区域数: {sum(1 for info in penetration_info.values() if info['count'] > 0)}")
print(f"  ✓ 总穿透点数: {sum(info['count'] for info in penetration_info.values())}")

# ════════════════════════════════════════════════════════════════════════════════
# 图: 轨迹-雷达交互总览
# ════════════════════════════════════════════════════════════════════════════════
print("\n生成交互总览图...")

fig, ax = plt.subplots(figsize=(12, 10))

# 绘制雷达区
for i, radar in enumerate(env.radar_zones):
    cx, cy = radar.x / KM, radar.y / KM
    circle = Circle((cx, cy), R_radar,
                   facecolor=COLOR_RADAR, edgecolor=COLOR_RADAR_EDGE,
                   linewidth=1.5, alpha=0.4, zorder=1)
    ax.add_patch(circle)

    # 标注雷达编号
    ax.text(cx, cy, f'R{i+1}',
           ha='center', va='center',
           fontsize=11, weight='bold',
           color=COLOR_RADAR_EDGE,
           bbox=dict(boxstyle='circle', facecolor='white',
                    edgecolor=COLOR_RADAR_EDGE, linewidth=1.5, alpha=0.9),
           zorder=5)

# 绘制火力区
for fire in env.fire_zones:
    cx, cy = fire.x / KM, fire.y / KM
    R_fire = fire.horizontal_radius / KM
    circle = Circle((cx, cy), R_fire,
                   facecolor=COLOR_FIRE, edgecolor=COLOR_FIRE_EDGE,
                   linewidth=1.2, alpha=0.3, zorder=1)
    ax.add_patch(circle)

# 绘制轨迹（普通段）
path_km = path[:, :2] / KM  # 只取 x, y 坐标
ax.plot(path_km[:, 0], path_km[:, 1],
       color=COLOR_PATH, linewidth=2.5, alpha=0.8,
       label='规划轨迹', zorder=3)

# 高亮穿透段
penetrated_segments = set()
for info in penetration_info.values():
    for p in info['penetrations']:
        penetrated_segments.add(p['segment'])

for seg_id in penetrated_segments:
    x0, y0 = path[seg_id, 0] / KM, path[seg_id, 1] / KM
    x1, y1 = path[seg_id + 1, 0] / KM, path[seg_id + 1, 1] / KM
    ax.plot([x0, x1], [y0, y1],
           color=COLOR_PATH_THICK, linewidth=4, alpha=0.9,
           zorder=4)

# 标注穿透点
if penetration_points:
    px = [p[0] for p in penetration_points]
    py = [p[1] for p in penetration_points]
    ax.scatter(px, py, s=100, c=COLOR_PENETRATION,
              marker='X', edgecolors='black', linewidth=1.2,
              label=f'穿透点 ({len(penetration_points)})', zorder=6, alpha=0.9)

# 起点和终点
ax.scatter(start[0]/KM, start[1]/KM, s=300, c=COLOR_START, marker='*',
          edgecolors='black', linewidth=2, label='起点', zorder=7)
ax.scatter(goal[0]/KM, goal[1]/KM, s=300, c=COLOR_GOAL, marker='*',
          edgecolors='black', linewidth=2, label='终点', zorder=7)

# 坐标轴设置
ax.set_xlabel('X 坐标 (千米)', fontsize=12, weight='bold')
ax.set_ylabel('Y 坐标 (千米)', fontsize=12, weight='bold')
ax.set_title('轨迹-雷达交互分析总览', fontsize=14, weight='bold', pad=15)
ax.legend(loc='upper left', fontsize=10, framealpha=0.95)
ax.grid(True, linestyle='--', alpha=0.3)
ax.set_aspect('equal')
ax.set_xlim([0, 100])
ax.set_ylim([0, 100])

# 添加统计信息文本框
info_text = (f"轨迹统计:\n"
            f"  航程: {stats['L']/1000:.1f} km\n"
            f"  累积暴露: {stats['Crisk']:.2f} s\n"
            f"  飞行时间: {stats.get('T', 0)/60:.1f} min\n"
            f"  燃油: {stats.get('F', 0):.2f} kg\n"
            f"\n权重配置:\n"
            f"  w_time={w_time:.2f}\n"
            f"  w_risk={w_risk:.2f}\n"
            f"  w_fuel={w_fuel:.2f}")

ax.text(0.98, 0.02, info_text,
       transform=ax.transAxes,
       fontsize=9,
       verticalalignment='bottom',
       horizontalalignment='right',
       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black', linewidth=1.2))

plt.tight_layout()

output = trajectory_output_path('radar_interaction_overview.png')
fig.savefig(output, dpi=600, bbox_inches='tight', facecolor='white')
print(f"  ✓ 已保存: {output}")
plt.close()

# ── 打印详细统计 ──────────────────────────────────────────────────────────────
print("\n" + "=" * 80)
print("  穿透详情")
print("=" * 80)

for radar_id, info in penetration_info.items():
    if info['count'] > 0:
        print(f"\n雷达区 R{radar_id+1} (中心: {info['center'][0]:.1f}, {info['center'][1]:.1f} km)")
        print(f"  穿透采样段数: {info['count']}")

        avg_dist = np.mean([p['dist_to_center'] for p in info['penetrations']])
        avg_expo = np.mean([p['exposure'] for p in info['penetrations']])

        print(f"  平均距中心: {avg_dist:.2f} km")
        print(f"  区内采样段融合探测概率均值 (max-Pd): {avg_expo:.4f}")

print("\n" + "=" * 80)
print("  完成！")
print(f"  - {output}")
print("=" * 80)
