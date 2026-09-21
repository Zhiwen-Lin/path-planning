#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
轨迹-雷达交互分析（优化配色版）
================================
分析轨迹是否穿过雷达区，生成专业图表
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import pickle
import os

# 中文字体设置
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 11
plt.rcParams['axes.linewidth'] = 1.2

from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel
from fixed_wing_dynamics import FixedWingDynamics

# 优化配色
COLOR_RADAR = '#FFB6C1'      # 浅粉色
COLOR_RADAR_EDGE = '#DC143C' # 深红色
COLOR_FIRE = '#FFA07A'       # 浅橙色
COLOR_FIRE_EDGE = '#FF4500'  # 橙红色
COLOR_PATH = '#00CED1'       # 青色
COLOR_START = '#32CD32'      # 亮绿色
COLOR_GOAL = '#FF1493'       # 深粉红

print("=" * 80)
print("  轨迹-雷达交互分析")
print("=" * 80)

# 生成环境
env = RadarEnvironment5Zones()
env.generate(seed=42)
threat = EnvironmentThreatModel(env, model='power', power_n=4.0)

# 尝试加载已规划路径
PICKLE_PATH = "output_5zones/planned_path.pkl"
if os.path.exists(PICKLE_PATH):
    print("\n加载已规划轨迹...")
    with open(PICKLE_PATH, 'rb') as f:
        data = pickle.load(f)
    path = np.array(data['path'])
    stats = data['stats']
    start = data['start']
    goal = data['goal']
    # 从 pickle 读取权重，与 main.py 保持一致
    _w_risk = data['w_risk']
    _w_time = data['w_time']
    _w_fuel = data['w_fuel']
    print(f"  已加载: {len(path)} 个航点, 航程 {stats['L']/1000:.1f} km")
else:
    print("\n未找到缓存路径，请先运行 main.py 生成 planned_path.pkl")
    import sys; sys.exit(1)

print(f"路径航点数: {len(path)}")

# ============================================================================
# 分析每个雷达区
# ============================================================================
print("\n" + "=" * 80)
print("  雷达穿透分析")
print("=" * 80)

KM = 1000.0
R_radar = env.radar_radius / KM

penetration_info = {}

for i, radar in enumerate(env.radar_zones):
    cx, cy = radar.x / KM, radar.y / KM

    print(f"\n[{radar.name}]")
    print(f"  位置: ({cx:.1f}, {cy:.1f}) km")
    print(f"  半径: {R_radar:.1f} km")

    # 检查穿透
    inside_points = []
    for j, pt in enumerate(path):
        px, py, pz = pt[0]/KM, pt[1]/KM, pt[2]/KM

        horiz_dist = np.sqrt((px - cx)**2 + (py - cy)**2)

        if horiz_dist <= R_radar:
            h_hemisphere = np.sqrt(R_radar**2 - horiz_dist**2)
            if pz <= h_hemisphere:
                inside_points.append((j, pt, horiz_dist, pz, h_hemisphere))

    penetration_info[radar.name] = {
        'penetrated': len(inside_points) > 0,
        'num_points': len(inside_points),
        'points': inside_points
    }

    if inside_points:
        print(f"  状态: 穿透")
        print(f"  穿透航点数: {len(inside_points)}")
    else:
        print(f"  状态: 安全")

# ============================================================================
# 生成专业图表
# ============================================================================
print("\n" + "=" * 80)
print("  Generating visualization...")
print("=" * 80)

fig = plt.figure(figsize=(18, 8))
fig.patch.set_facecolor('white')

# ============================================================================
# 左图：俯视图
# ============================================================================
ax1 = fig.add_subplot(1, 2, 1)
ax1.set_facecolor('#FAFAFA')

m = env.map_size / KM

# 雷达区
for i, radar in enumerate(env.radar_zones):
    cx, cy = radar.x / KM, radar.y / KM

    # 判断是否穿透
    penetrated = penetration_info[radar.name]['penetrated']
    edge_color = '#D00000' if penetrated else COLOR_RADAR_EDGE
    face_alpha = 0.35 if penetrated else 0.22

    circle = Circle((cx, cy), R_radar,
                   facecolor=COLOR_RADAR, edgecolor=edge_color,
                   alpha=face_alpha, linewidth=2.5, zorder=3)
    ax1.add_patch(circle)

    # 雷达站标记
    ax1.plot(cx, cy, '^', color=edge_color, markersize=11,
           markeredgecolor='white', markeredgewidth=1.8, zorder=6)

    # 标注
    label_text = radar.name
    if penetrated:
        label_text += f'\n(Penetrated)'
    ax1.text(cx, cy + R_radar + 3, label_text,
           ha='center', fontsize=10, fontweight='normal',
           color=edge_color, zorder=7,
           bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                    edgecolor=edge_color, linewidth=1.5, alpha=0.95))

# 火力区
for fire in env.fire_zones:
    cx, cy = fire.x / KM, fire.y / KM
    fire_r = fire.horizontal_radius / KM

    circle = Circle((cx, cy), fire_r,
                   facecolor=COLOR_FIRE, edgecolor='#D85D00',
                   alpha=0.4, linewidth=2.2, zorder=4)
    ax1.add_patch(circle)

# 轨迹
ax1.plot(path[:, 0]/KM, path[:, 1]/KM, '-',
        color=COLOR_PATH, linewidth=3, alpha=0.9, zorder=8)

# 航点（稀疏）
sample_indices = range(0, len(path), max(1, len(path)//15))
ax1.scatter(path[sample_indices, 0]/KM,
           path[sample_indices, 1]/KM,
           c=COLOR_PATH, s=90, edgecolors='white',
           linewidths=1.5, zorder=9, alpha=0.85)

# 起点和终点
ax1.scatter([start[0]/KM], [start[1]/KM],
           c=COLOR_START, s=220, marker='^',
           edgecolors='white', linewidths=2,
           zorder=10)
ax1.scatter([goal[0]/KM], [goal[1]/KM],
           c=COLOR_GOAL, s=220, marker='v',
           edgecolors='white', linewidths=2,
           zorder=10)

ax1.set_xlabel('X (km)', fontsize=13, fontweight='normal')
ax1.set_ylabel('Y (km)', fontsize=13, fontweight='normal')
ax1.set_title('(a) 俯视图: 轨迹与雷达区',
             fontsize=14, fontweight='normal', pad=15)
ax1.set_xlim(0, m)
ax1.set_ylim(0, m)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.25, linestyle='-', linewidth=0.6, color='#AAAAAA')

# 图例
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

legend_elements = [
    Line2D([0], [0], color=COLOR_PATH, linewidth=2.5, label='轨迹'),
    Line2D([0], [0], marker='^', color='w', markerfacecolor=COLOR_START,
           markersize=11, label='起点', markeredgewidth=0),
    Line2D([0], [0], marker='v', color='w', markerfacecolor=COLOR_GOAL,
           markersize=11, label='终点', markeredgewidth=0),
    Patch(facecolor=COLOR_RADAR, alpha=0.3, label='雷达区'),
    Patch(facecolor=COLOR_FIRE, alpha=0.4, label='火力区'),
]
ax1.legend(handles=legend_elements, loc='upper left',
          fontsize=10, framealpha=0.95, edgecolor='black')

# ============================================================================
# 右图：雷达间隙曲线
# ============================================================================
ax2 = fig.add_subplot(1, 2, 2)
ax2.set_facecolor('#FAFAFA')

# 计算累积距离
distances = [0]
for i in range(1, len(path)):
    d = np.linalg.norm(path[i, :2] - path[i-1, :2])
    distances.append(distances[-1] + d)
distances = np.array(distances) / KM

# 每个雷达的间隙曲线
colors_radar = ['#E63946', '#D90429', '#8D0801', '#590D22', '#2B2D42']

for idx, radar in enumerate(env.radar_zones):
    cx, cy = radar.x / KM, radar.y / KM

    clearances = []
    for pt in path:
        px, py, pz = pt[0]/KM, pt[1]/KM, pt[2]/KM

        # 3D距离到雷达表面
        dist_3d = np.sqrt((px - cx)**2 + (py - cy)**2 + pz**2)
        clearance = dist_3d - R_radar
        clearances.append(clearance)

    # 判断是否穿透
    penetrated = penetration_info[radar.name]['penetrated']
    linestyle = '-' if penetrated else '--'
    linewidth = 2.5 if penetrated else 1.8
    alpha_val = 0.9 if penetrated else 0.6

    ax2.plot(distances, clearances, linestyle,
            color=colors_radar[idx], linewidth=linewidth,
            label=radar.name, alpha=alpha_val, marker='o', markersize=3)

# 零线（雷达边界）
ax2.axhline(y=0, color='#D00000', linestyle='-', linewidth=2.5,
           alpha=0.8, label='雷达边界', zorder=10)

# 安全距离线
ax2.axhline(y=5, color='#F77F00', linestyle=':', linewidth=2,
           alpha=0.7, label='Safety Margin (5 km)')

# 危险区域阴影
ax2.axhspan(-50, 0, alpha=0.15, color='#E63946', zorder=1)

ax2.set_xlabel('Cumulative Distance (km)', fontsize=13, fontweight='normal')
ax2.set_ylabel('距雷达表面间隙 (km)', fontsize=13, fontweight='normal')
ax2.set_title('(b) 轨迹沿程雷达间隙\n(负值 = 穿透)',
             fontsize=14, fontweight='normal', pad=15)
ax2.grid(True, alpha=0.25, linestyle='-', linewidth=0.6, color='#AAAAAA')
ax2.legend(loc='upper right', fontsize=10, framealpha=0.95, edgecolor='black')

# 设置y轴范围
ax2.set_ylim(-5, max(max(clearances) for radar in env.radar_zones
                     for clearances in [[np.sqrt((pt[0]/KM - radar.x/KM)**2 +
                                                (pt[1]/KM - radar.y/KM)**2 +
                                                (pt[2]/KM)**2) - R_radar
                                        for pt in path]]) + 5)

# 总标题
fig.suptitle('轨迹-雷达交互分析',
            fontsize=16, fontweight='bold', y=0.98)

plt.tight_layout(rect=[0, 0, 1, 0.96])

output_path = "output_5zones/radar_interaction_analysis_sci.png"
plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
plt.close()

print(f"\n✓ SCI quality analysis figure saved: {output_path}")
print("=" * 80)
