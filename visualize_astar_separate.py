#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A* 路径可视化 - 4张独立图片（优化配色版）
============================
1. 3D轨迹图（基于现有的半球雷达建模）
2. 高度剖面图
3. 俯视图
4. 高度变化分析图
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D
import pickle
import os

# 中文字体设置
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel
from fixed_wing_dynamics import FixedWingDynamics
from astar_risk_planner import AStarRiskPlanner

# 优化后的配色方案（高对比度、易区分）
COLOR_RADAR = '#FFB6C1'      # 浅粉色 - 雷达区（柔和，不刺眼）
COLOR_RADAR_EDGE = '#DC143C' # 深红色 - 雷达边缘（鲜明）
COLOR_FIRE = '#FFA07A'       # 浅橙色 - 火力区（与雷达区分开）
COLOR_FIRE_EDGE = '#FF4500'  # 橙红色 - 火力边缘（鲜明）
COLOR_PATH = '#00CED1'       # 青色 - 路径（与所有威胁区对比强烈）
COLOR_START = '#32CD32'      # 亮绿色 - 起点（醒目）
COLOR_GOAL = '#FF1493'       # 深粉红 - 终点（醒目）

KM = 1000.0

print("=" * 80)
print("  A* 路径可视化 - 生成4张独立图片")
print("=" * 80)

# 生成环境
print("\n[1/5] 生成环境...")
env = RadarEnvironment5Zones()
env.generate(seed=42)

# 尝试加载已规划路径
PICKLE_PATH = "output_5zones/planned_path.pkl"
if os.path.exists(PICKLE_PATH):
    print("[2/5] 加载已规划路径...")
    with open(PICKLE_PATH, 'rb') as f:
        data = pickle.load(f)
    path = np.array(data['path'])
    stats = data['stats']
    start = data['start']
    goal = data['goal']
    planner_L0 = data['L0']
    # 从 pickle 读取权重，与 main.py 保持一致
    _w_risk = data['w_risk']
    _w_time = data['w_time']
    _w_fuel = data['w_fuel']
    print(f"  ✓ 已加载: {len(path)} 个路径点, 航程 {stats['L']/KM:.1f} km")
else:
    print("[2/5] 未找到缓存，请先运行 main.py 生成 planned_path.pkl")
    import sys; sys.exit(1)

print("[5/5] 生成可视化...")

# ============================================================================
# 图1：3D轨迹图（基于半球雷达建模）
# ============================================================================
print("\n  [1/4] 生成3D轨迹图...")

fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

m = env.map_size / KM
R = env.radar_radius / KM

# 设置坐标轴
ax.set_xlim(-5, m + 5)
ax.set_ylim(-5, m + 5)
ax.set_zlim(0, 12)  # 高度0-12km（留余量）
ax.set_xlabel('X (km)', fontsize=13, fontweight='bold', labelpad=10)
ax.set_ylabel('Y (km)', fontsize=13, fontweight='bold', labelpad=10)
ax.set_zlabel('高度 (km)', fontsize=13, fontweight='bold', labelpad=10)

# 背景设置
ax.xaxis.pane.fill = True
ax.yaxis.pane.fill = True
ax.zaxis.pane.fill = True
ax.xaxis.pane.set_facecolor('white')
ax.yaxis.pane.set_facecolor('white')
ax.zaxis.pane.set_facecolor('white')
ax.xaxis.pane.set_edgecolor('#cccccc')
ax.yaxis.pane.set_edgecolor('#cccccc')
ax.zaxis.pane.set_edgecolor('#cccccc')
ax.xaxis.pane.set_alpha(0.1)
ax.yaxis.pane.set_alpha(0.1)
ax.zaxis.pane.set_alpha(0.1)
ax.grid(True, alpha=0.15, linestyle='-', linewidth=0.6, color='#999999')

# 绘制半球形雷达区
def draw_hemisphere(cx, cy, R):
    u = np.linspace(0, np.pi / 2, 30)
    v = np.linspace(0, 2 * np.pi, 50)
    U, V = np.meshgrid(u, v)
    X = cx + R * np.sin(U) * np.cos(V)
    Y = cy + R * np.sin(U) * np.sin(V)
    Z = R * np.cos(U)

    ax.plot_surface(X, Y, Z,
                   color=COLOR_RADAR,
                   alpha=0.25,
                   edgecolor='none',
                   antialiased=True,
                   shade=True,
                   zorder=3)

    # 底面圆
    theta = np.linspace(0, 2 * np.pi, 80)
    xc = cx + R * np.cos(theta)
    yc = cy + R * np.sin(theta)
    zc = np.zeros_like(theta)
    ax.plot(xc, yc, zc,
           color=COLOR_RADAR_EDGE,
           linewidth=1.5,
           alpha=0.8,
           zorder=4)

# 绘制所有雷达区
for rz in env.radar_zones:
    cx, cy = rz.x / KM, rz.y / KM
    draw_hemisphere(cx, cy, R)

# 绘制火力区（圆柱体）
for fz in env.fire_zones:
    cx, cy = fz.x / KM, fz.y / KM
    fire_r = fz.horizontal_radius / KM
    max_alt = fz.max_altitude / KM

    # 圆柱侧面
    theta = np.linspace(0, 2 * np.pi, 50)
    z_cyl = np.linspace(0, max_alt, 20)
    Theta, Z = np.meshgrid(theta, z_cyl)
    X = cx + fire_r * np.cos(Theta)
    Y = cy + fire_r * np.sin(Theta)

    ax.plot_surface(X, Y, Z,
                   color=COLOR_FIRE,
                   alpha=0.3,
                   edgecolor='none',
                   antialiased=True,
                   shade=True,
                   zorder=2)

    # 底面和顶面圆
    xc = cx + fire_r * np.cos(theta)
    yc = cy + fire_r * np.sin(theta)
    ax.plot(xc, yc, np.zeros_like(theta),
           color=COLOR_FIRE_EDGE, linewidth=1.5, alpha=0.8, zorder=2)
    ax.plot(xc, yc, np.ones_like(theta) * max_alt,
           color=COLOR_FIRE_EDGE, linewidth=1.5, alpha=0.8, zorder=2)

# 绘制轨迹（粗线，加强对比，光滑后不显示航点）
ax.plot(path[:, 0]/KM, path[:, 1]/KM, path[:, 2]/KM,
       color=COLOR_PATH, linewidth=4.5, label='A* 路径', zorder=10, alpha=1.0)

# 起点和终点（加大标记）
ax.scatter([start[0]/KM], [start[1]/KM], [start[2]/KM],
          c=COLOR_START, s=500, marker='^', edgecolors='black',
          linewidths=2.5, label='起点', zorder=12)
ax.scatter([goal[0]/KM], [goal[1]/KM], [goal[2]/KM],
          c=COLOR_GOAL, s=500, marker='v', edgecolors='black',
          linewidths=2.5, label='终点', zorder=12)

# 标题（中文）
ax.set_title(f'A* 三维轨迹\n'
            f'航点数: {len(path)} | 航程: {stats["L"]/KM:.1f} km | '
            f'累积风险: {stats["Crisk"]:.2f}',
            fontsize=14, fontweight='bold', pad=20)

# 图例
ax.legend(loc='upper left', fontsize=11, framealpha=0.9)

# 视角
ax.view_init(elev=20, azim=45)

plt.tight_layout()
plt.savefig("output_5zones/astar_3d_trajectory.png", dpi=300, bbox_inches='tight', facecolor='white')
plt.close()
print("    ✓ 已保存: output_5zones/astar_3d_trajectory.png")

# ============================================================================
# 图2：高度剖面图
# ============================================================================
print("  [2/4] 生成高度剖面图...")

fig, ax = plt.subplots(figsize=(12, 6))

# 计算累积距离
distances = [0]
for i in range(1, len(path)):
    d = np.linalg.norm(path[i, :2] - path[i-1, :2])
    distances.append(distances[-1] + d)
distances = np.array(distances) / KM

# 绘制高度剖面
ax.plot(distances, path[:, 2]/KM, '-', color=COLOR_PATH, linewidth=3,
       label='高度剖面')

# 高度范围参考
ax.axhspan(0, 0.5, alpha=0.15, color='#90EE90', label='低空区域')
ax.axhspan(10, 11, alpha=0.15, color='#FFB6C1', label='高空限制区')

# 标注关键点（减少数量）
key_indices = [0, len(path)//2, -1]  # 只标注起点、中点、终点
for idx in key_indices:
    d = distances[idx]
    h = path[idx, 2]/KM
    label_text = ['起点', '中点', '终点'][key_indices.index(idx)]
    ax.annotate(label_text, xy=(d, h), xytext=(d, h+0.5),
               fontsize=10, ha='center', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.4', facecolor='#FFEB3B',
                        alpha=0.8, edgecolor='black', linewidth=1.5))

ax.set_xlabel('累积距离 (km)', fontsize=12, fontweight='bold')
ax.set_ylabel('高度 (km)', fontsize=12, fontweight='bold')
ax.set_title(f'高度剖面\n总航程: {stats["L"]/KM:.2f} km',
            fontsize=13, fontweight='bold')
ax.grid(True, alpha=0.3, linestyle='--')
ax.set_ylim(0, 11)  # 0-11km
ax.legend(loc='upper right', fontsize=10)

plt.tight_layout()
plt.savefig("output_5zones/astar_altitude_profile.png", dpi=300, bbox_inches='tight', facecolor='white')
plt.close()
print("    ✓ 已保存: output_5zones/astar_altitude_profile.png")

# ============================================================================
# 图3：俯视图
# ============================================================================
print("  [3/4] 生成俯视图...")

fig, ax = plt.subplots(figsize=(10, 10))

# 雷达区
for rz in env.radar_zones:
    cx, cy = rz.x / KM, rz.y / KM
    circle = Circle((cx, cy), R,
                   facecolor=COLOR_RADAR,
                   edgecolor=COLOR_RADAR_EDGE,
                   alpha=0.3,
                   linewidth=2,
                   zorder=3)
    ax.add_patch(circle)

    # 雷达站标记
    ax.plot(cx, cy, '^', color=COLOR_RADAR_EDGE, markersize=10,
           markeredgecolor='white', markeredgewidth=1.5, zorder=6)
    ax.text(cx, cy + R + 2, rz.name, ha='center', fontsize=11,
           fontweight='bold', color=COLOR_RADAR_EDGE, zorder=7)

# 火力区
for fz in env.fire_zones:
    cx, cy = fz.x / KM, fz.y / KM
    fire_r = fz.horizontal_radius / KM
    circle = Circle((cx, cy), fire_r,
                   facecolor=COLOR_FIRE,
                   edgecolor=COLOR_FIRE_EDGE,
                   alpha=0.5,
                   linewidth=2.5,
                   zorder=4)
    ax.add_patch(circle)

# 轨迹（光滑后不显示航点）
ax.plot(path[:, 0]/KM, path[:, 1]/KM, '-', color=COLOR_PATH,
       linewidth=4, label='A* 路径', alpha=1.0, zorder=8)

# 起点和终点
ax.scatter([start[0]/KM], [start[1]/KM], c=COLOR_START, s=300,
          marker='^', edgecolors='black', linewidths=2.5,
          label='起点', zorder=10)
ax.scatter([goal[0]/KM], [goal[1]/KM], c=COLOR_GOAL, s=300,
          marker='v', edgecolors='black', linewidths=2.5,
          label='终点', zorder=10)

ax.set_xlabel('X (km)', fontsize=12, fontweight='bold')
ax.set_ylabel('Y (km)', fontsize=12, fontweight='bold')
ax.set_title(f'俯视图（二维投影）\n'
            f'{len(path)} 个航点 | 绕行增加: +{(stats["L"]/planner_L0-1)*100:.1f}%',
            fontsize=13, fontweight='bold')
ax.set_xlim(-5, m+5)
ax.set_ylim(-5, m+5)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3, linestyle='--')
ax.legend(loc='upper left', fontsize=11, framealpha=0.9)

plt.tight_layout()
plt.savefig("output_5zones/astar_topview.png", dpi=300, bbox_inches='tight', facecolor='white')
plt.close()
print("    ✓ 已保存: output_5zones/astar_topview.png")

# ============================================================================
# 图4：高度变化分析
# ============================================================================
print("  [4/4] 生成高度变化分析图...")

fig, ax = plt.subplots(figsize=(12, 6))

# 高度变化量
height_changes = []
for i in range(1, len(path)):
    dh = path[i, 2] - path[i-1, 2]
    height_changes.append(dh)

# 柱状图（优化配色，深色轮廓）
colors = ['#4CAF50' if dh >= 0 else '#FF5252' for dh in height_changes]
bars = ax.bar(range(len(height_changes)), np.array(height_changes)/1000,
             color=colors, alpha=0.8, edgecolor='#424242', linewidth=0.8)

ax.axhline(y=0, color='black', linestyle='-', linewidth=2, alpha=1.0)
ax.set_xlabel('航段序号', fontsize=12, fontweight='bold')
ax.set_ylabel('高度变化 (km)', fontsize=12, fontweight='bold')
ax.set_title('各航段高度变化', fontsize=13, fontweight='bold')
ax.grid(True, alpha=0.3, axis='y', linestyle='--')

# 统计信息
total_climb = sum([dh for dh in height_changes if dh > 0]) / 1000
total_descent = sum([-dh for dh in height_changes if dh < 0]) / 1000
max_climb = max(height_changes) / 1000
max_descent = min(height_changes) / 1000

info_text = f'航点数: {len(path)}\n'
info_text += f'总爬升: {total_climb:.2f} km\n'
info_text += f'总下降: {total_descent:.2f} km\n'
info_text += f'最大爬升: {max_climb:.2f} km\n'
info_text += f'最大下降: {max_descent:.2f} km'

ax.text(0.98, 0.98, info_text, transform=ax.transAxes,
       fontsize=11, verticalalignment='top', horizontalalignment='right',
       bbox=dict(boxstyle='round', facecolor='#FFEB3B', alpha=0.9, edgecolor='black', linewidth=1.5))

plt.tight_layout()
plt.savefig("output_5zones/astar_altitude_changes.png", dpi=300, bbox_inches='tight', facecolor='white')
plt.close()
print("    ✓ 已保存: output_5zones/astar_altitude_changes.png")

print("\n" + "=" * 80)
print("✓ 所有图片生成完成！")
print("=" * 80)
print("\n生成的图片：")
print("  1. output_5zones/astar_3d_trajectory.png - 3D轨迹图（半球雷达建模）")
print("  2. output_5zones/astar_altitude_profile.png - 高度剖面图")
print("  3. output_5zones/astar_topview.png - 俯视图")
print("  4. output_5zones/astar_altitude_changes.png - 高度变化分析")
