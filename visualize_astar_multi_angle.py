#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
多角度3D轨迹可视化 - SCI论文质量版
==================================
4个不同视角，配色专业，适合一区期刊
"""

from trajectory_data_loader import load_trajectory_data, trajectory_output_path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
from astar_risk_planner import AStarRiskPlanner

# 优化配色（高对比度、易区分）
COLOR_RADAR = '#FFB6C1'       # 浅粉色（雷达区，柔和）
COLOR_RADAR_EDGE = '#DC143C'  # 深红色（边缘鲜明）
COLOR_FIRE = '#FFA07A'        # 浅橙色（火力区）
COLOR_FIRE_EDGE = '#FF4500'   # 橙红色（边缘鲜明）
COLOR_PATH = '#00CED1'        # 青色（轨迹，与威胁区对比强烈）
COLOR_START = '#32CD32'       # 亮绿色（起点）
COLOR_GOAL = '#FF1493'        # 深粉红（终点）

KM = 1000.0

print("=" * 80)
print("  生成SCI论文质量多角度3D图")
print("=" * 80)

# 生成环境
print("\n[1/4] 生成环境...")
env = RadarEnvironment5Zones()
env.generate(seed=42)

# 尝试加载已规划路径
data = load_trajectory_data()
path = np.asarray(data['path'])
stats, start, goal = data['stats'], data['start'], data['goal']
_w_time, _w_risk, _w_fuel = data['w_time'], data['w_risk'], data['w_fuel']

print(f"\n路径规划完成: {len(path)}个点")

# ============================================================================
# 绘制函数
# ============================================================================

def draw_3d_scene(ax, path, env, start, goal, elev, azim, title):
    """绘制专业3D场景"""
    m = env.map_size / KM
    R = env.radar_radius / KM

    # 坐标轴设置
    ax.set_xlim(0, m)
    ax.set_ylim(0, m)
    ax.set_zlim(0, 12)
    ax.set_xlabel('X (km)', fontsize=12, fontweight='normal', labelpad=10)
    ax.set_ylabel('Y (km)', fontsize=12, fontweight='normal', labelpad=10)
    ax.set_zlabel('高度 (km)', fontsize=12, fontweight='normal', labelpad=10)

    # 白色背景
    ax.xaxis.pane.fill = True
    ax.yaxis.pane.fill = True
    ax.zaxis.pane.fill = True
    ax.xaxis.pane.set_facecolor('#FFFFFF')
    ax.yaxis.pane.set_facecolor('#FFFFFF')
    ax.zaxis.pane.set_facecolor('#FFFFFF')
    ax.xaxis.pane.set_edgecolor('#CCCCCC')
    ax.yaxis.pane.set_edgecolor('#CCCCCC')
    ax.zaxis.pane.set_edgecolor('#CCCCCC')
    ax.xaxis.pane.set_alpha(1.0)
    ax.yaxis.pane.set_alpha(1.0)
    ax.zaxis.pane.set_alpha(1.0)

    # 精细网格
    ax.grid(True, alpha=0.25, linestyle='-', linewidth=0.6, color='#AAAAAA')

    # 绘制半球形雷达区（更精细、更透明）
    for rz in env.radar_zones:
        cx, cy = rz.x / KM, rz.y / KM

        # 半球面（高精度）
        u = np.linspace(0, np.pi / 2, 25)
        v = np.linspace(0, 2 * np.pi, 40)
        U, V = np.meshgrid(u, v)
        X = cx + R * np.sin(U) * np.cos(V)
        Y = cy + R * np.sin(U) * np.sin(V)
        Z = R * np.cos(U)

        ax.plot_surface(X, Y, Z,
                       color=COLOR_RADAR,
                       alpha=0.12,
                       edgecolor='none',
                       antialiased=True,
                       shade=True)

        # 底面圆（加粗清晰）
        theta = np.linspace(0, 2 * np.pi, 80)
        xc = cx + R * np.cos(theta)
        yc = cy + R * np.sin(theta)
        ax.plot(xc, yc, np.zeros_like(theta),
               color=COLOR_RADAR_EDGE,
               linewidth=2.0,
               alpha=0.7)

    # 绘制火力区（简化为底面圆环）
    for fz in env.fire_zones:
        cx, cy = fz.x / KM, fz.y / KM
        fire_r = fz.horizontal_radius / KM

        theta = np.linspace(0, 2 * np.pi, 80)
        xc = cx + fire_r * np.cos(theta)
        yc = cy + fire_r * np.sin(theta)
        ax.plot(xc, yc, np.zeros_like(theta),
               color=COLOR_FIRE,
               linewidth=3.5,
               alpha=0.8)

    # 绘制轨迹（粗实线，加强对比，光滑后不显示航点）
    ax.plot(path[:, 0]/KM, path[:, 1]/KM, path[:, 2]/KM,
           color=COLOR_PATH, linewidth=4.5,
           zorder=10, alpha=1.0, solid_capstyle='round')

    # 起点和终点（大标记，黑色轮廓）
    ax.scatter([start[0]/KM], [start[1]/KM], [start[2]/KM],
              c=COLOR_START, s=400, marker='^',
              edgecolors='black', linewidths=2.5,
              zorder=12)
    ax.scatter([goal[0]/KM], [goal[1]/KM], [goal[2]/KM],
              c=COLOR_GOAL, s=400, marker='v',
              edgecolors='black', linewidths=2.5,
              zorder=12)

    # 标题（不加粗，更专业）
    ax.set_title(title, fontsize=13, fontweight='normal', pad=15)

    # 刻度设置
    ax.tick_params(labelsize=10)

    # 视角
    ax.view_init(elev=elev, azim=azim)

# ============================================================================
# 生成4个视角
# ============================================================================
print("\n生成多角度视图...")

fig = plt.figure(figsize=(20, 15))
fig.patch.set_facecolor('white')

# 视角1：右前视图
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
draw_3d_scene(ax1, path, env, start, goal,
             elev=18, azim=45, title='(a) 右前视图')

# 视角2：右侧视图
ax2 = fig.add_subplot(2, 2, 2, projection='3d')
draw_3d_scene(ax2, path, env, start, goal,
             elev=22, azim=90, title='(b) 右侧视图')

# 视角3：俯视图
ax3 = fig.add_subplot(2, 2, 3, projection='3d')
draw_3d_scene(ax3, path, env, start, goal,
             elev=75, azim=45, title='(c) 俯视图')

# 视角4：左后视图
ax4 = fig.add_subplot(2, 2, 4, projection='3d')
draw_3d_scene(ax4, path, env, start, goal,
             elev=18, azim=225, title='(d) 左后视图')

# 总标题
fig.suptitle('多角度三维轨迹可视化',
            fontsize=16, fontweight='bold', y=0.98)

# 图例（单独添加在右上角）
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

legend_elements = [
    Line2D([0], [0], color=COLOR_PATH, linewidth=3, label='A* 轨迹'),
    Line2D([0], [0], marker='^', color='w', markerfacecolor=COLOR_START,
           markersize=12, label='起点', markeredgewidth=0),
    Line2D([0], [0], marker='v', color='w', markerfacecolor=COLOR_GOAL,
           markersize=12, label='终点', markeredgewidth=0),
    Patch(facecolor=COLOR_RADAR, alpha=0.3, label='雷达区'),
    Line2D([0], [0], color=COLOR_FIRE, linewidth=3, label='火力区（禁飞区）'),
]

fig.legend(handles=legend_elements, loc='upper right',
          fontsize=11, framealpha=0.95, edgecolor='black',
          bbox_to_anchor=(0.98, 0.95))

plt.tight_layout(rect=[0, 0, 1, 0.96])

output_path = trajectory_output_path("astar_3d_multi_view.png")
plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
plt.close()

print(f"\n✓ 多角度3D图已保存: {output_path}")
print("=" * 80)
