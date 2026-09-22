"""Radar clearance and sampled penetration analysis for validated trajectories."""
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Patch
from matplotlib.lines import Line2D
from trajectory_data_loader import load_trajectory_data, trajectory_output_path
from radar_environment_5zones import RadarEnvironment5Zones
from environment_threat_model import EnvironmentThreatModel

COLOR_RADAR = '#FFB6C1'      # 浅粉色
COLOR_RADAR_EDGE = '#DC143C' # 深红色
COLOR_FIRE = '#FFA07A'       # 浅橙色
COLOR_FIRE_EDGE = '#FF4500'  # 橙红色
COLOR_PATH = '#00CED1'       # 青色
COLOR_START = '#32CD32'      # 亮绿色
COLOR_GOAL = '#FF1493'       # 深粉红

COLOR_PATH_THICK = '#2874a6'
COLOR_PENETRATION = '#f39c12'

def plot_clearance(data, env):
    path = np.asarray(data['path'])
    stats, start, goal = data['stats'], data['start'], data['goal']
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
        d = np.linalg.norm(path[i] - path[i-1])
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

    output_path = trajectory_output_path("radar_interaction_analysis_sci.png")
    plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

    print(f"\n✓ SCI quality analysis figure saved: {output_path}")
    print("=" * 80)


def plot_penetration(data, env, threat):
    path = np.asarray(data['path'])
    stats, start, goal = data['stats'], data['start'], data['goal']
    w_time, w_risk, w_fuel = data['w_time'], data['w_risk'], data['w_fuel']
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

    return info_text


def main(argv=None):
    parser = argparse.ArgumentParser(description='轨迹与雷达交互分析')
    parser.add_argument("--view", choices=('clearance', 'penetration', 'all'), default="all")
    args = parser.parse_args(argv)
    data = load_trajectory_data()
    env = RadarEnvironment5Zones()
    env.generate(seed=42)
    threat = EnvironmentThreatModel(env, model='power', power_n=4.0)
    with plt.rc_context({"font.sans-serif": ["SimHei", "Microsoft YaHei", "Arial Unicode MS"],
                         "axes.unicode_minus": False}):
        if args.view in ('clearance', "all"):
            plot_clearance(data, env)
        if args.view in ('penetration', "all"):
            plot_penetration(data, env, threat)


if __name__ == "__main__":
    main()
