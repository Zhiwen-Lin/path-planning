"""Speed, acceleration and turn-radius diagnostics using saved final profiles."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from fixed_wing_dynamics import FixedWingDynamics
from trajectory_data_loader import load_trajectory_data, trajectory_output_path

DS_PLOT = 100.0

def _deduplicate_polyline(path):
    P = np.asarray(path, dtype=float)
    if len(P) <= 1:
        return P

    keep = [0]
    for i in range(1, len(P)):
        if np.linalg.norm(P[i] - P[keep[-1]]) > 1e-9:
            keep.append(i)
    return P[keep]


def _densify_polyline_preserve_vertex_curvature(path, dyn, ds_target=100.0):
    """
    Densify a raw A* polyline without changing its curvature proxy.

    The old script first interpolated to 100 m and THEN applied Menger
    curvature. At a sharp polyline corner, that makes the radius depend on
    interpolation spacing and creates artificial stall-speed dips.

    Here:
      1) Menger curvature is computed on ORIGINAL A* vertices.
      2) The polyline is densified for forward/backward acceleration planning.
      3) Only the original vertices carry the original curvature constraints.
    """
    P = _deduplicate_polyline(path)
    n = len(P)

    if n < 2:
        return P, np.zeros(n)

    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s_orig = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s_orig[-1])

    k_orig = np.zeros(n)
    for i in range(1, n - 1):
        R = dyn.get_path_curvature_radius(P[i - 1], P[i], P[i + 1])
        if 1e-6 < R < float("inf"):
            k_orig[i] = 1.0 / R

    # Include every original vertex exactly, plus a regular arclength grid.
    regular = np.arange(0.0, total, max(ds_target, 1e-6))
    s_dense = np.unique(np.concatenate([regular, s_orig, [total]]))

    dense = np.column_stack([
        np.interp(s_dense, s_orig, P[:, j]) for j in range(3)
    ])

    k_dense = np.zeros(len(s_dense))
    for i in range(1, n - 1):
        idx = int(np.searchsorted(s_dense, s_orig[i]))
        if idx < len(s_dense) and abs(s_dense[idx] - s_orig[i]) < 1e-6:
            k_dense[idx] = k_orig[i]

    return dense, k_dense


def _profile_time(path, speeds):
    P = np.asarray(path, dtype=float)
    ds = np.linalg.norm(np.diff(P, axis=0), axis=1)
    v_avg = np.maximum((speeds[:-1] + speeds[1:]) / 2.0, 1e-3)
    return float(np.sum(ds / v_avg))


def build_profiles(coarse_data, fine_data, dyn):
    profiles = {}

    # 1) Coarse raw
    coarse_dense, coarse_k = _densify_polyline_preserve_vertex_curvature(
        coarse_data["path"], dyn, DS_PLOT
    )
    s, v, k, a = dyn.compute_speed_profile(
        coarse_dense,
        curvatures_in=coarse_k,
        strict_feasibility=False
    )
    profiles["Coarse A*"] = {
        "path": coarse_dense, "s": s, "v": v, "k": k, "a": a,
        "physical_label": "polyline proxy"
    }

    is_sweep = fine_data.get('source_kind') == 'sweep'
    # main.py has a fine search; the coarse-only sweep has two stages to plot.
    if not is_sweep:
        fine_dense, fine_k = _densify_polyline_preserve_vertex_curvature(
            fine_data['path_raw'], dyn, DS_PLOT)
        s, v, k, a = dyn.compute_speed_profile(
            fine_dense, curvatures_in=fine_k, strict_feasibility=False)
        profiles["Fine A* raw"] = {
            "path": fine_dense, "s": s, "v": v, "k": k, "a": a,
            "physical_label": "polyline proxy"
        }

    # 3) Fine + B-spline
    tck = fine_data.get("smooth_result", {}).get("tck", None)
    if tck is None:
        raise RuntimeError(
            "Fine result does not contain a B-spline tck. "
            "Run the revised main.py first."
        )

    # Final figures use exactly the samples and speeds used for saved T/C/F.
    smooth_dense = np.asarray(fine_data['path'])
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(smooth_dense, axis=0), axis=1))]
    v = np.asarray(fine_data['speed_profile'])
    k = np.asarray(fine_data['curvature_smooth'])
    a = np.asarray(fine_data['accel_profile'])
    dyn.validate_speed_profile(smooth_dense, v, k)
    final_label = 'Coarse + B-spline' if is_sweep else 'Fine + B-spline'
    profiles[final_label] = {
        "path": smooth_dense, "s": s, "v": v, "k": k, "a": a,
        "physical_label": "final physical profile"
    }

    return profiles


def plot_speed_profiles(fine_data, dyn):
    if fine_data['source_kind'] == 'sweep':
        coarse_data = {'path': fine_data['path_raw']}
    else:
        coarse_data = load_trajectory_data(Path(fine_data['source_file']).with_name('planned_path_coarse.pkl'))
        for key in ('trajectory_signature', 'w_time', 'w_risk', 'w_fuel', 'start', 'goal'):
            if not np.array_equal(coarse_data[key], fine_data[key]):
                raise ValueError(f'Coarse and fine caches belong to different experiments: {key}')

    profiles = build_profiles(coarse_data, fine_data, dyn)

    fig, axes = plt.subplots(3, 1, figsize=(14, 11))
    fig.patch.set_facecolor("white")

    # --------------------------------------------------------------
    # (a) Speed
    # --------------------------------------------------------------
    ax = axes[0]
    for name, p in profiles.items():
        ax.plot(p["s"] / 1000.0, p["v"] * 3.6, linewidth=2.2, label=name)

    ax.axhline(dyn.cruise_speed * 3.6, linestyle="--", linewidth=1.8,
               label=f"Cruise ({dyn.cruise_speed * 3.6:.0f} km/h)")
    ax.axhline(dyn.v_stall * 3.6, linestyle=":", linewidth=1.8,
               label=f"Stall ({dyn.v_stall * 3.6:.0f} km/h)")
    ax.set_ylabel("Speed (km/h)")
    ax.set_title("(a) Speed profile comparison")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.legend(ncol=2)

    # --------------------------------------------------------------
    # (b) Tangential acceleration
    # --------------------------------------------------------------
    ax = axes[1]
    for name, p in profiles.items():
        ax.plot(p["s"] / 1000.0, p["a"], linewidth=1.8, label=name)

    ax.axhline(dyn.max_horizontal_accel, linestyle="--", linewidth=1.5,
               label=f"+a limit ({dyn.max_horizontal_accel:.1f} m/s²)")
    ax.axhline(-dyn.max_decel, linestyle="--", linewidth=1.5,
               label=f"-a limit ({dyn.max_decel:.1f} m/s²)")
    ax.axhline(0.0, linewidth=1.0)
    ax.set_ylabel("Tangential acceleration (m/s²)")
    ax.set_title("(b) Forward-backward acceleration consistency")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.legend(ncol=2)

    # --------------------------------------------------------------
    # (c) Speed deficit from cruise -- makes differences visible
    # --------------------------------------------------------------
    ax = axes[2]
    for name, p in profiles.items():
        deficit = np.maximum(dyn.cruise_speed - p["v"], 0.0) * 3.6
        ax.plot(p["s"] / 1000.0, deficit, linewidth=2.0, label=name)

    ax.set_xlabel("Cumulative distance (km)")
    ax.set_ylabel("Cruise-speed deficit (km/h)")
    ax.set_title("(c) Where geometry / vertical-rate constraints force slowing")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.legend()

    fig.suptitle(
        "Speed Planning Comparison — " + fine_data['source_kind'],
        fontsize=15, fontweight="bold"
    )
    plt.tight_layout(rect=[0, 0, 1, 0.97])

    output = trajectory_output_path('speed_profile_three_way_english.png')
    plt.savefig(output, dpi=300, bbox_inches="tight", facecolor="white")
    plt.close(fig)

    # Console summary
    print("\n" + "=" * 92)
    print("Speed-profile summary")
    print("=" * 92)
    print(
        f"{'Stage':22s} {'N':>7s} {'L(km)':>9s} {'T(s)':>9s} "
        f"{'Vmin*':>9s} {'Vmean':>9s} {'|a|max':>9s}"
    )

    for name, p in profiles.items():
        L = p["s"][-1] if len(p["s"]) else 0.0
        T = _profile_time(p["path"], p["v"])
        vmin = float(np.min(p["v"]))
        vmaxa = float(np.max(np.abs(p["a"]))) if len(p["a"]) else 0.0
        print(
            f"{name:22s} {len(p['s']):7d} {L/1000:9.2f} {T:9.1f} "
            f"{vmin*3.6:9.1f} {np.mean(p['v'])*3.6:9.1f} {vmaxa:9.2f}"
        )

    print("\n* Vmin includes every point; the airborne start speed is the stall speed.")
    print("  Raw A* curves are polyline proxies; the B-spline curve is the final")
    print("  physically validated profile using analytic curvature.")
    print(f"\nSaved: {output}")


def compute_turn_radii(path, dynamics):
    """
    折线路径沿程转弯半径（三点 Menger 曲率）

    仅适用于折线。对 B 样条采样点用本函数会得到随点距发散的半径，
    光滑轨迹使用缓存中的解析曲率。
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




def plot_turn_radius(data, dynamics):
    path_smooth = np.array(data['path'])           # 光滑后路径
    path_raw = np.array(data.get('path_raw', []))  # 原始细网格路径

    print(f"  光滑后路径: {len(path_smooth)} 个航点")
    if len(path_raw) > 0:
        print(f"  原始路径: {len(path_raw)} 个航点")
        has_raw = True
    else:
        print(f"  原始路径: 未保存（老版本 pkl）")
        has_raw = False


    # ============================================================================
    # 计算转弯半径
    # ============================================================================
    print("\n计算转弯半径...")

    # load_trajectory_data(allow_raw=False) already requires a validated spline.
    k = np.asarray(data['curvature_smooth'])
    radii_smooth = np.clip(1.0 / np.maximum(k, 1e-12), 0, 50000)
    dist_smooth = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path_smooth, axis=0), axis=1))] / 1000.
    # 原始路径（如果有）
    if has_raw and len(path_raw) >= 3:
        dist_raw, radii_raw = compute_turn_radii(path_raw, dynamics)
    else:
        dist_raw, radii_raw = np.array([]), np.array([])

    # 统计
    min_R_smooth = data['stats']['min_turn_radius']
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

    output_path = trajectory_output_path("turn_radius_analysis.png")
    plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

    print(f"\n✓ 转弯半径分析图已保存: {output_path}")
    print("=" * 80)


def main(argv=None):
    parser = argparse.ArgumentParser(description='速度、加速度及转弯半径分析')
    parser.add_argument("--view", choices=('speed', 'turn', 'all'), default="all")
    args = parser.parse_args(argv)
    data = load_trajectory_data(allow_raw=False)
    dyn = FixedWingDynamics()
    with plt.rc_context({"font.sans-serif": ["SimHei", "Microsoft YaHei", "Arial Unicode MS"],
                         "axes.unicode_minus": False}):
        if args.view in ('speed', "all"):
            plot_speed_profiles(data, dyn)
        if args.view in ('turn', "all"):
            plot_turn_radius(data, dyn)


if __name__ == "__main__":
    main()
