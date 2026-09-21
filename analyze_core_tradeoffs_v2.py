#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
analyze_core_tradeoffs_v2.py - 核心权衡分析（高质量重构版）
==============================================================
基于 72 组核心数据，生成三组核心分析图：

1. 绕行代价分析（4子图）
   - 绕行增加热力图
   - 绕行-暴露权衡
   - 绕行-时间关系
   - 绕行效率曲线

2. 目标相关性分析（6子图）
   - 相关系数矩阵
   - 时间-暴露散点
   - 时间-油耗散点
   - 暴露-油耗散点
   - 高度-暴露关系
   - 三目标权衡空间

3. 高度策略分析（3子图）
   - 高度利用热力图
   - 高度-权重关系
   - 最大高度分布
"""

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy import stats
import os, warnings
from analysis_data_loader import load_sweep_dataframe, analysis_output_dir
from select_representative_weights import select_representatives
warnings.filterwarnings('ignore')

# ── 路径配置 ──────────────────────────────────────────────────────────────────
OUTDIR = str(analysis_output_dir())

# ── 字体配置 ──────────────────────────────────────────────────────────────────
matplotlib.rcParams.update({
    'font.sans-serif'   : ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS'],
    'font.family'       : 'sans-serif',
    'axes.unicode_minus': False,
    'font.size'         : 10,
    'axes.titlesize'    : 12,
    'axes.labelsize'    : 11,
    'xtick.labelsize'   : 10,
    'ytick.labelsize'   : 10,
    'legend.fontsize'   : 9,
    'figure.dpi'        : 120,
    'savefig.dpi'       : 600,
    'axes.linewidth'    : 1.0,
    'grid.linewidth'    : 0.6,
    'grid.alpha'        : 0.4,
    'pdf.fonttype'      : 42,
    'ps.fonttype'       : 42,
})

# ── 配色方案 ──────────────────────────────────────────────────────────────────
COLORS_6 = ['#e74c3c', '#e67e22', '#f39c12', '#27ae60', '#3498db', '#9b59b6']

print("=" * 80)
print("  核心权衡分析（v2 高质量版）")
print("=" * 80)

# ── 读取数据 ──────────────────────────────────────────────────────────────────
print("\n[1/3] 读取数据...")
df, DATA = load_sweep_dataframe()

# 派生指标：altkm/altm 均表示最大高度，不再误标为平均高度。
df['T_min'] = df['T'] / 60.0
df['F_kg'] = df['F']
df['C_val'] = df['C']
df['alt_km'] = df['altkm']
df['altm_km'] = df['altm'] / 1000.0
df['detour_pct'] = df['detour']

# 直线距离
x0, y0 = 2.0, 2.0
x1, y1 = 98.0, 98.0
L_direct = np.sqrt((x1-x0)**2 + (y1-y0)**2)
df['L_direct'] = L_direct
df['detour_ratio'] = (df['L'] - L_direct) / L_direct * 100.0

print(f"  ✓ 读取 {len(df)} 组数据")
print(f"  w_time: {sorted(df['wt'].unique())}")
print(f"  w_fuel: {sorted(df['wf'].unique())}")

wt_vals = sorted(df['wt'].unique())
wf_vals = sorted(df['wf'].unique())

# ════════════════════════════════════════════════════════════════════════════════
# 图1: 绕行代价分析（2×2）
# ════════════════════════════════════════════════════════════════════════════════
print("\n[2/3] 生成绕行代价分析...")

fig, axes = plt.subplots(2, 2, figsize=(14, 11))

# 1.1 绕行增加热力图
ax = axes[0, 0]
pivot_detour = df.pivot_table(index='wf', columns='wt', values='detour_ratio', aggfunc='mean')
pivot_detour = pivot_detour.reindex(index=sorted(wf_vals, reverse=True), columns=wt_vals)

cmap_detour = LinearSegmentedColormap.from_list(
    'detour', ['#d4edda', '#fff3cd', '#f8d7da'], N=256)

im = ax.imshow(pivot_detour.values, cmap=cmap_detour, aspect='auto', origin='upper')

# 数值标注
for i in range(len(pivot_detour.index)):
    for j in range(len(pivot_detour.columns)):
        val = pivot_detour.values[i, j]
        if not np.isnan(val):
            vmin, vmax = np.nanmin(pivot_detour.values), np.nanmax(pivot_detour.values)
            norm_val = (val - vmin) / max(vmax - vmin, 1e-9)
            txt_color = 'black' if norm_val < 0.7 else 'white'
            ax.text(j, i, f'{val:.1f}', ha='center', va='center',
                   fontsize=8, color=txt_color, weight='bold', family='Arial')

ax.set_xticks(range(len(pivot_detour.columns)))
ax.set_xticklabels([f'{v:.1f}' for v in pivot_detour.columns], fontsize=10)
ax.set_xlabel('时间权重 $w_{time}$', fontsize=11, weight='bold')

ax.set_yticks(range(len(pivot_detour.index)))
ax.set_yticklabels([f'{v:.1f}' for v in pivot_detour.index], fontsize=10)
ax.set_ylabel('燃油权重 $w_{fuel}$', fontsize=11, weight='bold')

ax.set_title('(a) 绕行增加百分比 (%)', fontsize=12, weight='bold', pad=10)

cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
cbar.set_label('绕行增加 (%)', fontsize=10)

# 1.2 绕行-暴露权衡
ax = axes[0, 1]
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf].sort_values('wt')
    ax.scatter(subset['detour_ratio'], subset['C_val'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=80, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.8)

ax.set_xlabel('绕行增加 (%)', fontsize=11, weight='bold')
ax.set_ylabel('累积暴露 C (s)', fontsize=11, weight='bold')
ax.set_title('(b) 绕行-暴露权衡曲线', fontsize=12, weight='bold', pad=10)
ax.legend(loc='best', fontsize=9, framealpha=0.95, ncol=2)
ax.grid(True, linestyle='--', alpha=0.4)

# 1.3 绕行-时间关系
ax = axes[1, 0]
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf].sort_values('wt')
    ax.scatter(subset['detour_ratio'], subset['T_min'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=80, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.8)

ax.set_xlabel('绕行增加 (%)', fontsize=11, weight='bold')
ax.set_ylabel('飞行时间 (分钟)', fontsize=11, weight='bold')
ax.set_title('(c) 绕行-时间关系', fontsize=12, weight='bold', pad=10)
ax.legend(loc='best', fontsize=9, framealpha=0.95, ncol=2)
ax.grid(True, linestyle='--', alpha=0.4)

# 1.4 绕行效率曲线
ax = axes[1, 1]
# 计算绕行效率：每增加1%绕行，累积暴露降低多少
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf].sort_values('wt')
    if len(subset) > 1:
        # 对每个 w_time，计算相对于 w_time=0 的变化
        base_detour = subset.iloc[0]['detour_ratio']
        base_exposure = subset.iloc[0]['C_val']

        delta_detour = subset['detour_ratio'] - base_detour
        delta_exposure = base_exposure - subset['C_val']  # 暴露降低（正值=好）

        # 效率 = 暴露降低 / 绕行增加
        efficiency = delta_exposure / (delta_detour + 1e-6)

        ax.plot(subset['wt'], efficiency,
               label=f'$w_{{fuel}}$={wf:.1f}', marker='o', linewidth=2,
               color=COLORS_6[i], markersize=6, alpha=0.85)

ax.set_xlabel('时间权重 $w_{time}$', fontsize=11, weight='bold')
ax.set_ylabel('绕行效率 (暴露降低/绕行增加)', fontsize=11, weight='bold')
ax.set_title('(d) 绕行效率曲线', fontsize=12, weight='bold', pad=10)
ax.legend(loc='best', fontsize=9, framealpha=0.95, ncol=2)
ax.grid(True, linestyle='--', alpha=0.4)
ax.axhline(y=0, color='red', linestyle='--', alpha=0.6, linewidth=1.5)

plt.tight_layout(pad=1.5)

output1 = os.path.join(OUTDIR, 'analysis_detour_tradeoff.png')
fig.savefig(output1, dpi=600, bbox_inches='tight', facecolor='white')
print(f"  ✓ 已保存: {output1}")
plt.close()

# ════════════════════════════════════════════════════════════════════════════════
# 图2: 目标相关性分析（3×2布局）
# ════════════════════════════════════════════════════════════════════════════════
print("\n[3/3] 生成目标相关性分析...")

fig = plt.figure(figsize=(16, 13))
gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.35)

# 2.1 相关系数矩阵（左上，2×2）
ax = fig.add_subplot(gs[0:2, 0:2])

metrics = ['T_min', 'C_val', 'F_kg', 'detour_ratio', 'altm_km']
metric_names = ['飞行时间\n(min)', '累积暴露 C (s)', '燃油消耗\n(kg)', '绕行增加\n(%)', '最大高度\n(km)']

corr_matrix = df[metrics].corr()

cmap_corr = LinearSegmentedColormap.from_list(
    'correlation', ['#d73027', '#f46d43', '#fdae61', '#fee090', '#ffffff',
                    '#e0f3f8', '#abd9e9', '#74add1', '#4575b4'], N=256)

im = ax.imshow(corr_matrix, cmap=cmap_corr, aspect='auto',
              vmin=-1, vmax=1, interpolation='nearest')

ax.set_xticks(np.arange(len(metrics)))
ax.set_yticks(np.arange(len(metrics)))
ax.set_xticklabels(metric_names, fontsize=10)
ax.set_yticklabels(metric_names, fontsize=10)
ax.set_title('(a) 指标相关系数矩阵', fontsize=12, weight='bold', pad=15)

# 添加数值
for i in range(len(metrics)):
    for j in range(len(metrics)):
        val = corr_matrix.iloc[i, j]
        txt_color = 'white' if abs(val) > 0.6 else 'black'
        ax.text(j, i, f'{val:.2f}', ha='center', va='center',
               color=txt_color, fontsize=10, weight='bold')

cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
cbar.set_label('Pearson 相关系数', fontsize=10)

# 2.2 时间-暴露散点
ax = fig.add_subplot(gs[0, 2])
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf]
    ax.scatter(subset['T_min'], subset['C_val'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=60, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.7)

r, p = stats.pearsonr(df['T_min'].dropna(), df['C_val'].dropna())
ax.text(0.05, 0.95, f'$r={r:.3f}$', transform=ax.transAxes,
       fontsize=10, verticalalignment='top', weight='bold',
       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.85))

ax.set_xlabel('飞行时间 (分钟)', fontsize=11, weight='bold')
ax.set_ylabel('累积暴露 C (s)', fontsize=11, weight='bold')
ax.set_title('(b) 时间-暴露关系', fontsize=11, weight='bold', pad=8)
ax.legend(loc='best', fontsize=8, framealpha=0.9, ncol=2)
ax.grid(True, linestyle='--', alpha=0.4)

# 2.3 时间-油耗散点
ax = fig.add_subplot(gs[1, 2])
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf]
    ax.scatter(subset['T_min'], subset['F_kg'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=60, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.7)

r, p = stats.pearsonr(df['T_min'].dropna(), df['F_kg'].dropna())
ax.text(0.05, 0.95, f'$r={r:.3f}$', transform=ax.transAxes,
       fontsize=10, verticalalignment='top', weight='bold',
       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.85))

ax.set_xlabel('飞行时间 (分钟)', fontsize=11, weight='bold')
ax.set_ylabel('燃油消耗 (千克)', fontsize=11, weight='bold')
ax.set_title('(c) 时间-油耗关系', fontsize=11, weight='bold', pad=8)
ax.legend(loc='best', fontsize=8, framealpha=0.9, ncol=2)
ax.grid(True, linestyle='--', alpha=0.4)

# 2.4 暴露-油耗散点
ax = fig.add_subplot(gs[2, 0])
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf]
    ax.scatter(subset['C_val'], subset['F_kg'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=60, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.7)

r, p = stats.pearsonr(df['C_val'].dropna(), df['F_kg'].dropna())
ax.text(0.05, 0.95, f'$r={r:.3f}$', transform=ax.transAxes,
       fontsize=10, verticalalignment='top', weight='bold',
       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.85))

ax.set_xlabel('累积暴露 C (s)', fontsize=11, weight='bold')
ax.set_ylabel('燃油消耗 (千克)', fontsize=11, weight='bold')
ax.set_title('(d) 暴露-油耗关系', fontsize=11, weight='bold', pad=8)
ax.legend(loc='best', fontsize=8, framealpha=0.9, ncol=2)
ax.grid(True, linestyle='--', alpha=0.4)

# 2.5 高度-暴露关系
ax = fig.add_subplot(gs[2, 1])
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf]
    ax.scatter(subset['altm_km'], subset['C_val'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=60, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.7)

r, p = stats.pearsonr(df['altm_km'].dropna(), df['C_val'].dropna())
ax.text(0.05, 0.95, f'$r={r:.3f}$', transform=ax.transAxes,
       fontsize=10, verticalalignment='top', weight='bold',
       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.85))

ax.set_xlabel('最大高度 (千米)', fontsize=11, weight='bold')
ax.set_ylabel('累积暴露 C (s)', fontsize=11, weight='bold')
ax.set_title('(e) 高度-暴露关系', fontsize=11, weight='bold', pad=8)
ax.legend(loc='best', fontsize=8, framealpha=0.9, ncol=2)
ax.grid(True, linestyle='--', alpha=0.4)

# 2.6 三目标归一化对比（平行坐标图简化版）
ax = fig.add_subplot(gs[2, 2])

# 归一化
objectives = df[['T', 'C', 'F']]
spans = objectives.max() - objectives.min()
plot_scale = spans.where(spans > 0, 1.0)
representatives = select_representatives(df)

x_pos = [0, 1, 2]
x_labels = ['时间', '暴露', '油耗']

for name in ('绕飞', '平衡', '快穿'):
    row = representatives[name]
    values = (row[['T', 'C', 'F']].astype(float) - objectives.min()) / plot_scale
    label = f"{name}\n({row['wt']:.1f}, {row['wf']:.1f})"
    ax.plot(x_pos, values, marker='o', linewidth=2.5,
            markersize=8, label=label, alpha=0.85)

ax.set_xticks(x_pos)
ax.set_xticklabels(x_labels, fontsize=11, weight='bold')
ax.set_ylabel('本批数据 min-max 缩放 (0-1)', fontsize=11, weight='bold')
ax.set_title('(f) 典型配置的三目标权衡', fontsize=11, weight='bold', pad=8)
ax.legend(loc='best', fontsize=9, framealpha=0.95)
ax.grid(True, linestyle='--', alpha=0.4, axis='y')
ax.set_ylim([-0.05, 1.05])

plt.tight_layout()

output2 = os.path.join(OUTDIR, 'analysis_objective_correlation.png')
fig.savefig(output2, dpi=600, bbox_inches='tight', facecolor='white')
print(f"  ✓ 已保存: {output2}")
plt.close()

# ── 统计摘要 ──────────────────────────────────────────────────────────────────
print("\n" + "=" * 80)
print("  统计摘要")
print("=" * 80)

print("\n【绕行分析】")
print(f"  绕行范围: {df['detour_ratio'].min():.1f}% ~ {df['detour_ratio'].max():.1f}%")
print(f"  直线距离: {L_direct:.2f} km")

print("\n【相关性分析】（前3对）")
pairs = [('T_min', 'C_val', '时间-暴露'),
         ('T_min', 'F_kg', '时间-油耗'),
         ('C_val', 'F_kg', '暴露-油耗')]
for col1, col2, name in pairs:
    r, p = stats.pearsonr(df[col1].dropna(), df[col2].dropna())
    print(f"  {name}: r={r:.3f}, p={p:.4f}")

print("\n" + "=" * 80)
print("  完成！")
print(f"  - {output1}")
print(f"  - {output2}")
print("=" * 80)
