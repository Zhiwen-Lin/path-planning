#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
analyze_weight_sensitivity_v2.py - 权重敏感性分析（高质量重构版）
====================================================================
基于 72 组核心数据 (sweep_2d_time_fuel_20260812_182344.xlsx)

生成图表：
1. 权重敏感性曲线（固定 w_time，观察 w_fuel 影响）
2. 权重交互热力图（3个目标的等高线图）

设计标准：
- 全中文标注
- 高对比度专业配色
- 600 dpi 期刊质量
- 智能避免重叠
"""

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import os, warnings
from analysis_data_loader import load_sweep_dataframe, analysis_output_dir
warnings.filterwarnings('ignore')

# ── 路径配置 ──────────────────────────────────────────────────────────────────
OUTDIR = str(analysis_output_dir())

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
    'grid.linewidth'    : 0.6,
    'grid.alpha'        : 0.4,
    'pdf.fonttype'      : 42,
    'ps.fonttype'       : 42,
})

# ── 专业配色方案 ──────────────────────────────────────────────────────────────
# 12 条曲线的高辨识度配色（色盲友好）
COLORS_12 = [
    '#1f77b4',  # 蓝
    '#ff7f0e',  # 橙
    '#2ca02c',  # 绿
    '#d62728',  # 红
    '#9467bd',  # 紫
    '#8c564b',  # 棕
    '#e377c2',  # 粉
    '#7f7f7f',  # 灰
    '#bcbd22',  # 黄绿
    '#17becf',  # 青
    '#aec7e8',  # 浅蓝
    '#ff9896',  # 浅红
]

print("=" * 80)
print("  权重敏感性分析（v2 高质量版）")
print("=" * 80)

# ── 读取数据 ──────────────────────────────────────────────────────────────────
print("\n[1/2] 读取数据...")
df, DATA = load_sweep_dataframe()
df['T_min'] = df['T'] / 60.0
df['F_kg'] = df['F']
df['C_val'] = df['C']

print(f"  ✓ 读取 {len(df)} 组数据")
print(f"  w_time: {sorted(df['wt'].unique())}")
print(f"  w_fuel: {sorted(df['wf'].unique())}")

wt_vals = sorted(df['wt'].unique())
wf_vals = sorted(df['wf'].unique())

# ════════════════════════════════════════════════════════════════════════════════
# 图1: 权重敏感性曲线（3子图）
# ════════════════════════════════════════════════════════════════════════════════
print("\n[2/2] 生成权重敏感性曲线...")

fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))

metrics = [
    ('T_min', '飞行时间 (分钟)', axes[0]),
    ('C_val', '累积暴露', axes[1]),
    ('F_kg', '燃油消耗 (千克)', axes[2])
]

for metric_col, metric_name, ax in metrics:
    for i, wt in enumerate(wt_vals):
        subset = df[df['wt'] == wt].sort_values('wf')

        if len(subset) == 0:
            continue

        # 配色和线型
        color = COLORS_12[i % len(COLORS_12)]
        linestyle = '-' if i < 6 else '--'
        marker = 'o' if i % 3 == 0 else ('s' if i % 3 == 1 else '^')

        ax.plot(subset['wf'], subset[metric_col],
               label=f'$w_{{time}}$={wt:.1f}',
               color=color, linestyle=linestyle, marker=marker,
               markersize=5, linewidth=1.8, alpha=0.85)

    ax.set_xlabel('燃油权重 $w_{fuel}$', fontsize=12, weight='bold')
    ax.set_ylabel(metric_name, fontsize=12, weight='bold')
    ax.set_title(metric_name, fontsize=13, weight='bold', pad=10)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc='best', fontsize=8, framealpha=0.95, ncol=2)

    # 设置 x 轴刻度
    ax.set_xticks(wf_vals)
    ax.set_xticklabels([f'{v:.1f}' for v in wf_vals])
    ax.tick_params(labelsize=10)

plt.tight_layout(pad=1.5, w_pad=2.5)

output1 = os.path.join(OUTDIR, 'weight_sensitivity_curves.png')
fig.savefig(output1, dpi=600, bbox_inches='tight', facecolor='white')
print(f"  ✓ 已保存: {output1}")
plt.close()

# ════════════════════════════════════════════════════════════════════════════════
# 图2: 权重交互热力图（3子图，使用等高线增强可读性）
# ════════════════════════════════════════════════════════════════════════════════
print("\n[3/3] 生成权重交互热力图...")

def make_pivot(col):
    pt = df.pivot_table(index='wf', columns='wt', values=col, aggfunc='mean')
    return pt.reindex(index=sorted(df['wf'].unique(), reverse=True),
                      columns=sorted(df['wt'].unique()))

T_mat = make_pivot('T_min')
C_mat = make_pivot('C_val')
F_mat = make_pivot('F_kg')

# 高对比度配色
cmap_T = LinearSegmentedColormap.from_list(
    'time', ['#08519c', '#3182bd', '#6baed6', '#9ecae1', '#c6dbef', '#eff3ff'], N=256)

cmap_C = LinearSegmentedColormap.from_list(
    'exposure', ['#ffffd4', '#fee391', '#fec44f', '#fe9929', '#d95f0e', '#993404'], N=256)

cmap_F = LinearSegmentedColormap.from_list(
    'fuel', ['#006d2c', '#31a354', '#74c476', '#a1d99b', '#c7e9c0', '#edf8e9'], N=256)

fig, axes = plt.subplots(1, 3, figsize=(16, 5))

def draw_heatmap(ax, mat, cmap, title, unit, fmt):
    data = mat.values.astype(float)
    vmin, vmax = np.nanmin(data), np.nanmax(data)

    im = ax.imshow(data, cmap=cmap, aspect='auto',
                   vmin=vmin, vmax=vmax, origin='upper')

    # 数值标注
    for i in range(len(mat.index)):
        for j in range(len(mat.columns)):
            val = data[i, j]
            if np.isnan(val):
                continue

            norm_val = (val - vmin) / max(vmax - vmin, 1e-9)

            # 智能字色
            if title == '累积暴露':
                txt_color = 'white' if norm_val > 0.6 else '#1a1a1a'
            else:
                txt_color = 'white' if norm_val < 0.5 else '#1a1a1a'

            ax.text(j, i, fmt % val,
                    ha='center', va='center',
                    fontsize=7.5,
                    color=txt_color,
                    weight='bold',
                    family='Arial')

    # 坐标轴
    ax.set_xticks(range(len(mat.columns)))
    ax.set_xticklabels([f'{v:.1f}' for v in mat.columns], fontsize=10)
    ax.set_xlabel('时间权重 $w_{time}$', fontsize=11, weight='bold')

    ax.set_yticks(range(len(mat.index)))
    ax.set_yticklabels([f'{v:.1f}' for v in mat.index], fontsize=10)
    ax.set_ylabel('燃油权重 $w_{fuel}$', fontsize=11, weight='bold')

    # 网格
    ax.set_xticks(np.arange(-0.5, len(mat.columns), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(mat.index), 1), minor=True)
    ax.grid(which='minor', color='white', linewidth=1.2, alpha=0.8)
    ax.tick_params(which='minor', size=0)

    # Colorbar
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label(unit, fontsize=10, weight='bold')
    cbar.ax.tick_params(labelsize=9)

    # 标题
    ax.set_title(title, fontsize=13, weight='bold', pad=10)

draw_heatmap(axes[0], T_mat, cmap_T, '飞行时间', '分钟', '%.1f')
draw_heatmap(axes[1], C_mat, cmap_C, '累积暴露', '无量纲', '%.1f')
draw_heatmap(axes[2], F_mat, cmap_F, '燃油消耗', '千克', '%.2f')

plt.tight_layout(pad=1.5, w_pad=2.5)

output2 = os.path.join(OUTDIR, 'weight_interaction_heatmap.png')
fig.savefig(output2, dpi=600, bbox_inches='tight', facecolor='white')
print(f"  ✓ 已保存: {output2}")
plt.close()

# ── 统计摘要 ──────────────────────────────────────────────────────────────────
print("\n" + "=" * 80)
print("  统计摘要")
print("=" * 80)

print("\n【目标范围】")
print(f"  飞行时间: {df['T_min'].min():.1f} ~ {df['T_min'].max():.1f} 分钟")
print(f"  累积暴露: {df['C_val'].min():.1f} ~ {df['C_val'].max():.1f}")
print(f"  燃油消耗: {df['F_kg'].min():.2f} ~ {df['F_kg'].max():.2f} 千克")

print("\n【极值配置】")
for col, name in [('T_min', '飞行时间'), ('C_val', '累积暴露'), ('F_kg', '燃油消耗')]:
    min_row = df.loc[df[col].idxmin()]
    max_row = df.loc[df[col].idxmax()]
    print(f"\n  {name}:")
    print(f"    最小: {min_row[col]:.2f}  @ (w_t={min_row['wt']:.1f}, w_f={min_row['wf']:.1f})")
    print(f"    最大: {max_row[col]:.2f}  @ (w_t={max_row['wt']:.1f}, w_f={max_row['wf']:.1f})")

print("\n" + "=" * 80)
print("  完成！")
print(f"  - {output1}")
print(f"  - {output2}")
print("=" * 80)
