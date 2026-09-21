#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
plot_heatmap_3obj_v3.py  三目标热力图（中文版 + 高对比度）
====================================
改进：
1. 全中文标注（符合国防科大学报要求）
2. 高对比度配色（数值清晰可见）
3. 加大图幅和字号
4. 智能文字颜色（深浅背景自适应）
"""

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import os, warnings
from analysis_data_loader import load_sweep_dataframe, analysis_output_dir
warnings.filterwarnings('ignore')

# ── 路径 ──────────────────────────────────────────────────────────────────────
OUTDIR = str(analysis_output_dir())

# ── 字体设置 ─────────────────────────────────────────────────────────────────
matplotlib.rcParams.update({
    'font.sans-serif'   : ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans'],
    'font.family'       : 'sans-serif',
    'axes.unicode_minus': False,
    'font.size'         : 10,
    'axes.titlesize'    : 13,
    'axes.labelsize'    : 11,
    'xtick.labelsize'   : 10,
    'ytick.labelsize'   : 10,
    'legend.fontsize'   : 10,
    'figure.dpi'        : 120,
    'savefig.dpi'       : 600,
    'axes.linewidth'    : 1.0,
    'xtick.major.width' : 0.8,
    'ytick.major.width' : 0.8,
    'xtick.major.size'  : 4,
    'ytick.major.size'  : 4,
    'xtick.direction'   : 'out',
    'ytick.direction'   : 'out',
    'pdf.fonttype'      : 42,
    'ps.fonttype'       : 42,
})

# ── 读取数据 ──────────────────────────────────────────────────────────────────
df, DATA = load_sweep_dataframe()

wt_vals = sorted(df['wt'].unique())
wf_vals = sorted(df['wf'].unique(), reverse=True)

def make_pivot(col):
    pt = df.pivot_table(index='wf', columns='wt', values=col, aggfunc='mean')
    return pt.reindex(index=wf_vals, columns=wt_vals)

T_mat = make_pivot('T') / 60.0
C_mat = make_pivot('C')
F_mat = make_pivot('F')

# ── 高对比度配色（专业期刊风格）──────────────────────────────────────────────
# 配色原则：清晰 > 漂亮，数值必须可读

# 飞行时间：蓝色系（冷色调，科技感）
cmap_T = LinearSegmentedColormap.from_list(
    'time', ['#08519c', '#3182bd', '#6baed6', '#9ecae1', '#c6dbef', '#eff3ff'], N=256)

# 累积暴露：橙红色系（暖色调，警示感）
cmap_C = LinearSegmentedColormap.from_list(
    'exposure', ['#ffffd4', '#fee391', '#fec44f', '#fe9929', '#d95f0e', '#993404'], N=256)

# 燃油消耗：绿色系（中性色调）
cmap_F = LinearSegmentedColormap.from_list(
    'fuel', ['#006d2c', '#31a354', '#74c476', '#a1d99b', '#c7e9c0', '#edf8e9'], N=256)

# ── 图幅尺寸 ──────────────────────────────────────────────────────────────────
FIG_W = 16
FIG_H = 5

fig, axes = plt.subplots(1, 3, figsize=(FIG_W, FIG_H))

# ── 绘图函数（智能字色）──────────────────────────────────────────────────────
def draw_heatmap(ax, mat, cmap, title, unit, fmt):
    data = mat.values.astype(float)
    vmin, vmax = np.nanmin(data), np.nanmax(data)

    im = ax.imshow(data, cmap=cmap, aspect='auto',
                   vmin=vmin, vmax=vmax, origin='upper')

    # 数值标注（智能对比度）
    for i in range(len(wf_vals)):
        for j in range(len(wt_vals)):
            val = data[i, j]
            if np.isnan(val):
                continue

            # 归一化背景亮度
            norm_val = (val - vmin) / max(vmax - vmin, 1e-9)

            # 智能字色：根据背景明度选择黑/白
            # 蓝色和绿色色系：浅色=高值用黑字，深色=低值用白字
            # 橙黄色系：深色=高值用白字，浅色=低值用黑字
            if title == '累积暴露':
                txt_color = 'white' if norm_val > 0.6 else '#1a1a1a'
            else:
                txt_color = 'white' if norm_val < 0.5 else '#1a1a1a'

            ax.text(j, i, fmt % val,
                    ha='center', va='center',
                    fontsize=8,
                    color=txt_color,
                    weight='bold',
                    family='Arial')

    # 坐标轴
    ax.set_xticks(range(len(wt_vals)))
    ax.set_xticklabels([f'{v:.1f}' for v in wt_vals], fontsize=10)
    ax.set_xlabel('时间权重', fontsize=12, weight='bold')

    ax.set_yticks(range(len(wf_vals)))
    ax.set_yticklabels([f'{v:.1f}' for v in wf_vals], fontsize=10)
    ax.set_ylabel('燃油权重', fontsize=12, weight='bold')

    # 网格线
    ax.set_xticks(np.arange(-0.5, len(wt_vals), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(wf_vals), 1), minor=True)
    ax.grid(which='minor', color='white', linewidth=1.2, alpha=0.8)
    ax.tick_params(which='minor', size=0)

    # Colorbar
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label(unit, fontsize=11, weight='bold')
    cbar.ax.tick_params(labelsize=10)

    # 标题
    ax.set_title(title, fontsize=14, weight='bold', pad=12)

    return im

# ── 绘制三张子图 ─────────────────────────────────────────────────────────────
draw_heatmap(axes[0], T_mat, cmap_T,
             title='飞行时间',
             unit='分钟',
             fmt='%.1f')

draw_heatmap(axes[1], C_mat, cmap_C,
             title='累积暴露',
             unit='暴露时间 (s)',
             fmt='%.1f')

draw_heatmap(axes[2], F_mat, cmap_F,
             title='燃油消耗',
             unit='千克',
             fmt='%.2f')

# ── 布局与保存 ────────────────────────────────────────────────────────────────
plt.tight_layout(pad=1.8, w_pad=3.0)

output_path = os.path.join(OUTDIR, 'heatmap_3obj_v3.png')
fig.savefig(output_path, dpi=600, bbox_inches='tight',
            facecolor='white', edgecolor='none')

print(f'✓ 已保存: {output_path}')
print(f'  图幅: {FIG_W}" × {FIG_H}"')
print(f'  改进: 全中文标注 + 高对比度配色 + 智能字色')
print(f'  数据范围:')
print(f'    时间: {T_mat.values.min():.1f} ~ {T_mat.values.max():.1f} min')
print(f'    暴露: {C_mat.values.min():.1f} ~ {C_mat.values.max():.1f}')
print(f'    油耗: {F_mat.values.min():.2f} ~ {F_mat.values.max():.2f} kg')

# plt.show()  # 注释掉，避免阻塞命令行
print('\n提示：图片已保存，如需预览请手动打开 PNG 文件')
