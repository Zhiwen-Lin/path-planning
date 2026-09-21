#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
analyze_altitude_strategy_v2.py - 高度策略分析（高质量重构版）
================================================================
基于 72 组核心数据，分析 3D 航迹规划的高度策略（权重 → 高度选择）

数据说明（重要）:
  扫描表仅含【最大高度】两列：最大高度_km（第9列）与最大高度_m（第17列）。
  没有"平均高度"列，因此本图不使用平均高度指标。

生成图表（1×3布局）：
  (a) 最大高度分布热力图（权重配置 vs 高度选择）
  (b) 高度策略的燃油代价（最大高度-燃油散点）
  (c) 高度-暴露相关性（散点 + 拟合线）

修订记录:
  2026-09-17 修复数据列标签错位：
    - 原 (a) 标注"平均高度"实为最大高度（数值正确、标签错误）→ 已改名
    - 原 (b) 对已是千米的列又除以 1000，全图≈0.01 失效 → 该面板改为燃油代价散点
    - 原 (c) 横轴"平均飞行高度"→ 最大飞行高度
"""

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
import os, warnings
from analysis_data_loader import load_sweep_dataframe, analysis_output_dir
warnings.filterwarnings('ignore')

# ── 路径配置 ──────────────────────────────────────────────────────────────────
# 默认只读取当前版本 sweep 工作簿；也可用环境变量
# SWEEP_FILE / ANALYSIS_OUTDIR 覆盖，不再绑定作者本机 D: 盘。
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

# ── 配色 ──────────────────────────────────────────────────────────────────────
COLORS_6 = ['#e74c3c', '#e67e22', '#f39c12', '#27ae60', '#3498db', '#9b59b6']

print("=" * 80)
print("  高度策略分析（v2 高质量版）")
print("=" * 80)

# ── 读取数据 ──────────────────────────────────────────────────────────────────
print("\n[1/2] 读取数据...")
df, DATA = load_sweep_dataframe()

# 最大高度按列名读取；共享 loader 已检查完整性、成功状态及来源签名。
df['alt_km'] = df['altm'] / 1000.0
if not np.allclose(df['alt_km'], df['altkm'], atol=1e-6, equal_nan=True):
    print("  ⚠ 警告：最大高度两列（km/m）不一致，请检查数据表！")
else:
    print("  ✓ 最大高度两列（km/m）一致")
df['C_val'] = df['C']
df['T_min'] = df['T'] / 60.0

print(f"  ✓ 读取 {len(df)} 组数据")
print(f"  最大高度范围: {df['alt_km'].min():.2f} ~ {df['alt_km'].max():.2f} km")

wt_vals = sorted(df['wt'].unique())
wf_vals = sorted(df['wf'].unique())

def make_pivot(col):
    pt = df.pivot_table(index='wf', columns='wt', values=col, aggfunc='mean')
    return pt.reindex(index=sorted(wf_vals, reverse=True), columns=wt_vals)

# ════════════════════════════════════════════════════════════════════════════════
# 图: 高度策略分析（1×3）
# ════════════════════════════════════════════════════════════════════════════════
print("\n[2/2] 生成高度策略分析图...")

fig, axes = plt.subplots(1, 3, figsize=(16, 5))

# 配色：高度越高，颜色越深
cmap_alt = LinearSegmentedColormap.from_list(
    'altitude', ['#ffffd9', '#edf8b1', '#c7e9b4', '#7fcdbb', '#41b6c4', '#1d91c0', '#225ea8', '#0c2c84'], N=256)

# ── (a) 最大高度热力图 ────────────────────────────────────────────────────────
ax = axes[0]
alt_mat = make_pivot('alt_km')
data = alt_mat.values.astype(float)
vmin, vmax = np.nanmin(data), np.nanmax(data)

im = ax.imshow(data, cmap=cmap_alt, aspect='auto', vmin=vmin, vmax=vmax, origin='upper')

for i in range(len(alt_mat.index)):
    for j in range(len(alt_mat.columns)):
        val = data[i, j]
        if np.isnan(val):
            continue
        norm_val = (val - vmin) / max(vmax - vmin, 1e-9)
        txt_color = 'white' if norm_val > 0.5 else '#1a1a1a'
        ax.text(j, i, f'{val:.2f}', ha='center', va='center',
               fontsize=7.5, color=txt_color, weight='bold', family='Arial')

ax.set_xticks(range(len(alt_mat.columns)))
ax.set_xticklabels([f'{v:.1f}' for v in alt_mat.columns], fontsize=10)
ax.set_xlabel('时间权重 $w_{time}$', fontsize=11, weight='bold')

ax.set_yticks(range(len(alt_mat.index)))
ax.set_yticklabels([f'{v:.1f}' for v in alt_mat.index], fontsize=10)
ax.set_ylabel('燃油权重 $w_{fuel}$', fontsize=11, weight='bold')

ax.set_xticks(np.arange(-0.5, len(alt_mat.columns), 1), minor=True)
ax.set_yticks(np.arange(-0.5, len(alt_mat.index), 1), minor=True)
ax.grid(which='minor', color='white', linewidth=1.2, alpha=0.8)
ax.tick_params(which='minor', size=0)

cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
cbar.set_label('最大高度 (千米)', fontsize=10, weight='bold')

ax.set_title('(a) 最大高度分布', fontsize=12, weight='bold', pad=10)

# ── (b) 高度策略的燃油代价（最大高度-燃油散点） ─────────────────────────────
ax = axes[1]
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf].sort_values('wt')
    ax.scatter(subset['alt_km'], subset['F'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=80, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.8)

ax.set_xlabel('最大高度 (千米)', fontsize=11, weight='bold')
ax.set_ylabel('燃油消耗 (千克)', fontsize=11, weight='bold')
ax.set_title('(b) 高度策略的燃油代价', fontsize=12, weight='bold', pad=10)
ax.grid(True, linestyle='--', alpha=0.4)

# 纵轴按本次实验数据自适应，不再沿用旧数据集的固定 6.8~19.8 kg。
fmin_plot, fmax_plot = float(df['F'].min()), float(df['F'].max())
margin = max((fmax_plot - fmin_plot) * 0.08, 0.2)
ax.set_ylim(fmin_plot - margin, fmax_plot + margin)

# ── (c) 高度-暴露相关性 ───────────────────────────────────────────────────────
ax = axes[2]
handles = []
for i, wf in enumerate(wf_vals):
    subset = df[df['wf'] == wf].sort_values('wt')
    h = ax.scatter(subset['alt_km'], subset['C_val'],
              label=f'$w_{{fuel}}$={wf:.1f}', s=80, alpha=0.75,
              color=COLORS_6[i], edgecolors='black', linewidth=0.8)
    handles.append(h)

ax.set_xlabel('最大飞行高度 (千米)', fontsize=11, weight='bold')
ax.set_ylabel('累积暴露 C (s)', fontsize=11, weight='bold')
ax.set_title('(c) 高度-暴露相关性', fontsize=12, weight='bold', pad=10)
ax.grid(True, linestyle='--', alpha=0.4)

# 拟合线
from scipy.stats import linregress
mask = ~np.isnan(df['alt_km']) & ~np.isnan(df['C_val'])
x_data = df.loc[mask, 'alt_km']
y_data = df.loc[mask, 'C_val']
slope, intercept, r_value, p_value, std_err = linregress(x_data, y_data)
x_fit = np.linspace(x_data.min(), x_data.max(), 100)
y_fit = slope * x_fit + intercept
h_fit, = ax.plot(x_fit, y_fit, 'r--', linewidth=2, alpha=0.7,
                 label=f'拟合线 ($R^2$={r_value**2:.3f})')
handles.append(h_fit)
ax.legend(handles=handles, loc='best', fontsize=9, framealpha=0.95, ncol=2)

plt.tight_layout(pad=1.5, w_pad=2.5)

output = os.path.join(OUTDIR, 'analysis_altitude_strategy.png')
fig.savefig(output, dpi=600, bbox_inches='tight', facecolor='white')
print(f"  ✓ 已保存: {output}")
plt.close()

# ── 统计摘要 ──────────────────────────────────────────────────────────────────
print("\n" + "=" * 80)
print("  统计摘要")
print("=" * 80)

print("\n【高度策略构成】")
n_ground = int((df['alt_km'] < 0.1).sum())
n_mid    = int(((df['alt_km'] >= 5) & (df['alt_km'] <= 7)).sum())
n_high   = int((df['alt_km'] > 7).sum())
print(f"  贴地 (0 km)     : {n_ground} 组")
print(f"  中高空 (5~7 km) : {n_mid} 组")
print(f"  高空 (>7 km)    : {n_high} 组")
print(f"  最大高度: 均值 {df['alt_km'].mean():.2f} km (标准差 {df['alt_km'].std():.2f} km)")

row_high = df[df['alt_km'] > 7]
for r in row_high.itertuples():
    print(f"  高空策略配置: (w_t={r.wt:.1f}, w_f={r.wf:.1f}) → 最大高度 {r.alt_km:.2f} km")

print("\n【高度-暴露相关性】")
print(f"  Pearson r = {r_value:.3f}")
print(f"  R² = {r_value**2:.3f}")
print(f"  斜率 = {slope:.3f} (高度每增加1km，暴露变化{slope:.2f})")

print("\n" + "=" * 80)
print("  完成！")
print(f"  - {output}")
print("=" * 80)
