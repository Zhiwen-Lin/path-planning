#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
select_representative_weights.py  代表性权重筛选工具
====================================================
从 72 组正交实验（sweep_2d）中筛选"代表性权重"，流程：

  1. 行为指纹聚类 : 把 72 组权重压缩为若干"典型行为"（相同指标=同一行为）
  2. 平台稳健度   : 统计每格同行为邻居数（Moore 邻域），高分=对权重微扰不敏感
  3. 分区推荐     : 绕飞区 / 平衡区 / 快穿区 / 省油区 各取平台最深点
  4. 输出         : 权重-行为地图 + 时间-暴露分布图 + 推荐表 CSV

判据（什么是"合理"的权重）：
  a. 行为典型——出现在成片的行为分区内（不是孤立点/伪影）
  b. 平台中心——邻居多为同行为，权重微扰不改变策略
  c. 物理可信——所有候选均保留固定的 w_risk=1，不把 (w_time,w_fuel)=(0,0) 误判为零目标
  d. 可辩场景——3 个解分别代表明确的目标偏好

用法:
  python select_representative_weights.py
"""

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import os
import warnings
from analysis_data_loader import load_sweep_dataframe, analysis_output_dir
warnings.filterwarnings('ignore')

# ── 路径 ──────────────────────────────────────────────────────────────────────
OUTDIR = str(analysis_output_dir())
BASE = os.path.dirname(os.path.abspath(__file__))

# ── 字体（与 v3 图统一） ──────────────────────────────────────────────────────
matplotlib.rcParams.update({
    'font.sans-serif'   : ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans'],
    'font.family'       : 'sans-serif',
    'axes.unicode_minus': False,
    'font.size'         : 11,
    'axes.titlesize'    : 14,
    'axes.labelsize'    : 12,
    'figure.dpi'        : 120,
    'savefig.dpi'       : 600,
    'pdf.fonttype'      : 42,
})

# ── 读数据 ────────────────────────────────────────────────────────────────────
df, DATA = load_sweep_dataframe()

# ── 1) 行为指纹聚类 ──────────────────────────────────────────────────────────
fp_cols = ['L', 'T', 'C', 'F', 'altm']
df['fp'] = df[fp_cols].round(3).astype(str).agg('|'.join, axis=1)
behavior_order = df.groupby('fp')['C'].min().sort_values().index
bid_map = {b: i + 1 for i, b in enumerate(behavior_order)}   # 1..K，暴露越小号越小
df['bid'] = df['fp'].map(bid_map)
K = df['bid'].nunique()
print(f'72 组权重 → {K} 种典型行为')

# ── 网格化 ────────────────────────────────────────────────────────────────────
wt_vals = sorted(df['wt'].unique())                 # x: 0 → 1.1
wf_vals = sorted(df['wf'].unique(), reverse=True)   # y: 0.5(上) → 0.0(下)

def pivot(col):
    return df.pivot_table(index='wf', columns='wt', values=col, aggfunc='first') \
             .reindex(index=wf_vals, columns=wt_vals)

B_mat = pivot('bid')    # 行为编号
C_mat = pivot('C')      # 暴露
T_mat = pivot('T')      # 时间 s
F_mat = pivot('F')      # 油耗
A_mat = pivot('altm')   # 最大高度 m

n_wt, n_wf = len(wt_vals), len(wf_vals)

# ── 2) 平台稳健度：同行为邻居计数（Moore 邻域） ─────────────────────────────
score = np.zeros((n_wf, n_wt), dtype=int)
for i in range(n_wf):
    for j in range(n_wt):
        s = 0
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                ii, jj = i + di, j + dj
                if 0 <= ii < n_wf and 0 <= jj < n_wt and B_mat.values[ii, jj] == B_mat.values[i, j]:
                    s += 1
        score[i, j] = s

# ── 3) 分区推荐 ───────────────────────────────────────────────────────────────
def best_in(mask, tiebreak):
    """mask 内取稳健度最高格；并列时用 tiebreak(候选 df) 挑选."""
    cand = df[mask].copy()
    cand['score'] = [score[wf_vals.index(r.wf), wt_vals.index(r.wt)] for r in cand.itertuples()]
    top = cand[cand['score'] == cand['score'].max()]
    return tiebreak(top).iloc[0]

sel = {}

# P1 / P2 用数据分位数定义，避免风险模型或平滑流程更新后固定阈值失效。
q15, q40, q60 = df['C'].quantile([0.15, 0.40, 0.60]).tolist()
sel['绕飞'] = best_in(df['C'] <= q15, lambda c: c.sort_values('C'))
sel['平衡'] = best_in((df['C'] >= q40) & (df['C'] <= q60),
                      lambda c: c.sort_values(['C', 'T']))
# P3 快穿（时间最优附近）
tmin = df['T'].min()
sel['快穿'] = best_in(df['T'] <= tmin + 2.0, lambda c: c.sort_values('T'))
# P4 省油（油耗最优，低空贴地策略）
fmin = df['F'].min()
sel['省油'] = best_in(df['F'] <= fmin + 0.15, lambda c: c.sort_values('F'))

print('\n' + '=' * 100)
print('分区推荐（平台稳健度 = 同行为邻居数 / 最大 8）')
print('=' * 100)
print(f"{'用途':<6}{'w_time':>7}{'w_fuel':>7}{'行为':>5}{'暴露s':>8}{'时间min':>8}{'距离km':>8}{'油耗kg':>8}{'高度km':>7}{'稳健度':>6}")
for name, r in sel.items():
    sc = score[wf_vals.index(r['wf']), wt_vals.index(r['wt'])]
    print(f"{name:<6}{r['wt']:>7.1f}{r['wf']:>7.1f}{int(r['bid']):>5}{r['C']:>8.2f}"
          f"{r['Tm']:>8.1f}{r['L']:>8.1f}{r['F']:>8.2f}{r['altm']/1000:>7.1f}{sc:>5}/8")

# ── 推荐表存盘 ───────────────────────────────────────────────────────────────
rows = []
for name, r in sel.items():
    sc = score[wf_vals.index(r['wf']), wt_vals.index(r['wt'])]
    rows.append({'用途': name, 'w_time': r['wt'], 'w_risk': r['wr'], 'w_fuel': r['wf'],
                 '行为编号': int(r['bid']), '暴露_s': round(r['C'], 3), '时间_min': round(r['Tm'], 2),
                 '距离_km': round(r['L'], 2), '油耗_kg': round(r['F'], 3),
                 '最大高度_km': r['altm'] / 1000, '平台稳健度': int(sc)})
pd.DataFrame(rows).to_csv(os.path.join(BASE, 'output_5zones', 'representative_weights.csv'),
                          index=False, encoding='utf-8-sig')
print('\n已保存: output_5zones/representative_weights.csv')

# ── 4) 图：权重-行为地图 + 时间-暴露分布 ─────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(17, 5.6))

# (a) 行为地图：颜色 = log10(暴露)，数字 = 行为编号，红线 = 行为边界
ax = axes[0]
im = ax.imshow(np.log10(C_mat.values + 0.05), cmap='YlOrRd', aspect='auto', origin='upper')

for i in range(n_wf):
    for j in range(n_wt):
        ax.text(j, i, f'{int(B_mat.values[i, j])}', ha='center', va='center',
                fontsize=8, color='#1a1a1a', weight='bold')

# 行为边界（相邻格行为不同 → 画红线）
Bv = B_mat.values
for i in range(n_wf):
    for j in range(n_wt):
        if j + 1 < n_wt and Bv[i, j] != Bv[i, j + 1]:
            ax.plot([j + 0.5, j + 0.5], [i - 0.5, i + 0.5], color='red', lw=2.2)
        if i + 1 < n_wf and Bv[i, j] != Bv[i + 1, j]:
            ax.plot([j - 0.5, j + 0.5], [i + 0.5, i + 0.5], color='red', lw=2.2)

# 代表点标记
markers = {'绕飞': ('P1', '#0066cc'), '平衡': ('P2', '#009944'), '快穿': ('P3', '#cc0000'), '省油': ('P4', '#7b2fbe')}
for name, (lab, col) in markers.items():
    r = sel[name]
    j, i = wt_vals.index(r['wt']), wf_vals.index(r['wf'])
    ax.plot(j, i, marker='*', ms=24, mfc=col, mec='white', mew=1.6, zorder=5)
    lx = -22 if j == n_wt - 1 else 0     # 右缘列标签左移，避免出图
    ax.annotate(lab, (j, i), textcoords='offset points', xytext=(lx, 14),
                ha='center', fontsize=12, weight='bold', color=col, zorder=6)

ax.set_xticks(range(n_wt)); ax.set_xticklabels([f'{v:.1f}' for v in wt_vals])
ax.set_yticks(range(n_wf)); ax.set_yticklabels([f'{v:.1f}' for v in wf_vals])
ax.set_xlabel('时间权重 $w_{time}$', weight='bold')
ax.set_ylabel('燃油权重 $w_{fuel}$', weight='bold')
ax.set_title('(a) 权重-行为分区地图（数字=行为编号，红线=行为边界，星=代表点）', weight='bold', fontsize=12)
ax.set_xticks(np.arange(-0.5, n_wt, 1), minor=True)
ax.set_yticks(np.arange(-0.5, n_wf, 1), minor=True)
ax.grid(which='minor', color='white', lw=1.0, alpha=0.7)
ax.tick_params(which='minor', size=0)
cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
cb.set_label('log$_{10}$(累积暴露+0.05)', fontsize=11)

# (b) 时间-暴露分布：颜色 = 高度策略
ax = axes[1]
def alt_color(a):
    if a < 1000:     return '#2ca02c'   # 低空 <1 km
    if a < 7000:     return '#1f77b4'   # 中空
    return '#d62728'                    # 高空

for r in df.itertuples():
    ax.scatter(r.T / 60, r.C + 0.05, s=52, c=alt_color(r.altm), alpha=0.75,
               edgecolors='white', linewidths=0.6, zorder=3)
ax.set_yscale('log')
ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f'{v:g}'))
ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
tx = df['T'] / 60.0
tpad = max((tx.max() - tx.min()) * 0.08, 0.2)
ax.set_xlim(tx.min() - tpad, tx.max() + tpad)
ymax = max(float((df['C'] + 0.05).max()) * 1.25, 0.2)
ax.set_ylim(0.04, ymax)
ax.set_xlabel('飞行时间（分钟）', weight='bold')
ax.set_ylabel('累积暴露（对数坐标）', weight='bold')
ax.set_title('(b) 时间-暴露权衡（颜色=最大高度策略）', weight='bold', fontsize=12)

label_offsets = {'绕飞': (10, 2), '平衡': (-84, 0), '快穿': (-72, -6), '省油': (8, 10)}
for name, (lab, col) in markers.items():
    r = sel[name]
    ax.scatter(r['Tm'], r['C'] + 0.05, marker='*', s=420, c=col, edgecolors='white',
               linewidths=1.2, zorder=5)
    ax.annotate(f"{lab} ({r['wt']:.1f},{r['wf']:.1f})", (r['Tm'], r['C'] + 0.05),
                textcoords='offset points', xytext=label_offsets[name],
                fontsize=11, weight='bold', color=col)

from matplotlib.lines import Line2D
handles = [Line2D([], [], marker='o', ls='', color='#2ca02c', label='低空策略（最大高度 <1 km）'),
           Line2D([], [], marker='o', ls='', color='#1f77b4', label='中空策略（1~7 km）'),
           Line2D([], [], marker='o', ls='', color='#d62728', label='高空策略（≥7 km）')]
ax.legend(handles=handles, loc='upper right', fontsize=10)
ax.grid(alpha=0.3)

plt.tight_layout(pad=1.8, w_pad=3.0)
out = os.path.join(OUTDIR, 'weight_selection_map.png')
fig.savefig(out, dpi=600, bbox_inches='tight', facecolor='white')
print(f'已保存: {out}')
