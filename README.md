# 固定翼多目标路径规划 V5.3.1

面向五区雷达环境的固定翼路径规划与实验分析项目，综合考虑飞行时间、雷达暴露和燃油消耗，包含两阶段 A* 搜索、B 样条轨迹光滑、运动学约束验证和权重扫描。

## 环境准备

在项目根目录执行以下命令（Windows PowerShell）：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\Activate.ps1
```

依赖清单根据源码导入整理，尚未锁定经过验证的版本组合。如果已有可运行环境，可直接使用该环境。

## 运行

先执行项目自带的预检：

```powershell
python -m unittest -v test_regressions test_postprocessing
python preflight_check.py
```

单次两阶段规划与可视化：

```powershell
python main.py
```

新版 72 组权重扫描（自动复用配置匹配的 anchor 基准）：

```powershell
python -u sweep_2d.py
```

中断后继续、或重试失败工况，使用同一命令即可。只有需要全部重算时才使用 `--fresh`；它会重新计算全部工况和三个 anchor。

新版结果独立保存在 `output_5zones/sweep_2d_time_fuel_v5_3_1.xlsx`，不会覆盖原 V5.3 工作簿。每组的原始路径和最终轨迹保存在 `output_5zones/sweep_cases_v5_3_1/`，用于故障恢复及后续复核。不要同时运行两个扫描进程写入同一目录。

全部 72 组通过后，按论文规则选择代表工况：

```powershell
python select_representative_weights.py
```

所有 sweep 分析图共用严格数据验收：必须是当前版本的 72 组成功结果，且来源签名一致、与本地代码和基准匹配。新工作簿不存在时会明确报错，不会自动回退旧表。核心权衡图中的典型工况也使用同一个代表工况选择器。累积暴露 `C=∫Pd dt` 的单位统一为秒；图中的批内 min-max 缩放与搜索使用的 anchor 归一化分别标注。

轨迹图默认读取当前 `main.py` 生成且签名匹配的 `planned_path.pkl`。也可以直接画某个已完成的 sweep 工况，无需重新搜索：

```powershell
$env:TRAJECTORY_FILE = "output_5zones/sweep_cases_v5_3_1/wt_0.3_wf_0.2.pkl"
python visualize_astar_separate.py
python visualize_speed_profile_english.py
python visualize_turn_radius.py
python analyze_radar_interaction_v2.py
Remove-Item Env:TRAJECTORY_FILE
```

指定工况的图默认写入 `analysis_figures/<工况文件名>/`，也可通过 `ANALYSIS_OUTDIR` 指定目录。速度和转弯图复用保存的最终路径、解析曲率及速度；原始折线仅作诊断比较。缺少签名的历史 `main.py` 缓存会被拒绝，需用当前 `main.py` 重新生成；本次后处理修改不会使既有 sweep 缓存失效。

`main.py` 保留粗细两阶段搜索，sweep 保留粗网格筛选；二者共用物理评价，网格及平滑信赖域按实验用途区分。

运行参数和版本说明见 [RUN_FIRST.txt](RUN_FIRST.txt)、[VERSION.txt](VERSION.txt) 和 [REVISION_NOTES.md](REVISION_NOTES.md)。

V5.3.1 统一了变速段的时间、暴露和燃油积分，速度边界为 `v(0)=33 m/s`。起点和终点高度保持 0 m，作为任务高度参考；模型不包含地面滑跑和着陆滑跑。全程执行失速速度下限。搜索网格、威胁模型和目标权重范围保持 V5.3 配置。

当前采用 `V5.3.2-terminal-heading-repair` 平滑器：仅在原候选全部失败且存在末端航向问题时，调整末端控制点后重新执行全部约束校验。航向 12°、爬升角 15° 等阈值不变。Excel 增加平滑版本、航向修复标记及实际候选评估次数；数据结构仍为 V5.3.1，源代码签名区分本次修复。不要修改源码后强行沿用旧签名的中间结果。

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `main.py` | 两阶段规划、光滑及可视化主入口 |
| `astar_risk_planner.py`、`two_stage_planner.py` | A* 搜索与粗细网格规划 |
| `trajectory_smoothing.py`、`altitude_constraints.py` | 轨迹光滑、高度与飞行约束 |
| `fixed_wing_dynamics.py` | 固定翼动力学、速度与燃油模型 |
| `radar_environment_5zones.py`、`environment_threat_model.py` | 雷达环境与威胁模型 |
| `baseline_manager.py` | 多目标归一化基准管理 |
| `sweep_2d.py`、`select_representative_weights.py` | 权重扫描与代表性权重选择 |
| `analysis_data_loader.py`、`analyze_*.py` | 实验数据读取与分析 |
| `trajectory_data_loader.py` | main/sweep 轨迹缓存的校验与统一读取 |
| `visualize_*.py`、`plot_heatmap_3obj_v3.py` | 轨迹和统计图表 |
| `preflight_check.py` | 正式实验前的回归预检 |
| `test_regressions.py` | 积分、速度、筛选及续跑回归测试 |
| `test_postprocessing.py` | 后处理数据校验、代表工况和轨迹图回归测试 |
| `AST_paper_detailed_plan_V5.3.md` | 论文详细规划 |
| `output_5zones/` | 本地实验结果、缓存和图表 |

## Git 管理约定

源码、说明文档和依赖清单纳入版本管理。`output_5zones/`、Python 缓存、虚拟环境、编辑器配置和临时文件由 `.gitignore` 忽略；现有结果仍保留在本地，克隆仓库不会包含这些结果。实验结果需要另外备份或归档。

保留原有文件布局以兼容脚本中的相对路径，请从项目根目录运行。`SHA256SUMS.txt` 保留为原发布包的校验清单，不覆盖本次新增的仓库管理文件。

日常提交示例：

```powershell
git status
git add <文件路径>
git commit -m "描述本次修改"
```
