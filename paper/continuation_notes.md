# 论文续写与手动粘贴说明

依据：用户最新粘贴的 Elsevier LaTeX、`AST_paper_detailed_plan_V5.3.md` 和当前实际执行代码。本文不修改规划程序、实验参数或 sweep 缓存。

## 交付文件

- `manuscript_updated.tex`：完整单文件稿。保留用户导言区、题目、作者信息和未完成的摘要/引言；修正第 2 章结构，加入完整第 3 章与 4.1，以及第四章全部 `%%` 分析备注。可以直接复制到 Overleaf 主文件。插图尚未生成，所有插图命令均已注释，不会因缺图文件而参与编译。
- `section3_method.tex`：仅第 3 章，包含五个小节。
- `section4_experiments.tex`：第 4 章，4.1 有正文与两张参数表；4.2–4.4 保留标题，逐节以中文 `%%` 注释写明分析步骤、数据来源、图表内容、可支持的结论与解释边界。
- `section2_revised.tex`：结构正确的第 2 章。
- `preview.tex`：简单 article 预览外壳，需与三个章节文件放在一起；正式投稿继续使用 Elsevier 主模板。

## 最新粘贴稿的两处结构错误

1. 高度包络方程与 `pr:eq:altitude` 标签重复。完整稿中只保留一份。
2. 第 3 章标题进入 2.3 燃油模型内部，而 `sec:method` 标签留在 2.4 之后。完整稿把标题和标签一起移到第 2 章全部结束之后。

若按章节复制：先完整替换原第 2 章（包括误插的第 3 章标题），再放入第 3 章和第 4 章。建议用完整单文件，避免遗漏旧标题或重复标签。

原模板的示例附录、Lamport 示例引用已移除；新增方法的参考文献位置保留 `%%` 提示，尚未构成正式投稿所需的完整文献综述。摘要、Highlights、引言、结果叙述和结论仍需补齐，不能将本稿视作已完成投稿版。

## 第四章建议执行顺序

1. **4.1**：锁定设置、备份结果与代码，补真实机器/软件版本、环境图和数值采样复核。
2. **4.3**：先验收已有 72 组，再做热力图、控制变量切片、权衡散点和代表权重选择。全部基于已有合格结果，不为画图重跑。
3. **4.2.1 与 4.4**：对时间单目标和选定的代表权重，分别保存完整两阶段运行。重合的代表权重只需运行一次。每次先备份旧输出，避免 main 的固定文件名覆盖上一案例。
4. **4.2.2–4.2.3**：从同次 main 运行提取粗原始、细原始和最终轨迹，分别分析网格细化、几何与速度可行性。
5. 最后根据实际结果写结论、摘要结果句和提升数字。

参数表已明确：sweep 只有粗搜索，样条局部半径约 3015.90 m；完整方法包含 fine search，局部半径 200 m。两者对比同时改变了搜索和后处理局部范围，不能只归因于网格。

## 第 3 章关键实现映射

| 内容 | 核对位置 |
| --- | --- |
| A* 加性目标、启发函数、位置状态与几何平局规则 | `astar_risk_planner.py`、`baseline_manager.py` |
| 三维线段距离、粗细搜索与走廊 | `two_stage_planner.py` |
| 21 个拟合预算、条件性航向修复与全部候选比较 | `trajectory_smoothing.py::default_s_candidates/smooth_with_validation` |
| 曲率、均匀参数 RMS 偏离与无量纲几何评分 | `trajectory_smoothing.py::smoothing_objective/validate_spline` |
| 速度上限和前后向传播 | `fixed_wing_dynamics.py::compute_speed_profile` |
| 最终变速积分 | `astar_risk_planner.py::_segment_metrics` |
| main/sweep 的实验协议与参数差异 | `main.py`、`sweep_2d.py` |
| 代表选择与批内理想点距离 | `select_representative_weights.py::select_representatives` |
| 实验版本/来源一致性验收 | `analysis_data_loader.py`、`trajectory_data_loader.py` |

描述限定在实际实现：几何候选选择不直接最小化最终任务 J；速度验收失败不自动回选第二名样条；有限采样不是连续安全证明；同一合成环境的权重平台不代表对风或模型误差的鲁棒性。

## 验证方式

在项目根目录运行：`python -m unittest -v test_regressions test_postprocessing paper.test_paper_models`。

新增检查覆盖模型公式、样条评分、速度约束、参数与代码一致性，以及完整稿的标签、引用、章节次序和 LaTeX 环境结构。这些是本地一致性检查，**不是实际 TeX 编译**。本机没有可用 TeX 引擎，用户手动粘贴后仍需在 Overleaf 编译并检查公式换行、表格宽度、分页和交叉引用。

本次联合验证：原有 42 项回归检查与新增 11 项稿件检查，共 53 项通过。日志：`output_5zones/paper_review/review_tests.log`。原程序有 NumPy `trapz` 的弃用提示，本次没有为消除提示改动实验源码。

浏览器编辑已按用户最新要求改为手动复制交付；未修改线上 Overleaf 项目。没有新增实测性能结论，没有启动新的 sweep。
