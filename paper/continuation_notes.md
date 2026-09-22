# 论文续写与手动粘贴说明

依据：用户最新粘贴的 Elsevier LaTeX、`AST_paper_detailed_plan_V5.3.md` 和当前实际执行代码。本文不修改规划程序、实验参数或 sweep 缓存。

## 交付文件

- `manuscript_updated.tex`：优先使用的完整单文件稿，可以直接复制到 Overleaf 主文件。保留作者信息；题目改为明确的flight-envelope-constrained refinement，补齐摘要方法段、四条Highlights、引言三小节、三项贡献、结论方法段及六条正式参考文献。所有插图命令仍以`%%`注释，不因缺图参与编译。摘要/结论的实测结果段仍有明确待填说明。
- `abstract_draft.tex`、`section1_introduction.tex`、`section5_conclusions.tex`：对应可单独复制的片段，与完整稿保持一致。
- `references.tex`：六条核实文献的thebibliography列表，完整稿已内嵌，不需要额外.bib文件。`literature_alignment.md`记录原始链接及每条引文支持什么。
- `section3_method.tex`：仅第 3 章，包含五个小节。
- `section4_experiments.tex`：第 4 章，4.1 实验设置；4.2 集中三组对比；4.3 四张敏感性核心图；4.4 三个代表、一张三维图与一张表。已写英文比较协议、代表选择公式与中文 `%%` 分析步骤。结果叙述仍等待实测数据；代表表的 `--` 是待填占位，不是零值。
- `chapter4_design.md`：第四章最新紧凑结构、最小执行顺序及引言衔接建议；取代上一轮按三个贡献各占一节的展开版。
- `section2_revised.tex`：结构正确的第 2 章。
- `preview.tex`：覆盖摘要、第1--5章和参考文献的article预览外壳，需与全部章节片段放在paper目录中编译；正式投稿使用Elsevier完整稿。

## 最新粘贴稿的两处结构错误

1. 高度包络方程与 `pr:eq:altitude` 标签重复。完整稿中只保留一份。
2. 第 3 章标题进入 2.3 燃油模型内部，而 `sec:method` 标签留在 2.4 之后。完整稿把标题和标签一起移到第 2 章全部结束之后。

若按章节复制：先完整替换原第 2 章（包括误插的第 3 章标题），再放入第 3 章和第 4 章。建议用完整单文件，避免遗漏旧标题或重复标签。

原模板示例附录、Lamport示例引用和空白摘要已移除。引言按背景、相关工作、研究重点与贡献组织，明确承认已有风险感知规划、分层搜索和路径/速度分解研究。题目避免直接承诺完整动力学可执行。尚待填入真实结果、图表、机器/软件信息及作者单位；这些完成后再做最终投稿格式核对。

4.2标题调整为`Overall planning performance and controlled comparisons`：A是目标设置比较，B/C是同次运行的阶段比较，不能把它们概括为完整因素消融。稳定交叉引用标签`sec:ablation`保留，不影响显示标题。C组新增几何限速与暴露积分的关联解释和`%% C-7`分析步骤，不增加场景或搜索任务。

## 第四章建议执行顺序

1. **4.1**：锁定设置、备份结果与代码，补真实机器/软件版本、环境图和数值采样复核。
2. **4.3 与 4.4**：验收72组后，输出四张敏感性图并按规则选Risk/Time/Balanced。直接复用三组sweep最终缓存画一张三维叠加图、填一张表，不额外重跑三个main。
3. **4.2 A**：复用或补齐时间单目标与一组预先说明的多目标权重的完整两阶段结果，每次先备份旧输出，避免main固定文件名覆盖。
4. **4.2 B/C**：从其中同一次main提取Coarse/Fine Raw/Final，完成网格重规划和轨迹可行性对比，不分别重跑。
5. 最后用真实结果补结论和摘要中的`%%`待填段。只写观测到的变化，不将预期取舍改写成已证实结果。

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

首版联合验证为53项，紧凑版为55项。本轮增加引文完整性与摘要/预览同步检查；2026-09-22联合验证57项全部通过（15.554秒）。日志为`output_5zones/paper_review/review_tests.log`。本机再次检查未发现pdflatex、xelatex、latexmk或tectonic，因此未进行实际TeX编译。原程序有NumPy `trapz`弃用提示，本次没有为消除提示改动实验源码。

浏览器编辑已按用户最新要求改为手动复制交付；未修改线上 Overleaf 项目。没有新增实测性能结论，没有启动新的 sweep。
