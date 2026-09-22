# 稿件与程序核对记录

> 本文保留首次PDF审阅记录。本轮已补引言、摘要/结论的方法段和正式引文；最新状态以continuation_notes.md、literature_alignment.md及完整稿为准。下文历史待办不代表仍未修改。

核对对象：用户提供的 `D:/edge下载路径/REASEARCH1.pdf`（10 页），与代码提交 `dcd5ab5`。PDF 正文页码比文件页码少 2。本文档只核对已有实现与论文表述，不把模板中的提示当作用户指令，也没有新增实验结果。

## 判断

“问题建模 → 风险感知搜索 → 粗细两阶段规划 → 样条连续化 → 速度分配和最终评价 → 对比及偏好分析”的主线合理。现有 2.1 的半球雷达、幂律衰减、max-Pd 和硬圆柱禁飞模型基本正确；2.2 的高度包络、爬升角、垂直率和切向加速度也基本吻合。

需要修正的重点不是重新组织算法，而是避免把简化模型、离散搜索和有限采样验证写成完整飞行动力学或连续全局优化保证。

## 必须修改或明确的内容

| 稿件位置 | 核对结果及建议 | 代码依据 |
| --- | --- | --- |
| 2.2.2，原式 (21) | 原文说起终点速度均预先指定。实际只固定起点为 33 m/s，没有终点速度等式；终点速度由局部上限和前后向计算决定。 | `fixed_wing_dynamics.py::compute_speed_profile` |
| 2.2.2，原式 (20) | 83 m/s 是飞机参数上限；当前最终速度默认还受 55 m/s 巡航上限限制。区分飞机能力上限与本实验速度策略。 | 同上 |
| 2.2.3，原式 (26) | 未截断的转弯允许速度在直线段趋于无穷，不能说它本身始终小于飞机最大速度。实际速度受多个上限的最小值限制；允许上限低于失速速度时拒绝几何。 | 同上；`trajectory_smoothing.py::spline_curvature` |
| 2.2.3，原式 (22)–(25) | 协调平飞转弯关系用于三维空间曲率，是简化包络判据。它不是一般三维机动的精确滚转角反算；程序也没有随载荷因数变化的失速速度或滚转动力学。 | `fixed_wing_dynamics.py`；`trajectory_smoothing.py` |
| 2.2.1，原式 (18)–(19) | 过渡距离需同时考虑起点、终点的较大高度差。现有例子两端同高，因此数值没有矛盾。坡度条件不能证明有障碍物时必然存在可行路径。 | `altitude_constraints.py::validate_transition_distance` |
| 2.1、2.2 的可行性表述 | 可以保留对所有连续时刻成立的理想约束，但应说明实现以有限采样及容差验收，不能声称已经数学证明连续无碰撞或经过飞控验证。端点容差 2 m，样条几何检查 4096 个参数点。 | `trajectory_smoothing.py::validate_spline` |
| 2.1 风险场 | max-Pd 是主要暴露场的建模选择，不是多个独立雷达联合探测概率。实际当前雷达缩放系数为 1，内半径比例 0.3，指数 4。 | `environment_threat_model.py::_pd_single/calculate_risk`；主程序初始化 |
| 2.3 时间评价 | 搜索边采用长度/巡航速度与垂直率限时的较大值；最终路径采用相邻点速度确定的变速段时间。不能称两阶段都用同一实际速度，也不能用统一 55 m/s 代替最终速度。 | `fixed_wing_dynamics.py::get_segment_time`；`astar_risk_planner.py::_segment_metrics` |
| 2.3 暴露指标 | C 为 Pd 的时间积分，单位秒，满足 0≤C≤T；不能称累计概率、击落概率或无量纲风险总值。 | `astar_risk_planner.py::_segment_metrics` |
| 2.3 燃油模型 | 实际使用寄生功率和诱导功率两项，随高度改变密度，额外加入爬升功率，再除推进效率。部分旧代码注释的固定升阻比公式不代表实际执行公式。 | `fixed_wing_dynamics.py::get_power_at_state` |
| 2.3 模型边界 | 指数密度不是完整 ISA；下降回收系数为 0；质量恒定，未加转弯载荷引起的诱导阻力修正、发动机瞬态或显式切向加速功率。论文可作为相对比较模型，不能声称真实机型燃油精确预测。 | 同上 |
| 2.4 anchor | 三个 anchor 为粗网格单目标搜索原始路径，没有先进行最终样条及变速优化。下参考取各自目标 anchor 的对角项，上参考取三条 anchor 的列最大值。 | `baseline_manager.py::BaselineManager.compute` |
| 2.4 归一化 | 全部权重及粗细搜索共用基准；不截断到 [0,1]；退化区间报错；上参考是 payoff-table 的 nadir 近似。最终值可能超出这些参考范围。 | `normalize_metrics`、`safe_range` |
| 2.4 搜索边代价 | 每条边只除以归一化尺度，不能每条边都减最小值。下参考偏移只在整条路径报告 J 时应用。 | `weighted_increment`、`weighted_objective` |
| 3.2 与 3.5 | 都写“可行性验证”，造成重复，且验证出现在样条构造之前。调整为下面的顺序。 | 稿件章节结构 |
| 3.4 中的“优化” | 代码评价多个平滑候选，用独立几何评分选取可行候选；不是直接最小化最终 T/C/F 加权和，更不是路线和速度的联合全局优化。 | `trajectory_smoothing.py::smoothing_objective/smooth_with_validation` |
| 第 4 章 sweep | 72 组只有粗搜索＋后处理；不能用它们直接证明 72 组完整两阶段算法的性能。粗细两阶段比较、传统 A* 对比需要各自受控实验。 | `sweep_2d.py::finalise_path`；`two_stage_planner.py` |

## 推荐的第 3 章结构

```latex
\section{Proposed risk-aware two-stage trajectory planning framework}
% 在本节开头用一段文字和流程图说明完整流程，不另设重复验证节。
\subsection{Multi-objective risk-aware A* search}
\subsection{Coarse-to-fine three-dimensional planning}
\subsection{Cubic B-spline trajectory refinement}
\subsection{Geometric feasibility validation and candidate selection}
\subsection{Adaptive speed planning and final mission evaluation}
```

将原 3.2 的已有验证文字移到新的 3.4，补上局部偏离约束、有限采样设置和终端航向来源。原 3.3 调到 3.2，原 3.4 调到 3.3。3.5 专注速度上限、前后向计算、严格验收和指标重算。原文引用 Section 3.5 的速度规划仍然正确。

原始折线的速度、转弯半径图只能作诊断，不能将其作为已经物理可执行的基线。不同路径如要比较燃油、时间，应明确各自是搜索估计还是最终验证后的评价。

4.2.1 中的“Traditional A*”还需要单独定义边代价、启发函数、邻接方式、约束与共同后处理协议。将风险和燃油权重设为 0 得到的是本程序的时间单目标搜索；存在垂直率限时，因此它不能未经说明直接等同于最短几何路径的传统 A*。当前稿件尚无该对比的结果，不能先写性能提升结论。

## 2.3、2.4 已补充内容

`section2_revised.tex` 是可整体替换原 Section 2 的英文 LaTeX：保留四个二级小节，修订 2.1/2.2 并完整补充 2.3/2.4，所有内部交叉引用使用 `pr:` 标签，避免手写公式编号。

- 2.3 给出连续 T/C/F 定义、实际燃油功率、单位、搜索/最终评价差别，以及采用同一时间增量的数值积分。
- 2.4 给出三 anchor payoff table、固定尺度、加权 J、等价加性搜索边代价、权重扫描范围和结论适用边界。
- 不填造摘要、结果数字或参考文献。封面、Highlights、Abstract、作者单位、示例附录和 Lamport 示例参考文献仍需在正式稿完善或清除；本次未代写实验结论。

若保留题目中的 “Dynamically feasible”，须在摘要和方法中限定为所用 reduced flight-envelope model 下的可行性。更谨慎的备选题目是：

**Risk-aware trajectory planning for fixed-wing UAVs using multi-objective two-stage search and flight-envelope-constrained refinement**

## 来源说明

燃油公式以实际执行代码为准。其寄生/诱导阻力基础与 NASA 的抛物线阻力表达一致，但爬升效率、密度近似及其余模型简化是本项目采用的假设，并非 NASA 对本模型的验证：[NASA Induced Drag Coefficient](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/induced-drag-coefficient/)。

样条构造采用 FITPACK 参数平滑接口。`s` 控制拟合的平滑条件，不能把这一接口本身描述为求解本文完整任务目标的优化器：[SciPy 1.15.3 splprep 文档](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.interpolate.splprep.html)。

## 使用与验证

1. 导言区需要 `amsmath`、`amssymb`。用 `section2_revised.tex` 替换原 `\section{Problem formulation}` 到下一节标题之前的全部内容；或者上传后在原位置写 `\input{section2_revised}`。
2. `preview.tex` 是独立编译预览用的简单 article 外壳，不是替代 Elsevier 模板的主文件。
3. 数学一致性及 LaTeX 结构检查：在项目根目录运行 `python -m unittest -v paper.test_paper_models`。这不是 TeX 编译，不能代替最终 Overleaf 编译检查。
4. 未修改 Overleaf：Edge 接口不可用；内置浏览器项目显示 Restricted。用户随后明确改为手动粘贴，当前交付以 `manuscript_updated.tex` 为准，续写和粘贴说明见 `continuation_notes.md`。本机未找到 TeX 编译器，仍需在 Overleaf 编译并检查最终分页和公式排版。
5. 所有原 Python 程序、sweep 基准和工况缓存保持不变；本次论文修订不要求重跑 sweep。

首轮第 2 章验证：新增 6 项论文模型/结构检查通过；与原 42 项回归测试一起运行，共 48 项通过。续写第 3–4 章后扩充了评分、速度、参数及完整文稿检查，最新联合测试日志位于 `output_5zones/paper_review/review_tests.log`。未完成 TeX 编译与新版 PDF 视觉检查，不应把这些测试表述为“论文已编译通过”。
