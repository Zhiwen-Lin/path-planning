# AST 期刊论文详细写作框架与最小补实验路线（V5.3 完整版）

> 目标期刊：**Aerospace Science and Technology (AST)**  
> 当前程序版本：**Fixed-wing multi-objective planner V5.3 FINAL**  
> 论文类型：Research Article  
> 当前研究主线：**复杂威胁环境下固定翼无人机的任务偏好驱动风险感知规划与动力学可执行轨迹生成**  
> 核心原则：**不再追求“算法无限增加”，而是把现有程序形成一条完整、可验证、可投稿的证据链。**

> **V5.3.1 实现校正（2026-09-21）**：以下写作框架保留；正式数值改用
> `sweep_2d_time_fuel_v5_3_1.xlsx`，不与旧 V5.3 结果混用。变速段采用恒定切向
> 加速度（速度平方随弧长线性变化），T/C/F 共享子段时间。起始速度为
> `v(0)=v_stall=33 m/s`，起终点 0 m 是任务高度参考，模型不包括地面滑跑和
> 着陆滑跑，失速下限适用于全部飞行采样点。Section 16 的代表工况选择已与
> 代码统一：最小 C、最小 T、最小理想点距离；并列按其他指标及权重升序选取。
> 旧版扫描虽已完成，修正后的正式扫描仍需完成和审核后才能锁定论文数字。
> 当前平滑器另含 `V5.3.2-terminal-heading-repair` 补丁：原候选全部失败且存在
> 航向不满足时，调整末端控制点并重新检查全部约束；航向 12°、爬升角 15°
> 限值不变。实验中应报告是否启用航向修复，源代码签名区分本次版本。

---

# 0. 这篇论文到底要讲什么

这篇文章不建议包装成：

> “本文提出了一种改进 A* 算法。”

因为这样的表述会让文章显得像一个局部算法修补工作，而且容易被审稿人追问：

- A* 本身哪里“新”？
- B-spline 也不是新的；
- Forward–Backward 速度规划也不是新的；
- 多目标加权也不是新的。

更合适的总体定位是：

> **本文针对复杂威胁环境下固定翼无人机任务规划中“软威胁难以连续表达、任务偏好难以进入路径选择、全局高分辨率三维搜索计算量大、离散网格路径难以直接满足固定翼动力学约束”四个相互关联的问题，构建时间–威胁–燃油统一多目标代价，并结合粗–细两阶段三维搜索、B 样条连续化、解析曲率验证和固定翼自适应速度规划，实现从“可通行离散路径”到“任务偏好驱动且动力学可执行连续轨迹”的完整规划链。**

全文应该围绕下面这条主线展开：

```text
任务空间建模
    ↓
硬禁飞区 + 连续软威胁
    ↓
时间 / 风险 / 燃油统一代价
    ↓
多目标风险感知 A*
    ↓
粗网格全局路线
    ↓
三维走廊内细网格再规划
    ↓
B-spline 连续轨迹生成
    ↓
转弯 / 爬升 / 高度 / 碰撞约束验证
    ↓
Forward–Backward 速度规划
    ↓
沿最终轨迹重新计算 T / C / F
```

全文真正要证明的是：

\[
\boxed{
\text{collision-free path}
\rightarrow
\text{mission-preference-driven path}
\rightarrow
\text{dynamically executable trajectory}
}
\]

而不是“某个 A* 变体比所有算法都好”。

---

# 1. 建议最终题目

## 1.1 推荐题目 1（最推荐）

**Dynamically feasible risk-aware trajectory planning for fixed-wing UAVs via multi-objective two-stage search**

优点：

- `fixed-wing UAVs` 明确航空对象；
- `trajectory planning` 而不是泛泛的 path planning；
- `risk-aware` 对应软威胁；
- `dynamically feasible` 对应 B-spline + dynamics；
- `multi-objective two-stage search` 对应核心方法。

---

## 1.2 推荐题目 2

**Mission-preference-driven two-stage trajectory planning for fixed-wing UAVs in complex threat environments**

优点：

- 突出任务偏好；
- “mission-preference-driven” 与 72 组权重实验对应；
- 很适合全文“不同任务需求 → 不同轨迹策略”的叙事。

---

## 1.3 推荐题目 3

**Risk-aware two-stage trajectory planning for fixed-wing UAVs with time–threat–fuel trade-offs**

优点：

- 直接点出三个目标；
- 对实验部分非常直观。

---

# 2. 论文中的严谨表述：一定统一

## 2.1 不建议写

> 传统 A* 只能处理二值栅格，不能处理连续威胁。

原因：

A* 本质上是通用图搜索算法，它并不天然局限于二值代价。

---

## 2.2 建议改成

> **Traditional occupancy-grid A* planners commonly use free/occupied states for collision and reachability checks and employ geometric distance or travel time as the dominant search cost. Such a formulation does not explicitly distinguish different continuous threat levels within otherwise traversable regions, nor does it directly encode mission-dependent trade-offs among time, threat exposure, and fuel consumption.**

中文：

> **传统基于二值占据栅格的 A* 路径规划通常主要利用 free/occupied 状态进行可达性和碰撞判定，并以几何距离或飞行时间作为主要搜索代价，因此难以显式区分可通行空间内部连续变化的软威胁程度，也难以直接表达不同任务对时间、威胁和燃油之间的偏好。**

---

# 3. AST 首稿建议最终目录

```text
Abstract

1. Introduction

2. Problem formulation
   2.1 Three-dimensional threat environment
   2.2 Fixed-wing flight constraints
   2.3 Time, threat-exposure and fuel models
   2.4 Anchor-based normalized mission-oriented objective

3. Proposed risk-aware two-stage trajectory planning framework
   3.1 Multi-objective risk-aware A* search
   3.2 Coarse-to-fine three-dimensional planning
   3.3 Cubic B-spline trajectory optimization
   3.4 Fixed-wing feasibility validation and adaptive speed planning

4. Numerical results and discussion
   4.1 Simulation setup
   4.2 Overall performance and controlled ablation
       4.2.1 Traditional A* vs proposed method
       4.2.2 Coarse vs fine planning
       4.2.3 Fine raw path vs B-spline trajectory
   4.3 Mission-preference sensitivity
   4.4 Representative mission scenarios

5. Conclusions
```

一级标题就控制在 5 个。

---

# 4. Abstract 怎么写

摘要建议 5 句结构。

---

## 4.1 第一句：问题

> Complex threat environments require fixed-wing UAVs to simultaneously consider mission efficiency, threat exposure, fuel consumption, and dynamic executability, whereas conventional occupancy-grid planning mainly focuses on collision-free connectivity or geometric efficiency.

---

## 4.2 第二句：方法核心

> This paper develops a risk-aware two-stage trajectory planning framework in which hard no-fly regions are treated as infeasible constraints, while continuous radar exposure, flight time, and fuel consumption are incorporated into a normalized multi-objective A* search cost.

---

## 4.3 第三句：粗–细规划

> A coarse global search is first employed to determine the threat-avoidance route topology, followed by a fine-grid replanning stage inside a local three-dimensional corridor using the same mission-oriented objective.

---

## 4.4 第四句：动力学可执行化

> The resulting discrete path is further transformed into a dynamically feasible trajectory through cubic B-spline optimization, analytical curvature evaluation, hard flight-envelope validation, and forward–backward adaptive speed planning.

---

## 4.5 第五句：结果

这一句必须等最终数据出来后再写。

模板：

> Numerical results show that the proposed method reduces cumulative threat exposure by XX% relative to a traditional time-cost A* baseline at the expense of only XX% additional mission time, while maintaining the prescribed turning-radius, climb-angle, altitude, speed, and acceleration constraints. A 72-case weight sweep further demonstrates systematic mission-preference-dependent variations in time, threat exposure, fuel consumption, and route topology.

现在先不要填百分比。

---

# 5. Section 1 — Introduction 详细写法

---

## 5.1 第一段：研究背景

### 写作目的

建立问题的重要性，不要一上来讲 A*。

### 建议内容

复杂威胁环境下固定翼无人机执行侦察、穿透、巡逻、远程任务时，通常同时面临：

\[
\text{mission time}
\leftrightarrow
\text{threat exposure}
\leftrightarrow
\text{fuel consumption}
\]

需要说明：

- 直接路径时间最短，但可能穿越高威胁雷达区域；
- 保守绕行可降低风险，但会增加距离和燃油；
- 高度机动会改变雷达探测概率和固定翼功率需求；
- 固定翼不能悬停或任意急转，因此“路径可达”不等于“轨迹可执行”。

### 本段最后一句

> Therefore, fixed-wing mission planning in complex threat environments should simultaneously address route selection, threat-aware mission preference, spatial refinement, and dynamic executability.

---

## 5.2 第二段：传统 grid / graph-based 方法

### 写作目的

说明 A* 的优点，不要把传统方法写得一无是处。

建议：

> Grid-based graph search remains attractive because of its deterministic search structure, explicit obstacle handling, and straightforward extension to three-dimensional spaces.

然后指出：

> However, conventional occupancy-grid formulations often use binary free/occupied states together with distance- or time-dominant edge costs.

再引出：

> Such representations cannot directly distinguish different continuous soft-threat levels inside free space.

---

## 5.3 第三段：risk-aware / multi-objective 研究

### 写什么

概述：

- risk field；
- threat-aware path planning；
- multi-objective path planning；
- weighted objectives。

### 不要写

> Existing studies do not consider risk.

太绝对。

### 建议写

> Previous studies have incorporated threat or energy terms into UAV path planning; however, threat, time, and fuel are often evaluated using different metrics or are not consistently propagated from discrete search to the final dynamically feasible trajectory.

这样更稳妥。

---

## 5.4 第四段：coarse-to-fine / hierarchical planning

说明：

- 高分辨率 3-D 网格全局搜索节点规模大；
- 低分辨率搜索快，但局部空间精度不足；
- 因此分层搜索合理。

然后引出你的特点：

> 本文 coarse 和 fine 两阶段都保持相同的任务综合代价，而不是 coarse 确定路径后只做几何插值。

---

## 5.5 第五段：固定翼轨迹连续化

说明：

- A* 输出折线；
- 折线在节点处航向突变；
- 数学曲率不连续；
- 速度规划无法直接建立在这种折线上。

现有方法采用：

- B-spline；
- Bézier；
- polynomial trajectory optimization；
- curvature-constrained smoothing。

你的 gap：

> 本文重点不是提出新的 spline，而是把离散任务层规划与固定翼动力学可执行化连接起来。

---

## 5.6 第六段：Research gap

建议只列三点：

### Gap 1

连续软威胁和任务偏好没有在统一搜索代价中充分表达。

### Gap 2

高分辨率 3-D 搜索与全局计算效率之间存在矛盾。

### Gap 3

离散可行路径到固定翼连续可执行轨迹之间存在明显断层。

---

## 5.7 第七段：本文方法

用 4 句概括：

1. hard/soft threat；
2. time–risk–fuel objective；
3. coarse-to-fine；
4. B-spline + dynamics + speed。

---

## 5.8 第八段：Contributions

建议最终只写 3 条。

---

# 6. Contributions 中英文最终版本

---

## Contribution 1

### 中文

> **构建硬禁飞约束与连续软威胁并存的任务偏好驱动多目标规划模型。** 将硬火力区作为不可行约束，将雷达探测概率通过时间积分形成累计威胁暴露，并与飞行时间和燃油消耗共同构成归一化多目标搜索代价，从而使路径选择能够由任务偏好参数直接驱动。

### English

> **A mission-preference-driven multi-objective planning formulation is developed by combining hard no-fly constraints with continuous soft-threat exposure.** Hard fire zones are treated as infeasible regions, whereas radar detection probability is integrated over exposure time and jointly optimized with flight time and fuel consumption through a normalized search cost.

---

## Contribution 2

### 中文

> **提出保持统一任务代价的粗–细两阶段三维搜索机制。** 粗网格用于确定全局风险规避路线拓扑，随后在局部三维走廊内利用相同时间–威胁–燃油综合代价进行细网格再规划，在控制全局搜索规模的同时提高局部空间与高度规划精度。

### English

> **A coarse-to-fine three-dimensional search strategy with a consistent mission-oriented objective is proposed.** The coarse stage determines the global threat-avoidance topology, while the fine stage replans within a local three-dimensional corridor using the same time–threat–fuel objective, thereby improving local spatial accuracy without requiring a globally fine grid.

---

## Contribution 3

### 中文

> **构建从离散任务路径到固定翼动力学可执行轨迹的一体化转换流程。** 通过三次 B 样条候选优化、解析曲率评价、高度与爬升约束验证以及 Forward–Backward 自适应速度规划，将离散 A* 路径进一步转换为满足固定翼转弯、爬升、速度和加减速约束的连续轨迹，并沿最终轨迹统一重新计算时间、累计威胁暴露和燃油性能。

### English

> **An integrated path-to-trajectory transformation procedure is constructed for fixed-wing dynamic executability.** Cubic B-spline candidate optimization, analytical curvature evaluation, flight-envelope validation, and forward–backward adaptive speed planning are combined to convert the discrete A* path into a dynamically feasible trajectory, along which time, cumulative threat exposure, and fuel consumption are consistently recomputed.

---

# 7. Section 2 — Problem formulation 详细写法

---

# 7.1 2.1 Three-dimensional threat environment

## 写作目的

定义问题空间和 hard/soft threat。

---

## 7.1.1 三维位置

\[
\mathbf p=[x,y,z]^T
\]

离散路径：

\[
\Gamma_d=
\{\mathbf p_0,\mathbf p_1,\ldots,\mathbf p_N\}
\]

最终连续轨迹：

\[
\mathbf r(t)=
[x(t),y(t),z(t)]^T
\]

---

## 7.1.2 当前任务端点

程序中：

```text
START = (2000, 2000, 0) m
GOAL  = (98000, 98000, 0) m
```

论文中不要解释成“无人机全程可贴地”。

必须与高度包络一起说明。

---

## 7.1.3 Hard threat

火力区：

\[
\Omega_{\rm hard}
\]

要求：

\[
\Gamma\cap\Omega_{\rm hard}
=
\varnothing
\]

在搜索中：

\[
J_e=\infty
\]

或直接删除不可行边。

---

## 7.1.4 Soft radar threat

定义探测概率：

\[
P_d(\mathbf p)\in[0,1]
\]

soft threat 特点：

> 允许飞入，但会产生连续累计暴露代价。

累计威胁：

\[
C=
\int_0^T
P_d(\mathbf r(t))dt
\]

离散：

\[
C\approx
\sum_k
P_d(\mathbf p_k)\Delta t_k
\]

一定强调：

> **不是简单 \(\sum P_d\)，而是 exposure probability × dwell time。**

因为同样风险场下，停留越久，累计暴露越高。

---

# 7.2 2.2 Fixed-wing flight constraints

当前程序可支持的核心参数：

```text
v_stall = 33 m/s
v_cruise = 55 m/s
v_max = 83 m/s
max bank angle = 30 deg
max climb angle = 15 deg
```

---

## 7.2.1 Speed constraint

\[
v_{\rm stall}
\le
v(t)
\le
v_{\max}
\]

---

## 7.2.2 Turn radius constraint

协调转弯关系：

\[
R=
\frac{v^2}
{g\tan\phi}
\]

因此：

\[
R_{\rm feasible}
=
\frac{v_{\rm stall}^2}
{g\tan\phi_{\max}}
\]

最终轨迹要求：

\[
R_{\min,\Gamma}
\ge
R_{\rm feasible}
\]

---

## 7.2.3 Climb-angle constraint

\[
|\gamma|
=
\left|
\tan^{-1}
\frac{\dot z}
{\sqrt{\dot x^2+\dot y^2}}
\right|
\le
\gamma_{\max}
\]

当前：

\[
\gamma_{\max}=15^\circ
\]

---

## 7.2.4 Acceleration constraint

\[
-a_{\rm dec,max}
\le
\dot v
\le
a_{\rm acc,max}
\]

Forward–Backward 速度规划用于保证这个条件。

---

# 7.3 高度包络

当前程序：

```text
endpoint altitude = 0 m
en-route minimum altitude = 1000 m
maximum altitude = 10000 m
transition distance = 5000 m
```

空间最低高度可写成：

\[
h_{\min}(\mathbf p)
=
\min
\left[
h_s(\mathbf p),
h_g(\mathbf p),
1000
\right]
\]

其中起飞端和进场端最低高度按水平距离线性变化。

简化写法：

\[
h_s=
z_s+
(1000-z_s)
\operatorname{clip}
\left(
\frac{d_s}{D_t},0,1
\right)
\]

\[
h_g=
z_g+
(1000-z_g)
\operatorname{clip}
\left(
\frac{d_g}{D_t},0,1
\right)
\]

其中：

\[
D_t=5000\;m
\]

然后：

\[
h_{\min}(\mathbf p)
\le z
\le10000
\]

对于：

\[
z_s=z_g=0
\]

得到：

```text
0 m
→ linear climb over first 5 km
→ 1000 m minimum en-route
→ linear descent over final 5 km
→ 0 m
```

### 为什么 5 km 可行

\[
D_{\min}
=
\frac{1000}
{\tan15^\circ}
\approx3732.1\,m
\]

而：

\[
5000>3732.1
\]

因此存在满足最大爬升角的过渡空间。

这一点建议写成一句，不要单独做实验。

---

# 7.4 2.3 Time model

对边 \(e\)：

\[
T_e
\]

最终：

\[
T=
\sum_eT_e
\]

论文中可以解释：

> 边时间由水平/空间飞行时间和垂向运动限制共同确定，确保不会由于高度变化而低估实际飞行时间。

如果正文公式过多，可以把具体边时间公式简化，详细实现写 Appendix 或算法描述。

---

# 7.5 Threat-exposure model

\[
C_e
=
\int_{t_e}
P_d(\mathbf r(t))dt
\]

然后：

\[
C=
\sum_eC_e
\]

单位可以写：

> exposure-s

或者称：

> cumulative exposure index

只要全文统一。

---

# 7.6 Fuel model

程序采用固定翼寄生阻力 + 诱导阻力 + 爬升功率逻辑。

可写：

\[
P_{\rm para}
=
\frac12\rho v^3C_{D0}S
\]

\[
P_{\rm ind}
=
\frac{
2(mg)^2
}{
\rho vS\pi AR e
}
\]

\[
P=
P_{\rm para}
+
P_{\rm ind}
+
P_{\rm climb}
\]

然后：

\[
F=
\int_0^T
\frac{P(t)}
{H_f}
dt
\]

其中：

\[
H_f
\]

为燃油低位热值。

最终单位：

\[
kg
\]

---

# 7.7 2.4 Anchor-based normalization

这部分是论文里很重要但容易写错的地方。

---

## 7.7.1 三个单目标 anchor

### Time anchor

\[
\mathbf w^{(T)}
=
(1,0,0)
\]

得到：

\[
(T^{(T)},C^{(T)},F^{(T)})
\]

### Risk anchor

\[
\mathbf w^{(C)}
=
(0,1,0)
\]

得到：

\[
(T^{(C)},C^{(C)},F^{(C)})
\]

### Fuel anchor

\[
\mathbf w^{(F)}
=
(0,0,1)
\]

得到：

\[
(T^{(F)},C^{(F)},F^{(F)})
\]

组成 payoff table：

\[
\mathbf P=
\begin{bmatrix}
T^{(T)} & C^{(T)} & F^{(T)}\\
T^{(C)} & C^{(C)} & F^{(C)}\\
T^{(F)} & C^{(F)} & F^{(F)}
\end{bmatrix}
\]

---

## 7.7.2 normalization range

\[
T_{\min}
=
T^{(T)}
\]

\[
C_{\min}
=
C^{(C)}
\]

\[
F_{\min}
=
F^{(F)}
\]

reference maxima 由三锚点 payoff table 中对应列最大值给出。

一定写成：

> **reference maximum / anchor-derived upper reference**

不要声称是：

> 全部可行轨迹空间的数学全局最大值。

---

## 7.7.3 normalized objectives

\[
T_n=
\frac{T-T_{\min}}
{T_{\max}-T_{\min}}
\]

\[
C_n=
\frac{C-C_{\min}}
{C_{\max}-C_{\min}}
\]

\[
F_n=
\frac{F-F_{\min}}
{F_{\max}-F_{\min}}
\]

---

## 7.7.4 mission-oriented objective

\[
\boxed{
J=
w_TT_n+
w_RC_n+
w_FF_n
}
\]

这里解释：

> 权重表达相对任务偏好，而不是某种固定的“算法最优参数”。

因此不同权重：

\[
\mathbf w
=
(w_T,w_R,w_F)
\]

对应不同任务需求。

---

# 8. Section 3 — Proposed method 详细写法

---

# 8.1 先放 Overall framework

建议 Fig. 1：

```text
Mission definition
      ↓
Threat environment
      ↓
Hard / soft threat representation
      ↓
Anchor-based normalization
      ↓
Coarse multi-objective A*
      ↓
3-D corridor construction
      ↓
Fine multi-objective A*
      ↓
B-spline candidate generation
      ↓
All-s evaluation
      ↓
Curvature / altitude / climb / collision validation
      ↓
Forward–Backward speed planning
      ↓
Final trajectory evaluation
```

---

# 8.2 3.1 Multi-objective risk-aware A*

标准：

\[
f(n)=g(n)+h(n)
\]

路径累计：

\[
g(n)
=
\sum_{e\in\Gamma(n)}
\Delta J_e
\]

其中：

\[
\Delta J_e
=
w_T\frac{\Delta T_e}{T_{\max}-T_{\min}}
+
w_R\frac{\Delta C_e}{C_{\max}-C_{\min}}
+
w_F\frac{\Delta F_e}{F_{\max}-F_{\min}}
\]

这里有一个实现细节：

完整归一化：

\[
\frac{T-T_{\min}}{T_{\max}-T_{\min}}
\]

其中减去 \(T_{\min}\) 是整条路径上的常数项，因此在 additive A* 边累计中只需要 denominator scaling；最终完整路径再恢复完整 normalized objective。

这个细节正文可以简化写，不要把实现推导写太长。

---

## 8.2.1 Neighbor expansion

写：

- 3-D neighboring nodes；
- horizontal/vertical step；
- edge feasibility；
- altitude envelope；
- maximum climb angle；
- hard collision sampling。

---

## 8.2.2 Heuristic

采用乐观时间/距离下界即可。

重点：

> 风险和燃油代价非负，因此启发式可以只使用剩余几何/时间下界，以避免高估。

不要把启发函数包装成创新点。

---

# 8.3 3.2 Coarse-to-fine planning

---

## 8.3.1 Coarse global search

当前：

\[
\Delta x=\Delta y=2000\,m
\]

\[
\Delta z=500\,m
\]

功能：

> 决定从雷达/火力区域哪一侧通过，以及主要的水平与垂向路线拓扑。

---

## 8.3.2 Corridor construction

当前：

\[
R_c=600\,m
\]

注意：

> 这个 corridor 是 Fine A* 的 search-space restriction，不是最终飞行硬边界。

当前 `main.py` 中 B-spline 不强制留在 coarse corridor 内，只把 Fine Raw path locality 作为连续化的约束/诊断。

---

## 8.3.3 Fine replanning

当前：

\[
\Delta x=\Delta y=200\,m
\]

\[
\Delta z=50\,m
\]

在局部 corridor 内重新运行相同 multi-objective A*。

一定写：

> **Fine stage is replanning, not interpolation.**

---

## 8.3.4 为什么两阶段合理

写：

> coarse stage retains global route selection capability with moderate complexity, whereas fine stage improves local spatial resolution only where it is needed.

不要声称一定比全局 fine 在理论最优性上更优。

---

# 8.4 3.3 B-spline trajectory optimization

---

## 8.4.1 为什么需要

离散 A* 路径是 polyline。

在折点：

- tangent discontinuous；
- curvature not continuously defined；
- speed planning unstable / physically meaningless。

所以：

\[
\Gamma_{\rm discrete}
\rightarrow
\mathbf r(u)
\]

---

## 8.4.2 cubic B-spline

\[
\mathbf r(u)=
[x(u),y(u),z(u)]^T
\]

使用 cubic spline：

\[
k=3
\]

获得：

\[
C^2
\]

连续性。

---

## 8.4.3 candidate smoothing parameters

程序当前对 21 个 \(s\) 候选进行全局评价。

不要写：

> increase s until the first feasible solution is found.

当前正确写法：

> all candidate smoothing levels are evaluated, and the best feasible candidate is selected.

---

## 8.4.4 smoothing objective

\[
J_s
=
0.4J_{\rm smooth}
+
0.3J_{\rm curvature}
+
0.2J_{\rm deviation}
+
0.1J_{\rm length}
\]

各项解释：

### \(J_{\rm smooth}\)

反映平均弯曲程度。

### \(J_{\rm curvature}\)

由：

\[
J_{\kappa'}
=
\int
\left(
\frac{d\kappa}{ds}
\right)^2
ds
\]

构造无量纲评价。

### \(J_{\rm deviation}\)

轨迹相对原始 Fine path 的 RMS 偏离。

### \(J_{\rm length}\)

连续轨迹长度相对原始路径长度。

---

# 8.5 解析曲率

\[
\kappa(u)
=
\frac{
\|\mathbf r'(u)\times\mathbf r''(u)\|
}{
\|\mathbf r'(u)\|^3
}
\]

\[
R(u)=
\frac1{\kappa(u)}
\]

强调：

> 对连续 B-spline 使用解析导数计算曲率，而不是密集采样后三点 Menger curvature。

---

# 8.6 最终 hard constraints

每个候选检查：

### Turn

\[
R_{\min}
\ge
R_{\rm feasible}
\]

### Climb

\[
\gamma_{\max}
\le15^\circ
\]

### Altitude

\[
h_{\min}(\mathbf p)
\le z
\le10000
\]

### Endpoints

起点、终点必须保持。

### Terminal heading

限制终端水平航向误差。

### Collision

连续 spline probe 不得进入 hard fire zone。

### Locality

连续轨迹不应过度偏离原规划路径。

---

# 8.7 V5.3 feasibility repair 怎么写

不要用“repair algorithm”作为创新点。

建议一段：

> Cubic interpolation may occasionally produce local vertical overshoot even when the underlying discrete path is feasible. To preserve the hard flight-envelope constraints without relaxing them, the vertical spline profile is projected into the feasible altitude/climb envelope and then refitted. Every repaired candidate is subsequently revalidated against all original constraints.

中文：

> 三次 B 样条在局部可能产生垂向超调。本文不放宽最大爬升角或高度限制，而是将连续轨迹的垂向分量重新投影至可行飞行包络并再次拟合，随后重新检查转弯半径、高度、爬升角、端点、末端航向及硬禁飞碰撞等全部约束。

---

# 8.8 3.4 Adaptive speed planning

---

## 8.8.1 curvature-limited speed

根据转弯：

\[
v_{\rm turn}
=
\sqrt{
R\,g\tan\phi_{\max}
}
\]

---

## 8.8.2 vertical constraint

根据路径坡度和垂向速度限制得到：

\[
v_{\rm vertical}
\]

---

## 8.8.3 local speed cap

\[
v_{\max,i}
=
\min
(
v_{\rm aircraft,max},
v_{\rm turn,i},
v_{\rm vertical,i}
)
\]

并保持：

\[
v_i\ge v_{\rm stall}
\]

---

## 8.8.4 Forward pass

\[
v_{i+1}
\le
\sqrt{
v_i^2
+
2a_{\max}\Delta s
}
\]

---

## 8.8.5 Backward pass

\[
v_i
\le
\sqrt{
v_{i+1}^2
+
2a_{\rm dec}\Delta s
}
\]

---

## 8.8.6 最终统一 evaluator

这一句建议原文放论文：

> **All final mission metrics are evaluated on the smoothed trajectory with the dynamically feasible speed profile rather than inherited from the discrete search path.**

即重新计算：

\[
T_{\rm final}
\]

\[
C_{\rm final}
\]

\[
F_{\rm final}
\]

这样避免：

> A* path 看起来很好，但最终 spline/speed 性能统计仍然用原 A* 数据。

---

# 9. 代码与论文一一对应表

| 程序文件 | 已实现内容 | 论文位置 | 当前状态 |
|---|---|---|---|
| `radar_environment_5zones.py` | 雷达、火力区、三维环境 | 2.1 / 4.1 | 已完成 |
| `environment_threat_model.py` | soft radar risk \(P_d\)、hard collision、\(C=\int P_d dt\) | 2.1 / 2.3 | 已完成 |
| `altitude_constraints.py` | 起终点 0 m、5 km transition、1000–10000 m | 2.2 | 已完成 |
| `fixed_wing_dynamics.py` | stall、turn radius、climb、acceleration、power/fuel | 2.2–2.3 / 3.4 | 已完成 |
| `baseline_manager.py` | 三 anchor payoff table | 2.4 | 已完成 |
| `astar_risk_planner.py` | 多目标 A*、统一 edge/final stats | 3.1 | 已完成 |
| `two_stage_planner.py` | coarse→corridor→fine | 3.2 | 已完成 |
| `trajectory_smoothing.py` | B-spline、all-s、curvature、repair、validation | 3.3–3.4 | 已完成 |
| `main.py` | 完整 two-stage + trajectory pipeline | 4.2 / 4.4 | 已完成 |
| `sweep_2d.py` | 72-case coarse weight sweep | 4.3 | 正在正式重跑 |
| `select_representative_weights.py` | 代表性权重选择 | 4.4 | 已具备 |
| `analyze_weight_sensitivity.py` | 权重敏感性与三目标热力图 | 4.3 | 已具备 |
| `analyze_core_tradeoffs_v2.py` | T/C/F trade-off | 4.3 | 已具备 |
| `analyze_altitude_strategy_v2.py` | altitude 策略解释 | 可选辅助 | 已具备 |
| `analyze_radar_interaction.py` | 雷达净空与穿透分析 | 可选辅助 | 已具备 |

---

# 10. Section 4 — Numerical results and discussion

最终只保留：

```text
4.1 Simulation setup
4.2 Overall performance and controlled ablation
4.3 Mission-preference sensitivity
4.4 Representative mission scenarios
```

---

# 11. 4.1 Simulation setup

---

## 11.1 Table 1：Fixed-wing parameters

建议包含：

| Parameter | Symbol | Value |
|---|---|---:|
| Aircraft mass | \(m\) | 1100 kg |
| Stall speed | \(v_{\rm stall}\) | 33 m/s |
| Cruise speed | \(v_{\rm cruise}\) | 55 m/s |
| Max horizontal speed | \(v_{\max}\) | 83 m/s |
| Max bank angle | \(\phi_{\max}\) | 30° |
| Max climb angle | \(\gamma_{\max}\) | 15° |
| Max acceleration | \(a_{\max}\) | 3 m/s² |
| Max deceleration | \(a_{\rm dec}\) | 4 m/s² |
| Max ascent rate |  | 12 m/s |
| Max descent rate |  | 15 m/s |
| Wing area | \(S\) | 11.5 m² |
| Zero-lift drag coefficient | \(C_{D0}\) | 0.025 |
| Aspect ratio | \(AR\) | 18 |
| Oswald efficiency | \(e\) | 0.85 |

参数要最终从代码核对一次再放正文。

---

## 11.2 Table 2：Planning parameters

建议：

| Parameter | Value |
|---|---:|
| Start | (2000, 2000, 0) m |
| Goal | (98000, 98000, 0) m |
| En-route altitude | 1000–10000 m |
| Altitude transition | 5 km |
| Coarse XY grid | 2000 m |
| Coarse Z grid | 500 m |
| Fine XY grid | 200 m |
| Fine Z grid | 50 m |
| Corridor radius | 600 m |
| Fixed \(w_R\) in sweep | 1 |
| \(w_T\) | 0:0.1:1.1 |
| \(w_F\) | 0:0.1:0.5 |
| Number of weight cases | 72 |

---

# 12. 4.2 Overall performance and controlled ablation

---

# 12.1 4.2.1 Traditional A* vs Proposed

这是后续唯一需要新增的真正 baseline 对比。

---

## 12.1.1 Traditional baseline 怎么定义

保持：

- 同地图；
- 同 start/goal；
- 同 hard fire zones；
- 同 coarse grid；
- 同 neighbor expansion；
- 同 altitude envelope。

区别：

Traditional：

\[
J_e=T_e
\]

或 normalized equivalent：

\[
(w_T,w_R,w_F)
=
(1,0,0)
\]

soft radar risk 不参与搜索。

---

## 12.1.2 为什么这个 baseline 合理

因为你的论文问题不是：

> A* 是否比 RRT* 快。

而是：

> 连续软威胁和任务偏好是否值得被显式加入搜索。

所以最干净的 controlled comparison 就是：

```text
相同搜索器
相同环境
相同硬约束
只改变 objective
```

这样更能证明 contribution 1。

---

## 12.1.3 图

建议 Fig. 4：

背景：

- radar continuous threat field；
- hard fire zones。

路径：

```text
Traditional time A*
Proposed risk-aware A*
```

---

## 12.1.4 表

| Metric | Traditional | Proposed | Change |
|---|---:|---:|---:|
| Distance | | | |
| Time | | | |
| Threat exposure | | | |
| Fuel | | | |
| Max altitude | | | |
| CPU time | | | |

---

## 12.1.5 正确结论写法

如果：

\[
T_{\rm proposed}
>
T_{\rm traditional}
\]

但：

\[
C_{\rm proposed}
<
C_{\rm traditional}
\]

就写：

> The proposed planner sacrifices a limited amount of travel efficiency to obtain substantially lower cumulative threat exposure, demonstrating an explicit risk–efficiency trade-off rather than uniformly improving all metrics.

绝对不要写：

> Proposed outperforms traditional A* in all aspects.

---

# 13. 4.2.2 Coarse vs Fine

---

## 13.1 不需要重新跑 72 次

只取一个代表性完整 `main.py` case。

比较：

```text
Coarse multi-objective A*
vs
Fine raw multi-objective A*
```

---

## 13.2 关注指标

- Distance；
- \(T\)；
- \(C\)；
- \(F\)；
- number of nodes；
- local altitude；
- planning time。

---

## 13.3 图

Fig. 3 可以同时完成这一对比：

```text
coarse path
corridor
fine path
```

---

## 13.4 结论模板

> The coarse solution determines the global threat-avoidance topology, whereas the fine-grid replanning adjusts local geometry and altitude within the refinement corridor without changing the overall mission preference.

不要硬说：

> Fine 一定全面优于 coarse。

---

# 14. 4.2.3 Fine Raw vs B-spline

---

## 14.1 核心问题

> collision-free path 是否等于 fixed-wing executable trajectory？

答案：

不是。

---

## 14.2 对比对象

```text
Fine Raw polyline
vs
Fine + B-spline + adaptive speed
```

---

## 14.3 指标

### Geometry

- path length；
- heading continuity。

### Curvature

- \(\kappa_{\max}\)；
- \(R_{\min}\)；
- \(J_{\kappa'}\)。

### Vertical

- \(\gamma_{\max}\)。

### Dynamics

- speed；
- acceleration。

---

## 14.4 注意 Raw curvature 怎么写

不要把 raw polyline 在折点处的三点 Menger radius 当作严格连续曲率。

正确写：

> For the discrete fine path, a three-point geometric turning radius is reported only as a grid-scale reference because the polyline itself has discontinuous tangent directions at vertices. The B-spline trajectory is evaluated using analytical curvature derivatives.

---

## 14.5 建议 Fig. 5 四联图

```text
(a) Fine Raw vs B-spline 3-D / XY
(b) curvature or turning-radius profile
(c) speed profile
(d) acceleration profile
```

---

# 15. 4.3 Mission-preference sensitivity

---

# 15.1 当前 sweep 设计

固定：

\[
w_R=1
\]

扫描：

\[
w_T=
0,0.1,\ldots,1.1
\]

\[
w_F=
0,0.1,\ldots,0.5
\]

共：

\[
12\times6=72
\]

组。

---

# 15.2 非常重要：72 组是什么，不是什么

当前 sweep：

```text
coarse A*
→ B-spline
→ speed
→ final T/C/F
```

不是：

```text
coarse
→ fine
→ B-spline
→ speed
```

所以论文中一定写：

> The 72-case sweep is used as a global mission-preference screening experiment on the coarse planning space.

目的：

- 找到权重空间变化趋势；
- 识别策略分区；
- 自动选择 representative weights。

然后再对代表权重运行完整 two-stage。

这是非常合理的计算折中。

---

# 15.3 为什么固定 \(w_R=1\)

因为归一化后：

\[
w_T:w_R:w_F
\]

表达相对偏好。

因此取：

\[
w_R=1
\]

作为 risk reference。

改变：

\[
w_T,\quad w_F
\]

研究时间和燃油相对于风险的重要性。

---

# 15.4 最少只画 4 类结果

---

## Fig. 6a Time heatmap

\[
(w_T,w_F)
\rightarrow
T
\]

---

## Fig. 6b Threat heatmap

\[
(w_T,w_F)
\rightarrow
C
\]

---

## Fig. 6c Fuel heatmap

\[
(w_T,w_F)
\rightarrow
F
\]

把三张合成一个 figure。

---

## Fig. 7 Trade-off scatter

x：

\[
T
\]

y：

\[
C
\]

color：

\[
F
\]

这张图非常关键。

---

# 15.5 不要强行找“最佳权重”

正确说法：

> different weights correspond to different mission preferences.

而不是：

> 0.3/1/0.2 is the optimal weight.

---

# 16. 4.4 Representative mission scenarios

---

# 16.1 Risk-dominant

\[
i_R
=
\arg\min_i C_i
\]

---

# 16.2 Time-dominant

\[
i_T
=
\arg\min_iT_i
\]

---

# 16.3 Balanced

对 72 组最终结果再归一化：

\[
T_i'
=
\frac{T_i-\min T}
{\max T-\min T}
\]

\[
C_i'
=
\frac{C_i-\min C}
{\max C-\min C}
\]

\[
F_i'
=
\frac{F_i-\min F}
{\max F-\min F}
\]

理想点距离：

\[
D_i=
\sqrt{
(T_i')^2+
(C_i')^2+
(F_i')^2
}
\]

选：

\[
i_B=
\arg\min_iD_i
\]

注意：

这里用于代表工况选择的 normalization 可以基于 sweep 数据本身。

不要与 Section 2 中 anchor-based objective normalization 混为一谈。

论文要明确：

- Section 2 normalization：用于 A* objective；
- Section 4 ideal-point normalization：用于从 72 组结果中选 Balanced case。

---

# 16.4 Fuel-dominant

只在 trade-off 图中标：

\[
i_F=
\arg\min_iF_i
\]

不单独再跑第四套 detailed trajectory，除非版面富余。

---

# 17. 三组 representative case 需要重新跑什么

只对：

```text
Risk
Time
Balanced
```

运行完整：

```text
Coarse A*
↓
600 m corridor
↓
Fine A*
↓
B-spline
↓
adaptive speed
↓
final evaluator
```

不是再跑 72 个 fine case。

---

# 18. 还需要补什么：最终最小清单

---

## 必须补 A：72-case V5.3 sweep

当前正在进行。

完成后先做 numerical audit：

```text
72/72 rows?
any failure?
any NaN?
any inf?
altitude valid?
gamma <= 15°?
Rmin feasible?
T/C/F finite?
weights actually produce different strategies?
```

---

## 必须补 B：Traditional time A*

只补 **1 个 case**。

推荐直接使用：

\[
(1,0,0)
\]

anchor。

---

## 必须补 C：3 representative full main

Risk / Time / Balanced。

只需 **3 次完整 main**。

---

## 必须整理 D：Coarse vs Fine

从上述 3 次 main 里选一个主 case，直接提取。

无需新搜索。

---

## 必须整理 E：Fine Raw vs B-spline

同样直接从 main 中提取。

无需重新 A*。

---

# 19. 明确不再补什么

首稿阶段不做：

- RRT*；
- RRT；
- ACO；
- PSO；
- GA；
- DRL；
- D* Lite；
- Hybrid A*；
- 多 spline 方法；
- Bézier 对比；
- 多 random seeds；
- corridor sensitivity；
- grid convergence；
- 完整 \(w_R\) sensitivity；
- 动态威胁；
- multi-UAV；
- 6-DOF high-fidelity dynamics。

如果审稿人返修明确要求，再选最必要的一项。

---

# 20. 为什么首稿可以不和很多外部算法比较

你的核心 claim 不是：

> 搜索复杂度比 RRT* 更好。

而是：

```text
soft threat modeling
+
mission preference
+
coarse-to-fine
+
dynamic executability
```

因此首稿最有针对性的验证方式是 controlled ablation：

```text
Traditional objective
vs
Proposed objective

Coarse
vs
Fine

Fine Raw
vs
B-spline
```

它们分别对应三个 contribution。

这比随便找几个不同范式算法然后比较一个最终距离，更有解释力。

但是要接受一个现实：

> AST reviewer 有可能要求 state-of-the-art planner comparison。

应对策略：

> 首稿不做；如 reviewer 明确要求，返修只补 1 个最接近本文 problem formulation 的 risk-aware / multi-objective 3-D planner。

---

# 21. 论文图表最终建议

---

# Figures

## Fig. 1 Overall planning framework

全文最重要方法图。

---

## Fig. 2 3-D threat environment

显示：

- radar zones；
- fire zones；
- start；
- goal；
- altitude concept。

---

## Fig. 3 Coarse–corridor–fine

三者同图。

---

## Fig. 4 Traditional vs Proposed

背景带 continuous radar risk。

---

## Fig. 5 Raw vs B-spline dynamic feasibility

4 panels：

```text
(a) geometry
(b) curvature / radius
(c) speed
(d) acceleration
```

---

## Fig. 6 Weight sensitivity heatmaps

3 panels：

```text
(a) T
(b) C
(c) F
```

---

## Fig. 7 T–C–F trade-off

scatter。

---

## Fig. 8 Representative trajectories

Risk / Time / Balanced。

---

# Tables

## Table 1 UAV parameters

## Table 2 Environment and planning parameters

## Table 3 Controlled ablation metrics

建议可以做成：

| Case | L | T | C | F | Rmin | gamma_max | CPU |
|---|---:|---:|---:|---:|---:|---:|---:|
| Traditional | | | | | | | |
| Proposed | | | | | | | |
| Coarse | | | | | | | |
| Fine Raw | | | | | | | |
| B-spline final | | | | | | | |

如果指标定义不完全一致，可以拆成 Table 3a / 3b。

## Table 4 Representative mission cases

Risk / Time / Balanced。

---

# 22. V5.3 中几个工程参数在论文里怎么定位

---

## 22.1 Coarse sweep trust radius

当前 coarse grid：

\[
\Delta x=\Delta y=2000\,m
\]

\[
\Delta z=500\,m
\]

最大 3-D neighbor step：

\[
D_{\rm grid}
=
\sqrt{
2000^2+
2000^2+
500^2
}
\approx2872.3\,m
\]

V5.3：

\[
D_{\rm trust}
=
1.05D_{\rm grid}
\approx3015.9\,m
\]

它只用于：

> coarse-sweep B-spline locality control。

不要声称它是创新。

---

## 22.2 Altitude repair

只作为 numerical feasibility maintenance。

---

## 22.3 Climb-angle repair

同样只是：

> spline overshoot correction under hard \(\gamma_{\max}\) constraint。

不要写成第四创新点。

---

# 23. 当前可以直接写到什么程度

---

## Section 1

可以写：

- background；
- gap；
- contributions；
- organization。

Related work 需要正式查文献后补引用。

---

## Section 2

现在可以写 **100%**。

---

## Section 3

现在可以写 **100%**。

---

## Section 4.1

现在可以写 **100%**。

---

## Section 4.2

结构和方法可以先写，数值留空。

---

## Section 4.3

实验设计可写，数值等 sweep。

---

## Section 4.4

选择方法可写，具体 weights 等 sweep。

---

## Abstract / Conclusions

框架先写，最终数字最后填。

---

# 24. 现在不要写死什么

不要写：

> risk reduced by XX%

不要写：

> fuel improved by XX%

不要写：

> best weight is XX

不要写：

> proposed is superior to all baselines

直到最终 experiment data 完整。

---

# 25. 建议 Section 4 的每个实验如何回答审稿人的问题

---

## Traditional vs Proposed

审稿人隐含问题：

> 为什么不是普通 A*？

回答：

> soft threat + mission objective 改变了路线选择。

---

## Coarse vs Fine

隐含问题：

> 为什么要多一层 fine？

回答：

> coarse 确定 topology，fine 提高局部空间和高度精度。

---

## Fine Raw vs B-spline

隐含问题：

> 你的 A* path 为什么不能直接执行？

回答：

> tangent / curvature / speed 不连续，因此必须 trajectory generation。

---

## 72 weights

隐含问题：

> 这些 weights 到底有什么实际意义？

回答：

> weights are mission-preference parameters and systematically map to different mission outcomes.

---

# 26. 文章中建议反复出现的 4 个关键词

建议全文保持一致：

### Risk-aware

对应 soft radar exposure。

### Mission-preference-driven

对应 weights。

### Coarse-to-fine

对应 spatial refinement。

### Dynamically feasible

对应 B-spline + fixed-wing constraints + speed。

---

# 27. 不建议频繁使用的词

少用：

- improved A*；
- novel B-spline；
- optimal trajectory（除非明确是相对 objective / discretization）；
- globally optimal（不要轻易写）；
- superior in all aspects。

更稳妥：

- proposed framework；
- mission-oriented objective；
- risk-aware search；
- dynamically feasible trajectory；
- representative mission case；
- trade-off。

---

# 28. 可能的审稿人问题与预备答案

---

## Q1. Why not compare with RRT*/PSO/ACO?

答题思路：

> 本文贡献集中于 threat representation、mission-oriented cost consistency、spatial refinement and dynamic executability，因此首要验证采用 controlled ablation，在相同搜索器和环境下逐层改变 objective / resolution / trajectory processing，以隔离每个模块的作用。

如果 reviewer 要求，再补 1 个。

---

## Q2. Why use weighted sum?

答：

> weights directly represent mission preference；objective normalization reduces scale inconsistency；the paper does not claim one universal optimum weight but studies systematic behavior over 72 preferences.

---

## Q3. Why fix \(w_R=1\)?

答：

> normalized objectives make only relative ratios important；\(w_R=1\) is used as a reference scale and \(w_T,w_F\) vary relative to threat.

---

## Q4. Why not run all 72 cases with fine planning?

答：

> The sweep is intended for global preference screening; full fine-grid planning is computationally expensive and unnecessary for every preference. Representative cases are then evaluated through the complete two-stage pipeline.

---

## Q5. Is the B-spline itself a contribution?

答：

> No. The contribution is the integration of continuous trajectory generation and fixed-wing feasibility into the mission-oriented planning chain.

---

## Q6. Does the 3-anchor normalization provide true global maxima?

答：

> No. It provides anchor-derived reference ranges from a single-objective payoff table and is used for consistent objective scaling.

---

## Q7. Why can start and goal be 0 m when minimum altitude is 1000 m?

答：

> A spatial altitude envelope is defined with 5-km departure and arrival transitions; the 1000-m minimum applies to en-route flight rather than exact endpoints.

---

# 29. 最终实验数据需要建立一个“锁定表”

当实验完成后，建议建立：

```text
FINAL_EXPERIMENT_LOG.md
```

记录：

- Git / folder version；
- date；
- seed；
- baseline；
- grid；
- weights；
- start / goal；
- parameter tables；
- exact Excel filename；
- representative weights；
- Traditional case；
- main output filenames。

以后论文数字只从这个版本拿。

不要：

> 今天用 V5.2 的一张图，明天用 V5.3 的 Excel。

---

# 30. 写作顺序建议

现在：

```text
Step 1
写 Section 2

Step 2
写 Section 3

Step 3
画 Fig. 1 overall framework

Step 4
写 Section 1 research gap + contributions

Step 5
整理 Table 1 / Table 2

Step 6
等 72 sweep 完成

Step 7
检查 72 数据

Step 8
选 Risk / Time / Balanced

Step 9
跑 3 个完整 main

Step 10
Traditional baseline

Step 11
整理 ablation

Step 12
完成 Section 4

Step 13
写 Abstract

Step 14
写 Conclusions

Step 15
统一 notation / figures / English
```

---

# 31. 什么时候正式停止改程序

满足下面条件：

- V5.3 preflight PASS；
- 72 sweep 数据完整；
- Traditional baseline 成功；
- Risk / Time / Balanced 成功；
- final trajectory hard constraints 全部满足；
- \(T,C,F\) 全部由 final evaluator；
- 图表可生成。

此后：

> **除明确 bug 或 reviewer 要求外，不再修改 objective、grid、altitude、smoothing strategy、speed model。**

只做：

- writing；
- plotting；
- formatting；
- literature；
- language polishing。

---

# 32. 最终最小工作量总结

真正还需要“大计算”的：

```text
1 × 72-case coarse sweep
1 × Traditional baseline
3 × representative full two-stage main
```

基本就这些。

---

# 33. 最终证据链

---

## Evidence 1

\[
\text{Traditional}
\rightarrow
\text{Proposed}
\]

证明：

> soft threat / mission objective 有必要。

---

## Evidence 2

\[
\text{Coarse}
\rightarrow
\text{Fine}
\]

证明：

> spatial refinement 有必要。

---

## Evidence 3

\[
\text{Fine Raw}
\rightarrow
\text{B-spline + speed}
\]

证明：

> path ≠ executable trajectory。

---

## Evidence 4

\[
72\text{ weights}
\]

证明：

> mission preference 会系统改变 trajectory strategy。

---

# 34. 最后建议你全文反复强调的一句话

> **The main contribution is not an isolated modification to A*, but a consistent planning chain that connects mission preference, threat-aware spatial search, multi-resolution refinement, and fixed-wing dynamic executability.**

中文：

> **本文的核心贡献并非对 A* 的孤立修改，而是建立了一条贯通任务偏好、连续威胁感知、空间多分辨率细化与固定翼动力学可执行化的统一规划链。**

这句话基本可以作为：

- Introduction 末段；
- Discussion；
- Conclusion；

三个位置的核心思想。

---

# 35. 当前建议立刻执行的事情

如果 V5.3 sweep 已经开始：

> **不要暂停去改算法。**

你现在可以同步开始写：

```text
Section 2
Section 3
Section 4.1
Introduction 的 research gap + contributions
```

等 sweep 完成后再补：

```text
Section 4.3
Section 4.4
Traditional comparison
```

这样时间利用率最高。

---

# 36. 首稿完成标准

达到以下状态就应该开始准备投稿，而不是继续堆内容：

- 一个清晰的研究问题；
- 三个克制的贡献；
- 一条完整 method pipeline；
- 一个 traditional controlled baseline；
- 一个 coarse/fine ablation；
- 一个 raw/smooth ablation；
- 一个 72-case preference study；
- 三个 representative mission cases；
- 完整固定翼 dynamic feasibility；
- 清晰 limitation。

这已经构成完整 AST Research Article 的骨架。

---

# 37. 最终执行路线图（压缩版）

```text
现在
│
├─ V5.3 72-case sweep
│
├─ 写 Section 2 / 3 / 4.1
│
▼
72 数据完成
│
├─ 数值体检
├─ T/C/F heatmaps
├─ T-C-F scatter
│
▼
选择 Risk / Time / Balanced
│
├─ 3 × full main
│
▼
补 Traditional baseline
│
▼
整理 Coarse vs Fine
│
▼
整理 Raw vs B-spline
│
▼
锁死数据和代码
│
▼
完成 Section 4
│
▼
Abstract + Conclusions
│
▼
English polishing
│
▼
AST submission
```

---

# 38. 最终一句话

从现在开始最重要的不是让程序“再多一个功能”，而是让每个现有模块都在论文中回答一个明确的问题：

```text
Multi-objective cost
→ 为什么需要 mission preference？

Coarse-to-fine
→ 为什么需要 spatial refinement？

B-spline + dynamics
→ 为什么 path 还不够？

72 weights
→ mission preference 是否真的产生系统性结果变化？
```

只要这四个问题都有清晰实验回答，这篇文章的逻辑就已经闭环。
