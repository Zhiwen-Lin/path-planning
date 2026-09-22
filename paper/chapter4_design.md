# 第四章紧凑版：三组对照、四张敏感性图、三个代表偏好

本文件采用用户最新结构，取代上一轮按三个贡献各设一个subsection的展开版。只改论文与检查，不改规划算法、代表选择器或sweep缓存。

## 结构及证据对应

| 小节 | 内容 | 证明的问题 | 数据与图表 |
| --- | --- | --- | --- |
| 4.1 Simulation setup and evaluation protocols | 环境、参数、评价协议 | 结果在什么条件下成立 | 环境图、两张参数表 |
| 4.2 Overall planning performance and controlled comparisons | A. 时间单目标 vs Proposed；B. Coarse vs Fine Raw；C. Fine Raw vs B-spline | 分别验证代价建模、局部重规划和固定翼轨迹生成 | A用共同协议的两次完整运行；B/C共用同次main |
| 4.3 Mission-preference sensitivity | 72组偏好响应 | 时间、暴露、燃油如何权衡 | T/C/F三张热力图；T-C散点以F着色 |
| 4.4 Representative mission preferences | Risk/Time/Balanced | 用可解释案例收束任务取舍 | 同批sweep最终缓存；一张三维叠加图和一张表 |

4.2改称controlled comparisons：A改变复合代价，B/C比较同次运行的阶段输出，不将其包装成完整因素消融。

## 4.2：三个贡献压入同一小节

**A. 时间单目标与多目标。** 基线权重(1,0,0)，仍满足地图、高度、爬升和硬禁飞约束。只是不把软暴露与燃油加入搜索代价，并非A*不能处理连续代价。两者使用相同环境、网格、基准与后处理流程；各自产生的fine corridor可能不同。主表比较通过最终验收的L/T/C/F。较低C伴随较高T是可以解释的权衡，不是必须出现的结果。不能把软雷达区全部设为基线障碍。

**B. Coarse与Fine Raw。** 直接读取同次main的planned_path_coarse.pkl与planned_path.pkl中的path_raw/stats_raw。展示三维走廊中的局部转向、高度调整和搜索口径指标；无需独立新实验。不把最后的样条轨迹放进这一组比较，也不在没有全域fine基线时报告对全域fine的提速倍数。

**C. Fine Raw与最终轨迹。** 复用同次main的原始细路径、样条、解析曲率、速度及加速度。重点是最小转弯半径、曲率、角度、v(s)、a(s)、高度/碰撞/航向验收，不追求T/C/F全部下降。原始折线拐角没有可直接等同于连续样条的曲率；其三点估计只作诊断。最终指标必须按最终速度重算，不能复制A*统计。

本小节不再拆更多三级标题；A/B/C的英文协议与中文步骤均在section4_experiments.tex。没有结果的位置不填造数值。

## 4.3：只保留四张核心图

1. Time heatmap：横轴w_T、纵轴w_F、色值T(s)。
2. Threat-exposure heatmap：同一网格、色值C(exposure-s)。
3. Fuel heatmap：同一网格、色值F(kg)。
4. T-C scatter：横轴T、纵轴C、颜色F。

w_C=1，w_T为0至1.1步长0.1，w_F为0至0.5步长0.1，共72组。正文不再追加切片图、最大高度图、行为分区图；这些可用于内部检查而不占正文。不同指标不同量纲，不强行用相同数值色标。

数据先通过版本、签名、完整性与成功状态验收。当前绘图脚本已有相关输出；三张独立热力图的文件名是计划中的图位，本次没有修改脚本或生成图片。后续从已验收数据导出即可，不重跑搜索。

## 4.4：三个代表，一图一表

- Risk-dominant：最小C；并列按T、F、w_T、w_F升序。
- Time-dominant：最小T；并列按C、F、w_T、w_F升序。
- Balanced：对72组内T/C/F分别做min-max缩放，取欧氏理想点距离最小者；常数列贡献为0；并列按T、C、F、w_T、w_F升序。

公式已写入LaTeX并与select_representative_weights.py核对。批内缩放仅用于代表选择，不替代搜索的三锚点归一化。代码另有省油输出，但本文主展示只用三个角色。若代表重合，如实注明，不强行换一个工况。

**默认直接使用所选sweep案例的最终缓存。** 一张三维叠加图与一张Case/w_T/w_C/w_F/T/C/F表均来自这一协议，不额外要求三个代表重跑main。完整两阶段的证据由4.2承担；不得混用main轨迹与sweep指标。表中--为待填占位，不是零值。

正文统一用w_C对应代码w_risk。若投稿时选择w_R记号，应全文一致替换，不在图表与公式间交替使用。

## 与引言衔接

引言用三个问题导入：可通行区域仍有不同软暴露；全域高分辨率三维搜索开销；离散可通行路径与固定翼连续可执行轨迹之间的差别。不要用“传统A*只知道撞与不撞”概括通用算法。

相关工作按三类组织：风险感知/多目标规划；分层/多分辨率规划；固定翼连续化与动力学可行性。每类说明已有工作、在本文问题设置下仍待解决的环节以及本文关系，避免长篇罗列。

研究缺口需由实际文献支撑，不能仅因把模块连起来就宣布已有工作都没有一致规划链。文献核对完成前，可以采用以本文问题为中心的表述：

> The problem addressed here is to connect mission-dependent time--exposure--fuel route selection, local three-dimensional refinement, and flight-envelope-constrained trajectory generation while retaining consistent objective references and final performance evaluation.

这段不宣称无人研究过组合框架。现已在section1_introduction.tex写入1.1背景、1.2相关工作、1.3研究重点与贡献，并补入经核实的引文。文献与主张对应见literature_alignment.md。

## 最小执行顺序

先验收已有72组，完成4.1、4.3和4.4。随后为4.2复用或补齐时间基线与一个明确多目标权重的完整main结果；B/C共用其中同次main。已有有效完整结果则不重跑。仅在缺少基线/多目标结果时补对应配置，不把整个72组当作重跑任务。

最后检查图表单位、raw/final口径、实际数值和约束证据，再补摘要和结论的实测结果段。摘要和结论的方法部分已写好。可以报告内部受控比较的结论，不能据此声称胜过所有现有方法或跨场景泛化。本次未执行新实验，也未完成TeX编译。
