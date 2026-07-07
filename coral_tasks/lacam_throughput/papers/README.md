# Paper Library README

这是 CORAL LaCAM throughput 任务的论文库入口。检索必须按渐进式披露进行：

1. 先读本 README，用一句话简介筛选候选论文。
2. 再打开对应 `briefs/*.md` 简介页，确认该论文是否能支撑当前 idea。
3. 只有需要章节、算法、设计模式或实验观察细节时，才打开简介页链接的原论文 HTML/PDF。

硬性规则：所有 throughput idea、候选假设、pivot 方向和非平凡实现任务都必须来自本论文库；记录时必须引用至少一个 `briefs/*.md` 简介页，并在需要时引用原论文中的具体锚点。

当前可用本地论文/资料：76 篇。完整下载元数据见 [index.md](index.md)，未解析来源见 [unresolved.md](unresolved.md)。

## Idea 分类路由

在打开具体 brief 前，先用 [idea_categories.md](idea_categories.md) 给候选 idea 选一个主类别，并在实验 note 与 `coral eval -m` 中记录：

- **Map / guidance-graph / weight optimization**：论文主要提供地图、通道、边方向、边权、区域压力、拥堵或任务分配权重方向。只能转化为通用算法代价，不能修改固定 benchmark map/task/seed。
- **Algorithm design**：论文主要提供求解器机制，例如 LaCAM 配置生成、目标-路径耦合、PIBT/优先级继承、局部修复、窗口化、commitment 或实时预算控制。
- **Heuristic function design**：论文主要提供 move ordering、tie-breaker、lower bound、local guidance、冲突估计、候选排序或学习启发式的可实现代理。

当前已确认 real 提升来自 **heuristic function design**，主要对应 [2605.16855](briefs/2605.16855.md) 的 local guidance / hindrance avoidance 思路；详表见 [idea_categories.md](idea_categories.md#observed-paper-grounded-results-so-far)。

## arcs-group

- [2103.08835 - Multi-Robot Routing with Time Windows: A Column Generation Approach](briefs/2103.08835.md)：研究多机器人路由与时间窗/覆盖约束，可借鉴任务排序、容量和路径分解。
- [2203.02475 - Cooperative Task and Motion Planning for Multi-Arm Assembly Systems](briefs/2203.02475.md)：面向多机器人任务-运动规划，可借鉴约束分解、协同装配和局部可行性检查。
- [2305.06436 - Multi-Robot Coordination and Layout Design for Automated Warehousing](briefs/2305.06436.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2307.00663 - Solving Multi-Agent Target Assignment and Path Finding with a Single Constraint Tree](briefs/2307.00663.md)：联合求解目标分配与路径规划，可启发 pickup/drop 选择和路径代价的耦合设计。
- [2308.11234 - Traffic Flow Optimisation for Lifelong Multi-Agent Path Finding](briefs/2308.11234.md)：把交通流优化用于 lifelong MAPF，可启发通道流向、拥堵缓解和吞吐诊断。
- [2310.18622 - Arbitrarily Scalable Environment Generators via Neural Cellular Automata](briefs/2310.18622.md)：提供 MAPF 环境、测试床或设计取舍分析，可辅助构造诊断指标和压力场景。
- [2311.14145 - Multi-Agent Motion Planning with Bézier Curve Optimization under Kinodynamic Constraints](briefs/2311.14145.md)：关注连续轨迹、动力学或扩散式规划，可借鉴平滑、约束和执行可行性思想。
- [2401.00315 - Bidirectional Temporal Plan Graph: Enabling Switchable Passing Orders for More Efficient Multi-Agent Path Finding Plan Execution](briefs/2401.00315.md)：研究时序依赖图和执行顺序切换，可启发局部等待、通过顺序与执行鲁棒性。
- [2401.17044 - Scalable Mechanism Design for Multi-Agent Path Finding](briefs/2401.17044.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2402.01446 - IJCAI-24 Formatting Instructions](briefs/2402.01446.md)：疑似会议格式模板误抓取，使用前应先确认是否可作为有效 MAPF 论文来源。
- [2403.18145 - A Real-Time Rescheduling Algorithm for Multi-robot Plan Execution](briefs/2403.18145.md)：研究故障容错或实时重调度，可启发执行失败后的快速恢复和局部修补。
- [2403.20300 - Improving Learnt Local MAPF Policies with Heuristic Search](briefs/2403.20300.md)：改进 MAPF 搜索启发式，可启发冲突排序、下界估计和候选选择。
- [2404.00143 - Accelerating Search-Based Planning for Multi-Robot Manipulation by Leveraging Online-Generated Experiences](briefs/2404.00143.md)：面向多机器人任务-运动规划，可借鉴约束分解、协同装配和局部可行性检查。
- [2404.05223 - ITA-ECBS: A Bounded-Suboptimal Algorithm for the Combined Target-Assignment and Path-Finding Problem](briefs/2404.05223.md)：围绕 CBS/ECBS 的冲突分解和有界次优搜索，可启发冲突诊断与局部修复。
- [2404.16162 - Scaling Lifelong Multi-Agent Path Finding to More Realistic Settings: Research Challenges and Opportunities](briefs/2404.16162.md)：面向 lifelong MAPF/AGV 持续任务，提供吞吐、拥堵和真实执行设置参考。
- [2405.01772 - Unconstraining Multi-Robot Manipulation: Enabling Arbitrary Constraints in ECBS with Bounded Sub-Optimality](briefs/2405.01772.md)：围绕 CBS/ECBS 的冲突分解和有界次优搜索，可启发冲突诊断与局部修复。
- [2405.17794 - LNS2+RL: Combining Multi-Agent Reinforcement Learning with Large Neighborhood Search in Multi-Agent Path Finding](briefs/2405.17794.md)：用大邻域搜索修复 MAPF 解，可启发局部重规划、破坏-修复和失败复用。
- [2407.09451 - Reevaluation of Large Neighborhood Search for MAPF: Findings and Opportunities](briefs/2407.09451.md)：用大邻域搜索修复 MAPF 解，可启发局部重规划、破坏-修复和失败复用。
- [2409.06888 - QD-MAPPER: A Quality Diversity Framework to Automatically Evaluate Multi-Agent Path Finding Algorithms in Diverse Maps](briefs/2409.06888.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2409.14491 - Work Smarter Not Harder: Simple Imitation Learning with CS-PIBT Outperforms Large Scale Imitation Learning for MAPF](briefs/2409.14491.md)：围绕 PIBT/优先级规划改进局部避让，可启发密集场景的优先级和冲突处理。
- [2410.01798 - Windowed MAPF with Completeness Guarantees](briefs/2410.01798.md)：研究带完备性保证的窗口化 MAPF，可启发有限时域重规划和稳定性控制。
- [2410.03072 - Multi-Robot Motion Planning with Diffusion Models](briefs/2410.03072.md)：关注连续轨迹、动力学或扩散式规划，可借鉴平滑、约束和执行可行性思想。
- [2410.21415 - Deploying Ten Thousand Robots: Scalable Imitation Learning for Lifelong Multi-Agent Path Finding](briefs/2410.21415.md)：面向 lifelong MAPF/AGV 持续任务，提供吞吐、拥堵和真实执行设置参考。
- [2411.16506 - Online Guidance Graph Optimization for Lifelong Multi-Agent Path Finding](briefs/2411.16506.md)：优化 lifelong MAPF 的在线引导图，可启发通道方向、边权和拥堵调节。
- [2412.13359 - Multi-Agent Motion Planning For Differential Drive Robots Through Stationary State Search](briefs/2412.13359.md)：关注连续轨迹、动力学或扩散式规划，可借鉴平滑、约束和执行可行性思想。
- [2412.15908 - Speedup Techniques for Switchable Temporal Plan Graph Optimization](briefs/2412.15908.md)：研究时序依赖图和执行顺序切换，可启发局部等待、通过顺序与执行鲁棒性。
- [2503.02992 - RAILGUN: A Unified Convolutional Policy for Multi-Agent Path Finding Across Different Environments and Tasks](briefs/2503.02992.md)：用学习模型指导 MAPF 决策，可启发优先级、局部策略和搜索候选排序。
- [2503.03779 - Accelerating Focal Search in Multi-Agent Path Finding with Tighter Lower Bounds](briefs/2503.03779.md)：改进 MAPF 搜索启发式，可启发冲突排序、下界估计和候选选择。
- [2503.04798 - Advancing MAPF Toward the Real World: A Scalable Multi-Agent Realistic Testbed (SMART)](briefs/2503.04798.md)：提供 MAPF 环境、测试床或设计取舍分析，可辅助构造诊断指标和压力场景。
- [2503.15836 - APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly](briefs/2503.15836.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2504.06091 - Real-Time LaCAM for Real-Time MAPF](briefs/2504.06091.md)：研究实时 LaCAM，在限时 MAPF 中压缩搜索开销并保持可用解质量。
- [2507.17054 - New Mechanisms in Flex Distribution for Bounded Suboptimal Multi-Agent Path Finding](briefs/2507.17054.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2508.01495 - WinkTPG: An Execution Framework for Multi-Agent Path Finding Using Temporal Reasoning](briefs/2508.01495.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2508.04849 - BTPG-max: Achieving Local Maximal Bidirectional Pairs for Bidirectional Temporal Plan Graphs](briefs/2508.04849.md)：研究时序依赖图和执行顺序切换，可启发局部等待、通过顺序与执行鲁棒性。
- [2508.05027 - Benchmarking Shortcutting Techniques for Multi-Robot-Arm Motion Planning](briefs/2508.05027.md)：提供 MAPF 环境、测试床或设计取舍分析，可辅助构造诊断指标和压力场景。
- [2509.15381 - Dynamic Agent Grouping ECBS: Scaling Windowed Multi-Agent Path Finding with Completeness Guarantees](briefs/2509.15381.md)：围绕 CBS/ECBS 的冲突分解和有界次优搜索，可启发冲突诊断与局部修复。
- [2510.00425 - Conflict-Based Search as a Protocol: A Multi-Agent Motion Planning Protocol for Heterogeneous Agents, Solvers, and Independent Tasks](briefs/2510.00425.md)：围绕 CBS/ECBS 的冲突分解和有界次优搜索，可启发冲突诊断与局部修复。
- [2510.03472 - Destination-to-Chutes Task Mapping Optimization for Multi-Robot Coordination in Robotic Sorting Systems](briefs/2510.03472.md)：面向仓储/AGV 协同场景，可启发站点分配、通道容量和吞吐评测设计。
- [2512.09736 - Analyzing Planner Design Trade-offs for MAPF under ADG-based Realistic Execution](briefs/2512.09736.md)：研究时序依赖图和执行顺序切换，可启发局部等待、通过顺序与执行鲁棒性。
- [2602.15721 - Lifelong Scalable Multi-Agent Realistic Testbed and A Comprehensive Study on Design Choices in Lifelong AGV Fleet Management Systems](briefs/2602.15721.md)：面向 lifelong MAPF/AGV 持续任务，提供吞吐、拥堵和真实执行设置参考。
- [2602.23468 - Optimization of Edge Directions and Weights for Mixed Guidance Graphs in Lifelong Multi-Agent Path Finding](briefs/2602.23468.md)：优化 lifelong MAPF 的在线引导图，可启发通道方向、边权和拥堵调节。
- [2603.23405 - Planning over MAPF Agent Dependencies via Multi-Dependency PIBT](briefs/2603.23405.md)：围绕 PIBT/优先级规划改进局部避让，可启发密集场景的优先级和冲突处理。
- [2604.15610 - Scalable Algorithms with Provable Optimality Bounds for the Multiple Watchman Route Problem](briefs/2604.15610.md)：研究多机器人路由与时间窗/覆盖约束，可借鉴任务排序、容量和路径分解。
- [Poster_cbsh2-poster - Improved Heuristics for Multi-Agent Path Finding with Conflict-Based Search](briefs/poster-cbsh2-poster.md)：改进 MAPF 搜索启发式，可启发冲突排序、下界估计和候选选择。
- [Preprint_2021-HPLAN - A Hierarchical Approach to Multi-Agent Path Finding](briefs/preprint-2021-hplan.md)：用层级分解扩展 MAPF 求解，可启发区域划分、分层搜索和跨区协调。
- [Preprint_ChanWoMAPF20 - Nested ECBS for Bounded-Suboptimal Multi-Agent Path Finding](briefs/preprint-chanwomapf20.md)：围绕 CBS/ECBS 的冲突分解和有界次优搜索，可启发冲突诊断与局部修复。
- [Slides_cbsh2 - Improved Heuristics for Multi-Agent Path Finding with Conflict-Based Search](briefs/slides-cbsh2.md)：改进 MAPF 搜索启发式，可启发冲突排序、下界估计和候选选择。
- [pdf_phd-thesis-final - Efficient and Effective Techniques for Large-Scale Multi-Agent Path Finding](briefs/pdf-phd-thesis-final.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。

## kei18

- [1905.10149 - winPIBT: Extended Prioritized Algorithm for Iterative Multi-agent Path Finding](briefs/1905.10149.md)：围绕 PIBT/优先级规划改进局部避让，可启发密集场景的优先级和冲突处理。
- [2102.12331 - Iterative Refinement for Real-Time Multi-Robot Path Planning](briefs/2102.12331.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2102.12748 - Active Modular Environment for Robot Navigation](briefs/2102.12748.md)：介绍模块化多机器人实验环境，可参考其状态管理和可复现实验组织。
- [2105.07132 - Offline Time-Independent Multi-Agent Path Planning](briefs/2105.07132.md)：用 time-independent 执行降低目标分配与路径规划耦合，可启发吞吐型任务调度。
- [2108.04629 - Roadside-assisted Cooperative Planning using Future Path Sharing for Autonomous Driving](briefs/2108.04629.md)：用未来路径共享做协同规划，可启发局部通信、预约和冲突提前规避。
- [2109.04264 - Solving Simultaneous Target Assignment and Path Planning Efficiently with Time-Independent Execution](briefs/2109.04264.md)：联合求解目标分配与路径规划，可启发 pickup/drop 选择和路径代价的耦合设计。
- [2201.09467 - CTRMs: Learning to Construct Cooperative Timed Roadmaps for Multi-agent Path Planning in Continuous Spaces](briefs/2201.09467.md)：用学习模型指导 MAPF 决策，可启发优先级、局部策略和搜索候选排序。
- [2203.00315 - Quick Multi-Robot Motion Planning by Combining Sampling and Search](briefs/2203.00315.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2211.13432 - LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding](briefs/2211.13432.md)：介绍 LaCAM 的两层搜索与 lazy constraint 生成，是本任务算法改动的核心参考。
- [2211.13908 - Fault-Tolerant Offline Multi-Agent Path Planning](briefs/2211.13908.md)：研究故障容错或实时重调度，可启发执行失败后的快速恢复和局部修补。
- [2305.03632 - Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding](briefs/2305.03632.md)：改进 LaCAM 的扩展性和近优性，可启发配置生成、剪枝和实时搜索工程化。
- [2308.04292 - Engineering LaCAM * : Towards Real-Time, Large-Scale, and Near-Optimal Multi-Agent Pathfinding](briefs/2308.04292.md)：改进 LaCAM 的扩展性和近优性，可启发配置生成、剪枝和实时搜索工程化。
- [2408.15443 - Pathfinding with Lazy Successor Generation](briefs/2408.15443.md)：研究 lazy successor generation，可启发按需生成邻居以降低搜索分支和运行时。
- [2503.12204 - D4orm: Multi-Robot Trajectories with Dynamics-aware Diffusion Denoised Deformations](briefs/2503.12204.md)：关注连续轨迹、动力学或扩散式规划，可借鉴平滑、约束和执行可行性思想。
- [2505.12623 - Lightweight and Effective Preference Construction in PIBT for Large-Scale Multi-Agent Pathfinding](briefs/2505.12623.md)：围绕 PIBT/优先级规划改进局部避让，可启发密集场景的优先级和冲突处理。
- [2507.11464 - LF: Online Multi-Robot Path Planning Meets Optimal Trajectory Control](briefs/2507.11464.md)：关注连续轨迹、动力学或扩散式规划，可借鉴平滑、约束和执行可行性思想。
- [2507.19151 - ReCoDe: Reinforcement Learning-based Dynamic Constraint Design for Multi-Agent Coordination](briefs/2507.19151.md)：用学习模型指导 MAPF 决策，可启发优先级、局部策略和搜索候选排序。
- [2508.05253 - Congestion Mitigation Path Planning for Large-Scale Multi-Agent Navigation in Dense Environments](briefs/2508.05253.md)：研究密集多智能体导航的拥堵缓解，可启发局部等待、绕行和流量调节。
- [2510.17382 - Graph Attention-Guided Search for Dense Multi-Agent Pathfinding](briefs/2510.17382.md)：用学习模型指导 MAPF 决策，可启发优先级、局部策略和搜索候选排序。
- [2510.19072 - Local Guidance for Configuration-Based Multi-Agent Pathfinding](briefs/2510.19072.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2510.19567 - Polynomial-time Configuration Generator for Connected Unlabeled Multi-Agent Pathfinding](briefs/2510.19567.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2512.06796 - db-LaCAM: Fast and Scalable Multi-Robot Kinodynamic Motion Planning with Discontinuity-Bounded Search and Lightweight MAPF](briefs/2512.06796.md)：扩展 LaCAM 到动力学约束和 discontinuity-bounded 搜索，可借鉴轻量 MAPF 分解。
- [2602.06733 - Pairwise is Not Enough: Hypergraph Neural Networks for Multi-Agent Pathfinding](briefs/2602.06733.md)：用学习模型指导 MAPF 决策，可启发优先级、局部策略和搜索候选排序。
- [2605.07744 - Alternating Target-Path Planning for Scalable Multi-Agent Coordination](briefs/2605.07744.md)：研究 MAPF/TAPF 或多机器人协同规划机制，可作为吞吐优化假设的论文来源。
- [2605.11503 - Distance-Constrained Unlabeled Multi-Agent Pathfinding](briefs/2605.11503.md)：研究带距离约束的 unlabeled MAPF，可启发任务分配中的距离限制和可达性过滤。
- [2605.13035 - Conveyor Parcel Routing with Order-Contiguous Arrivals](briefs/2605.13035.md)：研究多机器人路由与时间窗/覆盖约束，可借鉴任务排序、容量和路径分解。
- [2605.15799 - From Gridworlds to Warehouses: Adapting Lightweight One-shot Multi-Agent Pathfinding for AGVs](briefs/2605.15799.md)：面向仓储/AGV 协同场景，可启发站点分配、通道容量和吞吐评测设计。
- [2605.16855 - Lifelong LaCAM with Local Guidance for Lifelong MAPF](briefs/2605.16855.md)：研究 lifelong LaCAM 的局部引导，可启发吞吐优化中的邻域选择和引导策略。
