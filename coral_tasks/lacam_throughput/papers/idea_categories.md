# 论文 Idea 分类

实现前先用这个文件给论文支撑的吞吐优化 idea 做路由。分类描述的是论文应该怎样影响 LaCAM lifelong throughput agent。单篇论文可以出现在多个分类里，但每个候选 idea 在 notes 和 eval message 中都必须标注一个主要类别。

## 1. 地图、引导图或权重优化

当 idea 会改变 planner 对空间、通道、边方向、拥堵、区域或 assignment/map weights 的估值方式时，使用这一类论文。本任务的固定官方 map 和 task distribution 不能修改；允许的转化方式是在算法内部加入通用 guidance cost、congestion penalty、flow preference、region pressure 或 assignment weight。

代表来源：

| 论文 | 本地来源 | 在本任务中的最佳用途 |
| --- | --- | --- |
| Online Guidance Graph Optimization for Lifelong MAPF | [brief](briefs/2411.16506.md) | 边/区域方向偏好和在线拥堵反馈。 |
| Optimization of Edge Directions and Weights for Mixed Guidance Graphs in Lifelong MAPF | [brief](briefs/2602.23468.md) | 边方向、边权和 flow-bias idea。 |
| Multi-Robot Coordination and Layout Design for Automated Warehousing | [brief](briefs/2305.06436.md) | 把仓储布局/协同信号转化为通道或站点压力。 |
| QD-MAPPER: A Quality Diversity Framework to Automatically Evaluate MAPF Algorithms in Diverse Maps | [brief](briefs/2409.06888.md) | 地图特征诊断和压力场景指标，不能用于 benchmark-specific hard-coding。 |
| Traffic Flow Optimisation for Lifelong MAPF | [brief](briefs/2308.11234.md) | flow、瓶颈和拥堵缓解信号。 |
| From Gridworlds to Warehouses: Adapting Lightweight One-shot MAPF for AGVs | [brief](briefs/2605.15799.md) | 仓储站点/容量观察和 AGV-style 吞吐诊断。 |

适合产出的候选方向：

- 拥堵感知的 move 或 assignment costs。
- service stations、drop goals 或 narrow aisles 周围的 region pressure。
- 通用 edge/vertex weighting、direction bias 或 flow smoothing。
- 展示 waiting、conflict density 和 completed-task flow 的诊断地图。

避免：

- 修改官方 benchmark map、task target、seed list 或 release process。
- 硬编码只匹配 `symbotic_star.map` 的坐标。

## 2. 算法设计

当 idea 改变 solver mechanism 本身时，使用这一类论文，例如 LaCAM configuration generation、priority inheritance、target-path coupling、local repair、commitment policy、bounded windows 或 real-time search control。

代表来源：

| 论文 | 本地来源 | 在本任务中的最佳用途 |
| --- | --- | --- |
| LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding | [brief](briefs/2211.13432.md) | baseline LaCAM search structure、lazy constraints 和 configuration reasoning。 |
| Improving LaCAM for Scalable Eventually Optimal MAPF | [brief](briefs/2305.03632.md) | LaCAM pruning、expansion 和 scalability mechanisms。 |
| Engineering LaCAM*: Towards Real-Time, Large-Scale, and Near-Optimal MAPF | [brief](briefs/2308.04292.md) | large-scale LaCAM-style search 的 real-time engineering choices。 |
| Real-Time LaCAM for Real-Time MAPF | [brief](briefs/2504.06091.md) | time-budgeted search control 和 fallback behavior。 |
| Lifelong LaCAM with Local Guidance for Lifelong MAPF | [brief](briefs/2605.16855.md) | lifelong LaCAM mechanisms 和 local guidance integration。 |
| Lightweight and Effective Preference Construction in PIBT | [brief](briefs/2505.12623.md) | priority/preference construction 和 dense local avoidance。 |
| Alternating Target-Path Planning for Scalable Multi-Agent Coordination | [brief](briefs/2605.07744.md) | target assignment 与 path planning 的耦合或交替优化。 |
| Solving Multi-Agent Target Assignment and Path Finding with a Single Constraint Tree | [brief](briefs/2307.00663.md) | assignment decisions 和 path conflicts 之间的 TAPF coupling。 |
| Multi-Robot Routing with Time Windows | [brief](briefs/2103.08835.md) | task ordering、service windows、capacity 和 route decomposition。 |
| Windowed MAPF with Completeness Guarantees | [brief](briefs/2410.01798.md) | bounded-window planning 和 stability constraints。 |
| Large Neighborhood Search MAPF papers | [2405.17794](briefs/2405.17794.md), [2407.09451](briefs/2407.09451.md) | destroy-repair、local replan 和 failure reuse mechanisms。 |

适合产出的候选方向：

- 改 candidate ordering、parent selection、local repair 或 reservation logic。
- 对不同 maps 和 seeds 都通用的 target assignment policy change。
- 在不改变 task semantics 的前提下降低 churn 的 service/commitment behavior。
- 在官方 timeout 下保持 valid output 的 runtime-budget handling。

避免：

- 用 tune-only evidence 取代 hidden real evaluation。
- 对 retained chain 不跑 real eval，就把失败或成功的 tune 结果当最终结论。

## 3. 启发式函数设计

当核心改动是 scoring function、lower bound、tie-breaker、learned/local guidance proxy、conflict estimate 或 candidate ranking function，而 solver 主体基本不变时，使用这一类论文。

代表来源：

| 论文 | 本地来源 | 在本任务中的最佳用途 |
| --- | --- | --- |
| Lifelong LaCAM with Local Guidance for Lifelong MAPF | [brief](briefs/2605.16855.md) | 用 local guidance features 排序 neighbor moves 并避免 hindrance。 |
| Local Guidance for Configuration-Based MAPF | [brief](briefs/2510.19072.md) | configuration-level guidance 和 local move scoring。 |
| Improving Learnt Local MAPF Policies with Heuristic Search | [brief](briefs/2403.20300.md) | 把 local policy signals 和 explicit search heuristics 结合。 |
| Accelerating Focal Search in MAPF with Tighter Lower Bounds | [brief](briefs/2503.03779.md) | lower-bound 和 focal-ranking 灵感。 |
| Improved Heuristics for MAPF with CBS | [poster](briefs/poster-cbsh2-poster.md), [slides](briefs/slides-cbsh2.md) | conflict-aware heuristic 和 tie-breaker design。 |
| Pathfinding with Lazy Successor Generation | [brief](briefs/2408.15443.md) | on-demand successor ranking 和 pruning。 |
| Graph Attention-Guided Search for Dense MAPF | [brief](briefs/2510.17382.md) | 可以不用训练近似实现的 learned-guidance proxy features。 |
| Congestion Mitigation Path Planning for Dense Environments | [brief](briefs/2508.05253.md) | congestion、wait 和 detour scoring features。 |

适合产出的候选方向：

- 考虑 target progress、occupancy、swaps、conflicts、waiting 或 local congestion 的 move ordering functions。
- 面向 distance、service dwell、carrying state 或 goal pressure 的 assignment scoring terms。
- 在不改变 validity semantics 的前提下降低 blocking 的 tie-breakers。
- 能解释 heuristic 为什么提升或退化的 trace metrics。

避免：

- 在没有 paper-grounded mechanism 和 same-seed diagnostic evidence 时加入 opaque constants。
- 没有最终 real eval 就把 heuristic 过拟合到 public tune seeds。

## 目前已观察到的论文支撑结果

当前 real-confirmed performance lift 来自 heuristic/local guidance 路线，主要依据是 [2605.16855](briefs/2605.16855.md)。

| 步骤 | 主要类别 | 论文依据 | Real throughput | 相对上一个 real 的提升 |
| --- | --- | --- | ---: | ---: |
| Baseline | 无 | existing repository behavior | 1.18275 | - |
| Immediate occupancy/target contention ordering | Heuristic function design | [2605.16855](briefs/2605.16855.md) local guidance / hindrance avoidance | 1.27700 | +0.09425 |
| Swap-friendly occupancy exemption | Heuristic function design | [2605.16855](briefs/2605.16855.md) local guidance / local interaction handling | 1.32950 | +0.05250 |

其他探索方向目前还没有作为最终提升保留：

| 方向 | 主要类别 | 论文依据 | 当前状态 |
| --- | --- | --- | --- |
| Vacancy/displacement-regret preference | Algorithm design / heuristic function design | [2505.12623](briefs/2505.12623.md) | tune-promising variant 在 real 上退化。 |
| Loaded-distance slack and service dwell urgency | Algorithm design | [2103.08835](briefs/2103.08835.md) | tune 下降且 runtime 增加。 |
| Downstream delivery-region pressure | Map, guidance-graph, or weight optimization | [2307.00663](briefs/2307.00663.md) | tune 持平，未保留。 |
| Bounded service-target commitment | Algorithm design | [2605.07744](briefs/2605.07744.md) | screening 中更差。 |
