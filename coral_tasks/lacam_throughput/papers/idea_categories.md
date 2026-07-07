# Paper Idea Categories

Use this file to route paper-grounded throughput ideas before implementation.
The categories describe how a paper should influence the LaCAM lifelong
throughput agent. A paper may appear in more than one category, but every
candidate idea should name one primary category in notes and eval messages.

## 1. Map, Guidance-Graph, Or Weight Optimization

Use these papers when the idea changes how the planner values space, corridors,
edge directions, congestion, zones, or assignment/map weights. In this task the
fixed official map and task distribution must not be modified; the allowed
translation is to add general guidance costs, congestion penalties, flow
preferences, region pressure, or assignment weights inside the algorithm.

Representative sources:

| Paper | Local source | Best use in this task |
| --- | --- | --- |
| Online Guidance Graph Optimization for Lifelong MAPF | [brief](briefs/2411.16506.md) | Directional edge/region preferences and online congestion feedback. |
| Optimization of Edge Directions and Weights for Mixed Guidance Graphs in Lifelong MAPF | [brief](briefs/2602.23468.md) | Edge direction, edge weight, and flow-bias ideas. |
| Multi-Robot Coordination and Layout Design for Automated Warehousing | [brief](briefs/2305.06436.md) | Warehouse layout/coordination signals translated into corridor or station pressure. |
| QD-MAPPER: A Quality Diversity Framework to Automatically Evaluate MAPF Algorithms in Diverse Maps | [brief](briefs/2409.06888.md) | Map-feature diagnostics and stress-case metrics, not benchmark-specific hard-coding. |
| Traffic Flow Optimisation for Lifelong MAPF | [brief](briefs/2308.11234.md) | Flow, bottleneck, and congestion-mitigation signals. |
| From Gridworlds to Warehouses: Adapting Lightweight One-shot MAPF for AGVs | [brief](briefs/2605.15799.md) | Warehouse station/capacity observations and AGV-style throughput diagnostics. |

Good candidate outputs:

- Congestion-aware move or assignment costs.
- Region pressure around service stations, drop goals, or narrow aisles.
- General edge/vertex weighting, direction bias, or flow smoothing.
- Diagnostic maps showing waiting, conflict density, and completed-task flow.

Avoid:

- Editing the official benchmark map, task target, seed list, or release process.
- Hard-coding coordinates that only match `symbotic_star.map`.

## 2. Algorithm Design

Use these papers when the idea changes the solver mechanism itself: LaCAM
configuration generation, priority inheritance, target-path coupling, local
repair, commitment policy, bounded windows, or real-time search control.

Representative sources:

| Paper | Local source | Best use in this task |
| --- | --- | --- |
| LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding | [brief](briefs/2211.13432.md) | Baseline LaCAM search structure, lazy constraints, and configuration reasoning. |
| Improving LaCAM for Scalable Eventually Optimal MAPF | [brief](briefs/2305.03632.md) | LaCAM pruning, expansion, and scalability mechanisms. |
| Engineering LaCAM*: Towards Real-Time, Large-Scale, and Near-Optimal MAPF | [brief](briefs/2308.04292.md) | Real-time engineering choices for large-scale LaCAM-style search. |
| Real-Time LaCAM for Real-Time MAPF | [brief](briefs/2504.06091.md) | Time-budgeted search control and fallback behavior. |
| Lifelong LaCAM with Local Guidance for Lifelong MAPF | [brief](briefs/2605.16855.md) | Lifelong LaCAM mechanisms and local guidance integration. |
| Lightweight and Effective Preference Construction in PIBT | [brief](briefs/2505.12623.md) | Priority/preference construction and dense local avoidance. |
| Alternating Target-Path Planning for Scalable Multi-Agent Coordination | [brief](briefs/2605.07744.md) | Coupling or alternating target assignment and path planning. |
| Solving Multi-Agent Target Assignment and Path Finding with a Single Constraint Tree | [brief](briefs/2307.00663.md) | TAPF coupling between assignment decisions and path conflicts. |
| Multi-Robot Routing with Time Windows | [brief](briefs/2103.08835.md) | Task ordering, service windows, capacity, and route decomposition. |
| Windowed MAPF with Completeness Guarantees | [brief](briefs/2410.01798.md) | Bounded-window planning and stability constraints. |
| Large Neighborhood Search MAPF papers | [2405.17794](briefs/2405.17794.md), [2407.09451](briefs/2407.09451.md) | Destroy-repair, local replan, and failure reuse mechanisms. |

Good candidate outputs:

- Changes to candidate ordering, parent selection, local repair, or reservation logic.
- Target assignment policy changes that remain general across maps and seeds.
- Service/commitment behavior that reduces churn without changing task semantics.
- Runtime-budget handling that preserves valid output under the official timeout.

Avoid:

- Replacing hidden real evaluation with tune-only evidence.
- Treating a failed tune result as final without a real eval for retained chains.

## 3. Heuristic Function Design

Use these papers when the core change is a scoring function, lower bound,
tie-breaker, learned/local guidance proxy, conflict estimate, or candidate
ranking function used by an otherwise similar solver.

Representative sources:

| Paper | Local source | Best use in this task |
| --- | --- | --- |
| Lifelong LaCAM with Local Guidance for Lifelong MAPF | [brief](briefs/2605.16855.md) | Local guidance features for ranking neighbor moves and avoiding hindrance. |
| Local Guidance for Configuration-Based MAPF | [brief](briefs/2510.19072.md) | Configuration-level guidance and local move scoring. |
| Improving Learnt Local MAPF Policies with Heuristic Search | [brief](briefs/2403.20300.md) | Combining local policy signals with explicit search heuristics. |
| Accelerating Focal Search in MAPF with Tighter Lower Bounds | [brief](briefs/2503.03779.md) | Lower-bound and focal-ranking inspiration. |
| Improved Heuristics for MAPF with CBS | [poster](briefs/poster-cbsh2-poster.md), [slides](briefs/slides-cbsh2.md) | Conflict-aware heuristic and tie-breaker design. |
| Pathfinding with Lazy Successor Generation | [brief](briefs/2408.15443.md) | On-demand successor ranking and pruning. |
| Graph Attention-Guided Search for Dense MAPF | [brief](briefs/2510.17382.md) | Learned-guidance proxy features that can be approximated without training. |
| Congestion Mitigation Path Planning for Dense Environments | [brief](briefs/2508.05253.md) | Congestion, wait, and detour scoring features. |

Good candidate outputs:

- Move ordering functions that account for target progress, occupancy, swaps,
  conflicts, waiting, or local congestion.
- Assignment scoring terms for distance, service dwell, carrying state, or goal
  pressure.
- Tie-breakers that reduce blocking without changing validity semantics.
- Trace metrics that explain why a heuristic improved or regressed.

Avoid:

- Adding opaque constants without a paper-grounded mechanism and same-seed
  diagnostic evidence.
- Overfitting a heuristic to public tune seeds without a final real eval.

## Observed Paper-Grounded Results So Far

The current real-confirmed performance lift came from the heuristic/local
guidance path, primarily grounded in [2605.16855](briefs/2605.16855.md).

| Step | Primary category | Paper grounding | Real throughput | Lift vs previous real |
| --- | --- | --- | ---: | ---: |
| Baseline | none | existing repository behavior | 1.18275 | - |
| Immediate occupancy/target contention ordering | Heuristic function design | [2605.16855](briefs/2605.16855.md) local guidance / hindrance avoidance | 1.27700 | +0.09425 |
| Swap-friendly occupancy exemption | Heuristic function design | [2605.16855](briefs/2605.16855.md) local guidance / local interaction handling | 1.32950 | +0.05250 |

Other explored directions were not retained as final improvements yet:

| Direction | Primary category | Paper grounding | Current status |
| --- | --- | --- | --- |
| Vacancy/displacement-regret preference | Algorithm design / heuristic function design | [2505.12623](briefs/2505.12623.md) | Tune-promising variant regressed on real. |
| Loaded-distance slack and service dwell urgency | Algorithm design | [2103.08835](briefs/2103.08835.md) | Tune down and runtime increased. |
| Downstream delivery-region pressure | Map, guidance-graph, or weight optimization | [2307.00663](briefs/2307.00663.md) | Tune flat, not retained. |
| Bounded service-target commitment | Algorithm design | [2605.07744](briefs/2605.07744.md) | Worse in screening. |
