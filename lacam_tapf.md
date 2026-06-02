# LaCAM-TAPF 方法说明

本文档记录当前仓库里实现的 LaCAM-style TAPF 求解器，以及为分析
LaCAM / FOCAL-LaCAM / IR-TAPF 对比所加入的实验工具。

## 问题形式

输入是一个 grid graph、`N` 个 agent 的起点，以及一组 TAPF task/goal。
每个 agent 有自己的 `potentialGoals` 可达集合。求解器需要同时满足：

- 每个 agent 最终停在一个允许的 goal 上；
- 每个 goal 最多被一个 agent 使用；
- 每一步没有 vertex conflict，也没有 edge-swap conflict；
- 输出 joint path，并报告 SOC、makespan、sum-of-loss 等指标。

和固定 MAPF 不同，TAPF 的 assignment 不是输入固定值。当前实现会在搜索过程中
动态维护一个 agent-to-task matching，但这个 matching 只是低层规划和 heuristic 的
guidance，不进入 high-level state 的判重 key。

## High-Level State

TAPF high-level node `S` 包含：

- `C`: 当前 joint configuration，即每个 agent 所在 vertex；
- `parent`: 最优已知父节点，用于回溯最终 solution；
- `search_tree`: LaCAM lazy constraints 队列；
- `assignment`: 当前 configuration 下的 agent-to-task matching；
- `assignment_state`: 增量 assignment 的缓存状态；
- `g`: 从 root 到当前 node 的累计 SOC-style cost；
- `h`: 当前 assignment 的剩余距离和；
- `f = g + h`;
- `order`: PIBT 处理 agent 的动态优先级顺序；
- search metrics: `non_goal_waits`、`reversals`、`distance_increases`、
  `settled_pushes`，用于 FOCAL tie-break。

`CLOSED` 只用 `Config C` 做 key。也就是说，同一个 physical configuration 不会因为
assignment 不同被当成不同 high-level state。这一点保留了 LaCAM 的 configuration
search 风格，避免 assignment 维度把状态空间放大。

## Dynamic Assignment

root node 对所有 agent 做一次 Hungarian assignment。之后每次生成新 configuration
`C_new` 时，只对移动过的 agent 触发增量更新：

```text
changed_agents = { i | C_new[i] != C_parent[i] }
assignment = assign_tapf_tasks_dynamic(C_new, changed_agents)
```

默认模式下 assignment 是动态 guidance。也可以用 `force_full_assignment` 强制每一步
重算完整 matching，主要用于实验对照。

assignment 的作用：

- 给 PIBT 提供每个 agent 当前追踪的 task；
- 给 `h` 提供 admissible-ish 的剩余距离估计；
- 判断当前 configuration 是否已经是 TAPF goal configuration；
- 计算每条 high-level edge 的 SOC-style cost。

## Cost 定义

当前 high-level edge cost 在 `get_edge_cost(from, to)` 中计算：

```text
edge_cost = count_i not (from.C[i] == assigned_goal_i and to.C[i] == assigned_goal_i)
```

也就是：如果 agent 已经在当前 assigned goal 上，并且下一步仍留在这个 goal 上，则这
一步不继续累加 cost；否则计 1。最终 `g` 是沿父链累加的 SOC-style solution cost。

注意这里的 `assigned_goal_i` 来自 `to` node 的 assignment。由于 TAPF assignment
可以在搜索过程中变化，`g` 不是简单的固定-goal MAPF SOC，而是“以最终父链上各 node
assignment 为准”的 TAPF cost。`tapf_benchmark` 同时输出 `sum_of_loss`，用于和
路径级别的 loss 指标交叉检查。

## Successor 生成

每个 node 有一棵 lazy local-constraint tree。一次 high-level expansion 做：

```text
S = SelectOpenNode()
M = S.search_tree.pop()

if depth(M) < N:
    i = S.order[depth(M)]
    for u in Neighbors(S.C[i]) union {S.C[i]}:
        S.search_tree.push(M + positive constraint i -> u)

C_new = GetNewConfig(S.C, M, S.assignment)
```

`GetNewConfig` 先应用 constraint path 里已经指定的 agent move，再按 `S.order` 调用
PIBT 补全其他 agent 的下一步。成功后得到一个完整的 one-step successor
configuration。

## PIBT-TAPF

PIBT 里每个 agent 的候选下一格是：

```text
Neighbors(v_now) union {v_now}
```

候选排序使用当前 assignment 的 task distance：

```text
primary:   D[assigned_task_i][candidate] 越小越好
secondary: hindrance(candidate) 越小越好
tertiary:  random tie-breaker
```

`hindrance(candidate)` 是这次加入的局部避让项。它只在“当前 agent 到自己 assigned
task 的距离相同”时生效：

```text
hindrance(u) =
    count neighbor agent j such that
        j is not currently on u, and
        D[assigned_task_j][u] < D[assigned_task_j][current_position_j]
```

直观含义：如果某个候选格子也是相邻 agent 变得更接近其目标所需要的格子，那么当前
agent 在同距离选择里尽量不抢这个格子。这个规则对门口、窄通道和相邻让路有帮助，但
不会压过 primary distance；如果走向某格对当前 agent 严格更接近自己的 task，
hindrance 不会阻止它。

## LaCAM2 Swap

当前 TAPF PIBT 包含 LaCAM2 风格的 swap operation，并把固定 MAPF goal 替换为当前
assignment 下的 task。

当 agent `i` 的最佳候选格被 agent `j` 占据，且局部模拟判断：

- `i` 推动 `j` 是 required；
- 周围结构允许完成 swap；

则反转候选顺序，并在可行时把被 swap 的 agent 拉回 `i` 的原位置。这个机制主要修
PIBT 在一类 narrow-passage / mutual blocking 场景里的 livelock。它不是 SOC 优化
器，也不会单独解决所有长时间绕行或 late-departure 行为。

## Search Mode

`tapf_benchmark` 支持两种 high-level OPEN 选择：

```text
dfs
focal
```

### DFS

默认模式保持 LaCAM 风格的 LIFO stack 行为：

```text
SelectOpenNode() = OPEN.back()
```

这通常能很快找到 first solution，但 first solution 质量可能受 PIBT 局部选择和
assignment guidance 影响。

### FOCAL

FOCAL 模式不是从一开始就 best-first。当前实现是：

- 在还没有 incumbent solution 前，仍使用 DFS，以保留 LaCAM 快速找到首解的行为；
- 找到首解后，OPEN 中只考虑 `f < incumbent_cost` 且 `search_tree` 非空的 viable
  node；
- 计算 `f_min`，再在 `f <= focal_weight * f_min` 的 focal set 里选 tie-break 最好
  的 node。

默认 `focal_weight = 1.5`。tie-break 首先比较 `h`，再根据配置比较 search metrics：

```text
h:
    use h only

anti_wait:
    8*non_goal_waits + 4*reversals + 2*distance_increases + settled_pushes

anti_zigzag:
    8*reversals + 4*distance_increases + 2*settled_pushes + non_goal_waits

anti_push:
    8*settled_pushes + 4*reversals + 2*distance_increases + non_goal_waits

anti_all:
    10*settled_pushes + 6*reversals + 3*non_goal_waits + 2*distance_increases
```

这些 metrics 是沿 high-level parent chain 累积的：

- `non_goal_waits`: agent 不在 assigned goal 上却原地等待；
- `reversals`: `A -> B -> A` 形式的来回震荡；
- `distance_increases`: 下一步离 assigned task 更远；
- `settled_pushes`: 一个移动 agent 进入了其他 agent 刚从其 assigned goal 离开的格子。

### 设计取舍

这套实现保留了 LaCAM 的核心特点：high-level state 是 joint configuration，
successor 由 PIBT lazy completion 生成，OPEN 默认用 DFS 快速找 first solution。
这样做的优点是首解速度很快，缺点是首解质量会强烈依赖局部 PIBT 决策；如果局部让路
被 tie-break 选坏，DFS 可能沿着一条可行但很长的分支继续向前走，表现为长时间
zigzag、远距离游走，或者 agent 离开 goal 之后很久才回来。

IR-TAPF 的主流程和这里不同。IR 先固定或迭代改进 target assignment，再调用它的
底层 path solver 评估 assignment，所以它的搜索压力更多落在 reassignment 上。
当前 LaCAM-TAPF 则把 assignment 当作每个 configuration 的动态 guidance：
同一个 physical configuration 不因为 assignment 不同被复制。因此它更像
configuration search + online assignment，而不是 assignment search + MAPF repair。

FOCAL 的目标不是替代 LaCAM 的首解策略，而是在 first solution 之后更稳定地消费
剩余时间。首解前仍用 DFS，是为了避免 best-first 一开始倾向展开大量低 `f` 的等待
状态，导致很久没有可行解。首解后 incumbent 给出了 cost 上界，此时用
`f = g + h` 过滤 OPEN，并用 tie-break 避开明显病态的局部行为，才比较安全。

`hindrance` 和 LaCAM2 swap 只处理低层 PIBT 的局部冲突：

- `hindrance` 只在当前 agent 对自己的 task 距离不变时生效，避免抢邻居通往目标的
  同距离格子；
- swap 处理两 agent 互相挡路的一类局部 livelock；
- 它们都不会把 high-level search 变成全局最短路搜索，也不会保证 SOC 最优。

所以当前方法的定位是：用 LaCAM 的快速可行性搜索处理大规模 TAPF，用动态 assignment
给 PIBT 和 heuristic 提供 TAPF 目标，用 FOCAL/metrics 在 anytime 阶段压低明显的
高 cost 分支。它不是 LaCAM* 的完整最优实现；`f` 目前主要用于剪枝和 FOCAL 选择，
而不是从 root 开始严格 best-first 展开。

## Anytime 行为

`anytime=1` 时，找到 first solution 后继续搜索更低 cost 的 incumbent。搜索会：

- 剪掉 `f >= incumbent_cost` 的 node；
- 记录 `first_solution_cost` 和 `first_solution_time_ms`；
- 记录 `incumbent_updates`；
- 保留一小段 cleanup 时间，避免 deadline 到达时析构大量 search tree 节点导致外部
  timeout。

`anytime=0` 时，找到第一个 goal configuration 就返回。

## 命令行

`tapf_benchmark` 的 TAPF 参数：

```sh
build/tapf_benchmark YAML MAP_DIR TIME_LIMIT_SEC SCHEDULE_YAML \
  ANYTIME FULL_TA SEED SEARCH_MODE FOCAL_WEIGHT FOCAL_TIE_BREAK
```

例子：

```sh
build/tapf_benchmark case.yaml "" 10 out.yaml 1 0 -1 dfs 1.5 h
build/tapf_benchmark case.yaml "" 10 out.yaml 1 0 -1 focal 1.5 h
build/tapf_benchmark case.yaml "" 10 out.yaml 1 0 -1 focal 1.5 anti_zigzag
```

输出增加了这些字段：

- `search_mode`
- `focal_weight`
- `focal_tie_break`
- `solution_cost`
- `first_solution_cost`
- `first_solution_time_ms`
- `incumbent_updates`
- `swap_checks`
- `swap_applied`

## 实验工具

### 全量三方法实验

`tools/run_full_three_method_experiment.py` 同时跑：

- `lacam_dfs`
- `lacam_focal_h`
- `ir` (`dbs_hungarian`)

覆盖：

- ITA-CBS exp1 fixtures；
- ITA-CBS exp2 fixtures；
- IR repo 的所有 `.matrix` testcase。

命令：

```sh
python3 -u tools/run_full_three_method_experiment.py \
  --all-itacbs-data \
  --ir-matrix-root /home/yimin/research/ir-tapf/matrix \
  --time-limit 10 \
  --timeout 30 \
  --jobs 32 \
  --resume \
  --out-dir build/results/full_three_method_hindrance_exp1_exp2_ir
```

输出：

- `rows.jsonl`: 增量写入，适合中断后 `--resume`；
- `rows.csv`: 完整明细；
- `summary.csv`: 按 `(suite, method)` 聚合。

### FOCAL Tie-Break 实验

`tools/run_lacam_focal_tiebreak_experiment.py` 用已有 case set 比较：

- `dfs`
- `focal:h`
- `focal:anti_wait`
- `focal:anti_zigzag`
- `focal:anti_push`
- `focal:anti_all`

它会输出每个 variant 相对 IR 的 SOC 差、solve rate、wall time，并生成图。

### Repair 实验

`tools/run_tapf_repair_experiment.py` 做两阶段修复：

1. 先用 no-anytime 求首解；
2. 从首解里选 per-agent cost 最高的前 `repair_fraction` agent 解除绑定；
3. 其他 agent 固定到首解最终 target；
4. 再跑一次 LaCAM-TAPF。

这个实验用于判断高 cost 是否集中在少数 agent，以及固定其他 assignment 后局部重搜
是否能改善 SOC / sum-of-loss。

### arXiv 2605.07744 复现实验

`tools/run_paper_2605_07744_experiments.py` 用来复现 arXiv 2605.07744 里的主要
TAPF 实验，并把同一批 generated matrix 同时交给 IR-TAPF 和 LaCAM-TAPF：

1. 调用 `/home/yimin/research/ir-tapf/target/release/ir_tapf setup` 生成 paper
   matrix；
2. 对 paper baseline，调用 `ir_tapf solve --matrix ... --solver METHOD`；
3. 对 LaCAM baseline，把同一个 matrix 用 `matrix_to_yaml` 转成 TAPF YAML；
4. 调用 `build/tapf_benchmark` 跑 `lacam_dfs` 和 `lacam_focal_h`；
5. 所有结果增量写入 `rows.jsonl`，结束后写 `rows.csv` 和 `summary.csv`。

这样可以保证 IR 和 LaCAM 用的是同一张地图、同一组 start、同一组 per-agent
potential targets，而不是各自重新采样实例。matrix 生成加了 `.lock` 文件，支持
多进程并发时避免读到半写入文件。runner 在复用或新生成 matrix 前会检查两个
TAPF 必要条件：每个 agent 的 target row 非空，并且 agent-target 二分图存在完整
matching；不满足条件的缓存 matrix 会被删除并重新生成，避免把不可解实例写进
paper result rows。

支持的 suite：

- `smoke`: 小规模 sanity check，1 张图、1 个 seed、少量 solver；
- `fig3`: component comparison，覆盖 6 张 paper map、RANDOM/HOTSPOT、200/400/600/800
  agents、30 seeds，baseline 为 DBS/SBS/Random x Hungarian/PIBT，并加入
  `lacam_dfs`、`lacam_focal_h`；
- `table1`: time-limited 和 fixed-iteration 对照；fixed-iteration 行只跑 IR，因为
  LaCAM-TAPF 没有 paper 中的 iteration-mode 接口；
- `table3`: random initial assignment variants；
- `fig4`: final path optimization variants，`lak303d`、200 agents、HOTSPOT；
- `fig5`: scalability，`warehouse-20-40-10-2-2` 上 1000/2000/5000/10000 agents；
- `fig6`: DBS-Hungarian 和 DBS-PIBT 的 multi-bottleneck `k ∈ {1,3,10}` sweep，
  需要配套使用已暴露 `--num-pickup-agents` 的 `ir-tapf` binary；
- `table4`: 使用 `tools/run_paper_2605_07744_table4.py` 跑 ITA-CBS2 的
  `map_file_ecbs` YAML fixtures，对比 ITA-ECBS、IR `dbs_hungarian` 和
  `lacam_focal_h`。

常用命令：

```sh
python3 -u tools/run_paper_2605_07744_experiments.py \
  --paper-suite fig3 \
  --jobs 32 \
  --timeout 45 \
  --resume \
  --out-dir build/results/paper_2605_07744_fig3
```

生成图和派生指标：

```sh
python3 tools/plot_paper_2605_07744_results.py \
  --rows build/results/paper_2605_07744_fig3/rows.csv \
  --out-dir build/results/paper_2605_07744_fig3/plots
```

`tools/plot_paper_2605_07744_results.py` 会写：

- `derived_metrics.csv`: 每条 row 的 normalized cost 和 improvement；
- `fig3_components/*.png`: normalized flowtime 和 improvement；
- `fig5_scalability/*.png`: scalability runtime / improvement；
- `fig6_k_sweep/*.png`: `k ∈ {1,3,10}` 下的 improvement 和 iteration count；
- `fig7_profiling/*.png`: IR pathfinding 与 reassignment profiling；
- `table4_ita_ecbs/*.png`: ITA-ECBS / DBS-Hungarian / LaCAM-TAPF success rate 和 cost。

Table 4 单独 runner：

```sh
python3 -u tools/run_paper_2605_07744_table4.py \
  --jobs 16 \
  --resume \
  --out-dir build/results/paper_2605_07744_table4
```

指标解释：

- `soc`: solver 输出的 solution cost；
- `sum_shortest_distances`: 对每个 agent 取其 start 到任一 reachable potential target
  的最短距离，再求和，用作 normalized flowtime 的下界；
- `normalized_cost = soc / sum_shortest_distances`；
- `initial_solution_cost`: IR 或 LaCAM 首解 cost；
- `improvement_pct = (initial_solution_cost - soc) / initial_solution_cost * 100`；
- `external_timed_out`: runner 的 subprocess timeout，不等同于 solver 自己的
  `timed_out`。

当前限制：

- Table 4 runner 默认使用 IR 的 `dbs_hungarian`。论文 Table 4 描述了 20 s target
  refinement + 10 s LaCAM3 final path optimization；但当前 `ir-tapf`
  `opt_dbs_hungarian` 在 ITA-CBS2 YAML 转出的单目标 fixtures 上会触发 LaCAM3 FFI
  double-free/corruption，因此暂不作为默认 Table 4 baseline；
- LaCAM-TAPF 的 `focal` 配置目前固定使用 tie-break `h`，如果要复用
  `anti_zigzag` 等 tie-break，需要在 runner 里扩展 method matrix；
- plotter 目前覆盖 Fig.3/Fig.5/Fig.6/Fig.7/Table4 风格图，Table 1/3 和 Fig.4 的表格化输出需要
  从 `summary.csv` 或 `derived_metrics.csv` 继续整理。

## 当前全量结果

一次 10 秒 time limit、30 秒 external timeout、32 jobs 的全量结果：

```text
suite | method        | cases | solved | solve_rate | mean_soc | mean_sum_of_loss | mean_wall_s | ext_to
exp1  | ir            | 3200  | 3025   | 0.9453     | 15452.9  | nan              | 10.928      | 0
exp1  | lacam_dfs     | 3200  | 3200   | 1.0000     | 14721.0  | 13201.4          | 8.747       | 0
exp1  | lacam_focal_h | 3200  | 3200   | 1.0000     | 14715.9  | 13201.4          | 8.746       | 0
exp2  | ir            | 6160  | 5973   | 0.9696     | 2464.8   | nan              | 10.360      | 0
exp2  | lacam_dfs     | 6160  | 6135   | 0.9959     | 2019.8   | 1951.2           | 7.380       | 0
exp2  | lacam_focal_h | 6160  | 6127   | 0.9946     | 2011.8   | 1944.2           | 7.382       | 0
ir    | ir            | 100   | 100    | 1.0000     | 92.5     | nan              | 10.036      | 0
ir    | lacam_dfs     | 100   | 100    | 1.0000     | 91.5     | 91.5             | 2.868       | 0
ir    | lacam_focal_h | 100   | 100    | 1.0000     | 91.5     | 91.5             | 2.868       | 0
```

验证项：

- `rows.jsonl`: `28380` rows；
- unique `(case_id, method)`: `28380`；
- duplicates: `0`；
- external timeouts: `0`；
- `summary.csv`: 9 groups。

其中 exp2 有 58 个 LaCAM 记录是 10 秒求解时限内未解：

```text
exp2/lacam_dfs: 25
exp2/lacam_focal_h: 33
```

这些是 solver 自身的 `timed_out=1 solved=0`，不是 runner 的 external timeout。
