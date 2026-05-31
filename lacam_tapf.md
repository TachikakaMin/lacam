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
