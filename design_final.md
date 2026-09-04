# Carrier-LaCAM 最终设计：基于 Task-BR-PIBT 的双层动态分配

状态：**已实现、完成 release 验证并通过最终独立复核**，
2026-09-03（此前独立 xhigh 审核，本修订周期
三轮：R1 NOT_APPROVED 4 项阻塞 → 修复；R2 确认 3 项、custody 职责
残余 2 处矛盾 → 修复；R3 全项 PASS、APPROVE，唯一编辑性 nit
“两类→三类”已随后修正）。前六轮审查通过的 2026-09-03 04:07 版本为
本周期基础。本文以
`new.md` 为语义来源，取代旧
`Objective-PIBT` 设计。当前 production code 已迁移到本文的
Task-BR-PIBT 路径；`results_v4_1_final7` 和 Objective-PIBT tests 仅作为
迁移前基线与受治理的测试迁移证据。最终实现、预算细化与配对验证结果见
§20。

**2026-09-04 storage-map 修订状态：设计、实现、新回归、同机配对验证与
最终独立审查均已完成并通过。** 本轮严格按
`test -> RED -> implementation -> GREEN -> benchmark -> regression ->
debug` 执行。该修订不推翻相邻 `ShelfTask` 边界，而是补上
storage-to-storage transfer commitment，修复 blocker 被推进 transit
走廊后失去 custody、无法 Drop、再返回原位的往复问题。§21 是本轮修订的
权威增量；§22 记录实际验证证据。与前文冲突时，以 `new.md` 新增的
storage 语义和 §21 为准。

本文保留现有 LaCAM-TAPF 的 physical-state search、operator constraint
tree、`TAPFPlanner::solve()`、`apply_ops()`、两遍求解、输出修补、严格
返回截止和独立重放，只替换 carrier guidance。最终 execution path 必须仍是：

```text
TAPFPlanner::solve()
  -> attach_carrier_guidance()
  -> Carrier-PIBT / funcPIBT()
  -> apply_ops()
  -> physical successor
```

不得新增平行 planner、第二套 search loop、运行时 legacy fallback，或在
Carrier-LaCAM 节点内嵌完整 BR-LaCAM。

---

## 0. 最终决策

Carrier-LaCAM 只搜索真实 robot-shelf physical configurations。每个节点的
guidance 分为两个严格隔离的 assignment 层：

```text
physical state X
    ↓
UpperProjection U(X)
    ↓
对每个 eligible (target shelf b, goal g)
运行 single-root Task-BR-PIBT bounded rollout
    ↓
shelf-side PairCost C_B[b,g]
    ↓
injective Hungarian
    ↓
tau_guide : target shelf -> temporary terminal goal
    ↓
固定 tau_guide，联合编译 Task-BR-PIBT dependency graph D
    ↓
ReadyTasks(D, X)
    ↓
carried continuation 直接绑定；
grounded ready tasks 与 free robots 做 rho matching
    ↓
Carrier-PIBT 生成 preferred primitive joint action
    ↓
apply_ops() 独立裁决
    ↓
physical successor X'
```

必须保持以下边界：

1. `PairCost` 和 `tau_guide` 只读取 upper shelf layout，不读取 robots；
2. `tau_LB` 是独立的 admissible matching，只用于 `h`；
3. Task-BR-PIBT 生成 shelf-side 因果依赖，不负责 lower-deck robot 路径；
4. `rho` 只看当前 ready tasks、free robots 和 task-switch 粘滞；
5. Carrier-PIBT 只处理当前 timestep 的 robot conflicts；
6. guidance 失败、预算耗尽或判断错误都不得删除 physical successors；
7. terminal condition 只读物理状态和 eligible goal sets，不读取 `tau`。

---

## 1. 当前代码审计

### 1.1 可以保留的部分

| 当前机制 | 结论 | 后续用途 |
|---|---|---|
| `SearchKey{Config, ShelfState}` | 合理 | 继续只存物理状态；任何 assignment、task、priority、cache 都不进 key |
| `is_goal_config()` / `is_dd_goal()` | 合理 | 继续要求每个 target grounded 且位于自己的 eligible set |
| `apply_ops()` | 合理且必须保留 | 继续作为所有 fully constrained joint actions 的唯一物理裁判 |
| operator constraint tree 的穷举保底 | 合理且是完备性核心 | guidance 只改首选顺序，树仍最终枚举全部 primitive combinations |
| exact Hungarian 基础设施 | 合理 | 分别复用于 `tau_guide`、`tau_LB` 和 `rho` |
| `DDDistCache`、wall distance、lower-deck distance | 合理 | wall distance 用于候选排序、residual 和 `tau_LB`；lower distance 只进入 `rho` |
| custody continuity 的基本思想 | 合理 | 改为“一步 ShelfTask”的 preferred continuity：从 Lift 持续到相邻 loaded Move 完成，但不成为物理硬约束 |
| Carrier-PIBT 的 lower-deck priority inheritance | 合理 | 接收 task priority 后处理 robot blockers |
| 两遍求解、修补、严格 deadline、C++/Python 重放 | 与 guidance 正交且合理 | 原样保留并继续作为 release gate |

### 1.2 与 `new.md` 不一致、必须替换的部分

| 当前实现 | 问题 | 必须修改 |
|---|---|---|
| `solve_tau()` 用 wall-distance/Lift-Drop LB 直接产生 guidance `tau` | 没有评估 blocker displacement 和 shelf-side task difficulty | 将该逻辑拆为 `tau_LB`；新 `tau_guide` 使用 `PairCost(U,b,g)` |
| `compute_execution_prices()` 用 robot 距离翻转 `tau` | 违反“robot-only movement 不改变 terminal goal assignment” | 从 production guidance 删除；robot 距离只进入 `rho` |
| `preserve_tau` 在 loaded Move 后仍沿用 parent `tau` | upper layout 已改变却不重评；与 upper-epoch 语义相反 | loaded Move 后必须重取 PairCost 并重解 `tau_guide` |
| robot-only 节点仍可能因 execution price、aging 或 reguide 改 `tau`/task priority | 同一 upper layout 的 shelf guidance 不稳定 | robot-only、Lift、Drop 均复用同一 upper-epoch `C_B/tau/D/priority` |
| `ObjectiveOption + claims + Phase T/R + yield` | 协商的是路线套餐和格子认领，不是真正递归解决 blocker shelf | 删除，改为 Task-BR-PIBT recursive displacement 与 candidate backtracking |
| `n_vacancies == 1` 专门做 vacancy BFS | one-empty 被特殊化；无法统一解释 multi-empty/zero-empty | 删除特判；同一递归自然生成 vacancy chain |
| serve task 可表示 `from -> terminal goal`，clear task 可 `to=-1` | 不是相邻原子 shelf shift，也不能形成精确依赖 | 所有 `ShelfTask` 必须满足 `from` 与 `to` 相邻，且 `to` 永不缺失 |
| TaskId 只含 `(shelf, from)` | 同一 shelf 从同一位置去不同 `to` 会被错误合并 | TaskId/EffectKey 改为 `(shelf selector, from, to)` |
| 任务池包含 non-ready clear/serve tasks，rho 再按 priority/depth 截取 | robot 会被派往尚不能执行的内部依赖节点 | 只有 `ReadyTasks(D,X)` 能进入 `rho` |
| `ACTIVE_TARGET_CAP` 只编译一部分 roots | 不符合“固定 tau 后所有 unfinished targets 联合编译” | joint compiler 必须接收全部 unfinished roots；预算不足时显式标记 paused，不得静默截断 |
| loaded target 在 `funcPIBT()` 中直接持续向 `tau` 移动 | 绕过一步 task 完成、upper epoch 重编译和 continuation 判定 | loaded carrier 首选只执行当前相邻 task；每次 loaded Move 后重新 attach guidance |
| `target_park` / `parking_cell` 根据目标路线把 shelf 或 carrier 送去历史相关停车位 | 在 `D -> ready -> rho` 之外又建立一套 shelf 决策，并可受 path-cache history 影响 | 删除 target park/parking pipeline；没有 continuation 的 loaded carrier 只首选原地 Drop，idle free robot 只按当前 `D/ready` footprint 避让 |
| `least_blocking_path(..., prev_path)` 用 parent path inertia 打破平局 | 相同 `U` 可因 ancestry 不同产生不同 PairCost/task graph | shelf-side path/candidate tie 必须只由 `U` 和稳定 cell id 决定；删除 production `prev_path` bias |
| revisit/no-progress 触发 wait-for graph、rho taboo 和 livelock `reguide` | 用搜索历史在正常 `U -> D -> ready -> rho` 链外改写 rho，且现有路径还会进一步 taboo tau | 删除这条重指导通道；rho 只允许当前 ready set、robot 距离和上一拍 TaskId switch penalty |
| 全局 `futile_lift` 计数与 cooldown | 同一物理状态的 ready Lift 会因先前 episode 被降级，且该记忆不在 state/upper projection 中 | 删除 cooldown；是否可 Lift 只由当前 ready task、物理前置条件和 `apply_ops()` 决定 |
| Objective-PIBT tests 把 robot-flips-tau、one-empty 特判、旧 TaskId 等语义保护为 GREEN | 测试证明的是旧算法，不是本文 | 迁移时按 `rules.md` 先经独立 reviewer 批准，再替换冲突的 protected tests |

### 1.3 算法判断

旧实现中“全局 injective matching、任务携带 root provenance、custody、
Hungarian rho、lower-deck PIBT、完备性保底”都是合理基础；问题不在这些
组件本身，而在三层边界被混合：

* robot execution difficulty 被写回 terminal goal assignment；
* shelf blocker causality 被近似成路径扫描与 claims；
* 内部 dependency nodes 被提前交给 robot。

本文的修改不是另起炉灶，而是在现有 guidance 入口内重新划清这三层。

---

## 2. 物理状态、终点与 upper projection

### 2.1 Physical state

```text
X = (Q_robot, Q_target, Q_anon, kappa)
```

* `Q_robot`：labeled robot positions；
* `Q_target`：labeled target-shelf positions；
* `Q_anon`：canonicalized grounded anonymous-shelf positions；
* `kappa[r]`：robot `r` 为 free、携带 target `b`，或携带 anonymous shelf。

每个 robot 每拍选择：

```text
Wait | Move(neighbor) | Lift | Drop
```

搜索 key 继续只包含上述物理信息。`tau_guide`、`tau_LB`、PairPlan、
task graph、priority、rho、custody metadata 都不进入 key。

### 2.2 Goal

```text
is_goal(X) iff
  every target b is grounded
  and position(b) ∈ G_b
```

`is_goal()` 不读取当前 `tau_guide`。已经位于 eligible goal 的 shelf 仍是
可逆物理 blocker；只要全局终点尚未成立，搜索仍可把它搬开。

### 2.3 UpperProjection

定义：

```text
U(X) = (
  labeled target positions,
  sorted positions of every anonymous shelf
)
```

anonymous positions 包括：

* `Q_anon` 中 grounded anonymous shelves；
* 每个 `kappa[r] == ANON` 的 robot 当前格子。

`U` 不包含：

* free robot positions；
* 哪个 robot 正携带某个 shelf；
* shelf 当前 grounded 还是 carried；
* rho、custody、priority 或 parent history。

因此：

* free robot `Move/Wait`：`U` 不变；
* `Lift`：shelf 坐标不变，`U` 不变；
* `Drop`：shelf 坐标不变，`U` 不变；
* loaded `Move`：shelf 坐标改变，`U` 改变。

若实例有显式 `storage_map`，`U` 可以暂时包含位于 non-storage transit
cell 的 carried shelf，但 grounded shelf 永远只能位于 storage cell。
所有密度/空位判断使用：

```text
upper_vacancy_count =
    number_of_storage_cells - number_of_shelves
```

不能使用 traversable cells，因为 transit aisle 不是合法 shelf vacancy。

### 2.4 Upper signature 与 upper epoch

```cpp
struct UpperSignature {
  std::vector<int> target_pos;  // labeled
  std::vector<int> anon_pos;    // sorted, grounded + carried
};
```

root node 或 `UpperSignature(parent) != UpperSignature(child)` 时开启新的
upper epoch。一个 upper epoch 内必须复用：

```text
PairCost table
tau_guide
target priorities
joint task graph D
```

每个节点仍重新计算：

```text
custody status
ready tasks
rho
Carrier-PIBT preferred action
```

PairCost cache 可跨不同 robot configurations 复用，只要 `UpperSignature`
完全相同。

---

## 3. Guidance 数据结构

### 3.1 Shelf selector

```cpp
struct ShelfSelector {
  enum class Kind { TARGET, ANON_AT_EPOCH_CELL };
  Kind kind;
  int value;  // target index, or anonymous shelf's cell in this upper epoch
};
```

target shelf 用 label 标识。anonymous shelves 仍保持对称性；在一个 upper
epoch 内，用其当前 cell 标识。任何 shelf movement 都会开启新 epoch，
因此 anonymous selector 不需要跨 movement 保持永久身份。

PairCost 的抽象 rollout 是唯一需要在多个模拟 movement 之间追踪“同一匿名
shelf”的地方。它必须在 rollout 内创建局部 `AbstractShelfToken`：初始
anonymous cells 排序后确定性编号，执行 `u -> v` 时 token 随 shelf 移动。
该 token 只存在于一次 PairCost 调用中，不进入真实 `UpperSignature`、
TaskId、search key 或跨节点 cache。这样既保持匿名对称性，也能正确识别
同一 anonymous shelf 的连续 Lift/Drop episode。

### 3.2 一步 ShelfTask

```cpp
struct ShelfTask {
  ShelfSelector shelf;
  int from;
  int to;
  std::vector<RootDemand> roots;
  int priority;
  StorageTransfer transfer;
};

struct RootDemand {
  int target;
  int goal;
};
```

强制不变量：

```text
from 与 to 是相邻 traversable cells
from != to
to 永不为 -1
TaskId = exact effect (shelf, from, to)
```

任务表示一次相邻 upper-deck shelf shift，而不是单个 robot primitive。
真实执行通常是：

```text
approach -> Lift -> one loaded Move
```

任务在 loaded Move 后完成；不要求立即 Drop。

显式 `storage_map` 下新增：

```cpp
struct StorageTransfer {
  int endpoint;           // can_store_shelf(endpoint) == true
  std::vector<int> route; // route[0]=from, route[1]=to,
                          // adjacent throughout, route.back()=endpoint
};
```

每个 `ShelfTask` 仍只执行 route 的第一格，因此 `TaskId` 和 physical
successor 语义不变。`transfer` 是跨 upper epoch 的安全承诺：一旦第一格
进入 transit cell，当前 carrier 必须沿同一 route 继续到合法 endpoint。
legacy/no-storage-map 实例中 route 恒为 `[from,to]`，自然退化为原语义。

### 3.3 Task graph

```cpp
struct ShelfTaskGraph {
  std::vector<ShelfTask> tasks;
  std::vector<std::vector<int>> predecessors;
  std::vector<std::vector<int>> successors;
  std::vector<int> paused_roots;
  std::vector<RotationCandidate> rotations;
};
```

普通依赖边：

```text
blocker task -> requesting shelf task
```

graph 是当前 upper epoch 的 guidance snapshot，不跨 shelf movement 累积。
loaded Move 后旧 graph 整体失效，由新 `U` 重新编译。

### 3.4 PairPlan

```cpp
struct PairPlan {
  double estimated_cost;
  int rollout_steps;
  bool reached_goal;
  bool truncated;
  bool stalled;
};
```

PairPlan 只存 shelf-side rollout 结果，不存 robot assignment。

### 3.5 CarrierGuidance

最终节点 guidance 至少包含：

```cpp
struct CarrierGuidance {
  UpperSignature upper_signature;
  PairCostTable pair_cost;
  std::vector<int> tau_guide;
  std::vector<int> target_priority;
  ShelfTaskGraph task_graph;
  std::vector<int> ready_tasks;
  std::vector<std::optional<TaskId>> rho_task_id; // nullopt = IDLE
  std::vector<int> rho_ready_index; // derived index in this guidance only
  std::vector<std::optional<Custody>> custody_by_robot;
};
```

`TaskId` 是 exact tuple value，不是可能碰撞的 hash；hash 只用于容器索引。
`rho_task_id` 是跨 guidance snapshot 的稳定 binding；switch penalty 只比较
它。`rho_ready_index` 只是当前 `ready_tasks/task_graph` 的执行加速视图，
每次 attach 都由 `rho_task_id` 重新解析，绝不能传给下一 epoch 计算
hysteresis。task vector 重排但 TaskId 不变时不是 switch；本地 index 相同但
TaskId 已变时必须算 switch。

`custody_by_robot.size() == number_of_robots`。free robot 必为 `nullopt`；
loaded robot 可以是 exact bound `Custody`，也可以是合法的 unbound
`nullopt`，其 loaded/free 状态由 physical `kappa` 判断，绝不使用伪
TaskId sentinel。

旧字段 `ObjectiveOption`、`selected_packages`、claims ledger、
`obj_reselect_*`、`obj_yields` 在迁移完成后删除。

### 3.6 Search edge 与 transition trace

rewire 需要的是“候选新父边”的 trace，而不是 node 当前旧父边的 trace。
因此 adjacency 不能继续只存目标 node 指针。每条生成出来的普通或 macro
edge 都必须有不可变记录：

```cpp
struct TransitionStep {
  PhysicalState previous_X;
  JointOps ops;
  PhysicalState next_X;
};

struct SearchEdge {
  TAPFNode* to;
  double physical_cost;
  std::vector<TransitionStep> transition_trace;
};

using SearchEdgeHandle = std::shared_ptr<const SearchEdge>;
```

语义：

* 普通 edge 的 `transition_trace.size() == 1`；
* macro edge 保存每一拍 ops 与对应 `next_X`；
* successor 即使命中 CLOSED duplicate，也要把本次 candidate edge record
  挂到 `from.outgoing_edges`，供 anytime rewrite 使用；
* `from.outgoing_edges` 保存 `SearchEdgeHandle`，避免 adjacency 容器扩容
  使 handle 失效；
* node 保存 `parent` 与其当前 `SearchEdgeHandle incoming_edge`；两者必须
  一起替换；
* `get_edge_cost()`、plan extraction、guidance replay 都读取同一 edge
  record，不能各自维护一份可能不一致的 macro map；
* replay 从 fresh parent `X` 开始，逐步验证
  `apply_ops(anchor.X, step.ops) == step.next_X`，终态必须等于 `to.X`。

同一 `(from,to)` 可以存在多条不同 cost/trace 的 candidate edges；rewrite
必须遍历 edge records，而不是只遍历去重后的 neighbor nodes。
`RegisterOutgoingEdge()` 至少合并逐拍 trace 完全相同的重复记录；允许再按
稳定 `(physical_cost, serialized trace)` 规则为同一 `(from,to)` 只保留
不会劣于其他候选的 canonical record。若替换 canonical record，已经作为
某 node `incoming_edge` 的 immutable handle 仍保持有效，直到该 node
reparent 或 search cleanup。所有 edge handles 随 CLOSED/deferred cleanup
统一释放，不能形成无界的进程级 cache。

---

## 4. 第一层：single-root PairCost

### 4.1 纯度

对每个 target shelf `b` 和每个 `g ∈ G_b`：

```text
C_B[b,g] = PairCost(U, b, g)
```

`PairCost` 可以读取：

* walls；
* 当前 target/anonymous shelf cells；
* shelf kind；
* `b` 和 `g`；
* 固定的 shelf-side cost weights 和 compiler budgets。

`PairCost` 禁止读取：

* robot positions 或 robot 数量的动态状态；
* free/loaded robot 集合；
* lower-deck distance/congestion；
* rho 或 parent rho；
* 当前由哪个 robot 携带 shelf；
* execution price、claims、park 或 robot taboo。

因此同一个 `U,b,g` 必须逐位得到同一个 PairPlan。

### 4.2 “单独评估”

`CompilePair(U,b,g)` 只有一个 root demand：

```text
b -> g
```

其他 shelves 保留在布局中并可作为 movable blockers，但其他 target
shelves 暂时没有自己的 terminal objective。它们若被搬动，只作为
`b -> g` 的 blocker tasks。

### 4.3 Bounded rollout

```text
PairCost(U, b, g):
    if position_U(b) == g:
        return 0

    U_hat = U
    assign deterministic local AbstractShelfTokens
    open_episode_token = NONE
    cost = 0

    repeat until b reaches g or pair budget exhausted:
        D = CompileTaskBRPIBT(
                U_hat,
                roots = {b -> g},
                single_root_mode = true)

        ready = ReadyAbstractTasks(D, U_hat)
        m = deterministic highest-ranked task in ready

        if m does not exist:
            stalled = true
            break

        token = AbstractTokenOf(m.shelf)

        if open_episode_token != token:
            if open_episode_token != NONE:
                cost += gamma          // drop previous shelf
            cost += gamma              // lift new shelf
            open_episode_token = token

        for each adjacent leg u -> v in m.transfer.route:
            abstractly apply u -> v in U_hat
            move token from u to v
            cost += alpha              // one loaded shelf shift
            if m.shelf is anonymous:
                cost += delta

    if open_episode_token != NONE:
        cost += gamma                  // final drop estimate

    if b did not reach g:
        cost += ResidualEstimate(U_hat, b, g)
        if stalled:
            cost += STALL_PENALTY

    return cost
```

这只是 guidance estimate，不要求 admissible。连续移动同一 shelf 时只估计
一次 Lift/Drop episode；切换 shelf 时结束旧 episode 并开启新 episode。
anonymous token 的初始编号只由 canonical sorted positions 决定，所以输入
中的匿名排列不能改变 PairCost。

`m.transfer.route` 的 endpoint 必须是合法 storage cell。PairCost 在下一次
调用 compiler 前把整条 route 原子地抽象执行完；它不能在 transit cell
停下并重新选择 blocker，否则 abstract rollout 会重现真实执行中的
“进走廊—无任务—回原位”振荡。rollout budget 仍按 route 中的相邻 shelf
effects 计数；若剩余预算不足以完成整条 transfer，则不开始该 transfer，
返回有限 truncated residual。

### 4.4 Residual 与 failure

最低 residual 形式：

```text
alpha * wall_distance(position(b), g)
+ generic remaining manipulation estimate
+ bounded stall/truncation penalty
```

Task-BR-PIBT rollout 没找到 ready task、命中 recursion budget、遇到
zero-empty cycle 或候选耗尽，都只能产生有限 penalty，不能把 edge 设为
不可行。

只有以下情况可令 `C_B[b,g] = INF`：

1. `g` 不属于 `G_b`；
2. walls 证明 `position(b)` 与 `g` 不在同一 connected component。

### 4.5 Pair cache

```text
PairCacheKey = (
  UpperSignature,
  target b,
  goal g,
  compiler-cost version
)
```

第一版采用保守失效：任何 shelf coordinate 改变都生成新 signature，
该 signature 下的 lazy-exact certificate 按需重建；未求精 edge 只保留
§5.1.1 的 lower bound。只有证明 rollout footprint 不受某次 movement
影响后，才能增加细粒度增量复用；不得先假设“只影响某几行”。

缓存必须有容量上限或 LRU，避免 CLOSED 中大量 upper layouts 造成无界内存。

---

## 5. Shelf-goal matching

### 5.1 `tau_guide`

```text
tau_guide(U) =
  argmin over injective eligible assignments
  sum_b C_B[b, tau_guide[b]]
```

要求：

* `tau_guide[b] ∈ G_b`；
* 不同 targets 不得占用同一个 terminal goal；
* primary objective 只使用 `C_B`；
* matching 精确优化词典序
  `(total PairCost, moved-away-eligible-count, assignment-vector)`：
  `moved-away-eligible-count` 统计当前 coordinate 已在 `G_b`、却被分到其他
  goal 的 targets；`assignment-vector` 按 target id 排列 goal cell；
* 上述 secondary/tertiary tie 不读取 grounded/carried 状态，也不得覆盖
  更小的 total PairCost；实现须用精确词典序比较（分层比较或缩放前
  证明 `secondary 项总和 < primary 最小可分辨差`），禁止把 double
  PairCost 与 tie 项直接线性相加后因量化丢失 primary 差异；
* 不使用 parent tau hysteresis、robot execution price 或 claims pressure。

由此，同一个 `U` 必须得到同一个 `tau_guide`。robot-only movement、
Lift、Drop 都不能改变它。

### 5.1.1 Lazy exact matching certificate

production 不必先把每条 eligible edge 的完整 rollout 都算完，但最终
`tau_guide` 必须与完整 PairCost matrix 的精确词典序解逐位一致。实现使用
四步 certificate：

1. 每条 edge 先存 prefix lower bound \(L_e\)，并保证
   \(L_e \le C_e\)；有限的完整 PairCost 不得对应 `INF` lower bound。
2. mixed matrix 中未求值 edge 存 \(L_e\)，已求值 edge 存精确
   \(C_e\)。edge 只允许从 lower bound 单调转换为 exact cost。
3. 每轮先把当前 Hungarian assignment 上的所有 edge 求精，再重解，直到
   当前 assignment 的总代价 \(C^\*\) 已完全精确。
4. 对每条尚未求精的 edge \(e=(b,g)\)，强制 \(b\mapsto g\)，删除该
   row/column 后对剩余 matrix 再做 Hungarian，得到
   \(F_L(e)\)。只要 \(F_L(e)\le C^\*\) 就必须求精；相等也不能跳过。

终止时所有未求精 edge 都满足 \(F_L(e)>C^\*\)，所以任何 primary-optimal
injective assignment 都不可能包含它们；所有 primary-optimal edges 已是
exact，secondary `moved-away` 与 tertiary assignment-vector tie 也因此精确。
`PairPlan.exact=false` 只表示 branch-and-bound lower bound；priority、
rollout stall/truncation 统计、cache consumer 或任何把 steps 当完整 rollout
的逻辑都不得读取它。selected `tau_guide` edge 在进入 priority 前有运行时
exact 断言。求值次序和 cache warm-up history 不能改变结果。

### 5.2 `tau_LB`

admissible heuristic 使用另一张 matrix：

```text
LB[b,g] =
  alpha * upper_wall_distance(position(b), g)
  + admissible operation lower bound from X

h_shelf(X) =
  min over injective eligible assignments
  sum_b LB[b, assignment[b]]
```

`tau_LB`/`h_shelf` 可以读取完整物理状态 `X`，例如 carried target 至少还需
一次 Drop；它不能读取 PairCost、blocker count、rollout、priority、rho
或 task graph。

代码上必须拆成两个明确 API，避免再次把 guidance matching 和 admissible
matching 混在 `solve_tau()` 中：

```text
solve_tau_guide(pair_cost_matrix)
solve_tau_lb(X)
```

---

## 6. 第二层：联合 Task-BR-PIBT compiler

### 6.1 输入与输出

```text
CompileJoint(U, tau_guide, target_priority)
    -> ShelfTaskGraph D
```

所有尚未完成 upper movement 的 target shelves 都作为 roots：

```text
b -> tau_guide[b]
```

compiler 不再选择 terminal goal。目标变化只能发生在下一次 upper epoch
重新计算 PairCost 与 Hungarian 时。

### 6.2 Candidate ordering

无显式 `storage_map` 时，候选仍是四邻接 cell。显式 `storage_map` 时，
候选是 storage-to-storage transfers：

```text
from storage cell
  -> zero or more currently empty transit cells
  -> first encountered storage endpoint
```

搜索 route 时不能穿过中间 storage cell；遇到 storage 即形成 endpoint 并
停止该 branch。route 内部 transit cell 不能被 shelf 占据，endpoint 可以
为空或被 blocker 占据。每个 endpoint 只保留确定性的最短 route，并按稳定
cell id 打破平局；最多把排序最优的四个 transfers 送入现有 candidate
window。若当前 shelf 已因真实 transition 位于 transit cell，则只允许找
到 storage 出口的 route，不能把另一个 transit cell 当成 endpoint。

对 root target `b` 的 transfer 候选：

1. 更小的 `wall_distance(endpoint, tau_guide[b])`；
2. 更低的预计 displacement cost；
3. 不与已接受的更高优先级 tasks 冲突；
4. 更短 route；
5. 稳定 endpoint/cell-id tie。

对被要求让路的 target blocker：

1. 能完成 inherited vacate request；
2. empty storage endpoint 优先；
3. 尽量不增加 endpoint 到自己 `tau_guide` 的距离；
4. 避免已预约 endpoint 和第一格 destination；
5. 更短 route；
6. 稳定 endpoint/cell-id tie。

对 anonymous blocker：

1. empty storage endpoint 优先；
2. 最短 displacement chain；
3. 避免已预约 endpoint 和第一格 destination；
4. 更短 route；
5. 稳定 endpoint/cell-id tie。

候选只依赖 upper deck；不得使用当前 robot 距离。

### 6.3 Recursive displacement

```text
ResolveShelf(s, root, inherited_priority, context,
             forced_first_transfer = NONE):
    u = position_U(s)

    transfers =
      forced_first_transfer != NONE
        ? [forced_first_transfer]
        : OrderedStorageTransferCandidates(s, root, context)

    for transfer in transfers:
        v = transfer.route[1]
        endpoint = transfer.endpoint

        if route is not adjacent throughout
           or route[0] != u
           or route.back() != endpoint
           or endpoint is not legal storage:
            continue

        snapshot reservations and graph

        if exact task (s, u, v) already exists:
            merge root demand into that task
            mark predecessor-demand closure dirty
            return existing task

        if s already reserved for a different effect:
            restore snapshot
            continue

        if v is reserved as an incompatible destination:
            restore snapshot
            continue

        if endpoint is reserved by an incompatible transfer:
            restore snapshot
            continue

        if any internal transit route cell is occupied by a shelf:
            restore snapshot
            continue

        if endpoint is occupied in U:
            blocker = shelf occupying endpoint

            if blocker is in recursion stack:
                record a rotation candidate when cycle length >= 3
                restore snapshot
                continue

            pred = ResolveShelf(
                     blocker,
                     root,
                     inherited_priority,
                     context,
                     forced_first_transfer = NONE)

            if pred fails:
                restore snapshot
                continue

        create Task(s, u -> v, transfer)
        reserve s, v, endpoint and route
        if pred exists:
            add dependency pred -> task
        return task

    return FAIL
```

`context` 至少维护：

```text
reserved_shelf_effect
reserved_destination
recursion_stack
task index by exact effect
candidate/backtrack budget
```

递归失败时请求方必须尝试下一 candidate；失败不能直接产生“不可解”判断。
`forced_first_transfer` 只绑定当前 root 在 root-level DFS 选择的第一个
storage transfer；其 `TaskId` 仍只取 route 的第一格 effect；
它不得传给 blocker。这样 blocker 仍可枚举自己的 displacement candidates，
但当前 root 失败时不能偷偷改选另一个 destination 并让外层误以为指定
option 成功。

storage endpoint reservation 与第一格 destination reservation 都是
transactional。失败 branch rollback 时二者必须一起撤销。两个 transfers
即使第一格不同，只要最终 endpoint 相同，也不能同时被接受；相反，transit
cell 只作为中间路径，不能作为 endpoint，也不在静态 dependency graph 中
被永久预约。真实同时在途 route 的冲突由 active-transfer claims 处理。

graph 完成后必须执行一次 reverse-topological demand propagation：

```text
for task in reverse topological order:
    for each predecessor pred of task:
        pred.roots |= task.roots

for each task:
    task.priority = max(priority[root] for root in task.roots)
```

语义是：一个 root 需要某个 non-ready task，就同时需要让该 task 成立的全部
predecessors。仅在 shared effect node 上合并 roots 而不向 blocker chain
传播，会让真正 ready 的 leaf 丢失高优先级，属于错误实现。

### 6.4 多 roots 与 root-level backtracking

roots 按 effective priority 降序、target id 稳定排序。联合 compiler 使用
有界 DFS/transaction：

```text
CompileRoots(k):
    if k == number of roots:
        record current graph as a complete candidate
        return

    A = root_order[k]

    for v in OrderedRootMoveOptions(A):
        snapshot graph/reservations

        if ResolveShelf(
             A,
             root = A,
             inherited_priority = priority[A],
             context,
             forced_first_to = v) succeeds:
            assert selected root effect is exactly
                   (A, position_U(A), v)
            CompileRoots(k + 1)

        restore snapshot

    if A has not started an active transfer:
        record A as deferred-unstarted for this upper epoch
    CompileRoots(k + 1)
```

这里的 deferred-unstarted（旧统计字段仍可记作 `paused_roots`）只表示本轮
不为尚未开始的 root 生成新 transfer。例如两个未启动请求只能竞争同一个
合法 endpoint 时，本轮启动优先级更高者，另一个留待下一 upper epoch；
它不关闭任何编号的 robot、shelf、target、Task-BR 或 physical successor。
已进入 transit 的 active transfer 不参与这条分支，必须继续到既定
storage endpoint。

候选图按以下词典序比较：

1. 更高优先级 root 是否成功编译；
2. aggregate remaining distance；
3. density-aware priority progress：
   - upper vacancy 至少为 2 时，按已排序 root 顺序逐项比较 chosen first
     effect 到 terminal goal 的完整 remaining wall distance；
   - upper vacancy 为 0 或 1 时，只逐项比较 root 是否在本 effect 后完成，
     不追逐尚未完成 root 的微小距离差；
4. 总 task/work 估计；
5. 稳定 effect order。

aggregate 必须先于逐根 tie-break：`[1,1]` 应优于 `[0,100]`。aggregate
相同时，density-aware 第 3 层同时解决两种相反风险：在单空位链中，
`[0,2]` 的首根已完成，必须优于 `[2,0]`；在至少两空位时，完整逐根
residual 又能阻止高优先级 root 的非零进展被低优先级 root 抵消。否则
compiler 可能追逐一格局部改善，或让刚搬开的 blocker 立即反向复位。

这使得：

* 低优 root 首先围绕高优 root 自适应；
* 如果高优 root 的第一个 option 让低优 root 无法编译，高优 root 可以在
  自己仍成功的前提下回退到下一 option；
* 不会为了让低优 root 成功而直接删除高优 root 的全部 guidance；
* budget 耗尽时返回目前最好的 partial graph，并显式记录 paused roots。

`JOINT_RECURSION_CAP`、`JOINT_BACKTRACK_CAP` 和每个 shelf 的 candidate cap
都是 guidance runtime guards，不是物理剪枝。production 的最终预算为：

* single-root PairCost rollout 最多走
  `max(8, min(128, 2 * |V|))` 个相邻 shelf effects；单次递归/回溯窗口分别为
  `max(32, 4 * |V|)` 与 `max(64, 8 * |V|)`；
* joint compiler 对每个 top-level root option 重新获得 256 次局部递归窗口，
  整个 upper epoch 最多检查 512 个 root options；
* upper vacancy 不超过 2 时，不再另设 epoch-wide recursion cap，因为一个
  有效备选本身就可能需要完整 vacancy chain；vacancy 大于 2 时，累计递归
  上限为 `256 + 512 = 768`，防止大量失败 option 反复刷新局部窗口；
* 任何预算耗尽都返回当时最好的 partial graph 和 paused roots，不能删除
  operator tree 中的物理 successor。

每格最多四个相邻候选。PairCost 热路径先一次性计算四个完整 score tuple，
再做固定长度 insertion sort；score 末位是稳定 cell id，因此它与原
`stable_sort` 的总序逐位等价，只消除了数百万次小对象排序开销。
PairCost 与 joint compiler 调用同一个 templated recursive core；PairCost
context 只省略不被返回值消费的 graph materialization，并使用 direct
target/anonymous index、generation-stamped dense reservations 和可复用
scratch。它不能拥有第二套 candidate/recursion semantics。

storage-aware 扩展仍保持同一个 recursive core，但候选窗口不得把四个
可能拥有动态内存的 transfer object 放在热路径数组中。窗口以固定 primitive
数组保存 `endpoint/first_step/route_size/route-slot`，非相邻 route 单独
放入窗口拥有的 route pool；resolver 只持有该窗口生命周期内的 trivial
view。direct-storage 分支继续使用原五字段 score 总序，只有通用 transfer
分支才把 route length 与 first step 加入 score。joint compiler 在 task
被接受时物化 `StorageTransfer`；PairCost 在选中 ready transfer 时才复制
长 route，直接二点 route 不发生动态分配。single-root scratch 同理只在
首次出现 `endpoint != first_step` 时创建 endpoint-reservation dense
storage，并按 shelf 数一次预留 rollback undo。以上均为表示与容量优化，
候选集合、排序、冲突、递归和 rollback 结果必须逐位不变。

### 6.5 Shared effect 与 conflict

相同 exact effect：

```text
A requires ShelfTask(X, u -> v)
B requires ShelfTask(X, u -> v)
```

合并为一个 node：

```text
roots = {A, B}
priority = max(priority[A], priority[B])
```

以下情况是真冲突，不能只加 dependency：

```text
same shelf, same from, different to
different shelves, same destination
one shelf reserved by two different effects
```

旧 `(shelf,from)` 合并规则必须删除。`to` 是 task effect 的组成部分，不是
可在 custody 中随意重写的 advisory hint。

### 6.6 Target priority

priority 属于 root mission，不属于被搬动的 blocker shelf。blocker task
继承请求它移动的 root priority；shared task 取 roots 最大 priority。

base priority 在每个 upper epoch 由
`PairCost(b, tau_guide[b])` 产生：代价更大的未完成 mission 优先，target id
负责稳定打破平局。真实 loaded Move 完成一个 exact custody task 后，允许
把该 task 所服务的 root priority 短暂承诺到下一个 upper epoch，但必须同时
满足：

```text
root 尚未到达 goal
新 tau_guide[root] 仍等于完成 task 时的 root goal
singleton fixed-goal target 的 self move 不自我续约
multi-goal root 只在 target-dense upper layout 中续约
```

若同一 completed shared effect 服务多个通过上述过滤的 roots，先选择
“旧 priority 最高，随后 exact TaskId、robot id 最小”的 group。该 group
严格超过 active roots 的一半时全体续约；否则通常只续约其中最高优先 root。
有两种情况仍采用 collective renewal：active roots 已达到至少两倍 upper
vacancies 的 vacancy pressure，或上一个 collective commitment 与本 group
有交集。承诺 root 在下一 epoch 被提升到 base priority 之上；tau 改变、
mission 完成或过滤失败时立即丢弃。

target-dense 的实现谓词为：

```text
n_targets > n_vacancies
and n_targets - n_vacancies >= n_vacancies
```

priority commitment 只改变 guidance order；它不进入 state key、不进入
`h`，也不改变 legal successor set。UpperEpochCache 的 key 因此是
`(UpperSignature, priority_commitment)`，而同一 `UpperSignature` 的
PairCost/tau 数据仍可复用。

**已测量的 SOC 风险（迁移验证必须覆盖）**：v4.1 的 SOC 根因分析
（`soc_root_cause_report.md`；2026-09-02 经三轮独立审核、第三轮
APPROVE，审核轮次/复算范围的可审计记录见该报告头部）证明，
“最难使命恒占最高优先级 + 优先级无条件抢占分配行”在松散盘面上会把
nearest-first 的 SOC 友好贪心整体反转为 farthest-first，共同成功集 SOC
几何比恶化至 1.15，最差单例 3.82x；按搜索节点链累积的 aging 又使抢占
目标反复翻转、打断粘滞。本设计的结构性缓解是：rho 只见 ready tasks
（不存在派往不可启动 pickup 的浪费）、priority 只随 upper epoch 更新
（消除节点级翻转）、且没有“替换整个前缀”的保留槽。但 §8.2 的
priority-first 词典序在 `|ready| > |free|` 时仍可能复现 LPT 式 SOC 税。
因此 release gate（§17.3）必须报告 common-set SOC/mk 几何比对
`results_v3_strict_return_final` 与 `results_v4_1_final7` 两个基线的
对照；若出现系统性恶化，优先审视 base priority 的方向（hardest-first
vs nearest-first）与 starvation age 的触发条件，而不是引入新的
node-level 抢占阀门。

---

## 7. Dependency graph 与 ready tasks

### 7.1 Ready definition

任务 `m` ready 当且仅当：

```text
all predecessors are already physically satisfied
m.from is occupied by m.shelf
m.to is currently empty on the upper deck
m.shelf is not held by an unrelated robot
m is not already in another robot's custody
```

因为 graph 会在每次 shelf movement 后重编译，普通 DAG 中“所有
predecessors satisfied”等价于当前 snapshot 的 zero-indegree leaf 加上
物理检查。

只有 ready tasks 能进入 robot assignment。内部 dependency nodes 绝不让
robot 提前 approach 并长期等待。

### 7.2 Grounded task 与 carried continuation

* ready task 的 shelf grounded：进入 free-robot matching；
* robot 已有 exact custody：继续执行该 one-step task，不参加 Hungarian；
* 上一拍 loaded Move 开启新 epoch，且新 graph 对同一 carried shelf 有
  selector/from 匹配的 exact ready continuation：可直接建立新 custody，
  保持 Lift，不参加 Hungarian；但 roomy layout 中若该 continuation 恰好
  返回上一拍的 `from`，不自动绑定，dense layout 才允许直接绑定这种 exact
  reverse；
* 其他 `custody_by_robot[r] == nullopt` 的 carried shelf 都视为 unbound，
  preferred action 为原地 Drop；不得在同一 U 的后续 Wait/reattach 中
  retroactively 认领一个 task。

roomy layout 被抑制的 reverse task 仍保留在 ready set，operator tree 也仍
枚举该 loaded Move；这里只避免把 carrier 立即锁进两格往返的 preferred
continuation，不是物理剪枝。

### 7.3 Custody

```cpp
struct Custody {
  TaskId task_id;       // exact tuple value (shelf, from, to)
  std::optional<int> current_task_index; // derived in current D only
  ShelfSelector shelf;
  int from;
  int to;
  std::vector<RootDemand> roots;
  int priority;
  int transfer_endpoint;
  std::vector<int> transfer_route;
  size_t transfer_index; // from=route[index], to=route[index+1]
};
```

强制保持 `task_id == TaskId{shelf, from, to}`。`current_task_index` 不是
identity：每次 attach/reanchor 都按 exact TaskId 在当前 `D` 中重新解析；
找不到时可为 `nullopt`，但 loaded Wait 不能因此把 custody 换成本地相同
index 的另一 effect。custody 的连续性只由 exact tuple 与真实 transition
验证。

生命周期：

```text
free robot reaches task.from
    -> Lift
    -> custody starts; U/tau/D unchanged
    -> zero or more robot-only Waits caused by lower-deck conflicts
    -> one loaded Move from task.from to task.to
    -> task completes; new upper epoch starts
```

新 epoch 不延长旧 TaskId。若新 graph 中同一 physical shelf 有
`current_cell -> next_cell` ready continuation，当前 carrier 按 §7.2 的
density-aware 规则决定是否直接绑定新 TaskId；未绑定时保持
loaded-but-unbound，并首选 Drop。

上述规则对普通 `[from,to]` transfer 不变。若 custody 的
`transfer_route.size() > 2` 且尚未到达 `transfer_endpoint`，loaded Move
只完成当前 route leg，不结束 storage transfer：

```text
old TaskId = (shelf, route[k], route[k+1])
loaded Move to route[k+1]
new TaskId = (shelf, route[k+1], route[k+2])
transfer_endpoint unchanged
transfer_index = k + 1
```

这个下一 leg custody 由 Recover 阶段从真实 loaded Move 和旧 custody
确定性派生，不依赖新 `tau` 或新 graph，也不参加 rho。下一 route cell
暂时被其他 carried shelf/robot 占用时保留 commitment 并首选 Wait；不能
把 transit cell 当作完成位置，也不能自动反向返回 route 起点。

到达 `transfer_endpoint` 后 storage transfer 才完成，随后才能按普通规则
绑定新 graph continuation 或首选 Drop。若 operator tree 强制偏离 route，
旧 transfer 失效；偏离后的 shelf 若仍在 transit cell，Recover 阶段必须从
该真实 transition anchor 建立到当前可达空 storage endpoint 的 recovery
transfer。没有可达空 endpoint 时保持 loaded-but-unbound 并 Wait，绝不
尝试非法 Drop。

in-flight task 在 loaded Move 前不能被新的 rho 或 priority 取消。这是
正常 unconstrained guidance 的 preferred continuity，不是 physical hard constraint，也
不进入 state key。operator constraint tree 仍可强制枚举 Drop、其他 loaded
Move 或其他合法 primitive：

* 若实际 loaded Move 恰好等于 `from -> to`，旧 task 完成；
* 若实际 loaded Move 偏离 `to`，upper epoch 改变，旧 custody 失效并从新
  physical state 重编译；
* 若实际 Drop，`U` 不变但 custody 清除，旧 task 可重新成为 grounded ready；
* Wait 或 lower-deck blocking 不取消 custody。

所以“不可被 rho/priority 取消”只约束 preferred guidance，不得删除任何
合法 successor。

custody 只能从紧邻的真实 transition anchor
`{previous X, previous guidance, executed ops}` 恢复。**职责划分是
固定的三段式，禁止重叠**：(1) Recover 阶段做三类 transition 派生
操作——保留仍有效的旧 custody（loaded Wait）、使已完成或失效的
custody 变为 nullopt（loaded Move / Drop / 偏离），以及在
assigned-ready **Lift** 上把上一拍 `rho_task_id[r]` 确定性地**转入**
custody（这是 rho 绑定随执行进入 custody 的转移，完全由
`{previous rho, executed Lift}` 决定，不含任何新决策）；它绝不建立
loaded-Move 新 epoch 的 continuation custody；(2)
`ReadyTasks(D, X, custody)` 用物理谓词判定 ready，其中
“shelf 未被无关 robot 持有”对“恰好由刚完成 predecessor 的当前
carrier 持有”的 shelf 放行，使 continuation 候选可见；(3)
`BindReadyContinuations` 是唯一建立 **continuation** custody 的入口，
且只对上一拍 loaded Move 开启新 epoch 的 carrier 生效。逐 robot
规则如下：

```text
free robot:
    custody_by_robot[r] = nullopt

Lift:
    if previous rho_task_id[r] resolves to the exact ready shelf/effect lifted:
        establish that exact Custody
    else:  // exhaustive tree forced an unassigned/non-ready Lift
        custody_by_robot[r] = nullopt

loaded Wait:
    preserve previous exact Custody only if shelf/from/to still validate
    otherwise remain/turn unbound nullopt

loaded Move:
    if exact move completes only an intermediate storage-transfer leg:
        deterministically advance the same transfer Custody
        (endpoint unchanged; this is not a graph continuation decision)
    else:
        complete or invalidate the previous Custody
    compile the new upper epoch, then compute ReadyTasks
    the follow-up BindReadyContinuations step—and only it—binds a new
    graph-derived Custody iff the new graph has an exact ready continuation for this
    carried shelf at its current cell and either it is not the immediate
    reverse or the upper layout is target-dense
    otherwise custody_by_robot[r] stays nullopt

Drop:
    custody_by_robot[r] = nullopt
```

因此 loaded-but-unbound 是一等合法状态，Carrier-PIBT 首选原地 Drop，
operator constraint tree 仍保留 Wait/Move/Drop 等所有合法 primitive。
不得提供“在同一个 node 上调用 reguide 并复制任意旧 custody”的接口；
没有对应真实 transition 的手工注入或历史 guidance 不能成为 custody 来源。

---

## 8. 第三层：rho matching

### 8.1 输入

先移除：

* 非 free robots；
* 已直接绑定给当前 carrier 的 continuation tasks；
* non-ready tasks。

剩余问题：

```text
free robots R_f
grounded ready tasks M_r
private IDLE choices
```

### 8.2 Lexicographic objective

rho 严格按以下顺序优化：

1. 服务更高 task priority；
2. 在 priority cutoff 并列时选择总体 approach distance 更小的 tasks；
3. 最小化 `beta * lower_distance(robot, task.from)`；
4. 最小化 TaskId switch penalty；
5. 稳定 `(robot id, TaskId)` tie。

不得像当前代码一样先按旧 `100/50-k/depth` 截断任务池，再只对截断结果做
Hungarian。

一种精确实现方式：

1. 若 `|M_r| <= |R_f|`，全部 tasks 进入 assignment；
2. 若 `|M_r| > |R_f|`，找出第 `|R_f|` 位 priority cutoff；
3. 高于 cutoff 的 tasks 禁止丢弃；
4. cutoff 同级 tasks 与 robot、必要的 drop-dummy columns 一起进入 padded
   Hungarian，让 distance/switch 决定同级中选谁；
5. 低于 cutoff 的 tasks 本节点不服务。

matcher 的接口以 TaskId 为跨节点 identity：

```text
MatchReadyTasks(
    free_robots,
    current_ready_tasks,
    previous_rho_task_id)
  -> current_rho_task_id

rho_ready_index =
    ResolveTaskIdsInCurrentGraph(
        current_rho_task_id,
        current_ready_tasks)
```

`previous_rho_task_id` 中已经不在当前 ready set 的 ID 只表示“旧 task 已
消失”，不能按其旧 vector index 解析。IDLE 使用 `nullopt`，不与任一合法
TaskId/index 混用。

### 8.3 Robot-only repair

同一 upper epoch 内：

```text
free robot Move/Wait
    -> PairCost unchanged
    -> tau_guide unchanged
    -> priority unchanged
    -> task graph unchanged
    -> ready set normally unchanged
    -> only rho is repaired
```

TaskId switch hysteresis只作用于 `rho` 的次级 cost，不能反馈进
`tau_guide`。

rho repair 的全部历史输入仅限上一拍 `rho_task_id`，用于上述 switch
penalty。当前 graph 重编译或 task vector 重排后，先按 exact TaskId 判断
continuity，再生成本地 `rho_ready_index`。不得读取 revisit/no-progress
计数、wait-for cycle memory、
taboo pair 或 `reguide` 次数。lower-deck PIBT 本拍未能执行首选 task 时，
保留 Wait/其他 primitive fallback；只有真实 successor transition 才触发
下一次 repair。

---

## 9. Carrier-PIBT execution

Carrier-PIBT 根据 task phase 生成 preferred operator candidates：

```text
free + assigned grounded ready task:
    move toward task.from
    if at task.from and shelf/effect still valid:
        Lift
    else:
        Wait / ordinary fallback moves

carrying + bound one-step task:
    prefer loaded Move exactly to task.to
    if lower-deck robot blocks:
        inherit task priority through ordinary PIBT recursion
    retain Wait/Drop/other primitive candidates as completeness fallbacks

carrying + bound storage transfer:
    prefer the next adjacent route leg
    keep the same legal storage endpoint across upper epochs
    if the next leg is temporarily blocked, prefer Wait
    never offer Drop while current cell is non-storage

carrying + newly bound exact continuation:
    use the bound one-step task without Drop/Lift

carrying + unbound:
    if current cell is storage:
        prefer Drop at the current cell
    else:
        use transition-anchored recovery route to an empty storage endpoint
        or Wait when no legal endpoint is currently reachable

free + IDLE:
    leave cells used by current ready/custody effects when possible
    otherwise Wait
```

优先级链为：

```text
target root priority
  -> inherited blocker ShelfTask priority
  -> assigned/carrier robot priority
  -> blocking lower-deck robot priority
```

Task-BR-PIBT 负责多步 shelf causality；Carrier-PIBT 只负责当前 timestep 的
robot conflicts。最终 joint operator 始终交给 `apply_ops()`。

这里不存在 `target_park` 或 `parking_cell`。idle 避让只能从当前
`D/ready/custody` footprint 派生，不能读取旧 least-blocking paths。一个
当前物理前置条件成立的 ready Lift 也不得因全局 futile-Lift counter 或
cooldown 被降级；若 Lift 后无法产生期望进展，搜索依靠真实 successor、
下一拍重编译和约束树的其他 primitive 分支处理。

---

## 10. Transition 与 cache invalidation

| Physical transition | UpperSignature | PairCost / tau | priority / D | ready / rho / custody |
|---|---|---|---|---|
| root | 新建 | 计算或 cache hit | 编译 | 计算 |
| free robot `Move/Wait` | 不变 | 复用 | 复用 | repair rho |
| assigned-ready `Lift` | 不变 | 复用 | 复用 | exact TaskId 从 rho 转入 custody；重算 ready/rho |
| forced/unassigned `Lift` | 不变 | 复用 | 复用 | loaded 但 `custody=nullopt`；不得伪造 TaskId；重算 ready/rho |
| loaded `Wait` | 不变 | 复用 | 复用 | bound 时验证并保持 exact custody；unbound 时仍为 nullopt；repair lower guidance/rho |
| loaded `Move` 到 custody.to | 改变 | 重新取得 lazy-exact PairCost certificate 并解 tau；selected/primary-tight edges exact | 更新 upper-epoch priority，重新 CompileJoint | 旧 task 完成；判断 continuation；重算 rho |
| `Drop` | 不变 | 复用 | 复用 | 清 custody；旧 task 可重新 ready；重算 rho |
| exhaustive tree 产生的偏离 loaded Move | 改变 | 与普通 loaded Move 相同 | 全部重编译 | 旧 custody 失效；按新物理事实重新绑定或 Drop |

禁止再使用“只有 target Drop 才允许 tau 改变”的 `preserve_tau` 语义。

若该 loaded Move 只完成 storage transfer 的中间 leg，表中最后一列改为：
旧 adjacent `TaskId` 完成，但 Recover 从真实 transition 推进
`transfer_index` 并建立下一 adjacent `TaskId`；PairCost/tau/D 仍按新的 U
正常重算，但不得覆盖 transfer endpoint。

### 10.1 Macro rollout

当前 `carrier_rollout()` 把整份 guidance 冻结复用 8 步；这与 upper-epoch
规则冲突。新实现每个 rollout step 都必须以紧邻前一步为 anchor 执行轻量
attach：

```text
same U:
    复用 PairCost/tau/priority/D
    重算 custody/ready/rho

loaded Move changed U:
    重新获取 PairCost/tau
    更新 priority 并重编 D
    再算 custody/ready/rho
```

因此删除“每 8 步才刷新整份 guidance”的语义；可以每 8 步做 cache maintenance
或统计，但不能跨 Lift/Drop/loaded Move 冻结 ready/rho/custody。

接口不能只接收 macro 起点的 search parent。每一步必须传入：

```text
TransitionContext = {
    previous_X,
    previous_guidance,
    executed_joint_ops
}
```

rollout 必须保存可重放的 primitive transition trace，并返回 terminal
guidance anchor：

```text
anchor = {node.X, EnsureGuidanceFresh(node).guide}
trace = []

for each rollout step:
    ops = GeneratePreferredOps(anchor)
    X_next = apply_ops(anchor.X, ops)
    G_next = AttachCarrierGuidance(
                 X_next,
                 {anchor.X, anchor.G, ops})
    trace.push({anchor.X, ops, X_next})
    anchor = {X_next, G_next}

return MacroResult{
    terminal_X = anchor.X,
    terminal_guidance = anchor.G,
    transition_trace = trace
}
```

macro child 的 search parent 可以跨多拍，但其 guidance 必须直接采用
`terminal_guidance`，不能拿 distant parent 再 attach 一次。incoming macro
edge 同时保留 `transition_trace`，供 duplicate rewire 后从新 parent 逐拍
重放。这样一个 rollout 中连续发生多次 loaded Move 时，每个 upper epoch
的 priority、D 与 custody 都有准确锚点。

生成终点后必须先构造并登记 candidate edge：

```text
edge = RegisterOutgoingEdge(
           from = node,
           to = terminal_node_or_duplicate,
           physical_cost = rollout.cost,
           transition_trace = trace)
```

若终点是新 node，安装 `terminal_guidance`，并把该 edge 设为
`incoming_edge`；若终点是 duplicate，edge 仍保留在 `node.outgoing_edges`，
供本次或后续 rewrite 选择。普通一拍 successor 走完全相同的登记路径，只是
trace 长度为 1。

### 10.2 Duplicate rewire / reparent

保留当前 `guidance_stale` 机制。任何 g-relax/reparent 后：

1. node 的物理 `X`、`h` 和 frozen `constraint_order` 不变；`g` 降低后
   立即重算 `f = g + h`；
2. 扩展前必须先递归刷新新 parent，再从其 fresh terminal anchor 重建；
3. `U` 相同可复用全局 PairCost/tau cache，但 parent-derived priority age
   必须重算；
4. custody 从真实 parent transition 恢复，不能沿用旧 parent episode；
5. rho hysteresis 以新 parent 的 TaskId bindings 为基准；
6. `h_shelf_LB` 不重复累加。

stale flag 必须传播到被 relaxation 影响的 descendants，保持现有 lazy
rebuild control flow。rewrite 必须遍历 §3.6 的 candidate edge records：

```text
for edge in node_from.outgoing_edges:
    node_to = edge.to
    candidate_g = node_from.g + edge.physical_cost

    if candidate_g < node_to.g:
        atomically:
            node_to.parent = node_from
            node_to.incoming_edge = immutable_handle(edge)
            node_to.g = candidate_g
            node_to.f = candidate_g + node_to.h
            node_to.guidance_stale = true
        enqueue node_to in the rewrite propagation queue
        reinsert/reprioritize node_to in OPEN under the existing policy
        mark current parent-tree descendants guidance-stale
```

reparent 必须原子更新 `parent` 与该候选 edge 的 handle；不得从 node 当前
旧 `incoming_edge` 复制 trace，也不得只凭 `(from,to)` 去查询一张可能覆盖
多条 trace 的 map。rewrite queue 继续向 descendants 传播降低后的
`g/f`；仅设置 stale flag 不能替代 cost relaxation 或必要的 OPEN 更新。

禁止 stale child 直接读取 stale `parent->guide`。统一入口为：

```text
EnsureGuidanceFresh(node):
    if node.guidance_stale == false:
        return node.guide

    if node is root:
        InstallGuidance(
            node,
            AttachRootGuidance(node.X))
    else:
        EnsureGuidanceFresh(node.parent)
        anchor = {node.parent.X, node.parent.guide}
        for step in node.incoming_edge.transition_trace:
            assert step.previous_X == anchor.X
            assert apply_ops(anchor.X, step.ops) == step.next_X
            G_next = AttachCarrierGuidance(
                         step.next_X,
                         {anchor.X, anchor.G, step.ops})
            anchor = {step.next_X, G_next}
        assert anchor.X == node.X
        InstallGuidance(node, anchor.G)

    node.guidance_stale = false
    return node.guide
```

OPEN 即使先弹出 child，也会通过该递归先刷新整条 stale parent chain。
incoming trace 对普通 edge 长度为 1，对 macro edge 可大于 1。刷新过程不得
修改 relaxation 已确定的 `g/h/f`、physical key 或 frozen
`constraint_order`；其中 `f` 已在 reparent 原子更新中变为 `g+h`。
`InstallGuidance()` 必须同时刷新用于 Carrier-PIBT 的 mutable preferred
robot order；新 node 首次安装 guidance 后只冻结一次 `constraint_order`，
rewire/reattach 绝不重写它。macro terminal guidance 与普通 attach 使用同一
安装函数。

---

## 11. One-empty 与 zero-empty

### 11.1 One-empty

示例：

```text
[A][B][C][ ]
```

若 `A` 需要进入 `B` 当前 cell，通用递归得到：

```text
Task(C, 2 -> 3)
    ↓
Task(B, 1 -> 2)
    ↓
Task(A, 0 -> 1)
```

当前唯一 ready task 是 `C:2->3`。执行后 upper layout 改变，重新编译，
下一 epoch 的 leaf 自然变成 `B:1->2`。

production code 中不得存在：

```text
if one_empty:
    run vacancy-routing compiler
```

测试可以继续验证 one-empty 行为，但必须验证它来自通用 recursion，而不是
特殊分支。

这里的 empty/vacancy 指空 storage slot。显式 storage map 中无论有多少空
aisle cells，它们都不增加 vacancy 数；一条 vacancy chain 可以通过 transit
route 把空 storage slot 在不同 storage blocks 之间搬运。

### 11.2 Zero-empty

zero-empty 可能形成：

```text
A -> B cell
B -> C cell
C -> A cell
```

当前实现会从同一 recursion stack 识别长度至少 3 的 closed cycle，以最小
`TaskId` 为 canonical 起点，去重后 transactionally 写入
`ShelfTaskGraph::rotations`；失败 option rollback 时对应 rotation 也必须
撤销。当前版本只记录 `RotationCandidate`，不生成 preferred ready task。
compiler failure 必须返回有限 PairCost 和 partial/empty guidance，不能判
状态不可解；operator constraint tree 仍枚举 Lift 与同步 joint Moves。

后续可实现：

```cpp
struct JointShiftBundle {
  std::vector<ShelfTask> cycle;
};
```

bundle 只有在所有 constituent shelves 已有 carriers，或 robot matching 能
同时覆盖全部 tasks 时才成为 preferred guidance。bundle 是优化项，不是
第一版正确性的前置条件。

---

## 12. 正确性与完备性

### 12.1 三种 cost 严格分离

```text
g:
  真实 apply_ops 后的 weighted physical cost

h:
  h_total = h_existing_TAPF + h_shelf_LB
  h_shelf_LB 来自 tau_LB 上的 admissible injective matching

guidance:
  PairCost, target priority, task graph, rho, switch penalty
  可非 admissible，只改 successor ordering
```

PairCost、blocker 数、rollout stall penalty、robot approach distance 和
task priority 都不得进入 `h`。

`h_existing_TAPF` 只覆盖原 TAPF allowed-task rows；carrier-only robots 的
allowed row 为空。`h_shelf_LB` 只覆盖 target-shelf delivery，因此两部分
不能重复计算同一 obligation。zero-shelf 时 `h_shelf_LB = 0`，必须逐位
退化为原 LaCAM-TAPF。shelf LB 只在 node creation 加一次；rewire rebuild、
rollout reattach 和其他 guidance refresh 都不得再次累加。

### 12.2 完备性

完整性依赖以下事实：

1. state key 只含真实物理状态；
2. terminal condition 不读取临时 assignment；
3. PairCost、tau、D、priority、rho、custody metadata 都是 ordering-only；
4. Task-BR-PIBT 失败不删除任何 primitive operator；
5. operator constraint tree 最终枚举所有 robot primitive combinations；
6. fully constrained action 由 `apply_ops()` 独立判断；
7. zero-empty/no-ready 只表示“没有 preferred guidance”，不表示无解。

所以错误 guidance 最多降低搜索速度，不能永久删除合法方案。

---

## 13. 现有代码的具体落点

### 13.1 `lacam/include/tapf_planner.hpp`

保留：

* `ShelfState`；
* physical `TAPFNode`；
* `CarrierGuidance` 作为 per-node ordering metadata；
* stats 载体。

替换：

* 删除 `DemandKey`/`ManipulationTask` 中旧 representative-root 兼容语义；
* 删除 `ObjectiveOption`、`selected_option`、`selected_packages`；
* 新增 `UpperSignature`、`ShelfSelector`、exact `ShelfTask`、
  `ShelfTaskGraph`、`PairPlan`、`StorageTransfer`、`Custody`；
* `TaskId` 明确包含 `to`；
* `Custody.task_id` 使用 exact TaskId tuple；若保留执行 index，只能是每次
  attach 从当前 `D` 重解的 optional derived field；
* `ShelfTask/Custody` 保存合法 storage endpoint、完整相邻 route 与当前
  route index；这些字段不进入 SearchKey；
* `CarrierGuidance.custody_by_robot` 使用
  `vector<optional<Custody>>`；physical kappa loaded 且 `nullopt` 明确表示
  loaded-but-unbound，不使用 `id==0` sentinel；
* `CarrierGuidance` 持久化 `rho_task_id`，本地 `rho_ready_index` 只作当前
  snapshot 的 derived view；
* 用 `SearchEdge{to,cost,transition_trace}` 替换只含 node pointer 的
  adjacency；node 保存当前不可变 `incoming_edge` handle；**shelf-free
  实例的 edge 注册与遍历顺序必须复现旧 `std::set<TAPFNode*>` 的确定性
  次序**（zero-shelf 逐位退化不仅约束 guidance 数值，也约束 rewrite
  遍历顺序；测试 #27 按此审计）；
* 普通 edge 与 macro edge 都保存一拍或多拍的 replayable primitive
  transition trace；删除独立且信息不足的 `macro_edges` authoritative map；
* 删除 `target_park`、`parking_cell`、`lift_futile`/cooldown 和专用
  `reguide` metadata/API；
* stats 改为 Pair/Task-BR-PIBT 诊断。

### 13.2 `lacam/src/carrier_guidance.hpp`

保留并复用：

* `DDDistCache` / `LowerDist`；
* Hungarian wrapper；
* occupancy scratch；
* wall-aware candidate distance；
* 可证明仍纯净的 cache helpers。

新增：

```text
make_upper_signature()
upper_vacancy_count_from_storage_slots()
ordered_storage_transfer_candidates()
compile_single_root_task_br_pibt()
pair_cost()
solve_tau_guide()
solve_tau_lb()
compile_joint_task_br_pibt()
ready_tasks()
match_ready_tasks_by_task_id()
resolve_rho_ready_indices()
advance_or_recover_storage_transfer()
```

删除 production 路径：

```text
compute_execution_prices()
ObjectiveOption generation
task_hard_claims()
resolve_objective_options()
claims pressure -> tau repair
n_vacancies == 1 compiler branch
old (shelf,from) merge semantics
ACTIVE_TARGET_CAP semantic truncation
carried/settled tau locks
parent-tau hysteresis and tau taboo retargeting
target_park / parking_cell computation
parent-path inertia / previous-path cache bias
wait-for rho taboo and revisit/no-progress reguide
futile-Lift counters and cooldown
```

`least_blocking_path()` 若保留，只能作为 Task-BR-PIBT candidate ordering
helper，不能再直接扫描路径并发射任务；其 production 接口不得接收
`prev_path`，所有等价候选按稳定 cell id 裁决。任何 cache value 必须是
`UpperSignature` 与版本参数的纯函数，cache warm-up history 不得改变结果。

显式 storage map 下，`ordered_shelf_candidate_window()` 必须改为对合法
storage endpoints 排序，并为每个 endpoint 返回 route；不能再把所有
grid neighbors 直接当作 shelf vacancy。PairCost 与 joint compiler 必须
调用同一个 transfer candidate core。

### 13.3 `lacam/src/tapf_planner.cpp`

`attach_carrier_guidance()` 改为接收紧邻 transition context，而不是只接收
一个可能相隔多拍的 parent：

```text
1. validate {previous_X, previous_guidance, executed_ops} -> current X
2. build UpperSignature
3. compare previous upper epoch
4. if upper changed:
     pair table -> tau_guide -> priority -> joint graph
   else:
     inherit those four objects
5. 按 §7.3 从真实 transition 恢复
   `optional<Custody> custody_by_robot`；强制/unassigned Lift 保持 unbound
   ；storage transfer 的中间 loaded Move 推进 route index 和下一 exact
   TaskId，endpoint 保持不变
6. compute ready tasks
7. direct-bind carried continuation
8. 用 previous `rho_task_id` repair rho，再在当前 graph 解析
   `rho_ready_index`
9. derive Carrier-PIBT robot order
10. 新 node 单独初始化 tau_LB h；reattach 不修改 h
```

删除：

* execution-price second matching；
* target-goal taboo retargeting on robot-only livelock；
* wait-for/rho taboo、revisit/no-progress `reguide` 和对应 stats；
* `target_park`/`parking_cell` loaded-carrier 分支；
* parent-path inertia；
* futile-Lift cooldown/demotion；
* Objective-PIBT stats folding；
* `preserve_tau` 的旧 task-boundary规则。

`funcPIBT()` 改为让 loaded carrier 首选当前 task 的相邻 `to`，而不是直接
持续向 terminal `tau` 移动。原 lower-deck recursion 与 primitive fallback
候选继续保留。

对位于 transit cell 的 loaded carrier，`funcPIBT()` 只能消费 bound
storage-transfer 的下一 route leg，或 transition-anchored recovery route；
不能使用“相邻 storage 优先”的无状态贪心，因为它会选择刚刚腾空的原位并
形成往复。

`carrier_rollout()` 改为每步轻量 attach，并返回 terminal guidance 与完整
transition trace；不得继续把整份 guidance 跨
`GUIDANCE_REFRESH_STEPS` 移动复用。`rewrite()` 继续设置并传播
`guidance_stale`。所有 expansion 统一先调用
`EnsureGuidanceFresh(node)`；该函数按 §10.2 递归刷新 parent chain，再沿
candidate `incoming_edge` trace 重放，绝不读取 stale parent guidance。
successor generation 必须为普通边和 macro 边统一登记 immutable
`SearchEdge`；rewrite 遍历 edge records，并从命中的 candidate edge 原子
更新 `parent + incoming_edge + g + f(g+h) + guidance_stale`。guidance
安装同步刷新 mutable preferred robot order，但只在 node 首次创建时冻结
一次 `constraint_order`，且不得再次修改 relaxation 已更新的 `g/h/f`。

### 13.4 `lacam/src/dd_planner.cpp` 与 public probes

旧 Objective probes 替换为：

```text
dd_upper_signature_probe
dd_pair_cost_probe
dd_tau_guide_probe
dd_compile_joint_graph_probe
dd_ready_tasks_probe
dd_rho_ready_probe
dd_custody_continuation_probe
```

B0/B1、两遍、修补、finalization reserve 和 deadline 分类不因 guidance
重写而另建执行路径。

### 13.5 `CMakeLists.txt`

迁移完成后删除旧结构消融：

```text
DD_OBJECTIVE_FORCE_DEFAULT
DD_OBJECTIVE_NO_INHERIT
DD_OBJECTIVE_DROP_SECOND_ROOT
```

如需新消融，只能是 compile-time research variants，且不得成为 production
fallback。建议的新消融：

```text
A. wall-distance-only PairCost vs rollout PairCost
B. independent roots vs joint Task-BR-PIBT
C. ready-only rho vs legacy all-task rho
```

### 13.6 Benchmark 与 diagnostics

删除或停止解释：

```text
obj_default_resolutions
obj_reselect_requests
obj_inherit_depth_max
obj_backtracks
obj_yields
tasks_merged   // 旧 (shelf,from) 语义
tau_price_repairs
target_parks / park_yields
rho_taboo_reguides / wait_for_cycles
futile_lift_demotions
```

新增：

```text
upper_epoch_builds
pair_cache_hits / pair_cache_misses
pair_rollout_steps
pair_rollout_truncations
pair_rollout_stalls
tau_guide_changes_on_upper_move
joint_task_nodes / joint_task_edges
joint_shared_effects
joint_effect_conflicts
joint_candidate_backtracks
joint_paused_roots
ready_task_count
rho_repairs
custody_continuations
zero_empty_no_ready
```

---

## 14. 分阶段实现顺序

实现必须遵循 `test -> RED -> implementation -> GREEN -> benchmark ->
regression test -> debug`。任何阶段都只修改现有 execution path。

### Phase 0：冻结迁移前证据

1. 保存当前 source/binary SHA 和 `results_v4_1_final7`；
2. 记录当前完整 tests 与 68-case benchmark；
3. 这些结果只作旧 Objective-PIBT baseline，不作新算法验收。
4. 在 Phase 1 修改任何 protected test 前，先按 §15 完成独立审批。

### Phase 1：UpperSignature 与 matching 分层

先写 RED tests：

* robot-only Move 后 PairCost/tau 完全不变；
* Lift/Drop 后 PairCost/tau 完全不变；
* loaded Move 后 upper signature 改变并允许 tau 改变；
* robot placement 不得翻转 tau。

实现：

* `UpperSignature`；
* `solve_tau_lb()`；
* 暂用简单 shelf-only cost 接通 `solve_tau_guide()`；
* 从 production path 移除 execution price。

### Phase 2：single-root Task-BR-PIBT PairCost

先写 recursion/backtracking/finite-failure RED tests，再实现：

* exact one-step task；
* recursive blocker displacement；
* abstract execution；
* rollout-local anonymous token；
* residual/stall；
* PairPlan cache。

### Phase 3：joint compiler 与 task graph

先写：

* shared exact effect；
* shared non-ready effect 的高优 root 向 predecessor closure 传播；
* same shelf/different destination conflict；
* blocker candidate failure 后 requester backtrack；
* multi-root root-level backtrack，且断言最终 graph 中高优 root 的 exact
  destination 确实从首选切换到备选；
* all roots included；
* one-empty generic chain；
* zero-empty finite failure。

然后用 joint graph 替换 ObjectiveOption/claims。

### Phase 4：ready-only rho 与一步 custody

先写：

* 只有 leaf 进入 rho；
* internal dependency node 不得被 robot approach；
* Lift 后 task 固定；
* Lift 后即使当前 task vector 重排，custody 仍按 exact TaskId 保持；相同
  index 指向其他 effect 时不得替换；
* constraint tree 强制 Lift 一个未 assigned/非 ready shelf 时，successor
  仍合法且 `custody_by_robot[r] == nullopt`，随后走 loaded-unbound fallback；
* constraint tree 强制 Drop/偏离 Move 时 custody 正确失效且 successor 不丢；
* loaded Move 完成一步 task并开启新 epoch；
* 同一 carrier 连续执行新 epoch 的 continuation；
* 无 continuation 则 Drop；
* task vector 重排但 TaskId 不变时 rho 不产生 switch；本地 index 相同但
  TaskId 改变时必须产生 switch；
* macro rollout 每步 attach，按 upper epoch 选择复用或重编；
* macro trace 含多次 loaded Move 时，terminal guidance 的
  priority/custody 与逐拍 fresh attach 完全一致；
* parent 与 child 同时 stale、且 child 先从 OPEN 弹出时，先递归刷新
  parent，再重锚 child，`h/constraint_order` 不变，且 relaxation 后
  `f == g + h`；
* node 先经 macro edge 建立，再由普通 edge 或另一 macro edge reparent；
  rewrite 必须从新 candidate edge 取得 trace，完全替换旧 incoming edge。

再修改 `build_guidance()` 与 `funcPIBT()`。

### Phase 5：删除旧语义

在新 production path 全绿后删除：

* ObjectiveOption resolver；
* execution price；
* one-empty branch；
* old task merge；
* `target_park` / `parking_cell`；
* parent-path inertia 和 history-dependent path cache；
* wait-for rho taboo、revisit/no-progress `reguide`；
* futile-Lift memory/cooldown；
* 旧 diagnostics/ablation flags；
* dead probes 和不再成立的注释。

最终 `git diff` 中不得保留两套 guidance pipeline。

### Phase 6：完整验证与性能优化

只有 correctness gate 全绿后才能优化 PairCost budgets、cache 容量、
candidate ordering 和可选 rotation bundle。性能优化不得改变本文语义。
完成实现与完整 benchmark 后，还必须按 `rules.md` 制作中文最终汇报网页，
用至少一个小例子解释 `U -> PairCost -> tau -> D -> ready -> rho`，面向大一
新生但保留算法、测试和 benchmark 细节；网页须交给独立 GPT-5.6 Sol
subagent review，修正其发现的问题后才能发布最终报告。

### Phase 7：storage-aware transfer 修订（2026-09-04）

本轮修订继续遵守同一流程，不修改既有 protected tests 的语义：

1. 先冻结并保留 `44bcd2f` 基线、问题 warehouse plan 和往复统计；
2. 固定 §17.1.1 的开发案例；
3. 新增独立 storage-transfer tests 并确认 RED；
4. 实现 storage vacancy、endpoint candidate、PairCost 原子 transfer 和
   跨 epoch custody；
5. GREEN 后先跑固定案例，再跑全量 C++/Python；
6. 同配置、同 seed、每例 10 秒重跑 warehouse suite 与 release benchmark；
7. 发现任何新 bug 时先新增回归并确认 RED，再修改实现；
8. 最终网页加入问题 plan 与修复 plan 的并排动画/指标，并再次由独立
   GPT-5.6 Sol review。

不得用“显式 storage map 时关闭 Task-BR”、切换旧算法或 feature flag
替代该修订。

---

## 15. Protected tests 的迁移原则

当前以下 protected tests 与本文直接冲突，后续不能由主 agent 直接修改；
必须按 `rules.md` 先让独立 GPT-5.6 Sol reviewer 阅读 `new.md`、
`design_final.md`、代码、旧 test 和 proposed change，并明确 `APPROVE`：

* `robot_placement_flips_tau_guide_goal`：新期望应为 robot-invariant；
* Objective-PIBT Phase T/R、claims、yield、pressure tests：对应机制将删除；
* TaskId 排除 `to` 的 assertions：新 TaskId 必须包含 exact destination；
* one-empty 专用分支的实现性 assertions：只保留行为，不保护 special branch；
* non-ready task pool、legacy `100/50-k/depth` 截断和 bounded priority
  slot 的**机制性** assertions；其中
  `dd_objective_priority_integration.farther_root_owns_the_frontline_slot`
  钉住的“最高优先级 mission 获得稀缺 assignment 行”是 §8.2 仍然要求的
  行为合同：reserved-slot 机制删除，但该行为断言必须迁移为 ready-only
  rho 的 priority-cutoff 测试（见 §16 #39），不得随机制一并丢弃；
* execution-price diagnostics/ablation tests；
* `dd_tau.hysteresis_is_tie_break_only` 与
  `dd_tau.carried_target_keeps_inflight_goal_commitment`（parent-lock
  合同）、tau-taboo tests：guide matching 不再读取
  parent/grounded/carried/taboo，机制合同被 §5.1 取代；
* `dd_tau.settled_pool_goal_preempts_conflicting_carried_commitment` 与
  `dd_tau.settled_pool_goal_reopens_when_matching_requires_it`：这两个
  测试保护的核心行为——eligible/injective matching 的全局可行性、以及
  “已 settled 的 target 仍是可逆 blocker、matching 需要时可以重开”——
  与 §2.2 terminal 语义和 §5.1 的 moved-away-eligible tie 完全一致，
  **行为合同保留**；迁移只替换其对 settled/carried lock 机制的实现性
  依赖（断言经由新 lexicographic matching 复现同一行为）；
* `dd_integration.rollout_steps_match_fresh_generation`：删除旧 8-step
  frozen-guidance contract，改为每步 lightweight attach；
* rewire tests：保留 stale/re-anchor correctness，但 expected guidance
  改为先刷新 stale parent chain，再从新 parent 的逐拍 transition trace
  重建 upper epoch、priority、custody 和 rho；
* `tests/test_dd_tasks.cpp::serve_task_carries_shelf_root_and_projection`：
  删除 `start -> terminal goal` 的 non-adjacent serve effect；新期望是 root
  的相邻 exact effect 与其后 dependency；
* `tests/test_dd_tasks.cpp::feasible_clear_head_gets_compiler_chosen_drop`：
  删除 blocker 跨多格直达 drop cell 的 clear effect；新期望是相邻
  predecessor task；
* `tests/test_dd_tasks.cpp::unstartable_head_skips_drop_hint`：不再允许
  `to == -1`；没有相邻 exact destination 的 shelf 只能通过递归先移动
  blocker，或让该 root 在有限预算后 paused；
* `tests/test_dd_tasks.cpp::custody_keeps_task_id_from_lift_through_drop`：
  exact TaskId 只跨 Lift 与 loaded Wait 保持，到相邻 loaded Move 即完成；
  后续 continuation 使用新 TaskId，Drop 则清除 custody；任何执行 index
  都只能从当前 graph 重解；
* `tests/test_dd_tasks.cpp::rho_binds_task_and_requests_follow`：保留“robot
  binding 可解析为当前 pickup request”的行为，但跨 snapshot identity 改为
  exact `rho_task_id`；本地 task/request index 只能由当前 graph 派生；
* `tests/test_dd_reguide_custody.cpp::committed_inflight_survives_livelock_reguide`：
  删除 livelock-reguide
  复制任意旧 custody 的语义；改测真实 same-U transition anchor 在 loaded
  Wait 后保留仍有效的 exact custody；
* `tests/test_dd_park_purity.cpp::default_lazy_policy_is_epoch_dependent_documented`
  及其他 park probes：
  删除 history-dependent park contract；改测相同 `U` 在不同 cache warm-up
  history 下产生相同 PairCost/tau/D，且 production guidance 不再输出 park；
* `tests/test_dd_oscillation.cpp::path_inertia_breaks_ties_toward_prev` 与
  `path_inertia_never_beats_real_cost`：删除 previous-path bias；改测
  U-only stable cell tie，传入不同 ancestry 不改变 shelf-side path；
* `tests/test_dd_oscillation.cpp::idle_escapes_active_path` 与
  `idle_off_path_keeps_wait_first`：保留 idle robot 避让/等待的行为目标，
  但 active footprint 必须来自当前 ready/custody effects，不再来自历史
  least-blocking path 或 target park；
* `tests/test_dd_oscillation.cpp::futile_lift_memory_triggers_and_expires_automatically`：
  删除全局 cooldown
  contract；改测 ready/valid Lift 不因历史 episode 被降级；
* `tests/test_dd_waitfor.cpp`、`tests/test_dd_reguide_stats.cpp` 与
  `dd_g1_conformance.revisit_reguide_preserves_enumeration`：删除
  wait-for taboo/reguide API 与 stats；保留并强化“有无 guidance 时合法
  physical successor 集合相同”的 completeness 测试。
* `tests/test_dd_g1.cpp` 的 brute-force successor oracle：继续保护 forced
  Lift/Move/Drop；新增期望是强制 Lift 非 assigned/ready shelf 后 physical
  successor 仍存在、对应 `custody_by_robot` 为 `nullopt`，不能因 guidance
  缺少 task 而删掉该 successor。

不冲突、应继续保护的测试包括：

* search key 与 physical state；
* eligible-goal terminal semantics；
* `apply_ops()` rule table；
* operator-tree successor completeness；
* zero-shelf 原 LaCAM-TAPF 逐位退化；
* admissible h 不超过真实 cost；
* plan repair legality；
* C++/Python replay；
* strict `deliverable_ms` / solver-return deadline。

---

## 16. 必须新增或替换的测试

| # | Requirement |
|---:|---|
| 1 | 同一 `U`、不同 free robot positions：PairCost matrix 和 `tau_guide` 逐位相同 |
| 2 | Lift/Drop 不改变 `UpperSignature`、PairCost、tau、priority、D |
| 3 | loaded Move 改变 `UpperSignature`，重新计算 PairCost/tau/D |
| 4 | `tau_guide` 使用 PairCost；`tau_LB` 与 PairCost 改动完全隔离 |
| 5 | 所有 ShelfTask 都是相邻 effect，且 TaskId 包含 `(shelf,from,to)` |
| 6 | single-root recursion 在 blocker 失败后尝试下一 candidate |
| 7 | one-empty 不检查 empty 数量也产生完整 dependency chain |
| 8 | 只有 empty-adjacent zero-indegree leaf 是 ready |
| 9 | 两个 roots 请求相同 exact effect 时合并 roots 与 max priority |
| 10 | 高优 root 后合并进 shared non-ready effect 时，高优 demand 传播到最深 predecessor leaf |
| 11 | 同一 shelf/from 的不同 `to` 是冲突，不能合并 |
| 12 | 不同 shelves 抢同一 destination 时触发 joint backtracking |
| 13 | target blocker 在固定 tau 下先尝试替代 displacement，并参考自己的 tau |
| 14 | 高优 root 的首选妨碍低优 root 时，最终 graph 中该 root 的 exact `(from,to)` 确实切换到备选，而不是只证明两个 roots 都被编译 |
| 15 | joint compiler 接收全部 unfinished roots，不受 ACTIVE_TARGET_CAP 静默截断 |
| 16 | non-ready internal tasks 永不进入 rho |
| 17 | carried ready continuation 直接绑定当前 carrier，不参加 Hungarian |
| 18 | 同一 carrier 可连续执行多个 one-step tasks，不强制重复 Lift/Drop |
| 19 | 正常 preferred path 中，in-flight task 不被 rho/priority 改写；强制 Drop/偏离 Move 仍在 successor set 且使 custody 正确失效 |
| 20 | 同一 anonymous shelf 连续移动两步只计一次 Lift/Drop episode；匿名输入排列不改变 PairCost |
| 21 | zero-empty 无 ready 时返回有限 PairCost/empty guidance，不判无解 |
| 22 | `|G_b|=1` 自然退化为 fixed-goal Carrier-LaCAM |
| 23 | `tau_guide` 改变不改变 admissible h；`h_total=h_existing_TAPF+h_shelf_LB` 且不超过独立 oracle |
| 24 | macro rollout 跨至少两次 loaded Move；每步更新 custody/ready/rho，并返回与逐拍 fresh attach 相同的 terminal priority/custody anchor |
| 25 | parent 与 child 同时 stale 且 child 先出 OPEN：relaxation 先令 `f=g+h` 并按现有策略更新 OPEN，再递归刷新 parent、从 candidate incoming edge trace 重锚 child；mutable preferred order 更新，guidance refresh 不再修改 g/h/f 或 frozen constraint_order |
| 26 | guidance failure 前后，operator-tree 枚举的合法 physical successors 集合不变 |
| 27 | mixed TAPF/carrier 的两个 h 分量正确相加；zero-shelf 原 LaCAM-TAPF 逐位一致 |
| 28 | 所有返回计划通过 `apply_ops()`、C++ replay 和 Python validator |
| 29 | strict deadline 包括 search、tree cleanup、repair、SOC 和最终 replay |
| 30 | production guidance 无 `target_park/parking_cell`；无 continuation 的 loaded carrier 首选原地 Drop，idle 避让只读当前 D/ready/custody |
| 31 | 相同 U 与相同 target-priority input、但 parent path/cache warm-up history 不同：shelf candidate order、PairCost、tau 和 D 完全相同 |
| 32 | 给定相同 X、D、ready 与上一拍 TaskId bindings，改变旧 revisit/no-progress/wait-for counters 不得改变 rho；production 不再有 taboo/reguide 输入 |
| 33 | 当前 ready 且物理合法的 Lift 在任意既往 Lift/Drop history 下保持相同首选次序，不存在全局 cooldown |
| 34 | custody 只能由真实 transition anchor 建立或延续：assigned-ready Lift（上一拍 rho 绑定确定性转入）、loaded Wait（保留仍有效的 exact custody）、loaded Move 开启的新 epoch（仅经 BindReadyContinuations 绑定 exact continuation）；无 transition 的重复 attach/手工旧 guidance 不能注入 custody |
| 35 | task vector 重排而 exact TaskId 不变时不计 rho switch；相同本地 index 指向不同 TaskId 时必须计 switch，执行 index 每次从当前 graph 重解 |
| 36 | node 先由 macro edge 建立，再由普通或另一 macro edge reparent：rewrite 选中的 edge record 完全替换旧 incoming trace，原子更新 g/f，逐拍 replay 终态等于 node.X |
| 37 | Lift 后重排 task vector，使原 index 指向不同 effect：loaded Wait 后 custody exact TaskId 仍等于原 `(shelf,from,to)`，derived index 重新解析或为 nullopt |
| 38 | constraint tree 强制 Lift 未 assigned/非 ready shelf：physical successor 仍在 exhaustive set，robot 为 loaded 且 `custody_by_robot[r]=nullopt`，Carrier-PIBT 走 unbound Drop/fallback |
| 39 | `|ready| > |free|` 时的 §8.2 词典序合同：高于 priority cutoff 的 ready task 必获 assignment 行（继承旧 `farther_root_owns_the_frontline_slot` 的行为保护）；cutoff 同级由 approach distance 决定；TaskId switch penalty 只在更后层生效且不得推翻 priority/distance |
| 40 | 显式 storage map 下 `upper_vacancy_count == storage_cells - shelves`，任意数量空走廊不改变 vacancy |
| 41 | blocker 的 candidate endpoint 全部满足 `can_store_shelf(endpoint)`；route 内部允许 transit，但逐格相邻且不穿过中间 storage |
| 42 | PairCost 对一条跨 aisle transfer 在下一次 compiler 调用前执行到 endpoint；整条 route 只形成一个 Lift/Drop episode，move cost 按 leg 计 |
| 43 | loaded Move 进入 transit 后，即使新 tau/D 不再包含该 blocker，custody 仍保持原 endpoint 并推进下一 route leg；不得立即 reverse |
| 44 | storage endpoint 被占用时，递归 predecessor 把 blocker 搬到另一个合法 endpoint；zero vacancy 返回有限 paused/no-ready，one vacancy 形成通用 chain |
| 45 | 两个 carriers 的 endpoint/first-leg 冲突不会产生两个同时接受的 incompatible transfers；暂时 route 冲突通过 Wait/PIBT 处理，任何 plan 均无 corridor Drop |
| 46 | forced deviation 后，recovery custody 只能由真实 transition anchor 建立；无可达空 storage 时保持 loaded/Wait |
| 47 | 无 storage map 时每条 transfer route 恰为 `[from,to]`，PairCost、TaskId、successor 集合和原测试逐位兼容 |

---

## 17. Benchmark 与 release gate

### 17.1 固定开发子集

继续使用已经冻结的 6 个 case，不因表现更换：

1. `brap_h4w10_a5_e1_R1_seed0`；
2. `brap_h6w10_a6_e1_R1_seed0`；
3. `brap_h10w10_a12_e3_R1_seed1`；
4. `brap_h10w10_a12_e8_R1_seed0`；
5. `brap_h10w10_a12_e3_B_seed0_pool`；
6. `brap_h8w10_a10_e2_R1_seed0`。

同一 dataset、seed、unit weights、following、strict 10s 和资源分配用于
旧 baseline 与新算法。

### 17.1.1 storage 修订固定案例

实现前冻结以下案例，不因结果更换：

1. 真实问题例：
   `warehouse_blocks_h20w20_b3_a1_d75_r8_t12_seed0`，重点检查原 plan 中
   `b6 (3,17) -> (3,16) -> (3,17)` 的重复 episode；
2. 单 target 跨 aisle 到合法 storage goal；
3. anonymous blocker 必须跨 aisle rehome 后 root 才能进入；
4. endpoint 被占用且只有一个 storage vacancy 的递归 chain；
5. storage slots 全满、只有空 aisle 的 zero-vacancy finite failure；
6. 两个 carriers 在窄 aisle 中请求冲突 transfer；
7. forced mid-route deviation 后的 recovery；
8. 无 `storage_map` 的 legacy adjacent-task case。

以上小案例先用于 RED/GREEN；真实 warehouse case 使用同一 YAML、seed=0、
unit weights、following allowed 和 10 秒上限。

### 17.2 迁移前 baseline

当前仓库 artifact 记录：

```text
results_v3_strict_return_final : 33/68
results_v4_1_final7            : 36/68
```

`results_v4_1_final7` 属于旧 Objective-PIBT；它只定义性能比较起点。

### 17.3 Release gate

新 Task-BR-PIBT production implementation 至少满足：

1. 全部 correctness tests 和 replay tests 通过；
2. 成功数验收是**单一可判定规则：配对不回退 + 绝对下限**。已测得同一
   binary 的贴线重例随机器负载在 33-36/68 间波动（2026-09-02 复测
   33-34/68），因此固定绝对线不可判定公平。正式规则：
   - 候选与 `results_v4_1_final7` 的旧 binary 在同一会话、同机、同
     jobs 并行度下配对重跑；
   - 候选 success（全集与 small 子集）均 `>=` 配对基线；
   - 绝对下限：全集 `>= 34/68`、small `>= 34/36`；
   - 若配对基线达到 36/68（或 small 36/36），则候选下限随之提升为
     36/68（或 36/36）。
   历史 nominal target 36/68、small 36/36 仅是记录在案的期望值，不是
   独立于配对基线的第二判据；
3. 每个成功 case 的 `deliverable_ms <= 10000` 且
   `solver_runtime_ms <= 10000`；
4. carrier 子进程 wall timeout 仍为 10s；
5. common-success set 报告 makespan/SOC 几何比及逐例改善/持平/恶化；
6. 报告 PairCost cache、rollout truncation、joint backtracking、
   paused roots、ready count 和 rho repair；
7. benchmark expected behavior 如需变更，必须在变更代码或 test 前经独立
   reviewer 批准，不能事后降低门槛；
8. 按 `rules.md` 生成带小例子的中文汇报网页，并由独立 GPT-5.6 Sol
   subagent review 网页内容、链接、数据和可读性；review 未通过不得发布。

2026-09-03 的最终同机配对运行满足以上 gate：

```text
protocol:
  68 cases, jobs=14, timeout=10s, seed=0
  alpha=beta=gamma=delta=1, following=allowed

paired baseline:
  binary sha256 1c32ba3e21136fe902c7d8ef0dbfdf13360b2d512e2668d409778f324830b097
  36 / 68 solved, wall 33.8s
  frozen small subset 36 / 36, wall 11.3s

Task-BR candidate:
  binary sha256 f7127998c198aa0cbb698e92fec0bea9e5a673314c701e3113164994efc4fc17
  38 / 68 solved, wall 29.2s
  frozen small subset 36 / 36, wall 7.1s
```

baseline 的 36 个成功例全部仍由 candidate 解出；candidate 另外解出
`brap_h20w20_a40_e100_R1_seed0/seed1`。common-success 36 例上，
candidate/baseline 的 makespan 几何均值比为 `0.4616`（33 改善、1 持平、
2 恶化）；weighted SOC 几何均值比为 `0.4820`（32 改善、3 恶化、1 个
`0/0` 平凡实例持平且不进入几何均值）。candidate paired-full 成功实例
最大 `deliverable_ms=6834.72`、最大 `solver_runtime_ms=6834.75`，均小于
10s。相对更早的 `results_v3_strict_return_final`，candidate 解出
`38 vs 33`；common-33 的 makespan 几何比为 `0.5206`（30 改善、1 持平、
2 恶化）；SOC 几何比为 `0.5343`（29 改善、3 恶化、1 个 `0/0` 持平且
不进入几何均值）。原始结果见
`benchmark/results_task_br_release_*_20260903/`。

---

## 18. 已解决的设计选择与开放工程项

### 18.1 已解决

* terminal assignment 只由 shelf layout 和 shelf-side PairCost 决定；
* robot 只参与 ready-task assignment 与执行；
* task 是相邻 shelf effect；
* TaskId 包含 destination；
* blocker chain 由通用 recursion 产生；
* one-empty 无特殊算法；
* all roots 联合编译；
* non-ready tasks 不进入 rho；
* loaded Move 后立即开启新 upper epoch；
* PairCost 与 admissible h 完全分离；
* ObjectiveOption/claims/execution-price 不属于最终算法。

### 18.2 已冻结参数与后续工程项

本次 release 已冻结 Pair rollout 与 joint compiler 的 §6.4 预算、
UpperEpochCache 的 256-entry LRU、exact lexicographic Hungarian，以及
density-aware commitment/continuation 规则。后续可研究：

1. zero-empty `JointShiftBundle`；
2. 更精细但仍严格 shelf-only 的 PairCost episode estimate；
3. 不改变 successor completeness 的更低开销 compiler/cache 实现；
4. 对两个已知质量回退实例的通用排序改进。

任何后续调整都必须重新走 protected-test 审批与同机配对 benchmark；不得
改变状态、终点、合法 successor 集合或三层边界。

---

## 19. 总伪代码

```text
AttachCarrierGuidance(X, transition = NONE):
    U = UpperProjection(X)

    if transition != NONE:
        assert apply_ops(
                 transition.previous_X,
                 transition.executed_joint_ops) == X
        previous_G = transition.previous_guidance
    else:
        previous_G = NONE

    if previous_G == NONE or U != previous_G.upper_signature:
        pair_table = PairCache.get_or_compute(U)
        tau_guide = HungarianInjectiveLex(
                        pair_table,
                        moved_away_eligible_count,
                        stable_assignment_vector)
        priority =
          previous_G == NONE
            ? InitialRootPriority(U, tau_guide)
            : UpdatePriorityOnUpperEpoch(
                  previous_G.priority,
                  U,
                  tau_guide)
        D = CompileJointTaskBRPIBT(
                U,
                tau_guide,
                priority)
    else:
        pair_table = previous_G.pair_table
        tau_guide = previous_G.tau_guide
        priority = previous_G.priority
        D = previous_G.task_graph

    // phase 1 of 3: transition-derived only — preserve valid custody
    // (loaded Wait), complete/invalidate (loaded Move/Drop/deviation),
    // and transfer the previous rho binding into custody on an
    // assigned-ready Lift; NEVER binds a loaded-Move continuation
    custody_by_robot = RecoverCustodyBindingsFromActualTransition(
                           transition,
                           X,
                           D)
    // phase 2 of 3: physical ready predicate; a shelf held by the
    // carrier that just completed its predecessor stays visible as a
    // continuation candidate
    ready = ReadyTasks(D, X, custody_by_robot)

    // phase 3 of 3: the ONLY site that establishes new continuation
    // custody, restricted to carriers whose last step was the loaded
    // Move that opened this upper epoch. Roomy layouts suppress only
    // automatic binding of the immediate reverse; dense layouts allow it.
    BindReadyContinuationsToCurrentCarriers(
        ready,
        custody_by_robot,
        only_after_loaded_move_epoch = true,
        allow_immediate_reverse = TargetDense(U))

    previous_rho_task_id =
      previous_G == NONE ? NONE : previous_G.rho_task_id

    rho_task_id = MatchFreeRobotsToGroundedReadyTasks(
                      X.free_robots,
                      ready.remaining_grounded,
                      previous_rho_task_id)

    rho_ready_index = ResolveTaskIdsInCurrentGraph(
                          rho_task_id,
                          ready,
                          D)

    return CarrierGuidance(
        U,
        pair_table,
        tau_guide,
        priority,
        D,
        ready,
        rho_task_id,
        rho_ready_index,
        custody_by_robot)
```

搜索节点只在创建时独立执行一次：

```text
InitializeNodeHeuristic(node):
    node.h = ExistingTAPFHeuristic(node.X) + SolveTauLB(node.X)
    node.f = node.g + node.h
```

rollout 中间 anchor、`EnsureGuidanceFresh()` 和普通 reattach 均不调用
`InitializeNodeHeuristic()`。

```text
GeneratePreferredSuccessor(node, operator_constraint):
    EnsureGuidanceFresh(node)
    honor all fixed primitive operators in the constraint

    order robots by:
        in-flight task priority
        assigned ready-task priority
        carrier/free phase
        stable robot tie

    for each unconstrained robot:
        free assigned robot ->
            resolve current rho_ready_index,
            verify its exact TaskId equals rho_task_id,
            then approach/Lift candidates
        loaded robot with custody_by_robot[r] ->
            one-step loaded Move candidate
        loaded robot with nullopt custody ->
            Drop at current cell first, then ordinary completeness fallbacks
        idle robot -> vacate current ready/custody effect cells or Wait

        use Carrier-PIBT recursion for lower-deck blockers
        do not consult park, taboo/reguide, previous shelf path,
        or futile-Lift cooldown

    return apply_ops(node.X, joint_operator)
```

最终结构为：

```text
U
  -> single-root Task-BR-PIBT PairCost
  -> injective tau_guide
  -> joint Task-BR-PIBT dependency graph
  -> ready one-step shelf tasks
  -> rho
  -> Carrier-PIBT
  -> apply_ops
  -> X'
```

这就是后续实现、测试、benchmark 和 code review 的唯一目标语义。

---

## 20. 实现与验证闭环

本节记录 2026-09-03、提交 `44bcd2f` 之前冻结的 Task-BR release 基线；
§21 的 storage-map 修订尚未包含在下述数字中。该基线实现没有增加第二套
planner。`TAPFPlanner::solve()`、LaCAM
OPEN/CLOSED、operator constraint tree、`funcPIBT()`、`apply_ops()`、两遍
求解、repair、strict deadline 与 final replay 均沿用原执行路径；变化集中在
原 `attach_carrier_guidance()` 及其直接消费的数据：

```text
UpperSignature + 256-entry LRU
  -> PairPlan/PairCost + exact tau_guide
  -> joint Task-BR compiler + ReadyTasks
  -> exact TaskId rho/custody
  -> Carrier-PIBT preferred joint action
  -> unchanged apply_ops physical transition
```

实现中还包括：`tau_LB` 与 guidance matching 隔离；undo-log joint
transaction；shared-effect roots 反向传播；transition-anchored custody；
roomy/dense continuation policy；priority commitment；macro edge trace 的
原子重锚；strict return deadline 内的 repair、cleanup 与 replay；以及
PairCost 四邻接固定排序优化。旧 ObjectiveOption、execution-price、
parking/taboo/reguide/cooldown、one-empty 专用 compiler 和 parallel
guidance path 已从 production 移除。

最终验证：

```text
./build/test_all --gtest_color=no
  222 / 222 PASS, 168.952s

PYTHONPATH=benchmark python3 -m unittest discover \
  -s benchmark/tests -p 'test_*.py' -v
  80 / 80 PASS, 42.876s
```

新增回归覆盖 Pair kernel、exact matching、joint compiler、ready/rho、
custody、dense tail、deadline/finalization、macro rewire、physical successor
completeness、priority commitment、aggregate-first candidate ordering、
one-vacancy completion tie、two-vacancy full-root progress、roomy reverse
suppression 与 dense reverse binding。zero-shelf LaCAM-TAPF compatibility
tests 同在 222 项全量 suite 中通过。

最终 benchmark 数据与 §17.3 一致。质量上不是逐例单调：例如
`h10w10_a1_e1_B_seed0_pool` 的 makespan `104 -> 130`、SOC `174 -> 182`；
`h6w10_a6_e1_B_seed1_pool` 的 makespan `240 -> 258`；另有两个共同成功例
只在 SOC 上轻微回退。这些回退已如实保留在报告中，没有通过替换
testcase、放宽 timeout 或修改 success semantics 隐藏。总体成功集合扩展
2 例，且 common-set 两个主指标相对 paired final7 与 v3 历史基线均显著
改善。

---

## 21. Storage-map 修订的最终增量设计

### 21.1 问题与不变量

当前错误路径把相邻空 transit cell 当成一次 blocker displacement 的完整
destination。blocker 进入 aisle 后，loaded Move 开启新 upper epoch，旧
one-step custody 完成；新 graph 不再需要该 blocker，于是 carrier 变成
unbound。执行器无法 Drop，只能贪心寻找相邻 storage，最容易选择刚腾空的
原位，从而反复：

```text
storage A -> transit C -> storage A
```

修订后必须同时保持：

1. `ShelfTask/TaskId` 仍是相邻 physical effect；
2. 每次开始进入 transit 前，已经选择一个合法 storage endpoint；
3. route 内部可以是 transit，endpoint 必须可 Drop；
4. 在同一选定 transition chain 内，endpoint 在 transfer 完成前不因
   tau、D 或 cache 改变；duplicate reparent 则从新 incoming trace
   原子替换整份 custody；
5. PairCost 在抽象层一次执行完整 transfer；
6. 真实层每拍仍只执行一个 primitive loaded Move；
7. 失败 guidance 不删除 operator-tree successor；
8. legacy 无 storage map 时逐位退化。

### 21.2 数据与 identity

```cpp
struct StorageTransfer {
  int endpoint = -1;
  std::vector<int> route;
};

struct ShelfTask {
  TaskId id;                    // shelf, route[0], route[1]
  std::vector<RootDemand> roots;
  int priority = 0;
  StorageTransfer transfer;
};

struct Custody {
  TaskId task_id;               // current route leg
  ...
  StorageTransfer transfer;
  size_t transfer_index = 0;    // current cell = route[index]
};
```

`TaskId` 不加入 endpoint，避免把同一个当前 physical effect 拆成不同
successors。若多个 root 合并到同一 exact effect 而给出不同 endpoints，
保留先被高优先级、稳定 root 顺序接受的 transfer；真实 move 后下一 epoch
重新评估其他 roots。`StorageTransfer` 是 custody continuity metadata，
不进入 SearchKey、goal、`h` 或 physical legality。

### 21.3 Candidate core

`ordered_storage_transfer_candidates()` 是 PairCost 和 joint compiler 共用
的唯一实现：

```text
legacy map:
    each adjacent traversable cell
    -> route [from,to], endpoint=to

explicit storage map:
    BFS from current cell
    traverse only non-storage cells after leaving from
    stop when a storage cell is first reached on a branch
    keep deterministic shortest route per endpoint
    discard routes whose internal cells contain shelves
    score endpoints with the existing root/blocker rules
    return best at most four
```

上述两段不是两套 planner。统一规则始终是“离开 `from` 后，在每条分支遇到
首个 storage 就停止”。因此当 `from` 的全部 traversable 邻格都是 storage
时，通用 BFS 的结果严格等于四邻格集合；实现必须在同一个 candidate core
中用固定大小数组直接评分，避免在 PairCost 热路径为每次调用分配整张网格的
`parent`、queue、map 和 route vector。相邻候选在内部可以只保存 endpoint
和隐含的第一步，直到任务被接受时才物化 `[from, endpoint]`。只要存在任一
transit 邻格，就使用通用 BFS。该局部特化只消除等价计算和临时分配，不改变
候选、score tuple、稳定 tie-break、TaskId、PairCost 或最终 transfer route，
也不得检测实例名、是否存在 `storage_map` 或是否包含 pick/place。直接相邻
候选由唯一的邻格枚举建立合法性，resolver 不再逐候选重复调用
`grid.neighbors()`；非相邻 BFS route 仍在进入 compiler transaction 前验证
逐格相邻、内部 transit 为空且 endpoint 可存储。直接 route 满足
`endpoint == first_step`，对应的 destination reservation 已经完整表达
endpoint claim，不再写第二份 undo entry；endpoint conflict 查询必须同时
识别这种 storage destination。非相邻 route 的 first step 是 transit，
所以继续使用独立 endpoint reservation。每个 cell 的
“全部 traversable 邻格都是 storage”标记由 `DDInstance::finalize()` 从
immutable grid/storage mask 预计算；candidate core 只读该局部标记，不以
实例类型或是否存在 `storage_map` 选择 execution path。
候选的内部 route storage 必须按需分配：直接候选只携带
`endpoint/first_step/route_size`，非相邻候选才拥有 route buffer；写入
`ShelfTask` 或 custody 时再物化规范化完整 route。

endpoint 被占用时递归搬动 occupant；endpoint 为空时建立当前 task。compiler
transactionally 预约当前 first leg 和 endpoint，但不把 transit route
interior 当成永久空间 reservation：依赖链中的 predecessor 与 requester
可以按顺序复用同一通道。真实 active transfers 的同时冲突由 epoch 外的
claims 过滤。zero storage vacancy 允许形成 rotation candidate 或 paused
root，但不能把空 aisle 误报为 vacancy。

### 21.4 PairCost

single-root core 仍返回最高优先 ready `ShelfTask`。PairCost 消费它时：

```text
remaining = step_cap - rollout_steps
if transfer legs > remaining:
    truncated = true
    stop before entering route
else:
    for every leg:
        charge alpha (+ delta for anonymous)
        move the same abstract token
        rollout_steps += 1
```

整条 route 共享同一 open episode，因此不会在 aisle 产生虚构 Drop/Lift。
完成 endpoint 后才重新调用 compiler。

### 21.5 真实 custody

`recover_task_br_custody()` 在 exact loaded Move 后分三类：

1. 普通一格 transfer 已到 endpoint：旧 custody 完成，交给普通 graph
   continuation；
2. storage transfer 尚有 route：从旧 custody 和真实 op 确定性推进 index，
   立即建立下一 exact leg custody；
3. forced deviation：旧 transfer 失效；若当前 cell 是 transit，确定性寻找
   可达空 storage endpoint 并建立 recovery custody。

第 2 类优先于 `BindReadyContinuations`，后者不能覆盖它。第 3 类若无可达
endpoint，保持 loaded/unbound；`funcPIBT()` 首选 Wait 和合法 Move，不加入
DROP。到达 endpoint 后才首选 Drop。

active custody 只在非缓存 guidance 层生成 `ActiveTransferClaims`。恢复顺序
必须是：

1. 保留所有由真实 transition 验证的旧 custody；
2. 收集 forced-deviation loaded carriers，按
   `(previous priority desc, robot id asc)` 联合分配 recovery；
3. recovery endpoint 不得与已接受 claim 重复；其 remaining route 不得与
   更高优先 claim 在窄 transit footprint 上冲突；
4. 用最终 claims 过滤与之冲突的 grounded ready tasks，再做 rho。

claims 不进入 `UpperEpochCache`、SearchKey、PairCost 或 tau。相同 `U` 在
不同 active custody/cache warm-up 下必须得到逐位相同的 cached graph；
只有 cache 外的 custody/ready/rho 可以不同。过滤只是 preferred ordering，
operator constraint tree 仍枚举全部合法 successor。

active custody 重绑 graph index 时必须同时匹配 exact `TaskId`、endpoint 和
旧 route 从 `transfer_index` 开始的 remaining suffix。仅第一腿相同但
endpoint/suffix 不同的 graph task 不得覆盖 roots、priority 或 endpoint，
derived index 留空。anonymous shelf 每推进一腿把 selector value 重锚到
当前 `from`；真实 identity 由 carrier 和 transition anchor 保证。

### 21.6 冲突与特殊情况

* **一个 vacancy**：递归把 occupied endpoint 的 shelf transfer 到下一个
  endpoint，空 storage slot 沿 chain 移动；不写专用 one-empty 分支。
* **zero vacancy**：有限 PairCost penalty、paused/no-ready 或 rotation
  record；不能开始没有合法 endpoint 的 aisle transfer。
* **多个 carriers**：同一 graph 内 endpoint 与 first leg 冲突通过
  transaction backtracking。完整未来 route 具有时间语义：dependency
  predecessor 与 requester 可以先后复用 transit cell，不能被静态永久
  reservation 拒绝；实际同时启动由 cache 外 active/ready claims 串行化。
  下一格被占用时 Wait，最终仍由 `apply_ops()` 裁决。
* **tau 改变**：可以改变未开始的 mission guidance，不能重写 active
  transfer endpoint。
* **goal 就在 route 中**：只有它是 route endpoint 且为 storage 才算到达；
  transit cell 永不因坐标接近 goal 而结束。
* **forced Drop**：`apply_ops()` 继续拒绝 corridor Drop，completeness tree
  不被 guidance 伪造。
* **forced Move 偏离**：从真实 transition recovery；不能沿用旧 route index。
* **anonymous identity**：route 内继续使用 rollout-local token；真实 custody
  用当前 carried shelf 与旧 transition anchor，不创建永久 anonymous label。
* **cache**：PairCost candidate 是 `(UpperSignature, storage_map,
  compiler-version)` 的纯函数。storage map 属于 instance immutable data，
  active transfer 不写入 PairCost key；只有 custody execution读取 route。

### 21.7 代码落点与验证证据

| 机制 | 修改点 |
|---|---|
| storage vacancy | `upper_vacancy_count()` |
| route/endpoint 数据 | `tapf_planner.hpp::{StorageTransfer,ShelfTask,Custody}` |
| shared candidate core | `carrier_guidance.hpp::ordered_shelf_candidate_window()` 及 single/joint contexts |
| endpoint/route transaction | `TaskBRCompilerState/Transaction` |
| PairCost full transfer | `pair_cost_prefix_lower_bound()`、`pair_cost()` |
| cross-epoch commitment | `recover_task_br_custody()` |
| recovery/active claims | `recover_task_br_custody()` 与 `ready_tasks_with_custody()` 的非缓存阶段 |
| preferred execution | `tapf_planner.cpp::funcPIBT()` carrier branch |
| zero-vacancy diagnostic | `tapf_planner.cpp::attach_carrier_guidance()`；必须复用 `upper_vacancy_count()`，不能把 transit 算作空位 |
| probes/tests | `test_dd_storage_transfer.cpp` 与独立新增的 claims/temporal regressions，不修改既有 protected assertions |

完成证据必须包括：新测试先 RED 后 GREEN；C++/Python 全量；固定 8 个
storage cases；真实 warehouse case 不再出现同一 shelf 的
`storage -> transit -> same storage` episode；所有 Drop 均在 storage；
同机 10 秒 release benchmark；最终 diff 无 parallel planner、feature flag
或旧算法 fallback；中文网页和独立 subagent review。

---

## 22. Storage-map 修订的实际验证证据（2026-09-04）

本节记录 §21 的实现结果；§20 的数字是 storage-map 修订前的历史
release 证据，不能拿来替代本节。

### 22.1 语义与回归

* 新增 `test_dd_storage_transfer.cpp` 9 项与
  `test_dd_storage_transfer_claims.cpp` 5 项，覆盖合法 storage endpoint、
  跨 epoch custody、forced deviation、active claims、endpoint/suffix
  rebinding、时序通道复用、zero/one-vacancy 与 corridor Drop 禁止。
  独立代码审查发现诊断口径遗漏后，又按 RED→GREEN 新增
  `test_dd_storage_transfer_stats.cpp` 1 项，锁定 forced-transit 状态也必须
  按 storage vacancy 统计。
* C++ 全量：`241/241`；Python 全量：`99/99`。
* 固定 development profile：9 例中 6 例按权威 validator 到达 goal，3 个
  既有 hard cases 在同一严格 deadline 下仍按预期 timeout。
* 9 个 20×20 warehouse cases 全部求解并通过权威重放。逐帧审计结果均为
  `corridor_drops=0`、`same_origin_returns=0`、
  `loaded_reversals=0`。用户报告的
  `b3/a1/d75/r8/t12/seed0` makespan 为 22，运行时间 91.2382 ms。

“延后一个请求”只适用于尚未开始、且与更高优先级请求争抢同一 endpoint
或窄通道 footprint 的 root。正常沿已验证 route 执行时，已经 Lift 并进入
storage transfer 的货架由 active custody 持续推进；其他未启动请求不能
暂停它或改写它的 endpoint。若真实 successor 强制偏离 route，旧 transfer
按设计失效，并从真实 transition anchor 建立 recovery；这时可以选择新的
合法 storage endpoint。

### 22.2 同机 10 秒配对 benchmark

固定数据集、seed、14 jobs 与每例 10 秒：

| 指标 | 修订前 baseline | storage 修订最终版 |
|---|---:|---:|
| solved | 36/68 | 38/68 |
| wall time | 33.9 s | 29.4 s |
| solver time sum | 393.4 s | 335.3 s |
| common-set makespan sum | 37387 | 15144 |
| common-set weighted SOC sum | 62938 | 29853 |

最终版新增求解
`brap_h20w20_a40_e100_R1_seed0/seed1`，没有丢失 baseline success。
36 个共同成功例中，makespan 为 better/equal/worse `33/1/2`，几何比
`0.460862`；weighted SOC 为 `32/1/3`，几何比 `0.481676`。SOC 几何比按
35 个正值对计算；余下一个共同为 `0/0` 的中性例仍计入 equal 与总和。
因此改动整体显著改善，但不声称逐例单调；2 个 makespan 与 3 个 SOC 回退
被原样保留。

最终 benchmark binary SHA-256 为
`70dbf8cb5096ce82ffbad5abc1c39d6454725fe798ec6d5faab0e2be6f6c1726`。
严格 pool 回归独立运行 6670 ms，strict return-deadline 6892 ms；全量
suite 168897 ms 完成。实现没有实例名/seed 检测、无 pick/place 检测、无 feature
flag、无 legacy fallback，也没有第二套 planner/search loop。相邻 storage
候选只使用与通用 BFS 等价的局部表示优化；非相邻 route 仍走同一个
candidate core。最终网页审查与 GPT-5.6 Sol/high 代码审查均输出
`APPROVE`；代码审查记录的唯一非阻塞残余风险是诊断回归直接锁定生产所用
predicate，而没有再复制一套端到端 attach fixture。当前生产调用已由审查
逐行确认。
