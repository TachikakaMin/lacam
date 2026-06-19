# Goal: Multi-carry Lifelong LaCAM-TAPF with Dynamic TA and Circle Cost

本文档描述当前 lifelong LaCAM-TAPF 分支的下一步实现目标。

当前代码已经实现：

```text
lifelong TAPF
event-driven replanning
single-layer Hungarian assignment
loaded / unloaded agents
pending / assigned / picked / completed task lifecycle
shared pickup start capacity
planner failure / timeout fallback
metrics and visualization
```

本轮已经实现：

```text
carried_task_ids
multi_carry_capacity
loaded_distance_since_last_delivery
per-agent-target distance weight
multi-carry schedule / visualization timeline
single-layer unified pickup/delivery candidate generation
circle-based throughput-aware integer cost
dynamic LaCAM node assignment with per-pair distance scale
row cost cache keyed by (agent_id, cell_id)
```

当前仍然不是本阶段目标 / 未做成默认行为：

```text
second-layer Hungarian
per-timestep forced replanning
explicit delivery-task assignment
incremental Hungarian
candidate pruning
```

不要重写 planner。
不要引入第二层 Hungarian。
不要把 event-driven replanning 改成每 timestep 强制 replanning。
不要显式在 cost 中决定 deliver 哪个 task。

---

# 1. 总体目标

当前实现中，每个 agent 最多只能 carry 一个 picked task。

现在希望允许每个 agent 最多 carry (K) 个 task：

[
K = \text{multi_carry_capacity}.
]

对 agent (i)，令：

[
C_i
]

表示当前已经 pickup 但尚未 completed 的 carried task 集合。

[
n_i = |C_i|
]

表示当前载货数量。

根据 (n_i)，agent 的逻辑状态为：

[
n_i = 0
]

表示 unloaded。

[
0 < n_i < K
]

表示 loaded but not full。

[
n_i = K
]

表示 full。

旧的 `load_state` 可以继续保留，但核心逻辑应该基于：

```text
carried_task_ids
carried_task_count
multi_carry_capacity
```

而不是只依赖单个 `current_task_id`。

兼容要求：

```text
K = 1 时，行为必须等价于当前 single-carry 实现。
load_state == LOADED 等价于 carried_task_count > 0。
current_task_id 在 K=1 下可以继续作为 sole carried task 的兼容字段。
K > 1 后，current_task_id 不能作为 correctness 的唯一来源。
```

---

# 2. 保留当前实现原则

必须保留当前实现中的以下原则。

## 2.1 仍然使用 single-layer Hungarian

当前实现已经把 loaded 和 unloaded agents 放在同一个 TAPF instance 中，并通过 Hungarian assignment 选择物理目标。

multi-carry 之后仍然使用同一个 Hungarian。

不要增加第二层 Hungarian。

具体来说：

```text
Hungarian 选择的是 physical target
不是显式选择 pickup/delivery task pair
```

---

## 2.2 仍然使用 event-driven replanning

replanning 仍然由事件触发：

```text
t = 0
pickup event
delivery/completion event
当前 plan 为空或执行完
planner failure/timeout 后没有有效 plan
存在 idle unloaded agent 且有 pending task
```

新 task release 本身不强制打断当前 plan。

---

## 2.3 planner failure / timeout 行为不变

如果 planner failure、timeout 或返回 empty solution：

```text
simulation 不崩溃
所有 agent 原地等待一步
下一 timestep 继续尝试 replanning
metrics 记录 failure / timeout / empty solution / infeasible snapshot
```

---

# 3. Task 生命周期

task 状态仍然是：

```text
pending -> assigned -> picked -> completed
```

并且 replanning 前仍然允许：

```text
assigned -> pending
```

multi-carry 下，`picked` 的含义变成：

```text
该 task 当前被某个 agent carry
```

每个 picked task 必须满足：

```text
task.picked_agent_id = i
task.task_id in agent[i].carried_task_ids
```

每个 agent 可以 carry 多个 picked tasks：

```text
agent[i].carried_task_ids = {task_1, task_2, ..., task_n}
```

其中：

[
0 \le n \le K.
]

---

# 4. Agent 状态扩展

每个 agent 至少维护：

```text
agent_id
current_location
carried_task_ids
carried_task_count
current_target
executed_path / history
completed_task_count
loaded_distance_since_last_delivery
```

为了兼容现有代码，可以暂时保留：

```text
load_state
current_task_id
```

但是 correctness check 应该基于 `carried_task_ids`，而不是单个 `current_task_id`。

---

# 5. Candidate set 统一定义

对每个 agent (i)，令：

[
p_i
]

表示当前位置。

[
C_i
]

表示当前 carried tasks。

[
n_i = |C_i|
]

表示当前载货数量。

[
K
]

表示最大载货量。

agent (i) 的 candidate set 定义为：

[
A_i = P_i \cup D_i.
]

---

## 5.1 Pickup candidates

pickup candidate 来自 pending task 的 start location。

[
P_i =
\begin{cases}
{P_k : k \text{ is pending}}, & n_i < K, \
\emptyset, & n_i = K.
\end{cases}
]

对于 pickup candidate (P_k)，物理目标是：

[
s_k
]

其中 (s_k) 是 task (k) 的 start location。

full agent 不允许选择 pickup candidate。

注意当前 task generator 允许同一个 start cell 上最多两个未 pickup tasks：

```text
kLifelongTaskStartCapacity = 2
```

这个容量只限制 task generation / pending pool，不表示 Hungarian 可以同时把两个 agents 分配到同一个 physical start。TAPF target 仍然是去重后的 physical cell。

---

## 5.2 Delivery candidates

delivery candidate 是当前 carried tasks 的所有 goal set 的并集。

[
D_i =
\begin{cases}
{D_g : g \in \bigcup_{j\in C_i}G_j}, & n_i > 0, \
\emptyset, & n_i = 0.
\end{cases}
]

对于 delivery candidate (D_g)，物理目标是：

[
g.
]

注意：这里的 candidate 是 physical goal location，不是具体 task。

也就是说，Hungarian 不需要决定：

```text
deliver task j
```

而只需要决定：

```text
go to physical goal location g
```

当 agent 真正到达 (g) 后，simulation 再检查：

[
{j\in C_i : g\in G_j}
]

并完成其中一个 task。

如果多个 carried tasks 都可以在 (g) 完成，使用 deterministic tie-breaking：

```text
1. 最早 pickup_timestep
2. 然后最小 task_id
```

---

# 6. 距离定义

令：

[
d(u,v)
]

表示地图上的最短路距离。

对 goal set：

[
d(u,G_k)=\min_{g\in G_k}d(u,g).
]

如果不可达，则距离为：

[
+\infty.
]

任何包含不可达距离的 assignment cost 都应该设为：

[
+\infty.
]

---

# 7. Circle heuristic

使用 circle-based route heuristic。

定义：

[
\operatorname{Circ}(x,S)
]

表示以 reference location (x) 为参考，对 task 集合 (S) 计算的 goal circle cost。

对每个 task (k\in S)，选择一个 representative goal：

[
\gamma_x(k)=\arg\min_{g\in G_k}d(x,g).
]

也就是选择离 reference location (x) 最近的 goal。

令：

[
\Gamma_x(S)={\gamma_x(k):k\in S}.
]

如果：

[
|S|\le 1,
]

则：

[
\operatorname{Circ}(x,S)=0.
]

如果：

[
|S|\ge 2,
]

把 (\Gamma_x(S)) 里的 representative goals 按 `(row, col, vertex_index)` 排序，得到：

[
q_1,q_2,\ldots,q_m.
]

计算 left-to-right circle cost：

[
C_{\rightarrow}
===============

\sum_{r=1}^{m}d(q_r,q_{r+1}),
]

其中：

[
q_{m+1}=q_1.
]

因为当前地图距离是无向最短路，反向绕同一个 closed circle 的距离和正向相同。因此第一版只需要计算一个 deterministic closed circle：

[
\operatorname{Circ}(x,S)
========================

C_{\rightarrow}.
]

这是故意使用 circle，而不是 open path。

circle 只用于 assignment cost 的 lookahead，不强制未来执行顺序。因为系统在 agent 到达目标点后会重新 replanning。

---

# 8. Throughput-aware cost

Hungarian 是 minimize cost，所以 cost 应该表示 estimated time per completed task。

总体形式是：

[
\text{cost}
===========

\frac{
\text{estimated service cost}
}{
\text{estimated number of completed tasks}
}.
]

不要使用：

[
1/\text{distance}
]

作为 cost。

实现注意：

当前 Hungarian / TAPF assignment 使用整数 cost。所有 fractional cost 需要用 fixed-point integer 编码，不能直接把 `double` 传入现有 Hungarian。

建议新增：

```text
assignment_distance_scales[i][t]
assignment_cost_offsets[i][t]
```

或等价的 cost callback，使每个 agent-target pair 可以表达：

```text
fixed_cost = scale[i][t] * distance + offset[i][t]
```

现有 `TAPFInstance::assignment_distance_scale` 是全局标量，只能表达当前 single-carry cost，不能表达 multi-carry 的 `1/(n_i+1)` 和 `1/n_i` per-pair weight。

---

# 9. Pickup cost

如果 agent (i) 选择 pickup pending task (k)，且：

[
n_i < K,
]

则：

[
c(i,P_k)
========

\frac{
d(p_i,s_k)
+
d(s_k,G_k)
+
\operatorname{Circ}(s_k,C_i\cup{k})
+
\mathbf{1}[n_i>0]L_i
}{
n_i+1
}.
]

其中：

[
L_i
]

是 agent (i) 从上一次 delivery 之后，在 loaded 状态下已经走过的距离。

解释：

```text
d(p_i, s_k)
    当前 agent 位置到 pickup start 的距离

d(s_k, G_k)
    新 task 从 start 到最近 goal 的 direct delivery estimate

Circ(s_k, C_i ∪ {k})
    pickup 新 task 后，当前 carried tasks 加新 task 的 circle estimate

1[n_i > 0] L_i
    如果 agent 已经 loaded，则把已经发生的 loaded travel 加入 pickup cost

n_i + 1
    pickup 后预计 carried task 数量
```

当：

[
n_i=0
]

时，公式退化为当前 single-carry unloaded cost：

[
c(i,P_k)
========

d(p_i,s_k)+d(s_k,G_k),
]

因为：

[
\operatorname{Circ}(s_k,{k})=0
]

且：

[
\mathbf{1}[n_i>0]L_i=0.
]

---

# 10. Delivery cost

如果 agent (i) 选择 delivery goal location (g)，且：

[
n_i>0
]

并且：

[
g\in \bigcup_{j\in C_i}G_j,
]

则：

[
c(i,D_g)
========

\frac{
d(p_i,g)
+
\operatorname{Circ}(g,C_i)
}{
n_i
}.
]

这里不显式决定完成哪个 task。

当 agent 到达 (g) 后，simulation 完成一个满足：

[
g\in G_j
]

的 carried task (j)。

如果有多个 eligible tasks，使用 deterministic tie-breaking。

---

# 11. Loaded distance since last delivery

对每个 agent (i)，维护：

[
L_i.
]

如果 agent 当前 carrying 至少一个 task，即：

[
n_i>0,
]

并且从 (p_i(t)) 移动到 (p_i(t+1))，则：

[
L_i \leftarrow L_i + d(p_i(t),p_i(t+1)).
]

对于 grid move：

```text
move: +1
wait: +0
```

当 agent 完成任意一个 delivery 时：

[
L_i \leftarrow 0.
]

当 agent unloaded 时：

[
L_i = 0.
]

不要引入 tunable parameter (\lambda)。

(L_i) 的作用是自然惩罚：

```text
agent 已经 loaded 很久，但还继续 pickup
```

而不是人为调参。

---

# 12. LaCAM node-level dynamic TA

当前 TAPF planning 中，LaCAM node 里的 agent 位置会不断变化。

因此 assignment cost 不能只在 replanning snapshot 计算一次。

对一个 LaCAM node (q)，令：

[
v_i(q)
]

表示 agent (i) 在该 LaCAM node 中的位置。

那么 node-level cost 应该写成：

## Pickup

[
c_q(i,P_k)
==========

\frac{
d(v_i(q),s_k)
+
d(s_k,G_k)
+
\operatorname{Circ}(s_k,C_i\cup{k})
+
\mathbf{1}[n_i>0]L_i
}{
n_i+1
}.
]

## Delivery

[
c_q(i,D_g)
==========

\frac{
d(v_i(q),g)
+
\operatorname{Circ}(g,C_i)
}{
n_i
}.
]

注意：

```text
v_i(q) 是 LaCAM node 内部动态变化的位置
C_i, n_i, K, L_i 在一次 planner invocation 内可以先视为 snapshot-static
```

第一版中，不需要在 LaCAM search 内部动态更新 (C_i) 或 (L_i)。
真正的 pickup/delivery 状态更新仍然发生在 simulation 执行 plan 后的 arrival event。

也就是说：

```text
LaCAM node 内动态变化的是 agent position
task state / carried_task_ids 在一次 planning invocation 中保持不变
```

这样实现最简单，也和当前 event-driven replanning 逻辑一致。

---

# 13. Cost decomposition for efficient update

为了避免每个 LaCAM node 都完整重算复杂 cost，把 cost 拆成：

[
c_q(i,t)
========

w_{i,t}\cdot d(v_i(q),\ell_t)+b_{i,t}.
]

其中：

```text
t       = physical target column
ell_t   = target location
w_{i,t} = dynamic distance coefficient
b_{i,t} = snapshot-static offset
```

落到当前代码时不要使用 floating-point matrix。使用 fixed-point 表示：

```text
COMMON_SCALE = lcm(1, 2, ..., multi_carry_capacity)
integer_cost = numerator * COMMON_SCALE / denominator
```

其中 denominator 对 pickup 是 `n_i + 1`，对 delivery 是 `n_i`。

这里的 `numerator` 是原始未除以 task count 的服务代价：

```text
pickup numerator
  = d(v_i(q), s_k)
  + d(s_k, G_k)
  + Circ(s_k, C_i union {k})
  + 1[n_i > 0] * L_i

delivery numerator
  = d(v_i(q), g)
  + Circ(g, C_i)
```

不要直接使用裸整数除法：

```text
cost = numerator / denominator
```

因为它会过早丢失排序信息。先乘 `COMMON_SCALE` 再除，可以保持整数 cost，同时保留 `1/2`、`1/3` 这类比例的主要排序差异。

需要同步扩展：

```text
TAPFInstance
assign_tapf_tasks
assign_tapf_tasks_dynamic
TAPFAssignmentState::repair_rows cost callback
```

让 dynamic assignment 在 LaCAM node 内读取 per-pair integer weight/offset。

---

## 13.1 Pickup decomposition

对 pickup target (P_k)：

[
\ell_{P_k}=s_k.
]

[
w_{i,P_k}
=========

\frac{1}{n_i+1}.
]

[
b_{i,P_k}
=========

\frac{
d(s_k,G_k)
+
\operatorname{Circ}(s_k,C_i\cup{k})
+
\mathbf{1}[n_i>0]L_i
}{
n_i+1
}.
]

因此：

[
c_q(i,P_k)
==========

w_{i,P_k}\cdot d(v_i(q),s_k)+b_{i,P_k}.
]

---

## 13.2 Delivery decomposition

对 delivery target (D_g)：

[
\ell_{D_g}=g.
]

[
w_{i,D_g}
=========

\frac{1}{n_i}.
]

[
b_{i,D_g}
=========

\frac{
\operatorname{Circ}(g,C_i)
}{
n_i
}.
]

因此：

[
c_q(i,D_g)
==========

w_{i,D_g}\cdot d(v_i(q),g)+b_{i,D_g}.
]

---

# 14. Per-replanning preprocessing

每次外层 replanning 开始时，构造一次 snapshot-static data。

这些数据在同一次 planner invocation 的所有 LaCAM nodes 中复用。

---

## 14.1 构造 physical target list

构造全局 physical target list：

```text
unique pending task starts
union of all carried-task goal locations
idle / deferred targets if needed
```

记为：

[
T.
]

每个 target (t\in T) 有一个 physical location：

[
\ell_t.
]

---

## 14.2 预处理 target distance table

对每个 physical target (t)，准备：

[
D_t[v]=d(v,\ell_t)
]

其中 (v) 是任意 traversable cell。

如果当前已有 all-pairs shortest path cache，则直接从 cache 读取。

如果 all-pairs cache 太大，可以改成：

```text
per-target BFS distance table
```

也就是每次 replanning 只对当前 target list (T) 中的 target 做 BFS。

推荐接口：

```text
get_distance_to_target(target_id, cell)
```

返回：

[
d(cell,\ell_t).
]

这样 LaCAM node 中可以 O(1) 查询：

[
d(v_i(q),\ell_t).
]

---

## 14.3 预处理 direct task delivery estimate

对每个 pending task (k)，预处理：

[
h_k=d(s_k,G_k).
]

也就是：

[
h_k=\min_{g\in G_k}d(s_k,g).
]

这个值在 task 生成后就可以缓存，不需要每次 node 计算。

---

## 14.4 预处理 circle offset

对每个 agent (i) 和每个 candidate target，预处理 circle 部分。

### Pickup circle

对 pending task (k)，预处理：

[
\operatorname{Circ}(s_k,C_i\cup{k}).
]

因为 (C_i) 在一次 planner invocation 内固定，所以这个值不依赖 LaCAM node。

### Delivery circle

对 delivery goal (g)，预处理：

[
\operatorname{Circ}(g,C_i).
]

这个值也不依赖 LaCAM node。

由于：

[
n_i \le K
]

且：

```text
goal_set_size 当前默认是 3
第一版实验建议 K <= 5
```

所以每次 circle 计算最多只涉及很少的 task 和 goal locations。
circle 本身不是主要瓶颈。

---

## 14.5 预处理 weight 和 offset

对每个 agent-target pair ((i,t))，预处理：

```text
valid[i][t]
weight[i][t]
offset[i][t]
semantic_type[i][t]
task mapping if pickup
```

其中：

[
c_q(i,t)
========

weight[i][t]\cdot D_t[v_i(q)]
+
offset[i][t].
]

如果该 pair 不合法：

```text
valid[i][t] = false
cost = INF
```

---

# 15. Shared pickup start 的处理

当前实现允许同一个 start cell 上有多个 pending tasks，但 Hungarian 只看到一个 physical target。

multi-carry 后继续使用这个设计。

如果多个 pending tasks 共享同一个 start location (s)，则对每个 agent (i)，保留该 start 上 cost offset 最小的 task。

对同一个 start (s)，因为 dynamic distance term：

[
d(v_i(q),s)
]

对所有共享该 start 的 task 都一样，所以可以只比较 static part。

对 agent (i) 和 start (s)，选择：

[
k^*(i,s)
========

\arg\min_{k:s_k=s}
\left[
d(s_k,G_k)
+
\operatorname{Circ}(s_k,C_i\cup{k})
+
\mathbf{1}[n_i>0]L_i
\right].
]

然后 Hungarian 中只放一个 physical target (s)。

apply assignment 时使用：

```text
agent i + start s -> task k*(i,s)
```

仍然必须保证：

```text
同一个 task 最多绑定一个 agent
```

在当前 TAPFInstance 设计中，physical target 去重后 Hungarian 每轮最多选择一次 start `s`，因此同一 start 上的 concrete task 冲突正常不会发生。仍然保留 `used_task_ids` 检查作为防御；如果未来引入非去重 target 或 candidate pruning 导致冲突，再实现 deterministic fallback 到同一 start 下的 next-best task。

---

# 16. Delivery target 的处理

delivery target 不需要映射到具体 task。

对 agent (i) 和 physical goal (g)，只需要判断：

[
g\in\bigcup_{j\in C_i}G_j.
]

如果成立，则该 target 对 agent (i) 合法。

cost 使用：

[
c_q(i,D_g)
==========

\frac{
d(v_i(q),g)
+
\operatorname{Circ}(g,C_i)
}{
n_i
}.
]

当 agent 真正到达 (g) 后，再从：

[
{j\in C_i:g\in G_j}
]

中选择一个 task 完成。

---

# 17. LaCAM node cost matrix update

对每个 LaCAM node (q)，cost matrix 为：

[
M_q[i,t]
========

\begin{cases}
weight[i][t]\cdot D_t[v_i(q)] + offset[i][t], & valid[i][t],\
+\infty, & \text{otherwise}.
\end{cases}
]

不要在每个 node 重新计算 circle。

不要在每个 node 重新扫描 task goal set 来计算 (d(s_k,G_k))。

每个 node 动态更新的核心只有：

```text
agent 当前 cell v_i(q)
target distance table D_t[v_i(q)]
linear formula weight * distance + offset
```

---

# 18. Incremental row update

从 parent LaCAM node (q) 到 child node (q') 时，通常只有一部分 agent 的位置发生变化。

令：

[
\Delta(q,q')={i:v_i(q)\neq v_i(q')}.
]

则只需要更新这些 rows：

[
i\in\Delta(q,q').
]

对每个 changed agent (i)，重新计算：

[
M_{q'}[i,t]
]

for all targets (t\in T)。

未移动 agent 的 row 可以复用 parent node 的 row。

建议实现：

```text
mutable_cost_matrix
row_cache keyed by (agent_id, cell)
```

如果同一个 agent 在同一个 cell 多次出现，可以直接复用该 row。

row cache 的 key：

```text
(agent_id, cell_id)
```

row cache 的 value：

```text
vector<int> fixed-point costs over target list T
```

这样每个 row 的计算复杂度是：

[
O(|T|)
]

而不是重新计算所有 task heuristic。

---

# 19. Hungarian cost

当前代码已经有 `TAPFAssignmentState::repair_rows`，可以在 agent 位置变化后只修复 changed rows，不必每个 node 都重新 full Hungarian。

即便 row construction 变快，assignment 本身仍然可能是主要瓶颈。

第一版建议：

```text
先只优化 matrix construction
继续使用现有 TAPFAssignmentState / Hungarian 框架
profile 后再决定是否实现 incremental Hungarian
```

也就是说：

1. 先实现：

[
M_q[i,t]=w_{i,t}D_t[v_i(q)]+b_{i,t}.
]

2. 先实现 row-level update / row cache。
3. 保持 Hungarian solver 和 `repair_rows` 框架不变。
4. 如果 profiler 显示 Hungarian 成为瓶颈，再考虑 incremental Hungarian。

不要第一版就实现 incremental Hungarian。

---

# 20. Optional candidate pruning

默认模式不要 pruning，保证行为更接近当前实现。

可以增加可选性能参数：

```text
MAX_PICKUP_CANDIDATES_PER_AGENT
```

默认：

```text
0
```

表示不启用 pruning。

如果启用，则对每个 agent 只保留 cost 最小的前 (B) 个 pickup starts。

delivery candidates 不需要 pruning，因为：

[
n_i\le K
]

且每个 task 的 goal_set_size 通常也很小。

因此 delivery target 数量最多约为：

[
n_i\cdot goal_set_size.
]

当：

[
K\le 5
]

且：

[
goal_set_size=3
]

时，每个 agent delivery candidates 约为 25 个以内。

pickup candidates 才可能是主要来源。

---

# 21. Complexity expectation

令：

```text
A = number of agents
T = number of physical targets
B = number of changed agents between parent and child node
```

每次 replanning 的 static preprocessing：

```text
direct delivery estimate: O(number_of_tasks * goal_set_size)
circle offsets: O(A * T * K * goal_set_size) approximately
target distance table: O(T * map_size) if using per-target BFS
```

每个 LaCAM node 的 matrix row update：

[
O(B\cdot T).
]

如果不用 incremental Hungarian，Hungarian 本身仍然是：

[
O(\max(A,T)^3).
]

因此第一版优化目标是：

```text
让 cost matrix construction 不再成为瓶颈
```

而不是立刻优化 Hungarian solver。

---

# 22. (L_i) 在 LaCAM node 内的处理

第一版建议：

```text
在一次 planner invocation 内，把 L_i 视为 snapshot-static
```

也就是说，LaCAM search 内部不模拟 (L_i) 的变化。

原因：

1. 简单。
2. 和当前 event-driven simulation 结构一致。
3. (L_i) 的主要作用是跨 replanning event 记录已经发生的 loaded travel。
4. node-level (d(v_i(q), target)) 已经反映了当前 node 到目标的剩余距离。

如果之后想更精确，可以扩展为：

[
L_i(q)=L_i^0+\Delta_i(q),
]

其中 (\Delta_i(q)) 是从 root node 到 node (q) 期间 agent (i) 在 loaded 状态下的移动距离。

但这不是第一版目标。

---

# 23. Arrival events

arrival event 仍然发生在 simulation 执行 plan 时，而不是 LaCAM internal node expansion 中修改真实 task state。

---

## 23.1 Pickup arrival

如果 agent (i) 到达 assigned task (k) 的 start：

```text
task k: assigned -> picked
task k.picked_agent_id = i
task k.pickup_timestep = current timestep

append k to agent[i].carried_task_ids
agent[i].carried_task_count += 1

release one unit of start capacity for s_k
```

如果 pickup 前：

[
n_i=0,
]

则：

```text
L_i = 0
```

触发 replanning。

---

## 23.2 Delivery arrival

如果 agent (i) 到达 goal location (g)，并且：

[
\exists j\in C_i,\quad g\in G_j,
]

则完成一个 eligible task。

eligible set：

[
E_i(g)={j\in C_i:g\in G_j}.
]

选择：

```text
earliest pickup_timestep
then smallest task_id
```

完成 task (j)：

```text
task j: picked -> completed
task j.completion_timestep = current timestep
task j.picked_agent_id = null

remove j from agent[i].carried_task_ids
agent[i].carried_task_count -= 1
agent[i].completed_task_count += 1

L_i = 0
```

如果完成后：

[
n_i=0,
]

则 agent 变成 unloaded。

触发 replanning。

---

# 24. Replanning release logic

replanning 前继续调用：

```text
release_unpicked_assignments
```

它应该释放：

```text
assigned but not picked tasks
```

不释放：

```text
picked tasks
```

multi-carry 后，picked tasks 是所有出现在某个 agent 的 `carried_task_ids` 中的 tasks。

对每个 agent：

```text
if carried_task_count == 0:
    可以重新 assignment 到 pending pickup task

if 0 < carried_task_count < K:
    保留 carried_task_ids
    可以选择 pickup 或 delivery

if carried_task_count == K:
    保留 carried_task_ids
    只能选择 delivery
```

---

# 25. Correctness invariants

debug / smoke 模式下增加以下检查。

1. 每个 agent 都在 traversable cell 上。
2. 没有 vertex conflict。
3. 没有 edge-swap conflict。
4. 每个 picked task 恰好属于一个 agent。
5. 每个 picked task 出现在对应 agent 的 `carried_task_ids` 中。
6. agent 的每个 carried task 状态必须是 `picked`。
7. agent 的每个 carried task 的 `picked_agent_id` 必须等于该 agent。
8. 没有 agent carry 超过 (K) 个 task。
9. completed task 不出现在任何 agent 的 `carried_task_ids` 中。
10. pending / assigned task 的 start usage 不超过 `kLifelongTaskStartCapacity`。
11. picked task 不占用 start capacity。
12. pending / assigned task 最多被一个 agent 绑定。
13. (n_i=0) 的 agent 不能收到 delivery target。
14. (n_i=K) 的 agent 不能收到 pickup target。
15. (0<n_i<K) 的 agent 可以收到 pickup 或 delivery target。
16. delivery arrival 每次最多完成一个 task。
17. delivery arrival 只能完成 goal set 包含当前 location 的 task。
18. replanning 不释放 picked tasks。
19. planner failure / timeout 后不能永久卡住。
20. cost matrix 中 invalid agent-target pair 必须是 INF。
21. shared start 上同一个 concrete task 不能同时分配给多个 agents。

---

# 26. Metrics updates

保留现有 metrics。

新增可选 metrics：

```text
multi_carry_capacity
average_carried_tasks
max_carried_tasks
average_loaded_distance_since_last_delivery
max_loaded_distance_since_last_delivery
pickup_while_loaded_count
delivery_events
average_tasks_carried_at_delivery
full_agent_replan_count
average_ta_matrix_build_time
average_ta_hungarian_time
max_ta_matrix_build_time
max_ta_hungarian_time
row_cache_hit_rate
```

其中：

```text
average_carried_tasks
= average over all agents and timesteps of carried_task_count

max_carried_tasks
= maximum carried_task_count observed

pickup_while_loaded_count
= number of pickup events where agent carried at least one task immediately before pickup

average_tasks_carried_at_delivery
= average carried_task_count immediately before each delivery event

row_cache_hit_rate
= reused row count / requested row count
```

TA timing 很重要，因为 multi-carry 后每个 LaCAM node 可能都要 dynamic TA。

还需要更新 schedule / visualization 输出结构。当前 YAML 的 `agent_task_timeline` 每个 timestep 只能表达一个 `task` 和一个 `phase`，multi-carry 后应改为或新增：

```text
assigned_task
carried_tasks
target_type: pickup | delivery | idle
target_task_if_pickup
target_location
```

visualizer 也需要能显示同一 agent 携带多个 cargo。

---

# 27. Tests to add

## 27.1 Multi-carry pickup test

构造小地图，一个 agent，容量：

[
K=2.
]

验证：

```text
agent 可以 pickup 第一个 task
agent 仍然可以 pickup 第二个 task
agent pickup 第二个 task 后变成 full
full agent 不再有 pickup candidate
```

---

## 27.2 Multi-carry delivery test

验证：

```text
agent carry 两个 tasks
到达一个 goal 后只完成一个 task
另一个 task 仍然是 picked
另一个 task 仍然在 carried_task_ids 中
agent 仍然 loaded
L_i reset to 0
```

---

## 27.3 Unified candidate test

对：

[
K=2
]

验证：

```text
n_i = 0: pickup candidates only
n_i = 1: pickup and delivery candidates
n_i = 2: delivery candidates only
```

---

## 27.4 Circle heuristic test

构造 goal 坐标已知的 tasks。

验证：

```text
Circ(x, empty) = 0
Circ(x, one task) = 0
Circ(x, two or more tasks) uses representative nearest goals
Circ uses one deterministic closed circle on undirected map distances
unreachable segment produces INF
```

---

## 27.5 Pickup cost test

验证：

[
c_q(i,P_k)
==========

\frac{
d(v_i(q),s_k)
+
d(s_k,G_k)
+
\operatorname{Circ}(s_k,C_i\cup{k})
+
\mathbf{1}[n_i>0]L_i
}{
n_i+1
}.
]

覆盖：

```text
n_i = 0
n_i = 1
shared pickup start
unreachable distance
different LaCAM node positions
```

---

## 27.6 Delivery cost test

验证：

[
c_q(i,D_g)
==========

\frac{
d(v_i(q),g)
+
\operatorname{Circ}(g,C_i)
}{
n_i
}.
]

覆盖：

```text
n_i = 1
n_i = 2
goal shared by multiple carried tasks
unreachable distance
different LaCAM node positions
```

---

## 27.7 Dynamic TA row update test

构造 parent node 和 child node。

验证：

```text
只有位置变化的 agent row 被更新
未变化 agent row 被复用
row cache 对相同 (agent_id, cell_id) 命中
cost = weight * distance + offset
```

---

## 27.8 Shared start mapping test

构造两个 pending tasks 共享同一个 start。

验证：

```text
Hungarian 只看到一个 physical start target
对每个 agent 选择 static offset 更小的 concrete task
同一个 concrete task 不会被多个 agents 同时绑定
```

---

## 27.9 Delivery tie-breaking test

构造两个 carried tasks 的 goal set 共享同一个 goal location。

验证：

```text
agent 到达 shared goal
只完成一个 task
使用 earliest pickup_timestep
如果相同则使用 smallest task_id
```

---

## 27.10 K=1 regression test

设置：

```text
MULTI_CARRY_CAPACITY = 1
```

验证行为接近当前 single-carry 实现：

```text
unloaded agent only pickup
loaded/full agent only delivery
不能 pickup while loaded
已有 lifelong tests 继续通过
```

---

# 28. Command-line / config updates

新增参数：

```text
MULTI_CARRY_CAPACITY
```

默认值：

```text
1
```

保证当前行为兼容。

建议 benchmark 命令扩展为：

```text
lifelong_benchmark MAP NUM_AGENTS HORIZON SEED OUTPUT_CSV \
  [CACHE] [TIME_LIMIT_SEC=2] [GOAL_SET_SIZE=3] [OUTBOUND_PROB=0.5] \
  [RELEASE_INTERVAL=10] [DEBUG=0] [SCHEDULE_YAML] [ANYTIME=0] \
  [MULTI_CARRY_CAPACITY=1]
```

可选性能参数：

```text
MAX_PICKUP_CANDIDATES_PER_AGENT=0
```

默认 0 表示不 pruning。

---

# 29. Implementation priority

按以下顺序实现。

1. 添加 `MULTI_CARRY_CAPACITY` 参数，默认 1。
2. 扩展 agent state：`carried_task_ids`、`carried_task_count`、`loaded_distance_since_last_delivery`。
3. 把当前 single-task loaded 逻辑重构到 `carried_task_ids` 上。
4. 保证 (K=1) 下现有测试继续通过。
5. 实现 unified candidate generation。
6. 实现 circle heuristic。
7. 实现 pickup cost 和 delivery cost。
8. 扩展 TAPF assignment cost API，支持 per-agent-target integer weight/offset。
9. 把 cost 拆成 fixed-point `weight * distance + offset`。
10. 实现 per-replanning preprocessing。
11. 实现 target distance table 或复用 all-pairs distance cache。
12. 实现 LaCAM node-level dynamic TA matrix update。
13. 实现 row cache。
14. 扩展 shared pickup start 的 concrete task mapping。
15. 实现 delivery arrival 的 eligible task tie-breaking。
16. 更新 replanning release logic。
17. 更新 YAML schedule 和 visualizer 的 multi-carry timeline。
18. 增加 correctness invariants。
19. 增加 unit tests。
20. 增加 small multi-carry smoke test。
21. 增加 TA timing metrics。
22. 跑完整测试。
23. 跑小规模 benchmark。
24. profile TA matrix build time 和 Hungarian time。
25. 如果 Hungarian 成为主要瓶颈，再考虑 incremental Hungarian，不作为第一版目标。

---

# 30. Non-goals

本阶段不要实现：

```text
task priority
task deadline
battery / charging
shelf rebalancing
dynamic obstacles
per-timestep forced replanning
second-layer Hungarian
explicit delivery-task assignment
incremental Hungarian
learned cost function
capacity-dependent task generation policy
```

本阶段目标仅为：

```text
multi-carry lifelong TAPF
single-layer Hungarian
LaCAM node-level dynamic TA
circle-based throughput-aware cost
efficient cost matrix update through preprocessing
parameter-free anti-over-pickup behavior through L_i
```

---

# 31. 本轮实现记录

## 31.1 工作流程

本轮按以下顺序完成：

1. 重新阅读 `goal2.md`，把目标拆成状态模型、candidate set、cost matrix、assignment API、simulation event、metrics / visualization、tests。
2. 检查当前代码中的 `current_task_id`、`load_state`、`TAPFInstance::assignment_distance_scale`、`assign_tapf_tasks_dynamic`、`lifelong_benchmark` 和 visualizer。
3. 先扩展底层状态，不改变 planner 结构：

```text
LifelongAgentState.assigned_task_id
LifelongAgentState.carried_task_ids
LifelongAgentState.loaded_distance_since_last_delivery
multi_carry_capacity
```

4. 保留旧字段：

```text
load_state
current_task_id
```

但 correctness 以 `carried_task_ids` / `assigned_task_id` 为准。`current_task_id` 只做 K=1 和旧 timeline 兼容。

5. 重写 lifelong snapshot 构造：

```text
n_i = 0: pickup candidates + defer
0 < n_i < K: pickup candidates + delivery candidates + defer
n_i = K: delivery candidates + defer
```

6. 保留 single-layer Hungarian：

```text
Hungarian column 仍然是 physical target
pickup concrete task 只在 shared start mapping 中记录
delivery 不显式选择 task
```

7. 扩展 TAPF assignment cost API：

```text
assignment_distance_scales[i][target]
assignment_cost_offsets[i][target]
cost = scale[i][target] * d(v_i(q), target) + offset[i][target]
```

8. 实现 fixed-point cost：

```text
COMMON_SCALE = lcm(1, ..., K)
pickup denominator = n_i + 1
delivery denominator = n_i
integer_cost = numerator * COMMON_SCALE / denominator
```

9. 实现 circle cost：

```text
每个 task 选 reference 最近 representative goal
按 (row, col, vertex_index) 排序
计算 deterministic closed circle
```

10. 在 `assign_tapf_tasks_dynamic` 中加入 row cost cache：

```text
key = (agent_id, cell_id)
value = vector<int> costs over current target list
```

仍然使用现有 Hungarian / `repair_rows`，没有实现 incremental Hungarian。

11. 更新 simulation events：

```text
pickup: assigned -> picked, append carried_task_ids, release start capacity
delivery: 从 eligible carried tasks 中只完成一个
tie-break: earliest pickup_timestep, then smallest task_id
loaded movement: loaded 时移动 +1，delivery 后 L_i reset
```

12. 更新 benchmark / YAML / visualizer：

```text
CLI 增加 MULTI_CARRY_CAPACITY
CSV/YAML 输出 multi-carry metrics
agent_task_timeline 增加 assigned_task 和 carried_tasks
HTML 能显示多个 carried cargo 和多个 carried task 的 goal set
```

13. 增加测试并运行验证。

## 31.2 已验证命令

完整测试：

```bash
cmake --build build -j 8 && ./build/test_all
```

结果：

```text
48 tests passed
```

K=2 smoke benchmark：

```bash
./build/lifelong_benchmark ./tests/assets/lifelong-task-small.map 2 30 0 \
  build/results/goal2_multi_carry_smoke/results.csv \
  build/results/goal2_multi_carry_smoke/cache.dist \
  1.0 1 0.5 5 1 \
  build/results/goal2_multi_carry_smoke/schedule.yaml \
  0 2
```

结果：

```text
valid=1
generated_tasks=14
completed_tasks=10
throughput=0.333333
multi_carry_capacity=2
average_carried_tasks=0.816667
max_carried_tasks=2
pickup_while_loaded_count=3
delivery_events=10
average_tasks_carried_at_delivery=1.2
assignment_row_cache_hit_rate=0.891109
planner_failure_count=0
```

可视化生成：

```bash
python3 tools/visualize_tapf_schedule.py \
  ./tests/assets/lifelong-task-small.map \
  build/results/goal2_multi_carry_smoke/schedule.yaml \
  build/results/goal2_multi_carry_smoke/visualization.html
```

并确认 YAML 中出现 multi-carry timeline：

```text
carried_tasks: [0, 8]
carried_tasks: [1, 9]
carried_tasks: [10, 11]
```

Python 脚本语法检查：

```bash
python3 -m py_compile tools/visualize_tapf_schedule.py tools/run_lifelong_experiment.py
```

## 31.3 当前实现边界

当前实现没有做以下内容：

```text
second-layer Hungarian
explicit delivery-task assignment
per-timestep forced replanning
incremental Hungarian
candidate pruning
```

当前 row cache 是 dynamic assignment cost row cache，不是 Hungarian dual / matching 的 incremental update。
