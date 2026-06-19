# Lifelong LaCAM-TAPF 当前实现说明

本文档按当前代码状态整理，不再描述早期草案。

当前分支已经把 one-shot TAPF / LaCAM-TAPF 扩展成 event-driven lifelong TAPF。核心原则是：每次 replanning 仍构造一个 TAPF instance，并继续使用已有 LaCAM-TAPF 与 Hungarian assignment；没有引入第二层 Hungarian。

---

# 1. 总体语义

每个 task 包含：

```text
task_id
task_type
start
goal_set
status
assigned_agent_id
picked_agent_id
release_timestep
pickup_timestep
completion_timestep
```

agent 有两种 load state：

```text
unloaded
loaded
```

状态含义：

* `unloaded` agent 可以被重新分配到 pending task 的 `start`。
* `loaded` agent 已经 pickup，只能去当前 task 的 `goal_set` 中某个点。
* `assigned` task 可以在 replanning 时释放并重新选择。
* `picked` task 不再参与新的 pickup assignment，直到完成 delivery。

---

# 2. 地图语义

地图字符：

| 字符 | 含义 |
| --- | --- |
| `@` | obstacle |
| `.` | traversable |
| `a` | storage / shelf aisle |
| `i` | inbound tunnel cell |
| `o` | outbound tunnel cell |

`a/i/o/.` 都是可通行格子，parser 保留 cell type。

当前主要地图：

```text
tests/assets/symbotic.map
tests/assets/symbotic_star.map
```

---

# 3. Task 生成规则

默认配置：

```text
goal_set_size = 3
release_interval = 10
backlog_multiplier = 2
outbound_probability = 0.5
start_capacity_per_cell = 2
```

release 规则：

* `t=0` 生成到 backlog 目标：`2 * num_agents`。
* 之后每 `release_interval` timestep 额外释放 1 个 task。
* 如果未完成 task 数量低于 `2 * num_agents`，会补齐 backlog。

未完成 task 指所有非 `completed` task，用于 backlog 计数。

---

# 4. Outbound / Inbound 定义

## 4.1 Outbound

```text
start: 随机选一个 a cell
goal_set: 随机选同一个 i/o tunnel component 中的 5 个 distinct cells
```

注意：outbound 的 goal set 不再只限制为 `o`，而是在一个连通 tunnel component 中采样，该 component 可以由 `i` 和 `o` 组成。

## 4.2 Inbound

```text
start: 随机选一个 i/o tunnel cell
goal_set: 随机选 5 个 distinct a cells
```

注意：inbound 的 start 不再只限制为 `i`，而是任意 tunnel cell，即 `i` 或 `o`。

如果首选 task type 无法生成合法 start/goal，generator 会尝试 alternate type；两种都失败则抛出：

```text
failed to generate task: no legal start/goal set
```

---

# 5. Start location 容量与释放

当前规则：

```text
每个 cell 最多允许 2 个未 pickup task start
```

容量只被以下状态占用：

```text
pending
assigned
```

容量不被以下状态占用：

```text
picked
completed
```

因此：

* agent 一旦 pickup，该 task 的 start 立即释放一个容量。
* 不需要等到 delivery/completion 才释放 start。
* 同一格可以同时有两个未 pickup task 排队。
* 但是 planner/Hungarian 中同一个物理 target 仍只能被一个 agent 选择，所以不会出现两个 agent 同时被分配到同一个 pickup cell。

相关常量：

```cpp
kLifelongTaskStartCapacity = 2
```

---

# 6. Task 生命周期

状态流转：

```text
pending -> assigned -> picked -> completed
```

额外允许：

```text
assigned -> pending
```

这是 replanning 前释放未 pickup assignment 的结果。

具体规则：

1. `pending -> assigned`
   unloaded agent 在 Hungarian 结果中选择了该 task。

2. `assigned -> pending`
   下一次 replanning 前，所有还没有 pickup 的 assigned task 会释放回 pending。

3. `assigned -> picked`
   assigned agent 到达 task start。

4. `picked -> completed`
   loaded agent 到达当前 task 的任一 goal。

5. `completed`
   清空 active binding，不再参与 assignment 或 backlog 补齐之外的未完成统计。

---

# 7. Assignment 与 TAPF 构造

当前实现是单层 Hungarian：

* loaded 和 unloaded agents 同时出现在同一个 TAPF instance 中。
* loaded agent 的候选目标是当前 task 的 `goal_set`，外加一个高 cost 的当前位置 deferred target。
* unloaded agent 的候选目标是 pending task 的 `start`，外加需要时的 idle target。

没有两层 Hungarian。

## 7.1 Unloaded cost

对 unloaded agent `u` 和 pending task `k`：

```text
cost(u, k)
= dist(u.current_location, k.start)
+ min_{g in k.goal_set} dist(k.start, g)
```

TAPF instance 中 unloaded agent 的 physical goal 是：

```text
k.start
```

不是 `k.goal_set`。

为了把完整 service cost 注入已有 TAPF/Hungarian，代码把 delivery estimate 编码进 `assignment_cost_offsets`。物理距离仍由 TAPF assignment 使用，offset 用来体现任务后半段代价。

如果同一个 start cell 上有两个 pending tasks：

* 对每个 agent，只保留该 start 上 offset 最小的 task 映射。
* Hungarian 仍只看到一个物理 target。
* apply assignment 时根据 `agent + start` 映射回具体 task。
* 同一个 task 仍最多只能绑定一个 agent。

## 7.2 Loaded cost

loaded agent 只能服务当前 picked task：

```text
min_{g in current_task.goal_set} dist(agent.current_location, g)
```

loaded agent 不参与新 task assignment，也不能切换 task。

---

# 8. Event-driven simulation loop

仿真不是每个 timestep 强制 replanning。

会触发 replanning 的情况：

* `t=0`
* pickup event
* completion event
* 当前 plan 为空或执行完
* 上次 planner failure/timeout 后没有有效 plan
* 存在 idle unloaded agent 且有 pending task

新 task release 本身不强制打断正在执行的 plan；它会进入 pending pool，并在下一次 replanning 中参与 assignment。

每次 replanning 前会调用：

```text
release_unpicked_assignments
```

它会：

* 将所有 `assigned` task 释放为 `pending`。
* 清空所有 unloaded agent 的 `current_task_id`。
* 保留 loaded agent 与 picked task 的绑定。

---

# 9. Planner failure / timeout 行为

每次 planner invocation 默认 time limit：

```text
2 seconds
```

命令行可通过 `TIME_LIMIT_SEC` 修改。

planner 支持 anytime 开关：

```text
ANYTIME=0/1
```

当 planner 返回空解或 failure：

* simulation 不崩溃。
* 所有 agent 原地等待一步。
* 下一 timestep 会继续尝试 replanning。
* metrics 记录 failure / timeout / empty solution / infeasible snapshot。

另外记录首次 empty solution 的诊断信息：

```text
first_empty_loaded_agents
first_empty_assigned_unloaded_agents
first_empty_idle_agents
first_empty_unique_target_count
first_empty_singleton_agents
first_empty_multi_goal_agents
```

---

# 10. Metrics

CSV 输出字段包括：

```text
map_name
num_agents
horizon
seed
generated_tasks
completed_tasks
throughput
alternating_completed_tasks
alternating_throughput
final_pending_tasks
final_assigned_tasks
final_picked_tasks
average_task_completion_time
average_pickup_time
average_delivery_time
planner_invocations
planner_success_count
planner_timeout_count
planner_failure_count
planner_snapshot_infeasible_count
planner_invalid_instance_count
planner_empty_solution_count
average_planner_runtime
max_planner_runtime
total_simulation_runtime
average_agent_idle_time
average_agent_loaded_time
average_agent_unloaded_time
valid
error
```

定义：

```text
throughput = completed_tasks / horizon
alternating_throughput = alternating_completed_tasks / horizon
```

`alternating_completed_tasks` 按 agent 统计：同一个 agent 本次完成 task 的 type 和上一次完成 task 的 type 不同，则计 1。

---

# 11. Schedule / Visualization 输出

`lifelong_benchmark` 可选输出 schedule YAML：

```text
lifelong_benchmark MAP NUM_AGENTS HORIZON SEED OUTPUT_CSV \
  [CACHE] [TIME_LIMIT_SEC=2] [GOAL_SET_SIZE=3] [OUTBOUND_PROB=0.5] \
  [RELEASE_INTERVAL=10] [DEBUG=0] [SCHEDULE_YAML] [ANYTIME=0]
```

YAML 中包含：

```text
statistics
assignments
schedule_binary
tasks
agent_task_timeline
```

大路径数据写到旁边的 binary schedule：

```text
*.yaml.bin
```

visualizer 当前支持：

* agent path playback。
* 播放速度调节。
* inbound cargo 黄色。
* outbound cargo 红色。
* selected agent 的 task start / goal set 高亮。
* 未 pickup task start 保持显示。
* 同一个 start 上多个未 pickup task 时显示数量，例如 `2`。
* throughput over time 曲线：淡色全流程曲线 + 实色当前进度覆盖。

---

# 12. 正确性检查

debug/smoke 中重点检查：

1. agent 在 traversable cell 上。
2. 没有 vertex conflict。
3. 没有 edge-swap conflict。
4. loaded agent 必须绑定 picked task。
5. picked task 必须绑定 matching loaded agent。
6. completed task 不保留 active agent binding。
7. pending/assigned task 的 start 每格数量不超过 `kLifelongTaskStartCapacity`。
8. picked task 的 start 不占用容量。
9. 一个 agent 不能同时绑定多个 task。
10. 一个 task 不能同时绑定多个 agents。
11. unloaded agent 可以在 replanning 时切换未 pickup task。
12. loaded agent 不允许切换 task。
13. planner failure/timeout 后不能永久等待。

---

# 13. 当前主要测试覆盖

已有测试覆盖：

* map parser 保留 `a/i/o` cell type。
* distance cache 计算、保存、读取、metadata mismatch。
* outbound / inbound task 区域合法性。
* tunnel component goal sampling。
* fixed seed reproducibility。
* start pickup 后释放并可复用。
* 每格允许两个未 pickup task start，第三个非法。
* task 状态转移：pickup / completion。
* alternating completed task 计数。
* unloaded assignment cost 包含 pickup + delivery estimate。
* loaded/unloaded 同一 TAPF instance。
* 同一 pickup start 上多个 task 时只分配一个。
* loaded agents 共享 drop 时通过 alternative drop 或 deferred target 处理。
* unloaded replanning 可以切换 task。
* equal-cost replanning 保持旧 task。
* small lifelong simulation smoke。
* planner failure fallback to wait。

当前完整测试命令：

```bash
cmake --build build --target test_all lifelong_benchmark -j8
./build/test_all
```

最近验证结果：

```text
43/43 tests passed
```

---

# 14. 已观察到的实验现象

## 14.1 start 容量为 1 时

`symbotic_star.map` 上 150 agents、300 steps：

```text
generated_tasks = 684
completed_tasks = 385
throughput = 1.28333
initial tasks = 40 inbound / 260 outbound
planner_failure_count = 0
```

原因：地图只有 40 个 tunnel start cells，start 容量为 1 时初始 inbound 最多 40 个。

## 14.2 start 容量为 2 时

同场景：

```text
generated_tasks = 654
completed_tasks = 354
throughput = 1.18
initial tasks = 80 inbound / 220 outbound
planner_failure_count = 0
max unpicked tasks per start = 2
simultaneous assigned same start steps = 0
```

解释：

* inbound 初始容量从 40 提升到 80。
* planner 仍不会同时派两个 agent 去同一个 start。
* 更多 inbound 会增加 tunnel 区域负载，因此 throughput 可能下降。

---

# 15. 当前不包含的功能

当前没有实现：

* task priority。
* task deadline。
* battery / charging。
* shelf rebalancing。
* multi-carry。
* dynamic obstacle。
* 每 timestep 强制 replanning。
* 第二层 Hungarian。

当前第一版目标已经完成：

```text
lifelong TAPF + event-driven replanning + unloaded reassignment
+ shared start capacity + metrics/visualization
```
