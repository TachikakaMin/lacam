# Carrier-LaCAM：闭环货架目标与机器人任务联合搜索

**状态：draft v3.0，2026-09-01**

本文保留现有的 physical-configuration search、operator-level constraint tree、`TAPFPlanner::solve()`、`apply_ops()`、anytime restart 和输出修补，只重写 guidance：让 shelf goal 与 robot task 在每个搜索节点互相反馈。

## 0. 核心思想

> 不先固定 shelf goal，再让 robot 被动执行。对每个候选 shelf goal，先预测它会产生什么搬运任务、当前 robots 执行这些任务有多难，再选择 goal。

```text
physical state X
  -> 为每个 shelf-goal 候选预测当前任务
  -> 估计 robot 执行代价
  -> tau_guide：target shelf -> goal
  -> 生成正式 manipulation tasks
  -> rho：free robot -> task
  -> Carrier-PIBT 生成 joint action
  -> X'
  -> 重新评价
```

这就是 Carrier 版本的 ITA 反馈：robot 的实际位置、vacancy 和 blocker 变化，会改变 shelf 选择哪个 goal；goal 改变后，生成的 task 也随之改变。

## 1. 搜索状态与终止条件 `[保留]`

```text
X = (Q_robot, Q_target, Q_anon, kappa)
```

* `Q_robot`：labeled robot positions；
* `Q_target`：labeled target-shelf positions；
* `Q_anon`：canonicalized anonymous-shelf occupancy；
* `kappa[r]`：robot `r` 当前 free、携带匿名 shelf，或携带 target shelf。

robot 是唯一 actuator，每拍选择：

```text
Wait | Move(neighbor) | Lift | Drop
```

shelf movement 是这些动作的确定性 effect。`tau`、`rho`、task、path 和 priority 都不进入 state key。

每个 target shelf `b` 保留完整 goal set `G_b`。终止条件为：

```text
is_goal(X) iff every target b is grounded and position(b) in G_b
```

终止条件不读取临时 assignment；已经位于合法 goal 的 shelf 仍可被再次搬开。

## 2. 中间层：Manipulation Task `[重写]`

当前的 `request = {cell, priority}` 信息不足：它只告诉 robot 去哪里 Lift，却没有说明要搬哪个 shelf、搬到哪里、为哪个 target goal 服务。

新 task 表示一次明确的 shelf 状态变化：

```text
MoveShelf(s, from -> to, root = target b -> goal g)
```

其中：

* `s`：当前要搬的 target 或 anonymous shelf；
* `from -> to`：这次搬运的精确起点和落点；
* `root = b -> g`：这次搬运最终服务于哪个 shelf-goal objective；
* task 完成条件：`s` grounded at `to`。

robot 被分配的是 task，而不是 pickup cell：

```text
free                 -> 去 from
到达 from             -> Lift
carrying task shelf   -> 去 to
到达 to               -> Drop，task 完成
```

因此 `free_goal`、`target_next` 和 `parking_cell` 只是同一 task 在不同阶段的局部 waypoint，不再是三个独立目标。

在 one-empty sliding-puzzle 中，ready task 必须明确为：

```text
MoveShelf(s, u -> current_empty_cell)
```

不能只生成 `clear(u)` 后再临时找停车点。

## 3. Shelf goal assignment：`tau_guide` `[重写]`

对每个候选 `(b, g)`，先预测为了让 target shelf `b` 朝 `g` 前进，当前最先需要执行的 task：

```text
m_X(b, g) = compile_frontier_task(X, b, g)
```

例如，`b` 的下一格被 blocker `s` 占据，当前 vacancy 是 `e`：

```text
m_X(b, g) = MoveShelf(s, position(s) -> e, root = b -> g)
```

然后计算 execution-aware guidance cost：

```text
C_guide(X, b, g)
  = shelf rearrangement estimate
  + lambda * robot realization estimate
  + goal-switch hysteresis
```

其中：

* shelf 部分：距离、blocker chain、vacancy routing；
* robot 部分：当前 robots 到 `m_X(b,g)` 的 pickup、Lift、carry、Drop 代价；
* hysteresis：新 goal 必须明显更好才替换旧 goal。

用该 cost 求 injective matching：

```text
tau_guide : target shelf -> eligible goal
```

因此，即使 target shelf 没移动，只要 robot、vacancy 或 blocker 改变，`tau_guide` 也可能改变。它不能只按 moved target rows 更新。

例如，左侧 goal 距离更近，但需要清三个 blocker，且 robots 都很远；右侧 goal 多一格，但 robot 已在 shelf 下方。此时右侧的 end-to-end cost 更低，`tau_guide` 应改选右侧，并生成不同 task。

## 4. Robot task assignment：`rho` `[重写]`

根据最终 `tau_guide` 构造当前 task pool：

```text
M(X, tau_guide)
```

再求：

```text
rho : free robot -> ManipulationTask or IDLE
```

匹配代价主要是：

```text
robot 到 task.from 的 lower-deck distance
+ task priority / dependency depth
+ task-switch hysteresis
```

`rho` 延续的是稳定的 `TaskId`，不是当前 pickup cell。loaded robot 不参与 `rho`，因为 `kappa` 已经给出 physical binding。

为考虑多个 tasks 争抢同一 robot，每个 node 做一轮轻量反馈即可：

```text
1. 初步计算 tau^0
2. 根据 tau^0 生成 tasks，并求全局 rho^0
3. 给无人可接、竞争严重或执行昂贵的 root objective 加 execution price
4. repair 得到 tau^1
5. 用 tau^1 生成最终 tasks 和 rho
```

这些计算只负责排序，不要求精确求解或迭代到收敛。

## 5. Feedback 与 commitment

反馈不等于每拍随意换目标。

* `tau_guide` 在每个 physical node 重新评价；
* 尚未被接手的 task 可以随 `tau_guide` 立即改变；
* robot 接手一个 `MoveShelf(from -> to)` 后，对该局部 task 使用 soft commitment，直到完成或 task 失效；
* Lift 后 `kappa` 是唯一 hard commitment；
* 当前 task 完成后，再根据新的 `X` 和 `tau_guide` 生成下一 task；
* carried target 的 root goal 使用较强 hysteresis，避免左右震荡，但不作为 hard pruning；
* duplicate node 被 rewire 到新 parent 后，必须重建 hysteresis anchor、task 和 `rho`，不能保留旧 parent 的 guidance。

这样，长期 shelf goal 能接收 robot execution feedback，而一次 pick-carry-drop 仍保持连续。

## 6. Carrier-PIBT `[保留框架，改 guidance]`

`funcPIBT` 不再分别读取独立的 `free_goal`、`target_next` 和 `parking_cell`，而是从 `rho[r]` 的 task 推导当前 operator 顺序：

```text
free + assigned task:
    Move toward task.from > Lift > Wait

carrying task shelf:
    Move toward task.to > Drop at task.to > Wait

free + IDLE:
    Wait or leave protected cells
```

更统一地：

```text
score(op) = 执行 op 后，当前 task 的估计剩余代价
```

PIBT 处理 lower-deck robot conflicts；task dependency 决定优先级。action-constraint tree 仍最终枚举所有 primitive joint actions，`apply_ops()` 仍是合法性终裁。

## 7. Cost 与正确性

必须分开两种 shelf-goal matching：

```text
tau_guide  -> 含 blocking、vacancy、robot feedback，只用于 guidance
tau_LB     -> 只含可证明 lower bound，只用于 admissible h
```

robot realization cost、hysteresis 和 taboo 不能进入 `tau_LB`。

```text
g = 已执行的真实 physical cost
h = admissible lower bound
guidance cost = 可以非 admissible，只决定 successor 顺序
```

完整性保持不变，因为：

1. state key 只含 physical `X`；
2. `tau_guide`、task 和 `rho` 都是 ordering-only；
3. action-constraint tree 最终枚举全部 robot primitive operators；
4. fully constrained joint action 由 `apply_ops()` 独立裁决；
5. terminal condition 不依赖临时 assignment。

错误的 guidance 最多让搜索变慢，不会删除合法解。

## 8. 工程落点

唯一 solve loop 仍是：

```text
TAPFPlanner::solve()
```

不新增平行 planner、第二套 search pipeline 或 feature-flag fallback。

```text
carrier_guidance.hpp
  - 分离 tau_LB 与 tau_guide
  - compile_frontier_task(X, b, g)
  - build_tasks(X, tau_guide)
  - rho: robot -> TaskId

tapf_planner.cpp
  - funcPIBT 从 task 推导 local waypoint/operator order

dd_carrier.cpp
  - apply_ops() 与 goal condition 保持权威
```

`free_goal` 可以作为缓存保留，但必须由 task 推导，不能再代表真实 assignment。现有 anytime restart、output repair 和 validator-first 流程保持不变。

零 shelf 输入时，以上 carrier 数据结构自然为空，原 LaCAM-TAPF 路径必须逐位退化，不能通过模式检测切换旧算法。

## 9. 最小测试与消融

必须测试：

1. 零 shelf 与原 LaCAM-TAPF 逐位一致；
2. `|G_b|=1` 时退化为 fixed-goal carrier；
3. target 位置不变、robot/vacancy 改变时，`tau_guide` 可以改变；
4. 同一 `TaskId` 能连续经历 approach、Lift、carry、Drop；
5. one-empty 时 ready task 是相邻 shelf 移入 vacancy；
6. execution feedback 不进入 admissible `h`；
7. 所有返回计划通过 C++/Python replay。

核心消融只保留三组：

```text
A. frozen tau vs feedback-aware tau_guide
B. request cell vs semantic ManipulationTask
C. shelf-only cost vs robot-realization-aware cost
```
