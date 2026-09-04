# Carrier-LaCAM v4：基于 Task-BR-PIBT 的双层动态分配

**状态：v4.1 已实现，release 验证与同机配对已完成并通过最终独立复核**

## 1. 设计目标

Carrier-LaCAM 仍然只搜索一条完整的 robot-shelf physical configuration path。robot 是唯一 actuator，shelf 的运动由 `Move / Lift / Drop` 产生。`tau`、任务依赖、priority 和 `rho` 都只是 successor guidance，不进入状态 key，也不改变 goal condition。

本设计要统一解决三个问题：

1. 每个 target shelf 应该去哪个 eligible goal；
2. 为了实现这些 shelf-goal assignments，当前必须先移动哪些 shelves；
3. 当前可执行的 shelf tasks 应该交给哪些 robots。

整体流程是：

```text
physical state X
    ↓
upper shelf layout U
    ↓
对每个 target shelf b 和 eligible goal g：
    单独评估 b → g 的 shelf-side task cost
    ↓
Hungarian：
    tau = target shelf → goal
    ↓
固定 tau，联合运行 Task-BR-PIBT
    ↓
task dependency graph D
    ↓
ready shelf tasks
    ↓
Hungarian：
    rho = free robot → ready task
    ↓
Carrier-PIBT
    ↓
真实 joint action
    ↓
physical successor X'
```

这包含两层动态 assignment：

```text
shelf layout 改变
    → shelf-goal cost 改变
    → tau 可能改变

robot position 改变
    → robot-task cost 改变
    → rho 可能改变
```

robot 的普通移动不会直接改变 shelf 的 terminal goal。robot 必须真实移动 shelf、改变 upper layout，第一层 assignment 才重新评价。

---

## 2. 物理状态

完整状态保持为：

$$
X=
\left(
Q^R,\,
Q^{\mathrm{tgt}},\,
Q^{\mathrm{anon}},\,
\kappa
\right).
$$

其中：

* \(Q^R\)：labeled robot positions；
* \(Q^{\mathrm{tgt}}\)：labeled target-shelf positions；
* \(Q^{\mathrm{anon}}\)：canonicalized anonymous-shelf occupancy；
* \(\kappa(r)\)：robot \(r\) 当前 free、携带 target shelf，或携带 anonymous shelf。

搜索 key 只包含上述物理信息；临时 assignment 与 tasks 不进入 key。

每个 robot 每拍选择：

```text
Wait | Move(neighbor) | Lift | Drop
```

所有 joint actions 最终由 `apply_ops()` 判断是否合法。

每个 target shelf \(b\) 保留完整的 eligible goal set \(G_b\)。终止条件为：

$$
\forall b\in B_{\mathrm{tgt}}:
\quad
b\text{ grounded}
\land
p_b\in G_b.
$$

终止条件不读取当前 `tau`。

### Upper projection

定义：

$$
U(X)=
\text{所有 target 和 anonymous shelves 的当前格子位置}.
$$

`U` 不包含 free robots 的位置，也不区分一个 shelf 当前由哪个 robot 携带。只有 shelf 的格子坐标发生变化时，`U` 才变化。

---

## 3. 第一层：单独评估每个 shelf-goal pair

对于每个 target shelf \(b\) 和每个 \(g\in G_b\)，计算：

$$
C_B[b,g]
=
\operatorname{PairCost}(U,b,g).
$$

### 3.1 “单独评估”的含义

调用：

```text
CompilePair(U, b, g)
```

时：

* 唯一的 root objective 是 `b → g`；
* 其他 shelves 都保留在当前布局中，可以成为 movable blockers；
* 其他 target shelves 暂时不考虑自己的 terminal goals；
* 不考虑 robots 在哪里；
* 不进行 robot-task assignment；
* 不处理多个 root objectives 之间的竞争。

因此，这一层回答的是：

> 只从 shelf rearrangement 的角度看，在当前 upper layout 中，把 \(b\) 送到 \(g\) 大概有多难？

### 3.2 使用单 root Task-BR-PIBT rollout

`CompilePair` 不运行完整 BR-LaCAM，而是重复调用一个单 root 的 Task-BR-PIBT compiler：

```text
PairCost(U, b, g):

    U_hat = U
    cost = 0

    while b 尚未到达 g 且未超过预算:
        D = TaskBRPIBT(U_hat, roots={b → g})
        m = D 中最高优先级的 ready task

        if 不存在 ready task:
            break

        在 U_hat 中抽象执行 m
        cost += shelf_cost(m)

    return cost + residual_estimate(U_hat, b, g)
```

BR-PIBT 的关键机制是：一个 block 想进入被另一个 block 占据的位置时，递归处理 blocker，直到 displacement chain 找到 empty vertex；若递归失败，请求方回退并尝试其他候选。它本身是 successor generator，而不是完整搜索。

因此这里复用的是：

* priority inheritance；
* recursive displacement；
* candidate backtracking；

而不是在 Carrier-LaCAM 内再嵌套一套 BR-LaCAM search。

### 3.3 Pair cost

第一层 cost 只包含 shelf-side estimate，例如：

$$
C_B[b,g]
=
\alpha\cdot
\widehat N_{\mathrm{shelf\ moves}}
+
\gamma\cdot
\widehat N_{\mathrm{manipulation}}
+
\delta\cdot
\widehat N_{\mathrm{anonymous\ moves}}.
$$

可以考虑：

* target 到 goal 的距离；
* 为移动 vacancy 所需的 shelf shifts；
* blocker displacement chain；
* 预计的 Lift/Drop episode 数量；
* anonymous shelf movement。

不能考虑：

* robot 到 pickup point 的距离；
* 当前哪个 robot 空闲；
* robot-task competition；
* lower-deck congestion；
* 当前 `rho`。

如果 bounded rollout 没有到达 goal，使用：

```text
已产生的 shelf cost
+ 剩余 wall-distance estimate
+ stall penalty
```

compiler 没找到方案不能把 edge 标记为不可行。只有 eligibility 或 wall connectivity 能证明不可达时，cost 才能设为无穷。

### 3.4 Lazy exact PairCost matrix

实现不会无条件展开所有 pair rollout，而是先给每条 edge 一个可证明不超过
完整 PairCost 的 prefix lower bound。当前 Hungarian 选中的 edge 全部求精；
然后对每条未求精 edge 强制该 row-goal 配对，并对剩余 rows/columns 解一次
Hungarian lower bound。若这个 forced lower bound 小于或等于当前精确最优值，
该 edge 必须求精，等号也不能跳过。

终止时，每条仍未求精的 edge 都不可能进入任何 primary-optimal assignment；
所以 primary、moved-away secondary 和 assignment-vector tertiary 的结果与
完整 matrix 完全相同。`PairPlan.exact=false` 只是一条 lower-bound
certificate，不能被 priority、stall/truncation 统计或其他完整-rollout
consumer 使用；最终 `tau` 上的 edge 必须全部 `exact=true`。

---

## 4. Shelf-goal matching

得到上述 lazy-exact certificate 后运行：

$$
\tau(U)
=
\arg\min_{\tau\text{ injective}}
\sum_{b\in B_{\mathrm{tgt}}}
C_B[b,\tau(b)].
$$

其中：

$$
\tau(b)\in G_b.
$$

不同 target shelves 不能得到同一个 goal。

这里的 `tau` 只回答：

> 在当前 shelf layout 下，每个 target shelf 暂时应该去哪个 terminal goal？

它不直接产生 robot assignment，也不保证不同 shelf-goal pair 的 isolated plans 彼此兼容。不同 roots 之间的共享、dependency 和冲突由下一层联合编译处理。

ITA-LaCAM 同样将 matching 附着在 configuration node 上，并在 configuration 变化后更新 matching，再用新的 temporary targets 指导 successor generation。 本设计中的对应关系是：

```text
ITA-LaCAM：
agent positions → agent-target costs → assignment

Carrier-LaCAM：
upper shelf layout → shelf-goal task costs → assignment
```

---

## 5. 第二层：联合 Task-BR-PIBT 编译任务依赖

确定 `tau` 后，调用：

```text
CompileJoint(U, tau, target_priorities)
```

所有 target shelves 同时成为 roots：

```text
b0 → tau[b0]
b1 → tau[b1]
...
```

Task-BR-PIBT 不再自行选择 target goal；goal 已由 Hungarian 给定。

### 5.1 上层原子任务

任务定义为一次相邻 shelf shift：

```text
ShelfTask(s, from → to)
```

它表示：

> shelf \(s\) 当前位于 `from`，需要移动到相邻的 `to`。

这是 upper-deck task，不是单个 robot action。执行它可能需要：

```text
robot approach
→ Lift
→ loaded Move
```

任务完成条件是 shelf 到达 `to`，不要求立即 Drop。若新 upper layout 中同一 shelf 仍有 continuation task，当前 carrier 可以继续举着它。

建议数据结构：

```cpp
struct ShelfTask {
    ShelfSelector shelf;
    Cell from;
    Cell to;
    RootSet roots;
    Priority priority;
};
```

`TaskId` 由物理 effect `(shelf, from, to)` 决定。两个 roots 要求完全相同的 shelf shift 时，共享同一个 task。

### 5.2 Priority

基础 priority 直接放在 target shelves 上：

```text
priority[b]
```

它表示完成 target shelf \(b\) 的紧迫程度。

当高优先级 target \(A\) 需要 blocker \(B\) 让路时，\(A\) 的 priority 传给搬动 \(B\) 的 task。如果移动 \(B\) 又要求 \(C\) 先让路，同一 priority 继续传给 \(C\)。

因此：

$$
\text{target shelf priority}
\rightarrow
\text{blocker task priority}.
$$

匿名 shelf 不拥有独立的 terminal priority，只继承请求它移动的 root priority。

priority 只在 upper layout 变化后更新，不能因为 free robots 走了几步就改变。

### 5.3 递归编译

核心过程：

```text
ResolveShelf(s, root, inherited_priority):

    对 s 的候选 next cells 按 preference 排序

    for each candidate v:

        如果 v 已被更高优先级 task 预约:
            尝试下一个 candidate

        如果 v 当前为空:
            创建 ShelfTask(s, pos(s) → v)
            返回成功

        blocker = 当前占据 v 的 shelf

        如果 blocker 已在递归栈中:
            尝试其他 candidate
            或记录 rotation candidate

        如果 ResolveShelf(blocker, root,
                           inherited_priority) 成功:
            创建 ShelfTask(s, pos(s) → v)
            添加 dependency:
                task(blocker) → task(s)
            返回成功

    返回失败
```

候选排序规则：

* 对 root target \(b\)：优先降低到 \(\tau(b)\) 的距离；
* 对被要求让路的 target shelf：优先满足 inherited request，同时尽量不远离自己的 \(\tau\)；
* 对 anonymous shelf：优先寻找空格，并避免其他高优先级任务已经预约的位置。

普通 PIBT 中，占据候选位置的 agent 会递归继承请求；若递归失败，请求者再尝试下一个候选。 Task-BR-PIBT 将同一机制提升到 shelf task 层。

### 5.4 多个 roots 的冲突

联合编译器维护：

```text
reserved_shelf
reserved_destination
selected_option[root]
```

如果高优先级 root \(A\) 的方案与低优先级 root \(B\) 冲突，\(B\) 依次尝试：

1. 保持自己的 `tau[B]`，换下一步方向；
2. 保持 goal，换 blocker 的临时移动方向；
3. 保持 goal，换另一条 displacement chain；
4. 当前 upper epoch 暂停推进。

如果 \(B\) 没有替代方案，\(A\) 自己回退并尝试下一候选。

联合编译阶段不修改 `tau`。只有真实 shelf movement 改变 `U` 后，下一次 Hungarian 才重新选择 terminal goals。

当多个候选编译了相同的 root success vector 时，先比较 aggregate
remaining goal distance，再做 density-aware priority tie-break：

* upper vacancy 至少为 2 时，按 priority 排序后的 root 顺序逐项比较完整
  remaining distance；
* upper vacancy 为 0 或 1 时，只比较各 root 是否已经完成，避免单空位
  displacement chain 为一格距离改善反复改道。

因此 `[1,1]` 优于 aggregate 更差的 `[0,100]`；aggregate 相同的
`[0,2]` 又优于 `[2,0]`，不能仅因后者少一个 shared task 就牺牲高优先级
root 的完成或直接进展。最后才比较 task/work cost 与稳定 effect order。

### 5.5 Shared task 与 conflict

相同 effect：

```text
A 需要 X: u → v
B 也需要 X: u → v
```

应合并为：

```text
ShelfTask(X, u → v)
roots = {A, B}
priority = max(priority[A], priority[B])
```

不同 effect：

```text
A 需要 X: u → v1
B 需要 X: u → v2
```

属于冲突，低优先级 root 必须换 option，不能只加一条 dependency。

---

## 6. Dependency graph 与 ready tasks

递归树直接转化为：

$$
\text{blocker task}
\rightarrow
\text{requesting shelf task}.
$$

例如：

```text
Task(C, 2 → 3)
    ↓
Task(B, 1 → 2)
    ↓
Task(A, 0 → 1)
```

任务 \(m\) 是 ready 当且仅当：

```text
所有 predecessor conditions 已满足
m.from 仍由指定 shelf 占据
m.to 当前为空
该 shelf 没有被其他 robot 或 task 占有
```

只有 ready tasks 进入 robot-task matching。

图中的内部节点只是当前 upper layout 下的因果意图。系统不会让 robot 提前跑到一个尚不能移动的 blocker 下方长期等待。

---

## 7. One-empty 不特殊处理

考虑：

```text
[A][B][C][ ]
```

`A` 希望进入 `B` 的位置：

```text
A 请求 B 离开
B 请求 C 离开
C 的候选位置是当前 empty
```

通用递归自然生成：

```text
Task(C, 2 → 3)
    ↓
Task(B, 1 → 2)
    ↓
Task(A, 0 → 1)
```

当前唯一 ready task 是：

```text
Task(C, 2 → 3)
```

执行以后，empty 从 3 移到 2。系统在新 upper layout 上重新编译，此时 `Task(B,1→2)` 自然成为 ready。

因此不需要：

```text
if one_empty:
    单独运行 vacancy routing
```

one-empty 只是通用递归中只有一条 displacement chain 能够最终到达 empty。现有设计中显式编写的 one-empty frontier 分支可以删除。

---

## 8. Zero-empty 与同步 rotation

零 empty 时，递归可能形成 cycle：

```text
A → B 当前格
B → C 当前格
C → A 当前格
```

如果物理语义允许 following，并且有足够 carriers，这可能是合法的同步 rotation。

当前实现检测长度至少 3 的 closed recursion cycle，按最小 `TaskId`
canonicalize，去重并 transactionally 记录 `RotationCandidate`。失败 branch
rollback 时不会泄漏 rotation。后续可以把记录提升为：

```text
JointShiftBundle {
    Task(A, ...)
    Task(B, ...)
    Task(C, ...)
}
```

bundle 只有在每个 constituent task 都已有 carrier，或都能被分配 robot 时，才作为 preferred joint guidance。

当前版本尚不把记录变成 preferred bundle，也不能把该状态判为不可解。
compiler 只返回“当前没有 preferred ready task”，外层 action-constraint
tree 仍然保留并枚举同步 rotation。

---

## 9. 第三层：Robot-task matching

已经被 robot 携带的 shelf，其 ready continuation task 直接绑定给当前 carrier，不参加 Hungarian。

剩余 grounded ready tasks 与 free robots 匹配：

$$
\rho:
R_{\mathrm{free}}
\rightarrow
M_{\mathrm{ready}}\cup\{\mathrm{IDLE}\}.
$$

代价为：

$$
C_R[r,m]
=
\beta d_{\mathrm{lower}}(q_r,m.from)
+
\eta_R
\mathbf 1[\text{switch TaskId}].
$$

匹配采用词典序：

1. 优先服务高-priority tasks；
2. 再最小化 robot approach distance；
3. 再偏向保持上一节点的 TaskId。

这一层才考虑 agent 与 task 的关系。

如果只有 free robots 发生移动：

```text
C_B 不变
tau 不变
dependency graph 不变
只 repair rho
```

这与 ITA-LaCAM 中“只有发生位置变化的 matching rows 需要 repair”的增量思想对应。

---

## 10. Carrier-PIBT 执行

Carrier-PIBT 根据 task phase 为 robot 排序动作。

```text
free + assigned task:
    朝 task.from 移动
    到达且 shelf 仍在 from：Lift
    否则 Wait

carrying + bound task:
    loaded Move 到 task.to
    到达 task.to 后，该 upper task 完成

carrying + continuation task:
    保持 Lift，继续下一个 loaded Move

carrying + no continuation:
    Drop

free + IDLE:
    离开 active pickup/drop cells
    否则 Wait
```

task priority 传递给 assigned robot。若 lower deck 上有另一个 robot 阻挡，则继续使用原 PIBT 的 priority inheritance。

完整优先级链为：

$$
\boxed{
\text{target shelf}
\rightarrow
\text{blocking shelf task}
\rightarrow
\text{assigned robot}
\rightarrow
\text{blocking robot}
}
$$

Task-BR-PIBT 处理多步 shelf causality；Carrier-PIBT 处理当前 timestep 的 robot conflicts。

---

## 11. 更新与 commitment

### Robot-only transition

free robot 执行 `Move` 或 `Wait`：

```text
U 不变
PairCost 不变
tau 不变
task graph 不变
只更新 rho
```

### Lift

shelf 位置不变：

```text
tau 不变
对应 task 进入 custody
该 robot 从 rho 中移除
```

### Loaded Move

shelf 位置真正改变：

```text
开启新的 upper epoch
更新 U
重新计算受影响的 PairCost
repair tau
重新联合编译 dependency graph
重新计算 ready tasks 和 rho
```

### Drop

如果 Drop 没有改变 shelf 坐标：

```text
tau 不变
解除 custody
重新计算 task 状态和 rho
```

### In-flight commitment

robot Lift 后，当前 one-step `ShelfTask` 至少持续到 shelf 到达 `task.to`。新的 `tau` 不会在半途中取消这个物理动作。

到达 `task.to` 后：

* 如果新 task graph 仍要求同一 shelf 继续移动，当前 robot 可以保持 custody；
* 否则 Drop。

实现时必须避免 roomy layout 中的两格往返：如果新 task 恰好让 shelf
立即回到上一拍的 `from`，该 reverse task 仍是 ready、仍可由 operator tree
枚举，但不自动续上 custody，当前 loaded robot 首选 Drop。只有
target-dense layout 才允许直接绑定这种 exact reverse：

```text
n_targets > n_vacancies
and n_targets - n_vacancies >= n_vacancies
```

这是 preferred-guidance 规则，不删除任何 physical successor。

### Priority commitment

一个 loaded Move 完成后，可以把它服务的未完成 roots 提升到下一 upper
epoch，但仅保留 `tau` 未改变的 roots。singleton fixed-goal target 的 self
move 不自我续约；multi-goal root 只在 target-dense layout 中续约。

选择完成的 shared-effect group 后，严格多数时整组续约，否则通常只续约
最高优先 root；当 active roots 至少达到 upper vacancies 的两倍，或上一轮
collective commitment 与该组相交时，整组续约。承诺只改变 guidance
priority，不进入 state key 或 admissible heuristic。

---

## 12. Guidance 与 admissible heuristic 分离

必须保留两种 shelf-goal matching：

```text
tau_guide：
    使用 Task-BR-PIBT PairCost
    决定当前 goal 与 task guidance
    可以非 admissible

tau_LB：
    只使用可证明的 lower bound
    只用于 h
```

例如：

$$
LB[b,g]
=
\alpha d_{\mathrm{upper-wall}}(p_b,g)
+
opLB(b,g).
$$

$$
h_{\mathrm{shelf}}(X)
=
\min_{\tau\text{ injective}}
\sum_b LB[b,\tau(b)].
$$

blocker 数量、displacement chain 和 pair rollout cost 不能进入 admissible \(h\)。

---

## 13. Completeness

完整性不依赖 Task-BR-PIBT 是否聪明：

1. search state 只包含真实物理状态 \(X\)；
2. `PairCost`、`tau_guide`、task graph、priority 和 `rho` 都是 ordering-only；
3. Task-BR-PIBT 失败不能删除任何 physical successor；
4. action-constraint tree 最终枚举所有 robot primitive joint actions；
5. fully constrained action 由 `apply_ops()` 独立裁决；
6. terminal condition 不读取临时 assignment。

因此错误的 pair cost 或 task dependency 最多让搜索变慢，不会永久删掉合法解。

---

## 14. 总伪代码

```text
AttachGuidance(X, parent):

    U = UpperProjection(X)

    if U changed:
        for each target b:
            for each eligible goal g:
                plan[b,g] = CompilePair(U, b, g)
                C_B[b,g] = plan[b,g].estimated_cost

        tau = HungarianInjective(C_B)

        priority =
            UpdateTargetPriority(parent.priority, U, tau)

        D =
            CompileJointTaskBRPIBT(U, tau, priority)

    else:
        reuse C_B, tau, priority, D

    ready = ReadyTasks(D, X)

    bind carried shelves to exact continuation tasks
    suppress only immediate-reverse auto-binding in roomy layouts

    rho =
        MatchFreeRobotsToReadyTasks(
            free_robots,
            remaining_ready_tasks,
            parent.rho)

    return Guidance(tau, D, ready, rho)
```

```text
GenerateSuccessor(N, operator_constraint):

    固定 constraint 已指定的 robot operators

    对其他 robots 按 task priority 顺序：
        free robot 使用 rho task
        loaded robot 使用 custody task
        Carrier-PIBT 递归处理 lower-deck blockers

    return apply_ops(N.X, joint_operator)
```

---

## 15. 工程落点

```text
carrier_guidance.hpp
    UpperProjection
    CompilePair
    PairPlan cache
    tau_guide matching
    TaskBRPIBTCompiler
    ShelfTaskGraph
    ReadyTasks
    rho matching

tapf_planner.cpp
    attach_carrier_guidance
    Carrier-PIBT task-phase candidates
    upper-epoch invalidation
    custody continuation

tapf_planner.hpp
    node fields:
        upper_signature
        tau_guide
        target_priority
        task_graph
        ready_tasks
        rho
        custody

dd_carrier.cpp
    PhysConfig
    apply_ops
    is_goal
```

不得新增平行 planner 或第二套 search loop。完整 BR-LaCAM 只作为 baseline 或离线 pair-cost oracle，不嵌套进每个 Carrier-LaCAM node。

---

## 16. 必须测试

1. 只移动 free robots，所有 `PairCost` 与 `tau` 完全不变。
2. loaded Move 改变 upper layout 后，`tau` 可以改变。
3. 不检查 empty 数量，通用递归仍能在 one-empty 中产生完整 dependency chain。
4. 只有 empty-adjacent leaf 进入 robot matching。
5. 两个 roots 要求相同 effect 时合并任务。
6. 同一 shelf 被要求前往不同 destinations 时触发 priority backtracking。
7. target blocker 在固定 `tau` 下优先尝试替代移动方向。
8. blocker 失败后，请求方尝试下一 candidate。
9. 同一 carrier 可以连续执行多个 one-step shelf tasks，不强制反复 Lift/Drop。
10. zero-empty compiler failure 不被解释为无解。
11. PairCost 的变化不影响 admissible `h`。
12. `|G_b|=1` 时自然退化为 fixed-goal Carrier-LaCAM。
13. PairCost 与 joint compiler 对同一 single-root state 使用同一 recursion
    core 和候选语义。
14. rollout 在最后一个 budget step 到达 goal 时必须报告 reached，而不是
    truncated。
15. zero-empty valid cycle 必须被 canonical、transactional 地记录。
16. 高优先级 root 的逐项 remaining distance 先于 shared-graph size
    tie-break。
17. aggregate remaining distance 必须先于逐根 priority tie-break。
18. 单空位只奖励 root completion；至少两空位时使用完整逐根 residual，
    两类固定回归都必须在 10 秒内交付合法计划。

---

## 17. 最终定义

$$
\boxed{
U
\rightarrow
\text{single-root Task-BR-PIBT pair costs}
\rightarrow
\tau
\rightarrow
\text{joint Task-BR-PIBT dependency graph}
\rightarrow
\text{ready shelf tasks}
\rightarrow
\rho
\rightarrow
\text{Carrier-PIBT}
\rightarrow
X'
}
$$

该结构的关键边界是：

* shelf-goal assignment 只读取 shelf layout；
* agent 只参与 ready task assignment 与执行；
* blocker chain 由通用 PIBT-style recursion 产生；
* one-empty 不需要特殊算法；
* terminal goals 由 Hungarian 动态更新；
* 局部 shelf tasks 通过 priority inheritance 与 backtracking 联合编译；
* 外层始终只有一个 Carrier-LaCAM physical configuration search。
