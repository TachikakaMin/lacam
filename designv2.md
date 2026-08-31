# BiTA-Carrier-LaCAM：双层任务分配与物理配置联合搜索

## 1. 核心目标

算法不再为目标货架预先采样少量 goal，也不在根节点固定 shelf-goal pairing。每个 target shelf \(b\) 保留完整的允许目标集合 \(\mathcal G_b\)。搜索仍然只有一个，即原有 LaCAM-TAPF 的 physical configuration search。

每个 high-level node 同时维护：

$$
N=(X,\tau_B,\mathcal I,\rho_R,\text{tree},g,h),
$$

其中：

* \(X\) 是真实 robot、shelf 和 carrying configuration；
* \(\tau_B\) 是 target shelf 到 candidate goal 的临时一一 matching；
* \(\mathcal I\) 是根据当前 vacancy 和 shelf dependencies 生成的 upper-deck one-step intent；
* \(\rho_R\) 是 free robot 到当前 manipulation task 的临时 matching；
* `tree` 是 LaCAM low-level constraint tree，但约束的是 robot primitive operator。

依赖关系为：

$$
X\rightarrow\tau_B\rightarrow\mathcal I
\rightarrow\rho_R\rightarrow\text{Carrier-PIBT}.
$$

这里没有完整 shelf path，也没有完整 robot path。每生成一个 physical successor，两个 assignment 都立刻更新。

## 2. Goal-set 问题定义

给定 target shelves \(B_T\)、goal pool \(G_B\) 和 eligibility matrix：

$$
A^B[b,g]=1
\iff g\in\mathcal G_b.
$$

B 型公共边界目标应表示为：

$$
\mathcal G_b=G_{\mathrm{boundary}},
\qquad \forall b\in B_T.
$$

loader 只负责保存完整 goal pool 和 eligibility matrix，并检查是否存在覆盖所有 target shelves 的 matching。禁止执行以下操作：

1. 从边界随机采样与 target 数量相同的 goal；
2. 用 greedy nearest 固定 shelf-goal pair；
3. 把该 pair 写成实例中的固定 \(g_b\)。

物理状态为：

$$
X=(Q^R,P^T,O,\kappa),
$$

其中 \(Q^R\) 是 labeled robot positions，\(P^T\) 是 labeled target-shelf positions，\(O\) 是 anonymous shelf occupancy，\(\kappa\) 是 carrying relation。

两层 assignment、priority、temporary targets 和 dependency graph 都不进入 state key。

## 3. Terminal condition

终止条件不读取当前临时 matching \(\tau_B\)，而直接检查物理 configuration 是否属于任意合法 assignment：

$$
\exists\tau:B_T\rightarrow G_B:
\quad
\tau(b)\in\mathcal G_b,\quad
\tau\text{ injective},\quad
p_b=\tau(b),
$$

并且所有 target shelves 均已 grounded。

如果 goal 是物理 cell，且 shelves 不可能占据同一 cell，那么可以简化为：

$$
\forall b\in B_T:
\quad p_b\in\mathcal G_b
\quad\land\quad b\text{ grounded}.
$$

“完成”不是不可逆状态。一个 shelf 即使已经位于允许 goal，仍可被再次 lift、搬开并最终落到同一个或另一个允许 goal。系统不保存单调 completed bit。

## 4. 第一层 assignment：Shelf → Goal

每个 node 求解全局 injective matching：

$$
\tau_B^N=
\arg\min_{\tau}
\sum_{b\in B_T}C_N^B[b,\tau(b)].
$$

不能使用逐对 greedy elimination。

为了区分搜索 guidance 与 admissible heuristic，使用两个 cost matrix。

### Guidance matching cost

$$
C_N^{B,\mathrm{guide}}[b,g]
=
\alpha d_U(p_b,g)
+\lambda_{\mathrm{blk}}\widehat B_X(b,g)
+\lambda_{\mathrm{vac}}\widehat V_X(b,g)
+\eta_B\mathbf 1[g\neq\tau_B^{parent}(b)].
$$

其中：

* \(d_U\) 是 upper-deck wall distance；
* \(\widehat B_X\) 是 blocker / least-blocking estimate；
* \(\widehat V_X\) 是 vacancy 到关键移动位置的估计；
* \(\eta_B\) 是 assignment hysteresis。

第一版可以只使用 wall distance 和 operation lower bound，从而完整复用 ITA-LaCAM 的 moved-row incremental Hungarian。blocking-aware 项可以在 plateau 或小规模实例上重算，因为它只影响 ordering。

### Admissible matching cost

$$
C_N^{B,\mathrm{LB}}[b,g]
=
\alpha d_U(p_b,g)+opLB(b,g),
$$

其中：

$$
opLB(b,g)=
\begin{cases}
0,&b\text{ grounded},\ p_b=g,\\
\gamma_L+\gamma_D,&b\text{ grounded},\ p_b\neq g,\\
\gamma_D,&b\text{ currently carried}.
\end{cases}
$$

因此：

$$
h_{\mathrm{shelf}}(X)=
\min_{\tau}
\sum_b C_N^{B,\mathrm{LB}}[b,\tau(b)]
$$

是考虑 goal exclusivity 的 lower bound。

每次 target shelf 移动、lift 或 drop 后，repair 对应 matching row。只移动 robot 或 anonymous shelf 时，静态 base matching 可以直接复用。

定义：

$$
settled_N(b)
\iff b\text{ grounded}\land p_b=\tau_B^N(b).
$$

这只是 node guidance。`settled` shelf 仍可作为 blocker 被搬动。

## 5. Vacancy-Centric Shelf Intent

固定-goal 文档中的 `clear(blocker, parking region)` 不适合一空格 sliding-puzzle regime。新设计采用 Goal-Assigned BR-PIBT 风格的 upper guidance。

对每个 target shelf \(b\)，其临时目标直接取自全局 matching：

$$
g_b^{temp}=\tau_B^N(b).
$$

然后运行递归 vacancy request：

```text
request_progress(b):
    按到 τB(b) 的距离和 vacancy distance 排序候选 next cells

    如果候选 v 在本 timestep 开始时为空：
        产生 executable intent move(b, v)

    如果 v 被 shelf c 占据：
        记录 dependency b -> c
        递归 request_vacancy(c)
        在 BRaP-compatible 语义下 b 本 timestep 等待

    如果所有候选失败：
        intent(b) = wait
```

递归最终终止在当前 empty cell 邻接的 shelf。它输出：

* target root；
* blocker dependency chain；
* 当前可以进入 start-empty cells 的 shelf moves；
* dependency depth 和 inherited priority；
* carried shelf 的 preferred move/drop。

因此，在只有一个 empty cell 时，机器人不会盲目跑到最终 target shelf 下方，而是优先服务当前真正能够移动 vacancy 的 chain leaf。

这仍然只是 configuration generator 的 guidance。LaCAM operator constraints 可以强制生成任何其他合法 shelf effect。

## 6. 第二层 assignment：Robot → Manipulation Task

根据 upper intent 构造当前 task pool：

$$
\mathcal M_N=
\{
AcquireMove(s,u,v),
AcquireTarget(b,u),
Dummy
\}.
$$

其中：

* `AcquireMove(s,u,v)` 表示 shelf \(s\) 位于 \(u\)，当前 intent 希望它最终进入 empty cell \(v\)；
* `AcquireTarget(b,u)` 用于提前把 robot 送到尚未进入 executable frontier 的 target shelf 下方；
* `Dummy` 允许 robot idle 或 yield。

对于 anonymous shelf，task 以其当前 upper cell 标识，不给所有 anonymous shelves 引入永久 identity。

loaded robot 不参加 matching，因为：

$$
\kappa(r)=b
$$

已经是物理 hard binding。只有 free robots 参与：

$$
\rho_R^N=
\arg\min_\rho
\sum_{r\in R_F}C_N^R[r,\rho(r)].
$$

建议 cost 为：

$$
C_N^R[r,m]
=
d_L(q_r,pickup(m))
+\lambda_d\,depth(m)
-\mu\,priority(m)
+\eta_R\,switch(r,m).
$$

每个 real task 最多分配给一个 robot；通过 dummy columns 保证每个 robot 都有 assignment。任务多于机器人时，只选择最紧迫的一部分。

最重要的代码复用关系是：

$$
temporaryTarget(r)=pickup(\rho_R(r)).
$$

也就是说，对 free robot 而言，第二层 assignment 的 pickup cell 就是原 LaCAM-TAPF 中 agent 的 temporary target。原来的 target-guided PIBT、robot priority inheritance、reservation 和 swap 逻辑可以继续使用。

## 7. Robot Operator Search

每个 robot 的 primitive operator 为：

$$
a_r\in\{Wait,Move(v),Lift,Drop\}.
$$

low-level constraint 不能只固定 next vertex，因为 `Wait`、`Lift` 和 `Drop` 可能具有相同位置结果。constraint 必须保存完整 operator：

$$
C[r]=(a_r,q'_r,\kappa'(r)).
$$

configuration generator 的基本流程为：

```text
GenerateSuccessor(N, C):

    读取 N.τB、N.intent、N.ρR
    先应用 C 中已固定的 robot operators

    对尚未固定的 free robot：
        朝 pickup(ρR(r)) 运行原 TAPF PIBT
        已在 pickup cell 时优先 Lift

    对 carrying target shelf b 的 robot：
        若当前位置等于 τB(b)，优先 Drop
        否则优先 intent 指定的 loaded Move

    对 carrying anonymous shelf 的 robot：
        优先完成 vacancy-frontier Move
        随后优先 Drop

    fully constrained 时：
        跳过 heuristic generator
        直接调用 deterministic validator

    返回合法 physical successor 或 ⊥
```

生成 \(X'\) 后：

1. repair \(\tau_B'\)；
2. 重建 upper intent；
3. repair \(\rho_R'\)；
4. 将新 physical node 加入原 LaCAM OPEN/EXPLORED。

## 8. BRaP-Compatible Semantics

对 BRaP benchmark，loaded shelf 的 destination 必须在 timestep 开始时为空：

$$
v\in Empty_U(X_t).
$$

即使另一个 shelf 在同一步离开 \(v\)，当前 shelf 也不能 following 进入。这样，一空格实例仍然保持 sliding-puzzle 语义。

允许 convoy、following 和 full-cycle rotation 的显式物理模型可以保留，但必须作为不同 problem variant：

```text
transition_semantics = BRAP_NO_FOLLOWING
transition_semantics = PHYSICAL_ALLOW_FOLLOWING
```

两组结果不能混合汇报。

## 9. 对示例实例的实际处理

对于 `brap_h4w10_a5_e1_B_seed0`，五个 target shelves 全部使用完整 boundary goal set：

$$
\mathcal G_0=\cdots=\mathcal G_4=G_{\mathrm{boundary}}.
$$

root node 对五个 shelves 与所有 boundary cells 运行 rectangular Hungarian。它可以获得类似：

$$
0,0,0,1,1
$$

的全局 matching，而不会出现 sampled-goal greedy pairing 中最后一个 shelf 被迫移动 8 格的情况。

upper deck 只有 `(3,1)` 一个 vacancy。upper intent 从高优先级 target 向 blocker 递归，找到当前邻接 `(3,1)`、真正可以移动的 shelf。两个 robots 再通过 \(\rho_R\) 被派往当前 frontier tasks。

每次 lift、loaded move 或 drop 后，系统重新计算：

$$
X'\rightarrow\tau_B'\rightarrow
\mathcal I'\rightarrow\rho_R'.
$$

因此 shelf 4 不会被永久绑定到 `(0,9)`；shelf 0 和 shelf 1 即使已经位于 boundary goal，也可以临时被搬开，并由后续 matching 决定最终回到哪个 boundary cell。

## 10. Completeness

完整性只依赖 physical search：

1. physical configuration space 有限；
2. operator-constraint tree 最终枚举所有 robot primitive joint actions；
3. fully constrained branch 由 validator 精确判定；
4. \(\tau_B\)、upper intent、\(\rho_R\) 和 PIBT 只改变 successor ordering；
5. terminal test 不依赖 temporary matching；
6. `EXPLORED` key 只包含 physical \(X\)。

因此，即使 matching 或 intent 给出很差的建议，也不会删除合法 solution。

## 11. 工程落点

必须保留唯一的：

```cpp
TAPFPlanner::solve()
```

并在现有 node 中增加：

```cpp
ShelfAssignmentState shelf_assignment;
ShelfIntent shelf_intent;
RobotTaskAssignmentState robot_assignment;
ShelfState shelf_state;
```

现有 assignment engine 应泛化后复用两次，而不是新写两套 planner：

```cpp
AssignmentState solve_or_repair(
    EntitySet entities,
    TargetSet targets,
    Eligibility allowed,
    CostProvider costs,
    AssignmentState* parent,
    ChangedRows rows,
    ChangedCols cols);
```

第一实例是 `target shelf → candidate goal`；第二实例是 `free robot → manipulation task`。

零 shelf 实例中，shelf assignment 和 intent 都是空结构，原 TAPF agent-goal assignment 与 PIBT 自然保持原行为，不允许通过 feature flag、legacy planner 或 fallback 切换。
