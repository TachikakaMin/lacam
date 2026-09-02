# Carrier-LaCAM 最终设计文档：闭环货架目标与机器人任务联合搜索

状态：v3.0，2026-09-01。

- 设计基准：`new.md`（draft v3.0）——重写 guidance 层，让 shelf goal
  与 robot task 在每个搜索节点互相反馈；
- 实现基准：当前 `lacam/src`——v2.2 生产骨架 + 已落地的 v3.0
  step 1-3 与两轮 review 修复（R1-R7，见 §12 与 `debug.md` §10）；
  回归规模随修复批增长（2026-09-02 时点：C++ 158、Python 75，
  以 `debug.md` §7 与最新 commit 为准），当前 head gate =
  `results_v3_r2_taskexec`（严格 10s 协议，35/68）；
- 写法约定：每个机制先给 **[设计]**（v3.0 目标语义），必要时再给
  **[现状]**（当前代码的真实行为与落点）与 **[落差]**（落地时必须补齐
  的差异）。未标注的段落表示设计与现状一致、已实现。

本文取代 v2.2。v2.2 已验证的搜索骨架、输出修补、两遍流程、实验协议与
基线数字全部保留；被 v3.0 重写的是 guidance 的中间表示与 assignment
语义（§3-§7）。发现过程与被证伪路线见 `report.md`。

> **状态声明**：[设计] 段是目标语义，未实现前不代表已验证行为；已验证
> 行为 = [现状] 段 + §11.5 基线数字。落地进度由 §12 测试表与
> `debug.md` §7.2 追踪。
>
> **落地更新（2026-09-01）**：v3.0 step 1-3（task 池 + TaskId 绑定
> rho、frontier 编译器含 one-empty ready 规则、execution-price
> `tau_guide` + custody + 惰性 rewire 重建）已实现并通过 gate；当前
> head 结果 `results_v3_step3_price`（36/68、小图 36/36、vs baseline
> 共同成功集 mk 几何比 0.908435）。§3-§7 的 [现状] 段描述 v2.2 起点
> （保留作实现对照），各步落地细节与逐例披露见 §12。

---

## 0. 核心思想

v2.2 已证明并继续成立的骨架：

1. 在完整 robot-shelf 物理状态上做 configuration search；robot 是唯一
   actuator，shelf 运动是 `Move/Lift/Drop` 的确定性 effect；
2. 一切 assignment / path / priority 只负责排序：不进状态 key、不改
   goal condition、不永久删除合法 successor；
3. 多 goal 输入在首解后固定终态 assignment，用剩余 deadline 从根自动
   重跑一次，返回较低 SOC；
4. 输出端做可证明正确的等价类修补（exact state + shelf projection）；
5. 生产无策略开关，机制由输入规模与运行状态自动触发。

v3.0 重写 guidance 的内容：

> 不先固定 shelf goal，再让 robot 被动执行。对每个候选 shelf goal，
> 先预测它会产生什么搬运任务、当前 robots 执行这些任务有多难，再选择
> goal。

```text
physical state X
  -> 为每个 shelf-goal 候选预测当前任务（compile_frontier_task）
  -> 估计 robot 执行代价
  -> tau_guide：target shelf -> goal
  -> 生成正式 manipulation tasks（build_tasks）
  -> rho：free robot -> task
  -> Carrier-PIBT 生成 joint action
  -> X'
  -> 重新评价
```

这是 **ITA-inspired** 的反馈闭环——只借用"assignment 随执行状态反馈"
的思想：robot 的实际位置、vacancy 和 blocker 变化会改变 shelf 选哪个
goal；goal 改变后，生成的 task 也随之改变。不声称等价于 ITA-ECBS：
后者把 target assignment 与真实路径/冲突约束交织求解并带
bounded-suboptimal 理论，本设计的反馈是单轮 ordering-only price
repair（§5.1），无最优性保证。

**[现状]** 当前反馈只有半环：`build_guidance` 每个 node 都按最新
robot/vacancy 重造 requests 和 `rho`，但 shelf->goal 的选择
（`solve_tau`）只读 shelf 侧 admissible 下界矩阵，且在非任务边界节点
直接复用 parent assignment（§4）。robot 执行难度不影响 goal 选择——
这正是 v3.0 要补的另外半环。

---

## 1. 搜索状态与终止条件 `[保留，已实现]`

### 1.1 输入

- 4 连通 grid 和 wall；
- labeled robots；
- labeled target shelves 和匿名 shelves；
- 每个 target shelf 的 eligible goal 集 `G_b`；固定 goal 是
  `|G_b| = 1` 的自然特例。

loader 在 `DDInstance::finalize()` 中检查 goal 可达性（wall 连通分量
过滤）和覆盖 matching。它不把 goal pool 静态固化为一组 shelf-goal
pair。

### 1.2 状态

```text
X = (Q_robot, Q_target, Q_anon, kappa)
```

- `Q_robot`：labeled robot 位置；
- `Q_target`：labeled target shelf 位置；
- `Q_anon`：排序后的匿名 shelf occupancy（relabeling 已 canonicalize）；
- `kappa[i]`：robot `i` 当前 free、携带匿名 shelf，或携带 target `b`。

搜索 key 是 `SearchKey{Config, ShelfState}`。`tau_guide`、`rho`、task、
path 和 priority 都不进入 state key。

### 1.3 动作与合法性

每个 robot 每拍选择：

```text
Wait | Move(neighbor) | Lift | Drop
```

`apply_ops()`（`dd_carrier.cpp`）是 C++ 权威转移 oracle，检查
lower-deck vertex/swap 冲突、upper-deck 唯一占据、lift/drop
precondition 和 carrying 不变量。Python `ddbench.validator` 对输出计划
再次整体重放。

默认物理语义允许 following。BRaP-conservative no-following 仅通过
`apply_ops(..., allow_following=false)` 作为显式测试 oracle 参数存在，
不是生产开关。

### 1.4 Goal 与代价

```text
is_goal(X) iff
  every target b is grounded and Q_target[b] in G_b
```

upper-deck shelf 不能同格，终态位置本身构成 injective matching，
terminal check 不读取任何临时 assignment；已位于合法 goal 的 shelf
仍可被再次搬开。

物理 SOC：

```text
alpha * loaded_moves
+ beta  * free_moves
+ gamma * lift_drop
+ delta * anon_moves
```

四个权重默认为 1，是数值 objective 输入（`DD_ALPHA..DD_DELTA`），不是
搜索策略开关。

---

## 2. 搜索不变量 `[保留并扩展]`

1. 唯一生产 solve loop 是 `TAPFPlanner::solve()`。
2. `dd_planner.cpp` 只做 carrier 适配、两遍驱动、基线和输出修补，
   不搜索。
3. action-constraint tree 最终可枚举全部 robot primitive operator。
4. fully constrained joint op 由 `apply_ops()` 独裁，guidance 不决定
   合法性。
5. `tau_guide / task / rho / path / park / cooldown` 只改变候选顺序。
6. 零 shelf 输入自然退化为原 TAPF 路径（carrier 数据结构为空，逐位
   一致，不做模式检测）。
7. singleton goal-set 自然退化为固定 goal。
8. **[v3.0]** `tau_guide` 与 `tau_LB` 分离：execution feedback、
   hysteresis、taboo 只进 guidance，永不进入 admissible `h`。
9. **[v3.0]** task 完成条件只读物理状态与被指派 robot 的 custody
   episode（匿名 shelf 的等价类语义见 §3.1），不读其他 assignment，
   也不进入 search key。

这些不变量保留有限状态空间上的可行性 completeness 骨架。生产入口在
有限 deadline 下停止于首个 incumbent（每遍 `stop_at_first`），因此
**不声称首解最优或 eventually optimal**。

---

## 3. 中间层：Manipulation Task `[重写]`

### 3.1 设计

`request = {cell, priority}` 信息不足：它只告诉 robot 去哪里 Lift，
没有说明搬哪个 shelf、搬到哪里、为哪个 target goal 服务。

新 task 表示一次明确的 shelf 状态变化：

```text
MoveShelf(s, from -> to, root = target b -> goal g)
```

- `s`：要搬的 target 或 anonymous shelf；
- `from -> to`：这次搬运的精确起点和落点；
- `root = b -> g`：这次搬运最终服务于哪个 shelf-goal objective；
- 完成条件：`s` grounded at `to`（labeled target 为纯物理谓词；匿名
  shelf 见下方 custody 语义）。

**匿名货架的 `s` 与完成判定**：`Q_anon` 只保存排序后的占用集合（对称
性消除），物理状态里没有匿名 shelf 身份，单看某个 X 无法判定"是不是
原来那个 s"。因此匿名 task 的 `s` 用**当前 cell 标识**（等价类语义，
与 futile-lift memory 的 cell-keyed 匿名身份一致），custody 由
guidance 层记账：被指派 robot 在 `from` Lift 后进入 `kappa = ANON`
episode，该 episode 的 Drop 落点是否为 `to` 决定 task 完成/失效。
custody token 只存在于 task metadata——**禁止给物理状态增加匿名
身份**，否则破坏 `Q_anon` canonicalization 的对称性消除。

robot 被分配的是 task（稳定 `TaskId`），不是 pickup cell：

```text
free                 -> 去 from
到达 from             -> Lift
carrying task shelf   -> 去 to
到达 to               -> Drop，task 完成
```

`free_goal`、`target_next`、`parking_cell` 只是同一 task 在不同阶段的
局部 waypoint，不再是三个独立目标。

在 one-empty sliding-puzzle 中，ready task 必须在生成时就是：

```text
MoveShelf(s, u -> current_empty_cell)
```

不能只生成 `clear(u)` 后由 carrier 在 Lift 之后临时找停车点。

### 3.2 现状

`CarrierGuidance`（`tapf_planner.hpp` / `carrier_guidance.hpp`）：

- `requests = {cell, priority}`：serve 100（target 自己的 pickup），
  clear `50-k`（该 target least-blocking path 上前 `CLEAR_CHAIN_K=3`
  个 blocker cell）；
- `rho[i]` 指向 request **index**；request 每 node 重造，无稳定身份；
- robot 目标分散在三个字段：`free_goal[i]`（接受的 request cell）、
  `target_next[b]`（carried target 的下一格）、`parking_cell[i]`
  （匿名/parked carrier 用 BFS 自选的最近合法停车格）；
- request 不携带 shelf 身份、落点和 root objective；落点是 Lift 之后
  才由 carrier 决定的。

### 3.3 落差

- 新增 `ManipulationTask` / `TaskId` 与 `build_tasks(X, tau_guide)`；
- request 语义升级为 `MoveShelf(s, from -> to, root)`；
- 三个 waypoint 字段降级为同一 task 的阶段派生缓存；
- one-empty ready task 在编译期确定落点为当前 vacancy。

---

## 4. Shelf goal assignment：`tau_guide` `[重写]`

### 4.1 设计

对每个候选 `(b, g)`，先预测为了让 target shelf `b` 朝 `g` 前进，当前
最先需要执行的 task：

```text
m_X(b, g) = compile_frontier_task(X, b, g)
```

例如，`b` 的下一格被 blocker `s` 占据，当前 vacancy 是 `e`：

```text
m_X(b, g) = MoveShelf(s, position(s) -> e, root = b -> g)
```

该示例只在 `s` 的 carry 路径畅通（多空位）或 `s` 与 `e` 相邻时直接
可执行。正式定义必须覆盖 one-empty：

```text
compile_frontier_task(X, b, g):
  P = b 在 (X, g) 下的 least-blocking 路径
  if P 头部无 blocker 且 b 可推进:
      return MoveShelf(b, position(b) -> 推进落点, root = b -> g)
  s* = P 上第一个 blocker
  if 存在 vacancy e 使 s* 的 carry 路径可行（含 s* 与 e 相邻）:
      return MoveShelf(s*, position(s*) -> e, root = b -> g)
  else:  # one-empty / vacancy 需要先被"倒"到 s* 旁
      沿 vacancy 到 s* 的 routing 路径取第一个与当前 vacancy 相邻的
      shelf s'
      return MoveShelf(s', position(s') -> current_vacancy,
                       root = b -> g)
```

物理依据：carry 途中载货 robot 只能进入 upper 空格（S1）；one-empty
时可用 upper 空格只有 {lift 格, vacancy}，可执行的 shelf 移动恰是
"与 vacancy 相邻的 shelf 移入 vacancy"（15-puzzle 语义）。因此 ready
task 永远是可执行的一步 shelf 状态变化，不会编译出无法启动的搬运。
**dependency depth** = 从 root objective 到该 task 的编译链深度，作为
`rho` 的优先级输入（§5.1）。**重编译触发**：vacancy 移动、路径失效、
`tau_guide` 改选 root、task 完成或 custody 失效。

然后计算 execution-aware guidance cost：

```text
C_guide(X, b, g)
  = shelf rearrangement estimate      （距离、blocker chain、vacancy routing）
  + lambda * robot realization estimate
       （当前 robots 到 m_X(b,g) 的 pickup、Lift、carry、Drop 代价）
  + goal-switch hysteresis            （新 goal 必须明显更好才替换）
```

用该 cost 求 injective matching：

```text
tau_guide : target shelf -> eligible goal
```

因此，即使 target shelf 没移动，只要 robot、vacancy 或 blocker 改变，
`tau_guide` 也可能改变；它不能只按 moved target rows 更新。

例：左侧 goal 距离更近，但要清三个 blocker 且 robots 都远；右侧 goal
多一格，但 robot 已在 shelf 下方。右侧 end-to-end cost 更低，
`tau_guide` 应改选右侧，并生成不同 task。

### 4.2 现状

`solve_tau`（`carrier_guidance.hpp`）一次求解输出唯一 `tau`：

- 主序 = admissible shelf-goal 下界矩阵
  `lb(b,g) = alpha * upper_wall_dist(pos_b, g) + gamma * (1|2)`；
  其 **unrestricted matching 最优值同时就是 `h_shelf`**——这正是 v3.0
  的 `tau_LB` 值（§8）；
- 次序 = `eta_B = 2` 的 parent-assignment hysteresis（lexicographic
  编码 `lb*S + pen`，`S = eta_B*n + 1`，读 h 时整除剥离）；
- settled lock：grounded 在任意 eligible goal 的 target 优先锁住当前
  cell；carried lock：in-flight goal 保留。两级不可扩展 fallback：先
  释放 carried locks，再只保留 unrestricted matching 见证可扩展的
  settled 行；
- livelock repair：对 rho pair 加 taboo，并且每个 livelock epoch 只
  释放**一个** unfinished、grounded、multi-goal 行的当前 pair；
- `targets > ASSIGNMENT_EXACT_LIMIT = 256` 时退化为 row-wise nearest
  eligible（h 忽略 injectivity，仍 admissible）。该 regime 的 guidance
  **不再是单射 matching**——多个 shelf 可指向同一 goal（语义上是
  `tau_hint`），冲突最终由 terminal 的 upper-deck 独占性解决；当前
  68 例 gate 没有 >256 targets 的实例，该 regime 仅有单元测试覆盖
  （§13）。

重算时机（`attach_carrier_guidance`）：root、target drop boundary、
reguide；其余节点 `preserve_parent` 复用 parent assignment（合法性
检查通过时），h 用 row-relaxed 下界。

### 4.3 落差

- cost 没有 robot realization 项：robots 全部远离某 goal 的 blocker
  链不影响该 goal 的得分；
- `tau_guide` 与 `tau_LB` 是同一次求解的两个读数（同一 lb 矩阵），
  尚未拆成两个公式；拆分后 `tau_LB` 保持现状（可证明下界），guidance
  侧换 `C_guide`；
- 非边界节点复用 parent assignment，robot/vacancy 变化无法触发改选；
  v3.0 要求每个 physical node 重新评价（用 hysteresis 防震荡）；
- `compile_frontier_task` 不存在——现状最接近的是 path 上的
  clear requests，但它们不带落点与 root。

---

## 5. Robot task assignment：`rho` `[重写]`

### 5.1 设计

根据最终 `tau_guide` 构造当前 task pool `M(X, tau_guide)`，再求：

```text
rho : free robot -> ManipulationTask or IDLE
```

匹配代价：

```text
robot 到 task.from 的 lower-deck distance
+ task priority / dependency depth
+ task-switch hysteresis（按 TaskId 判定）
```

`rho` 延续稳定 `TaskId`，不是当前 pickup cell。loaded robot 不参与
`rho`（`kappa` 已给出 physical binding）。

为考虑多个 tasks 争抢同一 robot，每个 node 做一轮轻量反馈：

```text
1. 初步计算 tau^0
2. 按 tau^0 生成 tasks，求全局 rho^0
3. 给无人可接、竞争严重或执行昂贵的 root objective 加 execution price
4. repair 得到 tau^1
5. 用 tau^1 生成最终 tasks 和 rho
```

只负责排序，不要求精确求解或迭代到收敛。

### 5.2 现状

`build_guidance` 中 `rho: free robot -> request index`：

- `targets <= 256` 且有 free robot 时，对按 priority 排序的前
  `|free|` 个 request 做 Hungarian（共享 `tapf_hungarian_row_to_col`；
  cost = lower-deck distance；parent 同 cell 给 `eta = 2` 折扣；taboo
  行 `INT_MAX/8`）；更大规模贪心最近；
- loaded robot 不参与（与设计一致）；
- hysteresis 按 `free_goal` **cell** 对齐，不是 TaskId；request index
  每 node 重排；
- 无 execution-price 反馈轮：tau 在前、rho 在后、单向一遍。

### 5.3 落差

`rho` 改绑 `TaskId`；task-switch hysteresis 按 TaskId 判定；新增
`tau^0 -> rho^0 -> execution price -> tau^1` 的单轮 repair
（ordering-only）。

---

## 6. Feedback 与 commitment `[重写]`

### 6.1 设计

反馈不等于每拍随意换目标：

- `tau_guide` 在每个 physical node 重新评价；
- 尚未被接手的 task 可以随 `tau_guide` 立即改变；
- robot 接手一个 `MoveShelf(from -> to)` 后，对该局部 task 使用 soft
  commitment，直到完成或 task 失效；
- Lift 后 `kappa` 是唯一 hard commitment；
- 当前 task 完成后，再按新的 `X` 和 `tau_guide` 生成下一 task；
- carried target 的 root goal 使用较强 hysteresis，避免左右震荡，但
  不作为 hard pruning；
- duplicate node 被 rewire 到新 parent 后，必须重建 hysteresis
  anchor、task 和 `rho`，不能保留旧 parent 的 guidance。

### 6.2 现状

对应机制已存在但绑定在 request/cell 语义上：

- tau 的 task-episode commitment：`preserve_parent` 复用 + drop
  boundary 重算。代表 probe 曾把 20x20 的 shelf-goal pair 改写从
  86,317 降到 2,757，最大完成数 27/40 -> 34/40（见 `report.md`）；
- carried target 保持 in-flight goal（solve_tau carried lock）；
- settled lock 与两级可扩展性 fallback（§4.2）；
- livelock 每 epoch 释放一行；
- rho hysteresis `eta = 2`（cell 对齐）。

### 6.3 落差

- duplicate hit 经 `rewrite()` 换 parent 后，节点 guidance 与
  hysteresis anchor **不重建**——现状只在该节点每 8 次 revisit 时
  reguide 一次（以自身旧 guide 为 hysteresis 源）。v3.0 要求 rewire
  即重建；
- "未接手 task 随 `tau_guide` 立即改变"依赖 task 有稳定身份，现状
  request 每 node 重造，无法表达；
- soft commitment 的失效判定（task 失效条件）需在 `build_tasks` 中
  定义。

---

## 7. Carrier-PIBT `[保留框架，改 guidance 来源]`

### 7.1 设计

`funcPIBT` 不再分别读取独立的 `free_goal`、`target_next` 和
`parking_cell`，而是从 `rho[r]` 的 task 阶段推导 operator 顺序：

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

PIBT 处理 lower-deck robot conflicts；task dependency 决定优先级。
action-constraint tree 仍最终枚举所有 primitive joint actions，
`apply_ops()` 仍是合法性终裁。

### 7.2 现状

`funcPIBT` 的候选表已按角色结构化，形状与设计一致，只是 waypoint 来源
是三个独立字段：

| 角色 | 现状候选顺序 |
|---|---|
| loaded、target 未 park | `target_next` 头格 > 按 goal 距离排序的 S1 可行 Move > （头格被 grounded shelf 结构性阻塞时 Drop 提前作 yield）> Wait > Drop |
| loaded、匿名或 parked | 到 `parking_cell` 的 Move（S1 过滤）> Wait > Drop；到达停车格则 Drop 优先 |
| free、有 `rho` | 在 `free_goal` 且格上有 shelf 时 Lift 优先（futile cooldown 时降到 Wait 之后）> 朝 `free_goal` 的 Move > Wait |
| free、IDLE | 在被保护走廊上先离开（protect 排序），否则 Wait 优先 |

其余保留：类分层 PIBT order（loaded-target 未 park > loaded-anon/
parked > free-assigned > idle，类内按剩余距离，stable）；carrier 角色
失败释放-重试的预约语义与可行 wait fallback；B1 `plan_bound` 硬约束
分支；LaCAM2 swap 只作用于 task agents。

### 7.3 落差

waypoint 改由 task 阶段推导；per-role 距离排序统一成
`score(op) = task 剩余代价`（现状是它的特例）。行为语义（S1 过滤、
yield-drop、futile 降序、失败释放重试）全部保留。

---

## 8. Cost 与正确性

### 8.1 两种 matching、三种量

```text
tau_guide  -> 含 blocking、vacancy、robot feedback、hysteresis、taboo，
              只用于 guidance
tau_LB     -> 只含可证明 lower bound（upper-deck wall distance
              + lift/drop 计数），只用于 admissible h
```

```text
g = 已执行的真实 physical cost（get_edge_cost，按 alpha..delta 记账）
h = admissible lower bound（tau_LB matching 最优值）
guidance cost = 可以非 admissible，只决定 successor 顺序
```

robot realization cost、execution price、hysteresis 和 taboo 不能进入
`tau_LB`。

**[现状]** "值"的一半已经满足分离：`solve_tau` 的 h 取 unrestricted
LB matching 最优——hysteresis 经 lexicographic 编码整除剥离；taboo 行
h 回退 unrestricted 行最小；`preserve_parent` 路径 h 用 row-relaxed
下界。由测试钉住：`dd_tau.h_equals_bruteforce_min_matching`、
`dd_tau.hysteresis_is_tie_break_only`、
`dd_tau.rowwise_taboo_does_not_bias_admissible_h`、
`dd_anytime.admissible_h_never_exceeds_true_cost`。
落差在"式"的一半：guidance 还没有独立的 `C_guide`。落地时新增的
robot 项只能进 `C_guide/rho`。

**纯性口径**：`tau_LB`/h 是 X 的纯函数（只经 wall-distance 缓存）。
`C_guide` 与 paths/park/requests 允许依赖 guidance 引擎的缓存状态：
`PathCache` 的 path-local invalidation 在占用减少时沿用旧路径，
`dd_park_purity` 测试钉住了该 (X, cache epoch) 语义。因此 §4.1 的
"每个 physical node 重新评价"定义在 **(X, 引擎状态)** 上——同一 X
经不同搜索轨迹可得到不同 guidance。这不影响合法性、完备性与 h，给定
seed 仍逐位可复现；strict snapshot 只是测试探针，生产不为数学纯性
付该成本。duplicate rewire 后的 guidance 重建（§6.1）同样按此口径。

### 8.2 Completeness 论证 `[保留]`

完整性保持不变，因为：

1. state key 只含 physical `X`；
2. `tau_guide`、task 和 `rho` 都是 ordering-only；
3. action-constraint tree 最终枚举全部 robot primitive operators
   （futile-lift 只降序不删除；`constraint_order` 创建后冻结，
   reguide 只扰动 PIBT preference）；
4. fully constrained joint action 由 `apply_ops()` 独立裁决；
5. terminal condition 不依赖临时 assignment。

错误的 guidance 最多让搜索变慢，不会删除合法解。生产入口有限
deadline 首解即停，不声称最优。

### 8.3 输出修补正确性 `[保留，已实现]`

`repair_carrier_plan()`（`dd_plan_repair.cpp`）对每遍候选先用 oracle
重放得到 `X_0 .. X_T`，记录每个完整状态和每个全 grounded shelf
projection 的最后出现位置，然后单遍扫描：

```text
at state X_t:
  if the exact physical state appears again at u > t:
      jump to u
  else if all shelves grounded and P_shelf(X_t) reappears at u > t:
      find a lower-deck-only robot bridge from Q_robot(t) to Q_robot(u)
      if bridge length < u - t: emit bridge and jump to u
  else:
      emit original action t
```

其中 `P_shelf(X) = (Q_target, Q_anon)`。bridge 策略：

- 1 robot：wall-aware shortest path；
- 2 robots：labeled two-robot configuration graph 上 exact A*
  （启发 = 两个独立 wall 距离的 max，admissible）；
- 更多 robots：原片段投影到 lower deck 并去除重复 robot
  configuration，必然合法的候选 bridge。

**精确状态剪切**：若 `X_t = X_u`，原后缀在相同状态上必然合法，删除
`[t,u)` 不改变终态。

**Projection 剪切**：仅当两端点全部 shelves grounded 且
`P_shelf(X_t) = P_shelf(X_u)`。bridge 只含 lower-deck `Move/Wait`：
(1) grounded shelf 不随 bridge 改变；(2) bridge 终点 robot
configuration 等于 `Q_robot(X_u)`；(3) 因此 bridge 后完整状态恰为
`X_u`；(4) 原后缀继续合法。这是状态等价修补，非近似语义。

**防御性保证**：修补只在 bridge 严格更短**且 bridge 加权 SOC 不高于
被替换段**时采用（2026-09-01 review 修复：生产候选选择按 SOC，修补
不得为缩短步数抬高 SOC；bridge 全部为 free move，成本 `beta * 移动
数`，被替换段成本由前缀和 O(1) 查询）；最后从初态用 `apply_ops()`
整体重放并检查 goal 与总 SOC 不增，任何失败、非缩短或异常都返回原
计划。因此：

```text
valid(raw) => valid(returned)
length(returned) <= length(raw)
soc(returned)    <= soc(raw)
goal(returned) = true
```

输出修补不参与搜索，不影响 completeness。

---

## 9. 生产流程与自动机制 `[保留，已实现]`

v3.0 落地时本节机制全部保留；tau 相关条目按 §4-§6 的 task 语义替换
实现，自动规则（规模分级、hysteresis、重算边界）继续有效。

### 9.1 首解与固定 assignment 重跑

`solve_carrier_lacam`（`dd_planner.cpp`）：

```text
use_macro = (number of targets <= 64)
plan1 = repair(first_incumbent_search(dynamic tau, use_macro, deadline))
if no plan1:
    return failure
if every goal set is singleton or deadline expired:
    return plan1
fixed_goal[b] = terminal_position(plan1, target b)
plan2 = repair(first_incumbent_search(fixed_goal, use_macro, remaining))
return lower_SOC(plan1, plan2)  // plan1 on failure/tie
```

两遍共享同一个 10 秒总 planning deadline（不是每遍各 10 秒）。第二遍
只在首解存在、多 goal assignment 可固定且仍有时间时自动触发；
singleton/R1 结构性跳过。固定 goal 是原 eligible set 的子集，第二遍
计划对原实例仍合法；第二遍失败、持平或更差都返回第一遍。

每遍内部 `stop_at_first`；deadline 预留
`min(1000ms, max(100ms, 10%))` 作 cleanup reserve。macro rollout 最多
64 步、只在每遍首解前注入（`macro_after_first` 恒 0）；rollout 每步
清除 address-keyed scratch，每 8 步重建 guidance，中间复用同一
guidance——逐拍重建在 dense rollout 上已验证造成成功率回归，刷新周期
固定在实现内部。

### 9.2 自动 guidance

所有策略由输入规模或当前状态决定：

| 机制 | 自动规则 |
|---|---|
| shelf-goal matching | target 数 <= 256 时 exact Hungarian；更大时 row-wise admissible relaxation |
| active targets | unfinished <= 256 时全量；否则 carried 优先并 cap 64 |
| `rho` | target 数 <= 256 时 Hungarian；更大时 nearest greedy |
| hysteresis | tau/rho 固定 tie preference 2 |
| macro | target 数 <= 64 时启用，每个 search pass 首解后不可达 |
| assignment restart | 多 goal 首解终态 + 剩余 deadline 时一次 |
| path cache | 生产固定 path-local invalidation；strict snapshot 仅测试构造参数 |
| idle avoidance | active path 上的 idle robot 自动先避让 |
| carrier yield | 对头时自动让剩余距离更大的 carrier park |
| livelock | 24 步无 h_guidance 进展或每 8 次 revisit 时定向 taboo repair |

shelf-goal matching 的主序是 admissible 下界（v3.0 落地后主序换
`C_guide`，`tau_LB` 继续供 h），parent assignment 只在 tie 内提供
稳定性。goal 改变时 `PathCache` 的 `dst` 参与 cache key，不会沿用指向
旧 goal 的路径。

matching 只在 drop/task boundary 或定向 livelock repair 时重新求解；
普通 robot motion 和 loaded motion 复用 parent assignment（v3.0 改为
每 node 重评 `C_guide`，见 §4.3）。carried target 保留 in-flight
goal。grounded target 已位于 eligible goal 时优先锁住当前 cell；若这
组锁不能扩展成完整 matching，则先释放 carried commitment，再只保留
unrestricted matching 可证明可扩展的 settled locks。

必须区分 terminal 与 guidance："done" 的局部判断是
`position == assigned goal`，因为不可扩展的 settled placement 仍可能
必须重开；terminal 判断任意 eligible membership。completed target 在
path cost 中仍是普通、可逆 blocker——把它设成支配任意绕路的特殊障碍
曾使一个 10x10 singleton 案例停在 11/12、正式 gate 从 36/68 降为
35/68，该方案不进入生产。

### 9.3 自动 futile-lift memory

搜索和 macro rollout 都观察三状态模式：

```text
grounded at cell c -> lifted at c -> grounded at c
```

同一 shelf/cell 在近期窗口内重复 >= 3 次时，下一次 `Lift` 被移到
`Wait/Move` 之后。窗口由图规模自动取 `max(64, 8*|V|)`；匿名 shelf 按
cell 共享身份。只降序、不删除 `Lift`，不破坏 constraint-tree 枚举。
更长的 hover episode 不在生成侧禁止——两轮实验都显示禁止会令拥挤
实例超时（"悬停抬放是给上层洗牌的承重梁"，见 `report.md`）。

### 9.4 输出修补

见 §8.3。修补统计由 `DDStats` 暴露：

```text
exact_loops, projected_loops, bridge_steps, plan_steps_removed,
futile_lift_demotions, assignment_restarts, assignment_second_solved,
assignment_improvements, assignment_{first,second}_{soc,makespan}
```

失败分类（2026-09-01 review 修复）：`timed_out` 仅在某遍搜索确实到达
deadline 时置位；OPEN 耗尽与 generator failure 的空计划报告为普通
失败（与 B0/B1 的 stuck/cycle 语义一致），benchmark 状态列相应区分
`timeout` 与 `failed`。

---

## 10. 工程落点

唯一 solve loop 仍是 `TAPFPlanner::solve()`。不新增平行 planner、
第二套 search pipeline 或 feature-flag fallback。

| 文件 | 现状职责 | v3.0 新增职责 |
|---|---|---|
| `lacam/src/carrier_guidance.hpp` | `solve_tau`（matching + h）、least-blocking paths、requests、`rho`、park/yield、parking、自动规模分级 | 分离 `tau_LB` 与 `tau_guide`；`compile_frontier_task(X, b, g)`；`build_tasks(X, tau_guide)`；`rho: robot -> TaskId`；execution-price repair |
| `lacam/src/tapf_planner.cpp` | 唯一搜索 loop、Carrier-PIBT、macro、futile-lift、livelock、`attach_carrier_guidance` | `funcPIBT` 从 task 推导 waypoint/operator order；rewire 后重建 guidance anchor |
| `lacam/src/dd_plan_repair.cpp` | 精确剪圈、shelf-projection bridge、最终重放 | 不变 |
| `lacam/src/dd_planner.cpp` | 共享 deadline 两遍、assignment 固定、B0/B1、统计映射、测试探针 | 不变（探针随新接口扩展） |
| `lacam/src/dd_carrier.cpp` | 物理 oracle 与 goal condition | 不变（保持权威） |
| `benchmark/ddbench/validator.py` | 独立 Python 输出验证 | 不变 |

`free_goal` 可以作为缓存保留，但必须由 task 推导，不能再代表真实
assignment。现有 anytime restart（fixed-assignment 第二遍）、输出修补
和 validator-first 流程保持不变。

零 shelf 输入时，carrier 数据结构自然为空，原 LaCAM-TAPF 路径逐位
退化，不通过模式检测切换。

### 10.1 环境输入（无策略开关）

生产源码中的环境输入仅有：

| 输入 | 用途 |
|---|---|
| `DD_ALPHA` | loaded-move objective weight |
| `DD_BETA` | free-move objective weight |
| `DD_GAMMA` | lift/drop objective weight |
| `DD_DELTA` | anonymous-shelf move objective weight |
| `DD_DEBUG_DUMP` | 失败诊断输出 |

已删除且不得回流的策略入口：milestone、OPEN greedy、`h_guidance`
in `f`、lift gate、loop erase toggle、macro knobs、guidance frequency
knob、tau freeze、frontier bonus、parking escape、yield toggle、idle
toggle、strict invalidation env、Hungarian/greedy env boundary、
hysteresis env。v3.0 的 task 层同样不得引入布尔/枚举开关；`lambda`
等新参数若要暴露，只能作为数值 objective 类输入并先过协议评审。

### 10.2 诊断统计

现有 `DDStats`：搜索/生成计数、`tau_change_builds / tau_pair_changes /
rho_change_builds / rho_pair_changes`、`tau_time_ms /
guidance_time_ms`、path cache、macro/rollout、修补与两遍统计（§9.4）。
v3.0 落地需新增 task 层诊断（命名落地时定）：task 编译次数、未接手
task 改写次数、soft-commitment 中断次数、execution-price repair 触发
次数。

---

## 11. 实验规范与基线 benchmark `[保留]`

### 11.1 不可变实验协议

算法、优化和文档结论必须遵守：

1. **实例不变**：比较实验复用完全相同的 YAML 字节、目录划分和 seed；
   不得重新采样 start、empty、target、goal pool 或 robot。
2. **时间不变**：每个 carrier testcase 使用严格 10 秒内部 deadline，
   **搜索、强制修补、SOC 计算与最终重放全部计入**（2026-09-02 R1：
   预算内完不成修补的 pass 诚实超时）。runner 的额外 wall allowance
   只用于进程启动、输出 IO 与 runner 侧独立验证；deadline 后的进程
   收尾（节点析构等）测得 <=0.7s，自 v2.2 起存在。
3. **并发不变**：统一 `jobs=14`（16 物理核留 2）。benchmark 运行期间
   不得并行运行测试或其他高负载任务。Python 全量回归也用 14 workers；
   CREST/MAWR 固定 10 秒内部时限。
4. **seed 不变**：solver seed 0；BRaP 每个 layout combo 两个实例 seed
   0/1。多 solver-seed 是新轴，单独报告。
5. **成功定义不变**：binary `solved=1` 不够；runner 必须用独立 Python
   validator 从初态重放每个 joint action 并检查最终 goal；timeout、
   空计划、非法动作、非 goal 终态都是失败。
6. **物理语义不混用**：默认 following 与 BRaP-conservative
   no-following 结果分表；主结果只使用默认 following。
7. **objective 不混用**：主表固定单位权重。非单位
   `alpha/beta/gamma/delta` 是独立实验轴，必须显式
   `--weights ALPHA BETA GAMMA DELTA` 并记录到 `timing.json`；
   `run_benchmark.py` 会覆盖 shell 中残留的 `DD_*`。非单位轴不得与
   native-objective 外部方法同跑。
8. **产物可审计**：保存 `rows.csv`、`timing.json` 和每个成功实例的
   `.plan`；结果目录不得覆盖历史对照目录。

成功率与质量是两个问题，必须同时报告：success 用完整 suite 分母并按
尺寸/goal family 分层；makespan/SOC 只在共同成功实例比较；正值 ratio
用几何均值并报告逐例改善/持平/恶化；trivial plan 单独标识；不得只
摘录成功或改善案例。

统一指标至少包括：

```text
executed_makespan, weighted_soc, loaded_moves, free_moves, lift_drop,
shelf_switches, robot_utilization, reversals, first_solution_ms,
runtime_sec, status
```

### 11.2 方法与消融纪律

| 名称 | runner method | 唯一变化 |
|---|---|---|
| full | `carrier` | 动态首解 + fixed-assignment restart + mandatory repair |
| B0 | `carrier_b0` | rollout only，无 high-level search |
| B1 | `carrier_b1` | 根状态冻结 shelf path 的 decomposition baseline |
| B2 | `crest_base` / `crest_full` | 外部 decomposed executor |
| B3 | `natcbs` | 小图 makespan-optimal 外部基线 |
| B4 | `b4` | 单 robot sequential construction |

`full/B0/B1` 是结构差异，不是环境开关。`run_ablations.py` 与主 runner
共用 `row_carrier`：固定 10 秒、`jobs=14`、seed 0、单位权重、默认
following，Python validator 重放每个 success，保存统一指标、
`timing.json` 和成功计划。

**v3.0 核心消融只保留三组**（以结构 runner method 实现，不是环境
开关；已删除策略不得重新加入）：

```text
A. frozen tau vs feedback-aware tau_guide
B. request cell vs semantic ManipulationTask
C. shelf-only cost vs robot-realization-aware cost
```

外部基线保留各自原生 objective 和失败语义；不可比数字不合并成一个
均值。

### 11.3 固定 suite 与 gate

**语义/兼容 gate**

- shelf-free TAPF：`test_tapf_compat` 固定（零 shelf 逐位一致）；
- singleton goal：goal-set 单测和 fixed-goal fixtures 固定；
- transition：C++ `apply_ops`、Python validator、golden corpus、G1
  穷举一致；
- output repair：所有返回计划满足 oracle replay 且长度不增。

**BRaP-pool 主 gate**

- suite：`benchmark/instances_brap_pool`，68 例；
- 命名说明：这是 **BRaP-style 自采样回归集**（默认 following、完成
  target 可再搬、自有规模与采样），不是 BRaP 原基准的复现；其数字
  不得与 BRaP 文献横向对比，只用于本仓库内回归；
- 分层：`<=10x10` 36 例是当前可解质量域，`>=20x20` 32 例单独报告
  horizon；不得只报混合值而隐藏分层；
- 基线回归：success `36/68`、小图 `36/36`；
- 代表锚：`h4w10_a5_e1_R1_seed0 = 1053`，
  `h10w10_a12_e3_R1_seed1 = 2620`。

**Fixed-assignment restart gate**

- treatment `results_task_commit_final` vs control
  `results_rootfix_protocol_verify`；
- 第二遍和修补共享原 10 秒总 deadline；
- 动态 B-pool：18 attempts、18 second solved、6 strictly improved；
- final success `36/68`、小图 `36/36`；
- singleton/R1 输出计划逐字节不变。

**Rootfix 质量对照**

- treatment `results_rootfix_final` vs control `results_fix1_mscd`；
- 相同 pool suite、seed、10 秒、jobs、validator；
- 主统计为共同成功实例 makespan ratio；辅助统计 free moves、
  lift/drop、reversals、utilization。

### 11.4 复现命令

```sh
cmake --build build -j 16 --target dd_benchmark
python3 benchmark/run_benchmark.py \
  --instances benchmark/instances_brap_pool \
  --out-dir benchmark/results_task_commit_final_repro \
  --methods carrier --timeout 10 --jobs 14

python3 benchmark/run_ablations.py \
  --out-dir benchmark/results_ablation_rootfix_repro
```

运行后核对 `timing.json` 的 task 数、timeout、jobs、solver seed、
objective weights 和 following 语义；统计一律从结构化 CSV 计算，不从
console 截断文本抄数。

### 11.5 基线结果（v3.0 起点）

```text
结果目录                         solved   <=10x10   总 makespan(成功例)
results_fix1_mscd                  36       36          214,199
results_rootfix_final              36       36           35,595
results_fixed_assignment_restart   36       36           35,325
results_task_commit_final          36       36           34,860
```

- 相对 `results_fix1_mscd` 的 makespan 几何均值比 `0.204960`，
  改善/持平/恶化 33/1/2；
- `h4w10_a5_e1_R1_seed0`：`6,967 -> 1,053`（Python 投影原型曾达 719，
  bridge 仍有优化空间）；
- `h10w10_a12_e3_R1_seed1`：2,620；
- 20x20 及以上 32 例在 10 秒内 0/32。

相对 rootfix control：success/status 不变；18/18 动态 B-pool 成功例
触发并完成第二遍，6 选择第二遍、12 保留第一遍；36 共同成功例
makespan/SOC 几何比 `0.917520/0.927884`（B-pool 子集
`0.841843/0.857187`）；成功例总 makespan `35,595 -> 34,860`、SOC
`61,049 -> 59,907`；第二遍改善包括 `53->41`、`36->34`、`51->34`、
`622->519`、`78->77`、`241->168`。

逐例披露（协议要求，不得只报几何均值）：makespan 改善/持平/恶化为
**13/19/4**；SOC 几何均值覆盖 **35 个正值样本**
（`h10w10_a1_e1_B_seed1_pool` 是 0/0 的 trivial 例，按协议单列并从
比值中排除）。4 个恶化例全文列出：

```text
h6w10_a6_e15_B_seed1_pool    mk  60 ->   64  (+6.7%)   soc  +4.1%
h6w10_a6_e1_B_seed1_pool     mk 254 ->  271  (+6.7%)   soc  +5.1%
h8w10_a10_e20_B_seed0_pool   mk  75 ->   77  (+2.7%)   soc  +1.9%
h8w10_a10_e2_B_seed1_pool    mk 695 -> 1073 (+54.4%)   soc +59.9%
```

恶化与 13 个改善同源：task commitment 改变了动态 B-pool 的首遍轨迹，
最差例 `h8w10_a10_e2_B_seed1` 的首遍落入更长 incumbent，且其第二遍
候选按 lower_SOC 规则未被选中。单 solver-seed（0）协议对字节级回归
gate 是刻意设计；跨方法的**泛化/正式比较**需另开多 solver-seed 轴并
报告置信区间与 paired 检验（§11.1 第 4 条），当前结论仅限"固定协议
下的小图质量域（<=10x10）"。

边界敏感性：`h10w10_a12_e3_B_seed1_pool` 的第二遍在 8.9s/10s 贴线
完成且结果更差被丢弃；机器状态不同时该遍可能不完成
（`assignment_second_solved` 18 或 17，改动前后二进制在同一机器状态
下行为一致），最终计划不受影响。gate 的决定性判据是 68 行核心字段
零差异与 36 个成功 plan 逐字节一致，不是 second_solved 计数本身。

独立复跑 `results_rootfix_protocol_verify` 与
`results_rootfix{,_final}` 四个核心字段零差异、36 个成功 `.plan`
逐字节一致。`results_task_commit_final_verify` 与
`results_task_commit_final` 分别零差异、逐字节一致。结构消融
`results_ablation_rootfix_verify`（9 protected cases）：full 9/9、
B0 9/9、B1 5/9；full/B0 makespan 几何比 0.638179（5/3/1），full/B1
1.077651（1/3/1）。

v2.2 冻结时点回归：C++ 137/137；Python 69/69（14 workers，25.01
秒）。外部 baseline 测试 solver deadline 固定 10 秒。（此后 v3.0 与
review 修复批持续增长，当前规模见文档头部与 `debug.md` §7。）

### 11.6 v3.0 验收 gate

task/`tau_guide` 层落地时按 11.1 协议开新结果目录，并满足：

1. success 不低于基线：`36/68` 且小图 `36/36`（分母不变）；
2. 共同成功集 makespan/SOC 几何比与逐例改善/持平/恶化同时报告，
   恶化案例逐个解释；
3. 语义变更允许首遍轨迹改变——singleton/R1 逐字节等价只对声称
   "无行为变化"的重构强制；
4. 三组消融 A/B/C 在同协议下报告；
5. 每个成功计划过 C++ oracle 重放与 Python validator；
6. 每节点 guidance 预算复核：大图上 tau+guidance 已可消耗
   ~8.9/10 秒（80x80 历史测量），新增 robot realization 项必须给出
   增量/缓存实现并报告 `tau_time_ms / guidance_time_ms`。

---

## 12. 最小测试计划

必须测试（new.md §9；映射到现有测试或标注待写）：

| # | 要求 | 现状 |
|---|---|---|
| 1 | 零 shelf 与原 LaCAM-TAPF 逐位一致 | 已有：`test_tapf_compat`（deterministic/anytime/rng 轨道） |
| 2 | `\|G_b\|=1` 退化为 fixed-goal carrier | 已有：`dd_tau.singleton_degenerates_and_matches_root_h`、`dd_goalset.singleton_assignment_skips_second_search` |
| 3 | target 不动、robot/vacancy 改变时 `tau_guide` 可以改变 | 已覆盖：`dd_tasks.robot_placement_flips_tau_guide_goal` |
| 4 | 同一 `TaskId` 连续经历 approach、Lift、carry、Drop | 已覆盖：`dd_tasks.custody_keeps_task_id_from_lift_through_drop`（身份）+ `dd_tasks.one_empty_drop_lands_at_custody_task_to`（落点，R2) |
| 5 | one-empty 时 ready task 是相邻 shelf 移入 vacancy | 已覆盖：编译面 `dd_tasks.one_empty_ready_task_moves_vacancy_adjacent_shelf` + 执行面 `dd_tasks.one_empty_drop_lands_at_custody_task_to` |
| 6 | execution feedback 不进入 admissible `h` | 已覆盖：`dd_tasks.execution_price_never_enters_admissible_h`、`dd_tau.hysteresis_is_tie_break_only`、`dd_tau.rowwise_taboo_does_not_bias_admissible_h`、`dd_anytime.admissible_h_never_exceeds_true_cost` |
| 7 | 所有返回计划通过 C++/Python replay | 已有：`dd_plan_repair.*`、benchmark validator、golden corpus |

现有保护性测试（commitment/修补/搜索语义）继续有效，见 `debug.md`。

2026-09-01 review 修复批新增保护测试（RED->GREEN 已完成）：

```text
dd_goalset.finalize_rejects_duplicate_target_starts        重复 target start 拒绝
dd_weights.rejects_negative_and_non_finite_env_weights     权重输入校验
dd_planner.exhausted_search_is_not_reported_as_timeout     失败分类
dd_plan_repair.repaired_soc_never_exceeds_raw_under_any_weights  修补 SOC 契约
DuplicateTargetStartTest (Python)                          loader 对齐
```

v3.0 step 1（task 层，RED->GREEN 已落地）：`ManipulationTask` 池由
`build_guidance` 在原 serve/clear 生成点发射，request 是 task 的
pickup 投影，`rho_task` 绑定池索引，hysteresis 按 TaskId
（shelf, from, root）判定；保护测试
`dd_tasks.{serve_task_carries_shelf_root_and_projection,
clear_task_identifies_blocker_and_root,
task_id_stable_across_robot_motion,
rho_binds_task_and_requests_follow}`。step-1 gate
（`results_v3_step1_tasks`）：36/68、小图 36/36，共同成功集 mk/SOC
几何比 1.006988/1.005509（7/22/7），总 makespan 34,860 -> 35,029——
hysteresis 从 cell 视图收紧为 task 身份后，跨 root 共享 cell 的折扣
消失所致；作为 step 2/3 的基础被接受，最终 v3.0 与 baseline 的对比在
§11.6 验收时整体评估。

v3.0 step 2（frontier compiler，RED->GREEN 已落地）：clear 候选按
§4.1 编译为可执行 task——one-empty regime（全盘恰一个 upper 空格）
下，不可启动的 blocker 被替换为"vacancy 相邻 shelf 移入 vacancy"的
ready task（`to` = 当前 vacancy）；可启动的 chain head 获得 capped
BFS drop hint（`HEAD_DROP_SCAN_CAP = 64`，不可启动 head 跳过 hint，
2620-例贴线超时的回归修复）；零 vacancy 保留原 clear 发射（洗牌承重
梁，report.md 教训）。保护测试：
`dd_tasks.{one_empty_ready_task_moves_vacancy_adjacent_shelf,
feasible_clear_head_gets_compiler_chosen_drop,
unstartable_head_skips_drop_hint}`。step-2 gate
（`results_v3_step2_frontier`）：36/68、小图 36/36；vs baseline 共同
成功集 mk/SOC 几何比 0.932204/0.955072（12/17/7），总 makespan
34,860 -> 32,723；one-empty 锚例 `h4w10_a5_e1_R1_seed0` 1053 -> 417。
恶化例全披露：`h6w10_e1_B_seed1` +93%、`h10w10_a1_e1_R1_seed0`
+86%、其余 5 例 <= 11.3%（含贴线恢复例 `h10w10_e3_R1_seed1`
2620 -> 2689，first_ms 9.9s 仍贴 deadline）。

v3.0 step 3（execution-aware `tau_guide`，RED->GREEN 已落地）：共享
`compute_execution_prices`（§5.1 单轮反馈）——multi-goal root 的
chain-head frontier 以 **delta 价**（当前 frontier 实现代价 − shelf
接近基线，反对称防震荡）计入 guidance matching，仅当 delta 超过 lb
gap + eta 才触发重配；price 只进 guidance（`solve_tau` 的 price 参数
不参与 unrestricted h 解）；custody（§6）跨节点追踪 ANON carrier 的
TaskId（身份记账，不覆盖 parking 落点——编译期 `to` 在执行期已陈旧，
d50 bound 测试证伪了落点覆盖）；duplicate rewire 重建改为**惰性**
（reparent 标记 stale，下次扩展时以新 parent 为 anchor 重建——anytime
搜索千级 relax，急切重建不可承受）。保护测试：
`dd_tasks.{robot_placement_flips_tau_guide_goal,
execution_price_never_enters_admissible_h,
custody_keeps_task_id_from_lift_through_drop}`、
`dd_rewire.duplicate_rewire_rebuilds_guidance`（anytime 入口；
stop-at-first 生产遍经 80-fixture 扫描证实无 relax）。step-3 gate
（`results_v3_step3_price`）：36/68、小图 36/36；vs baseline 共同
成功集 mk/SOC 几何比 0.908435/0.930686（17/8/11），总 makespan
34,860 -> 31,278；vs step 2 几何比 0.974503（10/19/7）。变化行恰为
17 个 multi-goal B-pool 例（R1/singleton 零变化、18 个 R1 plan 与
step 2 逐字节一致——price 作用域与设计精确一致）；显著改善：
`h10w10_e3_B_seed0` 555->306、`h8w10_e2_B_seed1` 1108->679；恶化例
（全部 7 例，2026-09-02 R5 补披露漏报项）：`h6w10_e15_B_seed0`
112->164、`h8w10_e20_B_seed1` 168->233、`h4w10_e10_B_seed0` 41->71、
**`h4w10_e10_B_seed1` 32->42（+31.2%，首轮披露遗漏）**、
`h4w10_e1_B_seed0` 57->70、`h4w10_e1_B_seed1` 34->44、
`h8w10_e2_B_seed0` 648->668。

---

## 13. 边界与后续研究

1. 修补发生在输出端；搜索内部仍按 labeled robot full state 判重。
   它改善计划长度，不减少找到首解所需状态数（`h4w10_e1` 的 6,968 状态
   只有 861 个 shelf projection——主噪声是 free robot 站位）。
2. 1/2 robot bridge 最短；更多 robot 的 trajectory projection 只保证
   合法和缩短。
3. 搜索层按 shelf equivalence 合并状态需要同时保存可重构 robot
   bridge，不能只改 hash。
4. 20x20 以上 dense puzzle 的主要问题仍是首解 horizon 和每节点
   guidance 成本。`tau_guide` 的 execution 反馈针对的是 assignment
   churn 与 task 不连贯（诊断：20x20 曾 10 秒内构造 tau 45,839 次、
   改写 86,317 对；commitment probe 降到 2,757 并把 27/40 提到
   34/40，但正式 suite 大图仍 0/32）——它是必要改进，不是 horizon
   的充分解；层级搜索或可重构等价约减仍是开放项。
5. 生产不恢复已证伪的参数矩阵。新机制必须由输入结构或运行状态自动
   触发，并通过完整计划重放验证。
6. `targets > 256` 的 row-wise guidance regime（非单射 `tau_hint`，
   §4.2）没有任何 gate 实例覆盖，只有单元测试保护；扩大 suite 之前
   对该规模的质量结论一律不做。
7. **[v3.0 特有]** 每 node 重评 `tau_guide` 与 robot realization 项
   直接增加常数成本；必须以增量维护/缓存实现，且在 guidance 预算
   （§11.6 第 6 条）下验收，防止把大图 horizon 推得更远。
