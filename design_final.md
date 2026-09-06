# Carrier-LaCAM 最终设计 v5：清障因果依赖、动态运输与 Makespan 优先

**状态：v5 已在原 LaCAM-TAPF execution path 上实现；代码测试、Testcase C、
固定 quick 77、E5 独立消融和经独立审查放行的正式 full 509 均已完成。
正式 full 为 479/509，新增 factorial 为 432/432；这些实现与验证证据保留在
§23。2026-09-06 又完成了当前 \(\rho\) 派工机制的专项代码与计划重放审计，
确认普通任务仍可能在进入 matching 前被 priority top-\(F\) 硬过滤。本稿已
把该审查修正并入规范性章节。随后 F0/F1/F2 已在同一 execution path 上
实现并完成固定 quick 77：production 取消普通 cutoff，并只在严格同质的
直接目标交付阶段启用有限 frontier/continuity defer V2。additive G 与
节点局部 H shadow 已独立实现、测量并因 Testcase C 回归而回滚；它们不是
当前 production 功能。正式 full 509 尚待独立终审放行。**
**日期：2026-09-06。实现起点：commit `03e99ba`；专项审计基线：
commit `80148a7`；当前 production commit：`64a3941`。**
**2026-09-05 设计独立审查：主设计有条件通过，`debug.md` 按审查重写。本版已并入
审查修正：episode 与 route 成功解耦（§9.1）、horizon 与质量比较语义
（§9.2/§9.7）、D0/ExecutionView 对齐契约（§10）、成本比较器与权重域
（§12.1）、mixed 时间下界（§5.2）、首次 goal 前缀强制规范（§12.3）、
objective 显式契约与单一控制器（§12.2/§13.7）、compiler 接纳条件审计
（§13.3）、阶段重排 B/C1/C2/D/E（§14）。受保护测试的后续迁移均先获得
独立 GPT-5.6 Sol/xhigh `APPROVE`。**

本稿把 `Carrier_LaCAM_v5_Makespan_Design.md` 的修订并入最终设计文档。
§0–§19、§21 是规范；§20、§22 原文保留为 2026-09-03/04 的历史验证；
§23 是 2026-09-05 v5 实现证据；§24 是 2026-09-06 \(\rho\) 专项审计，
§25 记录该审计后的实现、回滚决策与新验证。被取代版本的 SHA-256：
`2010f90cd6d6aa8b9ea68393df23a58432c006e584b4422cbf3b050517cdf524`。

与 v5 草案不同，§1.1 和 §13.1–§13.3 的历史落点以 commit `03e99ba`
为准；当前 \(\rho\) 行为及 §13.4 后续修改以 commit `80148a7` 的 S6
审计为准。

依据与证据等级：

- **S0：本设计稿。** “当前规定”指本稿规定；实现符合性由测试、benchmark
  和独立代码审查共同判定。
- **S1：2026-09-05 动态绕路提案。** 它是设计来源；2026-09-05 release
  行为以 §23 为准，当前 \(\rho\) 机制以 S6/§24 为准。
- **S2：用户提供的 Testcase C 地图和 31 行参考计划。** 用户报告权威
  validator 验证通过；54/90 与 31/93 是该报告中的指标。
- **S3：此前构造的 31 步、SOC 90 候选。** 按 S2 的地图和动作规则独立重放
  得到 31 步、49 loaded moves、29 free moves、12 Lift/Drop，并检查通过；
  未运行仓库的权威 validator，不作为 release 成果或 SOC 最优性证据。
- **S4：2026-09-05 代码审计。** 对 commit `03e99ba` 的
  `tapf_planner.hpp/cpp`、`carrier_guidance.hpp`、`dd_planner.cpp`、
  `dd_plan_repair.cpp`、`dd_carrier.*` 的逐符号核对，见 §1.1 与 §13。
- **S5：2026-09-05 v5 实现证据。** 新增与迁移后的 C++/Python tests、
  Testcase C 权威重放、固定 quick 77 配对和当前二进制 provenance；
  具体数字集中记录在 §23，避免覆盖历史 §20/§22。
- **S6：2026-09-06 \(\rho\) 专项审计。**
  `carrier_lacam_rho_global_matching_report_20260906.md` 对 commit
  `80148a7`、正式 full benchmark 和
  `brap_h10w10_a12_e8_R1_seed1` 的交付计划进行了代码核对与确定性重放。
  它是当前 priority cutoff、bottleneck matching、retarget、利用率口径及
  增量 Hungarian 契约的依据；不回写或覆盖 §20/§22/§23 的历史证据。
- **S7：2026-09-06 \(\rho\) 修订实现。** F0/F1/F2 的独立提交、G/H
  实验及回滚、312 个 C++ tests、162 个 Python tests、四个受保护样例和
  固定 quick 77。当前 production 为 `64a3941`，二进制 SHA-256 为
  `d90a0efa2ee2436778ceda107bbc785d2d371e51fd13eba9b00d90ec1758af7d`；
  完整证据见 §25。full 509 在独立终审前不属于 S7 已完成证据。

## 0. 最终决策

保留两层动态 assignment 和一个真实物理搜索。不要把 Testcase C 的运输冲突
误当成清障 dependency，也不要因为路线预测相交就取消整个搬运任务。

```text
完整物理状态 X
    ↓ 取 upper layout U
single-root PairCost(U, b, g)
    ↓
Hungarian tau_guide：目标货架 → eligible goal
    ↓
joint Task-BR-PIBT：联合选择 blocker 搬法，生成清障因果图 D0
    ↓
与真实 custody 对齐，得到当前 ExecutionView
    ↓
rho：在全部当前 mode 下物理/因果可行的候选上，
     选择机器人 → EXECUTE/PREPARE transfer 或 idle
    ↓
联合运输 guidance：保持 endpoint，选择路线、等待和通过顺序
    ↓
Carrier-PIBT + operator constraints
    ↓
apply_ops → 一个真实 joint transition
```

三个决定分别回答：

1. **tau：**目标货架最终去哪个合法 goal？只读取货架布局。
2. **Task-BR-PIBT：**为了推进这些目标，哪个 blocker 必须先搬到哪里？
   共享上下文，递归、回退、合并兼容需求。
3. **执行协调：**谁搬、何时可开始哪一个阶段、从哪里走、谁先经过交点？
   读取机器人和实际执行状态，但不写回 `PairCost/tau`。

\(\rho\) 的候选边界和求解器必须遵守以下额外合同：

1. priority 可以参与 Task-BR 的图排序、有限延期损失或确定性 tie，但普通
   可行任务不能仅因 rank 较低而在 matching 前被 top-\(F\) 删除；
2. 第一组修订保留当前 bottleneck-then-sum 与 canonical assignment，只
   改候选准入；robot-row additive `S-D` 是独立 full-solve 实验，不与
   makespan 等价，也不以 maximum-cardinality 为硬主目标；
3. 增量 Hungarian 只能在候选、成本、mode 与 canonical tie 全部冻结后
   加速同一个 assignment。matching/duals 属于具体 LaCAM 节点，不能只按
   upper layout 全局复用。

最终优化目标为词典序 `(executed_makespan, weighted_SOC)`。首解质量、搜索
改进、两遍候选比较、rewrite 和 repair 必须使用相同目标。

最终 execution path 必须仍是：

```text
TAPFPlanner::solve()
  -> attach_carrier_guidance()
  -> Carrier-PIBT / funcPIBT()
  -> apply_ops()
  -> physical successor
```

不得新增平行 planner、第二套 search loop、运行时 legacy fallback，或在
Carrier-LaCAM 节点内嵌完整 BR-LaCAM。

## 1. 当前代码审计与逐章修改表

### 1.1 代码审计（2026-09-05，commit `03e99ba`，S4）

以下事实已逐条对照源码验证；行号为当前 commit 的近似锚点。

**目标函数是标量 weighted work，没有时间分量：**

| 位置 | 现状 |
|---|---|
| `tapf_planner.cpp::get_edge_cost()` l.1069 | edge cost = 任务项 + `alpha`(loaded move) + `beta`(free move) + `gamma`(lift/drop) + `delta`(anon 附加)。全体 Wait 的 joint op 成本为 **0**，时间不计入 |
| `tapf_planner.hpp::TAPFNode` | `g/h/f` 均为 `double`；`SearchEdge.physical_cost` 为 `double` |
| `tapf_planner.cpp::solve()` l.506 | incumbent 剪枝 `S->g < incumbent`（l.629–630）；`first_solution_g` 记录 weighted SOC |
| `tapf_planner.cpp::rewrite()` l.977 | 沿 immutable `SearchEdge` 记录传播标量 `g/f`，按 goal `g` 剪 |
| `tapf_planner.cpp` l.654–688 | macro rollout successor 只在首 incumbent 前插入（`macro_after_first` 只做统计） |
| `dd_planner.cpp::plan_soc()`；两遍比较 l.362 | `if (soc2 < soc)`：两遍候选按标量 SOC 选择；`assignment_*_makespan` 只记录不比较 |
| `dd_plan_repair.cpp` l.389/415 | repair 接受条件是分段与总量 SOC 非增（`SOC_EPS`），没有 tick 维度 |

**h 只有 work 下界：** `carrier_guidance.hpp::solve_tau_lb()` l.694 是
admissible weighted-work matching，在首次 attach 时加入 `nd->h`
（`tapf_planner.cpp` l.231–235）。不存在 makespan 时间下界 `h_T`。

**任务身份把 leg 与 transfer 混在一起：**

| 位置 | 现状 |
|---|---|
| `tapf_planner.hpp::TaskId{shelf,from,to}` l.129 | 一步 exact effect，即本稿的 `LegId` |
| `tapf_planner.hpp::StorageTransfer{endpoint,route}` l.163 | `operator==` 同时比较 endpoint 与整条 route：transfer 身份被 route 绑死 |
| `tapf_planner.hpp::Custody` l.198 | 含 `transfer/transfer_index`，是 route 后缀绑定 |
| （缺失） | 没有 `TransferKey/TransferId/ExecutionView/PlanCost` 类型 |

**custody 是 route 后缀绑定，偏离即失效、恢复不保 endpoint：**

| 位置 | 现状 |
|---|---|
| `carrier_guidance.hpp::custody_physically_valid()` | 要求 `route[transfer_index]==from && route[transfer_index+1]==to`、全 route 相邻、endpoint 可存储：任何偏离 stored route 都判失效 |
| `recover_task_br_custody()` l.2914 | 三类恢复：exact loaded Move 沿 `custody.to` 推进 index；WAIT 复验保持；LIFT 把上一拍 rho 绑定转为 custody |
| `make_storage_recovery_custody()` | forced deviation 后选择“最短可达空 endpoint”（按 route 长度再 endpoint id 排序），**不优先原 episode endpoint** |

**ready 集合被 full-route 空间交集过滤（v5 明令删除的捷径）：**

| 位置 | 现状 |
|---|---|
| `ActiveTransferClaims` l.2588 | endpoint claim + route 内部 transit 格 claims |
| `ready_tasks_with_custody()` l.2757 | 谓词（predecessor 空、目的格未被占、无 custody 归属、shelf grounded/continuation）之后，尾部按 claims 过滤：与 active custody 剩余 route 相交、或与本轮更高优先 ready 任务的 route 相交的任务被整体移出 ready |
| `bind_ready_continuations()` | roomy 布局抑制立即反向（`task.to == 上一拍 from`），dense 布局允许 |

**rho 是 grounded-only 的 min-sum Hungarian：**
`match_ready_tasks()` l.3128 —— 只匹配 free robots × grounded ready；按
TaskId 去重（同 id 取高优先）；priority cutoff 截到 `|free|`；成本 =
lower-deck distance × scale + switch penalty（按 **leg TaskId** 比较上一拍
绑定）；确定性词典序精化。没有完成时间估计。

**attach 管线没有执行层视图：**
`build_task_br_guidance_from_upper_epoch()` l.3651 依次做 recover custody →
ready（含 claims 过滤）→ bind continuations → grounded ready → rho。没有
ExecutionView 对齐、没有 preparation 候选、没有 time-expanded 运输
guidance。`UpperEpochCache` 是 (UpperSignature, priority-commitment) 键的
LRU；claims 在 cache 之外——该纯度边界是正确的，必须保留。

**执行偏好：** `funcPIBT()` l.1231 —— loaded+bound 依次首选 exact
`custody->to`、WAIT、storage DROP、其余合法 Move（`custody.to` 排位 0）；
loaded-unbound 在 storage 首选 Drop、在 transit 首选 Wait（避免贪心
retarget）；free+assigned 在 shelf 处 LIFT、否则按 lower distance
approach；free idle 按 ready/custody footprint 避让。carrier 角色失败 push
释放预约重试。oracle guards（S1 upper-taken、lift/drop 前置）保留完整
candidate 集，完备性由 operator tree 保底。

**正确保留的部分（审计确认，不改）：** `SearchKey` 只含物理状态；
`is_goal_config()`/`is_dd_goal()`（目标 grounded 于 eligible set、carried
target 不算 grounded、carried anonymous 不阻塞终点）；`apply_ops()` 唯一
物理裁判与 storage-only Drop；operator constraint tree 穷举保底；
immutable `SearchEdge.transition_trace` 与 `guidance_stale` 重锚；
两遍求解 + repair + strict deadline + 双侧 replay 交付管线；
`upper_vacancy_count()` l.90（storage cells − shelves）；candidate core
`reachable_storage_transfers()` l.1160 / `ordered_shelf_candidate_window()`
l.1206 的确定性 BFS 与 lazy-exact certificate
（`pair_cost_prefix_lower_bound()` l.371）。

**测试现状：** C++ 241 项、Python 99 项全绿（§22 记录）。其中
`tests/test_dd_storage_transfer_claims.cpp` 把 route-claims 过滤语义锁为
GREEN，`tests/test_dd_plan_repair.cpp` 锁 SOC 非增，
`tests/test_dd_objective_*` 锁标量目标行为——这些是 §15 要迁移的策略契约，
不是物理语义。

### 1.2 相对 2026-09-04 基线稿的逐章修改表

基线稿 §6.3/§6.4 已经规定 shared context、递归清障和 root-level
backtracking；不能把它整体描述成“只有独立路径加依赖”。本轮主要纠正
storage transfer 引入后的粒度混用和过度互斥。

| 基线稿位置 | 原规定或遗漏 | 本稿修改 |
|---|---|---|
| 头部、§0、§1.1 | 只改 guidance，两遍求解和 repair 原样保留 | 本轮最终目标涉及 search cost、incumbent、repair；历史验证不自动沿用 |
| §2.3、§11 | `storage_cells - shelves` 被称为当前 vacancy 数 | 区分净 storage 余量、当前空 storage、当前空 transit |
| §3.2、§21.2 | `TaskId=(shelf,from,to)` 同时承担一步动作与整个 transfer 身份 | 保留 `LegId`，增加不含 route 的 `TransferKey/TransferId` |
| §21.2 | 相同第一腿、不同 endpoints 仍合并 roots，保留较高优先 endpoint | 不得把共享动作前缀当成共同完成整个搬运；不同终点保留为不同方案 |
| §6.2、§21.3 | 每个 endpoint 只保留一条确定性最短 route | canonical route 可继续用于 PairCost；实际运输能重新寻找同 endpoint 路线 |
| §6.3、§21.3 | first-leg destination 和 endpoint 在编译层混合预约 | endpoint 放置冲突仍联合处理；未来 transit 第一腿竞争交给时序协调 |
| §21.5、§21.6 | active/ready 的完整 route claims 过滤 grounded tasks | 删除空间交集过滤；保留真实 occupancy、custody 与 endpoint 放置语义 |
| §7、§8、测试 #16 | 非 leaf 永远不得 approach | 分开 `assignable/preparable/move_executable`，允许受控提前准备 |
| §7.3、§9 | route suffix 必须保持；偏离就终止 transfer | 正常绕路保持 endpoint/episode，只更新 route 和 LegId |
| §8.2 | priority-first、min-sum approach，并倾向填满所有行 | v5 后续已接入 bottleneck dispatch，但 S6 发现 top-\(F\) cutoff 仍先删除普通任务；先保留 bottleneck 只修候选边界，再独立比较 additive `S-D`，不能把满载率当目标 |
| §10、§19 | custody 和缓存不足以区分纯上层计划与执行时间表 | `D0` 可缓存；ExecutionView、lease、时序预约不进入 upper cache |
| §12、SearchEdge、两遍/repair | weighted cost、加法式 h、SOC 优先 | `Cost{ticks,work}`；makespan 下界；词典序比较和相应剪枝 |
| §15–§17 | 一步身份、ready-only、SOC 不增、指定 robot 活动等保护测试 | 保留物理正确性；审查后迁移已经改变的策略契约 |
| §20、§22 | 已通过验证的历史实现 | 原文归档；不得改写成 v5 的验证结论 |

### 1.3 算法判断

旧实现中“全局 injective matching、任务携带 root provenance、custody、
Hungarian rho、lower-deck PIBT、完备性保底、storage transfer 承诺”都是
合理基础；问题不在这些组件本身，而在三处粒度错配：

* 一步 `LegId` 同时承担整个 transfer 的身份与合并判断；
* 未来 route 的空间集合被当成永久互斥资源，删除了本可分时复用的任务；
* 搜索与交付的目标只有 work，没有时间，无法表达 makespan 优先。

本文的修改不是另起炉灶，而是在现有 guidance 入口与 cost 接口内重新划清
任务身份、时序协调与优化目标。

### 1.4 2026-09-06 \(\rho\) 专项审计（commit `80148a7`，S6）

§1.1 记录的是 v5 实现前的历史代码基线。当前代码已经具有
bottleneck-then-sum dispatch、EXECUTE/PREPARE、两遍控制和 projection
repair，但 S6 对当前实现确认了新的候选边界问题：

1. `target_priorities_from_pair_cost()` 仍把 PairCost 转成整数 rank；
2. `ready_tasks_with_custody()` 先按 priority 建 transfer claims，冲突候选
   的胜出者可能已经由顺序决定；
3. `match_ready_tasks()` 对 free robots 数 \(F\) 取第 \(F\) 名 priority
   作为 cutoff，低于 cutoff 的普通任务在矩阵建立前删除，高于 cutoff 的
   任务近似 mandatory；
4. 幸存任务才进入 bottleneck-then-sum assignment 和确定性 canonical
   refinement；EXECUTE 先匹配，剩余机器人再匹配 PREPARE；
5. `CarrierGuidance` 没有节点局部 matching/duals，当前每次重新构造并
   full solve \(\rho\)。

在诊断实例 `brap_h10w10_a12_e8_R1_seed1` 的交付路径上，没有发现“同一
货架同时对应多个临时 endpoint task”的状态。可复现的直接原因是：距离
空闲机器人 1–2 格的普通 ready task 会因 priority cutoff 不进入矩阵，而
距离约 10–11 格的高 priority task 留在矩阵中。这解释了动画中的跨区域
派工，但不证明附近任务必然带来更小 makespan；远处 blocker 仍可能位于
更关键的因果链。

该实例正式数据为：首解 1188 ms、`(T,W)=(2659,5691)`；最终交付
`(1844,3927)`；第二遍没有候选，projection repair 删除 815 拍。CSV 的
`robot_utilization=0.1508` 只表示 loaded moves 占 robot-time slots 的
15.08%；把 free moves 和 Lift/Drop 计入后，非 Wait 动作占比为 95.36%。
因此不能从该字段推出“机器人经常闲置”，也不能据此把
maximum-cardinality 设成 dispatch 第一目标。

S6 将后续工作拆成三个独立问题：

```text
候选边界：哪些任务有资格参加比较？
调度目标：在同一候选集上怎样比较 task、mode 与 idle？
求解方法：怎样更快地精确求出已冻结目标的同一个 assignment？
```

前两者决定派工行为；增量 Hungarian 只处理第三个问题。

## 2. 物理状态、终点、计时与 upper projection

### 2.1 保留物理模型

```text
X = (Q_robot, Q_target, Q_anon_grounded, kappa)
Op = Wait | Move(neighbor) | Lift | Drop
```

robot 是唯一 actuator。匿名货架不加入永久身份。`SearchKey` 仍只表示真实
物理状态。`apply_ops()` 独立决定 vertex、swap、following、upper
occupancy、Lift/Drop 和 storage legality；本轮不修改其允许的转移集合。

```text
is_goal(X) = 所有 target grounded 且 position(b) ∈ G_b
```

终止不依赖 tau、task graph、route 或 custody。当前终点定义没有要求所有
机器人归位，也没有要求所有匿名货架的 guidance episode 都结束（代码中
`is_goal_config()` 只拒绝 carried **target**）；不能通过“等待全部
transfer 清空”暗中加一个终点条件。若以后要求所有匿名货架也落地，必须
另行修改问题定义和双侧 validator。

### 2.2 目标函数

设 `X_0 ... X_T` 是首次满足 `is_goal` 的物理前缀：

```text
T = 真实 joint transition 数
W = alpha*loaded_moves + beta*free_moves
    + gamma*lift_drop + delta*anonymous_loaded_moves
J = (T, W)，按词典序比较
```

一个普通 joint transition 的时间成本是 1，即使所有机器人都 Wait；其中多少
机器人同时行动不改变这 1 拍。macro 的时间成本是实际 trace 长度，不是 1。

alpha/beta/gamma/delta 是 work 权重（对应现有 `DD_ALPHA..DD_DELTA` 环境
输入与 `TAPFPlanner::Weights`），不是动作时长。本文仍使用单位时长
Move/Lift/Drop；非单位操作时长需要新增物理 mode/剩余时长，不在本次修改中。

### 2.3 三种“空位”

令 `S` 为合法 storage cells，`O(U)` 为全部货架当前坐标，`V` 为
traversable cells：

```text
storage_slack        = |S| - number_of_shelves
empty_storage(U)     = S \ O(U)
empty_transit(U)     = (V \ S) \ O(U)
```

`storage_slack` 是静态净存储余量（现有 `upper_vacancy_count()` 即此值），
不一定等于当前空 storage 数。货架离开 storage 进入通道时，其源 storage
已经空出，即使它还没在另一个 endpoint Drop。

空 transit 不是合法 Drop 位置，但可以供 loaded Move 使用。不得把“不能
Drop”写成“不能作为移动空格”，也不得由 `storage_slack==0` 推断全物理问题
无解。

### 2.4 纯上层与执行层

```text
U = labeled target coordinates + sorted all anonymous coordinates
```

`PairCost/tau_guide` 只依赖 `U` 和不可变实例参数。Lift/Drop、free Move 不
改变它们。`D0` 依赖 `U/tau/upper priority commitment`，继承已实现的
commitment cache key（`UpperEpochCache`）；不能再笼统声称 D0 在任何
ancestry 下都是 U 的纯函数。

ExecutionView、rho、prep admission、route、预计 release time 和 timed
reservations 可以读取完整 X 及紧邻真实 transition，不写入 PairCost 或 D0
的缓存值。

## 3. Guidance 数据结构：把任务与动作分开

### 3.1 Root、transfer、leg

```text
RootDemand(b, g)              目标货架 b 当前朝 g 完成
Transfer(s, source, endpoint) 一次明确的 storage 搬运意图
Leg(s, current, next)         这一拍建议实现的相邻 shelf effect
```

保留现有 exact `TaskId{shelf,from,to}` 的一步含义，可兼容命名为 `LegId`。
增加：

```cpp
struct TransferKey {
    ShelfSelector shelf;  // pending anonymous 用当前真实 source cell
    Cell source;
    Cell endpoint;
};

struct TransferTask {
    TransferKey key;
    RootSet roots;
    CausalRequirements requirements;
    RouteHint canonical_hint;
};

struct TransferEpisode {
    TransferId id;        // branch-local、transition-anchored 的稳定值
    RobotId carrier;
    ShelfBinding shelf;
    Cell original_source;
    Cell endpoint;
    RootSet roots;
    RouteHint preferred_route;
    optional<LegId> preferred_leg;
};
```

这里是语义接口，不要求按这些名字新建平行框架。可在现有
`ShelfTask/StorageTransfer/Custody` 内逐步实现（例如：`StorageTransfer`
的身份判断只用 endpoint，route 降级为可重算的 hint；`Custody` 增加
original endpoint 与 rebinding 标记）。`TransferId` 不使用全局递增计数
影响排序；anonymous 的稳定 episode 由实际 carrier 与 Lift/Move anchor
延续，不进入 physical key。

### 3.2 Identity 契约

route、下一格、局部时间表改变时，transfer 不一定改变。只有 source/所搬
货架/endpoint 的语义改变，或真实 Drop 终止了该 episode，才失效或建立新
transfer。

以下两种方案不能合并为同一个完成任务：

```text
s: u -> v -> ... -> e1
s: u -> v -> ... -> e2
```

它们共享第一腿 `u->v`，但完成条件不同。物理 successor 可以按同一个 LegId
去重；task graph 不能仅因第一腿相同就把两个 root 的要求都标成已满足。

只有相同 source、相同 shelf、相同 endpoint，且因果要求可协调的 transfer，
才共享一个 task。不同路径是其候选实现；不同 endpoint 是不同方案。若另一个
root 只要求 `Vacate(source)`，并不要求特定 endpoint，它可以在重新检查自身
条件后接受已有 transfer；不能只因第一腿相同就省掉这个检查。合并 roots 后
仍向全部 predecessor closure 传播需求与优先级。

### 3.3 Transfer 不强制每段 Drop

事件至少包括 `PickupReady`、`Lifted`、`Vacated(source)`、
`Arrived(endpoint)`、`Dropped`。到达 endpoint 完成该 transfer 的搬运
effect；释放机器人需要 Drop；target 完成则必须在合法 goal Drop。

同一个 carrier 可以在合法 storage endpoint 接续下一 transfer，省掉中间
Drop/Lift。不能为了清晰的 task 身份而强制每个相邻 shift 举放一次。无
storage map 时各 transfer 退化为相邻 effect，物理语义不变。

## 4. 第一层：single-root PairCost

保留 S0 的 shelf-only、bounded rollout、rollout-local anonymous token、
有限 stall/truncation、cache version 与 lazy-exact certificate
（`pair_cost_prefix_lower_bound()` 的 `L_e <= C_e` 义务）。

第一批修改不重写 PairCost，不把 execution price 加回来。canonical route
仍可由确定性 BFS（`reachable_storage_transfers()`）产生，作为估价样本；
“用于估价只存一条 route”不等于“实际执行只能走这一条”。

PairCost 精确值的含义是：精确计算这个确定性有界启发式定义的数值，不是
得到真实最优搬运成本。单 root rollout 的顺序执行估计也不是多 root
makespan 下界。

禁止混入机器人位置、free/loaded availability、rho、timed reservation 和
等待历史。其读取集合不因下面新增的 execution view 而扩大。

若之后修改 PairCost 输出的时间/工作量统计，必须更新 cost/compiler
version 并重新验证 lazy certificate 中 `L_e <= C_e` 的义务；不能更换代价
语义却沿用旧证明和 cache。

## 5. Shelf-goal matching 与 admissible bound

### 5.1 tau_guide：本轮先保留 min-sum

保留当前基于 PairCost 的 injective min-sum Hungarian
（`solve_tau_guide()`），以及不读取 parent/robots 的稳定 tie。它仍是 goal
guidance，不声称精确最小化 makespan。

Testcase C 的目标均固定；它的主要问题不能通过修改 tau 解决。本轮不同时把
第一层改成 bottleneck matching，以免混淆 route 修复、派工和 goal
allocation 的贡献。

后续可独立比较 min-sum work 与 min-max isolated completion estimate。
多目标共享 blocker、vacancy、robot capacity 时，两种可分估计都可能失真，
不能宣称简单换成 min-max 就得到全局 makespan matching。

### 5.2 Makespan lower bound

下面是本稿的推导，要求当前模型为单位时长、四邻接、robot 是唯一
actuator，且每个机器人一拍最多做一个 primitive action。

`d_U` 和 `d_L` 是只考虑不可变墙的距离，忽略其他 robots/shelves。对
grounded 且需要移动的 target：

```text
a_b(X) = min_r d_L(Q_robot[r], position(b))
```

这里可以乐观地把 loaded robot 也当成可以直接接货；这只会降低估计，不会
高估。

定义每对 eligible `(b,g)` 的时间下界：

```text
ell_T(b,g) = 0                        grounded 且 position(b)=g
             d_U(position(b),g) + 1   carried（最低 final Drop）
             a_b + 1 + d_U + 1        grounded 且 position(b)!=g
```

定义最低 target 操作工作量（不重复加入 approach）：

```text
w(b,g) = 0                    grounded 且已经在 g
         d_U + 1              carried
         d_U + 2              grounded 且需要移动
```

则：

```text
h_bottleneck(X) = min_injective_tau max_b ell_T(b,tau(b))
h_work(X)       = ceil(min_injective_tau sum_b w(b,tau(b)) / |R|)
h_T(X)          = max(h_bottleneck(X), h_work(X))
```

证明要点：任何实际解选择某个 injective 最终 assignment；每个 target 到
对应 goal 的完成时刻至少是 ell_T，故最晚完成时刻至少是其最大值。每个
robot 一拍最多执行一个动作，target 必要工作量也不能超过 `|R|*T`。对全部
合法 assignment 分别取最小值仍是下界。机器人相互竞争、清障与拥堵被忽略，
估计可以弱，但不因此高估。

`h_W` 保留现有 `solve_tau_lb()` 不加 locks 的 weighted target-work LB
matching。`h_T` 可以读取完整 X 中的机器人位置；这不是对 tau 的反馈。原稿
“任何 robot approach 都不准进入 h”应改成“未经证明的执行估计不准进入 h”。

若保留 mixed TAPF/carrier 输入，必须先分别构造两个**时间**下界，再取
`max`：

```text
h_TAPF_time  = max_i min_{允许 goal} d(C[i], goal)   逐 agent 取 max，不是求和
h_shelf_time = h_T
h_ticks      = max(h_TAPF_time, h_shelf_time)
```

原 `get_h_value()` 的任务项是逐 agent 距离**求和**，是 work/SOC 下界而
不是时间下界：三个互不干涉、各差一步的 agent 求和得 3，实际一拍同时完成。
把它直接放进 ticks 维（即使随后与 shelf time 取 max）已经高估；也不能把
它与 shelf time 相加。“取 max”不能修复输入本身单位和语义错误的问题。
新的 combined bound 必须另有证明与小图 oracle 测试。没有证明时取更弱下界
甚至 0。

## 6. 联合 Task-BR-PIBT：保留清障因果，去掉运输假互斥

### 6.1 编译对象

compiler（现有 `TaskBRCompilerState/Transaction`）继续处理全部未满足
root 的候选搬法，保持 shared transaction context、递归栈、root-level
backtracking 和预算耗尽返回 partial guidance。

“联合”要求一个 root 的已选 blocker 方案影响其他 root 的可选方案；冲突
可以触发换 endpoint、换 displacement chain 或回退先前选择。不能退化成
独立选完后仅扫描并加边。

### 6.2 依赖边的准确语义

```text
A 需要进入 u
u 当前由 B 占据
B 需要搬到 v，但 v 当前由 C 占据
C 可搬到 e
```

生成的是：

```text
C 腾出 v → B 允许进入 v
B 腾出 u → A 允许进入 u
```

不默认要求 `C 在 e Drop 后，B 才能开始 approach`。边应携带被释放的物理
条件和被约束的事件：

```cpp
struct CausalEdge {
    TransferKey producer;
    Cell must_be_vacated;
    TransferKey consumer;
    Event consumer_event;  // Enter(cell)，必要时 Depart/Acquire 等
};
```

仅当任务需要机器人释放、或需要前驱确实落地时，才增加 Drop/RobotAvailable
依赖。

条件不是永久的“曾经腾空过”。若格子被重新占据，等待者进入前必须重新
验证。D0 与 ExecutionView 均不依赖单调 completed bit 来跳过 occupancy。

### 6.3 冲突分类

| 关系 | 处理位置 | 行为 |
|---|---|---|
| 当前 grounded blocker 占据必要 cell/endpoint | Task-BR-PIBT | 递归搬 blocker，生成因果边 |
| 同一 shelf 被要求搬到不兼容 endpoints | Task-BR-PIBT | 选择替代方案或 backtrack，不能伪合并 |
| 两件 shelf 要占据同一 endpoint | 放置/因果协调 | 本轮选兼容放置，或显式安排一次再次 vacate；不是无限容量目标 |
| 两条未来 transit 路线有交点 | 执行协调 | 选择时序/绕路，不产生清障先后边 |
| 两个未来 first legs 经过同一 transit cell | 执行协调 | 不能直接删除整个 transfer；实际同拍冲突在 joint action 检查 |
| 某个 carried shelf 暂挡 transit | 执行协调/active episode | 等待或绕路，不让另一 free robot 去“搬走”已被 carry 的 shelf |

endpoint 是放置位置，transit 是可以分时复用的位置。不要把二者放进同一张
永久 exclusive route ledger（现状 `ActiveTransferClaims` 对 ready 集合的
过滤正是这种 ledger，必须删除）。

### 6.4 Storage 候选

保留“从 source 离开后，在一个 transfer 内到达第一个 storage endpoint”的
分段约定；它是 transfer 抽象，不是新增物理禁行规则。长路径可由多个
transfer 及连续 custody 表达。

几何 candidate core（`reachable_storage_transfers()` /
`ordered_shelf_candidate_window()`）继续由 single/joint compiler 共用。
PairCost 可只消费 canonical route；执行层调用同一拓扑/endpoint 约束寻找
替代 route，不要求 BFS 穷举所有最短路。

还要区分“墙使 endpoint 不连通”和“目前有在途货架挡路”。后者可以生成暂待
交通释放的 endpoint 意图，不能因为当前没有全空的 route hint 就把 root 的
整个搬运需求永久隐藏。D0 保留其 endpoint/需求及阻塞原因，执行层用真实
custody 判断能否等待或重路由。PairCost 的抽象 rollout 仍不得把 shelf 移入
另一个 shelf 当前占据的格子；有界估价失败继续是有限 penalty，不是不可达
证明。

原 `reserved_destination` 在 storage endpoint 上有放置意义；在非 storage
first-step 上只有当前同拍动作选择意义，应移出纯因果图的整 transfer 接纳
条件。

### 6.5 图评分与优先级

第一阶段保留已有 density-aware priority/progress 作为基线，避免同时推翻
密集图启发式。后续为少量候选图估计剩余清障链长度与总 work；执行层补入
机器人 availability 后才评价时间。

被暂停的 root 不能从完成时间估计中消失，否则“少编任务”会虚假地得到更小
makespan。共享 blocker 的 work 只计算一次；后继尾长取 max，不能把整个
root PairCost 在每层重复相加。

基础优先级仍放在目标货架上，沿因果边传给 blocker。执行紧迫性可在
ExecutionView 中提升关键 blocker；不要恢复固定的“目标货架永远比匿名货架
高”。

priority 在这一层表示图排序和有限紧迫性信息，不是 \(\rho\) 的普通任务
准入证书。一个 task 只要通过当前 dispatch mode 的物理、因果和冲突检查，
就不能仅因 priority rank 较低而在 matching 前消失。若共享 blocker 同时
服务多个 roots，其延迟影响按受影响 roots 聚合，但同一条关键链不能沿多个
task 重复奖励。

还要区分两种 tail：当前机器人执行 transfer 所占用的即时 service，与该
transfer 完成后仍需推进的 causal tail。后者可以参与 bottleneck 完成时刻
估计；不能未经定义就以正号加入 additive 即时服务成本，同时又以 urgency
奖励一次。

## 7. Readiness：派工、准备与移动不是同一个判断

### 7.1 三个派生谓词

```text
Assignable(m): 有效 transfer，source/shelf 正确，未被别的真实 carrier 操作，
               endpoint 放置方案明确；可以考虑给它安排 robot。

Preparable(m,r): 即使 consumer 的移动条件尚未满足，r 的 approach/准备
                 仍有价值，不夺走关键前驱的必要执行者，也不堵其交通。

MoveExecutable(m,r,a): 当前具体 loaded Move 满足所需占据/事件条件，
                       且完整 joint action 可被 apply_ops 接受。
```

候选进入 \(\rho\) 前还应记录 dispatch mode 和显式 conflict group。同一
实体货架、同一 transfer 的 EXECUTE/PREPARE mode、当前不可同时占用的
endpoint 等，不是普通二分图“一列至多一个机器人”就能完整表达的关系；
第一版可继续在矩阵外做确定性兼容预选，但必须记录删除范围和原因，不能把
它重新包装成 priority cutoff。

现状只有单一 `ready`（predecessor-free + 目的格空 + shelf 可用 + claims
过滤）。不能因为 canonical route 的旧第一格被挡，就判定整个 transfer 不可
分配：先允许同 endpoint 换路或等待。也不能因为 assignable 就承诺这一拍能
进入通道。

### 7.2 提前准备的边界

最小阶段只把 causally ready transfers 全部保留下来，足以修复 C 的核心
问题。第二阶段再将执行器扩展到 selected chain 的有界后继准备，不恢复
“任意 non-ready clear 都派机器人”的旧行为。

优先确保当前必要前驱有可用 robot，再用剩余资源提前 approach。只有一个
机器人时，不得让它举着 A 等待还没有执行者的 B。Lift 仍只由物理
precondition 决定是否合法，但 preferred Lift 需要考虑是否会阻塞必要前驱
或把机器人困在无法启动的等待链里。

PREPARE 只表示提前 approach 或在 pickup 附近等待；它没有完成 transfer，
也不能直接抵扣完整任务延期损失。机器人到达 pickup 后，只有当任务重新
通过 EXECUTE 条件检查时才允许 preferred Lift。若将两个 mode 放进统一
矩阵，它们必须属于同一 transfer conflict group，不能同时获得两个 owner。

这些限制都是 guidance admission，不是 operator tree 的合法性限制。

### 7.3 事件释放

前驱货架刚离开源格，后继就可以准备利用该格；不必等它完成整段运输或
Drop。following 允许时甚至可以在同一个 joint transition 释放/使用；不允许
时必须等到模型规定的下一拍。所有判断以 oracle（`apply_ops()`）的时间约定
为准。

预测前驱未来会离开并不能当成“现在已经 empty”。若前驱实际延误、转向或
再次占据该格，刷新 ExecutionView 和后继候选。

## 8. rho：候选边界、调度目标与增量求解

### 8.1 第一合同是让普通可行任务参加比较

S6 确认当前 `match_ready_tasks()` 不是在全部普通可行任务上直接匹配：
它先按 priority 排序，在有 \(F\) 台 free robots 时取第 \(F\) 名 priority
作为 cutoff，删除所有更低 rank 的任务，并把更高 rank 的任务近似设为
mandatory。后面的 bottleneck、距离、tail 和 continuity 都无法挽回已经
删除的附近任务。

因此第一批 \(\rho\) 修订只改变候选边界，尽量不改变已有 objective：

```text
ready / preparable tasks
  → 物理、因果、custody、mode 与显式 conflict-group 检查
  → 普通任务不再经过 priority top-F cutoff
  → 全部幸存候选进入同一个版本化 assignment 问题
```

允许在更早的 Task-BR 层选择互相兼容的 blocker 方案，但每个未进入矩阵的
task 都必须有可观测的硬约束或冲突组原因。priority rank 本身不是硬约束。
这项修改不承诺最终选择最近任务；远处 critical blocker 只要在同一成本模型
中胜出，仍然可以被选择。

第一组对照保留当前 task-row、robot/dummy columns、
bottleneck-then-sum、EXECUTE 先于 PREPARE、continuity 和 canonical
refinement。这样 benchmark 首先回答“改善是否来自让普通任务获得比较
机会”，而不是同时更换调度目标。

### 8.2 bottleneck 基线继续面向 makespan

对当前 mode 下的候选 task \(m\) 和机器人 \(r\)，定义 guidance 完成估计：

```text
E(r,m) = approach(r,m)
       + immediate_service(m)
       + causal_tail(m)
```

接近、前驱释放和其他可并行事件应使用 event 的 max 关系组合，不把所有
duration 机械相加。基线 assignment 使用：

1. 最小化已选择真实边的最大 \(E(r,m)\)；
2. 在最优 threshold 内最小化距离、work 和有限 continuity 成本；
3. 使用有保证的 canonical refinement 得到确定结果。

未派 task 不能从目标中消失。task-row 模型继续用 dummy 表示本轮延期；普通
task 均允许连接 dummy，不再因高 priority 获得无限 mandatory 地位。若要让
priority 影响选择，应把它写成有限 defer delay，例如：

\[
\operatorname{completion}(m,\operatorname{dummy})
=\operatorname{bestRealCompletion}(m)+\Delta_{\mathrm{defer}}(m),
\]

其中 \(\Delta_{\mathrm{defer}}\) 有界、版本化并单独消融。先运行不改变
priority 数值语义的 cutoff-removal 对照，再测试有限 defer delay，不能把
两项行为变化合并后只报告一个结果。

在 task-row 矩阵中，把同一个 priority 常数同时加到该 task 的所有真实列和
dummy 列只会给每个完整 assignment 加同一个常数，不会改变选择。priority
必须改变“现在服务”和“本轮延期”的相对代价，而不是统一平移整行。

内部 task 的 causal tail 只来自当前所选因果方案，是 guidance，不进入
admissible \(h\)。共享 blocker 的 tail/root impact 不重复计算。

#### 8.2.1 生产版有限延期：只用于同质的直接目标交付阶段

候选边界实验表明，完全取消 cutoff 后仍需解决一个更窄的问题：当当前
EXECUTE 候选全部是在推进各自货架的最终目标时，纯物理 bottleneck 会为了
避免延期一个远端低 priority 任务，打断已经开始的近端交付序列。生产版
`BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2` 因此只在下列**全体候选合同**
同时成立时修改 dummy completion：

```text
dispatch mode == EXECUTE
每个候选 m 都满足：
    m 搬运 TARGET b；
    critical_tail(m) == 0；
    task roots 包含 (b, g)；
    transfer.endpoint == g；
    g 属于 b 的合法 goal set。
```

这一定义排除“目标货架正在替其他 root 清障”的任务，也排除 PREPARE。
只要集合中混入一个匿名 blocker、目标货架 blocker、非终端因果任务或
PREPARE 候选，整个 matching 必须退化为 V1 的
`bottleneck → physical secondary → finite priority tie`。这个全集合门槛是
有意的：混合清障阶段不能让目标 priority 改写主 bottleneck，否则可能重新
饿死刚刚通过 F1 放入矩阵的 blocker。它不读取实例名、地图大小或 seed。

设当前有 \(F\) 台 free robots，候选已按稳定 priority 顺序排列。候选多于
\(F\) 时，第 \(F\) 名的 priority 为有限 frontier；并定义：

\[
\begin{aligned}
I_{\mathrm{frontier}}(m)
  &=\mathbf 1[p_m\ge p_{(F)}],\\
I_{\mathrm{continue}}(m)
  &=\mathbf 1[\text{某台当前 free robot 的 parent anchor 仍指向 }m],\\
\Delta_{\mathrm{defer}}(m)
  &=\operatorname{service}(m)
    \left(1+I_{\mathrm{frontier}}(m)
             +I_{\mathrm{continue}}(m)\right).
\end{aligned}
\]

于是：

\[
\operatorname{completion}(m,\operatorname{dummy})
=\operatorname{bestPhysicalCompletion}(m)
+\Delta_{\mathrm{defer}}(m).
\]

三个乘数分别表示普通延期、有限 priority frontier 延期和放弃尚可继续的
parent assignment。最大乘数固定为 3；它们修改的是主 bottleneck 中的
dummy completion，不只是 secondary tie，但永远不产生 INF、mandatory row
或候选删除。足够大的真实 completion 改善仍可战胜这两个有限项。候选数
不超过 free robots 时没有 dummy，因此该规则不改变 assignment。

V2 仍保留 V1 的低阶 secondary：

```text
physical distance / work
→ finite continuity bit
→ deferred priority tie
→ canonical refinement
```

所有候选、矩阵尺寸、`priority_filtered == 0` 和硬删除原因合同保持不变。
该 objective version 必须进入 telemetry、benchmark CSV 和发布产物；V1/V2
数据不能在同一次搜索或同一结果目录中混合。

### 8.3 additive `S-D` 是独立调度实验

若采用 robot-row additive 模型，定义机器人现在启动 EXECUTE task 的即时
服务成本：

\[
S^E_{rm}
=w_d\,d(q_r,\operatorname{pickup}_m)
+w_s\,\operatorname{immediateService}(m)
+w_c\,\mathbf 1[\operatorname{anchor}(r)\ne m]
+w_{\mathrm{mode}}\,\operatorname{modePenalty}(r,m).
\]

再定义 task 本轮未启动的有限延期损失：

\[
D_m
=\lambda_p\operatorname{urgency}(m)
+\lambda_a\operatorname{transitionAge}(m)
+\lambda_k\operatorname{rootDelayImpact}(m).
\]

对固定候选集合，

\[
\min\left(\sum_{r,m}x_{rm}S^E_{rm}+\sum_m y_mD_m\right)
\quad\Longleftrightarrow\quad
C_{rm}=S^E_{rm}-D_m.
\]

该代数成立，但它把目标改成 additive min-sum，不与 bottleneck/makespan
等价。即使所有 task 都会服务，减去 \(\sum_mD_m\) 也只是常数，不能消除
min-sum 与 min-max 的差别。因此 additive 版本必须先用 full solver 独立
benchmark；不能为了方便复用 Hungarian 或增量对偶状态就把它直接设为默认
production objective。

`immediateService` 只描述当前 approach、准备、Lift 和本次 transfer 的
资源占用。正的 `criticalTail` 不直接塞进该项，否则会反而惩罚应尽早启动的
长关键链；关键链影响应通过有明确物理含义的 `rootDelayImpact` 或其他有限
延期项表达。若使用 age，它只能随真实 primitive transition 推进，不能随
guidance rebuild、节点重访、wall-clock 或 sibling expansion 增长。

### 8.4 idle、PREPARE 与 cardinality

并行数量是手段，不是目标。`robot_utilization` 的 loaded-move ratio 也不是
机器人闲置率，不能据此要求 maximum-cardinality。

additive 实验采用固定机器人行：

```text
rows    = all robots
columns = task/mode slots + N 个 idle/locked slots
```

free robot 可以选择合法 task 或 idle；carrying/不可用 robot 只连接自己的
continuation/locked slot。assignment 直接比较 task edge 与 idle edge 的完整
成本，不先最大化非空 assignment 数量。

PREPARE 只获得部分准备收益 \(B^P_{rm}\)，并满足
\(0\le B^P_{rm}\le D_m\)；不能把 PREPARE 当成 EXECUTE 已完成而抵扣完整
延期损失。同一 transfer 的 EXECUTE/PREPARE columns 属于同一 conflict
group。第一组 bottleneck 对照可继续保留“先 EXECUTE、剩余机器人再
PREPARE”的两阶段结构；是否合并为统一矩阵是后续独立变化。

有界 dispatch lookahead 仍可预测 busy robot 的 release time 和下一件任务，
但只执行当前第一步，不提前改变真实 kappa，也不无限复制虚拟机器人。

### 8.5 Handoff 与 continuity

比较未来完成时间和真实剩余工作，再以有限 continuity 成本稳定接近的方案。
旧 owner 已走距离是沉没成本，不能硬加成未来损失；也不能用无限 lease
保证永久 owner。当前 assignment 是 `mate`，而计算本矩阵 continuity 时
实际使用的上一代绑定是 `anchor_used`，两者必须区分。

一次 augmenting path 可能同时更换多台机器人的 mate。进入子节点后，这些
新 mate 才成为下一代 anchors，因此即使机器人没有移动，其 row cost 也
可能因 anchor 更新而变化。

### 8.6 增量 Hungarian 只加速冻结后的决定

在相同候选、成本矩阵、objective 和 canonical tie 下，精确 incremental
solver 必须返回与 full solver 相同的 assignment。因此仅保存 duals 不会
自动减少 retarget；retarget 的变化来自候选、urgency、continuity、idle 或
mode 语义，CPU 变化才来自增量求解。

标准 Hungarian 状态直接维护 additive objective。若 G 阶段最终采用
robot-row `S-D`，机器人位置或局部 eligibility 变化主要对应 row 更新，适合
复用仓库已有的 `TAPFAssignmentState::repair_rows()` 和 ITA-CBS 单行增广
经验。若最终保留严格 bottleneck，则必须另行维护最小可行 threshold 和
threshold 内 secondary matching，不能把 additive row repair 原样套用。

增量状态必须属于具体 LaCAM 节点，至少保存：

```text
mateL / mateR
row / column potentials
ColumnModelVersion
RowFingerprint[r]
anchor_used[r]
objective value
canonicalization version
```

`ColumnModelVersion` 覆盖 ordered task/mode identities、实际
service/urgency/age/root-impact 数值、endpoint/conflict 版本、objective、
fixed-point scaling、INF 和 tie 版本。任务身份没变但列数值改变时也必须
失效。第一版只有完整列模型相同时才修复 changed rows；列结构或任意列数值
改变均 full solve。

不能假设每个 joint transition 严格只改一台机器人，应支持
`repair_rows(changed_rows)`：0 行直接复用，1 行单次增广，\(k\) 行逐行修复。
实现还必须明确矩形矩阵、负有限成本、64 位 checked arithmetic、INF 与
canonical matching 契约，并用 full solver 做逐节点 shadow oracle。

## 9. 联合运输 guidance 与 Carrier-PIBT

### 9.1 保留 endpoint，不固定 route

一旦 Lift 启动 transfer，默认保持 shelf、carrier 与合法 endpoint；route、
预计通过时间和 LegId 可从当前真实状态重算。它是一项 preferred completion
commitment，不是“未来一定无死锁”的证明，也不是 physical successor 限制。

normal reroute 不取消 TransferId（现状 `custody_physically_valid()` 的
route 后缀检查必须放宽为 endpoint/episode 检查）。forced Move 先经过
`apply_ops`，再优先保留原 endpoint 重路由。若原 endpoint 不能产生
preferred route，显式记录 no-route，不把有限搜索失败等同于物理不可达；
必要 recovery 候选仍可使用其他合法 endpoint，但必须标明重绑定原因
（现状 `make_storage_recovery_custody()` 直接选最短任意 endpoint，需要
改成先试原 endpoint）。

episode 有效性与“这一轮找没找到路线”必须彻底解耦，拆成三个独立判断：

```text
PhysicalBindingValid: kappa、货架位置、carrier 与真实 transition anchor 一致
EpisodeActive:        episode 尚未完成、取消或显式重绑定
RouteHintUsable:      本轮 route 提示存在，且与当前状态/预测一致
```

`RouteHintUsable` 为假时，前两者可以仍为真。route helper 预算耗尽或通道
被暂时挡住时的规范状态是：

```text
custody         = 原 TransferEpisode（TransferId/carrier/endpoint 不变）
preferred_route = none
preferred_leg   = none
route_status    = TEMPORARILY_BLOCKED | BUDGET_EXHAUSTED | ...
```

绝不允许“找不到 route → custody 消失 → 原 endpoint 不再约束 guidance →
下一轮重找任意 endpoint”。`loaded-unbound` 只保留给确实没有可恢复
episode 的情况（例如 constraint tree 强制 Lift 了一个没有 assignment 的
货架），不是普通求路失败的通用状态。

必须新增回归：已有合法 custody，强制 route helper 零预算，执行合法
Wait；TransferId、carrier 与 endpoint 必须保持。之后通道释放或预算恢复，
继续同一 episode。

### 9.2 时序路线的生成范围

对 active custodies 和 grounded/preparing assignments，建立有界、按冲突
分组的 time-expanded route guidance：

```text
state = (cell, relative_tick)
actions = adjacent Move | Wait
destination = committed storage endpoint
```

搜索使用当前几何与 shared timed reservations。空 loaded 路线、不同时间
交点、同向 following、对向 edge、endpoint arrival/hold 全部按模型区分。
free robot 的预测路径若可得也进入 lower occupancy 视图；预测不全时不得
宣称整个 horizon 是 robot-shelf collision-free。

给定完整已验证预测轨迹的局部候选，可以声称“在该预测条件下无冲突”；只有
`apply_ops(X, joint_op)` 验证后的下一步可以无条件接受。整个剩余 route
不是未来执行保证。

有界 horizon 不足以到达 endpoint 时（例如 endpoint 在 40 格外而 horizon
只有 16），helper 的返回必须区分三种：

```text
完整 endpoint route hint
条件性 prefix hint（只到 horizon 内某个安全前缀）
当前没有 preferred hint
```

prefix hint 仍只是基于当前预测的 guidance，不是未来安全证明，也不解锁
corridor Drop；不得因为“没到终点”就让载货车永久 Wait，也不得把计算预算
不足再次转化为隐性的任务禁止。

### 9.3 预约覆盖对象

每一件实际存在的货架都必须在预测视图中有占据解释：

- grounded 待接货架：在实际/预测 departure 前占据 source，包含 approach
  和 Lift；
- carried 货架：从相对时刻 0 的真实位置开始，占据其预测路径；
- 没有 preferred route 的货架：在已知范围内保留实际占据，不能从
  reservation table 消失；
- 抵达 endpoint：保留后续占据；Drop 只改变 carrying mode，不让 endpoint
  变空；
- horizon 以外：不得当成已知空闲；用保守 terminal occupancy 或 unknown
  状态。

other carried shelf 的当前位置不能同时被永久 static-block 和 timed
trajectory 两套规则重复解释。暂时静止的预测若阻塞所有候选，应允许本次
联合协调回退其首选轨迹，而不是用自造的静态 hold 证明无路。

### 9.4 Release time 与重规划

grounded task 的预计出发时间来自 robot approach、Lift 和相应 causal
release event。未分配的 predecessor 没有可信 departure，不能凭空生成未来
空位。预计时间只影响执行层，不进入 tau。

每个真实 transition 后校正轨迹锚点。可以复用未失效的几何 suffix，但发生
Wait、PIBT deviation、owner 改变、前驱延迟或新冲突时，要更新受影响的
时间表。宏展开同样逐拍处理。

只在 Lift 时规划不够：robot approach 期间已有已知 route conflict 时就应
保留替代方向；尤其不能等两台载货车进入无会车空间的走廊后才第一次协调。

### 9.5 回退，而非只让低优先者永久 Wait

按稳定优先级先选路线是候选生成策略，不是绝对通行权。低优先 transfer 失败
时，在工作预算内尝试：同 endpoint 绕路、调整出发时间、改变冲突对的通过
顺序、重新选择先前高优先 transfer 的 route。rollback 撤销该尝试的所有
预约。

一个辅助搜索失败只返回 no-preferred-route/partial guidance。不要删
transfer，不修改 tau，不把该状态判无解。

若本轮所有 robots 都 Wait，而所等待的“未来释放”没有任何实际执行者，应
回退造成等待的预测安排。不能依靠墙钟时间或同一 X 的重复 attach，让一个
虚构 reservation 自动消失。

### 9.6 Candidate ordering

```text
已经到合法 endpoint：
    若需完成 target 或释放 carrier：优先 Drop
    若有有效同棚 continuation：比较继续搬与释放，不强制逐段举放

carrying + route 可用：
    推荐 next leg / 合法等待 / 其他合法 Move / 合法 storage Drop

carrying + no preferred route：
    Wait 和其他合法 Move；在 storage 保留合法 Drop
    必要时生成 recovery 候选

free + assigned/preparing：
    approach；条件允许时 Lift；否则等待或避让
```

当前 joint constraints 必须先被尊重。route 与 endpoint 偏好都不能删除
fully constrained 的合法 Move/Lift/Drop。生产 storage-only Drop 规则始终
由 oracle 保持。`funcPIBT()` 现有的 loaded-unbound transit 行为（首选
Wait，不做无状态贪心 retarget）与完整 fallback 候选保留。

### 9.7 比较“成功但慢”的联合方案

只在失败（no-route）时回退是不够的。示意：

```text
方案 1：A 走最短路，B 等待再走      A=20, B=40, makespan 40
方案 2：A 多绕一步，B 可同时通过    A=21, B=20, makespan 21
```

方案 1 没有失败；若只有 `NO_ROUTE` 触发回退，方案 2 永远不会被比较。
运输 helper 在工作预算内应允许保留并比较少量完整候选 frame：

```text
Score(candidate frame) = (T̂_all_targets, Ŵ, 稳定性次序)
```

`T̂_all_targets` 不能只计入本轮成功排上的 transfers：未分配、被暂停、仍有
后续搬运的 root 都必须保留剩余代价估计（§6.5 的同一风险），否则“少安排
几件事”会虚假地更快。该 Score 只是 guidance 评分，不进入 admissible h；
不要求每次找到最优，只要求“当前方案有路”不等于立即停止全部替代尝试。

## 10. Transition、cache 与 rewire

保留已实现的不可变 `SearchEdge` trace、candidate edge 登记、stale parent
先刷新（`ensure_guidance_fresh()` + `guidance_stale`）、逐拍重锚和 frozen
`constraint_order`。新增时间/work 字段后，所有读取必须来自同一 edge
record。

| 数据 | 输入/生命周期 | 更新规则 |
|---|---|---|
| PairCost/tau | U + immutable version | 只在 U 变动或版本变动时重新评价 |
| D0/upper priority | U + commitment key | 不读取 robots；缓存值不可被执行层原地改写 |
| ExecutionView | D0 + X + 实际 custody | 每个真实节点重新对齐；可标 active、fulfilled、pending、shadowed |
| rho candidates/cost model | 当前 view、robot positions、mode、上一 episode binding、objective version | 每个节点重建或验证完整 `ColumnModelVersion`；不改 tau |
| rho matching/duals | 具体 LaCAM 节点及其真实 parent | 只在列模型相同且 row fingerprint 改变时增量修复；不得进入 upper cache |
| route/timed reservations | X + 当前 jobs + transition anchor | 时间每步校正；几何按失效事件重搜 |
| admissible h | X + problem objective | 独立计算/缓存，不读取 D0/route/lease |

\(\rho\) 的增量缓存分为不可变列模型和节点局部动态状态：

```text
ColumnModelVersion:
    ordered task/mode identities
    service / urgency / age / root-impact 的实际固定点值
    active / endpoint / conflict-group version
    objective / scaling / INF / canonicalization version

RowFingerprint[r]:
    robot position
    free/carry/custody 与 mode eligibility
    本矩阵实际使用的 anchor_used

RhoAssignmentState:
    mateL / mateR
    row / column potentials
    ColumnModelVersion
    RowFingerprint[]
    anchor_used[]
```

父节点的 state 可以复制到真实 child，再按 changed rows 修复；duplicate、
rewire 或 sibling 分支只能从经过验证的真实 parent 获取自己的可变副本。
只按 `UpperSignature` 共享 matching 会混入另一条分支的机器人位置和
continuity 状态，属于错误。

第一版的安全规则是：完整 `ColumnModelVersion` 相同才允许 row repair；
任何列 identity、列数值、mode、conflict、objective 或 tie 版本变化都
full solve。若使用 age，它必须属于可从搜索节点/路径重放的 transition
状态，不能存进 U-only cache，也不能随 guidance 被重复构造而增长。

in-flight transfer 在新 D0 中消失，并不使 custody 自动丢失（现有
`compatible_task_index_by_custody` 的 derived index 语义保留）。
ExecutionView 导入真实 active episode，移除对同一 shelf 的重复派工，重查
依赖条件；active endpoint 冲突导致暂时等待或显式重新选局部方案，不写入
全局 U-only cache（`UpperEpochCache` 继续只含 U+commitment 键值）。

`U` 不含 carried/grounded mode（`make_upper_signature()` 把 carried
匿名货架按 robot 坐标折叠进 `anon_pos`，无任何 mode 位），因此 **D0 不能
也不得区分 grounded blocker 与 carried blocker**：它只按坐标记录“cell u
需要被腾空”及候选清障方案。grounded/carried 的判断只发生在
ExecutionView 对齐，转换规则：

```text
occupant grounded：            保留可派工的 clearing transfer
occupant 已被 carry：          关联真实 active episode 的 vacate 事件，
                               不再创建第二个实际执行者
u 已经空：                     该占据条件当前已满足（fulfilled）
active episode 与原方案不兼容： 显式记录未解决条件或选择替代方案，
                               不得直接标 fulfilled
```

四个视图状态的精确含义：`active` = 有真实 carrier 正在执行同一
TransferKey；`fulfilled` = 对应物理条件当前成立（可因重新占据失效，
§6.2）；`pending` = 条件未满足且暂无执行者；`shadowed` = 已有别的
active episode 覆盖同一要求，本方案不再派第二个执行者。测试必须使用
同一 `U`、不同 `kappa`：cached D0 逐位相同，ExecutionView 可以不同；
任何一侧都不得凭空产生第二台 carrier，也不得丢掉尚未满足的清障条件。

重锚只来自 `{previous_X, previous_guidance, executed_ops}`。新的 route
选择可以发生于 reanchored 当前 X，但“新选择”与“物理事实恢复”使用不同
阶段，不伪造已经发生的 Lift/Move。

## 11. One-empty、zero-empty 与 endpoint

通用递归仍自然得到 `C vacates → B moves → A moves`。无需 one-empty 特殊
compiler；预算限制、循环或候选耗尽也不保证一定找到有效链。

对所有可通行格均可存储且禁止 following 的一空格模型，同一拍能够进入当前
空格的 shelf 受唯一空格限制。生产允许 following 或具有额外 transit cells
时，不能推广成“任何时刻全场只能执行一个任务”。approach、Lift、多个
transport legs 都可能重叠。

zero storage slack、zero current empty storage、zero empty upper
traversable cells 是三件不同的事。原 rotation-record 与 exhaustive
physical search 保留；同步 bundle 可后续优化，不能因为 compiler 没有
ready leaf 而删除可行物理 successors（`zero_empty_no_ready` 诊断保留）。

endpoint ownership 表示已选放置方案的未来占据，不是不可逆的最终完成。
已经落位的 target 仍允许被搬开。搬运方案形成 cycle 时需另选
endpoint/方向，或在模型允许的同步组合中解决；不能用无条件 task-DAG 成功
假设掩盖 cycle。

## 12. Makespan-first 搜索、修补与保证

### 12.1 Cost API

```cpp
struct PlanCost {
    int64_t ticks;
    WorkCost work;
};
// 比较：先 ticks，再 work；不使用 T + epsilon*W 近似。
```

```text
edge.cost = (trace.size(), sum work of all actual joint ops)
node.g    = parent.g + edge.cost
node.h    = (h_T(X), h_W(X))
node.f    = node.g + node.h
```

现状是 `TAPFNode.g/h/f: double` 与 `SearchEdge.physical_cost: double`，
`get_edge_cost()` 只算 weighted work（全 Wait 的 joint op 记 0）；必须改为
上述结构。rewrite、OPEN 的界、goal incumbent（含
`TAPFSearchConfig.incumbent_init` 外部上界）、两遍候选选择与 repair 使用
同一比较器。在静态、无外部绝对时钟约束的物理问题中，同一 X 的较早
lexicographic g 支配较晚到达；timed reservations 只是 guidance，不改变这
个性质。

若 `g_T+h_T > T_inc` 可剪；若时间下界相等，仍可能改进 work，应再检查
`g_W+h_W`。等价地，在两个分量均为对应下界时使用 `f >=lex incumbent`。
不能继续沿用只针对 SOC 的 scalar f。

work 分量的表示与比较契约：

- 单位权重下 work 用整数计费；支持小数权重时，用显式声明的固定精度或
  有理数计费语义（例如按公共分母放大为整数），保证所有模块共享同一个
  稳定比较器；
- search、rewrite、OPEN、两遍候选与 repair 必须使用**同一个**“更好”
  定义。epsilon 只用于诊断数值误差，不得进入 OPEN 排序、全局比较器或
  接受条件：“差小于 epsilon 算相等”不具传递性（`0≈0.75ε`、
  `0.75ε≈1.5ε`，但 `0` 与 `1.5ε` 不相等）；
- 权重域显式声明并在入口校验：`alpha,beta,gamma,delta >= 0` 且有限。
  现状 `DD_ALPHA..DD_DELTA` 由共享 env parser 读入，无域校验。`h_W` 的
  “忽略其他动作”下界论证依赖非负权重：若允许 `beta<0`，一次额外 free
  Move 即可使真实后缀 work 低于估计，`h_W` 就会高估。

### 12.2 首解以后

保留快速首解和同一个 `TAPFPlanner::solve()`。首个可交付解产生后，剩余
预算用于同一实现的 makespan 改进。固定首解终态 assignment 的重跑（现有
第二遍）只是一种候选尝试，不是 singleton 实例跳过所有改进的理由。

第一批可以保留两遍控制器，只改比较目标并单独验证；后续接通普通首解后
继续探索。首解后的全部改进尝试由**同一个求解控制器**管理剩余预算、
可交付 incumbent 与后续搜索尝试，不得另起第三套 deadline/返回路径。
不同阶段通过代码提交与实验变体对照，不在 production 添加 legacy
fallback。

macro rollout successor 现只在首 incumbent 前插入（`macro_after_first`
守卫）；该结构保留，macro edge 的时间成本按其 trace 长度计。

修改 g 并不意味着 DFS 首解自动变好；route/dispatch guidance 和剩余时间内
的替代搜索决定有限预算表现。未经完整公平展开和 frontier lower-bound
证明，不声称一般实例 eventually optimal。

### 12.3 Repair 与 deadline

repair 继续做原来可重放的 exact-state / grounded-shelf-projection 变换，
但接受条件改成 `candidate.cost <lex original.cost`（现状是分段与总量
SOC 非增 `SOC_EPS`）。允许少用时间而多走几步；相同时间也可接受更低
work。

保留 strict total deadline：搜索、清理、修补、成本计算和最终 replay 都占
预算（`dd_classify_finalization_probe` 的 ACCEPT/DEADLINE 分类不变）。
第二候选超时或非法时保留第一份已经完成验证的可交付 incumbent。不得因尝试
一个更好的 raw candidate 而丢掉它。

修补后的外部 incumbent 是上界，不能直接覆写原 search node 的 g；若希望把
修补路径加入搜索图，必须注册其真实 trace。

首次 goal 前缀是强制规范，不是可选修补：

```text
NormalizeGoalPrefix(plan):
    从初态逐拍重放；
    在第一个满足原问题 goal 的状态终止计划；
    T 与 W 只按该前缀计算。
```

所有交付计划、候选比较与统计（含 `T = plan.size()` 这类读法）都必须先
经过该规范化。macro 途中已达物理 goal 时，必须注册到该 goal 状态的前缀
边，不得保留更长 trace 的终态 key 却只把 cost 改短。边界测试：初态已是
goal 时 `T=0`；macro 中途达 goal 正确截断；goal 之后的匿名货架动作不计入
返回计划的 T/W。

### 12.4 兼容性边界

不改变物理模型、operator set 或 SearchKey。zero-shelf 原 TAPF 在其原
objective/API 下继续要求逐位兼容；若请求的 objective 本身改成 makespan，
就不能同时无条件要求输出与旧 SOC 搜索完全相同。

通过现有 solver 的显式 objective/cost 接口表达问题目标，不通过检测实例名
或“有没有货架”切换另一套 planner。shared search 内仍只有一个执行流程。

### 12.5 完备性

需要同时满足：有限物理状态、固定完整 operator 枚举、每次有限的 guidance
预算、fully constrained 直通 oracle、无不合法剪枝、必要节点最终有机会
展开。route/dispatch 只调整偏好，不能重置并丢弃原 constraint-tree 的未
枚举项。

“保留 endpoint”本身不是安全或活性证明；“有一条无碰撞预测 route”也不
意味着真实机器人一定按时执行。正确性仍由每个实际 transition 与完整输出
重放保证。

## 13. 代码修改落点（S4 历史基线 + S6 当前 \(\rho\) 审计）

本节各小节原有的“现状/修改”主要记录 commit `03e99ba` 到 S5 v5 的历史
迁移，旧行号是历史锚点，是否已实现以 §23 为准。§13.4 及 §13.5 新增的
\(\rho\) 条目描述 commit `80148a7` 之后的 S6 后续工作。新增类型和 helper
名是建议接口，落地时对照实际源码命名，不假定仓库已存在。

### 13.1 类型与身份 —— `lacam/include/tapf_planner.hpp`

现状（S4）：`TaskId{shelf,from,to}`（l.129）是一步 exact effect；
`StorageTransfer{endpoint,route}`（l.163）的相等比较含整条 route；
`Custody`（l.198）带 `transfer/transfer_index` 的 route 后缀绑定；
`TAPFNode.g/h/f` 与 `SearchEdge.physical_cost` 是 `double`；
`CarrierGuidance` 只有
`upper_epoch/ready_tasks/rho_task_id/rho_ready_index/custody_by_robot`。

修改：

* `TaskId` 保留为 `LegId`（物理 successor 去重、`funcPIBT` 首选 leg 继续
  使用）；
* 新增不含 route 的 `TransferKey{shelf, source, endpoint}` 与
  branch-local `TransferId`；`StorageTransfer` 的身份/等价判断降为
  endpoint（route 是可重算 hint），或在 `Custody` 增补
  `original_endpoint/rebind_reason` 字段实现同等语义；
* 新增 `PlanCost{ticks, work}`；`SearchEdge` 记录
  `(trace.size(), work)` 两个分量；`TAPFNode.g/h/f` 改为 `PlanCost`；
  `TAPFSearchConfig.incumbent_init` 改为可表达两分量的上界；
* `CarrierGuidance` 增加 ExecutionView 输出（active episodes、
  assignable/preparable 集、no-route 标记）与 bounded timed transport
  guidance；这些字段不进入 `UpperEpochGuidance`/`UpperEpochCache`；
* 若 H 阶段接入增量 additive matching，新增节点局部
  `RhoAssignmentState`，包含 matching/duals、`ColumnModelVersion`、
  `RowFingerprint`、`mate` 与 `anchor_used`；它随 parent→child 复制，
  不进入 `UpperEpochGuidance`；
* `TAPFStats` 增加真实 T、首解 T、timed-helper 时间/展开数、causal/
  traffic waiting 诊断，以及 \(\rho\) 候选各层数量、full/reuse/repair/
  fallback 次数、changed rows、matching 构造/求解/复制耗时。

### 13.2 启发式与候选核 —— `lacam/src/carrier_guidance.hpp`

现状：`solve_tau_guide()` l.389（min-sum Hungarian + 确定性 tie）；
`solve_tau_lb()` l.694（weighted-work admissible LB，即 `h_W`）；
`pair_cost_prefix_lower_bound()` l.371 / `build_pair_cost_table()` l.376
（lazy-exact certificate）；`reachable_storage_transfers()` l.1160 与
`ordered_shelf_candidate_window()` l.1206（确定性 BFS candidate core，
每 endpoint 一条最短 route）；`upper_vacancy_count()` l.90。

修改：

* 新增 `h_T = max(h_bottleneck, h_work)`（§5.2），节点 h 变为
  `(h_T, h_W)`；`solve_tau_lb()` 保留为 `h_W`；小图 oracle 对照测试；
* `tau_guide` 本轮不换 min-sum（§5.1）；PairCost 不重写（§4）；
* candidate core 保留；新增执行层“同 endpoint 重新寻路”入口（复用同一
  拓扑与 endpoint 约束的 BFS/时序搜索），canonical route 只作估价样本；
* 按 §2.3 增加 `empty_storage(U)/empty_transit(U)` 帮助函数；
  `upper_vacancy_count()` 保留为静态 storage_slack。

### 13.3 Readiness、custody 与 claims —— `carrier_guidance.hpp`

现状：`ActiveTransferClaims` l.2588（endpoint + route 内部格）；
`ready_tasks_with_custody()` l.2757 尾部按 claims 过滤 ready 任务（含
ready 任务彼此 route 互斥）；`custody_physically_valid()` 的 route 后缀
检查；`recover_task_br_custody()` l.2914 的三类恢复；
`make_storage_recovery_custody()` 选最短任意空 endpoint；
`bind_ready_continuations()` 的 roomy/dense reverse suppression。

修改：

* 删除 ready 过滤中的 full-route 空间互斥；保留真实 occupancy、custody
  去重与 endpoint 放置冲突语义（§6.3/§21）；
* readiness 拆为 `Assignable/Preparable/MoveExecutable`（§7.1）；最小
  阶段先保证 causally-ready 全部进入候选，第二阶段加有界 preparation；
* custody 有效性按 §9.1 拆成三谓词：`PhysicalBindingValid`（kappa/坐标/
  anchor）、`EpisodeActive`（episode 未完成/未取消/未重绑定）、
  `RouteHintUsable`（本轮提示可用）。route 检查只属于第三个谓词；同
  endpoint 换 route、等待、route helper 失败都不丢 custody；当前
  `LegId` 由所选 route 重算，无 route 时置空并记 `route_status`；
* forced deviation 恢复顺序：先尝试保留原 endpoint 重路由；失败记
  no-route（episode 仍保持，`preferred_route=none`）；仅必要时换
  endpoint 并标注 rebinding 原因（改
  `make_storage_recovery_custody()` 的排序与接受规则）；
  `loaded-unbound` 只留给无可恢复 episode 的强制 Lift；
* claims 的“时间性”改由 §9.2–§9.5 的 timed reservations 表达，仍留在
  cache 之外；
* 审计 compiler（`TaskBRCompilerState/Transaction`，l.864 起）的全部
  transfer 接纳条件：storage endpoint 的放置冲突保留联合处理；非 storage
  first-step 的 `reserved_destination` 只保留当前同拍动作选择意义，不得
  让“两个未来 first legs 共用一个 transit cell”把整个 transfer 挡在候选
  图之外（§6.4）。只删下游 `ready_tasks_with_custody()` 的过滤不够，
  上游接纳条件必须同步修正；测试要覆盖“第一格就是同一 transit cell、但
  可错时出发”的情形。

### 13.4 rho —— 当前 `match_ready_tasks()` 与后续分层修改

S6 当前现状：task-row × robot/dummy columns；按 `TransferKey`/shelf
去重后按 priority 排序；top-\(F\) cutoff 删除低 rank 普通候选，高于
cutoff 的候选近似 mandatory；幸存者执行 bottleneck-then-sum assignment
与 canonical refinement。EXECUTE 先匹配，剩余 robots 再做 PREPARE。
`CarrierGuidance` 没有节点局部 matching/duals，每次 full solve。

修改严格拆为三层：

1. **候选边界：**删除普通 priority top-\(F\) cutoff 和无限 mandatory；
   输入改为通过物理、因果、custody、mode 与显式 conflict-group 检查的
   全部候选。保留当前 task-row、dummy、bottleneck、secondary、
   continuity、canonical 和 EXECUTE→PREPARE 顺序，形成行为对照。
2. **调度目标：**在上述对照后，独立测试有限 dummy defer delay；再实现
   §8.3 robot-row additive `S-D` full solver。idle 合法，PREPARE 只获得
   部分收益，不使用 maximum-cardinality。该层改变 objective，必须单独
   benchmark。
3. **求解加速：**只有 additive full solver 的候选、成本、矩形/负成本、
   64 位范围和 canonical assignment 冻结后，才接入 §8.6 的 node-local
   incremental Hungarian。列模型变化 full solve，列模型不变才
   `repair_rows(changed_rows)`，debug/shadow 模式逐次对照 full oracle。

若 benchmark 最终保留严格 bottleneck，不直接复用 additive
`TAPFAssignmentState`；另行实现 threshold 可行性与 threshold 内 secondary
matching 的动态维护。

### 13.5 attach 管线 —— `build_task_br_guidance_from_upper_epoch()`
l.3651 与 `tapf_planner.cpp::attach_carrier_guidance()` l.160

现状管线：recover custody → ready（含 claims 过滤）→ bind
continuations → grounded ready → rho；无 ExecutionView、无 preparation、
无 timed routes；`UpperEpochCache`（U+commitment 键）纯度边界正确。

修改（对应 §19 伪代码）：

* recover 之后加 `ReconcileCausalGraphWithActualEpisodes` 产出
  ExecutionView（active/fulfilled/pending/shadowed 标记）；
* `FindAssignableTransfers` + `FindBoundedSafePreparationCandidates`；
* 在 dispatch 前构造显式候选审计记录：input、claim/conflict 后、
  same-key/shelf 后、priority 阶段后数量及每个删除原因；priority 修订后
  普通候选的“priority 阶段删除数”必须为 0；
* full/incremental additive 阶段生成 `ColumnModelVersion` 与各机器人
  `RowFingerprint`，只从经过验证的 parent 继承 `RhoAssignmentState`；
* dispatch 之后 `BuildBoundedJointTransportGuidance`（time-expanded
  routes，§9.2–§9.4），失败只产生 no-preferred-route/partial guidance；
* upper cache 内容与读写边界不变；执行 overlay 不回写缓存。

### 13.6 执行 —— `tapf_planner.cpp::funcPIBT()` l.1231

现状：loaded+bound 首选存储 route 的下一格（exact `custody->to`）；
loaded-unbound 在 storage 首选 Drop、在 transit 首选 Wait；free+assigned
就地 LIFT/按 lower distance approach；完整 oracle fallback。

修改：首选 leg 改为消费动态 route（timed guidance 或同 endpoint
reroute 的当前建议），不再只认 custody 存储 route；到达 endpoint 后按
§9.6 比较 Drop 与同棚 continuation；其余候选顺序、S1/lift/drop guards 与
完备 fallback 不变。

### 13.7 搜索目标 —— `tapf_planner.cpp`

现状：`get_edge_cost()` l.1069 纯 work；`solve()` l.506 标量 incumbent
剪枝（l.629）；`rewrite()` l.977 标量 g/f 传播；首次 attach 加
`solve_tau_lb` 到 h（l.231–235）；macro 只在首 incumbent 前（l.654–688）；
`carrier_rollout()` l.1755 返回 trace。

修改：edge cost 改 `(trace_steps, work)`；node `g/h/f` 改 `PlanCost`；
incumbent/OPEN/rewrite/macro/f-prune 全部换 §12.1 词典序比较器；
`first_solution_*` 与 goal 更新记录 `(T,W)`。新增显式 objective/cost
契约参数（入口放在 `TAPFSearchConfig` 或 instance）：由调用方声明目标——
zero-shelf 原 TAPF 调用方继续请求旧标量 objective 并保持逐位兼容；
carrier 调用方（`dd_planner.cpp`）请求 `(T,W)`。同一 engine 按该契约产生
edge cost 与 h，不按“有没有货架”猜测目标，也不让默认入口在类型迁移时
暗中改变目标。

### 13.8 两遍与 repair —— `dd_planner.cpp` / `dd_plan_repair.cpp`

现状：`plan_soc()` weighted work；两遍比较 `if (soc2 < soc)`（l.362）；
`finish()` 全量 replay + `is_dd_goal` + finalization probe；repair 接受
条件 SOC 非增（`SOC_EPS`，l.389/415）。

修改：两遍/全局候选比较与 repair 接受条件统一 `(T,W)` 词典序
（§12.2/§12.3）；保留 replay 合法性、strict deadline、“保留已验证
incumbent”；stats/benchmark 行增加真实 T 与首解 T。注：`dd_planner.cpp`
注释引用的旧 `debug.md §10 R1`（repair 共享 pass deadline）语义保留，
S6 的 F/G/H 实施清单以本稿 §14 和 §24 为准。

### 13.9 oracle 与 validator —— `dd_carrier.cpp` / benchmark Python

物理规则原则上不改；`is_dd_goal`（`dd_carrier.hpp` l.104）与
`apply_ops`（l.108）保持终点与转移语义。只核对成本报告/时间定义和
replay 一致（双侧 validator 输出的 makespan 与 solver 的 T 同义）。

### 13.10 tests / CMakeLists.txt / benchmark

* `tests/test_dd_storage_transfer_claims.cpp` 保护 route-claims 过滤，
  按 §15 先审查再迁移；`tests/test_dd_plan_repair.cpp` 的 SOC 非增断言改
  `(T,W)`；`tests/test_dd_objective_*` 的标量目标行为逐项审查迁移；
* 物理/交付契约测试原样保留（storage-only Drop、successor 完备性、
  replay、strict deadline、zero-shelf compatibility）；
* 新测试矩阵见 §16；`dd_planner.hpp` 现有 probes（l.105–149）按新谓词
  扩展（ExecutionView/timed-route probes），不另建平行探针体系；
* 新增 \(\rho\) probe/shadow oracle：记录候选删除阶段、每条成本分量、
  objective/canonical assignment、full/reuse/repair/fallback 原因；
* benchmark runner 报告增加 §17.1 指标；release 协议不变。

## 14. 分阶段实施顺序

实现遵循 `test -> RED -> implementation -> GREEN -> benchmark ->
regression -> debug`。A–E 是 S5 v5 的历史实施顺序，证据见 §23；F0/F1
和收紧后的 F2 V2 已由 S7 production 实现。G0/G1 与 H1 shadow 做过独立
实验后完整回滚，因为 additive 目标把 Testcase C 从 `(31,93)` 退化到
`(231,268)`；因此 H2 未进入 production。任何阶段都只修改现有
execution path。

**A. 契约与证据冻结。** 固定 S0/S1/S2、source/binary SHA、YAML 字节和
全部旧计划。明确哪些 tests 是物理语义，哪些只保护将被修改的策略。

**B. 成本契约接通。** 明确 objective 显式接口、严格词典序比较器与
整数/定点 work、权重域校验、正确的时间下界（含 mixed 的
`h_TAPF_time`）、首次 goal 前缀规范化。先以微型计划测试 `(54,90)` 与
`(31,93)` 的比较、macro cost、rewrite、repair 与 h；小图 oracle 对照。
单独记录该提交的性能，不假定首解会改善。

**C1. 任务连续性。** Transfer/Leg 分离；episode 有效性与 route 成功完全
解耦（§9.1 三谓词与 `route_status`）；最小 ExecutionView 对齐（active
episode 去重、grounded/carried 分类、§10 四状态语义）。

**C2. C 的运输修复。** 删除上下游两处未来 route 假互斥（compiler 接纳
条件中的非 storage first-step 预约 + `ready_tasks_with_custody()` 的
claims 过滤）；接入最小冲突组 timed coordination——同 endpoint 等长
绕路、错时出发、通过顺序与有限回退（§9.2/§9.5/§9.7 的最小实现，不必
覆盖整仓）；保留原 rho。先查 Testcase C，再检查原 storage 往复回归和
dense suite。

**D. 因果事件与准备。** 将等待整 transfer 的边细化到必要 vacate/use
事件，受控开启后继 approach；不得抢走前驱必需机器人；资源不足时防自锁。
测试当前空 storage 与净 storage slack 的区别。

**E. v5 质量增强基线。** 完成时间派工
（earliest-finish/critical-tail、瓶颈匹配）、有界 dispatch lookahead、
更强 route 候选比较（§9.7 完整 Score）、首解后改进（同一控制器）。
这些内容的 2026-09-05 实现证据保留在 §23；S6 证明它仍包含 priority
top-\(F\) 候选门槛，不能把 §23 当作后续 \(\rho\) 修订的验证。

**F. \(\rho\) 诊断与候选边界。** 先不改变 assignment 结果，拆分候选生成、
各层过滤、矩阵构造、bottleneck、secondary、canonical 和复制耗时；记录
每个 task 的删除原因、最近机器人距离、changed rows 与列模型变化。随后只
删除普通 priority top-\(F\) cutoff/mandatory，保留当前
bottleneck-then-sum、dummy、continuity、canonical 及
EXECUTE→PREPARE，运行目标实例和 full benchmark。

**G. 新调度目标 full-solve 实验。** 先做同 bottleneck objective 的有限
dummy defer-delay 消融；再实现 §8.3 的 robot-row additive `S-D`，明确
service/defer/idle/PREPARE/age/conflict 语义，每次 full solve。G 的 full
solver 是 H 的唯一行为 oracle；F 因 objective 不同，不能代替它。

**H. 精确增量 matching。** 在 G 的数学问题和 canonical assignment 冻结
后，接入 node-local matching/duals、`ColumnModelVersion`、
`RowFingerprint`、`mate/anchor_used` 和 `repair_rows`。列结构或数值变化
安全 full fallback；shadow 模式要求 objective 与 canonical assignment
逐次等价，再衡量 CPU、复制成本与内存。

把解决 C 的关键时序能力放在 C2 而不是 E；C1 先于 C2，避免在删除过滤后
用“route 是否存在”判断 custody。F、G、H 必须分别提交和 benchmark，不能
把调度语义变化伪装成增量求解收益。阶段是实现提交，不是新增 production
运行时策略开关。最终仍只有一个 Carrier-LaCAM pipeline。

## 15. Protected tests 的迁移

必须先审查再修改以下旧契约（迁移遵循 `rules.md` 的独立 reviewer 流程）：

- `non-ready internal tasks 永不进入 rho/不得 approach`：改为未满足
  movement 条件不能执行该 move，但满足准备条件可接近；
- `same first-leg effect 合并全部 roots`：改为完整 transfer 效果兼容才
  合并，LegId 只去重物理动作；
- `custody remaining suffix 必须完全相同`：改为 episode
  endpoint/physical binding 连续，route 可变；
- `任何空间 route overlap 都删除低优先任务`：删除，改成按时间协调
  （现锁定于 `test_dd_storage_transfer_claims.cpp`）；
- `priority 前 |free| 名之外的普通任务不得进入 rho`：删除该策略保护；
  改为每个未进入矩阵的 task 都必须有物理、因果、mode 或显式冲突组原因；
- `高于 cutoff 的任务必须占用真人列`：删除无限 mandatory 语义；priority
  只能通过版本化的有限 defer/urgency 成本参与比较；
- `所有修补 SOC 不增`：改成 `(T,W)` 不增且 replay 合法；
- `高优先行永不推迟/机器人必须全部活动`：只保留因果服务与明确调度策略的
  必要合同，不当成 makespan 定理；
- `incremental 与 full 只需成本相同`：若生产声明行为等价，必须同时锁定
  canonical assignment；若只锁最优值，则必须明确允许搜索顺序变化；
- `无 storage map / singleton 计划哈希恒等`：在无语义改动且相同
  objective 下保留；objective 或 dispatch 已改变时改验合法性与质量，不
  虚报 bit parity。

搜索 key、目标集合、storage-only Drop、全 primitive successor、真实
custody anchor、trace rewire、deadline 和双侧 replay 等物理与交付契约
继续保护。

## 16. 最小测试矩阵

| 组 | 必须验证 |
|---|---|
| 目标 | 更小 T 可接受更大 W；同 T 取更小 W；权重不改变 unit timestep |
| 启发式 | 小图 oracle 对照 h_T/h_W；carried-at-goal 只剩 1 次 Drop；grounded-at-goal 为 0；多 goal injectivity |
| Macro/rewire | k 拍 macro 记 k；换 parent 同时替换 trace/g/f；repair 上界不伪装成节点 g |
| 清障 | 两层以上 blocker chain；只释放必要 cell 事件；重新占据使 release 失效；不同候选失败后完整 rollback |
| 任务身份 | 同第一腿不同 endpoint 不伪合并；同 endpoint 不同 route 保持 TransferId；anonymous 跨腿与重锚 |
| 并行 | 路径相交但时间错开可启动；相同 transit 第一腿不同时间可用；真实 vertex/edge/following 冲突仍拒绝 |
| 路由 | 首选路失败能找同长替代；无 route 的货架仍占格；endpoint 到达后持续占据；到点正确 Drop |
| 延迟 | free robot 晚到、loaded Wait、PIBT 偏离后时间表重建；没有 producer 时不能预测 vacate |
| 准备 | 足够 robots 时先清障同时 approach；只有一个 robot 时不被后继抢走；Lift 不等于 departure |
| 候选边界 | 远处高 priority 与附近低 priority 均进入 assignment 输入；state 724/1108 的近任务不被 top-\(F\) 删除；所有其他删除均有硬约束或 conflict-group 原因 |
| bottleneck 派工 | 取消 cutoff 后仍复现 bottleneck→secondary→canonical 目标；priority 有限延期不会退化成 mandatory；测试不强制选择最近任务 |
| additive 派工 | `S-D` 改变服务/延期相对代价；idle 可胜出；不使用 maximum-cardinality；critical tail 不以正即时 service 自动惩罚；PREPARE 只获部分收益 |
| 增量 matching | 0/1/k 行变化与 full objective/canonical assignment 一致；一行更新可经增广链重分配其他行；列 identity 或数值变化安全 full fallback |
| 数值与 tie | 矩形、负成本、64 位固定点、INF 不溢出；多最优解的 canonicalization 在 cold/warm 路径一致 |
| anchor/age | `mate` 与 `anchor_used` 分离；增广后下一代正确判 row 变化；age 只随真实 transition 增长，sibling/rewire/rebuild 不污染 |
| 缓存 | 同 U 的 PairCost/tau 不随 robots 改；D0 不被 execution overlay 写坏；时序信息不进 upper cache |
| matching 缓存 | sibling 不共享可变 matching；只有完整 `ColumnModelVersion` 相同才 row repair；Lift 使任务对其他机器人失效时不能只修 lifting row |
| 完备性 | no-route、endpoint 偏好、prep admission 失败时，fully constrained successor 与 oracle 相同 |
| 交付 | no corridor Drop；所有输出 replay；strict 10s；保留已验证 incumbent |
| Episode 连续 | route helper 零预算/暂时受阻 + 合法 Wait 后，TransferId/carrier/endpoint 保持；预算或通道恢复后继续同一 episode |
| 视图对齐 | 同一 U、不同 kappa：cached D0 逐位相同、ExecutionView 不同；不凭空产生第二台 carrier，不丢未满足清障条件 |
| Goal 前缀 | 初态即 goal 时 `T=0`；macro 中途达 goal 正确截断并注册前缀边；goal 之后动作不计入返回计划的 T/W |

禁止用“所有已选择局部任务都必须不可逆完成”作为新 physical invariant，
也不要用 C 的某个 robot 编号或指定通道替代通用测试。

## 17. Benchmark、Testcase C 与验收

### 17.1 固定协议

保留原 77 例 release 集（`benchmark/release_benchmark.json`）、固定
development 子集、实例字节、following 语义、seed、10s 和物理核并发
协议。C 及本轮微例新增为独立可审计组；扩展集合与算法改动分开记录，不
覆盖历史结果。

同机配对报告 success、真实 T、W、首次可交付解时间、deliverable runtime、
raw search/repair/第二遍各自贡献、timed-helper 时间/展开数、owner handoff、
causal waiting 和 traffic waiting。先检查 success 和合法性，再在
common-success 集比较质量。新 objective 下 SOC 是次级指标，SOC 增加要
披露但不能自动推翻更小 T。

\(\rho\) 专项报告还必须拆出：

- input、claim/conflict 后、same-key/shelf 后和最终矩阵的候选数；
- cost matrix 构造、bottleneck threshold、full Hungarian、incremental
  augmentation、canonicalization、state copy/cleanup 的独立耗时；
- full solve、exact reuse、row repair 和各类 fallback 的次数；
- changed rows 为 0/1/2/\(>2\) 的分布，以及列 identity/数值/mode/conflict
  版本变化率；
- `rho_task_id` changes、owner handoffs 和交付路径重建 guidance 中的
  free→free non-null retarget。三者不是同一指标，不能混称为“增量修复”。

### 17.2 C 的证据层次

| 计划 | T | W | 证据 |
|---|---:|---:|---|
| 旧 Planner | 54 | 90 | 用户提供的 production 记录 |
| 并行参考 | 31 | 93 | 用户报告通过仓库权威 validator；不是 Planner 输出 |
| 后续候选 | 31 | 90 | 按正文规则独立重放通过；仍待仓库权威 validator |
| v5 Planner | 31 | 93 | 当前 production 二进制输出；权威 validator 重放通过，plan SHA-256 为 `8a103b1a80ad24ab5889d1c158c5983009d7719663c28869414b5634e7e52c4d` |

C 的六个 goal 初始为空，提供的参考方案不搬匿名货架，因此 b4 与 b5 之间
没有“必须先搬走对方”这一清障依赖。它主要验证运输协调，不替代递归
blocker-chain 测试。

### 17.3 C 的 makespan 下界

这是根据 S2 参数的推导，不是原报告已有的最优性证明。

b4 从 `(2,17)` 到 `(10,3)`，最少 22 次四邻接 loaded Move。最近 robot R3
从 `(4,12)` 到 pickup 最少 7 拍。Lift/Drop 各 1 拍，所以：

```text
T* >= 7 + 1 + 22 + 1 = 31
```

其他 robots、handoff 或改变 route 不能减少 b4 在该模型下必须经历的这条
操作链。若 31-step 参考经过当前模型的权威 replay，则它达到 makespan 最优
值；不推出 W 最优。模型、goal set 或操作时长改变后，必须重新计算此证书。

### 17.4 验收不要绑定 robot 身份

接口回归：原始六任务不因 full-route intersection 被删；v5 历史
bottleneck probe 可以继续作为旧行为记录，但 F 阶段的候选边界测试不要求
保持被 top-\(F\) cutoff 造成的 assignment。

最终质量回归：计划合法、goal 正确、无 corridor Drop，并在固定预算内
达到 `T=31, W=93`。不要要求 `R5 必须搬 b5` 或 `active_robots=6`。
31/90 的五机器人候选说明这种身份/满载约束并非 makespan 所必需。

开发阶段可设置暂行质量线，但必须显式标为阶段验收；不能把较松门槛称为
达到最优，也不能事后降线。`W<=90` 只有在候选通过权威 validator 后才可
考虑成为更强次级质量目标，不声称它是已知最优 W。

### 17.5 \(\rho\) 诊断实例与验收

固定使用 S6 的
`brap_h10w10_a12_e8_R1_seed1` 作为候选边界回归。当前正式基线为：

```text
first_solution_ms       = 1188
first_solution (T,W)    = (2659,5691)
final (T,W)             = (1844,3927)
phase2 candidate        = 0
projection removed      = 815 steps
reconstructed retarget  = 310 free→free non-null changes
```

F 阶段必须证明 `state_t=724` 的距离 2 task、`state_t=1108` 的距离 1/2
tasks 不再因 priority top-\(F\) cutoff 消失，并输出最终选择每条边的
bottleneck/secondary/canonical 解释。测试不强制机器人选择最近 task；
如果远处 critical task 在同一矩阵中胜出，仍然是合法结果。

310 次 retarget 来自最终交付路径上重新构造的 guidance，不是原搜索树实际
错误改派数；其下降不是成功必要条件。真正验收同时比较 full 509 的 solved、
首解时间、最终 `(T,W)`、free moves、Lift/Drop、matching CPU 和内存。

## 18. 本轮不一起重写的部分

不嵌套完整 BR-LaCAM，不把整个 warehouse 的时空路径一次性冻结，不引入
robot-to-tau execution price，不强制所有任务提前派工，不以无限 lease
避免所有 handoff。

无界 dispatch lookahead、完整 multi-task scheduling、同步 rotation
bundle、混合任务最优性证明、统一表达任意 shelf/endpoint conflict 的
min-cost flow/ILP，以及取消所有既有 density-aware 图排序，都留作独立
变更。S6 只要求取消普通任务的 priority top-\(F\) matching 门槛，不等于
删除 Task-BR 的全部 priority/progress 排序。

当前 production 已有有界 bottleneck dispatch、critical-tail 估计和少量
timed-frame 评分；additive `S-D` 与增量 Hungarian 尚未由当前实现和 §23
验证。设计上的层次清楚不意味着已证明它们在全部 dense cases 更快；新增
行为和计算开销必须分开测量。

## 19. 总伪代码

```text
AttachCarrierGuidance(X, actual_transition):
    recover = RecoverPhysicalEpisodeFacts(X, actual_transition)

    U = UpperProjection(X)
    pair, tau = GetPurePairAndTau(U)
    upper_priority = GetUpperPriorityWithExistingCommitment(U, actual_transition)
    D0 = GetOrCompilePureCausalGraph(U, tau, upper_priority)

    view = ReconcileCausalGraphWithActualEpisodes(D0, X, recover)
    # 不修改 D0 的缓存值；不让新 graph 偷换 active endpoint

    candidates = FindAssignableTransfers(view, X)
    preparations = FindBoundedSafePreparationCandidates(view, X)
    # 这里只执行物理/因果/mode/conflict 检查；
    # 不按 priority top-F 删除普通候选。

    rho_problem = BuildVersionedRhoProblem(
                      candidates, preparations, X,
                      previous_transfer_bindings,
                      frozen_dispatch_objective)

    dispatch, rho_state = SolveRho(
                              rho_problem,
                              parent_node_local_rho_state)
    # F 对照：bottleneck -> secondary -> canonical。
    # G 实验：robot-row additive S-D full solve，idle 合法。
    # H 只在完整列模型相同时 row repair，否则 full solve；
    # incremental 必须复现对应 full solver 的 canonical assignment。

    jobs = ActiveEpisodes(recover) + AssignedTransferHints(dispatch)
    traffic = BuildBoundedJointTransportGuidance(
                  X, view, jobs, previous_route_hints)
    # 少量完整 frame 以 (all-target T estimate, work, stability) 比较；
    # 未计划/暂停 roots 进入剩余尾长。失败不删除物理动作、不修改 tau。

    return Guidance(pair, tau, D0, view, dispatch, recover, traffic)
```

```text
GenerateSuccessor(N, C):
    G = EnsureGuidanceFresh(N)
    if C fixes all robot primitives:
        return apply_ops(N.X, C.joint_ops)

    scratch = SeedFromActualStateAndForcedOps(N.X, C)
    use G to order approach/Lift/Move/Drop/Wait candidates
    Carrier-PIBT completes a preferred joint action in shared scratch
    return apply_ops(N.X, joint_ops)
```

```text
AcceptCandidatePlan(candidate):
    replay candidate with authoritative transition and goal checks
    compute (first_goal_tick, weighted_work)
    replace deliverable incumbent only if lexicographically better
```

下文 §20、§22 为历史基线原文，§23 为 2026-09-05 v5 实现证据；它们都
不能证明 S6 的候选边界、additive 或增量 matching 修订。§21 是 storage
部分的统一摘要；§24 记录 2026-09-06 \(\rho\) 专项审计证据。

---

> **以下 §20 是历史基线原文，原样保留。它不是 v5 的实现与验证声明。**

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

## 21. Storage-map 修订后的统一合同

本节汇总 §3、§6–§10 的同一语义，不设另一套覆盖正文的优先规则。

| 对象 | 保留什么 | 允许改变什么 |
|---|---|---|
| 物理 shelf/carrier | kappa 与实际坐标 | 只能由真实 primitive transition 改变 |
| 当前 transfer | 选定 shelf、source、合法 endpoint 与需求 | 显式完成/失效/重绑定时改变，不因普通 reroute 丢身份 |
| route | 必须解释为到 endpoint 的合法几何/时序候选 | 中间方向、等待和时间表可重算 |
| LegId | 当前一次精确相邻 effect | 路由改第一腿时更新；不代表整个工作被换掉 |
| endpoint 占据 | 真实 occupancy 和选定放置关系 | producer vacate 后可供 consumer 使用；到达/Drop 后仍占据 |
| transit 使用 | 当前真实冲突规则 | 允许不同时间复用，不按 route 集合交集永久互斥 |
| PairCost | shelf-only、确定性有界估价 | 仅随 U/version 改变；不消费 timed routes |
| 因果图 | 递归清障及 selected placement 条件 | 上层变动重编，执行 overlay 按实际事件更新 |
| rho 候选 | 当前 mode 下通过物理、因果、custody 与显式冲突检查的普通任务 | priority 只作有限成本/排序，不能 top-\(F\) 硬删除 |
| rho 目标 | F 阶段保留 bottleneck→secondary→canonical；G 单独测试 additive `S-D` | 只能通过版本化 objective 切换，不能因实现方便隐式换目标 |
| rho matching state | 具体 LaCAM 节点的 matching/duals、列模型和 anchors | 列模型相同可 row repair；列结构/数值变化 full solve |

尤其禁止两种实现捷径：

```text
if first_preferred_route_conflicts:
    erase_entire_task_before_rho

if two_jobs_share_first_leg:
    merge_their_entire_endpoint_demands_without_checking

if task.priority < top_F_cutoff:
    erase_ordinary_feasible_task_before_matching

if same_upper_signature:
    reuse_mutable_matching_state_across_search_branches
```

同时保留原 storage 修复的必要部分：不能在 transit Drop；不能在每次
loaded Move 后丢失整个 transfer，只因新 D0 暂时看不到该 root 就立刻反向
返回 source；不把 route helper 失败当成物理无解。必要的有计划回退仍可
作为合法候选，不能将“没有任何 reverse”升级成全局正确性定理。

---

> **以下 §22 是 2026-09-04 基线原文。其 APPROVE、测试数量、时间和 plan
> hash 都属于旧实现，不用于证明本稿的新目标、动态绕路或事件准备已验证。**

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

### 22.3 正式 release benchmark 纳入 warehouse-block 实例

正式 benchmark 的 testcase 集合由
`benchmark/release_benchmark.json` 固定，不再只隐式指向
`instances_brap_pool`。该配置包含：

* 原 BRaP pool 68 例；
* `benchmark/viz_web/warehouse_block_suite/manifest.json` 中全部 9 个真实
  warehouse-block YAML，包括用户检查的
  `warehouse_blocks_h20w20_b3_a1_d75_r8_t12_seed0`；
* 固定 `carrier`、seed 0、单位权重、following allowed、每例 10 秒和
  14 jobs。

runner 在启动前校验每组 testcase 数量、路径与实例名唯一性，并拒绝通过命令行
改变上述协议。2026-09-04 的实际 77 例运行保存在
`benchmark/results_release_77_20260904`：总成功 `47/77`，其中原 68 例仍为
`38/68`，新增 warehouse-block 为 `9/9`。原 68 例的 success、status、
makespan、SOC、动作计数、诊断计数和 plan hash 与上一正式结果逐项一致；
只有机器时间字段正常波动。9 个新增行的 plan hash 与已有动画使用的 plan
完全相同，因此已有逐帧审计仍为 `corridor_drops=0`、
`same_origin_returns=0`、`loaded_reversals=0`。本节只扩展 benchmark
成员和 runner 输入编排，不修改 planner、搜索流程或动作语义。

---

## 23. v5 实现、正式 full 与最终验证证据（2026-09-05）

本节是 v5 的新证据，不修改也不借用 §20/§22 的历史结论。实现仍只有一个
`TAPFPlanner::solve()`，并在原节点、constraint tree、`funcPIBT()`、
`apply_ops()`、两遍候选、repair 和 final replay 路径上增量接入：

> 本节早于 S6 专项审计。它证明 2026-09-05 v5 production 的合法性与当时
> 的 benchmark 结果，不证明普通 priority cutoff 已取消，也不证明
> additive `S-D` 或增量 Hungarian 已实现。

```text
PlanCost(T,W) + admissible h_T
  -> endpoint-stable TransferId / ExecutionView
  -> causal preparation + bottleneck dispatch
  -> bounded timed transport frame
  -> unchanged physical successor oracle
  -> same-controller bounded improvement attempt
```

### 23.1 代码与测试

当前 `build/dd_benchmark` SHA-256 为
`854df1e692316017cbc26ab462ab3b22735d61caa08ff0645e28ad0cab4a574c`。
受保护测试的 objective、route-claims、dispatch 与 proposal golden 迁移均
在修改前取得独立 GPT-5.6 Sol/high 或 xhigh `APPROVE`。最终代码测试：

```text
./build/test_all --gtest_color=no
  285 / 285 PASS, 224.928s

cd benchmark && python3 -m unittest discover -s tests
  155 / 155 PASS, 301.829s
```

新增回归覆盖严格 `(T,W)` 比较、首次 goal 前缀、mixed makespan 下界、
TransferId/route-status 连续、同 transit 第一腿错时复用、对向走廊 passing
bay、paused-root frame score、因果条件重验证、受控 preparation、bottleneck
dispatch、同控制器 incumbent 改进，以及 lazy PairCost 8-step prefix 的
lower-bound/assignment certificate。另有两项回归锁定“canonical transit
当前被占用”以及“占用者正在搬货”时 transfer 仍保留并可被派工；占用的
storage placement endpoint 仍会阻止接纳。zero-shelf compatibility 与 fully
constrained successor completeness 同在 285 项中通过。新增的第 285 项
`recovered_transit_episode_allows_dependent_preparation` 复现一个真实
execution-path 缺陷：阻塞货架已由旧 episode 搬入 transit 后，重锚定的
当前 task 会成为 `SHADOWED`，但下游 target 仍应允许另一个机器人
`PREPARE`。修复只在对应因果边上，把物理有效、TransferId/carrier/endpoint
一致且首腿正离开阻塞格的 custody 视为 executor；它不把 `SHADOWED`
全局改成 `ACTIVE`，也不提前把因果条件标为 fulfilled。

`W` 的实现不是浮点近似：四个权重在唯一入口上被校验为有限、非负且恰好
落在 `10^-6` 网格，然后规范化为 `int64_t` 微单位；search、rewrite、
两遍、repair、replay、stats 与 runner cross-check 全程使用同一整数表示。
每个成功 carrier 方案都由权威 Python validator 重放，runner 要求最终
`weighted_work_scaled` 精确相等；LaCAM 还要求 `best_work_scaled` 与最终
交付方案精确相等。

首次获批 full 执行完成了 509 个 solver task，并在汇总阶段报告
`479/509`，但发布前最后一次 sealed-binary 重哈希因
`Path.resolve()` 把 `/proc/<pid>/fd/<fd>` 跟随成不可重新打开的
`/memfd:... (deleted)` 描述名而被 gate 拒绝；该目录没有 `rows.csv`，
因此不是 benchmark 结果。新增
`test_sealed_binary_can_be_rehashed_before_publication` 先复现同一 RED，
随后只把 hash helper 改为保留 procfs fd symlink 的绝对路径，继续读取并
比较同一封存字节。完整 Python 套件因此从 150 增至 151 项；正式 full
在该 gate delta 再次独立审查通过后，使用新目录完成重跑，见 §23.5。
最终汇报生成器又增加 3 项 data-driven、production sample schema 与链接
回归；最终独立终审再要求显式测试计数与 README 历史语义隔离，新增 1 项
历史隔离回归。因此最终 Python 总数为 155，终审 delta 子集为 5/5 GREEN。

静态审计只找到一处 `struct PlanCost`、一处严格 ticks-then-work
`operator<` 和一处 `TAPFPlanner::solve()`。production 没有 instance/seed
特判、运行时 feature flag、legacy planner fallback 或第三套 deadline。

### 23.2 Testcase C

当前 production 对
`warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0` 在 10s 配置下：

```text
first_solution_ms=38
first_solution_makespan=31
first_solution_soc=93
first_solution_work_scaled=93000000
best_makespan=31
best_soc=93
best_work_scaled=93000000
deliverable_ms=7710.78
```

权威 validator 重放得到 `T=31, W=93`、49 loaded moves、32 free moves、
12 Lift/Drop、0 anonymous move、0 reversal。plan SHA-256 为
`8a103b1a80ad24ab5889d1c158c5983009d7719663c28869414b5634e7e52c4d`。
它达到 §17.3 的 makespan 下界；不声称 W 最优。

### 23.3 固定 quick 77 配对

v5 结果位于 `benchmark/results_quick_v5_control_854df1_20260905`。suite
SHA-256 仍为
`a881292163ff2fcd2797cc83fddd8f9ebd12def79619586b940976bce07111c6`；
77 cases、10s/case、14 jobs、seed 0、unit weights、following allowed
均未改变。与 `benchmark/results_release_77_20260904` 配对：

| 指标 | 历史 baseline | v5 |
|---|---:|---:|
| solved | 47/77 | 47/77 |
| BRaP solved | 38/68 | 38/68 |
| warehouse-block solved | 9/9 | 9/9 |
| wall time | 29.4s | 52.0s |
| solver-time sum | 334.5s | 651.7s |
| common makespan sum | 19229 | 19749 |
| common weighted-work sum | 49014 | 54627 |

成功集合完全相同，无 lost/gained case。共同 47 例的 makespan
better/equal/worse 为 `29/3/15`，几何比 `0.940512`；weighted work 为
`14/4/29`，几何比 `1.064569`。因此 v5 在多数共同实例和 makespan 几何均值
上改善，但总 makespan、次级 work 与运行时间均有回退，不能表述为逐例或
所有聚合指标都改善。额外时间主要由首解后的有界改进尝试使用；成功例最大
`deliverable_ms=9277.23`，没有突破 10s。

### 23.4 E5 独立 feature-removal 消融

三份独立 Release build 分别只移除 E1 bottleneck dispatch、把 E2 timed
frame 候选预算从 8 降为 1，或跳过 E4 第二次同控制器改进。它们串行运行
同一固定 quick 77，不向 production 添加 feature flag。四组均为 47/77，
成功集合完全相同：

| 变体 | T better/equal/worse | T sum | W better/equal/worse | W sum | wall / solver sum |
|---|---:|---:|---:|---:|---:|
| E1 off | 14 / 22 / 11 | 17998 | 18 / 17 / 12 | 48932 | 52.1s / 653.7s |
| E2 off | 0 / 47 / 0 | 19749 | 0 / 47 / 0 | 54627 | 52.1s / 653.9s |
| E4 off | 0 / 33 / 14 | 20679 | 0 / 30 / 17 | 56324 | 28.9s / 324.4s |
| control | — | 19749 | — | 54627 | 52.0s / 651.7s |

结果不支持在该 quick corpus 上宣称 E1/E2 有聚合质量收益：E1 off 的总 T/W
更小，E2 off 仅改变一个 plan hash 而 T/W 完全相同。E4 在 17 行产生严格
改善、没有丢失成功例，但
solver-time sum 约翻倍。完整 binary hash、patch 定义、几何比与 helper
诊断见 `benchmark/ablation_v5_20260905.md`；这些负结果不隐藏，也不用于
事后调参。

### 23.5 正式 full 509

冻结 full manifest 为 quick 77 加 432 个已生成随机配对 warehouse cases，
共 509 例；definition SHA-256 为
`fae83e9ba41dc8b933c79f7769992b29006bb1fc67004e770e621b0830c890ed`，
语义 corpus SHA-256 为
`7840959653b2056c6441ede0cbcd93031f9ec3c4796b8270af2c7d1447a72bae`。
第一次 full 发布失败后，一名新的独立 GPT-5.6 Sol/xhigh reviewer 审查
procfs memfd 路径修复、失败目录、gate tests 与未变化的 production binary，
明确 `APPROVE`。schema-v2 approval 位于
`benchmark/full_review_approval_v5.json`，其 SHA-256 为
`fdfbdd23e6dcdb3ec1a2ff2edb524f469a466c12a1c9ff87a8e60d44705b6d60`，
同时绑定上述 suite/corpus 和 binary
`854df1e692316017cbc26ab462ab3b22735d61caa08ff0645e28ad0cab4a574c`。

正式重跑使用新目录
`benchmark/results_full_v5_854df1_20260905_r2`，协议为 14 jobs、
10s/case、seed 0、unit weights、following allowed；`timing.json` 明确记录
`execution_snapshot=sealed_linux_memfd`。结果为：

| 范围 | solved | 说明 |
|---|---:|---|
| 全部 full | 479/509 | 94.1% |
| 固定 quick 子集 | 47/77 | 与单独 quick 的 T/W 逐例相同 |
| 新增 factorial | 432/432 | 全部成功 |
| 失败 | 30 | `g20x20` 6、`g40x40` 8、`g80x80` 16，均为原 BRaP timeout |

full wall time 为 `295.6s`，solver-time sum 为 `4037.5s`。479 个成功方案
均有 plan 文件并通过权威 validator 重放；不存在缺失、额外或 hash 不符。
正式 `rows.csv` SHA-256 为
`743a0b3bc2d635420216148ad52238db38d02b0bbb6288586cbcfc705c7e6e83`，
`timing.json` SHA-256 为
`dab81f04fd1cefc3e1a9398d8ff4cbb0c2c96c3694ef5356822a3f94eda98143`。
正式 full 中的 quick 子集与冻结 control 的 47 个成功例 T/W 完全相同；
仅有 3 个等质量 plan hash 不同。

factorial 432 例的平均/中位 makespan 为 `31.70/26`，平均 work 为
`209.20`。它们是构造可解且受保护的随机配对集合，不代表真实仓库流量分布，
因此这些分组统计只作描述，不解释为因果定律。

与隔离保存的 pre-v5 历史 full 做同 corpus 的回顾性配对时，成功集合同为
479 例。v5 makespan better/equal/worse 为 `348/85/46`，总和
`38716 -> 33442`，几何比 `0.801040`；work 为 `136/88/255`，总和
`135913 -> 145000`，几何比 `1.028228`。历史 wall time 为 `49.2s`，
当前为 `295.6s`，约 6.0 倍。这个历史目录只用于比较，不是 v5 完成证据；
旧 binary 结果和 schema-v1 approval 继续隔离在
`benchmark/historical/pre_v5/`。

### 23.6 可视化与最终汇报

正式 dashboard 位于
`benchmark/viz_web/full_benchmark_v5_854df1_20260905/index.html`，由明确的
rows/timing/manifest 输入生成，含全部 509 行和 479 个成功方案动画。面向
初学者的中文最终汇报位于
`benchmark/viz_web/carrier_lacam_v5_final_report_20260905/index.html`；
它从正式 full、固定 quick、历史隔离结果、E5 消融和 Testcase C 样例文件
生成核心统计，并链接原始 rows/timing、完整 dashboard 和逐帧动画。静态
HTML 可直接从本地文件打开，不需要 HTTP server。

---

## 24. 2026-09-06 \(\rho\) 专项审计与后续修订证据

本节记录 S6 审计时的基线事实。完整推导、代码锚点和状态表位于
`carrier_lacam_rho_global_matching_report_20260906.md`。正式输入为
`benchmark/results_full_two_pass_reference_20260906/rows.csv`，诊断实例为
`brap_h10w10_a12_e8_R1_seed1`。审计后的 production 结果另见 §25，不能把
本节的“当前机制”误读为 commit `64a3941` 的新行为。

### 24.1 当前机制的确认

当前 \(\rho\) 的真实顺序是：

```text
ready_tasks_with_custody
  → priority-ordered claims / compatibility filtering
  → same TransferKey / shelf 去重
  → priority top-F cutoff + mandatory
  → bottleneck-then-sum assignment
  → canonical refinement
  → EXECUTE；剩余 robots 再做 PREPARE
```

因此“全局匹配”只对 cutoff 后的幸存任务成立。诊断路径中没有出现同一货架
同时对应多个临时 endpoint task；`state_t=724` 和 `state_t=1108` 直接展示
了附近 task 在矩阵前消失。该证据解释候选没有参加比较，不能单独证明选择
附近 task 会改善最终 makespan。

### 24.2 目标实例的准确口径

| 指标 | 数值 |
|---|---:|
| 首解时间 | 1188 ms |
| 首解 `(T,W)` | `(2659,5691)` |
| 最终 `(T,W)` | `(1844,3927)` |
| loaded / free moves | `556 / 1871` |
| Lift/Drop | `1090` |
| 第二遍候选 / 严格改进 | `0 / 0` |
| projection repair 删除 | `815` 拍 |
| deliverable runtime | `9030.13` ms |
| guidance time | `3976.05` ms |

最终 1844 拍恰好是首解 2659 拍删除 815 拍后的交付前缀，不是第二遍找到的
严格改进。`robot_utilization=0.1508` 的 validator 定义是 loaded moves 除以
全部 robot-time slots；把 free moves 与 Lift/Drop 也计入后，非 Wait 动作
占比为 95.36%。所以该实例不是“机器人普遍闲置”，而是大量时间可能消耗在
空驶、举放和重新派工。

最终交付路径上重新构造 guidance 得到 310 次 free→free non-null retarget。
它不能还原原搜索树当时的 parent guidance 和 forced operators，因此只能
作为同一交付状态集上的稳定性指标，不能命名为“实际错误改派 310 次”。

### 24.3 full benchmark 的规模信号

正式 full 共 509 例，479 solved。479 个成功实例累计：

| 指标 | 数值 |
|---|---:|
| ready task count | `13,625,227` |
| `rho_task_id` changes（CSV 字段 `rho_repairs`） | `3,917,608` |
| owner handoffs | `961,068` |
| upper epoch builds | `387,854` |
| guidance time | `1697.780` s |

`rho_repairs` 只是相邻 guidance 中 assignment id 的变化计数，当前代码没有
执行增量 Hungarian repair。guidance time 还包含候选、PairCost、task graph
和 transport 等构造，不能把全部时间归因于 Hungarian。专项 telemetry 必须
先分离 matrix 构造、full solve、canonicalization、状态复制和 cleanup。

第二遍退出分布为：

```text
SEARCH_CUTOFF             367
STRICT_IMPROVEMENT         79
REFERENCE_SUFFIX_ACCEPTED  32
SEARCH_EXHAUSTED            1
```

这组数据与 \(\rho\) 修订的主要关系是：dispatch 计算处于严格总预算内，
任何增量优化都必须同时报告搜索节点、首解时间和最终质量，不能只报告
matching microbenchmark。

### 24.4 由审计冻结的后续合同

1. 普通可行任务不再因 priority top-\(F\) 被删除；每个过滤都必须有物理、
   因果、mode 或显式 conflict-group 原因。
2. 先保留当前 bottleneck-then-sum 做候选边界对照；additive `S-D` 是独立
   full-solve 调度实验，不宣称与 makespan 等价。
3. idle 合法，PREPARE 只有部分收益，不把 maximum-cardinality 设为主目标。
4. matching/duals 跟随具体 LaCAM 节点；完整列模型不变才 repair rows，
   列结构或数值变化 full solve。
5. full 与 incremental 若声明行为等价，必须同时复现 objective 和
   canonical assignment；增量 solver 的收益与调度模型变化分开衡量。

§24 只构成审计基线和实现依据；F/G/H 的实际决策及 production 证据以
§25 为准。

---

## 25. 2026-09-06 \(\rho\) 修订实现与验证

### 25.1 production 选择

实现严格按候选边界、调度目标和求解方法分开推进：

| 阶段 | 结果 | production 决策 |
|---|---|---|
| F0 telemetry | quick 47/77 | 保留诊断 |
| F1 no-cutoff | quick 45/77，两个大型 case 超时 | 保留“无 cutoff”合同，不直接发布该成本语义 |
| F2S 低阶有限 priority tie | quick 47/77，但样例 A 为 `(28,45)` | 作为 V2 的混合阶段基线 |
| additive G1 | quick 47/77，Testcase C 为 `(231,268)` | 回滚 |
| H1 node-local shadow | 建立在已回归的 G1 上 | 随 G1 回滚，不启用增量 production |
| 收紧 F2 V2 | quick 47/77，保护项全部满足 | commit `64a3941` |

最终 objective 名为
`BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2`。它不恢复 priority
cutoff：所有普通候选仍可连接真实 robot 或 dummy。只有 EXECUTE 且所有
候选都满足 §8.2.1 的“自己目标货架到自己合法终点”合同时，frontier 和
parent continuity 才以最大 3 倍的有限 service 延期修改 dummy
completion；任何 blocker、非终端 task 或 PREPARE 都使整次 matching
保持 F2S/V1 数值语义。

### 25.2 测试与保护结果

最终代码通过：

```text
C++ test_all       312 / 312
Python unittest    162 / 162
fixed quick         47 / 77
```

受保护结果为：

| 实例 | 最终 `(T,W)` |
|---|---:|
| 发布样例 A | `(17,32)` |
| 发布样例 B | `(12,36)` |
| Testcase C | `(31,93)` |
| 发布样例 D | `(8,23)` |
| `h20w20 a40 e100 R1 seed0` | `(1273,5796)` |
| `h20w20 a40 e100 R1 seed1` | `(1243,6384)` |

样例 A 的计划 SHA-256 为
`882701b39b2bdbf6c0cab46d8164e94171bd87c5ae9e358a36000c176bcb1023`；
其余 B/C/D 的既有计划哈希保持不变。

### 25.3 quick 77 的正负证据

最终 quick 位于
`benchmark/results_quick_rho_v2_20260906/rows.csv`。runner 报告
47/77、总 case runtime 624.3 秒、14 并发 wall time 47.9 秒。相对 F2S，
47 个共同成功实例中：

```text
V2 更好   2
完全相同 42
V2 更差   3
solved-set 差异 0
```

改善包括诊断实例
`brap_h10w10_a12_e8_R1_seed1` 的
`(1361,2964)→(1359,2958)`，以及
`brap_h6w10_a6_e15_B_seed1_pool` 的 `(53,106)→(45,89)`。
负面结果包括一个同 makespan 下 work `817→818`，以及两个 warehouse
case 的 makespan `24→26`、`21→23`。因此 V2 的结论是“以严格边界修复
受保护直接交付回归，同时大部分保持 F2S”，不是全 corpus 单调改善。

### 25.4 尚未完成

正式 full 509 只能在最终 diff、全部测试和 quick 通过后，由独立
GPT-5.6 Sol/high reviewer 明确批准再运行。full 结果、最终数据网页和网页
独立审查将在完成后补入本节；在此之前不得把 quick 47/77 外推成 full
结论，也不得宣称增量 Hungarian 已进入 production。
