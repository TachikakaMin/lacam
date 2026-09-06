# Carrier-LaCAM v5 修订草案：清障因果依赖、动态运输与 Makespan 优先

**状态：设计修订，尚未实现、尚未通过仓库 release 验证。**
**日期：2026-09-05。**

本稿根据本轮讨论修改 `Pasted markdown(6).md`，沿用原稿的 §0–§22 编号。§0–§19、§21 是建议替换的规范；§20、§22 原文保留为历史验证记录，不代表本稿已实现。本文没有审查仓库当前 commit 的全部源码，不把设计中的代码映射表当成独立代码审计结果。

基线文件 SHA-256：`2010f90cd6d6aa8b9ea68393df23a58432c006e584b4422cbf3b050517cdf524`。

依据与证据等级：

- **S0：当前设计稿。** 本稿中的“当前规定”指该附件规定，不额外声称代码一定完全符合文档。
- **S1：2026-09-05 动态绕路提案。** 其中新机制仍是提案，不能作为已实现行为。
- **S2：用户提供的 Testcase C 地图和 31 行参考计划。** 用户报告权威 validator 验证通过；54/90 与 31/93 是该报告中的指标。
- **S3：此前构造的 31 步、SOC 90 候选。** 本轮按 S2 的地图和动作规则独立重放得到 31 步、49 loaded moves、29 free moves、12 Lift/Drop，并检查通过；未运行仓库的权威 validator，不作为 release 成果或 SOC 最优性证据。

## 0. 最终决策

保留两层动态 assignment 和一个真实物理搜索。不要把 Testcase C 的运输冲突误当成清障 dependency，也不要因为路线预测相交就取消整个搬运任务。

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
rho：机器人 → 可执行或可安全准备的 transfer
    ↓
联合运输 guidance：保持 endpoint，选择路线、等待和通过顺序
    ↓
Carrier-PIBT + operator constraints
    ↓
apply_ops → 一个真实 joint transition
```

三个决定分别回答：

1. **tau：**目标货架最终去哪个合法 goal？只读取货架布局。
2. **Task-BR-PIBT：**为了推进这些目标，哪个 blocker 必须先搬到哪里？共享上下文，递归、回退、合并兼容需求。
3. **执行协调：**谁搬、何时可开始哪一个阶段、从哪里走、谁先经过交点？读取机器人和实际执行状态，但不写回 `PairCost/tau`。

最终优化目标为词典序 `(executed_makespan, weighted_SOC)`。首解质量、搜索改进、两遍候选比较、rewrite 和 repair 必须使用相同目标。

## 1. 当前稿的逐章修改表

当前稿 §6.3/§6.4 已经规定 shared context、递归清障和 root-level backtracking；不能把它整体描述成“只有独立路径加依赖”。本轮主要纠正 storage transfer 引入后的粒度混用和过度互斥。

| 原位置 | 当前规定或遗漏 | 本稿修改 |
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
| §8.2 | priority-first、min-sum approach，并倾向填满所有行 | 先保留为迁移基线；逐步引入完成时间派工，不能把满载率当目标 |
| §10、§19 | custody 和缓存不足以区分纯上层计划与执行时间表 | `D0` 可缓存；ExecutionView、lease、时序预约不进入 upper cache |
| §12、SearchEdge、两遍/repair | weighted cost、加法式 h、SOC 优先 | `Cost{ticks,work}`；makespan 下界；词典序比较和相应剪枝 |
| §15–§17 | 一步身份、ready-only、SOC 不增、指定 robot 活动等保护测试 | 保留物理正确性；审查后迁移已经改变的策略契约 |
| §20、§22 | 已通过验证的历史实现 | 原文归档；不得改写成 v5 的验证结论 |

## 2. 物理状态、终点、计时与 upper projection

### 2.1 保留物理模型

```text
X = (Q_robot, Q_target, Q_anon_grounded, kappa)
Op = Wait | Move(neighbor) | Lift | Drop
```

robot 是唯一 actuator。匿名货架不加入永久身份。`SearchKey` 仍只表示真实物理状态。`apply_ops()` 独立决定 vertex、swap、following、upper occupancy、Lift/Drop 和 storage legality；本轮不修改其允许的转移集合。

```text
is_goal(X) = 所有 target grounded 且 position(b) ∈ G_b
```

终止不依赖 tau、task graph、route 或 custody。当前终点定义没有要求所有机器人归位，也没有要求所有匿名货架的 guidance episode 都结束；不能通过“等待全部 transfer 清空”暗中加一个终点条件。若以后要求所有匿名货架也落地，必须另行修改问题定义和双侧 validator。

### 2.2 目标函数

设 `X_0 ... X_T` 是首次满足 `is_goal` 的物理前缀：

```text
T = 真实 joint transition 数
W = alpha*loaded_moves + beta*free_moves
    + gamma*lift_drop + delta*anonymous_loaded_moves
J = (T, W)，按词典序比较
```

一个普通 joint transition 的时间成本是 1，即使所有机器人都 Wait；其中多少机器人同时行动不改变这 1 拍。macro 的时间成本是实际 trace 长度，不是 1。

alpha/beta/gamma/delta 是 work 权重，不是动作时长。本文仍使用单位时长 Move/Lift/Drop；非单位操作时长需要新增物理 mode/剩余时长，不在本次修改中。

### 2.3 三种“空位”

令 `S` 为合法 storage cells，`O(U)` 为全部货架当前坐标，`V` 为 traversable cells：

```text
storage_slack        = |S| - number_of_shelves
empty_storage(U)     = S \ O(U)
empty_transit(U)     = (V \ S) \ O(U)
```

`storage_slack` 是静态净存储余量，不一定等于当前空 storage 数。货架离开 storage 进入通道时，其源 storage 已经空出，即使它还没在另一个 endpoint Drop。

空 transit 不是合法 Drop 位置，但可以供 loaded Move 使用。不得把“不能 Drop”写成“不能作为移动空格”，也不得由 `storage_slack==0` 推断全物理问题无解。

### 2.4 纯上层与执行层

```text
U = labeled target coordinates + sorted all anonymous coordinates
```

`PairCost/tau_guide` 只依赖 `U` 和不可变实例参数。Lift/Drop、free Move 不改变它们。`D0` 依赖 `U/tau/upper priority commitment`，继承原稿已存在的 commitment cache key；不能再笼统声称 D0 在任何 ancestry 下都是 U 的纯函数。

ExecutionView、rho、prep admission、route、预计 release time 和 timed reservations 可以读取完整 X 及紧邻真实 transition，不写入 PairCost 或 D0 的缓存值。

## 3. Guidance 数据结构：把任务与动作分开

### 3.1 Root、transfer、leg

```text
RootDemand(b, g)             目标货架 b 当前朝 g 完成
Transfer(s, source, endpoint) 一次明确的 storage 搬运意图
Leg(s, current, next)         这一拍建议实现的相邻 shelf effect
```

保留原 exact `TaskId` 的一步含义，可兼容命名为 `LegId`。增加：

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

这里是语义接口，不要求按这些名字新建平行框架。可在现有 `ShelfTask/StorageTransfer/Custody` 内逐步实现。`TransferId` 不使用全局递增计数影响排序；anonymous 的稳定 episode 由实际 carrier 与 Lift/Move anchor 延续，不进入 physical key。

### 3.2 Identity 契约

route、下一格、局部时间表改变时，transfer 不一定改变。只有 source/所搬货架/endpoint 的语义改变，或真实 Drop 终止了该 episode，才失效或建立新 transfer。

以下两种方案不能合并为同一个完成任务：

```text
s: u -> v -> ... -> e1
s: u -> v -> ... -> e2
```

它们共享第一腿 `u->v`，但完成条件不同。物理 successor 可以按同一个 LegId 去重；task graph 不能仅因第一腿相同就把两个 root 的要求都标成已满足。

只有相同 source、相同 shelf、相同 endpoint，且因果要求可协调的 transfer，才共享一个 task。不同路径是其候选实现；不同 endpoint 是不同方案。若另一个 root 只要求 `Vacate(source)`，并不要求特定 endpoint，它可以在重新检查自身条件后接受已有 transfer；不能只因第一腿相同就省掉这个检查。合并 roots 后仍向全部 predecessor closure 传播需求与优先级。

### 3.3 Transfer 不强制每段 Drop

事件至少包括 `PickupReady`、`Lifted`、`Vacated(source)`、`Arrived(endpoint)`、`Dropped`。到达 endpoint 完成该 transfer 的搬运 effect；释放机器人需要 Drop；target 完成则必须在合法 goal Drop。

同一个 carrier 可以在合法 storage endpoint 接续下一 transfer，省掉中间 Drop/Lift。不能为了清晰的 task 身份而强制每个相邻 shift 举放一次。无 storage map 时各 transfer 退化为相邻 effect，物理语义不变。

## 4. 第一层：single-root PairCost

保留 S0 的 shelf-only、bounded rollout、rollout-local anonymous token、有限 stall/truncation、cache version 与 lazy-exact certificate。

第一批修改不重写 PairCost，不把 execution price 加回来。canonical route 仍可由确定性 BFS 产生，作为估价样本；“用于估价只存一条 route”不等于“实际执行只能走这一条”。

PairCost 精确值的含义是：精确计算这个确定性有界启发式定义的数值，不是得到真实最优搬运成本。单 root rollout 的顺序执行估计也不是多 root makespan 下界。

禁止混入机器人位置、free/loaded availability、rho、timed reservation 和等待历史。其读取集合不因下面新增的 execution view 而扩大。

若之后修改 PairCost 输出的时间/工作量统计，必须更新 cost/compiler version 并重新验证 lazy certificate 中 `L_e <= C_e` 的义务；不能更换代价语义却沿用旧证明和 cache。

## 5. Shelf-goal matching 与 admissible bound

### 5.1 tau_guide：本轮先保留 min-sum

保留当前基于 PairCost 的 injective min-sum Hungarian，以及不读取 parent/robots 的稳定 tie。它仍是 goal guidance，不声称精确最小化 makespan。

Testcase C 的目标均固定；它的主要问题不能通过修改 tau 解决。本轮不同时把第一层改成 bottleneck matching，以免混淆 route 修复、派工和 goal allocation 的贡献。

后续可独立比较 min-sum work 与 min-max isolated completion estimate。多目标共享 blocker、vacancy、robot capacity 时，两种可分估计都可能失真，不能宣称简单换成 min-max 就得到全局 makespan matching。

### 5.2 Makespan lower bound

下面是本稿的推导，要求当前模型为单位时长、四邻接、robot 是唯一 actuator，且每个机器人一拍最多做一个 primitive action。

`d_U` 和 `d_L` 是只考虑不可变墙的距离，忽略其他 robots/shelves。对 grounded 且需要移动的 target：

```text
a_b(X) = min_r d_L(Q_robot[r], position(b))
```

这里可以乐观地把 loaded robot 也当成可以直接接货；这只会降低估计，不会高估。

定义每对 eligible `(b,g)` 的时间下界：

```text
ell_T(b,g) = 0                         grounded 且 position(b)=g
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

证明要点：任何实际解选择某个 injective 最终 assignment；每个 target 到对应 goal 的完成时刻至少是 ell_T，故最晚完成时刻至少是其最大值。每个 robot 一拍最多执行一个动作，target 必要工作量也不能超过 `|R|*T`。对全部合法 assignment 分别取最小值仍是下界。机器人相互竞争、清障与拥堵被忽略，估计可以弱，但不因此高估。

`h_W` 保留原来不加 locks 的 weighted target-work LB matching。`h_T` 可以读取完整 X 中的机器人位置；这不是对 tau 的反馈。原稿“任何 robot approach 都不准进入 h”应改成“未经证明的执行估计不准进入 h”。

若保留 mixed TAPF/carrier 输入，两个 obligations 的时间下界默认取 `max`，不能直接把原 SOC heuristic 与 shelf time 相加。新的 combined bound 必须另有证明与小图 oracle 测试。没有证明时取更弱下界甚至 0。

## 6. 联合 Task-BR-PIBT：保留清障因果，去掉运输假互斥

### 6.1 编译对象

compiler 继续处理全部未满足 root 的候选搬法，保持 shared transaction context、递归栈、root-level backtracking 和预算耗尽返回 partial guidance。

“联合”要求一个 root 的已选 blocker 方案影响其他 root 的可选方案；冲突可以触发换 endpoint、换 displacement chain 或回退先前选择。不能退化成独立选完后仅扫描并加边。

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

不默认要求 `C 在 e Drop 后，B 才能开始 approach`。边应携带被释放的物理条件和被约束的事件：

```cpp
struct CausalEdge {
    TransferKey producer;
    Cell must_be_vacated;
    TransferKey consumer;
    Event consumer_event;  // Enter(cell)，必要时 Depart/Acquire 等
};
```

仅当任务需要机器人释放、或需要前驱确实落地时，才增加 Drop/RobotAvailable 依赖。

条件不是永久的“曾经腾空过”。若格子被重新占据，等待者进入前必须重新验证。D0 与 ExecutionView 均不依赖单调 completed bit 来跳过 occupancy。

### 6.3 冲突分类

| 关系 | 处理位置 | 行为 |
|---|---|---|
| 当前 grounded blocker 占据必要 cell/endpoint | Task-BR-PIBT | 递归搬 blocker，生成因果边 |
| 同一 shelf 被要求搬到不兼容 endpoints | Task-BR-PIBT | 选择替代方案或 backtrack，不能伪合并 |
| 两件 shelf 要占据同一 endpoint | 放置/因果协调 | 本轮选兼容放置，或显式安排一次再次 vacate；不是无限容量目标 |
| 两条未来 transit 路线有交点 | 执行协调 | 选择时序/绕路，不产生清障先后边 |
| 两个未来 first legs 经过同一 transit cell | 执行协调 | 不能直接删除整个 transfer；实际同拍冲突在 joint action 检查 |
| 某个 carried shelf 暂挡 transit | 执行协调/active episode | 等待或绕路，不让另一 free robot 去“搬走”已被 carry 的 shelf |

endpoint 是放置位置，transit 是可以分时复用的位置。不要把二者放进同一张永久 exclusive route ledger。

### 6.4 Storage 候选

保留“从 source 离开后，在一个 transfer 内到达第一个 storage endpoint”的分段约定；它是 transfer 抽象，不是新增物理禁行规则。长路径可由多个 transfer 及连续 custody 表达。

几何 candidate core 继续由 single/joint compiler 共用。PairCost 可只消费 canonical route；执行层调用同一拓扑/endpoint 约束寻找替代 route，不要求 BFS 穷举所有最短路。

还要区分“墙使 endpoint 不连通”和“目前有在途货架挡路”。后者可以生成暂待交通释放的 endpoint 意图，不能因为当前没有全空的 route hint 就把 root 的整个搬运需求永久隐藏。D0 保留其 endpoint/需求及阻塞原因，执行层用真实 custody 判断能否等待或重路由。PairCost 的抽象 rollout 仍不得把 shelf 移入另一个 shelf 当前占据的格子；有界估价失败继续是有限 penalty，不是不可达证明。

原 `reserved_destination` 在 storage endpoint 上有放置意义；在非 storage first-step 上只有当前同拍动作选择意义，应移出纯因果图的整 transfer 接纳条件。

### 6.5 图评分与优先级

第一阶段保留已有 density-aware priority/progress 作为基线，避免同时推翻密集图启发式。后续为少量候选图估计剩余清障链长度与总 work；执行层补入机器人 availability 后才评价时间。

被暂停的 root 不能从完成时间估计中消失，否则“少编任务”会虚假地得到更小 makespan。共享 blocker 的 work 只计算一次；后继尾长取 max，不能把整个 root PairCost 在每层重复相加。

基础优先级仍放在目标货架上，沿因果边传给 blocker。执行紧迫性可在 ExecutionView 中提升关键 blocker；不要恢复固定的“目标货架永远比匿名货架高”。

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

不能因为 canonical route 的旧第一格被挡，就判定整个 transfer 不可分配：先允许同 endpoint 换路或等待。也不能因为 assignable 就承诺这一拍能进入通道。

### 7.2 提前准备的边界

最小阶段只把 causally ready transfers 全部保留下来，足以修复 C 的核心问题。第二阶段再将执行器扩展到 selected chain 的有界后继准备，不恢复“任意 non-ready clear 都派机器人”的旧行为。

优先确保当前必要前驱有可用 robot，再用剩余资源提前 approach。只有一个机器人时，不得让它举着 A 等待还没有执行者的 B。Lift 仍只由物理 precondition 决定是否合法，但 preferred Lift 需要考虑是否会阻塞必要前驱或把机器人困在无法启动的等待链里。

这些限制都是 guidance admission，不是 operator tree 的合法性限制。

### 7.3 事件释放

前驱货架刚离开源格，后继就可以准备利用该格；不必等它完成整段运输或 Drop。following 允许时甚至可以在同一个 joint transition 释放/使用；不允许时必须等到模型规定的下一拍。所有判断以 oracle 的时间约定为准。

预测前驱未来会离开并不能当成“现在已经 empty”。若前驱实际延误、转向或再次占据该格，刷新 ExecutionView 和后继候选。

## 8. rho：从接口修复到 Makespan 派工

### 8.1 第一批先保留 Hungarian

先删除 full-route overlap 的任务过滤，再让原 Hungarian 接收完整的 causal-ready transfer 集合。Testcase C 已有证据表明该原 matcher 能给六个任务分配机器人，因此这一批不需要靠换 matcher 才能验证绕路修复。

owner continuity 改按 TransferId/TransferKey 比较，不再按可变化的第一腿或 vector index。Lift 后真实 carrier 由 kappa 给定；before-Lift handoff 是软决策。

### 8.2 Makespan-aware 扩展

对选定的待派任务集合 S，估计：

```text
E(r,m) = r 接手 m 后该任务预计完成时刻 + m 之后的剩余因果尾长
```

接近和清障可重叠时通过 event 的 max 关系合并，不把全部 duration 机械相加。可用 bottleneck matching 最小化 `max_m E(rho(m),m)`，再在最小阈值内用 Hungarian 最小化总 approach/work 作为次序。

把 `tail(m)` 直接加到 min-sum Hungarian 的每一列没有实现这个目标：当 S 固定时，所有 tail 之和是常数，未必改变匹配。

S 的选择必须考虑未派任务的延后完成时间，不能只挑短任务压低当前 max。内部 task 的 tail 只来自当前所选因果方案，是 guidance，不进入 admissible h。

### 8.3 不强制所有机器人立刻有任务

并行数量是手段，不是目标。允许某台机器人完成短任务后顺路做下一件，让另一台提前启动长任务。普通一次一一 Hungarian 不能完整表达这种未来顺序。

这种改进作为有界 dispatch lookahead：保留真实 busy robot 的物理绑定，只在预测中登记其可能释放时间和下一件任务；仅执行当前第一步，不提前把它从 kappa 中变 free，不无限复制“虚拟机器人”。第一阶段无需实现它即可争取 C 的最优 makespan。

### 8.4 Handoff

比较未来完成时间和真实剩余工作，再以 continuity 打破接近的平局。旧 owner 已走的距离是沉没成本，不能硬加成“换手必然损失的未来时间”。反复换手可有有界稳定偏好，但不得保证某个 owner 永久保留任务。

## 9. 联合运输 guidance 与 Carrier-PIBT

### 9.1 保留 endpoint，不固定 route

一旦 Lift 启动 transfer，默认保持 shelf、carrier 与合法 endpoint；route、预计通过时间和 LegId 可从当前真实状态重算。它是一项 preferred completion commitment，不是“未来一定无死锁”的证明，也不是 physical successor 限制。

normal reroute 不取消 TransferId。forced Move 先经过 apply_ops，再优先保留原 endpoint 重路由。若原 endpoint 不能产生 preferred route，显式记录 no-route，不把有限搜索失败等同于物理不可达；必要 recovery 候选仍可使用其他合法 endpoint，但必须标明重绑定原因。

### 9.2 时序路线的生成范围

对 active custodies 和 grounded/preparing assignments，建立有界、按冲突分组的 time-expanded route guidance：

```text
state = (cell, relative_tick)
actions = adjacent Move | Wait
destination = committed storage endpoint
```

搜索使用当前几何与 shared timed reservations。空 loaded 路线、不同时间交点、同向 following、对向 edge、endpoint arrival/hold 全部按模型区分。free robot 的预测路径若可得也进入 lower occupancy 视图；预测不全时不得宣称整个 horizon 是 robot-shelf collision-free。

给定完整已验证预测轨迹的局部候选，可以声称“在该预测条件下无冲突”；只有 `apply_ops(X, joint_op)` 验证后的下一步可以无条件接受。整个剩余 route 不是未来执行保证。

### 9.3 预约覆盖对象

每一件实际存在的货架都必须在预测视图中有占据解释：

- grounded 待接货架：在实际/预测 departure 前占据 source，包含 approach 和 Lift；
- carried 货架：从相对时刻 0 的真实位置开始，占据其预测路径；
- 没有 preferred route 的货架：在已知范围内保留实际占据，不能从 reservation table 消失；
- 抵达 endpoint：保留后续占据；Drop 只改变 carrying mode，不让 endpoint 变空；
- horizon 以外：不得当成已知空闲；用保守 terminal occupancy 或 unknown 状态。

other carried shelf 的当前位置不能同时被永久 static-block 和 timed trajectory 两套规则重复解释。暂时静止的预测若阻塞所有候选，应允许本次联合协调回退其首选轨迹，而不是用自造的静态 hold 证明无路。

### 9.4 Release time 与重规划

grounded task 的预计出发时间来自 robot approach、Lift 和相应 causal release event。未分配的 predecessor 没有可信 departure，不能凭空生成未来空位。预计时间只影响执行层，不进入 tau。

每个真实 transition 后校正轨迹锚点。可以复用未失效的几何 suffix，但发生 Wait、PIBT deviation、owner 改变、前驱延迟或新冲突时，要更新受影响的时间表。宏展开同样逐拍处理。

只在 Lift 时规划不够：robot approach 期间已有已知 route conflict 时就应保留替代方向；尤其不能等两台载货车进入无会车空间的走廊后才第一次协调。

### 9.5 回退，而非只让低优先者永久 Wait

按稳定优先级先选路线是候选生成策略，不是绝对通行权。低优先 transfer 失败时，在工作预算内尝试：同 endpoint 绕路、调整出发时间、改变冲突对的通过顺序、重新选择先前高优先 transfer 的 route。rollback 撤销该尝试的所有预约。

一个辅助搜索失败只返回 no-preferred-route/partial guidance。不要删 transfer，不修改 tau，不把该状态判无解。

若本轮所有 robots 都 Wait，而所等待的“未来释放”没有任何实际执行者，应回退造成等待的预测安排。不能依靠墙钟时间或同一 X 的重复 attach，让一个虚构 reservation 自动消失。

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

当前 joint constraints 必须先被尊重。route 与 endpoint 偏好都不能删除 fully constrained 的合法 Move/Lift/Drop。生产 storage-only Drop 规则始终由 oracle 保持。

## 10. Transition、cache 与 rewire

保留原稿不可变 SearchEdge trace、candidate edge 登记、stale parent 先刷新、逐拍重锚和 frozen constraint_order。新增时间/work 字段后，所有读取必须来自同一 edge record。

| 数据 | 输入/生命周期 | 更新规则 |
|---|---|---|
| PairCost/tau | U + immutable version | 只在 U 变动或版本变动时重新评价 |
| D0/upper priority | U + commitment key | 不读取 robots；缓存值不可被执行层原地改写 |
| ExecutionView | D0 + X + 实际 custody | 每个真实节点重新对齐；可标 active、fulfilled、pending、shadowed |
| rho/preparation | 当前 view、robot positions、上一 episode binding | 每步可更新，不改 tau |
| route/timed reservations | X + 当前 jobs + transition anchor | 时间每步校正；几何按失效事件重搜 |
| admissible h | X + problem objective | 独立计算/缓存，不读取 D0/route/lease |

in-flight transfer 在新 D0 中消失，并不使 custody 自动丢失。ExecutionView 导入真实 active episode，移除对同一 shelf 的重复派工，重查依赖条件；active endpoint 冲突导致暂时等待或显式重新选局部方案，不写入全局 U-only cache。

重锚只来自 `{previous_X, previous_guidance, executed_ops}`。新的 route选择可以发生于 reanchored 当前 X，但“新选择”与“物理事实恢复”使用不同阶段，不伪造已经发生的 Lift/Move。

## 11. One-empty、zero-empty 与 endpoint

通用递归仍自然得到 `C vacates → B moves → A moves`。无需 one-empty 特殊 compiler；预算限制、循环或候选耗尽也不保证一定找到有效链。

对所有可通行格均可存储且禁止 following 的一空格模型，同一拍能够进入当前空格的 shelf 受唯一空格限制。生产允许 following 或具有额外 transit cells 时，不能推广成“任何时刻全场只能执行一个任务”。approach、Lift、多个 transport legs 都可能重叠。

zero storage slack、zero current empty storage、zero empty upper traversable cells 是三件不同的事。原 rotation-record 与 exhaustive physical search 保留；同步 bundle 可后续优化，不能因为 compiler 没有 ready leaf 而删除可行物理 successors。

endpoint ownership 表示已选放置方案的未来占据，不是不可逆的最终完成。已经落位的 target 仍允许被搬开。搬运方案形成 cycle 时需另选 endpoint/方向，或在模型允许的同步组合中解决；不能用无条件 task-DAG 成功假设掩盖 cycle。

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

rewrite、OPEN 的界、goal incumbent、两遍候选选择与 repair 使用同一比较器。在静态、无外部绝对时钟约束的物理问题中，同一 X 的较早 lexicographic g 支配较晚到达；timed reservations 只是 guidance，不改变这个性质。

若 `g_T+h_T > T_inc` 可剪；若时间下界相等，仍可能改进 work，应再检查 `g_W+h_W`。等价地，在两个分量均为对应下界时使用 `f >=lex incumbent`。不能继续沿用只针对 SOC 的 scalar f。

### 12.2 首解以后

保留快速首解和同一个 `TAPFPlanner::solve()`。首个可交付解产生后，剩余预算用于同一实现的 makespan 改进。固定首解终态 assignment 的重跑只是一种候选尝试，不是 singleton 实例跳过所有改进的理由。

第一批可以保留两遍控制器，只改比较目标并单独验证；后续接通普通首解后继续探索。不同阶段通过代码提交与实验变体对照，不在 production 添加 legacy fallback。

修改 g 并不意味着 DFS 首解自动变好；route/dispatch guidance 和剩余时间内的替代搜索决定有限预算表现。未经完整公平展开和 frontier lower-bound 证明，不声称一般实例 eventually optimal。

### 12.3 Repair 与 deadline

repair 继续做原来可重放的 exact-state / grounded-shelf-projection 变换，但接受条件改成 `candidate.cost <lex original.cost`。允许少用时间而多走几步；相同时间也可接受更低 work。

保留 strict total deadline：搜索、清理、修补、成本计算和最终 replay 都占预算。第二候选超时或非法时保留第一份已经完成验证的可交付 incumbent。不得因尝试一个更好的 raw candidate 而丢掉它。

修补后的外部 incumbent 是上界，不能直接覆写原 search node 的 g；若希望把修补路径加入搜索图，必须注册其真实 trace。macro 途中已达物理 goal 时，可截取最早满足 goal 的有效前缀，不附加非必要收尾动作。

### 12.4 兼容性边界

不改变物理模型、operator set 或 SearchKey。zero-shelf 原 TAPF 在其原 objective/API 下继续要求逐位兼容；若请求的 objective 本身改成 makespan，就不能同时无条件要求输出与旧 SOC 搜索完全相同。

通过现有 solver 的显式 objective/cost 接口表达问题目标，不通过检测实例名或“有没有货架”切换另一套 planner。shared search 内仍只有一个执行流程。

### 12.5 完备性

需要同时满足：有限物理状态、固定完整 operator 枚举、每次有限的 guidance 预算、fully constrained 直通 oracle、无不合法剪枝、必要节点最终有机会展开。route/dispatch 只调整偏好，不能重置并丢弃原 constraint-tree 的未枚举项。

“保留 endpoint”本身不是安全或活性证明；“有一条无碰撞预测 route”也不意味着真实机器人一定按时执行。正确性仍由每个实际 transition 与完整输出重放保证。

## 13. 代码修改落点

以下沿用原文已给出的模块/函数名；新增类型和 helper 名是建议接口，需对照实际源码落地，不是假定仓库已存在。

| 模块 | 修改 |
|---|---|
| `tapf_planner.hpp` | 区分 TransferKey/episode 与 LegId；event 条件；ExecutionView；PlanCost/edge 时间 |
| `carrier_guidance.hpp::ordered_shelf_candidate_window()` / `reachable_storage_transfers()` | canonical endpoint 几何与 runtime reroute 分工；执行层能找同 endpoint 替代路 |
| `TaskBRCompilerState/Transaction` | 保留递归/回退；修正 prefix-only merge；不按 transit first-leg 竞争删除整个 transfer |
| `ready_tasks_with_custody()` | 删除完整 route 空间交集过滤；区分 causal ready、准备、当前 move |
| `recover_task_br_custody()` | 恢复 episode endpoint；普通绕路不丢 episode；按实际 transition 更新 LegId |
| `match_ready_tasks()` 及 rho adapter | 先用 TransferId 稳定原匹配，再接完成时间派工 |
| `attach_carrier_guidance()` | 接入非缓存 ExecutionView 与 timed routes；U-only 内容继续复用 |
| `funcPIBT()` | 用动态 leg；显式到 endpoint Drop/continuation；保留全部 oracle 合法 fallback |
| `TAPFPlanner::solve()` / `rewrite()` | lexicographic g/h/f、incumbent 与 OPEN 重入 |
| `dd_planner.cpp` | 两遍/首解后预算及全局候选比较统一 `(T,W)` |
| `dd_plan_repair.cpp` | 词典序接受条件，原重放安全性保留 |
| `dd_carrier.cpp` / Python validator | 物理规则原则上不改；只核对成本报告/时间定义和 replay 一致 |

## 14. 分阶段实施顺序

**A. 契约与证据冻结。** 固定 S0/S1/S2、source/binary SHA、YAML 字节和全部旧计划。明确哪些 tests 是物理语义，哪些只保护将被修改的策略。

**B. Objective 接通。** 先以微型计划测试 `(54,90)` 与 `(31,93)` 的比较、macro cost、rewrite、repair 与 h。单独记录该提交的性能，不假定首解会改善。

**C. C 的最小运输修复。** 增加 transfer/leg 区分，取消 full-route filtering，支持固定 endpoint 的时序兼容路线和等长绕路；保留原 rho 作为基线。先查 Testcase C，再检查原 storage 往复回归和 dense suite。

**D. 因果事件与准备。** 将等待整 transfer 的边细化到必要 vacate/use 事件，受控开启后继 approach；不得抢走前驱必需机器人。测试当前空 storage 与净 storage slack 的区别。

**E. Makespan 派工与有限改进。** 接入 earliest-finish/critical-tail 估计、瓶颈匹配或小范围 dispatch lookahead，并接通首解后改进。每项单独做消融；第一层 tau 的目标不在同一提交改变。

阶段是实现提交，不是新增运行时策略开关。最终仍只有一个 Carrier-LaCAM pipeline。

## 15. Protected tests 的迁移

必须先审查再修改以下旧契约：

- `non-ready internal tasks 永不进入 rho/不得 approach`：改为未满足 movement 条件不能执行该 move，但满足准备条件可接近；
- `same first-leg effect 合并全部 roots`：改为完整 transfer 效果兼容才合并，LegId 只去重物理动作；
- `custody remaining suffix 必须完全相同`：改为 episode endpoint/physical binding 连续，route 可变；
- `任何空间 route overlap 都删除低优先任务`：删除，改成按时间协调；
- `所有修补 SOC 不增`：改成 `(T,W)` 不增且 replay 合法；
- `高优先行永不推迟/机器人必须全部活动`：只保留因果服务与明确调度策略的必要合同，不当成 makespan 定理；
- `无 storage map / singleton 计划哈希恒等`：在无语义改动且相同 objective 下保留；objective 或 dispatch 已改变时改验合法性与质量，不虚报 bit parity。

搜索 key、目标集合、storage-only Drop、全 primitive successor、真实 custody anchor、trace rewire、deadline 和双侧 replay 等物理与交付契约继续保护。

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
| 缓存 | 同 U 的 PairCost/tau 不随 robots 改；D0 不被 execution overlay 写坏；时序信息不进 upper cache |
| 完备性 | no-route、endpoint 偏好、prep admission 失败时，fully constrained successor 与 oracle 相同 |
| 交付 | no corridor Drop；所有输出 replay；strict 10s；保留已验证 incumbent |

禁止用“所有已选择局部任务都必须不可逆完成”作为新 physical invariant，也不要用 C 的某个 robot 编号或指定通道替代通用测试。

## 17. Benchmark、Testcase C 与验收

### 17.1 固定协议

保留原 77 例 release 集、固定 development 子集、实例字节、following 语义、seed、10s 和物理核并发协议。C 及本轮微例新增为独立可审计组；扩展集合与算法改动分开记录，不覆盖历史结果。

同机配对报告 success、真实 T、W、首次可交付解时间、总 runtime、timed-helper 时间/展开数、owner handoff、causal waiting、traffic waiting。先检查 success 和合法性，再在 common-success 集比较质量。新 objective 下 SOC 是次级指标，SOC 增加要披露但不能自动推翻更小 T。

### 17.2 C 的证据层次

| 计划 | T | W | 证据 |
|---|---:|---:|---|
| 旧 Planner | 54 | 90 | 用户提供的 production 记录 |
| 并行参考 | 31 | 93 | 用户报告通过仓库权威 validator；不是 Planner 输出 |
| 后续候选 | 31 | 90 | 按正文规则独立重放通过；仍待仓库权威 validator |

C 的六个 goal 初始为空，提供的参考方案不搬匿名货架，因此 b4 与 b5 之间没有“必须先搬走对方”这一清障依赖。它主要验证运输协调，不替代递归 blocker-chain 测试。

### 17.3 C 的 makespan 下界

这是根据 S2 参数的新推导，不是原报告已有的最优性证明。

b4 从 `(2,17)` 到 `(10,3)`，最少 22 次四邻接 loaded Move。最近 robot R3 从 `(4,12)` 到 pickup 最少 7 拍。Lift/Drop 各 1 拍，所以：

```text
T* >= 7 + 1 + 22 + 1 = 31
```

其他 robots、handoff 或改变 route 不能减少 b4 在该模型下必须经历的这条操作链。若 31-step 参考经过当前模型的权威 replay，则它达到 makespan 最优值；不推出 W 最优。模型、goal set 或操作时长改变后，必须重新计算此证书。

### 17.4 验收不要绑定 robot 身份

接口回归：原始六任务不因 full-route intersection 被删；原 min-sum rho 的固定 probe 可以继续复现它的已知六行匹配。

最终质量回归：计划合法、goal 正确、无 corridor Drop，争取在固定预算内达到 `T=31`，并报告 W。不要要求 `R5 必须搬 b5` 或 `active_robots=6`。31/90 的五机器人候选说明这种身份/满载约束并非 makespan 所必需。

开发阶段可设置暂行质量线，但必须显式标为阶段验收；不能把较松门槛称为达到最优，也不能事后降线。W<=90 只有在候选通过权威 validator 后才可考虑成为更强次级质量目标，不声称它是已知最优 W。

## 18. 本轮不一起重写的部分

不嵌套完整 BR-LaCAM，不把整个 warehouse 的时空路径一次性冻结，不引入 robot-to-tau execution price，不强制所有任务提前派工，不以无限 lease 避免所有 handoff。

第一层 bottleneck matching、完整 multi-task scheduling、同步 rotation bundle、混合任务最优性证明，以及取消所有既有 density-aware 排序，都留作独立变更。设计上的层次清楚不意味着已证明这套 guidance 在全部 dense cases 更快；新增计算开销必须单独测量。

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

    dispatch = AssignRobotsAndOptionalPreparation(
                   candidates, preparations, X, previous_transfer_bindings)
    # 第一阶段使用原 Hungarian，后续提交才引入完成时间调度

    jobs = ActiveEpisodes(recover) + AssignedTransferHints(dispatch)
    traffic = BuildBoundedJointTransportGuidance(
                  X, view, jobs, previous_route_hints)
    # 未计划货架仍占据；时空失败不删除物理动作、不修改 tau

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

下文 §20、§22 为基线原文，不是 v5 验证。§21 是 storage 部分的统一新摘要，不再引入与上文相冲突的另一层“增量优先”规则。


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

尤其禁止两种实现捷径：

```text
if first_preferred_route_conflicts:
    erase_entire_task_before_rho

if two_jobs_share_first_leg:
    merge_their_entire_endpoint_demands_without_checking
```

同时保留原 storage 修复的必要部分：不能在 transit Drop；不能在每次 loaded Move 后丢失整个 transfer，只因新 D0 暂时看不到该 root 就立刻反向返回 source；不把 route helper 失败当成物理无解。必要的有计划回退仍可作为合法候选，不能将“没有任何 reverse”升级成全局正确性定理。

---

> **以下 §22 是 2026-09-04 基线原文。其 APPROVE、测试数量、时间和 plan hash 都属于旧实现，不用于证明本稿的新目标、动态绕路或事件准备已验证。**

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
