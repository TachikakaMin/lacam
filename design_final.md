# Carrier-LaCAM 最终设计文档：Objective-PIBT——优先级协商下的目标-任务联合编译

状态：v4.0，2026-09-02。

- 设计基准：Objective-PIBT——把 PIBT 的"优先级继承 + 回退"从机器人层
  抬升到目标层，形成两层对称结构：**objective 层解决多拍语义冲突，
  carrier 层解决单拍物理冲突**；
- 实现基准：v3.0 final（task 池 + committed ready + custody +
  delta 执行价 + 三轮外部 review 的全部修复 R1-R7/S1-S3），当前 head
  gate = `results_v3_round3_final`（严格 10s 协议：34/68、小图
  34/36；回归规模 C++ 161、Python 75）；
- 写法约定：**[设计]** = v4.0 目标语义；**[现状]** = 已实现并通过
  gate 的 v3.0 行为；**[落差]** = 落地时必须补齐的差异。未标注段落
  表示设计与现状一致、已实现。

本文取代 v3.0。物理模型、搜索骨架、h 的下界纯净性、输出修补、两遍
流程、实验协议全部保留；被 v4.0 重写的是 guidance 的**决策结构**：
从"先定目标、再铺路、再编任务、事后修冲突"的串行流水线，改为
"每个目标一次性选择完整套餐（ObjectiveOption），套餐间冲突用优先级
继承与回退在编译期协商"（§4-§7）。发现过程见 `report.md`；已闭合的
修复契约见 `debug.md` §10 与 git 历史。

---

## 0. 核心思想：两层 PIBT

v3.0 已建成的闭环：

```text
X -> tau_guide -> tasks -> rho -> Carrier-PIBT -> X'
```

它的遗留缺陷是**串行 + 事后**：每个目标货架独立编译自己的任务，任务
之间的干涉（走廊交叉、落点竞争、目标格互占）靠去重、park、yield、
wait-for 等机制在冲突**发生之后**处理。A 的清障要挪的货架干涉了 B 的
计划时，没有任何渠道在编译期说：

> A 优先级更高，请 B 先重新考虑自己的目标、路径或临时落点。

v4.0 的回答是把 PIBT 的核心协议抬升一层：

```text
第一层 Objective-PIBT（语义级，多拍）：
  每个目标货架从若干"套餐"中选择
      套餐 = (目标格, 松弛路径, 当前前线任务, 资源认领, 代价)
  套餐冲突 -> 低优先级方继承高优先级、带约束重选 -> 递归 -> 回退

第二层 Carrier-PIBT（物理级，单拍）：
  任务 -> 机器人分配 -> Move/Lift/Drop/Wait -> 挡路机器人递归让位
```

完整的优先级链条：

```text
目标货架(使命) -> 被请求让路的目标 -> 前线搬运任务 -> 执行机器人 -> 挡路机器人
```

前半段由 v4.0 新建，后半段 v3.0 已经存在。两层都只做**排序**，物理
合法性与完备性仍由约束树和 `apply_ops()` 独立保证（§8）。

**[现状]** v3.0 的跨目标协调是四个局部机制的拼盘：tau 匈牙利匹配
（只管目标格互占）、park/yield（二元让路，从不考虑换目标）、settled
重开（事后踢开）、livelock taboo（盲目搅动）。执行价（delta 定价）
只表达"这个目标自己难干"的**数值反馈**，无法表达"A 的方案成立的
前提下 B 的方案已不成立"的**结构反馈**。

---

## 1. 物理模型 `[保留，已实现]`

### 1.1 输入

- 4 连通 grid 和 wall；labeled robots；labeled target shelves 与匿名
  shelves；每个 target shelf 的 eligible goal 集 `G_b`（`|G_b|=1` 为
  固定目标特例）。
- loader（`DDInstance::finalize()`）检查：机器人/货架不重叠、target
  start 唯一且必须是货架、goal 可达性（墙连通分量过滤）、覆盖
  matching（Hall 条件）。

### 1.2 状态

```text
X = (Q_robot, Q_target, Q_anon, kappa)
```

`Q_anon` 排序存储（匿名对称性消除）；`kappa[i]`：free / 携带匿名 /
携带 target b。搜索 key = `SearchKey{Config, ShelfState}`。
**option、claims、priority、task、path 一概不进 state key。**

### 1.3 动作与合法性

每拍每机器人 `Wait | Move | Lift | Drop`。`apply_ops()`
（`dd_carrier.cpp`）是唯一物理裁判：下层 vertex/swap 冲突、上层唯一
占据（S1）、lift/drop 前置条件、carrying 不变量。默认允许
following；no-following 仅为显式测试 oracle 参数。Python
`ddbench.validator` 对输出计划独立整体重放。

### 1.4 终点与代价

```text
is_goal(X) iff 每个 target b 落地且 Q_target[b] ∈ G_b
```

终点判定不读任何分配/优先级；已停好的货架可被再次搬开。物理 SOC =
α·载货移动 + β·空载移动 + γ·抬放 + δ·匿名搬运（默认单位权重；
`DD_ALPHA..DD_DELTA` 是数值输入，经共享 parser 严格校验）。

---

## 2. 搜索不变量 `[保留并扩展]`

1. 唯一生产 solve loop 是 `TAPFPlanner::solve()`。
2. `dd_planner.cpp` 只做适配、两遍驱动、基线与修补，不搜索。
3. action-constraint tree 最终可枚举全部 primitive operator。
4. fully constrained joint op 由 `apply_ops()` 独裁。
5. **一切 guidance 只改候选顺序**：option / claims / 优先级（基础、
   继承、任务）/ task / rho / path / park / cooldown 不进 state
   key、不改 goal condition、不永久删除合法 successor。
6. 零 shelf 输入自然退化为原 TAPF（数据结构为空、逐位一致、禁止
   模式检测）。
7. singleton goal-set 自然退化（option 的目标维度只剩一个候选，
   协商只剩路径/落点维度）。
8. `tau_LB`（admissible h）与一切协商信号分离：执行价、干涉、
   优先级、粘滞永不进入 h。
9. task 完成条件只读物理状态与执行机器人的 custody episode。
10. **[v4.0]** Objective-PIBT 协商失败或超出预算时，**回退到 v3.0
    匹配路径**（下界+粘滞的 solve_tau 匹配 + 现行任务编译），绝不
    判定后继不存在——协商是排序建议的生成器，不是可行性判定器。

生产入口有限 deadline 下停于首个 incumbent，不声称最优。

---

## 3. 任务层：ManipulationTask 与共享 `[现状为基础，v4.0 扩展]`

### 3.1 任务语义（已实现）

```text
MoveShelf(s, from -> to, root = b -> g)
```

- `s`：被搬运的货架。labeled target 用索引标识；匿名货架用**当前
  cell 标识**（等价类），custody 由执行机器人的 kappa episode 记账，
  身份绝不进物理状态；
- `to_committed`：编译期承诺的落点（one-empty ready 任务：to =
  vacancy）与 advisory hint 的区分；承诺型落点在执行期保持 soft
  commitment（自身站格计为可落、失效才重算），非承诺型交由 carrier
  每节点选择；
- `TaskId = hash(shelf, from, root)`：`to` 不参与——同一任务的落点
  细化不改变身份，rho 粘滞与 custody 生命周期依赖它；
- 完成条件：`s` 落地于 `to`（labeled 为纯物理谓词；匿名经 custody
  episode 判定）。

### 3.2 共享与合并 `[v4.0 重写 S3 去重]`

**[设计]** 相同的物理搬运是**共享**不是冲突：

```text
A 与 D 都需要 MoveShelf(C, u -> v)
  => 合并为一个任务：roots = {A, D}，task_priority = max(prio[A], prio[D])
```

真正的冲突只有两种，交给 §6 的协商：

```text
同一货架被要求去不同落点：MoveShelf(C, u->v1) vs MoveShelf(C, u->v2)
不同货架抢同一落点：      MoveShelf(C, ->e)   vs MoveShelf(D, ->e)
```

**[现状]** S3 的 `emit_task` 按 pickup 去重但**丢弃**第二个 root
（只留优先级槽位高者），服务关系信息丢失；task 只有单一 `root`。

**[落差]** `ManipulationTask.root` 扩展为 roots 集合（或主 root +
共享计数）；`task_priority` 按 roots 取最大；rho/req_order 与
custody 读取 task_priority。

---

## 4. ObjectiveOption：目标-路径-任务-资源的联合选择 `[v4.0 核心，重写]`

### 4.1 设计

对每个未完成目标货架 b，不再串行地"先定 tau[b]、再铺路、再编任务"，
而是生成并挑选**完整套餐**：

```text
ObjectiveOption ω_b = (
  g       ∈ G_b,          终态目标格
  P,                       到 g 的 least-blocking 松弛路径（信息，不预约）
  m,                       当前第一个可执行的前线任务（§4.2 编译）
  R,                       资源认领（§4.3，刻意极小）
  score                    货架侧代价 + β×机器人实现代价
)
```

选中后，旧概念全部成为套餐的**视图**：

```text
tau_guide[b] = ω_b.g          （目标分配）
task[b]      = ω_b.m          （前线任务，进池参与合并）
protect[b]   = ω_b.P          （软保护走廊，非认领）
```

goal 与 task 是**同一次选择的两个输出**——这消灭了 v3.0 里"tau 定了
之后任务才编、编完才发现互相拆台"的相位差。

**候选生成有硬上限**（防止 option 爆炸）：每个目标最多考虑
`OBJ_GOAL_CANDIDATES = 2` 个目标格（按 lb 取前二；singleton 自然只有
一个），每个目标格一条当前 PathCache 路线；替代路线/替代落点只在被
协商请求时**按需**生成（§6）。

### 4.2 前线任务编译（沿用 v3.0 frontier compiler，已实现）

对候选 (b, g) 沿路径 P 编译第一个可执行任务：

```text
路头畅通            -> serve：MoveShelf(b, pos_b -> g)
路上第一个 blocker s*，s* 可启动（有相邻上层空格）
                    -> clear：MoveShelf(s*, pos -> 建议落点)
s* 不可启动且全场恰一个上层空位（one-empty）
                    -> ready：沿空位到 s* 的 routing 链，取紧邻空位的
                       货架 s'，MoveShelf(s', pos -> vacancy, committed)
零空位              -> 保留普通 clear（悬停洗牌是承重梁，report.md 教训）
```

`HEAD_DROP_SCAN_CAP = 64` 限制落点建议 BFS；不可启动的 head 不给
hint。链上每 root 至多 `CLEAR_CHAIN_K = 3` 个候选任务。

### 4.3 资源认领（claims）——刻意极小 `[设计边界]`

一个套餐只认领：

```text
R = { 终态目标格 g,
      被操纵货架的当前格 m.from,
      当前落点 m.to（若已定）}
```

**禁止认领整条路径**。整路径预约会让 Objective-PIBT 膨胀成第二个
MAPF 规划器，与外层 LaCAM 抢活；走廊冲突继续用软代价（LAMBDA_BLK
罚 8 倍 + protect 走廊让空闲机器人避让）表达。协议保持 LaCAM 式的
lazy：**当前节点只裁决最有希望的下一段语义动作，走一步重算**。

**[现状]** 无 option 概念：tau 由 solve_tau 单独匹配（下界 + 粘滞 +
settled/carried 锁 + livelock taboo），路径/任务事后生成；机器人
实现代价以 delta 执行价的形式在 tau 之后单轮反馈
（`compute_execution_prices`），只能推动换目标格这一种替代。

**[落差]** 新增 ObjectiveOption 结构与候选生成器；`solve_tau` 的
guidance 侧让位于套餐选择（settled/carried 锁与不可扩展 fallback
语义并入 claims 账本的目标格维度）；delta 执行价并入 `score` 的
机器人项，独立的 price→tau¹ 管道退役。

---

## 5. 优先级体系 `[v4.0 重写]`

### 5.1 三个概念

优先级属于"**完成目标货架 b** 这件事"。one-shot 问题里每个目标货架
唯一对应一个使命，因此直接存 `base_priority[b]`；τ(b) 换格不改变
优先级归属。被搬动的货架**不一定**是优先级的拥有者——搬的是 blocker
C，活是为 A 干的，就挂 A 的牌：

```text
base_priority[b]     目标货架 b 的基础优先级（§5.2 冻结规则）
inherited priority   A 请求 B/C 让路时沿依赖链下传的优先级
task_priority(m)     = max over b ∈ roots(m) 的有效优先级
```

匿名货架没有自己的优先级；它被编入的清障任务全部继承 root 的
优先级。

### 5.2 冻结规则（防震荡的关键决策）

**base_priority 在每个 search pass 的根节点按 lb 剩余距离排序一次并
冻结，pass 内不变。**动态重排（如按当前剩余距离）会使压制方向随
推进翻转、A/B 互相翻烙饼——与 step-3 绝对价格震荡（实测 makespan
爆至 17782）同源，教训直接复用。并列按目标索引定序（确定性）。

### 5.3 对称规则（协议能终止的根基）

> 压力只从高优先级流向低优先级。A 的套餐若与**更高**优先级的既有
> 认领冲突，改方案的是 **A 自己**（换自己的下一个套餐）；A 无权请求
> 高优让路——与 PIBT 中低优不能赶走高优完全一致。

### 5.4 防饿死（aging）

低优目标可能被持续压制。复用现有 livelock 机制客串衰老：目标 b 的
`no_progress` 达到 `LIVELOCK_WINDOW = 24` 时，b 在**本次协商轮**获得
临时最高优先级（不改写 base_priority），协商结束即失效——这泛化了
v3.0"每 livelock epoch 释放一行 tau"的定向修复。

**[现状]** `active_targets` 已按剩余距离排序逐个处理（隐式优先级，
每节点重算、未冻结、未用于协商）；任务优先级是 100/50-k 槽位制；
livelock taboo 存在但与优先级无关。

**[落差]** pass 级冻结的 `base_priority[b]`；task_priority 取代
100/50-k 槽位（字典序 = 任务优先级降序、链深升序，泛化原有语义：
serve=链深 0）；aging 接入协商轮。

---

## 6. 协商协议：继承、回退、合并、终止 `[v4.0 核心，重写]`

### 6.1 主循环

每个 physical node 上：

```text
ResolveAll(X):
  按 base_priority 降序处理每个未完成目标 A（aging 目标插队到最前）:
    按 score 升序尝试 A 的候选套餐 ω:
      检查 ω.R 与"认领账本"的冲突:
        无冲突            -> 接受 ω，登记认领
        与相同任务重合    -> 合并（roots ∪，priority=max），接受
        被更高优认领占用  -> 跳过 ω，试 A 的下一个套餐（对称规则）
        被更低优 B 占用   -> B 继承 A 的优先级，递归请求 B 重选（6.2）
             B 重选成功   -> 接受 ω 与 B 的新套餐
             B 无路可走   -> 回滚，A 试下一个套餐
    A 所有套餐都失败 -> A 本节点退化为 yield（不认领，保留 goal 意向）
  输出：每目标的选中套餐 -> 合并后的任务池 -> rho -> Carrier-PIBT
```

### 6.2 被请求方的替代序（改动最小优先）

B 收到带继承优先级的重选请求时，依次尝试：

```text
1. 保持 goal，换一条路径（PathCache 替代路线，按需计算）
2. 保持 goal，换被操纵货架的临时落点（另一个空格）
3. 保持 goal，换 vacancy routing（one-empty 多路线时）
4. 更换终态目标格 g' ∈ G_B
5. yield：本节点放弃认领、暂不推进（保留 goal 意向）
```

B 的新套餐若又占用更低优 C 的资源，A 的继承优先级继续下传
（A→B→C 链）。**高优不是霸权：它只是先选的权利**——若 B 连 yield
都会使全局无解（不会发生：yield 总可行），或 A 更愿意保留 B 的方案，
A 回退自己换套餐。

### 6.3 终止与预算

- 处理序严格按优先级降序 + 压力只向下 + 每目标套餐数有限
  （≤ OBJ_GOAL_CANDIDATES × 替代序长度）⇒ 单节点协商必终止；
- 递归深度上限 `OBJ_PIBT_DEPTH = 4`、单节点重选总次数上限
  `OBJ_RESELECT_CAP = 16`（结构常量，调整须过 gate）；
- 超限 ⇒ **整体回退到 v3.0 匹配路径**（不变量 10）；
- 每节点协商时间计入 `guidance_time_ms`，受 §11.6 预算条款监督。

### 6.4 粘滞（option hysteresis）

上一节点的选中套餐在 score 差 ≤ `ASSIGNMENT_HYSTERESIS = 2` 内优先
保留（泛化 v3.0 的 tau 粘滞与 rho TaskId 粘滞）；custody 承诺
（§3.1）不受协商影响——正在执行的搬运做完为止，除非任务失效。

### 6.5 与 one-empty ready 的关系（特例验证）

one-empty 下协议自动推导出 v3.0 的 ready 任务：

```text
A 要前进 -> 需要 blocker 离开 -> blocker 无处可去
        -> 空位必须先移过来 -> 真正能动的是空位旁的 C
=> 叶任务 MoveShelf(C, pos -> vacancy, roots∋A, 继承 A 的优先级)
```

现行 ready 编译（固定 BFS 规则）是本协议在"单空位、单路线"下的
退化形态——该特例已被 gate 验证（one-empty 锚例 -63%），是通用化
方向的直接证据。多空位路线、多停车格、多目标格时，协议获得现行
规则没有的**比较与换方案**能力。

**[现状]** 无协商：冲突由 park/yield/settled 重开/livelock taboo
事后处理（§0 现状表）。

**[落差]** 认领账本、递归重选、回退、aging 插队、option 粘滞；
park/yield 语义被吸收为协议的第 5 替代（yield）与对头场景的自然
结果，作为独立规则退役（保留代码路径直至 v4 gate 证明可删）。

---

## 7. 两层 PIBT 的衔接 `[v4.0 扩展]`

优先级必须贯通到底，否则高优使命的执行机器人反而排队在后：

```text
rho：任务按 (task_priority 降序, 链深升序) 排序截取；
     匈牙利代价 = 下层距离，TaskId 粘滞折扣 2（不变）
Carrier-PIBT：机器人排队 = 角色分层内再按其任务/custody 的
     task_priority 降序（载目标 > 载匿名/让路 > 有任务空闲 > 空闲
     的分层保留）
custody：任务连同 roots/priority 随 Lift 过户，carry 全程生效；
     committed 落点优先于自身 tau 方向（S1 语义，已实现）
```

**[现状]** rho 按 100/50-k 槽位 + 深度 tie；PIBT 类内按剩余距离；
custody 已携带任务但无优先级字段。

**[落差]** task_priority 进入 rho 排序与 PIBT 类内次序；custody
结构体带 roots/priority。

---

## 8. Cost 与正确性 `[保留]`

### 8.1 三种量的铁律

```text
g  = 已执行的真实物理代价（get_edge_cost，α..δ 记账）
h  = tau_LB：无约束下界匹配的最优值（solve_tau 的 unrestricted 解）
score / 优先级 / 认领 / 继承 = 只决定尝试顺序，可以非 admissible
```

机器人实现代价（β 计价）、干涉、粘滞、taboo、优先级**永不进入 h**。
现有测试钉死：`dd_tau.h_equals_bruteforce_min_matching`、
`dd_tau.hysteresis_is_tie_break_only`、
`dd_tasks.execution_price_never_enters_admissible_h`、
`dd_anytime.admissible_h_never_exceeds_true_cost`。v4.0 落地时同型
扩展（option/claims 不进 h）。

**纯性口径**：`tau_LB`/h 是 X 的纯函数；option 选择与 score 允许
依赖 guidance 引擎缓存状态（PathCache 惰性失效，(X, cache epoch)
语义由 `dd_park_purity` 钉住），定义在 (X, 引擎状态) 上，给定 seed
逐位可复现。

### 8.2 完备性论证

1. state key 只含物理 X；
2. option/claims/优先级/task/rho 全部 ordering-only（不变量 5/10：
   协商失败只回退到 v3.0 路径，永不宣告不可行）;
3. 约束树最终枚举全部 primitive 组合（futile-lift 只降序；
   `constraint_order` 创建即冻结）；
4. fully constrained 组合由 `apply_ops()` 独裁；
5. 终点判定不依赖任何临时分配。

错误的协商至多变慢，不删除合法解。

### 8.3 输出修补（不变，已实现）

`repair_carrier_plan()`：先精确状态剪圈，再对"货架布局重复、仅机器人
站位不同"的段落用纯下层桥接（1 机器人最短路；2 机器人精确 A*；更多
机器人投影去环）。接受条件：**严格更短且加权 SOC 不增**；修补计入
所属 pass 的 10s deadline（超时返回 raw，pass 报诚实超时）；最终
整体重放兜底。保证：

```text
valid(raw) => valid(returned)
length(returned) <= length(raw)
soc(returned)    <= soc(raw)
goal(returned) = true
```

---

## 9. 生产流程与自动机制 `[保留，已实现]`

### 9.1 两遍规划（共享 10 秒）

```text
plan1 = 首解搜索(动态 guidance, 剩余时间) -> deadline 内修补
多目标且有时间: fixed = plan1 终态固化;
plan2 = 首解搜索(固定分配, 剩余时间) -> 修补
返回 SOC 低者（平手/失败保 plan1）; 最终 apply_ops 整体重放
```

搜索、修补、SOC 计算全部计入 10s；pass 内完不成修补即诚实超时
（R1）。runner 的 +30s 只保护进程启动与输出 IO；deadline 后的进程
收尾（大树析构）实测最长 ~1.6s，属测量包络非搜索预算。

### 9.2 自动机制表

| 机制 | 自动规则 |
|---|---|
| goal 匹配 / 套餐选择 | targets ≤256 精确（匈牙利/协商）；更大退化 row-wise（非单射 hint，gate 无覆盖，见 §13） |
| active targets | unfinished ≤256 全量；否则 carried 优先 cap 64 |
| rho | targets ≤256 匈牙利；更大贪心 |
| 粘滞 | tau/rho/option 固定 tie preference 2 |
| macro | targets ≤64 且每 pass 首解前；rollout 每 8 步刷新 guidance |
| assignment restart | 多目标首解终态 + 剩余时间时一次 |
| futile-lift | 同 shelf/cell 三态循环 ≥3 次（窗口 max(64, 8·V)）降序 Lift |
| livelock / aging | 24 步无进展：taboo 修复（现状）/ 协商插队（v4） |
| path cache | 生产 path-local 失效；strict 仅测试探针 |
| 优先级冻结 | 每 pass 根节点按 lb 排序一次（v4） |

### 9.3 无策略开关

环境输入仅 `DD_ALPHA..DD_DELTA`（共享 parser 严格校验：strtod 全
消耗、有限、非负）与 `DD_DEBUG_DUMP`。v4.0 的所有协商参数
（OBJ_GOAL_CANDIDATES / OBJ_PIBT_DEPTH / OBJ_RESELECT_CAP）是**结构
常量**，不暴露开关；调整须重跑 68 例 gate。

---

## 10. 工程落点

| 文件 | 现状职责 | v4.0 新增职责 |
|---|---|---|
| `lacam/src/carrier_guidance.hpp` | solve_tau（匹配+h）、frontier 编译、emit_task 去重、rho、custody、park/yield、执行价 | ObjectiveOption 候选生成、认领账本、协商主循环（ResolveAll）、替代序、合并（roots/priority）、aging 插队；执行价并入 score；park/yield 吸收 |
| `lacam/src/tapf_planner.cpp` | 搜索 loop、attach_carrier_guidance（tau→价→build）、funcPIBT、macro、futile、stale 重建 | attach 改挂协商入口（失败回退 v3.0 路径）；PIBT 类内序吃 task_priority |
| `lacam/include/tapf_planner.hpp` | ManipulationTask/CarrierGuidance/统计 | roots 集合、task_priority、ObjectiveOption、协商诊断计数 |
| `lacam/src/dd_planner.cpp` | 两遍、B0/B1、探针、统计映射 | 协商探针（option/claims/回退轨迹） |
| `lacam/src/dd_plan_repair.cpp` | 修补（deadline 内） | 不变 |
| `lacam/src/dd_carrier.cpp` | 物理 oracle 与终点 | 不变（权威） |
| `benchmark/ddbench/validator.py` | 独立重放 | 不变 |

诊断（`DDStats`，binary 输出并入 rows.csv——R5 管道已建）：现有
`tau_price_repairs / rewire_guidance_rebuilds / tau_time_ms /
guidance_time_ms` 之上新增：`obj_reselect_requests`（重选请求数）、
`obj_inherit_depth_max`、`obj_backtracks`、`obj_fallbacks`（回退到
v3.0 路径次数）、`tasks_merged`。

---

## 11. 实验规范与基线 `[保留]`

### 11.1 不可变协议

（与 v3.0 相同，全文有效）：实例/字节/seed 不变；**每 testcase 严格
10 秒内部 deadline，搜索+修补+SOC+重放全部计入**；jobs=14（16 物理核
留 2）；solver seed 0；成功 = 独立 Python validator 整体重放通过；
following 语义不混用；单位权重主表、非单位走 `--weights` 独立轴；
产物可审计（rows.csv + timing.json 带 git commit/binary sha/host
provenance、成功行带 plan_sha256、结果目录禁静默覆盖须 `--force`）。
成功率与质量同时报告：完整分母 + 尺寸分层；共同成功集几何均值 +
逐例改善/持平/恶化 + 恶化逐个解释；不得摘樱桃。

### 11.2 方法与消融

结构方法 full/B0/B1/B2/B3/B4 不变。v4.0 核心消融（结构变体，非
开关）：

```text
A. v3.0 匹配路径 vs Objective-PIBT 协商（即 obj_fallbacks 强制 100%
   vs 正常运行——同一二进制的结构对照）
B. 协商但无继承（冲突即自适应）vs 完整继承链
C. 合并共享任务 vs S3 式丢弃第二 root
```

（v3.0 的历史消融阶梯保留在 §12 记录中。）

### 11.3 基线阶梯与当前 head

```text
—— 宽松 wall 时代（修补不计入 10s；对照用）——
results_fix1_mscd                  36/68   总mk 214,199
results_rootfix_final              36/68   总mk  35,595
results_task_commit_final (v2.2)   36/68   总mk  34,860   ← 质量对比基准
results_v3_step{1,2,3}             36/68   0.9084（step3 vs 基准）
—— 严格 10s 时代（R1 起；诚实口径）——
results_v3_r1_strict10s            35/68   贴线例转超时
results_v3_review2_final           35/68   0.9369 vs 基准（common-35）
results_v3_round3_final (head)     34/68   0.9256/0.9341 vs 基准（common-34,
                                           18/7/9；S3 使 e8 例首解 5.2→8.5s
                                           丢失，二分归因，契约披露）
```

跨时代对比必须注明口径差异；质量比较一律在共同成功集上做。

### 11.4 v4.0 验收 gate

1. success 不低于当前 head：**≥34/68 且小图 ≥34/36**（若恢复被
   丢失的两个贴线/e8 例则如实报升）；
2. 共同成功集 mk/SOC 几何比 + 逐例三分 + 恶化逐例解释；锚例单列
   （`h4w10_e1_R1_s0` 现 880、`h6w10_e1_R1_s0` 现 933）；
3. 零 shelf（`test_tapf_compat`）逐位一致；R1/singleton 逐字节稳定
   **不**作为验收项（协商影响多目标与路径/落点维度，singleton 也会
   变）；
4. 消融 A/B/C 同协议报告；
5. `guidance_time_ms` 预算复核 + `obj_fallbacks` 比率报告（回退率
   过高说明协商没有实际生效，视为未落地）；
6. 全部成功计划过 C++ oracle 重放与 Python validator。

---

## 12. 测试计划与落地记录

### 12.1 v4.0 必测（RED 先行，逐条对应机制）

| # | 要求 | 对应机制 |
|---|---|---|
| 1 | 零 shelf 逐位一致 | 既有 `test_tapf_compat` |
| 2 | singleton 退化（协商只剩路径/落点维度） | §2.7 |
| 3 | 高优认领迫使低优改选替代目标格 | §6.1 核心场景 |
| 4 | 对称规则：低优不得驱逐高优，冲突时自适应 | §5.3 |
| 5 | 继承链 A→B→C 两级传导 | §6.2 |
| 6 | 回退：B 无替代时 A 换自己的次优套餐 | §6.1 |
| 7 | 合并：共享任务 roots={A,D}、priority=max、池内无重复 | §3.2 |
| 8 | one-empty：协议输出与现行 ready 编译在典型 fixture 等价 | §6.5 |
| 9 | option/claims/优先级不进 h（同型扩展现有纯性测试） | §8.1 |
| 10 | 防震荡：对称盘面跨节点不翻烙饼（优先级冻结+option 粘滞） | §5.2/§6.4 |
| 11 | aging：被压制目标 24 步后获得插队并推进 | §5.4 |
| 12 | 超限回退：深度/次数超限时行为等于 v3.0 路径 | 不变量 10 |
| 13 | custody/TaskId 生命周期与 committed 落点 | 既有（S1/R2 测试） |
| 14 | 全部返回计划过 C++/Python 重放 | 既有 |

### 12.2 已落地记录（保留作审计）

- **v3.0 step 1**（task 池/TaskId/粘滞）、**step 2**（frontier
  编译 + one-empty ready）、**step 3**（delta 执行价 + custody +
  惰性 rewire 重锚）：分步 gate 构成消融阶梯（B=1.0070、frontier=
  0.9237、A/C=0.9745），锚例 1053→417（宽松口径）。
- **R1-R7**（第二轮 review）：严格 10s、执行侧半环、全链 stale、
  strtod/ONE parser、β 计价、诊断列、provenance/plan-sha/防覆盖、
  文档与 CI。
- **S1-S3**（第三轮 review）：labeled ready 活锁修复（custody 扩展
  + committed 优先）、任意松弛标 stale、池级去重；depth 测试 fixture
  经两轮独立审查（REJECT→枚举验证→APPROVE）替换。
- 全部细节与逐例披露见 `debug.md` §10-§11（已闭合契约）与对应
  commit。

---

## 13. 边界与开放问题

1. **协商成本**：每节点 ResolveAll 的候选生成与递归受
   OBJ_* 常量硬约束，但真实成本要 gate 实测（大图 guidance 预算
   已达 2.5s/10s 量级；`obj_fallbacks` 是安全阀也是落地成色的
   试金石）。
2. **one-empty 串行化是物理**：单空位盘面同一时刻只有一个可执行
   搬运，机器人越多闲置越多；协商不改变这一点（改进方向：机器人
   富余时的深链补发与预测性站位，见已记录的后续工作）。
3. **大图首解 horizon**（20×20+ 10s 内 0/32）不因协商而解——协商
   优化的是小图质量域内的冲突浪费；horizon 需要层级搜索或可重构
   等价约减（研究项）。
4. `targets > 256` regime 无 gate 覆盖；row-wise 非单射 hint 语义
   仅单元测试保护。
5. 修补的多机器人桥仅保证合法缩短（1/2 机器人最短）；Python 原型
   719 vs 当前 1053（宽松口径）说明桥仍有余量。
6. 已知质量遗留：S3 的 e8 例丢失（并行度节流机制假设）、b4 式
   目标往返残余、pass2 大树析构包络（~1.6s）——均在契约中记录，
   v4.0 的协商与合并有望改善前两者，验收时专项检查。
