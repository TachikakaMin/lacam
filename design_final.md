# Carrier-LaCAM 最终设计文档：Objective-PIBT——优先级协商下的目标-任务联合编译

状态：v4.1，2026-09-02。

- v4.0 -> v4.1：第四轮外部 review 发现六处设计缺陷（继承/回退不可达、
  局部贪心破坏全局可行性、TaskId 与多 roots 冲突、claims 撑不起走廊
  宣称、aging 悬空、治理与回退措辞），经独立裁决（fable-5 max-effort
  review，全部判 REAL）后修订；裁决另发现 6 处补充缺陷（池宽度、
  不变量自矛盾、管道退役措辞、测试措辞先行、in-flight 豁免、命名），
  一并编入。**36/68 -> 34/68 的验收线变更获追溯性独立批准**（R1 严格
  10s 属收紧非放水；S3 丢例经二分归因并披露）；今后 gate 期望变更
  必须**先**过独立审查。
- 实现基准：v3.0 final（task 池 + committed ready + custody + delta
  执行价 + R1-R7/S1-S3），head gate = `results_v3_round3_final`
  （严格 10s：34/68、小图 34/36；C++ 161、Python 75）。
- 写法：**[设计]** = v4.1 目标语义；**[现状]** = 已实现并过 gate 的
  v3.0 行为；**[落差]** = 待实现差异。未标注 = 已实现。

本文取代 v4.0。物理模型、搜索骨架、h 纯净性、输出修补、两遍流程、
实验协议保留；v4.1 重写 guidance 决策结构：**目标维度由全局可行匹配
（tau0）+ 修复通道承担；路径/任务/落点维度由两阶段 Objective-PIBT
（tentative 登记 -> 优先级 Resolve）协商**。

---

## 0. 核心思想：两层 PIBT，三个决策维度

v3.0 闭环 `X -> tau -> tasks -> rho -> Carrier-PIBT` 的遗留缺陷：
每目标独立编译，冲突靠去重/park/yield/taboo 事后打补丁；A 的方案使
B 的方案失效时，编译期没有"请 B 带约束重选"的渠道。

v4.1 的结构（吸收裁决修正）：

```text
目标维度   tau0 = 全局可行匈牙利匹配（可行性由 Hall 条件兑现）
           目标变更只经"匹配修复通道"（压力项 -> 重解，即现有
           price 管道的保留与泛化）——局部贪心认领目标被裁决否决
路径/任务/落点维度
           两阶段 Objective-PIBT（前线资源协商）：
           Phase T：全体目标登记 tentative 套餐（来源=上节点选择，
                    根节点=零协商默认套餐），账本先建满
           Phase R：按有效优先级降序 Resolve——推挤/继承/回退真实
                    可达（v4.0 的空账本降序处理使其不可达，已修）
物理维度   Carrier-PIBT（单拍拼装）+ apply_ops 终裁（不变）
```

优先级链：目标使命 -> 被推挤使命 -> 前线任务 -> 执行机器人 -> 挡路
机器人。一切协商只做排序（§8）。

**[现状]** 跨目标协调 = tau 匹配（终点互占）+ park/yield + settled
重开 + livelock taboo + delta 执行价（数值反馈，仅能推动换目标格）。

---

## 1. 物理模型 `[保留，已实现]`

### 1.1 输入

4 连通 grid/wall；labeled robots；labeled targets（各带 eligible
goal 集 `G_b`，`|G_b|=1` 为特例）与匿名 shelves。loader
（`DDInstance::finalize()`）：不重叠、target start 唯一且为货架、
goal 可达性过滤、覆盖 matching（Hall）。

### 1.2 状态

```text
X = (Q_robot, Q_target, Q_anon, kappa)
```

`Q_anon` 排序（匿名对称性消除）；搜索 key =
`SearchKey{Config, ShelfState}`。**option、claims、优先级（基础/
有效/任务）、task、path 一概不进 state key。**

### 1.3 动作与合法性

每拍 `Wait | Move | Lift | Drop`；`apply_ops()` 唯一物理裁判（下层
vertex/swap、上层 S1 唯一占据、lift/drop 前置、carrying 不变量）；
默认 following，no-following 仅测试参数；Python validator 独立重放。

### 1.4 终点与代价

`is_goal(X)`：全部 target 落地于各自 `G_b` 内（不读任何分配）。
SOC = α·载货 + β·空载 + γ·抬放 + δ·匿名附加；权重经唯一共享
parser 严格校验（strtod 全消耗、有限、非负）。

---

## 2. 搜索不变量 `[保留并扩展]`

1. 唯一 solve loop：`TAPFPlanner::solve()`；`dd_planner.cpp` 只做
   适配/两遍/基线/修补。
2. 约束树最终可枚举全部 primitive 组合；fully constrained 组合由
   `apply_ops()` 独裁。
3. 一切 guidance（tau0/修复压力/option/claims/优先级/task/rho/
   path/park/cooldown）只改候选顺序：不进 key、不改终点、不删
   合法后继。
4. 零 shelf 逐位退化；singleton 自然退化（目标维度只剩单选，协商
   仅路径/落点维度）。
5. `tau_LB`（admissible h）与一切协商信号分离。
6. task 完成只读物理状态 + 执行机器人 custody episode。
7. **[v4.1]** 协商预算耗尽 = 未决目标**沿用 tentative（默认）
   套餐**——同一编译器的零协商输出，**不存在**对旧管道的调用或
   模式切换（裁决 6b：v3.0 行为即默认套餐，禁止 legacy fallback）。
8. **[v4.1]** 目标维度的全局可行性由 tau0 匹配保证；协商不得绕过
   匹配直接认领目标格（裁决 flaw 2）。

生产入口有限 deadline 停于首个 incumbent，不声称最优。

---

## 3. 任务层：物理身份、需求集合与合并 `[v4.1 修订]`

### 3.1 双键身份（裁决 flaw 3 修正版）

```text
PhysicalTaskKey = (shelf, from)           物理身份：粘滞/custody/池桶按它
DemandKey       = (root, root_goal)       需求身份
task.roots      = DemandKey 集合
task_priority   = max over roots 的有效优先级（每节点现算）
```

**落点与 roots 都不进物理身份**——这与既有"TaskId 不含 to"不变量
一致（advisory -> committed 的落点细化、roots 的增减都不改变"这是
对同一货架同一取货点的搬运"这一事实）。若把 committed 落点编入
key，两个异落点 committed 任务将互不相见、真冲突永不触发（裁决
指出的 v4.0 提案缺陷）。

`to_committed` 语义不变：one-empty ready 编译期承诺落点 =
vacancy，执行期 soft commitment（自身站格计可落，失效才重算）；
advisory 落点由 carrier 每节点选择。

### 3.2 合并与真冲突（同一 (shelf, from) 桶内）

```text
advisory  x advisory   -> 合并（落点 carrier 选）
committed x advisory   -> 合并，保留 committed 落点
committed x committed  同落点 -> 合并
committed x committed  异落点 -> 真冲突，进 §6 协商；
    但 in-flight 豁免：已被 custody 执行中的 committed 落点不可
    再协商——无论优先级高低，另一方自适应（裁决补充缺陷 5：
    执行中的承诺接近物理事实）
合并结果：roots = 并集，task_priority = max（不丢任何服务关系）
```

跨货架抢同一落点 = 另一类真冲突，进 §6。

### 3.3 池宽度（裁决补充缺陷 1）

**选中套餐发射完整清障链**（每 root 至多 `CLEAR_CHAIN_K = 3`），
而非只发射单个前线任务——池宽 = 机器人并行度的上限，收窄池宽会
放大已知的 e8 并行度节流问题（S3 教训）。

**[现状]** 单 root、`emit_task` 按 from 去重但丢弃第二 root；
TaskId 含 root。**[落差]** roots 集合、双键、合并规则、in-flight
豁免、task_priority 通道。

---

## 4. 目标维度：tau0 + 匹配修复通道 `[v4.1 重写]`

### 4.1 tau0：全局可行匹配（保留匈牙利）

每节点先解 `tau0 = solve_tau(...)`（现行实现保留全部语义：下界主
序、粘滞 tie、settled/carried 锁与两级 fallback、>256 row-wise
退化）。**可行性论证依赖它**：loader 的 Hall 条件 + 匹配 = 永远
存在完整分配；v4.0 的"每目标看前二贪心认领"被反例否决
（A{g1,g2} 高优、B{g1} 低优：贪心死锁，匹配显然可行）。

### 4.2 目标变更 = 匹配修复通道（price 管道保留并泛化）

目标格的重选**不走 claims 推挤**，走修复通道：

```text
压力来源（累积为 price 项，β 计价、delta 反震荡——现有实现）：
  a) 自身实现难度（现有：frontier 与机器人的 delta 距离差）
  b) [v4.1 新增] 协商残余干涉：路径/落点维度协商后仍无法消解的
     结构冲突，折算为对 (b, tau0[b]) 的压力
每节点至多一轮：tau0' = solve_tau(下界 + 压力 + 粘滞) —— 交替链
式的匹配修复，全局可行性始终由匹配保证
```

v4.0 中"price->tau¹ 管道退役"的表述**删除**（裁决补充缺陷 3）：
该管道正是目标维度结构反馈的实现载体。

**[现状]** 压力只有 (a)。**[落差]** 压力 (b) 的折算与累积。

---

## 5. ObjectiveOption：路径/任务/落点维度的套餐 `[v4.1 修订]`

### 5.1 套餐定义

```text
ω_b = ( g = tau0[b]（目标维度输入，非套餐自选）,
        P：到 g 的 least-blocking 路线（软信息，不预约）,
        chain：前线清障链（§4.2 编译规则，含 one-empty ready）,
        R：硬认领 = { chain 各任务的 from, committed 落点 },
        score：货架侧代价 + β·机器人实现 + 软干涉分 )
```

`tau_guide[b] = tau0'[b]`、任务池 = 各选中套餐的链合并（§3.2）、
软走廊 = P。goal 与 task 仍是同一节点联合产出，但目标维度经匹配、
其余维度经协商——分工来自裁决 flaw 2。

### 5.2 前线链编译（沿用 v3.0，已实现）

serve（路头畅通）/ clear（可启动 blocker，`HEAD_DROP_SCAN_CAP=64`
落点建议）/ ready（one-empty：routing 链上紧邻空位者入空位，
committed）/ 零空位保留普通 clear（悬停洗牌承重梁）。

### 5.3 认领与软干涉（裁决 flaw 4：范围收窄为"前线资源协商"）

**硬认领只有**：被操纵货架格（from）与 **committed** 落点。
advisory 落点绝不硬认领；目标格唯一性由 tau0 匹配保证，不进账本。
**禁止整路径预约**（协商器不得膨胀为第二个 MAPF 规划器）。

走廊交叉是**软干涉分**（进 score，不进账本）：下一步格 + 前 K 个
blocker 格 + vacancy 路线前缀与更高有效优先级足迹的重叠计分。
本机制据实命名为 **frontier resource negotiation**——它硬性解决
取货/落点/终点资源冲突，软性引导走廊避让；v4.0 宣称硬解走廊冲突
是动机与机制脱节。

---

## 6. 两阶段协商协议 `[v4.1 重写，修复继承/回退不可达]`

### 6.1 Phase T：tentative 登记（账本先建满）

```text
每个未完成目标 b 登记 tentative 套餐：
  有上节点选择且仍有效 -> 沿用（与 option 粘滞合一）
  根节点 / 已失效     -> 默认套餐 = (tau0[b], 当前 PathCache 路线,
                          现行编译链)  ← 零协商输出 ≡ v3.0 行为
账本由全体 tentative 认领构成
```

默认套餐同时是不变量 7 的退化载体：预算耗尽即"停止协商、沿用
tentative"，单管道无切换。

### 6.2 Phase R：按有效优先级降序 Resolve

```text
对目标 A（aging 插队者最前）：
  按 score 升序尝试 A 的套餐 ω：
    ω.R 无冲突                    -> 提交（替换 A 的 tentative）
    只与更高优的已提交认领冲突     -> 跳过 ω（对称规则：自适应）
    与更低优 B 的认领冲突          -> 推挤：B 以 A 的有效优先级重选，
        替代序（改动最小，goal 不在其中——goal 走 §4.2 修复通道）：
          1. 同 goal 换路线   2. 换被操纵货架的临时落点
          3. 换 vacancy 路线
        B 成功 -> 双方提交；B 的新认领又压更低优 C -> 继承下传
        B 耗尽 -> **返回 FAIL**（不得自行 yield）；A 试下一套餐
    A 全部套餐耗尽 -> 裁决器 yield：有效优先级低的一方本节点弃
        认领（保留 goal 意向）；同级按 base 优先级；确定性
未消解的结构冲突 -> 折算压力进 §4.2 修复通道
```

v4.0 的两个不可达缺陷在此修复：账本先建满使"低优占用"分支真实
可达；yield 移出 B 的替代序、置于双方耗尽后的裁决器，使"B FAIL ->
A 回退"路径真实可达（测试 #6 可 RED）。

### 6.3 终止与预算

优先级严格降序处理 + 推挤只向下 + 每目标套餐/替代有限 ⇒ 单节点
终止。上限：`OBJ_PIBT_DEPTH = 4`、`OBJ_RESELECT_CAP = 16`（结构
常量）。超限 -> 未决者沿用 tentative（默认解析，计数
`obj_default_resolutions`——不叫 fallback，行为上也不是：同一
编译器、同一管道）。协商耗时计入 `guidance_time_ms` 预算监督。

### 6.4 粘滞与豁免

上节点选中套餐在 score 差 ≤ `ASSIGNMENT_HYSTERESIS = 2` 内优先
保留；**in-flight committed custody 不可协商**（§3.2）；custody
承诺做完为止。

### 6.5 one-empty 等价（特例验证，保留）

协议在单空位/单路线下退化为现行 ready 编译（gate 已验证，锚例
-63% 的宽松口径证据）；多路线/多落点时获得比较与换方案能力。

**[现状]** 无协商（§0 现状表）。**[落差]** 两阶段协议、账本、
推挤/继承/FAIL/裁决 yield、软干涉分、压力折算、aging 插队；
park/yield 语义预期被协议吸收（保留代码路径直至 v4 gate 证明可删）。

---

## 7. 优先级体系 `[v4.1 修订]`

### 7.1 三个概念与归属

```text
base_priority[b]   目标货架（=使命）的基础优先级：每 search pass
                   根节点按 lb 排序一次并冻结（防震荡，step-3
                   绝对价教训）；并列按目标索引
有效优先级          每节点现算 = base + aging 插队（临时，不落盘）
task_priority(m)   = max over roots 的有效优先级
```

被搬运货架 ≠ 优先级拥有者：清障/让路任务挂请求方的牌（匿名货架
无自身优先级，全部继承）。

### 7.2 aging（裁决 flaw 5 修正版）

```text
progress[b] := best_lb[b] 下降（best_lb = 沿父链单调维护的
               lb(pos_b -> 已分配目标) 最小值）
               或 任一 root=b 的任务完成
no_progress[b]：父链传播的 per-target 计数；progress 即清零；
               tau0'[b] 改选（目标重分配）亦清零
no_progress[b] ≥ LIVELOCK_WINDOW(24) -> 本节点协商轮临时置顶；
               多个 aging 者之间按 base 排序；协商轮结束即失效
```

**custody 只存 roots（DemandKey 集合），绝不存优先级数值**——
有效优先级每节点从 base+aging 现算，否则 aging 的"过期"与"随
custody 过户"互相矛盾（裁决确认）。

### 7.3 贯通

rho 按 `(task_priority 降序, 链深升序)` 排序截取（泛化 100/50-k
槽位）；Carrier-PIBT 类内序吃 task_priority；匈牙利代价与 TaskId
（= PhysicalTaskKey）粘滞折扣不变。

**[现状]** 隐式优先级（active_targets 距离序，未冻结未协商）、
100/50-k 槽位、节点级 no_progress、custody 无 roots 字段。

---

## 8. Cost 与正确性 `[保留]`

### 8.1 三种量铁律

```text
g = 真实物理代价 | h = tau_LB（无约束下界匹配，纯 X 函数）
score/压力/优先级/认领/继承 = 只排序，可非 admissible，永不进 h
```

既有纯性测试（`h_equals_bruteforce`、`hysteresis_is_tie_break_only`、
`execution_price_never_enters_admissible_h`、
`admissible_h_never_exceeds_true_cost`）v4.1 落地时同型扩展到
option/claims/优先级。guidance 定义在 (X, 引擎缓存状态) 上，给定
seed 逐位可复现（`dd_park_purity` 钉住）。

### 8.2 完备性

state key 只含 X；一切协商 ordering-only 且退化路径 = 同一编译器
的默认套餐（不变量 7）；约束树保底枚举；oracle 独裁；终点不读
分配。错误协商至多变慢。

### 8.3 输出修补（不变）

精确剪圈 + grounded 投影桥（1 最短/2 精确 A*/N 投影去环）；接受
条件 = 严格更短且 SOC 不增；计入 pass deadline（超时返 raw、pass
诚实超时）；最终整体重放兜底。

---

## 9. 生产流程与自动机制 `[保留]`

两遍共享 10s（搜索+修补+SOC 全计入；pass 完不成修补即诚实超时）；
runner +30s 只保护进程启动与 IO。自动机制表沿 v4.0（macro ≤64 且
首解前、rollout 8 步刷新、futile-lift 3 次/max(64,8V) 窗、
active cap、path-local 失效、优先级 pass 冻结）。环境输入仅
`DD_ALPHA..DELTA + DD_DEBUG_DUMP`；OBJ_* 是结构常量非开关。

**deliverable_ms（裁决 6c）**：binary 在**最终重放/SOC 选择完成后**
以单调时钟打点输出 `deliverable_ms`；runner 落列并对成功行机器可查
地断言 `deliverable_ms <= 10000`。deadline 后的大树析构（实测最长
~1.6s）不再靠文字解释——可交付时刻有数为证。

---

## 10. 工程落点

| 文件 | 现状职责 | v4.1 新增 |
|---|---|---|
| `carrier_guidance.hpp` | solve_tau、链编译、emit_task、rho、custody、执行价 | tentative/默认套餐、账本、Phase R（推挤/FAIL/裁决 yield）、软干涉分、双键合并、压力折算、aging 计数 |
| `tapf_planner.cpp` | attach（tau->价->build）、funcPIBT、macro、stale 重锚 | attach 挂两阶段协商；PIBT 类内序吃 task_priority；per-target no_progress 传播 |
| `tapf_planner.hpp` | Task/Guidance/统计 | roots 集合、双键、ObjectiveOption、协商诊断 |
| `dd_planner.cpp` | 两遍、B0/B1、探针 | deliverable_ms 打点；协商探针 |
| `dd_plan_repair.cpp` / `dd_carrier.cpp` / validator | 修补 / oracle / 独立重放 | 不变 |
| `tools/dd_benchmark.cpp` + `run_benchmark.py` | 诊断输出/落列 | `obj_default_resolutions / obj_reselect_requests / obj_inherit_depth_max / obj_backtracks / obj_yields / tasks_merged / deliverable_ms` |

---

## 11. 实验规范与基线 `[保留 + 治理修订]`

### 11.1 协议

同前（实例/seed/10s/jobs14/独立 validator/权重纪律/provenance/
plan_sha/防覆盖/分层与逐例披露）。**新增**：成功行以
`deliverable_ms <= 10000` 机器断言严格 10s。

### 11.2 消融（结构变体）

```text
A. 强制默认解析（obj_default_resolutions=100%，即零协商）vs 完整协商
B. 无继承（冲突即自适应）vs 完整继承链
C. roots 合并 vs S3 式丢弃第二 root
```

### 11.3 基线阶梯（口径分栏）

```text
—— 宽松 wall 时代 ——  fix1_mscd 36/68·214,199 -> rootfix 36/68·35,595
                       -> task_commit(v2.2 基准) 36/68·34,860
                       -> v3 step3 36/68·0.9084
—— 严格 10s 时代 ——   r1_strict 35/68 -> review2_final 35/68·0.9369
                       -> round3_final(head) 34/68·0.9256/0.9341
                          (common-34, 18/7/9；丢例已二分归因披露)
```

### 11.4 v4.1 验收 gate（**36->34 变更已获追溯性独立批准**；今后
gate 期望变更须**事前**独立审查）

1. success ≥ **34/68** 且小图 ≥ **34/36**；若协商/合并恢复已丢失的
   `h10w10_e3_R1_seed1`（贴线）或 `h10w10_e8_R1_seed0`（S3 池收缩）
   则如实报升并单列说明；
2. 共同成功集 mk/SOC 几何比 + 逐例三分 + 恶化逐例解释；锚例单列
   （`h4w10_e1_R1_s0` 现 880、`h6w10_e1_R1_s0` 现 933）；
3. 零 shelf 逐位一致；singleton 字节稳定不作要求；
4. 消融 A/B/C 同协议；
5. `obj_default_resolutions` 比率与 `guidance_time_ms` 专项报告
   （默认解析占比过高 = 协商未实际生效，视为未落地）；
6. 全部成功计划过 oracle 重放 + Python validator +
   `deliverable_ms` 断言。

---

## 12. 测试计划与落地记录

### 12.1 v4.1 必测（RED 先行；措辞已按裁决修正，创建即受保护）

| # | 要求 |
|---|---|
| 1 | 零 shelf 逐位一致（既有） |
| 2 | singleton 退化：协商只余路径/落点维度 |
| 3 | **修复通道**：高优的残余干涉压力经 §4.2 使低优改选替代目标格（不经 claims 直抢） |
| 4 | 对称规则：低优认领不驱逐高优；冲突时低优自适应 |
| 5 | 继承链 A→B→C 两级传导（Phase T 后账本可见性） |
| 6 | 回退：B 路线/落点替代耗尽时**返回 FAIL**，A 换次优套餐；双方耗尽后裁决器让低有效优先级方 yield |
| 7 | 合并：(shelf,from) 桶 roots 并集、priority=max、committed 覆盖 advisory、异落点 committed = 真冲突 |
| 8 | in-flight committed 豁免：执行中的承诺落点不被任何优先级重协商 |
| 9 | one-empty：协议输出 ≡ 现行 ready 编译（典型 fixture） |
| 10 | option/claims/优先级不进 h（同型扩展） |
| 11 | 防震荡：对称盘面跨节点不翻烙饼（pass 冻结 + option 粘滞） |
| 12 | aging：per-target no_progress 达阈插队并推进；progress/改选即清零 |
| 13 | 默认解析：预算超限行为 ≡ 零协商默认套餐（同管道，非旧代码调用） |
| 14 | 池宽度：选中套餐发射完整链（≤CLEAR_CHAIN_K），非单任务 |
| 15 | 身份稳定：advisory→committed 细化不改变 (shelf,from) 物理身份（粘滞/custody 连续） |
| 16 | deliverable_ms：成功行 ≤ 10000（机器断言） |
| 17 | custody/committed 生命周期（既有 S1/R2 测试） |
| 18 | 全部返回计划过 C++/Python 重放（既有） |

### 12.2 已落地记录（审计保留）

v3.0 step1-3（消融阶梯 B=1.0070 / frontier=0.9237 / A~C=0.9745，
锚例 1053→417 宽松口径）；R1-R7（严格 10s、执行侧半环、全链
stale、parser 统一、β 计价、诊断列、provenance、文档 CI）；S1-S3
（labeled ready 活锁、任意松弛 stale、池去重 + depth fixture 两轮
审查替换）。细节见 `debug.md` §10 索引与对应 commit。

---

## 13. 边界与开放问题

1. 协商成本：OBJ_* 硬限 + `guidance_time_ms` 监控 +
   `obj_default_resolutions` 双角色（安全阀 & 落地成色试金石）；
   大图 guidance 已 ~2.5s/10s 量级。
2. one-empty 物理串行化不因协商改变（深链补发 §3.3 保池宽、预测性
   站位为后续项）。
3. 大图首解 horizon（20×20+ 0/32）非协商所解（层级搜索/等价约减为
   研究项）。
4. `targets > 256` row-wise 非单射 regime 无 gate 覆盖。
5. 多机器人修补桥仅合法缩短（Python 原型 719 vs 1053 宽松口径）。
6. 已知质量遗留：e8 并行度节流、b4 目标往返残余、pass2 析构包络
   （deliverable_ms 使其可机检）——v4.1 验收专项检查前两项。
