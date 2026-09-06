# Carrier-LaCAM 机器人—任务全局匹配与增量 Hungarian 方案报告

日期：2026-09-06；审计基线：`80148a7`；当前 production：`64a3941`；
问题实例：`brap_h10w10_a12_e8_R1_seed1`；状态：审计、分阶段实验与
quick 验证已完成，full 509 待独立终审

## 1. 结论摘要

可视化中机器人搬完一个货架后，有时会越过附近可执行任务，跑到远处搬另一个看似无关的货架。对当前代码和该实例的最终计划逐状态重放后，可以确认：

1. **该实例不是因为同一个货架同时存在多个临时放置点而产生歧义。**在最终 1844 拍路径的全部状态中，任务图和 grounded-ready 集合都没有出现“同一货架对应多个任务”的状态，最大值始终是 1。
2. **真正的直接原因是任务在进入 Hungarian 之前，先按 priority 做了硬过滤。**当有 \(F\) 台空闲机器人时，代码只保留 priority 不低于第 \(F\) 名的任务；附近但 priority 较低的任务根本不会进入 cost matrix。
3. 当前所谓的“全局匹配”只是在**过滤后的少量任务**之间全局匹配，并不是在全部当前可执行任务上综合比较 priority、距离、关键路径长度和切换代价。
4. priority 当前主要承担“准入/淘汰”作用。一旦任务进入矩阵，二级 Hungarian 代价里没有 priority，只有距离和一个很弱的切换惩罚。距离多 1 格通常比换任务贵，因此机器人在 Lift 之前可能频繁改派。
5. Carrier 的 \(\rho\) 匹配目前每次重建矩阵并重新求解，没有保存 matching 和 dual potentials。仓库里其实已经有两套可借鉴实现：ITA-CBS 的单行增量 Hungarian，以及 LaCAM-TAPF 按 changed agents 修复行的 `TAPFAssignmentState`。
6. 上述证据只能证明“附近任务没有参加比较”，不能证明“选择附近任务一定缩短最终 makespan”。远处 blocker 仍可能位于更关键的因果链上；修复目标是让普通可行任务获得有限权衡机会，而不是强制机器人永远选择最近任务。

因此，本报告保留用户提出的主方向，但把调度语义与求解加速拆成两个独立问题：

> **候选与成本模型负责决定“该派谁做什么”；增量 Hungarian 只能更快地求出同一个、已经冻结且可验证的决定。**

实施时先取消普通任务的 priority cutoff，并尽量保留现有 bottleneck-then-sum、dummy、continuity 和 canonical tie 语义，单独测量“扩大候选边界”的效果。`服务成本－延期成本` 可以作为后续独立的 full-solve 调度实验，但它是新的 additive surrogate，不与原 bottleneck 等价，也不能因为方便使用 Hungarian 就默认更符合 makespan 优先目标。

同样，不把 maximum-cardinality 设成 makespan 调度的硬主目标。给机器人分配一个 approach/PREPARE 任务不等于这一拍完成了有效工作；等待、保持可用或准备即将释放的关键任务都必须保留为合法选择。

增量复用的收益仍是待验证假设。Carrier 的一次 joint transition 可能移动多台机器人；Lift、Drop、custody、mode、任务延期代价和 continuity anchor 也可能改变一行或整列成本。第一版只有在完整列模型版本不变时才允许 row repair，否则安全退回 full solve。

## 2. 数据来源与口径

本报告只把能够从当前代码、正式 benchmark CSV 或同一二进制的确定性计划重放中核验的内容写成事实。

正式结果来自：

- `benchmark/results_full_two_pass_reference_20260906/rows.csv`
- 可视化：`benchmark/viz_web/full_benchmark_two_pass_20260906/cases/brap_h10w10_a12_e8_R1_seed1.html`
- 正式计划 SHA-256：`561631698435e1372d10c593b9b1df8072e73c8bf9e4989880d19318de19c8a9`
- benchmark 二进制 SHA-256：`bc08b69ab26ad026e890420b59cb58dadb58daa0b55edc2319ccecc16a57598a`

为取得 CSV 中没有记录的 repair 和逐状态 \(\rho\) 诊断，本报告还使用相同 SHA 的 `build/dd_benchmark` 重跑该单例。重跑得到的最终计划 SHA 与正式结果完全一致。运行时间和搜索计数受机器调度及 deadline 影响会有小幅波动，因此：

- 公开可比较的首解时间、最终时间和正式搜索计数采用正式 CSV；
- repair 次数和逐状态任务选择采用相同计划 SHA 的重放诊断；
- 不把单次重跑的毫秒差异解释成算法变化。

## 3. 用户看到的问题是什么

用户的预期流程是：

```text
PairCost / tau 决定目标货架最终去哪里
    ↓
Task-BR 类似货架侧 LaCAM，生成当前明确可执行的货架 move
    ↓
在机器人和这些明确任务之间做一次全局匹配
    ↓
机器人执行 Move / Lift / Drop
```

在这个理解下，如果一台机器人刚在某个区域放下货架，而附近仍有若干可执行货架任务，那么“继续处理附近任务”至少应该作为全局匹配中的低距离方案参与比较。只有当远处任务更紧急、位于更关键的因果链上，或者附近任务会阻塞其他机器人时，算法才应有理由选择远处任务。

当前实现破坏了这一点：它不是先建立“所有可执行任务 × 所有空闲机器人”的矩阵，而是先按 priority 把大部分任务删掉，再在剩余任务中求匹配。于是可视化中看起来像机器人“无视附近工作”，实际上是附近任务根本没有获得与远处任务比较的机会。

## 4. 当前实际机制

### 4.1 priority 从哪里来

`lacam/src/carrier_guidance.hpp:2524-2549` 中的 `target_priorities_from_pair_cost()`：

1. 读取当前 \(\tau\) 为每个目标货架选中的 `PairPlan`；
2. 按 `estimated_cost` 从大到小排序；
3. 为 \(N\) 个目标货架赋予 \(N,N-1,\ldots,1\) 的整数 rank。

因此当前 priority 首先是**排序名次**，不是“多延迟一拍会增加多少真实 makespan/work”的物理量。`carrier_guidance.hpp:5065-5087` 还会把 priority commitment 中的目标提升到普通最大 rank 之上，所以 12 个目标货架的实例中可以出现 priority 13。

Task-BR 编译清障任务时，会把目标根的 priority 传播给相关 blocker/transfer task。一个服务多个 roots 的任务可以继承较高的紧迫度。

### 4.2 ready task 先经过一次顺序过滤

`ready_tasks_with_custody()` 位于 `carrier_guidance.hpp:4057-4129`。它先检查：

- 依赖是否完成；
- 放置点是否可用；
- 货架是否 grounded，或是否属于允许继续执行的 carrier；
- 是否已经被 custody 占用。

这些属于合理的物理或因果可行性检查。

但随后它按 priority 从高到低排序，并按这个顺序建立 transfer claims。若另一个任务与已有 claim 冲突，后来的任务直接被删除。冲突本身可能是物理约束，但“冲突任务中由谁胜出”目前仍由 priority 的先后顺序决定，而不是由全局机器人—任务成本决定。

### 4.3 EXECUTE 和 PREPARE 分两次匹配

`carrier_guidance.hpp:5354-5377` 先对 grounded executable tasks 调用一次 `match_ready_tasks()`；已经获得 EXECUTE 任务的机器人被标记为不可用于 preparation，然后剩余机器人再对 PREPARE tasks 调用第二次相同函数。

因此当前还有一个硬顺序：

```text
先尽量分 EXECUTE
    ↓
剩余机器人再分 PREPARE
```

这未必错误，但它不是一个同时比较 EXECUTE 与 PREPARE 收益的统一全局矩阵。后续设计必须明确这是希望保留的策略，还是仅仅是当前实现方便。

### 4.4 `match_ready_tasks()` 如何过滤候选

核心代码位于 `carrier_guidance.hpp:4701-4948`：

1. 只收集 `KAPPA_FREE` 且 eligible 的机器人；
2. 对相同 `TransferKey` 去重；
3. 若同一货架仍有多个候选，只保留 priority 更高者；
4. 按 priority 降序排列；
5. 设空闲机器人数量为 \(F\)。若候选数大于 \(F\)，取第 \(F\) 名任务的 priority 为 cutoff，删除所有 priority 低于 cutoff 的任务；
6. priority 高于 cutoff 的候选被标记为 mandatory；与 cutoff 同 priority 的候选可以通过 dummy column 延期；
7. 只对上述幸存者建立 assignment matrix。

简化后的现有逻辑是：

```text
all ready tasks
    ↓
TransferKey / shelf 去重
    ↓
按 priority 排序
    ↓
只保留 top-F priority 层
    ↓
Hungarian
```

所以这里的 Hungarian 是“幸存任务内的全局匹配”，不是“全部 ready tasks 的全局匹配”。

### 4.5 当前矩阵和目标函数

当前矩阵方向是：

```text
行：过滤后的 task
列：free robot + dummy
```

对真实机器人列，代码计算：

\[
\operatorname{completion}(m,r)
=d(q_r,\operatorname{pickup}_m)
+\operatorname{service}(m)
+\operatorname{criticalTail}(m)
\]

然后先用 `bottleneck_then_sum_assignment()` 最小化最大 completion。超过最优 bottleneck 的边被置为不可用，再最小化：

\[
\operatorname{cost}(m,r)
=d(q_r,\operatorname{pickup}_m)\cdot(F+1)
+\mathbf 1[\text{switch}]
\]

这里有两个重要事实：

- **priority 没有进入这个二级 cost。**它在此之前已经通过删除任务和 mandatory/dummy 规则发挥作用。
- switch penalty 只有 1，而距离增加 1 格的代价是 \(F+1\)。当 \(F=1\) 时，一格距离值 2；当 \(F=2\) 时，一格距离值 3。因此“继续上一任务”的粘性弱于一格距离差，无法形成稳定承诺。

为了得到确定性 tie-breaking，`carrier_guidance.hpp:4865-4945` 还会在固定每个真实机器人列时反复调用 Hungarian。当前实现没有保存 matching 或 dual potentials，每次 guidance build 都从头构造并求解。

### 4.6 \(\rho\) 在 Lift 前不是承诺

`CarrierGuidance` 只保存当前节点的 `rho_task_id`、`rho_transfer_key` 和 mode；结构中没有 assignment state，见 `lacam/include/tapf_planner.hpp:561-574`。

机器人真正 Lift 后由 custody 约束继续搬运。在 Lift 之前，\(\rho\) 只是当前物理状态下重新计算的 dispatch guidance。机器人移动一格后，新节点会重新构造 guidance；如果候选、距离或上一份 \(\rho\) 改变，机器人可以被重新指向另一个任务。

这解释了动画中的两类现象：

- 机器人主动跨区域奔向高 priority 任务；
- 机器人还没 Lift，就在途中被改派到另一个任务。

## 5. 单实例的准确证据

### 5.1 正式 benchmark 结果

`brap_h10w10_a12_e8_R1_seed1` 的正式两遍结果为：

| 指标 | 数值 |
| --- | ---: |
| solved | 1 |
| 首解时间 | 1188 ms |
| 首解 makespan | 2659 |
| 首解 weighted work | 5691 |
| 最终 makespan | 1844 |
| 最终 weighted work | 3927 |
| loaded moves | 556 |
| free moves | 1871 |
| Lift/Drop | 1090 |
| shelf switches | 468 |
| robot utilization | 0.1508 |
| 第二遍候选 | 0 |
| 第二遍严格改进 | 0 |
| 第二遍 generator failures | 4782 |
| 第二遍退出原因 | `SEARCH_CUTOFF` |
| reference checkpoint hits | 64 |
| reference action hints | 192 |
| upper epoch builds | 1268 |
| PairCost cache hits / misses | 6242 / 1268 |
| 累计 ready task count | 128690 |
| 累计 `rho_task_id` changes（CSV 名为 `rho_repairs`） | 34960 |
| owner handoffs | 7887 |
| rewire guidance rebuilds | 9216 |
| guidance time | 3976.05 ms |
| timed transport time | 854.806 ms |
| deliverable time | 9030.13 ms |
| solver runtime | 9030.24 ms |

这里的 `robot_utilization` 不是“机器人非空闲时间占比”。`benchmark/ddbench/validator.py:314-317` 的正式定义是：

\[
\operatorname{robotUtilization}
=\frac{\operatorname{loadedMoves}}
       {R\cdot\operatorname{traceMakespan}}.
\]

因此本例的 \(0.1508\) 只表示 carrying move 占全部 robot-time slots 的约 15.08%。验证器把每个非 Wait 动作恰好计入 loaded move、free move 或 Lift/Drop 之一，所以该交付前缀的非 Wait 动作占比为：

\[
\frac{556+1871+1090}{2\cdot1844}
=95.36\%.
\]

这说明本例不是“两台机器人经常没有被派任务”，而是机器人绝大多数拍都在行动，其中相当多时间花在 free move 和 Lift/Drop 上。它也直接否定了“为了提高利用率，必须把 maximum-cardinality 设成匹配第一目标”的推理。

第二遍没有产生新候选，所以最终 1844 拍并不是第二遍找到的严格改进。使用同一二进制重跑并输出同一计划 SHA 后，repair 诊断为：

| repair 指标 | 数值 |
| --- | ---: |
| exact loops | 0 |
| projected loops | 15 |
| replacement bridge steps | 88 |
| removed plan steps | 815 |

恰好有 \(2659-815=1844\)。因此最终动画包含对首轮原始计划的 projection repair；这会改变最终展示路径，不能把动画中的每个转向都直接当作搜索树原始路径。下面对 1844 拍交付计划重新构造 guidance，只能说明“当前代码若处于这些交付状态，会怎样计算派工”，不能还原原搜索树当时的 parent guidance、forced operators 或真实决策过程。

首解到最终交付的 makespan 下降为：

\[
\frac{2659-1844}{2659}=30.6506\%
\]

weighted work 下降为：

\[
\frac{5691-3927}{5691}=30.9963\%
\]

### 5.2 不是“每个货架有多个临时终点”

对最终计划的全部 1844 个 transition 重放，并在每个状态重新构建 Task-BR guidance，结果是：

```text
graph_states_with_duplicate_shelf_tasks = 0
ready_states_with_duplicate_shelf_tasks = 0
max_graph_tasks_per_shelf = 1
max_grounded_ready_tasks_per_shelf = 1
```

所以对此实例，用户的理解是对的：在一个给定状态中，每个货架有一个明确的当前任务。远距离改派不能归因于“同一货架临时放置点不确定”。

### 5.3 状态 724：距离 2 的任务甚至进不了矩阵

在 `state_t=724`：

- `R0` 位于 `(0,0)`，正在 carrying，因此不参加 free-robot matching；
- `R1` 位于 `(7,6)`，是唯一 eligible free robot；
- 当前有 6 个 grounded ready tasks。

| 任务 | priority | R1 到 pickup 距离 | 是否被当前代码保留 |
| --- | ---: | ---: | --- |
| `anon@20 (2,0)->(1,0)` | 13 | 11 | 是 |
| `anon@48 (4,8)->(4,9)` | 11 | 5 | 否 |
| `b5 (5,0)->(6,0)` | 10 | 8 | 否 |
| `anon@67 (6,7)->(6,6)` | 8 | 2 | 否 |
| `anon@68 (6,8)->(7,8)` | 7 | 3 | 否 |
| `anon@84 (8,4)->(9,4)` | 5 | 3 | 否 |

因为 \(F=1\)，cutoff 就是最高 priority 13。R1 被指向距离 11 的 `anon@20`；距离 2 的 `anon@67` 在 Hungarian 建矩阵前已经被删除。

下一拍 R0 完成 Drop、重新成为 free robot，\(F\) 从 1 变成 2。候选准入边界立刻变化：R0 接手 priority 13 的任务，R1 又转去 priority 11 的任务。这说明 free-robot 数量变化也会造成明显的 dispatch 跳变。

### 5.4 状态 1108：两台机器人时仍越过距离 1 的任务

在 `state_t=1108`，两台机器人都 free：

- `R0=(6,9)`
- `R1=(1,5)`
- grounded ready tasks 仍为 6 个。

| 任务 | priority | R0 距离 | R1 距离 | 当前分配 |
| --- | ---: | ---: | ---: | --- |
| `anon@8 (0,8)->(1,8)` | 12 | 7 | 4 | R1 |
| `b1 (5,0)->(4,0)` | 10 | 10 | 9 | R0 |
| `anon@58 (5,8)->(5,9)` | 9 | 2 | 7 | 被过滤 |
| `anon@75 (7,5)->(6,5)` | 8 | 5 | 6 | 被过滤 |
| `anon@86 (8,6)->(8,7)` | 7 | 5 | 8 | 被过滤 |
| `anon@79 (7,9)->(7,8)` | 6 | 1 | 10 | 被过滤 |

因为 \(F=2\)，cutoff 是第二名的 priority 10。最终 R0 被派往距离 10 的 `b1`，而距离 1 和距离 2 的任务根本没有进入矩阵。

这就是动画中“明明附近有货架，机器人却跑到远处”的直接、可复现代码原因。它不是 Hungarian 在完整矩阵上权衡后认为远处更好，而是完整矩阵从未建立。

### 5.5 交付路径重建 guidance 中的 retarget 数量

对最终 1844 拍计划重放：

| 指标 | R0 | R1 | 合计 |
| --- | ---: | ---: | ---: |
| 任意相邻状态的 \(\rho\) 变化 | 711 | 687 | 1398 |
| 连续两状态均 free 且 non-null task 改变 | 160 | 150 | 310 |

第一行包含 Lift/Drop 附近自然出现的 task→IDLE 或 IDLE→task，不能全部视为坏改派。第二行更有解释力：机器人连续两个状态都 free，却从一个非空任务切换到另一个非空任务，共发生 310 次。

这 310 次应准确命名为“交付路径重建 guidance 中的 free→free non-null retarget”。它们不是原搜索实际错误改派次数，也不意味着这些切换都应该被禁止；全局派工本来就可能因其他机器人、依赖和拥堵而改变。该指标适合用于比较同一组交付状态下不同 guidance 的稳定性，不能单独作为最终 makespan 改善的代理。

## 6. 全 benchmark 的规模

正式 full benchmark 共 509 个实例，其中 479 个 solved。下列统计只聚合这 479 个 solved rows：

| 搜索期计数 | 总和 | 均值 | 中位数 | P95 | 最大值 | 大于 0 的实例 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `ready_task_count` | 13,625,227 | 28,445.15 | 7,010 | 139,015 | 317,975 | 478 |
| `rho_repairs` | 3,917,608 | 8,178.72 | 2,473 | 35,659 | 81,842 | 478 |
| `owner_handoffs` | 961,068 | 2,006.41 | 23 | 11,799 | 26,056 | 357 |
| `upper_epoch_builds` | 387,854 | 809.72 | 143 | 4,266 | 10,497 | 479 |
| `pair_cache_hits` | 5,208,456 | 10,873.60 | 9,713 | 31,185 | 53,689 | 478 |
| `rewire_guidance_rebuilds` | 420,994 | 878.90 | 0 | 7,696 | 27,080 | 149 |

`guidance_time_ms` 在 479 个 solved rows 上合计为 1,697,780.314 ms，即 1697.780 秒；单例均值 3544.43 ms，中位数 3223.60 ms，P95 为 8137.74 ms，最大值 8385.77 ms。

第二遍退出分布为：

```text
SEARCH_CUTOFF             367
STRICT_IMPROVEMENT         79
REFERENCE_SUFFIX_ACCEPTED  32
SEARCH_EXHAUSTED            1
```

这些计数必须正确解释：

- `rho_repairs` 这个字段名容易误导。当前代码并没有执行增量 Hungarian repair；它统计的是相邻 guidance 中每个机器人的 `rho_task_id` 是否变化，见 `lacam/src/tapf_planner.cpp:364-369`。
- `owner_handoffs` 统计同一个 `TransferKey` 的 owner 是否从一台机器人换到另一台，见 `tapf_planner.cpp:370-387`。
- 这些都是搜索树范围内的累计计数，不是最终交付路径上的动作数量。
- `guidance_time_ms` 包含完整 Carrier guidance 构建，不只包含 Hungarian，因此不能据此宣称“所有时间都耗在匹配上”。实现前必须增加独立的 matching timer。

对目标单例的同 SHA 重跑中：

```text
guidance_builds = 45,886
upper_epoch_builds = 1,268
```

只有 2.763% 的 guidance builds 新建了 upper epoch，说明静态的 PairCost、\(\tau\) 和 task graph 已经大量复用；但这**不能**直接证明 97.237% 的匹配矩阵都完全相同。机器人位置、free/carry 状态、ready/prepare 集合、列数值和上一份 assignment 仍可能变化。我们需要专门记录完整 `ColumnModelVersion` 与 `RowFingerprint` 才能测出真实增量复用率。

## 7. 根因归纳

### 7.1 priority 被实现成硬准入，而不是有限权重

当前语义近似于：

> 只要 priority 排名更高，即使远 10 格，也先排除近 1 格的低 priority 任务。

这是一种近似无限权重。它不能表达“高优先级值得多走 2 格，但不值得多走 20 格”这样的连续权衡。

### 7.2 priority 过滤发生在全局优化之前

一旦附近任务被删除，后面的 bottleneck、距离、critical tail 和 switch cost 都无法挽回它。因而问题不是 Hungarian 算错，而是 Hungarian 收到的问题已经被提前裁剪。

### 7.3 当前粘性只够做 tie-break，不够做承诺

切换任务只增加 1，而距离一格增加 \(F+1\)。这让算法可以为了很小的距离变化频繁切换任务。若希望机器人在没有明显收益时继续当前区域或当前 pickup，应把 continuity 作为可标定的有限成本，而不是 1 个最低位。

### 7.4 当前矩阵方向不利于复用“机器人移动导致的局部变化”

现在 task 是行、robot 是列。机器人位置变化会改变该机器人对应的整列。ITA-CBS 和现有 `TAPFAssignmentState::repair_rows()` 都是按行更新设计的，直接套用会更新错方向。

若阶段 2 采用 additive 目标，并希望按机器人局部更新，最自然的矩阵方向是：

```text
行：robot
列：task + idle/locked slots
```

这样机器人 \(r\) 的位置、eligibility 或 continuity reference 改变时，主要改变第 \(r\) 行。

### 7.5 当前 \(\rho\) 没有节点局部的 matching state

`UpperEpochGuidance` 可以跨 robot-only transition 共享静态 task graph，但 `CarrierGuidance` 没有 `mateL/mateR/lx/ly`。所以即使相邻节点只有一台机器人移动，当前代码仍重新建矩阵、重新求 bottleneck、重新多次运行 Hungarian。

## 8. 哪些约束应当硬保留，哪些应当进入 cost

不能把“删除 priority cutoff”误解成把所有任务无条件塞进矩阵。更准确的说法是：

> **所有在当前 dispatch mode 下物理或因果可行的候选，都不应仅因 priority 较低而被删除。**

### 8.1 EXECUTE 的硬约束

- 当前任务依赖必须已经满足；
- 货架必须位于预期 pickup，或存在合法 custody continuation；
- 机器人必须 free，或因 carrying/custody 只能继续特定 transfer；
- robot 到 pickup 在静态墙图上可达；
- 当前 transfer 的 Lift、loaded Move 和最终 Drop 必须有合法物理解释；
- 同一实体货架不能同时获得两个 executor；
- 当前不能同时执行的 endpoint/transfer conflict 必须由显式互斥约束处理。

“endpoint 当前被占据”不能脱离 mode 简单理解成该任务永久不可候选。它可能禁止现在 Drop，却不一定禁止机器人提前接近 pickup。

### 8.2 PREPARE 的硬约束

当前代码的 PREPARE 只允许在因果前驱尚未完成、但这些前驱已经有 executor 时，为后继任务提前 approach。机器人到达货架后只能 Wait，不能 Lift，见 `lacam/src/tapf_planner.cpp:1742-1757`。

因此：

- 前驱未完成可以禁止 EXECUTE，但不一定禁止 PREPARE；
- endpoint 尚未 vacate 可以禁止当前 Drop，但不一定禁止 approach；
- PREPARE 不能被记成 transfer 已经完成服务；
- 同一 transfer 的 EXECUTE/PREPARE 不能同时获得两个 owner；
- PREPARE 的收益只能对应缩短未来 approach 等准备时间，不能直接抵扣全部任务延期损失。

### 8.3 应作为有限代价或显式调度层级的因素

- target/task priority 或 urgency；
- pickup 距离；
- 当前 service/resource occupancy；
- 推迟任务对未完成 roots 的预计影响；
- 是否切换上一任务；
- EXECUTE 与 PREPARE 的偏好；
- 按真实 transition 时间定义的等待年龄；
- 区域连续性。

这些量应参与比较，而不应在比较前把普通可行任务删除。尤其需要把“当前执行成本”和“推迟损失”分开：后续 critical tail 不是当前机器人立即占用资源的时间，不能未经定义就以正号同时放进服务成本和延期成本。

## 9. 可选解决方案比较

| 方案 | 内容 | 优点 | 主要问题 |
| --- | --- | --- | --- |
| A. 候选边界实验 | 只删除 `match_ready_tasks()` 的普通 priority cutoff；尽量保留当前 task-row、dummy、bottleneck、continuity 和 canonical tie | 唯一主要变量是“低 priority 普通任务能否参加比较”，最适合验证根因 | 上游 transfer-claim、同 shelf 和 EXECUTE/PREPARE 仍可能预选；机器人移动改变列；矩阵会变大 |
| B. 保持当前矩阵，做增量列修复 | 保存 matching/duals，支持 changed robot columns | 最大程度保留现有 bottleneck 语义 | 现有增量实现是 row repair；动态列和反复 lex refinement 都要重写，复杂度高 |
| C. robot-row additive full solver | priority/urgency 进入有限延期成本，机器人为行，任务和 idle 为列；暂不增量 | 可以独立验证新的 `S-D` surrogate、idle 和 mode 语义 | 这是调度目标变化，不与 bottleneck 等价；不能默认改善 makespan |
| D. 对 C 做 node-local incremental Hungarian | 在 C 的数学问题完全冻结后，节点保存 matching/duals 并修复 changed rows | 能隔离测量增量求解的纯性能收益 | 需要完整成本版本、anchor、矩形负成本、canonical tie 和 full fallback 契约 |
| E. min-cost flow / ILP / 冲突感知匹配 | 在一个模型中表达 robot、task、endpoint、shelf 和 mode 约束 | 表达能力最完整 | 每个 LaCAM 节点求解成本高，实现和增量维护复杂 |
| F. 单独增加区域 lease/强承诺 | 机器人一定拍数内不改派，或优先同一区域 | 容易降低动画抖动 | 可能掩盖错误的候选过滤，并妨碍真正有价值的全局重分配 |

推荐顺序是 **先 A，再 C，再 D**。A 回答候选边界是否有价值；C 回答新的调度 surrogate 是否改善真实 `(T,W)`；D 只回答能否更快地精确求解 C。E/F 只有在 telemetry 证明普通 Hungarian 无法表达主要冲突或稳定性需求时再考虑。

## 10. 用户建议方案：可保留的代数与必须重新验证的目标

用户建议包含两个不同层次：

1. 普通可行任务不应因 priority 较低而在匹配前消失；
2. 相邻节点应复用 matching 和对偶势。

第一点改变候选与调度模型，第二点只改变求解方法。两者必须分开验证。

### 10.1 `服务成本－延期成本` 的代数成立

设当前显式候选任务集合为 \(M\)，机器人集合为 \(R\)。对 EXECUTE mode，可以定义机器人 \(r\) 现在启动任务 \(m\) 的即时服务成本：

\[
S^{E}_{rm}
=w_d d(q_r,\operatorname{pickup}_m)
+w_s\operatorname{immediateService}(m)
+w_c\mathbf 1[\operatorname{anchor}(r)\ne m]
+w_{\text{mode}}\operatorname{modePenalty}(r,m).
\]

这里的 `immediateService` 只描述当前 approach、必要准备、Lift 和当前 transfer 对资源的占用。后续 successor chain 的 `criticalTail` 不是当前机器人现在执行该动作本身的耗时，不应未经定义就以正号加入 \(S^E_{rm}\)。

再定义任务本轮未启动时的有限延期损失：

\[
D_m
=\lambda_p\operatorname{urgency}(m)
+\lambda_a\operatorname{transitionAge}(m)
+\lambda_k\operatorname{rootDelayImpact}(m).
\]

则 additive 实验模型可以写成：

\[
\min
\left(
\sum_{r,m}x_{rm}S^{E}_{rm}
+\sum_m y_mD_m
\right),
\qquad
\sum_mx_{rm}\le1,\quad
\sum_rx_{rm}+y_m=1.
\]

对固定候选集合，\(\sum_mD_m\) 是常数，因此等价的 robot→task 边成本为：

\[
\boxed{C_{rm}=S^{E}_{rm}-D_m}.
\]

这项代数是正确的：priority/urgency 表示“推迟该任务会损失多少”，而不是给任务整行统一加一个常数。

### 10.2 additive `S-D` 不是 makespan 模型

从 bottleneck 改成 additive min-sum 会改变派工目标。即使所有任务都会被服务，减去 \(D_m\) 也只是对完整匹配减去同一个常数，不能消除 min-sum 与 min-max 的区别。

例如：

\[
E=
\begin{array}{c|cc}
 &m_1&m_2\\ \hline
r_1&1&6\\
r_2&6&9
\end{array}
\]

匹配 \((r_1,m_1),(r_2,m_2)\) 的总和为 10、最晚完成为 9；交叉匹配的总和为 12、最晚完成为 6。min-sum 与 makespan surrogate 会选择不同结果。

所以 `S-D` 只能被称为新的 dispatch guidance 候选。它是否更利于最终 `(T,W)` 必须通过独立 full-solve benchmark 验证，不能因为增量 Hungarian 更容易实现就默认采用。

### 10.3 为什么“给任务行统一加 priority”没有用

若继续使用 task-row 矩阵，并把同一个 priority 常数 \(p_m\) 同时加到任务 \(m\) 的所有真人列和 dummy 列，那么每个完整匹配都会支付：

\[
\sum_m p_m.
\]

这是与 assignment 无关的常数，匹配结果不会改变。priority 必须表现为“服务该任务的有限奖励”或“延期该任务的有限损失”，而不是一整行所有列的共同偏移。

### 10.4 不使用 maximum-cardinality 作为硬主目标

idle 必须是合法调度选择。一个非空 \(\rho\) 只表示机器人开始 approach 或 PREPARE，并不表示这一拍完成了有益任务。强制先最大化非空 assignment 数可能：

- 把即将需要执行关键后继的机器人送往远处；
- 过早 Lift，降低后续清障灵活性；
- 增加通道竞争和 free moves；
- 在不降低 makespan 的情况下增加工作量。

因此 additive 实验直接比较 task edge 与 idle edge 的完整成本，不再额外施加 maximum-cardinality 词典序。若希望测试 work-conserving 策略，应作为单独 ablation，而不是正确性的必需不变量。

### 10.5 PREPARE 只能获得部分收益

PREPARE 不执行 Lift，也不完成 transfer。若未来把 EXECUTE 和 PREPARE 放进同一个矩阵，应为 PREPARE 定义单独的准备收益，例如预计减少的未来 approach：

\[
B^{P}_{rm}\le D_m,
\]

而不是直接使用完整的 \(-D_m\)。同一 transfer 的 EXECUTE/PREPARE columns 还必须属于同一个 owner/conflict group，不能同时被两台机器人选择。

### 10.6 urgency、age 与共享 blocker

第一版 additive 实验可以使用有限 priority rank，但必须标为启发式，不能解释成真实 makespan 边际。

若引入 age：

- 只能沿真实 primitive transition 的离散时间更新；
- sibling 分支各自拥有年龄状态；
- guidance 重建、节点再次弹出或 wall-clock 不能增加 age。

共享 blocker 的延期收益应按其影响的 roots 聚合，并避免把同一条 root critical chain 的收益沿多个任务重复计算。`criticalTail`、root impact 和 priority rank 必须分别定义，不能靠权重调参决定同一个量究竟应被奖励还是惩罚。

## 11. 如何复用 ITA-CBS 和现有 LaCAM-TAPF

### 11.1 仓库中已经存在的能力

`third_party/ITA-CBS2/include/dynamic_hungarian_assignment.hpp:223-243` 的 `incrementalSolutionX()` 会：

1. 只重建一个 agent row；
2. 解除该 row 当前匹配；
3. 根据现有列势 `ly` 重新计算该 row 的行势 `lx`；
4. 从该 row 重新增广；
5. 保留其他 matching 和 dual potentials。

`third_party/ITA-CBS2/include/ITACBS/ITACBSNode.cpp:108-152` 只在某个 agent 的 constraints 改变时更新其 cost row，然后调用上述增量求解。

仓库自己的 `lacam/include/tapf_assignment.hpp:37-107` 已经把同样思想整理成 `TAPFAssignmentState`，保存：

```text
mateL
mateR
lx
ly
cost_scale
tie_hash_mod
```

`lacam/src/tapf_planner.cpp:1078-1087` 会收集所有位置变化的 agents，把父节点的 assignment state 复制到子节点，再调用 `repair_rows(changed_agents)`。

共享的 cold solver `tapf_hungarian_row_to_col()` 已明确支持 rows \(\le\) columns 的矩形矩阵和负整数成本；`tests/test_tapf_hungarian_shared.cpp` 对这两项都有测试。`TAPFAssignmentState` 通过补齐为方阵保存 matching/duals，`augment_from_row()` 的增广路径可以连带重分配其他未改变行，而不是只替换输入 changed row。

因此我们不需要从零发明增量 Hungarian，但也不能把现有类型直接当作 Carrier 新模型已经满足的实现。

### 11.2 不能直接复制的差异

ITA-CBS 中通常是一个 agent 新增 constraint，因此一行 cost 发生变化。Carrier 中还存在：

- 一个 joint transition 可能移动 0、1 或多台机器人；
- Lift/Drop 会改变 free/carry eligibility；
- ready task、PREPARE task 和 endpoint availability 可能变化；
- 上一份 `TransferKey` 作为 continuity reference 变化时，也会改变对应机器人行；
- urgency/age/root impact 等列数值可以在任务身份不变时变化；
- 一次增广可能全局更换多台机器人的 assignment；
- 当前 objective 是 bottleneck-then-sum，而标准 Hungarian state 维护的是 additive sum。

所以正确接口应是 `repair_rows(changed_rows)`，而不是写死 `repair_one_row()`。用户的“多数时候只移动一个 agent”是很有价值的优化假设，但当前代码不保证每个 successor 永远只变一台机器人；changed-row 数量分布需要新增 telemetry 后再确认。

### 11.3 现有动态状态的数值契约不足

现有 `TAPFAssignmentState` 的 public result 和 cost callback 使用 `int`，不可达 sentinel 是 `100000000`；内部用：

\[
\operatorname{weight}
=4\cdot10^{18}
-(\operatorname{primaryCost}\cdot\operatorname{costScale}
+\operatorname{tieCost}).
\]

当前距离成本范围受控，但新的固定点 `S-D` 可能为负，也可能需要 64 位。直接复用会引入以下风险：

- `primaryCost * costScale` 或 weight 变换溢出；
- 有效大成本被 sentinel 误判成不可达；
- 不可达 sentinel 在变换后优于真实边；
- `TAPFAssignmentResult.cost` 累加溢出；
- 对 task column 做错误的非负化，改变哪些未匹配列被选择。

若需要非负化，只能对某一机器人行的所有合法列——包括 idle——同时加减同一个常数。不能对一个 task column 统一平移，因为矩形匹配不保证每个 task column 都会被选中。

第一版应新建带 checked arithmetic 的 64 位状态，或在证明范围后复用模板化实现；不能只修改 callback 返回类型。

### 11.4 deterministic tie 不是现成保证

当前 \(\rho\) 在得到 bottleneck 和最小 secondary cost 后，会按机器人列顺序、任务 ID 顺序反复求剩余最优值，逐步固定一个 canonical assignment。

`TAPFAssignmentState` 的 `tie_hash()` 只给边附加有限 hash。`cost_scale` 可以保证 tie 总量不改变 primary optimum，但“每条边 tie 值不同”并不证明“每个完整 matching 的 tie 总和唯一”。cold full solve 和 warm incremental solve 在多最优解时可能得到相同 objective、不同 matching。

阶段 3 必须二选一：

1. 保留或重新实现有保证的 canonicalization，并要求 full/incremental assignment 逐位一致；
2. 只要求最优值一致，接受不同最优 matching，并把它明确视为搜索顺序变化。

为了隔离增量求解的纯性能收益，本报告推荐第一种。不能用普通 hash 替代 canonicalization 后仍声明“行为完全等价”。

### 11.5 增量求解不会自动减少 retarget

在相同成本矩阵和相同 canonical tie 下，精确 incremental solver 应输出与 full solver 相同的 \(\rho\)。因此仅保存 duals 不会减少交付路径重建 guidance 中的 310 次 retarget。

retarget 的变化来自候选集合、continuity、urgency、idle 或 mode 语义；增量 Hungarian 只减少求解开销。在同一 10 秒预算内，更快的求解器可能探索更多节点并间接得到不同计划，但这必须与“同一矩阵 assignment 是否等价”分开报告。

## 12. 推荐的数据结构与缓存边界

### 12.1 固定机器人行

若采用阶段 2 的 robot-row additive 模型，矩阵始终以全部 \(N\) 台机器人作为固定行，而不是只为当前 free robots 建行：

```text
rows = all robots
columns = explicit task slots + N idle/locked slots
```

- free robot 可以连接合法 task columns 和 idle；
- carrying/不可用 robot 只允许连接自己的 locked/continuation slot；
- Lift/Drop 只改变该机器人行的可用边，不改变行数。

这样能避免 free robot 数量每变一次就改变矩阵维度。

固定行数只解决矩阵 shape 的一部分。一次 Lift 使某个 task 对其他所有机器人失效，仍可能改变整列；PREPARE eligibility 或延期成本更新也可能改变多行。

### 12.2 可共享的静态内容

`UpperEpochGuidance` 适合共享：

- canonical `TransferKey` / task column ID；
- pickup、drop、service、critical tail；
- root 和基础 priority 元数据；
- task graph 与静态可达距离缓存；
- immutable task-universe identity。

这些数据在相同 upper layout 下可以只读共享。随 transition 改变的 age、mode、endpoint availability、custody owner 和 continuity anchor 不能放进仅按 upper layout 共享的缓存。

### 12.3 matching state 必须属于具体 LaCAM 节点

新增的 `RhoAssignmentState` 应至少保存：

```text
mateL / mateR
row potentials / column potentials
ColumnModelVersion
per-row fingerprint
anchor_used_by_row
deterministic objective value
canonicalization version
```

父节点扩展子节点时复制这份状态，再修复变化行。这与现有 TAPF assignment state 的做法一致。

不能只按 `UpperSignature` 全局缓存整份 matching。相同货架布局下，机器人位置、free/carry 状态、父节点 assignment 和 continuity reference 可能不同；全局复用会把另一条搜索分支的状态错误带入当前分支。

### 12.4 `ColumnModelVersion` 必须覆盖实际数值

仅比较 ordered task identities 不够。第一版 `ColumnModelVersion` 至少包含：

```text
ordered task / mode column identities
每列实际 service、urgency、age、root-impact 数值
全局 active / endpoint / conflict-group 版本
EXECUTE/PREPARE 语义版本
objective kind 与全部固定点缩放
INF/sentinel 版本
canonical tie 版本
```

例如机器人没有移动、任务身份也没变，但 task B 的 transition-age 增加，使 \(D_B\) 改变，正确 assignment 仍可能从 A 切到 B。如果版本不包含这一数值变化，直接复用旧 matching 就是错误。

第一版只在列结构和列数值都相同时做 row repair；任何列模型版本变化都 full solve。之后再根据 telemetry 决定是否支持少量列更新。

### 12.5 `RowFingerprint` 与两代 continuity anchor

机器人 \(r\) 的 `RowFingerprint` 至少包含：

- 当前机器人位置；
- free/carry/custody 状态；
- EXECUTE/PREPARE eligibility；
- 生成当前矩阵时实际使用的 continuity anchor；
- 其他真正进入该行成本的局部上下文。

必须区分：

```text
mate:
    当前矩阵求出的 assignment。

anchor_used:
    当前缓存矩阵计算 continuity cost 时使用的上一代 assignment。
```

父节点的 `mate` 可能由一条增广路径同时更换多台机器人。进入子节点后，这些新 mate 才成为新的 continuity anchors，因此即使某些机器人没有移动，它们的 row cost 也可能变化。不能把最新 `mate` 自动当成旧缓存已经使用过的 `anchor_used`。

若 `ColumnModelVersion` 完全相同：

- 0 个 row 改变：直接复用 assignment；
- 1 个 row 改变：一次 row repair；
- \(k\) 个 row 改变：`repair_rows(k rows)`；
- 增广路径可以自动引起其他机器人的全局重分配，不需要把它们都当作输入 changed rows。

若列数、列成本、mode、冲突组或影响所有行的任务可用性变化，第一版直接 full solve。

### 12.6 age 必须属于搜索状态语义

如果阶段 2 引入 age，它必须由真实 primitive transition 推进，并能从具体搜索节点确定。以下事件不得增加 age：

- 同一节点重复构造 guidance；
- OPEN 中再次弹出节点；
- sibling 分支被扩展；
- cache warm-up；
- wall-clock 流逝。

否则搜索顺序会反过来改变成本模型，导致同一个物理状态的 guidance 不再纯粹由其路径状态决定。若不准备把 age 纳入节点状态或可重放路径元数据，第一版就不要使用 age。

### 12.7 为什么还可能需要 column repair

即使机器人位置变化很局部，一个任务从 not-ready 变成 ready，或 endpoint 被占用/释放，会改变一整列对所有机器人是否可用。标准 row repair 不足以描述这种变化。

可采用渐进实现：

1. 第一版只在完整 `ColumnModelVersion` 相同时增量修复，否则 full solve；
2. 记录 fallback 原因和列变化数量；
3. 如果数据证明“单列变化”非常常见，再实现对称 column repair，或通过转置后的临时状态处理；
4. 不在没有数据前同时实现动态行、动态列和复杂冲突组。

### 12.8 节点复制成本也要计入性能

每个节点保存 `mateL/mateR/lx/ly` 会增加内存和复制开销。目标实例只有两台机器人，但 task columns 可能明显多于机器人；若每个 child 都深拷贝整列势，复制和 cleanup 可能抵消增广节省。

因此 telemetry 必须单独计时：

- 状态复制与分配；
- signature/fingerprint；
- cost row/column 构造；
- full solve；
- incremental augmentation；
- canonicalization。

## 13. bottleneck 目标如何处理

当前 \(\rho\) 不是简单最小和匹配，而是：

1. 最小化最大 `distance + service + critical_tail`；
2. 在最优 bottleneck 内最小化距离和切换；
3. 再进行确定性 lex refinement。

标准增量 Hungarian 只直接维护 additive objective，不能原样维护“最大值优先”。因此不能把“更容易增量化”当作改用 additive 的理由。

### 13.1 第一组实验：取消 cutoff，但保留现有 objective

候选边界实验应尽量保持：

- task-row / robot+dummy 方向；
- `completion = approach + service + criticalTail`；
- bottleneck-then-secondary；
- 当前 continuity；
- 当前 canonical refinement；
- EXECUTE/PREPARE 两阶段。

唯一主要变化是：`match_ready_tasks()` 不再按 top-\(F\) priority 删除普通候选，也不再把高于 cutoff 的任务设成无限 mandatory。若候选多于 free robots，普通任务通过 dummy 表示本轮延期。

这一步仍保留当前“有足够候选时使用所有 free robot columns”的行为，只是为了隔离 cutoff 效果，不代表本报告认定这种 cardinality 语义就是 makespan 最优。

### 13.2 第二组 bottleneck 实验：有限 priority 延期

在第一组基础上，可将 priority 只作为 dummy 的有限 defer delay：

\[
\operatorname{completion}(m,\operatorname{dummy})
=\operatorname{bestRealCompletion}(m)
+\Delta_{\text{defer}}(m),
\]

其中高 urgency 任务的 \(\Delta_{\text{defer}}\) 较大，但保持有限。这样附近低 priority 任务仍能参加比较，同时保留 min-max surrogate。

这比直接切换到 additive 更接近当前研究目标，但 `criticalTail`、priority rank 和共享 roots 的语义仍需单独 ablation。

#### 13.2.1 回归审查后的收紧方案：直接目标交付 V2

F1/F2S 实验发现，若在所有阶段统一加强 priority/continuity，两个大型回归
会因搜索顺序变化而交替超时；若完全只保留低阶 priority tie，已发布样例 A
又从 `(18,34)` 退化到 `(28,45)`。因此生产候选不是“全局提高 priority
权重”，而是一个严格限定的 bottleneck V2。

V2 只在 EXECUTE matching 且**每个候选**都满足以下条件时启用：

```text
候选搬运 TARGET b；
critical tail 为 0；
task roots 包含该货架自己的 (b, g)；
transfer endpoint 等于 g；
g 是 b 的合法 goal。
```

目标货架替其他 root 清障不满足该条件；PREPARE 也不满足。集合中只要混入
一个匿名 blocker、目标货架 blocker 或非终端因果 task，整次 matching
保持 F2S 的矩阵和 assignment 语义。这一全集合边界是有意的：它把“同质
最终交付调度”和“混合清障调度”分开，避免在后者重新以 priority 压制
blocker。

在 V2 直接交付阶段，候选仍全部进入 task-row 矩阵。对 dummy 列使用：

\[
\Delta_{\rm defer}(m)
= service(m)\left(1+I_{\rm frontier}(m)+I_{\rm continue}(m)\right).
\]

`frontier` 表示 priority 不低于当前第 \(F\) 名，`continue` 表示某台当前
free robot 的 parent assignment 仍指向该 task。它们明确改变主 bottleneck
中的 dummy completion，而不只是 secondary tie；但 multiplier 最大为 3，
不产生 INF、mandatory 或候选删除，足够大的真实 completion 改善仍可胜出。
PREPARE 不获得这份完整延期收益。

隔离临时实现的 seed-0、10 秒结果为：

| 回归 | F2S | 收紧 V2 |
|---|---:|---:|
| 发布样例 A | `(28,45)` | `(17,32)` |
| 发布样例 B | `(12,36)` | `(12,36)` |
| Testcase C | `(31,93)` | `(31,93)` |
| 发布样例 D | `(8,23)` | `(8,23)` |
| `h20w20 a40 e100 R1 seed0` | `(1273,5796)` | `(1273,5796)` |
| `h20w20 a40 e100 R1 seed1` | `(1243,6384)` | `(1243,6384)` |

这些隔离实验随后由 commit `64a3941` 落入 production。最终固定 quick
仍为 47/77；两个大型回归和 A/B/C/D 均复现上表。正式版本以
`BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2` 导出 objective version，并用
true-direct、target-blocker、PREPARE、mixed-phase、有限可压倒性、
cutoff-tie 和 free-robot-count 变化测试冻结边界。full 509 仍需独立终审
后运行，因此本表不能代替 full 结论。

### 13.3 第三组实验：additive `S-D`

只有在上述 bottleneck 对照完成后，才运行第 10 节的 additive full solver。该实验允许显式 idle，不施加 maximum-cardinality，并把 PREPARE 作为部分收益。

比较必须同时报告：

- solved；
- 首解和最终 `(T,W)`；
- free moves、Lift/Drop；
- matching CPU；
- 搜索节点和 generator failures；
- raw search、repair、第二遍各自贡献。

### 13.4 若最终保留 bottleneck，增量化需要另一套设计

严格增量 bottleneck 需要维护或重新寻找最小可行 threshold，并在该 threshold 内维护 secondary matching。active task、dummy 和阈值改变都会使状态失效。

可能的后续路线是：

1. 先尝试复用父节点 threshold；
2. 在该 threshold 下做动态可行匹配；
3. 若不可行或存在更小 threshold，再向上/向下搜索；
4. 在最终 threshold 内增量维护 secondary objective。

这比 additive row repair 复杂得多，因此先通过阶段 1 benchmark 判断保留 bottleneck 的收益是否值得，而不是预先删除它。

## 14. 同货架、同 endpoint 和 EXECUTE/PREPARE 的处理

### 14.1 同货架与同 endpoint 不是普通二分图边约束

“每台机器人最多一个任务、每个任务最多一台机器人”可以由 Hungarian 表达。但下面的约束是任务—任务互斥：

- 两个不同任务属于同一实体货架；
- 两个任务会在同一时刻占用同一 endpoint。

普通 Hungarian 不能直接表达“这一组任务最多选一个”。当前代码通过按 priority 提前选赢家来规避冲突，但这会重新引入非全局决策。

建议：

1. 对当前应当唯一的每货架任务建立断言和计数；本报告目标实例已经验证为唯一；
2. 对 endpoint conflict group 记录实际出现频率；
3. 区分“本轮绝不能同时执行”与“存在明确 vacate 前驱后可继续”的时序冲突，不能把相同 endpoint 一概当成永久互斥；
4. 若冲突组稀少，可在组外做有界枚举，每个选择调用一次 Hungarian；
5. 若冲突组普遍且结构稳定，再引入带 capacity node 的 min-cost flow 或专门的冲突感知 assignment；
6. 在第一版暂时保留 priority-ordered transfer claims 时，必须把它标为“预选兼容集合内的全局匹配”，记录每个删除原因，并做独立 ablation。

### 14.2 EXECUTE/PREPARE

最低风险版本可以暂时保留“EXECUTE 先、PREPARE 后”，但分别做到：

- 不按 priority cutoff 删除本类中的普通任务；
- 阶段 1 保留原 task-row bottleneck；
- 阶段 2 再分别测试 robot-row additive full solver；
- 明确 PREPARE 只表示 approach/等待，不是完成 transfer。

后续再实验统一矩阵：

- EXECUTE 和 PREPARE 都是 task columns；
- EXECUTE 与 PREPARE 使用不同收益；
- PREPARE 不抵扣完整 \(D_m\)；
- 同一 transfer 的不同 mode columns 属于同一 owner/conflict group；
- idle 与等待关键任务保持合法；
- mode 偏好使用有限成本或经过验证的词典序，而不是默认 maximum-cardinality。

这能区分“物理上还不能执行”和“策略上更希望先执行”，避免把策略偏好伪装成可行性过滤。

## 15. 建议实施步骤

### 阶段 0：诊断补全，不改变行为

在不改变行为的前提下，为每次 \(\rho\) matching 记录：

- 输入 ready、transfer-claim 后、same-key/shelf 去重后、priority cutoff 后的候选数；
- 每个删除任务的阶段、原因、priority 和最近机器人距离；
- free/all robot 数；
- task/column universe、mode、数值成本版本是否变化；
- 相对父节点 changed rows 数量；
- continuity mate 与 anchor_used 的变化；
- assignment change 与 owner handoff；
- 交付路径重建 guidance 的 free→free non-null retarget；
- 每次 joint transition 实际移动、Lift、Drop 的机器人数量。

耗时至少拆成：

```text
候选生成与各层过滤
cost / completion matrix 构造
signature / fingerprint
matching state 复制与内存分配
bottleneck threshold search
Hungarian full solve
incremental augmentation（阶段 3 后）
canonical tie refinement
```

这一阶段用来回答“矩阵究竟有多稳定”，而不是凭动画猜测。

### 阶段 1：候选边界实验

唯一主要变化是删除 `match_ready_tasks()` 的普通 priority top-\(F\) cutoff。尽量保留：

- task-row / robot+dummy shape；
- bottleneck-then-sum；
- 当前 service/tail；
- 当前 continuity；
- 当前 canonical refinement；
- EXECUTE 先、PREPARE 后；
- 上游 transfer claims，先记录但不同时重写。

该阶段回答：

> 改善是否仅来自让附近普通任务获得比较机会？

必要时再增加一个同 objective 的有限 defer-delay ablation，但不切换到 additive。

### 阶段 2：新调度目标实验，始终 full solve

明确实现第 10 节的 robot-row `S-D` 模型：

- 服务成本与推迟损失分离；
- `criticalTail` 不直接作为正的即时服务成本；
- idle 合法，不使用 maximum-cardinality 主目标；
- PREPARE 只获得部分准备收益；
- age 若不能成为可重放的节点状态，则第一版关闭；
- 完整定义矩形、负成本、INF、固定点范围和 canonical tie。

每次都 full solve。阶段 2 的 full solver 是阶段 3 唯一正确的行为 oracle；阶段 1 因 objective 不同，不能作为阶段 3 oracle。

### 阶段 3：对阶段 2 的同一数学问题做精确增量

- 新增满足 64 位 checked arithmetic 的 `RhoAssignmentState`；
- matching/duals 跟随具体 LaCAM 节点；
- 保存 `ColumnModelVersion`、`RowFingerprint`、`mate` 和 `anchor_used`；
- 只有完整列模型版本一致时才 repair rows；
- 列结构或数值变化时 full solve；
- debug/shadow 模式同时运行阶段 2 full solver；
- 若承诺行为等价，逐次断言 objective 和 canonical assignment 都一致；
- 单独统计状态复制、augmentation 和 canonicalization。

只有 shadow comparison 在单元测试、目标实例和 benchmark sample 上全部通过后，才关闭生产 full oracle。

### 阶段 4：按数据扩展结构

- 实现单列/少量列 repair；
- 合并 EXECUTE/PREPARE；
- 对 endpoint conflict group 做外层枚举或 min-cost flow；
- 把 rank urgency 升级为 PairCost/slack 的固定点延期代价；
- 增加有限的区域 continuity/lease。

每一项都应单独 ablation，不能与增量实现捆绑后只报告一个最终结果。

## 16. 必需测试

### 16.1 候选边界测试

1. 远处高 priority 与附近低 priority 都到达阶段 1 assignment 输入；
2. 每个未进入矩阵的任务都有硬约束或显式冲突组原因；
3. `state_t=724` 的距离 2 任务不再被 top-\(F\) cutoff 删除；
4. `state_t=1108` 的距离 1、2 任务不再被 top-\(F\) cutoff 删除；
5. 测试不强制最终选择最近任务，只要求输出完整 bottleneck/secondary 解释；
6. transfer-claim 和 same-shelf 预选的删除数独立可见。

### 16.2 additive cost 语义测试

1. priority/urgency 作为延期损失会影响任务选择，而统一 task-row 常数不会；
2. idle 可以在“现在执行弱任务”比“等待关键任务”更差时胜出；
3. work-conserving/maximum-cardinality 只作为可选 ablation；
4. 较长 critical tail 不会因正的即时服务成本被自动惩罚；
5. 共享 blocker 的 root impact 不重复计算同一条关键链收益；
6. PREPARE 只获得部分准备收益，不能抵扣完整 EXECUTE 延期损失；
7. 同一 transfer 的 EXECUTE/PREPARE 不能同时分配两个 owner；
8. transition-age 只随真实动作拍增长，不随 guidance rebuild 增长；
9. continuity 只在收益接近时稳定 assignment，不压倒明显更好方案。

### 16.3 增量正确性测试

1. 随机矩阵单行更新后，incremental 与 full objective、assignment 完全一致；
2. 多行更新同样一致；
3. 一个 row 改变引起 augmenting path，全局重新分配多台机器人；
4. 0 行变化不重新求解；
5. task identities 不变但 urgency/age 数值变化时必须 full solve 或正确列更新；
6. 父节点增广链更换多个 mate 后，下一代相应 `anchor_used` 行被判为变化；
7. Lift 使一个 task 对其他机器人失效时不能只修复 lifting robot 一行；
8. task/column model version 改变安全 fallback；
9. 矩形矩阵与负有限成本；
10. INF、负成本和固定点缩放不溢出；
11. duplicate/rewire 节点只能从经过验证的真实前驱复制状态；
12. 同一 upper layout 的 sibling 分支不共享可变 matching state。

必须加入审查中的矩形增广例：

\[
C_{\mathrm{parent}}=
\begin{bmatrix}
0&100&100\\
0&1&2
\end{bmatrix},
\qquad
C_{\mathrm{child}}=
\begin{bmatrix}
100&100&0\\
0&1&2
\end{bmatrix}.
\]

只改变第一行后，子矩阵最优解必须把第一行移到第三列，并通过增广路径把第二行移到第一列，总成本从父矩阵的 1 变为 0。

还必须构造多最优解，验证 canonicalization：

- 若契约要求逐位等价，cold/full 与 warm/incremental assignment 必须完全相同；
- 若只要求最优值一致，测试和 benchmark 必须明确允许不同 assignment，不能再声称行为等价。

### 16.4 目标实例回归

对 `brap_h10w10_a12_e8_R1_seed1` 至少固定检查：

- `state_t=724` 的距离 2 任务不再被 priority cutoff 删除；
- `state_t=1108` 的距离 1、2 任务都进入相应阶段的比较；
- 记录最终是否仍选择远处任务，并输出每个 cost 分量解释原因；
- 比较交付路径重建 guidance 中 310 次 free→free non-null retarget，但不把下降作为成功必要条件；
- 比较 owner handoff、首解时间、最终 makespan/work 和 matching CPU。

新算法不应被测试强制为“永远选最近任务”。如果高 urgency 或 critical chain 的收益足够大，选择远处任务仍可能是正确结果；测试应要求它经过同一个矩阵的显式权衡。

### 16.5 full benchmark

使用同一 509 实例 manifest 比较：

- solved 数；
- first-solution time；
- deliverable runtime；
- first/final makespan 与 weighted work；
- guidance time 和独立 matching time；
- full solve / exact reuse / row repair / fallback 比例；
- changed rows 为 0、1、2、\(>2\) 的分布；
- 列 identity、列数值、mode/conflict 版本变化率；
- `rho` changes、owner handoffs、交付路径重建 retarget；
- matching state 平均/峰值内存及 cleanup；
- raw search、projection repair、第二遍各自对最终计划的贡献；
- 计划验证和确定性 SHA。

不能只看动画，也不能只看 matching microbenchmark。全局派工改变会影响 LaCAM 的搜索顺序，因此必须同时看求解率、首解速度和最终质量。

## 17. 风险与尚未证明的内容

1. **尚未证明增量 Hungarian 一定加速。**当前证据说明 upper epoch 被大量复用、\(\rho\) 高频重算，但尚未测量完整 `ColumnModelVersion` 不变率和 matching 独立耗时。
2. **尚未证明减少跨区域移动一定改善 makespan。**局部连续性通常降低 free moves，但关键 blocker 可能确实值得远距离优先处理。
3. **不能把旧 v5 单例结果当作该改动的 A/B。**旧结果同实例为 makespan 1652、work 3652、runtime 8.275 秒，并且第二遍找到过一个改进候选；当前结果为 1844/3927、第二遍无候选。两次运行的两遍控制、停止和 repair 语义不同，差异不能归因于 \(\rho\) matching。
4. **不能把 `rho_repairs` 当作增量算法次数。**它只是 assignment 变化计数。
5. **不能全局缓存 matching。**matching 和 duals 必须跟随搜索节点；upper epoch 只能共享静态元数据。
6. **不能把 priority 权重调到近似无穷大。**否则只是把当前硬 cutoff 换成数值版硬 cutoff。
7. **不能假设每一步严格只变一台机器人。**实现应利用“小 \(k\)”而不是依赖“\(k=1\)”。
8. **不能把 additive `S-D` 宣称为 makespan 等价模型。**它是新的 guidance surrogate，必须与保留 bottleneck 的候选边界实验分开。
9. **不能把 maximum-cardinality 设为默认主目标。**本例 0.1508 是 loaded-move ratio，不是闲置率；交付前缀非 Wait 动作占比为 95.36%。
10. **不能把正的 critical tail 同时混入即时服务成本和延期收益。**两者必须有独立物理解释，并避免共享 roots 重复奖励。
11. **不能只比较任务身份来决定缓存有效。**列数值、mode、age、objective、scaling 和 tie 版本都属于 `ColumnModelVersion`。
12. **不能把普通 tie hash 当成 canonical matching 的证明。**若 full/incremental 要行为等价，必须使用有保证的 canonicalization。
13. **不能把增量 solver 的收益与派工模型变化混在一起。**相同矩阵下增量算法应复现 full assignment；retarget 改变来自成本模型，CPU 改变来自求解方法。

## 18. 最终建议

本问题的第一优先级不是增加一个“同区域优先”补丁，也不是立刻把当前目标换成 additive Hungarian，而是依次冻结候选边界、调度 surrogate 和求解契约：

```text
当前：
ready tasks
  → priority 硬过滤
  → 小矩阵 Hungarian

阶段 1：
预选兼容集合
  → 取消普通 priority top-F cutoff
  → 保留 bottleneck / dummy / continuity / canonical tie
  → 验证候选边界

阶段 2：
明确定义 service、defer、idle、PREPARE
  → robot-row additive full solver
  → 不使用 maximum-cardinality 主目标
  → 与阶段 1 比较真实 (T,W)

阶段 3：
固定阶段 2 的数学问题和 canonical matching
  → node-local matching / dual state
  → 完整 ColumnModelVersion + RowFingerprint
  → 列模型稳定时 repair rows
  → 任一列结构或数值变化时 full solve
```

最小可验证闭环应是：

1. 先补候选、矩阵、matching、tie、复制和 fallback 的分阶段 telemetry；
2. 只取消普通 cutoff，保留原 bottleneck 做阶段 1 benchmark；
3. 单独实现 `S-D` full solver，明确 idle/tail/PREPARE 语义做阶段 2 benchmark；
4. 阶段 2 full solver 冻结后，再实现精确增量并进行逐节点 shadow 校验；
5. 最后根据冲突组、列变化频率和复制开销决定是否增加 column repair、统一 mode 或 min-cost flow。

一句话概括：

> **先让普通可行任务获得比较机会，再验证哪种调度目标真正改善 makespan；只有目标和 canonical assignment 冻结后，才用增量 Hungarian 加速同一个决定。**

## 19. 实施结果与最终 production 决策

本报告提出的路线已经按独立提交执行，而不是直接把 additive 与 incremental
一起并入 production：

| 阶段 | commit | 固定 quick / 关键结果 | 决策 |
|---|---|---|---|
| F0 telemetry | `c8c59b2` | 47/77 | 保留 |
| F1 no-cutoff | `5085efb` | 45/77；两个大型实例丢失 | 候选合同保留，继续修正成本 |
| F2S finite tie | `266e69b`、`be720df` | 47/77；样例 A `(28,45)` | 混合阶段基线 |
| additive G0/G1 | `7b829b1`、`d7e5412` | 47/77；Testcase C `(231,268)` | `968d02a`、`34d92be` 回滚 |
| H1 shadow | `339fe7e` | 建立在回归的 additive 目标上 | `dcaf496` 回滚 |
| strict direct-target V2 | `64a3941` | 47/77；A `(17,32)`，B/C/D 不变 | production |

这组结果验证了审查中的核心区分：additive full solver 和增量 state 可以在
工程上实现，但“可实现、可增量”不等于“更符合 makespan”。G/H 没有因为
方便复用 Hungarian 而保留；production 继续使用 task-row bottleneck，
只在 §13.2.1 的严格同质阶段增加有限 dummy 延期。

最终代码通过 C++ 312/312、Python 162/162。最终 quick 相对 F2S 的 47 个
共同成功实例为 2 个更好、42 个完全相同、3 个更差，solved-set 不变。
两个改善是 `(1361,2964)→(1359,2958)` 与 `(53,106)→(45,89)`；三个回退
包括同 makespan 下 work `817→818`，以及两个 warehouse case 的 makespan
`24→26`、`21→23`。因此当前结论不是 V2 单调改善所有实例，而是它以严格
边界修复样例 A 和目标交付连续性，同时保住 Testcase C 与大型可解性回归。

当前二进制 SHA-256 为
`d90a0efa2ee2436778ceda107bbc785d2d371e51fd13eba9b00d90ec1758af7d`。
full 509、最终比较网页及其独立审查仍是发布前剩余步骤。
