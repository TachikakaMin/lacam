# Carrier-LaCAM 两遍规划实施计划

日期：2026-09-06

## 0. 依据、范围与优先级

本计划以本轮确认的完整方案为实现目标：

```text
动态 goal sets 上找第一份可行解
    → 规范化、repair、重放并保存独立快照
    → 固定首轮实际终态目标
    → 从原始 X0 重新搜索
    → 用首轮成本作严格词典序上界
    → 第一份严格改进立即返回
    → 无改进、截止或候选失败时返回首轮快照
```

实现必须遵守 `rules.md`，并以 `design_final.md` 的算法边界为准。所有改动
都增量嵌入现有 `TAPFPlanner::solve()`、LaCAM OPEN/CLOSED、constraint
tree、Carrier-PIBT、`apply_ops()`、repair 和 replay 路径；禁止新增平行
planner、第二套 search loop、legacy fallback 或 testcase 特判。

本任务包括三层交付：

1. 两遍控制、上界、deadline、fallback 和统计的正确闭环；
2. 首轮参考动作与完整状态 checkpoint 后缀复用；
3. 按 `rules.md` 完成测试、quick/full benchmark、独立 review 和中文汇报网页。

## 1. 最终行为合同

统一成本为：

```text
PlanCost = (executed ticks, fixed-point weighted work)
```

比较使用严格词典序。普通 primitive joint transition 消耗一拍；macro edge
的 ticks 等于其中 primitive trace 的长度。

第一遍：

```text
start             = original X0
goal view         = original complete goal sets
external bound    = unbounded
stop policy       = FIRST_FEASIBLE
```

第一遍候选必须先经过 goal-prefix 规范化、事务式 repair、权威 replay 和精确
成本核算，形成与搜索树生命周期无关的 `VerifiedPlanSnapshot`。snapshot 至少
保存 primitive joint actions、`PlanCost`、最终 target positions 和可选
reference checkpoints。

第二遍：

```text
start             = original X0
goal view         = singleton goals from terminal(first)
external bound    = cost(first)
stop policy       = FIRST_STRICT_IMPROVEMENT
```

第二遍只固定目标货架的最终位置，不固定匿名货架、carrier、任务分配、清障
顺序、路线或时序。找到一份经过完整验证且严格优于首轮的候选即返回；未找到
时整体仍为 `SOLVED`，返回首轮 snapshot。

## 2. 正确性口径

### 2.1 上界与 repair

首轮 bound 是已验证、规范化、repair 后计划的成本；搜索 `g+h` 是原始搜索
路径成本。第一版明确采用以下可证明合同：

> 第二遍保证寻找“原始搜索成本已经严格优于首轮 bound”的计划。候选找到后
> repair 只能进一步改善；本版本不声称覆盖所有“只有 repair 后才变优”的
> raw candidate。

因此 `f >=lex bound` 对原始搜索计划空间是安全剪枝，但不能被描述为对所有
post-repair 交付计划完备。若以后要扩大该保证，必须把规范化成本接入内核，
或证明搜索前缀成本是交付成本不可撤销的下界。

### 2.2 admissible bound 与参考后缀

安全剪枝只能使用：

```text
g + admissible_h >=lex incumbent
```

首轮 checkpoint 后缀表示“存在一条这样完成的路径”，是可行上界，不是剩余
成本下界。因此 `g + reference_suffix >= incumbent` 绝不能剪掉节点。

### 2.3 deadline

只允许一层预算切分：

```text
hard return deadline
    └── search stop deadline
        为 snapshot、repair、replay、tree cleanup 和输出准备留一次余量
```

`dd_planner.cpp` 是求解调用内的唯一预算所有者；`TAPFPlanner::solve()` 只
服从传入 deadline，不再扣第二层 cleanup reserve。benchmark 外层仍以严格
10 秒 subprocess timeout 作为最终保险，开发时同时核对 solver runtime 与
端到端 process runtime，避免在文件发布阶段被外层杀死。

### 2.4 候选与验证

kernel goal、外部 incumbent 和最终 verified candidate 是三个不同概念。
external incumbent 只能用于 bound/fallback，不能伪装成本轮 `S_goal`。
kernel 找到候选后停止普通扩展；若 materialize/replay/repair 失败，返回首轮
snapshot。若未来要求候选失败后继续搜索，必须显式实现可恢复搜索，不能悄悄
启动第三遍。

## 3. 现有 execution path 的修改映射

| 机制 | 现有路径 | 计划中的修改 |
| --- | --- | --- |
| 停止策略 | `lacam/include/tapf_planner.hpp::TAPFSearchConfig` | 使用显式 `ANYTIME / FIRST_FEASIBLE / FIRST_STRICT_IMPROVEMENT`，删除含混布尔语义 |
| goal 接受与立即停止 | `lacam/src/tapf_planner.cpp::TAPFPlanner::solve()` | bounded pass 只接受 `g < incumbent_init`；首份严格改进设置本轮 goal 后立即退出 |
| bound 剪枝 | `TAPFPlanner::solve()` 的 OPEN 选择、pop、successor insert | 所有位置统一要求 `f < bound`；只使用 admissible `h` |
| cheaper arrival 重开 | `TAPFPlanner::rewrite()` 与 duplicate-state 分支 | 更新真实 `g/f`、传播改善并在新 `f < bound` 时重新入 OPEN，不永久拉黑物理状态 |
| 第一遍/第二遍控制 | `lacam/src/dd_planner.cpp::run_search_attempt()`、`solve_carrier_lacam_result()` | 同一控制器调用同一个 `TAPFPlanner::solve()` 两次；第一遍首解，第二遍首份严格改进 |
| snapshot 与固定目标 | `normalize_goal_prefix()`、`fixed_goal_instance_from_plan()` | 从最终可交付首轮计划重放得到 singleton target goals；snapshot 不持有搜索节点指针 |
| repair | `lacam/src/dd_plan_repair.cpp::repair_carrier_plan_impl()` | 保持事务式；只有合法且 lexicographically better 的 candidate 才替换 raw plan |
| deadline | `dd_planner.cpp` 的 controller deadline、`tapf_planner.cpp::solve()` | controller 只预留一次；删除 kernel 内重复 reserve；所有 cleanup 在 hard return 前完成 |
| 统计 | `lacam/include/dd_planner.hpp`、`tools/dd_benchmark.cpp`、`benchmark/run_benchmark.py` | 区分 first solution、phase-2 candidate、strict improvement、fallback 和 exit reason |
| reference 构造 | `dd_planner.cpp` 首轮 authoritative replay 后 | 构造有上限的 immutable checkpoint/index，保存完整物理状态、下一 joint action、suffix cost |
| reference 消费 | `TAPFSearchConfig` 与 `TAPFPlanner::solve()` 的现有节点扩展路径 | reference action 只影响候选顺序；exact checkpoint 可通过现有 trace/edge 路径注册合法后缀 |
| suffix 交付 | 现有 `plan_of()`、normalize、repair、replay 路径 | 拼接 primitive actions 后走同一交付管线，不复用旧 guidance/custody/reservation metadata |
| 无货架兼容 | 原 TAPF objective/config 与 shelf-empty 自然路径 | 不按“无 pick/place”切换算法；空 shelf layer 自然退化，旧 TAPF 行为保持一致 |

reference 类型只允许作为现有 planner/config 的只读输入，不得拥有独立 OPEN、
CLOSED、generator 或 search deadline。checkpoint 哈希只作索引，命中后必须
比较完整 `PhysConfig`。

## 4. TDD 实施阶段

当前工作树已有大量未提交实现和测试。开始修改前先按本计划审计现状：已经存在
的行为以测试和代码证据验证；任何尚未实现或发现错误的行为，必须先增加
regression test 并确认 RED，再做最小实现使其 GREEN。新建测试一经创建即为
protected test，后续不得直接弱化或修改。

### 阶段 A：两遍控制与 deadline 闭环

先写并运行以下 RED：

1. 第一遍使用完整 goal sets 且在第一份可行解停止；
2. 第二遍从原始 `X0`、固定首轮实际终态 goals 开始；
3. 相同成本不算改进；
4. 更小 ticks、较大 work 被接受；
5. 相同 ticks、更小 work 被接受；
6. 第二遍 cutoff/exhausted/candidate rejected 均返回首轮并保持 `SOLVED`；
7. 初态已满足 goal 返回合法空计划；
8. bounded pass 找到第一份严格改进后不继续产生第二次 incumbent update；
9. kernel 不再二次扣 cleanup reserve；
10. repair 超时或失败不清除已验证 raw plan。

随后只修改第 3 节对应的现有路径，使测试 GREEN。重点核对当前工作树中已经
出现的 stop-policy 枚举和 reserve 改动是否真正覆盖所有调用者、统计和
fallback，而不是仅编译通过。

### 阶段 B：bound、goal view 与重开

先写并运行以下 RED：

1. `(57,70)`、`(54,95)`、`(54,90)` 相对 `(54,90)` 被剪；
2. `(54,85)`、`(50,110)` 被保留；
3. 较贵路径到状态后被 bound 暂时剪掉，较便宜路径重达时可重新扩展；
4. 第二遍 goal、`h_T/h_W` 和 matching 均读取 singleton fixed-goal view；
5. dynamic-goal 第一遍缓存不能未经 goal-view key 验证进入第二遍；
6. 最终候选在原始实例上 replay，并以规范化 goal prefix 重新核算成本。

实现限定在现有 `current_bound`、OPEN/CLOSED、`rewrite()`、goal/h 和
adapter replay 路径内。

### 阶段 C：参考动作与 checkpoint 后缀复用

先写并运行以下 RED：

1. 只有 upper layout 相同、机器人或 `kappa` 不同时禁止拼接；
2. 完整 `PhysConfig` 相同且新前缀更便宜时可拼接首轮 primitive 后缀；
3. 拼接成本相同或更差时不返回该拼接，但也不剪掉其他后继；
4. 哈希冲突时通过完整相等比较拒绝错误 checkpoint；
5. 相同状态在首轮出现多次时选择成本最好的合法后缀；
6. reference next action 只改变排序，其他合法动作仍可生成；
7. 拼接后完整 replay 失败时保留首轮 snapshot；
8. 关闭 reference 优化不改变合法解集合、bound 条件或 fallback。

实现顺序：

1. authoritative replay 产生 bounded checkpoints 和精确 suffix cost；
2. 将 immutable reference pointer 接入现有 `TAPFSearchConfig`；
3. 先实现 exact-state suffix reuse；
4. 再接入 ordering-only joint-action hint；
5. 所有 stitched candidate 走现有 normalize/repair/replay 交付路径。

### 阶段 D：统计、回归与清理

先为每个发现的 crash、timeout inflation、错误状态或统计不一致写 RED
regression test。最终统计至少能区分：

```text
FIRST_PLAN_RETURNED
STRICT_IMPROVEMENT_RETURNED
NO_REMAINING_BUDGET
SEARCH_CUTOFF
SEARCH_EXHAUSTED
CANDIDATE_REJECTED
FIXED_GOAL_SETUP_FAILED
REFERENCE_SUFFIX_ACCEPTED
```

`assignment_second_solved` 只表示第二遍真实产生候选，不能把首轮 fallback
计入。

## 5. 固定开发期 benchmark 子集

开发期间固定以下 6 个 quick manifest 成员，不因结果更换：

1. `benchmark/instances_brap_pool/g4x10/brap_h4w10_a5_e1_R1_seed0.yaml`
2. `benchmark/instances_brap_pool/g6x10/brap_h6w10_a6_e1_B_seed0_pool.yaml`
3. `benchmark/instances_brap_pool/g10x10/brap_h10w10_a12_e3_B_seed1_pool.yaml`
4. `benchmark/instances_brap_pool/g20x20/brap_h20w20_a40_e100_R1_seed1.yaml`
5. `benchmark/viz_web/warehouse_block_suite/instances/warehouse_blocks_h20w20_b4_a1_d50_r8_t12_seed0.yaml`
6. `benchmark/viz_web/warehouse_block_suite/instances/warehouse_blocks_h20w20_b9_a1_d75_r8_t12_seed0.yaml`

该集合覆盖 singleton/dynamic goals、repair 压力、已知慢首解、大图和高密度
warehouse-block。它只用于开发循环，不能替代冻结的 77-case quick。

每个实现阶段执行：

```text
相关 C++ unit/integration tests
    → Carrier 相关 CTest
    → Python runner/validator tests
    → 固定 6-case subset（10s/case）
```

所有实现完成后才运行：

```text
完整相关测试
    → 原 LaCAM-TAPF backward-compatibility tests
    → benchmark/run_benchmark.py --benchmark-tier quick
    → 最终 diff 清理
    → 独立 GPT-5.6 Sol / high reviewer 明确 APPROVE
    → benchmark/run_benchmark.py --benchmark-tier full
```

review 前禁止运行新的 full benchmark，也禁止根据 protected full 的历史结果
选 seed、调参或替换 testcase。

## 6. Benchmark 比较指标

baseline 与新算法必须使用相同 dataset、method、10s timeout、seed、weights、
validator 和并行度。运行前检测物理 CPU cores，保留 1–2 个物理核心并监控
load、内存与 swap。

至少比较：

- solved / timeout / invalid；
- kernel first-solution time；
- verified first-snapshot time；
- first strict-improvement time；
- total solver runtime 与 subprocess runtime；
- returned `(makespan, fixed-point work)`；
- phase-2 attempted / candidate / improvement / cutoff；
- reference checkpoint hits / suffix attempts / accepted suffixes；
- `f_pruned`、`g_relaxed`、OPEN/CLOSED 与展开量；
- singleton 与 dynamic-goal、shared-pool 与 warehouse-block 分组结果；
- 与 `results_full_v5_854df1_20260905_r2` 及 pre-v5 full baseline 的配对比较。

## 7. Review、full benchmark 与网页

在 full benchmark 前，独立 reviewer 必须阅读本计划、`design_final.md`、
最终 implementation、测试和 quick 结果，并明确输出 `APPROVE` 或 `REJECT`。
若 `REJECT`，保持 protected tests/benchmarks 不变，修 implementation 后重新
走测试和 quick。

full 完成后制作静态中文汇报网页，内容包括：

- 一张从输入到最终返回的两遍流程图；
- 第一遍、第二遍和 fallback 的直观例子；
- 为什么过去约 7.65 秒停止、现在 deadline 如何只预留一次；
- 首解时间、严格改进时间、最终 runtime 和质量对比；
- representative case 动画或逐拍示例；
- backward compatibility、失败分类和仍存在的限制。

网页使用现有静态可视化路径，不新增临时 Python HTTP server。完成后再由独立
subagent review 页面内容、数据一致性、链接和可读性；发现问题时先补网页/
生成器 regression test，再修正。

## 8. 完成标准

只有以下全部有权威证据时才算完成：

1. 阶段 A–D 的行为合同均有测试覆盖并 GREEN；
2. 同一 `TAPFPlanner::solve()` 承担两遍搜索，没有平行 pipeline；
3. phase 2 第一份 verified strict improvement 即返；
4. phase 2 任何失败都保留 verified phase-1 snapshot；
5. bound、goal view、rewrite/reopen 和 reference suffix 语义正确；
6. strict 10s protocol 下没有重复 deadline reserve；
7. 原 LaCAM-TAPF 无 pick/place 测试行为保持兼容；
8. 77-case quick 完成并通过既定 gates；
9. 独立 reviewer `APPROVE`；
10. 509-case full 完成并与两套历史 baseline 公平比较；
11. 最终 diff 无 dead code、重复实现、feature flag、fallback 或 benchmark hack；
12. 中文汇报网页完成并通过独立 review。

## 9. 建议提交边界

1. `two-pass-controller-stop-policy-deadline`
2. `bounded-search-goal-view-reopen`
3. `reference-checkpoint-suffix-reuse`
4. `benchmark-regression-report-web`

每个提交都必须能对应本计划和 `design_final.md` 中的具体机制；阶段 C 是加速
项，但属于本轮明确范围，不能用阶段 A 已完成来替代其最终交付。
