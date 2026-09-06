# Carrier-LaCAM \(\rho\) 全局匹配实现计划与执行记录

状态：2026-09-06，F0 诊断、F1 取消普通 priority cutoff、F2S 对照和收紧后
的直接目标交付 V2 已完成；production 代码停在 commit `64a3941`。additive
G0/G1 与节点局部 H1 shadow 已按本计划做过独立实验，但因调度质量回归而
完整回滚，没有留在 production。C++ 312/312、Python 162/162 和最终固定
quick 77 已通过；独立终审与 full 509 尚未完成。审计起点为
`80148a7db6ee9a756dbe314b4f393d15c8f018e9`。

本文件同时保留原始实施计划和实际执行结果。未勾选的长期 G/H 项不表示当前
production 缺少承诺功能：additive 与增量 Hungarian 本来就是必须先由实验
证明收益的候选路线；实验失败后停止，正是 §7 的预定决策。

设计依据：

- `carrier_lacam_rho_global_matching_report_20260906.md`
- `design_final.md` §8、§10、§13.4、§14 F–H、§16、§17.5、§24
- 当前实现：
  `lacam/src/carrier_guidance.hpp::match_ready_tasks()`
- 可复用实现：
  `lacam/include/tapf_assignment.hpp::TAPFAssignmentState`
  与 `third_party/ITA-CBS2`

所有阶段遵循：

```text
冻结输入与基线
  → 先写 RED 测试
  → 最小实现
  → 定向 GREEN
  → C++/Python 全量 GREEN
  → 目标实例
  → quick 77
  → 独立审查
  → 必要时 full 509
```

每个阶段使用独立提交和独立 benchmark 目录。不得把候选模型、调度目标和
增量求解三个变化压在一个提交中。

---

## 当前实施结论

| 阶段 | 提交/结果 | 决策 |
|---|---|---|
| F0 telemetry | `c8c59b2` | 47/77；确认 priority 在矩阵前大量过滤候选 |
| F1 no-cutoff | `5085efb` | 45/77；普通候选全部进入比较，但两个大型实例超时 |
| F2S 有限低阶 priority tie | `266e69b`、`be720df` | 恢复 47/77 和两个大型实例，但受保护样例 A 退化到 `(28,45)` |
| additive G0/G1 | `7b829b1`、`d7e5412` | quick 47/77，但 Testcase C 退化到 `(231,268)`；由 `968d02a`、`34d92be` 回滚 |
| H1 node-local shadow | `339fe7e` | 建立在已回归的 additive 目标上，没有独立采用价值；由 `dcaf496` 回滚 |
| 收紧 F2 V2 | `64a3941` | production；严格 direct-target gate 下使用有限 frontier/continuity dummy defer |

最终 V2 的明确边界是：

```text
所有普通可行任务仍进入矩阵，priority_filtered = 0；
只有 EXECUTE 且全部候选都是“自己的目标货架 → 自己的合法最终 goal”
时，priority frontier 与 parent continuity 才有限增加 dummy 延期；
一旦出现 blocker、非终端 task 或 PREPARE，整次 matching 保持 F2S/V1
的物理 bottleneck 语义。
```

最终 quick 位于
`benchmark/results_quick_rho_v2_20260906/rows.csv`，结果为 47/77，
runner wall time 47.9 秒。相对 F2S，47 个共同成功实例中 2 个词典序更好、
42 个完全相同、3 个更差；没有 solved-set 变化。关键保护结果为：

| 检查项 | F2S | 最终 V2 |
|---|---:|---:|
| 发布样例 A | `(28,45)` | `(17,32)` |
| 发布样例 B | `(12,36)` | `(12,36)` |
| Testcase C | `(31,93)` | `(31,93)` |
| 发布样例 D | `(8,23)` | `(8,23)` |
| `h20w20 a40 e100 R1 seed0` | `(1273,5796)` | `(1273,5796)` |
| `h20w20 a40 e100 R1 seed1` | `(1243,6384)` | `(1243,6384)` |

F2S→V2 的 5 个 quick 质量变化也必须保留负面证据：两个改善是
`h10w10 a12 e8 R1 seed1` 的 `(1361,2964)→(1359,2958)` 和
`h6w10 a6 e15 B seed1` 的 `(53,106)→(45,89)`；三个回退是
`h8w10 a10 e20 R1 seed0` 同 makespan 下 work `817→818`，以及两个
warehouse `b3/d50`、`b3/d75` case 的 makespan `24→26`、`21→23`。
这些回退不能在报告中省略，full 509 用于判断其总体分布。

---

## 0. 本轮目标与非目标

本轮依次回答三个问题：

1. **候选边界：**普通可行任务是否都获得了参加 matching 的机会？
2. **调度目标：**bottleneck 与 additive `S-D` 哪个更有利于真实
   `(makespan, work)`？
3. **求解方法：**在数学问题冻结后，能否用节点局部 matching/duals
   更快地得到同一个 canonical assignment？

最终目标不是让机器人“看起来总在同一区域”，而是：

```text
priority 不再拥有无限准入权
  + 所有普通可行任务经过同一有限成本比较
  + 搜索仍返回合法且按 (T,W) 比较的计划
  + 增量版本不改变 full solver 的数学结果
```

本轮明确不做：

- 不修改 `PairCost` 或 `tau_guide` 的 robot-independence；
- 不改变 `SearchKey`、goal、`apply_ops()` 或 primitive successor 集；
- 不把“最近任务”写成硬规则；
- 不把同区域 lease/永久 owner 当作正确性条件；
- 不把 maximum-cardinality 设为默认主目标；
- 不同时实现动态 column repair、统一 EXECUTE/PREPARE、min-cost flow；
- 不用运行时环境变量在 production 中切换调度策略；
- 不把旧 v5 结果与新结果的差异直接归因于 \(\rho\)。

实验变体使用独立提交、独立 worktree/build 和
`benchmark/run_benchmark.py --carrier-bin`；production 始终只有一个明确
的 \(\rho\) 实现。

---

## 1. 不可违背的实现合同

### 1.1 候选合同

- 普通 task 只有在以下原因之一成立时才允许不进入矩阵：
  - 当前 dispatch mode 的因果条件不满足；
  - source/shelf/custody 与真实物理状态不一致；
  - robot 到 pickup 在静态墙图上不可达；
  - 同一实体货架或当前 endpoint 的显式 conflict group 不允许同时服务；
  - task 已由真实 carrier 执行或被兼容 active episode 覆盖；
  - task identity 无效或重复。
- priority rank 本身不是硬过滤理由。
- 第一版可以保留上游 priority-ordered transfer claims 和 same-shelf
  兼容预选，但必须单独计数、记录删除原因并做后续消融。
- 测试只要求附近 task 进入同一比较，不要求最终一定选择附近 task。

### 1.2 目标合同

- F 阶段保留当前：

```text
task rows
robot + dummy columns
bottleneck completion
secondary distance/continuity
deterministic canonical refinement
EXECUTE 先、PREPARE 后
```

- G 阶段的 `S-D` 是新的 additive guidance，不宣称与 makespan 等价。
- priority 只能改变“现在服务”和“本轮延期”的相对成本；给 task 整行所有
  列加同一个常数无效。
- `criticalTail` 不直接作为正的 additive 即时服务成本。
- idle 必须是合法选择；PREPARE 不能获得完整 EXECUTE 延期收益。

### 1.3 增量合同

- 相同 `RhoProblem`、相同 objective、相同 canonical 规则时：

```text
full objective == incremental objective
full assignment == incremental assignment
```

- matching/duals 属于具体 LaCAM 节点，不得放入 `UpperEpochCache`。
- 不能只比较 task identities。任何列数值、mode、conflict、objective、
  scaling、INF 或 canonical 版本变化都使第一版增量状态失效。
- joint transition 可以改变 0、1 或多台机器人；接口必须是
  `repair_rows(changed_rows)`。
- incremental 失败、状态不可信或 shadow 不一致时必须安全 full solve。
- 仅保存 duals 不应改变 retarget；如果 assignment 变化，必须能归因于
  候选/成本语义变化，而不是 warm solver 随机选择了另一个最优解。

### 1.4 交付合同

- search、rewrite、两遍、repair 和最终 replay 继续使用同一 `(T,W)`。
- 第二遍失败仍返回第一遍已验证 incumbent。
- benchmark 先报告 solved/合法性，再比较首解、最终质量和 runtime。
- `rho_repairs` 保留为旧字段时必须明确它是 assignment-id change count，
  不能重新解释为 incremental repair count。

---

## 2. 冻结基线

### 2.1 代码与正式产物

- [x] 源码 commit：
  `80148a7db6ee9a756dbe314b4f393d15c8f018e9`
- [x] `build/dd_benchmark` SHA-256：
  `bc08b69ab26ad026e890420b59cb58dadb58daa0b55edc2319ccecc16a57598a`
- [x] 正式 rows：
  `benchmark/results_full_two_pass_reference_20260906/rows.csv`
- [x] rows SHA-256：
  `0a60c0a45e99b919a313add482674e1231e712ef0eeb58b65acf1af7849160ab`
- [x] timing SHA-256：
  `9fa63cd81d91c9282a6faa27ace04a95a77118832f98b8c35b703c0040ce9ee6`
- [x] 目标计划：
  `benchmark/results_full_two_pass_reference_20260906/work/`
  `brap_h10w10_a12_e8_R1_seed1.carrier.plan`
- [x] 目标计划 SHA-256：
  `561631698435e1372d10c593b9b1df8072e73c8bf9e4989880d19318de19c8a9`

### 2.2 目标实例口径

实例：

```text
benchmark/instances_brap_pool/g10x10/
brap_h10w10_a12_e8_R1_seed1.yaml
```

冻结指标：

| 指标 | 当前值 |
|---|---:|
| first solution | `1188 ms` |
| first `(T,W)` | `(2659,5691)` |
| final `(T,W)` | `(1844,3927)` |
| loaded/free/LiftDrop | `556 / 1871 / 1090` |
| phase-2 candidate/improvement | `0 / 0` |
| projection removed | `815` |
| reconstructed free→free retarget | `310` |
| deliverable runtime | `9030.13 ms` |
| guidance time | `3976.05 ms` |

必须保留两个状态回归：

- `state_t=724`：附近距离 2 task 当前被 top-\(F\) cutoff 删除；
- `state_t=1108`：距离 1、2 tasks 当前被 cutoff 删除。

### 2.3 full 509 基线

| 指标 | 当前值 |
|---|---:|
| solved | `479/509` |
| ready task count | `13,625,227` |
| `rho_task_id` changes | `3,917,608` |
| owner handoffs | `961,068` |
| upper epoch builds | `387,854` |
| guidance time | `1697.780 s` |

这些数值只说明 \(\rho\) 高频构造且值得测量；不能在 F0 之前推断 Hungarian
占了多少时间，也不能用 upper epoch hit 推断矩阵完全相同。

### 2.4 基线验证命令

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 16 --target test_all dd_benchmark
./build/test_all --gtest_color=no

cd benchmark
python3 -m unittest discover -s tests -p 'test_*.py' -v
cd ..
```

目标实例：

```sh
./build/dd_benchmark \
  benchmark/instances_brap_pool/g10x10/brap_h10w10_a12_e8_R1_seed1.yaml \
  10 /tmp/brap_h10w10_a12_e8_R1_seed1.plan 0
```

quick 77：

```sh
python3 benchmark/run_benchmark.py \
  --benchmark-tier quick \
  --carrier-bin build/dd_benchmark \
  --out-dir benchmark/results_quick_rho_<stage>_20260906
```

---

## 3. 计划中的代码落点

| 文件 | 主要修改 |
|---|---|
| `lacam/src/carrier_guidance.hpp` | 拆分候选收集、当前 bottleneck problem、canonicalization；F1 删除 cutoff；G 构建 robot-row `S-D` problem |
| `lacam/include/tapf_planner.hpp` | 扩展 `DDReadyMatchProbe`、`CarrierGuidance`、`TAPFStats`；H 增加节点局部 `RhoAssignmentState` |
| `lacam/src/tapf_planner.cpp` | 聚合 \(\rho\) telemetry；从真实 parent transition 继承/失效节点局部 state |
| `lacam/include/dd_planner.hpp` | 将新计数和耗时汇总到 `DDPlanStats` |
| `lacam/src/dd_planner.cpp` | attempt/两遍之间累计新 stats，不改变 incumbent 语义 |
| `tools/dd_benchmark.cpp` | 输出新增标量指标；保留旧字段兼容 |
| `benchmark/run_benchmark.py` | 新增 CSV fields、严格解析与 schema tests |
| `lacam/include/rho_assignment.hpp` | G/H 专用 64 位 full/incremental assignment 类型；不直接改变旧 TAPF public contract |
| `lacam/src/rho_assignment.cpp` | 64 位矩形负成本 Hungarian、checked arithmetic、动态 row repair |
| `tests/test_dd_dispatch.cpp` | 保留当前 bottleneck tests；增加 cutoff-removal 与 defer tests |
| `tests/test_dd_rho_candidates.cpp` | 新增候选边界、删除原因、目标状态最小 fixture |
| `tests/test_dd_rho_additive.cpp` | 新增 `S-D`、idle、PREPARE、conflict/canonical tests |
| `tests/test_dd_rho_incremental.cpp` | 新增 full/incremental differential、rewire、anchor/version tests |
| `tests/test_tapf_hungarian_shared.cpp` | 只在抽取共享整数核心时扩展；旧 int API 必须逐位兼容 |
| `benchmark/tests/test_rho_metrics.py` | 新字段解析、非负/求和关系、旧 `rho_repairs` 口径 |
| `CMakeLists.txt` | 注册三个新 C++ test targets 和 `test_all` |

实现倾向：新建 `RhoAssignmentState`，不要直接把现有
`TAPFAssignmentState` 的 `int`、sentinel 和 hash tie 改成另一套语义。
若抽取共享 Hungarian 核心，旧 `tapf_hungarian_row_to_col(int)` 和 TAPF
assignment 的输出必须保持原样。

---

## Stage F0：补全诊断，不改变派工行为

目标：先测清矩阵是否稳定、时间花在哪里、task 在哪一层消失。

### F0.1 拆分 `match_ready_tasks()` 的内部阶段

- [ ] 将当前大函数拆成私有 helper，但保持输出逐位一致：

```text
collect_current_rho_candidates(...)
build_current_bottleneck_problem(...)
solve_current_bottleneck_problem(...)
canonicalize_current_rho_assignment(...)
materialize_rho_probe(...)
```

- [ ] 保留当前 priority cutoff、mandatory、dummy、bottleneck、
  secondary、canonical 行为，不在 F0 修复它。
- [ ] 保留 EXECUTE 和 PREPARE 两次独立调用。
- [ ] `DDReadyMatchProbe` 增加测试可见的诊断，不把完整矩阵永久存入每个
  production node。

### F0.2 候选诊断

建议增加：

```cpp
enum class RhoDropReason {
  INVALID_TASK,
  DUPLICATE_TRANSFER_KEY,
  SAME_SHELF_PRESELECTED,
  UPSTREAM_TRANSFER_CLAIM,
  MODE_INELIGIBLE,
  NO_REACHABLE_ROBOT,
  PRIORITY_TOP_F,
};

struct RhoCandidateAudit {
  int task_index;
  TransferKey key;
  TaskId id;
  DispatchMode mode;
  int priority;
  int nearest_robot_distance;
  RhoDropReason reason;
};
```

- [ ] 分别记录：
  - `ready_tasks_with_custody()` 输入及 transfer-claim 删除；
  - `match_ready_tasks()` 输入；
  - same-key 去重后；
  - same-shelf 预选后；
  - priority cutoff 后；
  - 最终 matrix rows/columns。
- [ ] EXECUTE/PREPARE 分开累计，不能混成一个候选数。
- [ ] 对每个被删 task 记录最近 eligible robot 距离；production 只保存
  reason counters，完整列表仅由 probe/有界 debug trace 输出。
- [ ] 增加断言和计数：同一 shelf 在 preselection 前有多少候选、当前
  endpoint conflict group 出现多少次。

### F0.3 耗时和输入稳定性

新增聚合字段：

```text
rho_match_calls_execute / rho_match_calls_prepare
rho_candidates_input
rho_candidates_after_claims
rho_candidates_after_key_dedupe
rho_candidates_after_shelf_preselect
rho_candidates_after_priority
rho_priority_filtered
rho_matrix_rows_total / rho_matrix_cols_total / rho_matrix_max_rows

rho_candidate_time_ms
rho_matrix_time_ms
rho_bottleneck_time_ms
rho_secondary_full_time_ms
rho_canonical_time_ms

rho_column_identity_same
rho_column_value_same
rho_mode_or_conflict_same
rho_changed_rows_0 / _1 / _2 / _gt2
```

- [ ] F0 只计算便宜且确定的 identity/value fingerprints，不复用
  assignment。
- [ ] fingerprint 必须由实际有序内容计算；测试中还要比较完整内容，不能
  把 hash 相等当正确性证明。
- [ ] changed rows 根据真实 parent transition、robot position、
  free/carry/custody 和 continuity anchor 计算。
- [ ] `rho_repairs` 保留原输出；新增 `rho_assignment_changes` 后可让旧字段
  成为兼容别名，但不得改名后偷换语义。

### F0.4 stats 管线

- [ ] `TAPFStats`、`DDPlanStats`、`accumulate_attempt_stats()`、
  `dd_benchmark` 和 `run_benchmark.py::FIELDS` 全部接线。
- [ ] 新字段必须有 Python schema/非负整数/有限浮点测试。
- [ ] raw log 与 CSV 数值逐项一致。
- [ ] F0 不增加按节点无上限的字符串或 vector stats，避免诊断本身耗尽
  deadline/内存。

### F0.5 RED/GREEN 测试

- [ ] 在重构前增加 direct probe golden，锁定当前 assignment、
  bottleneck、secondary 与 canonical 结果。
- [ ] 同一输入重构前后 `rho_task_id/rho_transfer_key/rho_ready_index`
  完全一致。
- [ ] 每次调用满足：

```text
input
  >= after_claims
  >= after_key_dedupe
  >= after_shelf_preselect
  >= after_priority
  == matrix rows
```

- [ ] priority 删除数量只出现在 F0 当前语义下。
- [ ] timing 字段有限、非负；计数不会因 stats disabled 改变 assignment。
- [ ] full benchmark parser 对旧 rows 缺失新字段仍能明确报 schema version，
  不静默填错含义。

### F0 完成门

- [ ] 定向 C++ tests GREEN；
- [ ] 全 C++/Python GREEN；
- [ ] 目标实例 direct assignment/probe 与当前基线一致；
- [ ] quick 77 没有因重构产生非预期合法性/目标变化；
- [ ] 输出一份 F0 telemetry 摘要，决定是否值得继续 H。

建议提交：

```text
rho-f0: split matching stages and add behavior-preserving telemetry
```

---

## Stage F1：取消普通 priority top-\(F\) cutoff

目标：只改变候选边界，保留当前 bottleneck dispatch 的其余语义。

### F1.1 RED 测试

- [ ] 两台 free robots、三个 tasks：
  - 远处高 priority；
  - 近处低 priority；
  - 另一个高 priority；
  三个 tasks 都必须到达 assignment problem。
- [ ] 低 priority task 可以匹配 dummy，但不能在矩阵前被删除。
- [ ] 每个未进入矩阵的 task 都有非 priority 的硬原因。
- [ ] synthetic `state_t=724` fixture：距离 2 task 进入矩阵。
- [ ] synthetic `state_t=1108` fixture：距离 1、2 tasks 进入矩阵。
- [ ] 测试不断言最终必须选择最近 task，只断言其 completion/secondary
  成本被显式计算。
- [ ] same-shelf/transfer-claim 的预选仍可发生，但原因和数量可见。

目标状态建议使用两层测试：

1. 快速 C++ synthetic fixture 固定 task graph、robots、priority 和距离；
2. 独立诊断脚本重放正式 1844 拍计划，在 724/1108 状态检查 probe。

不要让每次 `test_all` 都重放完整 1844 拍并依赖 deadline。

### F1.2 最小代码修改

当前删除块：

```cpp
const int cutoff = candidates[free_count - 1].priority;
erase(priority < cutoff);
mandatory = priority > cutoff;
```

修改为：

- [ ] 删除普通 top-\(F\) erase；
- [ ] 删除 `RhoCandidate::mandatory`，避免以后隐藏恢复无限优先级；
- [ ] 保留当前 deterministic candidate ordering；
- [ ] 保留 same-key 和 same-shelf 预选，留到单独阶段；
- [ ] `dummy_count = max(0, task_count - free_count)`；
- [ ] 当存在 dummy columns 时，**每一个**普通 candidate row 都可连接
  dummy；
- [ ] F1 继续使用当前：

```text
dummy completion = best_real_completion + max(1, service)
dummy secondary  = (best_real_approach + defer_delay) * switch_scale
```

此处不加入 priority 权重，确保唯一主要变量是 cutoff removal。

- [ ] 如果 task 对所有 eligible robots 都静态不可达，在建矩阵前以
  `NO_REACHABLE_ROBOT` 硬原因删除；不得让一行全 INF 导致整个 matching
  无解释失败。
- [ ] EXECUTE/PREPARE 两次调用都使用相同的“无 priority cutoff”合同。

### F1.3 兼容性检查

- [ ] 当前 bottleneck threshold 计算不变；
- [ ] threshold 内 secondary cost 不变；
- [ ] 当前 repeated optimum canonical refinement 不变；
- [ ] `criticalTail`、service、switch penalty 不变；
- [ ] task_count \(\le\) free_count 时仍保持当前“所有 tasks 可服务”的
  行为；F1 不解决 cardinality/idle 策略；
- [ ] 不改变上游 transfer claims、Task-BR priority propagation 或
  PairCost。

### F1.4 benchmark

依次运行：

1. direct unit/probe；
2. 目标实例；
3. quick 77；
4. 若 quick 无合法性/求解率阻塞，再申请 full 509 新 approval。

必须比较：

```text
solved
first_solution_ms
first/final (T,W)
free moves / LiftDrop
generator failures / search nodes
candidate counts at each stage
guidance and rho sub-timers
owner handoffs
reconstructed retarget
```

成功标准不是“310 必须下降”，也不是“724 必须选择距离 2 task”。F1 只需
证明附近 task 获得同一 objective 下的比较机会，并报告最终为何选中或未选中。

建议提交：

```text
rho-f1: remove priority top-F admission cutoff
```

---

## Stage F2：版本化的有限 priority/continuity defer delay

F1 证明普通任务可以全部进入矩阵，但也暴露了一个边界：在纯直接目标交付
阶段，V1 的物理 bottleneck 可能为了照顾远端低 priority task，打断已经
开始的目标交付。F2 不恢复 cutoff，而是只在严格、可测试的同质阶段增加
有限 dummy 延期。

### F2.1 规则

生产 objective 版本命名为：

```text
BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2
```

V2 gate 必须同时满足：

```text
mode == EXECUTE
所有当前候选都是 TARGET b
critical_tail == 0
task roots 包含该货架自己的 (b, g)
transfer.endpoint == g
g ∈ target_goal_sets[b]
```

任一候选是匿名 blocker、替其他 root 清障的目标货架、非终端任务，或者
本轮是 PREPARE，则整次 matching 与 V1 数值和 assignment 等价。这个
all-candidates gate 是刻意的：混合清障阶段继续让附近 blocker 与远端任务
按 F2S 物理 bottleneck 比较，不能重新制造 priority 饥饿。

只有 gate 成立时，priority 与 continuity 修改 dummy 延期成本：

\[
\operatorname{completion}(m,\operatorname{dummy})
=\operatorname{bestPhysicalCompletion}(m)+\Delta_{\mathrm{defer}}(m),
\]

其中：

\[
\Delta_{\mathrm{defer}}(m)
=\operatorname{service}(m)
 \left(1+I_{\mathrm{frontier}}(m)+I_{\mathrm{continue}}(m)\right).
\]

- `frontier`：候选多于 free robots 时，priority 不低于第 \(F\) 名；
- `continue`：某台当前 free robot 的 parent assignment 仍是该 task；
- 三项只作用于 dummy，最大 multiplier 固定为 3；
- priority/continuity 在 V2 中明确改变主 bottleneck completion，不只是
  secondary tie；
- V1 的 physical secondary、有限 priority tie 和 canonical refinement
  保持不变。

- [x] `DeltaDefer(service, frontier, continue)` 写成纯函数，返回有限
  `int64_t`。
- [x] 不允许任何 priority 值产生 INF/mandatory。
- [x] 不给真实列和 dummy 列统一加同一个 task 常数。
- [x] target-shelf blocker、匿名 blocker 和 PREPARE 不触发 V2。
- [x] mixed phase 必须逐矩阵、逐 assignment 等价于 V1。
- [x] frontier cutoff tie 和 free-robot 数变化保持确定。
- [x] 参数集合在 quick benchmark 前冻结；禁止按实例、尺寸或 seed 调参。
- [x] 不增加 production 运行时环境开关；使用独立提交/二进制做消融。

### F2.2 测试

- [x] true direct-target EXECUTE 激活 V2；
- [x] target-shelf blocker 和 PREPARE 保持 V1；
- [x] mixed target/blocker phase 的矩阵和 assignment 与 V1 相同；
- [x] priority 较高只增加“延期损失”，不会删除低 priority row；
- [x] continued assignment 只在旧 task 仍是候选时增加其 dummy 延期；
- [x] 足够大的距离差仍可让附近低 priority task 胜出；
- [x] cutoff ties 与 free robot 数变化有 canonical 确定结果；
- [x] `candidates_after_priority == matrix_rows` 且
  `priority_filtered == 0`；
- [x] 所有成本在 checked range 内；
- [x] C++ telemetry 与 benchmark runner 导出精确 V2 名称。

### F2 决策门

F2 只与 F1/F2S 比较，不与 G/H 捆绑。进入正式实现前的隔离实验必须同时
满足：

```text
样例 A：严格优于旧 (18,34)
样例 B/C/D：保持 (12,36)/(31,93)/(8,23)
quick 中两个大回归：
    seed0 保持 (1273,5796)
    seed1 保持 (1243,6384)
```

若 quick 出现 solved-set 回退，production 回到 F2S；不得修改 benchmark
或用实例特判兜底。additive G/H 仍是独立实验，不能用其历史结果替代 F2
验证。

---

## Stage G0：冻结 additive `S-D` 数学问题

目标：在写 solver 前先冻结 column、cost、idle、mode 和 canonical 语义。

### G0.1 第一版范围

最低风险版本继续：

```text
先对 EXECUTE 做 robot-row additive full solve
锁定已获 EXECUTE 的 robots
再对 PREPARE 做第二次 robot-row additive full solve
```

第一版不统一两个 mode，不解决任意 task-group conflict。same-shelf 和
transfer-claim 仍是显式、可计数的预选兼容集合。

第一版关闭 age：

```text
lambda_age = 0
```

因为 age 尚未进入可重放搜索状态。禁止用 guidance rebuild 次数或 wall-clock
模拟 age。

### G0.2 问题类型

建议类型：

```cpp
enum class RhoColumnKind {
  EXECUTE_TASK,
  PREPARE_TASK,
  IDLE,
  LOCKED,
};

struct RhoColumnKey {
  RhoColumnKind kind;
  TransferKey transfer;
  TaskId task;
  int stable_slot;
};

struct RhoCostBreakdown {
  int64_t approach;
  int64_t immediate_service;
  int64_t continuity;
  int64_t mode;
  int64_t urgency_reward;
  int64_t root_delay_reward;
  int64_t total;
};

struct RhoProblem {
  std::vector<int> robots;             // 固定为全部 robots
  std::vector<RhoColumnKey> columns;   // tasks + N idle/locked
  std::vector<std::vector<int64_t>> cost;
  std::vector<std::vector<RhoCostBreakdown>> breakdown;
};
```

矩阵：

```text
rows    = all N robots
columns = explicit task columns + N own idle/locked columns
```

- free eligible robot：合法 task edges + 自己的 idle；
- carrying/不可用 robot：只连接自己的 locked/continuation；
- task column：最多一台 robot；
- 未选择的 task column：保持 unmatched，表示本轮延期；
- 每个 robot row：始终有一个有限 idle/locked edge。

### G0.3 成本合同

EXECUTE：

\[
S^E_{rm}
=w_d\,distance
+w_s\,immediateService
+w_c\,switch
+w_{\mathrm{mode}}\,modePenalty
\]

\[
D_m
=\lambda_p\,urgency
+\lambda_k\,rootDelayImpact
\]

\[
C^E_{rm}=S^E_{rm}-D_m.
\]

第一版最小模型：

- [ ] `urgency` 使用有限 priority rank 的固定点映射；
- [ ] `rootDelayImpact=0`，待单独定义和消融；
- [ ] `age=0`；
- [ ] `immediateService` 使用当前 transfer 的立即资源占用，不加入正的
  successor `criticalTail`；
- [ ] continuity 是有限成本，不是永久 owner；
- [ ] idle edge 有明确定义的有限成本，不能通过 cardinality 前置强制服务。

PREPARE：

\[
C^P_{rm}=S^P_{rm}-B^P_{rm},
\qquad 0\le B^P_{rm}\le D_m.
\]

- [ ] `B^P` 只表示预计减少的未来 approach；
- [ ] PREPARE 不执行 Lift、不完成 transfer；
- [ ] 第一版两阶段求解确保同一 transfer 不同时获得 EXECUTE/PREPARE 两个
  owner；
- [ ] 后续统一矩阵前必须显式实现 owner/conflict group。

### G0.4 canonical 合同

定义唯一 canonical 顺序：

1. robot id 升序；
2. 在保持全局最优 objective 的 columns 中按
   `(kind-order, TransferKey, TaskId, stable_slot)` 选择；
3. task columns 在同成本时可排在 own idle 之前，但这只是确定性 tie，
   不是高于真实 cost 的 maximum-cardinality 层级。

- [ ] canonical 规则写入 version；
- [ ] full solver 单元测试要求 assignment 逐位稳定；
- [ ] 不使用普通有限 hash 宣称 matching 总和唯一。

G0 先形成设计测试和纯 cost-builder tests，不接 production。

---

## Stage G1：实现 64 位 additive full solver

### G1.1 数值实现

新建 `rho_assignment.hpp/.cpp`：

- [ ] `using RhoCost = int64_t`；
- [ ] `kRhoInf` 与最大合法有限成本分离；
- [ ] potentials、slack、总和及缩放中间值使用 checked `__int128`；
- [ ] 支持 rows \(\le\) columns；
- [ ] 支持负有限成本；
- [ ] forbidden edge 不参与最优性计算；
- [ ] 返回 objective、row→column 和 feasible；
- [ ] 禁止对 task column 做非负平移；
- [ ] 若做 row shift，必须同时平移该 row 的 task 和 idle/locked 合法列，
  并把常数正确还原到 objective。

优先从现有 Hungarian 抽取共享整数核心；旧
`tapf_hungarian_row_to_col(int)`、`TAPFAssignmentState` 和 zero-shelf TAPF
行为必须保持逐位兼容。

### G1.2 full canonical oracle

- [ ] 先求最小 additive objective；
- [ ] 再按 G0.4 顺序反复检查“固定此 edge 后剩余问题是否仍达到同一最优
  objective”；
- [ ] 返回唯一 canonical assignment；
- [ ] 单独计时：
  - cold optimum；
  - canonical refinement；
  - cost matrix construction。

G1 的 full canonical solver 是 H 阶段唯一行为 oracle。即使它暂时较慢，也
不能为了速度换成无证明的 tie hash。

### G1.3 RED 测试

新增 `tests/test_dd_rho_additive.cpp`：

- [ ] priority 作为 `D_m` 会影响服务/延期选择；
- [ ] 给 task 整行所有列加同一常数不改变 assignment；
- [ ] min-sum 与 bottleneck 的 2×2 反例得到不同结果；
- [ ] idle 可在弱 task 不值得立即启动时胜出；
- [ ] 不先最大化非空 task 数；
- [ ] 长 `criticalTail` 不因正即时 service 自动受罚；
- [ ] PREPARE 收益小于等于完整延期收益；
- [ ] 同一 transfer 两个 mode 不出现双 owner；
- [ ] shared blocker root reward 不重复；
- [ ] negative rectangular matrix；
- [ ] INF 与最大固定点边界；
- [ ] 多最优解 canonical assignment 固定；
- [ ] cold solver 重复运行逐位一致。

### G1.4 production 接线

- [ ] 仅在 G 独立提交中将两次 `match_ready_tasks()` 替换为 additive full
  solver；
- [ ] 上游 candidates 与 F1 相同；
- [ ] EXECUTE/PREPARE 仍分两阶段；
- [ ] 现有 `rho_task_id/rho_transfer_key/rho_ready_index/rho_mode` 接口
  不变；
- [ ] timed transport 和 PIBT 消费接口不变；
- [ ] stats 明确记录 objective kind/version。

### G1.5 benchmark 与决策

比较至少三组独立二进制：

```text
F1: no-cutoff bottleneck
F2: no-cutoff bottleneck + finite defer（若实现）
G1: no-cutoff additive S-D full solver
```

报告：

- solved；
- first/final `(T,W)`；
- first solution / deliverable runtime；
- free moves、Lift/Drop、shelf switches；
- search nodes、generator failures；
- candidate counts；
- full solver/canonical CPU；
- owner handoffs、retarget；
- raw search、projection repair、phase 2 贡献。

决策规则：

- 不能因为 G 容易增量化就默认采用 G；
- 不能只看目标实例；
- 若 G 在固定 quick/full 上明显劣于 F，停止 additive H 路线；
- 若最终保留 bottleneck，另开“动态 threshold + secondary matching”
  设计，不把 H 的 additive state 硬套上去。

建议提交：

```text
rho-g0: define additive rho problem and exact full oracle
rho-g1: route production rho through additive full solver
```

---

## Stage H0：定义节点局部复用边界

只有 G1 数学问题和 canonical assignment 冻结后开始。

### H0.1 状态位置

`CarrierGuidance` 本身是具体 `TAPFNode` 的 node-local 数据，可增加：

```cpp
std::optional<RhoAssignmentState> rho_execute_state;
std::optional<RhoAssignmentState> rho_prepare_state;
```

禁止加入：

```text
UpperEpochGuidance
UpperEpochCache
全局 map<UpperSignature, matching>
```

父→子复用必须经过 `attach_carrier_guidance()` 已验证的真实 transition：

- transition replay 与 child physical state 一致；
- parent guidance 不是 stale；
- rewire 时先刷新新 parent；
- sibling 各自复制 value state，不能共享可变 potentials。

### H0.2 `ColumnModelVersion`

不要只保存一个 hash。建议保存 exact descriptor，并附 hash 加速：

```cpp
struct ColumnModelVersion {
  std::vector<RhoColumnKey> ordered_columns;
  std::vector<int64_t> service;
  std::vector<int64_t> urgency;
  std::vector<int64_t> root_delay;
  std::vector<uint64_t> endpoint_conflict_version;
  uint32_t mode_semantics_version;
  uint32_t objective_version;
  uint32_t scaling_version;
  uint32_t inf_version;
  uint32_t canonical_version;
  uint64_t quick_hash;
};
```

- [ ] quick hash 相等后仍做 exact equality；
- [ ] task identities 相同但任一数值改变 → full solve；
- [ ] EXECUTE/PREPARE 分别保存各自 column model；
- [ ] age 第一版关闭，因此不进入版本；以后启用时必须是节点状态。

### H0.3 `RowFingerprint`

```cpp
struct RowFingerprint {
  Cell robot_position;
  KappaMode kappa;
  std::optional<TransferKey> custody;
  DispatchMode phase;
  EligibilityBits eligibility;
  std::optional<RhoColumnKey> anchor_used;
};
```

必须分开：

```text
mate         = 当前 problem 求出的 assignment
anchor_used  = 构建当前 row continuity cost 时使用的上一代 assignment
```

- [ ] parent augmenting path 改变多个 mates 后，child 对应 rows 即使没移动
  也因新 anchor 重新计算；
- [ ] Lift/Drop 使 task 对其他 robots 失效时通常属于 column/model 变化，
  第一版 full solve；
- [ ] changed rows 分布写入 telemetry。

### H0.4 fallback 原因

枚举并统计：

```text
NO_PARENT_STATE
STALE_OR_REWIRED_PARENT
SHAPE_CHANGED
COLUMN_IDENTITY_CHANGED
COLUMN_VALUE_CHANGED
MODE_CHANGED
CONFLICT_CHANGED
OBJECTIVE_VERSION_CHANGED
CANONICAL_VERSION_CHANGED
STATE_VALIDATION_FAILED
SHADOW_MISMATCH
```

任何无法分类的 fallback 是 bug，不使用笼统 `OTHER` 长期隐藏。

---

## Stage H1：64 位增量 objective solver（先 shadow）

### H1.1 动态状态

```cpp
struct RhoAssignmentState {
  int rows;
  int columns;
  std::vector<int> mateL;
  std::vector<int> mateR;
  std::vector<WideCost> row_potential;
  std::vector<WideCost> column_potential;
  ColumnModelVersion column_model;
  std::vector<RowFingerprint> row_fingerprint;
  std::vector<std::optional<RhoColumnKey>> anchor_used;
  int64_t objective;
  uint32_t canonical_version;
};
```

- [ ] `solve_full(problem)` 初始化 matching/duals；
- [ ] `repair_rows(changed_rows, problem)` 解除 changed rows 并重新增广；
- [ ] augmenting path 允许重分配未改变 rows；
- [ ] 0 rows 变化直接复用；
- [ ] 状态 validation 检查 matching injective、所有 matched edges 有限、
  primal/dual objective 一致；
- [ ] validation 失败 full solve，不把坏状态继续传播。

### H1.2 shadow 模式

H1 不立即用 incremental assignment 驱动 PIBT：

1. 正常运行 G1 full canonical solver；
2. 同时运行 incremental objective solver；
3. 比较 objective；
4. 记录 changed rows、copy/repair/full 时间与 fallback；
5. production 仍返回 full canonical assignment。

这样可以先回答：

- 列模型到底多常不变？
- 0/1/k row repair 比例是多少？
- state copy 是否比 full solve 更贵？
- objective repair 是否正确？

shadow 不通过时不能进入 H2。

### H1.3 differential tests

新增 `tests/test_dd_rho_incremental.cpp`：

- [ ] 随机小矩阵单行改变，objective 与 full 一致；
- [ ] 多行改变一致；
- [ ] 0 行变化不增广；
- [ ] 一行变化通过 augmenting path 全局重分配其他 rows；
- [ ] 报告中的矩形例：

\[
C_{\mathrm{parent}}=
\begin{bmatrix}
0&100&100\\
0&1&2
\end{bmatrix},
\quad
C_{\mathrm{child}}=
\begin{bmatrix}
100&100&0\\
0&1&2
\end{bmatrix}
\]

  child 必须得到成本 0，而不是只局部替换后保留成本 1。
- [ ] negative costs；
- [ ] task identities 不变但 urgency 数值改变 → full fallback；
- [ ] mode/conflict/INF/canonical version 改变 → full fallback；
- [ ] augment 改变多个 mates 后 child anchors 正确变化；
- [ ] sibling state value-copy 隔离；
- [ ] duplicate/rewire 从新真实 parent 重建；
- [ ] Lift 造成跨 rows eligibility 改变时不错误单行修复；
- [ ] checked overflow/INF；
- [ ] 随机 differential 至少覆盖数千个小矩阵和 row-update 序列。

建议提交：

```text
rho-h0: add node-local rho model versions and shadow state
rho-h1: add exact incremental objective repair in shadow mode
```

---

## Stage H2：canonical 等价与生产启用

objective 相同不够。H2 必须保证 warm/cold assignment 逐位一致。

### H2.1 最安全的第一步

- [ ] incremental repair 先给出最优 objective；
- [ ] 仍运行 G1 full canonical refinement；
- [ ] 比较 incremental mate 与 full canonical mate；
- [ ] production 返回 full canonical mate。

这一步可能没有净加速，但能分离：

```text
matrix construction
incremental optimum
full canonical refinement
```

如果 canonical 占主要时间，不得把“objective repair 很快”报告成整体
matching 已加速。

### H2.2 精确 canonical incremental oracle

只有 H2.1 数据证明值得继续时实现：

- [ ] 把 canonical refinement 写成消费抽象
  `OptimumUnderFixedEdges` oracle；
- [ ] full oracle 与 incremental oracle 使用同一 robot/column 顺序；
- [ ] 每次临时固定 edge 后，受影响 state 使用独立副本，失败不污染 parent；
- [ ] 任何 restricted problem 改变 columns/shape 时安全 full solve；
- [ ] 不用有限 tie hash 代替证明；
- [ ] 多最优解 tests 要求 full/incremental assignment 逐位相同。

### H2.3 启用门

仅当以下条件全部满足，production 才可使用 incremental 结果：

- [ ] 单元随机 differential 全绿；
- [ ] 目标实例逐次 shadow mismatch = 0；
- [ ] quick 77 shadow mismatch = 0；
- [ ] benchmark sample 中 objective/canonical mismatch = 0；
- [ ] 所有 fallback 都有原因；
- [ ] state copy + repair + canonical 总耗时小于 full solver；
- [ ] 峰值内存和 cleanup 未造成 deadline 回退；
- [ ] 输出计划全部 replay 合法；
- [ ] independent review 通过。

若没有净收益，保留 G1 full solver；“增量未上线”是允许的正确结论。

建议提交：

```text
rho-h2: enable canonical-equivalent incremental assignment
```

---

## Stage I：只在 telemetry 证明需要时扩展

这些项目不属于首轮闭环：

- [ ] 单列/少量列 repair；
- [ ] EXECUTE/PREPARE 统一矩阵；
- [ ] task-group/endpoint conflict 的有界枚举；
- [ ] 冲突普遍时引入 min-cost flow；
- [ ] 把 rank urgency 升级为 PairCost/slack 固定点延期代价；
- [ ] 把 transition age 加入可重放节点状态；
- [ ] 有限区域 continuity/lease；
- [ ] strict bottleneck 的动态 threshold + secondary matching。

每项必须独立测试和消融。不能因为 H 的 robot-row state 已存在，就默认这些
变化也正确。

---

## 4. Benchmark 协议

### 4.1 变体隔离

建议目录：

```text
benchmark/results_quick_rho_f0_20260906
benchmark/results_quick_rho_f1_20260906
benchmark/results_quick_rho_f2_20260906
benchmark/results_quick_rho_g1_20260906
benchmark/results_quick_rho_h_shadow_20260906
benchmark/results_full_rho_<accepted-stage>_20260906
```

每个结果目录必须记录：

- git commit；
- binary SHA-256；
- suite/manifest SHA-256；
- objective/canonical version；
- result rows/timing SHA-256；
- 是否 full、shadow 或 production assignment。

### 4.2 quick 与 full

quick：

```sh
python3 benchmark/run_benchmark.py \
  --benchmark-tier quick \
  --carrier-bin <variant-build>/dd_benchmark \
  --out-dir benchmark/results_quick_rho_<stage>_20260906
```

full：

```sh
python3 benchmark/run_benchmark.py \
  --benchmark-tier full \
  --review-approval <new-approval-bound-to-new-binary>.json \
  --carrier-bin <variant-build>/dd_benchmark \
  --out-dir benchmark/results_full_rho_<stage>_20260906
```

旧 approval 绑定旧 binary，不能复用于新实现。

### 4.3 必报指标

质量：

```text
solved
first/final makespan
first/final weighted work
first-solution time
deliverable runtime
raw search / repair / phase-2 contribution
```

动作：

```text
loaded moves
free moves
Lift/Drop
shelf switches
reversals
```

\(\rho\)：

```text
candidate counts by stage/reason
matrix rows/columns
full/reuse/repair/fallback counts
changed rows distribution
column identity/value/mode/conflict change rates
matrix/full/incremental/canonical/copy/cleanup time
matching state average/peak memory
assignment changes
owner handoffs
reconstructed free→free retarget
```

搜索：

```text
nodes/iterations
generator failures
rewire rebuilds
guidance total time
timed transport time
```

### 4.4 解释规则

- `robot_utilization` 是 loaded-move ratio，不是闲置率。
- `rho_repairs` 是 assignment change count，不是 incremental repairs。
- 310 retarget 是交付路径重建 guidance 指标，不是原搜索错误次数。
- nearby task 被纳入比较不意味着必须被选择。
- matching microbenchmark 变快不等于总 planner 变快。
- 同一 10 秒预算下，solver 加速可能改变搜索覆盖；必须同时报告计划质量。
- 旧 v5 单例 1652/3652 与当前 1844/3927 不是本轮直接 A/B。

---

## 5. 提交顺序与回滚点

| 提交 | 唯一主要变化 | 可安全回滚到 |
|---|---|---|
| `rho-f0` | 重构 + telemetry，行为不变 | `80148a7` |
| `rho-f1` | 删除普通 top-\(F\) cutoff | `rho-f0` |
| `rho-f2` | 有限 dummy defer delay | `rho-f1` |
| `rho-g0` | additive problem + full oracle，不接 production | `rho-f1/f2` |
| `rho-g1` | production 使用 additive full solver | `rho-g0` |
| `rho-h0` | node-local version/fingerprint + shadow state | `rho-g1` |
| `rho-h1` | incremental objective shadow | `rho-h0` |
| `rho-h2` | canonical-equivalent incremental production | `rho-h1` |

禁止：

- 在 `rho-f1` 同时改 continuity 权重；
- 在 `rho-g1` 同时接 incremental；
- 在 `rho-h*` 修改 `S-D` 权重或 mode 语义；
- 用 feature flag 在同一 production binary 中隐藏未审查的替代算法；
- benchmark 失败后覆盖已有结果目录。

---

## 6. Definition of Done

### F 完成

- [ ] F0 telemetry 完整，matching 独立耗时和矩阵稳定率可测；
- [ ] F1 普通低 priority task 不再被 top-\(F\) 删除；
- [ ] 724/1108 附近 task 进入比较；
- [ ] 每个未进入矩阵的 task 都有硬原因；
- [ ] 当前 bottleneck/secondary/canonical 语义未被偷换；
- [ ] C++/Python 全绿，计划 replay 合法；
- [ ] quick/full 报告完整，不用 retarget 单指标判定成功。

### G 完成

- [ ] `S-D` cost 合同、idle、PREPARE、canonical 已冻结；
- [ ] 64 位矩形负成本 full solver 有穷举/随机 oracle；
- [ ] full canonical assignment 确定；
- [ ] additive 与 bottleneck 使用独立 benchmark；
- [ ] 是否采用 additive 由真实 `(T,W)` 结果决定，而不是实现便利。

### H 完成

- [ ] matching/duals 节点局部，不污染 upper cache/sibling；
- [ ] `ColumnModelVersion` 覆盖结构和实际数值；
- [ ] `mate` 与 `anchor_used` 分离；
- [ ] 0/1/k changed rows 正确；
- [ ] 列变化安全 full fallback；
- [ ] full/incremental objective 和 canonical assignment 逐次一致；
- [ ] state copy、canonical 和 cleanup 纳入总性能；
- [ ] 无净收益时不启用 production incremental。

### 全局完成

- [x] 不改变物理 successor、goal、PairCost/tau 边界；
- [x] search/rewrite/two-pass/repair 仍统一 `(T,W)`；
- [x] 当前测试、样例和 quick 输出通过 C++ replay 和 Python validator；
- [x] no instance/seed special case；
- [x] no hidden runtime policy switch；
- [ ] quick/full artifacts、binary/hash、比较报告可审计；
- [ ] `design_final.md`、报告与本文件同步更新实际完成状态。

当前剩余项只包括独立终审、sealed full 509、最终比较网页及其独立网页审查。
G/H 的 production checkbox 保持未勾选，因为实验结论是“不采用”，不是把
已回滚的替代目标伪装成完成。

---

## 7. 停止与重新决策条件

出现以下情况时停止当前路线，不继续堆复杂度：

1. F0 显示列模型几乎每节点都变化：暂缓增量，先分析 column change 来源。
2. F1 让低 priority tasks 进入后求解率/首解明显恶化：检查矩阵规模、
   dummy 语义和上游冲突，不立即切 additive 掩盖问题。
3. G1 additive 在固定 corpus 上明显劣于 F：保留 bottleneck，单独设计
   dynamic threshold，不继续 additive H。
4. H1 objective repair 快但 canonical full 占绝大多数时间：不宣称整体
   加速，先决定是否值得实现 exact incremental canonicalization。
5. state copy/内存/cleanup 抵消增广收益：保留 full solver。
6. conflict-group 预选删除比例很高：下一步优先解决组约束，而不是继续调
   priority/continuity 权重。
7. 任一阶段出现合法性、replay、deadline 或 incumbent 保底回归：立即回滚
   到上一独立提交。

最终原则：

> 先修复候选边界，再选择调度目标，最后才优化求解器；任何性能优化都不能
> 改变已经冻结的数学问题和 canonical assignment。
