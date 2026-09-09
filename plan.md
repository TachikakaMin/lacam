# Carrier BR-LaCAM 分解基线代码实施计划

**日期：2026-09-06**

**权威设计：**

- `carrier_br_lacam_decomposition_baseline_design_20260906.md`
- SHA-256：
  `00b46d91f1b460a886d45b60720b8ef6bc641affe7803db7486a9e5e727dfd7e`

**开发约束：**

- `rules.md`
- SHA-256：
  `d278aecda84b0fbf88074f20cd83ffd4109df08aab16ca1ec8b1a9bfe8c8f423`

本计划实现 `carrier_brd`：

```text
production tau0
→ 一次完整 Carrier BR-LaCAM upper search
→ immutable transfer waves
→ completion-event Hungarian
→ 复用 TAPFPlanner::solve() 搜索到首个新 Drop
→ 立即释放所有未 Lift assignment 并全量重匹配
→ 权威重放与 goal-prefix 交付
```

实现必须增量扩展现有 LaCAM/TAPF execution path。禁止复制
`Planner::solve()`、另写 parallel search loop、隐藏 fallback 到
`carrier`/`carrier_b0`/`carrier_b1`，或为 benchmark case 添加特殊分支。

---

## 1. 完成定义

以下条件全部有当前工作区证据时，本任务才完成：

1. `carrier_brd` 的根 \(\tau_0\) 与 production root tau 完全一致；
2. MAPF 与 Carrier BR upper adapter 使用同一份 LaCAM DFS control loop；
3. upper fully constrained successor 使用无截断 exact oracle；
4. upper path 被编译为带完整 `StorageTransfer.route` 的 immutable waves；
5. 任一 task Drop 后立即重匹配：
   - 一次 `TAPFPlanner::solve()` 只搜索到下一个 completion event，不等待
     当前 wave 的 tasks 全部完成；
   - 已 Lift robot-task 锁定；
   - 除已 Lift robot 外的所有 robots 与当前 wave 全部 PENDING tasks
     重新进入 V2 matcher；
6. lower segment 复用 `TAPFPlanner::solve()`，从任意合法 carrying
   `PhysConfig` 启动，并停在第一个 completion event；
7. matcher 的 all-PENDING admission、\(F=0\)、Hall deficiency、\(M-K\)
   dummy 和 deadline cutoff 语义通过测试；
8. raw plan 完成全部 frozen tasks；deliverable 是 raw plan 的第一个原始 goal
   prefix；两者都通过原始 validator；
9. 普通 MAPF、shelf-free TAPF 和 production Carrier-LaCAM regression 全绿；
10. quick 77 通过，最终 diff 获得独立 GPT-5.6 Sol/high `APPROVE`；
11. 只有上述 gate 满足后才运行 protected full 518；
12. 最终生成中文汇报网页，并由独立 subagent 审查为 `APPROVE`。

---

## 2. 当前代码差距

| 需求 | 当前状态 | 实施位置 |
|---|---|---|
| 共享 LaCAM control loop | `Planner::solve()` 仍内置一份 DFS OPEN/CLOSED/constraint loop | `lacam/include/search_kernel.hpp`、`lacam/src/planner.cpp` |
| upper Carrier BR domain | 不存在 labeled shelf domain、upper constraint 和 edge payload | 新增 `lacam/include/br_lacam_upper.hpp`、`lacam/src/br_lacam_upper.cpp` |
| exact upper oracle | 当前 Task-BR compiler 有 cap/window，只适合 guidance | `carrier_guidance.hpp` + upper module |
| frozen task waves | 不存在 stable anonymous identity/wave compiler | upper module |
| all-PENDING matcher | 当前会在 `free_robots.empty()` 时早退，并删除无可达 robot 的 row | `carrier_guidance.hpp` |
| carrying root | `TAPFPlanner::solve()` 根固定为 `ins->starts + initial_shelf_state` | `tapf_planner.hpp/.cpp` |
| event hard contract | 不存在 immutable ledger、fixed transfers、phase/transition validator | `tapf_planner.hpp/.cpp`、`dd_carrier.hpp/.cpp` |
| controller | 只有 production、B0、B1，没有 BRD controller | `dd_planner.hpp/.cpp` |
| benchmark mode | 只有 `carrier`、`carrier_b0`、`carrier_b1` | `tools/dd_benchmark.cpp`、`benchmark/run_benchmark.py` |
| BRD telemetry | 不存在 | `DDStats`、CLI metrics、Python `FIELDS` |

工作区当前有与本任务无关的未提交文档、dense-channel 和可视化改动。实现只修改
本计划列出的 BRD 文件；不得覆盖或清理其他改动。

---

## 3. 冻结开发用例

这些用例从本计划创建时起视为 protected。后续若需修改预期、地图或语义，必须
先按 `rules.md` 取得独立 GPT-5.6 Sol/high reviewer 的明确 `APPROVE`。

### 3.1 十个 C++ representative cases

统一放入 `tests/test_carrier_brd.cpp`，使用 inline `DDInstance`，避免外部生成器
变化。坐标和 storage mask 在首次 RED patch 中一次性写死：

| ID | 固定场景 | 核心断言 |
|---|---|---|
| R1 | 1×4 全 storage；1 robot；target 1→3 | 单 task、相邻 storage transfer 可交付 |
| R2 | 1×5，storage=`S...S`；1 robot；target 0→4 | route 保留全部 transit cells，不在 aisle Drop |
| R3 | 1×5，storage=`S.S.S`；target 0→2，anonymous blocker 在 2 | upper 先搬 blocker，再搬 target |
| R4 | 2×4 全 storage；2 robots；两个互不冲突 target | 同 wave 两个 task 可并发 |
| R5 | 十字形单格窄通道；2 robots；两条 transfer route 共享中心 | lower joint LaCAM 通过 Wait 解决 |
| R6 | 两 task 距离不等；先完成者 Drop 时另一 robot 尚未 Lift | 立即重匹配，旧 assignment 可变化 |
| R7 | 两 task；先完成者 Drop 时另一 robot 已 Lift | carrying pair 跨 event 保持同一 task |
| R8 | 1 robot、3 tasks | 产生至少 3 个 completion-driven dispatch epochs |
| R9 | 两个 locked carrying routes 构成无解物理冲突 | `SEGMENT_EXHAUSTED`，无 fallback |
| R10 | 初态 target 已 grounded at goal | solved 空计划，ticks/work 均为 0 |

### 3.2 固定开发 benchmark 子集

实现期间只允许使用 quick 77 的以下固定 8 case 做快速观察；不能因结果不利替换：

```text
benchmark/instances_brap_pool/g4x10/brap_h4w10_a5_e1_R1_seed0.yaml
benchmark/instances_brap_pool/g4x10/brap_h4w10_a5_e10_B_seed0_pool.yaml
benchmark/instances_brap_pool/g6x10/brap_h6w10_a6_e1_R1_seed0.yaml
benchmark/instances_brap_pool/g8x10/brap_h8w10_a10_e20_B_seed0_pool.yaml
benchmark/instances_brap_pool/g10x10/brap_h10w10_a12_e8_R1_seed1.yaml
benchmark/instances_brap_pool/g20x20/brap_h20w20_a40_e10_B_seed0_pool.yaml
benchmark/viz_web/warehouse_block_suite/instances/warehouse_blocks_h20w20_b3_a1_d50_r8_t12_seed0.yaml
benchmark/viz_web/warehouse_block_suite/instances/warehouse_blocks_h20w20_b9_a1_d75_r8_t12_seed0.yaml
```

子集与正式 quick 使用相同 seed 0、objective weights `(1,1,1,1)` 和单 case
10 秒 timeout。它只是开发反馈，不替代 quick 77。

---

## 4. 实施阶段与 TDD gate

每个阶段严格执行：

```text
新增 protected test
→ 构建/运行并保存 RED 原因
→ 最小实现
→ 相关测试 GREEN
→ 运行既有 compatibility tests
→ 检查 diff
```

发现 bug 时先增加 regression test 并观察 RED，再修实现。

### P0：建立测试骨架和基线证据

**修改：**

- `CMakeLists.txt`
- 新增 `tests/test_carrier_brd.cpp`

**先写测试：**

- 固定 R1–R10 的 instance factory 与名称；
- 只暴露阶段性 probe，不在测试中复制 planner；
- 首批 RED 测试覆盖 shared-kernel fake domain、root validator、
  all-PENDING matcher 和 empty-goal controller。

**基线命令：**

```bash
cmake --build build -j2 --target test_tapf_compat test_tapf_planner \
  test_dd_carrier test_dd_dispatch
./build/test_tapf_compat
./build/test_tapf_planner
./build/test_dd_carrier
./build/test_dd_dispatch
```

在任何 implementation edit 前记录上述结果。新测试目标第一次必须因缺少 BRD
API/行为而 RED。

### P1：抽取唯一的 LaCAM DFS control loop

**RED：**

- MAPF golden parity 使用现有 `test_tapf_compat`；
- fake domain 证明 root、goal、constraint expansion、duplicate push/backtrack
  和 path extraction 经过同一 kernel；
- fake domain 禁止在 adapter 内自建 OPEN/CLOSED solve loop。

**实现：**

- 在 `search_kernel.hpp` 新增模板化
  `run_lacam_dfs_search(Domain&)`；
- domain 提供 state key、node/constraint ownership、goal、candidate expansion、
  successor 生成、duplicate handling 和 path extraction hooks；
- `Planner::solve()` 改为薄 wrapper/adaptor；
- 保持 MAPF candidate order、`std::shuffle` 次数、RNG 消耗、日志和 cleanup
  顺序；
- 本阶段不修改 `TAPFPlanner::solve()`。

**GREEN：**

```bash
cmake --build build -j2 --target test_planner test_tapf_compat \
  test_tapf_planner test_carrier_brd
./build/test_planner
./build/test_tapf_compat
./build/test_tapf_planner
./build/test_carrier_brd --gtest_filter='carrier_brd_kernel.*'
```

### P2：Carrier BR-LaCAM upper domain

**新增类型：**

```cpp
UpperShelfHandle
BRLabeledUpperState
BRUpperConstraintEntry
AppliedUpperTransfer
BRUpperTransition
BRUpperNodeMetadata
CarrierBRDomain
```

**RED：**

- forced WAIT/TRANSFER；
- exhaustive candidate set 包含所有 `reachable_storage_transfers()`；
- top candidate window 不得成为完整集合；
- exact oracle 拒绝非法 route、following、重复 endpoint 和占用；
- 小图 exact successor set 与 brute-force oracle 相等；
- fully constrained branch 不调用 capped Task-BR；
- stable anonymous id、labeled CLOSED key、age/omega/rank 确定；
- R2/R3 upper path 可解且最终满足 fixed \(\tau_0\)。

**实现：**

- 新增 `br_lacam_upper.hpp/.cpp`，只承载 domain 和 immutable data；
- 给 `compile_task_br_pibt()` 增加默认 null 的 forced-effect view；null 路径必须
  production parity；
- partial constraint 用 Task-BR 做 ordering/completion；
- full constraint 只调用 `validate_complete_upper_action()`；
- upper adapter 进入 P1 的共享 kernel，不拥有另一份 solve loop；
- accepted edge payload 按 `(kind, stable_id)` 排序。

**GREEN：**

```bash
cmake --build build -j2 --target test_carrier_brd \
  test_dd_task_br_production test_dd_storage_transfer \
  test_dd_task_br_compiler
./build/test_carrier_brd --gtest_filter='carrier_brd_upper.*'
./build/test_dd_task_br_production
./build/test_dd_storage_transfer
./build/test_dd_task_br_compiler
```

### P3：upper path → immutable frozen task waves

**新增类型：**

```cpp
FrozenTaskId
FrozenShelfTask
FrozenTaskWave
FrozenTaskPlan
CanonicalDispatchMetadata
```

**RED：**

- 每个 applied transfer 恰好一个 frozen task；
- route/payload 原样保存；
- 重复 physical effect 仍有不同 `FrozenTaskId`；
- anonymous stable id 转换为 wave source-cell selector；
- partial/full 发现同一 successor 产生相同 edge payload、roots 和 priority；
- wave barrier、expected_before/after 和 shadow projection；
- pure upper replay 最终满足 \(\tau_0\)。

**实现：**

- 编译逻辑放在 upper module，controller 只消费 immutable plan；
- canonical metadata 只读取 `(U_before, tau0, applied_transfer)`；
- 每个 wave 编译后立即 exact replay；
- 不保存 compiler 临时 roots/priority。

### P4：all-PENDING V2 matcher

**接口变化：**

```cpp
enum class CandidateAdmission {
  DROP_GLOBALLY_UNREACHABLE,
  KEEP_ALL_PENDING_ROWS,
};

enum class RhoMatchStatus { OK, CUTOFF, INFEASIBLE };
```

默认参数保持 production 行为。

**RED：**

- default admission 的 assignment、telemetry 和 fingerprints parity；
- all-PENDING 保留 all-INF row，`no_reachable_robot_filtered == 0`；
- \(F=0\) 时保留 \(M\) rows/\(M\) dummies；
- Hall-deficient graph 计算最大 cardinality \(K\)；
- 最终 real assignments 恰好为 \(K\)；
- forced-dummy rows 不改变 actionable rows 的 V2 aggregate/matching；
- K prepass 和所有 Hungarian/refinement 循环响应同一 budget probe；
- cutoff 返回 `RhoMatchStatus::CUTOFF`，不伪装成 \(K=0\)。

**实现：**

- 在候选 dedupe/same-shelf audit 后按 admission 分流；
- 新增 budget-aware deterministic bipartite maximum matching prepass；
- dummy 数改为 \(M-K\)；
- actionable rows 单独计算 V2 global aggregates；
- `free_robots.empty()` 仅 default admission 早退；
- 扩充 `RhoMatchTelemetry`，production 默认字段值保持兼容。

### P5：arbitrary physical root 与 event contract

**新增物理校验 API：**

```cpp
enum class PhysRootInvalidReason;
PhysRootValidation validate_phys_config_root(
    const DDInstance&, const PhysConfig&);
```

它不得读取 `Graph::U`，必须先检查 vector size、cell range、wall、`kappa`
domain，再做任何按 cell/target 索引。

**新增 contract 类型：**

```cpp
FrozenTaskId
TaskLedgerEntry
FixedRobotTransfer
CarrierEventContract
CarrierTaskPhase
```

`TAPFSearchConfig` 增加：

```cpp
std::optional<PhysConfig> initial_physical;
const CarrierEventContract* event_contract = nullptr;
```

**RED：**

- invalid root 在 `config_of_physical()` 和 guidance attach probe 前拒绝；
- target/anonymous carrying root 合法；
- `initial_physical != contract.start` 拒绝；
- ledger 无缺失/重复，CARRYING/locked 一一对应；
- `phase_of()` 对 APPROACH/CARRYING/COMPLETED/INVALID；
- transition validator 禁止错 Lift、错 route、错 Drop、active 外 shelf 运动；
- target/anonymous 已到 endpoint 但尚未 Drop 的 carrying root 合法；
- null contract 路径保持 production root behavior。

**实现：**

- `dd_carrier.hpp/.cpp` 实现纯物理 root validator；
- `tapf_planner.hpp/.cpp` 实现 contract validator、`phase_of()` 和
  `validate_event_transition()`；
- `TAPFPlanner::solve()` 先验证 `root_X`，再构造 `root_C/root_S`；
- root assignment、node、h、CLOSED、guidance 全部使用同一 root；
- 非 contract 模式不接受 `initial_physical`。

### P6：fixed-event lower guidance 与 first-completion goal

这里的 segment 边界必须与 wave 边界分开：

```text
一次 TAPFPlanner::solve()
    = 从当前完整 PhysConfig 联合搜索
    = 固定本 epoch 的 provisional assignments
    = 精确停在至少一个 active task 首次 Drop 的最早 goal state

返回以后
    = 同拍完成的 tasks 全部记为 COMPLETED
    = 已 Lift、尚未 Drop 的 robot-task pairs 保持 hard lock
    = 所有尚未 Lift 的旧 assignments 全部释放
    = 除 locked robots 外的全部 robots
      × 当前 wave 全部 PENDING tasks
      立刻重新运行 matcher

只有当前 wave 全部 COMPLETED，才释放下一 wave；
不能把“wave barrier”误写成“等整批完成后才重匹配”。
```

**RED：**

- R1/R2 lower segment 完成 approach→Lift→route→Drop；
- unassigned robot 不得 Lift；
- contract mode 不调用 PairCost/tau/Task-BR/rho/LB/cache；
- shelf h 为 0，reference/macro/repair 禁用；
- R5 通过 joint Wait 解决共享通道；
- R6 停在第一个 Drop，不等待其他 active task；
- 同拍多个 Drop 一次性返回全部 completed tasks；
- rewrite 后 guidance 仍来自 fixed contract。

**实现：**

- `attach_carrier_guidance()` 首先按 contract 分流；
- fixed guidance 只做 operator ordering；
- `apply_ops()` 后再执行 hard event transition validator；
- `is_goal_config()` 在 contract 模式检查 first new completion；
- segment 使用 `MAKESPAN_THEN_WORK + FIRST_FEASIBLE + macro=false`；
- 每个 segment 使用新的 planner/CLOSED。

### P7：BRD controller、预算与交付

**新增 API：**

```cpp
enum class CarrierBRDExitReason;
struct CarrierBRDStats;
DDSolveResult solve_carrier_brd_result(
    const DDInstance&, double time_limit_sec, int seed,
    DDStats* stats = nullptr);
```

**RED：**

- R10 返回合法空计划；
- tau0 与 production probe 相等；
- R6 completion 后 matcher 立即再次调用；
- R7 carrying lock 跨 event；
- R8 多 dispatch epochs；
- `F=0 + carrying` 继续，`F=0 + no carrying` 为 `DISPATCH_STUCK`；
- matcher cutoff 为 `DISPATCH_TIMEOUT`；
- R9 segment failure 不 fallback；
- raw plan 完成全部 frozen tasks；
- deliverable 只在 raw 验证后生成；
- 所有阶段共用一个 hard/search-stop deadline；
- planner tree 在下一阶段前析构；
- final replay 与 prefix 成本精确。

**实现：**

- controller 只放在 `dd_planner.cpp`；
- tau、upper、compile、waves、matching、segments 和 replay 串成唯一流程；
- 每个 dispatch epoch 都以“除 locked robots 外的所有 robots × 当前 wave
  全部 PENDING tasks”构造新矩阵，不沿用未 Lift 的硬 assignment；
- `TAPFPlanner::solve()` 一旦返回首个有效 completion event：
  - COMPLETED 更新 shadow；
  - carrying 转为 locked；
  - 未 Lift 全部回 PENDING；
  - 只保留 soft continuity anchor；
- 若同一 joint step 有多个 Drop，一次性完成全部对应 tasks，然后只重匹配一次；
- wave 尚有 PENDING/CARRYING 时立即进入下一 dispatch epoch；只有 wave ledger
  全部 COMPLETED 时才检查 `expected_after` 并进入下一 wave；
- 不在中途 original goal 时提前返回；
- 不做 projection repair 或任何 fallback。

### P8：CLI、benchmark 和 telemetry

**RED：**

- C++ CLI 接受 `brd`，未知 mode 仍失败；
- Python runner 接受 `carrier_brd`；
- same dataset/seed/timeout/validator/weights；
- BRD metrics 缺失或格式非法时 runner 失败；
- full benchmark review gate 继续拒绝未批准运行。

**实现：**

- `tools/dd_benchmark.cpp` 增加 `brd`；
- `benchmark/run_benchmark.py` 增加 `carrier_brd` 映射及 BRD fields；
- `DDStats`/stdout 增加设计 §12 全部 `brd_*` telemetry；
- 不修改 quick/full manifest 成员和 timeout。

---

## 5. 构建与验证矩阵

### 5.1 每个阶段的最小回归

```bash
cmake --build build -j2 --target test_carrier_brd test_tapf_compat \
  test_tapf_planner test_dd_carrier test_dd_dispatch
./build/test_carrier_brd
./build/test_tapf_compat
./build/test_tapf_planner
./build/test_dd_carrier
./build/test_dd_dispatch
```

根据修改范围追加相关既有 test；不得以窄测试代替相关 regression。

### 5.2 全部代码测试

```bash
cmake --build build -j2 --target test_all dd_benchmark
./build/test_all
python -m pytest benchmark/tests
```

### 5.3 quick benchmark

相关代码测试全部通过后才运行：

```bash
python benchmark/run_benchmark.py \
  --benchmark-tier quick \
  --methods carrier_brd \
  --out-dir benchmark/results_quick_carrier_brd_20260906
```

若 quick 发现 bug，先新增 protected regression test，观察 RED，再修实现。

### 5.4 full benchmark gate

禁止提前运行 full。只有以下证据齐全后：

1. implementation 完成；
2. 全部相关 C++/Python tests 通过；
3. quick 77 完成；
4. 最终 diff 清理；
5. 新的独立 GPT-5.6 Sol/high reviewer 明确 `APPROVE`；
6. approval JSON 绑定当前 `benchmark/full_benchmark.json` 和最终 binary SHA；

才允许运行：

```bash
python benchmark/run_benchmark.py \
  --benchmark-tier full \
  --review-approval <approved.json> \
  --methods carrier_brd \
  --out-dir benchmark/results_full_carrier_brd_20260906
```

---

## 6. Diff 与语义审计

每完成一个 phase 检查：

```bash
git diff --check
git status --short
git diff -- CMakeLists.txt lacam tests tools benchmark/run_benchmark.py
```

逐项回答：

- 新增代码是否在真实 `carrier_brd` execution path 中使用；
- 是否复制了 LaCAM OPEN/CLOSED/constraint loop；
- production 默认参数是否保持原行为；
- shelf-free TAPF 是否自然退化而非 feature-flag fallback；
- 是否出现 benchmark-instance 名称、seed 或尺寸特判；
- 是否扩大或弱化了 protected test；
- 是否把 timeout 当 infeasible；
- 是否在 first completion 后错误继续旧 segment；
- 是否在 Lift 前错误锁定 assignment，或在 Lift 后错误释放；
- 是否在 raw frozen tasks 完成前返回 incidental goal。

---

## 7. 预期提交切片

1. `test(brd): freeze representative cases and shared-kernel RED tests`
2. `refactor(lacam): share DFS control loop without MAPF behavior change`
3. `feat(brd): add exact Carrier BR upper domain`
4. `feat(brd): compile immutable frozen task waves`
5. `feat(rho): add all-pending admission and bounded cardinality prepass`
6. `feat(tapf): add validated arbitrary root and event contract`
7. `feat(tapf): stop fixed-contract search at first completion`
8. `feat(brd): add completion-event controller and replay`
9. `feat(benchmark): expose carrier_brd metrics and quick protocol`
10. `docs(report): publish reviewed benchmark report`

提交边界可以因编译依赖微调，但不得把 test RED、实现和 benchmark 结果混成无法
审计的大提交。

---

## 8. 最终汇报物

最终网页至少包含：

- 算法从 tau→upper→waves→matching→lower→return 的一张流程图；
- R6/R7 两个 completion-event 例子，解释“未 Lift 重匹配、已 Lift 锁定”；
- `carrier`、`carrier_b0`、`carrier_b1`、`carrier_brd` 的 solved/runtime/
  makespan/work 对比；
- upper、matching、lower segment 和 cleanup 时间拆分；
- 失败原因分布；
- no-pick/place compatibility 结果；
- 已知不完备性和没有实现的优化；
- benchmark protocol、manifest SHA、binary SHA 和 reviewer approval。

网页使用项目现有静态可视化框架；能直接打开时不启动临时服务器。完成后由独立
subagent 检查链接、数据、中文表达、移动端布局和结论是否与结果一致。

---

## 9. 当前实现与验证记录（2026-09-06）

当前代码已经完成 P1–P8 的主闭环：

- MAPF 与 Carrier BR upper 共用 `search_kernel.hpp` 中同一 LaCAM DFS control
  loop；
- upper 生成 labeled shelf path，并编译为 immutable transfer waves；
- lower 每次复用 `TAPFPlanner::solve()`，只搜索到下一个 Drop completion
  event；
- 任一 task Drop 后，已 Lift pair 保持 hard lock，其余全部 free robots 与
  当前 wave 全部 PENDING tasks 立即重新匹配；
- matcher 保留 all-PENDING rows，并显式区分 Hall deficiency、合法 dummy 与
  deadline cutoff；
- tau、PairCost、canonical Hungarian、partial Task-BR、upper 和 lower 共用
  同一个 deadline probe；
- upper task materialization、frozen replay、raw replay、goal-prefix
  normalization 和最终 cost replay 也在内部轮询同一个 deadline；
- controller 只扣一份
  `min(1500 ms, max(20 ms, 15% × total budget))` 的 finalization reserve；
- 失败结果仍完整输出 BRD phase telemetry 和 `brd_exit_reason`。

验证证据：

```text
C++ aggregate tests          361 / 361 passed
Python aggregate tests       187 / 187 passed
completion-event controller    4 / 4 passed
deadline/finalization tests     7 / 7 passed
quick 77                      43 / 77 solved
quick wall time               27.6 s
quick solver-time sum        305.1 s
full 509                     326 / 509 solved
full wall time               115.8 s
full solver-time sum        1525.8 s
```

最终 quick 目录：

```text
benchmark/results_quick_carrier_brd_20260906_r5
```

其 binary SHA-256 为
`e1261800dd6454b2f02e745aef7e648df3610301d61999e758ec06bb15b556de`。
相对 r4，77 个 case 的 success、makespan、weighted work、plan SHA 和
`brd_exit_reason` 均无变化；失败阶段仍为
`UPPER_TIMEOUT=16 / SEARCH_TIMEOUT=16 / DISPATCH_TIMEOUT=2`。solver-time
sum 的 304.9 s → 305.1 s 属于 wall-clock 波动，wall time 均为 27.6 s。

第一轮最终代码审查曾因 materialization/final replay 没有内部 deadline probe
而 `REJECT`。该唯一 blocker 已按 TDD 修复，并新增 3 个 protected regression
tests。新的独立 GPT-5.6 Sol/high reviewer 随后明确 `APPROVE`，审批文件为：

```text
benchmark/review_approval_carrier_brd_20260906.json
```

在审批绑定的同一 binary、suite 与 509-case corpus 上，正式 full 结果为
`326/509 solved`。与最新同 corpus production `carrier` 的 `479/509` 比较：

```text
共同成功                         323
严格 (T,W) 更好 / 相同 / 更差     41 / 17 / 265
BRD 新增成功 / 丢失成功             3 / 156
shared_pool                    215 / 216 solved
singleton                       68 / 216 solved
```

183 个失败例的 BRD 阶段分布为：

```text
SEGMENT_TIMEOUT       138
UPPER_TIMEOUT          16
SEARCH_TIMEOUT         16
SEGMENT_EXHAUSTED      11
DISPATCH_TIMEOUT        2
```

这说明 completion-event 重匹配语义已经正确落地，但“先冻结完整 upper shelf
plan，再逐个 Drop 做 first-feasible lower segment”的分解基线会失去 production
planner 的完备性与质量，尤其是在 singleton goal 上。全量重匹配本身不是主要
耗时：失败例的 lower-segment P50 为 `8510.45 ms`，matching P50 仅
`0.65 ms`。

最终汇报产物：

```text
benchmark/viz_web/carrier_brd_final_report_20260906/index.html
benchmark/viz_web/full_benchmark_carrier_brd_20260906/index.html
benchmark/viz_web/full_comparison_rho_v2_vs_carrier_brd_20260906/index.html
```

full dashboard 含 326 个本轮 plan SHA 校验后生成的动画。首次网页审查因动画页
缺少移动端响应式布局、测试计数过期而 `REJECT`；两项均按 TDD 修复，326/326
动画重新生成，最终独立 GPT-5.6 Sol/high 网页复审明确 `APPROVE`，并确认
1905 个链接无断链、报告统计与源 CSV 一致。

---

## 8. Pair/tau incremental Hungarian 共享核心修正（2026-09-08）

本修正不新增 assignment 算法。它把现有
`TAPFAssignmentState` 的 ITA primal/dual 增广实现抽成唯一共享核心：

```text
tapf_assignment.hpp shared incremental Hungarian core
├── TAPFAssignmentState integer/tie adapter
└── PairCost long-double/INF/deadline adapter
```

Pair lazy certificate 的 execution path 保持在
`build_lazy_pair_cost_assignment()` 内：PairCost 精算或 commitment 改变后只
repair 对应 target rows；forced-edge certificate 复制当前共享状态、限制一个
row 后调用同一个 repair。禁止在 `carrier_assignment.hpp` 保留第二份
augment/dual-update 循环。

Pair edge 的 lower bound 改为三级按需精化：

```text
cheap geometric bound -> 8-step prefix bound -> exact PairCost
```

新 upper epoch 中 dependency 失效的边只退回 cheap bound；只有当前 assignment
选中或 forced certificate 仍可能挑战 incumbent 的边才逐级提升。这样
incremental 不只复用 Hungarian state，也避免在 Hungarian 之前重新编译全部
失效候选边。

开发顺序：

1. 新增 shared-core parity test，并在共享核心尚不存在时确认编译 RED；
2. 抽取现有 `TAPFAssignmentState` 核心，先验证原 TAPF assignment parity；
3. 把 Pair adapter 改接共享核心，保留现有 full-solver randomized oracle；
4. 运行相关 C++ tests 和完整 C++ regression；
5. 只使用固定 quick 77、每 case 10s 做 benchmark；最终 diff review 前不运行
   full 518。

最终验证结果：

- C++ `test_all` 425/425 通过；Python benchmark tests 全部通过；
- quick 仍为 47/77，77 例的成功状态、makespan、SOC 和动作计数逐例一致；
- quick 的 Pair rollout steps 从 3,119,275 降到 1,343,788；
- 独立 GPT-5.6 Sol/high 审查 `APPROVE` 后运行 full 518；
- full 从 485/518 提到 487/518，没有丢解；40×40 dense-channel 从
  6/9 提到 8/9；
- 共同成功的 485 例按 `(makespan, weighted SOC)` 比较为
  1 better / 484 equal / 0 worse；
- 全量 Pair rollout steps 从 7,257,391 降到 4,474,931，累计 solver time
  从 3563.1 秒降到 3545.8 秒；
- b4/r32 和 b9/r16 由 10 秒 timeout 变为成功；b9/r32 仍 timeout，
  30 秒诊断在 24.146 秒取得首解。

最终汇报：

```text
benchmark/viz_web/pair_staged_bounds_final_20260908/index.html
benchmark/viz_web/full_benchmark_pair_staged_bounds_20260908/index.html
benchmark/viz_web/full_comparison_pair_shared_vs_staged_bounds_20260908/index.html
```
