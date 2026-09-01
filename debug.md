# Carrier-LaCAM 调试与回归契约

状态：2026-09-01，与 `design_final.md` v3.0（闭环 task/`tau_guide`
guidance 设计）对齐。

实现基线：当前 `lacam/src` 是 v2.2 生产行为（request/cell 语义的
guidance）。本契约分两类条目：**[基线]** 钉住当前已验证行为，任何
改动都必须维持；**[v3.0]** 是 task 层落地时立即生效的强制约束。两类
同时有效——v3.0 的实现不得以破坏 [基线] 条目为代价。

## 1. 生产路径

```text
solve_carrier_lacam                        (dd_planner.cpp)
  -> dynamic guidance: TAPFPlanner::solve -> repair_carrier_plan
  -> if multi-goal and time remains:
       freeze terminal shelf-goal assignment
       TAPFPlanner::solve from root -> repair_carrier_plan
  -> choose lower-SOC valid candidate (tie/failure -> first pass)
  -> apply_ops full replay
```

guidance 内部数据流（v3.0 目标形态；现状见 design_final §3-§7 的
[现状] 段）：

```text
X -> tau_LB (admissible h)
  -> compile_frontier_task / C_guide -> tau_guide
  -> build_tasks(X, tau_guide)
  -> rho: free robot -> TaskId
  -> Carrier-PIBT operator order -> constraint tree -> apply_ops
```

不得重新引入：

- 第二套 carrier search loop 或平行 planner；
- 不利用首解结构的 primitive-only root restart；
- production strategy environment switch；
- 未经 oracle 重放的计划后处理；
- **[v3.0]** 按"是否有 task 层"做模式检测切换旧算法——零 shelf /
  零 task 必须是数据结构自然为空的逐位退化；
- **[v3.0]** 把 `free_goal / target_next / parking_cell` 从"task 阶段
  派生缓存"升级回独立 assignment 来源。

## 2. 允许的环境输入

```text
DD_ALPHA
DD_BETA
DD_GAMMA
DD_DELTA
DD_DEBUG_DUMP
```

前四项是数值 objective，最后一项只输出失败诊断。静态测试
`TestAblationContract`（`benchmark/tests/test_tools.py`）钉住该集合。
权重必须**有限且非负**：非法值在共享 parser `load_solver_weights`
处抛出（`dd_weights.rejects_negative_and_non_finite_env_weights`
钉住；2026-09-01 review 修复）。
v3.0 的 `lambda`（robot realization 权重）等新参数默认编译期常量；
若要暴露必须作为数值输入先过协议评审，禁止布尔/枚举开关。

no-following 和 strict cache 只允许作为显式测试参数：

```text
apply_ops(..., allow_following=false)
PathCache(/*strict=*/true)
```

## 3. 关键不变量

物理与搜索层 **[基线]**：

1. `apply_ops` 是 joint transition 的最终裁决者；fully constrained
   joint op 不受 guidance 干预。
2. Python validator 必须重放每个 benchmark success。
3. `tau_guide / task / rho / path / park / cooldown` 只改 ordering，
   不进 `SearchKey`，不改 goal condition，不永久删除合法 successor。
4. `constraint_order` 创建后冻结；livelock reguide 只扰动 PIBT
   preference（类内重排），futile-lift 只降序不删除 `Lift`。
5. `SearchKey` 只含 physical state（`Config` + `ShelfState`），不含
   guidance。
6. macro 只在每个 search pass 的首解前生成（`macro_after_first`
   恒 0）。
7. rollout probe 回收前后必须清除 address-keyed scratch（逐步
   `invalidate_carrier_scratch`）。
8. 零 shelf 输入逐位退化为原 TAPF；singleton goal-set 退化为固定
   goal。

输出修补层 **[基线]**：

9. output repair 只能返回原计划，或严格更短**且加权 SOC 不增**的有效
   goal plan（`valid(raw) => valid(returned)`，per-bridge SOC 守卫 +
   最终整体重放兜底；2026-09-01 review 修复）。
10. shelf-projection cut 的两个端点必须全部 shelves grounded。
11. robot bridge 结束后的 labeled robot configuration 必须与原片段
    终点相同（bridge 期间 shelf 不动）。

两遍与 assignment 层 **[基线]**：

12. fixed-assignment restart 与第一遍共享同一个 10 秒总 deadline。
13. 第二遍 goal 必须来自第一遍合法终态，且只在原 eligible set 内。
14. 第二遍失败、持平或更差时必须返回第一遍。
15. shelf-goal matching 在 primitive/loaded motion 中复用 parent
    assignment，只在 drop/task boundary 或定向 livelock repair 重算
    （v3.0 改为每 node 重评 `C_guide` 后，本条替换为不变量 20/21）。
16. settled lock 必须可扩展成完整 matching，否则按两级 fallback
    重开；carried target 保留 in-flight goal。
17. completed target 在 path 层仍是普通可逆 blocker，不能成为绝对
    障碍（曾使 gate 36/68 -> 35/68，已证伪）。

task/guidance 层 **[v3.0]**：

18. `tau_LB` 只含可证明下界并且是 `h` 的唯一来源；robot realization、
    execution price、hysteresis、taboo 只进 `C_guide / rho`。
19. task 是 `MoveShelf(s, from -> to, root = b -> g)`；robot 绑定
    稳定 `TaskId`，不是 pickup cell。labeled target 的完成条件是纯
    物理谓词；匿名 shelf 的 `s` 用当前 cell 标识（等价类），完成由
    被指派 robot 的 custody episode（Lift@from 的 `kappa=ANON` 段的
    Drop 落点）判定。custody token 只在 task metadata——禁止给
    `Q_anon` 加身份（破坏 canonicalization）。
20. 未被接手的 task 随 `tau_guide` 立即改变；robot 接手后对该局部
    task 保持 soft commitment 直到完成或失效；Lift 后 `kappa` 是唯一
    hard commitment。
21. duplicate node 被 rewire 到新 parent 后，必须重建 hysteresis
    anchor、task 和 `rho`，不能沿用旧 parent 的 guidance。
22. one-empty 输入的 ready task 在编译时就是
    `MoveShelf(s, u -> current_empty_cell)`，不允许 Lift 后临时找
    停车点。
23. `funcPIBT` 的 operator 顺序从 `rho[r]` 的 task 阶段推导；
    `free_goal` 等字段只能是派生缓存。

## 4. 输出修补调试

`DDPlanRepairStats`：

```text
exact_loops
projected_loops
bridge_steps
steps_removed
```

若修补没有生效，依次检查：

1. raw plan 是否能由 `apply_ops` 重放并到 goal（重放失败直接原样
   返回）；
2. 重复 projection 的两个端点是否全部 grounded；
3. bridge 是否严格短于被替换段（等长不采用）；
4. bridge 加权 SOC 是否超过被替换段（超过则拒绝该 bridge——按步数
   更短但按 SOC 更贵的桥不接受；2026-09-01 review 修复）；
5. 1/2 robot 的 shortest bridge 是否找到（1 robot 贪心下降、2 robot
   exact A*）；
6. 多 robot trajectory projection 是否仍满足 lower-deck R1/R2；
7. 最终 full replay 是否触发 fallback（返回原计划）。

禁止通过跳过最终 replay 来"修复"命中率。

## 5. 自动策略调试

固定结构常量（`carrier_guidance.hpp` / `tapf_planner.cpp`）：

```text
macro cap                  MACRO_CAP = 64
macro target limit         MACRO_TARGET_LIMIT = 64
guidance refresh steps     GUIDANCE_REFRESH_STEPS = 8   (rollout 内)
assignment exact limit     ASSIGNMENT_EXACT_LIMIT = 256
active target limit/cap    ACTIVE_TARGET_LIMIT = 256 / ACTIVE_TARGET_CAP = 64
assignment hysteresis      ASSIGNMENT_HYSTERESIS = 2    (tau 与 rho 共用)
livelock window            LIVELOCK_WINDOW = 24         (revisit 重导阈值 8)
clear chain length         CLEAR_CHAIN_K = 3
blocked-cell path penalty  LAMBDA_BLK = 8
futile repeat limit        FUTILE_REPEAT_LIMIT = 3
futile memory window       max(64, 8*|V|)
deadline cleanup reserve   min(1000ms, max(100ms, 10% limit))
```

这些值不是运行开关。调整任何值都必须重新运行 68 例 BRaP-pool gate，
尤其检查：

```text
h4w10_a5_e1_R1_seed0       makespan 1053
h10w10_a12_e3_R1_seed1     makespan 2620
small suite                36/36
all suite                  36/68
```

逐拍重建 guidance 已验证会使 dense B0 失败，因此 rollout 保留 8 步
刷新周期，同时每拍失效 occupancy scratch
（`dd_integration.rollout_steps_match_fresh_generation` 钉住）。

## 6. Guidance / task 层调试

诊断计数（`DDStats`）：

```text
tau_change_builds / tau_pair_changes      shelf-goal 改写频度
rho_change_builds / rho_pair_changes      robot 任务改写频度
tau_time_ms / guidance_time_ms            每节点 guidance 预算
guidance_builds / path_recomputes / path_cache_hits
futile_lift_demotions
macro_* / rollout_*                       macro 注入与终止原因
assignment_restarts / _second_solved / _improvements
```

症状检查单：

1. **assignment churn 回归**：`tau_pair_changes` 相对节点数暴涨
   （v2.2 参照：20x20 曾 86,317 次，commitment 后 2,757）——检查
   preserve/commitment 边界与 hysteresis anchor 是否在 rewire 后
   丢失（不变量 21）。
2. **guidance 预算超支**：大图 `tau_time_ms + guidance_time_ms`
   接近 deadline（80x80 历史测量 ~8.9/10 秒）——robot realization
   项必须增量维护；先查 `path_recomputes` 与 active-target cap 是否
   失效。
3. **livelock 循环**：`waitfor` 探针（`dd_waitfor_cycle_robots`）
   返回非空且 taboo repair 未打破——检查 rho taboo 与单行 tau 释放
   的 epoch 轮转。
4. **task 身份漂移 [v3.0]**：同一搬运在 approach 中途换 `TaskId`
   （soft commitment 破坏）——用 §7 测试 4 的生命周期断言定位。
5. **h 污染**：任何 guidance 改动后 `dd_anytime.
   admissible_h_never_exceeds_true_cost` 失败即为 execution 项漏进
   `tau_LB`（不变量 18），立即回退。

失败诊断输出用 `DD_DEBUG_DUMP`；最深节点链在失败时经 best-effort
通道返回（`deepest_config / deepest_tau`）。

## 7. 测试

```sh
cmake --build build -j 16
./build/test_all --gtest_color=no

PYTHONPATH=benchmark \
  /tmp/dd-lacam-pytest/bin/python -m pytest benchmark/tests -q \
  -n 14 --dist=load
```

测试依赖固定在 `benchmark/requirements-test.txt`。系统 Python 若无
pytest/xdist，用临时 venv，不修改系统环境。Python 全套中的外部
solver 固定 10 秒内部预算；subprocess 只留 5 秒启动/终止余量。
14 workers 与 benchmark 协议一致（16 物理核留 2）。

### 7.1 现有保护性测试（基线，不得回归）

| 测试 | 责任 |
|---|---|
| `dd_plan_repair.*` | projection cut、bridge、不可缩短 fallback |
| `dd_integration.rollout_steps_match_fresh_generation` | scratch 失效 + 8 步 guidance 刷新协议 |
| `dd_integration.search_not_dominated_by_own_rollout_on_dense` | 角色相关 PIBT 预约语义 |
| `dd_nofollow.*` | 显式 conservative oracle 参数 |
| `dd_g1_conformance.*` / `dd_golden.corpus_agreement` | 约束树穷举与 oracle/validator 一致 |
| `dd_rewire.*` | 首解计划合法性和 objective 记账 |
| `dd_tau.*` | goal matching、admissible h、taboo、dst cache |
| `dd_tau.carried_target_keeps_inflight_goal_commitment` | task-episode commitment |
| `dd_tau.settled_pool_goal_*` | settled/carried 冲突与可扩展 fallback |
| `dd_tau_guidance.settled_target_remains_a_reversible_blocker` | completed target 是普通 blocker |
| `dd_goalset.dynamic_first_solution_restarts_with_fixed_assignment` | 自动第二遍 |
| `dd_goalset.singleton_assignment_skips_second_search` | singleton 结构性跳过 |
| `dd_anytime.admissible_h_never_exceeds_true_cost` | h 下界性 |
| `dd_anytime.macro_disabled_after_first_solution` | macro 首解前 only |
| `test_tapf_compat`（全部） | 零 shelf 逐位退化 |
| `TestAblationContract`（Python） | 无策略环境开关 |
| `dd_goalset.finalize_rejects_duplicate_target_starts` | 重复 target start 拒绝（2026-09-01） |
| `dd_weights.rejects_negative_and_non_finite_env_weights` | 权重输入校验（2026-09-01） |
| `dd_planner.exhausted_search_is_not_reported_as_timeout` | 失败分类：耗尽 != 超时（2026-09-01） |
| `dd_plan_repair.repaired_soc_never_exceeds_raw_under_any_weights` | 修补 SOC 不增契约（2026-09-01） |
| `DuplicateTargetStartTest`（Python） | loader 与 C++ finalize 对齐（2026-09-01） |

### 7.2 v3.0 必测（design_final §12；落地时补齐）

| # | 要求 | 状态 |
|---|---|---|
| 1 | 零 shelf 与原 LaCAM-TAPF 逐位一致 | 已覆盖（`test_tapf_compat`） |
| 2 | `\|G_b\|=1` 退化为 fixed-goal carrier | 已覆盖（`dd_tau.singleton_*`、`dd_goalset.singleton_*`） |
| 3 | target 不动、robot/vacancy 变时 `tau_guide` 可改变 | 待写（robot-realization cost 落地时） |
| 4 | 同一 `TaskId` 连续经历 approach/Lift/carry/Drop | 待写（ManipulationTask 落地时） |
| 5 | one-empty ready task = 相邻 shelf 移入 vacancy | 待写（compile_frontier_task 落地时） |
| 6 | execution feedback 不进 admissible `h` | 半覆盖（`hysteresis_is_tie_break_only`、`rowwise_taboo_does_not_bias_admissible_h`）；robot 项落地时同型扩展 |
| 7 | 所有返回计划过 C++/Python replay | 已覆盖（repair 测试 + validator + golden corpus） |

待写测试在对应机制合入的同一 CR 内交付，不允许"先合机制后补测试"。

## 8. Benchmark gate

```sh
python3 benchmark/run_benchmark.py \
  --instances benchmark/instances_brap_pool \
  --out-dir benchmark/results_<new-dir> \
  --methods carrier --timeout 10 --jobs 14
```

运行前确认没有 `DD_ALPHA..DD_DELTA` 残留和其他高负载任务。复跑必须写
新目录，保留 `rows.csv`、`timing.json`、`work/*.plan`。结构消融用
`run_ablations.py --out-dir <new-dir>`（固定 10 秒、14 jobs、seed 0、
单位权重、默认 following，复用 Python validator）。主 runner 显式
使用单位权重；非单位实验只能用 `--weights` 开独立结果轴，且不与
native-objective 外部方法混跑。

### 8.1 行为不变改动（重构、清理、修补优化）

接受条件（对照 `results_task_commit_final`）：

- success `36/68`，<=10x10 `36/36`；
- 成功计划 36 个，全部通过 Python validator；
- 成功例总 makespan 34,860、SOC 59,907；
- **决定性判据**：68 行 success/status/makespan/SOC 零差异 + 36 个
  成功 plan 逐字节一致；
- assignment restart 18 attempted / 6 improved；`second_solved` 为
  18 或 17——`h10w10_a12_e3_B_seed1_pool` 的第二遍在 8.9s/10s 贴线
  且其结果按 lower_SOC 被丢弃，是否在时限内完成对机器状态敏感（改动
  前后二进制在同一机器状态下一致），不影响任何输出字段；
- 相对 rootfix control 的 makespan/SOC 几何比约
  `0.917520 / 0.927884`；
- singleton/R1 输出计划逐字节不变；
- 失败分类：空计划仅当某遍真实到期才计 `timeout`，耗尽/generator
  失败计 `failed`（2026-09-01 review 修复）。

### 8.2 语义变更（v3.0 task/`tau_guide` 落地）

按 design_final §11.6 验收，开新结果目录并同时报告：

- success 不低于 `36/68` 与小图 `36/36`；
- 共同成功集 makespan/SOC 几何均值 + 逐例改善/持平/恶化，恶化逐个
  解释；锚案例 `h4w10_a5_e1_R1_seed0`（基线 1053）、
  `h10w10_a12_e3_R1_seed1`（基线 2620）单列；
- 首遍轨迹允许改变；singleton/R1 逐字节等价不作为语义变更的验收项，
  但零 shelf（`test_tapf_compat`）仍必须逐位一致；
- 三组消融 A（frozen tau vs `tau_guide`）、B（request cell vs
  ManipulationTask）、C（shelf-only vs robot-realization cost）以
  结构 runner method 同协议报告；
- `tau_time_ms / guidance_time_ms` 预算报告（§6 症状 2）。

### 8.3 可视化对照

当前基线动画（同一 `h8w10 e20 B seed1`）：

```text
benchmark/viz/task_commit_before_h8w10.html   rootfix control, 233 steps
benchmark/viz/task_commit_fixed_h8w10.html    selected fixed plan, 168 steps
```

run 内部统计是 first 241 -> second 168；runner 只持久化最终候选，
before 动画使用可独立重放的 rootfix control，不冒充当前 first。

## 9. 已知边界

- 20x20 以上 dense BRaP：10 秒内 0/32；`tau_guide` 反馈针对
  assignment churn 与 task 不连贯，不是首解 horizon 的充分解；
- fixed-assignment restart 在首解后才触发，不能改善 0 首解实例；
- output repair 不减少 raw first-incumbent search horizon；
- 1/2 robot bridge 最短，更多 robot 只保证合法缩短
  （`h4w10` Python 原型 719 vs 当前 1053，bridge 有余量）；
- search-level shelf-equivalence merging 仍是研究项，必须同时解决
  robot state reconstruction，不能只改 hash；
- `targets > 256` 的 row-wise guidance（非单射 `tau_hint`）没有 gate
  实例覆盖，只有单元测试保护；
- 每 node 重评 `tau_guide` + robot realization 是新的常数成本来源，
  受 §6 症状 2 与 design_final §11.6 第 6 条约束。
