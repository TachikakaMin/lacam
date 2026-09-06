# Carrier-LaCAM v5 Makespan 实现任务清单（debug.md）

状态：2026-09-05 v5，v5 production 实现、受保护测试迁移、Testcase C、
全量代码测试、固定 quick 77、E5 独立消融和正式 full 509 均已完成；
pre-full 与首次发布审查发现的阻塞项已按 RED→GREEN 修复。正式 full 为
479/509，新增 factorial 为 432/432；最终网页和本文件纳入同一独立终审。
设计起点 commit `03e99ba` 的冻结基线为 241 C++ / 140 Python 全绿。
本文件同时保留 coding 清单与实际证据；完成项已打钩，未完成的可选消融
明确保留为空。
所有阶段遵循 `test -> RED -> implementation -> GREEN -> benchmark ->
regression -> debug`；protected test 迁移先按 `rules.md` 走独立
reviewer。

## 三条不可违背的测试契约（每阶段合入前必须全绿）

```text
1. 路线暂时找不到，搬运 episode 仍然存在。
   （route helper 零预算/暂时受阻 + 合法 Wait 后，
     TransferId、carrier、endpoint 保持；恢复后继续同一 episode）
2. 未来路线相交（含相同的 transit first leg），搬运任务仍然可以被
   接手和重新协调，不得在 compiler 接纳或 ready 过滤中整体消失。
3. 所有方案最终都按同一个首次完成前缀上的 (T, W) 词典序比较；
   search / rewrite / 两遍 / repair 共用同一个比较器定义。
```

---

## Stage A：契约与证据冻结（无行为改动）

- [x] A1. 冻结证据：`git rev-parse HEAD`、`design_final.md` 与
      `Carrier_LaCAM_v5_Makespan_Design.md` 的 sha256、release 基线目录
      （`benchmark/results_release_77_20260904`）。
- [x] A2. Testcase C（S2 地图 + 31 行参考计划 YAML）与本轮微例加入
      `tests/fixtures/` 或 `benchmark/`，字节冻结；先只作数据。
- [x] A3. 测试分类清单（填本文件附录）：物理/交付契约（保留：
      storage-only Drop、successor 完备性、replay、strict deadline、
      zero-shelf compatibility、SearchKey）vs 将被修改的策略契约（迁移：
      `test_dd_storage_transfer_claims.cpp` 的 route-claims、
      `test_dd_plan_repair.cpp` 的 SOC 非增、`test_dd_objective_*` 的
      标量目标、rho ready-only/身份断言、custody route-suffix 断言）。
- [x] A4. 全量基线存档：`cmake -B build && make -C build -j &&
      ./build/test_all`；`PYTHONPATH=benchmark python3 -m unittest
      discover -s benchmark/tests -p 'test_*.py'`。

## Stage B：成本契约（design_final §2.2、§5.2、§12、§13.1/13.7/13.8）

目标接口与表示：

- [x] B1. 显式 objective/cost 契约参数（`TAPFSearchConfig` 或
      instance）：调用方声明目标。zero-shelf 原 TAPF 调用方请求旧标量
      objective（逐位兼容）；carrier 调用方（`dd_planner.cpp`）请求
      `(T,W)`。同一 engine 按契约产生 edge cost 与 h；禁止按“有没有
      货架”猜测，禁止默认入口在迁移中暗中改目标。
- [x] B2. `PlanCost{int64_t ticks; int64_t work}`：work 统一采用
      `10^-6` 微单位固定点；入口只接受恰好落在微单位网格上的有限非负
      权重，四个权重先转 `int64_t`，search、rewrite、两遍、repair、
      replay 和 stats 全程只做整数加法与严格词典序比较。epsilon 不进
      OPEN 排序、全局比较器或任何接受条件（近似相等不传递）。
- [x] B3. 权重域入口校验：`alpha,beta,gamma,delta >= 0` 且有限；
      `DD_ALPHA..DD_DELTA`（`carrier_detail::load_solver_weights`）解析
      后拒绝越域或非微单位网格值，并由唯一 parser 规范化成公共
      `SolverWeights` 整数表示。`h_W` 的下界论证依赖非负权重。
- [x] B4. `TAPFNode.g/h/f` 与 `SearchEdge.physical_cost` 迁移为
      `PlanCost`；`TAPFSearchConfig.incumbent_init` 改两分量上界。

成本产生点与比较点：

- [x] B5. `get_edge_cost()`（l.1069）：普通 joint transition 记
      `(1, work)`（全 Wait 也是 1 tick）；macro edge 在注册处记
      `(trace.size(), work_sum)`（`register_outgoing_edge` l.1012、macro
      分支 l.654–688、`carrier_rollout` l.1755 的 `cost` 同步）。
- [x] B6. h：`attach_carrier_guidance()`（h 安装 l.224–235）改
      `node.h = (h_T, h_W)`。`h_W` 沿用 `solve_tau_lb()`（l.694）；新增
      `h_T = max(h_bottleneck, h_work)`（§5.2）。mixed 输入必须先构造
      `h_TAPF_time = max_i min_goal d(C[i],goal)`（逐 agent 取 max，
      **不是** `get_h_value()` 的求和），再
      `h_ticks = max(h_TAPF_time, h_shelf_time)`；旧求和项只服务旧
      objective 调用方。
- [x] B7. 比较器统一接线：`solve()` incumbent 剪枝（l.629）、OPEN/f、
      `rewrite()`（l.977）、macro 接受、`f_pruned`/`g_relaxed`、
      `first_solution_*` 全部词典序；stats 记录 `(T,W)` 两分量。
- [x] B8. 首次 goal 前缀强制规范化：实现 `NormalizeGoalPrefix(plan)`
      （重放→第一个 goal 状态截断→T/W 只按前缀计）。所有交付、候选
      比较、`T = plan.size()` 读法都先规范化；macro 中途达 goal 必须
      注册到该 goal 状态的**前缀边**，不得保留长 trace 终态 key 只改
      cost。
- [x] B9. 两遍与 repair：`dd_planner.cpp` 候选比较（l.362 `soc2<soc`）
      改 `(T,W)`（先各自 NormalizeGoalPrefix）；`dd_plan_repair.cpp`
      接受条件（l.389/415）改同一比较器的词典序非增——与 search 完全
      相同的“更好”定义，无独立 epsilon 语义；replay 合法性与共享 pass
      deadline 不变。
- [x] B10. 测试（先 RED）：
      - `(31,93) <lex (54,90)`；同 T 比 W；比较器传递性抽查；
      - 全 Wait joint op = 1 tick / 0 work；k 拍 macro = k ticks；
      - 初态即 goal → `T=0`；macro 中途达 goal 截断且注册前缀边；
        goal 后匿名货架动作不计入返回计划 T/W；
      - repair 接受“少 T 多 W”、拒绝“多 T 少 W”；
      - h_T 小图 oracle（carried-at-goal 剩 1 Drop；grounded-at-goal
        为 0；injective 多 goal）；mixed 三 agent 各差一步
        `h_ticks = 1` 而非 3；小图穷举最优 T 验证不高估；
      - 权重校验拒绝负值/非有限值；
      - zero-shelf 调用方在旧 objective 契约下逐位兼容。
- [x] B11. 迁移 `test_dd_objective_*` 标量断言（A3 清单，reviewer 批准
      后）；单独记录本提交 dev 子集 + release 配对性能，不假定首解变好。

## Stage C1：任务连续性（design_final §3、§9.1、§10、§13.1/13.3）

- [x] C1.1 类型：`TransferKey{shelf, source, endpoint}`（不含 route）、
      branch-local `TransferId`；`Custody` 增 `original_endpoint`、
      `rebind_reason`（none/forced_deviation/no_route）、
      `route_status`（OK/TEMPORARILY_BLOCKED/BUDGET_EXHAUSTED/...）；
      `StorageTransfer` 身份判断收缩为 endpoint，route 降为可重算 hint。
- [x] C1.2 三谓词拆分（§9.1）：`custody_physically_valid()` 只保留
      `PhysicalBindingValid`（kappa/坐标/anchor/endpoint 可存储）；
      `EpisodeActive` 独立判定；route 一致性检查全部移入
      `RouteHintUsable`。**route 找不到 ≠ episode 失效**；
      `preferred_route/leg` 可为空。
- [x] C1.3 `recover_task_br_custody()`（l.2914）：WAIT/no-route 保持
      episode（custody 延续，置 `route_status`）；forced deviation 先试
      原 endpoint 重路由，失败保持 episode + no-route，仅必要时换
      endpoint 并写 `rebind_reason`；`loaded-unbound` 只留给无可恢复
      episode 的强制 Lift（constraint tree 强制举起无 assignment 货架）。
- [x] C1.4 最小 ExecutionView 对齐（§10）：active episode 去重、
      grounded/carried 分类（D0 只按坐标记录“需腾空 u”；carried
      occupant 关联真实 episode 的 vacate 事件，不派第二台车）；
      `active/fulfilled/pending/shadowed` 按 §10 精确语义实现；
      `fulfilled` 可因重新占据失效。
- [x] C1.5 task graph 合并规则：仅同 source+shelf+endpoint 且因果要求
      可协调才共享 task；同第一腿不同 endpoint 不伪合并
      （`TaskBRCompilerState` l.864 起的 merge 路径）。
- [x] C1.6 测试（先 RED）：
      - **契约 1 回归**：合法 custody + route helper 零预算 + 合法
        Wait → TransferId/carrier/endpoint 保持；预算恢复继续同一
        episode；
      - 同 endpoint 换 route：TransferId/custody 不变、LegId 更新；
      - forced deviation 优先回原 endpoint；无路时 episode 保持且不出现
        storage→transit→同 storage 往复（`same_origin_returns=0`）；
      - 同一 U、不同 kappa：cached D0 逐位相同、ExecutionView 不同；
        无第二 carrier、不丢未满足条件；
      - unbound 仅出现于强制无 assignment Lift。

## Stage C2：C 的运输修复（design_final §6.3/6.4、§8.1、§9.2–9.7、§13.3–13.6）

- [x] C2.1 删除下游过滤：`ready_tasks_with_custody()`（l.2757）尾部
      full-route claims 过滤（含 ready 互斥）移除；保留 endpoint 放置
      冲突与真实 occupancy/custody 去重；`ActiveTransferClaims`
      （l.2588）退役或降级为诊断。
- [x] C2.2 审计上游接纳：`TaskBRCompilerState/Transaction` 全部 transfer
      接纳条件——storage endpoint 放置冲突保留联合处理；非 storage
      first-step 的 `reserved_destination`（l.868/1009 等）只保留当前
      同拍动作选择意义，不得把“两个未来 first legs 共用一个 transit
      cell”的 transfer 挡在候选图外。只删 C2.1 不够。
- [x] C2.3 最小冲突组联合运输 helper（时序能力提前到本阶段）：输入
      `{当前 X, active/assigned transfers, 预测 departure, 候选轨迹或
      timed reservations, 有限回退预算}`；按冲突分组处理 2..k 个
      transfers；比较等待 / 等长绕路 / 通过顺序；输出每 transfer 的
      `完整 route hint | prefix hint | none`（§9.2 taxonomy）。不必
      覆盖整仓、不必最优；组内选择用局部 Score（组内完成时刻 + 被延后
      任务的剩余估计），失败只回 partial guidance。
- [x] C2.4 同 endpoint 重寻路入口 `reroute_to_endpoint(...)` 消费 C2.3
      的时序视图，**不是**只看当前 occupancy 的裸 BFS（否则 b4/b5 会
      重选原对向路线，直到进走廊才冲突）。
- [x] C2.5 `funcPIBT()`（l.1231）loaded+bound：首选 leg 消费动态
      route/prefix hint；无 hint 时按 §9.6 Wait/合法 Move，不贪心
      retarget；到 endpoint 后比较 Drop 与同棚 continuation。
- [x] C2.6 rho：保留 min-sum Hungarian（`match_ready_tasks()`
      l.3128），switch penalty/owner continuity 改按
      TransferKey/TransferId。
- [x] C2.7 迁移 `test_dd_storage_transfer_claims.cpp`（A3、reviewer
      批准后）：route-overlap 删除断言改时序协调语义；endpoint 冲突与
      corridor-Drop 禁止保留。
- [x] C2.8 测试（先 RED）：
      - **契约 2 回归**：两条路线相交但可错时 → 两个 transfer 都被
        接手；第一格是同一 transit cell、错时出发 → 都进入候选并先后
        通过（compiler 接纳 + ready 双层验证）；
      - 对向走廊（b4/b5 型）：进入无会车空间前完成协调（等长绕路或
        顺序化），不发生走廊内僵持；
      - 接口回归：C 的六个任务不被删除；固定 probe 复现原六行匹配；
      - horizon 不足：endpoint 超出 horizon 时返回 prefix hint，载货车
        不永久 Wait、不 corridor Drop。
- [x] C2.9 Testcase C 端到端：10s 预算，记录 T/W/首解时间；目标
      `T=31`（阶段验收线显式标注；未达标给瓶颈分析：dispatch 还是
      路线协调）。
- [x] C2.10 回归：storage 往复（`test_dd_storage_transfer.cpp`）、dense
      suite、release 77 例配对、9 例 warehouse 逐帧审计三指标为 0。

## Stage D：因果事件与受控准备（design_final §6.2、§7、§13.3）

- [x] D1. `CausalEdge{producer, must_be_vacated, consumer,
      consumer_event}`：整 transfer 等待细化为必要 vacate/enter 事件；
      仅需要机器人释放或前驱落地时加 Drop/RobotAvailable 依赖。
- [x] D2. 重占据失效：格子被重占后 consumer 进入前重验证；不用单调
      completed bit（测试：释放后再占据，等待者不得凭旧 release 进入）。
- [x] D3. readiness 三级：`Assignable/Preparable/MoveExecutable`
      （§7.1）；rho 消费 assignable；preparation admission 不夺关键
      前驱执行者、不堵其交通（§7.2）。
- [x] D4. 防自锁：单机器人不得举着 A 等无执行者的 B；preferred Lift 的
      admission 检查。
- [x] D5. 测试：多层 blocker chain 只等必要 cell 事件；机器人充足时
      清障与后继 approach 同拍并行；`empty_storage(U)` vs
      `storage_slack` 区分（货架进通道后源 storage 立即可作 endpoint
      候选）；“non-ready 永不 approach”断言迁移为“MoveExecutable 未
      满足不能执行该 move，Preparable 可接近”。

## Stage E：质量增强（design_final §8.2–8.4、§9.7、§12.2）

- [x] E1. 完成时间派工：`E(r,m) = 接手后完成时刻 + 剩余因果尾长`
      （事件 max 合并）；bottleneck matching 最小化 `max E`，阈值内
      min-sum 定次序；S 的选择计入未派任务延后。
- [x] E2. §9.7 完整 Score 比较：保留少量完整候选 frame，
      `Score = (T̂_all_targets, Ŵ, 稳定性)`；`T̂` 必须含未分配/暂停/
      有后续搬运的 root 的剩余估计（测试：漏计 paused root 的 frame
      不得胜出）；“有路但慢”的方案参与比较，不只在 NO_ROUTE 时回退。
- [ ] E3. （可选消融）有界 dispatch lookahead（§8.3）与 handoff 完成
      时间比较（§8.4）。
- [x] E4. 首解后改进：由**同一个求解控制器**管理剩余预算、可交付
      incumbent 与后续尝试（两遍作为其中一种候选尝试）；不新增第三套
      deadline/返回路径；第二候选超时保留第一份已验证 incumbent。
- [x] E5. 每项单独消融 + release 77 例配对：success、真实 T、W、首解
      时间、timed-helper 时间/展开数、owner handoff、causal/traffic
      waiting；SOC 回退披露但不推翻更小 T。

## 完成定义（Definition of Done）

- [x] 三条顶层测试契约各有专门回归并全绿；
- [x] C++/Python 全量 GREEN（新旧合并后）；zero-shelf 旧 objective 契约
      逐位 PASS；
- [x] 全代码只有一个 PlanCost 比较器定义（grep 验证无第二处词典序/
      epsilon 变体）；
- [x] Testcase C：合法、无 corridor Drop、报告规范化前缀上的 T/W；
      `T=31` 或附瓶颈分析；`W<=90` 仅在通过权威 validator 后作次级目标；
- [x] release 77 例配对：先 success/合法性再 common-set 质量；回退如实
      保留；
- [x] 无平行 planner、无实例名/feature-flag 分支、无 legacy fallback、
      无第三套 deadline 路径；
- [x] 经独立 GPT-5.6 Sol/xhigh 审查批准后运行 sealed full 509；正式
      rows/timing、479 个成功 plan 与 dashboard 均发布并校验；
- [x] design_final.md §20/§22 未被改写；新验证证据另立章节。

## 2026-09-05 实施证据

代码只扩展原 `TAPFPlanner::solve()`、`attach_carrier_guidance()`、
`funcPIBT()`、`apply_ops()`、两遍候选与 repair 链路。静态检查确认：
`PlanCost` 结构和严格 ticks-then-work 比较器各只有一处；
`TAPFPlanner::solve()` 只有一个定义；环境变量只用于通用非负权重和
debug dump，没有 instance/seed 分支或算法 fallback。

当前全量验证：

```text
./build/test_all --gtest_color=no
  285 / 285 PASS, 224.928s

cd benchmark && python3 -m unittest discover -s tests
  155 / 155 PASS, 301.829s
```

runner 对每个成功 carrier 方案再次用权威 validator 重放，并要求
`weighted_work_scaled` 与重放得到的精确微单位值逐位相等；LaCAM 模式还
要求 `best_work_scaled` 相等。B0/B1 只核对最终交付方案，不虚构不存在的
anytime incumbent。full approval 已升级为 schema v2，同时绑定 suite
definition、语义 corpus 与实际执行 binary 的 SHA-256。正式 full 已把
获批 binary 和全部 509 份 YAML 复制到只读 Linux sealed memfd，所有 worker
只执行这些快照；dispatch 前与发布 `rows.csv` 前均重新计算 binary/corpus
hash。旧 full 结果与 schema-v1 approval 继续隔离到
`benchmark/historical/pre_v5/`，只作历史记录。

主要回归文件：

* `test_dd_plan_cost.cpp`、`test_dd_makespan_heuristic.cpp`：B1–B10；
* `test_dd_transfer_episode.cpp`：C1 的 identity、route-status 与 episode
  连续性；
* `test_dd_storage_transfer_claims.cpp`、`test_dd_timed_transport.cpp`：
  C2 的 endpoint-only claims、错时复用、prefix/no-route、占用 transit
  仍保留 transfer，以及真实冲突；
* `test_dd_causal_preparation.cpp`：D 的事件重验证、preparation admission
  和单机器人防自锁；
* `test_dd_dispatch.cpp`、`test_dd_anytime.cpp`：E1/E2/E4 的 bottleneck
  dispatch、paused-root score 与同控制器 incumbent 改进；
* `test_dd_task_br_exact_matching.cpp`：lazy PairCost 的 8-step prefix
  lower-bound 与完整矩阵 assignment certificate。

Testcase C 使用
`warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0.yaml`。当前 production
输出经权威 validator 重放为 `T=31, W=93`，无 anonymous move、无 reversal，
plan SHA-256 为
`8a103b1a80ad24ab5889d1c158c5983009d7719663c28869414b5634e7e52c4d`。
用当前 binary 重新执行的独立 10s probe 为 `first_solution_ms=38`、
`deliverable_ms=7710.78`、`weighted_work_scaled=93000000`；
它达到 §17.3 的 31 拍 makespan 下界，不声称 W 最优。

固定 quick 运行位于
`benchmark/results_quick_v5_control_854df1_20260905`。协议 SHA-256 为
`a881292163ff2fcd2797cc83fddd8f9ebd12def79619586b940976bce07111c6`，
仍是 77 cases、10s/case、seed 0、unit weights、following allowed、
14 jobs；binary SHA-256 为
`854df1e692316017cbc26ab462ab3b22735d61caa08ff0645e28ad0cab4a574c`。
旧版与 v5 均为 `47/77`，成功集合完全相同。共同 47 例：

| 指标 | better/equal/worse | baseline sum → v5 sum | 几何比 |
|---|---:|---:|---:|
| makespan T | 29 / 3 / 15 | 19229 → 19749 | 0.940512 |
| weighted work W | 14 / 4 / 29 | 49014 → 54627 | 1.064569 |

因此只声称多数实例和几何均值上的 makespan 改善，不声称逐例单调，也不隐藏
总和与次级 work 的回退。v5 wall time 为 52.0s、solver-time sum 为 651.7s；
旧版分别为 29.4s、334.5s。额外运行时间主要来自首解后的有界改进尝试；
所有成功方案的最大 `deliverable_ms=9277.23`，仍在严格 10s 内。

E5 使用三份独立 Release 构建逐项移除 E1、E2 或 E4，串行运行同一固定
quick 77；没有在 production 中保留运行时开关。所有变体都为 47/77 且成功
集合不变：

| 变体（相对 control） | T better/equal/worse | T sum | W better/equal/worse | W sum | wall / solver sum |
|---|---:|---:|---:|---:|---:|
| E1 off | 14 / 22 / 11 | 17998 | 18 / 17 / 12 | 48932 | 52.1s / 653.7s |
| E2 off | 0 / 47 / 0 | 19749 | 0 / 47 / 0 | 54627 | 52.1s / 653.9s |
| E4 off | 0 / 33 / 14 | 20679 | 0 / 30 / 17 | 56324 | 28.9s / 324.4s |
| control | — | 19749 | — | 54627 | 52.0s / 651.7s |

这组数据不支持声称 E1 或 E2 在 quick corpus 上改善聚合质量：移除 E1
反而整体更好，E2 多 frame 仅改变一个 plan hash、T/W 完全相同。E4 则在
17 个方案上产生严格 incumbent 改进，没有丢失成功例，但约使用一倍
solver time。完整 patch
定义、binary hash、诊断计数和结果目录见
`benchmark/ablation_v5_20260905.md`。E3 仍是明确可选项，未实现无界
lookahead 或新的 handoff pipeline。

第一次 pre-full 独立审查给出 `REJECT`，阻塞项是：占用中的 canonical
transit 会在 compiler/ready 阶段错误删除任务；获批文件在校验后仍可被替换；
发布 proposal 的 planner provenance 仍指向旧 binary。修复后新增了
`compiler_keeps_root_when_canonical_transit_is_currently_occupied`、
`carried_transit_occupant_does_not_delete_assignable_transfer`、
sealed binary/YAML snapshot 以及 published-current-binary 回归。proposal
中四个实跑样例的 binary hash 现均为当前 production hash。该次 `REJECT`
不是 full 放行；后续取得新的独立 `APPROVE` 后才运行 509 例。

第二次独立 GPT-5.6 Sol/xhigh 审查在真实 execution path 中复现了另一处
阻塞：一个旧 anonymous custody 已搬入 transit，compiler 重锚定后当前
producer task 变为 `SHADOWED`，导致仍未 fulfilled 的因果边被误判为没有
executor，下游 target 无法 preparation。reviewer 先 `REJECT`，随后明确
批准用其精确 probe 替换较弱回归；该 protected test 先 RED，再由
`preparable_tasks_with_executors` 的 edge-local custody 校验修至 GREEN。
校验要求 physical carrier 位于待腾空格、custody/TransferId/carrier/
endpoint 与当前 producer 一致、route 可用且首腿离开该格；它不全局激活
`SHADOWED` task，也不提前 fulfill 因果条件。当前 C++ 总数为 285 项，
随后由新的独立最终审查绑定当前 binary hash 后才启动 full。

随后 GPT-5.6 Sol/xhigh reviewer 对 binary
`854df1e692316017cbc26ab462ab3b22735d61caa08ff0645e28ad0cab4a574c`
和冻结 suite/corpus 明确 `APPROVE`。首次正式 full 完成全部 509 个 solver
task，内部汇总为 `479/509`，但发布前的最后一次 binary rehash 调用
`Path.resolve()`，把 `/proc/<pid>/fd/<fd>` 跟随为不可重新打开的
`/memfd:approved-dd-benchmark (deleted)`，runner 以 exit 2 fail-closed。
失败目录 `benchmark/results_full_v5_854df1_20260905` 只有 479 个成功 plan，
没有 `rows.csv`，不得作为 full 结果。新增
`test_sealed_binary_can_be_rehashed_before_publication` 先得到同栈 RED；
最小修复仅改用不跟随该 symlink 的 absolute procfs 路径，hash 读取、
approval 比较和发布前检查均保留。该回归与 26 项 gate/bypass 组全绿，
完整 Python 套件现为 151 项。由于 runner diff 已改变，旧 approval 不直接
复用。新的 GPT-5.6 Sol/xhigh reviewer 审查该最小 delta、失败目录和 gate
证据后明确 `APPROVE`，approval 同时绑定：

```text
binary  854df1e692316017cbc26ab462ab3b22735d61caa08ff0645e28ad0cab4a574c
suite   fae83e9ba41dc8b933c79f7769992b29006bb1fc67004e770e621b0830c890ed
corpus  7840959653b2056c6441ede0cbcd93031f9ec3c4796b8270af2c7d1447a72bae
```

正式重跑保存在
`benchmark/results_full_v5_854df1_20260905_r2`，使用 sealed Linux memfd、
14 jobs、10s/case、seed 0、unit weights 和 following allowed：

| 范围 | solved | 备注 |
|---|---:|---|
| full | 479/509 | wall 295.6s；solver-time sum 4037.5s |
| quick 子集 | 47/77 | 与冻结 control 的 T/W 逐例相同 |
| factorial | 432/432 | 6 map families × 其余设计轴全覆盖 |
| timeout | 30 | g20x20 6、g40x40 8、g80x80 16 |

479 个成功 plan 全部存在，validator 重放、定点 W 和 plan hash 均一致。
`rows.csv` SHA-256 为
`743a0b3bc2d635420216148ad52238db38d02b0bbb6288586cbcfc705c7e6e83`；
`timing.json` SHA-256 为
`dab81f04fd1cefc3e1a9398d8ff4cbb0c2c96c3694ef5356822a3f94eda98143`。
正式 dashboard 位于
`benchmark/viz_web/full_benchmark_v5_854df1_20260905/index.html`，含 479
个成功方案动画。面向初学者的最终汇报位于
`benchmark/viz_web/carrier_lacam_v5_final_report_20260905/index.html`；
页面核心统计由正式 rows/timing、固定 quick、隔离历史 full、E5 消融与
Testcase C 样例文件生成，并保留 30 个 timeout、work 回退和约 6 倍 wall
time 这些限制。

## 已知风险与注意点

1. **PlanCost 迁移面大**：g/h/f、incumbent、OPEN、rewrite、macro、两遍、
   repair、stats、probe 全链路；先落 B1/B2 接口与比较器再逐点切换，
   B10 微测锁行为。
2. **删除 claims 后的同拍冲突压力**：由 C2.3 最小时序协调 + endpoint
   放置语义 + oracle 兜底；C1 必须先合入，否则删过滤后又会用“route
   是否存在”当 custody 开关（v1 清单的错误）。
3. **h 的可采性**：任何执行估计（tail、E(r,m)、Score）只进 guidance，
   不进 admissible h；mixed 时间下界必须逐 agent max，不得求和。
4. **缓存纯度**：ExecutionView/route_status/timed data 一律留在
   `UpperEpochCache` 外；同 U 不同 kappa 的逐位回归保护。
5. **性能**：h_T 匹配与 timed helper 有开销；每阶段单独测
   `tau_time_ms/guidance_time_ms` 与 timed-helper 时间，回退要可见。

## 附录：A3 测试分类清单（Stage A 填写）

| 测试文件 | 分类（物理/策略） | 处置 |
|---|---|---|
| `test_tapf_compat.cpp`、`test_dd_integration.cpp` | 物理/兼容 | 原 objective 契约下保留；验证 zero-shelf 自然退化 |
| `test_dd_carrier.cpp`、`test_dd_g1.cpp`、`test_dd_task_br_execution.cpp` | 物理 successor / oracle | 保留 fully constrained successor、Lift/Drop 与 replay 语义 |
| `test_dd_strict_return_deadline.cpp`、`test_dd_finalization_semantics.cpp` | 交付 | 保留 strict deadline、cleanup、final replay |
| `test_dd_storage_transfer.cpp` | 物理 storage | 保留 storage-only Drop、custody 与无 corridor Drop |
| `test_dd_storage_transfer_claims.cpp` | 策略 + 物理边界 | 经独立审查，将 route-overlap 删除契约迁移为 endpoint/时序契约；真实冲突保留 |
| `test_dd_plan_repair.cpp`、`test_dd_anytime.cpp` | 策略 + 交付 | 经独立审查，将 SOC-only 接受迁移为统一 `(T,W)`，replay/deadline 保留 |
| `test_dd_goalset.cpp`、`test_dd_planner.cpp`、`test_dd_task_br_audit.cpp` | objective/策略 | 经独立审查迁移 scalar/旧派工期望；goal 与 search-key 物理契约保留 |
| `test_dd_task_br_exact_matching.cpp` | assignment 策略 | 三项完成时间派工 golden 经独立 GPT-5.6 Sol/xhigh 审查批准；exact/lower-bound certificate 保留 |
