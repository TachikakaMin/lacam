# debug.md — Carrier-LaCAM → LaCAM-TAPF 增量集成计划（v3 重写）

来源：2026-08-30 用户指令。本文件**取代**第二轮审计返工清单（其 16 项
已全部完成，历史见 git：98516f7 → 2577f3f）。第三轮独立审计已判定
dd_planner.cpp 属**另起炉灶**（架构形状可追溯 LaCAM，但对原骨架符号
引用为 0）；此前的"骨架回迁"（P1-17/P1-18）只共享了 kernel 零件
（Hungarian/lazy-BFS/FOCAL/LacamNodeCore），solve loop、状态类型、
PIBT、guidance 仍是独立体系。**本轮任务：消除独立体系，把 DD 机制
重建为现有 LaCAM-TAPF（tapf_planner.cpp）execution path 上的增量扩展。**

---

## 0. 最高约束（用户指令，不可协商）

1. **禁止平行 planner / 第二套 search pipeline / 独立算法框架**。
   终态仓库里只有一个 solve loop：`TAPFPlanner::solve()`。
   dd_planner.cpp 的独立 loop、独立 Node/Constraint/PIBT 全部删除，
   其公开 API 降级为 thin adapter。
2. **Semantic invariant（spec 级）**：零 shelf（无 pick/place）实例上，
   扩展后的 `solve_tapf` 与修改前**行为和结果一致**（解序列、搜索轨迹、
   RNG 消耗流全部不变）。兼容性必须来自**保守扩展**（空数据结构 ⇒ 空
   循环 ⇒ 原代码路径逐字执行），禁止 feature flag / legacy mode /
   fallback / "检测无 pick/place 后切换 baseline"。
3. 开发流程严格 **test → RED → implementation → GREEN → benchmark →
   regression test → debug**；发现 bug 先写 regression test 复现再修。
4. 新增 tests / benchmark cases 一经创建即 **protected**；修改需独立
   subagent（GPT-5.6 Sol, high reasoning）明确 APPROVE。既有 protected
   资产（全部 test_dd_*、benchmark/tests/*、dev_cases.txt、benchmark
   语义）继续受保护。
5. Benchmark：与 baseline 相同 dataset/seed/metric/success 语义，每
   testcase 严格 10s；并行度钉在物理核数且留 1–2 核（本机 16 物理核
   ⇒ 全量跑 jobs=14）；单 testcase 内不超订。

## 1. 目标架构（一段话）

唯一 solve loop = `TAPFPlanner::solve()`。状态 = `Config`（robots，
`vector<Vertex*>`，原样）+ `ShelfState{target_pos, anon_occ, kappa}`
（零 shelf 时三个 vector 全空）。**角色模型**：**有实例 task 的 agent** ≡ 原 TAPF agent（候选排序、
hindrance、swap、优先级继承逐字保留）；carrier 实例的 robot 无实例
task，free robot 由 guidance 派发 request cell 并走载具角色候选分支
（lower-dist 排序，无 task-hindrance/swap，见 M7）；
loaded robot 与 Lift/Drop 是新增的 operator 维度；shelf 是被 robot op
驱动的状态变量（design §0）。`dd_carrier.cpp`（PhysConfig/apply_ops/
load_dd_instance）保留为 conformance oracle 与实例格式层，不是第二个
planner。`dd_planner.hpp` 的全部 API（solve_carrier_lacam/rollout/
2stage + 测试支持 API）变为集成机制的 adapter（类型换算 + 转发）。

## 2. 集成映射表（design.md 机制 → LaCAM-TAPF 修改位置）

| # | design.md 机制 | 现 dd_planner.cpp 位置 | 集成落点（文件 :: 函数/结构） | 零 shelf 退化论证 |
|---|---|---|---|---|
| M1 | §2.1/§2.2 双层实例 + goal 条件 | DDInstance / load_dd_instance / is_dd_goal | instance.hpp/.cpp :: `TAPFInstance` 增 `shelf_cells / target_starts / target_goals`（默认空）+ 新 ctor `TAPFInstance(const DDInstance&)`；graph.hpp/.cpp :: `Graph(rows)` inline-map ctor；tapf_planner.cpp :: `is_goal_config` = 原 task 匹配（仅遍历**有 allowed task** 的 agent）∧ 所有 target grounded@goal | 字段空；TAPF 实例每个 agent 都有 task（is_valid 保证），量词范围与原实现相同 |
| M2 | §3.1 状态 X=(Q^R,Q^B,κ) | PhysConfig | tapf_planner.hpp :: `TAPFNode` 增 `ShelfState`；solve() 的 CLOSED：`unordered_map<Config,·>` → `unordered_map<XKey{C,shelf},·>`，hasher = ConfigHasher(C) ⊕ shelf-hash（空 shelf ⇒ ⊕0） | shelf 恒空 ⇒ key 等价性与桶行为 ≡ Config |
| M3 | §3.2 primitive ops + §5.2 operator 约束树 | Op / legal_local_ops / Constraint(parent-chain) | planner.hpp :: `Constraint` 增 op payload（与 who/where 平行；上游 `Constraint(parent,i,v)` ctor 语义不变）；tapf_planner.cpp solve() 展开处：候选 = 邻居+self（原顺序、同一次 shuffle）∪ 条件追加（free 且站在 grounded shelf 上 → LIFT；carrying → DROP）；search_kernel.hpp :: `lacam_expand_constraint_vec` 携带 payload | 追加条件永不成立 ⇒ 候选集合、长度、RNG 消耗与原实现一致 |
| M4 | §3.3 转移合法性（R1/R2/S1/I1-I3）+ G1 直通 | carrier_pibt 末端 apply_ops | tapf_planner.cpp :: `get_new_config` — 部分约束下 forced-op 内联检查（R1/R2 原有 + 上层前置条件）；`funcPIBT` feasibility 增上层占用计数（S1）；**shelf 实例且 M.depth==N 全约束时跳过内联检查、oracle（apply_ops）独裁（G1 直通，实现为 oracle_decides）**；与 oracle 的一致性由 test_dd_g1 穷举对照钉住 | upper 结构恒空，新检查是空操作；R1/R2 路径逐字保留 |
| M5 | §2.3/§5.7 cost 与 admissible h | dd_edge_cost / dd_admissible_h | tapf_planner.cpp :: `get_edge_cost` = 原 task 项（对**有 task** 的 agent）+ 物理项（loaded/free move、lift/drop、anon，unit 权重；DD_SOLVER_WEIGHTS 语义保留）；TAPFNode g/h/f `unsigned`→`double`（整数值在 double 中精确，比较行为不变）；h = assignment.cost + h_shelf（dd_admissible_h 平移） | 物理项对有 task 的 agent 恒 0（无 loaded/lift/drop/taskless）；double 化不改变整数比较 |
| M6 | §5.3 guidance：D_b/requests/ρ/parking/ACTIVE_CAP | build_guidance / PathCache / LowerDist / make_order | tapf_planner.cpp :: guidance 模块（同文件；TAPFNode 创建路径调用）：PathCache/least_blocking_path/park/yield/parking 逐一平移；ρ 复用 `tapf_hungarian_row_to_col`（已共享）+ eta 迟滞；request-cell 与 target-goal 距离场由 `CarrierEngine` 持有（DDDistCache/LowerDist over oracle grid，均为共享 LazyBfsField 核心的 adapter；wallfree Manhattan fast-path 在 LowerDist）；TAPFDistTable 保持 task 场族原样不扩展（实施时的落点修正，语义等价） | targets 空 ⇒ requests 空 ⇒ 整个模块零调用（数据驱动，无开关） |
| M7 | §5.4 Carrier-PIBT 候选序 | PIBTContext::candidates | tapf_planner.cpp :: `funcPIBT` — 候选构造按**角色**分派：task agent = 原代码路径逐字（dist+hindrance+tie_breakers+swap）；**carrier 角色（含 free-with-request）走 §5.4 表的载具候选构造**（request/park 目标按 lower-dist 排序 + shuffle 平局，不含 task-hindrance/swap）；loaded：path-next 优先、S1 预过滤、结构阻塞时 Drop 前置；idle：避让 protect 格。**递归/预留语义按角色分派（WP6 修复 b）**：task agent 上游逐字（失败保留预约、wait=预约+false）；carrier agent 释放-重试 + wait 可行即成功（两层生成器原语义） | 全部 agent 都是 free-with-task ⇒ 原路径逐字执行 |
| M8 | §5.4a target-as-blocker park + 载具对头 yield | build_guidance 中段 | guidance 模块内平移；(X, D_b 缓存纪元) 语义（同 design §5.4a v2.3 降级声明）、carried-hover mask、环打破（最小下标）、DD_NO_YIELD 旋钮全部保持 | 无 target ⇒ 不执行 |
| M9 | §5.5 livelock 信号 / wait-for / 重访 re-guidance | make_node 信号 + solve 重访分支 + waitfor_cycles | tapf_planner.cpp :: 节点创建路径挂 `h_guidance/best_h/no_progress`（**仅 h_guidance>0 时累计**）；solve() duplicate 分支挂 revisit 计数与 re-guidance（requests 非空才有动作/抽签）；waitfor_cycles 平移 | h_guidance≡0 ⇒ 信号永不触发 ⇒ 无新增 RNG 消耗 |
| M10 | §7.1 macro rollout（与 B0 共码，D13） | rollout_from / solve_carrier_rollout | tapf_planner.cpp :: `TAPFPlanner` 新增无约束单步生成（复用 get_new_config 路径）+ rollout 循环；solve() 在节点首扩注入 macro child（条件：存在未完成 target ∧ 规模域 ∧ 首解前）；macro 边的多步 trace 与 cost 存 parent-edge 附注（供 rewrite/抽取） | 未完成 target = 0 ⇒ 永不注入（macro 事件语义定义在 shelf 事件上） |
| M11 | D14 两阶段 anytime | solve_carrier_lacam wrapper | tapf_planner.hpp :: `TAPFSearchConfig` 增 `{stop_at_first, incumbent_init, macro_enabled}`（默认值 = 现 TAPF 行为）；DD 入口按 D14 两次调用 solve_tapf，取更优 | 默认配置下 solve() 控制流与现在 bit 相同 |
| M12 | §4.3 duplicate g-relax/rewire | explored 命中 g_new<g 分支 | **已存在**：`TAPFPlanner::rewrite()` 即该机制（neighbor 集 + 传播 + 重入 OPEN）；macro 边 cost 从 trace 附注读取 | 原样 |
| M13 | §5.7 FOCAL / anytime 选点 / f 剪枝 / 早停 | focal_select_index 调用块 | **已存在**：tapf select_open_index / incumbent / `S->f >= S_goal->g` 剪枝 / `S_goal->g <= initial_lower_bound` 早停；DD 的 h_adm 进 h | 原样 |
| M14 | §6.1 canonical hash / Zobrist | phys_config_hash(_incremental) | XKey hasher = ConfigHasher ⊕ shelf Zobrist（splitmix64 派生）；oracle 侧 phys_config_hash(_incremental) 保留并有性质测试；**集成 rollout 的局部去重用全量 (ConfigHasher ⊕ shelf-hash) 重算**（增量版未接入该路径——落点修正，正确性不受影响） | ⊕0，桶行为不变 |
| M15 | §8.1 B1（2-stage） | solve_carrier_2stage | 入口 adapter：stage-1 冻结 least-blocking 计划 → guidance.plan_bound 硬约束；执行走同一 funcPIBT | — |
| M16 | 测试支持 API（G1 枚举/park/rho/path/waitfor/parking…） | dd_planner.hpp 全部 | 签名不变；dd_planner.cpp 变 thin adapter（DDInstance/PhysConfig ↔ TAPFInstance/Config+ShelfState 视图换算），转发到集成机制的**生产代码** | — |
| M17 | dd_benchmark CLI（MODE/统计/plan 格式） | tools/dd_benchmark.cpp | 外部合同不变（benchmark/tests/test_cli.py protected）；内部走 adapter | — |

**删除清单（终态不得残留）**：dd_planner.cpp 独立 solve loop、DD 私有
Node/Constraint/PIBTContext/pibt_recurse 调用、search_kernel.hpp 的
`dd_expand_constraint` 与 `pibt_recurse`（cutover 后无使用者即删）、
dd_dist_adapters.hpp——**结果：保留为生产依赖**（CarrierEngine 的
DDDistCache/LowerDist 挂共享 LazyBfsField 核心，非仅测试引用，
不满足删除条件）。

## 3. 向后兼容逐点论证（golden 特征化测试钉死）

1. **RNG 流**：展开 shuffle 作用于同长度同内容候选列表；funcPIBT
   tie_breakers 抽签次数不变；restart_rate 抽签位置不变；livelock/
   re-guidance/macro 的抽签被数据条件（h_guidance>0 / requests 非空 /
   未完成 target>0）自然屏蔽。
2. **order**：`init_priorities_and_order` 原样；角色分类排序对全同类
   agent 用 stable_sort ⇒ 原序保持；`constraint_order` = 创建时 order
   拷贝（冻结，D11），TAPF 原实现本就从不在创建后改 order，语义一致。
3. **类型**：g/h/f unsigned→double，整数值精确表示，所有比较/剪枝/
   早停判定不变。
4. **CLOSED**：XKey 等价 ⇔ Config 等价（shelf 空）；hash 值 = 原
   ConfigHasher 值 ⊕ 0。
5. **goal / edge cost / h**：量词范围"有 task 的 agent" = 原全体。
6. **get_new_config / funcPIBT**：新增检查全部以 shelf 数据为条件；
   upper 数组零元素。swap/hindrance 路径不动。
7. **assignment 层**：零 task 时不调用 Hungarian（空 universe 直接
   feasible/cost0）——仅 DD 实例可达（TAPF is_valid 要求 task≥N）。
8. **Solution 抽取 / post_processing / tapf_benchmark CLI**：不动。
9. **planner.cpp（上游 MAPF）**：共享 Constraint 增 payload 但默认
   ctor 语义不变；main.cpp 路径 golden 覆盖。
10. **验证手段**：WP0 golden 特征化测试（多实例×多 seed：解全序列
    hash + soc + makespan + hl_loop_iterations + hl_nodes_created），
    先于一切改动落地，全程必须绿。

## 4. 已知 DD 侧语义变化（诚实清单；由 benchmark gate 裁决）

集成后 DD 行为不承诺与旧 dd_planner bit 相同（旧实现删除），仅承诺
成功率/质量 gate（§6）。变化源：

- D1 PIBT 递归语义：**实测后按角色分派**（WP6 修复 b）——task agent
  上游逐字（占位保留、wait=预约+false）；carrier agent 保留 dd 生成器
  原语义（失败释放预约重试下一候选、wait 可行即成功）。全采上游被
  benchmark 否决：S1 使 carrier push 失败率远高于 task push，保留失败
  预约会级联毒化整步候选（dense 实例搜索劣于自家 B0）。回归：
  `test_dd_integration.search_not_dominated_by_own_rollout_on_dense`；
- D2 邻接序 DDGrid(下/上/右/左) → Graph::neighbor(左/右/上/下)，
  tie-break 平移；
- D3 类内序从 (余距,id) 改为 TAPF 优先级继承（design §5.4 N.order
  第 3 条"no-progress 提升"的原生实现即此机制）；
- D4（修正）free 载具候选**未**获得 task-hindrance/swap——它们走
  carrier 角色分支（lower-dist 排序）；hindrance/swap 仅作用于有实例
  task 的 agent（纯 TAPF / 混合实例）；
- D5 duplicate 命中的 restart_rate 抽签对 DD 生效；
- D6 goal 判定含 target grounded 条件平移（D10 不变）。

任何 gate 未达 → 按 regression-test-first 流程定位；候选回旋手段
（按 design 语义合法、非 hack）：候选序微调、swap 对 DD 关闭须以
"载具语义不满足 swap 前置"论证而非开关、类内序恢复余距键等。

## 5. 工作包（WP）与出口判据

- **WP0 golden 特征化（先于一切代码改动）**
  新 test_tapf_compat.cpp：固定 (实例,seed) 组 × {solve_tapf, solve}
  → 全解 hash/soc/makespan/关键 stats 常量断言（常量由改动前 HEAD
  运行采集）。出口：当前 HEAD 上全绿；此后每个 WP 结束必须仍全绿。
- **WP1 实例/图层（M1）**：DD YAML → TAPFInstance（经 DDInstance）；
  Graph inline ctor；is_valid 规则。RED：加载 DD fixture 断言字段。
- **WP2 状态/键/goal/cost（M2/M5/M14 + M11 config 字段）**：
  ShelfState、XKey CLOSED、goal、edge cost、h、double 化、空 task
  assignment 守卫。RED：已解 DD 实例 solve_tapf 返回单步；goal/cost
  单元；golden 全绿。
- **WP3 生成器（M3/M4 + plan 导出）**：op 约束、展开追加、forced-op
  上层检查、funcPIBT 上层 veto + lift/drop、G1、(C,shelf) 链 → DDPlan
  导出。RED：单 blocker / 双 blocker chain / corridor 小 fixture 经
  solve_tapf 解出且 oracle 重放通过。
- **WP4 guidance 全栈（M6/M7/M8/M9/M10 部分）**：requests/ρ(Hungarian
  +eta)/PathCache/park/yield/order 类/livelock/waitfor/idle 避让/
  parking/ACTIVE_CAP/rollout 单步。RED：dev 小例（scramble_h6w6 等）
  经 solve_tapf 10s 内解出。
- **WP5 cutover（M10/M11/M15/M16/M17）**：两阶段入口、B0/B1、全部
  dd_* API 转 adapter，删除旧 loop 与死代码。出口：**全部 C++ 测试 target + Python 套件绿**（WP5 当时 13 个
  dd/tapf target；终态 CMake 共 23 个独立 target、109 用例）；
  dev 9/9 within 10s。
- **WP6 benchmark parity**：dev → 全量 164×7（jobs=14，统一 10s，
  baseline 同批重跑）。gate 见 §6。回归→regression test→修。
- **WP7 审计与汇报**：git diff 逐项对照本映射表；无 dead code/平行
  实现/特殊分支；文档同步；最终报告。

## 6. Benchmark 协议与 gate

- dev cases：benchmark/dev_cases.txt 9 例（protected，不变），10s。
- 全量：instances_full2 全套（small/standard/paper/sweep，164 实例）×
  7 方法（carrier/carrier_b0/carrier_b1/b4/crest_base/crest_full/
  natcbs），统一 10s，jobs=14（16 物理核留 2），LD_LIBRARY_PATH 直跑
  二进制；**benchmark 运行期间不并行跑测试套件**（历史教训）。
- 对照基准：results_final_v5（旧实现官方数字：carrier 162/164，
  r2r 25/25 mk 548，dne 24/25，s2w 25/25）。
- **gate**：TAPF golden 全等；DD carrier 解出数 ≥162/164；r2r/s2w/dne
  家族平均 executed makespan ≤ v5 对应值 +5%；carrier_b0/b1 保持
  各自基线语义（共码退化关系不变）。
- 未达 gate：先固化 regression test，再调查；结果如实入报告。

## 7. Protected 清单增量

既有全部 protected 资产不变。本轮新增（创建即 protected）：
test_tapf_compat.cpp（golden 特征化）、WP1-WP5 各 RED 测试、
（若新增 benchmark case 则同样 protected）。

## 8. 进度

- [x] WP0 golden 特征化（e92c314；tests/test_tapf_compat.cpp 7 用例：
  deterministic 全轨迹钉扎 + anytime 结果级钉扎；采集自改动前 HEAD
  双跑验证）
- [x] WP1 实例/图层（23ce976；Graph inline-rows ctor 共享 builder、
  TAPFInstance carrier 字段 + DDInstance ctor、carrier-form is_valid;
  allowed 断言修正经 subagent APPROVE）
- [x] WP2 状态/键/goal/cost（70c420f；ShelfState、(Config,ShelfState)
  CLOSED key、goal(D10)、物理 edge-cost 项、double g/h/f、空 task
  assignment 守卫）
- [x] WP3 生成器（3b7daa5；Constraint op payload、op 候选展开（同一
  次 shuffle）、forced-op 前置、S1 PIBT veto、G1 oracle 直通、
  apply_carrier_effects 裁决、solution shelf 链 + derive_carrier_ops）
- [x] WP4 guidance 全栈（72a1257 + e278f2e 修复 + a6afe03；
  requests/ρ(Hungarian+eta)/PathCache/park/yield/waitfor/idle-avoid
  逐字移植、角色化 funcPIBT 候选、类分层 order + 冻结
  constraint_order、admissible shelf h、livelock/重访 re-guidance、
  macro rollout + 两阶段配置字段。教训：共享 PIBT 候选 buffer 被优先
  级继承递归破坏——per-agent buffer 修复，由 WP0 golden 抓出）
- [x] WP5 cutover（eadc575；dd_planner.cpp → thin adapter（两阶段
  D14/B0/B1/全部测试支持探针）、guidance infra → carrier_guidance.hpp
  （tapf_planner 与 adapter 共用）、search_kernel 死代码删除、
  test_tools glob 扩展经 subagent APPROVE。13 个 protected DD 套件
  对集成实现首跑全绿；C++ 106 + Python 57 全绿）
- [!] WP6 benchmark parity——**部分达成**（成功数 gate 达成；家族
  质量 gate 对 DnE 未达，见文末遗留）（86e6e85；官方结果
  benchmark/results_integrated_v2/rows.csv，164×7 统一 10s jobs=14，
  wall 533s）。**carrier 162/164 = v5 持平**（成功集 ±2：新解
  dneM_seed22 与 ddmapd_h24d60t16_seed2；丢 dneM_seed18 与
  s2wM_seed24——种子敏感互换）；carrier_b0 154（v4 147 ↑）、
  carrier_b1 74（v4 77 −3）、b4 115/crest_base 80/crest_full 38/
  natcbs 21 全部复现。共同解出实例家族均值 mk：r2r 1.03、s2w 0.99、
  standard_ddmapd 1.00、standard_scramble 1.00、全部 sweep 除两个
  n=2 家族的 ±1 步噪声（depth_40 5.5v5.0、fill_50 20.5v19.5、
  ntgt_64 71.5v63）外 ≤1.00；**唯一实质偏差：dne_m 1.12（+12%，
  超 +5% gate，如实记录为遗留）**。
  期间修复两个真 bug（先 RED regression 再修）：
  (a) 地址键 scratch 跨 rollout 探针回收污染（跨进程 40x mk 漂移；
  `rollout_steps_match_fresh_generation` + `generation_is_pure_under_
  heap_churn`；修复=rollout/B1 每步与 macro 返回后
  invalidate_carrier_scratch——旧实现同款教训）；
  (b) 预约语义级联（§4-D1 风险实锤）：上游"失败保留预约"使一次
  carrier push 失败毒化本步全部剩余候选（h24 实例 8760 次生成失败、
  搜索输给自家 B0）；修复=**角色化预约语义**——carrier agent 失败
  即释放并重试下一候选、wait 兜底可行即成功（dd 生成器原语义），
  task agent 上游逐字保留（golden 钉死）；
  `search_not_dominated_by_own_rollout_on_dense` 锚定。
- [x] WP7 git diff 审计 + 最终汇报（核心 diff 15 文件
  +2992/−2223；逐文件对照 M1-M17 映射；无平行 solve loop、无
  fallback、无 benchmark hack；search_kernel 死代码已删；诊断计数器
  guidance_builds/path_* 接回真实值）。注：当时声明的"文档同步"
  不完整——round-2/WP6 之后的状态漂移由 WP7c 审计逐项修正
- [x] WP7b 独立精简 review（GPT-5.6 Sol high，22 项裁决）并实施全部
  REMOVE/SIMPLIFY：死函数 is_open_viable、死成员 CarrierEngine::cand、
  3 个无用 include、carrier_ops_cost/rewrite/drain_node 无用参数、
  carrier_upper_base ≡ (carrier_grounded != 0) 等价谓词合并、
  cur_shelf 别名删除、不可达 macro_after_first 增量删除、
  solver 权重解析去重（load_solver_weights 单一实现）、
  park 纪元注释与 dd_planner.hpp 旧架构注释修正。
  KEEP 裁决要点：get_h_value（既有上游 API 面）、splitmix/zmix
  （oracle 边界两侧不同键域）、B1 guidance 块（刻意不同的基线语义）、
  全部 dd_* 测试探针（protected 消费者）、上游 3 参 Constraint ctor
  的 ops.push_back（堆布局扰动风险 > 2 行收益）。
  验证：109 C++ + 57 Python 全绿；dev 9 例 makespan 与精简前
  逐一相同（4/7/41/12/24/81/605/1264/1259）——零行为变化。

- [x] WP7c 独立文档-实现一致性审计（GPT-5.6 Sol high，62 项裁决）：
  实现侧 M1-M17 与删除清单全部 VERIFIED-DONE（M4/M6/M7/M8/M14 为
  文档措辞漂移，已按裁决改正本文件与 design.md；两处为此前静默失败
  的文本替换——教训：文档替换必须 assert 命中）；诚实性修正：
  WP6 降级为 [!] 部分达成（DnE 质量 gate 未达）、WP7"文档同步"
  声明加注；残留源码注释修正（carrier_guidance park 纪元、
  search_kernel 旧 DD-Node 段、README DD_FOCAL_W 退役说明）。
  审计确认：109 C++ + 57 Python 全绿；benchmark 后处理口径
  （b4/crest 按 wall 10.6s 重分类）与公布数字精确复现。

## 9. 遗留（如实）

1. **DnE-M 家族质量 gate 未达**：共同解出 23 例均值 mk 1.12×v5
   （gate ≤1.05）；ntgt_64（n=2）1.13。需要性能工作 + 重跑方可关闭，
   或正式豁免该 gate——当前选择如实记录不豁免。
2. carrier_b1 74 vs 旧 v4 77（B1 为刻意不完备基线，语义共码保持）。
3. 集成 rollout 未接增量 hash（正确性无影响，吞吐机会）。
4. 报表后处理口径需随结果目录一同引用：carrier 系按内部 deadline
   判定成功；b4/crest 按 wall+0.6s 重分类（复现 v5/v2 公布数字的
   同一规则）。

维护约定：每完成一项勾选并附 commit hash 与测试名；新发现问题追加
到对应 WP；protected 测试改动需独立 subagent APPROVE。
