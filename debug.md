# debug.md — Goal-Set τ 层集成计划（v4 重写）

来源：2026-08-31 用户指令。本文件**取代** v3 集成计划（其 WP0-WP7d
已全部完成，历史见 git：e92c314 → 65eea5f；官方结果
`results_integrated_v2`，carrier 162/164，TAPF golden 逐位不变）。
本轮任务：按 `design_final.md`（取代 design.md/designv2.md）落地
**M5 = goal-set 实例格式 + 动态 τ（shelf→goal matching）层 +
frontier-aware requests + BRaP-B 评估**。设计依据与决策记录见
design_final §1.2/§4.3/§5.3/§5.4/§8.2 与 D15-D23。

---

## 0. 最高约束（继承 + 本轮新增，不可协商）

1. **唯一 solve loop**：`TAPFPlanner::solve()`。禁止平行 planner/
   第二套 pipeline/独立框架；τ 层是 `attach_carrier_guidance` 路径上
   的增量，不是新模块体系。
2. **双退化不变量（spec 级，禁 feature flag/fallback/检测切换）**：
   - **零 shelf**：`solve_tapf` 与现 HEAD 行为逐位一致（解序列、
     搜索轨迹、RNG 流）——由既有 golden 特征化测试
     `test_tapf_compat`（protected）持续钉死；
   - **单点集 goal（|G_b|≡1，即全部既有 carrier 实例）**：τ 层必须
     结构性退化为恒等 matching（矩阵每行一个有限元 ⇒ repair 无行
     可变 ⇒ h_shelf 同现值 ⇒ 零额外 RNG 消耗），dev 9 例与全量
     164 例数字不回归（§6 gate）。
3. **loader 契约（D15）**：goal-set 实例只保存 pool + eligibility +
   load 期覆盖匹配检查；**禁止**静态采样/greedy/Hungarian 固化配对
   写进实例或 loader。benchmark 生成器的静态配对逻辑降级为消融专用。
4. 开发流程严格 **test → RED → implementation → GREEN → benchmark →
   regression test → debug**；发现 bug 先写 regression test 复现再修。
5. 新增 tests / benchmark cases 一经创建即 **protected**；修改需
   独立 subagent（GPT-5.6 Sol, high reasoning）明确 APPROVE。既有
   protected 资产（23 个 C++ 测试 target、benchmark/tests/* 11 个
   Python 套件、dev_cases.txt、dd_benchmark CLI 合同、benchmark
   语义）继续受保护。
6. Benchmark：与既有协议同 dataset/seed/metric/success 语义，每
   testcase 严格 10s；jobs=14（16 物理核留 2）；benchmark 运行期间
   不并行跑测试套件（历史教训）。
7. Admissibility 纪律：h_shelf 只能用 LB 主序值（design_final §4.3
   引理 1 + §5.3 D17 编码）；任何 blocking/hysteresis 项不得进入
   h 的数值（D18）。

## 1. 目标架构（一段话）

goal 结构从 `target_goals[b]`（单格）推广为 per-target 候选集
G_b（共享池是特例）。terminal = 逐 target `p_b ∈ G_b ∧ grounded`
（命题 3，τ-free）。每个 node 在 build_guidance 之前用共享
`TAPFAssignmentState`（第二实例化；第一实例是 agent-task 层，第三
是 ρ 的 row_to_col）对 (B_tgt × G_pool) 求/修 τ：主序 = admissible
LB 矩阵，副序 = η_B hysteresis，第三序 = engine tie_hash；Hungarian
最优值 / S 即 h_shelf。guidance 全栈（path dst、requests、park、
yield、waitfor、funcPIBT loaded 分支、guidance-h）把 `goals[b]`
替换为 `τ_N(b)`；clear 优先级加 movability bonus（frontier-first，
DD_CLEAR_FRONTIER=1）。key、转移合法性、macro/D14、FOCAL、rewire
全部不动。B1 的 τ 冻结为根节点值（D23）。

## 2. 集成映射表（design_final 机制 → 修改位置）

| # | 机制 | 落点（文件 :: 函数/结构） | 单点集退化论证 |
|---|---|---|---|
| T1 | §2.1 goal pool + eligibility + 覆盖匹配检查 + YAML 兼容 | dd_carrier.hpp/.cpp :: `DDInstance` 增 `goal_pool`/`per-target goal 集`（旧 `[start,goal]` ⇒ 单点集）；`finalize()` 增二部匹配可行性拒载；`TAPFInstance(const DDInstance&)` 透传 | 旧格式加载结果与现 HEAD 字段逐位相同（target_goals 向量保留为单点集视图） |
| T2 | §2.2/命题 3 terminal | dd_carrier.cpp :: `is_dd_goal`；tapf_planner.cpp :: `is_goal_config`（per-target goal bitset 预构建）；ddbench/validator.py 同步 | 单点集 bitset 恰含原 goal ⇒ 判定值逐状态相同 |
| T3 | §5.3/D17 τ matching + repair + h_shelf | carrier_guidance.hpp :: CarrierEngine 增 τ 引擎（LB·S+pen 编码、`solve_full`/`repair_rows`）；tapf_planner.cpp :: `attach_carrier_guidance` 在 build_guidance 前求 τ、moved-target 行修复、h_shelf=主序值；TAPFNode 增 `tau`（vector<int>）+ `tau_state`（与 assignment_state 同拷贝模式） | 每行单有限元 ⇒ τ 恒等/repair 空转/h_shelf 同现公式值；无 RNG 消耗 |
| T4 | §5.4 requests 的 τ 间接 + frontier 优先级 + §5.6 park/yield/waitfor 替换 | carrier_guidance.hpp :: `build_guidance`（dst=τ(b)；protect 只标 τ 指派格；movable bonus）；`waitfor_cycles`（同替换） | τ 恒等 ⇒ dst 同今值；movable(path[1]) 低填充恒真 ⇒ 优先级序不变（dev 9 例逐例核对） |
| T5 | §6.2 PathCache dst 失效（正确性项） | carrier_guidance.hpp :: PathCache::Entry 增 `dst`，miss 条件加 `dst 不匹配` | 单点集 dst 恒定 ⇒ 永不因 dst 失效 |
| T6 | §6.2/D21 共享 upper-wall 距离缓存 | carrier_guidance.hpp + tapf_planner.cpp :: `target_goal_dist[b]` → 单 `DDDistCache upper_wall`（先行独立重构） | 值域相同（dest-keyed 精确 BFS）；固定 goal 行为不变 |
| T7 | §5.6 funcPIBT loaded 分支 d(·,τ(b)) | tapf_planner.cpp :: funcPIBT（经 guide 读 τ；planner 不直读 goal 集） | 同 T3 |
| T8 | §5.6 livelock 的 τ-taboo re-guidance + guidance-h 随 τ | tapf_planner.cpp :: attach_carrier_guidance（taboo 集扩展到 (b,goal) 对；重访/plateau 时 repair 禁忌行；**|G_b|=1 行豁免禁忌**——否则行被 sentinel 清空致 matching 不可行） | 单点行全部豁免 ⇒ 禁忌集恒空 ⇒ 零动作（须测试钉住） |
| T9 | D23 B1 冻结 τ | dd_planner.cpp :: B1 入口取根 τ 后 plan_bound | B1 语义独立，不影响 full |
| T10 | §8.2 BRaP-B goal-set 套件 + 消融 | benchmark :: generate_brap_instances.py 增 goal-set 输出模式（原静态配对模式保留为消融变体）；run_ablations.py 增 frozen-τ/no-η_B/head-first 变体 | 生成器变更不触 solver |

**删除/替换清单（终态不得残留）**：solver 侧对 `target_goals[b]`
的直接语义依赖（全部经 τ 或 goal-set 查询；grep 审计清单）；
per-target `target_goal_dist` 向量；主生成器路径里的静态配对。

## 3. 向后兼容逐点论证（golden + parity 钉死）

1. **RNG 流**：τ 层（Hungarian/repair/bitset 判定）零随机；requests/
   park/order/livelock 的抽签位置不变；frontier bonus 只改确定性
   优先级数值。零 shelf 路径无一行新代码执行。
2. **key/CLOSED**：SearchKey 不变（τ 不进 key，design_final §3.1）。
3. **h/f/剪枝**：单点集 h_shelf 数值 = 现公式值（T3）；FOCAL/早停/
   f 剪枝判定输入不变。
4. **goal 判定**：单点集 bitset ⇔ 原等式（T2）。
5. **oracle/validator**：apply_ops 与转移规则零改动；is_dd_goal 按
   T2 同步，golden corpus 的转移用例不受影响（终止用例新增双侧）。
6. **adapters/CLI**：dd_planner 探针签名、dd_benchmark 合同、
   run_benchmark 方法集不变。
7. **验证手段**：`test_tapf_compat` 全绿（零 shelf 逐位）；dev 9 例
   makespan 逐例相同（单点集结构退化）；全量 164 例 parity（§6）。

## 4. 已知语义变化（诚实清单；仅 goal-set 实例可达）

- V1 terminal 从"到达指定格"变为"到达任一 eligible 格"——仅新格式
  实例；旧实例语义逐位不变。
- V2 done ≠ settled（design_final §5.3）：goal-set 实例中已 done 的
  target 可被 τ 改派并再次搬运（D2 可逆性的新形态）；报表的
  shelf_switches/robot_utilization 语义不变，新增 park 触发率与
  τ 改派次数观测列（rows.csv 追加列，不改既有列语义）。
- V3 protect 只标 τ 指派格（非全池）——单点集下与现行为相同。
- V4 B1 在 goal-set 实例上 = 静态 τ + 冻结 plan（D23），与 full 的
  对照变量是"逐节点重算 vs 全冻结"。

## 5. 工作包（WP）与出口判据

- **WP-R0 基线快照（先于一切代码改动）**：记录当前 HEAD 的 dev 9 例
  makespan/soc/首解时延与 `results_integrated_v2` 引用数字；跑通
  23 C++ + 11 Py 套件基线绿。出口：快照入本文件 §8。
- **WP-A 实例层（T1/T2）**：RED = 新格式 fixture 加载断言（pool/
  eligibility/覆盖检查拒载不可行实例）+ 旧格式字段逐位 + 终止判定
  双侧（C++/Python）新 fixture（含"done 于非指派 goal"用例）。
  GREEN 后：golden corpus 增终止判定用例（protected）。
- **WP-B 距离缓存重构（T6，独立先行）**：RED = 共享缓存值与
  per-target 缓存逐 dest 相等的性质测试；dev 9 例 makespan 不变。
  （风险隔离：此步零语义变化，先行合并可缩小 WP-C diff。）
- **WP-C τ 引擎（T3/T5/T7）**：RED = (a) 词典序编码性质测试（构造
  矩阵：主序最优被 pen 破坏当且仅当编码错误；h= result.cost/S 与
  brute-force min-LB 相等，含 carried/grounded opLB 分支）；(b) 单点
  集退化测试（τ 恒等 + h 同旧公式 + repair 空转）；(c) 2×2 共享池
  手工实例：τ 全局最优避免"最后一个 shelf 被迫走 8 格"（designv2
  §9 场景缩小版）；(d) PathCache dst 失效回归；(e) rollout 地址
  失效：τ/tau_state 若有 address-keyed 缓存必须挂
  invalidate_carrier_scratch（复用既有回归测试形状）。
- **WP-D guidance 全栈替换（T4/T8）**：RED = park owner 经 τ 的
  走廊 fixture（τ 改派消解 park 的用例 + 单点集 park 行为不变
  用例）；τ-taboo re-guidance 单点集零动作测试；frontier 优先级
  one-empty fixture（movable 链叶先获派）+ 低填充序不变测试。
- **WP-E 入口/基线/生成器（T9/T10）**：B1 冻结 τ 测试；生成器
  goal-set 模式 + 静态模式消融保留；`test_tools` 覆盖新轴。
- **WP-F benchmark 与消融**：§6 全部 gate；run_ablations 增
  frozen-τ/no-η_B/head-first；报表更新。
- **WP-G 审计与文档**：git diff 逐项对照 §2 映射表（独立 subagent）；
  删除清单核销；design_final/本文件状态同步；最终汇报。

## 6. Benchmark 协议与 gate

- dev cases：benchmark/dev_cases.txt 9 例（protected 不变），10s。
- **Gate A（singleton parity，硬）**：instances_full2 全 164 例 ×
  carrier：成功集与逐例 makespan 与 `results_integrated_v2` 一致
  （种子敏感 ±2 例互换的既有容差沿用；makespan 逐例相等——τ 层在
  单点集下结构退化，任何漂移都是 bug）。TAPF golden 全绿。
- **Gate B（BRaP-B 动态 τ 主实验）**：B 型 goal-set 套件 vs 三个
  静态对照（greedy 34/68、Hungarian 32/68、near-boundary 18 例
  carrier-only）：≤10×10 成功数 ≥ 34 基线不回归；common-solved
  makespan ≤ near-boundary 静态（在其 8.5× 改进之上继续改进）；
  ≥20×20 不设成功 gate（horizon 墙，如实记录）。R1 型（单点集）
  逐例与原 results_brap 相等（Gate A 的 within-suite 复核）。
- **Gate C（消融方向性）**：frozen-τ 不优于 dynamic-τ（common
  solved mk）；违反则如实记录并调查（不得静默调参掩盖）。
- 未达 gate：先固化 regression test，再调查；结果如实入报告。
- **执行结果（2026-08-31）**：
  - **Gate A PASS**（results_gateA，164×carrier 统一 10s jobs=14，
    wall 107.5s）：162/164 = v2；成功集 0 丢 0 增；common solved
    162/162 **逐例 makespan 完全相等**（report_gates.py）。
  - **Gate B PASS**（results_gateB，instances_brap_pool 68×carrier）：
    套件总成功 35/68 ≥ 34 基线（+1）；B 型 18/34（= near-boundary 18，
    > static-greedy 17）；common-solved makespan：vs static-greedy
    mean **0.275×**（16/17 更优；含 goal-set 语义红利极例——target
    已在边界 ⇒ mk 1 vs 24774）；vs near-boundary mean 0.851×/median
    0.510×（审计修正计数：14 优 / 1 平 / 3 劣，即 15/18 不劣；
    3 劣中含一小易例 6× 波动）；R1 单点集
    对照 34 行 **0 漂移**（Gate A 的 within-suite 复核）。
  - **独立审计（2026-08-31，GPT-5.6 Sol subagent）**：判定 CONFORMS
    ——35/68 的正确读法是 **≤10×10（gated 区间）35/36=97.2%
    （B 型 18/18=100% vs static 17/18）+ ≥20×20（声明的 horizon 墙、
    不设 gate）0/32（新旧套件同为 0/32）**；裸 35/68 是分母混入
    declared-unreachable 区间的口径效应。审计压力测试：20×20 e100
    B 型 pool 例 60s 预算仍无首解（best_targets_done 28/40、
    hl_nodes 81315）——墙的结论在 6× 预算下经受住检验（单例证据，
    非无限预算证明）。审计发现的两处报告瑕疵已修：report_gates.py
    增加机械尺寸过滤 + 显式 PASS/FAIL（输出 35/36 PASS / 0/32
    record-only）；near-boundary 严格计数改为 14 优/1 平/3 劣。
  - **Gate C PASS**（results_gateC_frozen，DD_TAU_FREEZE=1 同套件）：
    dynamic 18/34 > frozen 17/34；common 17 例 mk dynamic/frozen
    mean **0.742×** / median 0.619×（frozen 仅 4/17 占优）——
    "每节点重算"的贡献被隔离证实。

## 7. Protected 清单增量

既有全部 protected 资产不变。本轮新增（创建即 protected）：
WP-A/B/C/D/E 的全部 RED 测试、goal-set golden 终止用例、BRaP-B
goal-set 套件实例与生成器语义。

## 8. 进度

- [x] WP-R0 基线快照（HEAD 185d073：24 C++ target 全绿 + Python OK；
  dev 9 makespan 4/7/41/12/24/81/605/1264/1259 —— 全程 parity 基准）
- [x] WP-A 实例层 T1/T2（69c476e；`test_dd_goalset` 7/7 +
  `tests/test_goalset.py` 6/6，RED→GREEN；finalize 物化/组件过滤/
  Kuhn 覆盖检查；is_dd_goal/is_goal_config/Python is_goal 集合成员判定；
  dev parity 逐位）
- [x] WP-B 距离缓存重构 T6（a8f7443；`test_dd_tau` 共享 dest-keyed
  性质 pin；CarrierEngine::upper_wall 单缓存 + 6 个 adapter 站点；
  零行为变化）
- [x] WP-C τ 引擎 T3/T5/T7（6ca3243；`test_dd_tau` 10 用例中 8 个：
  singleton==root-h 精确、injective contention、暴力最小匹配相等、
  hysteresis 仅平局、opLB 分支、taboo+单点豁免、dst miss；
  **落点修正（如实）**：per-node `tau_state`+`repair_rows` 方案未采用
  —— hysteresis pen 随 parent 配对变化使"只有 moved rows 变化"前提
  不成立（增广路径会重排其它行），增量修复对 h 可采纳性不安全；
  实现为 CarrierEngine 级单一 TAPFAssignmentState 每节点 solve_full
  （规模域 ≤256 实测可承受），增量列为吞吐扩展）
- [x] WP-D guidance 全栈 T4/T7/T8（a990160；τ 先行→hg/paths/park/
  yield/waitfor/funcPIBT/order 全部经 τ；τ-taboo 于 livelock+reguide
  两路触发（单点行豁免 ⇒ 固定 goal 实例零动作）；dd_view 补拷
  target_goal_sets（RED pool 测试抓出的真 bug）；
  **D20 实施期修订（如实）**：DD_CLEAR_FRONTIER 默认 0 —— bonus 在
  50% 填充 dev6 (ddmapd_h16) 上重排 clear 序致 makespan 81→91，
  §0.2 singleton-parity 为 spec 级故默认关；机制+双默认断言保留
  （`frontier_movable_clear_outranks_chain_head`）；design_final
  D20/§6.5 已同步修订）
- [x] WP-E 入口/基线/生成器 T9/T10（T9 已随 WP-D 落地：B1 冻结根 τ
  per D23；63d1830：generate_brap_instances.py --goal-mode
  {static,pool}，static 全量 diff 字节相同，pool 同布局 RNG 流仅
  goal 结构变化；instances_brap_pool 68 例；
  `test_tools.TestBrapGoalSetMode` RED→GREEN）
- [x] WP-F benchmark 与 Gate A/B/C（见 §6 执行结果；DD_TAU_FREEZE
  消融旋钮进 solve_tau（h 保持未冻结 admissible 值）；
  **偏差（如实）**：run_ablations.py 未扩展 —— dev 9 例全为单点集，
  frozen-τ/η_B/frontier 在其上结构性 no-op；Gate C 改在 pool 套件
  （旋钮真正生效处）手动执行并存档 results_gateC_frozen）
- [x] WP-G 进度同步 + 里程碑 commit（本条）；26 个 C++ 测试二进制
  全绿 + Python 套件 OK + dev 9 parity 逐位（最终复验）

## 9. 继承遗留（v3 结转，如实）

1. **DnE-M 家族质量 gate 未达**（+12% vs v5，gate ≤1.05；未豁免）：
   固定 goal 家族，τ 层不触及——需独立性能工作，保持记录。
2. carrier_b1 74 vs 旧 77（刻意不完备基线，语义共码保持）。
3. 集成 rollout 未接增量 hash（吞吐机会）。
4. 报表口径随结果目录引用：carrier 系按内部 deadline；b4/crest 按
   wall+0.6s 重分类。
5. ≥20×20 puzzle 密度 horizon 墙（design_final §12.6，超本轮范围）：
   pool 模式下 ≥20×20 仍 0 解（预期内，horizon 不是 goal 结构问题）。
6. τ 增量修复（repair_rows）未接：见 §8 WP-C 落点修正——需先解决
   hysteresis 行失效语义才能安全增量；规模域内 solve_full 实测无
   吞吐问题（Gate A wall 107.5s vs v2 同口径），列为吞吐扩展。
7. run_ablations.py 的 goal-set 变体（frozen-τ/η_B/frontier on pool
   suite）未脚本化——Gate C 手动流程与命令已记录，脚本化留后续。

维护约定：每完成一项勾选并附 commit hash 与测试名；新发现问题追加
到对应 WP；protected 测试改动需独立 subagent APPROVE。
