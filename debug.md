# debug.md — 独立审计返工清单

来源：2026-08-29 独立 subagent 审计（GPT-5.6 Sol, high reasoning），对照
`design.md` 与实现逐项核查后给出 **REJECT**。本文档记录必修项、design.md
需修订项与遗留缺失项。修复流程遵循项目规范：**先写 regression test 复现
（RED）→ 修复（GREEN）→ 保留测试**。protected 测试的修改需独立 subagent
APPROVE。

进度标记：[ ] 未开始 / [x] 完成。

---

## P0 正确性必修（审计 REJECT 的直接原因）

### 1. [x] 约束树变量顺序中途可变，破坏 completeness 前提
- **位置**：`lacam/src/dd_planner.cpp` duplicate 分支（revisits % 8 时
  改写 `ex->order` 并 shuffle）；约束树按 `N.order[depth]` 展开。
- **问题**：同一节点的约束树已按旧 order 部分展开后，order 被改写导致
  某 depth 对应的 robot 中途变化——可能重复约束某 robot、漏枚举另一
  robot。违反 design D11（guidance 冻结），§4 completeness 骨架
  （"最终枚举所有 operator 组合"）不再成立。
- **修复方向**：为节点引入不可变 `constraint_order`（创建时冻结，
  约束树只用它）；livelock 扰动只允许改 PIBT 候选排序 / ρ 配对，
  不得触碰约束树变量顺序。若确需换序，必须销毁该节点约束树从 root
  重建，并记录 diversification epoch 保证确定性。
- **测试**：新增 regression test——构造小实例强制走 revisit 路径，
  断言同一节点约束树枚举的 (robot, op) 集合与冻结 order 一致；
  加小图穷举对照（见第 2 项）。

### 2. [x] G1 未按字面实现：fully constrained 时应直通 validator
- **位置**：`carrier_pibt`：forced ops 仍逐个过
  `PIBTContext::feasible/fix`，之后才调 `apply_ops`。
- **问题**：design §4-G1 要求约束覆盖全部 robots 时走 deterministic
  validator——"validator 接受的 joint action 必须被接受"。当前依赖
  第二套未证明等价的 feasibility 逻辑，PIBT 预筛可能拒绝 validator
  可接受的组合。
- **修复方向**：`depth == |R|` 时跳过 PIBT，直接组装 ops →
  `apply_ops` 裁决。
- **测试**：小网格（如 2×3、2-3 robots）穷举全部 joint operator 组合，
  断言【约束树完整叶子 + G1 直通】接受集合 == 【validator 逐一枚举】
  接受集合。该测试同时兜住第 1 项。

### 3. [x] C++ weighted_soc 算错：漏 δ·匿名搬运项
- **位置**：`tools/dd_benchmark.cpp`（`weighted_soc = loaded + free +
  lift_drop`，无 anon_moves 项）；Python `ddbench/validator.py`
  的 `plan_cost` 是正确参照。
- **问题**：design §2.3 cost = α·loaded + β·free + γ·lift/drop +
  δ·anon。C++ 与 Python 输出不一致，跨 runner 的 SOC 结论无效。
- **修复方向**：C++ replay 时单独统计 anon_moves（kappa==ANON 的
  MOVE），支持 αβγδ 参数（默认全 1）；与 Python plan_cost 用共享
  测试向量交叉校验。
- **测试**：固定 plan 的 cost 断言（含匿名搬运的用例），Python/C++
  各一份，数值必须一致。

### 4. [x] YAML flags 声明了但 C++ loader 不解析
- **位置**：`lacam/include/dd_carrier.hpp`（DDInstance 无 flags 字段）、
  `lacam/src/dd_carrier.cpp::load_dd_instance`（忽略 flags）。
- **问题**：design §2.2 的 `remove_on_complete` / `robots_return_to_rest`
  是"默认关"的可选语义，当前状态是"不支持"——文档与实现不符。
- **修复方向**（二选一，需决策）：
  - a) v1 明确不支持：从实例格式与 design v1 正文移除，挪到扩展章节；
    loader 遇到非默认值时报错拒载（fail loudly）。
  - b) 实现之：state/hash/goal/removal 语义全链路支持 + 完整测试。
  - 默认建议 a)（M1 范围最小化）。
- **测试**：a) 路线——loader 对非默认 flags 报错的断言。

## P1 设计-实现一致性（写入 design.md 或补机制）

### 5. [x] sticky park 机制无设计依据且依赖 parent 隐藏状态
- **位置**：`dd_planner.cpp`（`target_park`/`park_owner` 从 parent
  guidance 继承直至 owner 完成）。
- **问题**：机制有效（r2rM 端局死锁靠它解决）但 design 无此规则；
  同一物理配置经不同 parent 到达会得到不同 guidance——正是 D11 想
  避免的路径依赖。duplicate re-guidance 不传 parent 又会突然丢失
  stickiness，行为不确定。
- **修复方向**：在 design.md 新增小节正式定义：触发条件（goal 在他人
  活跃路径上）、owner、释放条件（owner 完成）、双向冲突（互为 owner）
  的裁决、ordering-only 性质声明；实现改为**从当前 X 的依赖关系
  确定性重建**（不依赖 parent 链），保证同一 X 得到同一 park 集合。
- **测试**：sticky park 触发/释放/双 owner 三个 regression case。

### 6. [x] §5.5 M1 livelock 信号与文档不符且实证不足
- **位置**：`dd_planner.cpp`（新节点仅看 guidance-h 24 步无降；
  duplicate 仅看 revisit 计数；guidance-h 为吞吐删掉了 approach 项，
  违反 §5.7 "matching 成本必须进 guidance-h" 的 plateau 要求）。
- **问题**：设计要求"h 无下降 **且** configuration 近周期重复"联合
  条件。DnE-M/S2W-M 0/25 全超时是 M1 信号不足的直接实证。
- **修复方向**（M2 提前项）：实现 cross-deck wait-for graph（robot
  等格子 → 格子等 shelf → shelf 等 robot），环检测触发局部
  re-assignment / 高层回溯；信号维度加入 request 完成率、approach
  进度、blocker 位移；approach 项以增量方式回到 guidance-h（避免
  O(B×R) 全量重算）。
- **测试**：以 dneM_n32_seed0 / s2wM_n32_seed0（已在 protected dev
  cases）10s 内可解为出口判据。

## P2 design.md 文本修订（不改代码）

### 7. [x] §5.6 placement 规则改为"被否决的假设"
- d50 实测：margin/corridor/dead-end 偏好使已解实例回归超时（A/B 数据
  见开发记录）；现行实现是 nearest-free-off-path。design 应记录该
  否决结论与数据，新假设（局部连通度/逃逸度/articulation）标注为
  未验证，待 §8.4 placement ablation。

### 8. [x] 贡献声明降级：v1 = M1 feasibility prototype
- 当前无 FOCAL/anytime/physical-g/f-剪枝（§5.7、D5 未实现），
  completeness 待第 1、2 项修复后才可主张。"complete + anytime +
  scalable" 声明需收窄或标注前提。

### 9. [x] B4 表述拆分
- 定理 1 是"给定 sequential pebble plan 可被单 robot 执行"的存在性
  构造；现实现同时承担"自行找 plan"的启发式职责且可失败。design/
  README 应区分 B4-executor（定理验证）与 B4-greedy-planner
  （成功率是观测指标，不 complete）。

### 10. [x] 命题 2 补同实例负面对照
- 现有正例（cycle rotation 单测）+ 异实例负例（CREST 在 scrambler
  实例失败）不构成同实例分离实证。补固定 zero-empty cycle fixture：
  carrier 可解 + no-following 语义开关下穷举无后继 + B4 报告无空格
  失败；PP 若可转换则同 fixture 运行。

### 11. [x] "validator 唯一实现"表述修正
- 实际架构是 C++ 运行时 validator + Python 独立 conformance oracle
  （双实现互查，比单实现更强但非"唯一"）。design §6.4 措辞更新，
  并建立共享 golden transition corpus 防漂移。

### 12. [x] D_b 缓存策略与文档对齐
- 文档：仅新增占据触发失效、腾空不失效；实现：任意 occupancy 变化
  即失效（保守但慢）。二选一：改文档 + profile 数据，或实现
  dirty-set 方案 + stale-but-safe regression。

## P3 已知缺失（如实记录，M2/M3 排期）

- [x] B0（solve_carrier_rollout，与 macro 共码；commit 22d609a 前置，
      tests: dd_b0_rollout.*）
- [x] B1（solve_carrier_2stage，plan_bound 硬约束；e1074ca，
      tests: dd_b1_2stage.* 含固定计划遵从性与走廊分离对照）
- [x] dead-cell：v1 双层同墙 ⇒ 形态为 finalize 可达性拒载（fcdeaec，
      test: dd_dead_cell.*）；Sokoban drop-pruning 留给分层墙扩展
- [x] anytime / physical g / admissible h / f-剪枝 / FOCAL-lite
      （22d609a，tests: dd_anytime.*；最优性早停）
- [x] Zobrist 增量 hash（fcdeaec，无表 splitmix64；test: dd_zobrist.*）
- [x] 双层可视化 visualize_dd_schedule.py（c32ba67；ascii+png，
      validator 复核回放）
- [x] §8.2 sweep（generate_sweep_instances.py：ratio/fill/|B_tgt|/depth
      四轴；γ 轴以计数器后处理加权，无需重跑）+ §8.3 指标
      （shelf_switches/robot_utilization/first_solution_ms 已入 rows.csv）
- [x] §8.4 ablations（run_ablations.py：full/no_macro/strict_inv/
      no_yield/no_astar/b0/b1 × 9 dev cases）
- [x] M1 出口字面验证：tests/fixtures/m1_exit_20x20_s52_r10.yaml
      （2×2 协议下 shelf 数为 4 的倍数，取 52≥50），首解 <2s
      （test: dd_m1_exit.*）

## 修复顺序建议

1. P0-2（G1 直通 + 穷举对照测试）——它同时是 P0-1 的检测网
2. P0-1（冻结 constraint_order）
3. P0-3（cost 修正）、P0-4（flags 决策+处理）
4. P1-5（sticky park 确定性化 + design 补章）
5. P1-6（wait-for 机制，目标 DnE/S2W 10s 可解）
6. P2 文本修订一次性提交
7. P3 按 M2/M3 排期

---
完成记录（2026-08-29/30）：P0 全部（G1+穷举网 test_dd_g1、
constraint_order 冻结、cost δ 项+跨语言一致性、flags 双侧拒载）；
P1-5 park 纯 X 函数化（dd_sticky_park.* 因果断言，subagent APPROVE）；
P1-6 出口达成：9/9 dev cases 10s 内可解（22d609a，DnE-M 403/403、
S2W-M 384/384、R2R-M 230/230）——根因为 park registry 跨分支泄漏；
P2 全部落入 design.md v2.2（d2c2472）；P3 全部完成（上表）。
维护约定：每完成一项勾选 [x] 并附 commit hash 与测试名；
新发现的问题追加到对应优先级段落。
