# Carrier-LaCAM 最终设计文档（design_final）

状态: final v1.0, 2026-08-31。

**本文取代 `design.md`(v3.1) 与 `designv2.md`**：
- 继承 design.md 中**已实现并经全量 benchmark/审计验证**的全部语义
  （§2-§6 的物理模型、约束树、guidance、livelock、macro/D14、
  工程与理论结论），这些内容在本文中如实标注"已落地"；
- 采纳 designv2.md（BiTA）的核心思路——**取消固定 shelf-goal 配对，
  每个 target shelf 保留完整候选 goal 集合，第一层 assignment
  （shelf→goal，τ）与第二层 assignment（robot→request，ρ）都在每个
  physical node 上重算/修复**——并按当前代码形态给出可落地的集成
  方案（标注"待实现"）；
- 教学版说明见 `carrier_lacam_for_freshmen.md`（与实现核对过的
  入门文档，术语与本文一致）。

写作约定：每节标注 [已落地] / [待实现] / [部分落地]；实现载体一律为
现有集成代码（唯一 solve loop = `TAPFPlanner::solve()`），工程约束
与工作包见 debug.md（v4，与本文同步重写）。

---

## 0. 一句话

> 不把 shelf path 和 robot path 当成两个 plan；它们是**同一条 physical
> configuration path 的两个投影**。搜索只在物理 configuration 空间上进行，
> robot 是唯一 actuator，shelf 的移动是 robot 动作的确定性 effect。
> 在此之上，**"哪个 shelf 去哪个 goal"（τ）与"哪个 robot 干哪件事"（ρ）
> 都不是输入，也不是 state——它们是每个 node 上重算的 guidance。**

对应到 LaCAM 框架，核心修改共三件（前两件已落地，第三件是本轮增量）：

1. [已落地] high-level state 从 agent configuration 扩展为
   robot + shelf + carrying 的物理 configuration；
2. [已落地] low-level constraint 从 "agent 的 next vertex" 提升为
   "robot 的 primitive operator"（wait/lift/drop 的 next vertex 相同，
   仅约束位置无法区分 transition）；
3. [待实现] goal 从"每 target 一个固定格"放宽为"每 target 一个候选
   集合 G_b"；terminal 判定、admissible h 与 guidance 全部改为
   matching-aware，τ 由每节点的 incremental Hungarian 维护。

依赖链（designv2 §1）：

```text
X  →  τ_B（shelf→goal matching）
   →  I（vacancy-centric upper intent / requests）
   →  ρ_R（free robot→request matching）
   →  Carrier-PIBT（constraint-respecting generator）
```

τ、I、ρ 全部是 ordering-only guidance：不进 state key、不进 terminal
判定、不删除任何合法 successor（见 §4.2 G2）。

---

## 1. 背景与动机

### 1.1 方法空档（继承 design.md，已被实验证实）

- **BR-LaCAM / BRaP**（arXiv 2509.01022）搜索 block-level rearrangement：
  block 被当作会自己移动的 agent，robot、carrying、lift/drop 不在
  search state 里，plan 从不验证可执行性。
- **DD-MAPD / MAPF-DECOMP**（arXiv 2304.14309）shelf-first decomposition：
  先规划 shelf 轨迹再切段给 robot；shelf 规划不感知 robot。
- **CREST**（arXiv 2603.28803）保留两阶段，仅在执行期修补；lift/place
  overhead 越大，分解损失越大。
- **NAT-CBS**（SOCS'25）coupled 且 makespan-optimal，但慢约四个数量级。

空档：**没有方法在搜索的每条边上保证 robot-shelf 联合可执行，同时保持
LaCAM 级 scalability 与 completeness。** [已落地] 这一主张已由统一协议
benchmark 支撑：164 实例 × 7 方法 × 10s，carrier 162/164，
carrier_b0 154、b4 115、crest_base 80、carrier_b1 74、crest_full 38、
natcbs 21（benchmark/results_integrated_v2）。

### 1.2 为什么还需要动态 τ（本轮增量的实证动机）

BRaP 协议套件（2026-08-31，三个 commit：f1a7cde/de513c6/5f66864）
在 sliding-puzzle 密度（93-97% 填充）上暴露了 v1 固定 goal 语义的
质量上限：

1. **carrier 34/68**（4×10..10×10 全解、含论文 fig-6h 深埋单 block 例；
   所有 baseline 0-6/68）——但 goal 由生成器**静态**贪心配对；
2. **near-boundary 静态重配对消融**：把 B 型（边界任意出口）goal 改为
   "全边界池最近去重匹配"，解出 17→18，makespan 最多好 **8.5×**、
   首解快 9×——静态配对的选择质量直接决定数倍的执行代价；
3. **Hungarian 静态配对消融**：同一采样集上用最优 min-sum Manhattan
   配对替换贪心，配对总距离 −14.6%，执行却**更差**（32/68 vs 34/68，
   8×10 makespan +31%）——min-sum 直线距离不建模 puzzle 密度下的
   清障拓扑（穿越少数空格的路径交叉）；**任何静态匹配器都注定拿不到
   正确答案，因为正确配对依赖搜索过程中不断演化的 occupancy**。

结论：goal 候选集 + 每节点 τ 重算（blocking-aware cost、可撤销、
hysteresis 防震荡）是结构性缺口，不是调参问题。这正是 designv2 的
出发点，也是本文第 5.3 节的核心增量。

同时，BRaP 套件也如实确认了不可归因于 goal 配对的墙：≥20×20 在 10s
内对**任何**输出可执行 plan 的方法都不可达（解出的 4×10..10×10 平均
makespan 2.8 万-7.4 万步；BRaP 自己的 80×80 可解性是其非可执行抽象的
属性）。动态 τ 的预期收益域是**已可解规模上的质量**与边界规模上的
成功率，不是横向穿越 horizon 墙。

---

## 2. 问题定义

### 2.1 环境与实例 [部分落地]

- 4-连通 grid，双层语义（DD-MAPD 风格）：
  - **lower deck**：robots 行驶层。free robot 可以从 grounded shelf
    正下方通过与停留（仅受 wall 限制）。
  - **upper deck**：shelves 层。grounded shelf 静止；被 lift 的 shelf
    随 carrier robot 移动。
- 实例输入：
  - grid（walls）；
  - robots R，初始位置 Q^R_0（labeled）；
  - shelves B，初始 occupancy；target shelves B_tgt ⊆ B 有 label，
    其余 shelf **匿名**（无 goal、无身份）；
  - **goal 结构（本轮增量）**：goal pool G_B 与 eligibility
    G_b ⊆ G_B（每 target 一个候选集合）。固定 goal 是
    |G_b| = 1 的特例。B 型公共边界目标表示为
    G_b = G_boundary（∀ b ∈ B_tgt，共享池）。

**Loader 契约（designv2 §2，钉死）** [待实现]：loader 只保存完整
goal pool 与 eligibility，并在 load 期检查存在覆盖所有 target 的
injective matching（二部图匹配可行性，Hall 条件由匹配算法隐式判定；
不可行 ⇒ fail loudly 拒载）。**禁止**以下任何一种"预处理"：

1. 从池中随机采样与 target 数量相同的 goal 子集；
2. 用 greedy / Hungarian / 任何静态算法固定 shelf-goal 配对；
3. 把配对写成实例中的固定 g_b。

（§1.2 的两个消融证明 1/2/3 都会留下数倍质量损失或成功率损失。）

**YAML 格式（向后兼容）** [待实现]：现有 `targets: [[start, goal],…]`
继续合法，解释为 G_b = {goal}（单点集）；新增
`goal_pool: [cells…]` + `targets: [[start, [g1,g2,…]],…]` 或
`targets_pool: [[start, "pool"],…]`（共享池简写）。二者混用合法。
非默认 flags 双侧拒载的既有规则不变。

### 2.2 Goal condition [待实现（goal-set 版）；单点集下已落地]

状态 X 是 goal 当且仅当存在 injective τ: B_tgt → G_B，
τ(b) ∈ G_b，使得每个 target b grounded 且 p_b = τ(b)。

**简化判定（v1 采用，见 §4.1 正确性论证）**：goal 是物理 cell 且两个
shelf 不可能同格（S1），故

```text
is_goal(X)  ⟺  ∀ b ∈ B_tgt: p_b ∈ G_b ∧ b grounded
```

- robot 终态不约束；
- 到达 goal 的 target **留在原地继续占格**（D2）；
- **"完成"不是不可逆状态**：已位于合法 goal 的 shelf 仍可被再次
  lift、搬开，最终落到同一个或另一个合法 goal。系统不保存单调
  completed bit（D12：χ 是 derived view）。
- terminal 判定**不读取 τ**（τ 只是 guidance；见 §4.2 骨架第 5 条）。

### 2.3 目标函数 [已落地]

两个 objective，分开报告：

- **executed makespan**：第一个到达 goal condition 的 timestep；
- **weighted action cost（SOC 风格）**：

```text
c(a) = α·#loaded moves + β·#free moves + γ·#lift/drop + δ·#anon moves
```

默认单位权重；`DD_SOLVER_WEIGHTS=1` 时 α..δ 贯穿 solver 的
g/admissible-h/rollout（加权穷举最优性质测试 `test_dd_m2`）。
"executed makespan" 是关键指标：decomposed 方法报告的 shelf-plan
makespan 不等于真实执行 makespan。

---

## 3. 模型语义（规范；除 goal 结构外全部 [已落地]）

### 3.1 状态

```text
X = (Q^R, Q^B, κ)
```

| 分量 | 含义 | 实现 |
|---|---|---|
| Q^R | robot 位置（labeled） | `Config`（`vector<Vertex*>`，原 LaCAM-TAPF 类型） |
| Q^B | shelf 层 | `ShelfState{target_pos, anon_occ(sorted), kappa}` |
| κ | carrying 关系 | `kappa[i] ∈ {KAPPA_FREE=-1, KAPPA_ANON=-2, b≥0}` |

- κ(r)=b ⇒ q_r = p_b（carrier 与 shelf 同格不变量）；
- 被 carry 的匿名 shelf 不需要身份（KAPPA_ANON flag，D9）；
- χ（completed 集）是 derived view，不进 state/key（D12）；goal-set
  语义下同样成立：`done(b) ⟺ grounded(b) ∧ p_b ∈ G_b`；
- **τ、I、ρ、距离场、requests、park 一律不进 state key**。
  `EXPLORED` 的 key = `SearchKey{Config, ShelfState}`（canonical：
  anon_occ 排序去除匿名 relabeling），hasher = 原 ConfigHasher ⊕
  shelf-hash（零 shelf 时 ⊕0）。**goal-set 引入后 key 不变**——这是
  designv2 §2 的要求，当前实现天然满足，无需改动。

### 3.2 Robot primitive actions

每个 robot 每 timestep 选一个：`Wait / Move(v) / Lift / Drop`。
lift/drop 无参数（只能操作脚下/手上的 shelf）。preconditions 与
effects 见 design.md §3.2 表（未变，`dd_carrier.cpp::apply_ops`
为权威实现）。joint action 下**只有 robot 选动作，shelf 不独立选
动作**。

### 3.3 Joint transition 合法性 [已落地]

规则表未变（R1/R2 lower 顶点/交换冲突；S1 upper 唯一占据；I1-I3
交互规则；S2 由 R2 蕴含）。三方一致架构：

1. 部分约束下 planner 内联检查（快速否决，允许保守错杀）；
2. 每个被接受的 joint op 经 C++ `apply_ops` 终裁；全约束深度（G1）
   跳过内联、oracle 独裁；
3. Python `ddbench.validator` 对输出 plan 整体重放（第二 oracle）。

跨语言 golden corpus（`tests/fixtures/golden/`）+ G1 穷举对照
（`test_dd_g1`）钉住三方语义一致。**goal-set 变更触及 goal 判定与
guidance，不触及转移合法性——本节零改动，是本轮增量风险可控的
结构性原因。**

### 3.4 两个刻意的语义决策（与 BRaP 不同） [已落地]

**(a) 不继承 no-following**：显式 robot 模型里 convoy = 两个 loaded
robot 同步移动，物理可行；统一模型约束更少、makespan 更优。
`DD_NO_FOLLOWING=1`（oracle 层旋钮，`test_dd_nofollow` 量化）提供
语义对齐实验。**designv2 §8 的要求由此已满足**：BRAP_NO_FOLLOWING
与 PHYSICAL_ALLOW_FOLLOWING 是同一 oracle 的两个变体，benchmark
两组结果分开汇报、不混合。

**(b) 零空格 cycle rotation 合法**（命题 2 的分离实例，穷举证明 +
双侧可执行测试固化，`test_dd_g1::dd_prop2_*`、`test_prop2.py`）。

---

## 4. 理论

### 4.1 可行性与 goal-set 终止判定的正确性

**定理 1（|R|=1 单 robot 模拟，已修正适用域）与命题 2（表达力分离，
含 MAPF-DECOMP(PP)/BRaP/sequential pebble motion 三分表述）**
原样继承 design.md §4.1，不再赘述；两者的可执行锚测试已在库中。

**命题 3（简化终止判定 = matching 终止判定）[新增，v1 采用]**。
若每个 goal 是物理 cell 且合法状态中任意两 shelf 不同格（S1），则

```text
∀b: p_b ∈ G_b ∧ b grounded   ⟺   ∃ injective τ, τ(b) ∈ G_b, p_b = τ(b) ∧ 全部 grounded
```

*证明*：(⇐) 显然。(⇒) 取 τ(b) := p_b；由 S1，b ↦ p_b 单射；
p_b ∈ G_b 给出 eligibility。∎

因此 terminal 判定是逐 target 的 O(|B_tgt|) membership 检查
（`p_b ∈ G_b` 用预构建的 per-target bitset），不需要在线匹配。
loader 期的覆盖匹配检查（§2.1）保证 goal 可达配置存在不因
eligibility 结构而空。

### 4.2 Completeness [已落地骨架；τ 层只需验证第 4/5 条]

沿 LaCAM 骨架的六条（design.md §4.2）：状态空间有限；每 node 的
action-constraint tree 最终枚举全部 robot primitive 组合（冻结
`constraint_order`，D11）；validator 恰好接受 §3.3 全部合法转移；
**τ、I、ρ、距离场、priority、park、livelock 只改变 successor 生成
顺序，不永久删除任何 successor**；terminal 不依赖 τ；EXPLORED key
只含 canonical(X)。

Guard G1（fully-constrained 直通 oracle）与 G2（只剪 provably-dead
或 f ≥ incumbent）不变。**τ 层的 completeness 义务清单**（新增机制
逐条对照 G2）：

| 新机制 | 类别 | 论证 |
|---|---|---|
| τ matching（Hungarian/greedy） | ordering-only | 只决定 guidance 的临时 goal 与候选序 |
| τ hysteresis η_B | ordering-only | 只扰动 matching 选择 |
| terminal 用 §4.1 简化式 | 判定 | 不读 τ（命题 3 证其与任意合法 τ 等价） |
| admissible h 用 LB-matching（§5.7） | cost 剪枝 | h ≤ 真实剩余代价（引理 1），f 剪枝合法 |
| goal-cell protect / park owner 经 τ | ordering-only | park 本就是 ordering（§5.4a 继承） |

两阶段 anytime（D14）与 completeness 的关系不变：phase 级、无时限
主张；macro 只增不减 successor。

### 4.3 Admissible h 在 goal-set 下的正确形态 [待实现]

**引理 1（assignment lower bound）**。定义 per-pair 下界矩阵

```text
LB[b,g] = α · d_upper_wall(p_b, g) + opLB(b,g)，   g ∈ G_b（否则 ∞）
opLB(b,g) = 0        若 b grounded ∧ p_b = g
          = 2γ       若 b grounded ∧ p_b ≠ g      （≥1 lift + ≥1 drop）
          = 1γ       若 b carried                  （≥1 drop）
```

则 h_shelf(X) = min over injective τ of Σ_b LB[b, τ(b)] 是剩余
weighted-SOC 的 admissible 下界。

*证明梗概*：任何把 X 补完到 goal 的合法后缀都以某个 injective τ'
结束（命题 3 方向 ⇒ 取终态位置）。对每个 b：每个 loaded move 至多让
一个 shelf 的 d_upper_wall 减 1 且计费 ≥ α（δ ≥ 0 时匿名搬运只会
更贵）；lift/drop 计费 γ 且次数下界即 opLB。对 b 求和 ≥ Σ LB[b,τ'(b)]
≥ min_τ。∎（要求 α,β,γ,δ ≥ 0；负权重破坏此前提，与现实现的
"不主动拒绝负数"诚实声明一致，见 freshmen §13.2。）

**规模分级（与现有 ρ 的规模域同构）**：

- |B_tgt| ≤ `DD_TAU_HUNGARIAN_TGT`（默认 256）：exact
  LB-matching（复用 `TAPFAssignmentState`，见 §5.3 的单矩阵词典序
  编码与 moved-row 增量修复）；
- 超出规模域：退化为无匹配松弛
  `h_relax = Σ_b ( min_{g∈G_b} α·d(p_b,g) + opLB_carried/grounded(b) )`
  ——忽略 injectivity，仍 admissible（min ≤ 任意 τ' 项），只是更弱。
  单点集下两式相等且都等于现实现的 h_shelf（退化一致性）。

单点集特例：LB 矩阵每行一个有限元 ⇒ matching 恒等、
h_shelf = 现实现值——**现有全部 fixed-goal 实例的 h/f/剪枝行为
不变**（这是 debug.md v4 的 singleton-parity gate 依据之一）。

### 4.4 通往最优性 [已落地，主张不变]

g = 真实 physical cost（D5，matching 永不进 g）；duplicate 的
g-relax/rewire 已落地（`rewrite()`，`test_dd_rewire` 小图穷举最优
对照）；anytime 改进 + admissible-h 剪枝 + f_min 早停证书是当前可证
部分；eventually-optimal 不作主张。goal-set 只改 h 的形态（§4.3），
不触碰此结论。

---

## 5. 算法

### 5.1 总览 [已落地；τ 插入点用 (*) 标注]

```text
# 外层：两阶段 anytime（D14）
plan1 = phase(macro=on if |B_tgt| ≤ DD_MACRO_TGT, stop_at_first=true)
plan2 = phase(macro=off, incumbent=cost(plan1), 剩余预算)   # 从根重启
return better(plan1, plan2)

# 单相内部（TAPFPlanner::solve，唯一 loop）
OPEN.push(N0)
loop until deadline:
    N = OPEN.select()            # 首解前 DFS(back)；首解后加权 FOCAL(w=1.5)
    if is_goal(N.X): extract & continue     # §2.2 简化判定；f_min 早停证书
    （首扩且首解前且规模域内：注入 macro rollout child，D14）
    C = N.tree.pop()                         # partial operator constraint
    if depth(C) < |R|:
        r = N.constraint_order[depth(C)]     # 创建时冻结（D11）
        for op in legal_local_ops(r, N.X): N.tree.push(C + {r→op})
    X' = CarrierPIBT(N, C)                   # 内联检查 + apply_ops 终裁；G1
    if X' == ⊥: continue
    if canonical(X') in EXPLORED:
        g-relax/rewire；重访计数 += 1；每 8 次 re-guidance
    else:
        N' = make_node(X')
        (*) τ' = repair(τ_parent, moved target rows)      # §5.3
        (*) I' = build_requests(X', τ')                    # §5.4
        (*) ρ' = match(free robots, I', η 迟滞)            # §5.5
        h(N') += h_shelf(X')                               # §4.3/§5.7
        OPEN.push(N')
```

(*) 三行今天已存在于 `attach_carrier_guidance`（τ 恒等、requests、
ρ Hungarian+η、order 分层、h_shelf、livelock 信号挂接）；goal-set
增量 = 把"τ 恒等"替换为真 matching，其余调用结构不动。

### 5.2 Action-constraint tree（核心修改 1）[已落地]

low-level constraint 固定完整 local operator
`C[r] = (a_r, q'_r, κ'(r))`；分支因子 ≤ deg+3，与 LaCAM 的 deg+1
同阶；lazy FIFO 展开、`constraint_order` 冻结。实现：
`Constraint` op payload + `build_op_candidates`（solve 与 G1 枚举
共用的唯一生产实现；零 shelf 时候选集与 RNG 消耗和原 TAPF 逐位一致）。

### 5.3 第一层 assignment：τ（shelf→goal）[待实现；本轮核心]

**每个 node 求解/修复全局 injective matching**（designv2 §4：
禁止逐对 greedy elimination——§1.2 消融 3 已证明静态逐对配对在
执行层是负优化；搜索期的逐对 greedy 继承同样的结构缺陷）。

**两个 cost 语义、一次 Hungarian（本文的关键工程决策）**。
designv2 §4 要求区分 guidance matching cost（含 blocking/vacancy/
hysteresis 项）与 admissible matching cost（LB，§4.3）。直接照抄要
每节点解两次 matching。利用现有 `TAPFAssignmentState` 的词典序编码
（`weight = INF − (primary·cost_scale + tie)`），把两层语义压进
**一个整数矩阵**：

```text
C_τ[b,g] = LB[b,g] · S  +  pen[b,g]，          g ∈ G_b（否则 sentinel）
pen[b,g] = η_B · 1[g ≠ τ_parent(b)]           （+ 可选 blocking 微调项，见下）
S        = η_B · |B_tgt| + 1                   （保证 Σpen < S，词典序严格）
```

Hungarian 最优解 τ* 满足：

1. **主序最优**：Σ LB[b,τ*(b)] = min_τ Σ LB（因为 Σpen < S 不可能
   翻转主序）⇒ `h_shelf = result.cost / S`（整除向下取整即主序和）
   **恰是 §4.3 的 admissible 值**——一次求解同时产出 τ 与 h；
2. **副序 hysteresis**：主序 tie 内偏向 parent 配对（η_B 语义，
   defaults η_B=2 与 ρ 的 DD_ETA 同族），防止等价 goal 间震荡；
3. **确定性**：engine 自带 tie_hash 第三序，保证同 X 同 parent 下
   τ 可复现。

溢出预算：LB ≤ α·diam + 2γ（80×80 网格 diam≈160，权重≤5 ⇒ LB ≤
~10^3）；S ≤ 2·400+1；LB·S ≤ ~10^6 ≪ sentinel 10^8 ≪ long 编码域
（engine 现有 cost_scale/tie 机制原样可用）。

**blocking-aware 项的处置（v1 收窄）**：designv2 §4 的
λ_blk·B̂/λ_vac·V̂ 项若直接加进主序会破坏 h 的 admissibility，加进
副序则预算受限。v1 决策：**主序只用 LB**；blocking 感知交给下游——
τ 选定 goal 后，target 的 least-blocking path/requests/candidate
排序（§5.4，已落地）本来就是 blocking-aware 的。plateau 时的
re-guidance（§5.6 信号 B）额外允许 τ 禁忌重配（taboo 当前配对后
re-match），提供跳出局部配对的通道。全量 blocking-aware τ cost 列为
扩展消融（§8.4）。

**增量修复（复用 `repair_rows`）**：node 间只有 moved/lifted/dropped
的 target 行变化（p_b 变 ⇒ 该行 LB 全列变；κ 变 ⇒ opLB 变）。
`TAPFAssignmentState::repair_rows(changed_rows)` 已实现 moved-row
增量（现服务 agent-task 层）；τ 层第二实例化之。parent 的
`tau_state` 随 node 拷贝（与现有 `assignment_state` 同模式——DFS
分支切换时 potentials 随 parent 值语义正确）。每节点代价：
O(changed_rows · |G|) 期望，全量 O(n^3) 仅根节点与 re-guidance。

**settled 定义（node 级 derived）**：
`settled_N(b) ⟺ grounded(b) ∧ p_b = τ_N(b)`。只是 guidance
（requests 不为 settled target 发 serve）；settled shelf 仍可作为
blocker 被搬走（D2 不变）。注意与 `done(b) = grounded ∧ p_b ∈ G_b`
（§3.1，terminal/χ 用）区分：done 但未 settled 的 target（占着别人
更需要的 goal）会被 τ 重新指派并再次搬运——这正是 goal-set 语义下
"完成可逆"的新形态。

**零/单点集退化**：|G_b|≡1 ⇒ 矩阵每行一个有限元 ⇒ τ 恒等、
repair 无行可变、h_shelf 同现值；τ 层结构上等价于现实现（不允许
feature flag 切换——退化必须来自数据形状，与零 shelf 退化同一
纪律）。零 target ⇒ τ 层空结构、整层零调用。

### 5.4 Vacancy-centric shelf intent（requests）[部分落地]

每个未完成 target b 的临时目标直接取自 τ：`g_b^temp = τ_N(b)`。
在 upper deck 以 edge weight = 1 + λ_blk·1[occupied]（λ_blk=8）求
b→τ(b) 的 least-blocking path（Dijkstra；PathCache 惰性非对称失效 +
head-advance + path 惯性 tie-break，全部已落地），然后生成 requests：

```text
path_b = least_blocking_path(p_b, τ_N(b))
若 b 未被 carry 且 path[1] 无落地 shelf：  emit serve(p_b)      # 优先级 100
沿 path 取前 K=3 个 grounded blockers c_i： emit clear(c_i)     # 50-i
```

[待实现·frontier 修正] designv2 §5 的递归 vacancy request 在
one-empty（sliding-puzzle）regime 的本质是：**唯一真正可执行的
manipulation 是空格邻接的那个 blocker**，而它往往是链上离 target
最远的一个。现实现的 clear 优先级是 head-first（离 target 近者高），
在 puzzle 密度下让 robot 聚集在暂不可动的链头。修正为
**movability-aware 优先级**：

```text
movable(c) ⟺ ∃ u ∈ N(c): upper(u) 为空          # c 现在就能被搬走
priority(clear c_i) = (50 − i) + M·1[movable(c_i)]   # M=常数, frontier 前置
```

仍是 ordering-only；低填充下 movable(path[1]) 几乎恒真、行为不变，
高填充下自动切到"服务链叶"。designv2 的 AcquireTarget（把 robot 预置
到尚未进入 executable frontier 的 target 下方）映射为现有 serve 的
低优先级变体（head 未空时也 emit，priority < clear），列为同一 WP 的
可选项（消融决定去留）。递归深度即 chain 截断 K，仍是质量旋钮
（§12 开放问题 2）。

其余已落地机制原样：active-target cap（>256 时 64，carried 优先）、
protect/owner 标记、goal cells 保护。**goal-set 差异**：protect 只
标 τ 当前指派的 goal 格（不是整个 G_b 池——池可能覆盖整条边界，
全保护会饿死 parking）。

### 5.5 第二层 assignment：ρ（free robot→request）[已落地]

现实现即 designv2 §6 的形态（对照映射如实记录）：

| designv2 概念 | 现实现 |
|---|---|
| AcquireMove(s,u,v) | clear(u) request（放置建议由 parking 层给出） |
| AcquireTarget(b,u) | serve(u)（+ §5.4 的预置变体，待实现） |
| Dummy | 未匹配即 idle（角色 4），无显式 dummy 列 |
| C^R = d_L + λ_d·depth − μ·priority + η_R·switch | d_lower − η·1[沿用 parent 配对] + taboo 屏蔽；priority 决定参与匹配的行序/截取（λ_d、μ 未实现，列为扩展） |
| loaded robot 不参与 | 同（κ 是物理 hard binding，只能 Drop 解除） |
| temporaryTarget(r) = pickup(ρ(r)) | `free_goal[i]`（PIBT free-with-request 分支的目标格） |

规模域：Hungarian（`tapf_hungarian_row_to_col` 共享实现）
≤ `DD_RHO_HUNGARIAN_TGT`=256，否则 greedy nearest；匿名 shelf 的
task 以当前 upper cell 标识（不引入永久身份，D9 一致）。

### 5.6 Carrier-PIBT（核心修改 2）、park、livelock [已落地；τ 替换点标注]

候选序按角色分派（`funcPIBT`；task agent 分支上游逐字保留）：

| robot 状态 | 候选序（goal-set 下的唯一改动 = g_b 处处替换为 τ_N(b)） |
|---|---|
| loaded target b（未 park） | q_r = τ(b)：Drop 最优先；否则 path-head 优先 + 按 d(·,τ(b)) 排序的 loaded Move（S1 预过滤）> 结构性堵头时 Drop 前置 > Wait |
| loaded ANON / parked target | 朝 parking cell 的 Move > 到位 Drop > Wait |
| free 且 ρ 指派 | 朝 free_goal 的 Move（d_lower 场）> 到达且脚下 grounded 时 Lift > Wait |
| free idle | 避让 protect 区（DD_IDLE_AVOID）后 Wait |

预约语义按角色分派（WP6 实证教训，保持）：task agent 上游逐字
（失败保留预约）；carrier agent 释放-重试 + wait 可行即成功。
`N.order` 类分层 + 类内 (余距, id) 稳定序 + livelock 类内 shuffle；
`constraint_order` 冻结。

**Target-as-blocker park（§5.4a 语义继承，τ 交互新增）**：触发条件
"g_b 落在另一未完成 target o 的当前 path 上" 改写为
"**τ(b)** 落在 o 的 path 上"。goal-set 的结构性红利：τ 的下一次
repair 往往直接把 b 改派到不冲突的 goal，park 从常态降级为
fallback（B 型实例中预期 park 触发率显著下降，作为观测指标）。
carried-hover 掩蔽、owner 完成即释放、环打破（最小下标）、载具对头
yield（余距大者让）全部原样；park 仍是 (X, D_b 缓存纪元) 的确定性
函数、ordering-only。

**Livelock 与 wait-for [已落地]**：信号 A（guidance-h 连续 W=24 无
下降 → 类内 shuffle + ρ 禁忌 re-match）；信号 B（duplicate 重访每
8 次 → re-guidance + 允许重试 macro）；wait-for graph（robot→robot
下层占位边 + carrier→grounded shelf→clearer 跨层边，函数图环检测，
环成员定向禁忌）。[待实现] goal-set 增量：re-guidance 的禁忌集扩展
到 τ 配对——禁忌 (b, τ(b)) 后 repair 该行；**仅 |G_b| ≥ 2 的行可被
禁忌，单点行豁免**（否则该行被 sentinel 清空导致 matching 不可行；
豁免同时给出单点集零动作退化）。这给"配对级 plateau"一条合法多样化
通道——纯 fixed-goal 下禁忌集自动为空。guidance-h 的
target 项改为 d(p_b, τ_N(b))+2（随 τ 变化；仍 ordering-only，
不承诺单调）。

### 5.7 Cost 与 heuristic [部分落地]

- **g**：真实 physical cost（§2.3，D5），macro 边由 trace cost 记账
  （`macro_edges`）。不变。
- **admissible h**：h = agent-task 项（原 TAPF assignment cost）+
  h_shelf。h_shelf 由 §5.3 的单 Hungarian 主序值给出（规模域外用
  §4.3 的无匹配松弛）。单点集退化 = 现实现。
- **guidance-h（livelock 信号）**：Σ_unfinished d(p_b, τ(b)) + 2。
- FOCAL：首解前 DFS(back)；首解后原版加权 FOCAL（w=1.5，f<incumbent
  可行域，h 平局）。不变。

### 5.8 Macro rollout 与两阶段 anytime（D13/D14）[已落地]

event-bounded rollout（与 B0 共码）、规模域 |B_tgt| ≤ DD_MACRO_TGT、
首解前注入、phase-2 从根 primitive-only 重启 + phase-1 上界 f 剪枝、
返回两相更优。goal-set 增量为零：rollout 内部本就走同一 guidance/
generator 栈，τ 随节点重算。地址复用陷阱的既有防护
（`invalidate_carrier_scratch` 于每 rollout 步与探针回收后）保持——
τ 层若增加任何 address-keyed 缓存，必须挂进同一失效点（debug.md v4
把它列为 RED 测试项）。

---

## 6. 状态表示与工程

### 6.1 Canonical form 与 hash [已落地，不变]

key = SearchKey{Config, ShelfState}（labeled targets + 匿名 occupancy
排序 + labeled robots + κ）；hasher = ConfigHasher ⊕ shelf-Zobrist
（splitmix64 无表派生）。oracle 层增量 hash 有性质测试；集成 rollout
的局部去重仍用全量重算（吞吐机会，遗留）。**τ/G_b 不进 key**。

### 6.2 距离场缓存 [部分落地；含一项待实现的结构修正]

| 场 | 性质 | 策略 |
|---|---|---|
| d_upper_wall(·, g) | 静态（只依赖 wall） | **[待实现·修正] 合并为单一共享 `DDDistCache`（dest-keyed）**。现实现是 per-target 的 `target_goal_dist[b]`——同一 grid 同一 wall 集，字段只依赖目的格；固定 goal 下每 target 恰查一个 dest，冗余无害；goal-set 下 B 型池（如 80×80 边界 316 格）× per-target 对象会产生大量重复 BFS 与内存。共享后：查询键 = dest cell，惰性展开，B 型 80×80 全池 ≈ 316×6400×4B ≈ 8MB，可控 |
| d_lower(·, cell) | 静态 | `LowerDist`：wallfree 精确 Manhattan fast-path，否则 per-dest 惰性 BFS。不变 |
| D_b（blocking-aware path） | 依赖 occupancy | PathCache 惰性非对称失效（占用新增才重算）+ head-advance + 惯性 tie-break；`DD_STRICT_INVAL=1` 对照。**goal-set 增量：cache entry 增加 dst 字段，τ(b) 改变 ⇒ 该 target 行强制重算**（dst 不匹配视同 miss；否则 stale path 指向旧 goal，guidance 语义错误——这是待实现清单里的正确性项，不是优化项） |

### 6.3 复杂度预算（每次 expansion）[更新]

τ repair O(changed_rows·|G|) 期望（全量 O(n^3) 仅根/re-guidance；
规模域 ≤256 target 同 ρ 实测可承受）；requests O(k·|B_active|)；
ρ O(rows·free) 建矩阵 + Hungarian；Carrier-PIBT O(|R|·deg)；
距离场摊销。对比 ITA-LaCAM 的 per-node Hungarian + PIBT 同量级——
**goal-set 后每节点是两次（小规模）matching，与 designv2 §11 的
"同一 assignment engine 实例化两次"完全对应**：

```text
第一实例: TAPFAssignmentState(rows=B_tgt, cols=G_pool)   # τ, §5.3
第二实例: tapf_hungarian_row_to_col(rows=requests, cols=free robots)  # ρ, §5.5
（第零实例: 原 TAPF agent-task 层，零 task 时空转）
```

### 6.4 Validator-first 与测试锚 [已落地]

三方一致（planner 内联 / C++ apply_ops / Python validator）+ 跨语言
golden corpus + G1 穷举对照 + §6.5 极限单测（|R|=1 pebble 等价、
全占据 cycle rotation、单/双 blocker、idle 挡 lift）。goal-set 触及
的只有 `is_dd_goal`/`is_goal_config` 与 Python validator 的终止
判定——双侧同步修改、同一 fixture 双跑器断言（debug.md v4 WP-A）。

### 6.5 环境旋钮配置表 [默认值即基准配置]

继承 design.md §6.6 全表（DD_MACRO_*、DD_GUIDE_EVERY、DD_ACTIVE_CAP、
DD_STRICT_INVAL、DD_NO_YIELD、DD_ALPHA..DELTA、DD_SOLVER_WEIGHTS、
DD_ETA、DD_RHO_HUNGARIAN(_TGT)、DD_IDLE_AVOID、DD_PLACE_ESCAPE、
DD_NO_FOLLOWING、DD_DEBUG_DUMP），新增：

| 变量 | 默认 | 语义 |
|---|---|---|
| `DD_TAU_HUNGARIAN_TGT` | 256 | τ exact matching 的规模域上限；超出走 §4.3 无匹配松弛 h + 行内最近 goal 的 greedy τ（consistency：单点集下两者同型） |
| `DD_ETA_B` | 2 | τ hysteresis（词典序副序，§5.3） |
| `DD_CLEAR_FRONTIER` | 1 | 0 = 关闭 movability-aware clear 优先级（§5.4 消融） |

历史注记（DD_NO_ASTAR 从未存在于生产代码、DD_FOCAL_W 已退役）继承。

---

## 7. 扩展（不进本轮 critical path）

1. **carrier-aware LNS**（anytime 第二卖点）：首解后对 executed-
   makespan 关键路径上的 robot/shelf 子集 + 时间窗局部重搜。
2. **blocking-aware τ cost**（designv2 §4 完整形）：λ_blk·B̂ 进
   matching 副序或 plateau 期主序重算；先做消融再决定默认。
3. **λ_d·depth / μ·priority 进 ρ cost**；per-robot no-progress 动态
   提升；h_mk（makespan 型 admissible h）。
4. **lifelong / throughput 版本**；free-robot 匿名化（D4 注记的
   canonical 去重义务）；robots_return_to_rest / remove_on_complete
   flags（state/hash/goal 全链路支持后）。
5. **两层 wall 集不同时的 Sokoban 式 dead-cell drop-pruning**
   （v1 仍是 load 期可行性拒载的正确形态）。

---

## 8. 基线与实验

### 8.1 基线与方法名映射 [已落地]

| 名称 | benchmark 方法名 | 说明 |
|---|---|---|
| full method | `carrier` | 本文方法（goal-set 落地后含动态 τ） |
| B0 | `carrier_b0` | rollout-only（与 macro 共码），search 贡献的 ablation |
| B1 | `carrier_b1` | 冻结 least-blocking plan 为硬约束（decomposition 对照，刻意不完备） |
| B2 | `crest_base` / `crest_full` | 外部 SOTA 执行框架 |
| B3 | `natcbs` | ≤150 格小实例 optimality gap |
| B4 | `b4` | 单 robot 顺序模拟（B4-greedy，可诚实失败） |

官方结果基线：`results_integrated_v2/rows.csv`（164×7、统一 10s、
jobs=14；carrier 162/164）+ `results_brap*/`（68 实例 BRaP 协议、
carrier 34/68、near-boundary 静态重配对 18/68@carrier-only、
Hungarian 静态配对 32/68）。报表口径：carrier 系按内部 deadline；
b4/crest 按 wall+0.6s 重分类。

### 8.2 goal-set 落地的评估协议 [待实现]

1. **Singleton-parity（硬 gate）**：全部既有 fixed-goal 套件
   （small/standard/paper/sweep，164 例）在 goal-set 代码路径下数字
   不回归（成功集与 makespan 逐例对照；τ 层单点集退化 §5.3 保证
   结构等价，benchmark 复跑钉实）。
2. **BRaP-B 动态 τ 主实验**：instances_brap 的 B 型实例改发全边界
   goal pool（同 seed 同布局，仅 goal 结构变化；R1 型天然单点集，
   作 within-suite 对照）。对照组：static-greedy（原 34/68 数据）、
   static-Hungarian（32/68）、static-near-boundary（18 例 carrier-only
   数据）。预期与 gate：≤10×10 成功数 ≥ 34 基线；common-solved
   makespan 显著优于 near-boundary 静态（它已比原始好 8.5×，动态 τ
   的增益在其之上度量）；≥20×20 不设成功 gate（horizon 墙，§1.2）。
3. **消融**（run_ablations 扩展）：dynamic-τ vs frozen-τ（root 一次
   matching 后锁死——隔离"每节点重算"的贡献）；η_B on/off；
   frontier-first vs head-first clear（DD_CLEAR_FRONTIER）；
   τ-taboo re-guidance on/off。
4. 既有 sweep 轴（robot:shelf 比例、γ 权重后处理、填充率、|B_tgt|、
   scramble depth）与指标（success/first-solution/executed makespan/
   weighted SOC/utilization/shelf switches）继承不变。

### 8.3 语义变体汇报纪律 [已落地]

BRAP_NO_FOLLOWING（DD_NO_FOLLOWING=1，oracle 层）与默认
PHYSICAL_ALLOW_FOLLOWING 两组结果永不混合汇报（designv2 §8）；
与 BRaP 抽象层数字对比时必须声明本方法输出的是可执行 plan（robot
timestep 计数），对方是 block-move 计数。

---

## 9. 相关工作与定位

一句话贡献（在 design.md v2.2 收窄版之上加入 τ 层）：

> A LaCAM-style search over **executable robot-shelf physical
> configurations** with operator-level lazy constraints,
> per-configuration guidance, and cross-deck conflict handling —
> feasibility-first with anytime cost improvement — **where both the
> shelf-to-goal assignment and the robot-to-task assignment are
> per-node revisable orderings, never commitments**。completeness
> 依赖 §4.2 骨架（G1 直通 + 冻结 constraint order，已实现并有穷举
> 对照）；最优性只在 admissible-h 剪枝意义下 anytime 逼近。

对照表继承 design.md §9（BR-LaCAM/BRaP、DD-MAPD、CREST、NAT-CBS、
MARPF、M-PAMO、ITA-LaCAM、pebble motion/Gue&Kim/Sokoban），新增
一行定位：

| 工作 | 与本文关系 |
|---|---|
| ITA-LaCAM（本组） | 方法论母体。本文把它的"assignment 是每 node 重算的 guidance"从 agent→task 推广到 **shelf→goal（τ）与 robot→manipulation（ρ）双层**，且 g 保持纯物理（D5），修正其 assignment-dependent g 的理论瑕疵 |

不声称"首个 joint robot-object planning"；声称：首个在每条搜索边上
保证联合可执行、保持 LaCAM 级 scalability、且 **goal 指派对搜索
在线可撤销**的方法。命题 2 的可行域分离与 §3.4 的 no-following
去保守化主张不变。静态配对的负结果（§1.2 消融 2/3）作为动机证据
写入论文实验节。

---

## 10. 里程碑与代码落点

### 10.1 里程碑状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| M0-M3 | 语义 spec/validator/scrambler；primitive Carrier-LaCAM；ρ/fields/FOCAL/anytime/Zobrist；B0-B4/sweep/ablation | **完成**（design.md §10 v3.1 记录，全量数字 §8.1） |
| M4 部分 | macro（D14 两阶段）、wait-for | **完成** |
| **M5（本轮）** | goal-set 实例格式 + τ 层（§5.3）+ frontier requests（§5.4）+ BRaP-B 评估（§8.2） | **待实现**（工作包 = debug.md v4） |
| M6 | LNS / lifelong / blocking-aware τ | 论文扩展 |

### 10.2 代码落点（唯一 solve loop 纪律不变）

用户指令（最高优先级，继承）：一切增量建立在现有集成代码上；
**禁止**平行 planner、第二套 search pipeline、feature-flag 切换的
legacy 路径。零 shelf 逐位退化（golden 特征化测试）与单点集结构
退化（§5.3）都必须来自数据形状。

| 机制 | 落点 |
|---|---|
| goal pool/eligibility 加载 + 覆盖匹配检查 | `dd_carrier.cpp::load_dd_instance/finalize`（DDInstance 增 `goal_pool`/`target_goal_sets`；单点集向后兼容），`TAPFInstance(const DDInstance&)` 透传 |
| terminal（命题 3 简化式） | `dd_carrier.cpp::is_dd_goal` + `tapf_planner.cpp::is_goal_config`（per-target goal bitset）+ `ddbench/validator.py` 同步 |
| τ matching/repair/h_shelf | `carrier_guidance.hpp`（CarrierEngine 持共享 upper-wall DDDistCache + τ 状态），`attach_carrier_guidance` 内在 build_guidance 之前求 τ；`TAPFNode` 增 τ 向量 + `tau_state`（与 assignment_state 同模式） |
| requests/park/yield/waitfor 的 g_b→τ(b) 替换 | `carrier_guidance.hpp::build_guidance/waitfor_cycles`（dst 参数化；PathCache entry 增 dst 失效） |
| funcPIBT loaded 分支 d(·,τ(b)) | `tapf_planner.cpp::funcPIBT`（经 guide 取 τ，避免 planner 直读 goal 集） |
| 共享距离缓存合并 | `CarrierEngine::target_goal_dist` per-target 向量 → 单 `DDDistCache upper_wall`（固定 goal 语义不变，先行重构可独立验证） |
| adapters/CLI | `dd_planner.cpp` 探针与 `dd_benchmark` 合同不变；B1 冻结 plan 语义对 τ 取根节点值（基线语义：静态 decomposition 对照更纯粹） |

### 10.3 删除/替换清单（终态不得残留）

- 生成器中的静态配对逻辑（benchmark 侧 greedy/Hungarian pairing）
  降级为**消融专用变体生成器**，主生成器输出 goal-set 实例；
- `target_goals` 单一向量语义的隐式假设（逐处替换为 goal-set 查询
  或 τ 间接层）；audit 用 grep 清单钉住（debug.md v4 WP-审计）。

---

## 11. Decision Log

D1-D14 继承 design.md §11 原文（no-following、completed 可逆、
1-step lift/drop、labeled robots、纯物理 g、canonical key、命名、
4-连通、ANON、grounded goal 条件、冻结 constraint_order、derived χ、
event-bounded macro、两阶段 anytime）。新增：

| # | 决定 | 理由 |
|---|---|---|
| D15 | goal 结构 = per-target 候选集 G_b + 共享池；loader 禁止任何静态配对/采样固化 | §1.2 两个消融：静态选择留下 8.5× 质量损失；最优静态配对反而更差——正确配对依赖演化 occupancy |
| D16 | terminal 用逐 target 简化式（命题 3），不在线求匹配 | S1 保证位置单射 ⇒ 与 injective-τ 判定等价；O(|B|) 且 τ-free（completeness 骨架第 5 条） |
| D17 | τ 与 admissible h 用**一次** Hungarian：主序 = LB（admissible 值），副序 = η_B hysteresis，第三序 engine tie_hash | 两个矩阵两次求解的开销减半；主序/副序词典分离同时保住 admissibility 与稳定性（§5.3 编码） |
| D18 | blocking-aware τ cost 不进 v1 主序 | 进主序破坏 h admissibility；blocking 感知已由下游 path/requests/候选序承担；plateau 由 τ-taboo re-guidance 兜底；完整形留扩展消融 |
| D19 | τ hysteresis 默认开（η_B=2） | 与 ρ 的 DD_ETA 同族经验；等价 goal 间震荡在共享池下是必然模式，必须默认抑制 |
| D20 | clear 优先级 movability-aware（frontier-first），旋钮可回退 | one-empty regime 唯一可执行 manipulation 在链叶；head-first 在 puzzle 密度下让 robot 空等（§5.4）；低填充下两者行为几乎相同 |
| D21 | upper-wall 距离场合并为单 dest-keyed 共享缓存 | 字段只依赖 wall+dest；per-target 对象在共享 goal 池下产生 O(|B|·|pool|) 重复 BFS/内存 |
| D22 | 单点集退化 = 结构等价（禁 flag）；既有 164 例套件作 parity gate | 与零 shelf 退化同一纪律：兼容性来自数据形状；回归风险由 benchmark 复跑钉死 |
| D23 | B1 的 τ 取根节点静态值 | B1 是 decomposition 对照基线——goal 也静态才是对"先定后执行"的忠实模拟，与 full method 的差异变量保持干净（"逐 configuration 重算" vs "冻结"） |

---

## 12. 开放问题

1. chain 截断 K 与 solution 质量（高密度）；AcquireTarget 预置的
   净收益（消融后定去留）；
2. blocking-aware τ cost 的正确进入方式（副序预算 vs plateau 重算）；
3. DnE-M 质量 gap（+12%，遗留 gate，固定 goal 家族——τ 层不触及，
   需独立的性能工作）；集成 rollout 增量 hash；
4. eventually-optimal 的正式论证；park 判定的严格纯化；
5. lifelong 化后 τ 的跨 episode 迟滞；free-robot 匿名化与 rewiring
   的相容性；
6. ≥20×20 puzzle 密度的 horizon 墙：需要层级化（corridor-level
   macro）或有界次优跳步——超出本轮范围，如实记录。

（design.md v2 的"D_b 增量维护"已由惰性失效关闭；"η 迟滞/往返震荡"
已落地并默认开。）
