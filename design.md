# Carrier-LaCAM 设计文档 (v2)

状态: draft v2.3, 2026-08-30。
v2.3 变更(吸收第二轮独立审计, 见 debug.md 重写版):定理 1 修正为
$|R|=1$ 并给出多 robot 反例(§4.1);§4.3/§5.1 撤回 LaCAM* rewiring 与
eventually-optimal 声明(实现无 g-relax/rewire);新增两阶段 anytime 与
macro 规模域规则(§7.1, D14);§5.4a park 纯度声明降级为 (X, D_b 缓存纪元)
的确定性函数;§5.3(4) η hysteresis 标注未实现;§5.7 guidance-h 按实现
如实描述(approach 项因吞吐被移除);§6.2 修正 A*→Dijkstra;新增 §6.6
环境旋钮配置表;§8.1 补方法名映射;§8.2/§8.4 标注实际覆盖范围与
无效 no_astar 消融的处置;§10/§11/§12 状态同步。
v2.2 变更(吸收实现期独立审计与实证, 见 debug.md):flags 接口移出 v1(§2.2);
新增 target-as-blocker parking 机制章节(§5.4a);§5.5 更新为已实证的
多信号方案;§5.6 placement 记录被否决假设;§6.1 Zobrist 落地;§6.2 惰性失效
与 §6.4 双 validator 架构表述修正;命题 2 增加同实例可执行论证(§4.1);
B4 表述拆分(§8.1);贡献声明按 M1 实证收窄(§9);里程碑状态如实标注(§10)。
v2.1 变更(吸收外部评审 R1):修正 $h_{\mathrm{mk}}$ 的 admissibility
off-by-one;χ 降级为 derived view(不进 state/key);命题 2 按 DD-MAPD 原文
查证后收窄;补 `N.order` 定义;macro 重定义为 event-bounded rollout(与 B0
共码);$D_b$ 惰性失效;实例 scrambler;新增 D11–D13。
本文取代 `design.md`(原文件是两轮讨论笔记的拼接,notation 不一致;v2 统一符号、
补齐语义规范,并加入理论锚点、deadlock 处理、baseline 与里程碑)。

---

## 0. 一句话

> 不把 shelf path 和 robot path 当成两个 plan;它们是**同一条 physical
> configuration path 的两个投影**。搜索只在物理 configuration 空间上进行,
> robot 是唯一 actuator,shelf 的移动是 robot 动作的确定性 effect。

对应到 LaCAM 框架,需要改的只有两件事:

1. high-level state 从 agent configuration 扩展为 robot + shelf + carrying 的
   物理 configuration;
2. low-level constraint 从 "agent 的 next vertex" 提升为 "robot 的 primitive
   operator"(因为 wait / lift / drop 的 next vertex 相同,仅约束位置无法区分
   transition)。

这两点是本方法的核心贡献。其余(matching、PIBT、FOCAL、anytime)都是
ITA-LaCAM 已有机制的迁移。

---

## 1. 背景与动机

- **BR-LaCAM / BRaP**(arXiv [2509.01022](https://arxiv.org/abs/2509.01022))
  搜索的是 block-level rearrangement:block 被当作会自己移动的 agent,
  robot 位置、carrying 状态、lift/drop 完全不在 search state 里。
  它假设 block plan 总能被执行,但从不验证。
- **DD-MAPD / MAPF-DECOMP**(arXiv [2304.14309](https://arxiv.org/abs/2304.14309))
  是 shelf-first decomposition:先给 shelves 规划轨迹,再切段分配给 robots。
  active robot 盲目跟随预算好的 shelf trajectory;论文自己列出的 future work
  就是让 shelf planning 感知 robot execution。
- **CREST**(arXiv [2603.28803](https://arxiv.org/abs/2603.28803))保留两阶段
  结构,只在执行期 release constraints;它同时证实了 lift/place overhead 越大,
  decomposition 的损失越大。
- **NAT-CBS**(SOCS'25)是最接近的 coupled search:obstacle plan + network-flow
  realization check,makespan-optimal 但比 MAPF-DECOMP 慢约四个数量级。

空档很清楚:**没有一个方法在搜索的每条边上都保证 robot–shelf 联合可执行,
同时保持 LaCAM 级别的 scalability 与 completeness。**

---

## 2. 问题定义

### 2.1 环境与实例

- 4-连通 grid,双层语义(DD-MAPD 风格):
  - **lower deck**:robots 行驶层。free robot 可以从 grounded shelf 正下方
    通过与停留(仅受 wall 限制)。
  - **upper deck**:shelves 层。grounded shelf 静止;被 lift 的 shelf 随
    carrier robot 移动。
- 实例输入:
  - grid(walls);
  - robots $R$,初始位置 $Q^R_0$(labeled);
  - shelves $B$,初始 occupancy;其中 target shelves
    $B_{\mathrm{tgt}}\subseteq B$ 有 label,其余 shelf **匿名**;
  - goal 集:每个 $b\in B_{\mathrm{tgt}}$ 一个固定 goal $g_b$
    (v1;goal 候选集 + 动态 matching 是 phase 3,见 §7.3)。

### 2.2 Goal condition(v1 之前的文档缺失,现在钉死)

状态 $X$ 是 goal 当且仅当:

$$
\forall b\in B_{\mathrm{tgt}}:\quad
p_b=g_b
\;\wedge\;
b \text{ grounded(没有 robot 正 carry 它)}.
$$

- robot 终态不约束;
- 到达 goal 的 target shelf **留在原地继续占格**(D2;completed 可逆)。
- (v2.2) `robots_return_to_rest` / `remove_on_complete` 两个可选 flag
  **移出 v1 接口**:v1 loader 遇到非默认值将显式报错拒载(fail loudly),
  而非静默忽略;二者移入 §7 扩展(实现时需 state/hash/goal 全链路支持)。

### 2.3 目标函数

两个 objective,分开报告:

- **executed makespan**:第一个到达 goal condition 的 timestep;
- **weighted action cost(SOC 风格)**:

$$
c(\mathbf a)=
\alpha\cdot\#\text{loaded moves}
+\beta\cdot\#\text{free moves}
+\gamma\cdot\#\text{lift/drop}
+\delta\cdot\#\text{anonymous-shelf moves}.
$$

注意 "executed makespan" 是关键指标:decomposed 方法报告的 shelf-plan makespan
不等于真实执行 makespan,这正是统一模型的核心卖点。

---

## 3. 模型语义(规范)

### 3.1 状态

$$
X=\left(Q^R,\;Q^B,\;\kappa\right)
$$

| 分量 | 含义 | 表示 |
|---|---|---|
| $Q^R$ | robot 位置(labeled) | `vector<Vertex*>` |
| $Q^B$ | shelf 层 | $\bigl(P^{\mathrm{tgt}},\,O\bigr)$:labeled target 位置 map + 匿名 occupancy bitset |
| $\kappa$ | carrying 关系 | $\kappa(r)\in\{\bot\}\cup B_{\mathrm{tgt}}\cup\{\textsf{ANON}\}$ |

**χ(completed 集)是 derived view,不是 state 分量**(v2.1 修正,D12)。
默认语义(`remove_on_complete=off`)下

$$
\chi(b)\;\Longleftrightarrow\;p_b=g_b\ \wedge\ b\ \text{grounded},
$$

完全可由 $X$ 推导(node 内缓存、增量维护,不进 canonical key)。
若把 χ 独立存储并由 `Drop` 单调置位,会出两个 bug:(a) dense 场景中
completed target 常须再次被搬走给别的 shelf 让路,单调 χ 与位置脱节,goal
判定出错;(b) 同一物理布局因 χ 不同被当成两个状态重复展开。
`remove_on_complete=on` 时,移除本身改变 $Q^B$(target 从地图消失),
"已移除"仍可由缺席推导,同样不需要独立 χ。

两个关键点:

- $\kappa(r)=b \Rightarrow q_r=p_b$(carrier 与 shelf 同格不变量);
- **被 carry 的匿名 shelf 不需要身份**:$\kappa(r)=\textsf{ANON}$ 只是一个
  flag。搬哪个 blocker 都一样,这在 grounded 匿名化基础上进一步收缩状态空间。

**temporary guidance(τ、ρ、距离场、requests)一律不进 state key。**
`EXPLORED` 的 key 只有 $X$ 的 canonical form(见 §6)。

### 3.2 Robot primitive actions

每个 robot 每 timestep 选一个:

$$
a_r\in\{\operatorname{Wait},\ \operatorname{Move}(v),\ \operatorname{Lift},\ \operatorname{Drop}\}
$$

lift/drop **无参数**:robot 只能 lift 自己脚下的 shelf、drop 自己举着的 shelf。
per-robot preconditions:

| action | precondition | effect |
|---|---|---|
| `Wait` | — | 无 |
| `Move(v)` | $v\in N(q_r)$;loaded 时 $v$ 的 upper 层在本步结束时无其他 shelf | $q_r\leftarrow v$;loaded 时 $p_{\kappa(r)}\leftarrow v$ |
| `Lift` | $\kappa(r)=\bot$;$q_r$ 上层存在 shelf 且该 shelf 在**本步开始时 grounded** | $\kappa(r)\leftarrow$ 该 shelf(target 则带 id,否则 ANON) |
| `Drop` | $\kappa(r)\neq\bot$ | shelf grounded 于 $q_r$;$\kappa(r)\leftarrow\bot$。completed 按 §2.2 谓词推导,**不在此置位**(`remove_on_complete=on` 时:若 $q_r=g_{\kappa(r)}$,该 target 从 $Q^B$ 移除) |

joint action $\mathbf a=(a_r)_{r\in R}$,$X'=F(X,\mathbf a)$。
**只有 robot 选动作;shelf 不独立选动作。**

### 3.3 Joint transition 合法性:冲突规则表

| # | 层 | 规则 |
|---|---|---|
| R1 | lower | 任意两 robot 在 $t{+}1$ 不同格(vertex conflict) |
| R2 | lower | 任意两 robot 不交换位置(edge/swap conflict);following 允许(标准 MAPF 语义) |
| S1 | upper | 任意两 shelf 在 $t{+}1$ 不同格。grounded shelf 不动;carried shelf 随 carrier。 |
| S2 | upper | shelf swap 不可能单独发生:只有 carried shelf 会动,两个 carried shelf 交换 ⟺ 两个 loaded robot 交换 ⟺ 已被 R2 禁止。**由 R2 蕴含,无需单独检查。** |
| I1 | 交互 | lift 的对象必须本步开始时 grounded;本步被 drop 的 shelf 本步内不可再被 lift(原子性)。 |
| I2 | 交互 | 一个 shelf 至多一个 carrier。由 R1 + `Lift` precondition 自动保证(同格至多一个 robot)。 |
| I3 | 交互 | free robot 可以位于 grounded shelf 下方;loaded robot 进入的格子受 S1 约束。 |

### 3.4 两个刻意的语义决策(与 BRaP 不同)

**(a) 不继承 no-following 约束。** BRaP 禁止 block 进入同一步刚被另一 block
腾出的格子,因为抽象模型必须保守地假设"不知道有没有 robot 能及时钻到下面"。
显式 robot 模型里,convoy 就是两个 loaded robot 同步移动,物理完全可行,
标准 MAPF following 语义即可。统一模型**约束更少、makespan 更优**,
这本身是论文卖点:decomposition 为了可执行性必须保守,统一搜索不需要。
(对比 BR-LaCAM 时须做语义归一化实验,见 §8.4。)

**(b) 零空格 cycle rotation 是合法 transition。** 全占据 cycle 上,若每个
shelf 都有 carrier,整体旋转一格是合法的(MAPF cycle rotation,R1/R2/S1 均
满足)。所有依赖 empty cell 的 BRaP 类 / sequential pebble-motion 方法都
表达不了这类解。见定理部分。

---

## 4. 理论

### 4.1 可行性

**定理 1(单 robot 模拟 ⇒ 可行性下界;v2.3 修正适用域为 $|R|=1$)。**
若 lower deck(仅考虑 wall)连通且 $|R|=1$,则:upper deck 上的
sequential pebble-motion plan(一次一个 block 移一格,目标格为空)可行
⟹ Carrier 问题可行。
*证明梗概:* 单 robot 依 plan 顺序执行:free 驶至 block 下方(free robot
仅受 wall 限制)→ lift → loaded 移一格(目标格空,S1 满足)→ drop →
驶向下一 block。每步 precondition 均满足。∎

**$|R|\ge 2$ 时该构造不成立(v2.3,审计反例)**:多余 robot 本身占据
lower deck,可能使系统自锁。最小反例:1×2 连通图、2 个 robot 占满
下层、1 个 shelf 需从左格移到右格——upper 层 sequential move 存在,
但全系统仅 2 个可达物理状态且 goal 不可达(两 robot 无处可让)。
推广 $|R|>1$ 需附加条件(如 lower deck 恒存在空闲格且可重排),
v1 不作此主张。原 v2.2 注记仍成立并加强:**可行性对 $|R|$ 不单调**,
不要顺手声称"更多 robot 不损害可行性"。

定理 1 的构造与 DD-MAPD III-B 的 BASE 算法(单 agent 锁步执行 1-robust
shelf 轨迹)本质相同,**不作为独立新颖性主张**;它的价值是把可行性接到
Kornhauser 等的 pebble-motion 多项式刻画上:**可行性判定不是本问题的难点;
难点在优化**。

**命题 2(表达力分离;范围已按 DD-MAPD 原文查证收窄,v2.1)。**
存在实例——零 empty cell 的全占据 cycle,$|R|\ge$ cycle 长度——使
Carrier 问题可行,但以下模型均不可解:

1. sequential pebble motion(一次一格、目标格须为空);
2. BRaP / BR-LaCAM 的 block 模型(no-following 约束);
3. **MAPF-DECOMP(PP)**——DD-MAPD 中唯一 complete 的变体:它要求 shelf
   轨迹 *safe 1-robust*,而 1-robust("下一步不得占据当前已被占据的格子",
   DD-MAPD Def. 1)恰是 no-following;且零空格实例不存在 1-robust 解,
   连 well-formed 都不是。

(v2.2 实证锚点)同实例分离已固化为可执行测试:固定 zero-empty 2×2 cycle
fixture 上,(a) Carrier-LaCAM 解出整体旋转;(b) 穷举证明该实例上**每个含
移动的合法转移都要求 following**(mover 进入本步被腾出的格),故任何
no-following 模型在此实例只能 lift/drop/wait,shelf 永不移动 ⇒ 不可解;
(c) B4 的顺序执行器因无空格诚实失败(定理 1 的适用域边界)。见
`tests/test_dd_g1.cpp::dd_prop2_*` 与 `benchmark/tests/test_prop2.py`。

对 **base MAPF-DECOMP 分离不成立**:其 shelf 层是标准 MAPF(允许 following
与 rotation),执行层通过 soft-dependency cycle 机制(Update/FindNoMove、
AssignAndPlan 第 (3) 步)显式支持"整环同步执行",条件是 $N\ge$ 环长
——论文失败原因 (b) 正是 $N$ 不足。因此对 base MAPF-DECOMP,差异必须
表述为 **completeness 与 executability-awareness**(它整体不 complete,
且 shelf plan 生成时不感知 robot 可达性),而非可行域包含。
论文写作严格按此三分表述,避免 overclaim。

### 4.2 Completeness

沿 LaCAM 的证明骨架:

1. grid、robots、shelves、$\kappa$ 有限 ⟹ 物理 configuration
   空间有限(χ 为 derived,不引入额外维度);
2. 每个 high-level node 的 **action-constraint tree** 最终枚举所有 robot
   primitive-operator 组合(§5.2);
3. transition validator(§6.4)恰好接受所有满足 §3.3 的 joint transition;
4. $\tau$、$\rho$、距离场、priority、deadlock 处理**只改变 successor 的
   生成顺序,不永久删除任何 successor**;
5. `EXPLORED` key 只含 $X$(canonical form);
6. ⟹ 所有可达物理 configuration 最终被枚举,算法对可行性 complete。

两个必须遵守的 guard:

- **G1**:generator(Carrier-PIBT)在 partial constraint 下允许失败,但当
  constraint 覆盖全部 robots 时,必须走 deterministic validator——合法的
  joint action 必须被接受。"PIBT 没找到" ≠ "successor 不存在"。
- **G2**:一切 pruning 只能剪 **provably dead** 的状态(§5.6),或 cost 剪枝
  ($f\ge$ incumbent)。任何 hard task commitment、"只允许某 robot 服务某
  shelf"、"只保留 relaxed planner 给出的 move" 都会破坏 completeness,
  一律只能作为 ordering。

**与两阶段 anytime(D14)的关系(v2.3)**:completeness 是**无时限的
phase 级主张**,不是 10s 预算内的保证。phase 1(macro 开)保留完整
primitive 约束树,macro 只增不减 successor,故 phase 1 单独即满足上述
骨架;phase 2 从根重启的 primitive-only 搜索以 phase 1 的 incumbent 作
$f$ 剪枝上界(admissible-h 下不剪掉更优解),且最终返回两相中的更优
incumbent——已找到的解不会丢失。因此两阶段结构不削弱 feasibility
completeness,也不引入新的剪枝风险。

### 4.3 通往最优性

- $g$ 使用**真实 physical cost**(§5.7),matching 只进 guidance/heuristic
  ——这与现有 LaCAM-TAPF 中 assignment-dependent $g$ 的做法不同,是刻意
  收紧,理论叙述会干净得多;
- (v2.3 收窄)**当前实现不做 LaCAM* 的 g-relax/rewire**:duplicate
  命中 EXPLORED 时只做重访计数与 re-guidance,不比较 $g$、不换 parent。
  后到的更廉价路径会被先占坑的贵路径屏蔽,故 **eventually-optimal 不
  成立,也不声称**。v1 主张:complete(§4.2 意义)+ anytime 改进
  (admissible-h 的 $f$ 剪枝保证 incumbent 单调不增,小例上可达最优并
  以 $f_{\min}$ 早停证书判定,见 `test_dd_anytime`);
- 匿名 shelf 的 quotient space 上 cost 对 relabeling 对称,canonical
  representative 保 cost——若未来实现 reopen/relax/rewire,LaCAM* 的
  eventually-optimal 论证可望恢复(列入扩展,须补穷举小图最优性对照)。

---

## 5. 算法

### 5.1 总览

```text
# 外层:两阶段 anytime(D14)
plan1 = phase(macro=on if |B_tgt| <= DD_MACRO_TGT, stop_at_first=true)
plan2 = phase(macro=off, incumbent=cost(plan1), 剩余预算)   # 从根重启
return better(plan1, plan2)

# 单相内部
OPEN.push(N0)
loop until deadline:
    N = OPEN.select()                     # 首解前 DFS(back);首解后 FOCAL-lite
    if is_goal(N.X): extract & continue   # anytime;f_min 早停证书
    C = N.tree.pop()                      # partial operator constraint
    if depth(C) < |R|:                    # low-level lazy expansion
        r = N.constraint_order[depth(C)]  # 创建时冻结(P0-1)
        for op in legal_local_ops(r, N.X):
            N.tree.push(C + {r -> op})
    X' = CarrierPIBT(N, C)                # constraint-respecting generator
    if X' == ⊥: continue
    if canonical(X') in EXPLORED:
        重访计数 += 1;每 8 次 re-guidance(禁忌配对、允许重试 macro)
        # v2.3 如实:无 LaCAM* relax/rewire(见 §4.3)
    else: N' = make_node(X'); lazy_guidance(N'); OPEN.push(N')
```

与 ITA-LaCAM 完全同构;差异集中在 state、constraint 的类型和 generator。

### 5.2 Action-constraint tree(核心修改 1)

low-level constraint 固定的不是 next vertex,而是**完整 local operator**:

$$
C[r]=\bigl(a_r,\ q_r',\ \kappa'(r)\bigr)
\quad\Longleftrightarrow\quad
(q_r,\kappa(r))\to(q_r',\kappa'(r)).
$$

原因:`Wait` / `Lift` / `Drop` 的 next vertex 相同但产生完全不同的后继可行域,
仅约束位置无法枚举完整 transition 空间。每个 robot 的分支因子约
$\deg+1+2\le 7$(4-连通),与 LaCAM 的 $\deg+1$ 同阶,lazy 结构不变。

好处:虽然 state 含 robots + shelves,**每步需要枚举的决策变量只有 robot 的
action**,数量由 $|R|$ 决定;shelf 只是被驱动的状态变量。

### 5.3 Node guidance(全部 lazy、全部不进 state key)

按依赖顺序,每个新 node 上:

**(1) $\tau$:shelf→goal matching。** v1 中 goal 固定,$\tau$ 恒等,跳过。
phase 3 接入 ITA-LaCAM 的 incremental Hungarian(只有 moved shelf 的 row 变)。

**(2) blocking-aware 距离场 $D_b$。** 对每个未完成 target $b$:
在 upper deck 上以 edge weight $=1+\lambda_{\mathrm{blk}}\cdot\mathbf 1[\text{occupied}]$
求 $b\to g_b$ 的 least-blocking 最短路(实现为 Dijkstra,无启发项——
v2.3 修正措辞,原文误写 A*;cache 与失效策略见 §6.2)。**不要用 1-step
lookahead**——高密度下过于短视;least-blocking path 与 BR-LaCAM 的
guidance 对齐,且只是 ordering,截断/过期都不伤 completeness。

**(3) requests $\mathcal M_X$。** 沿每个未完成 target $b$ 的 least-blocking
path 生成短期 manipulation requests:

```text
path_b = least_blocking_path(b, g_b)
u = path_b 的第一格
if u 为空(upper):        emit serve(b)          # carrier 推进 b
else:                      沿 path_b 取前 k 个 blockers c_1..c_k
                           emit clear(c_i, 放置建议区)   # k 默认 3
if b 无 carrier:           serve(b) 蕴含 "先到 b 下方 lift"
```

`clear` 的放置建议区 = 距 path 至少 1 格、placement score(§5.6)合格的空格。
priority 沿 blocker chain 从 target 向外传递(chain 头 urgency 最高)。

**(4) $\rho$:free robot → request matching。** min-cost matching
(v1 先 greedy nearest + 冲突退避,M2 换 Hungarian / min-cost flow):

$$
C^{R}_{r,q}=
d_{\mathrm{lower}}(q_r,\,\mathrm{cell}(q))
+\lambda\,\widehat d_{\mathrm{carry}}(q)
+\eta\,\mathbf 1[\rho_{\mathrm{parent}}(r)\neq q]
-\mu\,\operatorname{priority}(q).
$$

- $d_{\mathrm{lower}}$ 只看 wall(free robot 穿 shelf 下方),静态可 cache;
- $\eta$ 抑制逐步换配对(hysteresis)——**(v2.3 如实)未实现**:当前为
  纯 greedy nearest,无 parent 配对记忆;可视化中观察到的机器人往返震荡
  部分归因于此(配对翻转),η 项列入待验证改进(见 debug.md);
- loaded robot 不参与 matching——它的 physical binding 由 $\kappa$ 固定,
  只能通过 `Drop` 解除;
- dummy request 允许 idle。

### 5.4 Carrier-PIBT(核心修改 2)

先 apply $C$ 中固定的 operators(precondition 校验,失败返回 $\bot$
——partial constraint 下允许);再按 `N.order` 对其余 robots 走 PIBT。
候选动作排序:

| robot 状态 | 候选序 |
|---|---|
| loaded,$\kappa(r)=b\in B_{\mathrm{tgt}}$ | 若 $q_r=g_b$:`Drop` 最优先;否则:按 $D_b$ 下降排序的 loaded `Move` > `Wait` > 带 placement score 的 `Drop`(让路手段) |
| loaded,$\kappa(r)=\textsf{ANON}$(clear 任务) | 朝放置建议区的 loaded `Move` > 到位后 `Drop` > `Wait` |
| free,$\rho(r)=\operatorname{serve/clear}(b)$ | 朝 $p_b$ 的 `Move`($d_{\mathrm{lower}}$ 场)> 到达且 $b$ grounded 时 `Lift` > `Wait` |
| free,unassigned | 避让规则:**不停在任何被 request 的 shelf 正下方**(会物理性阻塞 lift,见 I2/I3)、不停在 target 的 least-blocking path 上;复用现有 hindrance 项 |

- lower-deck vertex 竞争 → 标准 PIBT priority inheritance;
- **cross-layer priority 只作用于 requests 与 matching 权重**,不是同步继承
  (见 §5.5 的时间尺度讨论);
- fully constrained 时直接走 validator(guard G1)。

由此形成的跨层 priority 链:

$$
\text{target shelf}\ \to\ \text{blocking shelf}\ \to\ \text{assigned/carrier robot}\ \to\ \text{blocking robot},
$$

前两个箭头经 requests/$\rho$ 传递(多 timestep 意图),最后一个箭头由
PIBT 同步解决(单 timestep)。

**`N.order` 的定义(v2.1 补,此前未定义)。** 每个 node 上按静态类序 +
动态提升计算:

1. 静态类序:loaded-target carrier > loaded-ANON(clear 执行中)>
   free 且被 $\rho$ 指派(chain-head 的 clear/serve 靠前)> free idle;
2. 同类内 tie-break:request priority 高者先 → 剩余距离小者先 → robot id;
3. 动态提升:沿 parent chain 继承每个 robot 的 no-progress 计数(连续未使
   其 guidance 目标距离下降的步数),计数高者在同类内前移——对应 PIBT 的
   优先级递增机制;
4. §5.5 的 livelock shuffle 只在同类内部扰动,静态类序不破坏
   (让 carrier 给 idle 让位没有意义)。

### 5.4a Target-as-blocker parking(v2.2 正式化;v2.3 纯度声明修正)

**动机**:target b 的 goal 可能位于另一未完成 target o 的活跃通道上
(走廊场景):若把 b 直接落位,o 将被 S1 永久阻断。

**定义**:

- 触发:$g_b$ 落在**另一**未完成 target $o$ 的当前 least-blocking path 上
  (owner=o);此时 b 的载具按 clear 语义走 parking 而非落位;
- 掩蔽规则:计算各 path 所用的 occupancy 视图中,**被 carry 且悬停于
  自身 goal 的 target 的 goal 格视为空**——恰好消除 park/deliver 的
  循环依赖(悬停期 owner path 不再因 b 的占位而绕行,park 判定稳定);
- 释放:owner o 在当前 X 中完成即释放(path 消失);
- 环打破:park 关系可能成环(互为 owner);沿 owner 链检出环后,
  **最小下标成员解除 park**(确定性);
- 性质(v2.3 修正,第二轮审计):**ordering-only,不改可行域;不依赖
  parent 链或全局 registry**——实证教训:solve 级 registry 会**跨 DFS
  分支泄漏** park 决策,导致大规模实例全局过度 park(r2r/dne/s2w
  平台期的总根因)。但严格地说 park 是 **(X, $D_b$ 缓存纪元)** 的
  确定性函数而非纯 X 函数:least-blocking path 走 §6.2 的非对称惰性
  失效缓存(腾空不失效),同一 X 在不同缓存历史下可能取到不同的
  stale-but-valid path,进而得到不同 owner/park 集。这**只影响
  ordering 的可复现性,不影响可行域与 completeness**;严格纯化
  (park 判定用当前 X 重算的 path)的成本与收益待验证,连同
  same-X/双缓存历史不变性测试列入 debug.md。

回归锚:`tests/test_dd_regression.cpp::dd_sticky_park.*`(走廊触发、
互锁环、goal 占用不抖动;断言为因果式而非完成时序式)。

### 5.5 Livelock 与 cross-deck deadlock 处理(核心修改 3)

**时间尺度错配是本域特有问题**:PIBT 的 priority inheritance 在同一 timestep
内闭环(被继承者总能立即让位);而 "shelf 等 blocker 清走" 需要 robot 先开
过去,是多步意图。因此会出现 PIBT/LaCAM2-swap 都覆盖不了的新死锁模式:

- r1 载着 target 原地等 blocker 清走,而负责清 blocker 的 r2 被 r1 挡住;
- idle robot 停在 blocker 正下方,任何人都无法 lift 它;
- 两条 blocker chain 互为对方的放置区。

处理(v2.2:实证后的落地形态):

- **信号 A(h 平台)**:连续 $W{=}24$ 步 guidance-h 无下降 → (a) PIBT
  priority `order` 类内 shuffle(**约束树的 `constraint_order` 冻结不动**,
  否则破坏 §4.2 的枚举完备前提——审计修正 P0-1);(b) $\rho$ 禁忌
  re-match。
- **信号 B(重访计数)**:duplicate successor 命中 EXPLORED 节点每 8 次 →
  对该节点 re-guidance(禁忌其当前配对)+ 允许重新尝试 macro rollout
  (rng 已前进,合法多样化)。
- **载具对头 yield**:两 carried target 的 next 互为对方当前格(R2 禁 swap,
  PIBT 无法跨步化解)→ 余距更大者按 park 语义让路(确定性 tie-break)。
- **结构驱动 wait-for graph** 保留为后续工作;实证中上述信号组合 +
  §5.4a + macro rollout 已使 dev 全集(含 DnE-M/S2W-M 400-target)10s 内
  可解,wait-for 的增量收益待评估。

以上全部只是 ordering 扰动,completeness 由 high-level search 兜底。

### 5.6 Sound pruning 与 placement score

**Dead-cell 分析(v2.2 修正)。** v1 中双层共用同一 wall 集,故 "从
$g_b$ 反向(仅 wall)不可达" ⟺ start 与 goal 处于不同连通分量 ⟺ 实例
本身不可行——正确形态是 **load 期可行性拒载**(`finalize` 抛错),
而非搜索期 drop-pruning。Sokoban 式 drop-pruning 只有在两层 wall 集
不同(如 upper 层有额外禁区)时才有非平凡剪枝量,推迟到该扩展。

**Placement score(v2.2:被实证否决的假设)。** 原假设("远离活跃
path、不堵单格宽走廊、靠近 dead-end")在 d50(16×16, 50% 填充)A/B 中
**使已解实例回归超时**:高密度下走廊/边角是仅存的机动空间,回避它们
等于自断退路。现行实现为 "最近的、避开活跃 path 与 goal 的空格",
必要时回退到任意非 goal 空格。新的 score 假设(局部空格连通度、
articulation、逃逸度)标注为未验证,留待 §8.4 placement ablation。

### 5.7 Cost 与 heuristic

**$g$:真实 physical cost,不含任何 matching/guidance 项**(D5):

$$
g(N')=g(N)+c(X,\mathbf a,X'),\qquad c\ \text{如 §2.3}.
$$

**admissible-h 核**(用于 $f$ 剪枝与 FOCAL 的 $f_{\min}$):

- SOC 型:$\displaystyle h_{\mathrm{soc}}=\sum_{b\ \text{未完成}}\Bigl[\alpha\, d^{\mathrm{wall}}_{\mathrm{upper}}(p_b,g_b)+\gamma\cdot\bigl(2\cdot\mathbf 1[b\ \text{无 carrier}]+\mathbf 1[b\ \text{有 carrier}]\bigr)\Bigr]$
  ——每个 loaded move 至多让一个 shelf 的 $d_{\mathrm{upper}}$ 减 1;
  无 carrier 的未完成 shelf 尚需 ≥1 lift + ≥1 drop,有 carrier 的尚需
  ≥1 drop。admissible;
- makespan 型(v2.1 修正 off-by-one):

$$
h_{\mathrm{mk}}=\max_{b\ \text{未完成}}\Bigl[\mathbf 1[b\ \text{无 carrier}]\cdot\Bigl(\min_r\bigl(d_{\mathrm{lower}}(q_r,p_b)+\mathbf 1[r\ \text{loaded}]\bigr)+1\Bigr)+d^{\mathrm{wall}}_{\mathrm{upper}}(p_b,g_b)+1\Bigr]
$$

  ——free travel 与 carry 可并行,用 max 不用 sum。**lift 的 +1 必须与
  approach 项一起条件化于 "b 无 carrier"**:已被 carry 的 shelf 剩余代价
  只有 loaded moves + 1 次 drop,无条件 +1 会高估 1 步而 inadmissible,
  并在 $f$ 剪枝 / FOCAL 的 $f_{\min}$ 中实际剪掉最优解。

**guidance-h(inadmissible,只用于 ordering / FOCAL tie-break)**:
(v2.3 按实现如实修订)当前实现 = $\sum_{b\ \text{未完成}}$
($D_b$ 路径长 + 2)——只含 target 侧信号。原 v2 规定 "robot→request
matching 成本必须进 guidance-h" 以消除 free-robot 空驶期的 h 平台;
实证中该 approach 项为 $O(|B|\times|R|)$ 全量重算,在 400-target 实例
上是吞吐瓶颈,被移除后 plateau 改由 §5.5 的 livelock 信号(h 平台
计数 + 重访计数 + re-guidance)兜住,dev 全集实证可解。**教训取代
MUST**:matching 项对小实例有益、对大实例是负担;增量式 approach
(只重算 moved robot/target 的行)列为待验证改进(debug.md)。
FOCAL 基础设施:首解前 DFS(back),首解后 FOCAL-lite(top-32 按 f 选点);
tie-break metrics(`loaded_move_ratio`、`shelf_switches`、
`futile_lift_drop`)已进 rows.csv 作为报表指标,未进选点。

**权重范围说明(v2.3)**:αβγδ 是**评价/报表**参数(benchmark 的
`DD_ALPHA..DD_DELTA`,C++/Python 双侧一致,见跨语言 cost 测试);
solver 内部的 $g$/admissible-h/rollout 固定单位权重(α=β=γ=δ=1)。
非单位权重下的 solver 目标函数(权重贯穿 g/h)列为扩展,须补
非单位权重的跨语言与剪枝一致性测试。

---

## 6. 状态表示与工程

### 6.1 Canonical form 与 hash

- `EXPLORED` key = canonical($X$):labeled targets map + 匿名 occupancy
  bitset + labeled robots(v1 不匿名化 robot,见 D4)+ $\kappa$
  (χ 是 derived,不进 key,D12);
- **Zobrist 增量 hash(v2.2 已落地)**:无表实现——key 由 splitmix64 对
  (role, id, cell) 即时派生,全量 hash = 各分量 XOR;增量版本按 joint op
  的 effect 组合 XOR(性质测试对照全量重算)。已接入 rollout 热路径。

### 6.2 距离场缓存

| 场 | 性质 | 策略 |
|---|---|---|
| $d^{\mathrm{wall}}_{\mathrm{upper}}(\cdot,g_b)$ | 静态(只依赖 wall) | 复用 `DistTable` 的 lazy BFS,per goal 一张 |
| $d_{\mathrm{lower}}(\cdot,\text{cell})$ | 静态(free robot 只受 wall) | per-cell lazy BFS + LRU cache(request cell 数量少) |
| $D_b$(blocking-aware) | 依赖当前 occupancy | **惰性失效(v2.2 已按本行语义落地)**:cache 整条 least-blocking path + 逐格占用快照;查询时仅当某 path 格 **由空变占** 才重算(非对称),腾空不失效(stale 保守无害);另有 head-advance(载具沿 path 前进一格时 O(1) 裁剪)。路径搜索用 **Dijkstra**(v2.3 修正,原误写 A*;加权格上 Manhattan 需除以 $1+\lambda_{\mathrm{blk}}$ 才 admissible,收益有限,未实现)。注意:非对称失效意味着 guidance 是 (X, 缓存纪元) 的函数,见 §5.4a 纯度讨论;`DD_STRICT_INVAL=1` 可切换为任意变化即失效(慢但纪元无关,ablation 对照 9/9 无成功率差异) |

### 6.3 复杂度预算(每次 expansion)

requests 构建 $O(k\cdot|B_{\mathrm{tgt}}|)$;$\rho$ matching
$O(n_{\mathrm{free}}\cdot|\mathcal M|)$(greedy)或 Hungarian 增量;
Carrier-PIBT $O(|R|\cdot\deg)$;距离场按 cache 命中摊销。
与 ITA-LaCAM 的 per-node Hungarian + PIBT 同量级。

### 6.4 Validator-first(开发顺序约束)

**先写 two-deck transition validator,再写 planner。**
(v2.2 表述修正)仓库实际架构为**双实现互查**:C++ `apply_ops` 是
planner 运行时的 authoritative arbiter;Python `ddbench.validator` 是
独立 conformance oracle,每个输出 plan 都会被其重放复核。两者由共享
语义测试(§6.5 + G1 穷举对照)钉住,防漂移。validator 职责:

1. 单测与 CI 的裁决(扩展现有 `tools/validate_tapf_solution.py` 到双层);
2. Carrier-PIBT 的最终裁决(generator 只 propose,validator accept)——
   防止 generator 与 validator 语义漂移;
3. completeness 证明里 G1 要求的 deterministic fallback。

(v2.3 如实)防漂移现状:C++ 侧 G1 穷举对照(generator vs C++ validator)
与 Python 侧 plan 重放各自成立,但**跨语言共享 golden transition corpus
尚未建立**(两个 validator 之间靠"整 plan 重放全过"间接互查),列入
debug.md。

### 6.5 极限单测(语义正确性的锚)

- $|R|=1$:解应等价于某个 sequential pebble plan(定理 1 的构造);
- 全占据 cycle + $|R|=$ cycle 长度:应能解出整体旋转(命题 2);
- 8×8 手工实例:单 blocker、双 blocker chain、idle robot 挡 lift 位。

### 6.6 环境旋钮配置表(v2.3 新增;默认值即基准配置)

| 变量 | 默认 | 作用域 | 语义 |
|---|---|---|---|
| `DD_MACRO_TGT` | 64 | 质量/成功率 | macro 规模域上限:$|B_{\mathrm{tgt}}|>$ 该值时整相禁 macro(D14) |
| `DD_MACRO_CAP` | 64 | 质量 | 单次 rollout 步数上限;0=全局禁 macro |
| `DD_MACRO_MIN` | 15 | 质量 | 大实例 rollout 最小 chunk(小实例首事件即停) |
| `DD_MACRO_SPARSE` | 1 | 质量 | 每隔多少深度尝试一次 macro |
| `DD_GUIDE_EVERY` | 1 | 吞吐 | guidance 重建的节点间隔 |
| `DD_ACTIVE_CAP` | 64(仅 $>256$ target 实例启用) | 吞吐 | 活跃 target 集合上限(距完成最近者优先) |
| `DD_STRICT_INVAL` | 0 | 对照 | 1=$D_b$ 缓存任意变化即失效(纪元无关、慢) |
| `DD_NO_YIELD` | 0 | 对照 | 1=关闭载具对头 yield 规则 |
| `DD_ALPHA..DD_DELTA` | 1 | 仅报表 | weighted SOC 权重(不进 solver 目标,见 §5.7) |
| `DD_DEBUG_DUMP` | 0 | 调试 | 失败时打印最深配置与 best-effort plan |

历史注记:`DD_NO_ASTAR` 从未被生产代码读取(路径搜索本就是 Dijkstra,
无 A* 开关),曾出现在 ablation 脚本中的该变体是 no-op,对应结果列
作废(v2.3 审计;脚本已修正,见 debug.md)。

---

## 7. 扩展(均不进 v1 critical path)

### 7.1 Macro successor = event-bounded rollout(修 depth,completeness 免费)

大仓库里大量 timestep 是 free robot 空驶,primitive 搜索深度 ≈ makespan。
v2 曾定义以单 robot 为中心的 macro(`approach-and-lift` 等),但那留下经典
未答问题:macro 的 k 个 timestep 里**其他 robot 在干什么**。v2.1 改为:

> **macro successor := 从当前 node 出发,用 Carrier-PIBT(无约束、无分支)
> 向前滚动,直到发生"事件"(任一 robot lift/drop、某 request 完成)或达到
> 步数上限 $k$;把终点 configuration 作为额外的、优先尝试的 successor
> 插入 OPEN。**

性质全部自动成立:

- 所有 robot 在 rollout 每一步的动作天然有定义(就是 generator 本身);
- 内部轨迹天然合法(每步都过 validator),trace 即 rollout 记录,
  中间态不进 `EXPLORED`;
- **不替换** primitive 展开,constraint tree 照常保留 ⟹ completeness
  平凡保持,不需要 "macro realization 可枚举" 这类强条件;
- **与 B0 共享全部代码**:B0(§8.1)就是"只 rollout、不 search"的退化
  情形,实现增量几乎为零;
- $k$ 与事件集是多样性旋钮(不同 $k$ 产生不同粒度的 jump successor);
  rollout 的确定性由 D11(guidance 冻结)+ 固定 seed 保证。

**(v2.3, D14)macro 是可行性装置,不是质量装置——两阶段 anytime 与
规模域规则。** 全量 162 实例 A/B(2026-08-30)实证:macro 的贪心
trace 被固化进最终 plan,在数百 target 的论文级实例上使 makespan/SOC
膨胀 2.5–3 倍;而在高密度中型实例(如 16×16/50% 填充)上 macro 是
凿穿死锁的关键。落地规则:

1. **规模域**:仅 $|B_{\mathrm{tgt}}|\le$ `DD_MACRO_TGT`(默认 64,与
   `DD_ACTIVE_CAP`/min-chunk 同一边界)时启用 macro;
2. **相位门控**:macro 只在 incumbent 出现前注入(feasibility 阶段);
   首解后禁 macro(`test_dd_anytime::macro_disabled_after_first_solution`,
   `stats.macro_after_first == 0`);
3. **两阶段重启**:phase 1 带 macro 拿首解即停(快速上界);phase 2 以
   剩余预算从根重启 primitive-only 搜索,以 phase-1 上界做 $f$ 剪枝;
   返回两相更优者。completeness 关系见 §4.2。

代价(如实):同实例对比 R2R makespan 1673→614、DnE 3934→1296、
S2W 3288→1292(2.5–3.0×),small/std/sweep 完全不变;总成功率
160/162→158/162(DnE seed18/22 需 macro 域才可行,`DD_MACRO_TGT`
调大可恢复)。先尝试过的"仅相位门控、不重启"无效:首解已被 macro
trace 填充,改进阶段在剩余预算内翻不了盘——重启是必要的。

### 7.2 Anytime:carrier-aware LNS

复用现有 repair 实验基础设施:首解后,选 executed-makespan 关键路径上的
robot/shelf 子集 + 时间窗,固定其余,局部重搜。很可能是论文第二卖点。

### 7.3 ITA 层:动态 $\tau$

goal 候选集 + incremental Hungarian(静态 $d^{\mathrm{wall}}_{\mathrm{upper}}$
cost,moved-row 增量)。放 phase 3 或独立扩展章节,保持 v1 故事紧凑:
"carrier-aware" 是主线,"assignment-aware" 是已被 ITA-LaCAM 建立的旧线。

---

## 8. 基线与实验

### 8.1 基线

| 名称 | 说明 | 作用 |
|---|---|---|
| **B0: Carrier-PIBT standalone** | §5.4 的 generator 去掉 LaCAM 外壳,滚动执行 | **最重要的 ablation**:外层 lazy search 买到了什么(PIBT 之于 LaCAM 的对照);反正 generator 必须写,成本≈0 |
| B1: 2-stage | BR-LaCAM 风格 block plan 固定后,**复用 B0 的 Carrier-PIBT 在"shelf plan 为硬约束"模式下执行**(requests 改由固定 plan 的下一步生成) | 对照 "shelf-first decomposition";与 full method 的差异被隔离为唯一变量"shelf intent 固定 vs 逐 configuration 重算",ablation 更干净且省一份实现 |
| B2: MAPF-DECOMP / CREST | CREST 有公开代码(github.com/ChristinaTan0704/CREST) | 外部 SOTA 执行框架 |
| B3: NAT-CBS / MARPF | 小实例 | optimality gap |
| B4: 单 robot 顺序模拟 | (v2.2 拆分表述)定理 1 本身只断言"给定 sequential pebble plan 可被单 robot 执行";当前实现是 **B4-greedy**:自行用贪心清障构造 plan 并执行,可失败(无空格/递归不收敛时诚实报错),其 success rate 是观测指标而非保证 | 可行性 sanity + 吞吐下界 |

**方法名映射(v2.3;`benchmark/run_benchmark.py` 实际方法集)**:
full method=`carrier`;B0=`carrier_b0`;B1=`carrier_b1`;
B2=`crest_base`(论文两阶段)与 `crest_full`(含执行期修补);
B3=`natcbs`(≤150 格小实例);B4=`b4`。官方全量结果:
`results_final_v4/rows.csv`(164 实例 × 7 方法,统一 10s,jobs=16
物理核;carrier 162/164,r2r 质量与 crest_base 打平)。历史:v2(旧
默认)、v3(两阶段)保留作演进对照。

### 8.2 自变量 sweep(优势应最大的两条轴放前面)

1. **robot:shelf 比例** ∈ {1:2, 1:5, 1:10, 1:20, 1:50}
   ——(v2.3 如实)`generate_sweep_instances.py` 当前生成 1:2/1:5/1:10/
   1:20,**1:50 档未生成**(20×20 下 shelf 数超出可放置空间,需更大
   地图,列入 debug.md);
2. **lift/drop overhead**(cost 权重 γ,及 phase 2 的多步 overhead)
   ∈ {0, 1, 2, 5}——CREST 已证明 overhead 越大 decomposition 越吃亏;
   注意 DD-MAPD 原文假设 lift/place **零耗时**,对齐比较必须含 0 档;
   (v2.3)γ 轴以 rows.csv 计数器后处理加权实现,无需重跑;
3. shelf 填充率 ∈ {50%…95%}(逼近 puzzle 密度);
4. **$|B_{\mathrm{tgt}}|$** ∈ {1, 4, 16, 64}(v2.1 补):单 target 是纯
   clear-path regime,多 target 才是 cross-layer priority 与 $\rho$
   matching 发力的 regime,必须分开报告;
5. **scramble depth**(见下);
6. maps:MAPF benchmark warehouse 系列 + 仓库 fixtures(`tests/assets/`);
   另复刻 DD-MAPD 的实例协议(2×2 block 采样至指定密度、$0.1n^2$ 个
   relocation targets、perimeter agent starts)以便与其报告数字直接可比。

**实例生成用 scrambler,不用"随机摆放 + 可行性检查"**(v2.1):
从 goal configuration 出发,用 validator 做 $k$ 步随机合法操作得到 start。
可行性 by construction:每个 primitive action 有逆动作
(`Move(v)`↔`Move(u)`、`Lift`↔`Drop`、`Wait`↔`Wait`),且 §3.3 的合法性
规则对时间反演逐条对称,故 scramble 轨迹之逆本身就是一个合法 solution
(validator 双向校验兜底)。$k$ 是直接的难度旋钮,高 $k$ 天然产生
"必须多次让路"的 dense 实例——比填充率更能刻画难度。

### 8.3 指标

success rate、first-solution time、**executed makespan**、weighted SOC、
loaded/free travel、lift+drop 次数、shelf switching 次数、
**robot utilization**(loaded-move timestep 占比)、anytime improvement 曲线。

### 8.4 Ablations

(v2.3)**已实现**(`run_ablations.py`,9 dev cases):

- B0 vs full(search 的贡献)——已跑:b0 8/9,common-case 质量差 ~44%;
- B1 vs full(冻结 plan vs 逐配置重算)——已跑:b1 5/9;
- `strict_inv`($D_b$ 严格失效 vs 惰性)——已跑:9/9 无成功率差;
- `no_yield`(关载具对头 yield)——已跑:9/9(该 dev 集不触发);
- `no_macro`——已跑并升级为全量 162 A/B,结论固化为 D14 规模域规则;
- ~~`no_astar`~~——**作废(v2.3 审计)**:`DD_NO_ASTAR` 从未被生产代码
  读取,该变体是 no-op,结果列不采信;已从脚本移除。

**计划中(未实现,如实标注)**:

- deadlock 处理 off/信号版/wait-for 版(wait-for 未实现);
- dead-cell pruning on/off(v1 形态为 load 期拒载,无搜索期开关);
- blocking-aware field vs 1-step lookahead;
- $\rho$ matching(Hungarian)vs greedy nearest(Hungarian 未实现);
- **no-following 语义对齐**:给我们的模型加回 no-following,与 BR-LaCAM 公平
  对比,同时展示去掉它的收益(§3.4a);
- fixed $\rho$(首次配对锁死)vs per-node re-match;
- placement score 变体(§5.6 新假设)。

---

## 9. 相关工作与定位

**一句话贡献(v2.2 按实证收窄):**

> A LaCAM-style search over **executable robot–shelf physical
> configurations** with operator-level lazy constraints,
> per-configuration guidance, and cross-deck conflict handling —
> feasibility-first with anytime cost improvement;每条搜索边过
> deterministic validator。completeness 论证依赖 §4.2 的骨架
> (G1 直通 + 冻结 constraint order 已按此实现并有穷举对照测试),
> 最优性只在 admissible-h 剪枝意义下 anytime 逼近,不作 LaCAM* 级主张。

| 工作 | 与本文关系 |
|---|---|
| BR-LaCAM / BRaP ([2509.01022](https://arxiv.org/abs/2509.01022)) | block-only 抽象;我们加入 robot/carrier 层,且证明其 no-following 保守性不必要 |
| DD-MAPD ([2304.14309](https://arxiv.org/abs/2304.14309)) | 问题语义来源;其 MAPF-DECOMP 是 shelf-first 分解,我们消除分解 |
| CREST ([2603.28803](https://arxiv.org/abs/2603.28803)) | 保留分解、执行期修补;我们从根上让每条边可执行 |
| NAT-CBS (SOCS'25) | coupled 但 "先规划 obstacle、再 flow 验证 realization";我们生成时即可执行;它 optimal 但慢 4 个数量级 |
| MARPF ([2403.12376](https://arxiv.org/abs/2403.12376)) | fully coupled ILP,小规模 |
| **M-PAMO ([2509.26050](https://arxiv.org/abs/2509.26050))** | agent 为**自己的** goal 清障;我们是 object 本身有 rearrangement goal。定位不同但 reviewer 必问,必须引 |
| ITA-LaCAM(本组) | 方法论母体:configuration search + per-node matching;本文把它从 assignment-aware 推广到 carrier/manipulation-aware |
| Pebble motion (Kornhauser)、puzzle-based storage (Gue & Kim)、Sokoban 技术 | 定理 1/命题 2 的锚点;dead-cell pruning 与 escort(empty cell)视角的出处 |

**不声称** "首个 joint robot-object planning";声称的是:首个在**每条搜索边
上保证联合可执行**、同时保持 LaCAM 级 scalability 的方法(completeness
为 §4.2 意义下的无时限 phase 级主张,optimality 不主张,见 §4.3),
外加两点理论说明:对 1-robust / no-following 类分解(含 MAPF-DECOMP(PP)、
BRaP)的**可行域严格分离**(命题 2,精确范围见 §4.1),以及统一模型无需
no-following 保守约束(§3.4a)。

---

## 10. 里程碑

| 阶段 | 内容 | 出口判据 |
|---|---|---|
| **M0**(~1 周) | 语义 spec 冻结;YAML 实例格式(walls/robots/shelves/targets/goals);two-deck validator;**scrambler 实例生成器**;可视化(`visualize_tapf_schedule.py` 扩双层) | validator 通过 §6.5 手工实例;scrambler 实例的逆轨迹通过 validator;可视化能回放手造 schedule |
| **M1**(~2 周) | Primitive Carrier-LaCAM:action-constraint tree + Carrier-PIBT(greedy ρ)+ B0 + livelock 信号版 | 1-robot 与 cycle-rotation 单测过;20×20/50 shelves/10 robots 秒级首解 |
| **M2**(~2 周) | min-cost ρ + blocking-aware fields + dead-cell pruning + FOCAL/anytime + Zobrist | B0 vs full 的初步曲线;高密度实例不 livelock |
| **M3** | 基线接入(B1–B4)+ sweep + 论文图 | §8 全套结果 |
| **M4**(可选) | macro successors、LNS、wait-for deadlock、ITA τ 层 | 论文扩展章节 / paper 2 |

(v2.3 状态)M0 完成(双层可视化已落地:`visualize_dd_schedule.py`
ascii/png + `generate_web_viz.py` 交互网页);M1 完成(出口判据以
20×20/52 shelves/10 robots 通过,2×2 协议下 shelf 数为 4 的倍数,取
≥50 最近值);M2 的 min-cost ρ(Hungarian)未做(greedy + 禁忌
re-match 实证够用,Hungarian 移入扩展),其余(blocking-aware field、
dead-cell v1 形态、FOCAL/anytime、Zobrist)完成;M3 的 B0-B4 完成,
sweep/ablation 覆盖范围见 §8.2/§8.4 如实标注;M4 中 macro rollout 已
提前落地并固化为 D14 两阶段规则,LNS/wait-for/ITA-τ 未做。
最终数字(统一 10s,164 实例,round-2 完成后):carrier 162/164
(r2r 25/25 mk 548 ≈ crest_base 546、dne 24/25、s2w 25/25),对比
carrier_b0 147、b4 115、crest_base 81、carrier_b1 77、crest_full 38、
natcbs 21。round-2 新增默认:η 迟滞、路径惯性、idle 避让、
g-relax/rewire、wait-for 定向禁忌、Hungarian ρ(≤256 targets 规模域,
DD_RHO_HUNGARIAN_TGT)。

代码落点(v2.4 按 2026-08-30 骨架审计修正;原文声称 "fork
tapf_planner.cpp、DistTable 直接复用" 与实现不符):dd_planner.cpp 是
**以 LaCAM 思路为参照的独立实现**——Constraint→lazy tree→PIBT 补全→
DFS/EXPLORED 的架构形状可追溯到原骨架,但类型体系(DDGrid/PhysConfig/
DistCache)自成一套,原 Graph/Vertex/DistTable/TAPF* 符号引用为 0。
现按"逐组件回迁"路线向原 lacam-tapf 骨架对齐(见 debug.md 骨架回迁
清单):

| 组件 | 状态 | 说明 |
|---|---|---|
| Hungarian | **已回迁** | DD 复制版删除,统一调用 tapf_assignment 的 `tapf_hungarian_row_to_col`(从原匿名类提出;算法与扫描序逐字节一致,benchmark 确定性不变) |
| 距离场(DistCache/LowerDist vs DistTable) | 待回迁 | 抽共享 lazy-BFS core,DDGrid/Graph 各挂 adapter;Manhattan fast path 保留为策略层 |
| 网格拓扑(DDGrid vs Graph) | 待对齐 | 非侵入 topology 接口;注意邻接序(down/up/right/left)决定确定性 |
| OPEN 选点(FOCAL-lite vs TAPF FOCAL) | **已对齐** | 默认=原版 select_open_index 语义(全 OPEN f_min、bound=1.5·f_min、h 平局;DD_FOCAL_W=0 回退 legacy top-32);门禁 162/164、r2r mk 548 精确达标,dev 中性 |
| Node/Constraint 骨架 | 部分同形 | driver 可模板化共享;payload(Op vs Vertex)本质不同,保留 |
| **正当保留的差异** | — | operator constraint、PhysConfig/canonical hash、apply_ops 裁决、constraint_order 冻结、least-blocking PathCache、serve/clear/park/yield/wait-for、macro rollout 与 parent_edge、双层 YAML schema——均为双层语义特有,原骨架无对应物 |

macro rollout 与 B0 共码,可视 M2 进度提前。

---

## 11. Decision Log

| # | 决定 | 理由 |
|---|---|---|
| D1 | **去掉 no-following**(§3.4a) | 抽象模型的保守性 artifact;统一模型物理可行;对比实验加语义开关 |
| D2 | complete 的 target 默认留在原地占格,**且 completed 状态可逆**(可再次被 lift 让路) | 更物理;dense 场景必然需要;`remove_on_complete` flag 备用 |
| D3 | lift/drop 各 1 timestep;overhead 走 cost 权重 | 多步 overhead 需把剩余操作时间放进 state(Markov 性),推迟到 phase 2 |
| D4 | v1 robots labeled,不匿名化 | free-robot multiset 化影响 path reconstruction,收益后置。注:labeled robots + 无参数 `Lift`/`Drop` 下,单 robot 合法 operator ↦ $(q',\kappa')$ 是单射,故 joint 层不会产生 canonical 重复分支;一旦匿名化 robot,须在 `legal_local_ops`/joint 层按 canonical 等价去重(quotient 上 completeness 保持) |
| D5 | $g$ = 真实 physical cost,matching 只进 guidance | 修正 ITA-LaCAM 中 assignment-dependent g 的理论瑕疵 |
| D6 | `EXPLORED` key 只含 canonical($X$) | 同一物理状态不因 guidance 不同重复展开 |
| D7 | 算法名 **Carrier-LaCAM** | 比 DD-LaCAM 更能表达 "actuator 与 object 分离" 的核心 |
| D8 | v1 无 orientation、4-连通、同步 timestep | 与全部 baseline 对齐;orientation 属产品化 |
| D9 | carried 匿名 shelf 用 $\textsf{ANON}$ 标记,不带 id | 搬哪个 blocker 等价;进一步收缩状态空间 |
| D10 | goal condition 要求 target grounded | 否则 "carry 着路过 goal" 会被误判完成 |
| D11 | (v2.3 重写)**冻结的是约束树枚举序 `constraint_order`**(node 创建时定死,livelock 扰动不得触碰,§4.2 完备性前提);guidance(ρ 配对、PIBT `order`、$D_b$)允许在 livelock 信号下重建(每 8 次重访 re-guidance)。原 "guidance 在 node 创建时冻结" 的表述与实现不符,且 ρ hysteresis 未实现(§5.3(4)) | 完备性只依赖枚举序不变;guidance 重建是合法多样化(ordering-only) |
| D12 | χ 为 derived view,不进 state/key;`Drop` 不置位 | 见 §3.1;单调存储的 χ 在 dense 场景是正确性 bug |
| D13 | macro successor := event-bounded Carrier-PIBT rollout | 其他 robot 的动作天然有定义;与 B0 共码;completeness 免费(§7.1) |
| D14 | (v2.3)**两阶段 anytime + macro 规模域**:macro 仅在 $|B_{\mathrm{tgt}}|\le$ `DD_MACRO_TGT`(64) 且首解前启用;phase 1 首解即停,phase 2 从根 primitive-only 重启并以 phase-1 上界剪枝,取两相更优 | 全量 A/B:论文级质量 2.5–3×,small/std/sweep 不变;代价 -2 成功率(旋钮可恢复);"仅门控不重启"被实证否决(§7.1) |

---

## 12. 开放问题

1. wait-for graph 的环检测粒度:每 node 做太贵,livelock 信号触发是否足够?
   (v2.3 注:dev 全集已可解,wait-for 的定位从"修 bug"变为"提质量");
2. 高密度下 requests 的 chain 截断 $k$ 与 solution 质量的关系;
3. lifelong / throughput 版本(接现有 ore workflow)——one-shot 之后的方向;
4. free-robot 匿名化后的 path reconstruction 与 LaCAM* rewiring 的相容性;
5. (v2.3)**往返震荡抑制**:可视化观察到 robot 反复调头,归因于
   ρ 无迟滞(η 未实现)、$D_b$ 平局翻转、idle 避让缺失三者叠加;
   候选手段:η 迟滞、路径惯性(前路径格微折扣)、idle 主动离开活跃
   path;须先落 `reversals` 指标再 A/B(见 debug.md);
6. (v2.3)duplicate 的 g-relax/rewire(恢复 eventually-optimal 主张
   的前提,§4.3);park 判定的严格纯化(§5.4a)。

(v2 的开放问题 "$D_b$ 增量维护" 已由 §6.2 的惰性失效方案关闭。)
