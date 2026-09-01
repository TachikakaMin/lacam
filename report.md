# Carrier-LaCAM 小地图病态行为研究报告（≤10x10）

> 日期：2026-08-31，最终更新：2026-09-01。写给学过基本图搜索
> （BFS/DFS）的读者，每个术语第一次出现都先解释，每个结论都注明
> "我怎么知道的"。公式全部使用终端可读的纯文本。
>
> 研究对象从 `results_gateC_frozen` 的旧计划开始，最终以
> `benchmark/results_task_commit_final/` 和当前 `lacam/src`
> 为准。
>
> 阅读提示：第 0-10 部分保留发现过程和被证伪路线。当前生产结论、最终
> 实现和 benchmark 见第 11 部分；旧开关不再存在。

---

## 第 0 部分：这个算法本来是怎么工作的

Carrier-LaCAM 解决的问题：仓库里有机器人和货架，要把几个"目标货架"搬到指定格子。它的解法是**在"仓库状态图"里找路**：

- 一个**状态** = 整个仓库某一刻的完整快照（每个机器人在哪、每个货架在哪、谁举着什么）
- 一条**边** = 所有机器人同时执行一步动作（移动/等待/抬起/放下），仓库从快照 A 变成快照 B
- **解** = 从初始快照走到"所有目标货架都落在目标格"快照的一条路径。路径有多长，最终执行就要多少个时间步（这个步数叫 makespan）

搜索分两层：

- **高层**：维护一个"待展开状态"列表（叫 OPEN），每轮挑一个状态出来，问它"你的下一步可以变成哪些新状态？"
- **低层（PIBT）**：负责快速拼出"这一步大家各自怎么动"的一个具体组合。它按优先级排好每个机器人的候选动作顺序（这套排序建议叫 guidance），谁挡路就递归请谁让开

## 第 0.5 部分：观察到的病态现象（问题是什么）

对 34 个计划逐步重放（用仓库自带的权威 Python validator），量化结果：

```text
总步数                 1,287,202
其中可证明的纯浪费        72.3%   (见第 4/6 部分"循环")
机器人立即掉头率          17.9%   (每 5-6 步移动就有一次 A->B->A)
原地抬放 / 全部抬起       89.5%   (抬起后一步没走就放回原格)
单个货架最长连续原地抬放   2,797 次
单个货架被抬起的最多次数   4,773 次
"举着货架原地等待"总步数  537,904
目标货架被放到更远处       5,175 次
```

用户最初报告的两个现象（机器人往复运动、实际搬动前反复抬放）都被证实，且只是冰山一角。

---

## 第 1 部分：什么是 heuristic，为什么它是本案的核心

标准搜索算法（比如 A*）给每个状态打两个分：

```text
g(X) = 从起点走到状态 X 已经花了多少代价
h(X) = 从 X 走到终点估计还要多少代价   <-- 这就是 heuristic（启发函数）
f(X) = g(X) + h(X) = 这条路线的总预估
```

h 的作用是给搜索**方向感**。类比：你在陌生城市找火车站，h 就是"直线距离还剩多少"。就算估得不准，它也能让你大体朝正确方向走。

**h = 0 意味着什么？** 意味着搜索认为"任何状态离终点都一样近"。它完全失去方向感，只能靠展开顺序瞎碰，直到运气好撞进终点。

## 第 2 部分：发现一——这个算法的 h 恒等于 0

读代码发现（`lacam/src/tapf_assignment.cpp`，`assign_tapf_tasks_dynamic` 开头）：对 carrier 类型的实例，计算 h 的函数直接走一个叫 `carrier_trivial_result` 的捷径，**返回 0**。于是：

```text
f(X) = g(X) + 0 = g(X)
```

再看高层怎么挑状态（`lacam/src/tapf_planner.cpp` 的 `select_open_index`）：默认是 DFS 模式，**永远拿 OPEN 里最新放进去的那个**（弹栈顶）；f 值只在一个次要场合使用（见第 3 部分）。

组合效果：

```text
高层：闭着眼睛沿着"刚刚生成的状态"一路往下钻（DFS）
低层：PIBT 只看眼前一步哪个动作"看起来"顺
```

每一步局部都说得通，但**没有任何全局力量把搜索往"货架离目标更近"的方向拉**。观察到的横跳、抬放折腾，宏观上就是搜索在状态空间里做布朗运动，直到碰巧撞进目标状态。所以在拥挤地图（几乎没有空格的 `e1` 实例）上，一个 4x10 的小仓库能跑出 162,215 步的解，而它的代价下界只有 24 左右。

讽刺的是：代码里**已经算了一个方向感指标**，叫 `h_guidance`（`tapf_planner.cpp` attach_carrier_guidance 内）：

```text
h_guidance(X) = 对每个未完成目标货架，
                累加（当前位置到被分配目标格的上层距离 + 2）
```

`+2` 是因为至少还要一次抬、一次放。这个数字越小越接近完成。但它目前只用来**检测"是否卡住"**（连续多步不下降就触发一次排序洗牌），**从不参与决定先搜哪个状态**。方向感被算出来，然后被闲置了。

## 第 3 部分：发现二——"代价"根本不影响搜索怎么走

直觉方案："给抬放设很贵的代价，搜索就不爱抬放了。"实验证明**完全无效**，原因如下。

g 的累加规则里确实有权重（`get_edge_cost`）：

```text
一步代价 = alpha·(载货移动) + beta·(空载移动) + gamma·(抬/放) + delta·(搬匿名货架)
```

但 g 在这个算法里**只有一个用途**：找到第一个解（花费记为 U）之后，f >= U 的状态不再展开（剪枝）。注意：

1. 找到第一个解**之前**，g 谁也不影响——DFS 挑状态只看"谁最新"，不看 f
2. 找到第一个解之后，10 秒预算基本已用完

类比：出租车计价器只在**下车结账时**看一眼，司机**开车过程中从来不看**。把抬放单价调成 8 倍，司机还是那么开，只是账单数字变大。

**实验证实**：设 `DD_SOLVER_WEIGHTS=1 DD_GAMMA=8` 重跑 8 个实例，每个实例的 makespan 与不改权重时**一个数字都不差**（soc 账面变大而已）。这不是"效果小"，是**结构性为零**。

## 第 4 部分：发现三——"宏跳步"把绕圈原封不动抄进最终答案

加速机制 **macro rollout**（`carrier_rollout`）：在某个状态上让 PIBT 自动连走最多 64 步，把终点状态当作"一跳可达"的邻居（宏边）挂进搜索图。探索因此变快。

问题在**出答案时**：最终解沿搜索树回溯，遇到宏边就把内部 64 步逐步展开、原样拼进计划。宏边内部只查"这 64 步内部无重复"，**不和计划其余部分查重**。

于是最终计划出现：第 t 步的快照与第 t+70380 步的快照**一模一样**——中间 70,380 步把所有东西搬来搬去又搬回原样，纯属白干。

**实验证实**：`DD_MACRO_CAP=0`（关宏）后，最终计划里"精确重复状态"从数千次降到 **0 次**——计划内的绕圈全部来自宏边拼接。但横跳率（约 0.18）和原地抬放率（约 0.89）**不变**：那是低层 PIBT 的独立问题。两种病，两个来源。

## 第 5 部分：实验一——现有 9 个开关全试一遍（全部无效）

**消融实验**（ablation）：一次只改一个变量，其余不动，看结果差异。8 个实例 x 9 个配置，各 10 秒（结果存 `/tmp/carrier_ablation2`）：

| 配置 | 改了什么 | 结果 |
|---|---|---|
| `DD_GAMMA=8` | 抬放代价 x8 | **逐实例与不改完全相同**（证实第 3 部分） |
| `DD_GUIDE_EVERY=1` | guidance 每步重算 | **完全相同**（主搜索本就每状态重算；开关只影响宏内部） |
| `DD_ETA=8, DD_ETA_B=8` | 任务匹配更"恋旧"（抗抖动） | 有的实例好一倍、有的差一倍，无一致方向 |
| `DD_CLEAR_FRONTIER=1` | 换清障货架挑选策略 | 同上，噪声 |
| `DD_STRICT_INVAL=1` | 路径缓存失效更严格 | 同上，噪声 |
| `DD_PLACE_ESCAPE=1` | 停车位置挑"易逃出"的格子 | 基本无变化 |

**教训**：这不是调参能救的问题。这些开关都只改"候选动作排序"，而醉汉的问题不在于每步先迈左脚还是右脚。

## 第 6 部分：实验二——循环消除后处理（唯一的大赢）

想法：最终计划本质是一串快照

```text
X0 -> X1 -> X2 -> ... -> X162215
```

若 `X100` 与 `X5000` **完全相同**，则第 100~4999 步可**整段剪掉**，把 X5000 之后的动作直接接在 X100 后面。

**为何剪完必然合法？** 动作是否可执行只取决于当前快照。X5000 之后的每一步在原计划中已验证合法；剪接后这些动作面对的快照序列完全相同（都从 X100 = X5000 出发），所以每步依然合法、终点依然到达。可严格证明，非启发式。

算法（单遍）：重放计划记录每个快照哈希；再从头扫，每到一个快照直接跳到"该快照最后一次出现的位置"。

**结果**（原型 `/tmp/loop_erase_proto.py`；每个都通过官方 validator，goal=True，剪后 revisits=0）：

```text
实例                          原计划      剪完       削减     对比"关宏"
h4w10_a5_e1_R1_seed0         162215  ->  18282    -88.7%    (关宏 55199)
h6w10_a6_e1_R1_seed1         165728  ->  40860    -75.3%    (关宏 131698)
h6w10_a6_e1_B_seed0_pool      54719  ->  10271    -81.2%    (关宏 13898)
h8w10_a10_e2_R1_seed0         87611  ->  26316    -70.0%    (关宏 99050)
h10w10_a12_e3_R1_seed0        93255  ->  32159    -65.5%    (关宏 102028)
h10w10_a12_e8_R1_seed0        81686  ->  35544    -56.5%    (关宏 63374)
```

反直觉发现：**"宏开着 + 事后剪圈"全面优于"一开始就关宏"**。宏让搜索更快撞到首解（探索快），其绕圈垃圾可事后免费删。正确做法不是禁用宏，而是在输出答案前加一道剪圈工序。

泼冷水：剪完离理论下界（无拥堵、每货架走最短路）**仍差 456~973 倍**。剪圈只删"回到原点的白干"，删不掉"没回到原点的瞎绕"。止血，不是治病。

## 第 7 部分：实验三——三个"治病"原型，全部失败，但每个失败都值钱

三个 C++ 原型（`lacam/src/tapf_planner.cpp`，全部环境变量门控、默认关闭；已回归验证默认路径结果与改动前**逐位相同**，mk=162215）。

**原型 A：抬起前先看一眼（`DD_LIFT_GATE=1`）**

- 动机：89% 的抬起是"原地放回"。规定：若四邻上层全被占（抬起也无处可去），把"抬"排到"等"之后。
- 结果：原地抬放率 0.89 -> 0.72~0.82（机制生效），**但总步数多数恶化**（如 55199 -> 198762），等待占比涨到 20~28%。
- 为什么：极挤地图上，"抬起挪一格放下"这种看似无意义的折腾，实际是搜索**给上层货架洗牌、腾出通路的唯一手段**。禁了它又不给替代方案，机器人集体罚站。教训：需要"抬起后存在可达放置路线"级别的检查（更贵），一步 lookahead 不够。

**原型 B：高层朝 h_guidance 小的方向挑状态（`DD_OPEN_GREEDY=32`）**

- 动机：把闲置的方向感用起来——在最新 32 个候选里挑 h_guidance 最小的展开。
- 结果：**灾难**。8 实例中 6 个 10 秒内连解都找不到（原本全能找到）。
- 为什么：(1) h_guidance 有大片"平原"——多数单步动作不改变任何货架到目标的距离，几十个候选同分，贪心失去区分度；(2) LaCAM 每次只从一个状态"挤"出一个后继，贪心反复咬住同一个低分状态一点点挤，真正的前进路线永远轮不到展开，搜索饿死。教训：这个指标不能这么粗暴地用。

**原型 C：把 h_guidance 加进 f（`DD_H_IN_F=1`）**

- 背景：算法有两阶段。阶段一找到首解即停；阶段二用剩余时间**从头重搜**并用首解花费 U 剪枝，期望更便宜的解。实测所有实例 `incumbent_updates=1`——**阶段二从未成功改进过**。
- 动机：f 加上 h，剪枝更狠，或许阶段二能更快逼近更好的解。
- 结果：毫无变化，仍是 1。
- 为什么：阶段二的根本困难是"从零重搜一个 10 万步量级的解，只剩两三秒"，剪枝再聪明也来不及。教训：改进解的正确姿势不是重搜，而是**在已有解上局部修补**（挑一小段时间窗/一小块区域重规划，其余照抄）。

## 第 8 部分：一页总结

```text
病根：
  1. h = 0，DFS 无方向           -> 醉汉游走（横跳、瞎绕的总根源）
  2. 代价权重不进搜索排序         -> 调权重无效（已证实为零效果）
  3. 宏边拼接不查重               -> 计划 56~89% 是回到原状态的白干
  4. 拥挤地图缺"洗牌记忆"         -> 原地抬放循环、货架被抬几千次

试过且无效/有害（均有实验数据）：
  调 gamma 权重、调 guidance 频率、eta/frontier/strict/escape 开关、
  一步式 LIFT 门控、OPEN 贪心化、h 加进 f

有效（已验证）：
  宏保留 + 解提取时循环消除：步数 -56% ~ -89%，全面优于关宏，
  100% 通过官方验证器
```

**建议落地顺序**：

1. **立即**：把循环消除写进 C++ 解提取出口（`TAPFPlanner::solve` 输出 solution 前对状态序列做环切除），或先作为 benchmark 后处理。收益已证明，风险为零，改动小。
2. **短期**：阶段二从"从头重搜"改为"在剪圈后的解上做局部修补"（时间窗 LNS / plan repair）——治"anytime 从不改进"的对症药。
3. **中期**：给清障请求加"失败冷却"跨节点记忆；研究 h_guidance 的正确用法（如用于回跳重启，而非贪心选点）。
4. **不要做**：调 gamma 权重、调 guidance 频率、朴素贪心选点、一步式 LIFT 门控——本次已证伪。

---

## 第 9 部分：阶段性 C++ 方案与验证（历史记录，已被第 11 部分取代）

本节记录投影根因被确认之前的一轮实现。milestone、可选 cooldown 和
loop-erase 开关后来都从生产代码删除；保留本节是为了说明最终删减依据。

**病根 3（已默认开启）：解提取出口循环消除。** `solve()` 在返回 solution 前，
对 (Config, ShelfState) 快照序列做单遍"跳到最后一次出现"环切除（`solution`
与 `solution_shelves` 同步剪切，剪后按物理口径重算 solution_cost）。
`DD_LOOP_ERASE=0` 恢复原始行为。这是唯一默认开启的改动：剪掉的段可证明纯冗余，
且逐实例通过官方 validator（剪后 revisits=0）。

**病根 1（`DD_H_MILESTONE=1`）：两级势函数 + 耐心门控回跳。** 针对原型 B 的两个死因各给一个机制：

- 破高原：势函数分两级，`phi1` = 加权货架进度（alpha·上层距离 + gamma·(1|2)，
  即带权 h_guidance），`phi2` = 机器人接近项（各被指派机器人到请求格的 beta·下层距离）。
  多数单步动作不动 phi1，但会动 phi2——高原上有了梯度。
- 防饿死：平时保持纯 DFS；只有连续 `DD_MS_PATIENCE`（默认 512）次扩展无全局
  (phi1, phi2) 改进（确认高原）才回跳一次到全局最优 OPEN 节点（懒删除最小堆），
  之后 DFS 在其子树继续。每节点回跳预算 `DD_MS_ELITE_MAX`（默认 8）次，
  "反复咬同一节点"结构性不可能；回跳不产出改进则耐心指数翻倍（上限 64x），
  无效时自动退化回纯 DFS。选点、CLOSED、约束树、RNG 流均不动——默认路径逐位不变。

**病根 2（随 milestone 生效）：权重结构性进入搜索。** phi1/phi2 直接携带
alpha/beta/gamma，堆比较器末级 tie-break 用 g。验证：不开 milestone 时
`DD_GAMMA=8` 的输出计划与默认**逐字节相同**（复现第 3 部分"结构性为零"）；
开 milestone 后同一开关产生**不同且更优**的轨迹（mk 8001，lift_drop 3116 vs 5970）。

**病根 4（`DD_LIFT_COOLDOWN=K`）：抬放冷却记忆。** 与被证伪的一步式 LIFT_GATE
不同：只记"同一货架在同一格发生过原地抬放（抬起后一步没动就放回）"这一精确事实
（跨节点 map，主搜索与宏 rollout 内部都记录），近期（`DD_LIFT_CD_WIN`，默认 512
拍）重复 >= K 次才把**这一个** (shelf, cell) 的 LIFT 降到 WAIT 之后，且冷却会过期。
洗牌机制整体保留，只是压制已知无效的重复。

**验证（8 实例 x 10s，seed 0，逐波单核并行；全部计划过官方 validator）**：

```text
实例                        gateC冻结   仅剪圈    +milestone  +cooldown   两者全开
h4w10_a5_e1_R1_seed0          162215    18282      12267       13770       6967
h6w10_a6_e1_R1_seed1          165728    40860      36158       17463      14103
h6w10_a6_e1_B_seed0_pool      54719*     5382      14410        7494       6292
h8w10_a10_e2_R1_seed0          87611    26316      36817       15582      16620
h10w10_a12_e3_R1_seed0         93255    32159      21701       15879       9802
h10w10_a12_e8_R1_seed0         81686    35544      55156       17911      20618
h8w10_a10_e20_R1_seed0(稀疏)    3148     1454       4018        1994       2602
h6w10_a6_e15_R1_seed1(稀疏)     3272     1587       1610        2242       1799
```

- 回归：`DD_LOOP_ERASE=0` 在 7/8 实例逐位复现 gateC 冻结 makespan；
  *pool 实例是唯一多目标格实例，冻结值 54719 需 Gate C 消融环境
  `DD_TAU_FREEZE=1`（已单独复现），默认 live-tau 路径本就更优（16997）。
- 6 个拥挤病态实例上：仅剪圈已 -56%~-89%；再叠 milestone/cooldown 在 5/6
  实例又拿到 1.6~2.6 倍（h4w10: 162215 -> 6967，共 -95.7%）。
- 2 个稀疏对照实例上新机制是纯开销（本就无高原/无抬放循环），故两者默认关闭。
- 126 个单元测试全过；无 shelf 实例（tapf_benchmark 路径）结构性不受影响。

**全量 benchmark（instances_brap_pool 68 实例，gateC 同协议：carrier 方法、
10s/实例、jobs=14=16 物理核留 2；runner 内置权威 validator 重放，success=1
即验证通过；结果在 `benchmark/results_fix1_{base,ms,cd,mscd}/`）**：

```text
配置             解出(68)  小图(36)  大图(32)  R1小图makespan几何均值比(vs gateC, 同解集)
gateC 冻结          34        34        0            1.000
base(仅剪圈,默认)    35        35        0            0.332   丢0 新增1(pool seed1)
ms(milestone)       33        33        0            0.358   丢1(h8w10_e2_R1_seed1)
cd(cooldown)        36        36        0            0.212   丢0 新增2
mscd(两者)          36        36        0            0.208   丢0 新增2
```

- `cd` / `mscd` 把 <=10x10 解出率打满（36/36），新解出 gateC 从未解出的
  `h10w10_a12_e3_B_seed1_pool` 与 `h10w10_a12_e3_R1_seed1`，且无一实例退化为超时。
- 同解集 R1（单目标格，无 tau-freeze 口径差异）makespan 几何均值：mscd 是
  gateC 的 0.208 倍（约 -79%）；逐实例最大值 h4w10_e1: 162215 -> 6967（-95.7%）。
- `ms` 单开确认与小矩阵一致：拥挤图有效、稀疏图有害，且丢一个实例——
  不建议单开，建议与 cooldown 同开或不开。
- 大图（>=20x20，32 实例）四配置仍全部超时——horizon 墙不动，与第 8 部分
  判断一致：这不是选点/排序问题，需要建议 2 的解修补或分层方法。
- 每配置 wall ~54s（14 并行），全套 4 配置 <4 分钟，可随代码改动例行重跑。

该轮仍离代价下界两个数量级。随后完成的等价类计划修补见第 10-11 部分。

---

## 第 10 部分：剩余无效移动的深度验尸（2026-09-01，看动画后追查）

修复后的 mscd 计划（h4w10_a5_e1, 6,967 步）在动画里仍满屏无意义走动。
对该计划逐步分类（工具 `/tmp/autopsy.py`、`/tmp/episodes.py`，经权威
validator 重放）。先交代实例的真实构成：4x10 板 36 个可用格，上层放着
35 个货架（5 目标 + 30 匿名），**只有 1 个上层空位，2 个机器人**——
这是字面意义的"35 数码华容道"：任何货架想挪一步，唯一的空位必须先被
"倒"到它前进方向的格子上。

**逐机器人步分类（13,934 机器人步）**：

```text
空载移动        8,371  (60.1%)   其中仅 46% 在 8 步内接上自己的一次抬起
等待            3,369  (24.2%)
抬 / 放         1,810  (13.0%)
搬匿名货架        238   (1.7%)
搬目标货架        146   (1.1%)   toward 80 / away 66，净进展 ~14 步
```

phi（货架势能）在 96.8% 的时间步完全不动；最长干旱段 4,460 步——
**整个计划三分之二的时间里货架侧零进展**。

**病灶一：85% 的抬放 episode 是无效的。** 905 次 lift->drop 配对分类：

```text
搬动了货架（有效）        132  (14.6%)   其中 99 次恰好挪 1 格 = 把空位倒 1 格
悬停后原地放回            639  (70.6%)   抬起->等 k 拍->原格放下，什么都没变
立即原地放回               97  (10.7%)
搬出去又搬回原格           37   (4.1%)
```

悬停型（639 次）是主体：机器人抬起货架时空位不在旁边，S1 约束下根本
迈不出第一步，举着等几拍又放回。**第 9 部分的 cooldown 只检测"抬起后
1 步内原地放回"（祖孙三代模式），悬停型全部漏网**——这解释了为什么
cooldown 有效但不够。连带地，60% 的空载移动大半是两个机器人在无效抬起
点之间往返通勤。

**病灶二（结构性，最重要）：自由机器人的站位污染了状态空间。**
6,968 个计划状态里，货架层（货架位置+搬运关系）只出现过 **861 个不同
构型**——87.6% 的时间货架层在重复已见构型，只是空载机器人站的格子不同。
这些状态在搜索眼里全是"新状态"（CLOSED 全状态判重），在循环消除眼里
也不相等（剪不掉）。基线计划同比 87.4%，稀疏对照 77.3%——普遍规律：
**计划长度的主体是自由机器人的站位噪声，货架轨迹本身只有 ~861 步量级。**
这同时解释了搜索为什么慢（状态空间被无关自由度乘爆）和计划为什么长。

**病灶三（物理下界失真）：** e1 华容道里每挪一步目标货架都要先把唯一
空位倒位，报告附录 B 的 LB=24 忽略 S1，物理上不可达；有效工作本身就
带放大系数。但这只能解释一小部分——861 步货架轨迹 vs 6,967 步计划，
噪声占主导。

**复测旧结论：悬停不能在生成侧禁。** 既然悬停 lift 的时刻空位必不邻接，
第 7 部分被证伪的 LIFT_GATE（四邻上层全占降级 lift）应恰好拦住它们。
在新栈（剪圈+milestone+cooldown）下复测 8 实例：2 个变超时、3 个恶化
（6967->10542 等）、仅稀疏实例改善。**两次证伪一致：悬停抬放对搜索是
"承重梁"——它给上层洗牌、改变 PIBT 局面，禁掉它搜索就饿死。**
出路只能是事后剪或搜索层等价约减，不是生成侧禁止。

**修复路线（按杠杆排序）**：

1. **投影等价剪切 + 机器人路径修补（后处理，最大杠杆）**：按货架层构型
   做"跳到最后一次出现"剪切，接缝处自由机器人位置对不上，插入一段
   仅下层行走的修补路径（无货架参与，平凡可解）。数据给出的上限：
   6,967 -> ~861 + 修补开销，还有 4~6 倍空间。这是第 8 部分建议 2
   （解修补）的具体化。
2. **悬停检测进 cooldown**：给节点加每机器人"本次搬运的锚格+是否移动过"
   （O(1) 增量维护），在放下时识别"锚格原地放回且从未移动"，喂进现有
   lift_futile 记忆。压制重复悬停的 (shelf, cell)，同时保留首次尝试
   （不重蹈 LIFT_GATE 全面禁止的覆辙）。
3. **搜索层对称约减（中期）**：CLOSED 判重把自由机器人位置规范化
   （如按货架层+机器人可达域分类），让货架等价状态碰撞，从源头止住
   状态空间污染。需要论证解可重构性，是真正的研究项。

---

## 第 11 部分：最终 rootfix、开关删减与全量验证（2026-09-01）

### 11.1 根因结论

最终根因不是"机器人偶尔选错一步"，而是**状态表示对搜索正确、对输出
质量过细**：

```text
完整状态 = shelf configuration + labeled robot positions + carrying
```

当 shelves 全部落地且 shelf configuration 相同时，不同 free-robot
站位在搜索中是不同状态，但从任务进度看属于同一个等价类。`h4w10_e1`
的 6,968 个状态只有 861 个 shelf projection，证明计划主体是 robot
站位噪声。精确全状态剪圈只能删除等价类中的同一点，删不掉同一 shelf
构型下的不同 robot 点。

### 11.2 已落地算法

新增 `lacam/src/dd_plan_repair.cpp`，生产入口无条件执行：

1. 重放原计划，先删除完整物理状态环；
2. 只在两个端点 shelves 全部 grounded 且 shelf projection 相同时，
   删除中间片段；
3. 接缝处只让 robots 在 lower deck 行走，使其到达原片段终点的
   labeled robot configuration；
4. 1 robot 用最短路，2 robots 用精确 A*，更多 robots 用原轨迹的
   lower-deck 投影去环；
5. 只有 bridge 严格更短才采用；
6. 最终用 C++ `apply_ops` 从初态整体重放，任何非法、未到 goal 或
   未缩短情况都返回原计划。

正确性关键：bridge 期间 shelves 固定；bridge 结束后的完整物理状态
恰好等于原片段终点，所以原后缀可原样继续。这是状态等价修补，不是删除
"看起来无用"的启发式动作。

阶段性 milestone 被删除。它在拥挤图上偶尔改善、在稀疏图上恶化，
且没有触及 861/6968 的等价类根因。原 primitive-only 第二遍被删除：
它不利用首解结构，从根重搜且从未产生第二次 incumbent update。当前的
fixed-assignment restart 不同：它在同一次 planning 中取得动态首解后，
只对多 goal 输入固定首解终态 assignment，再用总计 10 秒 deadline 的
剩余时间从根跑一遍。不是两遍各 10 秒。当前流程是：

```text
dynamic-assignment first incumbent -> repair
    -> freeze terminal shelf-goal assignment
    -> fixed-assignment root restart with remaining time -> repair
    -> choose lower SOC, then full oracle replay
```

首遍内部也加入 task-episode commitment：普通 robot/loaded motion 沿用
parent `tau`，carried shelf 不在运输途中改 goal，只在 drop 或定向
livelock repair 时重算。settled shelf 优先占有当前 eligible cell，但只有
该锁能扩展成完整 matching 时才保留；否则必须允许重开。completed target
在路径层仍按普通 blocker 处理。将它设成“任何绕路都优先”的特殊障碍曾使
正式 gate 从 36/68 降到 35/68，因此已否决。

### 11.3 自动策略与无开关审计

生产策略不再读取环境开关。Hungarian/greedy、active target cap、macro
和 cache policy 都由输入规模决定；重复立即原地抬放由运行时记忆自动
降序，阈值 3，窗口 `max(64, 8*|V|)`。该降序不删除 Lift，因此约束树
仍可枚举它。

源码中的环境输入只剩：

```text
DD_ALPHA / DD_BETA / DD_GAMMA / DD_DELTA   数值 objective
DD_DEBUG_DUMP                              失败诊断
```

no-following 改为 `apply_ops(..., allow_following=false)` 的显式测试
oracle 参数。`PathCache(strict=true)` 也只由测试探针构造。benchmark
消融只保留结构不同的 `full/b0/b1`。

已删除的生产 tricks 包括：

```text
milestone, OPEN greedy, h_guidance-in-f, lift gate,
loop-erase toggle, tau freeze, frontier bonus, parking escape,
yield/idle toggles, strict-invalidation env, macro/guidance/eta knobs
```

### 11.4 最终 benchmark

协议：固定同一组 `instances_brap_pool` YAML 和 solver seed 0；
carrier 每例严格 10 秒内部 deadline；`jobs=14` 且运行期间无并行
高负载任务；默认 following、单位权重；runner 用独立 Python validator
重放每个成功计划。success 使用全部 68 例作分母，质量只在共同成功例
上比较。每次复跑写新目录并保存 `rows.csv`、`timing.json` 和计划。
主 runner 显式覆盖为单位权重，不继承 shell 中残留的 `DD_*`；非单位
objective 只能通过 `--weights` 作为独立轴运行和记录。

```text
结果                         solved   <=10x10   成功例总 makespan
fix1_mscd                      36        36            214,199
rootfix_final                  36        36             35,595
fixed_assignment_restart       36        36             35,325
task_commit_final              36        36             34,860
```

- 成功集保持 `36/68`，小图保持 `36/36`；
- 相对 `fix1_mscd`，36 个共同成功实例的 makespan 几何均值比降至
  `0.204960`，改善/持平/恶化为 33/1/2；
- `h4w10_a5_e1_R1_seed0`：`6,967 -> 1,053`；
- Python 精确原型曾做到 719，说明多机器人 bridge 仍有优化空间；
- `h10w10_a12_e3_R1_seed1`：2,620；
- 20x20 及以上仍为 `0/32`，rootfix 改善输出，不缩短首次找到 raw
  incumbent 的 horizon。

固定 assignment 第二遍使用同一个 10 秒总 planning deadline。18 个
B-pool 成功例全部触发且第二遍全部求解：6 个选择第二遍，12 个自动保留
第一遍。相对 rootfix control，36 个共同成功例 makespan/SOC 几何比为
`0.917520/0.927884`；B-pool 子集为 `0.841843/0.857187`。成功例总
makespan `35,595 -> 34,860`，SOC `61,049 -> 59,907`。

```text
h4w10 e10 B seed0    53 -> 41
h4w10 e10 B seed1    36 -> 34
h4w10 e1  B seed1    51 -> 34
h6w10 e1  B seed0   622 -> 519
h8w10 e20 B seed0    78 -> 77
h8w10 e20 B seed1   241 -> 168
```

task commitment 会改变动态 B-pool 的首遍轨迹，因此不再要求它与旧
rootfix 逐行一致；singleton/R1 仍逐字节不变。第二遍的 6 个选择均由
同一调用内记录的 first/second SOC 比较决定。

`results_rootfix_final` 与前一轮 `results_rootfix` 的 success、status、
makespan、SOC 逐行相同，证明最终开关清理没有引入行为回归。

独立复跑 `results_rootfix_protocol_verify` 的 `timing.json` 记录 68
tasks、10 秒、14 jobs、seed 0、单位权重和默认 following；结果仍为
36/68、小图 36/36、成功例总 makespan 35,595、总 SOC 61,049。它与
两个 rootfix 目录及第一次 verify 的四个核心字段零差异，36 个成功计划
也逐字节相同。

结构消融 `results_ablation_rootfix_verify` 同样固定这套协议并复用主
runner 的 Python validator，只保留 `full/B0/B1`。9 个 protected cases
上分别成功 9/9、9/9、5/9，共 23 个非空计划全部验证通过；共同成功集
的 full/B0 makespan 几何比为 0.638179（5/3/1 改善/持平/恶化），
full/B1 为 1.077651（1/3/1）。该小 suite 只作结构和回归检查，不替代
68 例主 gate。

最终回归：C++ 137/137；Python 69/69，14 workers 并行耗时 25.01 秒。
外部 baseline 测试的 solver 时限永久固定为 10 秒。

### 11.5 结论

三项审查结论：

1. **tricks 确实过多**：大部分只是改变 ordering，数据无稳定方向，已删除；
2. **无意义移动的主因是算法状态等价问题**，不是 renderer 或 validator
   实现错误；大图 0 首解还叠加了 task assignment churn、近单链 DFS 和
   guidance 成本，commitment 只缓解了前者；
3. **生产策略已无布尔/枚举开关**，仅保留 objective 数值输入和诊断。

尚未解决的是搜索内部的状态空间污染。若未来把 shelf-equivalent 状态在
CLOSED 中合并，必须同时保存可重构 robot bridge；只改 hash 会破坏物理
可执行性。

---

## 附录 A：数据与复现

```text
分析工具（重放 + 病态指标 + 案例抽取）:
  /tmp/analyze_carrier_patterns.py     逐实例指标, 输出 /tmp/carrier_pattern_report.json
  /tmp/summarize_carrier_patterns.py   聚合表 + 循环压缩率
  /tmp/loop_erase_proto.py             循环消除原型（含 validator 验证）
  /tmp/run_carrier_ablation2.sh        消融矩阵 runner

实验产物:
  /tmp/carrier_ablation2/    9 配置 x 8 实例 (.plan/.log)
  /tmp/carrier_proto/        LIFT_GATE / OPEN_GREEDY / H_IN_F 原型结果
  /tmp/carrier_looperased/   剪圈后的合法计划

历史原型和阶段性开关均已删除，名称见第 11.3 节。

最终实验产物:
  benchmark/results_rootfix_final/rows.csv
  benchmark/results_rootfix_final/work/*.plan
  benchmark/results_rootfix_protocol_verify/rows.csv
  benchmark/results_rootfix_protocol_verify/timing.json
  benchmark/results_rootfix_protocol_verify/work/*.plan
  benchmark/results_fixed_assignment_restart/rows.csv
  benchmark/results_fixed_assignment_restart/timing.json
  benchmark/results_fixed_assignment_restart/work/*.plan
  benchmark/results_task_commit_final/rows.csv
  benchmark/results_task_commit_final/timing.json
  benchmark/results_task_commit_final/work/*.plan
  benchmark/results_task_commit_final_verify/rows.csv
  benchmark/results_task_commit_final_verify/timing.json
  benchmark/results_task_commit_final_verify/work/*.plan
  benchmark/results_ablation_rootfix_verify/ablation_rows.csv
  benchmark/results_ablation_rootfix_verify/timing.json
  benchmark/viz/task_commit_before_h8w10.html
  benchmark/viz/task_commit_fixed_h8w10.html

当前生产环境输入:
  DD_ALPHA..DD_DELTA   objective 数值
  DD_DEBUG_DUMP        失败诊断
```

## 附录 B：本次用到的实例与关键原始数据

```text
病态组（拥挤, empty=1~8）:
  g4x10/brap_h4w10_a5_e1_R1_seed0      g6x10/brap_h6w10_a6_e1_R1_seed1
  g6x10/brap_h6w10_a6_e1_B_seed0_pool  g8x10/brap_h8w10_a10_e2_R1_seed0
  g10x10/brap_h10w10_a12_e3_R1_seed0   g10x10/brap_h10w10_a12_e8_R1_seed0
对照组（稀疏）:
  g8x10/brap_h8w10_a10_e20_R1_seed0    g6x10/brap_h6w10_a6_e15_R1_seed1

拥挤度 vs 病态（34 计划分组）:
  empty=1   平均 26k~103k 步   原地抬放/lift ~0.95
  empty>=10 平均 1.4k~2.2k 步  原地抬放/lift ~0.5

代价下界对比（soc 口径）:
  h4w10_e1: LB=24,  base+erase=18282  (762x)
  h6w10_e1: LB=42,  base+erase=40860  (973x)
  h8w10_e2: LB=51,  base+erase=26316  (516x)
  h10w10_e8: LB=78, base+erase=35544  (456x)
```
