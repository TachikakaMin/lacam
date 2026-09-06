# Carrier-LaCAM 两遍规划实现与运行时间报告

日期：2026-09-06
状态：实现、单元/回归测试、77-case quick benchmark、独立审批和 509-case
full benchmark 均已完成。

## 1. 最终实现结论

当前 production 路径已经实现本轮确认的语义：

```text
第一遍：原始完整 goal sets + FIRST_FEASIBLE
    ↓
规范化、事务式 repair、权威 replay，保存独立可交付计划
    ↓
读取该计划的实际终态，固定每个目标货架的 singleton goal
    ↓
第二遍：从原始 X0 重新运行同一个 TAPFPlanner::solve()
         + 首轮 (makespan, work) 严格上界
         + FIRST_STRICT_IMPROVEMENT
         + 可选参考动作/checkpoint 后缀
    ↓
第一份通过验证的严格改进立即返回；
否则返回首轮计划，整体仍为 SOLVED
```

没有第三遍，也没有第二套 planner。两遍都进入
`lacam/src/tapf_planner.cpp::TAPFPlanner::solve()`；控制器只负责准备 goal view、
上界、参考计划和最终 fallback。

## 2. 成本、目标与停止条件

### 2.1 唯一成本

`lacam/include/tapf_planner.hpp::PlanCost` 使用：

```text
J(P) = (ticks, fixed-point weighted work)
```

比较是严格词典序：

```text
(T2, W2) < (T1, W1)
    iff T2 < T1
    or (T2 == T1 and W2 < W1)
```

`work` 以 `10^-6` 固定点整数存储，避免浮点 epsilon 改变 OPEN 排序或上界
判断。一个 primitive joint action 消耗一拍；macro edge 的 `ticks` 等于其
primitive trace 长度。

### 2.2 显式停止策略

`TAPFStopPolicy` 有三个值：

```text
ANYTIME
FIRST_FEASIBLE
FIRST_STRICT_IMPROVEMENT
```

构造器会拒绝不一致配置：

- `FIRST_FEASIBLE` 必须使用无界 external incumbent；
- `FIRST_STRICT_IMPROVEMENT` 必须带有界 incumbent。

goal 只有满足以下条件才会被接受：

```text
candidate.g < current in-search goal
candidate.g < external incumbent（若存在）
```

因此第二遍不会把与首轮同成本的计划误算成改进。接受第一份严格改进后，
`TAPFPlanner::solve()` 当轮立即退出普通扩展。

## 3. 第一遍如何形成保底快照

控制器在 `lacam/src/dd_planner.cpp::solve_carrier_lacam_result()` 中调用：

```text
stop policy       = FIRST_FEASIBLE
goal view         = 原始完整 goal sets
incumbent         = unbounded
macro             = 按原有规模条件启用
start             = 原始 X0
```

kernel 返回后，`run_search_attempt()` 依次执行：

1. 从 TAPF state chain 提取 primitive Carrier joint actions；
2. 截断到第一个有效 goal prefix；
3. 运行事务式 repair；
4. 再次截断并权威 replay；
5. 用同一固定点权重核算最终 `PlanCost`。

repair 不是破坏式更新。进入 repair 前的 normalized raw plan 已经是合法
fallback；repair 截止、找不到替换或候选无效，都保留原计划。第一遍搜索树随后
销毁，第二遍不持有首轮 `TAPFNode*` 或 guidance metadata。

## 4. 固定目标与第二遍

`fixed_goal_instance_from_plan()` 从初态逐拍调用 `apply_ops()` 重放首轮最终计划。
只有重放合法且终态满足原始 goal 时，才为每个目标货架写入：

```cpp
fixed.target_goals[b] = state.target_pos[b];
fixed.target_goal_sets[b] = {state.target_pos[b]};
```

固定的是最终目标货架位置，不是中间 `tau`，也不固定：

- 匿名货架临时位置；
- robot-to-shelf 分配；
- blocker endpoint；
- 路线、等待或冲突区通过顺序。

原本就是 singleton goal 的实例也进入第二次 bounded search；只是
`assignment_restarts` 统计不把它记作 dynamic-assignment restart。

第二遍配置为：

```text
stop policy       = FIRST_STRICT_IMPROVEMENT
goal view         = 首轮实际终态 singleton goals
incumbent         = 首轮 repair/replay 后 PlanCost
macro             = false
start             = 原始 X0
deadline          = 与第一遍共享
```

第二遍没有候选、搜索空间耗尽或到达 cutoff 时，控制器保留首轮 plan 和
`DDSolveStatus::SOLVED`。`assignment_second_solved` 只统计第二遍真实生成的新
候选，不再把 external fallback 冒充本轮 goal。

## 5. 上界剪枝和 cheaper-arrival 重开

`TAPFPlanner::solve()` 的 `current_bound()` 返回当前本轮 goal 成本；尚无本轮
goal 时返回 external incumbent。以下位置都统一要求 `f < bound`：

- FOCAL eligibility；
- OPEN pop 后检查；
- successor 入 OPEN；
- `rewrite()` 后的重新激活。

所以：

```text
f = (54, 90), bound = (54, 90)  → 剪掉
f = (54, 85), bound = (54, 90)  → 保留
f = (50,110), bound = (54, 90)  → 保留
```

剪枝只作用于当前到达成本，不永久拉黑物理状态。`rewrite()` 发现更便宜前缀时会
更新 `parent/incoming_edge/g/f`、传播到出边，并在新 `f < bound` 时重新放入
OPEN。

安全剪枝仍只使用 `g + admissible h`。PairCost、交通预测和首轮后缀成本只作
guidance/可行上界，不作为剩余代价下界。

## 6. 首轮参考路径

### 6.1 构造

首轮已验证计划最多均匀选取 256 个 checkpoint。每个 checkpoint 保存：

```text
完整 PhysConfig
state hash
action index
首轮 prefix cost
首轮 suffix cost
下一拍 primitive joint action
```

完整 `PhysConfig` 包括 robot 位置、目标货架位置、规范化匿名货架占据和
`kappa`。哈希只用于索引，命中后仍必须做完整相等比较。

### 6.2 动作提示

当第二遍到达 checkpoint 状态时，首轮下一动作只会被移动到已有合法候选列表的
前部。它不能删除其他 operators，也不能绕过当前 constraint tree 或
`apply_ops()`。

### 6.3 后缀拼接

只有满足：

```text
当前完整 PhysConfig == checkpoint PhysConfig
current.g + checkpoint.suffix_cost < incumbent
```

才逐拍重放首轮 primitive suffix。每一步重新计算成本并调用 `apply_ops()`。
终态再次通过 goal 检查后，suffix 作为普通多步 `SearchEdge` trace 注册到现有
CLOSED graph，再由普通 `accept_goal()` 接受。

如果：

```text
current.g + reference_suffix >= incumbent
```

只是不尝试该拼接，绝不剪掉当前节点。首轮后缀是“存在一种完成方式”的可行上界，
不是 admissible lower bound。

## 7. 为什么过去常在 7–8 秒停止，现在为什么仍可能接近 9 秒

旧控制有两层预留：

```text
10.00 s 外部协议
 - 1.50 s controller finalization reserve
 = 8.50 s shared search deadline
 - 0.85 s kernel incumbent cleanup reserve
 = 7.65 s bounded-search cutoff
```

所以许多无改进案例在约 7.65 秒停止扩展，清理后约 7.7–8.3 秒返回。

当前实现保留 controller 的一次 1.5 秒交付余量，但删除 kernel 内第二次
0.85 秒扣减。`TAPFPlanner::solve()` 直接服从传入的共享 deadline。固定
6-case 子集中的无改进案例现在通常在约 8.7–9.0 秒完成，说明重复预留已经消除。

这不表示算法还在找到改进后继续 anytime：

- 找到严格改进：立即停止第二遍，可能很早返回；
- 找不到改进且不能证明无改进：允许使用剩余搜索预算，之后清理并返回首轮；
- 两种情况共享同一个 10 秒总调用预算。

## 8. 77-case quick benchmark

配置：

```text
suite       benchmark/release_benchmark.json
cases       77
timeout     10 s/case
seed        0
jobs        14（16 个物理核，保留 2 核）
binary SHA  bc08b69ab26ad026e890420b59cb58dadb58daa0b55edc2319ccecc16a57598a
```

与旧 v5 anytime 控制
`benchmark/results_quick_v5_control_854df1_20260905` 配对比较：

| 指标 | 旧 v5 | 当前两遍 |
| --- | ---: | ---: |
| solved | 47/77 | 47/77 |
| 首解中位时间 | 109 ms | 111 ms |
| 首解平均时间 | 954.9 ms | 952.5 ms |
| 成功例 solver runtime 中位数 | 8087.4 ms | 8644.7 ms |
| 成功例 solver runtime 平均数 | 7989.9 ms | 6146.3 ms |

共同求解 47 例的最终词典序成本：

```text
better / equal / worse = 1 / 34 / 12
```

12 个变差案例是预期可见代价：旧控制找到第一份改进后继续 anytime，而当前
设计立即返回第一份严格改进。成功集合没有变化。

第二遍退出原因：

| 原因 | 数量 |
| --- | ---: |
| `SEARCH_CUTOFF` | 31 |
| `STRICT_IMPROVEMENT` | 14 |
| `REFERENCE_SUFFIX_ACCEPTED` | 1 |
| `SEARCH_EXHAUSTED` | 1 |

参考统计：

```text
checkpoint hits       356491
action hints          780743
suffix attempts       1
suffix accepted       1
```

真实 suffix 案例：

```text
brap_h6w10_a6_e15_R1_seed1
首轮          (162, 357)
最终          (162, 355)
solver time   69.1 ms
exit          REFERENCE_SUFFIX_ACCEPTED
```

这个案例说明第二遍用更便宜前缀到达首轮 checkpoint 后，可以直接复用并验证
已知后缀，而无需再次搜索后续全部动作。

## 9. 509-case full benchmark

full benchmark 在独立 GPT-5.6 Sol/high 明确 `APPROVE` 后运行。审批同时绑定：

```text
binary SHA   bc08b69ab26ad026e890420b59cb58dadb58daa0b55edc2319ccecc16a57598a
suite SHA    fae83e9ba41dc8b933c79f7769992b29006bb1fc67004e770e621b0830c890ed
corpus SHA   7840959653b2056c6441ede0cbcd93031f9ec3c4796b8270af2c7d1447a72bae
```

runner 使用 sealed binary/YAML snapshot，14 worker、10 秒/case。结果：

```text
solved                 479 / 509
timeout                 30 / 509
solver time sum       3629.4 s
wall time              265.9 s
成功案例最大进程时间       9.735 s
```

30 个 timeout 与旧 full 完全相同：

```text
g20x20   6
g40x40   8
g80x80  16
```

没有新增或丢失 solved case。full 中对应 quick 的 77 行与单独 quick 的
success、最终 `(T,W)` 和 plan SHA-256 全部相同。

### 9.1 相对旧 v5 anytime full

基线：`benchmark/results_full_v5_854df1_20260905_r2`

| 指标 | 旧 v5 anytime | 当前两遍 |
| --- | ---: | ---: |
| solved | 479/509 | 479/509 |
| 首解中位时间 | 123 ms | 122 ms |
| 首解平均时间 | 670.1 ms | 672.9 ms |
| 成功例 runtime 中位数 | 7776.0 ms | 8619.0 ms |
| 成功例 runtime 平均数 | 7834.9 ms | 6984.5 ms |
| solver time sum | 4037.5 s | 3629.4 s |
| wall time | 295.6 s | 265.9 s |

共同求解 479 行的最终严格词典序成本：

```text
better / equal / worse = 11 / 372 / 96
```

总 makespan 从 33442 变为 33752，总 work 从 145000 变为 145597。96 行成本
回退是新停止语义的直接代价：旧 v5 在第一份改进后继续 anytime，当前设计立即
返回。另一方面，111 行第二遍找到严格改进后不再等待，令平均 runtime、solver
总时间和 full wall time 都下降；无改进案例继续使用剩余预算，所以 runtime
中位数反而上升。

### 9.2 相对 pre-v5 full

基线：`benchmark/historical/pre_v5/results_full_factorial_20260905`

```text
solved                     479 → 479
better / equal / worse     359 / 51 / 69
共同案例 makespan 总和       38716 → 33752
共同案例 work 总和           135913 → 145597
首解中位时间                 127 ms → 122 ms
成功例 runtime 平均数         709.5 ms → 6984.5 ms
```

当前目标首先最小化 makespan，因此 359 个严格改进和 makespan 总和下降约
12.8% 是主要质量收益；work 总和上升约 7.1% 是词典序目标允许的次级代价。
pre-v5 基本不做第二遍优化，所以 runtime 明显更短；两者解决的是不同质量—延迟
取舍，不能只看 runtime 宣称当前全面更优。

### 9.3 第二遍和 reference 的 full 统计

| 第二遍退出原因 | 数量 |
| --- | ---: |
| `SEARCH_CUTOFF` | 367 |
| `STRICT_IMPROVEMENT` | 79 |
| `REFERENCE_SUFFIX_ACCEPTED` | 32 |
| `SEARCH_EXHAUSTED` | 1 |

全部 479 个 solved case 都尝试了第二遍；111 个产生并接受严格改进候选。
比较 kernel 首解与最终交付计划，129 个最终更好、350 个相同、0 个变差。

reference 汇总：

```text
checkpoint hits       3,135,194
action hints          6,208,118
suffix attempts              32
suffix accepted              32
```

32 次 suffix 尝试全部经过逐拍 replay 并成功形成严格改进，其中不少案例在几十到
几百毫秒内返回。后缀尝试少于 checkpoint 命中是正常现象：只有
`current.g + suffix < incumbent` 时才值得拼接，其余命中只用于动作排序。

## 10. 测试与兼容性

当前已通过：

```text
C++ tests      296 / 296
Python tests   159 / 159
quick cases     77 / 77 执行完成
```

重点覆盖：

- PlanCost 严格词典序和固定点表示；
- 第一遍首解、第二遍首份严格改进；
- cutoff/exhausted 时首轮 fallback；
- transactional repair；
- fixed-goal 构造；
- bound 剪枝和 cheaper-arrival reopen；
- 完整状态 checkpoint、哈希碰撞和重复状态；
- reference action 只排序；
- suffix trace 注册、逐拍 replay 和严格改进；
- 初态已满足 goal 的空计划；
- 原 shelf-free TAPF 自然退化与 60 秒 anytime 兼容测试；
- benchmark 10 秒、validator、full review gate 和产物哈希。

无 pick/place 输入不通过 feature flag 或旧 planner fallback。`ShelfState` 为空，
Carrier cost/guidance 分支结构性不执行，仍进入同一个
`TAPFPlanner::solve()`；原 TAPF compatibility tests 全部通过。

## 11. 已知边界

1. 第二遍的 bound 与 `g+h` 在 raw search-cost 空间比较。repair 只能替换成合法且
   更好的输出，但当前版本不声称覆盖“raw 计划不优、仅 repair 后才会优”的所有
   候选。
2. 固定 assignment 下没有改进，不代表原始多 goal 问题全局最优。
3. reference action hints 命中很多，但只有 32 次形成可接受后缀；单独区分
   “排序提示”对扩展量的净贡献还需要 reference-off ablation。
4. “第一份改进即返”降低平均返回时间，但会放弃旧 anytime 在同一预算内后来找到的
   更优计划；quick 中的 12 个成本回退已如实保留。
5. 367/479 个 solved case 没有找到严格改进并运行到 search cutoff。这是
   “愿意用剩余预算找改进”的当前策略；若以后要限制尾延迟，应新增明确的
   improvement budget，而不是重新引入隐式 deadline 双重预留。
