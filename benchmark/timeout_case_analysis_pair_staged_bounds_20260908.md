# Full 518 超时 testcase 分析

分析对象：

- 权威结果：`benchmark/results_full_pair_staged_bounds_518_20260908/rows.csv`
- 二进制 SHA-256：`855f1f1eb7cb776961a7e3391b4834c684cf3b68b6fd3867ed7f6b6e82bb2045`
- full suite SHA-256：`1998283745b17b636c8d363ada77cc083d73183f22aaec7b3d49e49b8fe2b1d6`
- 配置：`DD_ALPHA/BETA/GAMMA/DELTA=1`、solver seed 0、mode `lacam`
- 方法：31 个 timeout case 各由独立 `openai.gpt-5.6-sol / low` subagent 做只读代码检查和一次 30 秒单例诊断；后期诊断另加 40 秒外层硬上限，防止内部 deadline 失守。

## 总结

31 个 10 秒 timeout 中，只有 dense-channel 的最后一例能在 30 秒内解出，首解时间为 24.080 秒；另外 30 个 BRaP case 在 30 秒下仍无首解。

问题不是一个统一的 “Hungarian 太慢”，而是四类不同瓶颈：

1. **根 guidance / deadline 断链：11 例。** 其中 10 个大 `B_pool` case 在根节点 guidance 返回前就耗尽预算，LaCAM 搜索尚未开始；另 1 个 `a800/R1/seed1` 在后续 guidance 中超过 40 秒。根调用把 pair deadline 明确传成 `nullptr`。
2. **rho canonical 反复求 suffix Hungarian：6 例。** 单次完整 Hungarian 并不慢，慢的是 canonical tie-break 对每个列候选反复重建子矩阵并重新求最优匹配，通常占总时间 73%–98%。
3. **timed-transport 空工作重算 + ready-task 饥饿：5 例。** 即使没有 transport job、expansion 和 frame，代码仍先遍历残余 task，并对每个 task 扫描全部 robot；同时搜索大多只产生 robot-only successor。
4. **pair/vacancy/task-graph guidance 放大：9 例。** 包括全部 20×20 timeout、两个 40×40 `a80/e400/B_pool` 和唯一 dense timeout。这里 rho/timed-transport 不是主因，主要时间落在 pair rollout、vacancy potential、forced repair、task 编译和冲突回溯。

共同问题是：每个新节点都会同步重建 carrier guidance。10 秒协议预留最多 1.5 秒 finalization，所以实际搜索预算是 8.5 秒；30 秒诊断的实际搜索预算是 28.5 秒。

## 逐例结果

### A. 根 guidance / deadline 断链

| # | testcase | 30 秒诊断 | 主要原因 |
|---:|---|---|---|
| 13 | `brap_h40w40_a80_e40_B_seed0_pool` | 34.324s 后返回，根节点未展开 | 根 pair/vacancy；vacancy 26.739s，deadline 未传入 |
| 14 | `brap_h40w40_a80_e40_B_seed1_pool` | 39.155s 后返回，根节点未展开 | 根 pair/vacancy；vacancy 31.060s，deadline 未传入 |
| 15 | `brap_h80w80_a128_e1600_B_seed0_pool` | 运行 118.233s 后终止 | 根 pool pair assignment 未响应 30s deadline |
| 16 | `brap_h80w80_a128_e1600_B_seed1_pool` | 运行至少 165s 后终止 | 同上；正式 10s 只是被外层强杀 |
| 19 | `brap_h80w80_a128_e160_B_seed0_pool` | 40s 外层硬杀 | 根 guidance 未返回，搜索未开始 |
| 20 | `brap_h80w80_a128_e160_B_seed1_pool` | 40s 外层硬杀 | 根 guidance 未返回，搜索未开始 |
| 23 | `brap_h80w80_a160_e1600_B_seed0_pool` | 40s 外层硬杀 | 根 guidance 未返回，搜索未开始 |
| 24 | `brap_h80w80_a160_e1600_B_seed1_pool` | 40s 外层硬杀 | 根 guidance 未返回，搜索未开始 |
| 25 | `brap_h80w80_a160_e160_B_seed0_pool` | 40s 外层硬杀 | 根 pair assignment 收到空 deadline |
| 26 | `brap_h80w80_a160_e160_B_seed1_pool` | 40s 外层硬杀 | 根 pair assignment 收到空 deadline |
| 28 | `brap_h80w80_a800_e1600_R1_seed1` | 40s 外层硬杀 | 后续 guidance 越过 deadline；具体尾部模块因无中途 telemetry 无法再细分 |

这组首先要修的是 deadline 贯穿，而不是搜索 heuristic。根节点在 guidance 完成后才进入 OPEN；当前 pair 调用明确传入 `nullptr`，rho 和 timed-transport 调用也没有收到 planner deadline。

### B. rho canonical 重复 suffix Hungarian

| # | testcase | 30 秒诊断 | canonical 时间 | 搜索进度 |
|---:|---|---|---:|---|
| 7 | `brap_h40w40_a160_e400_R1_seed0` | 无首解，30.815s | 30.233s（98.1%） | 深度 7 |
| 8 | `brap_h40w40_a160_e400_R1_seed1` | 无首解，28.757s | 28.262s（98.3%） | 深度 5 |
| 17 | `brap_h80w80_a128_e1600_R1_seed0` | 无首解，28.759s | 26.920s（93.6%） | 5/128 targets |
| 18 | `brap_h80w80_a128_e1600_R1_seed1` | 无首解，28.977s | 27.044s（93.3%） | 2/128 targets |
| 21 | `brap_h80w80_a128_e160_R1_seed0` | 无首解，29.126s | 23.555s（80.9%） | 4/128 targets |
| 22 | `brap_h80w80_a128_e160_R1_seed1` | 无首解，29.198s | 21.212s（72.6%） | 4/128 targets |

这些 case 的 pair full Hungarian 通常只有 1 次。真正的问题位于 rho canonical：固定一个 robot column 后，对候选 row 逐个调用 `minimum_cost(next_rows, next_cols)`，每次重新构造矩阵并运行 Hungarian。

### C. timed-transport 空工作重算与任务 readiness 停滞

| # | testcase | 30 秒诊断 | timed-transport | 搜索行为 |
|---:|---|---|---:|---|
| 9 | `brap_h40w40_a160_e40_R1_seed0` | 无首解，28.562s | 14.912s（52.2%） | 3946 个 robot-only successor，17/40 targets |
| 10 | `brap_h40w40_a160_e40_R1_seed1` | 无首解，28.557s | 15.360s（53.8%） | `ready_task_count=0`，仅 6 个 shelf-motion |
| 27 | `brap_h80w80_a800_e1600_R1_seed0` | 无首解，28.507s | 11.959s（42%） | rho 无候选；101/800 targets |
| 29 | `brap_h80w80_a800_e160_R1_seed0` | 无首解，28.557s | 13.165s（46%） | 0 个 shelf-motion，目标实际无进展 |
| 30 | `brap_h80w80_a800_e160_R1_seed1` | 无首解，28.630s | 12.894s（45%） | 0 个 shelf-motion，几乎全是单 robot 空走 |

共同证据是 `timed_transport_expansions=0`、`frames=0` 仍花费 12–15 秒。原因是 `jobs.empty()` 检查发生在残余 task × robot 距离扫描之后。

### D. pair/vacancy/task-graph guidance 放大

| # | testcase | 30 秒诊断 | 主要证据 |
|---:|---|---|---|
| 1 | `brap_h20w20_a40_e100_B_seed0_pool` | 无首解，28.616s | guidance 28.344s；动态 pool 造成 epoch/pair churn |
| 2 | `brap_h20w20_a40_e100_B_seed1_pool` | 无首解，28.678s | guidance 28.377s；121 万 rollout steps |
| 3 | `brap_h20w20_a40_e10_B_seed0_pool` | 无首解，28.826s | guidance 28.129s；113 万 effect conflicts |
| 4 | `brap_h20w20_a40_e10_B_seed1_pool` | 无首解，28.819s | guidance 28.160s；170 万 effect conflicts |
| 5 | `brap_h20w20_a40_e10_R1_seed0` | 无首解，28.746s | 低空位导致 epoch 抖动、433 万 effect conflicts |
| 6 | `brap_h20w20_a40_e10_R1_seed1` | 无首解，28.786s | guidance 27.560s；任务链 overlap 仅约 25% |
| 11 | `brap_h40w40_a80_e400_B_seed0_pool` | 无首解，29.084s | vacancy 8.995s；pair rollout/未细分 guidance 约 17.6s |
| 12 | `brap_h40w40_a80_e400_B_seed1_pool` | 无首解，32.033s | vacancy 11.950s；约 80.8 万 rollout steps |
| 31 | `dense_channel_bedge_h40w40_b9_a1_s76of81_r32_t48_seed0` | **24.080s 首解；26.918s 返回** | guidance 26.332s；pair/task 编译主导，rho 0.861s、timed 0.219s |

Dense case 不是无解：首解 makespan/SOC 为 `137/3287`，第二次改进得到 `134/3441`。它在 10 秒下失败，是因为 8.5 秒搜索预算远小于 24.080 秒首解时间。

## 已核对的代码事实

- 根节点先执行 guidance，之后才进入 OPEN：`lacam/src/tapf_planner.cpp:257-266`
- 每个新 successor 也同步执行 guidance：`lacam/src/tapf_planner.cpp:604-614`
- guidance 主入口没有 deadline 参数：`lacam/src/tapf_planner_carrier.cpp:130-137`
- 根 pair assignment 明确传入 `nullptr` deadline：`lacam/src/carrier_epoch_commitment.hpp:458-461`
- rho execute/prepare 调用没有传 deadline：`lacam/src/carrier_upper_epoch.hpp:282-311`
- rho canonical 为每个候选重新求 suffix Hungarian：`lacam/src/carrier_rho_match.hpp:791-899`
- timed-transport 在 `jobs.empty()` 前先执行 task × robot 距离扫描：`lacam/src/carrier_joint_transport.hpp:231-303`
- finalization reserve 最大为 1.5 秒：`lacam/src/dd_planner.cpp:107-115`

## 优化优先级

1. **先贯通 deadline。** 从 `TAPFPlanner` 传到 root/rebuild guidance、pair/vacancy、rho 和 timed-transport；cutoff 必须可向上返回，不能等完整 guidance 结束。
2. **改 rho canonical。** 复用 Hungarian primal/dual 或最优零边图，得到确定性 canonical assignment；不要为每个候选重跑 suffix Hungarian。
3. **改 timed-transport 空工作路径。** 没有 ready/custody job 时避免每节点重做 task × robot 扫描；按 upper epoch 缓存或增量更新 residual estimate。
4. **细化 pair/vacancy/task graph 的增量失效。** 只重算受少量货架移动影响的边、potential 和 task 子图，避免小变化触发整套 guidance 重建。
5. **最后处理 readiness/search quality。** `a800` 和部分 `e40 R1` case 即使去掉热点仍可能没有 shelf-motion，需要单独定位为什么 ready frontier 长期为空。

本轮只做分析和诊断，没有修改 planner 实现、benchmark testcase 或现有测试。
