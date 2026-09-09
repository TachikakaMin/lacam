# carrier 方法 vacancy Phase A 收尾：回退诊断与实施计划（debug.md）

状态：2026-09-07 实现、quick、全量代码回归、独立复审和当前 sealed
full 509 均已完成。
上游文档
`carrier_vacancy_shared_carrier_method_plan_20260906.md`（下称 plan）。
本文第 1、2 节保留实施前的核实与诊断证据，第 3、4 节记录执行方案，
第 6 节给出实际落地结果。工作树基线：commit `36622a9` + vacancy
Phase A；当前二进制和结果均来自其上的 shared-carrier 修复。

一句话结论：plan 的三个待办已经完成。A/B 证明主要回退来自执行期的
"epoch 间链重选/引导翻转"；最终实现把有限 continuity 保留在会发生
动态 tau 分配的 flexible shared-pool 实例，固定目标实例自然使用当前
几何重新编译，并补齐 telemetry、在途 effect、goal commitment 与缓存
归一化。carrier quick 已过门槛，carrier_brd quick 与原结果逐项一致。

---

## 1. plan 文档需要修改的地方（逐条核实）

1. **§5 初诊假设需要修订（最重要）**。plan 写"恶化发生在首解…在于
   多 target 场景下空位引导与联合编译/rho 匹配的交互"。本轮用 HEAD
   worktree（36622a9，无 vacancy 改动）与工作树做了 A/B：
   - 初始联合任务图（`dd_compile_joint_graph_probe`，
     `brap_h10w10_a12_e3_B_seed1_pool`）：HEAD 63 tasks /
     9193 effect_conflicts / 3451 backtracks；vacancy 树 **47 tasks /
     175 conflicts / 52 backtracks**。初始编译在新树上明显更好。
   - 同案例端到端 matched rerun（同机同参 10s，确定性复现）：
     HEAD mk 780 / soc 1738 / causal_waiting 1 / custody 1186 /
     epoch_builds 516；vacancy mk 1512 / soc 3286 /
     **causal_waiting 817 / custody 4207 / epoch_builds 1004**。
   即回退是**执行期累积**的，不是 t=0 的编译交互。§5 的诊断步骤 1
   （对比初始联合图）已执行且结论反向，后续步骤按本文 §2/§4 重写。
2. **§5 缺一个关键判别数据**：回退清单里有单 target 案例
   （warehouse b3_a1），多 root 交互假说对它们不成立。本轮 matched
   rerun（`warehouse_blocks_h20w20_b3_a1_d50_r8_t12_seed0`）：
   loaded_moves 33、lift_drop 24 **两树完全相同**，恶化全部来自
   free_moves 100→122 与 upper_epoch_builds 39→98。"链本身变贵"被
   排除，指向"epoch 间链选择翻转，机器人空跑接错任务"。
3. **§3 证据面已过时**：plan 写于 full 509 之前。现补：brd full
   `results_full_carrier_brd_20260906`(326/509) →
   `results_full_vacancy_phase_a_20260906`(**336/509**，+10 新解 0 丢失，
   公共 326 例 sum mk -21.8%、sum soc -14.2%，soc>5% 恶化 10 例)。
   carrier 的 full（479/509 @ rho-v2）在 vacancy 树上尚未跑（需审批）。
4. **§4.2 telemetry gap 已补齐**：`TAPFStats`、`DDStats`、
   `dd_benchmark` 与 runner 现在统一导出 vacancy potential、首选回退、
   epoch churn、cache eviction 与 guidance version。
5. **§4.3 carrier 入口回归测试已补齐**：
   `test_dd_carrier_vacancy_regression` 覆盖多目标 pool、单目标 warehouse
   churn 和报告案例；窄图边界另由
   `test_dd_vacancy_narrow_regression` 固化。
6. **§6 quick 验收已完成，full 尚未运行**：carrier 为 47/77，
   公共 47 例 sum mk/soc 为 14454/37216；carrier_brd 为 51/77，
   与 vacancy Phase A r3 的公共 51 例逐项相同，报告案例仍为
   15 transfers。full 继续受独立 APPROVE gate 约束。
7. §2 的行号在当前树上全部复核准确（1335/1813/2021/2039/2076/
   2233/2452/3274/3582、tapf 501/508/555/650、brd 1345+/415/428/500/517）。

## 2. 本轮代码 review 与诊断证据

### 2.1 实现质量结论（共享层，无需改动的部分）

- `build_vacancy_potential_layer`（1882+）：多源 Dijkstra/BFS 双路径、
  确定性 tie（`next_vacancy_cell` 取小 cell）、periodic deadline、
  INF 溢出保护（`clearance_cost_add_component`）齐全。
- tau 变体（2039+）已实现 plan §5 提到的保护：assigned goal 置为
  excluded source + settled-target 逐格罚项（词典序第一键），并分
  `cost` / `anonymous_cost` 两层。
- `VacancyPotentialCache`（2076+）key 只含占据位图——**正确**，因为
  只有非 tau 变体走缓存（3106-3115 单 root 路径）；tau 变体在联合
  编译内本地构建（3306-3319），无 stale-tau 风险。容量启发
  `262144/cells`。
- `record_first_choice_fallback`（2640-2643）只在
  `candidate_index==0 && count>1` 计数，语义是"首选被拒"，符合
  telemetry 命名。

### 2.2 回退机制的证据链（本轮新增）

复现命令（全部已跑通，数字确定性）：

```sh
# HEAD 对照树（一次性）：
git worktree add /tmp/dd-lacam-head HEAD
# 该 worktree 缺 submodule：把 third_party/{argparse,googletest,lacam2,ITA-CBS2}
# symlink 到主树；cmake 需 PKG_CONFIG_PATH=/home/yimint/.local/lib64/pkgconfig
cmake -B build-head && make -C build-head lacam dd_benchmark -j12

# A/B 初始联合图（tools/diag_joint_graph_probe.cpp，两树均可编译）：
g++ -std=c++17 -O3 -DNDEBUG -I lacam/include tools/diag_joint_graph_probe.cpp \
    <tree>/liblacam.a -L/home/yimint/.local/lib64 -lyaml-cpp -lstdc++fs -o probe

# matched 端到端：
<tree>/dd_benchmark benchmark/instances_brap_pool/g10x10/brap_h10w10_a12_e3_B_seed1_pool.yaml \
    10 out.plan 0 lacam
```

观测汇总（vacancy 树相对 HEAD）：

| 信号 | a12_e3_B_seed1（12 目标/3 空位/2 robot） | wh b3_a1_d50（单目标） |
|---|---|---|
| 初始联合图 | 47/175/52 vs 63/9193/3451（更好） | — |
| tau 变化 | 7/12 root 改选 | — |
| loaded/lift_drop | 变差（505/842 vs 267/494） | **完全相同**（33/24） |
| free_moves | 1553 vs 773 | 122 vs 100 |
| upper_epoch_builds | 1004 vs 516 | 98 vs 39 |
| causal_waiting | 817 vs 1 | 0 vs 0 |
| custody_continuations | 4207 vs 1186 | — |

### 2.3 根因候选（按证据强度排序）

- **H1 epoch 间链重选（churn）**：势能对上层状态高度敏感——每次
  transfer 都移动空位，下一个 epoch 的 Q 排序可能整体翻转，PIBT
  期望算子随之改向，机器人空跑/交还任务（custody ×3.5、free_moves
  +22% 且 loaded 不变）。单 root 案例是它的最纯净证据。
- **H2 新 tau 在低 robot 数下丢并行**：a12 案例 7/12 tau 改选 +
  causal_waiting 1→817，说明新链间因果依赖更深（b9 一条 27 任务链）。
  与 H1 叠加放大。
- **H3 epoch 缓存键抖动**：`UpperEpochCache` key 含
  priority_commitment；若 commitment 在等价状态间翻转会造成重复
  build（epoch_builds ×2.5 的一部分）。次要，先测量再定。

## 3. 不变约束

沿 rules.md：无 feature flag / legacy 路径 / fallback；先 RED 后改码；
开发期只跑 quick；分阶段独立提交与结果目录。诊断用的对照一律走
"HEAD worktree 二进制 vs 工作树二进制"，不在产品代码里做开关。

## 4. 阶段计划

### R1：churn 可观测性（先做，纯 telemetry，不改行为）

1. 共享层加计数（`VacancyGuidanceTelemetry` 旁）：
   - `epoch_first_transfer_flips`：同一 root 在相邻 epoch 的首选
     transfer（TransferKey）变化次数；
   - `epoch_chain_overlap_pct`：相邻 epoch ready 链的 TransferKey
     Jaccard；挂在 `build_task_br_guidance` 的 previous_guidance
     对比处（7340+ 已有 previous_epoch 访问）。
   - `upper_epoch_cache_evictions`（容量 256 驱逐计数）。
2. 导出到 lacam 模式（与 T 阶段合并做）：`TAPFStats`（866+）→
   `map_stats`（dd_planner.cpp:355）→ `dd_benchmark.cpp` 通用段 →
   `run_benchmark.py` 列。
3. RED：单 root warehouse 案例上断言 telemetry 字段存在且
   flips 在 HEAD 语义下为小值的合同不好写——改为纯观测，不设阈值
   断言；测试只验证字段导出与非负。
4. 验收：两个 focused 回退案例的 flips/overlap 数据支持或否定 H1。

### R2：判别实验（test-only probe，不动产品路径）

1. `dd_planner.hpp` 加 probe：`dd_solve_carrier_lacam_fixed_tau_probe(
   ins, tau_override, …)`——复用 b1 的 fixed-tau 装配方式，只在测试/
   诊断中调用（与现有 *_probe 家族同规格，rules.md 允许 probe）。
2. 交叉试验：新树 × HEAD tau、新树 × 新 tau，在 a12 案例上比较
   mk/soc/causal_waiting。tau 固定后回退消失 → H2 主导；仍在 →
   H1 主导。
3. 同时给 joint 图 probe 加 `--tau-override` 输入跑 t=0 交叉（已有
   `dd_compile_joint_graph_probe(tau_override)` 参数，纯脚本工作）。

### F：修复（依 R1/R2 结论选择，均在共享层，两方法同享）

- **F-H1 链连续性（hysteresis）**：候选排序在 Q 之后、reserved 之前
  加一个 continuity 键——上一 epoch 已选中的 TransferKey（经
  priority_commitment 通道传入，挂点 build_task_br_guidance 7340+ 与
  `ordered_shelf_candidate_window`）得 0，其余得 1。只影响 tie 区，
  不改变 exact oracle 与候选集合。
- **F-H2 tau 稳定性**：`solve_tau_guide` 的 moved_away 次键已有；若
  H2 主导，考虑把"上一 epoch tau"作为同成本 tie 的第三键（同样经
  epoch 通道传入）。禁止直接加权惩罚（会破坏 L(e)≤C(e) 合同需重验）。
- **F-H3**：若驱逐显著，容量自适应或 key 归一化（commitment 排序）。
- 每个 F 独立提交独立 quick 目录；RED 测试先行：
  1. a12_e3_B_seed1：lacam 模式 soc ≤ 1738×1.05（回到基线水平）；
  2. wh b3_a1_d50：free_moves ≤ 110（loaded 不变前提下）；
  3. 报告案例保持 mk≤49/soc≤101；
  4. brd quick 51/77 与 focused 15 transfer 不回退。

### T：telemetry parity（plan §4.2，独立小提交）

`TAPFStats` 增 `vacancy_potential_builds/_time_ms/_unreachable_cells`、
`clearance_first_choice_fallbacks`、`guidance_version`；CarrierEngine
的 compile/rollout 路径把 `VacancyGuidanceTelemetry` 累计进来
（tapf_planner.cpp 651-655 一处 + rollout 2718+ 共用）；
`map_stats`→`dd_benchmark` 通用段→`run_benchmark.py` 列。RED：跑
报告案例 lacam 模式断言字段非零。

### C：carrier 入口回归测试（plan §4.3）

`tests/test_dd_carrier_vacancy_entry.cpp`：报告案例
`solve_carrier_lacam_result` 10s 内 soc ≤ 101、mk ≤ 49（阈值给 10%
余量：soc ≤ 111）；断言引导首个 target transfer endpoint 为 (5,2)
（用 `dd_task_br_guidance_probe`）。

### 收尾

C++/Python 全量 GREEN → quick 77 双方法（carrier 主 slot +
carrier_brd 替换 slot）→ 独立审查 → full 509（需 APPROVE JSON）→
汇报页面（rules.md Final validation）。

## 5. 验收标准（在 plan §6 基础上修订）

- carrier quick：solved ≥47；公共案例 sum mk ≤ 15133、sum soc ≤ 38673
  （不劣于当前 vacancy 树）；§1.2 表 8 例中 soc>5% 恶化收敛到 ≤2，
  且 a12_e3_B_seed1 回到 soc ≤ 1825（基线×1.05）。
- carrier_brd quick 51/77、full ≥336/509 不回退；报告案例 15 transfer。
- telemetry：carrier 行可见 vacancy_potential_* 与 churn 字段。
- 新增测试全绿且进 protected 集。

## 6. 实际落地与当前证据

实现仍只有一条生产路径：

- `build_task_br_guidance` 在相邻合法物理转移上恢复 custody、forced
  effect、priority 与 root-goal commitment，再调用原有 Task-BR 编译；
- vacancy potential 只改变候选的 Q 排序，continuity 只在 exact-Q
  tie 后、reservation tie 前生效，不改变候选集合或合法性；
- continuity 只用于至少含一个多候选 goal set 的实例。singleton
  fixed-goal 实例没有动态 tau 抖动，因此不把历史首 transfer 写入新
  epoch；shared-pool 合同仍保留，并具有一 epoch 生命周期和 stale-source
  过滤；
- 在途货架只有位于非 storage transit cell 时才需要给新任务图注入
  forced transfer。位于 storage cell 的 episode 由 custody 延续，避免
  把已经可由当前 frontier 重现的 effect 再次强制；
- fixed-goal forced effect 先与当前无强制 frontier 比较。同 root 集的
  predecessorless transfer 已足够时归一化为空；真正参与共享依赖、后继
  解锁或 rotation 的 effect 才保留；
- flexible target 在搬运中或仍有 active root 时保持上一 tau goal，
  commitment 必须合法且全局 injective，到达目标后释放。singleton
  target 不创建历史依赖。

最终 quick 产物：

| 路径 | solved | 公共例 makespan | 公共例 SOC | 备注 |
|---|---:|---:|---:|---|
| carrier Phase A | 47/77 | 15133 | 38673 | 修复前对照 |
| carrier shared fix | 47/77 | **14454** | **37216** | SOC>5% 回归 2 例 |
| carrier_brd Phase A r3 | 51/77 | 25033 | 76005 | 共享层对照 |
| carrier_brd shared fix | 51/77 | **25033** | **76005** | 逐例相同 |

最终 release-candidate 二进制为
`build/dd_benchmark`，SHA-256
`e986ba3740aa259e8d485f2bbe96e32e887cf09494df470873b6b667eee8e1ae`。
carrier quick 位于
`benchmark/results_quick_vacancy_shared_carrier_20260907_r5`，
carrier_brd quick 位于
`benchmark/results_quick_vacancy_shared_carrier_brd_20260907_r2`。
r5 与 r4、brd r2 与上一版在 success、cost、动作计数和 plan SHA 上均为
零差异。

carrier 剩余两例 SOC>5% 回归是
`brap_h8w10_a10_e2_B_seed1_pool`（509→831）和
`brap_h6w10_a6_e1_B_seed0_pool`（287→321）。它们都是 flexible-pool
场景，已按 plan 的“≤2”边界保留为负面证据，不再据此选择 seed 或修改
benchmark。与此同时，`a12_e3_B_seed1` 从 3286 降至 1562，报告案例
保持 49/101，窄图 protected case 保持 389/769。

验证状态：

- C++ `test_all`：412/412；
- Python benchmark tests：198/198（含最终报告、去模板化、静态证据
  portability 与主索引回归测试）；
- warehouse proposal：5/5，四个 planner plan SHA 保持
  `64693a…`、`e37df7…`、`8a103b…`、`0565ea…`；
- warehouse proposal 的四条记录均绑定当前 `d1ea40…` 二进制；
- `git diff --check` clean；
- quick 使用 16 个物理核中的 14 个、每例 10 秒、seed 0、unit weights。

封存后对
`g6_dmedium_ascarce_pcross_heavy_gsingleton_seed0` 做了终局交付定点修复。
原计划中，携带 `b11` 的机器人已经与固定目标相邻，却被一台普通取货机器人
抢先占用终点，随后把货架带走再送回来，最终得到 `(T,W)=(198,857)`。
当前排序只在“另一台空闲 dispatch 的下一单正是在该终点取货”时优先完成
相邻交付；终点只是去往别处取货的最短路中间格时，不覆盖原任务优先级。
同时只在另一台载货机器人下一拍将进入当前通道格、原时序提示又会让本车
离目的地更远时，优先驶入相邻的空 storage 会车位。修复后为
`(106,770)`；载货移动由 377 降为 371。新增 terminal-delivery scope
回归后，曾稳定退化的 `brap_h10w10_a12_e8_R1_seed0` 连跑三次均恢复为
`(1099,2360)`。当前 quick 位于
`benchmark/results_quick_terminal_scope_20260907`，为 47/77，公共成功例
sum makespan/SOC 为 `14454/37216`，与 shared-fix r5 的成功集合和逐例
cost 完全相同；rows SHA-256 为 `e1ad57fc…`。

当前 sealed release 二进制
SHA-256 为
`d1ea40f67d281da6a82eea649d7de8e2d8a5e7d13d22407523608c3b44881fe2`。
旧 `e986ba…` sealed full 继续保留为历史证据，不回写其汇总。

终审指出的两个代码阻塞项已经完成 RED→GREEN：fixed-goal forced-effect
normalization cache key/API 已删除不参与计算的 continuity 维度；full
approval gate 现在强制 `reasoning_effort=high`、非空 reviewer agent、
严格 UTC 时间、非空摘要和空 `blocking_findings`，并继续绑定
suite/corpus/binary。相关 protected-test 修改已由独立 GPT-5.6 Sol/high
agent `01a079d6-79d0-7072-9927-507dafd5c3ab` 明确 `APPROVE`。

旧 `e986ba…` sealed full 的代码终审由 GPT-5.6 Sol/high agent
`01a079ea-2f3f-7a11-a13e-655e42b7e803` 明确 `APPROVE`。approval
绑定 full suite
`fae83e9ba41dc8b933c79f7769992b29006bb1fc67004e770e621b0830c890ed`、
corpus
`7840959653b2056c6441ede0cbcd93031f9ec3c4796b8270af2c7d1447a72bae`
和 release binary `e986ba…`。该 approval 不适用于当前 `d1ea40…`
候选。当前候选随后由同一独立 agent 在 `2026-09-07T21:27:58Z` 重新
`APPROVE`；schema-v2 approval 位于
`benchmark/full_review_approval_terminal_scope_20260907.json`，重新绑定
同一 suite/corpus 和 `d1ea40…` binary。

旧 `e986ba…` sealed full 509 的结果必须保留为负面证据：

- carrier 为 475/509，低于 rho V2 的 479/509，没有新增求解，新增 4 个
  timeout；公共 475 例为 141/201/133（更好/相同/更差），makespan
  几何平均变化 `-0.72%`，总 makespan `32471→28717`，SOC
  `137491→128793`。共同成功例的质量改善不能抵消 solved-set 回退；
- 4 个丢解全部是 g6 shared-pool factorial：三例 scarce、一例 surplus。
  它们已在报告中逐项列出，不能据此重新选 seed、删 case 或调参；
- carrier_brd 为 336/509，与 Phase A solved 集相同；336 个成功例的
  cost、动作计数和 plan SHA 全部相同。只有
  `brap_h40w40_a160_e40_R1_seed1` 的失败标签从
  `SEGMENT_TIMEOUT` 变为 `SEARCH_TIMEOUT`；
- carrier rows/timing SHA 分别为 `b061eef4…`、`53fcee47…`；
  carrier_brd 分别为 `30ae4722…`、`e2b4ecbb…`。两次 full 都使用
  14 jobs、10 秒/例和同一 `e986ba…` 二进制，没有 invalid success，
  失败行也没有残留 plan hash。

当前 `d1ea40…` sealed full 位于
`benchmark/results_full_terminal_scope_20260907`：

- 结果仍为 475/509，factorial 为 428/432，和旧 `e986ba…` 的 solved
  set 完全相同；wall time 263.1 秒，14 jobs，10 秒/例；
- 对旧版 475 个共同成功例，词典序为 30/393/52
  （更好/相同/更差），总 makespan `28717→28875`，SOC
  `128793→129669`。因此关键案例修好，但整体质量略退；
- 目标案例从 `(198,857)` 改为 `(106,770)`。同时必须保留明显回退：
  `g6_dlow_abaseline_plocal_gsingleton_seed0` 从 `(73,663)` 变为
  `(187,986)`，`g6_dmedium_abaseline_plocal_gsingleton_seed0` 从
  `(74,690)` 变为 `(173,881)`；
- 相对 rho V2 仍丢失同 4 个 g6 shared-pool case。公共 475 例为
  137/195/143，总 makespan `32471→28875`，SOC `137491→129669`；
- rows/timing SHA-256 分别为 `d938787b…`、`c72ed2fc…`，approval SHA-256
  为 `0b5c50a4…`。当前 dashboard、旧版对比和 rho V2 对比分别位于
  `full_benchmark_terminal_scope_20260907`、
  `full_comparison_vacancy_shared_carrier_vs_terminal_scope_20260907` 和
  `full_comparison_rho_v2_vs_terminal_scope_20260907`。

网页终审首次指出旧版对比页把实际回退的总指标错误标成绿色。新增
`test_full_comparison_semantics.py` 先复现 RED，再让严格词典序计数、
makespan/work 几何比和 wall ratio 按真实方向选择 good/bad/same。
相关 6/6 测试与 Python 全套 198/198 通过；两个 comparison 页面重生后，
独立 GPT-5.6 Sol/high reviewer 最终 `APPROVE`。

正式静态报告位于
`benchmark/viz_web/vacancy_shared_carrier_final_report_20260907/`，
并链接 current full dashboard 与 rho V2 逐例对比。报告生成器中的 lost
数量已按 protected-test 流程由硬编码改为 `len(full["lost"])`；测试变更由
agent `01a07a02-d678-7162-9a7b-134d844c3fad` 批准。报告共有 34 个本地
链接，全部存在；最终网页由 GPT-5.6 Sol/high agent
`01a07a03-f792-7b01-91cd-e02def74dd8c` 独立核对数字、负面结论、导航、
中文可读性与移动端布局后 `APPROVE`。本轮实现、验证、sealed full 和汇报
均已完成；尚存的算法回退就是上述 4 个 full timeout 与 2 个 quick SOC
回退，不在 sealed 结果后继续调参。

随后按 `kill-ai-slop` 清理最终报告展示：删除等权统计卡、彩色状态盒、
机制 tile、panel 套 table 和“不是 X，而是 Y”式文案，改为单一强调色、
主指标优先的数据行、definition list 与直表。根索引只展开当前
2026-09-07 结果，基线、专题和旧版本进入两个折叠归档；历史 benchmark
证据不删除，只去掉“最新”误标和首页内嵌旧图。

窄目录 HTTP 服务暴露出原报告的四个 `../../` 证据链接会 404。新增
`test_vacancy_shared_carrier_report_portability.py` 先复现，再由生成器把
carrier full、carrier_brd full、quick rows 和 approval 原样复制到报告
`evidence/` 下。四个 URL 均为 HTTP 200，副本 SHA 与源文件一致。最终
GPT-5.6 Sol/high reviewer
`01a07a50-1bbf-73e3-aea9-00ae8647ea49` 在独立复跑 197/197 后
`APPROVE`；`kill-ai-slop` scanner 为 0 命中。
