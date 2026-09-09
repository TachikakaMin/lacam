# 空位感知清障在非 baseline 方法（carrier）上的改进方案

状态：2026-09-06 计划稿。回答的问题：报告
`carrier_lacam_vacancy_aware_clearance_report_20260906.md` 里的空位感知
改进是在 `carrier_brd` 上分析与验证的，`carrier`（非 baseline 生产方法）
怎么写这个改进？两个方法在这个场景下是不是共用的？

**结论：是共用的，而且不需要为 carrier 复制任何算法代码。** 改进落在
两个方法共同调用的 `carrier_detail` 编译层（`lacam/src/carrier_guidance.hpp`），
carrier 通过它的引导构建入口自动获得同一套空位势能排序与 PairCost。
当前工作树上 carrier 已经实际受益（quick 77 公共解案例总 makespan
-17.3%、总 weighted soc -18.0%，报告案例 makespan 102→49）。剩下的工作
不是"再写一遍改进"，而是 carrier 专属的验证、telemetry 补齐和 8 个
回退案例的诊断。

---

## 1. 方法版图（谁是 baseline，谁不是）

| method | 求解入口 | 角色 |
|---|---|---|
| `carrier` | `solve_carrier_lacam_result()` → `TAPFPlanner`（单搜索 LaCAM + Task-BR-PIBT 引导 + 两趟 anytime） | **非 baseline 生产方法**（new.md v4 + rho v2） |
| `carrier_brd` | `solve_carrier_brd_result()` → tau 匹配 + BR upper 搜索 + frozen task 分派 | 分解基线（plan.md"Carrier BR-LaCAM 分解基线"；报告在它上面做的分析） |
| `carrier_b0` / `carrier_b1` | rollout / 冻结匹配两阶段 | 弱化 baseline |
| `b4` / `crest_*` / `natcbs` | 外部/独立 baseline | 与本改进无关 |

quick/full 清单的固定 method slot 是 `carrier`
（`release_benchmark.json` protocol `methods: ["carrier"]`）；
`--methods carrier_brd` 只是允许在该 slot 里替换运行分解基线
（`benchmark/run_benchmark.py::resolve_suite_methods`）。

## 2. 为什么是共用的（调用链证据）

改进的全部算法内容都在共享编译层，两个方法只是"入口不同"：

```text
共享实现（lacam/src/carrier_guidance.hpp）
  ClearanceCost / StorageTransferTopology(1335) / build_storage_transfer_topology(1813)
  build_vacancy_potential(2021/2039)  VacancyPotentialCache(2076)
  ordered_shelf_candidate_window(2233/2452)   ← 排序用 clearance Q 项
  compile_task_br_pibt(3274/3582)             ← 入口内部自建/复用势能(3306-3319)
  compile_single_root_next_ready_effect(…)    ← PairCost rollout 每步自建势能(3104-3115)
  build_lazy_pair_cost_assignment / pair_cost ← tau 分配同样吃到新排序

carrier 入口（非 baseline）
  TAPFPlanner::attach_carrier_guidance()  tapf_planner.cpp:555
    → build_task_br_guidance(…, CarrierEngine::storage_topology, …)  :650
  CarrierEngine 持有 storage_topology（tapf_planner.cpp:501/508，每实例建一次）
  节点扩展与 carrier_rollout 都走这同一个 attach 点（:919/:937/:968、rollout 2718+）

carrier_brd 入口（分解基线）
  solve_carrier_brd_result()  dd_planner.cpp:1345+
    → build_storage_transfer_topology + VacancyPotentialCache（tau 阶段）
  CarrierBRDomain 持有 storage_topology（br_lacam_upper.cpp:415/428），
  upper 展开把势能传进共享 compile（:500/:517）
```

顺带说明：`b0`（rollout 内部同样走 `attach_carrier_guidance`）和
`b1`（冻结的 root PairCost 匹配调用同一套 PairCost helper，
dd_planner.cpp:1790+ 也建 topology）也自动继承新语义。这符合它们
"生产机制的弱化版"的定义，但意味着**旧的 b0/b1 数据与新树不可比，
对照表必须同树重跑**。

## 3. carrier 已经受益的证据（当前工作树，10s 协议）

focused case `brap_h10w10_a1_e1_B_seed0_pool`（报告案例，单空位）：

| 指标 | carrier @ rho-v2 基线 | carrier @ vacancy 树 |
|---|---:|---:|
| makespan | 102 | **49** |
| weighted soc | 214 | **101** |
| 求解耗时 | 9059.75 ms | 31.9 ms |

（单独复跑一次同样得到 mk 49 / soc 101 / 首解 17ms，确认非 runner 偶然。）

quick 77（`benchmark/results_quick_vacancy_phase_a_carrier_20260906`，
对照 `results_quick_rho_v2_20260906`）：

```text
solved 47/77 -> 47/77（无新增无丢失）
公共 47 例：sum makespan 18290 -> 15133（-17.3%）
             sum weighted soc 47157 -> 38673（-18.0%）
soc 恶化 >5% 的案例：8 个（见 §5）
```

参考：carrier_brd 同一改进的独立验证是 43/77 → 51/77，报告案例
上层 transfer 31 → 15（抽象最优）。两个方法都从同一份实现受益，
用户的"共用"假设成立。

## 4. "怎么写这个改进"——carrier 需要与不需要的部分

**不需要做的：**

- 不复制算法。势能、排序、PairCost 已在共享层，carrier 的
  `attach_carrier_guidance` 已接线（§2）。rules.md 禁止平行实现，
  这里也没有必要。
- 不给 carrier 移植 BR upper 的 Phase C cost search。那是 brd 分解
  架构（先上层后下层）的专属阶段；carrier 的对应物是它已有的
  两趟 anytime（`FIRST_STRICT_IMPROVEMENT` 第二趟 + 参考计划拼接），
  改进入口天然共享（引导变好 → 首解与改进趟都变好）。

**需要做的（carrier 专属）：**

1. **8 个回退案例诊断（本方案核心工作，见 §5）。**
2. **telemetry 补齐**：vacancy 字段目前只在 `CarrierBRDStats`
   （dd_planner.hpp:111-120，`brd_vacancy_potential_*`、
   `guidance_version=TASKBR_VACANCY_V1`），lacam 模式不导出。
   在 `TAPFStats`/`DDStats` 增加同名计数并在 `tools/dd_benchmark.cpp`
   的通用段打印 + `run_benchmark.py` 收列，让 carrier 的 quick 行
   也能看势能构建次数/耗时/回退数。
3. **carrier 入口回归测试**：现有 vacancy 测试大多打在共享层与 brd
   入口（`test_dd_vacancy_potential.cpp` 用的就是报告案例 YAML）；
   补一个走 `solve_carrier_lacam_result`（或 `dd_task_br_guidance_probe`）
   的断言：报告案例 10s 内 soc ≤ 基线、引导首选候选为 `(5,2)` 方向。
   `test_dd_task_br_two_vacancy_regression.cpp` 已覆盖 TAPFPlanner 路径，
   沿它的写法扩。
4. **benchmark 纪律**：carrier 是 quick/full 协议的默认 method slot，
   Phase A 收尾时 carrier 与 carrier_brd 各自出 quick 77 结果目录；
   full 509 按 rules.md 需独立 APPROVE 后一次性跑。b1 若进对照表，
   必须同树重跑（§2 语义漂移）。

## 5. 回退案例与诊断计划

carrier 上 soc 恶化 >5% 的 8 例（quick，成功位不变）：

| case | mk 基线→新 | soc 基线→新 | brd 同案例（改进前→后） |
|---|---|---|---|
| brap_h10w10_a12_e3_B_seed0_pool | 155→167 | 337→362 | 207/345→191/301（改善） |
| brap_h10w10_a12_e3_B_seed1_pool | 780→1512 | 1738→3286 | 711/1240→500/921（改善） |
| brap_h4w10_a5_e10_B_seed1_pool | 19→34 | 31→65 | 25/32→42/56（同退） |
| brap_h4w10_a5_e10_R1_seed1 | 58→62 | 118→130 | 59/102→59/102（不变） |
| brap_h4w10_a5_e1_R1_seed0 | 448→511 | 879→965 | 失败→383/436（新解） |
| brap_h4w10_a5_e1_R1_seed1 | 367→389 | 724→769 | 194/225→194/225（不变） |
| warehouse_blocks_h20w20_b3_a1_d50_r8_t12_seed0 | 26→28 | 157→179 | 26/157→26/157（不变） |
| warehouse_blocks_h20w20_b9_a1_d25_r8_t12_seed0 | 23→23 | 148→161 | 27/144→26/159（同退） |

关键观察：最大回退 `a12_e3_B_seed1`（mk 780→1512）在 brd 上反而
显著改善，且 carrier 的恶化发生在**首解**（first_solution makespan
942→2713，improvement 趟两版都 SEARCH_CUTOFF 没有改进成功）。
即问题不在共享排序本身，而在多 target（a12/a5）场景下空位引导与
carrier 的联合编译 / rho 匹配 / LaCAM 搜索次序的交互。

诊断步骤（按序，先证据后改码）：

1. 对 `a12_e3_B_seed1` 用 `dd_compile_joint_graph_probe` 对比新旧树的
   联合任务图（root 顺序、每 root 首步、paused roots、backtracks）。
2. 检查多 root 势能的 settled-target mask / tau 变体
   （`build_vacancy_potential(ins, upper, topology, tau, …)`）在多空位
   竞争时是否把多个 root 的清障链引向同一个空位（wave 串行化）。
3. 若确认是"多 root 抢同一空位"：候选打分对"本 root 以外正在使用的
   势能链"加软惩罚（仍在共享层，一处改动两法共享），或恢复
   debug.md 勘误 3 的顺序实验（interference 与 Q 的位置）单独 A/B。
4. 每步都按 rules.md：先写复现回退的 regression test，再动实现；
   h4w10 窄图两例与 warehouse b9 纳入 focused set。

## 6. 验收标准

- carrier：quick 77 solved ≥ 47，公共案例总 makespan/soc 不高于本次
  vacancy 树（15133/38673），§5 表中恶化 >5% 的案例数收敛到 ≤2 且无
  单案例 soc 翻倍；报告案例保持 mk≤49/soc≤101 水平。
- carrier_brd：维持 51/77 与报告案例 15 transfer 的既有门槛不回退。
- 共享层测试全绿（当前 `test_dd_vacancy_potential` 7/7、
  `test_dd_task_br_two_vacancy_regression` 1/1 已 PASS）+ 新增 carrier
  入口回归测试。
- telemetry：carrier 行可见 vacancy 构建计数与 guidance version。
- full 509 只在独立 APPROVE 后运行，carrier 与 carrier_brd 同树同协议。
