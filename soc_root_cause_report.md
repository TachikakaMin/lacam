# SOC 退化根因分析报告（v4.1 vs 严格 v3 baseline）

状态：**已通过独立审核**（2026-09-02，三轮，最终 APPROVE）。
审核记录（可审计）：

- 审核形式：独立 subagent 审核会话（本项目治理规定的裁决者角色，
  与作者会话隔离），对照原始 artifacts 亲自复算。
- Round 1（2026-09-02）：NOT_APPROVED。审核者独立复现全部核心技术
  主张（E4 对毒性行的 v3 逐位恢复含 plan SHA、E1-E10 单机制隔离、
  旋转追踪计数逐项吻合、d50 122/145/1012、几何比 1.150348 /
  0.986541 / 0.973610、192/194 测试、替换 fixture 4/4/0），但列出
  4 项披露缺陷（e10/e15/e20 恢复表述失实、最差行表格遗漏 rank-2、
  debug.md 三分计数笔误未标注、残余恶化 regime 归类过宽）。
- Round 2（2026-09-02）：4 项缺陷确认修复（审核者按 YAML 重算
  regime 归属、按 rows.csv 重算 11 行残余拆分），新发现 1 项：
  §2.1 宣称 19 行逐行披露但只覆盖 16 行。NOT_APPROVED。
- Round 3（2026-09-02）：19 行并集经程序化核对与披露表一一对应
  （18 行确定性 SOC + 1 行当日环境不可复现单独补记），锚例
  E25=701 复跑吻合，判 **APPROVE**，"New issues: none found"。
- 审查对象：本文件 + `/tmp/socdbg/results_*`（rows.csv）+
  `/tmp/dd-lacam-exp` 实验树 + benchmark/results_v3_strict_return_final
  与 results_v4_1_final7。

原状态行（历史）：2026-09-02，供独立终审审核。作者：调查会话
（experiment tree `/tmp/dd-lacam-exp`，结果目录 `/tmp/socdbg/`）。

## 0. 问题

`results_v4_1_final7` 相对 `results_v3_strict_return_final` 在共同成功
集（32 个非平凡行）上 SOC 几何比 **1.1503**（14 改善 / 0 持平 / 18
恶化），最差 3.82x（`h4w10_a5_e10_R1_seed0`：539→2059）。debug.md
§8.3 的归因列只有机制级猜测（"ordering/custody continuity"、"pool
联合编译"），"未声称逐例存在更强的单一根因"。本次任务：找到真根因。

（勘误：debug.md §8.3 / design_final.md §11.5 记载的 SOC 三分
"14/1/18" 与"剔除 trivial 行后 32 例"不自洽——14+1+18=33。原始数据
为：剔除双方均 0 的 trivial 行后 32 例 **14/0/18**；若含该 trivial
持平行则 33 例 14/1/18。本报告使用 32 例 14/0/18 口径，主文档该处
数字应随本次修订一并更正。）

## 1. 排除法证据链

### 1.1 协商（Phase R）不是根因

按 SOC 比排序的**实际前五**回归行及其协商诊断计数（final7
rows.csv）：

```
rank instance                 default reselect yields merged   SOC v3->v4.1
1    h4w10_a5_e10_R1_s0             0        0      0    374   539->2059 (3.82x)
2    h4w10_a5_e1_R1_s1              0    10701  15067  12181   341-> 908 (2.66x)
3    h10w10_a12_e3_B_s0_pool        0        0      0     86   510->1111 (2.18x)
4    h8w10_a10_e20_R1_s0            0        0      0    273  1169->2456 (2.10x)
5    h6w10_a6_e15_R1_s0             0        0      0    244   758->1562 (2.06x)
```

前四中 3 个（rank 1/3/4）以及 rank 5 的协商计数全零：Phase T/R、
推挤、yield、默认解析都没发生。rank 2 是协商活跃行（reselect
10701 / yields 15067），说明该行还有协商侧贡献者，但下方 E4 隔离
显示其主导项与其余行相同。消融 A（强制默认解析）在这些行上同样
退化（A vs v3 几何比 1.1792），进一步排除协商本身作为主导根因。

### 1.2 单机制隔离（每个 always-on 变更一个编译开关）

在 `/tmp/dd-lacam-exp`（工作区快照）为每个 v4.1 always-on 机制建
互斥反转开关，逐一构建二进制并重跑最差行（全部 seed 0 确定性）：

```
E1  cls() committed-custody 提升 class 0     -> 无影响 (2059)
E2  livelock 重排 rem tie-break              -> 无影响 (2059)
E3  funcPIBT parked+custody 分支             -> 无影响 (2059)
E5  custody 跨 reguide 连续                  -> 次要 (1702)
E6  merge unique_rank 重算                   -> 无影响 (2059)
E7  链宽 already_global 记账                 -> 无影响 (2059)
E10 alternatives 生成                        -> 无影响 (2059)
E4  移除 rho 保留槽旋转                      -> 539 = v3 逐位恢复
```

E4 在 `h4w10_a5_e10_R1_s0` 上恢复出与 v3 **完全一致**的轨迹
（SOC 539、hl_nodes 2413、guidance_builds 4896 全部相等），并在
e8/e15/e20 行普遍恢复 v3 水平。E4 全量 68 例 vs v3 共同集 SOC 几何比
**0.9865**（13/9/8，9 行逐位相同）。

**结论：不变量 29 的 rho 保留槽旋转是共同集 SOC 退化的主导根因。**
（h6_e1/h4_e1 等协商活跃行还有次要贡献者，但主导项相同。）

### 1.3 旋转动力学（插桩计数）

对保留槽加 env-gated 追踪（`DD_ROT_TRACE`），对比毒性行与受保护
稠密锚：

```
                         h4_e10_R1_s0 (毒性)    d50_16x16_r8 (需要槽)
truncated builds              5801                    102
rotation fires                1346                     60
  其中 aging-boosted           391 (29%)                23
  其中 free_left==1           1318 (98%)                16 (27%)
  winner 已在前缀中               0                      1
winner 身份切换次数             258                     12
winner=最远使命占比        932/1346 (69%)            35/60
```

机制：`freeze_base_priorities` 把**最远（lb 最大）使命**排为最高
base priority；保留槽在**每个** `任务数 > 空闲机器人` 的节点触发。
在 `free_left==1`（毒性行 98% 的触发）时，"替换前缀最后一槽" =
**替换整个 assignment 前缀**，即把 v3 的 nearest-first 贪心（SPT
式，SOC 友好）整体反转为 farthest-first（LPT 式）。叠加 aging
插队（24 节点 lb 无改善即触发，长途搬运期间必然满足）造成 winner
身份 258 次翻转，打断 Hungarian/任务粘滞（eta=2 只对同一任务打折），
机器人反复改派 → loaded/free moves 各膨胀 2-4x。

### 1.4 但纯移除不可行（E4 的代价）

- 受保护稠密锚 `d50_16x16_r8_seed0`：search mk 122→**1012**
  （B0 143，违反 search ≤ 2×B0），复现设计文档 §7.3 记载的
  "完全移除 rho 优先级使 raw search 膨胀"。
- 194 项 C++ 套件挂 5 项（两个 slot 语义 fixture、d50 稠密、两个
  贴线 deliverable 回归）。
- 今日同协议重跑下 E4 与 base 成功数同为 33-34（final7 的 36 中
  3 个贴线重例在本机当前负载下对**两个二进制都**不可复现——base
  binary 本身今日 33-34/68，`h8_e2_R1_s0` 独立 3 连跑均 ~10.05s
  失败；即成功数差异属于 10s 线上的环境噪声，不能归功/归罪于槽）。

**即：SOC 税与稠密完成能力来自同一机制，是 regime 依赖的权衡。**

## 2. 修复提案 E25（regime-aware 保留槽）

对 `build_guidance()` rho 段的保留槽仅加两个门（其余语义不动，
不新增环境开关，纯结构常量逻辑）：

```
free_left >= 2                       -> 照旧旋转（只占最后一槽，
                                        贪心前缀头部保留）
free_left == 1                       -> 仅当 2*structural_vacancies
                                        < n_targets（结构性 one-empty
                                        串行化 regime）才允许旋转
structural_vacancies = 非墙格数 - 货架总数   （实例静态量，非动态）
```

依据：
- d50 追踪显示其 73% 触发在 free≥2，E18（仅 free≥2）已使 d50
  恢复 145 vs B0 144 ✓ —— free=1 旋转对 d50 非必需。
- `h10w10_e3`（2 机器人 / 3 空位 / 12 目标）等重例**需要** free=1
  旋转（E18/E22/E23 全部丢失该行成功）；其结构特征 = 空位极少而
  使命多（2*3 < 12 ✓）。
- 毒性行 `h4_e10`（10 空位 / 5 目标）2*10 ≥ 5 → free=1 旋转禁用。
- 动态空位数不可用作信号（E22 教训：机器人抬起货架时空位瞬时增加，
  恰好在高载运状态下错误关闭阀门）。

### 2.1 E25 实测（同协议 68 例、seed 0、严格 10s、jobs14）

```
                     成功        vs v3 共同集 SOC        vs v3 共同集 mk
v4.1 final7          36*        1.1503 (14/0/18)        1.1594 (13/1/19)
E25                  34         0.9736 (18/1/11)        0.9688 (18/2/11)
base(今日重跑)       33-34      1.1619                  —
E25 vs final7 直接对比：SOC 几何比 0.8512（11 改善/19 持平/3 恶化）
```

\* final7 的 36 在今日环境不可复现（同 binary 33-34），去噪后 E25
与 base 成功集并集完全相同（都是 34，无单侧丢例）；E25 保住了
final7 相对 v3 恢复的全部 3 例（`h10_e3_B_s1`、`h10_e3_R1_s1`、
`h6_e1_B_s0`）。两次重跑 E25 SOC 全行逐位一致（确定性）。

19 个披露恶化行在 E25 下的逐行结果（相对 v3；18 行有 E25 确定性
SOC 值，1 行（`h8_e2_R1_s0`）今日环境不可复现、单独补记，无汇总性
修饰）：

- 4 个 e10/e15/e20 R1 行中 **2 行反超 v3**（758→1562→**701**、
  1681→2544→**1038**），**2 行部分恢复但仍差于 v3**
  （539→2059→**1073** = 1.99x、1169→2456→**1315** = 1.12x）；
- e8 行全面反超 v3（2109→**1716**、1668→**1526**、4493→**4247**）；
- 其余 3 个披露行补记：`h4_e10_B_s1_pool` **35**（v3 75，反超）、
  `h4_e10_R1_s1` **362**（= v3 362，f7 曾 462）、
  `h8_e2_R1_s0`（serialized regime）**今日四次重跑全部超时**
  （E25_r1/r2 与 base_r1/r2 均 FAIL；v3 记录 3709@8.8s、final7
  记录 4610——该行本属 10s 贴线例，今日环境对未修改 base 同样
  不可复现，故无从给出 E25 SOC）；
- E25 vs v3 仍有 **11 个恶化行**，按 regime 拆分：
  - serialized regime（阀门有意保留 free=1 旋转）5 行：
    `h4_e1_R1_s1` 2.66x、`h10_e3_B_s0` 2.18x、`h6_e1_R1_s1` 1.78x、
    `h6_e1_R1_s0` 1.48x、`h8_e2_B_s0` 1.11x；
  - **非** serialized（松散）regime 6 行：`h4_e10_R1_s0` 1.99x、
    `h6_e15_B_s1` 1.44x、`h8_e20_B_s1` 1.27x、`h10_a1_e1_B_s0`
    1.18x、`h8_e20_R1_s0` 1.12x、`h8_e20_B_s0` 1.09x。
    松散板上 E25 仍保留 free≥2 旋转（d50 依赖），因此**不预期**
    逐行回到 v3；其中 h4_e10_R1_s0 从 3.82x 收敛到 1.99x 是部分
    恢复，其余 5 行幅度 ≤1.44x。
  （注：h4_e10_R1_s0 与 h8_e20_R1_s0 同时出现在第一组与残余组；
  各组 unique 并集 = 4 + 3 + 3 + 11 − 2 重复 = 19，与披露表一一
  对应。）

### 2.2 测试影响

E25 全量 C++ 套件 **192/194**：

1. `dd_objective_priority_integration.farther_root_owns_the_frontline_slot`
   —— 该 fixture（2x7 板、12 空位、2 目标、1 机器人）恰好钉住
   "松散板 + free=1 强占唯一行"这一被测出的毒性模式。已设计并验证
   替换 fixture（2x10 板、2 空位、5 目标 → serialized regime）：
   base ✓ / E25 ✓ / E4 RED（保留"移除阀门必须 RED"的保护力）。
   同文件其余 4 项（含 free=2 的
   `rho_reservation_replaces_only_the_last_truncated_slot`）在 E25
   下原样通过。按治理规则，此 protected fixture 替换需独立审查
   APPROVE（先例：depth fixture 两轮替换）。
2. `dd_objective_dense_repair_regression.h8_e2_seed0_compacts_before_deadline`
   —— 环境性失败：**未修改的 base binary 今日独立 3 连跑同样失败**
   （10.04-10.08s vs 记录的 8.19-8.50s），非 E25 引入。
3. 建议新增保护测试：松散板（2*vac ≥ targets）+ free=1 时贪心行
   不被抢占（在当前 base 下 RED，E25 下 GREEN —— TDD RED 先行）。

## 3. 交付物清单

- 实验树：`/tmp/dd-lacam-exp`（快照 + 全部 E* 开关，
  `build-{base,E4,E11,E12,E18,E20,E22,E23,E24,E25}/dd_benchmark`）
- 结果：`/tmp/socdbg/results_{base_r1,base_r2,E25_r1,E25_r2,E4_no_rho_slot,E11,E12,E18}/rows.csv`
- 追踪：`/tmp/socdbg/rot_trace.txt`（毒性行）、`/tmp/socdbg/d50_rot.txt`
- 替换 fixture 原型：`/tmp/socdbg/fixture_proto2.cpp`（三二进制验证输出如上）
- base 行为对照：`build-base` 与 final7 主 binary 在最差行 SOC/
  节点数逐位一致（2059/5360 nodes），final7 vs 今日 base 共同 32
  行 SOC 差异数 = 0。

## 4. 已知边界与诚实披露

- **根因表述的范围**：rho 保留槽是**本套件上经隔离验证的主导经验
  根因**（E4 单开关恢复、毒性行逐位 v3 一致、9 个非平凡行 SOC+plan
  SHA 双一致、追踪机制吻合），不是对每个恶化行的普适唯一根因：
  E5（custody 跨 reguide）有次要贡献（2059→1702），rank-2 行等
  协商活跃行还有 Phase R 侧因素。
- E25 未消除 serialized regime（e1/e2/e3）内的 5 个恶化行（最大
  `h4_e1_R1_s1` 2.66x）：free=1 旋转在该 regime 被有意保留以保住
  成功数与稠密 SOC 改善行；同时松散 regime 仍有 6 个恶化行（最大
  1.99x，为 3.82x 的部分恢复；其余 ≤1.44x），因为 free≥2 旋转在
  全 regime 保留（d50 依赖）。这是测得的 success-vs-SOC 真实权衡，
  不是遗漏。
- `2*vac < targets` 阈值拟合自现有 68 例 pool + d50/h10_e3 两个
  约束点，**无 held-out 验证**；20x20+ 全失败行不提供信号，>256
  row-wise regime 无 gate 覆盖。过拟合风险真实存在，合入前应在
  更多实例上复核阈值形态。
- 成功数在 10s 线上的重例对机器负载敏感（同 binary 33-36 波动；
  `h8_e2_R1_s0` 今日对 base 也 3/3 失败）；所有 SOC 结论基于确定
  性行（两次重跑逐位一致），成功数主张一律以并集口径限定。
- 共同成功集 SOC 以"求解成功"为条件，本身不构成端到端优劣的完整
  证明；须与成功率一并阅读。
- 本报告只交付根因 + 经验证的修复方向；是否合入 E25、替换 protected
  fixture、以及是否重跑正式 final 系列 artifact，按治理流程需事前
  独立审查。
