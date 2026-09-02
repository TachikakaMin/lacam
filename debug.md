# Carrier-LaCAM 调试与回归契约

状态：2026-09-02，与 `design_final.md` **v4.1（Objective-PIBT，
第四轮 review + 独立裁决修订版）**对齐。

实现基线：`lacam/src` = **v3.0 final**（task 池 + committed ready +
custody + delta 执行价 + R1-R7/S1-S3；C++ 161、Python 75 全绿；
head gate = `results_v3_round3_final`，严格 10s，34/68）。条目分
**[基线]**（当前已验证行为，任何改动必须维持）与 **[v4.1]**
（Objective-PIBT 落地时立即生效）。治理记录：**36/68 -> 34/68 验收
线变更已获追溯性独立批准；今后 gate 期望变更必须事前独立审查。**

## 1. 生产路径

```text
solve_carrier_lacam
  -> pass1: solve -> repair（同 deadline） -> [多目标] 固化终态 -> pass2
  -> SOC 低者；pass 完不成搜索+修补 = 诚实超时 -> apply_ops 整体重放
```

v4.1 guidance 数据流（[现状] 见 design_final 各节现状段）：

```text
X -> tau_LB（h，独立）
  -> tau0 = 全局可行匈牙利匹配（目标维度）
  -> Phase T：全体目标登记 tentative 套餐（上节点选择 / 默认套餐）
  -> Phase R：按有效优先级 Resolve（推挤/继承/FAIL/裁决 yield；
       目标格变更折算压力 -> tau0' 匹配修复，不走 claims）
  -> 选中套餐 => tau_guide / 完整链任务池（双键合并）/ 软走廊
  -> rho (task_priority 降序, 链深升序) -> Carrier-PIBT（类内吃
       task_priority）-> 约束树 -> apply_ops
```

不得重新引入：第二套 loop / 平行 planner / 策略开关 / 未重放的后
处理 / 模式检测切换；**[v4.1]** 整路径硬预约；把协商失败当可行性
判定；**对旧 guidance 管道的调用**（预算耗尽 = 沿用同一编译器的
tentative/默认套餐，禁止 legacy fallback——独立裁决 6b）。

## 2. 允许的环境输入

`DD_ALPHA..DD_DELTA`（唯一共享 parser：strtod 全消耗、有限、非负；
工具经 `dd_load_soc_weights`）+ `DD_DEBUG_DUMP`。
`TestAblationContract` 钉集合。v4.1 协商常量
（`OBJ_PIBT_DEPTH=4 / OBJ_RESELECT_CAP=16`）为结构常量，禁开关化。
no-following / strict cache 仅显式测试参数。

## 3. 关键不变量

物理与搜索 **[基线]**：

1. `apply_ops` 终裁；guidance 不决定合法性。
2. Python validator 重放每个 success。
3. 一切 guidance 只改序：不进 `SearchKey`、不改终点、不删后继。
4. `constraint_order` 创建即冻结；futile-lift 只降序。
5. macro 仅每 pass 首解前；rollout probe 每步清 scratch。
6. 零 shelf 逐位退化；singleton 自然退化。

修补 **[基线]**：

7. 只返原计划或严格更短且 SOC 不增的有效 plan；计入 pass
   deadline，超时返 raw、pass 诚实超时；最终整体重放兜底。
8. projection cut 端点全 grounded；桥终点 labeled 配置一致。

两遍与分配 **[基线]**：

9. 两遍共享 10s；pass2 goal 取自 pass1 合法终态且在原 eligible
   set；失败/持平返 pass1。
10. settled 锁可扩展否则两级 fallback；carried 保在途 goal。
11. completed target 是普通可逆 blocker。
12. 空计划仅真实到期计 timeout；耗尽/生成失败计 failed。

task 层 **[基线]**（v3.0 已落地）：

13. **物理身份 = (shelf, from)**：落点细化（advisory→committed）与
    roots 增减不改身份；粘滞/custody 按它连续。
14. labeled 完成 = 纯物理谓词；匿名 = cell 等价类 + custody
    episode；custody 覆盖两类载运且身份匹配。
15. committed 落点执行期 soft commitment（自身站格计可落、失效才
    重算）；advisory 由 carrier 每节点选。
16. one-empty ready 编译期落点 = vacancy，执行必须 Drop 于该格。
17. `rewrite()` 任何 g-松弛节点标 `guidance_stale`（父指针变否
    皆然），下次扩展惰性重锚。
18. `tau_LB` 是 h 唯一来源；价/干涉/优先级/粘滞/taboo 永不进 h。

objective 层 **[v4.1]**（编号接续）：

19. `base_priority[b]` 每 pass 根节点按 lb 冻结；有效优先级每节点
    现算 = base + aging 插队（临时，不落盘）。
20. 对称规则：与更高有效优先级的**已提交**认领冲突时自适应；推挤
    只向下；被推挤方替代耗尽必须**返回 FAIL**（不得自行 yield）；
    yield 由裁决器在双方耗尽后判给低有效优先级方（确定性）。
21. 被搬货架 ≠ 优先级拥有者；`task_priority = max over roots` 的
    有效优先级；**custody 只存 roots（DemandKey），不存优先级
    数值**。
22. 合并按 (shelf, from) 桶：roots 并集、priority=max；
    committed 覆盖 advisory；**异落点 committed = 真冲突**；
    **in-flight committed 豁免协商**（任何优先级不得重谈执行中的
    承诺落点）。
23. 硬认领仅 {from, committed 落点}；advisory 落点与目标格不进
    账本（目标唯一性由 tau0 匹配保证）；走廊 = 软干涉分；禁整路径
    预约。
24. 目标格变更只走匹配修复通道（压力 -> tau0' 重解）；协商不得绕
    过匹配直接认领目标格。
25. Phase T 先建满账本（tentative 来源 = 上节点选择 / 默认套餐）；
    默认套餐 = 零协商输出 ≡ v3.0 行为，为同一编译器产物。
26. 协商深度/次数超限 = 未决者沿用 tentative（默认解析，计
    `obj_default_resolutions`），不判后继不存在、不调旧管道。
27. 选中套餐发射**完整清障链**（≤CLEAR_CHAIN_K），非单任务（池宽
    = 并行度上限，e8 教训）。
28. goal 与前线链是同一节点联合产出（tau0'+套餐），tau_guide 是
    视图。
29. task_priority 贯通 rho 与 Carrier-PIBT 类内序。
30. per-target `no_progress[b]`：progress = best_lb[b] 下降（父链
    单调维护）或 root=b 任务完成；progress 或 tau0' 改选即清零；
    达 24 插队一轮即失效。

## 4. 输出修补调试 [基线，不变]

`DDPlanRepairStats`：exact/projected loops、bridge_steps、
steps_removed。未生效依次查：raw 可重放到 goal？端点 grounded？
桥严格更短？桥 SOC 未超段？1/2 机器人最短桥？多机器人投影合法？
deadline 中止？最终 replay fallback？**禁止跳过最终 replay。**

## 5. 结构常量

```text
—— 现有（调整须重跑 68 例 gate）——
MACRO_CAP 64 / MACRO_TARGET_LIMIT 64 / GUIDANCE_REFRESH_STEPS 8
ASSIGNMENT_EXACT_LIMIT 256 / ACTIVE_TARGET_{LIMIT 256, CAP 64}
ASSIGNMENT_HYSTERESIS 2（tau/rho/option 共用）
HEAD_DROP_SCAN_CAP 64 / CLEAR_CHAIN_K 3 / LAMBDA_BLK 8
LIVELOCK_WINDOW 24（revisit 重导 8）
FUTILE_REPEAT_LIMIT 3 / 窗口 max(64, 8·|V|)
repair EXPIRY_STRIDE 256 / cleanup reserve min(1s, max(0.1s, 10%))
—— v4.1 新增（RED 时定死）——
OBJ_PIBT_DEPTH 4 / OBJ_RESELECT_CAP 16
（OBJ_GOAL_CANDIDATES 已废：目标维度走 tau0 匹配，无候选截断）
```

锚值（head = `results_v3_round3_final`，严格 10s）：

```text
all 34/68 | small 34/36
vs v2.2 基准 (common-34)  mk 0.925567 / soc 0.934095（18/7/9）
h4w10_e1_R1_s0 = 880 | h6w10_e1_R1_s0 = 933
已知丢失：h10w10_e3_R1_seed1（贴线）、h10w10_e8_R1_seed0（S3 池
收缩，二分 3/3）——v4.1 若恢复如实报升
```

## 6. guidance / objective 层调试

诊断列（binary -> rows.csv，管道已建）：

```text
tau_price_repairs / rewire_guidance_rebuilds
tau_time_ms / guidance_time_ms（大图警戒 ~2.5s）
—— v4.1 新增 ——
obj_default_resolutions（默认解析占比 = 落地成色试金石）
obj_reselect_requests / obj_inherit_depth_max / obj_backtracks
obj_yields / tasks_merged / deliverable_ms（成功行 ≤ 10000 机检）
```

症状检查单：

1. 默认解析占比高 -> 协商未生效或预算不足（验收 §11.4(5)）。
2. 跨节点套餐/目标翻烙饼 -> 查 pass 冻结与 option 粘滞（step-3
   震荡教训 mk 17782）。
3. 低优 no_progress[b] 持续增长 -> 查 aging 插队与清零条件。
4. guidance_time 逼近预算 -> 查深度/次数上限与 PathCache 命中。
5. 任务池 < 空闲机器人且 mk 恶化 -> 池宽/并行度（不变量 27，e8
   机制）。
6. 纯性测试失败 -> 协商信号漏进 tau_LB，立即回退。
7. custody 载运不落（S1 型）-> 查 committed 有效性刷新与 funcPIBT
   custody 分支。
8. deliverable_ms 超限 -> 可交付物晚产，查修补/选择耗时归因。

## 7. 测试

```sh
cmake --build build -j 16 && ./build/test_all --gtest_color=no
PYTHONPATH=benchmark /tmp/dd-lacam-pytest/bin/python -m pytest \
  benchmark/tests -q -n 14 --dist=load
```

### 7.1 现有保护测试（不得回归；修改须独立审查 APPROVE——先例：
corridor fixture、depth fixture 枚举替换两例）

`test_tapf_compat`（零 shelf 逐位）；`dd_g1_conformance.*` /
`dd_golden`（穷举与 oracle 一致）；`dd_plan_repair.*`（含 SOC 契约
与 deadline 中止）；`dd_tau.*` / `dd_tau_guidance.*`（匹配、h、
taboo、settled、dst cache）；`dd_tasks.*` 15 个（池投影/身份/rho、
链编译、one-empty 编译+执行、价触发、h 纯性、custody 生命周期、
labeled ready、depth 次序、池去重）；`dd_rewire.*`（记账、权重校
验、全链 stale 含同父松弛）；`dd_planner.exhausted_*`（失败分类）；
`dd_goalset.*`；`dd_anytime.*`；`dd_integration.*`（scratch 协议、
dense 不劣于 B0）；Python 全套（validator/goalset/metrics/
theorem1/prop2/golden/CLI/AblationContract/V3Diagnostics/
Auditability/DuplicateTargetStart）。

### 7.2 v4.1 必测（design_final §12.1 全 18 条，RED 先行，措辞已按
裁决定稿——创建即受保护）

1 零shelf；2 singleton 退化；3 修复通道改选目标（经压力+tau0'，
非 claims 直抢）；4 对称规则；5 继承链两级（Phase T 账本可见）；
6 B 替代耗尽返 FAIL -> A 换套餐 -> 双耗尽裁决 yield；7 (shelf,from)
桶合并全规则；8 in-flight committed 豁免；9 one-empty 协议≡现行
ready；10 纯性同型扩展；11 防震荡；12 aging 插队与清零；13 默认
解析≡零协商同管道；14 池宽度=完整链；15 advisory→committed 身份
连续；16 deliverable_ms 机检；17 custody/committed 生命周期
（既有）；18 全计划重放（既有）。
待写测试与机制同 CR 交付，禁止先合机制后补测试。

## 8. Benchmark gate

```sh
python3 benchmark/run_benchmark.py \
  --instances benchmark/instances_brap_pool \
  --out-dir benchmark/results_<新目录> \
  --methods carrier --timeout 10 --jobs 14
```

新目录（防覆盖守卫，覆盖须 `--force`）；timing.json 带 provenance；
成功行带 plan_sha256；v4.1 起成功行断言 `deliverable_ms <= 10000`。

### 8.1 行为不变改动（对照 `results_v3_round3_final`）

success 34/68、小图 34/36；核心四字段逐行零差异；成功 plan sha 逐
字节一致；§5 锚值单列核对。

### 8.2 语义变更（v4.1 落地验收，= design_final §11.4）

success ≥34/68 且小图 ≥34/36（恢复丢例如实报升单列）；共同成功集
几何比 + 三分 + 恶化逐例；零 shelf 逐位（singleton 字节稳定不作
要求）；消融 A（强制默认解析 vs 协商）/ B（无继承 vs 完整链）/
C（合并 vs 丢 root）；`obj_default_resolutions` 与
`guidance_time_ms` 专项；deliverable_ms 全行断言。
**gate 期望的任何变更（含验收线）须事前独立审查**（36->34 已获
追溯批准，见 §10）。

## 9. 已知边界

20×20+ 首解 horizon 0/32（非协商所解）；one-empty 物理串行化
（深链补发/预测站位为后续项）；>256 row-wise regime 无 gate 覆盖；
多机器人桥仅合法缩短（原型 719 vs 1053 宽松口径）；遗留质量项：
e8 并行度节流、b4 目标往返、pass2 析构包络（deliverable_ms 机检
化）——v4.1 验收专项检查前两项；协商成本 = 新常数项（OBJ_* 硬限 +
预算列 + 默认解析安全阀）。

## 10. 治理与已闭合契约索引

- **验收线 36/68 -> 34/68**：R1（严格 10s 收紧）与 S3（设计正确的
  池去重，丢例二分 3/3 归因并披露）所致；**2026-09-02 获独立裁决
  追溯 APPROVE**，同时确认流程缺口（未事前审查）——今后 gate 期望
  变更一律事前独立审查。
- 第二轮 R1-R7：commits `e8a751d..a266e80`；第三轮 S1-S3：
  `4d51c11`（depth fixture 经 REJECT->枚举验证->APPROVE 两轮替换）；
  第四轮设计缺陷 6+6 条与本次 v4.1 修订：见 design_final 头部与
  本文件对应 commit。
- 各契约验收数字、二分归因、逐例披露见 commit message 与
  `results_*`（rows.csv + timing.json provenance）。
