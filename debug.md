# Carrier-LaCAM 调试与回归契约

状态：2026-09-02，与 `design_final.md` **v4.0（Objective-PIBT）**对齐。

实现基线：当前 `lacam/src` = **v3.0 final**（task 池 + committed
ready + custody + delta 执行价 + R1-R7/S1-S3 全部修复；C++ 161、
Python 75 全绿；head gate = `results_v3_round3_final`，严格 10s，
34/68）。条目分两类：**[基线]** 钉住当前已验证行为，任何改动必须
维持；**[v4.0]** 是 Objective-PIBT 落地时立即生效的强制约束。两类
同时有效——v4.0 的实现不得以破坏 [基线] 为代价。

## 1. 生产路径

```text
solve_carrier_lacam                        (dd_planner.cpp)
  -> pass1: TAPFPlanner::solve -> repair（同一 deadline 内）
  -> 多目标且有时间: 固化终态分配 -> pass2 -> repair
  -> 取 SOC 低者（平手/失败保 pass1）；pass 内完不成搜索+修补 = 诚实超时
  -> apply_ops 整体重放
```

guidance 数据流（v4.0 目标形态；[现状] 见 design_final §4-§7）：

```text
X -> tau_LB（admissible h，独立于一切协商）
  -> 候选套餐生成（每目标 ≤2 goal 候选 × 当前路线）
  -> Objective-PIBT：认领账本 + 优先级继承 + 回退 + 合并
       （超限/失败 -> 回退 v3.0 匹配路径）
  -> 选中套餐 => tau_guide / 任务池（roots 合并）/ 软走廊
  -> rho：(task_priority 降序, 链深升序) -> 匈牙利
  -> Carrier-PIBT（类内按 task_priority）-> 约束树 -> apply_ops
```

不得重新引入：

- 第二套 search loop / 平行 planner / feature-flag fallback；
- 生产策略环境开关；
- 未经 oracle 重放的计划后处理；
- 模式检测切换（零 shelf / 零 task / 零协商必须是数据结构自然为空
  的逐位退化）；
- **[v4.0]** 整路径资源预约（claims 只许 {goal, 被操纵货架格, 当前
  落点}）——协商器不得膨胀为第二个 MAPF 规划器；
- **[v4.0]** 把协商失败当作可行性判定（只允许回退到 v3.0 路径）。

## 2. 允许的环境输入

```text
DD_ALPHA / DD_BETA / DD_GAMMA / DD_DELTA   数值 objective
DD_DEBUG_DUMP                              失败诊断
```

权重经唯一共享 parser（`load_solver_weights`，工具侧经
`dd_load_soc_weights` 复用）：strtod 全消耗、有限、非负，否则抛出
（`dd_weights.*` 两个测试钉住）。`TestAblationContract` 钉住集合。
v4.0 协商参数（`OBJ_GOAL_CANDIDATES=2 / OBJ_PIBT_DEPTH=4 /
OBJ_RESELECT_CAP=16`）是结构常量，禁止开关化。

no-following 与 strict cache 只作显式测试参数：
`apply_ops(..., allow_following=false)`、`PathCache(/*strict=*/true)`。

## 3. 关键不变量

物理与搜索层 **[基线]**：

1. `apply_ops` 是 joint transition 终裁；guidance 不决定合法性。
2. Python validator 重放每个 benchmark success。
3. 一切 guidance（option/claims/base·inherited·task 优先级/task/
   rho/path/park/cooldown）只改候选顺序：不进 `SearchKey`、不改
   goal condition、不永久删除合法 successor。
4. `constraint_order` 创建即冻结；reguide/重锚只扰动 PIBT
   preference；futile-lift 只降序不删除。
5. macro 只在每 pass 首解前（`macro_after_first` 恒 0）；rollout
   probe 每步清 address-keyed scratch。
6. 零 shelf 逐位退化；singleton goal-set 自然退化。

修补层 **[基线]**：

7. 修补只返回原计划，或**严格更短且加权 SOC 不增**的有效 goal
   plan；修补计入所属 pass 的 10s deadline，超时中止返回 raw、
   pass 报诚实超时。
8. projection cut 两端全部 grounded；bridge 终点 labeled 机器人
   configuration 与原终点一致；最终整体重放兜底。

两遍与分配层 **[基线]**：

9. 两遍共享一个 10s deadline；第二遍 goal 取自第一遍合法终态且在
   原 eligible set 内；失败/持平返回第一遍。
10. settled 锁必须可扩展成完整 matching，否则按两级 fallback 重开；
    carried 目标保留在途 goal。
11. completed target 在 path 层是普通可逆 blocker。
12. 空计划仅在某 pass 真实到期时计 `timeout`；耗尽/generator 失败
    计 `failed`。

task 层 **[基线]**（v3.0 已落地）：

13. task = `MoveShelf(s, from -> to, root)`；TaskId =
    hash(shelf, from, root)，`to` 不参与身份。
14. labeled 完成条件为纯物理谓词；匿名 shelf 按 cell 等价类 +
    custody episode 判定；custody 覆盖 labeled 与匿名两类载运，
    身份匹配（task.shelf_target == kappa）。
15. `to_committed` 承诺落点执行期保持 soft commitment（自身站格计
    可落，失效才重算真实空格）；非承诺落点由 carrier 每节点选择。
16. one-empty ready task 编译期确定落点为当前 vacancy，执行必须
    Drop 于该格（不允许 Lift 后另找停车点）。
17. `rewrite()` 中**任何被 g-松弛的节点**标 `guidance_stale`
    （父指针变不变都算——祖先链已变），下次扩展惰性重锚。
18. `tau_LB` 是 h 的唯一来源；执行价/干涉/优先级/粘滞/taboo 永不
    进入 h。

objective 层 **[v4.0]**：

19. `base_priority[b]` 属于目标货架（=使命），每 search pass 根
    节点按 lb 排序**冻结**，pass 内不变；aging 只做协商轮内的临时
    插队，不改写基础值。
20. 压力只从高优流向低优（对称规则）：与更高优认领冲突时改自己的
    套餐；低优无权请求高优让路。
21. 被搬运货架 ≠ 优先级拥有者：清障/让路任务继承请求方优先级；
    `task_priority = max over roots`。
22. 相同 (shelf, from, to) 是共享任务：合并 roots、取最大优先级，
    **不丢弃**任何 root 的服务关系；真冲突仅"同货架异落点"与
    "异货架同落点"。
23. claims 极小化（goal + 被操纵货架格 + 当前落点）；走廊冲突用软
    代价表达，禁止整路径预约。
24. 协商递归深度与重选次数受结构常量硬限；超限整体回退 v3.0 匹配
    路径（`obj_fallbacks` 计数），绝不判定后继不存在。
25. goal 与前线任务是**同一次套餐选择的两个输出**（tau_guide 是
    选中套餐的视图，不再是独立前置计算）。
26. task_priority 贯通 rho 排序与 Carrier-PIBT 类内次序；custody
    携带 roots/priority 过户。

## 4. 输出修补调试 [基线，不变]

`DDPlanRepairStats`：exact_loops / projected_loops / bridge_steps /
steps_removed。修补未生效时依次查：raw 可重放且到 goal？端点全
grounded？bridge 严格更短？bridge SOC 未超被替换段？1/2 机器人
最短桥找到？多机器人投影仍满足 R1/R2？deadline 是否中止？最终
replay 是否触发 fallback？**禁止跳过最终 replay 修命中率。**

## 5. 结构常量

```text
—— 现有（调整须重跑 68 例 gate）——
MACRO_CAP 64 / MACRO_TARGET_LIMIT 64 / GUIDANCE_REFRESH_STEPS 8
ASSIGNMENT_EXACT_LIMIT 256 / ACTIVE_TARGET_LIMIT 256 / ACTIVE_TARGET_CAP 64
ASSIGNMENT_HYSTERESIS 2（tau/rho/option 共用）
HEAD_DROP_SCAN_CAP 64 / CLEAR_CHAIN_K 3 / LAMBDA_BLK 8
LIVELOCK_WINDOW 24（revisit 重导阈值 8）
FUTILE_REPEAT_LIMIT 3 / futile 窗口 max(64, 8·|V|)
repair EXPIRY_STRIDE 256 / deadline cleanup reserve min(1s, max(0.1s, 10%))
—— v4.0 新增（RED 时定死，同规则）——
OBJ_GOAL_CANDIDATES 2 / OBJ_PIBT_DEPTH 4 / OBJ_RESELECT_CAP 16
```

锚值（当前 head，`results_v3_round3_final`）：

```text
all suite            34/68
small suite          34/36
vs v2.2 基准 (common-34)   mk 0.925567 / soc 0.934095（18/7/9）
h4w10_e1_R1_seed0    880        h6w10_e1_R1_seed0    933
已知丢失：h10w10_e3_R1_seed1（首解贴线）、h10w10_e8_R1_seed0
（S3 池收缩使首解 5.2→8.5s，二分 3/3 归因，契约披露）
```

## 6. guidance / objective 层调试

诊断（binary 输出 → rows.csv 列，管道已建）：

```text
tau_price_repairs / rewire_guidance_rebuilds
tau_time_ms / guidance_time_ms            每节点预算（大图警戒 ~2.5s）
—— v4.0 新增 ——
obj_reselect_requests / obj_inherit_depth_max
obj_backtracks / obj_fallbacks / tasks_merged
```

症状检查单：

1. **协商空转**：`obj_fallbacks` 占比高 ⇒ 协商未实际生效或预算
   不足，v4 视为未落地（验收条款 §11.4(5)）。
2. **震荡回归**：跨节点套餐/目标翻烙饼 ⇒ 查优先级是否 pass 内冻结、
   option 粘滞是否生效（step-3 绝对价震荡教训：mk 曾爆 17782）。
3. **饿死**：低优目标 no_progress 持续增长 ⇒ 查 aging 插队是否
   触发（24 步阈值）。
4. **预算超支**：guidance_time 逼近 deadline ⇒ 查候选数/递归深度
   是否越界、PathCache 命中率。
5. **并行度节流**：任务池 < 空闲机器人数且 makespan 恶化 ⇒ S3/合并
   的已知机制（e8 例），核对深链补发后续项。
6. **h 污染**：`admissible_h_never_exceeds_true_cost` 或纯性测试
   失败 ⇒ 协商信号漏进 tau_LB，立即回退。
7. **活锁类**：custody 载运不落（S1 型）⇒ 查 committed 落点有效性
   刷新与 funcPIBT 的 custody 分支。

失败诊断走 `DD_DEBUG_DUMP`；最深节点链经 best-effort 通道
（`deepest_config / deepest_tau`）。

## 7. 测试

```sh
cmake --build build -j 16 && ./build/test_all --gtest_color=no
PYTHONPATH=benchmark /tmp/dd-lacam-pytest/bin/python -m pytest \
  benchmark/tests -q -n 14 --dist=load
```

依赖钉在 `benchmark/requirements-test.txt`（无则临时 venv）；外部
solver 固定 10s 预算；14 workers 与协议一致。

### 7.1 现有保护测试（基线，不得回归；修改须独立 GPT-5.6 Sol 审查
APPROVE——历史两例：corridor fixture、depth fixture 枚举替换）

| 测试 | 责任 |
|---|---|
| `test_tapf_compat`（全部） | 零 shelf 逐位退化 |
| `dd_g1_conformance.*` / `dd_golden.corpus_agreement` | 约束树穷举与 oracle/validator 一致 |
| `dd_plan_repair.*`（含 SOC 契约、deadline 中止） | 修补全套 |
| `dd_tau.*` / `dd_tau_guidance.*` | 匹配、admissible h、taboo、settled、dst cache |
| `dd_tasks.*`（15 个） | task 池投影/TaskId/rho 绑定、frontier 编译、one-empty ready 编译面+执行面、drop hint、价触发翻转、h 纯性、custody 生命周期、labeled ready 执行、depth 次序、池去重 |
| `dd_rewire.*` | 首解合法性/记账、权重校验、rewire 全链 stale（含同父松弛） |
| `dd_planner.exhausted_search_is_not_reported_as_timeout` | 失败分类 |
| `dd_goalset.*` | loader 契约、重复 start 拒绝、二遍触发/跳过 |
| `dd_anytime.*` | h 下界性、macro 首解前 |
| `dd_integration.*` | rollout scratch 协议、dense 不劣于 B0 |
| Python：validator/goalset/metrics/theorem1/prop2/golden/CLI/
  `TestAblationContract` / `TestV3DiagnosticsExported` /
  `TestAuditability` / `DuplicateTargetStartTest` | 独立 oracle 与协议契约 |

### 7.2 v4.0 必测（design_final §12.1 全表，RED 先行）

1 零shelf逐位（既有）；2 singleton 退化；3 高优认领迫使低优改选；
4 对称规则；5 继承链两级；6 回退到次优套餐；7 共享合并
roots/priority；8 one-empty 协议≡现行 ready（典型 fixture）；
9 option/claims/优先级不进 h；10 防震荡；11 aging 插队；12 超限
回退≡v3.0 路径；13 custody/committed（既有）；14 重放（既有）。
待写测试与机制同 CR 交付，禁止先合机制后补测试。

## 8. Benchmark gate

```sh
python3 benchmark/run_benchmark.py \
  --instances benchmark/instances_brap_pool \
  --out-dir benchmark/results_<新目录> \
  --methods carrier --timeout 10 --jobs 14
```

新目录（防覆盖守卫生效，覆盖须 `--force`）；跑前确认无 `DD_*` 残留
与并行高负载。timing.json 自动带 provenance；成功行带 plan_sha256。

### 8.1 行为不变改动（对照 `results_v3_round3_final`）

- success `34/68`、小图 `34/36`；核心四字段（success/status/mk/soc）
  逐行零差异；成功 plan 逐字节一致（sha 列比对）；
- 已知边界敏感例（§5 锚值表）单列核对；
- 收尾包络 ≤ ~1.6s（pass2 耗满预算的大树析构），可交付物严格 10s 内。

### 8.2 语义变更（v4.0 Objective-PIBT 落地）

按 design_final §11.4 验收：success ≥34/68 且小图 ≥34/36（恢复
已丢失例如实报升）；共同成功集几何比 + 逐例三分 + 恶化逐例解释 +
锚例单列；零 shelf 逐位一致（singleton 字节稳定不作要求——协商
影响路径/落点维度）；消融 A/B/C（v3.0 路径强制回退 vs 协商 /
无继承 vs 完整链 / 合并 vs 丢弃）同协议报告；`obj_fallbacks` 比率
与 `guidance_time_ms` 预算专项报告。

## 9. 已知边界

- 20×20+ 大图 10s 内 0/32（首解 horizon，协商不解决）；
- one-empty 物理串行化：任务池天然小于机器人数，闲置是物理不是
  调度错（改进方向：深链补发、预测性站位——后续工作）；
- `targets > 256` row-wise 非单射 hint regime 无 gate 覆盖；
- 1/2 机器人桥最短，更多机器人仅合法缩短（Python 原型 719 vs 1053
  宽松口径，桥有余量）；
- 已知质量遗留：e8 例（池收缩/并行度）、b4 式目标往返残余、贴线例
  wall 敏感——v4.0 验收时专项检查前两项是否被协商/合并改善；
- 协商成本是新的常数项：`obj_*` 常量硬限 + 预算列监控 + 回退安全阀。

## 10. 已闭合契约索引（审计入口）

- **第二轮 review R1-R7**（严格 10s / 执行侧半环 / 全链 stale /
  parser 统一 / β 计价 / 诊断列 / provenance / 文档 CI）：
  commits `e8a751d..a266e80`；
- **第三轮 review S1-S3**（labeled ready 活锁 / 任意松弛 stale /
  池去重 + depth fixture 两轮审查替换）：commit `4d51c11`；
- 各契约的验收数字、二分归因与逐例披露原文见对应 commit message
  与 `results_*` 目录（rows.csv + timing.json provenance）。
