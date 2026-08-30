# debug.md — 第二轮独立审计返工清单

来源:2026-08-30 独立 subagent 审计(GPT-5.6 Sol, high reasoning),对照
design.md v2.2 与实现逐项核查后给出 **REJECT**。本文件**取代**第一轮
清单(2026-08-29,14 项全部完成;历史内容与完成记录见 git,
commit 98516f7 及其前后)。

第二轮审计确认:核心 validator、G1 直通、constraint_order 冻结、flags
拒载、单位权重 cost、B0/B1/dead-cell/anytime/Zobrist/M1 出口全部真实
存在且 45 C++ + 39 Python 测试全绿;REJECT 的原因集中在**理论措辞过强、
文档滞后于代码、一个 no-op 消融、一个提交遗漏、若干勾选项缺其自称的
回归测试**。

修复流程遵循项目规范:**先写 regression test 复现(RED)→ 修复(GREEN)
→ 保留测试**;protected 测试的修改需独立 subagent APPROVE。
进度标记:[ ] 未开始 / [x] 完成(附 commit 与测试名)。

---

## P0 正确性 / 证据完整性

### 1. [x] 定理 1 对多 robot 不成立(理论错误)
- **位置**:design.md §4.1(原文 "$|R|\ge 1$")。
- **问题**:审计给出穷举反例——1×2 连通图、2 robot 占满下层、shelf
  需左→右:upper sequential move 存在,但全系统仅 2 个可达状态,
  goal 不可达。定理只对 $|R|=1$ 成立。
- **处置**:design.md v2.3 已改为 $|R|=1$ 并写入反例(本轮完成);
  **遗留**:反例固化为可执行回归测试(Python validator 穷举可达集
  断言 goal 不可达 + 同图 $|R|=1$ 时 carrier 可解)。
- [ ] **遗留测试**:`benchmark/tests/test_theorem1.py`(或并入
  test_validator.py):反例穷举 + 单 robot 对照。

### 2. [ ] park 纯度:same-X/双缓存历史不变性未测,声明已降级
- **位置**:dd_planner.cpp park/hover 逻辑 + PathCache 非对称失效
  (§6.2);design.md §5.4a。
- **问题**:park 实为 (X, D_b 缓存纪元) 的函数,非纯 X 函数——同一 X
  经不同缓存历史可得不同 owner/park 集。只影响 ordering 可复现性,
  不影响可行域,但原勾选声明("纯 X 函数")过强。
- **处置**:design.md v2.3 已把声明降级并说明纪元依赖(本轮完成)。
- **待办**(二选一,先测后定):
  - [ ] 写 same-X 双缓存历史测试:同一物理配置,分别经"路径格曾被
    占后腾空"与"从未被占"两条历史到达,断言 park 集是否一致;
  - [ ] 若选择严格纯化:park 判定改用对当前 X 严格重算的 path
    (不走缓存),A/B 验证吞吐代价;若保留纪元依赖,测试固化
    "不纯但 ordering-only"的边界(DD_STRICT_INVAL=1 下必须一致)。

### 3. [x] no_astar 消融是 no-op(假对照)
- **位置**:run_ablations.py(设 `DD_NO_ASTAR=1`);生产代码从不读取
  该变量,least-blocking 路径本就是 Dijkstra(无 A* 开关)。
- **问题**:"no_astar 9/9" 不能作为消融证据;§6.2 原文误写 A*。
- **处置**(本轮完成):变体已从 run_ablations.py 移除;design.md
  §5.3(2)/§6.2 修正 Dijkstra 措辞;§6.6 记录该历史。
  results_ablation/ablation_rows.csv 中的 no_astar 行**作废不采信**
  (其余变体行不受影响,各变体独立运行)。
- [ ] **可选**:重生成 ablation_rows.csv(6 变体)以移除作废行。

### 4. [x] dd_benchmark MODE 支持未提交(复现性)
- **位置**:tools/dd_benchmark.cpp 的 b0/b1 分派与 first_solution_ms
  输出一直是未提交 diff;results_final_v2 的 B0/B1 列无法从干净
  HEAD 复现。
- **处置**:已提交(本轮,见 git log "Commit stray dd_benchmark MODE
  support");runner 传参与 CLI 现在一致。
- [ ] **遗留测试**:dd_benchmark 对未知 MODE 应报错退出(当前静默
  回落 lacam);加 CLI 冒烟断言三种 mode 输出 method 可区分。

### 5. [ ] P0-1(第一轮)自称的 revisit 回归测试从未写
- **位置**:第一轮清单第 1 项承诺"构造小实例强制走 revisit 路径,
  断言同一节点约束树枚举的 (robot,op) 集合与冻结 order 一致";
  实际只有 fresh-node 穷举(test_dd_g1),revisit 路径无覆盖。
- **修复方向**:暴露测试 hook(或经 DDStats 观察),强制同一节点
  revisits≥8 触发 re-guidance 后,断言 constraint_order 与叶子
  (robot,op) 集合不变。
- **测试**:`tests/test_dd_g1.cpp::dd_g1.revisit_preserves_enumeration`。

## P1 文档-实现一致性(本轮已大部处置)与测试补强

### 6. [x] 两阶段 anytime + macro 规模域未入文档
- **处置**:design.md v2.3 全面同步(§4.2/§4.3/§5.1/§7.1/§8.1/§10/
  D14/§12);158/162 与质量 2.5–3× 数字已落;§6.6 新增旋钮配置表
  (DD_MACRO_TGT/CAP/MIN/SPARSE、DD_GUIDE_EVERY、DD_ACTIVE_CAP、
  DD_STRICT_INVAL、DD_NO_YIELD、DD_ALPHA..DELTA、DD_DEBUG_DUMP)。
- 回归锚:`test_dd_anytime::macro_disabled_after_first_solution`。

### 7. [x] eventually-optimal / D11 / guidance-h MUST 措辞过强
- **处置**:design.md v2.3——§4.3 撤回 LaCAM* 套用声明(实现无
  g-relax/rewire,如实记录屏蔽效应);D11 重写(冻结的是
  constraint_order,guidance 可在 livelock 信号下重建);§5.7
  guidance-h 按实现描述(approach 项因 O(B×R) 吞吐被移除,plateau
  由信号兜住);§5.3(4) η hysteresis 标注未实现。

### 8. [ ] 跨语言 golden transition corpus 缺失
- **位置**:第一轮 P2-11 勾选时只建立了"整 plan 重放互查",无共享
  单转移用例集。
- **修复方向**:固定 YAML 转移 corpus(合法/非法各若干,覆盖 §3.3
  全部规则 + §6.5 极限用例),C++ 与 Python validator 各自裁决,
  断言判定一致。
- **测试**:`benchmark/tests/test_golden_corpus.py` +
  `tests/test_dd_carrier.cpp::dd_validator.golden_corpus`。

### 9. [ ] 非单位权重 cost 一致性未测
- **位置**:跨语言 cost 测试只覆盖默认 α=β=γ=δ=1;DD_ALPHA..DELTA
  非默认值下 C++/Python 无对照。
- **测试**:同一固定 plan,权重 (2,1,5,3) 下两侧数值一致。

### 10. [ ] sweep 缺 1:50 档
- **位置**:generate_sweep_instances.py 只有 1:2/1:5/1:10/1:20
  (20×20 放不下 1:50 的 shelf 数)。
- **修复方向**:1:50 档改用更大地图(如 40×40)生成,重跑该轴。
  design §8.2 已如实标注现状。

### 11. [ ] 工具类勾选项缺最小自动测试
- visualizer(ascii/png/web)的 plan 解析与 validator 复核路径、
  sweep 生成器的轴集合断言、ablation runner 的 env wiring
  (每个变体对 planner 有可观察配置差异——正是本轮抓出 no_astar
  的检查)。各加冒烟测试。

### 12. [ ] 过时注释与 README 数字
- dd_planner.cpp 中仍有"park pair 记录在 solve-level registry"的
  旧注释(与实现相反),删除;
- benchmark/README.md 补 v3(两阶段配置)结果表与 no_astar 作废说明。

## P2 质量改进(非阻塞,按收益排序)

### 13. [ ] 往返震荡抑制(可视化实证观察)
- 三个叠加来源:ρ 无迟滞(η 未实现)、D_b 平局翻转、idle 避让缺失
  (design §12.5)。
- 步骤(test-first):(a) validator plan_cost 加 `reversals` 指标
  (A→B→A 计数)先测基线;(b) η 迟滞;(c) 路径惯性(前路径格
  微折扣);(d) idle 主动离开活跃 path;每步全量 A/B,质量指标
  (makespan/SOC/reversals)与成功率都不得回归。

### 14. [ ] duplicate g-relax/rewire(恢复 eventually-optimal 的前提)
- 实现 reopen/relax/rewire 后补穷举小图最优性对照;在此之前
  §4.3 的收窄措辞保持。

### 15. [ ] wait-for graph(定位:提质量而非修 bug)
- dev 全集已可解;评估其对 makespan/SOC 的增量收益。

### 16. [ ] 扩展篮子(v2 既定)
- Hungarian ρ、carrier-aware LNS、ITA-τ 动态 goal、非单位权重进
  solver 目标、robot 匿名化、no-following 语义开关对齐实验、
  placement score 新假设的 ablation。

## 修复顺序建议

1. P0-5(revisit 回归——完备性主张的最后一块测试空洞)
2. P0-1 遗留(定理 1 反例测试)、P0-4 遗留(MODE 报错)
3. P0-2(park 纯度测试,按结果定纯化与否)
4. P1-8/9(golden corpus + 非单位权重——语义防漂移)
5. P1-10/11/12(sweep 1:50、工具冒烟测试、注释/README)
6. P2-13(震荡抑制,用户可见的质量项)
7. P2-14/15/16 按 M4/论文需要排期

---
维护约定:每完成一项勾选 [x] 并附 commit hash 与测试名;新发现的
问题追加到对应优先级段落;protected 测试改动需独立 subagent APPROVE。
