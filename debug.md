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
- [x] **遗留测试**:`benchmark/tests/test_theorem1.py` 2 用例
  (84255fd;R2 放宽灵敏度验证:关掉 swap 检查测试即失败)。

### 2. [x] park 纯度:same-X/双缓存历史不变性未测,声明已降级
- **位置**:dd_planner.cpp park/hover 逻辑 + PathCache 非对称失效
  (§6.2);design.md §5.4a。
- **问题**:park 实为 (X, D_b 缓存纪元) 的函数,非纯 X 函数——同一 X
  经不同缓存历史可得不同 owner/park 集。只影响 ordering 可复现性,
  不影响可行域,但原勾选声明("纯 X 函数")过强。
- **处置**:design.md v2.3 已把声明降级并说明纪元依赖(本轮完成)。
- **处置**(84255fd,tests/test_dd_park_purity.cpp 4 用例,夹具经
  subagent APPROVE):同 X 双历史测试落地;审计发现原 strict 实现
  也非纪元无关(路径局部快照看不见 off-path 腾空)——按"修实现不放宽
  测试"方向,DD_STRICT_INVAL 升级为**全局有效占用快照失效**(真纪元
  无关,合同成立);默认 lazy 的纪元依赖固化为 characterization 断言
  (若未来变纪元无关,该断言失败提示升级 design 5.4a 声明)。

### 3. [x] no_astar 消融是 no-op(假对照)
- **位置**:run_ablations.py(设 `DD_NO_ASTAR=1`);生产代码从不读取
  该变量,least-blocking 路径本就是 Dijkstra(无 A* 开关)。
- **问题**:"no_astar 9/9" 不能作为消融证据;§6.2 原文误写 A*。
- **处置**(本轮完成):变体已从 run_ablations.py 移除;design.md
  §5.3(2)/§6.2 修正 Dijkstra 措辞;§6.6 记录该历史。
  results_ablation/ablation_rows.csv 中的 no_astar 行**作废不采信**
  (其余变体行不受影响,各变体独立运行)。
- [x] 已重生成:results_ablation/ablation_rows.csv(11 变体 × 9 dev
  cases,新默认配置,无 no_astar;2026-08-30)。

### 4. [x] dd_benchmark MODE 支持未提交(复现性)
- **位置**:tools/dd_benchmark.cpp 的 b0/b1 分派与 first_solution_ms
  输出一直是未提交 diff;results_final_v2 的 B0/B1 列无法从干净
  HEAD 复现。
- **处置**:已提交(本轮,见 git log "Commit stray dd_benchmark MODE
  support");runner 传参与 CLI 现在一致。
- [x] **遗留测试**:benchmark/tests/test_cli.py 3 用例(84255fd,
  RED→GREEN):未知 MODE exit 2 + stderr 提示;mode= 回显三模式可
  区分;默认 mode=lacam。

### 5. [x] P0-1(第一轮)自称的 revisit 回归测试从未写
- **位置**:第一轮清单第 1 项承诺"构造小实例强制走 revisit 路径,
  断言同一节点约束树枚举的 (robot,op) 集合与冻结 order 一致";
  实际只有 fresh-node 穷举(test_dd_g1),revisit 路径无覆盖。
- **修复方向**:暴露测试 hook(或经 DDStats 观察),强制同一节点
  revisits≥8 触发 re-guidance 后,断言 constraint_order 与叶子
  (robot,op) 集合不变。
- **测试**:`dd_g1_conformance.revisit_reguide_preserves_enumeration`
  (84255fd;dd_enumerate_node_successors_reguided API 复刻生产
  re-guidance 变异中途施加;注入"改写 constraint_order"bug 灵敏度
  验证通过后还原)。

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

### 8. [x] 跨语言 golden transition corpus 缺失
- **位置**:第一轮 P2-11 勾选时只建立了"整 plan 重放互查",无共享
  单转移用例集。
- **修复方向**:固定 YAML 转移 corpus(合法/非法各若干,覆盖 §3.3
  全部规则 + §6.5 极限用例),C++ 与 Python validator 各自裁决,
  断言判定一致。
- **测试**(736f63f):tests/fixtures/golden/ 3 案例 23 行(R1/R2
  swap+following、S1、S2-via-R2、I1 前置、I3 hover、D1 上层
  following、2×2 零空格 rotation)+ 双跑器 test_dd_golden.cpp /
  test_golden_corpus.py;判定翻转灵敏度双侧验证。

### 9. [x] 非单位权重 cost 一致性未测
- **位置**:跨语言 cost 测试只覆盖默认 α=β=γ=δ=1;DD_ALPHA..DELTA
  非默认值下 C++/Python 无对照。
- **测试**:benchmark/tests/test_weights.py(736f63f):强制匿名搬运
  夹具上 (2,1,5,3) 的 weighted_soc 与全部分量计数器跨语言一致。

### 10. [x] sweep 缺 1:50 档
- **位置**:generate_sweep_instances.py 只有 1:2/1:5/1:10/1:20
  (20×20 放不下 1:50 的 shelf 数)。
- **处置**(736f63f):ratio_r4x50 档(40×40、4 robots、200 shelves、
  8 targets、k=320)生成并纳入全量套件;两实例均 10s 内可解。

### 11. [x] 工具类勾选项缺最小自动测试
- 处置(736f63f):benchmark/tests/test_tools.py 5 用例——ascii 帧
  渲染、web viz 拒坏 plan(validator 门)、sweep 轴集合含 1:50、
  ablation env 接线静态检查(runner 设的每个 DD_* 必须被生产源码
  读取——正是抓 no_astar 的那个检查)、变体 mode 合法性。

### 12. [x] 过时注释与 README 数字
- 处置(736f63f):两处 registry/sticky 旧注释替换为 per-X 语义;
  README round-2 章节(v3 结果、no_astar 作废、CLI 合同、1:50)。

## P2 质量改进(非阻塞,按收益排序)

### 13. [x] 往返震荡抑制(可视化实证观察)
- 处置(a4c6cb7,四步 test-first 全落地):(a) reversals 指标
  (test_metrics.py 5 用例;顺带修复 plan_cost 首 goal 截断 bug);
  (b) η 迟滞 DD_ETA=2(dd_match_free_goals,3 用例);(c) 路径惯性
  (2N 缩放-1 折扣严格平局,dd_least_blocking_path,2 用例);
  (d) idle 避让 DD_IDLE_AVOID=1(dd_root_joint_ops,2 用例)。
  dev 9 例累计:reversals 27392→17512(-36%)、SOC -11%、mk -8.6%、
  9/9 不变;叠加 16a Hungarian 后 reversals 13104(-52%)。

### 14. [x] duplicate g-relax/rewire(恢复 eventually-optimal 的前提)
- 处置(b21aaf2):cheaper-g 命中更新 g/parent_edge 并 reopen,
  goal 配置 relax 即再抽取;两相 stats 合并修复。测试
  test_dd_rewire:12 种子 3×3 族 solver best_soc == 全转移图
  穷举最优、relax 在族上触发、rewired plan 重放合法;dev 中性。

### 15. [x] wait-for graph(定位:提质量而非修 bug)
- 处置(0e7231c):robot→robot(下层占位)与 carrier→(grounded
  shelf)→clearer 两类边,函数图环检测;livelock 时定向禁忌环成员
  ρ 对(兜底回退全量禁忌)。测试 test_dd_waitfor 3 用例(对头环用
  DD_NO_YIELD 隔离、跨层环、无环对照)。收益:dev 质量不变;
  dneM_seed18(此前两阶段默认下不可解)恢复可解。

### 16. [x] 扩展篮子(v2 既定)——有界子集完成,其余按本清单排期
  留 design §7 扩展
- [x] Hungarian ρ(3da02d5):O(n³) 位势匹配,ACTIVE_CAP 限界;
  dev A/B 大胜(mk -10%、SOC -7%、reversals -25%)→ **转正默认**
  (DD_RHO_HUNGARIAN=0 回退);两个丢失 DnE 实例(seed18/22)全部
  恢复可解。交叉对分离单元测试。
- [x] 非单位权重进 solver(3da02d5):DD_SOLVER_WEIGHTS=1 贯穿
  g/admissible-h/rollout(默认仍单位,design v1);(2,1,5,3) 加权
  穷举最优性质测试通过。
- [x] no-following 开关实验(16c):DD_NO_FOLLOWING=1 双层拒
  following;量化:5 例子集 mk 总和 159→350(2.2×)、d50 3.6×,
  命题 2 实例诚实不可解(test_dd_nofollow 3 用例)。
- [x] placement 新假设 ablation(16d):DD_PLACE_ESCAPE 同层逃逸度
  平局裁决;dev A/B 更差(mk +4.4%)→ **评估后不采纳**,默认关,
  单元测试钉住旋钮行为。
- [ → design §7] LNS、ITA-τ、robot 匿名化:按本清单原文"按 M4/
  论文需要排期",维持 design.md §7/§12 扩展记录,不属本轮范围。

## P1-17 骨架回迁(2026-08-30 第三轮独立审计,GPT-5.6 Sol high)

审计结论:dd_planner 属**另起炉灶**(置信 95%)——架构形状可追溯
LaCAM,但对原骨架符号引用为 0;自写 Hungarian 与 tapf_assignment 原版
token 相似率 72.4%(复制而非调用)。违反 design §10 "fork tapf_planner、
DistTable 直接复用" 的既定方针。按审计的最小代价路径逐组件回迁
(全程 125+ 测试全绿 + 164 套件性能不回退为门禁):

1. [x] **Hungarian 复用**:原匿名 HungarianAssignment 提为公开
   `tapf_hungarian_row_to_col`,TAPF 原类改薄包装,DD 复制版删除;
   契约测试 test_tapf_hungarian_shared 5 用例(矩形/负代价/禁制/
   平局确定性/穷举对照);确定性逐位验证(dev SOC 38178/15801/35949
   与 v4 一致)。
2. [x] **共享 lazy distance core**:lazy_dist.hpp 落地——原 DistTable
   的 resumable lazy BFS(RRA*)提为拓扑无关模板 LazyBfsField,DD 侧
   完成接入(DDLazyDist/DDDistCache adapter,旧 bfs_dist/DistCache
   复制版删除;`.to()` 保 legacy 全量视图、`dist()` 提供 lazy 迁移
   路径;sentinel 按调用方传入)。契约测试 test_lazy_dist 4 用例
   (任意查询序等价/可续性 expanded 计数/哨兵/adapter 全量视图);
   benchmark 数字逐位不变(479/15801/38178/35949)。
   上游已收口:dist_table / tapf_dist_table 均改为 GraphIdTopology
   adapter 挂同一核心(近似逐行重复的两份 BFS 删除;test_all 86 测试
   门禁通过)。**本项完成**(标记改 [x])。
3. [ ] **topology 接口**:非 owning GridTopology 适配 Graph::U 与
   DDGrid;保 DD 邻接序 down/up/right/left(确定性);地图解析工具
   共享。
4. [x] **FOCAL selector 对齐**:原 tapf_planner select_open_index
   语义(全 OPEN viable f_min、bound=w·f_min、h_adm 平局、栈顶兜底)
   落入 DD;shadow A/B 门禁精确通过(164 套 162/164、r2r mk=548、
   dev 中性 mk 3264 vs 3270)后转正默认(w=1.5;DD_FOCAL_W=0 回退
   legacy top-32)。design §10 表已更新。
5. [ ] **Node/OPEN 骨架接口层**:模板化 state key/parent/order/
   constraint FIFO/g-h-f/EXPLORED;先定义"搜索 parent vs solution
   parent(parent_edge) vs guidance ancestry"三语义再动手。
6. [ ] **PIBT 递归框架抽共享**(reserve-next/occupant recursion/
   swap hook),Carrier 语义留 policy;若 benchmark 不稳则保留改造
   fork 并记录理由。
7. [ ] **loader 解析工具共享**(坐标/inline map/wall 字符),schema
   保持两套。

正当保留差异(审计确认,design §10 表格已记录):operator constraint、
PhysConfig/canonical+Zobrist hash、apply_ops 裁决、constraint_order、
least-blocking PathCache、serve/clear/park/yield/wait-for、macro
rollout+parent_edge、两阶段 anytime、DD YAML schema。

## 修复顺序建议

1. P0-5(revisit 回归——完备性主张的最后一块测试空洞)
2. P0-1 遗留(定理 1 反例测试)、P0-4 遗留(MODE 报错)
3. P0-2(park 纯度测试,按结果定纯化与否)
4. P1-8/9(golden corpus + 非单位权重——语义防漂移)
5. P1-10/11/12(sweep 1:50、工具冒烟测试、注释/README)
6. P2-13(震荡抑制,用户可见的质量项)
7. P2-14/15/16 按 M4/论文需要排期

---
完成记录(2026-08-30,round-2 全部落地):P0 1-5、P1 6-12、P2 13-16
全部 [x](16 的 LNS/ITA-τ/robot 匿名化按本清单排期维持 design §7
扩展记录)。最终验证:C++ 13 target 68 测试 + Python 57 测试全绿;
164 实例 × 7 方法统一 10s(results_final_v4,jobs=16 物理核):
carrier 162/164(r2r 25/25 且 mk 548 与 crest_base 546 打平、
s2w 25/25、dne 24/25),对比 b4 115、crest_base 81、natcbs 21;
论文级质量对 v2 默认 2.6–3.1×。遗留:dneM_seed22(统一规模域
边界的已记录代价)、1 个 std scramble、strict_inv 诊断模式变重
(7/9,全局快照代价)。耗时记录:全量 470s、ablation 46s(16 路)。
教训入档:benchmark 并行度必须钉在物理核数(HT 超订制造假超时);
与计时基准并行跑测试套件会污染判定(v4 首跑作废重跑)。

维护约定:每完成一项勾选 [x] 并附 commit hash 与测试名;新发现的
问题追加到对应优先级段落;protected 测试改动需独立 subagent APPROVE。
