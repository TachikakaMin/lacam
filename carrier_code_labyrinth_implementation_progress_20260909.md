# Carrier-LaCAM × Code-Labyrinth 实现进度

日期：2026-09-09

## 1. 固定基线

- dd-lacam 设计/实现分支：`carrier-dsr-integration`
- dd-lacam 起点：`447546d`
- Carrier 算法基线：`2d4563c`
- Code-Labyrinth 实现分支：`carrier-lacam-integration`
- Code-Labyrinth 起点：`share/bofu/DSRS-mppf-classic-dig@1b4ca13`
- 参考旧版：`share/aormiga/DSRS-official@9bf9d99`

规范来源：

- `rules.md`
- `design_final.md` §28
- `carrier_code_labyrinth_dsr_integration_plan_20260909.md`

## 2. 不可破坏的边界

1. 只有一个搜索入口：`TAPFPlanner::solve()`。
2. arbitrary-root 必须继续经过现有 tau、Task-BR、rho、timed transport、
   Carrier-PIBT 和 `apply_ops()`。
3. normal root 不能借用 BRD `CarrierEventContract` 固定 robot、endpoint
   或 route。
4. search、normalize、reference、repair、cost 和最终 replay 必须使用同一
   `PhysConfig` root。
5. shelf-free LaCAM-TAPF 必须自然保持原行为，不能用 fallback 或
   feature flag 规避。
6. 新增 tests 和 benchmark cases 创建后即为 protected。
7. 开发期只运行固定 simple Labyrinth subset 和仓库 quick 77；full 518
   必须等最终 tests、quick、diff 和独立 GPT-5.6 Sol/high 审查通过。

## 3. 阶段状态

| 阶段 | 状态 | 当前证据 |
|---|---|---|
| Phase 0：设计和基线 | 完成（FC smoke 待 Brazil workspace） | §28、两个分支、固定 benchmark、425-test 与 3-case BR-LaCAM baseline 已完成 |
| Phase 1：normal arbitrary-root | GREEN | public from-state API 与 root-aware search/finalization 已接入同一条 `TAPFPlanner::solve()` 路径；432/432 C++ tests 与 quick 77 无回归 |
| Phase 2：C ABI/shared library | RED | protected C ABI contract 已固定；target 在链接阶段因 `carrier_lacam_*` 尚未实现而按预期失败 |
| Phase 3：Java adapter/JNA | 未开始 | 等 Phase 2 GREEN |
| Phase 4：joint executor | 未开始 | 等 Phase 3 GREEN |
| Phase 5：DSR lifecycle | 未开始 | 等 Phase 4 GREEN |
| Phase 6：跨 prefix session | 未开始 | 等端到端 cold session 正确 |
| Phase 7：rho incremental repair | 未开始 | 等 session telemetry |
| Phase 8：一般并发 | 未开始 | 首版独占 epoch 完成后再做 |

## 4. 固定开发 benchmark

机器可读定义见 `benchmark/labyrinth_simple_benchmark.json`。以下四个用例在
Phase 1 实现前固定，全部直接来自 Code-Labyrinth
`share/bofu/DSRS-mppf-classic-dig@1b4ca13`，之后不得根据 Carrier 的表现
替换：

| 用例 | 来源 | 固定参数 | 主要覆盖 |
|---|---|---|---|
| `goal0_x4_y10_t1_b1_g0_rand4` | `paper_testcases/goal0/x4_y10_t1_b1_g0_rand4.yaml` | seed 1、objective 4、10 秒、`--eve` | 单目标、单空位、最小 dig |
| `goal0_x4_y10_t4_b4_g0_rand0` | `paper_testcases/goal0/x4_y10_t4_b4_g0_rand0.yaml` | seed 1、objective 4、10 秒、`--eve` | 四目标/四空位、并发目标和 blocker |
| `goal2_x6_y10_t1_b14_g2_rand3` | `paper_testcases/goal2/x6_y10_t1_b14_g2_rand3.yaml` | seed 1、objective 4、10 秒、`--eve` | 一个目标对应两个可接受 goal、稠密货架 |
| `saz1_test_fc_smoke` | `res/SAZ1/SAZ1_test.kmap` | seed 0、8 drives、60 simulated seconds、native planner 10 秒 | 最小真实 FC 端到端装配和执行 |

前三个 planner case 的固定命令模板是：

```text
cd third_party/code-labyrinth-mppf/external/mapf-planner
build/main -p <yaml> -v 2 -t 10000 -s 1 -O 4 --eve -o <output>
```

不能只看进程退出码，因为现有 `main` 对空解和 invalid solution 也可能返回
0。成功必须同时满足：结果非空、`solved=1`、起点一致、终点满足所有 goal、
整条轨迹通过 `is_feasible_solution()`，且 console 没有 invalid
diagnostic。报告优先写首次找到可行解的 runtime，再写 10 秒结束时的最终
cost。

真实 FC smoke 的完整参数已写入 JSON；其中必须显式覆盖
`MppfPlanTimeLimitMs=10000`，不能使用代码默认的 50ms。baseline 和
Carrier 除 `DsrPlanningStrategy=mppf/carrier` 外，experiment、KMAP、
seed、drive count、请求、模拟时长、线程数和统计口径完全相同。

原版 BR-LaCAM planner baseline 记录于
`benchmark/labyrinth_simple_baseline_20260909.json`：

| 用例 | 首次可行解 | 首次 cost | 10 秒内最终 cost | 原版返回/证明 |
|---|---:|---:|---:|---|
| `goal0_x4_y10_t1_b1_g0_rand4` | 0.167861 ms | 26 | 26 | 152.74 ms，optimal |
| `goal0_x4_y10_t4_b4_g0_rand0` | 0.160578 ms | 41 | 38 | 10845.9 ms，deadline 时 suboptimal |
| `goal2_x6_y10_t1_b14_g2_rand3` | 0.168270 ms | 10 | 10 | 10046.0 ms，deadline 时 suboptimal |

三例结果文件均非空、`solved=1`，且 console 没有 internal feasibility
validator 的 invalid diagnostic。第二例说明原版在 10 秒 search deadline
之后还有约 0.85 秒收尾，因此后续报告必须分开记录首次解、10 秒最终解和
总返回时间，不能把三者混成一个 runtime。SAZ1 FC smoke 需要 Java package
位于 Brazil workspace；当前 clone 不是 workspace，因此状态保留为
`pending_brazil_workspace`，不得换成别的 KMAP。

## 5. RED/GREEN 与验证日志

| 时间 | 阶段 | 操作 | 结果 |
|---|---|---|---|
| 2026-09-09 | Phase 0 | 完整读取 `rules.md`、实施计划和 Brazil HappyTrails/C++ 指南 | 完成 |
| 2026-09-09 | Phase 0 | 对照代码确认 arbitrary-root 与 finalization 的修改面 | 完成：搜索 root 与交付 root 当前不一致 |
| 2026-09-09 | Phase 0 | 更新 `design_final.md` §28 | 完成 |
| 2026-09-09 | Phase 0 | 创建 dd-lacam 与 Code-Labyrinth 实现分支 | 完成 |
| 2026-09-09 | Phase 0 | 固定 3 个 paper YAML + 1 个 SAZ1 真实 FC smoke | 完成：`benchmark/labyrinth_simple_benchmark.json` |
| 2026-09-09 | baseline | `cmake --build build -j14 && ./build/test_all --gtest_brief=1` | GREEN：425/425，246.018 秒 |
| 2026-09-09 | baseline | 构建 Code-Labyrinth 原版 `external/mapf-planner` | 首次发现 GCC 7 缺少标准 `<charconv>`/`<filesystem>`；仅用临时兼容头处理，不修改算法源码 |
| 2026-09-09 | baseline | 固定 3-case BR-LaCAM planner subset，10 秒/seed 1/objective 4 | GREEN：3/3；首次解均约 0.16ms，详见 baseline JSON |
| 2026-09-09 | review | protected EventContract test 语义变更 | 独立 GPT-5.6 Sol/high：APPROVE；contract validation 必须原样保留 |
| 2026-09-09 | Phase 1 RED | `test_dd_arbitrary_root` + EventContract root 语义测试 | 预期编译失败：缺少 from-state API，以及 root-aware normalize/cost/replay/reference/fixed-goal/repair |
| 2026-09-09 | review | protected arbitrary-root reference assertion 变更 | 第一次方案被独立 GPT-5.6 Sol/high REJECT；加入生产 pass-2 `reference_plans_received/validated` telemetry 后，第二次方案 APPROVE |
| 2026-09-09 | Phase 1 GREEN | arbitrary-root、EventContract、plan cost、reference、repair 定向测试 | GREEN：7/7、6/6、11/11、8/8、7/7 |
| 2026-09-09 | Phase 1 regression | `cmake --build build -j14 && ./build/test_all --gtest_brief=1` | GREEN：432/432，245.801 秒 |
| 2026-09-09 | Phase 1 compatibility | `test_tapf_compat --gtest_brief=1` | GREEN：7/7，120.462 秒；shelf-free 原路径保持兼容 |
| 2026-09-09 | benchmark tooling | Python benchmark tests | 200 项直接通过；4 项因初次启动缺少 `PYTHONPATH=benchmark`，按正确入口补跑 12/12 通过；另 1 项正确识别旧网页仍绑定旧 binary SHA，留待最终发布时重生成 |
| 2026-09-09 | Phase 1 quick | 固定 quick 77，10 秒、seed 0、unit weights、14 workers | 47/77，首次可交付 solver runtime 总和 588.4 秒，墙钟 47.1 秒；与上一权威 quick 的成功集合和全部 solution metrics 逐 case 完全相同 |
| 2026-09-09 | Phase 1 code review | 当前实现 diff | 独立 GPT-5.6 Sol/low：APPROVE；未发现 root 丢失、兼容 wrapper 误用、EventContract 弱化、parallel planner、fallback 或 dead production code |
| 2026-09-09 | Phase 2 RED | `test_carrier_lacam_c_api` | configure 与测试编译成功，链接按预期失败：缺少全部 `carrier_lacam_*` C ABI symbols |

## 6. 当前下一步

1. 提交并推送 Phase 1 arbitrary-root 实现和验证证据。
2. 按 TDD 开始 Phase 2：在现有 Carrier 入口外增加独立 C ABI 数据边界和
   shared-library build target，但 native 内部仍只调用同一条
   `solve_carrier_lacam_from_state_result()` 路径。
3. C ABI 能表达完整 `DDInstance`、当前 `PhysConfig`、逐 timestep 联合动作、
   status、首次解 runtime 和最终 cost 后，再把固定三个 Labyrinth planner
   case 转入 Carrier adapter 的 simple benchmark。
4. Code-Labyrinth Java package 放入 Brazil workspace 后再跑固定 SAZ1 smoke。
