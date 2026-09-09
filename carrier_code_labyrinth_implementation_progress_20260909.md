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
| Phase 0：设计和基线 | 完成（FC smoke 待 Carrier lifecycle） | §28、两个分支、固定 benchmark、Brazil workspace、425-test 与 3-case BR-LaCAM baseline 已完成 |
| Phase 1：normal arbitrary-root | GREEN | public from-state API 与 root-aware search/finalization 已接入同一条 `TAPFPlanner::solve()` 路径；432/432 C++ tests 与 quick 77 无回归 |
| Phase 2：C ABI/shared library | GREEN | portable、YAML-free `libcarrier_lacam.so` 已完成；C ABI 5/5、异常边界 1/1、portable build 回归、432/432 主测试与 quick 77 均通过 |
| Phase 3：Java adapter/JNA | GREEN | graph/JNA/state adapter、严格 sidecar loader、cold planning session 和真实 adapter→native round-trip 均通过；native sidecar 的 Brazil packaging 留待 lifecycle build wiring |
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
总返回时间，不能把三者混成一个 runtime。SAZ1 已进入固定 Brazil
workspace；当前缺口是 Carrier planning session、joint executor 和 DSR
lifecycle 尚未接通，因此 smoke 保留为 pending，不得换成别的 KMAP。

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
| 2026-09-09 | Phase 2 GREEN | `carrier_lacam_jna.cpp` + shared target | opaque handle、bulk-copy input、完整 joint action matrix、status、首次解/可交付时间与 cost 已接入；native 内部只调用 `solve_carrier_lacam_from_state_result()` |
| 2026-09-09 | Phase 2 regression RED | C ABI 错误报告分配失败 | 注入连续两次 `std::bad_alloc` 后，旧实现因 catch 路径再次分配而 `terminate`；新增 protected exception-boundary test 固化 |
| 2026-09-09 | Phase 2 exception GREEN | `test_carrier_lacam_exception_boundary` | GREEN：1/1；错误字符串分配失败时退回静态消息，异常不穿过 `extern "C"` |
| 2026-09-09 | Phase 2 build review | C API 测试链接边界 | 独立 GPT-5.6 Sol/high：APPROVE；C API 测试改为独立 executable，必须真实 `NEEDED libcarrier_lacam.so`，且继续单独运行 `test_all` |
| 2026-09-09 | Phase 2 regression RED | portable build 无 YAML 环境 | 显式禁用 PkgConfig/yaml-cpp 后，旧配置按预期失败于 unconditional `find_package(PkgConfig REQUIRED)` |
| 2026-09-09 | Phase 2 portable GREEN | `test_carrier_lacam_portable_build.cmake` | GREEN：`CARRIER_LACAM_PORTABLE_ONLY=ON` 可在无 PkgConfig/yaml-cpp 的全新目录配置并构建共享库 |
| 2026-09-09 | Phase 2 ABI validation | shared-library 边界 | C API 5/5；真实 ctypes round-trip 返回预期 4×2 MOVE/WAIT/LIFT/DROP 矩阵；纯 C11 header 通过；`.so` 无 yaml 依赖和 native CPU flags；`liblacam.a` 无重复 C ABI symbols |
| 2026-09-09 | Phase 2 regression | 默认 native build + 全量 C++ tests | GREEN：432/432，245.687 秒；独立 C API 5/5，异常边界 1/1 |
| 2026-09-09 | Phase 2 quick | 固定 quick 77，10 秒、seed 0、unit weights、14 workers | 47/77，solver runtime 总和 588.8 秒，墙钟 47.1 秒；与 Phase 1 的成功集合、plan hash 和全部 solution metrics 逐 case 完全一致 |
| 2026-09-09 | Phase 2 code review | 最终 C ABI/build diff | 独立 GPT-5.6 Sol/low：APPROVE；异常边界、符号隔离、YAML-free 配置、默认 native 路径和 CPU 可移植性均通过 |
| 2026-09-09 | Phase 3 workspace | Code-Labyrinth Brazil workspace 与基线 release build | GREEN：`share/bofu/DSRS-mppf-classic-dig@1b4ca13` 在固定 workspace 完整构建通过 |
| 2026-09-09 | Phase 3 RED | `CarrierLacamNativeTest` + `CarrierSessionGridTest` | 预期编译失败：缺少 Carrier C ABI 的 Java 表达、严格 native lifecycle、joint action matrix 与 storage-block grid |
| 2026-09-09 | Phase 3 GREEN | JNA wrapper、action/result 类型与 session grid | GREEN：native contract 4/4、grid contract 4/4；保留 WAIT/MOVE/LIFT/DROP 的完整 robot×timestep 动作矩阵，并拒绝对角边、长边、单向边和非法 portal |
| 2026-09-09 | Phase 3 RED/GREEN | `CarrierProblemAdapterTest` | 先因缺少确定性快照适配器而 RED；实现固定 entity identity、robot/shelf/target/goal 数组和 source block 后 3/3 GREEN |
| 2026-09-09 | Phase 3 identity regression | 同 ID、不同 Pod 对象的 aliasing | 旧实现按 `Pod.equals()` 误接受而 RED；改用 identity semantics 并校验 drive/pod/storage 双向 custody 后 GREEN |
| 2026-09-09 | Phase 3 native round-trip | Java/JNA 直接装载 Phase 2 `libcarrier_lacam.so` | GREEN：1×4、2 robots、1 target；首次解 `<1 ms`，可交付解 0.687796 ms，返回 4 步 MOVE/WAIT、LIFT/WAIT、MOVE/WAIT、DROP/WAIT |
| 2026-09-09 | Phase 3 failure telemetry RED/GREEN | `CarrierLacamFailureMetricsRegressionTest` | timeout 路径原先把 native 的 12.5 ms 首次解覆盖成 0 而 RED；wrapper 保留 failure-side first/deliverable timing 后 GREEN |
| 2026-09-09 | Phase 3 regression | Code-Labyrinth `brazil-build release` | GREEN：完整 package 构建通过，最新日志 `.build-logs/brazil-build-20260909-084256-15296.log`，9 秒 |
| 2026-09-09 | Phase 3 planning session RED | `CarrierPlanningSessionTest` | 预期编译失败：缺少稳定 Java schema 与每次 solve 的 cold native context 协调层 |
| 2026-09-09 | Phase 3 planning session GREEN | cold `CarrierPlanningSession` | GREEN：4/4；每次 attempt 重新抓 live snapshot、创建并关闭独立 native context，TIMEOUT/JNA 异常不污染外层 session，结果绑定本次输入 snapshot |
| 2026-09-09 | Phase 3 loader RED/GREEN | `CarrierLacamLibraryLoaderTest` | 先因缺少 loader 而 RED，随后 4/4 GREEN；只接受显式绝对 `libcarrier_lacam.so` 或 `${root}/lib` sidecar，拒绝 cwd、系统路径、默认 JNA 搜索和 fallback，并立即校验 ABI 1 |
| 2026-09-09 | Phase 3 real loader smoke | 新 loader → JNA → Phase 2 `.so` | GREEN：首次解 `<1 ms`，可交付解 0.669906 ms，完整 4 步 MOVE/WAIT、LIFT/WAIT、MOVE/WAIT、DROP/WAIT |
| 2026-09-09 | Phase 3 real adapter smoke | `CarrierProblemAdapter` → `CarrierPlanningSession` → native | GREEN：root robots `[1,4]`、targets `[4,2]`、anonymous `[1,3]`；首次解 5.0 ms，可交付解 6.210154 ms，9 步 |
| 2026-09-09 | Phase 3 regression | Code-Labyrinth `brazil-build release` | GREEN：planning session 与 loader tests 各 4/4；完整 package 构建日志 `.build-logs/brazil-build-20260909-085135-12899.log`，10 秒 |
| 2026-09-09 | Phase 3 simple benchmark | 固定 Labyrinth 3-case subset，10 秒/seed 1/objective 4 | GREEN：3/3；首次解 0.158906/0.164177/0.156555 ms，最终 cost 26/38/10，与冻结 baseline 质量一致且无 invalid diagnostic |
| 2026-09-09 | Phase 3 code review | planning session 与 loader 边界 | 独立 GPT-5.6 Sol/low 首轮 REJECT：cold context 被焊死在外层 session 会卡住 Phase 6，runtime sidecar 允许 symlink 逃逸 |
| 2026-09-09 | Phase 3 backend fix | 可替换 `CarrierPlanningBackend` | GREEN：cold create/destroy 下沉到 `ColdCarrierPlanningBackend`；外层 session 只捕获 snapshot、绑定 result 和串行调用，未来 persistent backend 不需修改 protected cold test |
| 2026-09-09 | Phase 3 symlink RED/GREEN | `CarrierLacamLibrarySymlinkRegressionTest` | 当前 loader 对最终 `.so` 和 runtime `lib` symlink 的两例均先 RED；增加 real-path containment 与 NOFOLLOW 检查后 2/2 GREEN，JNA 调用次数保持 0 |
| 2026-09-09 | Phase 3 hardened regression | Code-Labyrinth `brazil-build release` + 真实 loader smoke | GREEN：完整 package 日志 `.build-logs/brazil-build-20260909-085753-25994.log`，10 秒；真实 `.so` 返回 OK、首次解 `<1 ms`、可交付解 0.677 ms、4 步 |
| 2026-09-09 | Phase 3 re-review | 修正后的完整 Phase 3 diff | 独立 GPT-5.6 Sol/low：APPROVE；backend seam 可承接 Phase 6，loader fail closed，未发现第二条 planner、fallback、snapshot 错配、关闭竞态或 native context 泄漏 |

## 6. 当前下一步

1. 按 TDD 实现 Phase 4 joint executor，逐 timestep 原子执行完整动作矩阵，
   同时保存 Java 侧 Pod identity/custody。
2. 修正 Carrier 模式的 DSR 完成判定：target 必须完成 DROP，并由
   `StorageManager` 在 border cell 绑定后才算完成。
3. 接入 `CarrierDsrCoordinator`、execution lease 和
   `DsrPlanningStrategy=carrier`，同时补齐 `${ENVROOT}/lib` native sidecar
   的 Brazil build/package wiring。
4. 接通 lifecycle 后运行固定 SAZ1 smoke；开发期继续只用固定
   Labyrinth simple subset 和 quick 77，不运行 full 518。
