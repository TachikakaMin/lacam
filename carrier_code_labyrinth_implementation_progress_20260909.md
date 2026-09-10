# Carrier-LaCAM × Code-Labyrinth 实现进度

日期：2026-09-09（最后更新：2026-09-10）

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
7. 开发期固定使用 simple Labyrinth subset；只有改动影响原算法时才运行
   quick 77。full 518 必须等最终 tests、quick、diff 和独立
   GPT-5.6 Sol/high 审查通过。

## 3. 阶段状态

| 阶段 | 状态 | 当前证据 |
|---|---|---|
| Phase 0：设计和基线 | 完成 | §28、两个分支、固定 benchmark、Brazil workspace、425-test、3-case BR-LaCAM baseline 与 SAZ1 FC smoke 已完成 |
| Phase 1：normal arbitrary-root | GREEN | public from-state API 与 root-aware search/finalization 已接入同一条 `TAPFPlanner::solve()` 路径；432/432 C++ tests 与 quick 77 无回归 |
| Phase 2：C ABI/shared library | GREEN | portable、YAML-free `libcarrier_lacam.so` 已完成；C ABI 5/5、异常边界 1/1、portable build 回归、432/432 主测试与 quick 77 均通过 |
| Phase 3：Java adapter/JNA | GREEN | graph/JNA/state adapter、严格 sidecar loader、cold planning session、真实 adapter→native round-trip 和 Brazil sidecar packaging 均通过 |
| Phase 4：joint executor | GREEN | joint timestep、WAIT barrier、MOVE/LIFT/DROP、真实 dynamics、custody 原子提交、偏差截断和 MAS reset 均通过完整 Brazil release 与独立复审 |
| Phase 5：DSR lifecycle | GREEN，已提交并推送 | Carrier coordinator、staging、execution lease、control owner、participant-scoped reset、DSR bypass/completion handoff、Pick-to-station、telemetry、legacy regression、native sidecar packaging 和真实 SAZ1 smoke 均已接通；resident-drive lease 回归已修复并由独立 Sol/high APPROVE |
| Phase 6：跨 prefix session | GREEN，已提交并推送 | C++ persistent session、prefix commit、状态 rebase、schema invalidation、PairCost 安全复用，以及 Java persistent backend、单步 checkpoint 和 coordinator 续算均已接入；真实 SAZ1 已完成 19 拍、18 次续算，C++ 440/440、Brazil release、Labyrinth subset 与 quick 77 通过 |
| Phase 7：rho incremental repair | 完成，已提交并推送 | 保留 full bottleneck threshold，只对冻结 threshold 后的 secondary Hungarian 做 changed-row repair，并重新执行 exact canonicalization；C++ 447/447、C ABI 11/11、无货架兼容 7/7、quick 77、Brazil release 和真实 SAZ1 均通过。SAZ1 标准入口完成 19 拍/18 次续算，CSV 累计记录 13,963 次 repair，且 Phase 6/7 的成功集合、plan hash 和全部解质量逐 case 零差异；算法提交 `76d54bf`，LMS 提交 `2587828` |
| Phase 8：一般并发 | 进行中；8.1～8.5 GREEN 并通过独立复审 | 8.4 已把 external lower/upper vertex、directed edge、tail policy、absolute tick 和 time-aware CLOSED 接入同一 `TAPFPlanner::solve()`/`apply_ops()` 路径；8.5 已让 persistent session 与 C ABI 保存 immutable snapshot/origin，commit 成功后推进 origin，rebase 必须显式确认或替换 commitment。新增 Phase 8.4/8.5 tests 16/16、相关 topology/repair/reference/rewire/session/C ABI 回归全部通过，portable shared-library build 和全 target 编译通过；独立 GPT-5.6 Sol/low 复审 53/53 并明确 `APPROVE`。默认空 commitment 不改变原算法，按 `rules.md` 未运行原始 benchmark；下一步是 8.6 Java adapter/block lease |

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
workspace。Carrier lifecycle 接通后，固定 60-simulation-second smoke
已通过；该 case 不得换成别的 KMAP。

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
| 2026-09-09 | Phase 4 RED | joint executor、动态 drive、Pod custody、DSR 完成判定和 execution reset tests | 按预期缺少完整联合执行协议、物理 transfer 共享语义和 post-Carrier MAS resync |
| 2026-09-09 | Phase 4 GREEN | `CarrierJointExecutorTest`、`CarrierDynamicJointExecutorTest`、`PodTransferStateUpdaterTest`、`DsrBlockManagerCarrierCompletionTest`、`DeterministicExecutionCarrierResetTest` | GREEN：6/6、2/2、2/2、1/1、1/1 |
| 2026-09-09 | Phase 4 review regression | queued ordinary motion、异构 MOVE duration、transfer 原子性、prepare cleanup、reset fail-before-write | 五个新增测试先 RED；修复后全部 GREEN，日志分别为 `.build-logs/brazil-build-20260909-151012-3469.log`、`151022-6220.log`、`151029-7444.log`、`151048-9286.log`、`151102-11060.log` |
| 2026-09-09 | Phase 4 regression | Code-Labyrinth `brazil-build release` | GREEN：完整 package 日志 `.build-logs/brazil-build-20260909-151157-20728.log`，10 秒 |
| 2026-09-09 | Phase 4 re-review | joint executor 与普通 MAS handoff 边界 | 独立 GPT-5.6 Sol/low：APPROVE；Phase 5 必须补 participant control owner 和 participant-scoped reset |
| 2026-09-09 | Phase 4 commit | Code-Labyrinth joint executor | Brazil 工作树 `c24fdd6`；集成分支 `e8a2b56`。远端 push 首次因 SSH host-key verification 失败，待复用已认证的 Amazon Git 配置重试 |
| 2026-09-09 | Phase 5 RED/GREEN | Carrier coordinator、drive staging、execution lease、control owner、freeze、participant-scoped reset、DSR bypass 和 completion handoff | GREEN：无可达 portal 时保持 pending；target 多于 robot、anonymous blocker、storage reservation、epoch abort 和普通 traffic 隔离均由 protected tests 固化 |
| 2026-09-09 | Phase 5 native integration | 真实 Carrier-LaCAM 求解两个 target/两个 border goal | GREEN：tau 产生 injective border assignment，Java 完整执行并在 DROP 后完成两个 DsrDigout |
| 2026-09-09 | Phase 5 business lifecycle | 完整 Pick → Carrier DIG → border DROP → 原 Pick 送站 | RED 首先暴露 `DriveShell` 被错误强转为 `DynamicDrive`；改为通过 `IDrive` 委托 lift/lower timing 后 GREEN，日志 `.build-logs/brazil-build-20260909-163519-31028.log` |
| 2026-09-09 | Phase 5 legacy regression | baseline/adaptive/mppf assembly 与 legacy DSR execution | GREEN：legacy FC 不创建 MAS/Carrier backend，single/multi legacy planner 分支仍生成原 `Digout`、reservation 和 pending 状态；日志 `.build-logs/brazil-build-20260909-163609-4537.log`、`.build-logs/brazil-build-20260909-163659-9641.log` |
| 2026-09-09 | Phase 5 telemetry | `CarrierPlanStats.csv` 与 coordinator aggregate | GREEN：列顺序固定为 first-solution、deliverable、cost 和 adapter/native/staging/execution/replan；Phase 6 前 cache/rho 字段显式为 unsupported；日志 `.build-logs/brazil-build-20260909-164034-2630.log` |
| 2026-09-09 | Phase 5 native coexistence RED | 同一 JVM 先加载 legacy BrLaCAM，再运行 Carrier-LaCAM | 稳定复现 `free(): invalid pointer`；两个 `.so` 暴露 242 个同名 C++ 符号，日志 `.build-logs/brazil-build-20260909-164350-25362.log` |
| 2026-09-09 | Phase 5 native coexistence GREEN | Carrier shared object 隐藏静态 core symbols，仅导出 JNA C ABI | 最小 coexistence 回归和完整 Brazil release 均 GREEN；日志 `.build-logs/brazil-build-20260909-164444-30480.log`、`.build-logs/brazil-build-20260909-164505-2397.log` |
| 2026-09-09 | Phase 5 real FC smoke | 固定 SAZ1、seed 0、8 drives、60 simulated seconds、Carrier strategy | GREEN：Java 17 运行完成 `Building model` → `Running model` → `Finished running model`，无 ERROR/Exception；结果 `/tmp/carrier-saz1-smoke.STBtuS` |
| 2026-09-09 | Phase 5 review | DSR lifecycle、lease、control ownership、legacy 隔离和 benchmark 配置 | 独立 GPT-5.6 Sol/high：APPROVE；Carrier owner 会暂停 Pick/Stow/普通 Drive，Carrier 模式不进入 legacy DIG path，baseline/Carrier smoke 除 strategy 外共享参数 |
| 2026-09-09 | Phase 6 C++ RED/GREEN | persistent planning session、prefix commit、external-state rebase、schema guard、PairCost dependency reuse 和增量 C ABI | GREEN：schema 2、session 4、rebase 2、pair incremental 5、carried commitment 2、arbitrary-root 7、old C API 5、incremental C API 2、rebase C API 2、exception boundary 1 |
| 2026-09-09 | Phase 6 C++ review/commit | normal root continuation 与 cache 生命周期 | 独立 GPT-5.6 Sol/low 在补齐完整 grid/wall/entity/goal schema guard 后 APPROVE；提交并推送 `df3deba carrier: persist incremental planning continuation` |
| 2026-09-09 | Phase 6 Java RED/GREEN | `CarrierPersistentPlanningSessionTest`、`CarrierJointExecutorCheckpointTest`、`CarrierDsrCoordinatorIncrementalSessionTest` | GREEN：同一 native handle 跨 solve 复用；精确 prefix 走 commit，偏差走 rebase；生产 coordinator 每执行一拍续算并最终只关闭一次 session |
| 2026-09-09 | Phase 6 Brazil regression | Code-Labyrinth `brazil-build release` | GREEN：完整 package 日志 `.build-logs/brazil-build-20260909-174840-16028.log`，11 秒 |
| 2026-09-09 | Phase 6 Java review RED/GREEN | persistent backend 构造失败的 native handle 生命周期 | 独立 GPT-5.6 Sol/low 首轮 REJECT：`setGrid`/`setEntities` 失败会泄漏已创建 handle；新增独立 protected regression 先 RED（`create=1,destroy=0`），再用局部 candidate 初始化并在失败时 close，GREEN 后复审 APPROVE；日志 `.build-logs/brazil-build-20260909-180915-8338.log`、`180945-12504.log` |
| 2026-09-09 | Phase 6 final Brazil regression | 修复 handle leak 后 `brazil-build release` | GREEN：完整 package 日志 `.build-logs/brazil-build-20260909-181014-18084.log`，10 秒 |
| 2026-09-09 | Phase 5/6 Code-Labyrinth checkpoint | production wiring、persistent session、全部新增 tests 与 Carrier gitlink | 提交 `0cf86ce lms: route DSR through persistent Carrier sessions`，已推送到 `share/yimint/carrier-lacam-phase3-ws` |
| 2026-09-09 | Phase 6 SAZ1 assembly smoke | 固定 SAZ1、seed 0、8 drives、60 simulated seconds、Carrier strategy | GREEN：最新 Phase 6 Java/native package 完成 `Building model` → `Running model` → `Finished running model`，无 ERROR/Exception；结果 `/tmp/carrier-saz1-phase6.4tPZ7G`。后续诊断纠正了“没有产生 DIG”的判断：DIG 已触发，但会话卡在 resident-drive staging，因此当时没有写出 native telemetry |
| 2026-09-09 | Phase 6 Labyrinth simple subset | 固定 3-case、10 秒、seed 1、objective 4 | GREEN：3/3；首次解 0.152116/0.161280/0.156828 ms，最终 cost 26/38/10，结果非空、`solved=1` 且无 `invalid solution`；结果 `/tmp/carrier-labyrinth-phase6.NOP0YC` |
| 2026-09-09 | Phase 6 C++ full regression | `cmake --build build -j14 && ./build/test_all --gtest_brief=1` | GREEN：440/440，245.695 秒 |
| 2026-09-09 | Phase 6 quick | 固定 quick 77，10 秒、seed 0、unit weights、14 workers | GREEN：47/77，solver runtime 总和 590.1 秒，墙钟 47.1 秒；与 Phase 2 的成功集合、plan hash、makespan、SOC、work 和首解质量逐 case 零差异；结果 `benchmark/results_quick_carrier_dsr_phase6_20260909` |
| 2026-09-09 | Phase 5 regression diagnosis | 真实 SAZ1 resident-drive staging | RED 前确认 32-drive 会话只 lease 最低 ID 的 8 台车；其余已位于 session grid 的 idle drives 被视为 `EXTERNAL_OCCUPIES_LEASE`，而普通 allocator 已暂停，所以会话永久停在 `WAITING_FOR_QUIESCENCE` |
| 2026-09-09 | Phase 5 resident-drive RED | `CarrierResidentDriveLeaseTest` | 2/2 按预期失败：旧实现错误选择 lease 外低 ID drive，并在 resident 数超过上限时仍错误建立 reservation；日志 `.build-logs/brazil-build-20260909-183706-29151.log` |
| 2026-09-09 | Phase 5 resident-drive GREEN | resident drives 强制参与、原地 staging、精确 drive claim | GREEN：2/2；lease 内所有 idle/unladen drive 必须进入候选池，剩余名额才按 ID 从 lease 外补齐；resident 超过上限时保持 pending，不暂停 allocator；日志 `.build-logs/brazil-build-20260909-184001-13098.log` |
| 2026-09-09 | Phase 5 related regression | lease/stager/epoch 相关 tests + Brazil release | GREEN：`CarrierDriveStagerTest`、三个 execution-lease tests、`MultiAgentSystemCarrierEpochTest` 全部通过；完整 release 日志 `.build-logs/brazil-build-20260909-184149-29449.log`，10 秒 |
| 2026-09-09 | Phase 6 real SAZ1 multi-prefix | seed 0、8 drives、2 targets、180 simulated seconds、10 秒 native limit | 运行时状态确认共执行 19 拍、18 次续算并释放 lease；首次 solve `changed_pair_edges=36`，之后均为 0。后续 Phase 7 审计发现该旧结果目录的 terminal CSV 未最终 flush，不能单独作为完成证据；已由下面的标准入口验证结果取代 |
| 2026-09-09 | Phase 5 resident-drive review/commit | protected test 与 production diff | 独立 GPT-5.6 Sol/high：APPROVE，blocking finding 为 none；提交 `3f2a267 fix: include resident drives in Carrier leases`，已推送到 `share/yimint/carrier-lacam-phase3-ws` |
| 2026-09-09 | Phase 7 design audit | 当前 V2 exact incremental rho | 完成：旧 `339fe7e` 依赖已回滚的 additive objective，不能复用；首版保留 full bottleneck threshold，转置冻结后的 secondary matrix 后复用 shared `IncrementalHungarianState::repair_rows()`，最终 canonical assignment 仍逐位对照 full oracle |
| 2026-09-09 | Phase 7 rho RED/GREEN | exact full-vs-incremental、parent invalidation、tie canonicalization、200 次随机 changed-row、deadline 和 C ABI telemetry | GREEN：rho incremental 6/6、deadline 1/1、C ABI metrics 1/1；实现直接复用 shared `IncrementalHungarianState::repair_rows()`，未增加第二套 assignment solver |
| 2026-09-09 | Phase 7 C++ full regression | `cmake --build build -j14 && ./build/test_all --gtest_brief=1`，另跑全部 Carrier C ABI 和无货架兼容 | GREEN：447/447，246.753 秒；5 组 C ABI 共 11/11；`test_tapf_compat` 7/7，120.470 秒 |
| 2026-09-09 | Phase 7 quick | 固定 quick 77，10 秒、seed 0、unit weights、14 workers | GREEN：47/77，solver runtime 总和 587.9 秒，墙钟 47.4 秒；与 Phase 6 的成功集合、status、plan hash、makespan、SOC、work、首次解和最终解质量逐 case 零差异；累计 769,541 次 changed-row repair、32,479 次 zero-row reuse；相同成功集上的 rho pipeline 计时由 23.50 秒降至 16.02 秒（-31.8%），其中 assignment 由 16.48 秒降至 8.46 秒（-48.7%）；结果 `benchmark/results_quick_carrier_dsr_phase7_20260909` |
| 2026-09-09 | Phase 7 Java telemetry RED/GREEN | native rho metrics → `CarrierSolveResult` → `CarrierPlanTelemetry` CSV | GREEN：新 protected mapping test 1/1；Phase 6 persistent session 2/2、native wrapper 4/4、failure metrics 1/1、DSR telemetry 2/2 和 loader 4/4 回归通过；rho API 使用 additive 子接口，旧 Phase 6 实现自然返回 unsupported，未修改 protected tests |
| 2026-09-09 | Phase 7 real SAZ1 RED | Carrier MOVE 后交还普通控制，随后普通 `SpaceRelease` | 真实标准入口稳定复现 `SpaceAllocatorAggregator.released()` NPE：Carrier MOVE 更新了物理位置，却没有登记目的格子的空间生命周期；结果 `/tmp/carrier-saz1-phase7.84OXys` |
| 2026-09-09 | Phase 7 handoff GREEN | `CarrierSpaceAllocatorAggregationHandoffTest` + 生产事件链 | 新 protected test 先以同一 NPE RED；`DataAggregator` 处理 `CarrierStepCompleted(MOVE)`，关闭来源格并登记目的格后 GREEN，普通 `MorePath` 与 joint executor 回归均通过 |
| 2026-09-09 | Phase 7 terminal telemetry RED/GREEN | FC 最终 flush + 跨 prefix session 指标累计 | `FCFinalEventLoggerFlushTest` 先读到空文件；`CarrierDsrSessionTelemetryAggregationTest` 先错误读到最后一拍 0.5 ms。修复后 FC 返回前 flush，终态记录保留第一次成功计划的 timing/quality，并累计每次 solve 的 native/cache/rho 指标；两个新 protected tests 均 GREEN |
| 2026-09-09 | Phase 7 final Brazil regression | Code-Labyrinth `brazil-build release` | GREEN：完整 package 日志 `.build-logs/brazil-build-20260909-phase7-saz1-final-release.log`，10 秒；新增 handoff、final flush、session aggregation 和 rho mapping tests 均进入 release |
| 2026-09-09 | Phase 7 verified SAZ1 multi-prefix | seed 0、8 drives、2 targets、183 simulated seconds、10 秒 native limit，标准 `FlexCCILegacyExecutor` | GREEN：无 ERROR/Exception；2 个 DIG 在模拟时间 180 完成；首次解 4.0000 ms、首次可交付 8668.8135 ms、初始计划 19 拍、执行 19 拍/18 次续算。累计 `cache_hits=95298`、`changed_pair_edges=36`、`rho_full=73474`、`rho_repairs=13963`、`rho_zero_row_reuses=4261`、`rho_changed_rows=19797`；正式 CSV 无需手动 flush，结果 `/tmp/carrier-saz1-phase7-verified.RVr1Pp` |
| 2026-09-09 | Phase 7 pre-commit regression | Code-Labyrinth `brazil-build release` | GREEN：完整 package 日志 `.build-logs/brazil-build-20260909-phase7-precommit-final.log`，10 秒 |
| 2026-09-09 | Phase 7 final review/commit | JNA rho ABI、跨 prefix telemetry、Carrier MOVE 空间生命周期、terminal logger flush | 独立 GPT-5.6 Sol/low：APPROVE，无 blocking finding；算法提交并推送 `76d54bf carrier: repair rho assignment incrementally`，LMS 提交并推送 `2587828 lms: report incremental Carrier rho telemetry` |
| 2026-09-09 | Phase 8 design audit | fixed upper pod、显式有向图、外部时空 commitment、block lease 与 normal Pick/Stow eligibility | 完成：所有机制继续进入现有 `TAPFPlanner::solve()`、PIBT 和 `apply_ops()`；确认不能直接删除全局 pause，必须先让 native 表达固定上层占用和外部交通，再缩小 Java lease；规范写入 `design_final.md` §28.7 |
| 2026-09-10 | Phase 8.1 fixed upper RED | 固定 pod 的 schema、root legality、MOVE/LIFT/DROP、规划绕行和 BR upper compiler | 新增 protected tests 首先暴露 BR upper distance cache 仍使用原 lower grid：错误实现会探索 9 个节点，而正确的固定障碍拓扑只需探索 8 个节点 |
| 2026-09-10 | Phase 8.1 fixed upper GREEN | `fixed_upper_cells` 接入现有 Carrier execution path | GREEN：固定格不进入 movable shelf、tau、rho 或 Task-BR；空载机器人可从下层经过，携架机器人不得进入，且不能 LIFT/DROP；BR upper cache 改用同一份 upper-deck grid，错误的 phantom shortcut 不再生成 |
| 2026-09-10 | Phase 8.1 targeted regression | fixed upper、BR upper 和全部 Carrier C ABI tests | GREEN：新增 fixed-upper tests 6/6，BR upper 16/16，C ABI 11/11；此前完整 C++ regression 453/453 |
| 2026-09-10 | Phase 8.1 clean quick | 固定 quick 77，10 秒、seed 0、unit weights、14 workers | GREEN：47/77，solver runtime 总和 587.7 秒，墙钟 47.4 秒；与 Phase 7 的成功集合、status、plan hash、makespan、SOC、work、首次解质量和最终解质量逐 case 零差异；结果 `benchmark/results_quick_carrier_dsr_phase8a_clean2_20260910`。一轮与其他 full benchmark 并发的 46/77 结果因 CPU contention 作废，不用于验收 |
| 2026-09-10 | Phase 8.2 core RED | explicit undirected adjacency、degree > 4、Graph 直传和 persistent schema | 预期编译失败：`DDGrid` 尚无显式边/outgoing/incoming API；加入底层动态容器后，高出度 case 继续稳定失败于旧 `int out[4]` adapter，证明生产调用点仍会截断 |
| 2026-09-10 | Phase 8.2 core GREEN | DDGrid → Lazy BFS → Graph → TAPFPlanner/dd_view → apply_ops | GREEN：显式边完全替换坐标推导；6 邻居不截断，非坐标长边可执行，Graph 与 DD transition 消费同一边集，默认矩形保持 DD 的 down/up/right/left 和 Graph 的 legacy 顺序；core tests 4/4，高出度 task-agent PIBT 1/1 |
| 2026-09-10 | Phase 8.2 C ABI RED/GREEN | 对称 CSR adjacency setter | RED：header 缺少 `carrier_lacam_set_undirected_adjacency`；GREEN：setter 在 grid 副本上校验 offsets、destination、墙、重复边和对称性后原子替换 topology，未调用时保持默认矩形；新增 C ABI 2/2，原有 C ABI 11/11，符号已从 `libcarrier_lacam.so` 导出 |
| 2026-09-10 | Phase 8.2 related regression | topology、lazy distance、repair、storage transfer、fixed upper、DD/TAPF/LaCAM planner | GREEN：相关定向回归 42/42；生产代码已无固定 `[4]`、`Candidates[5]` 或 `MAX_DEG=4` 调用，旧 fixed-buffer adapter 只为现有矩形单测兼容保留。按 `rules.md` 的原始 benchmark 触发条件，本阶段不改变默认算法决策，因此没有重复运行原始 quick benchmark |
| 2026-09-10 | Phase 8.3 directed RED | asymmetric MOVE、reverse distance、Graph predecessor、target feasibility 和 directed C ABI | 预期编译失败：缺少 `set_directed_edges` 与 directed CSR setter；旧距离场也没有独立 incoming 语义 |
| 2026-09-10 | Phase 8.3 directed GREEN | outgoing operator contract + incoming reverse-distance contract | GREEN：MOVE、route、storage transfer、repair successor 和 PIBT 只沿 outgoing；DD/TAPF distance 从目标沿 incoming/predecessor 扩张；`finalize()` 按每个 target 的 start→goal 有向可达过滤 goal set；post-processing 按 from→to 校验边。核心 3/3、directed C ABI 1/1、directed schema 1/1 |
| 2026-09-10 | Phase 8.3 review RED | directed task-agent PIBT forced swap | 独立 Sol/low 发现旧 swap shortcut 会在只有 `0→1` 时强制 occupied agent 执行不存在的 `1→0`；新增 `test_dd_directed_swap` 先稳定 RED，并报告具体非法 arc `1→0` |
| 2026-09-10 | Phase 8.3 swap GREEN | swap candidate outgoing-arc legality | `swap_possible_and_required()` 只有在 swap agent 的当前位置确实存在通往 pusher 原位置的 outgoing arc 时才返回候选；默认无向图中该条件恒成立。新增 directed tests 6/6，directed/undirected adjacency、C ABI、repair、post-processing、TAPF compatibility、fixed upper 和 lazy distance 相关回归 49/49；按 `rules.md` 未运行与本次输入扩展无关的原始 benchmark |
| 2026-09-10 | Phase 8.3 re-review | 完整 directed topology diff 与 forced-swap 修复 | 独立 GPT-5.6 Sol/low：APPROVE；blocking finding 为 none，确认 MOVE/swap 使用 outgoing、距离使用 incoming/predecessor、新回归真实覆盖旧 bug，且无 parallel planner、fallback 或 testcase-specific hack |
| 2026-09-10 | Phase 8.4 commitment RED | external lower/upper occupancy、reverse-edge swap、RELEASE/HOLD_LAST、等待后通行和 empty commitment | 首轮按预期缺少 commitment 类型、API、absolute tick 和 time-aware CLOSED；实现后新增核心 5/5 GREEN |
| 2026-09-10 | Phase 8.4 delivery GREEN | ordinary/macro/reference/repair/final replay 共享 absolute-tick transition contract | 新增 delivery 2/2 GREEN：repair 不会删除外部交通要求的 WAIT，cold solver 返回计划可由同一 commitment 权威重放；shelf-free commitment 路径发现空 `kappa` 崩溃后由已有测试复现并修复 |
| 2026-09-10 | Phase 8.4 compatibility regression | 空 commitment 与旧 incremental continuation 自然退化 | `test_dd_incremental_session` 首轮暴露 continuation 在无 commitment 时错误使用 tick=-1，修复后 4/4；`test_tapf_compat` 7/7、120.463 秒；未运行原始 benchmark |
| 2026-09-10 | Phase 8.5 session RED/GREEN | persistent snapshot、origin advance 和 explicit rebase | 新增 session 2/2 GREEN：commit 一拍后 origin 从 0 到 1，续算不再重复等待；普通 rebase 在 active commitment 下拒绝，显式 keep/replace 后成功 |
| 2026-09-10 | Phase 8.5 C ABI RED/GREEN | 独立 spacetime setter、origin getter、explicit rebase | 新增 C ABI 2/2 GREEN；setter 使用 frame offsets 与 `(tick,from,to)` edge triples，不改变 `set_state()` 身份数组；导出三个新符号，旧 C ABI/session/rebase tests 保持 GREEN |
| 2026-09-10 | Phase 8.5 C boundary regression | hostile edge tick 与 absolute origin overflow | `INT_MAX` edge tick 首先稳定触发 native SIGSEGV，改为索引前无加法范围检查后 GREEN；review 又发现 `int64_t` origin/step 加法可能 UB，新增 C++ 3/3 + C ABI origin 1/1 RED/GREEN，并用统一 checked tick addition 覆盖 search、rollout、reference、rewire、repair、final replay 和 session |
| 2026-09-10 | Phase 8.4/8.5 related regression | topology、commitment、repair、reference、rewire、session 与全部 Carrier C ABI | 相关定向测试全部 GREEN，`git diff --check` 通过；YAML-free portable `carrier_lacam` 独立构建通过，目录 `/tmp/dd-lacam-portable-phase85.CwpRTm`；按 `rules.md` 未运行与新增输入/接口无关的原始 benchmark |
| 2026-09-10 | Phase 8.4/8.5 final review | external commitment、absolute tick、session origin、C ABI 与空 commitment 兼容性 | 独立 GPT-5.6 Sol/low：`APPROVE`，无 blocking finding；reviewer 定向运行 53/53 GREEN，确认没有 parallel planner、fallback、testcase/seed hard-code 或原算法语义变化；按规则未运行原始 benchmark |

## 6. 当前下一步

1. 在最新 Code-Labyrinth workspace
   `/local/home/yimint/brazil-workspaces/carrier-labyrinth-20260909/src/Skkiesel_LightweightMovementSimulator`
   实现 Phase 8.6 Java adapter、external commitment 导出、block lease 和 normal Pick/Stow 并发。
2. 全部阶段结束后再按
   gate 运行最终 quick、Sol/high review、获批 full 518，并生成最终汇报网页。
