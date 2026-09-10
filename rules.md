**最高优先级约束：新算法必须建立在现有 LaCAM-TAPF 代码和算法流程之上进行增量式扩展，禁止另起炉灶。**

* 必须沿用现有 LaCAM-TAPF 的核心数据结构、search/control flow、节点扩展、状态表示和规划逻辑，在原有 execution path 上增plan中的新机制。
* 禁止实现平行 planner、第二套 search pipeline、独立算法框架，或通过大量独立函数/文件绕开原有 LaCAM-TAPF 逻辑。
* 可以增加必要 helper、data structure 或局部模块，但必须直接服务于并嵌入原 LaCAM-TAPF 主流程。
* 开始编码前，先明确plan中每个新增机制具体对应现有 LaCAM-TAPF 的哪些修改位置，并据此更新 design；两者冲突时以 `design_final.md` 的算法边界为准。
* 避免无必要的大规模重构。

必须保持以下 **semantic invariant**：

> 当 testcase 不涉及 rack 的 pick/place 行为时，新代码应自然退化为原始 LaCAM-TAPF；运行原有 LaCAM-TAPF testcase 时，其算法行为和结果应与修改前保持一致。

禁止通过 feature flag、legacy mode、fallback、单独调用旧算法、检测“没有 pick/place”后直接切换 baseline 等方式规避该要求。兼容性必须自然来自新算法对原 LaCAM-TAPF 的保守扩展。

开发流程严格遵循：

**test → RED → implementation → GREEN → 按触发条件运行 benchmark → regression test → debug**

* 使用 baseline 原始 benchmark 流程，新算法与 baseline 必须使用相同 dataset、metric、seed、success/failure semantics 和运行配置；每个 testcase 严格限时 **10s**。
* 实现开始前固定一小组 representative benchmark cases 用于开发阶段快速测试，不得根据算法表现随意更换 testcase。
* 每个重要 function/module/behavior 修改前，先写对应 test，并先运行确认其因为功能尚未实现而失败（RED）；然后再实现最小必要修改使其通过（GREEN）。
* 实现过程中，小步运行相关 tests + 固定的小规模 benchmark subset，不需要每次运行完整 benchmark。
* 发现 bug、incorrect behavior、crash、unexpected timeout 或 regression 时，**禁止直接修改 implementation**：先写 regression test 固化并复现问题，确认失败后再 debug；修复后保留该 test，并重新运行相关 tests 和 benchmark cases。
* 优先测试稳定的算法行为和接口，避免为了 TDD 而过度测试无意义的内部实现细节。

### 原始 benchmark 的触发条件

* **如果本次修改没有改变原有算法，就禁止运行原始 benchmark。** 不得仅仅为了“流程完整”“确认一下”或生成新报告而运行。这类修改包括只增加新输入表达、新场景语义、接口、adapter、schema validation、telemetry、构建或文档，并且在不使用新增输入时，原 LaCAM-TAPF 的搜索、状态转移、cost、heuristic、assignment 和 tie-breaking 都保持不变。
* 这类修改只运行新增功能的定向 unit/integration/regression tests，以及确实覆盖本次接入行为的 Labyrinth simple benchmark；不得为了流程完整而重复运行与修改无关的原始 quick/full benchmark。
* 只有修改会影响原 testcase 使用的搜索/control flow、状态转移、cost、heuristic、assignment、邻接语义、tie-breaking，或者相关 regression test 表明旧行为可能改变时，才运行原始 quick benchmark。
* 如果本次工作始终没有改变原算法或默认行为，则原始 quick benchmark 和 full benchmark 都不运行。只有确实触发原始 benchmark 的修改，full benchmark 才在全部实现完成并满足下述最终 gate 后运行一次；不能因为阶段性非算法改动而提前或重复运行。

### Quick / full benchmark 层级

* 本节的 quick/full 只指 **benchmark 运行层级**，不限制 unit test、integration/regression test、生成器测试、静态检查或 authoritative validator 验证；这些代码测试在开发期间仍应按 TDD 正常运行。
* 当前冻结的 `benchmark/release_benchmark.json`（68 个原始 BRaP-pool case + 9 个 warehouse-block case，共 **77 cases**）是唯一的开发期 **quick benchmark**。用户口中的“原本 60 多个测试”以仓库当前固定的 77-case manifest 为准。
* **full benchmark** 固定为 quick benchmark 的全部 77 cases，加上 432 个随机配对 warehouse cases 和 9 个 40×40 dense-channel block-edge cases，共 **518 cases**。full 必须是 quick 的严格超集，不得删除、替换或重命名 quick 中的任何 testcase。
* 代码开发、debug、局部修复和 review 前验证期间，benchmark 最多只能运行 quick benchmark；禁止提前运行 full benchmark，也禁止用 full benchmark 的结果反向挑选 seed、修改 testcase 或调参。
* 只有在实现和全部相关代码测试完成、quick benchmark 通过、最终 diff 已清理，并获得独立 **GPT-5.6 Sol / high** reviewer 明确 `APPROVE` 后，才允许运行 full benchmark。
* `benchmark/run_benchmark.py --benchmark-tier quick` 是开发期固定入口。`--benchmark-tier full` 必须提供与 `benchmark/full_benchmark.json` SHA-256 绑定的独立 review approval JSON，否则 runner 必须拒绝启动。
* quick benchmark 和 full benchmark 使用完全相同的 method、10s timeout、solver seed、objective weights、parallelism、validator、metric 与 success/failure semantics；两者唯一允许的差异是 testcase 成员数量。
* 新增 testcase 一经生成即属于 protected full benchmark。不得根据 Planner 的成功率、makespan、SOC 或其他表现删除、替换或重新抽样。

### Protected tests / benchmarks

本任务中新增加的 unit tests、integration tests、regression tests、benchmark testcases 和 benchmark expected behavior，一旦创建即视为 **protected**。

后续如果需要修改这些 test 或 benchmark testcase，主 agent 不得直接修改。必须先启动独立 subagent：

* model: **GPT-5.6 Sol**
* reasoning: **high**
* 独立阅读原本plan、`design_final.md`、相关原代码、当前 implementation、原 test 和 proposed change
* 明确输出 `APPROVE` 或 `REJECT`，并说明理由

只有得到明确 `APPROVE` 后才能修改 protected test/benchmark；如果 `REJECT`，必须保持测试不变并修改 implementation。

Reviewer 重点检查：

* test 是否真的与 design/specification 冲突；
* 是否在弱化 correctness requirement；
* 是否只是为了让当前 implementation 通过；
* 是否改变 benchmark difficulty、semantics、metric 或 evaluation protocol；
* 是否存在 benchmark overfitting。

默认假设 **implementation 有问题，而不是 test 有问题**。

禁止通过以下方式让算法通过：

* 放宽 assertion；
* 修改 benchmark semantics；
* 更换表现不利的 testcase；
* hard-code benchmark instance；
* testcase-specific hacks；
* fallback 到原算法；
* 添加只为了绕过测试的特殊分支。

### Experiment / benchmark execution

实验应尽量根据机器的**实际物理 CPU cores**并行运行独立 testcase，以缩短总体实验时间。

* 运行前检测 physical core 数量，而不是只看 logical threads。
* 原则上始终保留 **1–2 个物理核心**给系统和其他任务，其余核心可用于并行 benchmark。
* 不要盲目一次启动最大数量任务；实验过程中持续监控 CPU utilization、load、memory、swap 和 testcase runtime。
* 如果出现 CPU contention、memory pressure、swap、系统负载异常或明显 runtime inflation，应主动降低并行度。
* 独立 testcase 可以并行，但不要在单个 testcase 内通过额外 CPU oversubscription 改变算法运行条件。
* 单个 testcase 的 **10s timeout 永远不因并行运行而改变**。
* Baseline 和新算法必须使用相同的资源分配、并行度策略和 benchmark protocol，确保结果公平可比。

### Git diff / semantic control

实现过程中持续检查 `git diff`，确保：

1. 每一处新增或修改代码都能对应 plan 与 `design_final.md` 中的具体设计；
2. 所有新增代码都在真实 execution path 中被使用；
3. 没有 dead code、重复实现或平行 pipeline；
4. 修改集中在原 LaCAM-TAPF 真正需要扩展的位置；
5. 没有与 design 无关的算法语义变化；
6. 没有为了通过 test/benchmark 加入的特殊代码；
7. 如果某段新增代码无法解释为什么是新算法所必需的，应删除或重新设计。

### Final validation

完成后必须：

* 运行所有新增和与本次修改直接相关的 existing tests；
* 只有本次修改影响原算法或默认行为时，才运行原 LaCAM-TAPF regression tests，重点验证无 pick/place 时的 backward compatibility；
* 只有满足“原始 benchmark 的触发条件”及 full benchmark gate 时，才运行完整 benchmark，每个 testcase 严格限时 **10s**；
* 只有实际运行 baseline 与新算法 benchmark 时，才要求使用相同配置、seed、资源分配和并行策略进行比较；
* review 最终 `git diff`，逐项确认主要新增代码的必要性；
* 检查不存在 parallel implementation、fallback、benchmark-specific hack 或无效代码, 并且清理.

最后汇报：

* 主要算法修改；
* 修改了哪些原 LaCAM-TAPF execution paths；
* 新增了哪些 tests / regression tests；
* 实际运行过的 benchmark 结果；如果因未修改原算法而没有运行原始 benchmark，应明确写“未触发”，不得为了补齐报告而补跑；
* backward compatibility 结果；
* 最终 git diff 中主要新增代码的作用；
* 尚存在的 regression、semantic difference 或未解决问题。
* 制作一个汇报网页, 配上例子来进行最终汇报, 里面用词需要符合中文母语人士, 让大一新生也能看懂,同时不丢失细节而且尽可能不要太长, 使用独立的subagent来review网页确保所有东西都是正常
