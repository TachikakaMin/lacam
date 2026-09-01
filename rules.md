**最高优先级约束：新算法必须建立在现有 LaCAM-TAPF 代码和算法流程之上进行增量式扩展，禁止另起炉灶。**

* 必须沿用现有 LaCAM-TAPF 的核心数据结构、search/control flow、节点扩展、状态表示和规划逻辑，在原有 execution path 上增加 `design.md` 中的新机制。
* 禁止实现平行 planner、第二套 search pipeline、独立算法框架，或通过大量独立函数/文件绕开原有 LaCAM-TAPF 逻辑。
* 可以增加必要 helper、data structure 或局部模块，但必须直接服务于并嵌入原 LaCAM-TAPF 主流程。
* 开始编码前，先明确 `design.md` 中每个新增机制具体对应现有 LaCAM-TAPF 的哪些修改位置，并据此更新 design。
* 避免无必要的大规模重构。

必须保持以下 **semantic invariant**：

> 当 testcase 不涉及 rack 的 pick/place 行为时，新代码应自然退化为原始 LaCAM-TAPF；运行原有 LaCAM-TAPF testcase 时，其算法行为和结果应与修改前保持一致。

禁止通过 feature flag、legacy mode、fallback、单独调用旧算法、检测“没有 pick/place”后直接切换 baseline 等方式规避该要求。兼容性必须自然来自新算法对原 LaCAM-TAPF 的保守扩展。

开发流程严格遵循：

**test → RED → implementation → GREEN → benchmark → regression test → debug**

* 使用 baseline 原始 benchmark 流程，新算法与 baseline 必须使用相同 dataset、metric、seed、success/failure semantics 和运行配置；每个 testcase 严格限时 **10s**。
* 实现开始前固定一小组 representative benchmark cases 用于开发阶段快速测试，不得根据算法表现随意更换 testcase。
* 每个重要 function/module/behavior 修改前，先写对应 test，并先运行确认其因为功能尚未实现而失败（RED）；然后再实现最小必要修改使其通过（GREEN）。
* 实现过程中，小步运行相关 tests + 固定的小规模 benchmark subset，不需要每次运行完整 benchmark。
* 发现 bug、incorrect behavior、crash、unexpected timeout 或 regression 时，**禁止直接修改 implementation**：先写 regression test 固化并复现问题，确认失败后再 debug；修复后保留该 test，并重新运行相关 tests 和 benchmark cases。
* 优先测试稳定的算法行为和接口，避免为了 TDD 而过度测试无意义的内部实现细节。

### Protected tests / benchmarks

本任务中新增加的 unit tests、integration tests、regression tests、benchmark testcases 和 benchmark expected behavior，一旦创建即视为 **protected**。

后续如果需要修改这些 test 或 benchmark testcase，主 agent 不得直接修改。必须先启动独立 subagent：

* model: **GPT-5.6 Sol**
* reasoning: **high**
* 独立阅读 `design.md`、相关原代码、当前 implementation、原 test 和 proposed change
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

1. 每一处新增或修改代码都能对应 `design.md` 中的具体设计；
2. 所有新增代码都在真实 execution path 中被使用；
3. 没有 dead code、重复实现或平行 pipeline；
4. 修改集中在原 LaCAM-TAPF 真正需要扩展的位置；
5. 没有与 design 无关的算法语义变化；
6. 没有为了通过 test/benchmark 加入的特殊代码；
7. 如果某段新增代码无法解释为什么是新算法所必需的，应删除或重新设计。

### Final validation

完成后必须：

* 运行原 LaCAM-TAPF tests，重点验证无 pick/place 时的 backward compatibility；
* 运行所有新增和相关 existing tests；
* 运行完整 benchmark，每个 testcase 严格限时 **10s**；
* 使用相同配置、seed、资源分配和并行策略比较 baseline 与新算法；
* review 最终 `git diff`，逐项确认主要新增代码的必要性；
* 检查不存在 parallel implementation、fallback、benchmark-specific hack 或无效代码。

最后汇报：

* 主要算法修改；
* 修改了哪些原 LaCAM-TAPF execution paths；
* 新增了哪些 tests / regression tests；
* baseline vs 新算法 benchmark 结果；
* backward compatibility 结果；
* 最终 git diff 中主要新增代码的作用；
* 尚存在的 regression、semantic difference 或未解决问题。
