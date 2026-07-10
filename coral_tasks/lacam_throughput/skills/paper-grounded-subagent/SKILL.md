---
name: paper-grounded-subagent
description: 为论文支撑的 LaCAM throughput 工作提供可选的 coding subagent 隔离、模型建议、日志和报告工作流。适合复杂实现、diagnostics、builds、tests、benchmarks、长时间命令或大输出，是否使用由 main agent 自主决定。
---

# 论文支撑 Subagent

## 建议工作流

复杂 throughput idea、code edit、diagnostic script、build、test、benchmark、
官方 `coral eval`、长时间 shell command 或大输出任务适合使用这个 skill，以免
污染 main context window；但不强制。main agent 根据复杂度、延迟、quota、工具和
当前 context 自主决定直接执行还是委托。

1. 用渐进式披露把 idea grounding 到本地论文库：
   - 先读较短的 `coral_tasks/lacam_throughput/papers/idea_categories.md`，选择 idea 的主要类别：map/guidance-weight optimization、algorithm design 或 heuristic function design；
   - 然后打开相关 `coral_tasks/lacam_throughput/papers/briefs/*.md`；
   - 只有需要章节、算法、定理、设计模式或实验观察细节时，才读链接的原始 HTML/PDF 论文。
   - 较长的 `papers/README.md` 只用于局部检索相关条目，不要默认全文加载。
2. 至少选择一个本地 brief 文件、它链接的论文文件；如果需要原论文，还要选择一个具体 anchor。
3. 如果委托有助于隔离 context、并行或执行长任务，使用下面的 helper script；否则 main agent 可以直接完成。
4. 使用 subagent 时，在决定下一步或运行官方 `coral eval` 前 review report 和 diff。
5. 持续在 experiment notes 和运行目录中记录 primary category、brief path、linked paper file、concrete anchor、subagent report path、files touched、commands run、outcome、当前状态和下一步。

main CORAL agent 对是否使用 subagent、选择什么模型和如何执行负最终责任。鼓励
把复杂实现、长命令和大输出隔离到 subagent，但 main agent 可以直接做
implementation 或运行 build/test/benchmark/eval commands。

## Helper

从仓库根目录运行：

```bash
python .codex/skills/paper-grounded-subagent/scripts/run_paper_subagent.py \
  --paper "coral_tasks/lacam_throughput/papers/briefs/2308.11234.md" \
  --idea "基于 traffic-flow optimization 尝试拥堵感知 pickup assignment" \
  --task "实现最小安全候选改动，运行 focused build/tests，并写报告。"
```

Codex 路线下，建议默认模型是：

```bash
codex exec --model gpt-5.3-codex-spark -c model_reasoning_effort="high" ...
```

建议 fallback 是：

```bash
codex exec --model gpt-5.6-luna -c model_reasoning_effort="xhigh" ...
```

Claude Code 路线下，建议默认模型可这样显式指定：

```bash
python .codex/skills/paper-grounded-subagent/scripts/run_paper_subagent.py \
  --runtime claude \
  --claude-model claude-opus-4-8 \
  --claude-effort medium \
  --paper "coral_tasks/lacam_throughput/papers/briefs/2308.11234.md" \
  --idea "基于 traffic-flow optimization 尝试拥堵感知 pickup assignment" \
  --task "实现最小安全候选改动，运行 focused build/tests，并写报告。"
```

logs 和请求的 report 会写到 `hl_agent/runs/paper_grounded_subagents/`。
这些模型、effort 和 runtime 都只是建议。agent 可以按任务性质、可用性、速度、
能力、quota 或 rate limit 自动选择其他可用配置，无需等待用户确认。

## 委托规则

- `--paper` 必须传具体的本地 `papers/briefs/*.md` 路径；不要传 `papers/README.md`、`papers/index.md` 或原论文本身。
- 每个 subagent report 必须标明 primary category：map/guidance-weight optimization、algorithm design 或 heuristic function design。
- 每个 subagent task 只聚焦一个 idea。
- 要求 subagent 运行 focused verification commands，并总结精确 command outcomes。
- subagent 开始前写 checkpoint，记录 parent commit、hypothesis、计划、风险和验证信号；运行中及时记录重要命令结果、artifacts、changed files、commit/eval attempt、决策和下一步。预计自动 compact、重启或切换 agent 前再次 checkpoint，resume 后先读日志。
- 永远不要要求 subagent 跑 `coral eval --tune`。自测时让它直接用 `tools/run_symbotic_requested_grid.py` 跑 public tune seeds，输出到 `hl_agent/runs/<experiment>/runner`，这样结果不会注册为 CORAL attempt。
- diagnostic-only、counter-only、logging-only、refactor-only、runtime-only、no-op 或 parameter-path-not-hit 改动，不要要求 direct public-seed self-tests 或普通 `coral eval`。
- 任何 direct public-seed self-test 前，必须要求证据证明 candidate 在 benchmark CLI defaults 下改变了 active algorithmic decision path。report 应说明 changed behavior signal，例如 selected targets、move ordering、assignment rows、conflict/blocking counts、local probe completed vector 或其他 trace-derived decision metric。
- 每个已定形、可 build/test 且输出 valid 的行为改变型 candidate，无论 local/public 结果改善或退化，都必须先 commit 并跑普通 `coral eval -m "..."`，之后才能 retain、revert、丢弃或 pivot。回退到已有 official eval 的旧状态时引用已有结果，不为回退本身重复 benchmark/eval。
- candidate 失败时，保留 report 并记录 failure mode，避免后续 agents 重复。
- 不得用 paper grounding 为 benchmark-specific hard-coding、seed shortcuts、fake outputs 或 scenario changes 辩护。
