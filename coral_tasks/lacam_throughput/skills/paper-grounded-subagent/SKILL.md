---
name: paper-grounded-subagent
description: 强制 idea generation 基于论文，并把 implementation、diagnostics、builds、tests、benchmarks 和长时间命令委托给 Codex subagent。本 CORAL task 中每个非平凡 LaCAM throughput idea、code edit 或 command execution 都要使用。
---

# 论文支撑 Subagent

## 必须遵守的工作流

每个非平凡 throughput idea、code edit、diagnostic script、build、test、
benchmark、官方 `coral eval` 或长时间 shell command 前，都要使用这个 skill。

1. 用渐进式披露把 idea grounding 到本地论文库：
   - 先读 `coral_tasks/lacam_throughput/papers/README.md`；
   - 再读 `coral_tasks/lacam_throughput/papers/idea_categories.md`，选择 idea 的主要类别：map/guidance-weight optimization、algorithm design 或 heuristic function design；
   - 然后打开相关 `coral_tasks/lacam_throughput/papers/briefs/*.md`；
   - 只有需要章节、算法、定理、设计模式或实验观察细节时，才读链接的原始 HTML/PDF 论文。
2. 至少选择一个本地 brief 文件、它链接的论文文件；如果需要原论文，还要选择一个具体 anchor。
3. 把 implementation 和命令执行委托给下面的 helper script。
4. 决定是否运行官方 `coral eval` 前，先 review subagent report 和 diff。
5. 尽量在 experiment notes 和 `coral eval -m` 中记录 primary category、brief path、linked paper file、concrete anchor、subagent report path、files touched、commands run 和 outcome。

main CORAL agent 是 controller。它负责选择 hypotheses、启动 subagents、
review results、写简洁 notes。它不应在自己的上下文里直接做非平凡
implementation，也不应直接运行 build/test/benchmark/eval commands。

## Helper

从仓库根目录运行：

```bash
python .codex/skills/paper-grounded-subagent/scripts/run_paper_subagent.py \
  --paper "coral_tasks/lacam_throughput/papers/briefs/2308.11234.md" \
  --idea "基于 traffic-flow optimization 尝试拥堵感知 pickup assignment" \
  --task "实现最小安全候选改动，运行 focused build/tests，并写报告。"
```

helper 会先启动：

```bash
codex exec --model gpt-5.3-codex-spark -c model_reasoning_effort="high" ...
```

如果该调用因为 quota、rate-limit、capacity、billing 或 insufficient credits
类错误失败，helper 会自动重试一次：

```bash
codex exec --model gpt-5.5 -c model_reasoning_effort="medium" ...
```

logs 和请求的 report 会写到 `hl_agent/runs/paper_grounded_subagents/`。

## 委托规则

- `--paper` 必须传具体的本地 `papers/briefs/*.md` 路径；不要传 `papers/README.md`、`papers/index.md` 或原论文本身。
- 每个 subagent report 必须标明 primary category：map/guidance-weight optimization、algorithm design 或 heuristic function design。
- 每个 subagent task 只聚焦一个 idea。
- 要求 subagent 运行 focused verification commands，并总结精确 command outcomes。
- 永远不要要求 subagent 跑 `coral eval --tune`。自测时让它直接用 `tools/run_symbotic_requested_grid.py` 跑 public tune seeds，输出到 `hl_agent/runs/<experiment>/runner`，这样结果不会注册为 CORAL attempt。
- diagnostic-only、counter-only、logging-only、refactor-only、runtime-only、no-op 或 parameter-path-not-hit 改动，不要要求 direct public-seed self-tests 或普通 `coral eval`。
- 任何 direct public-seed self-test 前，必须要求证据证明 candidate 在 benchmark CLI defaults 下改变了 active algorithmic decision path。report 应说明 changed behavior signal，例如 selected targets、move ordering、assignment rows、conflict/blocking counts、local probe completed vector 或其他 trace-derived decision metric。
- 每当 retained candidate 或 progress result 要汇报给用户时，要求 subagent 跑普通 `coral eval -m "..."`；eval message 中包含 paper grounding。user-facing outcomes 必须是 real。
- candidate 失败时，保留 report 并记录 failure mode，避免后续 agents 重复。
- 不得用 paper grounding 为 benchmark-specific hard-coding、seed shortcuts、fake outputs 或 scenario changes 辩护。
