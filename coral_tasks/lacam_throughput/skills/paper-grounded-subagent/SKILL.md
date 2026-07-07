---
name: paper-grounded-subagent
description: Enforces paper-grounded idea generation and delegates implementation, diagnostics, builds, tests, benchmarks, and long-running commands to a Codex subagent. Use for every non-trivial LaCAM throughput idea, code edit, or command execution in this CORAL task.
---

# Paper-Grounded Subagent

## Required Workflow

Use this skill before every non-trivial throughput idea, code edit, diagnostic
script, build, test, benchmark, official `coral eval`, or long-running shell
command.

1. Ground the idea in the local paper library with progressive disclosure:
   - first read `coral_tasks/lacam_throughput/papers/README.md`;
   - then read `coral_tasks/lacam_throughput/papers/idea_categories.md`
     and choose the idea's primary category: map/guidance-weight optimization,
     algorithm design, or heuristic function design;
   - then open a relevant `coral_tasks/lacam_throughput/papers/briefs/*.md`;
   - only then read the linked original HTML/PDF paper if section, algorithm,
     theorem, design-pattern, or empirical-observation detail is needed.
2. Select at least one local brief file, its linked paper file, and one concrete
   anchor when the original paper is needed.
3. Delegate implementation and command execution to the helper script below.
4. Review the subagent report and diff before deciding whether to run official
   `coral eval`.
5. Record the primary category, brief path, linked paper file, concrete anchor,
   subagent report path, files touched, commands run, and outcome in experiment
   notes and in `coral eval -m` when practical.

The main CORAL agent is the controller. It should choose hypotheses, launch
subagents, review results, and write concise notes. It should not directly
perform non-trivial implementation or run build/test/benchmark/eval commands in
its own context.

## Helper

Run from the repository root:

```bash
python .codex/skills/paper-grounded-subagent/scripts/run_paper_subagent.py \
  --paper "coral_tasks/lacam_throughput/papers/briefs/2308.11234.md" \
  --idea "Try congestion-aware pickup assignment inspired by traffic-flow optimization" \
  --task "Implement the smallest safe candidate, run focused build/tests, and write a report."
```

The helper first launches:

```bash
codex exec --model gpt-5.3-codex-spark -c model_reasoning_effort="high" ...
```

If that call fails with a quota, rate-limit, capacity, billing, or insufficient
credits style error, the helper automatically retries once with:

```bash
codex exec --model gpt-5.5 -c model_reasoning_effort="medium" ...
```

It writes logs and the requested report under
`hl_agent/runs/paper_grounded_subagents/`.

## Delegation Rules

- Always pass a specific local `papers/briefs/*.md` path with `--paper`; do not
  pass `papers/README.md`, `papers/index.md`, or an original paper directly.
- Ensure each subagent report labels the primary category as
  map/guidance-weight optimization, algorithm design, or heuristic function
  design.
- Keep each subagent task focused on one idea.
- Ask the subagent to run focused verification commands and summarize exact
  command outcomes.
- Ask the subagent to run `coral eval --tune` or normal `coral eval` when the
  candidate is ready to score; include the paper grounding in the eval message.
- If a candidate fails, keep the report and record the failure mode so later
  agents do not repeat it.
- Do not use paper grounding to justify benchmark-specific hard-coding, seed
  shortcuts, fake outputs, or scenario changes.
