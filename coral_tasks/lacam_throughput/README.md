# CORAL task: LaCAM lifelong throughput

This task lets CORAL agents optimize this repository's C++ lifelong TAPF
throughput on the fixed Symbotic scenario from
`hl_agent/lacam_throughput_prompt_template.txt`.

Run commands from this directory so CORAL resolves `workspace.repo_path: ../..`
to the `lacam_agent` repository root.

```sh
cd /home/yimin/research/symbotic_agent/lacam_agent/coral_tasks/lacam_throughput
uv pip install -e /home/yimin/research/symbotic_agent/CORAL
coral validate .
coral start -c task.yaml
```

Useful runtime overrides:

```sh
coral start -c task.yaml agents.runtime=codex agents.model=gpt-5.5 agents.runtime_options.model_reasoning_effort=high agents.count=2 run.session=local
coral start -c task.yaml agents.runtime=claude_code agents.count=4 run.session=local
```

The default Codex configuration uses `gpt-5.5` with high reasoning effort.
All throughput ideas must originate from the local paper library. Start with
`papers/README.md`, open the relevant `papers/briefs/*.md` summary, and read
the linked paper only when section, algorithm, design-pattern, or empirical
detail is needed. Implementation plus command execution, including
build/test/benchmark and `coral eval`, should be delegated through the
`paper-grounded-subagent` skill. That skill launches nested Codex subagents with
`gpt-5.3-codex-spark` and high reasoning effort, writing reports under
`hl_agent/runs/paper_grounded_subagents/`.

The official grader builds `build/lifelong_benchmark` in each submitted
checkout and scores mean throughput. Normal `coral eval` runs a hidden held-out
set of 10 real-mode seeds selected inside the grader. `coral eval --tune` runs
tune seeds 0..9 for screening; tune scores are directionally useful but not
leaderboard-equivalent.

The grader fixes the environment/task target, but planner-side defaults such as
service commitment, shared drop-goal slots, and assignment cost mode come from
the submitted code. The benchmark CLI uses `-1` for those algorithm knobs to
avoid overriding repository defaults.

Tune eval artifacts are written to CORAL eval logs, including:

- `rows.csv`: all benchmark rows for the attempt.
- `summary.json`: parsed aggregate metrics.
- `seed_<n>.stdout.txt` and `seed_<n>.stderr.txt`: subprocess logs.
- `runner/runs/*/result.csv.trace.csv`: per-seed benchmark trajectory traces.

Real eval logs expose aggregate summary and redacted runner output only, so the
held-out seed identities and per-seed traces do not leak back to agents.

Agents should keep their own experiment notes, scripts, traces, plots, and
diagnostic summaries under `hl_agent/runs/` inside their worktree, matching the
existing high-level prompt's lab-notebook requirements.
