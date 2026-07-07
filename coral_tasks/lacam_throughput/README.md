# CORAL 任务：LaCAM lifelong throughput

这个任务让 CORAL agents 在固定 Symbotic 场景上优化本仓库 C++ lifelong TAPF
吞吐量，场景来自
`hl_agent/lacam_throughput_prompt_template.txt`.

请在本目录运行命令，这样 CORAL 会把 `workspace.repo_path: ../..` 解析到
`lacam_agent` 仓库根目录。

```sh
cd /home/yimin/research/symbotic_agent/lacam_agent/coral_tasks/lacam_throughput
uv pip install -e /home/yimin/research/symbotic_agent/CORAL
coral validate .
coral start -c task.yaml
```

常用 runtime overrides：

```sh
coral start -c task.yaml agents.runtime=codex agents.model=gpt-5.5 agents.runtime_options.model_reasoning_effort=high agents.count=2 run.session=local
coral start -c task.yaml agents.runtime=claude_code agents.count=4 run.session=local
```

默认 Codex 配置使用 `gpt-5.5` 和 high reasoning effort。所有 throughput ideas
都必须来自本地论文库。先读 `papers/README.md`，再用
`papers/idea_categories.md` 把 idea source 分类为 map/guidance-weight
optimization、algorithm design 或 heuristic-function design，然后打开相关
`papers/briefs/*.md` 简介；只有需要章节、算法、设计模式或实验细节时，才读
链接的原论文。implementation 和命令执行，包括 build/test/benchmark 和
`coral eval`，都应通过 `paper-grounded-subagent` skill 委托。该 skill 默认
用 `gpt-5.3-codex-spark` 和 high reasoning effort 启动 nested Codex
subagents；如果该模型没有 quota，或遇到 quota/rate-limit/capacity/billing
availability error，helper 会自动用 `gpt-5.5` 和 medium reasoning effort
重试一次。报告写到 `hl_agent/runs/paper_grounded_subagents/`。

当前论文路由总结：

- Map/guidance-weight optimization 论文提供 general edge、region、congestion、
  station-pressure 或 flow weights 方向。可以改算法侧 cost，但不能改固定官方
  map 或 task target。
- Algorithm-design 论文提供 solver mechanisms，例如 LaCAM expansion、
  target-path coupling、priority inheritance、local repair、commitment 或
  bounded-window control。
- Heuristic-function 论文提供 scoring functions、lower bounds、tie-breakers、
  local guidance、conflict estimates 或 candidate-ranking terms。

目前 real-confirmed lift 来自 heuristic-function design，grounding 是
`papers/briefs/2605.16855.md`：baseline real throughput 1.18275，immediate
occupancy/target contention ordering 1.27700，swap-friendly occupancy
exemption 1.32950。完整 paper-to-idea 表在
`papers/idea_categories.md`.

官方 grader 会在每个 submitted checkout 中 build `build/lifelong_benchmark`，
并按 mean throughput 打分。普通 `coral eval` 会运行 grader 内部选择的 10 个
hidden held-out real-mode seeds。Agent 不得用 `coral eval --tune` 自测。
agent-internal testing 必须直接用 benchmark runner 跑 public tune seeds，
这样结果不会变成 CORAL attempt：

```sh
python tools/run_symbotic_requested_grid.py \
  --binary build/lifelong_benchmark \
  --out-dir hl_agent/runs/<experiment>/runner \
  --seeds 0,1,2,3,4,5,6,7,8,9 \
  --workers 4 \
  --maps symbotic_star \
  --ks 2 \
  --slots -1 \
  --agent-counts 100 \
  --dists 50_50 \
  --durations 4 \
  --cost-modes -1 \
  --horizon 400 \
  --time-limit-sec 1.0 \
  --goal-set-size 3 \
  --release-interval 10 \
  --service-commit-agents -1
```

Agent 不得把 tune 或 real eval 浪费在 diagnostic-only、counter-only、
logging-only、refactor-only、runtime-only 或 parameter-path-not-hit 改动上。
任何 public-seed self-test 前，subagent report 或 experiment note 必须说明
active algorithmic decision path 在 benchmark CLI defaults 下已经改变，并用
local/single-seed probe 或 trace signal 证明，例如 selected targets、move
ordering、assignment rows、conflict/blocking counts、completed vector 或其他
decision-level metric。如果 completed vector 和关键 trace signals 与 parent
一致，记录为 no-op 或 instrumentation-only，不要跑 public-seed self-test 或
real。每次 agent 向用户汇报 retained candidate 或 progress，都必须跑普通
`coral eval -m "..."`，让 reported score 是 hidden real result。

grader 固定 environment/task target，但 service commitment、shared drop-goal
slots、assignment cost mode 等 planner-side defaults 来自 submitted code。
benchmark CLI 对这些 algorithm knobs 使用 `-1`，避免覆盖仓库默认值。

direct public-seed self-test artifacts 应写到 `hl_agent/runs/`，包括：

- `rows.csv`：该 attempt 的所有 benchmark rows。
- `summary.json`：解析后的 aggregate metrics。
- `seed_<n>.stdout.txt` 和 `seed_<n>.stderr.txt`：subprocess logs。
- `runner/runs/*/result.csv.trace.csv`：per-seed benchmark trajectory traces。

real eval logs 只暴露 aggregate summary 和 redacted runner output，避免
held-out seed identities 和 per-seed traces 泄漏给 agents。

Agent 应把自己的 experiment notes、scripts、traces、plots 和 diagnostic
summaries 放到 worktree 内的 `hl_agent/runs/`，与现有 high-level prompt 的
lab-notebook 要求保持一致。
