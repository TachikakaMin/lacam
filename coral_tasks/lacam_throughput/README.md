# CORAL 任务：LaCAM lifelong throughput

这个任务让 CORAL agents 在固定 Symbotic 场景上优化本仓库 C++ lifelong TAPF
吞吐量。CORAL 所需规则以 `task.yaml` 为准；不要默认完整读取
`hl_agent/lacam_throughput_prompt_template.txt`，该文件仅用于 standalone
长程实验，确有需要时只读取相关小节。

启动脚本会自动切换到本任务目录，使 CORAL 将 `workspace.repo_path: ../..`
解析到 `lacam_agent` 仓库根目录。

```sh
cd /home/yimin/research/symbotic_agent/lacam_agent/coral_tasks/lacam_throughput
uv pip install -e /home/yimin/research/symbotic_agent/CORAL
coral validate .
# 必须显式选择 runtime：
scripts/start_coral.sh codex
# 或：
scripts/start_coral.sh claude

# 安全恢复指定 run（必须显式提供 runtime 和 RUN_ID）：
scripts/start_coral.sh resume codex 2026-07-09_003415
scripts/start_coral.sh resume claude 2026-07-09_003415
```

常用 runtime overrides：

```sh
CORAL_AGENTS=2 scripts/start_coral.sh codex
CORAL_AGENTS=2 scripts/start_coral.sh claude

# 等价的裸 coral 命令：
coral start -c task.yaml agents.runtime=codex agents.model=gpt-5.6-sol agents.runtime_options.model_reasoning_effort=high agents.count=1 run.session=local
coral start -c task.yaml agents.runtime=claude_code agents.model=fable agents.runtime_options.model_reasoning_effort=high agents.count=1 run.session=local
```

默认 Codex 配置使用 `gpt-5.6-sol` 和 high reasoning effort。Claude Code 配置使用
main model `fable` / high policy。启动脚本不再设置 coding subagent model；
由 agent 根据任务、可用性和 quota 自主选择。若委托 Codex subagent，建议使用
`gpt-5.3-codex-spark` / high；没有额度或不可用时 fallback 到
`gpt-5.6-luna` / xhigh。agent 仍可以根据任务性质、可用性、速度、能力、quota 或 rate
limit 自动选择其他可用模型、effort 或 runtime。所有 throughput ideas
都必须来自本地论文库。先读较短的 `papers/idea_categories.md`，把 idea source 分类为 map/guidance-weight
optimization、algorithm design 或 heuristic-function design，然后打开相关
`papers/briefs/*.md` 简介；只有需要章节、算法、设计模式或实验细节时，才读
链接的原论文。较长的 `papers/README.md` 只用于局部检索候选论文条目，不要
默认全文加载。鼓励把复杂 implementation、diagnostics、build/test/benchmark、
长时间命令和大输出交给 subagent，以避免污染 main context window；但是否使用
subagent 由 agent 自主决定，main agent 也可以直接编辑和执行。使用
`paper-grounded-subagent` helper 时，Codex 建议默认
`gpt-5.3-codex-spark` / high，Claude 建议默认
`claude-opus-4-8` / medium，也可以显式选择其他模型。报告写到
`hl_agent/runs/paper_grounded_subagents/`。

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
real。

Claude Code 和 Codex runtime 遵循相同的 candidate 提交规则：每个已定形、
能够 build/test 且输出 valid 的行为改变型 candidate，无论 local/public 自测
改善、持平还是退化，都必须先单独 git commit，再立即跑普通
`coral eval -m "..."`。Agent 不得因为自己的 benchmark/eval 结果不好而静默
revert、丢弃或切换方向；必须先为该 candidate 留下 commit 和 hidden real
attempt，再决定 retain 或放弃。回退到已有 official eval 的旧状态时，记录直接
引用旧 commit/attempt 的分数，不要为回退本身重复跑 benchmark/eval。每次 agent
向用户汇报 retained、rejected、reverted candidate 或 progress，都必须确保对应
行为改变型 candidate 已有普通 `coral eval`。

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
lab-notebook 要求保持一致。运行过程中应持续更新日志，而不是只写最终总结；
尤其在 candidate/长任务开始前、重要命令完成后以及可能自动 compact、重启或
切换 agent 前，写入 parent commit、hypothesis、commands/results、artifact
路径、changed files、commit/eval attempt、当前状态和下一步。resume 后先读
这些 checkpoint，避免丢失细节或重复实验。
