lacam1
---
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)
[![CI](https://github.com/Kei18/lacam/actions/workflows/ci.yml/badge.svg)](https://github.com/Kei18/fast-mapf/actions/workflows/ci.yml)

The code repository of the paper ["LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding"](https://kei18.github.io/lacam) (AAAI-23).

__A refactored, clean version is available: [lacam0](https://github.com/Kei18/lacam0). I recommend using it instead of this repo.__

## Branches (this fork)

This fork ([TachikakaMin/lacam](https://github.com/TachikakaMin/lacam)) extends upstream
[Kei18/lacam](https://github.com/Kei18/lacam) toward lifelong warehouse throughput research:

| Branch | Purpose |
|---|---|
| `dev` | Mirror of the upstream LaCAM (AAAI-23) code; base of all other branches. |
| `lacam_mapd` | Gym-style lifelong MAPD environment, congestion-aware assignment cost modes, and MAPD benchmarks. |
| `lacam_tapf` | LaCAM-TAPF extension (task-and-path-finding), symbotic warehouse experiments, and adaptive experiment runners. |
| `lacam_agent` | LLM-agent research setup: the CORAL lifelong-throughput task (`coral_tasks/lacam_throughput`), paper briefs, and `hl_agent` tooling; built on `lacam_tapf`. |
| `agent_fable` | Best snapshot from the CORAL Fable run — deterministic five-candidate rollout portfolio, hidden 10-seed mean throughput 1.65525 at commit `05d91e9` — plus follow-up rollout-horizon experiments. |

**This branch: `lacam_tapf`** — LaCAM-TAPF: integrated task assignment and path finding on top of LaCAM, with symbotic warehouse experiments and adaptive runners (see `lacam_tapf.md` and `tools/`).

## Building

All you need is [CMake](https://cmake.org/) (≥v3.16). The code is written in C++(17).

First, clone this repo with submodules.

```sh
git clone --recursive https://github.com/TachikakaMin/lacam.git
cd lacam
git switch lacam_tapf
git submodule update --init --recursive
```
Then, build the project.

```sh
cmake -B build && make -C build
```

### Conda (Linux)

With a system C++17 compiler (such as `g++`) available:

```sh
conda env create -f environment.yml
conda activate lacam-tapf
git submodule update --init third_party/argparse third_party/googletest third_party/ITA-CBS2
export PKG_CONFIG_PATH="$CONDA_PREFIX/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
cmake --build build -j 4
./build/test_all
./build/tapf_benchmark third_party/ITA-CBS2/map_file/debug_cbs_data.yaml \
  third_party/ITA-CBS2/map_file 10 build/tapf_demo.yaml 1 0 0 focal 1.5 h
python tools/validate_tapf_solution.py \
  third_party/ITA-CBS2/map_file/debug_cbs_data.yaml build/tapf_demo.yaml
```

CMake is pinned below 4 for compatibility with the bundled dependencies.

### Docker

You can also use the [docker](https://www.docker.com/) environment (based on Ubuntu18.04) instead of the native one.

```sh
# ~10 min, mostly for CMake build
docker compose up -d
docker compose exec dev bash
> cmake -B build && make -C build
```

## Usage

### TAPF benchmark (this branch)

Extra CMake target: `tapf_benchmark` (LaCAM-TAPF; the method is documented in [`lacam_tapf.md`](lacam_tapf.md)).

```sh
build/tapf_benchmark YAML MAP_DIR [TIME_LIMIT_SEC] [SCHEDULE_YAML] [ANYTIME=1] \
  [FULL_TA=0] [SEED=-1] [SEARCH_MODE=dfs] [FOCAL_WEIGHT=1.5] [FOCAL_TIE_BREAK=h]
```

Experiment runners and symbotic scenarios live in `tools/` and `tests/assets/`.

### Basic MAPF solver (upstream)
```sh
build/main -i assets/random-32-32-10-random-1.scen -m assets/random-32-32-10.map -N 50 -v 1
```
The result will be saved in `build/result.txt`.

<details><summary>Output File</summary>

This is an example output of `random-32-32-10-random-1.scen`.
`(x, y)` denotes location.
`(0, 0)` is the left-top point.
`(x, 0)` is the location at `x`-th column and 1st row.

```
agents=50
map_file=random-32-32-10.map
solver=planner
solved=1
soc=1316
soc_lb=1113
makespan=55
makespan_lb=53
sum_of_loss=1191
sum_of_loss_lb=1113
comp_time=1
seed=0
starts=(11,6),(29,9),[...]
goals=(7,18),(1,16),[...]
solution=
0:(11,6),(29,9),[...]
1:(10,6),(29,10),[...]
[...]
```

</details>

You can find details of all parameters with:
```sh
build/main --help
```

## Visualizer

[@Kei18/mapf-visualizer](https://github.com/kei18/mapf-visualizer) is available.

## Experiments

The experimental script is written in Julia ≥1.7.
Setup may require around 10 minutes.

```sh
sh scripts/setup.sh
```

Edit the config file as you like.
Examples are in `scripts/config` .
The evaluation starts by following commands.

```
julia --project=scripts/ --threads=auto
> include("scripts/eval.jl"); main("scripts/config/mapf-bench.yaml")
```

## LaCAM-TAPF Method

This fork includes a LaCAM-style TAPF solver with dynamic task assignment,
FOCAL high-level anytime search, PIBT hindrance tie-breaking, LaCAM2-style swap
support, and experiment runners for comparing `lacam_dfs`, `lacam_focal_h`, and
IR-TAPF. See [`lacam_tapf.md`](lacam_tapf.md) for the method details, cost
definitions, known tradeoffs, and experiment notes.

Basic TAPF benchmark usage:

```sh
build/tapf_benchmark case.yaml "" 10 out.yaml 1 0 -1 dfs 1.5 h
build/tapf_benchmark case.yaml "" 10 out.yaml 1 0 -1 focal 1.5 h
```

Full exp1/exp2/IR comparison:

```sh
python3 -u tools/run_full_three_method_experiment.py \
  --all-itacbs-data \
  --ir-matrix-root /home/yimin/research/ir-tapf/matrix \
  --time-limit 10 \
  --timeout 30 \
  --jobs 32 \
  --resume \
  --out-dir build/results/full_three_method_hindrance_exp1_exp2_ir
```

arXiv 2605.07744-style paper experiment runner:

```sh
python3 -u tools/run_paper_2605_07744_experiments.py \
  --paper-suite fig3 \
  --jobs 32 \
  --timeout 45 \
  --resume \
  --out-dir build/results/paper_2605_07744_fig3

python3 tools/plot_paper_2605_07744_results.py \
  --rows build/results/paper_2605_07744_fig3/rows.csv \
  --out-dir build/results/paper_2605_07744_fig3/plots
```

To collect paper-numbered figures into one folder:

```sh
python3 tools/plot_paper_2605_07744_results.py \
  --paper-summary \
  --out-dir build/results/paper_2605_07744_figures \
  --rows \
    build/results/paper_2605_07744_fig3/rows.csv \
    build/results/paper_2605_07744_fig4/rows.csv \
    build/results/paper_2605_07744_fig5/rows.csv \
    build/results/paper_2605_07744_fig6/rows.csv \
    build/results/paper_2605_07744_table1/rows.csv \
    build/results/paper_2605_07744_table2/rows.csv \
    build/results/paper_2605_07744_table3/rows.csv \
    build/results/paper_2605_07744_table4/rows.csv
```

The paper runner supports filtered retries for incomplete shards:

```sh
python3 -u tools/run_paper_2605_07744_experiments.py \
  --paper-suite fig3 \
  --resume \
  --skip-rows-jsonl build/results/paper_2605_07744_fig3/rows.jsonl \
  --scenarios fig3_Boston_0_256_random \
  --agent-counts 400 \
  --seeds 5 9 \
  --out-dir build/results/paper_2605_07744_fig3_retry
```

Table 4 ITA-ECBS comparison:

```sh
python3 -u tools/run_paper_2605_07744_table4.py \
  --jobs 16 \
  --resume \
  --out-dir build/results/paper_2605_07744_table4
```

The paper runner generates IR-TAPF matrices with the `ir-tapf setup` command,
runs the paper IR solvers on those matrices, converts the same matrices to
LaCAM-TAPF YAML, and records comparable `rows.csv`, `summary.csv`, and plot
outputs. Comparable paper suites include both `lacam_dfs` and `lacam_focal_h`
as same-instance baselines, with solver time limits aligned within each suite.
Cached matrices are accepted only when every agent has at least one reachable
target in its connected component and the reachable per-agent target graph
admits a full matching; invalid cached matrices are deleted and regenerated
before a solver is launched. Supported suites and current limitations are documented in
[`lacam_tapf.md`](lacam_tapf.md).

For large Fig. 5 shards, `sum_shortest_distances` can dominate runner overhead
because exact normalization requires many grid BFS calls on 10,000-agent
matrices. The runner therefore caches this derived value beside each matrix and
also supports `--skip-sum-shortest` for retry shards where raw solve
rate/runtime/SOC are the required outputs.

## TAPF Data

### Reproducing arXiv:2609.14208

The existing repository generators and monitored runner cover the paper's eight
maps, 3,200 Group Test instances, and 6,560 Common Target Test instances.
`prepare_repository_paper_2609.py` calls the existing exp1/exp2 programs without
replacing their sampling logic. Common retains its fixed seed 1 and continuous
stream. Group has no seed in the original program; the launcher supplies initial
seed 1 for repeatability. Solver seeds are the test indices 0–19. Original lost
files and the historical Group RNG state were not recovered. Policies, commands,
source hashes, matching checks, and YAML/matrix equivalence are recorded in the
new dataset manifest and audit.

**Review finding (2026-10-06):** the first batch copied Symbotic's annotated
map characters unchanged. ITA-CBS treats non-`.` cells as obstacles, while the
generator and LaCAM treat non-`@`/`T` cells as free. Its Symbotic CBS results are
therefore invalid for comparison. New generation normalizes free cells to `.`
and refuses to overwrite an existing manifest. Existing results are preserved;
see `review_notice.json` in the first batch's result directory. The original
Common Test sampling rules must also be aligned before claiming exact replication.

```sh
conda activate lacam-tapf
python tools/prepare_repository_paper_2609.py --out-dir data/paper_2609_14208_repository
python -u tools/run_paper_2609_14208.py \
  --manifest data/paper_2609_14208_repository/manifest.json \
  --out-dir build/results/paper_2609_14208_repository \
  --jobs 48 --reserve-threads 4 --time-limit 10
```

Build `third_party/ITA-CBS2/build/ITACBS_remake` and the sibling repository
`../ir-tapf/target/release/ir_tapf` before running. The local ITA-CBS patch adds
`--time-limit-sec`; the IR patch fixes dependency interfaces, passes remaining
time into MAPF calls, rejects late first solutions, and writes incumbent paths
when `TAPF_SCHEDULE_OUTPUT` is set. Patches, environment exports, and binary
hashes are saved with the reproduction results.

The runner uses DFS with anytime enabled for ITA-LaCAM, DBS-Hungarian for
IR-TAPF, and optimal ITA-CBS. Each solver has a 10-second search budget;
initialization and output overhead are included separately in `wall_time_s`.
It pins solver processes to individual CPUs and excludes four CPUs from the
entire experiment process tree. It pauses new launches below 32 GiB available
RAM. Monitor `status.json`, `monitor.jsonl`, and `run.log` in the result directory.
Trajectory validation uses eight processes (`--validation-jobs 8`) on CPUs
outside the solver slots. After solving finishes, final archive validation uses
32 processes (`--audit-jobs 32`) and reports progress through `status.json`.
Each task saves compressed logs and trajectories plus an atomic result file;
rerunning the same command resumes completed tasks. Failed solvers are retained
as experimental outcomes; infrastructure errors require review.

Successful trajectories are checked independently for obstacles, valid moves,
vertex conflicts, edge swaps, eligibility, and distinct final goals. Final-path
SOC is recomputed from trajectories. After all tasks finish, the runner exports
CSV tables, audits every expected case/method pair, and generates Figures 2–6
in PNG/PDF. To rerun reporting:

```sh
python tools/report_paper_2609_14208.py \
  --run-dir build/results/paper_2609_14208_reproduction
```

The ITA-CBS TAPF fixtures used by the LaCAM-TAPF experiments live outside the
repository:

```sh
/media/project0/yimin/lacam_tapf_itacbs_data
```

`third_party/ITA-CBS2/map_file/` contains symlinks to those generated
directories. The generated dataset contains ITA-CBS exp1 and exp2 fixtures for
all paper maps.

The ITA-CBS generation scripts were adjusted for maps with multiple connected
components. Starts are sampled from all free cells, then each agent's
`potentialGoals` are sampled from the connected component containing that
agent's start. This avoids infeasible TAPF fixtures where an agent is assigned
only unreachable potential goals. For exp2, common goals are component-local
when sampled starts span multiple connected components.

The full regenerated dataset has:

- exp1: 8 directories, each with `400` YAML files plus one map file.
- exp2: 28 non-maze ratio directories, each with `200` YAML files plus one map
  file.
- exp2 maze: 4 ratio directories, each with `240` YAML files plus one map file.
  These directories now cover `5..60` agents in steps of 5, with 20 tests per
  agent count.

The maze exp2 `40..60` fixtures were generated with
`third_party/ITA-CBS2/python/generate_data_for_exp2.py` using
`--agent_start 5 --agent_stop 60 --agent_step 5`. The regenerated `5..35`
files matched the existing fixtures byte-for-byte before the missing `40..60`
files were copied into `/media/project0/yimin/lacam_tapf_itacbs_data`.

For arXiv 2307.00663-style plots, `ITA-CBS`, `LaCAM-TAPF`, and `IR-TAPF` use
row-level local reruns on these YAML fixtures. The `ITA-ECBS` Figure 2 curve is
different: it is loaded from the precomputed aggregate success-rate table
`/home/yimin/research/ITA-CBS2/plot_figure_ecbs/cvsdata.csv`, not from local
row-level YAML reruns.

Validation checks should confirm that every potential goal is reachable from
its agent's start and that each fixture has a reachable perfect task matching.


## Group 250–500 agent extension (local experiment)

The native exp1 generator accepts `--agent_start`, `--agent_stop`, and
`--agent_step`. Its defaults still generate 10–200 agents in steps of 10.
`tools/prepare_repository_paper_2609.py --group-extension` generates eight maps,
six sizes (250, 300, 350, 400, 450, 500), and 20 instances per size. It audits
start uniqueness, reachable goals, perfect task matching, and YAML/matrix
identity before writing the 960-case manifest. The group size is five; the
last group in a connected component may contain fewer agents.

```sh
conda activate lacam-tapf
python tools/prepare_repository_paper_2609.py --group-extension \
  --out-dir data/group_250_500_repository_new --jobs 8
python tools/run_paper_2609_14208.py \
  --manifest data/group_250_500_repository_new/manifest.json \
  --out-dir build/results/group_250_500_repository_new \
  --methods lacam_dfs ir --jobs 56 --validation-jobs 8 --skip-report
python tools/report_group_extension.py \
  --manifest data/group_250_500_repository_new/manifest.json \
  --run-dir build/results/group_250_500_repository_new
```

The run selects only ITA-LaCAM and IR-TAPF. Each solve uses a 10-second search
budget and one CPU. Four logical CPUs are reserved. Validation runs in fresh
processes and successful compressed trajectories are independently checked.
The selected job count requires at least 64 available logical CPUs.

The original IR distance cache contains an early-return error. It removes a
BFS frontier node before expanding it, so later queries can incorrectly return
unreachable. The local experiment preserves the original 960 IR results and
adds a separate 960-case cache-fixed run. The patch, executable regression,
and real-case traces are under
`build/results/group_250_500_repository/ir_diagnosis/`. The corrected binary is
`ir_tapf_cache_fixed` in that directory; use `--ir-binary` to select it explicitly.
The sibling IR repository's default executable remains the original version.

- Original extension: `build/results/group_250_500_repository/comparison.html`.
- IR cache-fixed raw run: `build/results/group_250_500_cache_fixed/`.
- Combined audited comparison: `build/results/group_250_500_cache_comparison/comparison.html`.

The comparison reuses the same ITA-LaCAM results and the same input manifest.
It shows original and fixed IR separately. It does not choose the best result
from repeated attempts. SOC and first-solution plots compare paired successful
instances. These larger sizes are an extension, not numerical points from the
paper.

## Notes

- The grid maps and scenarios in `assets/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
- The empirical data of the manuscript was obtained with [[exp/AAAI2023]](https://github.com/Kei18/lacam/releases/tag/exp%2FAAAI2023).
- LaCAM with different design choices: see [[pilot/greedy]](https://github.com/Kei18/lacam/releases/tag/pilot%2Fgreedy) and [[pilot/dbs]](https://github.com/Kei18/lacam/releases/tag/pilot%2Fdbs)
- `tests/` is not comprehensive. It was used in early developments.
- Auto formatting (clang-format) when committing:

```sh
git config core.hooksPath .githooks && chmod a+x .githooks/pre-commit
```

## Licence

This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author

[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at Tokyo Institute of Technology, interested in controlling multiple moving agents.

## Dynamic-TA failure analysis (up to 500 agents)

The audit covers all 10,720 available cases. There are 25 cases where cache-fixed
IR succeeds and the original ITA-LaCAM fails, plus three with independently
validated successful paths from the historical IR run. All 28 original ITA-LaCAM
failures are timeouts, not proofs of infeasibility.

The original restart branch selects the initial node but then rejects it when
`queued` is already true. Across 28 instrumented runs, all 570 selected restarts
were blocked. Moving the existing initial-node OPEN entry to the stack top makes
restarts effective without replacing dynamic assignments. The isolated restart
change solves 18/28 counterexamples. Dynamic TA still drives node goals,
priorities, and PIBT; the constraints are recorded in [AGENTS.md](AGENTS.md).

A paired 10-second experiment over all 960 cases with 250–500 agents gives:

| Version | Validated successes |
| --- | ---: |
| Original ITA-LaCAM, fresh replay | 905/960 |
| Restart correction plus equivalent computation optimizations | 928/960 |

There are 24 newly solved cases and one regression relative to the same-round
baseline: `symbotic_agents_500_test_4`. The same 18 counterexamples succeed in
the screening, trace, and full extension runs. These candidate results remain
separate from the original benchmark statistics. No candidate success-rate claim
is made for the smaller cases that were audited but not rerun.

The default planner includes only the equivalent computation changes: indexed
goal checks and skipping unused FOCAL metrics in DFS. The restart correction
remains an experimental patch. To try it on this version:

```sh
git apply experiments/ita_dynamic_500/restart_only.patch
```

The other archived patches are relative to baseline commit `b1f42c4`; combined
patches must not be applied on top of the computation changes a second time.
[Summary](experiments/ita_dynamic_500/analysis_summary.json),
[per-case diagnostics](experiments/ita_dynamic_500/counterexample_details.json),
[paired results](experiments/ita_dynamic_500/extension_results.csv), and
[comparison figure](experiments/ita_dynamic_500/success_250_500.png) are versioned.

The local analysis workflow uses `tools/build_dynamic_ita_variants.py`,
`tools/run_ita_optimization.py`, `tools/build_dynamic_ita_traces.py`, and
`tools/report_dynamic_ita_analysis.py`. These research scripts require the local
run snapshots, baseline build objects, fixtures, and Conda paths recorded in the
experiment artifacts; they are not a standalone fresh-clone reproduction command.
The interactive page is generated at
`build/results/ita_optimization_500/analysis.html` and linked from the existing
comparison page. Large generated datasets, binary outputs, raw logs, and replay
archives remain local under ignored `data/` and `build/` directories.

`generate_paper_2609_dataset.py` is an abandoned sampling experiment; use the
native-generator wrapper `prepare_repository_paper_2609.py` for benchmark data.
The fixed-target controls in `analyze_ir_lacam_failures.py` and its report are
historical diagnostics only, not ITA-LaCAM optimization candidates or formal
benchmark results.
