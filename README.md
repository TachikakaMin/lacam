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

**This branch: `lacam_mapd`** — adds a Gym-style lifelong MAPD environment core, congestion-aware assignment cost modes, and benchmark plumbing/results (see `docs/` and `python/`).

## Building

All you need is [CMake](https://cmake.org/) (≥v3.16). The code is written in C++(17).

First, clone this repo with submodules.

```sh
git clone --recursive https://github.com/Kei18/lacam.git
cd lacam
```
Then, build the project.

```sh
cmake -B build && make -C build
```

### Docker

You can also use the [docker](https://www.docker.com/) environment (based on Ubuntu18.04) instead of the native one.

```sh
# ~10 min, mostly for CMake build
docker compose up -d
docker compose exec dev bash
> cmake -B build && make -C build
```

## Usage

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


## Lifelong Ore/TAPF Simulation

A LaCAM-TAPF lifelong ore workflow is available at `python/lacam_ore_workflow.py`.
It mirrors the ITA-CBS ore simulation loop: each round converts the current
lifelong ore state into a one-shot TAPF YAML, calls `build/tapf_benchmark`, then
executes the returned joint path until the first pickup/dropoff event.

Example:

```sh
python3 python/lacam_ore_workflow.py simulate \
  --input examples/lifelong/debug_data_LTAPF_mine_tunnel_new.yaml \
  --binary build/tapf_benchmark \
  --output outputs/lacam_event_sim.json \
  --work-dir outputs/lacam_rounds \
  --max-rounds 100 --max-steps 2000 --time-limit 10 --timeout 15 --verbose
```

For quick lifecycle debugging, add `--no-anytime` so each round stops after the
first feasible TAPF solution instead of spending the whole time budget improving
an incumbent.

Replay the generated JSON interactively with Tk:

```sh
python3 python/lacam_ore_workflow.py replay \
  --input outputs/lacam_event_sim.json
```

Render a single frame to PNG, useful on headless machines:

```sh
python3 python/lacam_ore_workflow.py snapshot \
  --input outputs/lacam_event_sim.json \
  --output outputs/lacam_event_sim_t0.png \
  --time 0
```

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
