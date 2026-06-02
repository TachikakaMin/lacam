lacam1
---
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)
[![CI](https://github.com/Kei18/lacam/actions/workflows/ci.yml/badge.svg)](https://github.com/Kei18/fast-mapf/actions/workflows/ci.yml)

The code repository of the paper ["LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding"](https://kei18.github.io/lacam) (AAAI-23).

__A refactored, clean version is available: [lacam0](https://github.com/Kei18/lacam0). I recommend using it instead of this repo.__

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
outputs. Cached matrices are accepted only when every agent has at least one
reachable target in its connected component and the reachable per-agent target
graph admits a full matching; invalid cached matrices are deleted and
regenerated before a solver is launched. Supported suites and current
limitations are documented in
[`lacam_tapf.md`](lacam_tapf.md).

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
- exp2 maze: 4 ratio directories, each with `140` YAML files plus one map file
  because maze uses `5..35` agents.

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
