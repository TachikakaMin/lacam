# Goal: mode2 congestion-aware assignment cost

## Current retained state

- Keep `assignment_cost_mode` plumbing through planning, simulation,
  benchmark CSV output, and grid runner.
- Keep `mode0` as the default baseline.
- Keep `mode1` as a mild loaded-pickup-delay comparator only.
- Remove the rejected strong pickup-delay, delivery-first, and WIP-delay
  formulas from the main implementation; benchmark evidence showed they
  reduced throughput overall.

## Problem

On `symbotic_star`, increasing agent count beyond roughly 100 creates visible
queues near aisle mouths / channel entrances. The current cost matrix mostly
uses static distance and loaded-pickup delay. It does not price the fact that a
target or corridor is already crowded, so many loaded delivery agents are sent
through the same narrow area at the same time.

The next useful experiment is `mode2`: a general congestion-aware cost mode
that improves assignment decisions without hard-coding a specific K, map, or
agent count.

## Mode2 first version

Add a new assignment cost mode:

```text
LIFELONG_ASSIGNMENT_COST_CONGESTION = 2
```

For every pickup or delivery candidate target, add a local crowding penalty:

```text
final_cost = baseline_or_mode1_cost
           + congestion_weight * local_congestion(target_region)
```

where:

```text
target_region = free cells within radius r around the target

local_congestion =
    current agents inside target_region
  + already-created candidate targets in the same target_region
```

Start with simple constants:

```text
r = 1 or 2
congestion_weight = common_cost_scale
```

The first version should stay local and cheap. Do not implement full
shortest-path traffic prediction until the local penalty shows a measurable
benefit.

## Metrics to add before trusting mode2

Record congestion diagnostics in benchmark traces or CSV summaries:

- Average and max number of agents near aisle-mouth / target regions.
- Loaded agents waiting in those regions.
- Loaded agents stopped in place while near those regions.
- Final picked tasks.
- Average delivery time.
- Throughput.

These metrics are needed to distinguish true congestion relief from simply
doing fewer pickups.

## Benchmark plan

Run paired comparisons:

```text
maps: symbotic_star, symbotic
agents: 50,100,150,200
K: 1,2,4
slot: 1,2,3
duration: 0,2,4,8
dist: 50_50,80_20
seeds: 0,1,2
cost modes: 0,1,2
horizon: 200
time limit: 1s
```

Primary success criteria:

- `mode2` improves or preserves throughput on `symbotic_star` at 150/200
  agents.
- `mode2` lowers final picked tasks and average delivery time.
- `mode2` does not significantly regress `symbotic`.
- Benefits hold across K without K-specific rules.

## Expected decision

- If mode2 reduces congestion and improves high-agent star throughput, make it
  the next candidate default.
- If mode2 only reduces WIP but loses throughput, keep it as diagnostics and
  explore a path-aware congestion estimate.

## Current result

Implemented `LIFELONG_ASSIGNMENT_COST_CONGESTION = 2` as the first local
version:

```text
mode2_cost = mode1_cost
           + common_cost_scale * local_congestion(target_region)
```

where `target_region` is the radius-2 traversable neighborhood around each
pickup or delivery target.  `local_congestion` counts current agents in that
region, excluding the candidate agent itself, plus already-created candidate
targets in that region.  The implementation uses per-cell count tables, not a
scan over all previous candidates, so high-agent benchmarks do not time out.

Added trace metrics:

- `average_target_region_agent_count`
- `max_target_region_agent_count`
- `average_target_region_loaded_agent_count`
- `max_target_region_loaded_agent_count`
- `average_target_region_loaded_waiting_count`
- `max_target_region_loaded_waiting_count`
- `average_target_region_loaded_stopped_count`
- `max_target_region_loaded_stopped_count`

Validation run:

```text
./build/test_lifelong_planning
./build/test_lifelong_simulation
cmake --build build --target lifelong_benchmark
```

All direct tests passed.  `ctest --test-dir build --output-on-failure` reports
that this build directory has no registered tests.

Benchmark subset run, all valid:

```text
maps: symbotic, symbotic_star
agents: 100,150,200
K: 1,2,4
slot: 2
duration: 4
dist: 50_50,80_20
seeds: 0,1,2
cost modes: 0,1,2
horizon: 200
time limit: 1s
cases: 324
aggregate: tmp_runs/mode2_congestion_subset_opt_aggregate.csv
```

Mean matched deltas for mode2:

```text
overall:             throughput -0.1164 vs mode0, -0.0825 vs mode1
symbotic all:        throughput -0.1881 vs mode0, -0.1142 vs mode1
star agents 150/200: throughput -0.0433 vs mode0, -0.0442 vs mode1
star K=1 150/200:    throughput +0.0308 vs mode0/mode1
star K=2/4 150/200:  throughput -0.0804 vs mode0, -0.0817 vs mode1
```

Mode2 reduced WIP-style pressure in the subset:

```text
overall final_picked_tasks:         -11.62 vs mode0
overall loaded_stopped near target: -0.078 vs mode0
star agents 150/200 final_picked:   -10.72 vs mode0
star agents 150/200 loaded_stopped: -0.106 vs mode0
```

Decision: keep mode2 as diagnostics/experiment only.  It helps K=1 high-agent
`symbotic_star`, but it does not hold across K and it significantly regresses
`symbotic`.  Next useful direction is a path-aware or bottleneck-aware
congestion estimate that prices shared approaches without broadly suppressing
pickup/delivery work.
