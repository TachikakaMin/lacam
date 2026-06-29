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
