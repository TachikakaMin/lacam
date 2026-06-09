# Lifelong TAPF Code Reading Notes

This note records the current one-shot TAPF / LaCAM-TAPF structure before the
lifelong conversion.

## Map Reading

- `Graph` is defined in `lacam/include/graph.hpp` and implemented in
  `lacam/src/graph.cpp`.
- `Graph::Graph(const std::string& filename)` reads MovingAI-style map files:
  `height`, `width`, then the `map` grid.
- Current obstacle semantics are binary. Cells with `T` or `@` are skipped, and
  every other character becomes a traversable `Vertex`.
- `Graph::U` is indexed by `width * y + x` and contains either `nullptr` for
  obstacles or a `Vertex*` for traversable cells. `Graph::V` is the compact list
  of traversable vertices.
- Current code does not preserve original cell type, so `a`, `o`, and `i`
  cannot yet be distinguished after parsing.

## Task / Goal Set Data Structures

- One-shot MAPF uses `Instance` in `lacam/include/instance.hpp`.
- One-shot TAPF uses `TAPFInstance` in `lacam/include/instance.hpp`.
- `TAPFInstance::starts` is the current start configuration for all agents.
- `TAPFInstance::tasks` is a deduplicated flat list of all goal vertices from
  all agent goal sets.
- `TAPFInstance::allowed[i][j]` records whether agent `i` may use deduplicated
  task/goal vertex `j`.
- The YAML loader accepts `potentialGoals` or `goal` per agent and turns each
  agent goal set into the `allowed` compatibility matrix.

## TAPF Instance Construction

- `TAPFInstance(map_filename, start_indexes, task_indexes)` builds a fresh
  `Graph`, converts `start_indexes` to `starts`, deduplicates all
  `task_indexes`, and fills `allowed`.
- `TAPFInstance(yaml_filename, map_dir)` reads the YAML fixture and delegates to
  the constructor above.
- `TAPFInstance::is_valid()` currently requires at least one allowed task per
  agent and `tasks.size() >= N`.
- Lifelong replanning will need to build a temporary one-shot `TAPFInstance`
  from current agent locations plus per-agent planning targets.

## Cost Matrix And Distance Tables

- TAPF distance lookup is implemented by `TAPFDistTable` in
  `lacam/include/tapf_dist_table.hpp` and `lacam/src/tapf_dist_table.cpp`.
- `TAPFDistTable` is lazy and indexed by deduplicated task id and vertex id.
  Each task starts a reverse BFS from its goal vertex, and `get(task, vertex)`
  expands until the queried vertex distance is known or unreachable.
- Assignment cost construction lives in `build_cost_matrix()` inside
  `lacam/src/tapf_assignment.cpp`.
- Current one-shot assignment cost is `D.get(task_id, current_agent_vertex)`,
  optionally plus a sticky reassignment penalty.
- Lifelong unloaded-agent assignment cost must be changed outside this one-shot
  assumption to:
  `dist(agent_current_location, task_start) + min dist(task_start, task_goal)`.

## Task Assignment

- `assign_tapf_tasks()` performs a full Hungarian assignment over the current
  cost matrix.
- `assign_tapf_tasks_dynamic()` keeps a `TAPFAssignmentState` and repairs only
  changed rows when possible.
- `TAPFPlanner::solve()` currently computes an initial dynamic assignment, and
  each new search node can refresh assignment based on changed agent rows.
- Current assignment assumes every planning agent participates and every
  assigned target is a final TAPF goal vertex. Lifelong loaded agents and
  unloaded agents will need different handling before constructing the temporary
  one-shot instance.

## Planner Entry Points

- Public one-shot MAPF entry point: `solve()` in `lacam/include/planner.hpp`.
- Public one-shot TAPF entry point: `solve_tapf()` in
  `lacam/include/tapf_planner.hpp`.
- CLI MAPF entry point: `main.cpp`, using `Instance` from a `.scen` file or
  random starts/goals.
- CLI TAPF benchmark entry point: `tools/tapf_benchmark.cpp`, using
  `TAPFInstance` from YAML and printing planner/validation metrics.

## Tests And Experiment Scripts

- Unit tests live in `tests/`.
- Relevant TAPF tests:
  - `tests/test_tapf_planner.cpp`
  - `tests/test_instance.cpp`
  - `tests/test_graph.cpp`
  - `tests/test_dist_table.cpp`
- Existing test assets include `tests/assets/symbotic.map` and small maps such
  as `tests/assets/2x1.map`.
- Existing TAPF experiment/analysis scripts live under `tools/`, including:
  - `tools/tapf_benchmark.cpp`
  - `tools/run_tapf_experiment.py`
  - `tools/run_tapf_repair_experiment.py`
  - `tools/validate_tapf_solution.py`
  - `tools/visualize_tapf_schedule.py`
