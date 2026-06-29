# Goal: Gymnasium-style Lifelong TAPF Environment Refactor

## 0. Scope

把当前 event-driven lifelong TAPF simulation 改造成 Gymnasium 风格的标准
RL environment，但保持现有 LaCAM-TAPF / one-shot replanning 语义不变。

本目标不是把 planner 改成每 timestep 重算，也不是把 centralized planner
改成多智能体独立 policy。目标是明确拆边界：

- `env` 负责地图、agent 状态、task 产生、合法移动、pickup/delivery service、
  碰撞检查、episode 终止、reward/metrics。
- `planner/policy` 负责根据 env 给出的 observation / planner request 计算 action。
- planner 可以一次性算出整段 one-shot path，并在内部缓存；之后每个 env step
  只返回当前 timestep 的 joint action。
- 当 env 标记需要 replanning 时，planner 再重新调用 LaCAM-TAPF。

最终需要同时支持：

- C++ 内核级 step/reset API。
- Python/Gymnasium wrapper。
- 旧的 `run_lifelong_simulation(config)` / `lifelong_benchmark` 行为兼容。

## 1. Current Code Anchors

当前所有 loop 逻辑集中在：

```text
lacam/src/lifelong_simulation.cpp
```

关键位置：

- 初始化 `Graph/agents/tasks/generator/distances/metrics`:
  `run_lifelong_simulation`, roughly lines 780-846.
- 每 timestep 生成新 task:
  `generator.generate_for_timestep`, roughly lines 848-852.
- service readiness / pickup / completion event:
  roughly lines 854-952 and 1376-1379.
- replanning gate:
  roughly lines 950-962.
- LaCAM-TAPF invocation:
  roughly lines 1150-1168.
- `solution + assignment_schedule` 转成可执行 plan:
  roughly lines 1216-1268.
- plan cache 更新:
  roughly lines 1339-1346.
- 每步移动和 assignment schedule 应用:
  roughly lines 1359-1389.

当前 planning adapter 已经存在：

```text
lacam/include/lifelong_planning.hpp
lacam/src/lifelong_planning.cpp
```

重要细节：`prepare_lifelong_planning_snapshot(...)` 内部会调用
`release_unpicked_assignments(agents, tasks)`。这意味着当前所谓
"build planner input" 不是纯函数，它会修改真实 env state：

- assigned-but-unpicked task 被释放回 pending。
- unloaded agent 的未 pickup assignment 被清空。
- picked / carried task 绑定被保留。

Env 化时必须把这个状态突变显式收回 env 侧，不能让 policy/planner 隐式修改
env state。

## 2. Gymnasium Correspondence

Gymnasium 标准结构：

```text
obs, info = env.reset(seed=None, options=None)
obs, reward, terminated, truncated, info = env.step(action)
```

对应到本项目：

- `reset` 初始化 map、agents、task generator、distance cache、metrics，并生成
  `t=0` 的初始 task backlog，使 planner 首次拿到的信息和当前 simulation
  `t=0` replanning 前一致。
- `step(action)` 执行一个 timestep 的 joint action。
- `terminated` 表示任务型 episode 自然结束。例如有限 task scenario 全部完成。
- `truncated` 表示 horizon 到达、debug invariant 失败、非法 action 或外部时间限制。
- `info.needs_replan` 表示下一次 policy 调用应该跑 planner。
- `info.planner_request` 是 env 给 planner 的结构化输入，不要求直接等于 Gym obs。

这里应使用 centralized single-policy action，而不是 PettingZoo 风格的
per-agent multi-agent API。一个 action 是全体 agent 的 joint action。

## 3. Environment State Ownership

Env owns:

- `Graph`
- `MapDistanceCache`
- `LifelongTaskGenerator`
- `std::vector<LifelongAgentState>`
- `std::vector<LifelongTask>`
- service state:
  - `service_active`
  - `service_keys`
  - `service_target_indexes`
  - `service_progress`
- inherited priority offsets
- timestep
- pending arrival/replan flags
- metrics accumulator
- debug/invariant checks

Policy/planner must not own or mutate:

- task status
- carried task ids
- assigned task ids
- current agent location in env
- service progress
- metrics

Policy may own:

- cached one-shot solution path
- cached per-step assignment keys
- cached per-step target indexes
- planner statistics from last invocation
- internal random seed / deadline config

## 4. Replanning Rules To Preserve

Do not convert to forced per-timestep replanning.

Env should set `needs_replan = true` only when the current implementation would
enter `should_replan`:

```text
t == 0
pickup event happened
completion/delivery event happened
current plan/action stream finished while unfinished work remains
previous planner failed and there is no valid plan
no valid plan and idle unloaded agent exists while pending task exists
```

New task release must not interrupt a valid cached plan by itself. New tasks
enter the pending pool and become visible at the next replanning event.

Planner failure / timeout behavior must remain:

- simulation/env does not crash.
- all agents wait one step.
- next timestep requests replanning again.
- metrics record timeout/failure/empty/infeasible reason.

## 5. Action Contract

Use a joint action. A minimal C++ shape should be equivalent to:

```cpp
struct LifelongEnvAction {
  std::vector<int> next_indexes;                 // size N
  std::vector<int> assignment_keys;              // size N, optional per step
  std::vector<int> assignment_target_indexes;    // size N, optional per step
  bool commits_replan_assignment = false;
  std::vector<int> initial_assignment_keys;      // size N when committing
  std::vector<int> initial_assignment_targets;   // size N when committing
  bool planner_failed = false;
  bool planner_timed_out = false;
};
```

Why action must include assignment metadata:

- Current planner returns a `Solution` and an `assignment_schedule`.
- Simulation immediately applies the first assignment frame after replanning.
- Later timesteps apply per-step assignment key/target frames.
- If action only contains next locations, env cannot update
  `assigned_task_id/current_target` consistently with the one-shot TAPF plan.

Alternative acceptable API:

```cpp
env.accept_plan(LifelongEnvPlan plan);
env.step(policy.next_action(obs, info));
```

But the Gymnasium wrapper still needs each `step(action)` to consume one
timestep. If `accept_plan` exists, wrapper should combine `accept_plan` with
the first step action when `needs_replan` is true.

## 6. Observation Contract

Use fixed schemas with masks. Dynamic task pools must be padded.

Recommended observation content:

```text
timestep
map_id or grid
num_agents
agent_position_index[N]
agent_load_state[N]
agent_assigned_task_id[N]
agent_current_task_id[N]
agent_current_target_index[N]
agent_carried_task_ids[N, capacity]
service_active[N]
service_key[N]
service_target_index[N]
service_progress[N]
task_id[MAX_TASKS]
task_status[MAX_TASKS]
task_type[MAX_TASKS]
task_start_index[MAX_TASKS]
task_goal_indexes[MAX_TASKS, goal_set_size]
task_release_timestep[MAX_TASKS]
task_pickup_timestep[MAX_TASKS]
task_completion_timestep[MAX_TASKS]
task_mask[MAX_TASKS]
valid_action_mask or legal_next_indexes[N, degree+wait]
```

Recommended `info` content:

```text
needs_replan
replan_reason
planner_request
released_task_count
event_happened
pickup_events
completion_events
plan_finished
previous_planner_failed
valid
error
metrics_delta
planner_trace_delta
```

`planner_request` can be richer than Gym obs and C++ native:

```cpp
struct LifelongPlannerRequest {
  int timestep;
  const Graph* graph;
  const MapDistanceCache* distances;
  std::vector<LifelongAgentState> agents;
  std::vector<LifelongTask> tasks;
  std::vector<bool> service_active;
  std::vector<int> service_keys;
  std::vector<int> service_target_indexes;
  std::vector<int> service_progress;
  std::vector<float> inherited_priorities;
  LifelongSimulationConfig config;
};
```

The request should be a snapshot. Policy must not mutate env state through it.

## 7. Planner / Policy Contract

The current LaCAM-TAPF should be wrapped as a policy:

```cpp
class LacamTapfPolicy {
 public:
  LifelongEnvAction act(const LifelongEnvObservation& obs,
                        const LifelongEnvInfo& info);
};
```

Behavior:

- If `info.needs_replan == true`, call a refactored equivalent of the current
  planning block:
  - env-side release of unpicked assignments already happened, or is requested
    through `env.begin_replan()`.
  - build `LifelongPlanningSnapshot`.
  - build `TAPFInstance`.
  - run `solve_tapf`.
  - translate `assignment_schedule` to env action frames.
  - cache the resulting plan.
- If `info.needs_replan == false`, return the next cached action frame.
- If no cached frame exists, return wait action and mark planner failure, so env
  uses the existing failure/wait/replan behavior.

The planner wrapper may keep `planner_anytime`, `planner_force_full_assignment`,
`planner_time_limit_sec`, `service_commit_agents`, and random seed config.

## 8. C++ Refactor Shape

Introduce a core env API, likely in:

```text
lacam/include/lifelong_env.hpp
lacam/src/lifelong_env.cpp
```

Suggested public API:

```cpp
struct LifelongEnvResetResult {
  LifelongEnvObservation observation;
  LifelongEnvInfo info;
};

struct LifelongEnvStepResult {
  LifelongEnvObservation observation;
  double reward = 0;
  bool terminated = false;
  bool truncated = false;
  LifelongEnvInfo info;
};

class LifelongEnvCore {
 public:
  explicit LifelongEnvCore(LifelongSimulationConfig config);
  LifelongEnvResetResult reset(int seed);
  LifelongEnvStepResult step(const LifelongEnvAction& action);
  LifelongSimulationMetrics metrics() const;
};
```

Keep `run_lifelong_simulation(config)` as an adapter:

```text
create LifelongEnvCore
create LacamTapfPolicy
reset
while not terminated/truncated:
  action = policy.act(obs, info)
  step(action)
return env.metrics()
```

This preserves all existing benchmark scripts and tests.

## 9. Python / Gymnasium Wrapper

After C++ core exists, add a Python wrapper:

```text
python/lacam_lifelong_env.py
```

The wrapper should expose:

```python
class LacamLifelongEnv(gymnasium.Env):
    metadata = {"render_modes": ["human", "rgb_array"]}
    action_space = ...
    observation_space = ...
    def reset(self, *, seed=None, options=None): ...
    def step(self, action): ...
```

Use `spaces.Dict` for observations and either:

- `spaces.MultiDiscrete` for per-agent direction/wait actions plus assignment
  metadata encoded separately, or
- `spaces.Dict` action with `next_indexes`, assignment fields, and masks.

For classical LaCAM planner use, the Python action can be an opaque plan/action
object if the wrapper is not targeting generic neural policies yet. For generic
RL, prefer numeric arrays and masks.

## 10. Reward

Initial reward should be simple and diagnostic:

```text
reward = completed_tasks_delta
       - small_wait_penalty * total_waiting_agents
       - invalid_action_penalty
```

Do not optimize reward before Env correctness is locked down. Throughput,
pickup time, delivery time, planner runtime, and final pending/picked/assigned
counts remain primary evaluation metrics.

## 11. Migration Plan

1. Extract `LifelongEnvCore` without changing behavior.
2. Move local lambdas from `run_lifelong_simulation` into env methods:
   - task release for timestep
   - service readiness
   - process arrivals
   - should replan
   - apply joint motion
   - apply assignment frame
   - metric accumulation
3. Make planning block a `LacamTapfPolicy`.
4. Rebuild `run_lifelong_simulation` using env + policy.
5. Compare legacy and env-adapter metrics on deterministic small tests.
6. Add Python/Gymnasium wrapper.
7. Add optional numeric action/observation spaces for learning policies.

## 12. Main Risks

### 12.1 Hidden state mutation during planner request

`prepare_lifelong_planning_snapshot` currently mutates `agents/tasks` by calling
`release_unpicked_assignments`. Refactor this into an env-owned explicit phase:

```text
env.prepare_replan_state()
planner_request = env.make_planner_request()
```

The planner request itself should not mutate env state.

### 12.2 Assignment schedule loss

If the new action only moves agents, task binding diverges from current
simulation. Keep per-step assignment keys/target indexes or introduce an
accepted plan object.

### 12.3 New task release semantics

Task release cannot trigger replanning by itself while a valid cached plan is
running. This is central to current event-driven behavior.

### 12.4 Metrics compatibility

Existing experiments depend on `LifelongSimulationMetrics`, trace CSV, schedule
YAML, and binary path output. Keep metrics identical for the legacy adapter.

### 12.5 Invalid action handling

Gymnasium envs normally should not crash on invalid actions. For this project:

- debug mode may truncate with `info.valid=false`.
- normal mode may convert invalid action to wait plus penalty.

Pick one behavior and make it explicit in tests.

## 13. Contract Tests Added

Add `tests/test_lifelong_env_contract.cpp`.

These tests are opt-in contract tests for the future API. They compile today
with a single skipped placeholder, and become active when the implementation
defines:

```text
LACAM_ENABLE_FUTURE_ENV_CONTRACT_TESTS
```

Expected coverage:

- reset initializes `t=0` backlog and sets `needs_replan`.
- policy can cache one-shot plans while env consumes one joint action per step.
- task release does not interrupt a valid cached plan.
- pickup/completion arrivals trigger replanning.
- planner failure/timeout causes one-step wait and requests replanning again.
- service duration is tracked by env, not hidden inside planner.
- vertex and edge-swap conflicts are rejected or truncated according to config.
- assignment schedule frames update task bindings and current targets.
- future env adapter matches legacy `run_lifelong_simulation` metrics.
- observation shapes remain stable and masks represent dynamic task validity.

Once `LifelongEnvCore` exists, remove the opt-in skip from the default build and
make these tests regular CI coverage.
