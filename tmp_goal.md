# Goal: partial service state across lifelong replanning

## Problem

LaCAM one-shot planning may produce a full plan until all currently assigned
agents finish service, but the lifelong loop executes only the prefix ending at
the first completed pickup/delivery. Other agents may have already started
pickup/delivery service inside that prefix. The next replanning cycle must
preserve this partial progress when it is still continuous.

## Service semantics

- Pickup and delivery use the same service-progress rule.
- Track per-agent partial service: type, task id, target index, and remaining
  service steps.
- Progress is reusable only for the same agent, same task, same service vertex,
  with continuous staying on that vertex.
- If the next plan keeps the agent there, the service needs only the remaining
  steps.
- If the agent leaves, another agent takes over pickup, or the same agent later
  re-enters, the old progress is discarded and full service duration is needed.
- Delivery handoff is not allowed unless cargo transfer is explicitly added;
  pickup handoff is allowed, but the new pickup starts from full duration.

## Replanning initialization

At the start of each lifelong replanning cycle:

- Validate each stored partial service against current agent location and task
  state. Invalid entries are discarded.
- Build TAPF root state with optional partial service metadata per agent. This
  must not force continuation; it only says continuation is cheaper/faster if
  the agent keeps staying.
- Release ordinary en-route pickup assignments as before. A partial pickup may
  be abandoned or reassigned, but only the original agent can reuse its partial
  progress by continuously staying at the same pickup vertex.
- For delivery, keep the task carried by the current agent; only that carrier
  gets delivery candidates.

## Cost matrix rule

The cost matrix must be state-aware. Do not permanently bake full service
duration into a snapshot offset for agents with partial service.

For each agent-target pair:

```text
cost = distance(current_cell, target) * distance_scale
     + base_static_offset_without_service
     + service_steps(agent_state, target) * distance_scale
```

where:

```text
service_steps = remaining_duration
  if this target continues the same valid partial service

service_steps = full_pickup_or_delivery_duration
  otherwise
```

The same rule must be used during internal TAPF assignment repairs, not only at
the root. If an agent leaves the partial-service vertex, its node state clears
the partial service and later costs use full duration.

Because service cost now depends on agent service state, the dynamic assignment
row cache cannot be keyed only by `(agent_id, cell_id)`. Include partial-service
identity/remaining state in the cache key, or bypass row caching for stateful
partial-service rows.

## Planner behavior

- TAPF service transition logic must support optional continuation:
  staying consumes remaining service; leaving resets progress.
- Continued partial service and restarted service at the same physical target
  must be distinguishable through node state and assignment costs.
- Solution prefix extraction still returns the prefix ending at the first
  completed real pickup/delivery.

## Tests

- Replanning after a prefix preserves remaining service time when the agent
  stays on the same task and vertex.
- Leaving a partially completed pickup/delivery resets progress.
- Reassigning a partial pickup to another agent charges full pickup duration.
- Cost rows charge remaining duration for continuation and full duration after
  interruption.
