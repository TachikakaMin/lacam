# Service-duration action plan

## Goal

Add a service action for pickup and delivery locations. The action is only valid
at the assigned service location and lasts `n` discrete transitions. During the
action the agent remains on that vertex, so normal TAPF vertex and edge-conflict
logic must account for the blockage.

## Semantics

1. A pickup service can start only when an agent is at the assigned pending task
   start vertex.
2. A delivery service can start only when a loaded agent is at a goal vertex of
   one of its carried tasks.
3. A service of duration `n` requires `n` consecutive stay transitions at the
   service vertex.
4. The transition that enters the service vertex does not count as service
   progress. Therefore `duration = 1` means one extra wait transition after
   arrival.
5. If the agent starts the planning round already at the service vertex, the
   root state does not count as service progress. The first self-loop transition
   counts as progress 1.
6. Service progress belongs to the planner node state. Two nodes with the same
   positions but different service progress are distinct search states.
7. The task state changes only after service completion:
   - pickup completion calls `try_pickup`
   - delivery completion calls `try_complete`
8. While service is incomplete, the service agent cannot change assignment or
   leave the service vertex.

## Planner design

Extend `TAPFSearchConfig` with service durations:

```cpp
int pickup_service_duration = 1;
int delivery_service_duration = 1;
```

Then extend `TAPFNode` service state. The existing `satisfied[i]` is not enough,
because "at target" and "service completed" are different states.

Suggested node fields:

```cpp
std::vector<int> service_assignment;
std::vector<int> service_progress;
std::vector<bool> satisfied;
std::vector<int> satisfied_assignment;
```

`service_assignment[i]` is `-1` when the agent is not currently committed to an
incomplete service. Once it starts service, it remains fixed until completion.

## State transition

For each child node:

1. Build the next config with the existing PIBT/reservation machinery.
2. For each agent:
   - if it has incomplete service, require `C_next[i] == C_parent[i]`;
   - if it newly reaches its assigned service target, commit to that service
     target with progress 0;
   - if it was already servicing and stays at the target, increment progress;
   - if progress reaches the configured duration, set `satisfied[i] = true`.
3. Fixed assignment rules:
   - incomplete service fixes the current task column;
   - completed service fixes `satisfied_assignment[i]`;
   - other agents may still be reassigned normally.

## Closed key

Replace the current service closed key:

```cpp
ServiceConfigKey{C, satisfied}
```

with:

```cpp
ServiceConfigKey{C, satisfied, service_assignment, service_progress}
```

Without this, the search can collapse an incomplete service state into an
equivalent-position state with different remaining service time.

## Goal and truncation

`is_goal_node` must count only completed services. Solution truncation must stop
at the first step where the configured number of required real services has
completed, not at the first arrival at a service target.

## Lifelong simulation integration

The simulation layer should not independently decide that arrival means pickup
or delivery. Instead:

1. Ask TAPF for a service-complete execution prefix.
2. Execute that prefix.
3. At the final timestep, use the assignment schedule/key to identify the
   completed service and call `try_pickup` or `try_complete`.
4. Trigger event-driven replanning after the state transition.

This preserves the current event-driven architecture while making the service
duration visible to planning.

## Cost and heuristic

Add service duration to assignment costs:

The current lifelong planner encodes the cost matrix as:

```text
assignment_cost(i, option) =
    distance(current_i, target_option) * distance_scale(i, option)
  + cost_offset(i, option)
```

where pickup and delivery use different denominators through
`common_cost_scale`:

```text
pickup distance_scale  = common_cost_scale / (carried_count + 1)
delivery distance_scale = common_cost_scale / carried_count
```

Service wait must be included in the same scaled numerator before
`scaled_static_cost(...)`, not bolted on after the matrix is built. Otherwise
Hungarian/TAPF assignment will still think a service target is cheaper than it
really is.

Pickup option:

```text
pickup_static_cost =
    direct_delivery_estimate(new_task)
  + circle_cost(task.start, carried_tasks + new_task)
  + loaded_distance_since_last_delivery_if_loaded
  + switched_task_penalty
  + pickup_service_duration

pickup_offset =
    scaled_static_cost(
        pickup_static_cost,
        common_cost_scale,
        carried_count + 1)
```

Delivery option:

```text
delivery_static_cost =
    circle_cost(goal, carried_tasks)
  + delivery_service_duration

delivery_offset =
    scaled_static_cost(
        delivery_static_cost,
        common_cost_scale,
        carried_count)
```

This means the extra service wait participates in assignment selection exactly
like the rest of the lookahead cost. The dynamic travel-to-target part remains in
`distance * distance_scale`; only the service-time constant goes into the offset.

For agents already in the middle of service, heuristic should include remaining
service transitions directly:

```text
remaining_service = required_duration - service_progress
h += remaining_service * active_service_distance_scale
```

The active scale should match the committed service option's scale so that a
partially completed pickup/delivery remains comparable to the original cost
matrix row.

## Tests to add first

1. Root service duration: one agent starts at its target, duration 3, returned
   path has four configs and three stay transitions.
2. Blocking service: one agent services a middle vertex for duration 3 while
   another agent needs to pass through it; the second agent cannot occupy the
   vertex during the service window.
3. Consecutive service: a service does not complete unless the agent stays at
   the target for `n` consecutive transitions after arrival.
4. Lifelong pickup delay: pickup timestamp is delayed by the configured pickup
   service duration.
5. Lifelong delivery delay: completion timestamp is delayed by the configured
   delivery service duration.

## Implementation order

1. Add configuration fields and disabled specification tests.
2. Extend `TAPFNode` service state and `ServiceConfigKey`.
3. Enforce stay-only successors for incomplete service.
4. Update satisfaction, goal, heuristic, and solution truncation.
5. Wire completed service identity back to lifelong simulation.
6. Enable tests one by one.
