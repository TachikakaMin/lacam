# Goal: Preserve partial service across lifelong replanning

## Problem

LaCAM one-shot planning may solve until all currently assigned agents complete
their service, but the lifelong loop only executes the prefix ending at the
first completed pickup/delivery. Agents that reached a service vertex during
that prefix may have partial pickup/delivery progress. A later replanning cycle
must not forget that progress.

## Required semantics

- Track per-agent partial service state: service type, task id, target index,
  and elapsed or remaining service steps.
- Partial progress is valid only while the agent stays continuously on the same
  service vertex for the same task.
- If the next plan keeps the agent on that vertex, it needs only the remaining
  service time.
- If the next plan moves the agent away, the partial progress is discarded.
  Re-entering the same service later requires the full service duration.
- A task in partial pickup service must remain bound to that agent; ordinary
  replanning must not release it as a fresh pending task.

## Planner/cost changes

- Add this partial service state to the TAPF root state used by the next
  replanning call.
- TAPF service transition logic must support optional continuation: stay and
  consume remaining service, or leave and reset progress.
- Assignment cost offsets must become agent-state-aware:
  - continuing the same partial service uses remaining duration;
  - all other pickup/delivery options use full duration.
- Dynamic assignment row caching must include partial service state in its key,
  or be bypassed for agents with partial service.

## Tests

- Replanning after executing a prefix preserves another agent's partial service
  progress if it stays.
- Moving away from a partially completed service resets progress.
- Cost matrix prefers/charges remaining duration for continuation and full
  duration after interruption.
