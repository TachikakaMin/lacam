# Selector telemetry checkpoint

- Retained parent: `97d3d20ca6b3abc2bdfef5462bb9738726e2a23a`, official throughput 1.632000, hidden mean completed 652.8, hidden mean planner runtime 213439.8 ms.
- Rejected child: `2d6decf93fc1`, static rollout candidates 12 -> 4, official throughput 1.587750 (-0.044250), hidden mean planner runtime 224296.9 ms (+5.09%). Restored retained parent with `coral checkout 97d3d20c`.
- Primary category: algorithm design.
- Paper brief: `coral_tasks/lacam_throughput/papers/briefs/2504.06091.md`.
- Linked source: `coral_tasks/lacam_throughput/papers/arcs-group/2504.06091_2504.06091.html`, finite per-iteration budget/window-control discussion in Sections 1-2.1.
- Diagnostic hypothesis: attempts 4-11 remain valuable because stochastic restarts within repeated guidance policies often produce strictly higher h12 completion counts. The static k=4 cap removed those winners and caused worse downstream trajectories.
- Planned diagnostic-only change: add trace fields for admitted candidate count, winning attempt index/policy, winning rollout completions, number of strict improvements, and optionally runner-up margin/per-attempt elapsed time. Do not alter selection behavior.
- Validation signal: on fixed public seed 0, the completed vector and throughput must remain bit-for-bit equal to retained `hl_agent/runs/hetero-portfolio/seed0.csv`; new trace fields must show nonzero admissions and a distribution of winning attempts. If behavior changes, instrumentation is unsound and must be fixed before interpreting it.
- Commands: build, focused rollout/unit tests, then one sequential seed-0 benchmark with exact fixed scenario args and output under this directory. This diagnostic-only change receives neither a public 10-seed performance screen nor official eval.
- Next behavior decision: if attempts 4-11 win materially, preserve k=12 and test an evidence-backed early-stop or candidate allocation rule; if they almost never win, locate why k=4 changed downstream behavior before trying any new cap.
- Risk: trace plumbing can accidentally perturb timing; all decision state must be local POD bookkeeping and written only after selection.

## Diagnostic outcome

- Build passed. `test_lifelong_simulation` passed 7/7. `test_lifelong_planning` passed 21/25, with four retained/pre-existing service-slot expectation failures unrelated to the telemetry; the command stopped before the remaining binaries, so this is not a candidate test gate.
- Sequential fixed seed 0 completed valid after 212185 ms total planner runtime, but the behavior-invariance gate failed: retained completed/throughput/alternating were `664 / 1.6600 / 419`, while instrumentation produced `667 / 1.6675 / 422`; the per-timestep completed vector differed (`cmp=1`).
- The changed result exactly matches the rejected k=4 candidate's focused seed-0 completed/throughput signal. The seemingly passive local integer bookkeeping perturbed the half-budget wall-clock admission boundary, so all observed winner/admission distributions are invalid for inference.
- Unsound telemetry (not evidence): 379 rows, admitted mean 4.063 (range 3-5), only 2 winners at index 4. These values must not drive a candidate because they describe the perturbed trajectory/admission schedule.
- Root-cause signal: the selector's active candidate count is timing-boundary sensitive. A behavior candidate should replace the unstable `elapsed > 0.5 * budget` admission boundary with a deterministic, paper-grounded work allocation only if it preserves enough stochastic restarts; static k=4 is already officially refuted.
- Status: diagnostic-only source edits will be discarded without a public screen, commit, or official eval. Report retained at `hl_agent/runs/paper_grounded_subagents/selector-telemetry/report.md`.
