# Rollout selector telemetry design

## Checkpoint

- Retained parent: `97d3d20ca6b3abc2bdfef5462bb9738726e2a23a` (official throughput `1.632000`, mean completed `652.8`, mean planner runtime `213439.8 ms`). The rejected static `k=4` child `2d6decf93fc1` scored `1.587750`; this diagnostic is intended to explain that loss, not to change behavior.
- Primary category: **algorithm design** (real-time budget control and portfolio admission).
- Paper grounding: `coral_tasks/lacam_throughput/papers/briefs/2504.06091.md`, linked source `coral_tasks/lacam_throughput/papers/arcs-group/2504.06091_2504.06091.html`.
- Concrete anchor: Sections 1 and 2.1, especially HTML lines 63-66 and 89-91. The paper formulates planning as a fixed finite per-iteration budget, observes that the reached partial-path length is timeout-dependent rather than fixed, and notes that practical window sizes are empirically adjusted to meet the runtime limit. This supports measuring marginal portfolio value and time before another budget-control change; it does not justify a benchmark-specific cap.
- Inspected selector: `lacam/src/lifelong_env.cpp:1487-1583`. It admits contiguous attempts `0..n-1` until the half-budget gate, cycles guidance costs `{4,2,8,6}`, and replaces the incumbent only when `completions > best_completions` (apart from first-nonempty fallback). Therefore the final winner is the earliest admitted attempt attaining the maximum valid score; a later tie never wins.
- Existing plumbing: `LifelongPlannerTraceRecord` in `lacam/include/lifelong_simulation.hpp`, copied from `LifelongEnvAction` by `LifelongEnvCore::record_planner_result` in `lacam/src/lifelong_env.cpp:797-820`, and serialized by `write_trace_csv` in `tools/lifelong_benchmark.cpp:89-224`. Focused policy tests live in `tests/test_lifelong_env_contract.cpp`; trace-record tests live in `tests/test_lifelong_simulation.cpp`.
- Files touched by this subtask: this report only. No source edit, build, benchmark, public screen, or `coral eval` was run.

## Smallest useful telemetry

Add the following to `LifelongPlannerTraceRecord`, populate them locally inside `LacamTapfPolicy::replan`, and append them to the existing trace CSV:

1. `int rollout_candidates_admitted` — count an attempt immediately after it passes the half-budget gate, including empty/invalid attempts.
2. `int rollout_winner_attempt` (default `-1`) and `int rollout_winner_guidance_cost` (default `-1`) — update in the exact branch that updates `solution`; recording both avoids making analysis depend on the current portfolio order.
3. `std::vector<int> rollout_attempt_completions` — one entry per admitted attempt in index order. Use `-2` for an empty outer solve and retain `-1` for a nonempty candidate whose assignment/rollout pipeline is invalid; nonnegative values are valid h12 completion scores.
4. `std::vector<double> rollout_attempt_runtime_ms` — one wall-clock duration per admitted attempt, covering its outer solve plus closed-loop scoring and all failure paths.

No separate best/margin/tie counters are needed: the vectors and winner fields derive every requested statistic without redundant state. For each planner row:

- admitted count = `rollout_candidates_admitted` (also assert it equals both vector lengths);
- winning attempt/policy = the two winner fields;
- later repeated-policy win = `winner_attempt >= 4`;
- baseline strict margin = `max(valid_scores) - score[0]` when attempt 0 is valid;
- winner tie multiplicity = count of valid scores equal to the maximum;
- runner-up strict margin = maximum minus the greatest strictly lower valid score (undefined if all valid scores tie);
- policy win rates and time distributions = group rows by `winner_guidance_cost` and attempt-array position.

Implementation detail: reserve capacity 12 before the loop; perform no logging or allocation-dependent formatting inside it. Take the start timestamp only after admission and append the duration on every exit, including the current empty-solution `continue`. Assign the completed vectors to `action.planner_trace` only after selection. A tiny helper/finalizer is preferable to duplicated append logic because missing the early `continue` would make attempt indices silently misalign.

## Focused validation (do not run an official or 10-seed screen)

Build and focused tests:

```bash
cmake --build build --target test_lifelong_env_contract test_lifelong_simulation lifelong_benchmark -j4
./build/test_lifelong_env_contract \
  --gtest_filter=lifelong_env_contract.policy_replans_once_then_serves_cached_step_action
./build/test_lifelong_simulation \
  --gtest_filter=lifelong_simulation.congestion_cost_mode_records_target_region_trace_metrics
```

Add one focused contract assertion (either to the existing policy test or a new rollout-telemetry test): admitted count is positive; both vectors have exactly that size; winner is `-1` iff no solution was selected, otherwise it is in range; winner guidance matches `{4,2,8,6}[winner % 4]`; every duration is nonnegative; and for ordinary valid candidates the winner is the first maximum. Add a trace-CSV smoke check by running exactly one sequential public seed:

```bash
python tools/run_symbotic_requested_grid.py \
  --binary build/lifelong_benchmark \
  --out-dir hl_agent/runs/selector-telemetry/seed0-instrumented \
  --seeds 0 --workers 1 --timeout-sec 300 --force \
  --maps symbotic_star --ks 2 --slots -1 --agent-counts 100 \
  --dists 50_50 --durations 4 --cost-modes -1 \
  --horizon 400 --time-limit-sec 1.0 --goal-set-size 3 \
  --release-interval 10 --service-commit-agents -1
```

Parse `runs/.../result.csv.trace.csv` with Python's `csv` module, not string splitting, because the two vector cells should be quoted and delimiter-encoded. Confirm every planner row has aligned vectors and summarize admitted-count, winner-index/policy, later-win, tie/margin, and per-attempt-time distributions.

The instrumentation safety gate is stricter than throughput equality: compare the candidate run with retained `hl_agent/runs/hetero-portfolio/seed0.csv` and its trace. It must remain valid with `completed_tasks=664`, `throughput=1.66`, `alternating_completed_tasks=419`, and the same per-timestep cumulative `completed_tasks` vector. If a retained schedule artifact is regenerated before instrumentation, compare the sparse schedule binaries byte-for-byte as well. Any changed completion vector, selected targets, or motion schedule means the extra timing/bookkeeping perturbed the active path; treat the change as unsound diagnostic instrumentation and do not interpret the telemetry or run broader screens.

Expected decision signal: the focused run yields an empirical distribution of how many candidates the gate actually admits, whether indices 4-11 are first strict maxima or merely ties, their h12 margin over attempt 0/earlier candidates, and their marginal wall time. This directly separates useful repeated-policy RNG diversity from redundant restarts and from rows where the existing time gate already prevents late admission.

## Risks and outcome-contingent behavior candidates

- **Measurement perturbation:** extra clock reads and vector bookkeeping occur before later gate/deadline decisions. Pre-reserving and deferring serialization minimize this, but only the behavior-invariance check can validate it.
- **Sentinel ambiguity:** empty outer solves and invalid rollouts currently both otherwise disappear into the control flow. Distinct `-2`/`-1` scores are required before computing ties or margins.
- **Prediction is not causality:** h12 completion margins are selector predictions. A winning late candidate can still worsen realized downstream runtime; retain realized completion/runtime comparisons in any behavior-candidate evaluation.
- **Legacy path:** the legacy simulation path does not use this selector, so its new fields should remain defaults. Analysis must filter to planner rows with `rollout_candidates_admitted > 0`.
- **CSV stability:** vector cells must be quoted and use a delimiter that cannot be confused with the outer comma-separated format.

Highest-EV next behavior candidate for each telemetry outcome:

| Telemetry outcome | Implied behavior candidate |
|---|---|
| Attempts 8-11 frequently win by a positive completion margin | Keep `k=12`; preserve all restarts. Reorder the empirically strongest guidance/restart strata earlier under the same gate, then real-evaluate that behavior change. |
| Attempts 4-7 win materially but 8-11 almost never strictly improve | Test an evidence-backed `k=8` cap (not `k=4`) or stop after the second policy cycle; submit it through the mandatory behavior-candidate lifecycle. |
| One guidance cost dominates winners across multiple restart cycles | Allocate earlier RNG restarts to that policy while retaining at least one candidate for each other policy; do not simply delete diversity before checking margins. |
| Late candidates are expensive and mostly tie the incumbent | Test an adaptive early stop after a complete policy cycle when subsequent restart cycles provide no strict improvement, rather than a static cap independent of observed scores. |
| The gate usually admits four or fewer candidates | Candidate count did not directly explain the k4 loss on those rows. Instrument/compare deadline slack and downstream replan cost next; avoid another portfolio-size candidate. |
| Late strict wins are rare, yet k4 changed the focused trajectory | First audit time-boundary sensitivity and RNG/selection equivalence. The highest-EV candidate is no behavior change until the discrepancy is explained. |
| Late strict wins are common but have zero/negative realized downstream value in paired diagnostics | Replace raw h12 completion-only ranking with a paper-grounded cost-aware tie-break/score in a separate candidate; retain strict completion as the primary criterion until that paired signal is demonstrated. |

The retained `k=12` chain remains the default unless telemetry identifies a narrower, active rule. Static one-candidate-per-policy admission is already refuted by official attempt `2d6decf93fc1`.
