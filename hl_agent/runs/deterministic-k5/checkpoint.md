# Deterministic five-candidate portfolio checkpoint

- Parent: `97d3d20ca6b3abc2bdfef5462bb9738726e2a23a`, official throughput 1.632000 and hidden mean planner runtime 213439.8 ms.
- Primary category: algorithm design / real-time work allocation.
- Paper brief: `coral_tasks/lacam_throughput/papers/briefs/2504.06091.md`.
- Linked source: `coral_tasks/lacam_throughput/papers/arcs-group/2504.06091_2504.06091.html`, lines 63-66 and 89-91 (fixed per-iteration planning budget; empirical window sizing to fit it), plus line 187 (overly small warehouse cutoffs harm quality).
- Evidence: static k=4 attempt `2d6decf9` lost 0.04425 official throughput and increased runtime 5.09%, so one candidate per policy is insufficient. Diagnostic-only telemetry changed seed-0 behavior merely through passive bookkeeping, proving that the current half-budget wall-clock admission boundary is unstable and cannot be safely instrumented in-line.
- Hypothesis: execute exactly five candidates per replan: one for each guidance policy `{4,2,8,6}` plus the first repeated-policy stochastic restart (guidance 4). This is the smallest deterministic portfolio that restores stochastic diversity beyond the officially refuted k=4 cap while removing the micro-timing-dependent admission gate.
- Source change: `kRolloutCandidates` 12 -> 5 and remove the `elapsed > 0.5 * budget` break. Retain the per-candidate deadline, h12 rollout, RNG salts, strict-completion selection, and all fixed scenario semantics.
- Magic-number source: 5 is not tune-fitted; it is the minimal count implied by four distinct policies plus one repeated RNG restart after k=4 failed officially. Runtime expectation is only a risk estimate, not a claim: k=4 focused mean invocation was measured 569.6 ms; five exact candidates must be measured against the 1.0-second limit.
- Active-path/validity gate: build and focused tests, then exact fixed public seed 0 sequentially. Require valid output, changed completed vector or other decision signal relative to parent, and max invocation below 1 second. If behavior-active and valid, lifecycle requires a separate ordinary official eval regardless of focused throughput direction.
- Public-screen policy: do not claim a 10-seed mean unless the exact prescribed runner completes. Prior four-worker k4 grid hit its 120-second worker timeout, so a repeated timeout supplies no candidate evidence and must be disclosed rather than averaged partially.
- Retain gate: official throughput must exceed 1.632000. Otherwise return to the already-scored parent without re-evaluating it.
- Risks: five full candidates may violate the per-invocation budget; deterministic work count may remove beneficial later restarts; focused seed 0 is active-path evidence only.

## Focused verification result

- Build succeeded. Focused binaries: `test_lifelong_simulation` 7/7, `test_lifelong_env_contract` 10/10, `test_tapf_planner` 23/24; the single `solve_ita_cbs_yaml_fixture` failure throws `bad file` from its fixture path and is pre-existing/unrelated to this one-file selector change. Total 40/41 pass.
- Exact fixed public seed 0 completed valid with 652 tasks / throughput 1.6300 / alternating completed 416, versus parent 664 / 1.6600 / 419. The completed vector differs, proving the decision path is active.
- Runtime remained inside the fixed per-invocation limit: max 751.923 ms < 1000 ms. Total planner runtime rose from parent 207430 ms to 238377 ms (+30947 ms, +14.92%); mean invocation rose 534.613 -> 617.558 ms (+15.52%).
- Interpretation: deterministic exact-five removes timing-boundary instability but loses 0.0300 throughput on the focused seed and costs substantially more. No 10-seed public mean is claimed; prior four-worker runs of this cost class exceed the runner's per-case timeout.
- Lifecycle decision before official result: behavior-active, valid, build-clean candidate must receive one ordinary `coral eval` now. Likely abandon unless hidden throughput exceeds 1.632000 strongly enough to outweigh the runtime cost.
