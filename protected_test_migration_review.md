# Protected Test Migration Review

Date: 2026-09-03

Reviewer: independent `openai.gpt-5.6-sol`, reasoning effort `xhigh`

Decision: **APPROVE**

The reviewer confirmed that replacing the Objective-PIBT mechanism tests with
the 39 Task-BR-PIBT behavior contracts in `design_final.md` is required by the
new specification and does not weaken correctness. The approval is conditional
on an atomic migration:

1. Add production-path tests for the new contracts and observe RED before
   deleting old mechanism gates.
2. Preserve physical-state, eligible-goal, `apply_ops()`, exhaustive successor,
   zero-shelf compatibility, admissible-heuristic, repair, replay, deadline,
   fixed benchmark case/seed, and strict 10-second assertions.
3. Migrate rather than silently delete behavior contracts for injective
   matching, reversible settled blockers, priority cutoff, idle avoidance,
   custody continuity, rollout freshness, and rewire correctness.
4. Add explicit assertions for exact `(shelf, from, to)` identity, transactional
   rollback, `paused_roots`, three-stage custody recovery, full successor-set
   preservation, candidate-edge trace replacement, cache-history purity, and
   all three rho lexicographic layers.
5. Audit production source for removal of Objective options, active-root
   truncation, one-empty specialization, park, taboo/reguide, cooldown,
   frozen-guidance refresh, and old ablation flags.

The reviewer made no file changes. The full decision and per-test migration
list are preserved in the parent Codex thread.
