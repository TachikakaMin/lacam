# dd-lacam Benchmark & Baselines

Benchmark infrastructure for the Carrier-LaCAM project (design.md §8, §10 M0).
This directory contains the instance format, the two-deck transition
validator, instance generators, converters to external baseline formats, the
B4 baseline, and the benchmark runner.  **No Carrier-LaCAM algorithm here.**

## Baseline availability (design.md §8.1)

| Baseline | Code | Status |
|---|---|---|
| B2: CREST / MAPF-DECOMP-style execution | [ChristinaTan0704/CREST](https://github.com/ChristinaTan0704/CREST) | cloned at `baselines/CREST`, builds & runs. `crest_base` = all constraint-release flags off (MAPF-DECOMP-style decomposed execution); `crest_full` = `--STR --DW --GTR`. |
| B3: NAT-CBS (MAWR, SoCS'25 best paper) | [CRL-Technion/wh-rearrangement](https://github.com/CRL-Technion/wh-rearrangement) | cloned at `baselines/wh-rearrangement`, builds & runs (`MAWR -a NATCBS`). Makespan-optimal; small instances only. |
| B3 alt: MARPF (arXiv 2403.12376) | none published | not available — ILP formulation, no public repo found. |
| BR-LaCAM / BRaP (arXiv 2509.01022) | none published | not available. B1 (2-stage) will reuse our own Carrier-PIBT later per design.md. |
| DD-MAPD / MAPF-DECOMP original (arXiv 2304.14309) | none published | CREST baseline mode stands in for it (same decomposition + 1-robust ECBS shelf stage). |
| B4: single-robot sequential simulation | ours | `ddbench/b4_baseline.py` (Theorem 1 construction; recursive blocker clearing; shoos idle robots). |
| B0/B1 | ours (Carrier-PIBT) | deferred — part of the algorithm work, not this milestone. |

## Build environment for external baselines

Both C++ baselines build inside a user-level micromamba env `ddtool`
(GCC 13, CMake 3.28, Boost, Eigen, libortools):

```sh
export MAMBA_ROOT_PREFIX=$HOME/micromamba
micromamba run -n ddtool bash -c 'cd baselines/CREST && cmake -S . -B build -DCMAKE_PREFIX_PATH=$CONDA_PREFIX && cmake --build build -j8'
micromamba run -n ddtool bash -c 'cd baselines/wh-rearrangement && cmake -S . -B build -DCMAKE_PREFIX_PATH=$CONDA_PREFIX && cmake --build build -j8'
```

Environment quirks that were patched (env-side, not in the repos):
- `$CONDA_PREFIX/bin/fzn-ortools` stub created (ortools cmake target references
  it; we don't use FlatZinc);
- `PREFIX=` line added to `$CONDA_PREFIX/lib/pkgconfig/coinutils.pc`.

## Instance format (YAML)

```yaml
name: scramble_h8w8r2s8t4k40_seed0
map: |     # '.' free, '@'/'T'/'#' wall
  ........
robots:    # labeled robot starts [row, col]
  - [1, 0]
shelves:   # ALL shelf cells (upper deck occupancy), incl. target starts
  - [0, 1]
targets:   # labeled target shelves
  - {id: b0, start: [0, 1], goal: [0, 3]}
flags: {}
anon_goals:          # OPTIONAL witness pairing for anonymous shelves
  - [[2, 2], [3, 3]] # [start, goal]; realizable full layout hint for
                     # full-goal-layout baselines. NOT part of our goal.
```

Goal condition (design.md §2.2): every target grounded at its goal; anonymous
shelves unconstrained; robot end positions unconstrained.

## Components

- `ddbench/validator.py` — the single implementation of the §3.3 rule table
  (R1 R2 S1 I1 I2 I3; S2 implied by R2). Also `plan_cost` (executed makespan,
  weighted SOC per §2.3).
- `ddbench/generators.py` —
  - `scramble_with_witness`: scrambles backwards from the goal configuration
    with random legal joint actions (validator = only legality oracle); the
    reversed trajectory is a legal solution (feasibility by construction).
    Internally scrambles a fully-labeled twin so shelf identities are tracked,
    then anonymizes; `anon_goals` records the witness pairing.
  - `ddmapd_instance`: DD-MAPD protocol (2×2 blocks to density, perimeter
    agent starts, free-cell goals).
- `ddbench/converters.py` — to CREST (`map`+`scen`) and MAWR (`map`+`scen`).
  CREST semantics: map `@` = shelf (movable); every `@` not listed as a task
  becomes an implicit goal=start task. MAWR scen: all shelves are obstacles
  `pcol prow dcol drow`; agents `col row`.
- `ddbench/b4_baseline.py` — B4.
- `generate_instances.py` / `run_benchmark.py` — see below.

## Running

```sh
cd benchmark
# unit tests (validator semantics §6.5, scrambler witness, B4)
PYTHONPATH=. python3 -m unittest discover -s tests

# instance sets
python3 generate_instances.py --out-dir instances_small --small --seeds 3
python3 generate_instances.py --out-dir instances_standard --seeds 3

# run baselines -> rows.csv
python3 run_benchmark.py --instances instances_small --out-dir results_small --timeout 60
```

`rows.csv` columns: instance, family, method, success, executed_makespan,
weighted_soc, loaded_moves, free_moves, lift_drop, runtime_sec, status, raw.

## Paper-protocol suites (CREST arXiv:2603.28803 Table I)

The papers do not publish their instance data (CREST repo ships a single
24x24 demo; MAWR ships none), so `generate_paper_instances.py` regenerates
instances following the papers' textual protocols:

| Layout | Size | Shelves | Rearranged | Agents | Seeds |
|---|---|---|---|---|---|
| R2R-M (DD-MAPD protocol) | 48×48 | 460 (20%) | 230 (=0.1·48²) | 32 | 25 |
| S2W-M | 64×33 | 384 | 384 | 32 | 25 |
| DnE-M | 47×45 | 672 | 403 | 32 | 25 |

```sh
python3 generate_paper_instances.py --out-dir instances_paper --seeds 25
python3 run_benchmark.py --instances instances_paper_main \
  --out-dir results_paper_main --methods b4 crest_base crest_full \
  --timeout 1800 --jobs 24
```

The runner is process-parallel (`--jobs`); wall time and per-method solver
time are recorded in `<out>/timing.json`.  On the 32-core machine we run with
24–25 workers, keeping ~5–7 threads free.

Paper-suite results with the **-O3 patched build** (2026-08-29,
`results_paper_o3`, timeout 1800 s, jobs=25, wall 2449 s; b4/s2w numbers
from the earlier runs — b4 is pure Python, unaffected by the C++ flag bug):

| family | method | solved | mean executed mk | mean runtime (s) | failures |
|---|---|---|---|---|---|
| r2r_m | b4 | 25/25 | 15892.4 | 5.5 | |
| r2r_m | crest_base | 24/25 | 546.1 | 3.3 | 1× upstream rc=1 |
| r2r_m | crest_full | 19/25 | 478.9 | 858.7 | 5× upstream rc=1, 1× rc=-9 |
| dne_m | b4 | 25/25 | 34801.2 | 17.9 | |
| dne_m | crest_base | 23/25 | 980.3 | 12.9 | 2× upstream rc=1 |
| dne_m | crest_full | 0/25 | — | — | 19× upstream rc=1, 6× timeout |
| s2w_m | b4 | 25/25 | 39622.4 | 14.3 | |
| s2w_m | crest_base (w=5 probe) | 0/2 | — | — | ECBS stage timeout |

Makespans are identical between -O0 and -O3 runs on commonly-solved
instances (deterministic solver); only runtimes changed.  With -O3, the
previously-timed-out r2r crest_full instances now run far enough to hit the
upstream logic bug (signature B below) instead — i.e. the remaining
failures are correctness bugs, not slowness.

Findings:
- On R2R-M, constraint release improves makespan by ~12% over the
  MAPF-DECOMP-style baseline (478.9 vs 546.1), directionally consistent with
  the paper's reported 9.2% R2R-M improvement.
- crest_base at -O3 (3.3 s R2R-M incl. built-in ECBS stage) is now in the
  same ballpark as the paper's 9.5 s Time column.
- crest_full at -O3 needs ~860 s vs 18 s reported; residual gap is
  hardware/code-version, but no longer the absurd 120x (see bug section).

Experiment wall-clock times (all recorded in each `results_*/timing.json`):
- `results_paper_main` (-O0 build, 150 tasks, jobs=24): 3909 s
- `results_paper_full3600` (-O0 build, 50 tasks, jobs=25): 5863 s
- `results_paper_o3` (-O3 build, 100 tasks, jobs=25): 2449 s
- `results_paper_s2w_b4` (25 tasks, jobs=20): 59 s
- `results_s2w_probe` (2 tasks, jobs=2): 1830 s

- Known runtime-accounting difference: the paper's Time column excludes
  initial shelf-plan generation and was measured on their server; our
  runtime_sec includes the binary's built-in ECBS trajectory stage.

S2W-M limitation: the public CREST binary only contains ECBS for trajectory
generation; the paper generated S2W/DnE shelf plans externally with
MAPF-LNS2 (600 s), which is not shipped. 1-robust ECBS on the dense S2W
staging zone times out at 1800 s even with suboptimality 5.0
(`results_s2w_probe`), so S2W is currently only covered by B4
(b4 solves it, e.g. seed0 mk=40240 in 13.3 s).

Orphan-process note: external solvers are started in their own session and
killed via killpg on timeout; earlier runs used the micromamba wrapper whose
killed wrapper left orphaned 100%-CPU CREST children (cleaned up manually).
Binaries are now invoked directly with LD_LIBRARY_PATH to avoid the
micromamba run lock under high parallelism.

## Baseline bugs found (2026-08-29, pinned in tests/test_baselines.py)

**Bug 1 — upstream CREST builds at -O0 (root cause of the "120x runtime
gap").** `baselines/CREST/CMakeLists.txt` hardcodes
`set(CMAKE_CXX_FLAGS "... -g -O0")` and force-overrides `CMAKE_BUILD_TYPE`,
so `-DCMAKE_BUILD_TYPE=Release` is silently ignored and the last `-O0` wins.
Measured impact (A/B, same machine):

| case | -O0 | -O3 (patched) |
|---|---|---|
| official demo, full strategies | 7.9 s | 1.2 s |
| crest_base on R2R-M seed0 | 28.3 s | 1.9 s |
| crest_full on R2R-M seed0 | timeout @3600 s | 422 s |

We patch the vendored CMakeLists to `-O3` (marked `dd-lacam local patch`).
Guarded by `TestCrestBuildOptimized`: fails if `flags.make` ends with `-O0`
or if the demo runtime regresses past 5 s.  With -O3, crest_full remains
~20x slower than the paper's 18 s Time column (which excludes trajectory
generation and used different hardware/code version), but is no longer
absurd.

**Bug 2 — upstream CREST executor failures on DnE-M (logic, not perf).**
Two distinct exit-1 signatures, both reproduced at -O0 and -O3:
- `can not find path for agent ... taskID ...` (MLSIPP retry loop, then
  exit 1); triggers on 25/25 DnE instances in full mode and 2/25 (seed5,
  seed6) in baseline mode — the bug lives in the shared executor, and
  constraint release only raises the trigger probability.
- `dep from shelf X step k to shelf Y step k is added before the planned
  step` (dependency-graph consistency check).
Pinned canaries in `TestCrestUpstreamPathfindingBug` with fixtures
`tests/fixtures/dneM_n32_seed{0,6}.crest.{map,scen}`: seed6 fails in BOTH
modes, seed0 solves in base but fails in full.  If upstream ever fixes it,
the canary fails with an explicit "re-run the suites" message.

Run all baseline regression tests:

```sh
cd benchmark && PYTHONPATH=. python3 -m unittest tests.test_baselines -v
```

## Known semantic gaps (recorded, intentional)

1. CREST/NAT-CBS need a *complete* goal layout; our anonymous shelves are
   unconstrained. Converters use the scrambler's witness pairing
   (`anon_goals`), which is realizable by construction — but 1-robust
   decomposition (CREST ECBS stage) or makespan-optimal search (NAT-CBS) may
   still fail/time out on it. Those failures are *findings* (completeness
   gap, design.md Prop. 2), not conversion bugs: the witness plan proves each
   instance feasible in the unified model.
2. CREST executor has no wall concept (`@` = shelf); only wall-free instances
   are exported to it (`ConversionError` otherwise).
3. CREST metrics: `makespan` is executed makespan over ALL shelves reaching
   their delivery (incl. anonymous relocations), which is the objective the
   decomposed method must solve; our executed_makespan for B4 counts targets
   only. Comparisons must note this.
4. NAT-CBS reports makespan only (no SOC decomposition).

## First results (2026-08-28)

Small suite (21 instances, 60 s timeout): b4 21/21, crest_base 13/21,
crest_full 13/21, natcbs 14/21.  Standard suite (36 instances, 120 s):
b4 28/36, crest_base 14/36, crest_full 14/36, natcbs 8/36 (24 skipped as too
large for the optimal solver, 4 timeouts).

Observations:
- On instances solved by all: natcbs ≤ crest ≤ b4 in executed makespan, as
  expected (28 pairwise comparisons; one case where natcbs > b4 is explained
  by natcbs solving the harder full-layout objective).
- All crest failures are ECBS-stage "Failed to find solutions" / crashes
  (rc=134/139) on scrambler instances whose feasibility is witness-proven —
  concrete evidence for the 1-robust decomposition completeness gap
  (design.md Prop. 2).
- b4 fails on dense instances (fill ≥50%, ddmapd d≥0.5): its greedy clearing
  can displace completed targets and does not re-serve them. Its success rate
  *is* the measurement; do not tune it into a real solver.

## Round-2 audit follow-ups (2026-08-30)

- **Current default config = two-phase anytime + macro scale regime**
  (design.md §7.1/D14, commit 6670e22): phase 1 finds a fast upper bound
  (macro only when `|B_tgt| <= DD_MACRO_TGT` = 64), phase 2 restarts a
  primitive-only quality search from the root with that bound.
  `results_final_v3/rows.csv` (carrier only, unified 10 s) vs the older
  `results_final_v2` defaults on identical instances: R2R makespan
  1673→614, DnE 3934→1296, S2W 3288→1292 (2.5–3.0×); small/std/sweep
  unchanged; totals 158/162 vs 160/162 (dneM seed18/22 need the macro
  regime — recoverable via `DD_MACRO_TGT`).
- **`no_astar` ablation voided**: `DD_NO_ASTAR` was never read by any
  production source (guidance paths are plain Dijkstra; there is no A*
  toggle).  The variant was removed from `run_ablations.py`; `no_astar`
  rows in older `results_ablation/ablation_rows.csv` are not to be cited.
  `tests/test_tools.py::TestAblationEnvWiring` now statically checks that
  every knob the ablation runner sets is read by production code.
- **Sweep 1:50 tier added**: `instances_sweep/ratio_r4x50` (40×40, 4
  robots, 200 shelves, 8 targets; the 12×12 base map cannot hold 50
  shelves per robot).
- **CLI**: `dd_benchmark` now echoes `mode=` and rejects unknown modes
  (exit 2) instead of silently falling back to `lacam`.

### Final round-2 results (2026-08-30, results_final_v4, jobs=16 physical cores)

164 instances (162 + two 1:50-tier), unified 10 s, one seed:

| method | solved | notes |
|---|---|---|
| carrier (ours) | **162/164** | r2r 25/25 mk 548 (crest_base parity: 546), dne 24/25, s2w 25/25 |
| carrier_b0 | 147/164 | no-search ablation |
| b4 | 115/164 | single-robot; paper-scale mostly timeout |
| crest_base | 81/164 | best external; r2r-only at paper scale + 19 dne |
| carrier_b1 | 77/164 | frozen-plan ablation; 0/75 paper |
| crest_full | 38/164 | |
| natcbs | 21/164 | optimal, small maps only |

carrier quality evolution on identical paper instances (makespan mean):
r2r 1673 -> 548 (3.1x), dne 4024 -> 1473 (2.7x), s2w 3288 -> 1277 (2.6x)
vs the pre-round-2 defaults.  Open: dneM_seed22 (solvable only with
Hungarian rho above the 256-target regime boundary — uniform-boundary
trade-off), one std scramble.

Timing record: full 1148-task suite 470 s wall at jobs=16; 99-task
ablation 46 s (parallelized runner).  Benchmarks are pinned to PHYSICAL
cores (16) — hyper-threaded oversubscription (jobs=27) starved 9s-tail
tasks and produced false timeouts (s2w 12/25 vs 25/25).

### Skeleton-reuse migration (2026-08-30, audit-driven)

DD now runs on components shared with (or extracted verbatim from) the
original lacam-tapf skeleton: `tapf_hungarian_row_to_col` (the original
Hungarian, DD copy deleted), `lazy_dist.hpp` (original DistTable
resumable-BFS semantics; upstream DistTable/TAPFDistTable are adapters
of the same core), original weighted-FOCAL selection as the DD default
(historical DD_FOCAL_W knob — retired at the v3 integration: selection
is now the original TAPF weighted FOCAL, w=1.5, always on — see
results_final_v5/ for the pre-integration gate),
and `search_kernel.hpp` (constraint-tree expansion driver + PIBT
recursion frame; Carrier semantics live in policy hooks).  All swaps
verified bit-identical on benchmark SOC.  Upstream back-adoption of the
kernel by planner.cpp/tapf_planner.cpp is deferred with recorded
reasons (vector-form Constraint), per the audit's staged-migration
recommendation.


## Integrated planner results (design.md v3, 2026-08-30)

The carrier planner is now an incremental extension of the LaCAM-TAPF
planner (single solve loop `TAPFPlanner::solve()`; `dd_planner.cpp` is a
thin adapter layer).  Shelf-free TAPF behavior is pinned bit-for-bit by
`tests/test_tapf_compat.cpp`.  Official post-integration results:
`results_integrated_v2/rows.csv` (164 instances x 7 methods, unified 10 s,
jobs=14 physical cores): carrier 162/164 (= pre-integration v5; success
set differs by +dneM_seed22 +ddmapd_h24d60t16_seed2 / -dneM_seed18
-s2wM_seed24), carrier_b0 154, b4 115, crest_base 80, carrier_b1 74,
crest_full 38, natcbs 21.  Family-mean makespan on commonly solved
instances is at parity with v5 (r2r 1.03, s2w 0.99, standard_ddmapd 1.00)
except DnE-M at +12% (recorded as an open regression in debug.md WP6).
