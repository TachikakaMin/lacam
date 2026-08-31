/*
 * dd_planner: carrier (two-deck) entry adapters and test-support probes
 * over the integrated LaCAM-TAPF planner (design.md v3 section 10).
 * The ONE solve loop lives in tapf_planner.cpp; nothing here searches.
 */
#pragma once

#include <vector>

#include "dd_carrier.hpp"

using DDPlan = std::vector<std::vector<Op>>;  // per timestep, per robot

struct DDStats {
  long hl_nodes = 0;
  long hl_expanded = 0;
  long pibt_calls = 0;
  long validator_rejects = 0;  // PIBT PROPOSALS rejected (Bug C signal)
  long g1_rejects = 0;         // G1 full-constraint combos rejected (by design)
  long duplicate_configs = 0;
  long generator_failures = 0;  // Carrier-PIBT returned no joint op
  long max_depth = 0;
  long best_targets_done = 0;
  long macro_successors = 0;  // event-bounded rollout successors inserted
  long macro_steps = 0;       // total primitive steps inside macro edges
  long macro_after_first = 0; // macro successors inserted AFTER the first
                              // solution (two-phase policy: must stay 0)
  long g_relaxed = 0;         // duplicate hits relaxed to a cheaper g
                              // (LaCAM*-style rewire, round-2 P2-14)
  long guidance_builds = 0;
  long path_recomputes = 0;
  long path_cache_hits = 0;
  // anytime (design 5.7 / debug.md P3-FOCAL)
  double first_solution_ms = -1;
  double first_solution_soc = -1;  // weighted physical cost (a=b=g=d=1)
  double best_soc = -1;
  long incumbent_updates = 0;
  long f_pruned = 0;
  bool timed_out = false;
  PhysConfig deepest_config;  // config at max depth (debug)
};

// returns empty plan on failure/timeout; a trivially-solved instance yields
// a single all-wait step (never an empty plan on success).
// On failure, if best_effort != nullptr it receives the action sequence to
// the deepest explored node (debug/rollout aid).
DDPlan solve_carrier_lacam(const DDInstance& ins, double time_limit_sec,
                           int seed, DDStats* stats = nullptr,
                           DDPlan* best_effort = nullptr);

// B0 baseline (design 8.1) = Carrier-PIBT standalone: repeatedly apply the
// unconstrained generator from the current configuration until goal, dead
// end, or timeout.  No high-level search.  Shares the rollout core with the
// macro successor (design 7.1, D13).
DDPlan solve_carrier_rollout(const DDInstance& ins, double time_limit_sec,
                             int seed, DDStats* stats = nullptr);

// admissible SOC-style heuristic at the initial configuration (design
// 5.7 h_soc): test oracle for the production f-bound (the planner folds
// the same term into node h via attach_carrier_guidance).
double dd_root_admissible_h(const DDInstance& ins);

// B1 baseline (design 8.1): 2-stage — shelf paths fixed ONCE at the start
// configuration (stage 1), then executed by Carrier-PIBT with the fixed
// plan as a hard constraint (stage 2; requests derive from each target's
// next fixed waypoint).  Deliberately NOT complete: quantifies what the
// unified per-configuration replanning buys.  If fixed_paths != nullptr it
// receives the stage-1 plan (per target, cell sequence).
DDPlan solve_carrier_2stage(const DDInstance& ins, double time_limit_sec,
                            int seed, DDStats* stats = nullptr,
                            std::vector<std::vector<int>>* fixed_paths = nullptr);

// TEST SUPPORT (debug.md v4 WP-C T3, design_final 5.3/D17): run the
// production shelf->goal tau matching for X.  Returns the per-target
// assigned goal CELL; h_out (if given) receives the admissible
// LB-matching h_shelf (primary lexicographic value).  parent_tau
// exercises the eta_B hysteresis tie layer; taboo pairs (target,
// goal-cell) are excluded from matching (|G_b|=1 rows exempt).
std::vector<int> dd_solve_tau(
    const DDInstance& ins, const PhysConfig& X,
    const std::vector<int>* parent_tau = nullptr, double* h_out = nullptr,
    const std::vector<std::pair<int, int>>* taboo = nullptr);

// TEST SUPPORT (debug.md v4 WP-C T5): query the production PathCache for
// target b twice — dst1 then dst2 — on the same X occupancy.  Returns
// the SECOND path; recomputes_out receives the cache's recompute count.
// A dst change must be a cache miss (stale dst1 paths are a correctness
// bug once tau can reassign goals).
std::vector<int> dd_pathcache_dst_probe(const DDInstance& ins,
                                        const PhysConfig& X, int b, int dst1,
                                        int dst2, long* recomputes_out);

// TEST SUPPORT (G1/completeness conformance, debug.md P0-1/P0-2):
// drain ONE node's operator-constraint tree through the production
// machinery (tree expansion + Carrier-PIBT + validator) and return every
// distinct successor configuration it can produce.  Used to compare against
// brute-force validator enumeration.
std::vector<PhysConfig> dd_enumerate_node_successors(const DDInstance& ins,
                                                     const PhysConfig& X,
                                                     int seed);

// TEST SUPPORT (debug.md round-2 P0-5): same enumeration, but applies the
// production livelock re-guidance mutation (taboo rho re-match + order
// shuffle + class re-sort) `n_reguides` times, interleaved with the tree
// drain (first application happens mid-drain to model a partially expanded
// tree).  Outputs the frozen constraint_order before/after for assertion.
std::vector<PhysConfig> dd_enumerate_node_successors_reguided(
    const DDInstance& ins, const PhysConfig& X, int seed, int n_reguides,
    std::vector<int>* constraint_order_before = nullptr,
    std::vector<int>* constraint_order_after = nullptr);

// TEST SUPPORT (debug.md round-2 P0-2, design 5.4a): compute the park
// vector for X.  warm_block_cell >= 0 first warms the per-target path
// cache with that cell forced occupied (occupied->vacated history), then
// guidance is built on the true X occupancy.  strict_inval selects the
// D_b invalidation policy (DD_STRICT_INVAL semantics).  path_out, if
// given, receives target 0's least-blocking path as seen by guidance.
std::vector<uint8_t> dd_compute_park(const DDInstance& ins,
                                     const PhysConfig& X,
                                     int warm_block_cell, bool strict_inval,
                                     std::vector<int>* path_out = nullptr);

// TEST SUPPORT (debug.md round-2 P2-13b): run the production rho matching
// for X and return per-robot free_goal cells.  parent_free_goal, if given,
// simulates the parent node's assignment (cell per robot, -1 = none) for
// the eta-hysteresis term (design 5.3(4), DD_ETA).
std::vector<int> dd_match_free_goals(const DDInstance& ins,
                                     const PhysConfig& X,
                                     const std::vector<int>* parent_free_goal);

// TEST SUPPORT (debug.md round-2 P2-13c): production least-blocking path
// with optional previous-path inertia bias (ties break toward prev; total
// discount strictly below one base cost unit).
std::vector<int> dd_least_blocking_path(const DDGrid& g, int src, int dst,
                                        const std::vector<uint8_t>& occupied,
                                        const std::vector<int>* prev_path);

// TEST SUPPORT (debug.md round-2 P2-13d): run one unconstrained
// Carrier-PIBT step from X and return the chosen joint ops.
std::vector<Op> dd_root_joint_ops(const DDInstance& ins, const PhysConfig& X,
                                  int seed);

// TEST SUPPORT + livelock machinery (debug.md round-2 P2-15): build the
// cross-deck wait-for graph for X (guidance computed internally) and
// return the SORTED ids of robots on cycles (empty = no structural
// deadlock detected).
std::vector<int> dd_waitfor_cycle_robots(const DDInstance& ins,
                                         const PhysConfig& X);

// TEST SUPPORT (debug.md round-2 P2-16d): the parking cell guidance picks
// for robot i in X (DD_PLACE_ESCAPE selects the escape-degree tie-break).
int dd_parking_cell(const DDInstance& ins, const PhysConfig& X, int robot);
