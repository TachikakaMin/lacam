/*
 * dd_planner: Carrier-LaCAM search (design.md section 5, milestone M1).
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

// admissible SOC-style heuristic at the initial configuration
// (design 5.7 h_soc); used by tests and f-pruning.
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

// TEST SUPPORT (G1/completeness conformance, debug.md P0-1/P0-2):
// drain ONE node's operator-constraint tree through the production
// machinery (tree expansion + Carrier-PIBT + validator) and return every
// distinct successor configuration it can produce.  Used to compare against
// brute-force validator enumeration.
std::vector<PhysConfig> dd_enumerate_node_successors(const DDInstance& ins,
                                                     const PhysConfig& X,
                                                     int seed);
