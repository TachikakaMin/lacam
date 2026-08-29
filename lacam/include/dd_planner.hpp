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
  long validator_rejects = 0;
  long duplicate_configs = 0;
  long generator_failures = 0;  // Carrier-PIBT returned no joint op
  long max_depth = 0;
  long best_targets_done = 0;
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
