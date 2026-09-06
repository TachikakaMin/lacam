/*
 * Exact additive assignment helper for Carrier-LaCAM rho dispatch.
 */
#pragma once

#include <cstdint>
#include <limits>
#include <vector>

using RhoCost = std::int64_t;

// Finite rho costs must stay strictly inside (-INF, INF).  Positive values
// at or above INF represent forbidden edges.
constexpr RhoCost kRhoAssignmentInf =
    std::numeric_limits<RhoCost>::max() / 4;

struct RhoCostBreakdown {
  RhoCost approach = 0;
  RhoCost immediate_service = 0;
  RhoCost continuity = 0;
  RhoCost mode = 0;
  RhoCost urgency_reward = 0;
  RhoCost preparation_reward = 0;
  RhoCost total = 0;
  bool valid = false;
};

// EXECUTE pays only immediate resource use and receives the finite defer
// reward.  Successor critical-tail estimates intentionally are not inputs.
RhoCostBreakdown rho_execute_cost(
    RhoCost approach, RhoCost immediate_service,
    RhoCost continuity, RhoCost mode, RhoCost urgency);

// PREPARE receives at most the task's full defer reward.  It does not pay
// EXECUTE service because it neither Lifts nor completes the transfer.
RhoCostBreakdown rho_prepare_cost(
    RhoCost approach, RhoCost continuity, RhoCost mode,
    RhoCost urgency, RhoCost requested_prepare_reward);

struct RhoAssignmentTiming {
  double optimum_ms = 0;
  double canonical_ms = 0;
};

struct RhoAssignmentResult {
  std::vector<int> row_to_col;
  RhoCost objective = 0;
  bool feasible = false;
  bool overflow = false;
};

// Solves rows <= columns.  Matrix column order is the canonical order:
// robots are fixed row-by-row, choosing the first column that preserves the
// globally minimum additive objective.  Forbidden edges are >= INF.
RhoAssignmentResult solve_rho_assignment_full(
    const std::vector<std::vector<RhoCost>>& cost,
    RhoAssignmentTiming* timing = nullptr);
