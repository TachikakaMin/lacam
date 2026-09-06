/*
 * Exact additive assignment helper for Carrier-LaCAM rho dispatch.
 */
#pragma once

#include <cstdint>
#include <limits>
#include <vector>

using RhoCost = std::int64_t;
using RhoWideCost = __int128_t;

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

// Objective-only warm state for the additive rho assignment.  Rectangular
// rows <= columns instances are completed with zero-cost logical dummy rows,
// so mate_l/mate_r and the dual vectors describe one square perfect
// matching.  The matching is not required to be canonical: production keeps
// using solve_rho_assignment_full() for that separate contract.
struct RhoHungarianState {
  int real_rows = 0;
  int columns = 0;
  std::vector<int> mate_l;
  std::vector<int> mate_r;
  std::vector<RhoWideCost> row_potential;
  std::vector<RhoWideCost> column_potential;
  std::vector<std::vector<RhoCost>> real_cost;
  RhoCost objective = 0;
  bool valid = false;
};

struct RhoStateSolveResult {
  RhoAssignmentResult assignment;
  RhoHungarianState state;
  bool parent_valid = false;
  bool used_parent = false;
  int augmentations = 0;
};

// Solves rows <= columns.  Matrix column order is the canonical order:
// robots are fixed row-by-row, choosing the first column that preserves the
// globally minimum additive objective.  Forbidden edges are >= INF.
RhoAssignmentResult solve_rho_assignment_full(
    const std::vector<std::vector<RhoCost>>& cost,
    RhoAssignmentTiming* timing = nullptr);

// Builds an exact primal-dual state for the minimum additive objective.
// Unlike solve_rho_assignment_full(), this does not canonicalize among equal
// optima.
RhoStateSolveResult solve_rho_assignment_state_full(
    const std::vector<std::vector<RhoCost>>& cost);

// Repairs a value-copy of parent after exactly the listed real rows changed.
// Any undeclared row change, stale/invalid parent, or shape change rejects the
// parent instead of silently reusing it.
RhoStateSolveResult repair_rho_assignment_state_rows(
    const RhoHungarianState& parent,
    const std::vector<std::vector<RhoCost>>& cost,
    const std::vector<int>& changed_rows);

// Exact structural, primal, dual, objective, and source-matrix validation.
bool validate_rho_assignment_state(
    const std::vector<std::vector<RhoCost>>& cost,
    const RhoHungarianState& state);
