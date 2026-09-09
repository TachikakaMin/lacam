/*
 * Carrier BR-LaCAM upper-domain data and exact action oracle.
 *
 * This module contains no search loop.  CarrierBRDomain plugs these
 * semantics into run_lacam_dfs_search() from search_kernel.hpp.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "dd_carrier.hpp"
#include "tapf_planner.hpp"

struct BRLabeledUpperState {
  std::vector<int> target_pos;
  // Indexed by stable anonymous id, not sorted after shelves move.
  std::vector<int> anonymous_pos;

  bool operator==(const BRLabeledUpperState& o) const
  {
    return target_pos == o.target_pos &&
           anonymous_pos == o.anonymous_pos;
  }
  bool operator!=(const BRLabeledUpperState& o) const
  {
    return !(*this == o);
  }

  int position(UpperShelfHandle shelf) const;
  void set_position(UpperShelfHandle shelf, int cell);
};

struct BRLabeledUpperStateHash {
  size_t operator()(const BRLabeledUpperState& state) const;
};

struct BRUpperConstraintEntry {
  enum class Kind : uint8_t { WAIT = 0, TRANSFER = 1 };

  UpperShelfHandle shelf;
  Kind kind = Kind::WAIT;
  StorageTransfer transfer;

  static BRUpperConstraintEntry make_wait(UpperShelfHandle shelf);
  static BRUpperConstraintEntry make_transfer(
      UpperShelfHandle shelf, StorageTransfer transfer);
};

struct AppliedUpperTransfer {
  UpperShelfHandle stable_shelf;
  StorageTransfer transfer;
};

struct BRUpperTransition {
  std::vector<AppliedUpperTransfer> transfers;
};

struct BRUpperSuccessor {
  BRLabeledUpperState state;
  BRUpperTransition transition;
};

struct BRUpperNodeMetadata {
  std::vector<int> age;
  std::vector<int> target_priority;
  std::vector<UpperShelfHandle> variable_order;
};

enum class BRUpperExitReason : uint8_t {
  SOLVED = 0,
  EXHAUSTED = 1,
  TIMEOUT = 2,
  INVALID = 3,
};

struct BRUpperSearchStats {
  long loop_count = 0;
  long explored = 0;
  long duplicate_pushes = 0;
  long exact_oracle_calls = 0;
  long partial_compiler_calls = 0;
  long vacancy_potential_builds = 0;
  long vacancy_potential_unreachable_cells = 0;
  long clearance_first_choice_fallbacks = 0;
  double vacancy_potential_time_ms = 0;
};

struct BRUpperSearchResult {
  BRUpperExitReason exit_reason = BRUpperExitReason::INVALID;
  std::vector<BRLabeledUpperState> states;
  std::vector<BRUpperTransition> transitions;
  BRUpperSearchStats stats;

  bool solved() const
  {
    return exit_reason == BRUpperExitReason::SOLVED;
  }
};

struct CanonicalDispatchMetadata {
  std::vector<RootDemand> roots;
  int priority = 0;
};

struct FrozenShelfTask {
  FrozenTaskId id;
  UpperShelfHandle stable_shelf;
  ShelfTask task;
  size_t wave = 0;
};

struct FrozenTaskWave {
  BRLabeledUpperState expected_before;
  BRLabeledUpperState expected_after;
  std::vector<FrozenShelfTask> tasks;
};

struct FrozenTaskPlan {
  BRLabeledUpperState initial_labeled_state;
  std::vector<FrozenTaskWave> waves;
};

struct Deadline;

BRLabeledUpperState br_labeled_initial_state(const DDInstance& ins);

BRUpperNodeMetadata br_upper_root_metadata(
    const BRLabeledUpperState& state, const std::vector<int>& tau);

BRUpperNodeMetadata br_upper_child_metadata(
    const BRLabeledUpperState& state, const std::vector<int>& tau,
    const BRUpperNodeMetadata& parent);

// WAIT plus every deterministic reachable transfer whose route after the
// source is empty in the current wave-start state.  The exact oracle applies
// the same occupancy rule, so this remains the exhaustive legal low-level
// candidate set rather than the capped Task-BR top window.
std::vector<BRUpperConstraintEntry> br_upper_constraint_candidates(
    const DDInstance& ins, const BRLabeledUpperState& state,
    UpperShelfHandle shelf, const Deadline* deadline = nullptr,
    bool* cutoff = nullptr);

// Deterministic, uncapped legality oracle for a fully constrained upper
// action.  Nullopt means the complete action is illegal.
std::optional<BRUpperSuccessor> validate_complete_upper_action(
    const DDInstance& ins, const BRLabeledUpperState& state,
    const std::vector<BRUpperConstraintEntry>& constraints,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr);

// Task-BR completes an incomplete constraint vector for ordering only.
// The returned action has already passed validate_complete_upper_action().
std::optional<BRUpperSuccessor> complete_partial_upper_action(
    const DDInstance& ins, const BRLabeledUpperState& state,
    const std::vector<int>& tau,
    const BRUpperNodeMetadata& metadata,
    const std::vector<BRUpperConstraintEntry>& forced_constraints,
    BRUpperSearchStats* stats = nullptr,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr);

// First-feasible Carrier BR upper search using the shared LaCAM DFS kernel.
BRUpperSearchResult solve_carrier_br_upper(
    const DDInstance& ins, const BRLabeledUpperState& start,
    const std::vector<int>& tau, const Deadline* deadline);

UpperSignature project_labeled_upper_state(
    const BRLabeledUpperState& state);

CanonicalDispatchMetadata canonical_dispatch_metadata(
    const BRLabeledUpperState& state, const std::vector<int>& tau,
    const AppliedUpperTransfer& transfer);

// Deterministically compile one immutable task wave per accepted upper
// transition.  Every edge payload is checked again by the exact oracle.
std::optional<FrozenTaskPlan> compile_frozen_task_plan(
    const DDInstance& ins, const BRUpperSearchResult& upper,
    const std::vector<int>& tau,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr);

// Pure upper-model replay used both by compilation and the controller's
// shadow ledger checks.
bool replay_frozen_task_plan(
    const DDInstance& ins, const FrozenTaskPlan& plan,
    const std::vector<int>& tau,
    BRLabeledUpperState* final_state = nullptr,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr);
