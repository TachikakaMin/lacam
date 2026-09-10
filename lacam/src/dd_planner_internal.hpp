// Internal helpers shared by the dd_planner_*.cpp translation units
// (split of the original dd_planner.cpp).  NOT part of the public API.
#pragma once

#include "../include/dd_planner.hpp"
#include <chrono>
#include <cstdlib>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <tuple>
#include <unordered_set>
#include "../include/br_lacam_upper.hpp"
#include "../include/tapf_planner.hpp"
#include "../include/search_kernel.hpp"
#include "carrier_guidance.hpp"

namespace dd_detail
{

using Clock = std::chrono::steady_clock;
using carrier_detail::LowerDist;

// Physical-cost weights for reporting (unit unless a numeric objective
// input DD_ALPHA..DD_DELTA is present); must match TAPFPlanner's g.
using SocWeights = SolverWeights;

inline SocWeights soc_weights_from_env()
{
  SocWeights w;
  carrier_detail::load_solver_weights(w);  // same parser as the planner
  return w;
}

inline std::optional<DDPlan> normalize_goal_prefix(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan, const Deadline* deadline = nullptr,
    bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  PhysConfig state = root;
  if (is_dd_goal(ins, state)) {
    if (expired()) return std::nullopt;
    return DDPlan{};
  }
  DDPlan prefix;
  prefix.reserve(plan.size());
  for (const auto& ops : plan) {
    if (expired()) return std::nullopt;
    const auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    prefix.push_back(ops);
    state = *next;
    if (expired()) return std::nullopt;
    if (is_dd_goal(ins, state)) return prefix;
  }
  return std::nullopt;
}

inline std::optional<DDPlan> normalize_goal_prefix(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  return normalize_goal_prefix(
      ins, initial_phys_config(ins), plan, deadline, cutoff);
}

inline void add_scaled_work(int64_t& total, int64_t amount)
{
  if (amount < 0 ||
      total > std::numeric_limits<int64_t>::max() - amount)
    throw std::overflow_error("fixed-point plan work overflow");
  total += amount;
}

inline std::optional<int64_t> joint_ops_work_scaled(
    const SocWeights& weights, const PhysConfig& state,
    const std::vector<Op>& ops, const Deadline* deadline = nullptr,
    bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  if (ops.size() != state.robots.size() ||
      state.kappa.size() != state.robots.size())
    throw std::invalid_argument(
        "joint_ops_work_scaled: state/action arity mismatch");
  int64_t work = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
    if (expired()) return std::nullopt;
    if (ops[i].kind == Op::MOVE) {
      add_scaled_work(
          work, state.kappa[i] == KAPPA_FREE
                    ? weights.beta_scaled
                    : weights.alpha_scaled);
      if (state.kappa[i] == KAPPA_ANON)
        add_scaled_work(work, weights.delta_scaled);
    } else if (ops[i].kind == Op::LIFT ||
               ops[i].kind == Op::DROP) {
      add_scaled_work(work, weights.gamma_scaled);
    }
  }
  if (expired()) return std::nullopt;
  return work;
}

inline std::optional<int64_t> plan_work_scaled(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  const SocWeights w = soc_weights_from_env();
  auto s = root;
  int64_t work = 0;
  for (const auto& ops : plan) {
    if (expired()) return std::nullopt;
    bool step_cutoff = false;
    const auto step_work = joint_ops_work_scaled(
        w, s, ops, deadline, &step_cutoff);
    if (step_cutoff) {
      if (cutoff != nullptr) *cutoff = true;
      return std::nullopt;
    }
    if (!step_work.has_value()) return std::nullopt;
    add_scaled_work(work, *step_work);
    auto nxt = apply_ops(ins, s, ops);
    if (!nxt.has_value()) return std::nullopt;
    s = *nxt;
    if (expired()) return std::nullopt;
  }
  if (expired()) return std::nullopt;
  return work;
}

inline std::optional<int64_t> plan_work_scaled(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  return plan_work_scaled(
      ins, initial_phys_config(ins), plan, deadline, cutoff);
}

inline std::optional<PlanCost> plan_cost_checked(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  bool prefix_cutoff = false;
  const auto prefix = normalize_goal_prefix(
      ins, root, plan, deadline, &prefix_cutoff);
  if (prefix_cutoff) {
    if (cutoff != nullptr) *cutoff = true;
    return std::nullopt;
  }
  if (!prefix.has_value()) return std::nullopt;

  bool work_cutoff = false;
  const auto work = plan_work_scaled(
      ins, root, *prefix, deadline, &work_cutoff);
  if (work_cutoff) {
    if (cutoff != nullptr) *cutoff = true;
    return std::nullopt;
  }
  if (!work.has_value()) return std::nullopt;
  if (is_expired(deadline)) {
    if (cutoff != nullptr) *cutoff = true;
    return std::nullopt;
  }
  return PlanCost::from_scaled(prefix->size(), *work);
}

inline std::optional<PlanCost> plan_cost_checked(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  return plan_cost_checked(
      ins, initial_phys_config(ins), plan, deadline, cutoff);
}

inline PlanCost plan_cost(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  const auto cost = plan_cost_checked(ins, root, plan);
  return cost.has_value() ? *cost : PlanCost::unbounded();
}

inline PlanCost plan_cost(const DDInstance& ins, const DDPlan& plan)
{
  return plan_cost(ins, initial_phys_config(ins), plan);
}

inline std::optional<std::pair<PhysConfig, PlanCost>> replay_raw_prefix(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  const SocWeights weights = soc_weights_from_env();
  PhysConfig state = root;
  PlanCost cost;
  for (const auto& ops : plan) {
    if (expired()) return std::nullopt;
    if (ops.size() != state.robots.size()) return std::nullopt;
    bool step_cutoff = false;
    const auto step_work = joint_ops_work_scaled(
        weights, state, ops, deadline, &step_cutoff);
    if (step_cutoff) {
      if (cutoff != nullptr) *cutoff = true;
      return std::nullopt;
    }
    if (!step_work.has_value()) return std::nullopt;
    const PlanCost step = PlanCost::from_scaled(
        1, *step_work);
    const auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    cost += step;
    state = *next;
    if (expired()) return std::nullopt;
  }
  if (expired()) return std::nullopt;
  return std::make_pair(std::move(state), cost);
}

inline std::optional<std::pair<PhysConfig, PlanCost>> replay_raw_prefix(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  return replay_raw_prefix(
      ins, initial_phys_config(ins), plan, deadline, cutoff);
}

inline const TAPFReferenceCheckpoint* find_reference_checkpoint(
    const TAPFReferencePlan& reference, const PhysConfig& state)
{
  const uint64_t hash = phys_config_hash(state);
  const TAPFReferenceCheckpoint* best = nullptr;
  for (const auto& checkpoint : reference.checkpoints) {
    if (checkpoint.state_hash != hash ||
        !(checkpoint.state == state))
      continue;
    if (best == nullptr ||
        checkpoint.suffix_cost < best->suffix_cost ||
        (checkpoint.suffix_cost == best->suffix_cost &&
         checkpoint.action_index > best->action_index))
      best = &checkpoint;
  }
  return best;
}

inline std::optional<TAPFReferencePlan> build_reference_plan(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan,
    size_t max_checkpoints)
{
  if (max_checkpoints == 0) return std::nullopt;
  const SocWeights weights = soc_weights_from_env();
  std::vector<PhysConfig> states;
  std::vector<PlanCost> step_costs;
  states.reserve(plan.size() + 1);
  step_costs.reserve(plan.size());
  states.push_back(root);
  for (const auto& ops : plan) {
    if (ops.size() != states.back().robots.size())
      return std::nullopt;
    const auto step_work =
        joint_ops_work_scaled(weights, states.back(), ops);
    if (!step_work.has_value()) return std::nullopt;
    step_costs.push_back(PlanCost::from_scaled(
        1, *step_work));
    const auto next = apply_ops(ins, states.back(), ops);
    if (!next.has_value()) return std::nullopt;
    states.push_back(*next);
  }
  if (!is_dd_goal(ins, states.back())) return std::nullopt;

  std::vector<PlanCost> prefix(states.size());
  for (size_t i = 0; i < step_costs.size(); ++i)
    prefix[i + 1] = prefix[i] + step_costs[i];
  std::vector<PlanCost> suffix(states.size());
  for (size_t i = step_costs.size(); i > 0; --i)
    suffix[i - 1] = step_costs[i - 1] + suffix[i];

  std::vector<size_t> selected;
  const size_t count = states.size();
  if (count <= max_checkpoints) {
    selected.resize(count);
    std::iota(selected.begin(), selected.end(), 0);
  } else if (max_checkpoints == 1) {
    selected.push_back(0);
  } else {
    selected.reserve(max_checkpoints);
    for (size_t slot = 0; slot < max_checkpoints; ++slot) {
      const size_t index =
          slot * (count - 1) / (max_checkpoints - 1);
      if (selected.empty() || selected.back() != index)
        selected.push_back(index);
    }
  }

  TAPFReferencePlan reference;
  reference.actions = plan;
  reference.checkpoints.reserve(selected.size());
  for (const size_t index : selected) {
    TAPFReferenceCheckpoint checkpoint;
    checkpoint.state_hash = phys_config_hash(states[index]);
    checkpoint.state = states[index];
    checkpoint.action_index = index;
    checkpoint.prefix_cost = prefix[index];
    checkpoint.suffix_cost = suffix[index];
    if (index < plan.size())
      checkpoint.next_ops = plan[index];
    reference.checkpoints.push_back(std::move(checkpoint));
  }
  return reference;
}

inline std::optional<TAPFReferencePlan> build_reference_plan(
    const DDInstance& ins, const DDPlan& plan,
    size_t max_checkpoints)
{
  return build_reference_plan(
      ins, initial_phys_config(ins), plan, max_checkpoints);
}

// (Config, ShelfState) of an arbitrary physical configuration
inline std::pair<Config, ShelfState> state_of(const TAPFInstance& view,
                                       const PhysConfig& X)
{
  Config C;
  C.reserve(X.robots.size());
  for (const int cell : X.robots) C.push_back(view.G.U[cell]);
  ShelfState S;
  if (!view.shelf_cells.empty()) {  // empty-layer convention (M2)
    S.target_pos = X.target_pos;
    S.anon_occ = X.anon_occ;
    S.kappa = X.kappa;
  }
  return {C, S};
}

inline PhysConfig phys_of(const Config& C, const ShelfState& S)
{
  PhysConfig X;
  X.robots.reserve(C.size());
  for (const auto* v : C) X.robots.push_back(v->index);
  X.target_pos = S.target_pos;
  X.anon_occ = S.anon_occ;
  X.kappa = S.kappa;
  return X;
}

// Production PairCost matching at X on fresh local caches.  B1 freezes
// this root decision before executing its fixed shelf paths.
inline std::vector<int> tau_of(const DDInstance& ins, const PhysConfig& X)
{
  const SocWeights w = soc_weights_from_env();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache uw(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto table = carrier_detail::build_pair_cost_table(
      ins, upper, uw, storage_topology,
      w.alpha, w.gamma, w.delta);
  return carrier_detail::solve_tau_guide(ins, upper, table);
}

inline DDPlan plan_of(const TAPFInstance& view, const Solution& sol,
               const std::vector<ShelfState>& shelves)
{
  return derive_carrier_ops(view, sol, shelves);
}

inline void map_stats(const TAPFStats& t, DDStats* out,
               bool improvement_attempt = false)
{
  if (out == nullptr) return;
  out->hl_nodes += t.hl_nodes_created;
  out->hl_expanded += t.constraints_popped;
  out->pibt_calls += t.pibt_calls;
  out->validator_rejects += t.carrier_validator_rejects;
  out->g1_rejects += t.carrier_g1_rejects;
  out->duplicate_configs += t.hl_duplicate_configs;
  if (improvement_attempt)
    out->improvement_generator_failures +=
        t.constraint_failures;
  else
    out->generator_failures += t.constraint_failures;
  out->macro_successors += t.macro_successors;
  out->macro_steps += t.macro_steps;
  out->macro_after_first += t.macro_after_first;
  out->macro_shelf_motion_successors += t.macro_shelf_motion_successors;
  out->macro_robot_only_successors += t.macro_robot_only_successors;
  out->rollout_calls += t.rollout_calls;
  out->rollout_cycles += t.rollout_cycles;
  out->rollout_shelf_motion_steps += t.rollout_shelf_motion_steps;
  out->robot_only_successors += t.robot_only_successors;
  out->manipulation_successors += t.manipulation_successors;
  out->shelf_motion_successors += t.shelf_motion_successors;
  out->upper_epoch_builds += t.upper_epoch_builds;
  out->pair_cache_hits += t.pair_cache_hits;
  out->pair_cache_misses += t.pair_cache_misses;
  if (!improvement_attempt) {
    out->root_pair_cache_hits +=
        t.root_pair_cache_hits;
    out->root_pair_cache_misses +=
        t.root_pair_cache_misses;
    out->root_pair_edges_evaluated +=
        t.root_pair_edges_evaluated;
    out->root_pair_edges_total +=
        t.root_pair_edges_total;
    out->root_pair_edges_reused +=
        t.root_pair_edges_reused;
  }
  out->pair_edges_evaluated +=
      t.pair_edges_evaluated;
  out->pair_edges_total += t.pair_edges_total;
  out->pair_edges_reused += t.pair_edges_reused;
  out->pair_incremental_reuses +=
      t.pair_incremental_reuses;
  out->pair_hungarian_full_solves +=
      t.pair_hungarian_full_solves;
  out->pair_hungarian_row_repairs +=
      t.pair_hungarian_row_repairs;
  out->pair_hungarian_forced_repairs +=
      t.pair_hungarian_forced_repairs;
  out->pair_rollout_steps += t.pair_rollout_steps;
  out->pair_rollout_truncations += t.pair_rollout_truncations;
  out->pair_rollout_stalls += t.pair_rollout_stalls;
  out->tau_guide_changes_on_upper_move +=
      t.tau_guide_changes_on_upper_move;
  out->joint_task_nodes += t.joint_task_nodes;
  out->joint_task_edges += t.joint_task_edges;
  out->joint_shared_effects += t.joint_shared_effects;
  out->joint_effect_conflicts += t.joint_effect_conflicts;
  out->joint_candidate_backtracks += t.joint_candidate_backtracks;
  out->joint_paused_roots += t.joint_paused_roots;
  out->vacancy_potential_builds +=
      t.vacancy_potential_builds;
  out->vacancy_potential_unreachable_cells +=
      t.vacancy_potential_unreachable_cells;
  out->clearance_first_choice_fallbacks +=
      t.clearance_first_choice_fallbacks;
  out->vacancy_potential_time_ms +=
      t.vacancy_potential_time_ms;
  if (out->guidance_version != t.guidance_version)
    throw std::logic_error(
        "mixed carrier guidance versions across search attempts");
  out->epoch_first_transfer_comparisons +=
      t.epoch_first_transfer_comparisons;
  out->epoch_first_transfer_flips +=
      t.epoch_first_transfer_flips;
  out->epoch_chain_overlap_samples +=
      t.epoch_chain_overlap_samples;
  out->epoch_chain_overlap_intersection +=
      t.epoch_chain_overlap_intersection;
  out->epoch_chain_overlap_union +=
      t.epoch_chain_overlap_union;
  out->upper_epoch_cache_evictions +=
      t.upper_epoch_cache_evictions;
  out->ready_task_count += t.ready_task_count;
  out->rho_repairs += t.rho_repairs;
  out->rho_match_calls_execute += t.rho_match_calls_execute;
  out->rho_match_calls_prepare += t.rho_match_calls_prepare;
  out->rho_candidates_input += t.rho_candidates_input;
  out->rho_candidates_after_claims +=
      t.rho_candidates_after_claims;
  out->rho_candidates_after_key_dedupe +=
      t.rho_candidates_after_key_dedupe;
  out->rho_candidates_after_shelf_preselect +=
      t.rho_candidates_after_shelf_preselect;
  out->rho_candidates_after_priority +=
      t.rho_candidates_after_priority;
  out->rho_invalid_filtered += t.rho_invalid_filtered;
  out->rho_duplicate_key_filtered +=
      t.rho_duplicate_key_filtered;
  out->rho_same_shelf_filtered +=
      t.rho_same_shelf_filtered;
  out->rho_upstream_claim_filtered +=
      t.rho_upstream_claim_filtered;
  out->rho_mode_ineligible_filtered +=
      t.rho_mode_ineligible_filtered;
  out->rho_no_reachable_robot_filtered +=
      t.rho_no_reachable_robot_filtered;
  out->rho_priority_filtered += t.rho_priority_filtered;
  out->rho_matrix_rows_total += t.rho_matrix_rows_total;
  out->rho_matrix_cols_total += t.rho_matrix_cols_total;
  out->rho_matrix_max_rows = std::max(
      out->rho_matrix_max_rows, t.rho_matrix_max_rows);
  out->rho_incremental_full_solves +=
      t.rho_incremental_full_solves;
  out->rho_incremental_repairs +=
      t.rho_incremental_repairs;
  out->rho_incremental_zero_row_reuses +=
      t.rho_incremental_zero_row_reuses;
  out->rho_incremental_changed_rows_total +=
      t.rho_incremental_changed_rows_total;
  if (out->rho_objective_version != t.rho_objective_version)
    throw std::logic_error(
        "mixed rho objective versions across search attempts");
  out->rho_column_identity_same +=
      t.rho_column_identity_same;
  out->rho_column_value_same += t.rho_column_value_same;
  out->rho_mode_or_conflict_same +=
      t.rho_mode_or_conflict_same;
  out->rho_changed_rows_0 += t.rho_changed_rows_0;
  out->rho_changed_rows_1 += t.rho_changed_rows_1;
  out->rho_changed_rows_2 += t.rho_changed_rows_2;
  out->rho_changed_rows_gt2 += t.rho_changed_rows_gt2;
  out->rho_assignment_changes += t.rho_assignment_changes;
  out->rho_candidate_time_ms += t.rho_candidate_time_ms;
  out->rho_matrix_time_ms += t.rho_matrix_time_ms;
  out->rho_bottleneck_time_ms += t.rho_bottleneck_time_ms;
  out->rho_secondary_full_time_ms +=
      t.rho_secondary_full_time_ms;
  out->rho_secondary_repair_time_ms +=
      t.rho_secondary_repair_time_ms;
  out->rho_canonical_time_ms += t.rho_canonical_time_ms;
  out->custody_continuations += t.custody_continuations;
  out->timed_transport_expansions +=
      t.timed_transport_expansions;
  out->timed_transport_frames +=
      t.timed_transport_frames;
  out->owner_handoffs += t.owner_handoffs;
  out->causal_waiting += t.causal_waiting;
  out->traffic_waiting += t.traffic_waiting;
  out->zero_empty_no_ready += t.zero_empty_no_ready;
  out->rewire_guidance_rebuilds += t.rewire_guidance_rebuilds;
  out->g_relaxed += t.g_relaxed;
  out->f_pruned += t.f_pruned;
  out->reference_plans_received +=
      t.reference_plans_received;
  out->reference_plans_validated +=
      t.reference_plans_validated;
  out->reference_checkpoint_hits +=
      t.reference_checkpoint_hits;
  out->reference_action_hints += t.reference_action_hints;
  out->reference_suffix_attempts +=
      t.reference_suffix_attempts;
  out->reference_suffix_accepted +=
      t.reference_suffix_accepted;
  out->incumbent_updates += t.incumbent_updates;
  out->guidance_builds += t.guidance_builds;
  out->tau_time_ms += t.tau_time_ms;
  out->guidance_time_ms += t.guidance_time_ms;
  out->timed_transport_time_ms +=
      t.timed_transport_time_ms;
}

inline DDPlan run_search_attempt(
    const TAPFInstance& view, const DDInstance& ins,
    const PhysConfig& root,
    const Deadline* deadline, int seed, bool macro_enabled,
    TAPFStopPolicy stop_policy, PlanCost incumbent_init,
    const TAPFReferencePlan* reference_plan, DDStats* stats,
    DDPlan* best_effort, bool* solved_out, PlanCost* cost_out,
    double* first_ms, long* first_makespan,
    int64_t* first_work_scaled, double* first_soc,
    long* max_depth, long* targets_done,
    bool* search_cutoff_out,
    std::vector<std::unique_ptr<TAPFPlanner>>* deferred_cleanup,
    std::shared_ptr<TAPFCarrierPersistentState>
        carrier_persistent_state = nullptr,
    const TAPFCarrierRootContinuation*
        carrier_root_continuation = nullptr,
    std::shared_ptr<CarrierGuidance>*
        carrier_root_guidance_output = nullptr)
{
  if (solved_out == nullptr)
    throw std::invalid_argument(
        "run_search_attempt requires solved_out");
  *solved_out = false;
  if (search_cutoff_out != nullptr) *search_cutoff_out = false;
  std::mt19937 mt(seed);
  TAPFStats tstats;
  TAPFSearchConfig cfg;
  cfg.macro_enabled = macro_enabled;
  cfg.stop_policy = stop_policy;
  cfg.defer_cleanup = deferred_cleanup != nullptr;
  cfg.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  cfg.incumbent_init = incumbent_init;
  cfg.reference_plan = reference_plan;
  cfg.initial_physical = root;
  cfg.carrier_persistent_state =
      std::move(carrier_persistent_state);
  cfg.carrier_root_continuation =
      carrier_root_continuation;
  cfg.carrier_root_guidance_output =
      carrier_root_guidance_output;
  const bool continue_after_incumbent =
      stop_policy == TAPFStopPolicy::ANYTIME;
  auto planner = std::make_unique<TAPFPlanner>(
      &view, deadline, &mt, 0, 0, 0.001f,
      continue_after_incumbent,
      &tstats, cfg);
  const auto sol = planner->solve();
  if (search_cutoff_out != nullptr)
    *search_cutoff_out = tstats.timed_out;
  map_stats(
      tstats, stats,
      incumbent_init.is_bounded());
  auto defer_planner_cleanup = [&]() {
    if (deferred_cleanup != nullptr)
      deferred_cleanup->push_back(std::move(planner));
  };
  // Failure-class propagation (review fix 2026-09-01): remember whether
  // this pass actually hit the deadline. The adapter's final timed_out
  // must not conflate exhaustion/generator failure with timeout.
  if (stats != nullptr) stats->timed_out |= tstats.timed_out;
  if (max_depth != nullptr)
    *max_depth = std::max<long>(*max_depth, planner->deepest_depth);
  if (targets_done != nullptr)
    *targets_done =
        std::max<long>(*targets_done, planner->best_targets_done);
  if (first_ms != nullptr && *first_ms < 0 &&
      tstats.first_solution_ticks >= 0) {
    *first_ms = tstats.first_solution_time_ms;
    if (first_makespan != nullptr)
      *first_makespan = tstats.first_solution_ticks;
    if (first_work_scaled != nullptr)
      *first_work_scaled = tstats.first_solution_work_scaled;
    *first_soc = tstats.first_solution_work;
  }
  if (sol.empty()) {
    if (best_effort != nullptr && !planner->best_effort_solution.empty()) {
      *best_effort = plan_of(view, planner->best_effort_solution,
                             planner->best_effort_shelves);
      if (stats != nullptr && !planner->best_effort_shelves.empty()) {
        stats->deepest_config =
            phys_of(planner->best_effort_solution.back(),
                    planner->best_effort_shelves.back());
        stats->deepest_tau = planner->best_effort_tau;
      }
    }
    defer_planner_cleanup();
    return {};
  }
  const auto normalized = normalize_goal_prefix(
      ins, root, plan_of(view, sol, planner->solution_shelves));
  if (!normalized.has_value()) {
    defer_planner_cleanup();
    return {};
  }
  auto plan = *normalized;
  std::vector<PhysConfig> replayed_states;
  if (plan.size() + 1 <= sol.size() &&
      planner->solution_shelves.size() == sol.size()) {
    replayed_states.reserve(plan.size() + 1);
    for (size_t t = 0; t <= plan.size(); ++t)
      replayed_states.push_back(
          phys_of(sol[t], planner->solution_shelves[t]));
  }
  DDPlanRepairStats repair;
  // Repair is transactional.  The normalized raw plan above is already a
  // legal deliverable snapshot; expiry or failed repair returns that raw
  // plan and must not erase a solution that the search already produced.
  if (!plan.empty())
    plan = replayed_states.empty()
               ? repair_carrier_plan(
                     ins, root, plan, &repair, deadline)
               : repair_carrier_plan_from_replay(
                     ins, root, plan, replayed_states, &repair,
                     deadline);
  if (stats != nullptr) {
    stats->exact_loops += repair.exact_loops;
    stats->projected_loops += repair.projected_loops;
    stats->bridge_steps += repair.bridge_steps;
    stats->plan_steps_removed += repair.steps_removed;
  }
  const auto repaired_prefix =
      normalize_goal_prefix(ins, root, plan);
  if (!repaired_prefix.has_value()) {
    defer_planner_cleanup();
    return {};
  }
  plan = *repaired_prefix;
  if (cost_out != nullptr)
    *cost_out = plan_cost(ins, root, plan);
  *solved_out = true;
  defer_planner_cleanup();
  return plan;
}

inline bool has_dynamic_goal_sets(const DDInstance& ins)
{
  for (const auto& goals : ins.target_goal_sets)
    if (goals.size() > 1) return true;
  return false;
}

inline std::optional<DDInstance> fixed_goal_instance_from_plan(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  PhysConfig state = root;
  for (const auto& ops : plan) {
    auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    state = std::move(*next);
  }
  if (!is_dd_goal(ins, state)) return std::nullopt;

  DDInstance fixed = ins;
  for (size_t b = 0; b < fixed.n_targets(); ++b) {
    fixed.target_goals[b] = state.target_pos[b];
    fixed.target_goal_sets[b] = {state.target_pos[b]};
  }
  fixed.finalize();
  return fixed;
}

inline std::optional<DDInstance> fixed_goal_instance_from_plan(
    const DDInstance& ins, const DDPlan& plan)
{
  return fixed_goal_instance_from_plan(
      ins, initial_phys_config(ins), plan);
}

inline uint64_t state_hash(const Config& C, const ShelfState& S)
{
  return phys_config_hash(phys_of(C, S));
}

}  // namespace dd_detail
