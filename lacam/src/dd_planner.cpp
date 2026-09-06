//
// Carrier (DD) entry adapters over the INTEGRATED LaCAM-TAPF planner
// (design.md v3 section 10; debug.md v3 WP5, mappings M11/M15/M16).
//
// There is exactly ONE solve loop in this codebase: TAPFPlanner::solve()
// in tapf_planner.cpp. This file converts DDInstance/PhysConfig to TAPF
// state types, runs the dynamic-to-fixed assignment refinement policy,
// implements B0/B1 on the shared generator/rollout, and re-exports test
// probes over the production guidance machinery (carrier_guidance.hpp).
//
#include "../include/dd_planner.hpp"

#include <chrono>
#include <cstdlib>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <tuple>
#include <unordered_set>

#include "../include/tapf_planner.hpp"
#include "../include/search_kernel.hpp"
#include "carrier_guidance.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using carrier_detail::LowerDist;

// Physical-cost weights for reporting (unit unless a numeric objective
// input DD_ALPHA..DD_DELTA is present); must match TAPFPlanner's g.
using SocWeights = SolverWeights;

SocWeights soc_weights_from_env()
{
  SocWeights w;
  carrier_detail::load_solver_weights(w);  // same parser as the planner
  return w;
}

std::optional<DDPlan> normalize_goal_prefix(const DDInstance& ins,
                                            const DDPlan& plan)
{
  PhysConfig state = initial_phys_config(ins);
  if (is_dd_goal(ins, state)) return DDPlan{};
  DDPlan prefix;
  prefix.reserve(plan.size());
  for (const auto& ops : plan) {
    const auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    prefix.push_back(ops);
    state = *next;
    if (is_dd_goal(ins, state)) return prefix;
  }
  return std::nullopt;
}

void add_scaled_work(int64_t& total, int64_t amount)
{
  if (amount < 0 ||
      total > std::numeric_limits<int64_t>::max() - amount)
    throw std::overflow_error("fixed-point plan work overflow");
  total += amount;
}

int64_t joint_ops_work_scaled(
    const SocWeights& weights, const PhysConfig& state,
    const std::vector<Op>& ops)
{
  if (ops.size() != state.robots.size() ||
      state.kappa.size() != state.robots.size())
    throw std::invalid_argument(
        "joint_ops_work_scaled: state/action arity mismatch");
  int64_t work = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
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
  return work;
}

int64_t plan_work_scaled(const DDInstance& ins, const DDPlan& plan)
{
  const SocWeights w = soc_weights_from_env();
  auto s = initial_phys_config(ins);
  int64_t work = 0;
  for (const auto& ops : plan) {
    add_scaled_work(work, joint_ops_work_scaled(w, s, ops));
    auto nxt = apply_ops(ins, s, ops);
    if (!nxt.has_value()) return work;  // derived plans always replay
    s = *nxt;
  }
  return work;
}

PlanCost plan_cost(const DDInstance& ins, const DDPlan& plan)
{
  const auto prefix = normalize_goal_prefix(ins, plan);
  if (!prefix.has_value()) return PlanCost::unbounded();
  return PlanCost::from_scaled(
      prefix->size(), plan_work_scaled(ins, *prefix));
}

std::optional<std::pair<PhysConfig, PlanCost>> replay_raw_prefix(
    const DDInstance& ins, const DDPlan& plan)
{
  const SocWeights weights = soc_weights_from_env();
  PhysConfig state = initial_phys_config(ins);
  PlanCost cost;
  for (const auto& ops : plan) {
    if (ops.size() != state.robots.size()) return std::nullopt;
    const PlanCost step = PlanCost::from_scaled(
        1, joint_ops_work_scaled(weights, state, ops));
    const auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    cost += step;
    state = *next;
  }
  return std::make_pair(std::move(state), cost);
}

const TAPFReferenceCheckpoint* find_reference_checkpoint(
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

std::optional<TAPFReferencePlan> build_reference_plan(
    const DDInstance& ins, const DDPlan& plan,
    size_t max_checkpoints)
{
  if (max_checkpoints == 0) return std::nullopt;
  const SocWeights weights = soc_weights_from_env();
  std::vector<PhysConfig> states;
  std::vector<PlanCost> step_costs;
  states.reserve(plan.size() + 1);
  step_costs.reserve(plan.size());
  states.push_back(initial_phys_config(ins));
  for (const auto& ops : plan) {
    if (ops.size() != states.back().robots.size())
      return std::nullopt;
    step_costs.push_back(PlanCost::from_scaled(
        1, joint_ops_work_scaled(weights, states.back(), ops)));
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

// (Config, ShelfState) of an arbitrary physical configuration
std::pair<Config, ShelfState> state_of(const TAPFInstance& view,
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

PhysConfig phys_of(const Config& C, const ShelfState& S)
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
std::vector<int> tau_of(const DDInstance& ins, const PhysConfig& X)
{
  const SocWeights w = soc_weights_from_env();
  DDDistCache uw(ins.grid);
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto table = carrier_detail::build_pair_cost_table(
      ins, upper, uw, w.alpha, w.gamma, w.delta);
  return carrier_detail::solve_tau_guide(ins, upper, table);
}

DDPlan plan_of(const TAPFInstance& view, const Solution& sol,
               const std::vector<ShelfState>& shelves)
{
  return derive_carrier_ops(view, sol, shelves);
}

void map_stats(const TAPFStats& t, DDStats* out,
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
  out->ready_task_count += t.ready_task_count;
  out->rho_repairs += t.rho_repairs;
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

DDPlan run_search_attempt(
    const TAPFInstance& view, const DDInstance& ins,
    const Deadline* deadline, int seed, bool macro_enabled,
    TAPFStopPolicy stop_policy, PlanCost incumbent_init,
    const TAPFReferencePlan* reference_plan, DDStats* stats,
    DDPlan* best_effort, bool* solved_out, PlanCost* cost_out,
    double* first_ms, long* first_makespan,
    int64_t* first_work_scaled, double* first_soc,
    long* max_depth, long* targets_done,
    bool* search_cutoff_out,
    std::vector<std::unique_ptr<TAPFPlanner>>* deferred_cleanup)
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
      ins, plan_of(view, sol, planner->solution_shelves));
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
               ? repair_carrier_plan(ins, plan, &repair, deadline)
               : repair_carrier_plan_from_replay(
                     ins, plan, replayed_states, &repair, deadline);
  if (stats != nullptr) {
    stats->exact_loops += repair.exact_loops;
    stats->projected_loops += repair.projected_loops;
    stats->bridge_steps += repair.bridge_steps;
    stats->plan_steps_removed += repair.steps_removed;
  }
  const auto repaired_prefix = normalize_goal_prefix(ins, plan);
  if (!repaired_prefix.has_value()) {
    defer_planner_cleanup();
    return {};
  }
  plan = *repaired_prefix;
  if (cost_out != nullptr) *cost_out = plan_cost(ins, plan);
  *solved_out = true;
  defer_planner_cleanup();
  return plan;
}

bool has_dynamic_goal_sets(const DDInstance& ins)
{
  for (const auto& goals : ins.target_goal_sets)
    if (goals.size() > 1) return true;
  return false;
}

std::optional<DDInstance> fixed_goal_instance_from_plan(
    const DDInstance& ins, const DDPlan& plan)
{
  PhysConfig state = initial_phys_config(ins);
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

uint64_t state_hash(const Config& C, const ShelfState& S)
{
  return phys_config_hash(phys_of(C, S));
}

}  // namespace

const char* dd_improvement_exit_reason_name(
    DDImprovementExitReason reason)
{
  switch (reason) {
    case DDImprovementExitReason::NOT_ATTEMPTED:
      return "NOT_ATTEMPTED";
    case DDImprovementExitReason::NO_REMAINING_BUDGET:
      return "NO_REMAINING_BUDGET";
    case DDImprovementExitReason::STRICT_IMPROVEMENT:
      return "STRICT_IMPROVEMENT";
    case DDImprovementExitReason::SEARCH_CUTOFF:
      return "SEARCH_CUTOFF";
    case DDImprovementExitReason::SEARCH_EXHAUSTED:
      return "SEARCH_EXHAUSTED";
    case DDImprovementExitReason::CANDIDATE_REJECTED:
      return "CANDIDATE_REJECTED";
    case DDImprovementExitReason::FIXED_GOAL_SETUP_FAILED:
      return "FIXED_GOAL_SETUP_FAILED";
    case DDImprovementExitReason::REFERENCE_SUFFIX_ACCEPTED:
      return "REFERENCE_SUFFIX_ACCEPTED";
  }
  return "UNKNOWN";
}

DDFinalizationStatus dd_classify_finalization_probe(
    bool replay_valid, double elapsed_ms, double limit_ms)
{
  if (!replay_valid) return DDFinalizationStatus::INVALID;
  if (elapsed_ms > limit_ms) return DDFinalizationStatus::DEADLINE;
  return DDFinalizationStatus::ACCEPT;
}

DDSolveResult solve_carrier_lacam_result(
    const DDInstance& ins, double time_limit_sec, int seed,
    DDStats* stats, DDPlan* best_effort)
{
  const TAPFInstance view(ins);
  if (stats != nullptr) *stats = DDStats();
  const auto started = Clock::now();

  // One controller owns the verified incumbent and at most two calls to the
  // same TAPFPlanner::solve() implementation.  The first call stops at the
  // first deliverable plan.  The second call is bounded by that incumbent
  // and spends the remaining shared search budget on improvement.  Every
  // second pass receives a fresh singleton-goal view built from the
  // incumbent's actual terminal target positions.
  constexpr size_t MACRO_TARGET_LIMIT = 64;
  const bool use_macro = ins.n_targets() <= MACRO_TARGET_LIMIT;
  // A retained improvement planner may store a pointer to a copied fixed
  // assignment view. Keep that view alive through deferred destruction.
  std::unique_ptr<TAPFInstance> fixed_view_storage;
  std::optional<TAPFReferencePlan> first_reference;
  // Search trees can take seconds to destroy on dense cases.  Their
  // destruction is mandatory solver work and is completed before the
  // strict return timestamp.
  std::vector<std::unique_ptr<TAPFPlanner>> deferred_cleanup;
  deferred_cleanup.reserve(2);
  const double finalization_reserve_sec =
      std::min(1.5, std::max(0.25,
                            std::max(0.0, time_limit_sec) * 0.15));
  const double search_budget_sec =
      std::max(
          0.0,
          std::max(0.0, time_limit_sec) -
              finalization_reserve_sec);
  Deadline search_deadline(search_budget_sec * 1000.0);
  PlanCost cost = PlanCost::unbounded();
  double first_ms = -1, first_soc = -1;
  long first_makespan = -1;
  int64_t first_work_scaled = -1;
  long max_depth = 0, targets_done = 0;
  bool phase1_solved = false;
  DDPlan plan = run_search_attempt(
      view, ins, &search_deadline, seed, use_macro,
      TAPFStopPolicy::FIRST_FEASIBLE, PlanCost::unbounded(),
      nullptr, stats, best_effort, &phase1_solved, &cost, &first_ms,
      &first_makespan, &first_work_scaled, &first_soc,
      &max_depth, &targets_done,
      nullptr,
      &deferred_cleanup);
  // Do not accumulate two large search trees until finalization.  Pass 1's
  // tree no longer owns anything needed by the materialized incumbent.
  deferred_cleanup.clear();

  if (phase1_solved && is_expired(&search_deadline) &&
      stats != nullptr)
    stats->improvement_exit_reason =
        DDImprovementExitReason::NO_REMAINING_BUDGET;

  if (phase1_solved && !is_expired(&search_deadline)) {
    auto fixed = fixed_goal_instance_from_plan(ins, plan);
    if (!fixed.has_value()) {
      if (stats != nullptr)
        stats->improvement_exit_reason =
            DDImprovementExitReason::FIXED_GOAL_SETUP_FAILED;
    } else {
      const bool fixed_restart = has_dynamic_goal_sets(ins);
      fixed_view_storage =
          std::make_unique<TAPFInstance>(*fixed);
      first_reference = build_reference_plan(
          *fixed, plan, 256);
      const TAPFInstance* improvement_view =
          fixed_view_storage.get();
      const DDInstance* improvement_ins = &*fixed;
      if (stats != nullptr) {
        ++stats->improvement_attempts;
        if (fixed_restart) {
          ++stats->assignment_restarts;
          stats->assignment_first_soc = cost.work_value();
          stats->assignment_first_makespan = cost.ticks;
        }
      }
      PlanCost cost2 = PlanCost::unbounded();
      double second_ms = -1, second_soc = -1;
      long second_makespan = -1;
      bool phase2_solved = false;
      bool phase2_cutoff = false;
      DDPlan plan2 = run_search_attempt(
          *improvement_view, *improvement_ins,
          &search_deadline, seed,
          /*macro_enabled=*/false,
          TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT,
          cost,
          first_reference.has_value() ? &*first_reference : nullptr,
          stats, nullptr,
          &phase2_solved, &cost2, &second_ms,
          &second_makespan, nullptr, &second_soc, &max_depth,
          &targets_done, &phase2_cutoff, &deferred_cleanup);
      if (fixed_restart && phase2_solved && stats != nullptr) {
        // This diagnostic now means what its name says: the second search
        // itself produced a new, strictly-better candidate.  The retained
        // first-pass fallback is not counted as a second-pass solve.
        ++stats->assignment_second_solved;
        stats->assignment_second_solution_ms = second_ms;
        stats->assignment_second_soc = cost2.work_value();
        stats->assignment_second_makespan = cost2.ticks;
      }
      if (phase2_solved) {
        if (stats != nullptr) {
          ++stats->improvement_candidates;
          if (fixed_restart) {
            stats->assignment_second_soc = cost2.work_value();
            stats->assignment_second_makespan = cost2.ticks;
          }
        }
        if (cost2 < cost) {
          if (stats != nullptr) {
            ++stats->improvement_improvements;
            if (fixed_restart)
              ++stats->assignment_improvements;
          }
          plan = std::move(plan2);
          cost = cost2;
          if (stats != nullptr) {
            stats->improvement_exit_reason =
                stats->reference_suffix_accepted > 0
                    ? DDImprovementExitReason::
                          REFERENCE_SUFFIX_ACCEPTED
                    : DDImprovementExitReason::
                          STRICT_IMPROVEMENT;
          }
        } else if (stats != nullptr) {
          stats->improvement_exit_reason =
              DDImprovementExitReason::CANDIDATE_REJECTED;
        }
      } else if (stats != nullptr) {
        stats->improvement_exit_reason =
            second_ms >= 0
                ? DDImprovementExitReason::CANDIDATE_REJECTED
                : (phase2_cutoff
                       ? DDImprovementExitReason::SEARCH_CUTOFF
                       : DDImprovementExitReason::SEARCH_EXHAUSTED);
      }
    }
  }

  auto finish = [&](bool solved, DDPlan final_plan, PlanCost final_cost) {
    // A plan is not returned until all deferred solver-owned search state
    // has been destroyed.  fixed_view_storage remains alive across this
    // clear, preserving the pass-2 planner's instance pointer.
    deferred_cleanup.clear();
    DDSolveStatus status = solved ? DDSolveStatus::SOLVED
                                  : (stats != nullptr && stats->timed_out
                                         ? DDSolveStatus::TIMEOUT
                                         : DDSolveStatus::EXHAUSTED);
    if (solved) {
      const auto prefix = normalize_goal_prefix(ins, final_plan);
      const bool valid = prefix.has_value();
      if (valid) {
        final_plan = *prefix;
        final_cost = plan_cost(ins, final_plan);
      }
      const double deliverable_ms =
          std::chrono::duration<double, std::milli>(
              Clock::now() - started)
              .count();
      const auto finalization = dd_classify_finalization_probe(
          valid, deliverable_ms,
          std::max(0.0, time_limit_sec) * 1000.0);
      if (finalization != DDFinalizationStatus::ACCEPT) {
        final_plan.clear();
        final_cost = PlanCost::unbounded();
        status = finalization == DDFinalizationStatus::DEADLINE
                     ? DDSolveStatus::TIMEOUT
                     : DDSolveStatus::INVALID;
      } else if (stats != nullptr) {
        stats->deliverable_ms = deliverable_ms;
      }
    }
    if (stats != nullptr) {
      stats->first_solution_ms = first_ms;
      stats->first_solution_makespan = first_makespan;
      stats->first_solution_work_scaled = first_work_scaled;
      stats->first_solution_soc = first_soc;
      stats->best_makespan =
          status == DDSolveStatus::SOLVED ? final_cost.ticks : -1;
      stats->best_work_scaled =
          status == DDSolveStatus::SOLVED ? final_cost.work : -1;
      stats->best_soc =
          status == DDSolveStatus::SOLVED ? final_cost.work_value() : -1;
      stats->max_depth = max_depth;
      stats->best_targets_done = targets_done;
      // timed_out accumulated per pass above: an empty plan is a timeout
      // only if some pass actually expired; OPEN exhaustion and generator
      // failure report as a plain (non-timeout) failure.
      stats->timed_out = status == DDSolveStatus::TIMEOUT;
    }
    return DDSolveResult{status, std::move(final_plan)};
  };
  return finish(phase1_solved, std::move(plan), cost);
}

DDPlan solve_carrier_lacam(const DDInstance& ins, double time_limit_sec,
                           int seed, DDStats* stats, DDPlan* best_effort)
{
  return solve_carrier_lacam_result(
             ins, time_limit_sec, seed, stats, best_effort)
      .plan;
}

DDPlan solve_carrier_rollout(const DDInstance& ins, double time_limit_sec,
                             int seed, DDStats* stats)
{
  // B0 (design 8.1): the SHARED rollout core with no high-level search
  const TAPFInstance view(ins);
  if (stats != nullptr) *stats = DDStats();
  const auto t_start = Clock::now();
  std::mt19937 mt(seed);
  Deadline deadline(time_limit_sec * 1000);
  TAPFStats tstats;
  TAPFPlanner planner(&view, &deadline, &mt, 0, 0, 0.001f, true, &tstats);

  DDPlan plan;
  auto C = view.starts;
  auto S = initial_shelf_state(view);
  if (planner.is_goal_config(C, S)) {
    plan.push_back(std::vector<Op>(view.N, Op::make_wait()));
    return plan;
  }
  std::unordered_set<uint64_t> seen;
  seen.insert(state_hash(C, S));
  while (std::chrono::duration<double>(Clock::now() - t_start).count() <
         time_limit_sec) {
    auto r =
        planner.carrier_rollout(C, S, 512, 0, /*stop_on_event=*/false);
    for (auto& ops : r.ops) plan.push_back(std::move(ops));
    C = r.configs.back();
    S = r.shelves.back();
    if (r.reached_goal) {
      map_stats(tstats, stats);
      return plan;
    }
    if (r.ops.empty()) {
      map_stats(tstats, stats);
      if (stats != nullptr) ++stats->generator_failures;
      return {};  // stuck: honest failure (no search to recover)
    }
    if (!seen.insert(state_hash(C, S)).second) {
      map_stats(tstats, stats);
      return {};  // global cycle
    }
  }
  map_stats(tstats, stats);
  if (stats != nullptr) stats->timed_out = true;
  return {};
}

DDPlan solve_carrier_2stage(
    const DDInstance& ins, double time_limit_sec, int seed, DDStats* stats,
    std::vector<std::vector<int>>* fixed_paths_out)
{
  // B1 freezes both the root PairCost matching and one deterministic
  // shortest shelf path per target.  Its per-step execution still uses
  // the same Task-BR ready/rho/custody layer and Carrier-PIBT generator as
  // production; leaving a frozen path is an honest baseline failure.
  const TAPFInstance view(ins);
  if (stats != nullptr) *stats = DDStats();
  const auto started = Clock::now();
  std::mt19937 mt(seed);
  Deadline deadline(std::max(0.0, time_limit_sec) * 1000);
  TAPFStats tapf_stats;
  TAPFPlanner planner(
      &view, &deadline, &mt, 0, 0, 0.001f, true, &tapf_stats);
  DDDistCache upper_wall(ins.grid);
  LowerDist lower_distance(ins.grid);
  const auto weights = soc_weights_from_env();

  PhysConfig physical = initial_phys_config(ins);
  const std::vector<int> fixed_tau = tau_of(ins, physical);
  const auto initial_upper =
      carrier_detail::make_upper_signature(physical);
  std::vector<uint8_t> occupied(ins.grid.size(), 0);
  for (const int cell : initial_upper.target_pos) occupied[cell] = 1;
  for (const int cell : initial_upper.anon_pos) occupied[cell] = 1;

  auto shortest_least_blocking_path =
      [&](int src, int dst) -> std::vector<int> {
    using QueueItem = std::tuple<int, int, int>;
    const int inf = std::numeric_limits<int>::max() / 4;
    std::vector<std::pair<int, int>> distance(
        ins.grid.size(), {inf, inf});
    std::vector<int> previous(ins.grid.size(), -1);
    std::priority_queue<
        QueueItem, std::vector<QueueItem>, std::greater<QueueItem>>
        open;
    distance[src] = {0, 0};
    open.emplace(0, 0, src);
    while (!open.empty()) {
      const auto [steps, blockers, cell] = open.top();
      open.pop();
      if (distance[cell] != std::make_pair(steps, blockers)) continue;
      if (cell == dst) break;
      int raw_neighbors[4];
      const int count =
          ins.grid.neighbors(cell, raw_neighbors);
      std::vector<int> neighbors(
          raw_neighbors, raw_neighbors + count);
      std::sort(neighbors.begin(), neighbors.end());
      for (const int next : neighbors) {
        const std::pair<int, int> candidate{
            steps + 1,
            blockers +
                (occupied[next] && next != src ? 1 : 0)};
        if (candidate < distance[next] ||
            (candidate == distance[next] &&
             (previous[next] < 0 || cell < previous[next]))) {
          distance[next] = candidate;
          previous[next] = cell;
          open.emplace(candidate.first, candidate.second, next);
        }
      }
    }
    if (distance[dst].first >= inf) return {};
    std::vector<int> path;
    for (int cell = dst; cell >= 0; cell = previous[cell]) {
      path.push_back(cell);
      if (cell == src) break;
    }
    if (path.empty() || path.back() != src) return {};
    std::reverse(path.begin(), path.end());
    return path;
  };

  std::vector<std::vector<int>> fixed_paths(ins.n_targets());
  for (size_t target = 0; target < ins.n_targets(); ++target) {
    fixed_paths[target] = shortest_least_blocking_path(
        ins.target_starts[target], fixed_tau[target]);
    if (fixed_paths[target].empty()) return {};
  }
  if (fixed_paths_out != nullptr) *fixed_paths_out = fixed_paths;
  std::vector<size_t> fixed_index(ins.n_targets(), 0);

  auto at_fixed_goal = [&](const PhysConfig& state) {
    for (size_t target = 0; target < ins.n_targets(); ++target) {
      if (state.target_pos[target] != fixed_tau[target]) return false;
      if (std::find(state.kappa.begin(), state.kappa.end(),
                    (int)target) != state.kappa.end())
        return false;
    }
    return true;
  };
  if (at_fixed_goal(physical))
    return DDPlan{
        std::vector<Op>(ins.n_robots(), Op::make_wait())};

  carrier_detail::UpperEpochCache fixed_epoch_cache;
  std::optional<PhysConfig> previous_physical;
  std::optional<CarrierGuidance> previous_guidance;
  std::vector<Op> previous_ops;
  std::unordered_set<uint64_t> seen{
      phys_config_hash(physical)};
  DDPlan plan;

  while (std::chrono::duration<double>(
             Clock::now() - started)
             .count() < time_limit_sec) {
    const UpperSignature upper =
        carrier_detail::make_upper_signature(physical);
    auto epoch = fixed_epoch_cache.lookup(upper);
    if (epoch == nullptr) {
      epoch =
          carrier_detail::build_task_br_upper_epoch_for_tau(
              ins, upper, fixed_tau, upper_wall, weights.alpha,
              weights.gamma, weights.delta);
      fixed_epoch_cache.insert(upper, epoch);
    }
    CarrierGuidance guidance =
        carrier_detail::build_task_br_guidance_from_upper_epoch(
            ins, physical, std::move(epoch),
            previous_physical.has_value()
                ? &*previous_physical
                : nullptr,
            previous_guidance.has_value()
                ? &*previous_guidance
                : nullptr,
            previous_ops.empty() ? nullptr : &previous_ops);

    auto [config, shelf] = state_of(view, physical);
    auto node = std::make_unique<TAPFNode>(
        config, shelf, planner.D, &view,
        std::vector<int>((int)view.N, -1),
        TAPFAssignmentState(), nullptr);
    node->guide =
        std::make_unique<CarrierGuidance>(guidance);
    node->order = carrier_detail::task_br_robot_order(
        physical, guidance, lower_distance);
    node->constraint_order = node->order;
    planner.invalidate_carrier_scratch();
    TAPFConstraint root;
    if (!planner.get_new_config(node.get(), &root) ||
        !planner.apply_carrier_effects(node.get())) {
      map_stats(tapf_stats, stats);
      if (stats != nullptr) ++stats->generator_failures;
      return {};
    }

    Config next_config(view.N, nullptr);
    for (auto* agent : planner.A)
      next_config[agent->id] = agent->v_next;
    const PhysConfig next =
        phys_of(next_config, planner.shelf_next_scratch);
    std::vector<size_t> next_index = fixed_index;
    bool respects_fixed_paths = true;
    for (size_t target = 0; target < ins.n_targets(); ++target) {
      const auto& path = fixed_paths[target];
      const size_t index = fixed_index[target];
      if (next.target_pos[target] == path[index]) continue;
      if (index + 1 < path.size() &&
          next.target_pos[target] == path[index + 1]) {
        next_index[target] = index + 1;
        continue;
      }
      respects_fixed_paths = false;
      break;
    }
    if (!respects_fixed_paths) {
      map_stats(tapf_stats, stats);
      return {};
    }

    const auto ops = planner.ops_scratch;
    plan.push_back(ops);
    previous_physical = physical;
    previous_guidance = std::move(guidance);
    previous_ops = ops;
    physical = next;
    fixed_index = std::move(next_index);
    if (at_fixed_goal(physical)) {
      map_stats(tapf_stats, stats);
      return plan;
    }
    if (!seen.insert(phys_config_hash(physical)).second) {
      map_stats(tapf_stats, stats);
      return {};
    }
  }

  map_stats(tapf_stats, stats);
  if (stats != nullptr) stats->timed_out = true;
  return {};
}

DDSocWeights dd_load_soc_weights()
{
  // R4: the ONE parser (carrier_detail::load_solver_weights) behind a
  // public face for tools; planner and reporting weights always agree.
  DDSocWeights w;
  carrier_detail::load_solver_weights(w);
  return w;
}

PlanCost dd_plan_cost_probe(const DDInstance& ins, const DDPlan& plan)
{
  return plan_cost(ins, plan);
}

bool dd_plan_cost_better_probe(const PlanCost& candidate,
                               const PlanCost& incumbent)
{
  return candidate < incumbent;
}

std::optional<DDPlan> dd_normalize_goal_prefix_probe(
    const DDInstance& ins, const DDPlan& plan)
{
  return normalize_goal_prefix(ins, plan);
}

std::optional<TAPFReferencePlan> dd_build_reference_plan_probe(
    const DDInstance& ins, const DDPlan& plan,
    size_t max_checkpoints)
{
  return build_reference_plan(ins, plan, max_checkpoints);
}

std::optional<TAPFReferenceCheckpoint> dd_reference_checkpoint_probe(
    const TAPFReferencePlan& reference, const PhysConfig& state)
{
  const auto* checkpoint =
      find_reference_checkpoint(reference, state);
  if (checkpoint == nullptr) return std::nullopt;
  return *checkpoint;
}

std::optional<DDPlan> dd_reference_splice_probe(
    const DDInstance& ins, const TAPFReferencePlan& reference,
    const DDPlan& prefix, const PlanCost& incumbent)
{
  const auto replayed = replay_raw_prefix(ins, prefix);
  if (!replayed.has_value()) return std::nullopt;
  const auto* checkpoint =
      find_reference_checkpoint(reference, replayed->first);
  if (checkpoint == nullptr ||
      checkpoint->action_index >= reference.actions.size())
    return std::nullopt;
  if (!(replayed->second + checkpoint->suffix_cost < incumbent))
    return std::nullopt;

  DDPlan stitched = prefix;
  stitched.insert(
      stitched.end(),
      reference.actions.begin() + checkpoint->action_index,
      reference.actions.end());
  const auto normalized = normalize_goal_prefix(ins, stitched);
  if (!normalized.has_value() ||
      !(plan_cost(ins, *normalized) < incumbent))
    return std::nullopt;
  return normalized;
}

UpperSignature dd_upper_signature_probe(const PhysConfig& X)
{
  return carrier_detail::make_upper_signature(X);
}

PairPlan dd_pair_cost_probe(const DDInstance& ins, const PhysConfig& X,
                            int target, int goal)
{
  const auto w = dd_load_soc_weights();
  DDDistCache upper_wall(ins.grid);
  return carrier_detail::pair_cost(
      ins, carrier_detail::make_upper_signature(X), target, goal, upper_wall,
      w.alpha, w.gamma, w.delta);
}

DDLazyTauProbe dd_lazy_tau_guide_probe(const DDInstance& ins,
                                       const PhysConfig& X)
{
  const auto w = dd_load_soc_weights();
  DDDistCache upper_wall(ins.grid);
  const auto result = carrier_detail::build_lazy_pair_cost_assignment(
      ins, carrier_detail::make_upper_signature(X), upper_wall,
      w.alpha, w.gamma, w.delta);
  DDLazyTauProbe out;
  out.tau = result.tau;
  out.table = result.table;
  out.evaluated_edges = result.evaluated_edges;
  out.total_edges = result.total_edges;
  return out;
}

std::optional<TaskId> dd_pair_next_ready_effect_probe(
    const DDInstance& ins, const PhysConfig& X, int target, int goal,
    int recursion_cap)
{
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto abstract =
      carrier_detail::make_abstract_upper_state(ins, upper);
  DDDistCache upper_wall(ins.grid);
  return carrier_detail::compile_single_root_next_ready_effect(
             ins, abstract, RootDemand{target, goal}, upper_wall,
             carrier_detail::TaskBRCompilerLimits{
                 recursion_cap, 512})
      .ready_effect;
}

double dd_pair_episode_cost_probe(
    const std::vector<ShelfSelector>& shifted_shelves, double alpha,
    double gamma, double delta)
{
  return carrier_detail::pair_episode_cost(
      shifted_shelves, alpha, gamma, delta);
}

std::vector<int> dd_tau_guide_probe(const DDInstance& ins,
                                    const PhysConfig& X)
{
  const auto w = dd_load_soc_weights();
  DDDistCache upper_wall(ins.grid);
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto table = carrier_detail::build_pair_cost_table(
      ins, upper, upper_wall, w.alpha, w.gamma, w.delta);
  return carrier_detail::solve_tau_guide(ins, upper, table);
}

double dd_tau_lb_probe(const DDInstance& ins, const PhysConfig& X)
{
  const auto w = dd_load_soc_weights();
  DDDistCache upper_wall(ins.grid);
  return carrier_detail::solve_tau_lb(
      ins, X, upper_wall, w.alpha, w.gamma);
}

int64_t dd_makespan_lb_probe(const DDInstance& ins, const PhysConfig& X)
{
  DDDistCache wall_distance(ins.grid);
  return carrier_detail::solve_tau_time_lb(
      ins, X, wall_distance);
}

ShelfTaskGraph dd_compile_single_root_graph_probe(
    const DDInstance& ins, const PhysConfig& X, int target, int goal,
    int recursion_cap, int backtrack_cap)
{
  const auto upper = carrier_detail::make_upper_signature(X);
  auto abstract =
      carrier_detail::make_abstract_upper_state(ins, upper);
  DDDistCache upper_wall(ins.grid);
  std::vector<int> tau(ins.n_targets(), -1);
  std::vector<int> priority(ins.n_targets(), 0);
  if (target >= 0 && target < (int)ins.n_targets()) {
    tau[target] = goal;
    priority[target] = 1;
  }
  return carrier_detail::compile_task_br_pibt(
      ins, abstract, {RootDemand{target, goal}}, tau, priority, upper_wall,
      carrier_detail::TaskBRCompilerLimits{recursion_cap, backtrack_cap},
      true);
}

ShelfTaskGraph dd_compile_joint_graph_probe(
    const DDInstance& ins, const PhysConfig& X,
    const std::vector<int>* tau_override,
    const std::vector<int>* priority_override,
    int recursion_cap, int backtrack_cap)
{
  const auto w = dd_load_soc_weights();
  DDDistCache upper_wall(ins.grid);
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto table = carrier_detail::build_pair_cost_table(
      ins, upper, upper_wall, w.alpha, w.gamma, w.delta);
  const auto tau = tau_override != nullptr
                       ? *tau_override
                       : carrier_detail::solve_tau_guide(ins, upper, table);
  const auto priority =
      priority_override != nullptr
          ? *priority_override
          : carrier_detail::target_priorities_from_pair_cost(
                table, tau);
  std::vector<RootDemand> roots;
  for (size_t b = 0; b < ins.n_targets(); ++b)
    if (upper.target_pos[b] != tau[b])
      roots.push_back(RootDemand{(int)b, tau[b]});
  auto abstract =
      carrier_detail::make_abstract_upper_state(ins, upper);
  return carrier_detail::compile_task_br_pibt(
      ins, abstract, roots, tau, priority, upper_wall,
      carrier_detail::TaskBRCompilerLimits{recursion_cap, backtrack_cap},
      false);
}

std::vector<int> dd_ready_tasks_probe(const DDInstance& ins,
                                      const PhysConfig& X,
                                      const ShelfTaskGraph& graph)
{
  return carrier_detail::ready_tasks(ins, X, graph);
}

ShelfTaskGraph dd_propagate_root_demands_probe(
    ShelfTaskGraph graph, const std::vector<int>& target_priority)
{
  carrier_detail::propagate_root_demands(graph, target_priority);
  return graph;
}

bool dd_task_effects_conflict_probe(const TaskId& a, const TaskId& b)
{
  return carrier_detail::task_effects_conflict(a, b);
}

CarrierGuidance dd_task_br_guidance_probe(
    const DDInstance& ins, const PhysConfig& X,
    const PhysConfig* previous_X,
    const CarrierGuidance* previous_guidance,
    const std::vector<Op>* executed_ops)
{
  const auto weights = dd_load_soc_weights();
  DDDistCache upper_wall(ins.grid);
  return carrier_detail::build_task_br_guidance(
      ins, X, upper_wall, weights.alpha, weights.gamma, weights.delta,
      previous_X, previous_guidance, executed_ops);
}

CarrierGuidance dd_task_br_cached_guidance_probe(
    const DDInstance& ins, const PhysConfig& X,
    const std::vector<PhysConfig>& warmups, long* cache_hits)
{
  const auto weights = dd_load_soc_weights();
  DDDistCache upper_wall(ins.grid);
  carrier_detail::UpperEpochCache cache;
  for (const auto& warmup : warmups)
    (void)carrier_detail::build_task_br_guidance(
        ins, warmup, upper_wall, weights.alpha, weights.gamma,
        weights.delta, nullptr, nullptr, nullptr, &cache);
  auto result = carrier_detail::build_task_br_guidance(
      ins, X, upper_wall, weights.alpha, weights.gamma, weights.delta,
      nullptr, nullptr, nullptr, &cache);
  if (cache_hits != nullptr) *cache_hits = cache.hits;
  return result;
}

DDReadyMatchProbe dd_match_ready_tasks_probe(
    const DDInstance& ins, const PhysConfig& X,
    const ShelfTaskGraph& graph, const std::vector<int>& ready_tasks,
    const std::vector<std::optional<TaskId>>* previous_rho_task_id)
{
  return carrier_detail::match_ready_tasks(
      ins, X, graph, ready_tasks, previous_rho_task_id);
}

double dd_root_admissible_h(const DDInstance& ins)
{
  // Keep the admissible lower bound independent from tau_guide.
  const SocWeights w = soc_weights_from_env();
  DDDistCache uw(ins.grid);
  const auto X = initial_phys_config(ins);
  return carrier_detail::solve_tau_lb(
      ins, X, uw, w.alpha, w.gamma);
}

// ===================================================================
// TEST SUPPORT (protected suites): probes of the PRODUCTION machinery
// ===================================================================

namespace {

// drain one node's operator-constraint tree through the production
// expansion + generation (G1 conformance, debug.md P0-1/P0-2/P0-5 lineage)
std::vector<PhysConfig> drain_node(const TAPFInstance& view,
                                   const PhysConfig& X, int seed)
{
  std::mt19937 mt(seed);
  TAPFStats tstats;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &tstats);
  auto [C, S] = state_of(view, X);
  auto node = std::make_unique<TAPFNode>(C, S, planner.D, &view,
                                         std::vector<int>((int)view.N, -1),
                                         TAPFAssignmentState(), nullptr);
  planner.invalidate_carrier_scratch();
  planner.attach_carrier_guidance(node.get());

  std::vector<PhysConfig> out;
  std::unordered_set<uint64_t> seen;
  const int R = (int)view.N;
  std::vector<OpCand> cand;
  std::vector<TAPFConstraint*> popped;  // delete after the drain
  while (!node->search_tree.empty()) {
    auto M = node->search_tree.front();
    node->search_tree.pop();
    popped.push_back(M);
    if (M->depth < R) {
      const int i = node->constraint_order[M->depth];
      planner.build_op_candidates(node.get(), i, cand);
      lacam_expand_constraint_vec<TAPFConstraint>(M, i, cand,
                                                  node->search_tree);
    }
    if (!planner.get_new_config(node.get(), M)) continue;
    if (!planner.apply_carrier_effects(node.get())) continue;
    Config C_new(view.N, nullptr);
    for (auto a : planner.A) C_new[a->id] = a->v_next;
    auto nxt = phys_of(C_new, planner.shelf_next_scratch);
    if (seen.insert(phys_config_hash(nxt)).second) out.push_back(nxt);
  }
  for (auto* M : popped) delete M;
  return out;
}

}  // namespace

std::vector<PhysConfig> dd_enumerate_node_successors(const DDInstance& ins,
                                                     const PhysConfig& X,
                                                     int seed)
{
  const TAPFInstance view(ins);
  return drain_node(view, X, seed);
}

std::vector<Op> dd_root_joint_ops(const DDInstance& ins, const PhysConfig& X,
                                  int seed)
{
  const TAPFInstance view(ins);
  std::mt19937 mt(seed);
  TAPFStats tstats;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &tstats);
  auto [C, S] = state_of(view, X);
  auto node = std::make_unique<TAPFNode>(C, S, planner.D, &view,
                                         std::vector<int>((int)view.N, -1),
                                         TAPFAssignmentState(), nullptr);
  planner.attach_carrier_guidance(node.get());
  TAPFConstraint root;
  if (!planner.get_new_config(node.get(), &root)) return {};
  if (!planner.apply_carrier_effects(node.get())) return {};
  return planner.ops_scratch;
}
