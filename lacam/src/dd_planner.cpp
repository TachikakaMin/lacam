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
struct SocWeights {
  double alpha = 1, beta = 1, gamma = 1, delta = 1;
};

SocWeights soc_weights_from_env()
{
  SocWeights w;
  carrier_detail::load_solver_weights(w);  // same parser as the planner
  return w;
}

double plan_soc(const DDInstance& ins, const DDPlan& plan)
{
  const SocWeights w = soc_weights_from_env();
  auto s = initial_phys_config(ins);
  double c = 0;
  for (const auto& ops : plan) {
    for (size_t i = 0; i < ops.size(); ++i) {
      if (ops[i].kind == Op::MOVE) {
        c += s.kappa[i] == KAPPA_FREE ? w.beta : w.alpha;
        if (s.kappa[i] == KAPPA_ANON) c += w.delta;
      } else if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) {
        c += w.gamma;
      }
    }
    auto nxt = apply_ops(ins, s, ops);
    if (!nxt.has_value()) return c;  // derived plans always replay
    s = *nxt;
  }
  return c;
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
  auto plan = derive_carrier_ops(view, sol, shelves);
  if (plan.empty() && !sol.empty())  // trivially solved: single all-wait
    plan.push_back(std::vector<Op>(view.N, Op::make_wait()));
  return plan;
}

void map_stats(const TAPFStats& t, DDStats* out)
{
  if (out == nullptr) return;
  out->hl_nodes += t.hl_nodes_created;
  out->hl_expanded += t.constraints_popped;
  out->pibt_calls += t.pibt_calls;
  out->validator_rejects += t.carrier_validator_rejects;
  out->g1_rejects += t.carrier_g1_rejects;
  out->duplicate_configs += t.hl_duplicate_configs;
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
  out->zero_empty_no_ready += t.zero_empty_no_ready;
  out->rewire_guidance_rebuilds += t.rewire_guidance_rebuilds;
  out->g_relaxed += t.g_relaxed;
  out->f_pruned += t.f_pruned;
  out->incumbent_updates += t.incumbent_updates;
  out->guidance_builds += t.guidance_builds;
  out->tau_time_ms += t.tau_time_ms;
  out->guidance_time_ms += t.guidance_time_ms;
}

DDPlan run_first_incumbent_search(
    const TAPFInstance& view, const DDInstance& ins, double limit_sec,
    int seed, bool macro_enabled, DDStats* stats, DDPlan* best_effort,
    double* soc_out, double* first_ms, double* first_soc, long* max_depth,
    long* targets_done,
    std::vector<std::unique_ptr<TAPFPlanner>>* deferred_cleanup)
{
  std::mt19937 mt(seed);
  Deadline deadline(std::max(0.0, limit_sec) * 1000);
  TAPFStats tstats;
  TAPFSearchConfig cfg;
  cfg.macro_enabled = macro_enabled;
  cfg.stop_at_first = true;
  cfg.defer_cleanup = deferred_cleanup != nullptr;
  auto planner = std::make_unique<TAPFPlanner>(
      &view, &deadline, &mt, 0, 0, 0.001f, false, &tstats, cfg);
  const auto sol = planner->solve();
  map_stats(tstats, stats);
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
  if (first_ms != nullptr && *first_ms < 0 && tstats.first_solution_g >= 0) {
    *first_ms = tstats.first_solution_time_ms;
    *first_soc = tstats.first_solution_g;
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
  auto plan = plan_of(view, sol, planner->solution_shelves);
  std::vector<PhysConfig> replayed_states;
  if (sol.size() == plan.size() + 1 &&
      planner->solution_shelves.size() == sol.size()) {
    replayed_states.reserve(sol.size());
    for (size_t t = 0; t < sol.size(); ++t)
      replayed_states.push_back(
          phys_of(sol[t], planner->solution_shelves[t]));
  }
  DDPlanRepairStats repair;
  // R1 (debug.md §10): the repair consumes the SAME pass deadline; on
  // expiry it returns the raw plan.  A pass that cannot finish
  // search + mandatory repair inside its budget has NOT produced its
  // deliverable: the un-repaired raw incumbent (100k+ steps on the
  // borderline case) cannot be printed/validated inside the protocol
  // window either, so the pass reports an honest timeout instead.
  plan = replayed_states.empty()
             ? repair_carrier_plan(ins, plan, &repair, &deadline)
             : repair_carrier_plan_from_replay(
                   ins, plan, replayed_states, &repair, &deadline);
  if (is_expired(&deadline)) {
    if (stats != nullptr) stats->timed_out = true;
    defer_planner_cleanup();
    return {};
  }
  if (stats != nullptr) {
    stats->exact_loops += repair.exact_loops;
    stats->projected_loops += repair.projected_loops;
    stats->bridge_steps += repair.bridge_steps;
    stats->plan_steps_removed += repair.steps_removed;
  }
  if (soc_out != nullptr) *soc_out = plan_soc(ins, plan);
  defer_planner_cleanup();
  return plan;
}

std::optional<DDInstance> fixed_assignment_from_plan(
    const DDInstance& ins, const DDPlan& plan)
{
  bool has_dynamic_assignment = false;
  for (const auto& goals : ins.target_goal_sets)
    has_dynamic_assignment |= goals.size() > 1;
  if (!has_dynamic_assignment) return std::nullopt;

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

DDFinalizationStatus dd_classify_finalization_probe(
    bool replay_valid, double elapsed_ms, double limit_ms)
{
  if (!replay_valid) return DDFinalizationStatus::INVALID;
  if (elapsed_ms > limit_ms) return DDFinalizationStatus::DEADLINE;
  return DDFinalizationStatus::ACCEPT;
}

DDPlan solve_carrier_lacam(const DDInstance& ins, double time_limit_sec,
                           int seed, DDStats* stats, DDPlan* best_effort)
{
  const TAPFInstance view(ins);
  if (stats != nullptr) *stats = DDStats();
  const auto started = Clock::now();
  const auto finish_at =
      started + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(
                        std::max(0.0, time_limit_sec)));
  auto remaining = [&]() {
    return std::max(
        0.0,
        std::chrono::duration<double>(finish_at - Clock::now()).count());
  };

  // Phase 1 finds one executable incumbent under dynamic shelf-goal
  // matching. If it leaves time, phase 2 fixes the terminal assignment and
  // reruns the same search from the root. This is automatic for multi-goal
  // inputs and structurally absent for singleton-goal instances.
  constexpr size_t MACRO_TARGET_LIMIT = 64;
  const bool use_macro = ins.n_targets() <= MACRO_TARGET_LIMIT;
  // A retained pass-2 planner stores a pointer to its TAPFInstance.  Keep
  // that copied view alive until after deferred planner destruction.
  std::unique_ptr<TAPFInstance> fixed_view_storage;
  // Search trees can take seconds to destroy on dense cases.  Their
  // destruction is mandatory solver work and is completed before the
  // strict return timestamp.
  std::vector<std::unique_ptr<TAPFPlanner>> deferred_cleanup;
  deferred_cleanup.reserve(2);
  const double finalization_reserve_sec =
      std::min(1.5, std::max(0.25,
                            std::max(0.0, time_limit_sec) * 0.15));
  double soc = -1, first_ms = -1, first_soc = -1;
  long max_depth = 0, targets_done = 0;
  const double phase1_limit =
      std::max(0.0, remaining() - finalization_reserve_sec);
  DDPlan plan = run_first_incumbent_search(
      view, ins, phase1_limit, seed, use_macro, stats, best_effort, &soc,
      &first_ms, &first_soc, &max_depth, &targets_done, &deferred_cleanup);
  // Do not accumulate two large search trees until finalization.  Pass 1's
  // tree no longer owns anything needed by the materialized incumbent.
  deferred_cleanup.clear();

  if (!plan.empty()) {
    auto fixed = fixed_assignment_from_plan(ins, plan);
    // Pass 2 is optional improvement work.  It must not consume the
    // incumbent's final selection/replay window: an unsuccessful restart
    // returns pass 1, and that pass-1 plan still has to become a machine-
    // checked deliverable before the shared deadline.
    const double phase2_limit =
        std::max(0.0, remaining() - finalization_reserve_sec);
    if (fixed.has_value() && phase2_limit > 0) {
      if (stats != nullptr) {
        ++stats->assignment_restarts;
        stats->assignment_first_soc = soc;
        stats->assignment_first_makespan = static_cast<long>(plan.size());
      }
      const double phase2_started_ms =
          std::chrono::duration<double, std::milli>(
              Clock::now() - started)
              .count();
      fixed_view_storage = std::make_unique<TAPFInstance>(*fixed);
      double soc2 = -1, second_ms = -1, second_soc = -1;
      DDPlan plan2 = run_first_incumbent_search(
          *fixed_view_storage, *fixed, phase2_limit, seed, use_macro, stats,
          nullptr, &soc2, &second_ms, &second_soc, &max_depth,
          &targets_done, &deferred_cleanup);
      if (!plan2.empty()) {
        if (stats != nullptr) {
          ++stats->assignment_second_solved;
          stats->assignment_second_solution_ms =
              phase2_started_ms + second_ms;
          stats->assignment_second_soc = soc2;
          stats->assignment_second_makespan =
              static_cast<long>(plan2.size());
        }
        if (soc2 < soc) {
          if (stats != nullptr) ++stats->assignment_improvements;
          plan = std::move(plan2);
          soc = soc2;
        }
      }
    }
  }

  auto finish = [&](DDPlan final_plan, double final_soc) {
    // A plan is not returned until all deferred solver-owned search state
    // has been destroyed.  fixed_view_storage remains alive across this
    // clear, preserving the pass-2 planner's instance pointer.
    deferred_cleanup.clear();
    if (!final_plan.empty()) {
      auto state = initial_phys_config(ins);
      bool valid = true;
      for (const auto& ops : final_plan) {
        auto next = apply_ops(ins, state, ops);
        if (!next.has_value()) {
          valid = false;
          break;
        }
        state = std::move(*next);
      }
      valid = valid && is_dd_goal(ins, state);
      const double deliverable_ms =
          std::chrono::duration<double, std::milli>(
              Clock::now() - started)
              .count();
      const auto finalization = dd_classify_finalization_probe(
          valid, deliverable_ms,
          std::max(0.0, time_limit_sec) * 1000.0);
      if (finalization != DDFinalizationStatus::ACCEPT) {
        final_plan.clear();
        final_soc = -1;
        if (stats != nullptr)
          stats->timed_out =
              finalization == DDFinalizationStatus::DEADLINE;
      } else if (stats != nullptr) {
        stats->deliverable_ms = deliverable_ms;
      }
    }
    if (stats != nullptr) {
      stats->first_solution_ms = first_ms;
      stats->first_solution_soc = first_soc;
      stats->best_soc = final_plan.empty() ? -1 : final_soc;
      stats->max_depth = max_depth;
      stats->best_targets_done = targets_done;
      // timed_out accumulated per pass above: an empty plan is a timeout
      // only if some pass actually expired; OPEN exhaustion and generator
      // failure report as a plain (non-timeout) failure.
      stats->timed_out = final_plan.empty() && stats->timed_out;
    }
    return final_plan;
  };
  return finish(std::move(plan), soc);
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
