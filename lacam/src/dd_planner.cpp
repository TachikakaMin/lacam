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
#include <unordered_set>

#include "../include/tapf_planner.hpp"
#include "../include/search_kernel.hpp"
#include "carrier_guidance.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using carrier_detail::LowerDist;
using carrier_detail::PathCache;
using carrier_detail::Scratch;
using carrier_detail::build_guidance;
using carrier_detail::fill_occupancy;
using carrier_detail::least_blocking_path;
using carrier_detail::waitfor_cycles;
using carrier_detail::CLEAR_CHAIN_K;

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

// production tau at X (design_final 5.3) on fresh local caches; used by
// the probes and by B1's frozen ROOT matching (D23)
std::vector<int> tau_of(const DDInstance& ins, const PhysConfig& X)
{
  const SocWeights w = soc_weights_from_env();
  DDDistCache uw(ins.grid);
  carrier_detail::TauEngine te;
  return carrier_detail::solve_tau(ins, X, uw, te, w.alpha, w.gamma,
                                   nullptr, nullptr, nullptr);
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
  out->tau_change_builds += t.tau_change_builds;
  out->tau_pair_changes += t.tau_pair_changes;
  out->rho_change_builds += t.rho_change_builds;
  out->rho_pair_changes += t.rho_pair_changes;
  out->tau_price_repairs += t.tau_price_repairs;
  out->rewire_guidance_rebuilds += t.rewire_guidance_rebuilds;
  out->g_relaxed += t.g_relaxed;
  out->f_pruned += t.f_pruned;
  out->incumbent_updates += t.incumbent_updates;
  out->guidance_builds += t.guidance_builds;
  out->tau_time_ms += t.tau_time_ms;
  out->guidance_time_ms += t.guidance_time_ms;
  out->futile_lift_demotions += t.futile_lift_demotions;
}

// path-cache diagnostics (engine-internal counters via the planner)
void fold_path_stats(const TAPFPlanner& planner, DDStats* out)
{
  if (out == nullptr) return;
  out->path_recomputes += planner.carrier_path_recomputes();
  out->path_cache_hits += planner.carrier_path_cache_hits();
}

DDPlan run_first_incumbent_search(
    const TAPFInstance& view, const DDInstance& ins, double limit_sec,
    int seed, bool macro_enabled, DDStats* stats, DDPlan* best_effort,
    double* soc_out, double* first_ms, double* first_soc, long* max_depth,
    long* targets_done)
{
  std::mt19937 mt(seed);
  Deadline deadline(std::max(0.0, limit_sec) * 1000);
  TAPFStats tstats;
  TAPFSearchConfig cfg;
  cfg.macro_enabled = macro_enabled;
  cfg.stop_at_first = true;
  TAPFPlanner planner(&view, &deadline, &mt, 0, 0, 0.001f, false, &tstats,
                      cfg);
  const auto sol = planner.solve();
  map_stats(tstats, stats);
  fold_path_stats(planner, stats);
  // Failure-class propagation (review fix 2026-09-01): remember whether
  // this pass actually hit the deadline. The adapter's final timed_out
  // must not conflate exhaustion/generator failure with timeout.
  if (stats != nullptr) stats->timed_out |= tstats.timed_out;
  if (max_depth != nullptr)
    *max_depth = std::max<long>(*max_depth, planner.deepest_depth);
  if (targets_done != nullptr)
    *targets_done = std::max<long>(*targets_done, planner.best_targets_done);
  if (first_ms != nullptr && *first_ms < 0 && tstats.first_solution_g >= 0) {
    *first_ms = tstats.first_solution_time_ms;
    *first_soc = tstats.first_solution_g;
  }
  if (sol.empty()) {
    if (best_effort != nullptr && !planner.best_effort_solution.empty()) {
      *best_effort = plan_of(view, planner.best_effort_solution,
                             planner.best_effort_shelves);
      if (stats != nullptr && !planner.best_effort_shelves.empty()) {
        stats->deepest_config = phys_of(planner.best_effort_solution.back(),
                                        planner.best_effort_shelves.back());
        stats->deepest_tau = planner.best_effort_tau;
      }
    }
    return {};
  }
  auto plan = plan_of(view, sol, planner.solution_shelves);
  DDPlanRepairStats repair;
  // R1 (debug.md §10): the repair consumes the SAME pass deadline; on
  // expiry it returns the raw plan.  A pass that cannot finish
  // search + mandatory repair inside its budget has NOT produced its
  // deliverable: the un-repaired raw incumbent (100k+ steps on the
  // borderline case) cannot be printed/validated inside the protocol
  // window either, so the pass reports an honest timeout instead.
  plan = repair_carrier_plan(ins, plan, &repair, &deadline);
  if (is_expired(&deadline)) {
    if (stats != nullptr) stats->timed_out = true;
    return {};
  }
  if (stats != nullptr) {
    stats->exact_loops += repair.exact_loops;
    stats->projected_loops += repair.projected_loops;
    stats->bridge_steps += repair.bridge_steps;
    stats->plan_steps_removed += repair.steps_removed;
  }
  if (soc_out != nullptr) *soc_out = plan_soc(ins, plan);
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
  double soc = -1, first_ms = -1, first_soc = -1;
  long max_depth = 0, targets_done = 0;
  DDPlan plan = run_first_incumbent_search(
      view, ins, remaining(), seed, use_macro, stats, best_effort, &soc,
      &first_ms, &first_soc, &max_depth, &targets_done);

  if (!plan.empty()) {
    auto fixed = fixed_assignment_from_plan(ins, plan);
    const double phase2_limit = remaining();
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
      const TAPFInstance fixed_view(*fixed);
      double soc2 = -1, second_ms = -1, second_soc = -1;
      DDPlan plan2 = run_first_incumbent_search(
          fixed_view, *fixed, remaining(), seed, use_macro, stats, nullptr,
          &soc2, &second_ms, &second_soc, &max_depth, &targets_done);
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

  auto finish = [&](const DDPlan& plan, double soc) {
    if (stats != nullptr) {
      stats->first_solution_ms = first_ms;
      stats->first_solution_soc = first_soc;
      stats->best_soc = plan.empty() ? -1 : soc;
      stats->max_depth = max_depth;
      stats->best_targets_done = targets_done;
      // timed_out accumulated per pass above: an empty plan is a timeout
      // only if some pass actually expired; OPEN exhaustion and generator
      // failure report as a plain (non-timeout) failure.
      stats->timed_out = plan.empty() && stats->timed_out;
    }
    return plan;
  };
  return finish(plan, soc);
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
    auto r = planner.carrier_rollout(C, S, 512, 0, /*stop_on_event=*/false);
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

DDPlan solve_carrier_2stage(const DDInstance& ins, double time_limit_sec,
                            int seed, DDStats* stats,
                            std::vector<std::vector<int>>* fixed_paths_out)
{
  // B1 (design 8.1, M15): stage 1 freezes per-target least-blocking paths
  // at X0; stage 2 executes them with the SHARED generator under the
  // plan_bound hard constraint.  Deliberately NOT complete.
  const TAPFInstance view(ins);
  if (stats != nullptr) *stats = DDStats();
  const auto t_start = Clock::now();
  std::mt19937 mt(seed);
  Deadline deadline(time_limit_sec * 1000);
  TAPFStats tstats;
  TAPFPlanner planner(&view, &deadline, &mt, 0, 0, 0.001f, true, &tstats);

  DDDistCache tgd(ins.grid);
  LowerDist ld(ins.grid);
  Scratch sc(ins.grid.size());

  // ---- stage 1: freeze per-target least-blocking paths at X0 ----
  // B1's tau is the ROOT matching, frozen (D23): the honest "decide
  // goals once, then execute" decomposition baseline.
  PhysConfig X = initial_phys_config(ins);
  const std::vector<int> tau_root = tau_of(ins, X);
  fill_occupancy(ins, X, sc, tau_root);
  std::vector<std::vector<int>> fixed(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    fixed[b] = least_blocking_path(ins.grid, ins.target_starts[b],
                                   tau_root[b], sc.upper,
                                   ins.target_starts[b], sc);
    if (fixed[b].empty()) return {};  // no wall-feasible path
  }
  if (fixed_paths_out) *fixed_paths_out = fixed;
  // static park relation: goal_b on another target's FIXED path
  std::vector<uint8_t> park(ins.n_targets(), 0);
  std::vector<int> park_owner(ins.n_targets(), -1);
  for (size_t b = 0; b < ins.n_targets(); ++b)
    for (size_t o = 0; o < ins.n_targets(); ++o) {
      if (o == b) continue;
      for (int c : fixed[o])
        if (c == tau_root[b]) {
          park[b] = 1;
          park_owner[b] = (int)o;
          break;
        }
      if (park[b]) break;
    }

  std::vector<size_t> idx(ins.n_targets(), 0);

  auto build_2stage_guidance = [&](const PhysConfig& s) {
    CarrierGuidance g;
    g.tau = tau_root;  // frozen (D23)
    const size_t R = ins.n_robots();
    fill_occupancy(ins, s, sc, tau_root);
    std::fill(sc.protect.begin(), sc.protect.end(), 0);
    g.target_next.assign(ins.n_targets(), -1);
    g.target_park.assign(ins.n_targets(), 0);
    auto done_in_X = [&](int o) {
      if (s.target_pos[o] != tau_root[o]) return false;
      for (int k : s.kappa)
        if (k == o) return false;
      return true;
    };
    for (size_t b = 0; b < ins.n_targets(); ++b) {
      while (idx[b] + 1 < fixed[b].size() &&
             fixed[b][idx[b]] != s.target_pos[b])
        ++idx[b];
      if (fixed[b][idx[b]] != s.target_pos[b]) idx[b] = 0;  // safety
      const bool carried = [&] {
        for (int k : s.kappa)
          if (k == (int)b) return true;
        return false;
      }();
      const bool done = !carried && s.target_pos[b] == tau_root[b];
      if (done) continue;
      for (size_t j = idx[b]; j < fixed[b].size(); ++j)
        sc.protect[fixed[b][j]] = 1;
      if (idx[b] + 1 < fixed[b].size())
        g.target_next[b] = fixed[b][idx[b] + 1];
      if (park[b] && park_owner[b] >= 0 && !done_in_X(park_owner[b]))
        g.target_park[b] = 1;
      const int nxt = g.target_next[b];
      const bool head_free = nxt >= 0 && sc.grounded[nxt] == 0;
      if (!carried && head_free && !g.target_park[b]) {
        CarrierRequest r;
        r.cell = s.target_pos[b];
        r.priority = 100;  // serve
        g.requests.push_back(r);
      }
      int emitted = 0;
      for (size_t j = idx[b] + 1;
           j < fixed[b].size() && emitted < CLEAR_CHAIN_K; ++j) {
        const int cur = fixed[b][j];
        const int gr = sc.grounded[cur];
        if (gr == -1 || (gr > 0 && gr - 1 != (int)b)) {
          CarrierRequest r;
          r.cell = cur;
          r.priority = 50 - emitted;  // clear
          g.requests.push_back(r);
          ++emitted;
        }
      }
    }
    for (size_t b = 0; b < ins.n_targets(); ++b)
      sc.protect[tau_root[b]] = 1;
    // rho greedy off the fixed plan
    g.rho.assign(R, -1);
    g.free_goal.assign(R, -1);
    g.parking_cell.assign(R, -1);
    std::vector<int> req_order(g.requests.size());
    for (size_t i = 0; i < req_order.size(); ++i) req_order[i] = (int)i;
    std::stable_sort(req_order.begin(), req_order.end(), [&](int a, int b2) {
      return g.requests[a].priority > g.requests[b2].priority;
    });
    std::vector<bool> used(R, false);
    int free_left = 0;
    for (size_t i = 0; i < R; ++i) {
      if (s.kappa[i] != KAPPA_FREE)
        used[i] = true;
      else
        ++free_left;
    }
    for (int ri : req_order) {
      if (free_left == 0) break;
      const auto& req = g.requests[ri];
      int best = -1, bestd = INT_MAX / 2;
      for (size_t i = 0; i < R; ++i) {
        if (used[i]) continue;
        const int dd = ld.dist(req.cell, s.robots[i]);
        if (dd < bestd) {
          bestd = dd;
          best = (int)i;
        }
      }
      if (best >= 0) {
        used[best] = true;
        --free_left;
        g.rho[best] = ri;
        g.free_goal[best] = req.cell;
      }
    }
    // parking for ANON / parked-target carriers
    for (size_t i = 0; i < R; ++i) {
      const bool anon_c = s.kappa[i] == KAPPA_ANON;
      const bool parked_c = s.kappa[i] >= 0 && g.target_park[s.kappa[i]];
      if (!anon_c && !parked_c) continue;
      int found = -1, fallback = -1;
      std::fill(sc.prev.begin(), sc.prev.end(), 0);
      std::deque<int> dq;
      dq.push_back(s.robots[i]);
      sc.prev[s.robots[i]] = 1;
      int nb[4];
      while (!dq.empty() && found < 0) {
        int u = dq.front();
        dq.pop_front();
        if (u != s.robots[i] && !sc.upper[u]) {
          if (!sc.protect[u]) {
            found = u;
            break;
          }
          if (fallback < 0) {
            bool is_goal = false;
            for (size_t b2 = 0; b2 < ins.n_targets(); ++b2)
              if (tau_root[b2] == u) {
                is_goal = true;
                break;
              }
            if (!is_goal) fallback = u;
          }
        }
        const int n = ins.grid.neighbors(u, nb);
        for (int k = 0; k < n; ++k)
          if (!sc.prev[nb[k]]) {
            sc.prev[nb[k]] = 1;
            dq.push_back(nb[k]);
          }
      }
      g.parking_cell[i] = found >= 0 ? found : fallback;
    }
    return g;
  };

  // ---- stage 2: rolling execution via the SHARED generator ----
  DDPlan plan;
  auto [C, S] = state_of(view, X);
  if (planner.is_goal_config(C, S)) {
    plan.push_back(std::vector<Op>(ins.n_robots(), Op::make_wait()));
    return plan;
  }
  std::unordered_set<uint64_t> seen;
  seen.insert(state_hash(C, S));
  while (std::chrono::duration<double>(Clock::now() - t_start).count() <
         time_limit_sec) {
    auto node = std::make_unique<TAPFNode>(
        C, S, planner.D, &view, std::vector<int>((int)view.N, -1),
        TAPFAssignmentState(), nullptr);
    // per-step probe nodes recycle addresses (see carrier_rollout)
    planner.invalidate_carrier_scratch();
    const auto phys = phys_of(C, S);
    node->guide =
        std::make_unique<CarrierGuidance>(build_2stage_guidance(phys));
    node->guide->plan_bound = true;
    // class-layered order (loaded > clear > assigned > idle; rem, id)
    auto cls = [&](int i) {
      const int k = S.kappa[i];
      if (k >= 0) return node->guide->target_park[k] ? 1 : 0;
      if (k == KAPPA_ANON) return 1;
      if (node->guide->rho[i] >= 0) return 2;
      return 3;
    };
    auto rem = [&](int i) -> int {
      const int k = S.kappa[i];
      if (k >= 0) return tgd.to(tau_root[k])[phys.robots[i]];
      return 0;
    };
    std::iota(node->order.begin(), node->order.end(), 0);
    std::stable_sort(node->order.begin(), node->order.end(),
                     [&](int a, int b) {
                       const int ca = cls(a), cb = cls(b);
                       if (ca != cb) return ca < cb;
                       return rem(a) < rem(b);
                     });
    node->constraint_order = node->order;
    TAPFConstraint root;
    if (!planner.get_new_config(node.get(), &root)) {
      map_stats(tstats, stats);
      return {};
    }
    if (!planner.apply_carrier_effects(node.get())) {
      map_stats(tstats, stats);
      return {};
    }
    Config C_new(view.N, nullptr);
    for (auto a : planner.A) C_new[a->id] = a->v_next;
    plan.push_back(planner.ops_scratch);
    C = C_new;
    S = planner.shelf_next_scratch;
    if (!seen.insert(state_hash(C, S)).second) {
      map_stats(tstats, stats);
      return {};  // cycle -> honest failure
    }
    if (planner.is_goal_config(C, S)) {
      map_stats(tstats, stats);
      return plan;
    }
  }
  map_stats(tstats, stats);
  if (stats != nullptr) stats->timed_out = true;
  return {};
}

double dd_root_admissible_h(const DDInstance& ins)
{
  // design_final 4.3: the admissible LB-MATCHING value — exactly what
  // the planner folds into node h (singleton path == the old formula)
  const SocWeights w = soc_weights_from_env();
  DDDistCache uw(ins.grid);
  carrier_detail::TauEngine te;
  const auto X = initial_phys_config(ins);
  double h = 0;
  carrier_detail::solve_tau(ins, X, uw, te, w.alpha, w.gamma, nullptr,
                            nullptr, &h);
  return h;
}

// ===================================================================
// TEST SUPPORT (protected suites): probes of the PRODUCTION machinery
// ===================================================================

namespace {

// drain one node's operator-constraint tree through the production
// expansion + generation (G1 conformance, debug.md P0-1/P0-2/P0-5 lineage)
std::vector<PhysConfig> drain_node(const TAPFInstance& view,
                                   const PhysConfig& X, int seed,
                                   int n_reguides,
                                   std::vector<int>* order_before,
                                   std::vector<int>* order_after)
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
  if (order_before != nullptr) *order_before = node->constraint_order;

  std::vector<PhysConfig> out;
  std::unordered_set<uint64_t> seen;
  const int R = (int)view.N;
  size_t pops = 0;
  int applied = 0;
  std::vector<OpCand> cand;
  std::vector<TAPFConstraint*> popped;  // delete after the drain
  while (!node->search_tree.empty()) {
    if (applied < n_reguides && pops > 0 && pops % (R + 1) == 0) {
      planner.attach_carrier_guidance(node.get(), /*reguide=*/true);
      ++applied;
    }
    ++pops;
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
  while (applied < n_reguides) {  // requested count even on tiny trees
    planner.attach_carrier_guidance(node.get(), /*reguide=*/true);
    ++applied;
  }
  if (order_after != nullptr) *order_after = node->constraint_order;
  for (auto* M : popped) delete M;
  return out;
}

}  // namespace

std::vector<PhysConfig> dd_enumerate_node_successors(const DDInstance& ins,
                                                     const PhysConfig& X,
                                                     int seed)
{
  const TAPFInstance view(ins);
  return drain_node(view, X, seed, 0, nullptr, nullptr);
}

std::vector<PhysConfig> dd_enumerate_node_successors_reguided(
    const DDInstance& ins, const PhysConfig& X, int seed, int n_reguides,
    std::vector<int>* constraint_order_before,
    std::vector<int>* constraint_order_after)
{
  const TAPFInstance view(ins);
  return drain_node(view, X, seed, n_reguides, constraint_order_before,
                    constraint_order_after);
}

std::vector<uint8_t> dd_compute_park(const DDInstance& ins,
                                     const PhysConfig& X,
                                     int warm_block_cell, bool strict_inval,
                                     std::vector<int>* path_out)
{
  DDDistCache tgd(ins.grid);
  LowerDist ld(ins.grid);
  PathCache pc(strict_inval);
  Scratch sc(ins.grid.size());
  const auto tau = tau_of(ins, X);

  if (warm_block_cell >= 0) {
    // occupied->vacated history: warm each unfinished target's cached path
    // under an occupancy where warm_block_cell is occupied
    std::vector<uint8_t> occ(ins.grid.size(), 0);
    for (size_t b = 0; b < ins.n_targets(); ++b) occ[X.target_pos[b]] = 1;
    for (int c : X.anon_occ) occ[c] = 1;
    occ[warm_block_cell] = 1;
    for (size_t b = 0; b < ins.n_targets(); ++b) {
      bool carried = false;
      for (const int k : X.kappa) carried |= k == (int)b;
      const bool done = !carried && X.target_pos[b] == tau[b];
      if (done) continue;
      pc.get(ins.grid, (int)b, X.target_pos[b], tau[b], occ,
             X.target_pos[b], sc);
    }
  }

  sc.occ_node = nullptr;
  const int key = 1;  // any non-null key
  const auto g = build_guidance(ins, X, tau, tgd, ld, pc, sc, &key);
  if (path_out) {
    path_out->clear();
    sc.occ_node = nullptr;
    fill_occupancy(ins, X, sc, tau);
    *path_out = pc.get(ins.grid, 0, X.target_pos[0], tau[0],
                       sc.upper_path, X.target_pos[0], sc);
  }
  return g.target_park;
}

std::vector<int> dd_match_free_goals(const DDInstance& ins,
                                     const PhysConfig& X,
                                     const std::vector<int>* parent_free_goal)
{  DDDistCache tgd(ins.grid);
  LowerDist ld(ins.grid);
  PathCache pc;
  Scratch sc(ins.grid.size());
  CarrierGuidance parent;
  const CarrierGuidance* pg = nullptr;
  if (parent_free_goal) {
    parent.free_goal = *parent_free_goal;
    parent.rho.assign(ins.n_robots(), -1);
    for (size_t i = 0; i < ins.n_robots(); ++i)
      if ((*parent_free_goal)[i] >= 0) parent.rho[i] = 0;  // any valid idx
    pg = &parent;
  }
  sc.occ_node = nullptr;
  const int key = 1;
  const auto tau = tau_of(ins, X);
  const auto g =
      build_guidance(ins, X, tau, tgd, ld, pc, sc, &key, nullptr, pg);
  return g.free_goal;
}

std::vector<int> dd_least_blocking_path(const DDGrid& g, int src, int dst,
                                        const std::vector<uint8_t>& occupied,
                                        const std::vector<int>* prev_path)
{
  Scratch sc(g.size());
  return least_blocking_path(g, src, dst, occupied, /*exclude=*/-1, sc,
                             prev_path);
}

std::vector<ManipulationTask> dd_build_tasks(const DDInstance& ins,
                                             const PhysConfig& X,
                                             std::vector<int>* rho_task_out)
{
  // v3.0 §3/§5 probe: the PRODUCTION guidance builder's task pool for X
  // (fresh caches, same construction as the other guidance probes),
  // including the SHARED execution-price repair round (§5.1).
  const SocWeights w = soc_weights_from_env();
  DDDistCache tgd(ins.grid);
  LowerDist ld(ins.grid);
  PathCache pc;
  Scratch sc(ins.grid.size());
  carrier_detail::TauEngine te;
  sc.occ_node = nullptr;
  const int key = 1;
  auto tau = carrier_detail::solve_tau(ins, X, tgd, te, w.alpha, w.gamma,
                                       nullptr, nullptr, nullptr);
  auto g = build_guidance(ins, X, tau, tgd, ld, pc, sc, &key);
  if ((int)ins.n_targets() <= carrier_detail::ASSIGNMENT_EXACT_LIMIT &&
      !te.all_singleton) {
    std::vector<std::pair<int, double>> price;
    if (carrier_detail::compute_execution_prices(ins, X, g, tau, tgd, ld,
                                                 w.alpha, price)) {
      auto tau1 = carrier_detail::solve_tau(ins, X, tgd, te, w.alpha,
                                            w.gamma, nullptr, nullptr,
                                            nullptr, false, &price);
      if (tau1 != tau) {
        tau = std::move(tau1);
        sc.occ_node = nullptr;
        g = build_guidance(ins, X, tau, tgd, ld, pc, sc, &key);
      }
    }
  }
  if (rho_task_out != nullptr) *rho_task_out = g.rho_task;
  return std::move(g.tasks);
}

std::vector<DDCustodyStep> dd_rollout_custody_trace(const DDInstance& ins,
                                                    int robot, int max_steps,
                                                    int seed)
{
  // debug.md §7.2 test 4 probe: production generator loop with
  // parent-guide chaining (custody flows through rollout_parent_guide,
  // exactly as in carrier_rollout / the 2-stage executor).
  const TAPFInstance view(ins);
  std::mt19937 mt(seed);
  TAPFStats tstats;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &tstats);
  auto C = view.starts;
  auto S = initial_shelf_state(view);
  std::vector<DDCustodyStep> trace;
  std::unique_ptr<TAPFNode> prev;
  for (int step = 0; step < max_steps; ++step) {
    if (planner.is_goal_config(C, S)) break;
    auto node = std::make_unique<TAPFNode>(
        C, S, planner.D, &view, std::vector<int>((int)view.N, -1),
        TAPFAssignmentState(), nullptr);
    planner.invalidate_carrier_scratch();
    planner.attach_carrier_guidance(
        node.get(), false, prev != nullptr ? prev->guide.get() : nullptr);
    DDCustodyStep rec;
    rec.kappa = S.kappa.empty() ? KAPPA_FREE : S.kappa[robot];
    rec.cell = C[robot]->index;
    const auto& g = *node->guide;
    if ((size_t)robot < g.rho_task.size() && g.rho_task[robot] >= 0 &&
        g.rho_task[robot] < (int)g.tasks.size())
      rec.bound_id = g.tasks[g.rho_task[robot]].id;
    if ((size_t)robot < g.custody.size()) {
      rec.custody_id = g.custody[robot].id;
      rec.custody_to = g.custody[robot].to;
    }
    trace.push_back(rec);
    TAPFConstraint root;
    if (!planner.get_new_config(node.get(), &root)) break;
    if (!planner.apply_carrier_effects(node.get())) break;
    for (auto a : planner.A) C[a->id] = a->v_next;
    S = planner.shelf_next_scratch;
    prev = std::move(node);
  }
  return trace;
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

std::vector<int> dd_waitfor_cycle_robots(const DDInstance& ins,
                                         const PhysConfig& X)
{
  DDDistCache tgd(ins.grid);
  LowerDist ld(ins.grid);
  PathCache pc;
  Scratch sc(ins.grid.size());
  sc.occ_node = nullptr;
  const int key = 1;
  const auto tau = tau_of(ins, X);
  const auto g = build_guidance(ins, X, tau, tgd, ld, pc, sc, &key);
  sc.occ_node = nullptr;
  fill_occupancy(ins, X, sc, tau);
  return waitfor_cycles(ins, X, g, ld, sc);
}

std::vector<int> dd_solve_tau(const DDInstance& ins, const PhysConfig& X,
                              const std::vector<int>* parent_tau,
                              double* h_out,
                              const std::vector<std::pair<int, int>>* taboo)
{
  const SocWeights w = soc_weights_from_env();
  DDDistCache uw(ins.grid);
  carrier_detail::TauEngine te;
  double h = 0;
  auto tau = carrier_detail::solve_tau(ins, X, uw, te, w.alpha, w.gamma,
                                       parent_tau, taboo, &h);
  if (h_out != nullptr) *h_out = h;
  return tau;
}

std::vector<int> dd_pathcache_dst_probe(const DDInstance& ins,
                                        const PhysConfig& X, int b, int dst1,
                                        int dst2, long* recomputes_out)
{
  // WP-C T5: exercise the production PathCache with a dst change.
  PathCache pc;
  Scratch sc(ins.grid.size());
  carrier_detail::fill_occupancy(ins, X, sc, tau_of(ins, X));
  pc.get(ins.grid, b, X.target_pos[b], dst1, sc.upper_path,
         X.target_pos[b], sc);
  const auto path = pc.get(ins.grid, b, X.target_pos[b], dst2, sc.upper_path,
                           X.target_pos[b], sc);
  if (recomputes_out != nullptr) *recomputes_out = pc.recomputes;
  return path;
}
