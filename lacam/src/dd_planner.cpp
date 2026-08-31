//
// Carrier (DD) entry adapters over the INTEGRATED LaCAM-TAPF planner
// (design.md v3 section 10; debug.md v3 WP5, mappings M11/M15/M16).
//
// There is exactly ONE solve loop in this codebase: TAPFPlanner::solve()
// in tapf_planner.cpp.  This file only (a) converts DDInstance/PhysConfig
// to the TAPF state types, (b) drives the D14 two-phase anytime policy,
// (c) implements the B0/B1 baselines on the SHARED generator/rollout, and
// (d) re-exports the protected test-support probes on top of the
// production guidance machinery (carrier_guidance.hpp).
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
using carrier_detail::env_int;
using carrier_detail::LowerDist;
using carrier_detail::PathCache;
using carrier_detail::Scratch;
using carrier_detail::build_guidance;
using carrier_detail::fill_occupancy;
using carrier_detail::least_blocking_path;
using carrier_detail::waitfor_cycles;
using carrier_detail::CLEAR_CHAIN_K;

// physical-cost weights for REPORTING/phase comparison (unit unless
// DD_SOLVER_WEIGHTS=1; must match TAPFPlanner's g semantics)
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
  out->g_relaxed += t.g_relaxed;
  out->f_pruned += t.f_pruned;
  out->incumbent_updates += t.incumbent_updates;
  out->guidance_builds += t.guidance_builds;
}

// path-cache diagnostics (engine-internal counters via the planner)
void fold_path_stats(const TAPFPlanner& planner, DDStats* out)
{
  if (out == nullptr) return;
  out->path_recomputes += planner.carrier_path_recomputes();
  out->path_cache_hits += planner.carrier_path_cache_hits();
}

// one phase = one TAPFPlanner::solve() run (M11)
DDPlan run_phase(const TAPFInstance& view, const DDInstance& ins,
                 double limit_sec, int seed, bool macro_enabled,
                 bool stop_at_first, double incumbent_init, DDStats* stats,
                 DDPlan* best_effort, double* soc_out, double* first_ms,
                 double* first_soc, long* max_depth, long* targets_done)
{
  std::mt19937 mt(seed);
  Deadline deadline(std::max(0.0, limit_sec) * 1000);
  TAPFStats tstats;
  TAPFSearchConfig cfg;
  cfg.mode = TAPFSearchMode::FOCAL;  // DFS until 1st solution, FOCAL after
  cfg.macro_enabled = macro_enabled;
  cfg.stop_at_first = stop_at_first;
  cfg.incumbent_init = incumbent_init;
  TAPFPlanner planner(&view, &deadline, &mt, 0, 0, 0.001f, true, &tstats,
                      cfg);
  const auto sol = planner.solve();
  map_stats(tstats, stats);
  fold_path_stats(planner, stats);
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
      }
    }
    return {};
  }
  auto plan = plan_of(view, sol, planner.solution_shelves);
  if (soc_out != nullptr) *soc_out = plan_soc(ins, plan);
  return plan;
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
  const auto t_end =
      Clock::now() + std::chrono::duration_cast<Clock::duration>(
                         std::chrono::duration<double>(time_limit_sec));
  auto remaining = [&]() {
    return std::chrono::duration<double>(t_end - Clock::now()).count();
  };

  // D14 two-phase anytime: phase 1 macro-on (scale regime), stop at the
  // first incumbent; phase 2 primitive-only from the root with the
  // phase-1 upper bound; return the better plan.
  const bool macro_p1 =
      env_int("DD_MACRO_CAP", 64) > 0 &&
      ins.n_targets() <= (size_t)env_int("DD_MACRO_TGT", 64);
  double soc1 = -1, first_ms = -1, first_soc = -1;
  long max_depth = 0, targets_done = 0;
  DDPlan plan1 = run_phase(view, ins, remaining(), seed, macro_p1,
                           /*stop_at_first=*/true, -1, stats, best_effort,
                           &soc1, &first_ms, &first_soc, &max_depth,
                           &targets_done);
  auto finish = [&](const DDPlan& plan, double soc) {
    if (stats != nullptr) {
      stats->first_solution_ms = first_ms;
      stats->first_solution_soc = first_soc;
      stats->best_soc = plan.empty() ? -1 : soc;
      stats->max_depth = max_depth;
      stats->best_targets_done = targets_done;
      stats->timed_out = plan.empty();
    }
    return plan;
  };
  if (remaining() <= 0) return finish(plan1, soc1);

  double soc2 = -1;
  DDPlan plan2 = run_phase(view, ins, remaining(), seed + 1,
                           /*macro=*/false, /*stop_at_first=*/false,
                           plan1.empty() ? -1 : soc1, stats, nullptr, &soc2,
                           &first_ms, &first_soc, &max_depth, &targets_done);
  if (plan2.empty()) return finish(plan1, soc1);
  if (plan1.empty()) return finish(plan2, soc2);
  return soc2 < soc1 ? finish(plan2, soc2) : finish(plan1, soc1);
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

  std::vector<DDDistCache> tgd;
  tgd.reserve(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) tgd.emplace_back(ins.grid);
  LowerDist ld(ins.grid);
  Scratch sc(ins.grid.size());

  // ---- stage 1: freeze per-target least-blocking paths at X0 ----
  PhysConfig X = initial_phys_config(ins);
  fill_occupancy(ins, X, sc);
  std::vector<std::vector<int>> fixed(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    fixed[b] = least_blocking_path(ins.grid, ins.target_starts[b],
                                   ins.target_goals[b], sc.upper,
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
        if (c == ins.target_goals[b]) {
          park[b] = 1;
          park_owner[b] = (int)o;
          break;
        }
      if (park[b]) break;
    }

  std::vector<size_t> idx(ins.n_targets(), 0);

  auto build_2stage_guidance = [&](const PhysConfig& s) {
    CarrierGuidance g;
    const size_t R = ins.n_robots();
    fill_occupancy(ins, s, sc);
    std::fill(sc.protect.begin(), sc.protect.end(), 0);
    g.target_next.assign(ins.n_targets(), -1);
    g.target_park.assign(ins.n_targets(), 0);
    auto done_in_X = [&](int o) {
      if (s.target_pos[o] != ins.target_goals[o]) return false;
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
      const bool done = !carried && s.target_pos[b] == ins.target_goals[b];
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
      sc.protect[ins.target_goals[b]] = 1;
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
              if (ins.target_goals[b2] == u) {
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
      if (k >= 0) return tgd[k].to(ins.target_goals[k])[phys.robots[i]];
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
  const SocWeights w = soc_weights_from_env();
  std::vector<DDDistCache> tgd;
  tgd.reserve(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) tgd.emplace_back(ins.grid);
  const auto X = initial_phys_config(ins);
  std::vector<char> carried(ins.n_targets(), 0);
  for (int k : X.kappa)
    if (k >= 0) carried[k] = 1;
  double h = 0;
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    const bool done = !carried[b] && X.target_pos[b] == ins.target_goals[b];
    if (done) continue;
    const auto& d = tgd[b].to(ins.target_goals[b]);
    h += w.alpha * d[X.target_pos[b]];
    h += w.gamma * (carried[b] ? 1 : 2);
  }
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
  std::vector<DDDistCache> tgd;
  tgd.reserve(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) tgd.emplace_back(ins.grid);
  LowerDist ld(ins.grid);
  PathCache pc;
  pc.strict_inval = strict_inval;
  Scratch sc(ins.grid.size());

  if (warm_block_cell >= 0) {
    // occupied->vacated history: warm each unfinished target's cached path
    // under an occupancy where warm_block_cell is occupied
    std::vector<uint8_t> occ(ins.grid.size(), 0);
    for (size_t b = 0; b < ins.n_targets(); ++b) occ[X.target_pos[b]] = 1;
    for (int c : X.anon_occ) occ[c] = 1;
    occ[warm_block_cell] = 1;
    for (size_t b = 0; b < ins.n_targets(); ++b) {
      const bool done = X.target_pos[b] == ins.target_goals[b];
      if (done) continue;
      pc.get(ins.grid, (int)b, X.target_pos[b], ins.target_goals[b], occ,
             X.target_pos[b], sc);
    }
  }

  sc.occ_node = nullptr;
  const int key = 1;  // any non-null key
  const auto g = build_guidance(ins, X, tgd, ld, pc, sc, &key);
  if (path_out) {
    path_out->clear();
    sc.occ_node = nullptr;
    fill_occupancy(ins, X, sc);
    *path_out = pc.get(ins.grid, 0, X.target_pos[0], ins.target_goals[0],
                       sc.upper_path, X.target_pos[0], sc);
  }
  return g.target_park;
}

std::vector<int> dd_match_free_goals(const DDInstance& ins,
                                     const PhysConfig& X,
                                     const std::vector<int>* parent_free_goal)
{
  std::vector<DDDistCache> tgd;
  tgd.reserve(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) tgd.emplace_back(ins.grid);
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
  const auto g = build_guidance(ins, X, tgd, ld, pc, sc, &key, nullptr, pg);
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
  std::vector<DDDistCache> tgd;
  tgd.reserve(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) tgd.emplace_back(ins.grid);
  LowerDist ld(ins.grid);
  PathCache pc;
  Scratch sc(ins.grid.size());
  sc.occ_node = nullptr;
  const int key = 1;
  const auto g = build_guidance(ins, X, tgd, ld, pc, sc, &key);
  sc.occ_node = nullptr;
  fill_occupancy(ins, X, sc);
  return waitfor_cycles(ins, X, g, ld, sc);
}

int dd_parking_cell(const DDInstance& ins, const PhysConfig& X, int robot)
{
  std::vector<DDDistCache> tgd;
  tgd.reserve(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b) tgd.emplace_back(ins.grid);
  LowerDist ld(ins.grid);
  PathCache pc;
  Scratch sc(ins.grid.size());
  sc.occ_node = nullptr;
  const int key = 1;
  const auto g = build_guidance(ins, X, tgd, ld, pc, sc, &key);
  return g.parking_cell[robot];
}
