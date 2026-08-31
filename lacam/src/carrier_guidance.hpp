/*
 * Carrier guidance infrastructure (design 5.3/5.4a/5.5/6.2; mapping
 * M6/M8/M9) shared by the integrated TAPF planner (tapf_planner.cpp) and
 * the carrier entry/test-support adapters (dd_planner.cpp).  Internal
 * header (src/): NOT part of the public API.
 *
 * Everything here operates on the conformance-oracle view (DDInstance /
 * PhysConfig, identical cell-index encoding as Vertex::index) and is
 * ordering-only; none of it executes on shelf-free instances.  Called
 * from TAPFPlanner::attach_carrier_guidance (production) and from the
 * carrier adapters/test probes in dd_planner.cpp.
 */
#pragma once

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "../include/dd_dist_adapters.hpp"
#include "../include/tapf_planner.hpp"

namespace carrier_detail {

// =====================================================================
// Carrier guidance infrastructure (design 5.3/5.4a/5.5/6.2, mapping
// M6/M8/M9) — ported VERBATIM from the pre-integration planner (same
// cell-index encoding); operates on the oracle instance view.  None of
// this executes on shelf-free instances (no targets -> no engine).
// =====================================================================

inline int env_int(const char* k, int dflt)
{
  const char* v = std::getenv(k);
  return v ? std::atoi(v) : dflt;
}

// solver-objective weight loader (design 5.7): ONE parser for the planner
// g-weights and the adapter reporting weights (they must always agree).
// W needs fields alpha/beta/gamma/delta defaulting to 1.
template <typename W>
inline void load_solver_weights(W& w)
{
  const char* e = std::getenv("DD_SOLVER_WEIGHTS");
  if (e == nullptr || std::atoi(e) == 0) return;
  if (const char* a = std::getenv("DD_ALPHA")) w.alpha = atof(a);
  if (const char* b = std::getenv("DD_BETA")) w.beta = atof(b);
  if (const char* c = std::getenv("DD_GAMMA")) w.gamma = atof(c);
  if (const char* d = std::getenv("DD_DELTA")) w.delta = atof(d);
}

// lower-deck distance provider: exact Manhattan on wall-free grids,
// shared lazy-BFS cache otherwise.
struct LowerDist {
  const DDGrid& g;
  bool wallfree;
  DDDistCache bfs;
  explicit LowerDist(const DDGrid& g_) : g(g_), bfs(g_)
  {
    wallfree = true;
    for (uint8_t w : g.wall) wallfree &= (w == 0);
  }
  int dist(int cell, int from)
  {
    if (wallfree)
      return std::abs(g.row(cell) - g.row(from)) +
             std::abs(g.col(cell) - g.col(from));
    return bfs.to(cell)[from];
  }
};

constexpr int LAMBDA_BLK = 8;
constexpr int CLEAR_CHAIN_K = 3;
constexpr int LIVELOCK_WINDOW = 24;

// grid-sized scratch buffers reused across nodes (single-threaded)
struct Scratch {
  std::vector<uint8_t> upper;
  std::vector<uint8_t> protect;
  std::vector<int> grounded;  // 0 none; -1 anon; b+1 grounded target b
  std::vector<int> owner;
  std::vector<int> dist;
  std::vector<int> prev;
  std::vector<uint8_t> upper_path;  // carried-hover mask view (5.4a)
  std::vector<uint8_t> on_prev;     // path-inertia scratch
  const void* occ_node = nullptr;
  explicit Scratch(int n)
      : upper(n, 0), protect(n, 0), grounded(n, 0), owner(n, 0), dist(n),
        prev(n), upper_path(n, 0), on_prev(n, 0)
  {
  }
};

inline void fill_occupancy(const DDInstance& ins, const PhysConfig& s,
                           Scratch& sc, const std::vector<int>& tau)
{
  std::fill(sc.upper.begin(), sc.upper.end(), 0);
  std::fill(sc.grounded.begin(), sc.grounded.end(), 0);
  for (int p : s.anon_occ) {
    sc.upper[p] = 1;
    sc.grounded[p] = -1;
  }
  std::vector<bool> carried(ins.n_targets(), false);
  for (int k : s.kappa)
    if (k >= 0) carried[k] = true;
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    sc.upper[s.target_pos[b]] = 1;
    if (!carried[b]) sc.grounded[s.target_pos[b]] = (int)b + 1;
  }
  for (size_t i = 0; i < s.kappa.size(); ++i)
    if (s.kappa[i] == KAPPA_ANON) sc.upper[s.robots[i]] = 1;
  // path view: mask a target's own ASSIGNED goal cell (tau) ONLY while
  // that target is CARRIED and hovering on it — kills the park/deliver
  // circular dependency (design 5.4a); grounded targets stay real
  // blockers.  tau == the fixed goal on singleton instances.
  sc.upper_path = sc.upper;
  for (size_t i = 0; i < s.kappa.size(); ++i) {
    const int k = s.kappa[i];
    if (k >= 0 && s.target_pos[k] == tau[k]) sc.upper_path[tau[k]] = 0;
  }
}

inline void fill_occupancy_if_needed(const DDInstance& ins,
                                     const PhysConfig& s, Scratch& sc,
                                     const std::vector<int>& tau,
                                     const void* key)
{
  if (sc.occ_node == key && key != nullptr) return;
  fill_occupancy(ins, s, sc, tau);
  sc.occ_node = key;
}

inline std::vector<int> least_blocking_path(
    const DDGrid& g, int src, int dst,
    const std::vector<uint8_t>& occupied, int exclude, Scratch& sc,
    const std::vector<int>* prev_path = nullptr)
{
  if (src == dst) return {src};
  std::fill(sc.dist.begin(), sc.dist.end(), INT_MAX / 2);
  auto& dist = sc.dist;
  auto& prev = sc.prev;
  prev[src] = -1;
  // path inertia: scale base costs by 2N, previous-path cells get a -1
  // discount — a strict tie-break, never trades real cost (P2-13c).
  const int scale = 2 * (int)g.size();
  const bool use_prev = prev_path && !prev_path->empty();
  auto& on_prev = sc.on_prev;
  if (use_prev) {
    std::fill(on_prev.begin(), on_prev.end(), 0);
    for (int c : *prev_path) on_prev[c] = 1;
  }
  using QE = std::pair<int, int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  dist[src] = 0;
  pq.push({0, src});
  int nb[4];
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) continue;
    if (u == dst) break;
    const int n = g.neighbors(u, nb);
    for (int k = 0; k < n; ++k) {
      const int v = nb[k];
      const bool occ = v != exclude && occupied[v];
      int w = (1 + (occ ? LAMBDA_BLK : 0)) * scale;
      if (use_prev && on_prev[v]) w -= 1;
      if (dist[v] > d + w) {
        dist[v] = d + w;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }
  if (dist[dst] >= INT_MAX / 2) return {};
  std::vector<int> path;
  for (int c = dst; c != -1; c = prev[c]) path.push_back(c);
  std::reverse(path.begin(), path.end());
  return path;
}

// ---- tau: shelf->goal matching (design_final 5.3, D17/D19) ----
// ONE Hungarian per node: primary lexicographic order = admissible LB
// matrix (its optimal sum IS h_shelf), secondary = eta_B hysteresis
// toward the parent assignment, third = the engine's deterministic tie
// hash.  Structural degeneration on all-singleton (fixed-goal)
// instances: forced assignment + the exact pre-goal-set h arithmetic.
struct TauEngine {
  bool inited = false;
  bool all_singleton = false;
  std::vector<int> cols;                  // column id -> goal cell
  std::vector<std::vector<char>> elig;    // per target: eligibility by col
  TAPFAssignmentState state;
  long eta_b = 2;
  long S = 1;  // hysteresis scale: eta_b * n + 1 (sum of pens < S)
  long q = 1;  // LB quantization (1 for integral weights)
};

inline void tau_init(TauEngine& te, const DDInstance& ins, double alpha,
                     double gamma)
{
  te.inited = true;
  te.eta_b = env_int("DD_ETA_B", 2);
  te.all_singleton = true;
  for (const auto& set : ins.target_goal_sets)
    te.all_singleton = te.all_singleton && set.size() == 1;
  if (te.all_singleton) return;
  const size_t n = ins.n_targets();
  std::unordered_map<int, int> col_of;
  std::vector<std::vector<int>> col_ids(n);
  for (size_t b = 0; b < n; ++b)
    for (const int g : ins.target_goal_sets[b]) {
      const auto it = col_of.emplace(g, (int)col_of.size()).first;
      col_ids[b].push_back(it->second);
    }
  te.cols.resize(col_of.size());
  for (const auto& [cell, c] : col_of) te.cols[c] = cell;
  te.elig.assign(n, std::vector<char>(te.cols.size(), 0));
  for (size_t b = 0; b < n; ++b)
    for (const int c : col_ids[b]) te.elig[b][c] = 1;
  te.S = te.eta_b * (long)n + 1;
  const bool integral_w =
      alpha == std::floor(alpha) && gamma == std::floor(gamma);
  te.q = integral_w ? 1 : 256;
  te.state.init((int)n, (int)te.cols.size());
}

inline std::vector<int> solve_tau(
    const DDInstance& ins, const PhysConfig& s, DDDistCache& upper_wall,
    TauEngine& te, double alpha, double gamma,
    const std::vector<int>* parent_tau,
    const std::vector<std::pair<int, int>>* taboo, double* h_out)
{
  const size_t n = ins.n_targets();
  std::vector<int> tau(n, -1);
  if (n == 0) {
    if (h_out != nullptr) *h_out = 0;
    return tau;
  }
  if (!te.inited) tau_init(te, ins, alpha, gamma);
  std::vector<char> carried(n, 0);
  for (const int k : s.kappa)
    if (k >= 0) carried[k] = 1;
  // per-pair admissible lower bound (design_final 4.3 Lemma 1)
  auto lb = [&](size_t b, int g) -> double {
    double v = alpha * upper_wall.to(g)[s.target_pos[b]];
    if (carried[b])
      v += gamma;  // >= 1 drop
    else if (s.target_pos[b] != g)
      v += 2 * gamma;  // >= 1 lift + 1 drop
    return v;
  };

  if (te.all_singleton) {
    // structural degeneration (D22): forced tau; h via the EXACT
    // pre-goal-set arithmetic (same loop shape, same skip of done)
    double h = 0;
    for (size_t b = 0; b < n; ++b) {
      const int g = ins.target_goal_sets[b][0];
      tau[b] = g;
      const bool done = !carried[b] && s.target_pos[b] == g;
      if (done) continue;
      h += alpha * upper_wall.to(g)[s.target_pos[b]];
      h += gamma * (carried[b] ? 1 : 2);
    }
    if (h_out != nullptr) *h_out = h;
    return tau;
  }

  auto banned = [&](size_t b, int g) {
    if (taboo == nullptr) return false;
    if (ins.target_goal_sets[b].size() <= 1) return false;  // exempt
    for (const auto& [tb, tg] : *taboo)
      if (tb == (int)b && tg == g) return true;
    return false;
  };

  if ((int)n > env_int("DD_TAU_HUNGARIAN_TGT", 256)) {
    // scale-regime relaxation (design_final 4.3): row-wise nearest
    // eligible goal; h ignores injectivity (still admissible, weaker).
    // Hysteresis: stick to the parent goal within an eta_B slack.
    double h = 0;
    for (size_t b = 0; b < n; ++b) {
      const auto& set = ins.target_goal_sets[b];
      double best = -1;
      int best_g = -1;
      for (const int g : set) {
        if (banned(b, g)) continue;
        const double v = lb(b, g);
        if (best_g < 0 || v < best) {
          best = v;
          best_g = g;
        }
      }
      if (best_g < 0) {  // fully tabooed row: fall back to the raw min
        for (const int g : set) {
          const double v = lb(b, g);
          if (best_g < 0 || v < best) {
            best = v;
            best_g = g;
          }
        }
      }
      if (parent_tau != nullptr) {
        const int pg = (*parent_tau)[b];
        if (pg >= 0 && pg != best_g && !banned(b, pg) &&
            std::binary_search(set.begin(), set.end(), pg) &&
            lb(b, pg) <= best + (double)te.eta_b)
          best_g = pg;
      }
      tau[b] = best_g;
      h += best;  // unbiased row minimum (admissible)
    }
    if (h_out != nullptr) *h_out = h;
    return tau;
  }

  // exact regime: one Hungarian, lexicographic (LB, hysteresis, tie)
  const long INF = kTapfAssignmentInfCost;
  const long enc_cap =
      std::min<long>(INF / 2 - 1, (long)INT_MAX / (long)(n + 1));
  auto make_cost = [&](bool use_taboo) {
    return [&, use_taboo](int row, int col) -> int {
      if (!te.elig[row][col]) return (int)INF;
      const int g = te.cols[col];
      if (use_taboo && banned((size_t)row, g)) return (int)INF;
      const double v = lb((size_t)row, g);
      const long lbq = (long)std::floor(v * (double)te.q + 1e-9);
      long pen = 0;
      if (parent_tau != nullptr && (*parent_tau)[row] != g) pen = te.eta_b;
      const long enc = lbq * te.S + pen;
      if (enc > enc_cap)
        throw std::runtime_error(
            "solve_tau: cost encoding overflow (weights too large for "
            "the goal-set matching regime)");
      return (int)enc;
    };
  };
  auto res = te.state.solve_full(make_cost(true));
  if (!res.feasible && taboo != nullptr)
    res = te.state.solve_full(make_cost(false));  // taboo cornered: drop it
  if (!res.feasible)
    throw std::logic_error(
        "solve_tau: infeasible matching (covering checked at load)");
  for (size_t b = 0; b < n; ++b) tau[b] = te.cols[res.agent_to_task[b]];
  // decode the primary value: res.cost = S * sum(LBq) + sum(pen),
  // sum(pen) <= eta_b * n < S  =>  integer division recovers sum(LBq)
  if (h_out != nullptr)
    *h_out = (double)((long)res.cost / te.S) / (double)te.q;
  return tau;
}

// per-target cached least-blocking path, lazy asymmetric invalidation
// (design 6.2; DD_STRICT_INVAL=1 -> full-snapshot epoch-free mode)
struct PathCache {
  bool strict_inval = env_int("DD_STRICT_INVAL", 0) != 0;
  long recomputes = 0;  // diagnostics (DDStats.path_recomputes)
  long hits = 0;        // diagnostics (DDStats.path_cache_hits)
  struct Entry {
    std::vector<int> path;
    std::vector<uint8_t> occ_snapshot;
    std::vector<uint8_t> full_snapshot;  // strict mode only
    int src = -1;
    int dst = -1;  // T5: tau can reassign goals; a dst change is a MISS
  };
  std::unordered_map<int, Entry> by_target;

  const std::vector<int>& get(const DDGrid& g, int b, int src, int dst,
                              const std::vector<uint8_t>& occupied,
                              int exclude, Scratch& sc)
  {
    auto it = by_target.find(b);
    if (it != by_target.end() && it->second.dst == dst) {
      auto& e = it->second;
      if (e.src != src && e.path.size() >= 2 && e.path[1] == src) {
        e.path.erase(e.path.begin());
        e.occ_snapshot.erase(e.occ_snapshot.begin());
        e.src = src;
        if (!e.occ_snapshot.empty()) e.occ_snapshot[0] = 0;
      }
      if (e.src == src) {
        bool ok = true;
        if (strict_inval) {
          for (size_t c = 0; c < occupied.size() && ok; ++c) {
            const bool occ_now = (int)c != exclude && occupied[c];
            ok = occ_now == (e.full_snapshot[c] != 0);
          }
        } else {
          for (size_t i = 0; i < e.path.size() && ok; ++i) {
            const int c = e.path[i];
            const bool occ_now = c != exclude && occupied[c];
            ok = !(occ_now && e.occ_snapshot[i] == 0);
          }
        }
        if (ok) {
          ++hits;
          return e.path;
        }
      }
    }
    Entry e;
    e.src = src;
    e.dst = dst;
    ++recomputes;
    const std::vector<int>* prev_path = nullptr;
    if (it != by_target.end() && !it->second.path.empty() &&
        it->second.dst == dst)
      prev_path = &it->second.path;  // inertia baseline (same dst only)
    e.path =
        least_blocking_path(g, src, dst, occupied, exclude, sc, prev_path);
    e.occ_snapshot.resize(e.path.size());
    for (size_t i = 0; i < e.path.size(); ++i) {
      const int c = e.path[i];
      e.occ_snapshot[i] = (c != exclude && occupied[c]) ? 1 : 0;
    }
    if (strict_inval) {
      e.full_snapshot.resize(occupied.size());
      for (size_t c = 0; c < occupied.size(); ++c)
        e.full_snapshot[c] = ((int)c != exclude && occupied[c]) ? 1 : 0;
    }
    auto res = by_target.insert_or_assign(b, std::move(e));
    return res.first->second.path;
  }
};

// cross-deck wait-for graph (design 5.5, M9): one out-edge per robot;
// cycles are structural deadlocks -> targeted rho taboo.
inline std::vector<int> waitfor_cycles(const DDInstance& ins, const PhysConfig& s,
                                const CarrierGuidance& g,
                                LowerDist& lower_dist, Scratch& sc)
{
  const size_t R = ins.n_robots();
  std::vector<int> robot_at(ins.grid.size(), -1);
  for (size_t i = 0; i < R; ++i) robot_at[s.robots[i]] = (int)i;
  std::vector<int> assignee_of_cell(ins.grid.size(), -1);
  for (size_t i = 0; i < R; ++i)
    if (g.rho[i] >= 0 && g.free_goal[i] >= 0)
      assignee_of_cell[g.free_goal[i]] = (int)i;
  auto next_cell = [&](size_t i) -> int {
    const int k = s.kappa[i];
    if (k >= 0 && !g.target_park[k]) return g.target_next[k];
    if (k == KAPPA_ANON || (k >= 0 && g.target_park[k])) {
      const int park = g.parking_cell[i];
      if (park < 0 || park == s.robots[i]) return -1;
      int nb[4];
      const int n = ins.grid.neighbors(s.robots[i], nb);
      int best = -1, bd = INT_MAX / 2;
      for (int t = 0; t < n; ++t) {
        const int d = lower_dist.dist(park, nb[t]);
        if (d < bd) {
          bd = d;
          best = nb[t];
        }
      }
      return best;
    }
    const int goal = g.free_goal[i];
    if (goal < 0 || goal == s.robots[i]) return -1;
    int nb[4];
    const int n = ins.grid.neighbors(s.robots[i], nb);
    int best = -1, bd = INT_MAX / 2;
    for (int t = 0; t < n; ++t) {
      const int d = lower_dist.dist(goal, nb[t]);
      if (d < bd) {
        bd = d;
        best = nb[t];
      }
    }
    return best;
  };
  std::vector<int> out(R, -1);
  for (size_t i = 0; i < R; ++i) {
    const int nc = next_cell(i);
    if (nc < 0) continue;
    if (robot_at[nc] >= 0 && robot_at[nc] != (int)i) {
      out[i] = robot_at[nc];
      continue;
    }
    if (s.kappa[i] != KAPPA_FREE && sc.grounded[nc] != 0) {
      const int j = assignee_of_cell[nc];
      if (j >= 0 && j != (int)i) out[i] = j;
    }
  }
  std::vector<int> color(R, 0);
  std::vector<int> in_cycle;
  for (size_t st = 0; st < R; ++st) {
    if (color[st] != 0) continue;
    std::vector<int> stack;
    int cur = (int)st;
    while (cur >= 0 && color[cur] == 0) {
      color[cur] = 1;
      stack.push_back(cur);
      cur = out[cur];
    }
    if (cur >= 0 && color[cur] == 1) {
      auto itc = std::find(stack.begin(), stack.end(), cur);
      for (auto p = itc; p != stack.end(); ++p) in_cycle.push_back(*p);
    }
    for (int v : stack) color[v] = 2;
  }
  std::sort(in_cycle.begin(), in_cycle.end());
  return in_cycle;
}

// guidance construction (design 5.3/5.4a; ported: requests, park/yield,
// rho via the SHARED Hungarian + eta hysteresis, parking placement)
inline CarrierGuidance build_guidance(
    const DDInstance& ins, const PhysConfig& s, const std::vector<int>& tau,
    DDDistCache& upper_wall, LowerDist& lower_dist,
    PathCache& paths, Scratch& sc, const void* node_key,
    const std::vector<std::pair<int, int>>* taboo = nullptr,
    const CarrierGuidance* parent_guide = nullptr)
{
  CarrierGuidance g;
  g.tau = tau;
  const size_t R = ins.n_robots();
  fill_occupancy_if_needed(ins, s, sc, tau, node_key);
  std::fill(sc.protect.begin(), sc.protect.end(), 0);
  std::fill(sc.owner.begin(), sc.owner.end(), 0);
  g.target_next.assign(ins.n_targets(), -1);
  g.target_park.assign(ins.n_targets(), 0);

  // Active-target cap (throughput, ordering-only)
  const size_t n_unf_precount = [&] {
    size_t n = 0;
    std::vector<char> cf(ins.n_targets(), 0);
    for (int k : s.kappa)
      if (k >= 0) cf[k] = 1;
    for (size_t b = 0; b < ins.n_targets(); ++b)
      if (cf[b] || s.target_pos[b] != tau[b]) ++n;
    return n;
  }();
  const size_t ACTIVE_CAP = n_unf_precount > 256
                                ? (size_t)env_int("DD_ACTIVE_CAP", 64)
                                : n_unf_precount;
  std::vector<int> active_targets;
  {
    std::vector<char> carried_flag(ins.n_targets(), 0);
    for (int k : s.kappa)
      if (k >= 0) carried_flag[k] = 1;
    std::vector<int> rest;
    for (size_t b = 0; b < ins.n_targets(); ++b) {
      const bool done = !carried_flag[b] && s.target_pos[b] == tau[b];
      if (done) continue;
      if (carried_flag[b])
        active_targets.push_back((int)b);
      else
        rest.push_back((int)b);
    }
    std::stable_sort(rest.begin(), rest.end(), [&](int a, int b2) {
      const auto& da = upper_wall.to(tau[a]);
      const auto& db = upper_wall.to(tau[b2]);
      return da[s.target_pos[a]] < db[s.target_pos[b2]];
    });
    for (int b : rest) {
      if (active_targets.size() >= ACTIVE_CAP) break;
      active_targets.push_back(b);
    }
  }
  for (int b_int : active_targets) {
    const size_t b = (size_t)b_int;
    bool carried = std::any_of(s.kappa.begin(), s.kappa.end(),
                               [&](int k) { return k == (int)b; });
    const bool done = !carried && s.target_pos[b] == tau[b];
    if (done) continue;
    const auto& path =
        paths.get(ins.grid, (int)b, s.target_pos[b], tau[b],
                  sc.upper_path, s.target_pos[b], sc);
    if (path.size() >= 2) g.target_next[b] = path[1];
    for (int c : path) {
      sc.protect[c] = 1;
      if (sc.owner[c] == 0) sc.owner[c] = (int)b + 1;
    }

    const bool head_free = path.size() >= 2 && sc.grounded[path[1]] == 0;
    if (!carried && head_free) {
      CarrierRequest r;
      r.cell = s.target_pos[b];
      r.priority = 100;  // serve
      g.requests.push_back(r);
    }
    int emitted = 0;
    // movability bonus (design_final 5.4/D20): a blocker adjacent to an
    // upper-deck vacancy is the executable frontier — in the one-empty
    // regime it is the ONLY manipulable link of the chain.  Default OFF:
    // the singleton-parity gate (debug.md v4 section 0.2) is spec-level,
    // and at 50% fill the bonus reorders clears (dev ddmapd_h16 makespan
    // 81 -> 91).  Enable for puzzle-density goal-set runs / ablations.
    const bool frontier_on = env_int("DD_CLEAR_FRONTIER", 0) != 0;
    for (size_t pi = 1; pi < path.size() && emitted < CLEAR_CHAIN_K; ++pi) {
      const int cur = path[pi];
      const int gr__ = sc.grounded[cur];
      if (gr__ == -1 || (gr__ > 0 && gr__ - 1 != (int)b)) {
        CarrierRequest r;
        r.cell = cur;
        r.priority = 50 - emitted;  // clear, chain head higher
        if (frontier_on) {
          int nb__[4];
          const int nn__ = ins.grid.neighbors(cur, nb__);
          bool movable = false;
          for (int tt = 0; tt < nn__; ++tt)
            movable = movable || sc.upper[nb__[tt]] == 0;
          if (movable) r.priority += 25;
        }
        g.requests.push_back(r);
        ++emitted;
      }
    }
  }
  for (size_t b = 0; b < ins.n_targets(); ++b)
    sc.protect[tau[b]] = 1;  // only the tau-assigned cells (not the pool)

  // Park relation (design 5.4a): computed from X via the CURRENT masked
  // cached paths — a function of (X, D_b cache epoch) under the default
  // lazy invalidation; DD_STRICT_INVAL=1 is epoch-independent.  park[b]
  // iff goal_b lies on ANOTHER unfinished target's current path.
  std::vector<int> park_owner(ins.n_targets(), -1);  // build-local
  auto done_in_X = [&](int o) {
    if (s.target_pos[o] != tau[o]) return false;
    for (int k : s.kappa)
      if (k == o) return false;
    return true;
  };
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    const int ow__ = sc.owner[tau[b]];
    if (ow__ > 0 && ow__ - 1 != (int)b && !done_in_X(ow__ - 1)) {
      g.target_park[b] = 1;
      park_owner[b] = ow__ - 1;
    }
  }
  // carrier head-on yield (design 5.5): the one farther from its goal
  // parks (deterministic tie-break)
  if (env_int("DD_NO_YIELD", 0) == 0) {
    std::vector<int> carrier_of(ins.n_targets(), -1);
    for (size_t i = 0; i < R; ++i)
      if (s.kappa[i] >= 0) carrier_of[s.kappa[i]] = (int)i;
    for (size_t a = 0; a < ins.n_targets(); ++a) {
      if (carrier_of[a] < 0 || g.target_park[a]) continue;
      const int na = g.target_next[a];
      if (na < 0) continue;
      for (size_t b2 = a + 1; b2 < ins.n_targets(); ++b2) {
        if (carrier_of[b2] < 0 || g.target_park[b2]) continue;
        const int nb2 = g.target_next[b2];
        if (nb2 < 0) continue;
        if (na == s.target_pos[b2] && nb2 == s.target_pos[a]) {
          const auto& da = upper_wall.to(tau[a]);
          const auto& db = upper_wall.to(tau[b2]);
          const int ra = da[s.target_pos[a]], rb = db[s.target_pos[b2]];
          const size_t yield_b = (ra > rb || (ra == rb)) ? a : b2;
          g.target_park[yield_b] = 1;
          park_owner[yield_b] = (int)(yield_b == a ? b2 : a);
        }
      }
    }
  }
  // park cycle break: min-index member un-parks (deterministic)
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    if (!g.target_park[b]) continue;
    std::vector<int> chain;
    int cur = (int)b;
    bool cycle = false;
    for (size_t guard = 0; guard <= ins.n_targets(); ++guard) {
      chain.push_back(cur);
      const int nxt = g.target_park[cur] ? park_owner[cur] : -1;
      if (nxt < 0) break;
      if (nxt == (int)b) {
        cycle = true;
        break;
      }
      cur = nxt;
    }
    if (cycle) {
      const int drop = *std::min_element(chain.begin(), chain.end());
      g.target_park[drop] = 0;
      park_owner[drop] = -1;
    }
  }

  // rho matching: Hungarian (SHARED tapf implementation) within the
  // scale regime, greedy nearest otherwise; eta hysteresis on both.
  g.rho.assign(R, -1);
  g.free_goal.assign(R, -1);
  g.parking_cell.assign(R, -1);
  std::vector<int> req_order(g.requests.size());
  for (size_t i = 0; i < req_order.size(); ++i) req_order[i] = (int)i;
  std::stable_sort(req_order.begin(), req_order.end(), [&](int a, int b) {
    return g.requests[a].priority > g.requests[b].priority;
  });
  std::vector<bool> robot_used(R, false);
  int free_left = 0;
  for (size_t i = 0; i < R; ++i) {
    if (s.kappa[i] != KAPPA_FREE)
      robot_used[i] = true;
    else
      ++free_left;
  }
  const int ETA = env_int("DD_ETA", 2);
  const bool use_hyst = parent_guide != nullptr &&
                        parent_guide->free_goal.size() == R && ETA > 0;
  if (env_int("DD_RHO_HUNGARIAN", 1) != 0 &&
      ins.n_targets() <= (size_t)env_int("DD_RHO_HUNGARIAN_TGT", 256) &&
      free_left > 0 && !req_order.empty()) {
    std::vector<int> free_ids;
    for (size_t i = 0; i < R; ++i)
      if (!robot_used[i]) free_ids.push_back((int)i);
    std::vector<int> rows;
    for (int ri : req_order) {
      if ((int)rows.size() >= (int)free_ids.size()) break;
      rows.push_back(ri);
    }
    std::vector<std::vector<int>> cost(rows.size(),
                                       std::vector<int>(free_ids.size(), 0));
    for (size_t a = 0; a < rows.size(); ++a) {
      const auto& req = g.requests[rows[a]];
      for (size_t b2 = 0; b2 < free_ids.size(); ++b2) {
        const int i = free_ids[b2];
        bool banned = false;
        if (taboo)
          for (auto& [rb, cell] : *taboo)
            banned |= (rb == i && cell == req.cell);
        int dd = banned ? INT_MAX / 8 : lower_dist.dist(req.cell, s.robots[i]);
        if (!banned && use_hyst && parent_guide->rho[i] >= 0 &&
            parent_guide->free_goal[i] == req.cell)
          dd -= ETA;
        cost[a][b2] = dd;
      }
    }
    const auto row_to_col = tapf_hungarian_row_to_col(cost);
    for (size_t a = 0; a < rows.size(); ++a) {
      const int c = row_to_col[a];
      if (c < 0 || cost[a][c] >= INT_MAX / 8) continue;
      const int i = free_ids[c];
      robot_used[i] = true;
      --free_left;
      g.rho[i] = rows[a];
      g.free_goal[i] = g.requests[rows[a]].cell;
    }
  } else
    for (int ri : req_order) {
      if (free_left == 0) break;
      const auto& req = g.requests[ri];
      int best = -1, bestd = INT_MAX / 2;
      for (size_t i = 0; i < R; ++i) {
        if (robot_used[i]) continue;
        if (taboo) {
          bool banned = false;
          for (auto& [rb, cell] : *taboo)
            banned |= (rb == (int)i && cell == req.cell);
          if (banned) continue;
        }
        int dd = lower_dist.dist(req.cell, s.robots[i]);
        if (use_hyst && parent_guide->rho[i] >= 0 &&
            parent_guide->free_goal[i] == req.cell)
          dd -= ETA;
        if (dd < bestd) {
          bestd = dd;
          best = (int)i;
        }
      }
      if (best >= 0) {
        robot_used[best] = true;
        --free_left;
        g.rho[best] = ri;
        g.free_goal[best] = req.cell;
      }
    }

  // parking placement: nearest free off-path cell, goal-cell fallback;
  // DD_PLACE_ESCAPE selects the escape-degree tie-break (default off)
  std::vector<uint8_t> is_goal_cell(ins.grid.size(), 0);
  for (size_t b = 0; b < ins.n_targets(); ++b)
    is_goal_cell[tau[b]] = 1;  // tau-assigned cells only (design V3)
  for (size_t i = 0; i < R; ++i) {
    const bool anon_carrier = s.kappa[i] == KAPPA_ANON;
    const bool parked_target_carrier =
        s.kappa[i] >= 0 && g.target_park[s.kappa[i]];
    if (!anon_carrier && !parked_target_carrier) continue;
    int found = -1, fallback = -1;
    const bool esc_tb = env_int("DD_PLACE_ESCAPE", 0) != 0;
    const int rstart = s.robots[i];
    std::fill(sc.prev.begin(), sc.prev.end(), -1);
    std::deque<int> dq;
    dq.push_back(rstart);
    sc.prev[rstart] = 0;
    int nb[4];
    int found_depth = -1, found_escape = -1;
    while (!dq.empty()) {
      int u = dq.front();
      dq.pop_front();
      const int du = sc.prev[u];
      if (found >= 0 && du > found_depth) break;
      if (u != rstart && !sc.upper[u]) {
        if (!sc.protect[u]) {
          if (!esc_tb) {
            found = u;
            break;
          }
          int escd = 0;
          const int n2 = ins.grid.neighbors(u, nb);
          for (int k = 0; k < n2; ++k)
            if (!sc.upper[nb[k]]) ++escd;
          if (found < 0 || escd > found_escape) {
            found = u;
            found_depth = du;
            found_escape = escd;
          }
        } else if (fallback < 0 && !is_goal_cell[u]) {
          fallback = u;
        }
      }
      const int n = ins.grid.neighbors(u, nb);
      for (int k = 0; k < n; ++k)
        if (sc.prev[nb[k]] < 0) {
          sc.prev[nb[k]] = du + 1;
          dq.push_back(nb[k]);
        }
    }
    g.parking_cell[i] = found >= 0 ? found : fallback;
  }
  return g;
}

}  // namespace carrier_detail
