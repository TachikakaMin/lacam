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
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <queue>
#include <stdexcept>
#include <string>
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

// solver-objective weight loader (design 5.7): ONE parser for the planner
// g-weights and the adapter reporting weights (they must always agree).
// W needs fields alpha/beta/gamma/delta defaulting to 1.  Values must be
// finite and non-negative: negative weights break the non-negative
// edge-cost/admissible-LB assumptions (and would bypass the matching
// encoding's upper-bound overflow check); NaN/inf poison comparisons.
template <typename W>
inline void load_solver_weights(W& w)
{
  auto read = [](const char* key, double& out) {
    const char* raw = std::getenv(key);
    if (raw == nullptr) return;
    char* end = nullptr;
    const double v = std::strtod(raw, &end);
    const bool converted = end != nullptr && end != raw;
    if (converted)
      while (*end == ' ') ++end;  // tolerate trailing blanks only
    const bool consumed = converted && *end == '\0';
    // Costs are accumulated in double and some assignment paths quantize
    // them into signed integer keys.  Capping user-provided coefficients
    // keeps both representations finite under any practical plan length.
    constexpr double MAX_SAFE_SOLVER_WEIGHT = 1e6;
    if (!consumed || !std::isfinite(v) || v < 0 ||
        v > MAX_SAFE_SOLVER_WEIGHT)
      throw std::invalid_argument(
          std::string(key) +
          ": objective weight must be finite, non-negative, and <= 1e6, "
          "got '" + raw + "'");
    out = v;
  };
  read("DD_ALPHA", w.alpha);
  read("DD_BETA", w.beta);
  read("DD_GAMMA", w.gamma);
  read("DD_DELTA", w.delta);
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
constexpr int ASSIGNMENT_EXACT_LIMIT = 256;
constexpr int ACTIVE_TARGET_LIMIT = 256;
constexpr int ACTIVE_TARGET_CAP = 64;
constexpr int ASSIGNMENT_HYSTERESIS = 2;
constexpr int HEAD_DROP_SCAN_CAP = 64;  // frontier head drop-hint BFS cap

// Stable TaskId (design_final v3.0 §3): identity = (shelf, from, root).
// `to` is deliberately EXCLUDED: a clear task keeps its identity while
// the frontier compiler refines the drop cell, so rho hysteresis
// survives vacancy churn.  Anonymous shelves pass shelf_target = -1 and
// are distinguished by `from` (cell-equivalence-class semantics).
inline uint64_t task_ident_hash(int shelf_target, int from,
                                int root_target, int root_goal)
{
  auto mix = [](uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  };
  uint64_t h = mix((uint64_t)(uint32_t)(shelf_target + 2));
  h ^= mix(((uint64_t)1 << 40) ^ (uint64_t)(uint32_t)from);
  h ^= mix(((uint64_t)2 << 40) ^ (uint64_t)(uint32_t)(root_target + 1));
  h ^= mix(((uint64_t)3 << 40) ^ (uint64_t)(uint32_t)root_goal);
  return h == 0 ? 1 : h;  // 0 stays "invalid"
}

inline bool target_on_eligible_goal(const DDInstance& ins,
                                    const PhysConfig& s, size_t b)
{
  const auto& goals = ins.target_goal_sets[b];
  return std::binary_search(goals.begin(), goals.end(), s.target_pos[b]);
}

// grid-sized scratch buffers reused across nodes (single-threaded)
struct Scratch {
  std::vector<uint8_t> upper;
  std::vector<uint8_t> protect;
  std::vector<int> grounded;  // 0 none; -1 anon; b+1 grounded target b
  std::vector<int> owner;
  std::vector<long long> dist;
  std::vector<int> prev;
  // Path occupancy view. Completed targets remain ordinary reversible
  // blockers; task-level stability is handled by tau, not by path costs.
  std::vector<uint8_t> upper_path;
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
  constexpr long long PATH_INF = LLONG_MAX / 4;
  std::fill(sc.dist.begin(), sc.dist.end(), PATH_INF);
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
  using QE = std::pair<long long, int>;
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
      const uint8_t occ = v != exclude ? occupied[v] : 0;
      const int base = 1 + (occ ? LAMBDA_BLK : 0);
      long long w = (long long)base * scale;
      if (use_prev && on_prev[v]) w -= 1;
      if (dist[v] > d + w) {
        dist[v] = d + w;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }
  if (dist[dst] >= PATH_INF) return {};
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
  long eta_b = ASSIGNMENT_HYSTERESIS;
  long S = 1;  // hysteresis scale: eta_b * n + 1 (sum of pens < S)
  long q = 1;  // LB quantization (1 for integral weights)
};

inline void tau_init(TauEngine& te, const DDInstance& ins, double alpha,
                     double gamma)
{
  te.inited = true;
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
    const std::vector<std::pair<int, int>>* taboo, double* h_out,
    bool preserve_parent = false,
    // v3.0 §5.1 execution price: per-row (goal cell, price in lb units)
    // applied ONLY to guidance matchings — the admissible h always comes
    // from the price-free unrestricted matching (invariant 18).
    const std::vector<std::pair<int, double>>* price = nullptr)
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
  std::vector<char> settled(n, 0);
  for (size_t b = 0; b < n; ++b)
    settled[b] = !carried[b] && target_on_eligible_goal(ins, s, b);
  // per-pair admissible lower bound (design_final 4.3 Lemma 1)
  auto lb = [&](size_t b, int g) -> double {
    double v = alpha * upper_wall.to(g)[s.target_pos[b]];
    if (carried[b])
      v += gamma;  // >= 1 drop
    else if (s.target_pos[b] != g)
      v += 2 * gamma;  // >= 1 lift + 1 drop
    return v;
  };
  auto relaxed_h = [&]() {
    double h = 0;
    for (size_t b = 0; b < n; ++b) {
      double best = -1;
      for (const int g : ins.target_goal_sets[b]) {
        const double v = lb(b, g);
        if (best < 0 || v < best) best = v;
      }
      h += best;
    }
    return h;
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

  // A shelf-goal assignment is a task-level commitment, not a primitive
  // motion preference. Between task boundaries the physical LB matrix may
  // change as a carried shelf moves, but changing its destination every
  // timestep disconnects robot execution from shelf planning. Reuse the
  // parent assignment and retain an admissible row-relaxed lower bound;
  // callers explicitly disable this at drop/reguidance boundaries.
  if (preserve_parent && parent_tau != nullptr &&
      parent_tau->size() == n) {
    bool valid = true;
    for (size_t b = 0; b < n; ++b) {
      valid &= std::binary_search(ins.target_goal_sets[b].begin(),
                                  ins.target_goal_sets[b].end(),
                                  (*parent_tau)[b]);
      valid &= !settled[b] || (*parent_tau)[b] == s.target_pos[b];
    }
    if (valid) {
      tau = *parent_tau;
      if (h_out != nullptr) *h_out = relaxed_h();
      return tau;
    }
  }

  auto banned = [&](size_t b, int g) {
    if (taboo == nullptr) return false;
    if (ins.target_goal_sets[b].size() <= 1) return false;  // exempt
    if (carried[b] || settled[b]) return false;
    for (const auto& [tb, tg] : *taboo)
      if (tb == (int)b && tg == g) return true;
    return false;
  };
  if ((int)n > ASSIGNMENT_EXACT_LIMIT) {
    // scale-regime relaxation (design_final 4.3): row-wise nearest
    // eligible goal; h ignores injectivity (still admissible, weaker).
    // Hysteresis: stick to the parent goal within an eta_B slack.
    double h = 0;
    for (size_t b = 0; b < n; ++b) {
      const auto& set = ins.target_goal_sets[b];
      double h_best = -1;
      for (const int g : set) {
        const double v = lb(b, g);
        if (h_best < 0 || v < h_best) h_best = v;
      }
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
      if (settled[b]) {
        best_g = s.target_pos[b];
      } else if (carried[b] && parent_tau != nullptr &&
          std::binary_search(set.begin(), set.end(), (*parent_tau)[b])) {
        best_g = (*parent_tau)[b];
      } else if (parent_tau != nullptr) {
        const int pg = (*parent_tau)[b];
        if (pg >= 0 && pg != best_g && !banned(b, pg) &&
            std::binary_search(set.begin(), set.end(), pg) &&
            lb(b, pg) <= best + (double)te.eta_b)
          best_g = pg;
      }
      tau[b] = best_g;
      h += h_best;  // unrestricted row minimum (admissible)
    }
    if (h_out != nullptr) *h_out = h;
    return tau;
  }

  // exact regime: one Hungarian, lexicographic (LB, hysteresis, tie)
  const long INF = kTapfAssignmentInfCost;
  const long enc_cap =
      std::min<long>(INF / 2 - 1, (long)INT_MAX / (long)(n + 1));
  std::vector<int> locked_col(n, -1);
  std::vector<int> locked_owner(te.cols.size(), -1);
  std::vector<uint8_t> lock_kind(n, 0);  // 1 carried, 2 settled
  // Prefer terminal placement over historical intent: a grounded target on
  // any eligible goal owns that cell whenever the partial matching extends.
  // An in-flight target reroutes on such a commitment conflict.
  for (size_t b = 0; b < n; ++b) {
    if (!settled[b]) continue;
    const auto it =
        std::find(te.cols.begin(), te.cols.end(), s.target_pos[b]);
    if (it == te.cols.end()) continue;
    const int col = static_cast<int>(it - te.cols.begin());
    locked_col[b] = col;
    locked_owner[col] = (int)b;
    lock_kind[b] = 2;
  }
  if (parent_tau != nullptr && parent_tau->size() == n) {
    for (size_t b = 0; b < n; ++b) {
      if (!carried[b] || settled[b]) continue;
      const auto it = std::find(te.cols.begin(), te.cols.end(),
                                (*parent_tau)[b]);
      if (it == te.cols.end()) continue;
      const int col = static_cast<int>(it - te.cols.begin());
      if (!te.elig[b][col] ||
          locked_owner[col] >= 0)
        continue;
      locked_col[b] = col;
      locked_owner[col] = (int)b;
      lock_kind[b] = 1;
    }
  }
  auto make_cost = [&](bool use_taboo, bool use_locks, bool use_price) {
    return [&, use_taboo, use_locks, use_price](int row, int col) -> int {
      if (!te.elig[row][col]) return (int)INF;
      if (use_locks &&
          ((locked_col[row] >= 0 && locked_col[row] != col) ||
           (locked_owner[col] >= 0 && locked_owner[col] != row)))
        return (int)INF;
      const int g = te.cols[col];
      if (use_taboo && banned((size_t)row, g)) return (int)INF;
      const double v = lb((size_t)row, g);
      long lbq = (long)std::floor(v * (double)te.q + 1e-9);
      if (use_price && price != nullptr && (*price)[row].first == g) {
        long pq = (long)std::floor((*price)[row].second * (double)te.q);
        pq = std::min(pq, (long)1 << 20);  // encoding safety clamp
        lbq += pq;
      }
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
  auto unrestricted = te.state.solve_full(make_cost(false, false, false));
  if (!unrestricted.feasible)
    throw std::logic_error(
        "solve_tau: infeasible matching (covering checked at load)");
  if (h_out != nullptr)
    *h_out =
        (double)((long)unrestricted.cost / te.S) / (double)te.q;

  auto rebuild_locked_owner = [&]() {
    std::fill(locked_owner.begin(), locked_owner.end(), -1);
    for (size_t b = 0; b < n; ++b)
      if (locked_col[b] >= 0) locked_owner[locked_col[b]] = (int)b;
  };
  auto solve_with_locks = [&]() {
    auto candidate =
        te.state.solve_full(make_cost(taboo != nullptr, true, true));
    if (!candidate.feasible && taboo != nullptr)
      candidate = te.state.solve_full(make_cost(false, true, true));
    return candidate;
  };
  const bool have_locks =
      std::any_of(locked_col.begin(), locked_col.end(),
                  [](int col) { return col >= 0; });
  auto res = (!have_locks && taboo == nullptr && price == nullptr)
                 ? unrestricted
                 : solve_with_locks();
  if (!res.feasible) {
    // A set of individually valid carried commitments need not extend to a
    // complete matching. Settled rows have priority, so release carried
    // locks first and retain every settled placement when feasible.
    for (size_t b = 0; b < n; ++b)
      if (lock_kind[b] == 1) {
        locked_col[b] = -1;
        lock_kind[b] = 0;
      }
    rebuild_locked_owner();
    res = solve_with_locks();
  }
  if (!res.feasible) {
    // A currently settled placement can also be non-extendable (for
    // example, a flexible target occupies another target's singleton
    // goal). Keep the settled rows retained by the unrestricted matching;
    // that matching witnesses feasibility of this reduced lock set.
    for (size_t b = 0; b < n; ++b) {
      const int col = unrestricted.agent_to_task[b];
      const bool retained =
          settled[b] && col >= 0 && col < (int)te.cols.size() &&
          te.cols[col] == s.target_pos[b];
      locked_col[b] = retained ? col : -1;
      lock_kind[b] = retained ? 2 : 0;
    }
    rebuild_locked_owner();
    res = solve_with_locks();
  }
  if (!res.feasible)
    throw std::logic_error(
        "solve_tau: infeasible matching (covering checked at load)");
  for (size_t b = 0; b < n; ++b) tau[b] = te.cols[res.agent_to_task[b]];
  return tau;
}

// per-target cached least-blocking path, lazy asymmetric invalidation
// (design 6.2).  Strict snapshots remain available to the cache-purity
// test probe, but production uses the cheaper path-local invalidation.
struct PathCache {
  bool strict_inval;
  long recomputes = 0;  // diagnostics (DDStats.path_recomputes)
  long hits = 0;        // diagnostics (DDStats.path_cache_hits)
  explicit PathCache(bool strict = false) : strict_inval(strict) {}
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
            const uint8_t occ_now =
                (int)c != exclude ? occupied[c] : 0;
            ok = occ_now == e.full_snapshot[c];
          }
        } else {
          for (size_t i = 0; i < e.path.size() && ok; ++i) {
            const int c = e.path[i];
            const uint8_t occ_now = c != exclude ? occupied[c] : 0;
            ok = occ_now <= e.occ_snapshot[i];
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
      e.occ_snapshot[i] = c != exclude ? occupied[c] : 0;
    }
    if (strict_inval) {
      e.full_snapshot.resize(occupied.size());
      for (size_t c = 0; c < occupied.size(); ++c)
        e.full_snapshot[c] = (int)c != exclude ? occupied[c] : 0;
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

// v3.0 §5.1: one light execution-price feedback round over a BUILT
// guidance.  A multi-goal root is priced when the realization of its
// CURRENT frontier pickup (assigned robot's distance, else nearest free
// robot, else a board-diameter penalty when nobody can serve) exceeds
// its lb slack to the best alternative goal plus hysteresis.  Prices
// feed ONLY guidance matchings (invariant 18); carried/settled/singleton
// rows and goal-blind serve frontiers are exempt.  Shared by the planner
// (attach_carrier_guidance) and the guidance probes.
inline bool compute_execution_prices(
    const DDInstance& ins, const PhysConfig& s,
    const CarrierGuidance& guide, const std::vector<int>& tau,
    DDDistCache& upper_wall, LowerDist& lower_dist, double alpha,
    double beta, std::vector<std::pair<int, double>>& price)
{
  price.assign(ins.n_targets(), {-1, 0.0});
  if (guide.tasks.empty()) return false;
  bool any = false;
  std::vector<int> head(ins.n_targets(), -1);
  for (size_t k = 0; k < guide.tasks.size(); ++k) {
    const auto& t = guide.tasks[k];
    if (t.root_target < 0 || head[t.root_target] >= 0) continue;
    // the priced frontier is the first task that is NOT the goal-blind
    // serve pickup: the chain-head clear/ready move (emit order = per
    // root priority order)
    if (t.root_target < (int)ins.n_targets() &&
        t.from != s.target_pos[t.root_target])
      head[t.root_target] = (int)k;
  }
  std::vector<int> server(guide.tasks.size(), -1);
  for (size_t i = 0; i < ins.n_robots(); ++i)
    if (i < guide.rho_task.size() && guide.rho_task[i] >= 0 &&
        guide.rho_task[i] < (int)guide.tasks.size())
      server[guide.rho_task[i]] = (int)i;
  std::vector<char> carried(ins.n_targets(), 0);
  for (const int k : s.kappa)
    if (k >= 0) carried[k] = 1;
  const long far_realization = 2L * (ins.grid.height + ins.grid.width);
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    if (head[b] < 0 || carried[b]) continue;
    if (ins.target_goal_sets[b].size() <= 1) continue;  // singleton
    if (s.target_pos[b] == tau[b]) continue;            // settled
    const auto& t = guide.tasks[head[b]];
    if (t.from == s.target_pos[b]) continue;  // serve: goal-blind pickup
    // Delta pricing (anti-oscillation): the price is the EXCESS of
    // realizing the CURRENT frontier over the goal-independent shelf
    // approach (the optimistic baseline any alternative goal shares).
    // One-sided absolute prices flap on dense boards: after a flip the
    // reverse pair gets priced by the same magnitude and hysteresis is
    // tie-strength only.  The delta form is ~antisymmetric, so a flip
    // does not immediately price itself back.
    long r_cur = -1;
    long r_alt = -1;
    if (server[head[b]] >= 0) {
      const int rc = s.robots[server[head[b]]];
      r_cur = lower_dist.dist(t.from, rc);
      r_alt = lower_dist.dist(s.target_pos[b], rc);
    } else {
      for (size_t i = 0; i < ins.n_robots(); ++i)
        if (s.kappa[i] == KAPPA_FREE) {
          const long dc = lower_dist.dist(t.from, s.robots[i]);
          const long da = lower_dist.dist(s.target_pos[b], s.robots[i]);
          r_cur = r_cur < 0 ? dc : std::min(r_cur, dc);
          r_alt = r_alt < 0 ? da : std::min(r_alt, da);
        }
      if (r_cur < 0) {  // contested: nobody can serve this root at all
        r_cur = far_realization;
        r_alt = 0;
      }
    }
    const long excess = r_cur - r_alt;
    if (excess <= 0) continue;
    // R4(c): robot walking is priced in BETA units (free-move weight);
    // the lb gap and hysteresis threshold live in the same objective
    // scale, so the comparison is dimensionally consistent under
    // non-unit weights (beta = 0 => walking is free => no price).
    const double priced_excess = beta * (double)excess;
    if (priced_excess <= 0) continue;
    const double lb_cur = alpha * upper_wall.to(tau[b])[s.target_pos[b]];
    double lb_alt = -1;
    for (const int gg : ins.target_goal_sets[b]) {
      if (gg == tau[b]) continue;
      const double v = alpha * upper_wall.to(gg)[s.target_pos[b]];
      if (lb_alt < 0 || v < lb_alt) lb_alt = v;
    }
    if (lb_alt < 0) continue;
    if (priced_excess > (lb_alt - lb_cur) + ASSIGNMENT_HYSTERESIS) {
      price[b] = {tau[b], priced_excess};
      any = true;
    }
  }
  return any;
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
  const size_t ACTIVE_CAP =
      n_unf_precount > ACTIVE_TARGET_LIMIT ? ACTIVE_TARGET_CAP
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
  // one-empty regime detection (v3.0 §4.1, new.md §2): the ready-task
  // replacement applies to the literal sliding-puzzle case (exactly one
  // free upper cell).  On denser boards a surrounded blocker is still
  // movable through loaded rotations under following semantics
  // (report.md: hover shuffling is load-bearing), so the plain clear
  // task must survive there.
  int n_vacancies = 0;
  for (size_t c = 0; c < sc.upper.size(); ++c)
    n_vacancies += sc.upper[c] == 0 && !ins.grid.is_wall((int)c) ? 1 : 0;
  // S3 (2026-09-02 round 3): ONE physical shelf move = ONE pool task.
  // Cross-root duplicates (two objectives compiling the same pickup)
  // waste rho slots; keep the higher-priority version (serve > clear),
  // first emission wins ties.  `from` alone identifies the shelf (a
  // cell holds at most one grounded shelf).
  auto pool_find = [&](int from) -> int {
    for (size_t k = 0; k < g.tasks.size(); ++k)
      if (g.tasks[k].from == from) return (int)k;
    return -1;
  };
  auto emit_task = [&](const ManipulationTask& mt) -> bool {
    const int k = pool_find(mt.from);
    if (k < 0) {
      g.tasks.push_back(mt);
      g.requests.push_back(CarrierRequest{mt.from, mt.priority});
      return true;
    }
    if (mt.priority > g.tasks[k].priority) {
      g.tasks[k] = mt;
      g.requests[k] = CarrierRequest{mt.from, mt.priority};
      return true;
    }
    return false;  // an equal-or-better task already commands this shelf
  };
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
      // v3.0 §3: the request is the pickup projection of a serve task
      // MoveShelf(b, pos -> assigned goal, root = b -> tau[b]).
      ManipulationTask mt;
      mt.shelf_target = (int)b;
      mt.from = s.target_pos[b];
      mt.to = tau[b];
      mt.root_target = (int)b;
      mt.root_goal = tau[b];
      mt.priority = 100;  // serve
      mt.depth = 0;
      mt.id = task_ident_hash((int)b, mt.from, (int)b, tau[b]);
      emit_task(mt);
    }
    int emitted = 0;
    // frontier compiler (v3.0 §4.1): each clear candidate is refined into
    // an EXECUTABLE task.  A blocker whose carry cannot even start (no
    // adjacent free upper cell) is replaced by the READY task that moves
    // the first vacancy-adjacent shelf of the routing chain INTO the
    // vacancy (one-empty / 15-puzzle semantics); a feasible chain head
    // gets its compiler-chosen drop cell.  Ordering-only, like requests.
    // Dedupe is POOL-wide via emit_task (S3): chains converging on the
    // same shelf — within one root or across roots — emit it once.
    for (size_t pi = 1; pi < path.size() && emitted < CLEAR_CHAIN_K; ++pi) {
      const int cur = path[pi];
      const int gr__ = sc.grounded[cur];
      if (!(gr__ == -1 || (gr__ > 0 && gr__ - 1 != (int)b))) continue;
      int from = cur;
      int to = -1;
      int depth_extra = 0;
      int nb[4];
      const int n_adj = ins.grid.neighbors(cur, nb);
      bool carry_can_start = false;
      for (int t = 0; t < n_adj; ++t)
        carry_can_start |= sc.upper[nb[t]] == 0;
      if (!carry_can_start && n_vacancies == 1) {
        // vacancy routing: BFS from the blocker to the nearest free
        // upper cell e; the parent of e on that path is the
        // vacancy-adjacent shelf that can actually move (into e).
        std::fill(sc.prev.begin(), sc.prev.end(), -1);
        std::deque<int> dq;
        dq.push_back(cur);
        sc.prev[cur] = cur;
        int e = -1;
        while (!dq.empty() && e < 0) {
          const int u = dq.front();
          dq.pop_front();
          const int m = ins.grid.neighbors(u, nb);
          for (int t = 0; t < m; ++t) {
            const int v = nb[t];
            if (sc.prev[v] >= 0) continue;
            sc.prev[v] = u;
            if (sc.upper[v] == 0) {
              e = v;
              break;
            }
            dq.push_back(v);
          }
        }
        if (e < 0) {
          // ZERO vacancies anywhere: hover-lift shuffling is the only
          // remaining mechanism (rotations of loaded robots under
          // following semantics).  Emit the plain clear task — the
          // LIFT_GATE lesson (report.md §7/§10): suppressing these
          // starves dense zero/one-empty instances.
          from = cur;
          to = -1;
        } else {
          from = sc.prev[e];  // first vacancy-adjacent shelf on the chain
          to = e;             // ready task drops INTO the vacancy
          depth_extra = 1;
        }
      } else if (carry_can_start && emitted == 0) {
        // feasible chain head: nearest free upper cell off the protected
        // corridor as the compiler-chosen drop (-1 when none qualifies:
        // the carrier's parking fallback remains authoritative).  The
        // scan is capped (HEAD_DROP_SCAN_CAP): on protect-saturated
        // dense boards no qualifying cell may exist and an unbounded
        // per-head BFS is pure guidance overhead (cost regression on
        // h10w10_e3, gate 2026-09-01).  Unstartable heads skip the hint
        // entirely — their drop cell is meaningless until they can move.
        std::fill(sc.prev.begin(), sc.prev.end(), -1);
        std::deque<int> dq;
        dq.push_back(cur);
        sc.prev[cur] = cur;
        int scanned = 0;
        while (!dq.empty() && to < 0 && scanned < HEAD_DROP_SCAN_CAP) {
          const int u = dq.front();
          dq.pop_front();
          ++scanned;
          const int m = ins.grid.neighbors(u, nb);
          for (int t = 0; t < m; ++t) {
            const int v = nb[t];
            if (sc.prev[v] >= 0) continue;
            sc.prev[v] = u;
            if (sc.upper[v] == 0 && !sc.protect[v]) {
              to = v;
              break;
            }
            dq.push_back(v);
          }
        }
      }
      ManipulationTask mt;
      mt.shelf_target = sc.grounded[from] > 0 ? sc.grounded[from] - 1 : -1;
      mt.from = from;
      mt.to = to;
      mt.to_committed = depth_extra == 1;  // one-empty ready: to = vacancy
      mt.root_target = (int)b;
      mt.root_goal = tau[b];
      mt.priority = 50 - emitted;  // clear, chain head higher
      mt.depth = emitted + 1 + depth_extra;
      mt.id = task_ident_hash(mt.shelf_target, from, (int)b, tau[b]);
      if (emit_task(mt)) ++emitted;
    }
  }
  for (size_t b = 0; b < ins.n_targets(); ++b)
    sc.protect[tau[b]] = 1;  // only the tau-assigned cells (not the pool)

  // Park relation (design 5.4a): computed from X via the CURRENT masked
  // cached paths — a function of (X, D_b cache epoch) under the default
  // lazy invalidation; strict test-probe mode is epoch-independent. park[b]
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
  // v3.0 §5: rho binds the TASK (stable TaskId); the request index and
  // free_goal are derived views (request k is task k's pickup projection).
  g.rho.assign(R, -1);
  g.rho_task.assign(R, -1);
  g.free_goal.assign(R, -1);
  g.parking_cell.assign(R, -1);
  std::vector<int> req_order(g.requests.size());
  for (size_t i = 0; i < req_order.size(); ++i) req_order[i] = (int)i;
  std::stable_sort(req_order.begin(), req_order.end(), [&](int a, int b) {
    if (g.requests[a].priority != g.requests[b].priority)
      return g.requests[a].priority > g.requests[b].priority;
    // R2(c): within equal priority, shallower dependency depth first
    // (a directly startable head beats a one-empty ready hop)
    return g.tasks[a].depth < g.tasks[b].depth;
  });
  std::vector<bool> robot_used(R, false);
  int free_left = 0;
  for (size_t i = 0; i < R; ++i) {
    if (s.kappa[i] != KAPPA_FREE)
      robot_used[i] = true;
    else
      ++free_left;
  }
  const int ETA = ASSIGNMENT_HYSTERESIS;
  const bool use_hyst = parent_guide != nullptr &&
                        parent_guide->free_goal.size() == R && ETA > 0;
  // task-switch hysteresis by TASK IDENTITY (v3.0 §5): a robot keeps its
  // eta discount only toward the SAME task (shelf, from, root), not any
  // task that happens to share a pickup cell.  Parents without a task
  // pool (B1, legacy probes) fall back to the historical cell view.
  const auto same_task_as_parent = [&](size_t i, int req_idx) -> bool {
    if (!use_hyst || parent_guide->rho[i] < 0) return false;
    if (parent_guide->rho_task.size() == R &&
        parent_guide->rho_task[i] >= 0 &&
        parent_guide->rho_task[i] < (int)parent_guide->tasks.size() &&
        req_idx < (int)g.tasks.size())
      return parent_guide->tasks[parent_guide->rho_task[i]].id ==
             g.tasks[req_idx].id;
    return parent_guide->free_goal[i] == g.requests[req_idx].cell;
  };
  if (ins.n_targets() <= (size_t)ASSIGNMENT_EXACT_LIMIT &&
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
        if (!banned && same_task_as_parent(i, rows[a])) dd -= ETA;
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
      g.rho_task[i] = rows[a];
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
        if (same_task_as_parent(i, ri)) dd -= ETA;
        if (dd < bestd) {
          bestd = dd;
          best = (int)i;
        }
      }
      if (best >= 0) {
        robot_used[best] = true;
        --free_left;
        g.rho[best] = ri;
        g.rho_task[best] = ri;
        g.free_goal[best] = req.cell;
      }
    }

  // v3.0 §6 custody: a loaded-ANON robot keeps executing the task it was
  // bound to when it lifted (same TaskId through approach/Lift/carry/
  // Drop).  Copied metadata only — never part of the search key.
  // R2 (2026-09-02): the custody task's drop cell is REFINED
  // position-aware every node (same-task re-targeting; a stale
  // compile-time `to` was falsified on dense boards): in the one-empty
  // regime the drop IS the current vacancy; otherwise the nearest free
  // upper cell from the carrier with the parking preference order
  // (non-protected first, protected non-goal fallback).  funcPIBT
  // derives the carry waypoint from this field (invariant 23).
  g.custody.assign(R, ManipulationTask{});
  if (parent_guide != nullptr) {
    for (size_t i = 0; i < R; ++i) {
      // S1 (2026-09-02 round 3): custody covers BOTH anonymous and
      // LABELED carried shelves.  A labeled target lifted for a
      // committed ready task must execute that task's drop (its own tau
      // resumes afterwards); previously custody skipped kappa >= 0 and
      // the carrier shuttled toward tau forever (verified livelock).
      if (s.kappa[i] == KAPPA_FREE) continue;
      if (parent_guide->custody.size() == R &&
          parent_guide->custody[i].id != 0) {
        g.custody[i] = parent_guide->custody[i];  // carry continues
      } else if (parent_guide->rho_task.size() == R &&
                 parent_guide->rho_task[i] >= 0 &&
                 parent_guide->rho_task[i] <
                     (int)parent_guide->tasks.size()) {
        const auto& t =
            parent_guide->tasks[parent_guide->rho_task[i]];
        // fresh Lift: the robot stands on its bound task's pickup cell,
        // and for a labeled carry the task must name THAT shelf
        const bool shelf_matches =
            s.kappa[i] == KAPPA_ANON || t.shelf_target == s.kappa[i];
        if (t.from == s.robots[i] && shelf_matches) g.custody[i] = t;
      }
      if (g.custody[i].id == 0 || !g.custody[i].to_committed) continue;
      // Committed drop (one-empty ready): keep the destination while it
      // stays droppable — the carrier's OWN cell counts (its upper
      // "occupancy" is the carried shelf itself), which is exactly the
      // stand-on-the-vacancy-and-drop endgame.  Recompute only when the
      // destination was invalidated meanwhile; the new target must be a
      // REAL free upper cell (never the carrier's own cell — that made
      // the origin look like "the vacancy" and ping-ponged the carry).
      // Un-committed custody defers the drop to the per-node parking
      // choice (task semantics: carrier-chosen), which is what keeps
      // dense boards healthy (d50 bound test).
      const int prev_to = g.custody[i].to;
      const bool prev_ok =
          prev_to >= 0 &&
          (sc.upper[prev_to] == 0 || prev_to == s.robots[i]);
      if (prev_ok) continue;
      int vac = -1;
      for (size_t c = 0; c < sc.upper.size(); ++c)
        if (sc.upper[c] == 0 && !ins.grid.is_wall((int)c)) {
          vac = (int)c;
          break;
        }
      g.custody[i].to = vac;  // -1 when nothing droppable: parking fallback
    }
  }

  // parking placement: nearest free off-path cell, goal-cell fallback
  std::vector<uint8_t> is_goal_cell(ins.grid.size(), 0);
  for (size_t b = 0; b < ins.n_targets(); ++b)
    is_goal_cell[tau[b]] = 1;  // tau-assigned cells only (design V3)
  for (size_t i = 0; i < R; ++i) {
    const bool anon_carrier = s.kappa[i] == KAPPA_ANON;
    const bool parked_target_carrier =
        s.kappa[i] >= 0 && g.target_park[s.kappa[i]];
    if (!anon_carrier && !parked_target_carrier) continue;
    int found = -1, fallback = -1;
    const int rstart = s.robots[i];
    std::fill(sc.prev.begin(), sc.prev.end(), -1);
    std::deque<int> dq;
    dq.push_back(rstart);
    sc.prev[rstart] = 0;
    int nb[4];
    while (!dq.empty()) {
      int u = dq.front();
      dq.pop_front();
      const int du = sc.prev[u];
      if (u != rstart && !sc.upper[u]) {
        if (!sc.protect[u]) {
          found = u;
          break;
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
