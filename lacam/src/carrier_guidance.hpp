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
#include <array>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
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

inline UpperSignature make_upper_signature(const PhysConfig& s)
{
  UpperSignature out;
  out.target_pos = s.target_pos;
  out.anon_pos = s.anon_occ;
  for (size_t i = 0; i < s.kappa.size(); ++i)
    if (s.kappa[i] == KAPPA_ANON) out.anon_pos.push_back(s.robots[i]);
  std::sort(out.anon_pos.begin(), out.anon_pos.end());
  return out;
}

inline size_t upper_vacancy_count(
    const DDInstance& ins, const UpperSignature& upper)
{
  size_t storage_cells = 0;
  for (int cell = 0; cell < ins.grid.size(); ++cell)
    storage_cells += ins.can_store_shelf(cell);
  const size_t shelf_count =
      upper.target_pos.size() + upper.anon_pos.size();
  if (shelf_count > storage_cells)
    throw std::logic_error(
        "upper_vacancy_count: shelves exceed storage cells");
  return storage_cells - shelf_count;
}

inline bool zero_storage_vacancy_no_ready(
    const DDInstance& ins, const UpperSignature& upper,
    size_t ready_task_count, size_t graph_task_count)
{
  return ready_task_count == 0 && graph_task_count > 0 &&
         upper_vacancy_count(ins, upper) == 0;
}

inline bool target_dense_upper_layout(
    const DDInstance& ins, const UpperSignature& upper)
{
  const size_t vacancy_count =
      upper_vacancy_count(ins, upper);
  return ins.n_targets() > vacancy_count &&
         ins.n_targets() - vacancy_count >= vacancy_count;
}

struct LongDoubleAssignmentResult {
  std::vector<int> row_to_col;
  std::vector<long double> row_potential;
  std::vector<long double> col_potential;
  long double cost = 0;
  bool feasible = false;
};

// Floating-point Hungarian used only for Task-BR guidance/LB values.  Tie
// layers are solved separately below; they are never linearly mixed into
// the primary PairCost objective.
inline LongDoubleAssignmentResult hungarian_long_double(
    const std::vector<std::vector<long double>>& cost)
{
  LongDoubleAssignmentResult out;
  const size_t n = cost.size();
  const size_t m = cost.empty() ? 0 : cost.front().size();
  out.row_to_col.assign(n, -1);
  if (n == 0) {
    out.feasible = true;
    return out;
  }
  if (m < n || m == 0) return out;
  constexpr long double INF = 1e60L;
  std::vector<long double> u(n + 1, 0), v(m + 1, 0);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);
  for (size_t i = 1; i <= n; ++i) {
    p[0] = (int)i;
    int j0 = 0;
    std::vector<long double> minv(m + 1, INF);
    std::vector<uint8_t> used(m + 1, 0);
    do {
      used[j0] = 1;
      const int i0 = p[j0];
      long double delta = INF;
      int j1 = 0;
      for (size_t j = 1; j <= m; ++j) {
        if (used[j]) continue;
        const long double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta ||
            (minv[j] == delta && (j1 == 0 || (int)j < j1))) {
          delta = minv[j];
          j1 = (int)j;
        }
      }
      if (delta >= INF / 2) return out;
      for (size_t j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  out.cost = 0;
  out.feasible = true;
  for (size_t j = 1; j <= m; ++j)
    if (p[j] > 0) out.row_to_col[p[j] - 1] = (int)j - 1;
  for (size_t i = 0; i < n; ++i) {
    const int j = out.row_to_col[i];
    if (j < 0 || cost[i][j] >= INF / 2) {
      out.feasible = false;
      return out;
    }
    out.cost += cost[i][j];
  }
  out.row_potential.assign(u.begin() + 1, u.end());
  out.col_potential.assign(v.begin() + 1, v.end());
  return out;
}

struct LexAssignmentCost {
  long double primary = 0;
  long long secondary = 0;
  bool infinite = false;

  static LexAssignmentCost infinity()
  {
    LexAssignmentCost out;
    out.infinite = true;
    return out;
  }
};

inline bool operator==(const LexAssignmentCost& a,
                       const LexAssignmentCost& b)
{
  if (a.infinite || b.infinite)
    return a.infinite == b.infinite;
  return a.primary == b.primary &&
         a.secondary == b.secondary;
}

inline bool operator!=(const LexAssignmentCost& a,
                       const LexAssignmentCost& b)
{
  return !(a == b);
}

inline bool operator<(const LexAssignmentCost& a,
                      const LexAssignmentCost& b)
{
  if (a.infinite != b.infinite) return !a.infinite;
  if (a.infinite) return false;
  return a.primary != b.primary
             ? a.primary < b.primary
             : a.secondary < b.secondary;
}

inline LexAssignmentCost operator+(const LexAssignmentCost& a,
                                   const LexAssignmentCost& b)
{
  if (a.infinite || b.infinite)
    return LexAssignmentCost::infinity();
  return LexAssignmentCost{
      a.primary + b.primary,
      a.secondary + b.secondary,
      false};
}

inline LexAssignmentCost operator-(const LexAssignmentCost& a,
                                   const LexAssignmentCost& b)
{
  if (a.infinite) return LexAssignmentCost::infinity();
  if (b.infinite)
    throw std::logic_error(
        "LexAssignmentCost: finite minus infinity");
  return LexAssignmentCost{
      a.primary - b.primary,
      a.secondary - b.secondary,
      false};
}

inline LexAssignmentCost& operator+=(LexAssignmentCost& a,
                                     const LexAssignmentCost& b)
{
  a = a + b;
  return a;
}

inline LexAssignmentCost& operator-=(LexAssignmentCost& a,
                                     const LexAssignmentCost& b)
{
  a = a - b;
  return a;
}

struct LexAssignmentResult {
  std::vector<int> row_to_col;
  LexAssignmentCost cost;
  bool feasible = false;
};

inline LexAssignmentResult hungarian_lexicographic(
    const std::vector<std::vector<LexAssignmentCost>>& cost)
{
  LexAssignmentResult out;
  const size_t n = cost.size();
  const size_t m = cost.empty() ? 0 : cost.front().size();
  out.row_to_col.assign(n, -1);
  if (n == 0) {
    out.feasible = true;
    return out;
  }
  if (m < n || m == 0) return out;

  std::vector<LexAssignmentCost> u(n + 1), v(m + 1);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);
  for (size_t i = 1; i <= n; ++i) {
    p[0] = (int)i;
    int j0 = 0;
    std::vector<LexAssignmentCost> minv(
        m + 1, LexAssignmentCost::infinity());
    std::vector<uint8_t> used(m + 1, 0);
    do {
      used[j0] = 1;
      const int i0 = p[j0];
      LexAssignmentCost delta =
          LexAssignmentCost::infinity();
      int j1 = 0;
      for (size_t j = 1; j <= m; ++j) {
        if (used[j] || cost[i0 - 1][j - 1].infinite)
          continue;
        const LexAssignmentCost cur =
            cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta ||
            (minv[j] == delta &&
             (j1 == 0 || (int)j < j1))) {
          delta = minv[j];
          j1 = (int)j;
        }
      }
      if (delta.infinite) return out;
      for (size_t j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else if (!minv[j].infinite) {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  out.feasible = true;
  for (size_t j = 1; j <= m; ++j)
    if (p[j] > 0) out.row_to_col[p[j] - 1] = (int)j - 1;
  for (size_t i = 0; i < n; ++i) {
    const int j = out.row_to_col[i];
    if (j < 0 || cost[i][j].infinite) {
      out.feasible = false;
      return out;
    }
    out.cost += cost[i][j];
  }
  return out;
}

inline bool eligible_goal(const DDInstance& ins, int target, int goal)
{
  if (target < 0 || target >= (int)ins.n_targets()) return false;
  const auto& goals = ins.target_goal_sets[target];
  return std::binary_search(goals.begin(), goals.end(), goal);
}

inline PairPlan pair_cost(const DDInstance& ins, const UpperSignature& upper,
                          int target, int goal, DDDistCache& upper_wall,
                          double alpha, double gamma, double delta);
inline PairPlan pair_cost_prefix_lower_bound(
    const DDInstance& ins, const UpperSignature& upper, int target,
    int goal, DDDistCache& upper_wall, double alpha, double gamma,
    double delta, int prefix_cap);

inline PairCostTable build_pair_cost_table(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall, double alpha, double gamma, double delta)
{
  PairCostTable table(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b)
    for (const int goal : ins.target_goal_sets[b])
      table[b].push_back(PairCostEntry{
          goal, pair_cost(ins, upper, (int)b, goal, upper_wall, alpha,
                          gamma, delta)});
  return table;
}

inline std::vector<int> solve_tau_guide(const DDInstance& ins,
                                        const UpperSignature& upper,
                                        const PairCostTable& table)
{
  const size_t n = ins.n_targets();
  std::vector<int> tau(n, -1);
  if (n == 0) return tau;
  std::vector<int> goals;
  for (const auto& set : ins.target_goal_sets)
    goals.insert(goals.end(), set.begin(), set.end());
  std::sort(goals.begin(), goals.end());
  goals.erase(std::unique(goals.begin(), goals.end()), goals.end());
  std::vector<std::vector<LexAssignmentCost>> lex_cost(
      n, std::vector<LexAssignmentCost>(
             goals.size(), LexAssignmentCost::infinity()));
  for (size_t b = 0; b < n; ++b)
    for (const auto& entry : table[b]) {
      const auto it = std::lower_bound(goals.begin(), goals.end(), entry.goal);
      if (it == goals.end() || *it != entry.goal ||
          !std::isfinite(entry.plan.estimated_cost))
        continue;
      const int goal_index = (int)(it - goals.begin());
      const bool moved_away =
          std::binary_search(ins.target_goal_sets[b].begin(),
                             ins.target_goal_sets[b].end(),
                             upper.target_pos[b]) &&
          upper.target_pos[b] != goals[goal_index];
      lex_cost[b][goal_index] = LexAssignmentCost{
          (long double)entry.plan.estimated_cost,
          moved_away ? 1 : 0,
          false};
    }
  const auto optimum =
      hungarian_lexicographic(lex_cost);
  if (!optimum.feasible)
    throw std::logic_error(
        "solve_tau_guide: infeasible eligible-goal matching");

  // Exact tertiary assignment-vector tie: fix targets in id order to the
  // smallest goal that still admits the exact primary+secondary optimum.
  std::vector<uint8_t> used(goals.size(), 0);
  LexAssignmentCost prefix_cost;
  for (size_t b = 0; b < n; ++b) {
    bool fixed = false;
    for (size_t j = 0; j < goals.size(); ++j) {
      if (used[j] || lex_cost[b][j].infinite)
        continue;
      const size_t remaining_rows = n - b - 1;
      bool feasible = true;
      LexAssignmentCost remaining_opt;
      if (remaining_rows > 0) {
        std::vector<int> remaining_cols;
        for (size_t c = 0; c < goals.size(); ++c)
          if (!used[c] && c != j) remaining_cols.push_back((int)c);
        if (remaining_cols.size() < remaining_rows) {
          feasible = false;
        } else {
          std::vector<std::vector<LexAssignmentCost>> rem(
              remaining_rows,
              std::vector<LexAssignmentCost>(
                  remaining_cols.size(),
                  LexAssignmentCost::infinity()));
          for (size_t rr = 0; rr < remaining_rows; ++rr)
            for (size_t cc = 0; cc < remaining_cols.size(); ++cc)
              rem[rr][cc] =
                  lex_cost[b + 1 + rr][remaining_cols[cc]];
          const auto rem_assignment =
              hungarian_lexicographic(rem);
          feasible = rem_assignment.feasible;
          if (feasible) remaining_opt = rem_assignment.cost;
        }
      }
      const LexAssignmentCost candidate_total =
          prefix_cost + lex_cost[b][j] + remaining_opt;
      if (!feasible || candidate_total != optimum.cost)
        continue;
      tau[b] = goals[j];
      used[j] = 1;
      prefix_cost += lex_cost[b][j];
      fixed = true;
      break;
    }
    if (!fixed)
      throw std::logic_error(
          "solve_tau_guide: failed exact assignment-vector tie");
  }
  return tau;
}

struct LazyPairAssignment {
  PairCostTable table;
  std::vector<int> tau;
  long evaluated_edges = 0;
  long total_edges = 0;
  long rollout_work_steps = 0;
  long rollout_truncations = 0;
  long rollout_stalls = 0;
};

// Exact lazy assignment certificate.
//
// For every eligible edge e, pair_cost_prefix_lower_bound() returns L(e)
// with L(e) <= C(e), where C(e) is the full deterministic PairCost.  The
// mixed matrix below therefore remains a lower-bound matrix while an edge
// monotonically transitions from L(e) to exact C(e).  Each loop first
// refines every edge selected by the current Hungarian solution, so its
// value C* is exact.  For every still-lower-bound edge e, forcing e and
// solving the remaining injective assignment gives F_L(e), a lower bound
// on every exact assignment containing e.  We refine on F_L(e) <= C*
// (equality included).  At termination every unrefined edge has
// F_L(e) > C*, hence no primary-optimal assignment can contain it; all
// primary-optimal edges are exact.  The exact secondary moved-away count
// and tertiary assignment-vector tie can then be solved on this mixed
// table without changing the result of a fully evaluated matrix.
//
// PairPlan::exact=false remains only a branch-and-bound certificate.  It
// must never feed priorities, rollout diagnostics, or any consumer that
// interprets stalled/truncated/steps as a completed PairCost evaluation.
inline LazyPairAssignment build_lazy_pair_cost_assignment(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall, double alpha, double gamma, double delta)
{
  LazyPairAssignment out;
  const size_t target_count = ins.n_targets();
  out.table.resize(target_count);
  if (target_count == 0) return out;

  std::vector<int> goals;
  for (const auto& set : ins.target_goal_sets)
    goals.insert(goals.end(), set.begin(), set.end());
  std::sort(goals.begin(), goals.end());
  goals.erase(std::unique(goals.begin(), goals.end()), goals.end());
  constexpr long double INF = 1e60L;
  std::vector<std::vector<long double>> cost(
      target_count, std::vector<long double>(goals.size(), INF));
  std::vector<std::vector<int>> entry_index(
      target_count, std::vector<int>(goals.size(), -1));
  size_t eligible_edge_count = 0;
  for (const auto& set : ins.target_goal_sets)
    eligible_edge_count += set.size();
  const bool use_prefix_bounds =
      eligible_edge_count > target_count;

  for (size_t target = 0; target < target_count; ++target) {
    for (const int goal : ins.target_goal_sets[target]) {
      const auto goal_it =
          std::lower_bound(goals.begin(), goals.end(), goal);
      if (goal_it == goals.end() || *goal_it != goal)
        throw std::logic_error(
            "build_lazy_pair_cost_assignment: missing goal");
      const size_t goal_index = goal_it - goals.begin();
      PairPlan plan = use_prefix_bounds
                          ? pair_cost_prefix_lower_bound(
                                ins, upper, (int)target, goal,
                                upper_wall, alpha, gamma, delta, 4)
                          : PairPlan();
      if (!use_prefix_bounds) {
        const int distance =
            upper_wall.dist(goal, upper.target_pos[target]);
        plan.direct_distance =
            distance >= INT_MAX / 4 ? INT_MAX : distance;
        if (distance >= INT_MAX / 4) {
          plan.estimated_cost =
              std::numeric_limits<double>::infinity();
          plan.stalled = true;
        } else if (upper.target_pos[target] == goal) {
          plan.reached_goal = true;
        } else {
          plan.estimated_cost =
              alpha * (double)distance + 2.0 * gamma;
          plan.exact = false;
        }
      }
      entry_index[target][goal_index] =
          (int)out.table[target].size();
      out.table[target].push_back(PairCostEntry{goal, plan});
      cost[target][goal_index] =
          std::isfinite(plan.estimated_cost)
              ? (long double)plan.estimated_cost
              : INF;
      ++out.total_edges;
      out.evaluated_edges += plan.exact;
      out.rollout_work_steps += plan.rollout_steps;
      if (plan.exact) {
        out.rollout_truncations += plan.truncated;
        out.rollout_stalls += plan.stalled;
      }
    }
  }

  auto evaluate = [&](size_t target, size_t goal_index) {
    const int index = entry_index[target][goal_index];
    if (index < 0) return false;
    auto& entry = out.table[target][index];
    if (entry.plan.exact) return false;
    entry.plan = pair_cost(
        ins, upper, (int)target, goals[goal_index], upper_wall,
        alpha, gamma, delta);
    entry.plan.exact = true;
    out.rollout_work_steps += entry.plan.rollout_steps;
    out.rollout_truncations += entry.plan.truncated;
    out.rollout_stalls += entry.plan.stalled;
    cost[target][goal_index] =
        std::isfinite(entry.plan.estimated_cost)
            ? (long double)entry.plan.estimated_cost
            : INF;
    ++out.evaluated_edges;
    return true;
  };

  auto forced_lower_bound =
      [&](size_t forced_target, size_t forced_goal) {
        if (forced_target >= target_count ||
            forced_goal >= goals.size() ||
            cost[forced_target][forced_goal] >= INF / 2)
          return INF;
        std::vector<size_t> remaining_rows;
        std::vector<size_t> remaining_cols;
        for (size_t target = 0; target < target_count; ++target)
          if (target != forced_target)
            remaining_rows.push_back(target);
        for (size_t goal_index = 0; goal_index < goals.size();
             ++goal_index)
          if (goal_index != forced_goal)
            remaining_cols.push_back(goal_index);
        if (remaining_cols.size() < remaining_rows.size())
          return INF;

        std::vector<int> full_assignment(target_count, -1);
        full_assignment[forced_target] = (int)forced_goal;
        if (!remaining_rows.empty()) {
          std::vector<std::vector<long double>> remaining(
              remaining_rows.size(),
              std::vector<long double>(remaining_cols.size(), INF));
          for (size_t row = 0; row < remaining_rows.size(); ++row)
            for (size_t col = 0; col < remaining_cols.size(); ++col)
              remaining[row][col] =
                  cost[remaining_rows[row]][remaining_cols[col]];
          const auto assignment =
              hungarian_long_double(remaining);
          if (!assignment.feasible) return INF;
          for (size_t row = 0; row < remaining_rows.size(); ++row) {
            const int col = assignment.row_to_col[row];
            if (col < 0) return INF;
            full_assignment[remaining_rows[row]] =
                (int)remaining_cols[col];
          }
        }

        long double total = 0;
        for (size_t target = 0; target < target_count; ++target) {
          const int goal_index = full_assignment[target];
          if (goal_index < 0 ||
              cost[target][goal_index] >= INF / 2)
            return INF;
          total += cost[target][goal_index];
        }
        return total;
      };

  for (;;) {
    const auto assignment = hungarian_long_double(cost);
    if (!assignment.feasible)
      throw std::logic_error(
          "build_lazy_pair_cost_assignment: infeasible matching");

    bool refined = false;
    for (size_t target = 0; target < target_count; ++target) {
      const int goal_index = assignment.row_to_col[target];
      if (goal_index < 0)
        throw std::logic_error(
            "build_lazy_pair_cost_assignment: unassigned target");
      refined |= evaluate(target, (size_t)goal_index);
    }
    if (refined) continue;

    for (size_t target = 0; target < target_count; ++target) {
      for (size_t goal_index = 0; goal_index < goals.size();
           ++goal_index) {
        const int index = entry_index[target][goal_index];
        if (index < 0 || out.table[target][index].plan.exact ||
            cost[target][goal_index] >= INF / 2)
          continue;
        if (forced_lower_bound(target, goal_index) <=
            assignment.cost)
          refined |= evaluate(target, goal_index);
      }
    }
    if (!refined) break;
  }

  out.tau = solve_tau_guide(ins, upper, out.table);
  for (size_t target = 0; target < out.tau.size(); ++target) {
    const auto found = std::find_if(
        out.table[target].begin(), out.table[target].end(),
        [&](const PairCostEntry& entry) {
          return entry.goal == out.tau[target];
        });
    if (found == out.table[target].end() || !found->plan.exact)
      throw std::logic_error(
          "build_lazy_pair_cost_assignment: selected edge is not exact");
  }
  return out;
}

inline double solve_tau_lb(const DDInstance& ins, const PhysConfig& s,
                           DDDistCache& upper_wall, double alpha,
                           double gamma)
{
  const size_t n = ins.n_targets();
  if (n == 0) return 0;
  std::vector<int> goals;
  for (const auto& set : ins.target_goal_sets)
    goals.insert(goals.end(), set.begin(), set.end());
  std::sort(goals.begin(), goals.end());
  goals.erase(std::unique(goals.begin(), goals.end()), goals.end());
  constexpr long double INF = 1e60L;
  std::vector<uint8_t> carried(n, 0);
  for (const int k : s.kappa)
    if (k >= 0 && k < (int)n) carried[k] = 1;
  std::vector<std::vector<long double>> cost(
      n, std::vector<long double>(goals.size(), INF));
  for (size_t b = 0; b < n; ++b)
    for (size_t j = 0; j < goals.size(); ++j) {
      const int goal = goals[j];
      if (!eligible_goal(ins, (int)b, goal)) continue;
      const int d = upper_wall.dist(goal, s.target_pos[b]);
      if (d >= INT_MAX / 4) continue;
      long double v = alpha * (long double)d;
      if (carried[b])
        v += gamma;
      else if (s.target_pos[b] != goal)
        v += 2.0L * gamma;
      cost[b][j] = v;
    }
  const auto result = hungarian_long_double(cost);
  if (!result.feasible)
    throw std::logic_error("solve_tau_lb: infeasible eligible-goal matching");
  return (double)result.cost;
}

struct TaskBRCompilerLimits {
  int recursion_cap = 256;
  int backtrack_cap = 512;
  // Negative preserves the legacy/probe default.  Production may impose a
  // tighter epoch-wide work cap without removing the clean local window that
  // each top-level root option receives.
  int total_recursion_cap = -1;
};

struct AbstractUpperState {
  std::vector<ShelfSelector> shelves;
  std::vector<int> positions;
  int target_count = 0;
  std::vector<int> anon_index_by_epoch_cell;
  std::vector<int> occupant;

  int shelf_index(const ShelfSelector& shelf) const
  {
    if (shelf.kind == ShelfSelector::Kind::TARGET)
      return shelf.value >= 0 && shelf.value < target_count
                 ? shelf.value
                 : -1;
    return shelf.value >= 0 &&
                   shelf.value < (int)anon_index_by_epoch_cell.size()
               ? anon_index_by_epoch_cell[shelf.value]
               : -1;
  }

  int position(const ShelfSelector& shelf) const
  {
    const int index = shelf_index(shelf);
    return index < 0 ? -1 : positions[index];
  }

  bool empty(int cell) const
  {
    return cell >= 0 && cell < (int)occupant.size() &&
           occupant[cell] < 0;
  }

  const ShelfSelector* shelf_at(int cell) const
  {
    if (cell < 0 || cell >= (int)occupant.size()) return nullptr;
    const int index = occupant[cell];
    return index < 0 ? nullptr : &shelves[index];
  }

  void move(const ShelfSelector& shelf, int to)
  {
    const int index = shelf_index(shelf);
    if (index < 0) throw std::logic_error("unknown abstract shelf");
    const int from = positions[index];
    if (from >= 0) occupant[from] = -1;
    positions[index] = to;
    occupant[to] = index;
  }
};

inline AbstractUpperState make_abstract_upper_state(
    const DDInstance& ins, const UpperSignature& upper)
{
  AbstractUpperState out;
  out.target_count = (int)upper.target_pos.size();
  out.anon_index_by_epoch_cell.assign(ins.grid.size(), -1);
  out.occupant.assign(ins.grid.size(), -1);
  auto add = [&](const ShelfSelector& shelf, int cell) {
    if (cell < 0 || cell >= ins.grid.size() || ins.grid.is_wall(cell))
      throw std::logic_error("abstract shelf on invalid cell");
    if (out.occupant[cell] >= 0)
      throw std::logic_error("duplicate shelf cell in upper projection");
    const int index = (int)out.shelves.size();
    out.shelves.push_back(shelf);
    out.positions.push_back(cell);
    if (shelf.kind == ShelfSelector::Kind::ANON_AT_EPOCH_CELL)
      out.anon_index_by_epoch_cell[shelf.value] = index;
    out.occupant[cell] = index;
  };
  for (size_t b = 0; b < upper.target_pos.size(); ++b)
    add(ShelfSelector{ShelfSelector::Kind::TARGET, (int)b},
        upper.target_pos[b]);
  for (const int cell : upper.anon_pos)
    add(ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, cell},
        cell);
  return out;
}

inline void add_root_demand(std::vector<RootDemand>& roots,
                            const RootDemand& root)
{
  roots.push_back(root);
  std::sort(roots.begin(), roots.end());
  roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
}

struct StorageTransferCandidate {
  int endpoint = -1;
  int first_step = -1;
  int route_size = 0;
  // Null means the exact adjacent route [from, first_step].  Non-null is
  // a read-only view owned by the surrounding candidate window.
  const std::vector<int>* explicit_route = nullptr;
};

struct SelectedStorageTransfer {
  int endpoint = -1;
  int first_step = -1;
  int route_size = 0;
  // Empty means the exact adjacent route [from, first_step].  PairCost
  // copies a dynamic route only after this candidate becomes ready.
  std::vector<int> explicit_route;
};

inline SelectedStorageTransfer select_storage_transfer(
    const StorageTransferCandidate& candidate)
{
  SelectedStorageTransfer out;
  out.endpoint = candidate.endpoint;
  out.first_step = candidate.first_step;
  out.route_size = candidate.route_size;
  if (candidate.explicit_route != nullptr)
    out.explicit_route = *candidate.explicit_route;
  return out;
}

inline StorageTransfer materialize_storage_transfer(
    int from, const StorageTransferCandidate& candidate)
{
  if (candidate.explicit_route != nullptr)
    return StorageTransfer{
        candidate.endpoint, *candidate.explicit_route};
  return StorageTransfer{
      candidate.endpoint, {from, candidate.first_step}};
}

struct TaskBRCompilerState {
  ShelfTaskGraph graph;
  std::map<TaskId, int> effect_index;
  std::map<ShelfSelector, TaskId> reserved_shelf_effect;
  std::map<int, TaskId> reserved_destination;
  std::map<int, TaskId> reserved_endpoint;
};

struct TaskBRCompilerUndo {
  enum class Kind {
    ERASE_EFFECT_INDEX,
    ERASE_SHELF_RESERVATION,
    ERASE_DESTINATION_RESERVATION,
    ERASE_ENDPOINT_RESERVATION,
    POP_GRAPH_TASK,
    POP_SUCCESSOR,
    POP_ROTATION,
    RESTORE_TASK,
  };
  Kind kind = Kind::POP_GRAPH_TASK;
  TaskId task_id;
  ShelfSelector shelf;
  int index = -1;
  ShelfTask old_task;
};

struct TaskBRCompilerTransaction {
  static constexpr bool records_rotations = true;

  TaskBRCompilerState& state;
  std::vector<TaskBRCompilerUndo> undo;

  explicit TaskBRCompilerTransaction(TaskBRCompilerState& state_)
      : state(state_)
  {
  }

  size_t checkpoint() const { return undo.size(); }

  void enter_recursion(const ShelfSelector&) {}
  void leave_recursion(const ShelfSelector&) {}

  bool recursion_cycle(
      const ShelfSelector& shelf,
      const std::vector<ShelfSelector>& recursion_stack) const
  {
    return std::find(
               recursion_stack.begin(), recursion_stack.end(), shelf) !=
           recursion_stack.end();
  }

  int find_effect(const TaskId& effect) const
  {
    const auto found = state.effect_index.find(effect);
    return found == state.effect_index.end() ? -1 : found->second;
  }

  std::optional<TaskId> shelf_reservation(
      const ShelfSelector& shelf) const
  {
    const auto found = state.reserved_shelf_effect.find(shelf);
    return found == state.reserved_shelf_effect.end()
               ? std::nullopt
               : std::optional<TaskId>(found->second);
  }

  std::optional<TaskId> destination_reservation(int cell) const
  {
    const auto found = state.reserved_destination.find(cell);
    return found == state.reserved_destination.end()
               ? std::nullopt
               : std::optional<TaskId>(found->second);
  }

  bool shelf_effect_conflicts(const ShelfSelector& shelf,
                              const TaskId& effect) const
  {
    const auto found = state.reserved_shelf_effect.find(shelf);
    return found != state.reserved_shelf_effect.end() &&
           found->second != effect;
  }

  bool destination_effect_conflicts(int cell,
                                    const TaskId& effect) const
  {
    const auto found = state.reserved_destination.find(cell);
    return found != state.reserved_destination.end() &&
           found->second != effect;
  }

  bool destination_reserved(int cell) const
  {
    return state.reserved_destination.count(cell) != 0;
  }

  bool endpoint_reserved(int cell) const
  {
    return state.reserved_destination.count(cell) != 0 ||
           state.reserved_endpoint.count(cell) != 0;
  }

  bool has_distinct_endpoint_reservations() const
  {
    return !state.reserved_endpoint.empty();
  }

  bool distinct_endpoint_reserved(int cell) const
  {
    return state.reserved_endpoint.count(cell) != 0;
  }

  bool endpoint_effect_conflicts(int cell,
                                 const TaskId& effect) const
  {
    const auto destination =
        state.reserved_destination.find(cell);
    if (destination != state.reserved_destination.end() &&
        destination->second != effect)
      return true;
    const auto found = state.reserved_endpoint.find(cell);
    return found != state.reserved_endpoint.end() &&
           found->second != effect;
  }

  bool distinct_endpoint_effect_conflicts(
      int cell, const TaskId& effect) const
  {
    const auto found = state.reserved_endpoint.find(cell);
    return found != state.reserved_endpoint.end() &&
           found->second != effect;
  }

  void reserve_shelf(const ShelfSelector& shelf, const TaskId& effect)
  {
    const auto inserted =
        state.reserved_shelf_effect.emplace(shelf, effect);
    if (inserted.second)
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::ERASE_SHELF_RESERVATION,
          TaskId(), shelf});
  }

  void reserve_destination(int cell, const TaskId& effect)
  {
    const auto inserted =
        state.reserved_destination.emplace(cell, effect);
    if (inserted.second)
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::ERASE_DESTINATION_RESERVATION,
          TaskId(), ShelfSelector(), cell});
  }

  void reserve_endpoint(int cell, const TaskId& effect)
  {
    const auto inserted = state.reserved_endpoint.emplace(cell, effect);
    if (inserted.second)
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::ERASE_ENDPOINT_RESERVATION,
          TaskId(), ShelfSelector(), cell});
  }

  void merge_task(int index, const RootDemand& root, int priority)
  {
    auto& task = state.graph.tasks[index];
    const bool has_root =
        std::binary_search(task.roots.begin(), task.roots.end(), root);
    const int merged_priority = std::max(task.priority, priority);
    if (has_root && merged_priority == task.priority) return;
    TaskBRCompilerUndo entry;
    entry.kind = TaskBRCompilerUndo::Kind::RESTORE_TASK;
    entry.index = index;
    entry.old_task = task;
    undo.push_back(std::move(entry));
    add_root_demand(task.roots, root);
    task.priority = merged_priority;
  }

  int add_task(const TaskId& id,
               const StorageTransferCandidate& transfer,
               int from,
               const RootDemand& root, int predecessor, int priority)
  {
    const auto found = state.effect_index.find(id);
    if (found != state.effect_index.end()) {
      merge_task(found->second, root, priority);
      return found->second;
    }

    const int index = (int)state.graph.tasks.size();
    state.effect_index.emplace(id, index);
    undo.push_back(TaskBRCompilerUndo{
        TaskBRCompilerUndo::Kind::ERASE_EFFECT_INDEX, id});
    state.graph.tasks.push_back(
        ShelfTask{
            id, {root}, priority,
            materialize_storage_transfer(from, transfer)});
    state.graph.predecessors.emplace_back();
    state.graph.successors.emplace_back();
    undo.push_back(TaskBRCompilerUndo{
        TaskBRCompilerUndo::Kind::POP_GRAPH_TASK});
    if (predecessor >= 0) {
      state.graph.predecessors[index].push_back(predecessor);
      state.graph.successors[predecessor].push_back(index);
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::POP_SUCCESSOR,
          TaskId(), ShelfSelector(), predecessor});
    }
    return index;
  }

  void add_rotation(const RotationCandidate& rotation)
  {
    const auto duplicate = std::find_if(
        state.graph.rotations.begin(), state.graph.rotations.end(),
        [&](const RotationCandidate& existing) {
          return existing.cycle == rotation.cycle;
        });
    if (duplicate != state.graph.rotations.end()) return;
    state.graph.rotations.push_back(rotation);
    undo.push_back(TaskBRCompilerUndo{
        TaskBRCompilerUndo::Kind::POP_ROTATION});
  }

  void rollback(size_t checkpoint)
  {
    while (undo.size() > checkpoint) {
      auto entry = std::move(undo.back());
      undo.pop_back();
      switch (entry.kind) {
        case TaskBRCompilerUndo::Kind::ERASE_EFFECT_INDEX:
          state.effect_index.erase(entry.task_id);
          break;
        case TaskBRCompilerUndo::Kind::ERASE_SHELF_RESERVATION:
          state.reserved_shelf_effect.erase(entry.shelf);
          break;
        case TaskBRCompilerUndo::Kind::ERASE_DESTINATION_RESERVATION:
          state.reserved_destination.erase(entry.index);
          break;
        case TaskBRCompilerUndo::Kind::ERASE_ENDPOINT_RESERVATION:
          state.reserved_endpoint.erase(entry.index);
          break;
        case TaskBRCompilerUndo::Kind::POP_GRAPH_TASK:
          state.graph.tasks.pop_back();
          state.graph.predecessors.pop_back();
          state.graph.successors.pop_back();
          break;
        case TaskBRCompilerUndo::Kind::POP_SUCCESSOR:
          state.graph.successors[entry.index].pop_back();
          break;
        case TaskBRCompilerUndo::Kind::POP_ROTATION:
          state.graph.rotations.pop_back();
          break;
        case TaskBRCompilerUndo::Kind::RESTORE_TASK:
          state.graph.tasks[entry.index] = std::move(entry.old_task);
          break;
      }
    }
  }
};

struct TaskBRCompilerBudget {
  int recursion_calls = 0;
  long long total_recursion_calls = 0;
  int branch_calls = 0;
  bool recursion_exhausted = false;
  bool backtrack_exhausted = false;
  long effect_conflicts = 0;
  long candidate_backtracks = 0;
};

inline bool task_effects_conflict(const TaskId& a, const TaskId& b)
{
  if (a == b) return false;
  if (a.shelf == b.shelf) return true;
  return a.to == b.to;
}

inline bool adjacent_cells(const DDGrid& grid, int from, int to);

struct OrderedShelfCandidates {
  std::array<int, 4> endpoints{};
  std::array<int, 4> first_steps{};
  std::array<int, 4> route_sizes{};
  std::array<int, 4> route_slots{{-1, -1, -1, -1}};
  std::vector<std::vector<int>> explicit_routes;
  int count = 0;

  StorageTransferCandidate candidate(int index) const
  {
    const int route_slot = route_slots[index];
    return StorageTransferCandidate{
        endpoints[index], first_steps[index], route_sizes[index],
        route_slot >= 0 ? &explicit_routes[route_slot] : nullptr};
  }
};

inline std::vector<StorageTransfer> reachable_storage_transfers(
    const DDInstance& ins, const AbstractUpperState& upper, int from)
{
  std::vector<StorageTransfer> transfers;
  if (from < 0 || from >= ins.grid.size() || ins.grid.is_wall(from))
    return transfers;

  std::vector<int> parent(ins.grid.size(), -2);
  std::deque<int> queue;
  parent[from] = -1;
  queue.push_back(from);
  std::map<int, StorageTransfer> route_by_endpoint;
  while (!queue.empty()) {
    const int cell = queue.front();
    queue.pop_front();
    int raw_neighbors[4];
    const int count = ins.grid.neighbors(cell, raw_neighbors);
    std::array<int, 4> neighbors{};
    std::copy(raw_neighbors, raw_neighbors + count, neighbors.begin());
    std::sort(neighbors.begin(), neighbors.begin() + count);
    for (int index = 0; index < count; ++index) {
      const int next = neighbors[index];
      if (next == from) continue;
      if (ins.can_store_shelf(next)) {
        if (route_by_endpoint.count(next) != 0) continue;
        std::vector<int> route;
        for (int cursor = cell; cursor >= 0; cursor = parent[cursor])
          route.push_back(cursor);
        std::reverse(route.begin(), route.end());
        route.push_back(next);
        route_by_endpoint.emplace(
            next, StorageTransfer{next, std::move(route)});
        continue;
      }
      if (!upper.empty(next) || parent[next] != -2) continue;
      parent[next] = cell;
      queue.push_back(next);
    }
  }
  transfers.reserve(route_by_endpoint.size());
  for (auto& entry : route_by_endpoint)
    transfers.push_back(std::move(entry.second));
  return transfers;
}

template <typename CompilerContext>
inline OrderedShelfCandidates ordered_shelf_candidate_window(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const CompilerContext& context)
{
  const int from = upper.position(shelf);
  const bool root_shelf =
      shelf.kind == ShelfSelector::Kind::TARGET &&
      shelf.value == root.target;

  std::array<int, 4> neighbors{};
  const int neighbor_count =
      ins.grid.neighbors(from, neighbors.data());
  if (ins.has_adjacent_storage_frontier(from)) {
    using DirectCandidateScore =
        std::tuple<int, int, int, int, int>;
    std::array<DirectCandidateScore, 4> scores{};
    for (int index = 0; index < neighbor_count; ++index) {
      const int endpoint = neighbors[index];
      const int reserved =
          context.destination_reserved(endpoint) ||
                  (context.has_distinct_endpoint_reservations() &&
                   context.distinct_endpoint_reserved(endpoint))
              ? 1
              : 0;
      const int occupied = upper.empty(endpoint) ? 0 : 1;
      if (root_shelf) {
        const int mission =
            upper_wall.dist(root.goal, endpoint);
        scores[index] = std::make_tuple(
            mission, occupied, reserved, mission, endpoint);
      } else if (
          !single_root_mode &&
          shelf.kind == ShelfSelector::Kind::TARGET &&
          tau != nullptr && shelf.value >= 0 &&
          shelf.value < (int)tau->size()) {
        const int mission =
            upper_wall.dist((*tau)[shelf.value], endpoint);
        scores[index] = std::make_tuple(
            occupied, occupied * 2, reserved, mission, endpoint);
      } else {
        int assigned_goal_interference = 0;
        if (!single_root_mode && tau != nullptr)
          assigned_goal_interference =
              std::count(tau->begin(), tau->end(), endpoint);
        scores[index] = std::make_tuple(
            assigned_goal_interference, occupied, reserved,
            0, endpoint);
      }
    }
    for (int index = 1; index < neighbor_count; ++index) {
      const int endpoint = neighbors[index];
      const DirectCandidateScore score = scores[index];
      int insertion = index;
      while (insertion > 0 &&
             score < scores[insertion - 1]) {
        neighbors[insertion] = neighbors[insertion - 1];
        scores[insertion] = scores[insertion - 1];
        --insertion;
      }
      neighbors[insertion] = endpoint;
      scores[insertion] = score;
    }
    OrderedShelfCandidates out;
    out.count = neighbor_count;
    for (int index = 0; index < out.count; ++index) {
      const int endpoint = neighbors[index];
      out.endpoints[index] = endpoint;
      out.first_steps[index] = endpoint;
      out.route_sizes[index] = 2;
    }
    return out;
  }

  using CandidateScore =
      std::tuple<int, int, int, int, int, int>;
  struct RankedCandidate {
    CandidateScore score;
    StorageTransfer transfer;
  };
  std::array<RankedCandidate, 4> ranked{};
  int ranked_count = 0;
  const auto consider =
      [&](StorageTransfer transfer) {
    if (transfer.route.size() < 2) return;
    const int endpoint = transfer.endpoint;
    const int first_step = transfer.route[1];
    const int route_size = (int)transfer.route.size();
    const int reserved =
        context.destination_reserved(first_step) ||
                context.endpoint_reserved(endpoint)
            ? 1
            : 0;
    const int occupied = upper.empty(endpoint) ? 0 : 1;
    CandidateScore score;
    if (root_shelf) {
      const int mission = upper_wall.dist(root.goal, endpoint);
      score = std::make_tuple(
          mission, occupied, reserved, route_size,
          endpoint, first_step);
    } else if (!single_root_mode &&
               shelf.kind == ShelfSelector::Kind::TARGET &&
               tau != nullptr && shelf.value >= 0 &&
               shelf.value < (int)tau->size()) {
      const int own_goal = (*tau)[shelf.value];
      const int mission = upper_wall.dist(own_goal, endpoint);
      score = std::make_tuple(
          occupied, reserved, mission, route_size,
          endpoint, first_step);
    } else {
      int assigned_goal_interference = 0;
      if (!single_root_mode && tau != nullptr)
        assigned_goal_interference =
              std::count(tau->begin(), tau->end(), endpoint);
      score = std::make_tuple(
          assigned_goal_interference, occupied, reserved,
          route_size, endpoint, first_step);
    }

    int insert = 0;
    while (insert < ranked_count &&
           !(score < ranked[insert].score))
      ++insert;
    if (insert >= (int)ranked.size()) return;
    const int new_count =
        std::min<int>(ranked.size(), ranked_count + 1);
    for (int index = new_count - 1; index > insert; --index)
      ranked[index] = std::move(ranked[index - 1]);
    ranked[insert] =
        RankedCandidate{score, std::move(transfer)};
    ranked_count = new_count;
  };

  auto transfers =
      reachable_storage_transfers(ins, upper, from);
  for (auto& transfer : transfers)
    consider(std::move(transfer));

  OrderedShelfCandidates out;
  out.count = ranked_count;
  out.explicit_routes.reserve(ranked_count);
  for (int index = 0; index < out.count; ++index) {
    auto& transfer = ranked[index].transfer;
    out.endpoints[index] = transfer.endpoint;
    out.first_steps[index] = transfer.route[1];
    out.route_sizes[index] = (int)transfer.route.size();
    if (transfer.route.size() > 2) {
      out.route_slots[index] =
          (int)out.explicit_routes.size();
      out.explicit_routes.push_back(
          std::move(transfer.route));
    }
  }
  return out;
}

inline std::vector<int> ordered_shelf_candidates(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const std::map<int, TaskId>& reserved_destination)
{
  struct MapReservationView {
    const std::map<int, TaskId>& reservations;
    bool destination_reserved(int cell) const
    {
      return reservations.count(cell) != 0;
    }
    bool endpoint_reserved(int cell) const
    {
      return reservations.count(cell) != 0;
    }
    bool has_distinct_endpoint_reservations() const
    {
      return false;
    }
    bool distinct_endpoint_reserved(int) const { return false; }
  };
  const auto ordered = ordered_shelf_candidate_window(
      ins, upper, shelf, root, tau, single_root_mode, upper_wall,
      MapReservationView{reserved_destination});
  std::vector<int> endpoints;
  endpoints.reserve(ordered.count);
  for (int index = 0; index < ordered.count; ++index)
    endpoints.push_back(ordered.endpoints[index]);
  return endpoints;
}

template <typename CompilerContext>
inline std::optional<RotationCandidate> make_rotation_candidate(
    const std::vector<ShelfSelector>& recursion_stack,
    std::vector<ShelfSelector>::const_iterator cycle_begin,
    const CompilerContext& context)
{
  const size_t cycle_size =
      (size_t)std::distance(cycle_begin, recursion_stack.end());
  if (cycle_size < 3) return std::nullopt;

  RotationCandidate rotation;
  rotation.cycle.reserve(cycle_size);
  for (auto it = cycle_begin; it != recursion_stack.end(); ++it) {
    const auto effect = context.shelf_reservation(*it);
    if (!effect.has_value()) return std::nullopt;
    rotation.cycle.push_back(*effect);
  }
  for (size_t index = 0; index < rotation.cycle.size(); ++index)
    if (rotation.cycle[index].to !=
        rotation.cycle[(index + 1) % rotation.cycle.size()].from)
      return std::nullopt;

  const auto first = std::min_element(
      rotation.cycle.begin(), rotation.cycle.end());
  std::rotate(rotation.cycle.begin(), first, rotation.cycle.end());
  return rotation;
}

inline void add_unique_rotation_candidate(
    std::vector<RotationCandidate>& rotations,
    RotationCandidate rotation)
{
  const auto duplicate = std::find_if(
      rotations.begin(), rotations.end(),
      [&](const RotationCandidate& existing) {
        return existing.cycle == rotation.cycle;
      });
  if (duplicate == rotations.end())
    rotations.push_back(std::move(rotation));
}

template <typename CompilerContext>
inline int resolve_shelf_task_br_pibt(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root, int root_priority,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall, const TaskBRCompilerLimits& limits,
    TaskBRCompilerBudget& budget,
    CompilerContext& context,
    std::vector<ShelfSelector>& recursion_stack,
    std::vector<RotationCandidate>& encountered_rotations,
    const StorageTransferCandidate* forced_first_transfer = nullptr)
{
  const bool total_recursion_exhausted =
      limits.total_recursion_cap >= 0 &&
      budget.total_recursion_calls >= limits.total_recursion_cap;
  if (budget.recursion_calls >= limits.recursion_cap ||
      total_recursion_exhausted) {
    budget.recursion_exhausted = true;
    return -1;
  }
  ++budget.recursion_calls;
  ++budget.total_recursion_calls;
  const int from = upper.position(shelf);
  if (from < 0) return -1;
  if constexpr (CompilerContext::records_rotations)
    recursion_stack.push_back(shelf);
  context.enter_recursion(shelf);
  auto leave_recursion = [&]() {
    context.leave_recursion(shelf);
    if constexpr (CompilerContext::records_rotations)
      recursion_stack.pop_back();
  };
  OrderedShelfCandidates candidates;
  if (forced_first_transfer == nullptr) {
    candidates = ordered_shelf_candidate_window(
        ins, upper, shelf, root, tau, single_root_mode, upper_wall,
        context);
  }

  const int candidate_count =
      forced_first_transfer != nullptr ? 1 : candidates.count;
  for (int candidate_index = 0;
       candidate_index < candidate_count; ++candidate_index) {
    const StorageTransferCandidate transfer =
        forced_first_transfer != nullptr
            ? *forced_first_transfer
            : candidates.candidate(candidate_index);
    if (transfer.explicit_route == nullptr) {
      assert(
          transfer.route_size == 2 &&
          transfer.first_step == transfer.endpoint &&
          ins.can_store_shelf(transfer.endpoint));
    } else {
      const auto& route = *transfer.explicit_route;
      bool route_valid =
          transfer.route_size >= 2 &&
          transfer.first_step >= 0 &&
          ins.can_store_shelf(transfer.endpoint) &&
          transfer.route_size ==
              (int)route.size() &&
          route.front() == from &&
          route[1] == transfer.first_step &&
          route.back() == transfer.endpoint;
      for (size_t index = 1;
           index < route.size(); ++index)
        route_valid &= adjacent_cells(
            ins.grid, route[index - 1], route[index]);
      for (size_t index = 1;
           index + 1 < route.size(); ++index)
        route_valid &=
            !ins.can_store_shelf(route[index]) &&
            upper.empty(route[index]);
      if (!route_valid) continue;
    }

    const int to = transfer.first_step;
    const TaskId effect{shelf, from, to};
    const int existing = context.find_effect(effect);
    if (existing >= 0) {
      context.merge_task(existing, root, root_priority);
      leave_recursion();
      return existing;
    }

    if (context.shelf_effect_conflicts(shelf, effect)) {
      ++budget.effect_conflicts;
      continue;
    }
    if (context.destination_effect_conflicts(to, effect)) {
      ++budget.effect_conflicts;
      continue;
    }
    const bool endpoint_conflict =
        transfer.endpoint == to
            ? context.has_distinct_endpoint_reservations() &&
                  context.distinct_endpoint_effect_conflicts(
                      transfer.endpoint, effect)
            : context.endpoint_effect_conflicts(
                  transfer.endpoint, effect);
    if (endpoint_conflict) {
      ++budget.effect_conflicts;
      continue;
    }

    const size_t checkpoint = context.checkpoint();
    context.reserve_shelf(shelf, effect);
    context.reserve_destination(to, effect);
    if (transfer.endpoint != to)
      context.reserve_endpoint(transfer.endpoint, effect);
    int predecessor = -1;
    if (!upper.empty(transfer.endpoint)) {
      const ShelfSelector blocker =
          *upper.shelf_at(transfer.endpoint);
      if (context.recursion_cycle(blocker, recursion_stack)) {
        if constexpr (CompilerContext::records_rotations) {
          const auto cycle_begin = std::find(
              recursion_stack.begin(), recursion_stack.end(), blocker);
          if (const auto rotation = make_rotation_candidate(
                  recursion_stack, cycle_begin, context);
              rotation.has_value())
            add_unique_rotation_candidate(
                encountered_rotations, *rotation);
        }
        ++budget.candidate_backtracks;
        context.rollback(checkpoint);
        continue;
      }
      predecessor = resolve_shelf_task_br_pibt(
          ins, upper, blocker, root, root_priority, tau, single_root_mode,
          upper_wall, limits, budget, context, recursion_stack,
          encountered_rotations);
      if (predecessor < 0) {
        ++budget.candidate_backtracks;
        context.rollback(checkpoint);
        continue;
      }
    }
    const int result = context.add_task(
        effect, transfer, from, root, predecessor, root_priority);
    leave_recursion();
    return result;
  }
  leave_recursion();
  return -1;
}

struct SingleRootReadyResult {
  std::optional<TaskId> ready_effect;
  std::optional<SelectedStorageTransfer> ready_transfer;
  int recursion_calls = 0;
  bool recursion_exhausted = false;
  long effect_conflicts = 0;
  long candidate_backtracks = 0;
};

struct SingleRootCompilerScratch {
  static constexpr bool records_rotations = false;

  enum class UndoKind { SHELF, DESTINATION, ENDPOINT };
  struct Undo {
    UndoKind kind = UndoKind::SHELF;
    int index = -1;
  };

  const AbstractUpperState* upper = nullptr;
  std::vector<int> reserved_shelf_to;
  std::vector<uint32_t> reserved_shelf_stamp;
  std::vector<int> reserved_destination_shelf;
  std::vector<uint32_t> reserved_destination_stamp;
  std::vector<int> reserved_endpoint_shelf;
  std::vector<uint32_t> reserved_endpoint_stamp;
  std::vector<uint32_t> active_shelf_stamp;
  std::vector<Undo> undo;
  std::vector<ShelfSelector> recursion_stack;
  std::vector<RotationCandidate> encountered_rotations;
  std::optional<TaskId> ready_effect;
  std::optional<SelectedStorageTransfer> ready_transfer;
  int distinct_endpoint_reservation_count = 0;
  size_t reservation_cell_count = 0;
  uint32_t generation = 0;

  void reset(const AbstractUpperState& upper_, size_t cell_count)
  {
    upper = &upper_;
    reserved_shelf_to.resize(upper_.shelves.size());
    reserved_shelf_stamp.resize(upper_.shelves.size(), 0);
    reserved_destination_shelf.resize(cell_count);
    reserved_destination_stamp.resize(cell_count, 0);
    active_shelf_stamp.resize(upper_.shelves.size(), 0);
    reservation_cell_count = cell_count;
    const size_t max_undo = 3 * upper_.shelves.size();
    if (undo.capacity() < max_undo)
      undo.reserve(max_undo);
    ++generation;
    if (generation == 0) {
      std::fill(
          reserved_shelf_stamp.begin(), reserved_shelf_stamp.end(), 0);
      std::fill(
          reserved_destination_stamp.begin(),
          reserved_destination_stamp.end(), 0);
      std::fill(
          reserved_endpoint_stamp.begin(),
          reserved_endpoint_stamp.end(), 0);
      std::fill(
          active_shelf_stamp.begin(), active_shelf_stamp.end(), 0);
      generation = 1;
    }
    undo.clear();
    recursion_stack.clear();
    encountered_rotations.clear();
    ready_effect.reset();
    ready_transfer.reset();
    distinct_endpoint_reservation_count = 0;
  }

  int shelf_index(const ShelfSelector& shelf) const
  {
    return upper == nullptr ? -1 : upper->shelf_index(shelf);
  }

  size_t checkpoint() const { return undo.size(); }

  void enter_recursion(const ShelfSelector& shelf)
  {
    const int index = shelf_index(shelf);
    if (index >= 0 && index < (int)active_shelf_stamp.size())
      active_shelf_stamp[index] = generation;
  }

  void leave_recursion(const ShelfSelector& shelf)
  {
    const int index = shelf_index(shelf);
    if (index >= 0 && index < (int)active_shelf_stamp.size())
      active_shelf_stamp[index] = 0;
  }

  bool recursion_cycle(
      const ShelfSelector& shelf,
      const std::vector<ShelfSelector>&) const
  {
    const int index = shelf_index(shelf);
    return index >= 0 &&
           index < (int)active_shelf_stamp.size() &&
           active_shelf_stamp[index] == generation;
  }

  int find_effect(const TaskId&) const { return -1; }

  std::optional<TaskId> shelf_reservation(
      const ShelfSelector& shelf) const
  {
    const int index = shelf_index(shelf);
    if (index < 0 ||
        index >= (int)reserved_shelf_stamp.size() ||
        reserved_shelf_stamp[index] != generation)
      return std::nullopt;
    return TaskId{
        shelf, upper->positions[index], reserved_shelf_to[index]};
  }

  std::optional<TaskId> destination_reservation(int cell) const
  {
    if (cell < 0 ||
        cell >= (int)reserved_destination_stamp.size() ||
        reserved_destination_stamp[cell] != generation)
      return std::nullopt;
    const int index = reserved_destination_shelf[cell];
    if (index < 0 ||
        index >= (int)reserved_shelf_stamp.size() ||
        reserved_shelf_stamp[index] != generation)
      return std::nullopt;
    return TaskId{
        upper->shelves[index], upper->positions[index],
        reserved_shelf_to[index]};
  }

  bool destination_reserved(int cell) const
  {
    return cell >= 0 &&
           cell < (int)reserved_destination_stamp.size() &&
           reserved_destination_stamp[cell] == generation;
  }

  bool endpoint_reserved(int cell) const
  {
    return cell >= 0 &&
           ((cell < (int)reserved_destination_stamp.size() &&
             reserved_destination_stamp[cell] == generation) ||
           (cell < (int)reserved_endpoint_stamp.size() &&
             reserved_endpoint_stamp[cell] == generation));
  }

  bool has_distinct_endpoint_reservations() const
  {
    return distinct_endpoint_reservation_count > 0;
  }

  bool distinct_endpoint_reserved(int cell) const
  {
    return cell >= 0 &&
           cell < (int)reserved_endpoint_stamp.size() &&
           reserved_endpoint_stamp[cell] == generation;
  }

  bool shelf_effect_conflicts(const ShelfSelector& shelf,
                              const TaskId& effect) const
  {
    const int index = shelf_index(shelf);
    return index >= 0 &&
           index < (int)reserved_shelf_stamp.size() &&
           reserved_shelf_stamp[index] == generation &&
           reserved_shelf_to[index] != effect.to;
  }

  bool destination_effect_conflicts(int cell,
                                    const TaskId& effect) const
  {
    if (cell < 0 ||
        cell >= (int)reserved_destination_stamp.size() ||
        reserved_destination_stamp[cell] != generation)
      return false;
    const int index = reserved_destination_shelf[cell];
    return index != shelf_index(effect.shelf);
  }

  bool endpoint_effect_conflicts(int cell,
                                 const TaskId& effect) const
  {
    if (cell >= 0 &&
        cell < (int)reserved_destination_stamp.size() &&
        reserved_destination_stamp[cell] == generation) {
      const int index = reserved_destination_shelf[cell];
      if (index != shelf_index(effect.shelf)) return true;
    }
    if (cell < 0 ||
        cell >= (int)reserved_endpoint_stamp.size() ||
        reserved_endpoint_stamp[cell] != generation)
      return false;
    return reserved_endpoint_shelf[cell] !=
           shelf_index(effect.shelf);
  }

  bool distinct_endpoint_effect_conflicts(
      int cell, const TaskId& effect) const
  {
    if (cell < 0 ||
        cell >= (int)reserved_endpoint_stamp.size() ||
        reserved_endpoint_stamp[cell] != generation)
      return false;
    return reserved_endpoint_shelf[cell] !=
           shelf_index(effect.shelf);
  }

  void reserve_shelf(const ShelfSelector& shelf, const TaskId& effect)
  {
    const int index = shelf_index(shelf);
    if (index < 0 ||
        index >= (int)reserved_shelf_stamp.size() ||
        reserved_shelf_stamp[index] == generation)
      return;
    undo.push_back(Undo{UndoKind::SHELF, index});
    reserved_shelf_to[index] = effect.to;
    reserved_shelf_stamp[index] = generation;
  }

  void reserve_destination(int cell, const TaskId& effect)
  {
    if (cell < 0 ||
        cell >= (int)reserved_destination_stamp.size() ||
        reserved_destination_stamp[cell] == generation)
      return;
    const int index = shelf_index(effect.shelf);
    if (index < 0) return;
    undo.push_back(Undo{UndoKind::DESTINATION, cell});
    reserved_destination_shelf[cell] = index;
    reserved_destination_stamp[cell] = generation;
  }

  void reserve_endpoint(int cell, const TaskId& effect)
  {
    if (cell < 0 ||
        cell >= (int)reservation_cell_count ||
        (cell < (int)reserved_endpoint_stamp.size() &&
         reserved_endpoint_stamp[cell] == generation))
      return;
    if (reserved_endpoint_stamp.size() < reservation_cell_count) {
      reserved_endpoint_shelf.resize(reservation_cell_count);
      reserved_endpoint_stamp.resize(reservation_cell_count, 0);
    }
    if (cell >= (int)reserved_endpoint_stamp.size() ||
        reserved_endpoint_stamp[cell] == generation)
      return;
    const int index = shelf_index(effect.shelf);
    if (index < 0) return;
    undo.push_back(Undo{UndoKind::ENDPOINT, cell});
    reserved_endpoint_shelf[cell] = index;
    reserved_endpoint_stamp[cell] = generation;
    ++distinct_endpoint_reservation_count;
  }

  void merge_task(int, const RootDemand&, int) {}

  int add_task(const TaskId& id,
               const StorageTransferCandidate& transfer,
               int,
               const RootDemand& root, int predecessor, int priority)
  {
    if (predecessor < 0 && !ready_effect.has_value()) {
      ready_effect = id;
      ready_transfer = select_storage_transfer(transfer);
    }
    return 0;
  }

  void rollback(size_t checkpoint)
  {
    while (undo.size() > checkpoint) {
      const Undo entry = undo.back();
      undo.pop_back();
      switch (entry.kind) {
        case UndoKind::SHELF:
          reserved_shelf_stamp[entry.index] = 0;
          break;
        case UndoKind::DESTINATION:
          reserved_destination_stamp[entry.index] = 0;
          break;
        case UndoKind::ENDPOINT:
          reserved_endpoint_stamp[entry.index] = 0;
          --distinct_endpoint_reservation_count;
          break;
      }
    }
  }
};

inline SingleRootReadyResult compile_single_root_next_ready_effect(
    const DDInstance& ins, const AbstractUpperState& upper,
    const RootDemand& root, DDDistCache& upper_wall,
    const TaskBRCompilerLimits& limits,
    SingleRootCompilerScratch& scratch)
{
  TaskBRCompilerBudget budget;
  scratch.reset(upper, ins.grid.size());
  SingleRootReadyResult out;
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, root.target};
  const int result = resolve_shelf_task_br_pibt(
      ins, upper, shelf, root, 1, nullptr, true, upper_wall,
      limits, budget, scratch, scratch.recursion_stack,
      scratch.encountered_rotations);
  if (result >= 0) {
    out.ready_effect = scratch.ready_effect;
    out.ready_transfer = std::move(scratch.ready_transfer);
  }
  out.recursion_calls = budget.recursion_calls;
  out.recursion_exhausted = budget.recursion_exhausted;
  out.effect_conflicts = budget.effect_conflicts;
  out.candidate_backtracks = budget.candidate_backtracks;
  return out;
}

inline SingleRootReadyResult compile_single_root_next_ready_effect(
    const DDInstance& ins, const AbstractUpperState& upper,
    const RootDemand& root, DDDistCache& upper_wall,
    const TaskBRCompilerLimits& limits)
{
  SingleRootCompilerScratch scratch;
  return compile_single_root_next_ready_effect(
      ins, upper, root, upper_wall, limits, scratch);
}

inline void propagate_root_demands(ShelfTaskGraph& graph,
                                   const std::vector<int>& target_priority)
{
  for (size_t offset = graph.tasks.size(); offset-- > 0;) {
    for (const int predecessor : graph.predecessors[offset])
      for (const auto& root : graph.tasks[offset].roots)
        add_root_demand(graph.tasks[predecessor].roots, root);
  }
  for (auto& task : graph.tasks) {
    task.priority = 0;
    for (const auto& root : task.roots)
      if (root.target >= 0 &&
          root.target < (int)target_priority.size())
        task.priority =
            std::max(task.priority, target_priority[root.target]);
  }
}

struct JointCompileCandidate {
  TaskBRCompilerState state;
  std::vector<uint8_t> success;
  std::vector<int> paused;
  std::vector<long long> remaining_mission_by_root;
  long long remaining_mission_distance = 0;
  long long estimated_shelf_cost = 0;
  uint64_t stable_order = 0;
  bool valid = false;
};

inline bool better_joint_candidate(const JointCompileCandidate& candidate,
                                   const JointCompileCandidate& best,
                                   bool compare_full_root_progress = false)
{
  if (!best.valid) return true;
  if (candidate.success != best.success)
    return std::lexicographical_compare(
        best.success.begin(), best.success.end(), candidate.success.begin(),
        candidate.success.end());
  if (candidate.remaining_mission_distance !=
      best.remaining_mission_distance)
    return candidate.remaining_mission_distance <
           best.remaining_mission_distance;
  if (compare_full_root_progress &&
      candidate.remaining_mission_by_root !=
          best.remaining_mission_by_root)
    return std::lexicographical_compare(
        candidate.remaining_mission_by_root.begin(),
        candidate.remaining_mission_by_root.end(),
        best.remaining_mission_by_root.begin(),
        best.remaining_mission_by_root.end());
  const size_t compared_roots = std::min(
      candidate.remaining_mission_by_root.size(),
      best.remaining_mission_by_root.size());
  for (size_t root = 0; root < compared_roots; ++root) {
    const bool candidate_completes =
        candidate.remaining_mission_by_root[root] == 0;
    const bool best_completes =
        best.remaining_mission_by_root[root] == 0;
    if (candidate_completes != best_completes)
      return candidate_completes;
  }
  if (candidate.estimated_shelf_cost !=
      best.estimated_shelf_cost)
    return candidate.estimated_shelf_cost <
           best.estimated_shelf_cost;
  return candidate.stable_order < best.stable_order;
}

inline ShelfTaskGraph compile_task_br_pibt(
    const DDInstance& ins, const AbstractUpperState& upper,
    const std::vector<RootDemand>& requested_roots,
    const std::vector<int>& tau, const std::vector<int>& target_priority,
    DDDistCache& upper_wall, const TaskBRCompilerLimits& limits,
    bool single_root_mode)
{
  std::vector<RootDemand> roots = requested_roots;
  std::stable_sort(roots.begin(), roots.end(), [&](const RootDemand& a,
                                                   const RootDemand& b) {
    const int pa = a.target >= 0 &&
                           a.target < (int)target_priority.size()
                       ? target_priority[a.target]
                       : 0;
    const int pb = b.target >= 0 &&
                           b.target < (int)target_priority.size()
                       ? target_priority[b.target]
                       : 0;
    return pa != pb ? pa > pb : a.target < b.target;
  });
  TaskBRCompilerBudget budget;
  JointCompileCandidate best;
  size_t storage_cells = 0;
  for (int cell = 0; cell < ins.grid.size(); ++cell)
    storage_cells += ins.can_store_shelf(cell);
  if (upper.shelves.size() > storage_cells)
    throw std::logic_error(
        "compile_task_br_pibt: shelves exceed storage cells");
  const size_t vacancy_count =
      storage_cells - upper.shelves.size();
  // With one vacancy, minimizing every non-zero residual makes a dense
  // displacement chain chase tiny distance improvements instead of
  // finishing a root.  Two or more vacancies provide enough independent
  // routing freedom for the full priority-ordered residual vector to be a
  // useful anti-oscillation tie-break.
  const bool compare_full_root_progress = vacancy_count >= 2;

  if (single_root_mode) {
    JointCompileCandidate candidate;
    candidate.valid = true;
    candidate.success.assign(roots.size(), 0);
    TaskBRCompilerTransaction transaction(candidate.state);
    if (!roots.empty() && limits.recursion_cap > 0) {
      std::vector<ShelfSelector> stack;
      std::vector<RotationCandidate> encountered_rotations;
      const ShelfSelector shelf{
          ShelfSelector::Kind::TARGET, roots.front().target};
      const int result = resolve_shelf_task_br_pibt(
          ins, upper, shelf, roots.front(),
          roots.front().target < (int)target_priority.size()
              ? target_priority[roots.front().target]
              : 0,
          &tau, true, upper_wall, limits, budget, transaction, stack,
          encountered_rotations);
      if (result >= 0) {
        candidate.success[0] = 1;
      } else {
        for (const auto& rotation : encountered_rotations)
          transaction.add_rotation(rotation);
        candidate.paused.push_back(roots.front().target);
      }
    } else {
      for (const auto& root : roots)
        candidate.paused.push_back(root.target);
    }
    best = std::move(candidate);
  } else if (limits.backtrack_cap <= 0 || limits.recursion_cap <= 0) {
    best.valid = true;
    best.success.assign(roots.size(), 0);
    for (const auto& root : roots) best.paused.push_back(root.target);
  } else {
    TaskBRCompilerState current;
    TaskBRCompilerTransaction transaction(current);
    std::vector<uint8_t> success(roots.size(), 0);
    std::vector<int> selected_root_to(roots.size(), -1);
    std::vector<int> paused;
    uint64_t candidate_sequence = 0;
    // The hard cap is needed when the first candidate windows fail to
    // produce a jointly executable graph.  Once every root has succeeded
    // in one candidate, however, the success vector is already
    // lexicographically maximal (§6.4); keep only a smaller soft window
    // for the remaining distance/work tie-breaks.
    const int soft_backtrack_cap =
        std::min(limits.backtrack_cap,
                 std::max(1, limits.recursion_cap / 2));
    auto all_roots_succeeded = [&]() {
      return best.valid && best.success.size() == roots.size() &&
             std::all_of(best.success.begin(), best.success.end(),
                         [](uint8_t value) { return value != 0; });
    };
    auto active_backtrack_cap = [&]() {
      return all_roots_succeeded() ? soft_backtrack_cap
                                   : limits.backtrack_cap;
    };
    auto record_candidate = [&](std::vector<int> candidate_paused) {
      JointCompileCandidate candidate;
      candidate.state = current;
      candidate.success = success;
      candidate.paused = std::move(candidate_paused);
      candidate.estimated_shelf_cost =
          (long long)current.graph.tasks.size();
      candidate.stable_order = candidate_sequence++;
      candidate.valid = true;
      candidate.remaining_mission_by_root.assign(roots.size(), 0);
      for (size_t root_index = 0;
           root_index < roots.size(); ++root_index) {
        if (!candidate.success[root_index]) continue;
        const int to = selected_root_to[root_index];
        const int distance =
            to >= 0
                ? upper_wall.dist(
                      roots[root_index].goal, to)
                : INT_MAX;
        const long long finite_distance =
            distance >= INT_MAX / 4
                ? (long long)ins.grid.size() + 1
                : distance;
        candidate.remaining_mission_by_root[root_index] =
            finite_distance;
        candidate.remaining_mission_distance +=
            finite_distance;
        candidate.estimated_shelf_cost += finite_distance;
      }
      if (better_joint_candidate(
              candidate, best, compare_full_root_progress)) {
        best = std::move(candidate);
      }
    };
    std::function<void(size_t)> compile_roots = [&](size_t k) {
      if (k == roots.size()) {
        record_candidate(paused);
        return;
      }
      if (budget.branch_calls >= active_backtrack_cap()) {
        budget.backtrack_exhausted = true;
        auto candidate_paused = paused;
        for (size_t r = k; r < roots.size(); ++r)
          candidate_paused.push_back(roots[r].target);
        record_candidate(std::move(candidate_paused));
        return;
      }
      const auto& root = roots[k];
      const ShelfSelector shelf{
          ShelfSelector::Kind::TARGET, root.target};
      const auto options = ordered_shelf_candidate_window(
          ins, upper, shelf, root, &tau, false, upper_wall,
          transaction);
      std::vector<RotationCandidate> failed_rotations;
      for (int option_index = 0;
           option_index < options.count; ++option_index) {
        if (budget.branch_calls >= active_backtrack_cap()) break;
        const auto transfer =
            options.candidate(option_index);
        if (transfer.route_size < 2) continue;
        const int to = transfer.first_step;
        ++budget.branch_calls;
        if (budget.recursion_calls >= limits.recursion_cap)
          budget.recursion_calls = 0;
        const size_t checkpoint = transaction.checkpoint();
        std::vector<ShelfSelector> stack;
        std::vector<RotationCandidate> option_rotations;
        const int result = resolve_shelf_task_br_pibt(
            ins, upper, shelf, root,
            root.target < (int)target_priority.size()
                ? target_priority[root.target]
                : 0,
            &tau, false, upper_wall, limits, budget, transaction, stack,
            option_rotations, &transfer);
        const TaskId expected{
            shelf, upper.position(shelf), to};
        if (result >= 0 && current.graph.tasks[result].id == expected) {
          success[k] = 1;
          selected_root_to[k] = transfer.endpoint;
          compile_roots(k + 1);
          selected_root_to[k] = -1;
          success[k] = 0;
        } else {
          for (auto& rotation : option_rotations)
            add_unique_rotation_candidate(
                failed_rotations, std::move(rotation));
        }
        transaction.rollback(checkpoint);
      }
      if (budget.branch_calls < active_backtrack_cap()) {
        ++budget.branch_calls;
        const size_t checkpoint = transaction.checkpoint();
        for (const auto& rotation : failed_rotations)
          transaction.add_rotation(rotation);
        paused.push_back(root.target);
        compile_roots(k + 1);
        paused.pop_back();
        transaction.rollback(checkpoint);
      }
    };
    compile_roots(0);
  }

  if (!best.valid) {
    best.valid = true;
    best.success.assign(roots.size(), 0);
    for (const auto& root : roots) best.paused.push_back(root.target);
  }
  auto graph = std::move(best.state.graph);
  std::sort(best.paused.begin(), best.paused.end());
  best.paused.erase(std::unique(best.paused.begin(), best.paused.end()),
                    best.paused.end());
  graph.paused_roots = std::move(best.paused);
  graph.effect_conflicts = budget.effect_conflicts;
  graph.candidate_backtracks = budget.candidate_backtracks;
  propagate_root_demands(graph, target_priority);
  return graph;
}

inline const PairPlan* selected_pair_plan(
    const PairCostTable& table, const std::vector<int>& tau, int target)
{
  if (target < 0 || target >= (int)table.size() ||
      target >= (int)tau.size())
    return nullptr;
  for (const auto& entry : table[target])
    if (entry.goal == tau[target]) return &entry.plan;
  return nullptr;
}

inline std::vector<int> target_priorities_from_pair_cost(
    const PairCostTable& table, const std::vector<int>& tau)
{
  std::vector<int> order(table.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    const PairPlan* plan_a = selected_pair_plan(table, tau, a);
    const PairPlan* plan_b = selected_pair_plan(table, tau, b);
    if ((plan_a != nullptr && !plan_a->exact) ||
        (plan_b != nullptr && !plan_b->exact))
      throw std::logic_error(
          "target priority cannot consume an inexact PairCost bound");
    const double ca =
        plan_a != nullptr
            ? plan_a->estimated_cost
            : std::numeric_limits<double>::infinity();
    const double cb =
        plan_b != nullptr
            ? plan_b->estimated_cost
            : std::numeric_limits<double>::infinity();
    return ca != cb ? ca > cb : a < b;
  });
  std::vector<int> priority(table.size(), 0);
  for (size_t rank = 0; rank < order.size(); ++rank)
    priority[order[rank]] = (int)order.size() - (int)rank;
  return priority;
}

inline std::vector<int> ready_tasks(const DDInstance& ins,
                                    const PhysConfig& physical,
                                    const ShelfTaskGraph& graph)
{
  const auto upper = make_upper_signature(physical);
  std::vector<uint8_t> occupied(ins.grid.size(), 0);
  for (const int cell : upper.target_pos) occupied[cell] = 1;
  for (const int cell : upper.anon_pos) occupied[cell] = 1;
  std::vector<uint8_t> carried_target(ins.n_targets(), 0);
  bool any_carried_anon = false;
  for (const int k : physical.kappa) {
    if (k >= 0 && k < (int)ins.n_targets()) carried_target[k] = 1;
    if (k == KAPPA_ANON) any_carried_anon = true;
  }
  std::vector<int> ready;
  for (size_t index = 0; index < graph.tasks.size(); ++index) {
    const auto& task = graph.tasks[index];
    if (!graph.predecessors[index].empty()) continue;
    bool shelf_grounded = false;
    if (task.id.shelf.kind == ShelfSelector::Kind::TARGET) {
      const int target = task.id.shelf.value;
      shelf_grounded =
          target >= 0 && target < (int)physical.target_pos.size() &&
          !carried_target[target] &&
          physical.target_pos[target] == task.id.from;
    } else {
      shelf_grounded =
          !any_carried_anon &&
          std::binary_search(physical.anon_occ.begin(),
                             physical.anon_occ.end(), task.id.from) &&
          task.id.shelf.value == task.id.from;
    }
    if (!shelf_grounded || occupied[task.id.to]) continue;
    ready.push_back((int)index);
  }
  std::stable_sort(ready.begin(), ready.end(), [&](int a, int b) {
    if (graph.tasks[a].priority != graph.tasks[b].priority)
      return graph.tasks[a].priority > graph.tasks[b].priority;
    return graph.tasks[a].id < graph.tasks[b].id;
  });
  return ready;
}

struct PairEpisodeCost {
  std::optional<ShelfSelector> open_shelf;
  double value = 0;

  void apply_shift(const ShelfSelector& shelf, double alpha,
                   double gamma, double delta)
  {
    if (!open_shelf.has_value() || *open_shelf != shelf) {
      if (open_shelf.has_value()) value += gamma;
      value += gamma;
      open_shelf = shelf;
    }
    value += alpha;
    if (shelf.kind == ShelfSelector::Kind::ANON_AT_EPOCH_CELL)
      value += delta;
  }

  double finish(double gamma)
  {
    if (open_shelf.has_value()) {
      value += gamma;
      open_shelf.reset();
    }
    return value;
  }
};

inline double pair_episode_cost(
    const std::vector<ShelfSelector>& shifted_shelves, double alpha,
    double gamma, double delta)
{
  PairEpisodeCost cost;
  for (const auto& shelf : shifted_shelves)
    cost.apply_shift(shelf, alpha, gamma, delta);
  return cost.finish(gamma);
}

inline PairPlan pair_cost_prefix_lower_bound(
    const DDInstance& ins, const UpperSignature& upper, int target,
    int goal, DDDistCache& upper_wall, double alpha, double gamma,
    double delta, int prefix_cap)
{
  PairPlan out;
  if (!eligible_goal(ins, target, goal)) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return out;
  }
  const int initial_distance =
      upper_wall.dist(goal, upper.target_pos[target]);
  out.direct_distance =
      initial_distance >= INT_MAX / 4 ? INT_MAX : initial_distance;
  if (initial_distance >= INT_MAX / 4) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return out;
  }
  if (upper.target_pos[target] == goal) {
    out.reached_goal = true;
    return out;
  }

  auto abstract = make_abstract_upper_state(ins, upper);
  const TaskBRCompilerLimits limits{
      std::max(32, 4 * ins.grid.size()),
      std::max(64, 8 * ins.grid.size())};
  const RootDemand root{target, goal};
  PairEpisodeCost episode_cost;
  SingleRootCompilerScratch compiler_scratch;
  const int effective_prefix_cap = std::max(0, prefix_cap);
  while (out.rollout_steps < effective_prefix_cap) {
    const auto next = compile_single_root_next_ready_effect(
        ins, abstract, root, upper_wall, limits, compiler_scratch);
    if (!next.ready_effect.has_value() ||
        !next.ready_transfer.has_value()) {
      out.stalled = true;
      break;
    }
    const auto& effect = *next.ready_effect;
    const auto& transfer = *next.ready_transfer;
    const int legs = transfer.route_size - 1;
    if (legs <= 0 ||
        out.rollout_steps + legs > effective_prefix_cap)
      break;
    if (transfer.explicit_route.empty()) {
      episode_cost.apply_shift(
          effect.shelf, alpha, gamma, delta);
      abstract.move(effect.shelf, transfer.first_step);
      ++out.rollout_steps;
    } else {
      const auto& route = transfer.explicit_route;
      for (size_t index = 1;
           index < route.size(); ++index) {
        episode_cost.apply_shift(
            effect.shelf, alpha, gamma, delta);
        abstract.move(effect.shelf, route[index]);
        ++out.rollout_steps;
      }
    }
    if (abstract.positions[target] == goal) {
      out.reached_goal = true;
      break;
    }
  }

  if (out.reached_goal || out.stalled) {
    out.estimated_cost = episode_cost.finish(gamma);
    if (!out.reached_goal) {
      const int remaining =
          upper_wall.dist(goal, abstract.positions[target]);
      if (remaining < INT_MAX / 4)
        out.estimated_cost +=
            alpha * (double)remaining + 2.0 * gamma;
      out.estimated_cost +=
          alpha * (double)(ins.grid.size() + 1) + 2.0 * gamma;
    }
    return out;
  }

  const int remaining =
      upper_wall.dist(goal, abstract.positions[target]);
  out.estimated_cost = episode_cost.value;
  if (remaining < INT_MAX / 4)
    out.estimated_cost += alpha * (double)remaining;
  const ShelfSelector target_shelf{
      ShelfSelector::Kind::TARGET, target};
  if (!episode_cost.open_shelf.has_value()) {
    out.estimated_cost += 2.0 * gamma;
  } else if (*episode_cost.open_shelf == target_shelf) {
    out.estimated_cost += gamma;
  } else {
    out.estimated_cost += 3.0 * gamma;
  }
  out.exact = false;
  return out;
}

inline PairPlan pair_cost(const DDInstance& ins, const UpperSignature& upper,
                          int target, int goal, DDDistCache& upper_wall,
                          double alpha, double gamma, double delta)
{
  PairPlan out;
  if (!eligible_goal(ins, target, goal)) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return out;
  }
  const int initial_distance =
      upper_wall.dist(goal, upper.target_pos[target]);
  out.direct_distance =
      initial_distance >= INT_MAX / 4 ? INT_MAX : initial_distance;
  if (initial_distance >= INT_MAX / 4) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return out;
  }
  if (upper.target_pos[target] == goal) {
    out.reached_goal = true;
    return out;
  }

  auto abstract = make_abstract_upper_state(ins, upper);
  const int step_cap = std::max(8, std::min(128, 2 * ins.grid.size()));
  const TaskBRCompilerLimits limits{
      std::max(32, 4 * ins.grid.size()),
      std::max(64, 8 * ins.grid.size())};
  const RootDemand root{target, goal};
  PairEpisodeCost episode_cost;
  SingleRootCompilerScratch compiler_scratch;
  while (out.rollout_steps < step_cap) {
    if (abstract.positions[target] == goal) {
      out.reached_goal = true;
      break;
    }
    const auto next = compile_single_root_next_ready_effect(
        ins, abstract, root, upper_wall, limits, compiler_scratch);
    if (!next.ready_effect.has_value() ||
        !next.ready_transfer.has_value()) {
      out.stalled = true;
      break;
    }
    const auto& effect = *next.ready_effect;
    const auto& transfer = *next.ready_transfer;
    const int legs = transfer.route_size - 1;
    if (legs <= 0 || out.rollout_steps + legs > step_cap) {
      out.truncated = true;
      break;
    }
    if (transfer.explicit_route.empty()) {
      episode_cost.apply_shift(
          effect.shelf, alpha, gamma, delta);
      abstract.move(effect.shelf, transfer.first_step);
      ++out.rollout_steps;
    } else {
      const auto& route = transfer.explicit_route;
      for (size_t index = 1;
           index < route.size(); ++index) {
        episode_cost.apply_shift(
            effect.shelf, alpha, gamma, delta);
        abstract.move(effect.shelf, route[index]);
        ++out.rollout_steps;
      }
    }
    if (abstract.positions[target] == goal) {
      out.reached_goal = true;
      break;
    }
  }
  if (!out.reached_goal && !out.stalled &&
      out.rollout_steps >= step_cap)
    out.truncated = true;
  out.estimated_cost += episode_cost.finish(gamma);
  if (!out.reached_goal) {
    const int remaining =
        upper_wall.dist(goal, abstract.positions[target]);
    if (remaining < INT_MAX / 4)
      out.estimated_cost += alpha * (double)remaining + 2.0 * gamma;
    const double finite_penalty =
        alpha * (double)(ins.grid.size() + 1) + 2.0 * gamma;
    if (out.stalled || out.truncated)
      out.estimated_cost += finite_penalty;
  }
  return out;
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

inline int task_index_by_id(const ShelfTaskGraph& graph, const TaskId& id)
{
  for (size_t i = 0; i < graph.tasks.size(); ++i)
    if (graph.tasks[i].id == id) return (int)i;
  return -1;
}

inline StorageTransfer normalized_transfer(const ShelfTask& task)
{
  if (task.transfer.route.size() >= 2 &&
      task.transfer.route.front() == task.id.from &&
      task.transfer.route[1] == task.id.to &&
      task.transfer.route.back() == task.transfer.endpoint)
    return task.transfer;
  return StorageTransfer{
      task.id.to, {task.id.from, task.id.to}};
}

inline StorageTransfer normalized_transfer(const Custody& custody)
{
  if (custody.transfer.route.size() >= 2 &&
      custody.transfer.route.back() ==
          custody.transfer.endpoint &&
      custody.transfer_index + 1 <
          custody.transfer.route.size() &&
      custody.transfer.route[custody.transfer_index] ==
          custody.from &&
      custody.transfer.route[custody.transfer_index + 1] ==
          custody.to)
    return custody.transfer;
  return StorageTransfer{
      custody.to, {custody.from, custody.to}};
}

inline void reanchor_anonymous_custody(Custody& custody)
{
  if (custody.shelf.kind !=
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL)
    return;
  custody.shelf.value = custody.from;
  custody.task_id =
      TaskId{custody.shelf, custody.from, custody.to};
}

inline bool task_matches_active_transfer(
    const ShelfTask& task, const Custody& custody)
{
  if (task.id != custody.task_id) return false;
  const auto active = normalized_transfer(custody);
  const auto candidate = normalized_transfer(task);
  if (candidate.endpoint != active.endpoint ||
      custody.transfer_index >= active.route.size())
    return false;
  return candidate.route.size() ==
             active.route.size() - custody.transfer_index &&
         std::equal(
             candidate.route.begin(), candidate.route.end(),
             active.route.begin() + custody.transfer_index);
}

inline int compatible_task_index_by_custody(
    const ShelfTaskGraph& graph, const Custody& custody)
{
  const int index = task_index_by_id(graph, custody.task_id);
  return index >= 0 &&
                 task_matches_active_transfer(
                     graph.tasks[index], custody)
             ? index
             : -1;
}

struct ActiveTransferClaims {
  std::vector<uint8_t> endpoint;
  std::vector<uint8_t> transit;

  explicit ActiveTransferClaims(size_t cell_count = 0)
      : endpoint(cell_count, 0), transit(cell_count, 0)
  {
  }
};

inline bool transfer_conflicts_with_claims(
    const ActiveTransferClaims& claims,
    const StorageTransfer& transfer)
{
  if (transfer.endpoint < 0 ||
      transfer.endpoint >= (int)claims.endpoint.size() ||
      claims.endpoint[transfer.endpoint])
    return true;
  for (size_t index = 1; index + 1 < transfer.route.size(); ++index) {
    const int cell = transfer.route[index];
    if (cell < 0 || cell >= (int)claims.transit.size() ||
        claims.transit[cell])
      return true;
  }
  return false;
}

inline void add_transfer_claim(
    ActiveTransferClaims& claims,
    const StorageTransfer& transfer)
{
  if (transfer.endpoint >= 0 &&
      transfer.endpoint < (int)claims.endpoint.size())
    claims.endpoint[transfer.endpoint] = 1;
  for (size_t index = 1; index + 1 < transfer.route.size(); ++index) {
    const int cell = transfer.route[index];
    if (cell >= 0 && cell < (int)claims.transit.size())
      claims.transit[cell] = 1;
  }
}

inline StorageTransfer remaining_transfer(const Custody& custody)
{
  const auto transfer = normalized_transfer(custody);
  if (custody.transfer_index >= transfer.route.size())
    return StorageTransfer{};
  return StorageTransfer{
      transfer.endpoint,
      std::vector<int>(
          transfer.route.begin() + custody.transfer_index,
          transfer.route.end())};
}

inline ActiveTransferClaims active_transfer_claims(
    int cell_count,
    const std::vector<std::optional<Custody>>& custody_by_robot)
{
  ActiveTransferClaims claims(cell_count);
  for (const auto& custody : custody_by_robot)
    if (custody.has_value())
      add_transfer_claim(claims, remaining_transfer(*custody));
  return claims;
}

inline Custody make_custody(const ShelfTask& task, int task_index)
{
  Custody out;
  out.task_id = task.id;
  out.current_task_index =
      task_index >= 0 ? std::optional<int>(task_index) : std::nullopt;
  out.shelf = task.id.shelf;
  out.from = task.id.from;
  out.to = task.id.to;
  out.roots = task.roots;
  out.priority = task.priority;
  out.transfer = normalized_transfer(task);
  out.transfer_index = 0;
  return out;
}

inline bool task_matches_loaded_shelf(const PhysConfig& physical, int robot,
                                      const TaskId& id)
{
  if (robot < 0 || robot >= (int)physical.robots.size() ||
      robot >= (int)physical.kappa.size() ||
      physical.robots[robot] != id.from)
    return false;
  if (id.shelf.kind == ShelfSelector::Kind::TARGET)
    return physical.kappa[robot] == id.shelf.value;
  return physical.kappa[robot] == KAPPA_ANON;
}

inline bool adjacent_cells(const DDGrid& grid, int from, int to)
{
  int neighbors[4];
  const int count = grid.neighbors(from, neighbors);
  for (int i = 0; i < count; ++i)
    if (neighbors[i] == to) return true;
  return false;
}

inline bool custody_physically_valid(const DDInstance& ins,
                                     const PhysConfig& physical, int robot,
                                     const Custody& custody)
{
  if (custody.task_id !=
      TaskId{custody.shelf, custody.from, custody.to})
    return false;
  if (!task_matches_loaded_shelf(physical, robot, custody.task_id) ||
      !adjacent_cells(ins.grid, custody.from, custody.to))
    return false;
  const auto transfer = normalized_transfer(custody);
  if (transfer.route.size() < 2 ||
      transfer.route.back() != transfer.endpoint ||
      !ins.can_store_shelf(transfer.endpoint) ||
      custody.transfer_index + 1 >= transfer.route.size() ||
      transfer.route[custody.transfer_index] != custody.from ||
      transfer.route[custody.transfer_index + 1] != custody.to)
    return false;
  for (size_t index = 1; index < transfer.route.size(); ++index)
    if (!adjacent_cells(
            ins.grid, transfer.route[index - 1],
            transfer.route[index]))
      return false;
  const auto upper = make_upper_signature(physical);
  const bool destination_empty =
      std::find(upper.target_pos.begin(), upper.target_pos.end(),
                custody.to) == upper.target_pos.end() &&
      !std::binary_search(upper.anon_pos.begin(), upper.anon_pos.end(),
                          custody.to);
  return transfer.route.size() > 2 || destination_empty;
}

inline bool task_shelf_is_grounded(const DDInstance& ins,
                                   const PhysConfig& physical,
                                   const TaskId& id)
{
  if (id.shelf.kind == ShelfSelector::Kind::TARGET) {
    const int target = id.shelf.value;
    if (target < 0 || target >= (int)ins.n_targets() ||
        physical.target_pos[target] != id.from)
      return false;
    return std::find(physical.kappa.begin(), physical.kappa.end(),
                     target) == physical.kappa.end();
  }
  return id.shelf.value == id.from &&
         std::binary_search(physical.anon_occ.begin(),
                            physical.anon_occ.end(), id.from);
}

inline int carrier_of_task_shelf(const DDInstance& ins,
                                 const PhysConfig& physical,
                                 const TaskId& id)
{
  for (size_t robot = 0; robot < physical.kappa.size(); ++robot) {
    if (physical.robots[robot] != id.from) continue;
    if (id.shelf.kind == ShelfSelector::Kind::TARGET) {
      if (id.shelf.value >= 0 &&
          id.shelf.value < (int)ins.n_targets() &&
          physical.kappa[robot] == id.shelf.value)
        return (int)robot;
    } else if (physical.kappa[robot] == KAPPA_ANON &&
               id.shelf.value == id.from) {
      return (int)robot;
    }
  }
  return -1;
}

inline std::vector<int> ready_tasks_with_custody(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph,
    const std::vector<std::optional<Custody>>& custody_by_robot,
    const std::vector<uint8_t>& continuation_carrier)
{
  const auto upper = make_upper_signature(physical);
  std::vector<uint8_t> occupied(ins.grid.size(), 0);
  for (const int cell : upper.target_pos)
    if (cell >= 0 && cell < (int)occupied.size()) occupied[cell] = 1;
  for (const int cell : upper.anon_pos)
    if (cell >= 0 && cell < (int)occupied.size()) occupied[cell] = 1;

  std::unordered_map<TaskId, int, TaskIdHash> custody_owner;
  for (size_t robot = 0; robot < custody_by_robot.size(); ++robot)
    if (custody_by_robot[robot].has_value())
      custody_owner[custody_by_robot[robot]->task_id] = (int)robot;

  std::vector<int> ready;
  for (size_t index = 0; index < graph.tasks.size(); ++index) {
    const auto& task = graph.tasks[index];
    if (index >= graph.predecessors.size() ||
        !graph.predecessors[index].empty())
      continue;
    if (task.id.to < 0 || task.id.to >= (int)occupied.size() ||
        occupied[task.id.to] || custody_owner.count(task.id))
      continue;

    bool shelf_available =
        task_shelf_is_grounded(ins, physical, task.id);
    if (!shelf_available) {
      const int carrier =
          carrier_of_task_shelf(ins, physical, task.id);
      shelf_available =
          carrier >= 0 &&
          carrier < (int)continuation_carrier.size() &&
          continuation_carrier[carrier] &&
          carrier < (int)custody_by_robot.size() &&
          !custody_by_robot[carrier].has_value();
    }
    if (shelf_available) ready.push_back((int)index);
  }
  std::stable_sort(ready.begin(), ready.end(), [&](int a, int b) {
    if (graph.tasks[a].priority != graph.tasks[b].priority)
      return graph.tasks[a].priority > graph.tasks[b].priority;
    return graph.tasks[a].id < graph.tasks[b].id;
  });
  auto claims = active_transfer_claims(
      ins.grid.size(), custody_by_robot);
  std::vector<int> filtered;
  filtered.reserve(ready.size());
  for (const int index : ready) {
    const auto transfer =
        normalized_transfer(graph.tasks[index]);
    if (transfer_conflicts_with_claims(claims, transfer))
      continue;
    filtered.push_back(index);
    add_transfer_claim(claims, transfer);
  }
  return filtered;
}

struct CustodyRecovery {
  std::vector<std::optional<Custody>> custody_by_robot;
  std::vector<uint8_t> continuation_carrier;
  std::vector<int> previous_loaded_move_from;
  bool transition_valid = false;
};

inline bool exact_ready_binding(const CarrierGuidance& guidance,
                                const TaskId& id, int* task_index)
{
  if (guidance.upper_epoch == nullptr) return false;
  const int index =
      task_index_by_id(guidance.upper_epoch->task_graph, id);
  if (index < 0 ||
      std::find(guidance.ready_tasks.begin(), guidance.ready_tasks.end(),
                index) == guidance.ready_tasks.end())
    return false;
  if (task_index != nullptr) *task_index = index;
  return true;
}

inline std::optional<Custody> make_storage_recovery_custody(
    const DDInstance& ins, const PhysConfig& physical, int robot,
    const ShelfTaskGraph& current_graph,
    const std::optional<Custody>& previous_custody,
    const ActiveTransferClaims& claims)
{
  if (robot < 0 || robot >= (int)physical.robots.size() ||
      robot >= (int)physical.kappa.size() ||
      physical.kappa[robot] == KAPPA_FREE ||
      ins.can_store_shelf(physical.robots[robot]))
    return std::nullopt;
  const auto upper_signature = make_upper_signature(physical);
  const auto upper =
      make_abstract_upper_state(ins, upper_signature);
  auto transfers = reachable_storage_transfers(
      ins, upper, physical.robots[robot]);
  transfers.erase(
      std::remove_if(
          transfers.begin(), transfers.end(),
          [&](const StorageTransfer& transfer) {
            return transfer.route.size() < 2 ||
                   !upper.empty(transfer.endpoint);
          }),
      transfers.end());
  if (transfers.empty()) return std::nullopt;
  std::stable_sort(
      transfers.begin(), transfers.end(),
      [](const StorageTransfer& a, const StorageTransfer& b) {
        return a.route.size() != b.route.size()
                   ? a.route.size() < b.route.size()
                   : a.endpoint < b.endpoint;
      });
  const auto selected = std::find_if(
      transfers.begin(), transfers.end(),
      [&](const StorageTransfer& transfer) {
        return !transfer_conflicts_with_claims(
            claims, transfer);
      });
  if (selected == transfers.end()) return std::nullopt;

  Custody custody;
  if (previous_custody.has_value()) {
    custody.shelf = previous_custody->shelf;
    custody.roots = previous_custody->roots;
    custody.priority = previous_custody->priority;
  } else if (physical.kappa[robot] >= 0) {
    custody.shelf = ShelfSelector{
        ShelfSelector::Kind::TARGET, physical.kappa[robot]};
  } else {
    custody.shelf = ShelfSelector{
        ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
        physical.robots[robot]};
  }
  custody.transfer = *selected;
  custody.transfer_index = 0;
  custody.from = custody.transfer.route[0];
  custody.to = custody.transfer.route[1];
  reanchor_anonymous_custody(custody);
  custody.task_id =
      TaskId{custody.shelf, custody.from, custody.to};
  const int index =
      compatible_task_index_by_custody(
          current_graph, custody);
  custody.current_task_index =
      index >= 0 ? std::optional<int>(index) : std::nullopt;
  if (index >= 0) {
    custody.roots = current_graph.tasks[index].roots;
    custody.priority = current_graph.tasks[index].priority;
  }
  if (!custody_physically_valid(ins, physical, robot, custody))
    return std::nullopt;
  return custody;
}

inline CustodyRecovery recover_task_br_custody(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& current_graph, const PhysConfig* previous_physical,
    const CarrierGuidance* previous_guidance,
    const std::vector<Op>* executed_ops)
{
  CustodyRecovery out;
  const size_t robot_count = ins.n_robots();
  out.custody_by_robot.resize(robot_count);
  out.continuation_carrier.assign(robot_count, 0);
  out.previous_loaded_move_from.assign(robot_count, -1);
  if (previous_physical == nullptr || executed_ops == nullptr ||
      previous_physical->robots.size() != robot_count ||
      previous_physical->kappa.size() != robot_count ||
      executed_ops->size() != robot_count)
    return out;
  const auto replayed = apply_ops(ins, *previous_physical, *executed_ops);
  if (!replayed.has_value() || !(*replayed == physical)) return out;
  out.transition_valid = true;

  for (size_t robot = 0; robot < robot_count; ++robot) {
    if (physical.kappa[robot] == KAPPA_FREE) continue;
    const auto& op = (*executed_ops)[robot];
    const int previous_kappa = previous_physical->kappa[robot];

    if (previous_kappa != KAPPA_FREE && op.kind == Op::MOVE) {
      out.continuation_carrier[robot] = 1;
      out.previous_loaded_move_from[robot] =
          previous_physical->robots[robot];
      const std::optional<Custody> previous_custody =
          previous_guidance != nullptr &&
                  robot <
                      previous_guidance->custody_by_robot.size()
              ? previous_guidance->custody_by_robot[robot]
              : std::nullopt;
      if (previous_custody.has_value() &&
          previous_physical->robots[robot] ==
              previous_custody->from &&
          op.to == previous_custody->to &&
          physical.robots[robot] == previous_custody->to) {
        Custody custody = *previous_custody;
        const auto transfer = normalized_transfer(custody);
        const size_t next_index = custody.transfer_index + 1;
        if (next_index + 1 < transfer.route.size()) {
          custody.transfer = transfer;
          custody.transfer_index = next_index;
          custody.from = transfer.route[next_index];
          custody.to = transfer.route[next_index + 1];
          reanchor_anonymous_custody(custody);
          custody.task_id =
              TaskId{custody.shelf, custody.from, custody.to};
          const int current_index =
              compatible_task_index_by_custody(
                  current_graph, custody);
          custody.current_task_index =
              current_index >= 0
                  ? std::optional<int>(current_index)
                  : std::nullopt;
          if (current_index >= 0) {
            custody.roots =
                current_graph.tasks[current_index].roots;
            custody.priority =
                current_graph.tasks[current_index].priority;
          }
          if (custody_physically_valid(
                  ins, physical, (int)robot, custody)) {
            out.custody_by_robot[robot] = std::move(custody);
            out.continuation_carrier[robot] = 0;
          }
        }
      }
      continue;
    }

    if (previous_kappa != KAPPA_FREE && op.kind == Op::WAIT &&
        previous_guidance != nullptr &&
        robot < previous_guidance->custody_by_robot.size() &&
        previous_guidance->custody_by_robot[robot].has_value()) {
      Custody custody =
          *previous_guidance->custody_by_robot[robot];
      reanchor_anonymous_custody(custody);
      if (!custody_physically_valid(ins, physical, (int)robot, custody))
        continue;
      const int current_index =
          compatible_task_index_by_custody(
              current_graph, custody);
      custody.current_task_index =
          current_index >= 0 ? std::optional<int>(current_index)
                             : std::nullopt;
      if (current_index >= 0) {
        const auto& current_task = current_graph.tasks[current_index];
        custody.roots = current_task.roots;
        custody.priority = current_task.priority;
      }
      out.custody_by_robot[robot] = std::move(custody);
      continue;
    }

    if (previous_kappa == KAPPA_FREE && op.kind == Op::LIFT &&
        previous_guidance != nullptr &&
        robot < previous_guidance->rho_task_id.size() &&
        previous_guidance->rho_task_id[robot].has_value()) {
      const TaskId id = *previous_guidance->rho_task_id[robot];
      int previous_index = -1;
      if (!exact_ready_binding(*previous_guidance, id,
                               &previous_index) ||
          !task_matches_loaded_shelf(physical, (int)robot, id))
        continue;
      const auto& previous_task =
          previous_guidance->upper_epoch->task_graph.tasks[previous_index];
      Custody custody = make_custody(previous_task, -1);
      const int current_index =
          compatible_task_index_by_custody(
              current_graph, custody);
      custody.current_task_index =
          current_index >= 0 ? std::optional<int>(current_index)
                             : std::nullopt;
      if (current_index >= 0) {
        custody.roots = current_graph.tasks[current_index].roots;
        custody.priority = current_graph.tasks[current_index].priority;
      }
      if (custody_physically_valid(ins, physical, (int)robot, custody))
        out.custody_by_robot[robot] = std::move(custody);
    }
  }

  auto claims = active_transfer_claims(
      ins.grid.size(), out.custody_by_robot);
  struct RecoveryCandidate {
    size_t robot = 0;
    int priority = 0;
    std::optional<Custody> previous_custody;
  };
  std::vector<RecoveryCandidate> recovery_candidates;
  for (size_t robot = 0; robot < robot_count; ++robot) {
    if (physical.kappa[robot] == KAPPA_FREE ||
        out.custody_by_robot[robot].has_value() ||
        ins.can_store_shelf(physical.robots[robot]))
      continue;
    const std::optional<Custody> previous_custody =
        previous_guidance != nullptr &&
                robot < previous_guidance->custody_by_robot.size()
            ? previous_guidance->custody_by_robot[robot]
            : std::nullopt;
    recovery_candidates.push_back(
        RecoveryCandidate{
            robot,
            previous_custody.has_value()
                ? previous_custody->priority
                : 0,
            previous_custody});
  }
  std::stable_sort(
      recovery_candidates.begin(), recovery_candidates.end(),
      [](const RecoveryCandidate& a,
         const RecoveryCandidate& b) {
        return a.priority != b.priority
                   ? a.priority > b.priority
                   : a.robot < b.robot;
      });
  for (const auto& candidate : recovery_candidates) {
    const auto custody =
        make_storage_recovery_custody(
            ins, physical, (int)candidate.robot,
            current_graph, candidate.previous_custody,
            claims);
    if (!custody.has_value()) continue;
    out.custody_by_robot[candidate.robot] = custody;
    add_transfer_claim(
        claims, remaining_transfer(*custody));
    out.continuation_carrier[candidate.robot] = 0;
  }
  return out;
}

inline void bind_ready_continuations(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph, const std::vector<int>& ready_tasks,
    const std::vector<uint8_t>& continuation_carrier,
    const std::vector<int>& previous_loaded_move_from,
    std::vector<std::optional<Custody>>& custody_by_robot)
{
  const bool suppress_immediate_reverse =
      !target_dense_upper_layout(
          ins, make_upper_signature(physical));
  for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
    if (robot >= continuation_carrier.size() ||
        !continuation_carrier[robot] ||
        physical.kappa[robot] == KAPPA_FREE ||
        custody_by_robot[robot].has_value())
      continue;
    for (const int index : ready_tasks) {
      if (index < 0 || index >= (int)graph.tasks.size()) continue;
      const auto& task = graph.tasks[index];
      if (!task_matches_loaded_shelf(physical, (int)robot, task.id))
        continue;
      if (suppress_immediate_reverse &&
          robot < previous_loaded_move_from.size() &&
          previous_loaded_move_from[robot] >= 0 &&
          task.id.to == previous_loaded_move_from[robot])
        continue;
      custody_by_robot[robot] = make_custody(task, index);
      break;
    }
  }
}

struct RhoCandidate {
  int task_index = -1;
  TaskId id;
  int priority = 0;
  bool mandatory = false;
};

inline DDReadyMatchProbe match_ready_tasks(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph, const std::vector<int>& ready_tasks,
    const std::vector<std::optional<TaskId>>* previous_rho_task_id)
{
  DDReadyMatchProbe out;
  const size_t robot_count = ins.n_robots();
  out.rho_task_id.resize(robot_count);
  out.rho_ready_index.assign(robot_count, -1);

  std::vector<int> free_robots;
  for (size_t robot = 0; robot < robot_count; ++robot)
    if (physical.kappa[robot] == KAPPA_FREE)
      free_robots.push_back((int)robot);
  if (free_robots.empty()) return out;

  std::vector<RhoCandidate> candidates;
  std::unordered_map<TaskId, int, TaskIdHash> seen;
  for (const int index : ready_tasks) {
    if (index < 0 || index >= (int)graph.tasks.size()) continue;
    const auto& task = graph.tasks[index];
    auto it = seen.find(task.id);
    if (it != seen.end()) {
      if (task.priority > candidates[it->second].priority) {
        candidates[it->second].task_index = index;
        candidates[it->second].priority = task.priority;
      }
      continue;
    }
    seen.emplace(task.id, (int)candidates.size());
    candidates.push_back(
        RhoCandidate{index, task.id, task.priority, false});
  }
  if (candidates.empty()) return out;
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const RhoCandidate& a, const RhoCandidate& b) {
                     if (a.priority != b.priority)
                       return a.priority > b.priority;
                     if (a.id != b.id) return a.id < b.id;
                     return a.task_index < b.task_index;
                   });

  const size_t free_count = free_robots.size();
  if (candidates.size() > free_count) {
    const int cutoff = candidates[free_count - 1].priority;
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
                       [&](const RhoCandidate& candidate) {
                         return candidate.priority < cutoff;
                       }),
        candidates.end());
    for (auto& candidate : candidates)
      candidate.mandatory = candidate.priority > cutoff;
  } else {
    for (auto& candidate : candidates) candidate.mandatory = true;
  }

  const size_t task_count = candidates.size();
  const size_t dummy_count =
      task_count > free_count ? task_count - free_count : 0;
  const size_t column_count = free_count + dummy_count;
  constexpr long long INF = std::numeric_limits<long long>::max() / 16;
  const long long switch_scale = (long long)free_count + 1;
  LowerDist lower_distance(ins.grid);
  std::vector<std::vector<long long>> cost(
      task_count, std::vector<long long>(column_count, INF));
  for (size_t row = 0; row < task_count; ++row) {
    for (size_t col = 0; col < free_count; ++col) {
      const int robot = free_robots[col];
      const int distance =
          lower_distance.dist(candidates[row].id.from,
                              physical.robots[robot]);
      if (distance >= INT_MAX / 4) continue;
      const bool switched =
          previous_rho_task_id != nullptr &&
          robot < (int)previous_rho_task_id->size() &&
          (*previous_rho_task_id)[robot].has_value() &&
          *(*previous_rho_task_id)[robot] != candidates[row].id;
      cost[row][col] =
          (long long)distance * switch_scale + (switched ? 1 : 0);
    }
    for (size_t col = free_count; col < column_count; ++col)
      if (!candidates[row].mandatory) cost[row][col] = 0;
  }

  auto minimum_cost =
      [&](const std::vector<int>& rows,
          const std::vector<int>& cols) -> std::optional<long long> {
    if (rows.empty()) return 0;
    if (rows.size() > cols.size()) return std::nullopt;
    constexpr long double HINF = 1e60L;
    std::vector<std::vector<long double>> matrix(
        rows.size(), std::vector<long double>(cols.size(), HINF));
    for (size_t r = 0; r < rows.size(); ++r)
      for (size_t c = 0; c < cols.size(); ++c)
        if (cost[rows[r]][cols[c]] < INF)
          matrix[r][c] = (long double)cost[rows[r]][cols[c]];
    const auto assignment = hungarian_long_double(matrix);
    if (!assignment.feasible) return std::nullopt;
    long long total = 0;
    for (size_t r = 0; r < rows.size(); ++r) {
      const int local_col = assignment.row_to_col[r];
      if (local_col < 0 ||
          cost[rows[r]][cols[local_col]] >= INF)
        return std::nullopt;
      total += cost[rows[r]][cols[local_col]];
    }
    return total;
  };

  std::vector<int> active_rows(task_count);
  std::iota(active_rows.begin(), active_rows.end(), 0);
  std::vector<int> active_cols(column_count);
  std::iota(active_cols.begin(), active_cols.end(), 0);
  auto remaining_optimum = minimum_cost(active_rows, active_cols);
  if (!remaining_optimum.has_value()) return out;

  for (size_t real_col = 0; real_col < free_count; ++real_col) {
    const auto col_it =
        std::find(active_cols.begin(), active_cols.end(), (int)real_col);
    if (col_it == active_cols.end()) continue;
    std::vector<int> row_options = active_rows;
    std::stable_sort(row_options.begin(), row_options.end(),
                     [&](int a, int b) {
                       if (candidates[a].id != candidates[b].id)
                         return candidates[a].id < candidates[b].id;
                       return candidates[a].task_index <
                              candidates[b].task_index;
                     });
    bool fixed = false;
    for (const int row : row_options) {
      if (cost[row][real_col] >= INF) continue;
      auto next_rows = active_rows;
      next_rows.erase(
          std::find(next_rows.begin(), next_rows.end(), row));
      auto next_cols = active_cols;
      next_cols.erase(
          std::find(next_cols.begin(), next_cols.end(),
                    (int)real_col));
      const auto suffix = minimum_cost(next_rows, next_cols);
      if (!suffix.has_value() ||
          cost[row][real_col] + *suffix != *remaining_optimum)
        continue;
      const int robot = free_robots[real_col];
      out.rho_task_id[robot] = candidates[row].id;
      out.rho_ready_index[robot] = candidates[row].task_index;
      active_rows = std::move(next_rows);
      active_cols = std::move(next_cols);
      *remaining_optimum -= cost[row][real_col];
      fixed = true;
      break;
    }
    if (fixed) continue;

    auto next_cols = active_cols;
    next_cols.erase(
        std::find(next_cols.begin(), next_cols.end(), (int)real_col));
    const auto suffix = minimum_cost(active_rows, next_cols);
    if (suffix.has_value() && *suffix == *remaining_optimum) {
      active_cols = std::move(next_cols);
      continue;
    }
    throw std::logic_error(
        "match_ready_tasks: failed deterministic lexicographic refinement");
  }
  return out;
}

inline std::vector<RootDemand>
filter_priority_commitment_for_tau(
    const std::vector<RootDemand>& candidates,
    const std::vector<int>& tau)
{
  std::vector<RootDemand> out;
  for (const auto& root : candidates)
    if (root.target >= 0 &&
        root.target < (int)tau.size() &&
        tau[root.target] == root.goal &&
        std::find_if(
            out.begin(), out.end(),
            [&](const RootDemand& existing) {
              return existing.target == root.target;
            }) == out.end())
      out.push_back(root);
  return out;
}

struct PriorityCommitmentRoot {
  RootDemand demand;
  int priority = 0;
};

struct PriorityCommitmentGroup {
  std::vector<PriorityCommitmentRoot> roots;
  TaskId task_id;
  size_t robot = 0;
};

inline std::vector<RootDemand>
select_priority_commitment_for_epoch(
    const std::vector<PriorityCommitmentGroup>& completed_groups,
    const std::vector<int>& tau, size_t active_root_count,
    const std::vector<int>& previous_commitment,
    size_t vacancy_count =
        std::numeric_limits<size_t>::max())
{
  PriorityCommitmentGroup best_group;
  bool have_best = false;
  for (const auto& group : completed_groups) {
    std::map<int, PriorityCommitmentRoot> unique_roots;
    for (const auto& root : group.roots) {
      if (root.demand.target < 0 ||
          root.demand.target >= (int)tau.size() ||
          tau[root.demand.target] != root.demand.goal)
        continue;
      auto inserted = unique_roots.emplace(
          root.demand.target, root);
      if (!inserted.second &&
          root.priority > inserted.first->second.priority)
        inserted.first->second = root;
    }
    PriorityCommitmentGroup filtered;
    filtered.task_id = group.task_id;
    filtered.robot = group.robot;
    for (const auto& [unused_target, root] : unique_roots) {
      (void)unused_target;
      filtered.roots.push_back(root);
    }
    std::stable_sort(
        filtered.roots.begin(), filtered.roots.end(),
        [](const auto& a, const auto& b) {
          return a.priority != b.priority
                     ? a.priority > b.priority
                     : a.demand.target < b.demand.target;
        });
    if (filtered.roots.empty()) continue;
    const bool better =
        !have_best ||
        filtered.roots.front().priority >
            best_group.roots.front().priority ||
        (filtered.roots.front().priority ==
             best_group.roots.front().priority &&
         (filtered.task_id < best_group.task_id ||
          (!(best_group.task_id < filtered.task_id) &&
           filtered.robot < best_group.robot)));
    if (better) {
      best_group = std::move(filtered);
      have_best = true;
    }
  }
  if (!have_best) return {};

  bool intersects_previous_collective = false;
  if (previous_commitment.size() > 1 &&
      best_group.roots.size() > 1)
    for (const auto& root : best_group.roots)
      if (std::find(
              previous_commitment.begin(),
              previous_commitment.end(),
              root.demand.target) != previous_commitment.end()) {
        intersects_previous_collective = true;
        break;
      }
  const bool collective =
      best_group.roots.size() * 2 > active_root_count ||
      (active_root_count > vacancy_count &&
       active_root_count - vacancy_count >= vacancy_count) ||
      intersects_previous_collective;
  const size_t selected_count =
      collective ? best_group.roots.size() : 1;
  std::vector<RootDemand> selected;
  for (size_t i = 0; i < selected_count; ++i)
    selected.push_back(best_group.roots[i].demand);
  return selected;
}

inline void compile_task_br_upper_epoch_graph(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall,
    const std::vector<int>& priority_commitment,
    UpperEpochGuidance& epoch)
{
  epoch.priority_commitment.clear();
  epoch.target_priority =
      target_priorities_from_pair_cost(
          epoch.pair_cost, epoch.tau_guide);
  for (const int target : priority_commitment)
    if (target >= 0 && target < (int)ins.n_targets() &&
        target < (int)upper.target_pos.size() &&
        target < (int)epoch.tau_guide.size() &&
        upper.target_pos[target] != epoch.tau_guide[target] &&
        std::find(
            epoch.priority_commitment.begin(),
            epoch.priority_commitment.end(),
            target) == epoch.priority_commitment.end())
      epoch.priority_commitment.push_back(target);
  int promoted_priority =
      epoch.target_priority.empty()
          ? (int)epoch.priority_commitment.size() - 1
          : *std::max_element(
                epoch.target_priority.begin(),
                epoch.target_priority.end()) +
                (int)epoch.priority_commitment.size();
  for (const int target : epoch.priority_commitment)
    epoch.target_priority[target] = promoted_priority--;

  std::vector<RootDemand> roots;
  for (size_t target = 0; target < ins.n_targets(); ++target)
    if (target < upper.target_pos.size() &&
        target < epoch.tau_guide.size() &&
        upper.target_pos[target] != epoch.tau_guide[target])
      roots.push_back(
          RootDemand{(int)target, epoch.tau_guide[target]});
  auto abstract = make_abstract_upper_state(ins, upper);
  TaskBRCompilerLimits compiler_limits{256, 512};
  // With at most two vacancies, useful alternatives can each require a full
  // vacancy-chain recursion window; the branch cap still bounds the epoch.
  // Roomier layouts instead cap accumulated recursion so many failed root
  // options cannot repeatedly refresh the local 256-call window.
  if (upper_vacancy_count(ins, upper) > 2)
    compiler_limits.total_recursion_cap =
        compiler_limits.recursion_cap +
        compiler_limits.backtrack_cap;
  epoch.task_graph = compile_task_br_pibt(
      ins, abstract, roots, epoch.tau_guide,
      epoch.target_priority, upper_wall,
      compiler_limits, false);
}

inline std::shared_ptr<const UpperEpochGuidance>
build_task_br_upper_epoch(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall, double alpha, double gamma,
    double delta,
    const std::vector<PriorityCommitmentGroup>*
        priority_commitment_groups = nullptr,
    size_t active_root_count = 0,
    const std::vector<int>*
        previous_priority_commitment = nullptr)
{
  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->upper_signature = upper;
  auto assignment = build_lazy_pair_cost_assignment(
      ins, upper, upper_wall, alpha, gamma, delta);
  epoch->pair_cost = std::move(assignment.table);
  epoch->pair_edges_evaluated = assignment.evaluated_edges;
  epoch->pair_edges_total = assignment.total_edges;
  epoch->pair_rollout_work_steps = assignment.rollout_work_steps;
  epoch->pair_rollout_truncations =
      assignment.rollout_truncations;
  epoch->pair_rollout_stalls = assignment.rollout_stalls;
  epoch->tau_guide = std::move(assignment.tau);
  std::vector<int> priority_commitment;
  if (priority_commitment_groups != nullptr)
    for (const auto& root :
         select_priority_commitment_for_epoch(
             *priority_commitment_groups,
             epoch->tau_guide, active_root_count,
             previous_priority_commitment != nullptr
                 ? *previous_priority_commitment
                 : std::vector<int>{},
             upper_vacancy_count(ins, upper)))
      priority_commitment.push_back(root.target);
  compile_task_br_upper_epoch_graph(
      ins, upper, upper_wall, priority_commitment, *epoch);
  return epoch;
}

inline std::shared_ptr<const UpperEpochGuidance>
rebuild_task_br_upper_epoch(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall,
    const UpperEpochGuidance& pair_source,
    const std::vector<int>& priority_commitment)
{
  auto epoch =
      std::make_shared<UpperEpochGuidance>(pair_source);
  compile_task_br_upper_epoch_graph(
      ins, upper, upper_wall, priority_commitment, *epoch);
  return epoch;
}

inline std::shared_ptr<const UpperEpochGuidance>
build_task_br_upper_epoch_for_tau(
    const DDInstance& ins, const UpperSignature& upper,
    const std::vector<int>& fixed_tau, DDDistCache& upper_wall,
    double alpha, double gamma, double delta)
{
  if (fixed_tau.size() != ins.n_targets())
    throw std::invalid_argument(
        "build_task_br_upper_epoch_for_tau: tau size mismatch");
  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->upper_signature = upper;
  epoch->pair_cost = build_pair_cost_table(
      ins, upper, upper_wall, alpha, gamma, delta);
  for (const auto& row : epoch->pair_cost) {
    epoch->pair_edges_total += row.size();
    epoch->pair_edges_evaluated += row.size();
    for (const auto& entry : row) {
      epoch->pair_rollout_work_steps += entry.plan.rollout_steps;
      epoch->pair_rollout_truncations += entry.plan.truncated;
      epoch->pair_rollout_stalls += entry.plan.stalled;
    }
  }
  epoch->tau_guide = fixed_tau;
  for (size_t target = 0; target < ins.n_targets(); ++target) {
    if (!eligible_goal(ins, (int)target, fixed_tau[target]))
      throw std::invalid_argument(
          "build_task_br_upper_epoch_for_tau: ineligible goal");
  }
  compile_task_br_upper_epoch_graph(
      ins, upper, upper_wall, {}, *epoch);
  return epoch;
}

struct UpperEpochCache {
  static constexpr size_t DEFAULT_CAPACITY = 256;

  struct Key {
    UpperSignature upper;
    std::vector<int> priority_commitment;

    bool operator<(const Key& other) const
    {
      if (upper != other.upper) return upper < other.upper;
      return priority_commitment < other.priority_commitment;
    }
  };

  explicit UpperEpochCache(size_t max_entries = DEFAULT_CAPACITY)
      : capacity(max_entries)
  {
    if (capacity == 0)
      throw std::invalid_argument(
          "UpperEpochCache capacity must be positive");
  }

  std::shared_ptr<const UpperEpochGuidance> lookup(
      const UpperSignature& signature,
      const std::vector<int>* priority_commitment = nullptr)
  {
    const auto it = entries.find(
        Key{signature,
            priority_commitment != nullptr
                ? *priority_commitment
                : std::vector<int>{}});
    if (it == entries.end()) {
      ++misses;
      return nullptr;
    }
    ++hits;
    it->second.last_used = ++clock;
    return it->second.epoch;
  }

  std::shared_ptr<const UpperEpochGuidance> peek_any(
      const UpperSignature& signature)
  {
    const auto it = std::find_if(
        entries.begin(), entries.end(),
        [&](const auto& entry) {
          return entry.first.upper == signature;
        });
    if (it == entries.end()) return nullptr;
    it->second.last_used = ++clock;
    return it->second.epoch;
  }

  void insert(
      const UpperSignature& signature,
      std::shared_ptr<const UpperEpochGuidance> epoch)
  {
    if (epoch == nullptr)
      throw std::invalid_argument(
          "UpperEpochCache cannot store a null epoch");
    const Key key{
        signature, epoch->priority_commitment};
    const auto existing = entries.find(key);
    if (existing != entries.end()) {
      existing->second = Entry{std::move(epoch), ++clock};
      return;
    }
    if (entries.size() == capacity) {
      const auto victim = std::min_element(
          entries.begin(), entries.end(),
          [](const auto& a, const auto& b) {
            return a.second.last_used < b.second.last_used;
          });
      entries.erase(victim);
      ++evictions;
    }
    entries.emplace(
        key, Entry{std::move(epoch), ++clock});
  }

  size_t size() const { return entries.size(); }

  bool contains(const UpperSignature& signature) const
  {
    return std::find_if(
               entries.begin(), entries.end(),
               [&](const auto& entry) {
                 return entry.first.upper == signature;
               }) != entries.end();
  }

  long hits = 0;
  long misses = 0;
  long evictions = 0;

 private:
  struct Entry {
    std::shared_ptr<const UpperEpochGuidance> epoch;
    uint64_t last_used = 0;
  };

  size_t capacity;
  uint64_t clock = 0;
  std::map<Key, Entry> entries;
};

inline CarrierGuidance build_task_br_guidance_from_upper_epoch(
    const DDInstance& ins, const PhysConfig& physical,
    std::shared_ptr<const UpperEpochGuidance> upper_epoch,
    const PhysConfig* previous_physical = nullptr,
    const CarrierGuidance* previous_guidance = nullptr,
    const std::vector<Op>* executed_ops = nullptr)
{
  if (upper_epoch == nullptr ||
      upper_epoch->upper_signature != make_upper_signature(physical))
    throw std::invalid_argument(
        "build_task_br_guidance_from_upper_epoch: epoch mismatch");
  CarrierGuidance out;
  out.upper_epoch = std::move(upper_epoch);
  auto recovered = recover_task_br_custody(
      ins, physical, out.upper_epoch->task_graph, previous_physical,
      previous_guidance, executed_ops);
  out.custody_by_robot = std::move(recovered.custody_by_robot);
  out.ready_tasks = ready_tasks_with_custody(
      ins, physical, out.upper_epoch->task_graph,
      out.custody_by_robot, recovered.continuation_carrier);
  bind_ready_continuations(
      ins, physical, out.upper_epoch->task_graph, out.ready_tasks,
      recovered.continuation_carrier,
      recovered.previous_loaded_move_from,
      out.custody_by_robot);

  std::vector<int> grounded_ready;
  for (const int index : out.ready_tasks)
    if (index >= 0 &&
        index < (int)out.upper_epoch->task_graph.tasks.size() &&
        task_shelf_is_grounded(
            ins, physical,
            out.upper_epoch->task_graph.tasks[index].id))
      grounded_ready.push_back(index);
  const auto* previous_rho =
      recovered.transition_valid && previous_guidance != nullptr
          ? &previous_guidance->rho_task_id
          : nullptr;
  auto rho = match_ready_tasks(
      ins, physical, out.upper_epoch->task_graph,
      grounded_ready, previous_rho);
  out.rho_task_id = std::move(rho.rho_task_id);
  out.rho_ready_index = std::move(rho.rho_ready_index);
  return out;
}

inline CarrierGuidance build_task_br_guidance(
    const DDInstance& ins, const PhysConfig& physical,
    DDDistCache& upper_wall, double alpha, double gamma, double delta,
    const PhysConfig* previous_physical = nullptr,
    const CarrierGuidance* previous_guidance = nullptr,
    const std::vector<Op>* executed_ops = nullptr,
    UpperEpochCache* upper_epoch_cache = nullptr)
{
  const auto upper = make_upper_signature(physical);
  std::shared_ptr<const UpperEpochGuidance> upper_epoch;

  bool transition_valid = false;
  if (previous_physical != nullptr && executed_ops != nullptr) {
    const auto replayed =
        apply_ops(ins, *previous_physical, *executed_ops);
    transition_valid = replayed.has_value() && *replayed == physical;
  }
  std::vector<PriorityCommitmentGroup>
      priority_commitment_groups;
  size_t priority_commitment_active_roots = 0;
  std::vector<int> previous_priority_commitment;
  if (transition_valid && previous_physical != nullptr &&
      previous_guidance != nullptr &&
      previous_guidance->upper_epoch != nullptr &&
      executed_ops != nullptr) {
    const auto& previous_epoch =
        *previous_guidance->upper_epoch;
    previous_priority_commitment =
        previous_epoch.priority_commitment;
    size_t active_root_count = 0;
    for (size_t target = 0;
         target < previous_epoch.upper_signature.target_pos.size() &&
         target < previous_epoch.tau_guide.size();
         ++target)
      active_root_count +=
          previous_epoch.upper_signature.target_pos[target] !=
          previous_epoch.tau_guide[target];
    const bool dense_upper_layout =
        target_dense_upper_layout(
            ins, previous_epoch.upper_signature);
    for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
      if (robot >= previous_physical->kappa.size() ||
          robot >= previous_physical->robots.size() ||
          robot >= physical.robots.size() ||
          robot >= executed_ops->size() ||
          robot >= previous_guidance->custody_by_robot.size() ||
          previous_physical->kappa[robot] == KAPPA_FREE ||
          (*executed_ops)[robot].kind != Op::MOVE ||
          !previous_guidance->custody_by_robot[robot].has_value())
        continue;
      const auto& custody =
          *previous_guidance->custody_by_robot[robot];
      if (previous_physical->robots[robot] != custody.from ||
          (*executed_ops)[robot].to != custody.to ||
          physical.robots[robot] != custody.to)
        continue;
      const auto transfer = normalized_transfer(custody);
      if (custody.transfer_index + 2 < transfer.route.size())
        continue;
      std::map<int, PriorityCommitmentRoot> group_roots;
      for (const auto& root : custody.roots) {
        if (root.target < 0 ||
            root.target >= (int)ins.n_targets() ||
            root.target >= (int)physical.target_pos.size() ||
            root.target >=
                (int)previous_epoch.tau_guide.size() ||
            previous_epoch.tau_guide[root.target] != root.goal ||
            physical.target_pos[root.target] == root.goal)
          continue;
        const bool flexible_goal =
            ins.target_goal_sets[root.target].size() > 1;
        const bool self_root =
            custody.shelf.kind == ShelfSelector::Kind::TARGET &&
            custody.shelf.value == root.target;
        if ((flexible_goal && !dense_upper_layout) ||
            (self_root && !flexible_goal))
          continue;
        const int previous_priority =
            root.target <
                    (int)previous_epoch.target_priority.size()
                ? previous_epoch.target_priority[root.target]
                : 0;
        auto inserted = group_roots.emplace(
            root.target,
            PriorityCommitmentRoot{root, previous_priority});
        if (!inserted.second)
          inserted.first->second.priority =
              std::max(
                  inserted.first->second.priority,
                  previous_priority);
      }
      std::vector<PriorityCommitmentRoot> ordered_group;
      for (const auto& [unused_target, root] : group_roots) {
        (void)unused_target;
        ordered_group.push_back(root);
      }
      std::stable_sort(
          ordered_group.begin(), ordered_group.end(),
          [](const auto& a, const auto& b) {
            return a.priority != b.priority
                       ? a.priority > b.priority
                       : a.demand.target < b.demand.target;
          });
      if (ordered_group.empty()) continue;
      priority_commitment_groups.push_back(
          PriorityCommitmentGroup{
              std::move(ordered_group),
              custody.task_id,
              robot});
    }
    if (!priority_commitment_groups.empty())
      priority_commitment_active_roots = active_root_count;
  }
  if (transition_valid && previous_guidance != nullptr &&
      previous_guidance->upper_epoch != nullptr &&
      previous_guidance->upper_epoch->upper_signature == upper) {
    upper_epoch = previous_guidance->upper_epoch;
  } else if (upper_epoch_cache != nullptr) {
    bool insert_epoch = false;
    const auto pair_source =
        upper_epoch_cache->peek_any(upper);
    if (pair_source != nullptr) {
      const auto filtered =
          select_priority_commitment_for_epoch(
              priority_commitment_groups,
              pair_source->tau_guide,
              priority_commitment_active_roots,
              previous_priority_commitment,
              upper_vacancy_count(ins, upper));
      std::vector<int> priority_commitment;
      for (const auto& root : filtered)
        priority_commitment.push_back(root.target);
      const auto* key =
          priority_commitment.empty()
              ? nullptr
              : &priority_commitment;
      upper_epoch = upper_epoch_cache->lookup(upper, key);
      if (upper_epoch == nullptr) {
        upper_epoch = rebuild_task_br_upper_epoch(
            ins, upper, upper_wall, *pair_source,
            priority_commitment);
        insert_epoch = true;
      }
    } else {
      std::vector<int> requested_targets;
      for (const auto& group : priority_commitment_groups)
        for (const auto& root : group.roots)
          if (std::find(
                  requested_targets.begin(),
                  requested_targets.end(),
                  root.demand.target) ==
              requested_targets.end())
            requested_targets.push_back(root.demand.target);
      const auto* requested_key =
          requested_targets.empty()
              ? nullptr
              : &requested_targets;
      upper_epoch =
          upper_epoch_cache->lookup(upper, requested_key);
      if (upper_epoch != nullptr)
        throw std::logic_error(
            "UpperEpochCache peek/lookup disagreement");
      upper_epoch = build_task_br_upper_epoch(
          ins, upper, upper_wall, alpha, gamma, delta,
          priority_commitment_groups.empty()
              ? nullptr
              : &priority_commitment_groups,
          priority_commitment_active_roots,
          previous_priority_commitment.empty()
              ? nullptr
              : &previous_priority_commitment);
      insert_epoch = true;
    }
    if (insert_epoch)
      upper_epoch_cache->insert(upper, upper_epoch);
  } else {
    upper_epoch = build_task_br_upper_epoch(
        ins, upper, upper_wall, alpha, gamma, delta,
        priority_commitment_groups.empty()
            ? nullptr
            : &priority_commitment_groups,
        priority_commitment_active_roots,
        previous_priority_commitment.empty()
            ? nullptr
            : &previous_priority_commitment);
  }
  return build_task_br_guidance_from_upper_epoch(
      ins, physical, std::move(upper_epoch), previous_physical,
      previous_guidance, executed_ops);
}

inline std::vector<int> task_br_robot_order(
    const PhysConfig& physical, const CarrierGuidance& guide,
    LowerDist& lower_distance)
{
  auto robot_class = [&](int robot) {
    const bool loaded = physical.kappa[robot] != KAPPA_FREE;
    const bool bound =
        robot < (int)guide.custody_by_robot.size() &&
        guide.custody_by_robot[robot].has_value();
    const bool assigned =
        robot < (int)guide.rho_task_id.size() &&
        guide.rho_task_id[robot].has_value();
    if (loaded && bound) return 0;
    if (loaded) return 1;
    if (assigned) return 2;
    return 3;
  };
  auto robot_priority = [&](int robot) {
    if (robot < (int)guide.custody_by_robot.size() &&
        guide.custody_by_robot[robot].has_value())
      return guide.custody_by_robot[robot]->priority;
    if (guide.upper_epoch != nullptr &&
        robot < (int)guide.rho_ready_index.size()) {
      const int index = guide.rho_ready_index[robot];
      if (index >= 0 &&
          index < (int)guide.upper_epoch->task_graph.tasks.size())
        return guide.upper_epoch->task_graph.tasks[index].priority;
    }
    return 0;
  };
  auto robot_distance = [&](int robot) {
    if (guide.upper_epoch == nullptr ||
        robot >= (int)guide.rho_ready_index.size())
      return 0;
    const int index = guide.rho_ready_index[robot];
    if (index < 0 ||
        index >= (int)guide.upper_epoch->task_graph.tasks.size())
      return 0;
    return lower_distance.dist(
        guide.upper_epoch->task_graph.tasks[index].id.from,
        physical.robots[robot]);
  };
  std::vector<int> order(physical.robots.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    const int priority_a = robot_priority(a);
    const int priority_b = robot_priority(b);
    if (priority_a != priority_b) return priority_a > priority_b;
    const int class_a = robot_class(a);
    const int class_b = robot_class(b);
    if (class_a != class_b) return class_a < class_b;
    const int distance_a = robot_distance(a);
    const int distance_b = robot_distance(b);
    return distance_a != distance_b ? distance_a < distance_b : a < b;
  });
  return order;
}


}  // namespace carrier_detail
