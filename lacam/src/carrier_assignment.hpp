// Part of carrier_guidance.hpp (internal, src/): Solver weights, upper-layout helpers, Hungarian assignment kernels.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>
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
  auto read = [](const char* key, double& out, int64_t& out_scaled) {
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
    try {
      out_scaled = PlanCost::from_values(0, v).work;
    } catch (const std::invalid_argument&) {
      throw std::invalid_argument(
          std::string(key) +
          ": objective weight must be exactly representable at 1e-6 "
          "scale, got '" + raw + "'");
    }
    out = static_cast<double>(out_scaled) /
          static_cast<double>(PlanCost::WORK_SCALE);
  };
  read("DD_ALPHA", w.alpha, w.alpha_scaled);
  read("DD_BETA", w.beta, w.beta_scaled);
  read("DD_GAMMA", w.gamma, w.gamma_scaled);
  read("DD_DELTA", w.delta, w.delta_scaled);
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
    storage_cells += ins.can_place_movable_shelf(cell);
  const size_t shelf_count =
      upper.target_pos.size() + upper.anon_pos.size();
  if (shelf_count > storage_cells)
    throw std::logic_error(
        "upper_vacancy_count: shelves exceed storage cells");
  return storage_cells - shelf_count;
}

inline std::vector<uint8_t> upper_occupancy_bitmap(
    const DDInstance& ins, const UpperSignature& upper)
{
  std::vector<uint8_t> occupied(ins.grid.size(), 0);
  for (const int cell : ins.fixed_upper_cells)
    occupied[cell] = 1;
  for (const int cell : upper.target_pos)
    if (cell >= 0 && cell < ins.grid.size())
      occupied[cell] = 1;
  for (const int cell : upper.anon_pos)
    if (cell >= 0 && cell < ins.grid.size())
      occupied[cell] = 1;
  return occupied;
}

inline std::vector<int> empty_storage_cells(
    const DDInstance& ins, const UpperSignature& upper)
{
  const auto occupied =
      upper_occupancy_bitmap(ins, upper);
  std::vector<int> empty;
  for (int cell = 0; cell < ins.grid.size(); ++cell)
    if (ins.can_place_movable_shelf(cell) && !occupied[cell])
      empty.push_back(cell);
  return empty;
}

inline std::vector<int> empty_transit_cells(
    const DDInstance& ins, const UpperSignature& upper)
{
  const auto occupied =
      upper_occupancy_bitmap(ins, upper);
  std::vector<int> empty;
  for (int cell = 0; cell < ins.grid.size(); ++cell)
    if (!ins.grid.is_wall(cell) &&
        !ins.can_store_shelf(cell) && !occupied[cell])
      empty.push_back(cell);
  return empty;
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
  long double cost = 0;
  bool feasible = false;
  bool cutoff = false;
};

// Floating-point Hungarian used only for Task-BR guidance/LB values.  Tie
// layers are solved separately below; they are never linearly mixed into
// the primary PairCost objective.
inline LongDoubleAssignmentResult hungarian_long_double(
    const std::vector<std::vector<long double>>& cost,
    const Deadline* deadline = nullptr)
{
  LongDoubleAssignmentResult out;
  const auto cutoff = [&]() {
    if (!is_expired(deadline)) return false;
    out.cutoff = true;
    out.feasible = false;
    return true;
  };
  if (cutoff()) return out;
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
    if (cutoff()) return out;
    p[0] = (int)i;
    int j0 = 0;
    std::vector<long double> minv(m + 1, INF);
    std::vector<uint8_t> used(m + 1, 0);
    do {
      if (cutoff()) return out;
      used[j0] = 1;
      const int i0 = p[j0];
      long double delta = INF;
      int j1 = 0;
      for (size_t j = 1; j <= m; ++j) {
        if (cutoff()) return out;
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
  return out;
}

// PairCost adapter over the one incremental Hungarian core owned by
// tapf_assignment.hpp.  This layer defines only long-double cost/INF/deadline
// semantics; matching, dual maintenance, augmentation, and row repair are
// shared with TAPFAssignmentState.
class IncrementalLongDoubleHungarian {
 public:
  bool restore(const PairAssignmentHungarianState& saved)
  {
    invalidate();
    const int dimension = saved.column_count;
    if (!saved.valid || saved.row_count < 0 ||
        saved.column_count < saved.row_count ||
        saved.row_to_column.size() !=
            static_cast<size_t>(dimension) ||
        saved.column_to_row.size() !=
            static_cast<size_t>(dimension) ||
        saved.row_dual.size() !=
            static_cast<size_t>(dimension) ||
        saved.column_dual.size() !=
            static_cast<size_t>(dimension))
      return false;
    row_count_ = saved.row_count;
    column_count_ = saved.column_count;
    core_.dimension = dimension;
    core_.row_to_column = saved.row_to_column;
    core_.column_to_row = saved.column_to_row;
    core_.row_dual = saved.row_dual;
    core_.column_dual = saved.column_dual;
    if (!core_.structurally_valid()) {
      invalidate();
      return false;
    }
    valid_ = true;
    return true;
  }

  PairAssignmentHungarianState snapshot() const
  {
    PairAssignmentHungarianState out;
    out.row_count = row_count_;
    out.column_count = column_count_;
    out.row_to_column = core_.row_to_column;
    out.column_to_row = core_.column_to_row;
    out.row_dual = core_.row_dual;
    out.column_dual = core_.column_dual;
    out.valid = valid_;
    return out;
  }

  bool ready(int row_count, int column_count) const
  {
    return valid_ && row_count_ == row_count &&
           column_count_ == column_count;
  }

  template <typename CostFn>
  LongDoubleAssignmentResult solve_full(
      int row_count, int column_count,
      const CostFn& cost_fn,
      const Deadline* deadline = nullptr)
  {
    invalidate();
    LongDoubleAssignmentResult out;
    out.row_to_col.assign(
        std::max(0, row_count), -1);
    if (is_expired(deadline)) {
      out.cutoff = true;
      return out;
    }
    if (row_count < 0 || column_count < row_count ||
        (row_count > 0 && column_count == 0))
      return out;

    row_count_ = row_count;
    column_count_ = column_count;
    core_.init(column_count_);
    if (column_count_ == 0) {
      valid_ = true;
      out.feasible = true;
      return out;
    }
    const auto score =
        [&](int row, int column,
            long double& value) {
          return edge_score(
              row, column, cost_fn, value);
        };
    const auto stop = [&]() {
      return is_expired(deadline);
    };
    const auto status = core_.solve_full(
        score, LongDoubleArithmetic{}, stop);
    if (status == Status::CUTOFF) {
      invalidate();
      out.cutoff = true;
      return out;
    }
    if (status != Status::OK) {
      invalidate();
      return out;
    }
    valid_ = true;
    return current_result(cost_fn, deadline);
  }

  template <typename CostFn>
  LongDoubleAssignmentResult repair_rows(
      std::vector<int> changed_rows,
      const CostFn& cost_fn,
      const Deadline* deadline = nullptr)
  {
    if (!valid_) {
      LongDoubleAssignmentResult out;
      out.row_to_col.assign(
          std::max(0, row_count_), -1);
      return out;
    }
    if (changed_rows.empty())
      return current_result(cost_fn, deadline);
    const auto score =
        [&](int row, int column,
            long double& value) {
          return edge_score(
              row, column, cost_fn, value);
        };
    const auto stop = [&]() {
      return is_expired(deadline);
    };
    const auto status = core_.repair_rows(
        row_count_, changed_rows, score,
        LongDoubleArithmetic{}, stop);
    if (status == Status::INVALID_DUAL) {
      invalidate();
      throw std::logic_error(
          "IncrementalLongDoubleHungarian::repair_rows: invalid dual");
    }
    if (status == Status::CUTOFF) {
      const int rows = row_count_;
      invalidate();
      LongDoubleAssignmentResult out;
      out.row_to_col.assign(rows, -1);
      out.cutoff = true;
      return out;
    }
    if (status == Status::INFEASIBLE) {
      const int rows = row_count_;
      invalidate();
      return LongDoubleAssignmentResult{
          std::vector<int>(rows, -1),
          0, false, false};
    }
    return current_result(cost_fn, deadline);
  }

  template <typename CostFn>
  LongDoubleAssignmentResult current_result(
      const CostFn& cost_fn,
      const Deadline* deadline = nullptr) const
  {
    LongDoubleAssignmentResult out;
    out.row_to_col.assign(row_count_, -1);
    if (!valid_) return out;
    out.feasible = true;
    for (int row = 0; row < row_count_; ++row) {
      if (is_expired(deadline)) {
        out.cutoff = true;
        out.feasible = false;
        return out;
      }
      const int column =
          core_.row_to_column[row];
      if (column < 0 ||
          column >= column_count_) {
        out.feasible = false;
        return out;
      }
      const long double edge =
          cost_fn(row, column);
      if (!cost_finite(edge)) {
        out.feasible = false;
        return out;
      }
      out.row_to_col[row] = column;
      out.cost += edge;
    }
    return out;
  }

 private:
  static constexpr long double COST_INF = 1e60L;
  using Core = tapf_assignment_detail::
      IncrementalHungarianState<long double>;
  using Status = tapf_assignment_detail::
      IncrementalHungarianStatus;

  static bool cost_finite(long double value)
  {
    return value < COST_INF / 2;
  }

  static long double reduced_tolerance(
      long double left, long double right,
      long double score)
  {
    return 1e-12L *
           (1 + std::fabs(left) +
            std::fabs(right) +
            std::fabs(score));
  }

  struct LongDoubleArithmetic {
    bool normalize_reduced(
        long double& value, long double left,
        long double right, long double score) const
    {
      const long double tolerance =
          reduced_tolerance(
              left, right, score);
      if (value < -tolerance) return false;
      if (value < 0) value = 0;
      return true;
    }

    bool normalize_slack(
        long double& value,
        long double delta) const
    {
      const long double tolerance =
          reduced_tolerance(delta, 0, 0);
      if (value < -tolerance) return false;
      if (value < 0) value = 0;
      return true;
    }

    bool is_zero(long double value) const
    {
      return value == 0;
    }
  };

  template <typename CostFn>
  bool edge_score(
      int row, int column,
      const CostFn& cost_fn,
      long double& score) const
  {
    if (row >= row_count_) {
      score = 0;
      return true;
    }
    const long double edge =
        cost_fn(row, column);
    if (!cost_finite(edge)) return false;
    score = -edge;
    return true;
  }

  void invalidate()
  {
    row_count_ = 0;
    column_count_ = 0;
    core_.init(0);
    valid_ = false;
  }

  int row_count_ = 0;
  int column_count_ = 0;
  Core core_;
  bool valid_ = false;
};

struct BottleneckAssignmentResult {
  std::vector<int> row_to_col;
  long long bottleneck = 0;
  long long secondary_cost = 0;
  bool feasible = false;
  bool cutoff = false;
};

inline BottleneckAssignmentResult
bottleneck_then_sum_assignment(
    const std::vector<std::vector<long long>>& completion,
    const std::vector<std::vector<long long>>& secondary,
    const Deadline* deadline = nullptr)
{
  BottleneckAssignmentResult out;
  const auto cutoff = [&]() {
    if (!is_expired(deadline)) return false;
    out.cutoff = true;
    out.feasible = false;
    return true;
  };
  if (cutoff()) return out;
  const size_t row_count = completion.size();
  const size_t column_count =
      completion.empty() ? 0 : completion.front().size();
  out.row_to_col.assign(row_count, -1);
  if (row_count == 0) {
    out.feasible = true;
    return out;
  }
  if (column_count < row_count ||
      secondary.size() != row_count)
    return out;
  constexpr long long INF =
      std::numeric_limits<long long>::max() / 16;
  std::vector<long long> thresholds;
  for (size_t row = 0; row < row_count; ++row) {
    if (cutoff()) return out;
    if (completion[row].size() != column_count ||
        secondary[row].size() != column_count)
      return out;
    for (size_t column = 0; column < column_count; ++column) {
      if (cutoff()) return out;
      if (completion[row][column] < INF &&
          secondary[row][column] < INF)
        thresholds.push_back(completion[row][column]);
    }
  }
  if (thresholds.empty()) return out;
  std::sort(thresholds.begin(), thresholds.end());
  thresholds.erase(
      std::unique(thresholds.begin(), thresholds.end()),
      thresholds.end());
  constexpr long double HINF = 1e60L;
  const auto feasible_at = [&](long long threshold) {
    std::vector<std::vector<long double>> allowed(
        row_count,
        std::vector<long double>(column_count, HINF));
    for (size_t row = 0; row < row_count; ++row) {
      if (is_expired(deadline))
        return LongDoubleAssignmentResult{{}, 0, false, true};
      for (size_t column = 0; column < column_count; ++column) {
        if (is_expired(deadline))
          return LongDoubleAssignmentResult{{}, 0, false, true};
        if (completion[row][column] <= threshold &&
            secondary[row][column] < INF)
          allowed[row][column] = 0;
      }
    }
    return hungarian_long_double(allowed, deadline);
  };
  size_t lo = 0;
  size_t hi = thresholds.size();
  while (lo < hi) {
    if (cutoff()) return out;
    const size_t mid = lo + (hi - lo) / 2;
    const auto feasibility = feasible_at(thresholds[mid]);
    if (feasibility.cutoff) {
      out.cutoff = true;
      return out;
    }
    if (feasibility.feasible)
      hi = mid;
    else
      lo = mid + 1;
  }
  if (lo == thresholds.size()) return out;
  out.bottleneck = thresholds[lo];

  std::vector<std::vector<long double>> secondary_matrix(
      row_count,
      std::vector<long double>(column_count, HINF));
  for (size_t row = 0; row < row_count; ++row) {
    if (cutoff()) return out;
    for (size_t column = 0; column < column_count; ++column) {
      if (cutoff()) return out;
      if (completion[row][column] <= out.bottleneck &&
          secondary[row][column] < INF)
        secondary_matrix[row][column] =
            (long double)secondary[row][column];
    }
  }
  const auto assignment =
      hungarian_long_double(secondary_matrix, deadline);
  if (assignment.cutoff) {
    out.cutoff = true;
    return out;
  }
  if (!assignment.feasible) return out;
  out.row_to_col = assignment.row_to_col;
  for (size_t row = 0; row < row_count; ++row) {
    const int column = out.row_to_col[row];
    if (column < 0 ||
        completion[row][column] > out.bottleneck ||
        secondary[row][column] >= INF)
      return BottleneckAssignmentResult();
    out.secondary_cost += secondary[row][column];
  }
  out.feasible = true;
  return out;
}

inline int task_service_ticks(const ShelfTask& task)
{
  const int legs =
      task.transfer.route.size() >= 2
          ? (int)task.transfer.route.size() - 1
          : 1;
  return legs + 2;
}

inline std::vector<int> task_critical_tail_ticks(
    const ShelfTaskGraph& graph)
{
  std::vector<int> tail(graph.tasks.size(), 0);
  std::vector<uint8_t> state(graph.tasks.size(), 0);
  const std::function<int(int)> visit = [&](int index) {
    if (index < 0 || index >= (int)graph.tasks.size()) return 0;
    if (state[index] == 2) return tail[index];
    if (state[index] == 1) return 0;
    state[index] = 1;
    int best = 0;
    if (index < (int)graph.successors.size())
      for (const int successor : graph.successors[index])
        if (successor >= 0 &&
            successor < (int)graph.tasks.size())
          best = std::max(
              best,
              task_service_ticks(graph.tasks[successor]) +
                  visit(successor));
    tail[index] = best;
    state[index] = 2;
    return best;
  };
  for (size_t index = 0; index < graph.tasks.size(); ++index)
    visit((int)index);
  return tail;
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
  bool cutoff = false;
};

inline LexAssignmentResult hungarian_lexicographic(
    const std::vector<std::vector<LexAssignmentCost>>& cost,
    const Deadline* deadline = nullptr)
{
  LexAssignmentResult out;
  const auto cutoff = [&]() {
    if (!is_expired(deadline)) return false;
    out.cutoff = true;
    out.feasible = false;
    return true;
  };
  if (cutoff()) return out;
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
    if (cutoff()) return out;
    p[0] = (int)i;
    int j0 = 0;
    std::vector<LexAssignmentCost> minv(
        m + 1, LexAssignmentCost::infinity());
    std::vector<uint8_t> used(m + 1, 0);
    do {
      if (cutoff()) return out;
      used[j0] = 1;
      const int i0 = p[j0];
      LexAssignmentCost delta =
          LexAssignmentCost::infinity();
      int j1 = 0;
      for (size_t j = 1; j <= m; ++j) {
        if (cutoff()) return out;
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
        if (cutoff()) return out;
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
    if (cutoff()) return out;
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

}  // namespace carrier_detail
