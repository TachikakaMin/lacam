/*
 * Assignment helper for TAPF.
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include "tapf_dist_table.hpp"

constexpr int kTapfAssignmentInfCost = 100000000;
constexpr long kTapfAssignmentWeightInfCost = 4000000000000000000L;

struct TAPFAssignmentResult {
  std::vector<int> agent_to_task;
  int cost;
  bool feasible;
};

// Shared O(n^3) potentials Hungarian (rows <= cols rectangular), extracted
// from the original anonymous HungarianAssignment so BOTH the TAPF
// assignment layer and the DD (Carrier-LaCAM) rho matching reuse ONE
// implementation (skeleton audit 2026-08-30).  Returns per-row assigned
// column.  Negative costs allowed; forbidden pairs are the CALLER's
// convention (large sentinel, filter the result).
std::vector<int> tapf_hungarian_row_to_col(
    const std::vector<std::vector<int> >& cost);

struct TAPFAssignmentStats {
  int calls = 0;
  double time_ms = 0;
};

namespace tapf_assignment_detail {

enum class IncrementalHungarianStatus {
  OK,
  INFEASIBLE,
  CUTOFF,
  INVALID_DUAL,
};

template <typename Scalar>
struct ExactIncrementalHungarianArithmetic {
  bool normalize_reduced(
      Scalar&, Scalar, Scalar, Scalar) const
  {
    return true;
  }

  bool normalize_slack(Scalar&, Scalar) const
  {
    return true;
  }

  bool is_zero(Scalar value) const
  {
    return value == Scalar{0};
  }
};

struct NeverStopIncrementalHungarian {
  bool operator()() const { return false; }
};

// Shared square primal/dual state and ITA-style row repair.  Callers own
// their cost encoding and dummy-edge semantics through score_fn, which
// returns false for a forbidden edge and otherwise writes a maximization
// score.  TAPFAssignmentState and PairCost must both use this one core.
template <typename Scalar>
struct IncrementalHungarianState {
  int dimension = 0;
  std::vector<int> row_to_column;
  std::vector<int> column_to_row;
  std::vector<Scalar> row_dual;
  std::vector<Scalar> column_dual;

  void init(int new_dimension)
  {
    dimension = std::max(0, new_dimension);
    row_to_column.assign(dimension, -1);
    column_to_row.assign(dimension, -1);
    row_dual.assign(dimension, Scalar{0});
    column_dual.assign(dimension, Scalar{0});
  }

  bool ready() const { return dimension > 0; }

  bool structurally_valid() const
  {
    if (dimension < 0 ||
        row_to_column.size() !=
            static_cast<size_t>(dimension) ||
        column_to_row.size() !=
            static_cast<size_t>(dimension) ||
        row_dual.size() !=
            static_cast<size_t>(dimension) ||
        column_dual.size() !=
            static_cast<size_t>(dimension))
      return false;
    for (int row = 0; row < dimension; ++row) {
      const int column = row_to_column[row];
      if (column < 0 || column >= dimension ||
          column_to_row[column] != row)
        return false;
    }
    for (int column = 0; column < dimension; ++column) {
      const int row = column_to_row[column];
      if (row < 0 || row >= dimension ||
          row_to_column[row] != column)
        return false;
    }
    return true;
  }

  template <
      typename ScoreFn, typename Arithmetic,
      typename StopFn>
  IncrementalHungarianStatus solve_full(
      const ScoreFn& score_fn,
      const Arithmetic& arithmetic,
      const StopFn& stop)
  {
    if (stop())
      return IncrementalHungarianStatus::CUTOFF;
    reset_matching();
    if (dimension == 0)
      return IncrementalHungarianStatus::OK;

    for (int row = 0; row < dimension; ++row) {
      if (stop())
        return IncrementalHungarianStatus::CUTOFF;
      Scalar best =
          std::numeric_limits<Scalar>::lowest();
      bool found = false;
      for (int column = 0;
           column < dimension; ++column) {
        Scalar score = Scalar{0};
        if (!score_fn(row, column, score))
          continue;
        best = found ? std::max(best, score) : score;
        found = true;
      }
      if (!found)
        return IncrementalHungarianStatus::INFEASIBLE;
      row_dual[row] = best;
    }

    for (int row = 0; row < dimension; ++row) {
      const auto status =
          augment_from_row(
              row, score_fn, arithmetic, stop);
      if (status != IncrementalHungarianStatus::OK)
        return status;
    }
    return IncrementalHungarianStatus::OK;
  }

  template <
      typename ScoreFn, typename Arithmetic,
      typename StopFn>
  IncrementalHungarianStatus repair_rows(
      int real_row_count,
      const std::vector<int>& changed_rows,
      const ScoreFn& score_fn,
      const Arithmetic& arithmetic,
      const StopFn& stop)
  {
    if (stop())
      return IncrementalHungarianStatus::CUTOFF;
    if (dimension == 0)
      return changed_rows.empty()
                 ? IncrementalHungarianStatus::OK
                 : IncrementalHungarianStatus::INFEASIBLE;
    const int row_limit =
        std::max(0, std::min(
                        real_row_count, dimension));
    std::vector<int> rows;
    std::vector<uint8_t> seen(row_limit, 0);
    rows.reserve(changed_rows.size());
    for (const int row : changed_rows) {
      if (row < 0 || row >= row_limit || seen[row])
        continue;
      seen[row] = 1;
      rows.push_back(row);
    }
    if (rows.empty())
      return IncrementalHungarianStatus::OK;

    for (const int row : rows) {
      if (stop())
        return IncrementalHungarianStatus::CUTOFF;
      const int column = row_to_column[row];
      if (column >= 0)
        column_to_row[column] = -1;
      row_to_column[row] = -1;

      Scalar best =
          std::numeric_limits<Scalar>::lowest();
      bool found = false;
      for (int candidate = 0;
           candidate < dimension; ++candidate) {
        Scalar score = Scalar{0};
        if (!score_fn(row, candidate, score))
          continue;
        const Scalar candidate_dual =
            score - column_dual[candidate];
        best = found
                   ? std::max(best, candidate_dual)
                   : candidate_dual;
        found = true;
      }
      if (!found)
        return IncrementalHungarianStatus::INFEASIBLE;
      row_dual[row] = best;
    }

    for (const int row : rows) {
      const auto status =
          augment_from_row(
              row, score_fn, arithmetic, stop);
      if (status != IncrementalHungarianStatus::OK)
        return status;
    }
    return IncrementalHungarianStatus::OK;
  }

 private:
  void reset_matching()
  {
    std::fill(
        row_to_column.begin(),
        row_to_column.end(), -1);
    std::fill(
        column_to_row.begin(),
        column_to_row.end(), -1);
  }

  template <
      typename ScoreFn, typename Arithmetic,
      typename StopFn>
  IncrementalHungarianStatus augment_from_row(
      int root, const ScoreFn& score_fn,
      const Arithmetic& arithmetic,
      const StopFn& stop)
  {
    std::vector<uint8_t> in_left(dimension, 0);
    std::vector<uint8_t> in_right(dimension, 0);
    std::vector<int> parent_column(dimension, -1);
    std::vector<Scalar> slack(
        dimension,
        std::numeric_limits<Scalar>::max());
    std::queue<int> queue;
    queue.push(root);
    in_left[root] = 1;

    while (true) {
      while (!queue.empty()) {
        if (stop())
          return IncrementalHungarianStatus::CUTOFF;
        const int row = queue.front();
        queue.pop();
        for (int column = 0;
             column < dimension; ++column) {
          if (in_right[column]) continue;
          Scalar score = Scalar{0};
          if (!score_fn(row, column, score))
            continue;
          Scalar reduced =
              row_dual[row] +
              column_dual[column] - score;
          if (!arithmetic.normalize_reduced(
                  reduced, row_dual[row],
                  column_dual[column], score))
            return IncrementalHungarianStatus::
                INVALID_DUAL;
          if (reduced < slack[column]) {
            slack[column] = reduced;
            parent_column[column] = row;
          }
          if (!arithmetic.is_zero(
                  slack[column]))
            continue;
          in_right[column] = 1;
          if (column_to_row[column] < 0) {
            augment_path(
                column, parent_column);
            return IncrementalHungarianStatus::OK;
          }
          const int matched =
              column_to_row[column];
          if (!in_left[matched]) {
            in_left[matched] = 1;
            queue.push(matched);
          }
        }
      }

      Scalar delta =
          std::numeric_limits<Scalar>::max();
      for (int column = 0;
           column < dimension; ++column)
        if (!in_right[column])
          delta = std::min(
              delta, slack[column]);
      if (delta ==
          std::numeric_limits<Scalar>::max())
        return IncrementalHungarianStatus::INFEASIBLE;

      for (int row = 0;
           row < dimension; ++row)
        if (in_left[row])
          row_dual[row] -= delta;
      for (int column = 0;
           column < dimension; ++column) {
        if (in_right[column]) {
          column_dual[column] += delta;
        } else if (
            slack[column] !=
            std::numeric_limits<Scalar>::max()) {
          slack[column] -= delta;
          if (!arithmetic.normalize_slack(
                  slack[column], delta))
            return IncrementalHungarianStatus::
                INVALID_DUAL;
        }
      }

      for (int column = 0;
           column < dimension; ++column) {
        if (in_right[column] ||
            !arithmetic.is_zero(slack[column]))
          continue;
        in_right[column] = 1;
        if (column_to_row[column] < 0) {
          augment_path(
              column, parent_column);
          return IncrementalHungarianStatus::OK;
        }
        const int matched =
            column_to_row[column];
        if (!in_left[matched]) {
          in_left[matched] = 1;
          queue.push(matched);
        }
      }
    }
  }

  void augment_path(
      int column,
      const std::vector<int>& parent_column)
  {
    while (column >= 0) {
      const int row = parent_column[column];
      const int next_column =
          row_to_column[row];
      row_to_column[row] = column;
      column_to_row[column] = row;
      column = next_column;
    }
  }
};

}  // namespace tapf_assignment_detail

struct TAPFAssignmentState {
  int org_n = 0;
  int org_m = 0;
  long cost_scale = 1;
  long tie_hash_mod = 1;
  tapf_assignment_detail::
      IncrementalHungarianState<long> hungarian;

  void init(const int agent_num, const int task_num)
  {
    org_n = agent_num;
    org_m = task_num;
    hungarian.init(std::max(org_n, org_m));
    tie_hash_mod = compute_tie_hash_mod(org_n, org_m);
    cost_scale = compute_cost_scale(org_n, org_m, tie_hash_mod);
  }

  bool ready() const { return hungarian.ready(); }

  template <typename CostFn>
  TAPFAssignmentResult solve_full(const CostFn& cost_fn)
  {
    const auto score =
        [&](int row, int column, long& out) {
          out = weight(row, column, cost_fn);
          return true;
        };
    (void)hungarian.solve_full(
        score,
        tapf_assignment_detail::
            ExactIncrementalHungarianArithmetic<long>{},
        tapf_assignment_detail::
            NeverStopIncrementalHungarian{});
    return make_result(cost_fn);
  }

  template <typename CostFn>
  TAPFAssignmentResult repair_rows(const std::vector<int>& changed_rows,
                                   const CostFn& cost_fn)
  {
    if (!ready()) return solve_full(cost_fn);
    if (changed_rows.empty()) return make_result(cost_fn);
    const auto score =
        [&](int row, int column, long& out) {
          out = weight(row, column, cost_fn);
          return true;
        };
    (void)hungarian.repair_rows(
        org_n, changed_rows, score,
        tapf_assignment_detail::
            ExactIncrementalHungarianArithmetic<long>{},
        tapf_assignment_detail::
            NeverStopIncrementalHungarian{});
    return make_result(cost_fn);
  }

 private:
  static long compute_tie_hash_mod(const int agent_num, const int task_num)
  {
    const auto safe_task_num = std::max(1, task_num);
    const auto base = static_cast<long>(std::max(1, agent_num)) *
                      std::max(1, agent_num) * safe_task_num;
    return std::max(1L, 50000000000L / std::max(1L, base));
  }

  static long compute_cost_scale(const int agent_num, const int task_num,
                                 const long hash_mod)
  {
    const auto max_col = std::max(0, task_num - 1);
    const auto max_pair_tie =
        static_cast<long>(std::max(1, agent_num)) * max_col * hash_mod +
        std::max(0L, hash_mod - 1);
    return static_cast<long>(std::max(1, agent_num)) * max_pair_tie + 1;
  }

  long tie_hash(const int row, const int col) const
  {
    auto value = static_cast<unsigned long long>(row + 1) * 11995408973635179863ULL;
    value ^= static_cast<unsigned long long>(col + 1) * 10150724397891781847ULL;
    value ^= value >> 33;
    value *= 0xff51afd7ed558ccdULL;
    value ^= value >> 33;
    return static_cast<long>(value % static_cast<unsigned long long>(tie_hash_mod));
  }

  long tie_cost(const int row, const int col) const
  {
    if (row >= org_n || col >= org_m) return 0;
    return static_cast<long>(org_n - row) * col * tie_hash_mod +
           tie_hash(row, col);
  }

  template <typename CostFn>
  long weight(const int row, const int col, const CostFn& cost_fn) const
  {
    if (row >= org_n) return 0;
    const auto primary_cost = static_cast<long>(cost_fn(row, col));
    if (primary_cost >= kTapfAssignmentInfCost / 2) return 0;
    const auto encoded_cost = primary_cost * cost_scale + tie_cost(row, col);
    return kTapfAssignmentWeightInfCost - encoded_cost;
  }

  template <typename CostFn>
  TAPFAssignmentResult make_result(const CostFn& cost_fn) const
  {
    TAPFAssignmentResult result;
    result.agent_to_task.assign(org_n, -1);
    result.cost = 0;
    result.feasible = true;
    for (int i = 0; i < org_n; ++i) {
      const auto task =
          hungarian.row_to_column[i];
      if (task < 0 || task >= org_m) {
        result.feasible = false;
        continue;
      }
      const auto cost = static_cast<int>(cost_fn(i, task));
      result.agent_to_task[i] = task;
      result.cost += cost;
      if (cost >= kTapfAssignmentInfCost / 2) result.feasible = false;
    }
    return result;
  }
};

TAPFAssignmentResult assign_tapf_tasks(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    const std::vector<int>& previous_assignment = std::vector<int>(),
    const int sticky_penalty = 0,
    TAPFAssignmentStats* stats = nullptr);

TAPFAssignmentResult assign_tapf_tasks_dynamic(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    TAPFAssignmentState& state, const std::vector<int>& changed_agents,
    const bool force_full = false, TAPFAssignmentStats* stats = nullptr);
