/*
 * Assignment helper for TAPF.
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "tapf_dist_table.hpp"

constexpr int kTapfAssignmentInfCost = 100000000;
constexpr long kTapfAssignmentWeightInfCost = 4000000000000000000L;

struct TAPFAssignmentResult {
  std::vector<int> agent_to_task;
  int cost;
  bool feasible;
};

struct TAPFAssignmentStats {
  int calls = 0;
  double time_ms = 0;
  long row_cache_requests = 0;
  long row_cache_hits = 0;
};

struct TAPFAssignmentServiceCostState {
  std::vector<int> partial_task_by_agent;
  std::vector<int> partial_remaining_by_agent;
};

struct TAPFAssignmentRowCacheKey {
  int agent_id = -1;
  int cell_id = -1;
  int partial_task = -1;
  int partial_remaining = 0;

  bool operator==(const TAPFAssignmentRowCacheKey& other) const
  {
    return agent_id == other.agent_id && cell_id == other.cell_id &&
           partial_task == other.partial_task &&
           partial_remaining == other.partial_remaining;
  }
};

struct TAPFAssignmentRowCacheKeyHash {
  size_t operator()(const TAPFAssignmentRowCacheKey& key) const
  {
    auto seed = size_t{0xcbf29ce484222325ULL};
    auto combine = [&](const int value) {
      seed ^= std::hash<int>()(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) +
              (seed >> 2);
    };
    combine(key.agent_id);
    combine(key.cell_id);
    combine(key.partial_task);
    combine(key.partial_remaining);
    return seed;
  }
};

struct TAPFAssignmentState {
  int org_n = 0;
  int org_m = 0;
  int n = 0;
  std::vector<int> mateL;
  std::vector<int> mateR;
  std::vector<long> lx;
  std::vector<long> ly;
  long cost_scale = 1;
  long tie_hash_mod = 1;
  std::unordered_map<TAPFAssignmentRowCacheKey, std::vector<int>,
                     TAPFAssignmentRowCacheKeyHash>
      row_cost_cache;

  void init(const int agent_num, const int task_num)
  {
    org_n = agent_num;
    org_m = task_num;
    n = std::max(org_n, org_m);
    mateL.assign(n, -1);
    mateR.assign(n, -1);
    lx.assign(n, 0);
    ly.assign(n, 0);
    row_cost_cache.clear();
    row_cost_cache.reserve(static_cast<size_t>(std::max(1, org_n)) * 32);
    tie_hash_mod = compute_tie_hash_mod(org_n, org_m);
    cost_scale = compute_cost_scale(org_n, org_m, tie_hash_mod);
  }

  bool ready() const { return n > 0; }

  template <typename CostFn>
  TAPFAssignmentResult solve_full(const CostFn& cost_fn)
  {
    reset_matching();
    for (int i = 0; i < n; ++i) {
      long best = 0;
      if (i < org_n) {
        best = std::numeric_limits<long>::min();
        for (int j = 0; j < n; ++j) {
          best = std::max(best, weight(i, j, cost_fn));
        }
      }
      lx[i] = best;
    }
    for (int i = 0; i < n; ++i) {
      if (mateL[i] == -1) augment_from_row(i, cost_fn);
    }
    return make_result(cost_fn);
  }

  template <typename CostFn>
  TAPFAssignmentResult repair_rows(const std::vector<int>& changed_rows,
                                   const CostFn& cost_fn)
  {
    if (!ready()) return solve_full(cost_fn);
    if (changed_rows.empty()) return make_result(cost_fn);

    for (const auto row : changed_rows) {
      if (row < 0 || row >= org_n) continue;
      if (mateL[row] != -1) {
        mateR[mateL[row]] = -1;
        mateL[row] = -1;
      }
      long best = std::numeric_limits<long>::min();
      for (int j = 0; j < n; ++j) {
        best = std::max(best, weight(row, j, cost_fn) - ly[j]);
      }
      lx[row] = best;
    }

    for (const auto row : changed_rows) {
      if (row < 0 || row >= org_n) continue;
      if (mateL[row] == -1) augment_from_row(row, cost_fn);
    }
    return make_result(cost_fn);
  }

 private:
  void reset_matching()
  {
    std::fill(mateL.begin(), mateL.end(), -1);
    std::fill(mateR.begin(), mateR.end(), -1);
  }

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
  bool augment_from_row(const int root, const CostFn& cost_fn)
  {
    std::vector<bool> in_left(n, false);
    std::vector<bool> in_right(n, false);
    std::vector<int> parent_col(n, -1);
    std::vector<long> slack(n, std::numeric_limits<long>::max());
    std::queue<int> queue;

    queue.push(root);
    in_left[root] = true;

    while (true) {
      while (!queue.empty()) {
        const auto u = queue.front();
        queue.pop();
        for (int v = 0; v < n; ++v) {
          if (in_right[v]) continue;
          const auto cur = lx[u] + ly[v] - weight(u, v, cost_fn);
          if (cur < slack[v]) {
            slack[v] = cur;
            parent_col[v] = u;
          }
          if (slack[v] != 0) continue;
          in_right[v] = true;
          if (mateR[v] == -1) {
            augment_path(v, parent_col);
            return true;
          }
          const auto matched_row = mateR[v];
          if (!in_left[matched_row]) {
            in_left[matched_row] = true;
            queue.push(matched_row);
          }
        }
      }

      long delta = std::numeric_limits<long>::max();
      for (int v = 0; v < n; ++v) {
        if (!in_right[v]) delta = std::min(delta, slack[v]);
      }
      if (delta == std::numeric_limits<long>::max()) return false;

      for (int u = 0; u < n; ++u) {
        if (in_left[u]) lx[u] -= delta;
      }
      for (int v = 0; v < n; ++v) {
        if (in_right[v]) {
          ly[v] += delta;
        } else {
          slack[v] -= delta;
        }
      }

      for (int v = 0; v < n; ++v) {
        if (in_right[v] || slack[v] != 0) continue;
        in_right[v] = true;
        if (mateR[v] == -1) {
          augment_path(v, parent_col);
          return true;
        }
        const auto matched_row = mateR[v];
        if (!in_left[matched_row]) {
          in_left[matched_row] = true;
          queue.push(matched_row);
        }
      }
    }
  }

  void augment_path(int right_vertex, const std::vector<int>& parent_col)
  {
    while (right_vertex != -1) {
      const auto left_vertex = parent_col[right_vertex];
      const auto next_right = mateL[left_vertex];
      mateL[left_vertex] = right_vertex;
      mateR[right_vertex] = left_vertex;
      right_vertex = next_right;
    }
  }

  template <typename CostFn>
  TAPFAssignmentResult make_result(const CostFn& cost_fn) const
  {
    TAPFAssignmentResult result;
    result.agent_to_task.assign(org_n, -1);
    result.cost = 0;
    result.feasible = true;
    for (int i = 0; i < org_n; ++i) {
      const auto task = mateL[i];
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
    TAPFAssignmentStats* stats = nullptr,
    const TAPFAssignmentServiceCostState& service_cost_state =
        TAPFAssignmentServiceCostState());

TAPFAssignmentResult assign_tapf_tasks_dynamic(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    TAPFAssignmentState& state, const std::vector<int>& changed_agents,
    const bool force_full = false, TAPFAssignmentStats* stats = nullptr,
    const std::vector<int>& fixed_task_by_agent = std::vector<int>(),
    const std::vector<bool>& unavailable_tasks = std::vector<bool>(),
    const TAPFAssignmentServiceCostState& service_cost_state =
        TAPFAssignmentServiceCostState());

TAPFAssignmentResult assign_hungarian_cost_matrix(
    const std::vector<std::vector<int> >& cost);
