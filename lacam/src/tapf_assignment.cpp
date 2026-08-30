#include "../include/tapf_assignment.hpp"

#include "../include/utils.hpp"

// The one Hungarian in this codebase (skeleton audit 2026-08-30): the
// original anonymous-namespace HungarianAssignment hoisted into a public
// function so the DD (Carrier-LaCAM) rho matching reuses it instead of
// keeping a copy.  Algorithm and scan order are byte-for-byte the original
// (classic potentials formulation), so TAPF results and deterministic tie
// behavior are unchanged.
std::vector<int> tapf_hungarian_row_to_col(
    const std::vector<std::vector<int> >& cost)
{
  const size_t n = cost.size();
  const size_t m = cost.empty() ? 0 : cost.front().size();
  if (n == 0 || m == 0) return std::vector<int>(n, -1);
  auto a = std::vector<std::vector<int> >(n + 1, std::vector<int>(m + 1, 0));
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < m; ++j) a[i + 1][j + 1] = cost[i][j];
  auto u = std::vector<int>(n + 1, 0);
  auto v = std::vector<int>(m + 1, 0);
  auto p = std::vector<int>(m + 1, 0);
  auto way = std::vector<int>(m + 1, 0);
  for (size_t i = 1; i <= n; ++i) {
    p[0] = i;
    auto j0 = 0;
    auto minv = std::vector<int>(m + 1, std::numeric_limits<int>::max());
    auto used = std::vector<bool>(m + 1, false);
    do {
      used[j0] = true;
      const auto i0 = p[j0];
      auto delta = std::numeric_limits<int>::max();
      auto j1 = 0;
      for (size_t j = 1; j <= m; ++j) {
        if (used[j]) continue;
        const auto cur = a[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
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
      const auto j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  auto row_to_col = std::vector<int>(n, -1);
  for (size_t j = 1; j <= m; ++j)
    if (p[j] > 0) row_to_col[p[j] - 1] = (int)j - 1;
  return row_to_col;
}

namespace {
class HungarianAssignment {
 public:
  explicit HungarianAssignment(const std::vector<std::vector<int> >& cost)
      : cost_(cost)
  {
  }

  TAPFAssignmentResult solve()
  {
    const auto row_to_col = tapf_hungarian_row_to_col(cost_);
    auto result = TAPFAssignmentResult();
    result.agent_to_task = row_to_col;
    result.cost = 0;
    result.feasible = true;
    for (size_t i = 0; i < row_to_col.size(); ++i) {
      const auto j = row_to_col[i];
      if (j < 0) continue;
      result.cost += cost_[i][j];
      if (cost_[i][j] >= kTapfAssignmentInfCost / 2) result.feasible = false;
    }
    return result;
  }

 private:
  const std::vector<std::vector<int> >& cost_;
};

TAPFAssignmentResult invalid_result()
{
  return TAPFAssignmentResult{std::vector<int>(), kTapfAssignmentInfCost,
                              false};
}

bool invalid_input(const TAPFInstance& ins, const Config& C)
{
  return ins.tasks.size() < ins.N || C.size() != ins.N;
}

void record_assignment_time(const Time::time_point& t_start,
                            TAPFAssignmentStats* stats)
{
  if (stats == nullptr) return;
  stats->time_ms +=
      std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() -
                                                           t_start)
          .count() /
      1000000.0;
}

std::vector<std::vector<int> > build_cost_matrix(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    const std::vector<int>& previous_assignment, const int sticky_penalty)
{
  auto cost = std::vector<std::vector<int> >(
      ins.N, std::vector<int>(ins.tasks.size(), kTapfAssignmentInfCost));
  for (size_t i = 0; i < ins.N; ++i) {
    for (size_t j = 0; j < ins.tasks.size(); ++j) {
      if (!ins.allowed[i][j]) continue;
      auto d = D.get(j, C[i]);
      if (d >= D.K) continue;
      cost[i][j] = d;
      if (!previous_assignment.empty() &&
          previous_assignment[i] != static_cast<int>(j)) {
        cost[i][j] += sticky_penalty;
      }
    }
  }
  return cost;
}
}  // namespace

TAPFAssignmentResult assign_tapf_tasks(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    const std::vector<int>& previous_assignment, const int sticky_penalty,
    TAPFAssignmentStats* stats)
{
  const auto t_start = Time::now();
  if (stats != nullptr) ++stats->calls;

  if (invalid_input(ins, C)) {
    record_assignment_time(t_start, stats);
    return invalid_result();
  }

  auto result = HungarianAssignment(build_cost_matrix(
                                        ins, D, C, previous_assignment,
                                        sticky_penalty))
                    .solve();
  record_assignment_time(t_start, stats);
  return result;
}

TAPFAssignmentResult assign_tapf_tasks_dynamic(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    TAPFAssignmentState& state, const std::vector<int>& changed_agents,
    const bool force_full, TAPFAssignmentStats* stats)
{
  const auto t_start = Time::now();
  if (stats != nullptr) ++stats->calls;

  if (invalid_input(ins, C)) {
    record_assignment_time(t_start, stats);
    return invalid_result();
  }

  if (!state.ready() || state.org_n != static_cast<int>(ins.N) ||
      state.org_m != static_cast<int>(ins.tasks.size())) {
    state.init(ins.N, ins.tasks.size());
  }

  auto cost = [&](const int i, const int j) -> int {
    if (j >= static_cast<int>(ins.tasks.size())) return kTapfAssignmentInfCost;
    if (!ins.allowed[i][j]) return kTapfAssignmentInfCost;
    auto d = D.get(j, C[i]);
    if (d >= D.K) return kTapfAssignmentInfCost;
    return d;
  };

  auto result = force_full ? state.solve_full(cost)
                           : state.repair_rows(changed_agents, cost);
  record_assignment_time(t_start, stats);
  return result;
}
