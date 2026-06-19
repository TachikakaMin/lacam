#include "../include/tapf_assignment.hpp"

#include "../include/utils.hpp"

namespace {
class HungarianAssignment {
 public:
  explicit HungarianAssignment(const std::vector<std::vector<int> >& cost)
      : n(cost.size()),
        m(cost.empty() ? 0 : cost.front().size()),
        a(n + 1, std::vector<int>(m + 1, 0)),
        u(n + 1, 0),
        v(m + 1, 0),
        p(m + 1, 0),
        way(m + 1, 0)
  {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) a[i + 1][j + 1] = cost[i][j];
    }
  }

  TAPFAssignmentResult solve()
  {
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

    auto result = TAPFAssignmentResult();
    result.agent_to_task = std::vector<int>(n, -1);
    result.cost = 0;
    result.feasible = true;
    for (size_t j = 1; j <= m; ++j) {
      if (p[j] == 0) continue;
      const auto i = p[j] - 1;
      const auto task = j - 1;
      result.agent_to_task[i] = task;
      result.cost += a[p[j]][j];
      if (a[p[j]][j] >= kTapfAssignmentInfCost / 2) result.feasible = false;
    }
    return result;
  }

 private:
  size_t n;
  size_t m;
  std::vector<std::vector<int> > a;
  std::vector<int> u;
  std::vector<int> v;
  std::vector<int> p;
  std::vector<int> way;
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

long long row_cache_key(int agent_id, Vertex* cell)
{
  const auto cell_id = cell == nullptr ? -1 : cell->id;
  return (static_cast<long long>(agent_id) << 32) ^
         static_cast<unsigned int>(cell_id);
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
      const auto offset = ins.assignment_cost_offsets[i][j];
      const auto distance_scale = ins.assignment_distance_scales.empty()
                                      ? ins.assignment_distance_scale
                                      : ins.assignment_distance_scales[i][j];
      const auto scaled_distance =
          static_cast<long long>(d) * distance_scale;
      if (scaled_distance + offset >= kTapfAssignmentInfCost) continue;
      cost[i][j] = static_cast<int>(scaled_distance + offset);
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
    const bool force_full, TAPFAssignmentStats* stats,
    const std::vector<int>& fixed_task_by_agent,
    const std::vector<bool>& unavailable_tasks)
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

  auto build_row = [&](const int i) {
    auto row =
        std::vector<int>(ins.tasks.size(), kTapfAssignmentInfCost);
    for (size_t j = 0; j < ins.tasks.size(); ++j) {
      if (!ins.allowed[i][j]) continue;
      auto d = D.get(j, C[i]);
      if (d >= D.K) continue;
      const auto offset = ins.assignment_cost_offsets[i][j];
      const auto distance_scale = ins.assignment_distance_scales.empty()
                                      ? ins.assignment_distance_scale
                                      : ins.assignment_distance_scales[i][j];
      const auto scaled_distance =
          static_cast<long long>(d) * distance_scale;
      if (scaled_distance + offset >= kTapfAssignmentInfCost) continue;
      row[j] = static_cast<int>(scaled_distance + offset);
    }
    return row;
  };

  auto cost = [&](const int i, const int j) -> int {
    if (j >= static_cast<int>(ins.tasks.size())) return kTapfAssignmentInfCost;
    const auto fixed = i < static_cast<int>(fixed_task_by_agent.size())
                           ? fixed_task_by_agent[i]
                           : -1;
    if (fixed >= 0 && j != fixed) return kTapfAssignmentInfCost;
    if (fixed < 0 && j < static_cast<int>(unavailable_tasks.size()) &&
        unavailable_tasks[j]) {
      return kTapfAssignmentInfCost;
    }
    if (stats != nullptr) ++stats->row_cache_requests;
    const auto key = row_cache_key(i, C[i]);
    auto iter = state.row_cost_cache.find(key);
    if (iter == state.row_cost_cache.end()) {
      iter = state.row_cost_cache.emplace(key, build_row(i)).first;
    } else if (stats != nullptr) {
      ++stats->row_cache_hits;
    }
    return iter->second[j];
  };

  auto result = force_full ? state.solve_full(cost)
                           : state.repair_rows(changed_agents, cost);
  record_assignment_time(t_start, stats);
  return result;
}

TAPFAssignmentResult assign_hungarian_cost_matrix(
    const std::vector<std::vector<int> >& cost)
{
  if (cost.empty()) return TAPFAssignmentResult{std::vector<int>(), 0, true};
  const auto width = cost.front().size();
  if (width == 0 || width < cost.size()) return invalid_result();
  for (const auto& row : cost) {
    if (row.size() != width) return invalid_result();
  }
  return HungarianAssignment(cost).solve();
}
