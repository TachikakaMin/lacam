#include "../include/tapf_assignment.hpp"

#include "../include/utils.hpp"

#include <limits>

namespace {
constexpr int kInfCost = 100000000;

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
      if (a[p[j]][j] >= kInfCost / 2) result.feasible = false;
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
}  // namespace

TAPFAssignmentResult assign_tapf_tasks(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    const std::vector<int>& previous_assignment, const int sticky_penalty,
    TAPFAssignmentStats* stats)
{
  const auto t_start = Time::now();
  if (stats != nullptr) ++stats->calls;

  auto finish = [&](TAPFAssignmentResult result) {
    if (stats != nullptr) {
      stats->time_ms +=
          std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() -
                                                               t_start)
              .count() /
          1000000.0;
    }
    return result;
  };

  if (ins.tasks.size() < ins.N || C.size() != ins.N) {
    return finish(TAPFAssignmentResult{std::vector<int>(), kInfCost, false});
  }

  auto cost = std::vector<std::vector<int> >(
      ins.N, std::vector<int>(ins.tasks.size(), kInfCost));

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

  return finish(HungarianAssignment(cost).solve());
}
