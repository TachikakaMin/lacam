// Skeleton-reuse refactor #2 (skeleton audit 2026-08-30): ONE lazy
// distance-field implementation — the original DistTable's resumable BFS
// (Reverse Resumable A*, AIIDE'05) hoisted into a topology-agnostic core —
// with DistTable and the DD planner's caches as thin adapters.
//
// Contract:
//   - exactness: lazily-queried distances equal full-BFS distances, in any
//     query order;
//   - resumability: a near query must NOT expand the whole graph (expanded
//     counter exposed), and later farther queries continue correctly;
//   - sentinel is the CALLER's (DD: INT_MAX/2, DistTable: K);
//   - multi-component graphs: unreachable cells return the sentinel.
#include <climits>
#include <dd_carrier.hpp>
#include <lazy_dist.hpp>

#include <queue>
#include <vector>

#include "gtest/gtest.h"

namespace {

std::vector<int> full_bfs(const DDGrid& g, int src)
{
  std::vector<int> d(g.size(), INT_MAX / 2);
  std::queue<int> q;
  d[src] = 0;
  q.push(src);
  int nb[4];
  while (!q.empty()) {
    int u = q.front();
    q.pop();
    const int n = g.neighbors(u, nb);
    for (int k = 0; k < n; ++k)
      if (d[nb[k]] > d[u] + 1) {
        d[nb[k]] = d[u] + 1;
        q.push(nb[k]);
      }
  }
  return d;
}

}  // namespace

TEST(lazy_dist, exact_in_any_query_order)
{
  DDGrid g({".....", ".###.", ".....", "....."});
  const int src = g.idx(0, 0);
  const auto oracle = full_bfs(g, src);
  DDLazyDist f(g, src);
  // farthest first, then scattered
  std::vector<int> order;
  for (int c = (int)g.size() - 1; c >= 0; --c) order.push_back(c);
  for (int c : order) {
    if (g.is_wall(c)) continue;
    EXPECT_EQ(f.get(c), oracle[c]) << "cell " << c;
  }
  // re-query after full expansion
  EXPECT_EQ(f.get(g.idx(3, 4)), oracle[g.idx(3, 4)]);
}

TEST(lazy_dist, resumable_near_query_expands_little)
{
  DDGrid g({"..........", "..........", "..........", "..........",
            ".........."});
  DDLazyDist f(g, g.idx(0, 0));
  EXPECT_EQ(f.get(g.idx(0, 1)), 1);
  EXPECT_LT(f.expanded(), (int)g.size() / 2)
      << "near query must not expand the whole grid (lazy contract)";
  // and the far corner still resolves exactly afterwards
  EXPECT_EQ(f.get(g.idx(4, 9)), 13);
}

TEST(lazy_dist, unreachable_uses_caller_sentinel)
{
  DDGrid g({"..#..", "..#..", "..#.."});  // two components
  DDLazyDist f(g, g.idx(0, 0));
  EXPECT_EQ(f.get(g.idx(0, 4)), INT_MAX / 2);
  EXPECT_EQ(f.get(g.idx(0, 1)), 1);  // own side still exact
}

TEST(lazy_dist, dd_distcache_adapter_full_vector_view)
{
  // DD call sites use tgd[b].to(goal)[cell]; the adapter's to() must give
  // the same values as the oracle over ALL cells.
  DDGrid g({".....", ".###.", "....."});
  const int goal = g.idx(2, 4);
  const auto oracle = full_bfs(g, goal);
  DDDistCache cache(g);
  const auto& vec = cache.to(goal);
  for (size_t c = 0; c < g.size(); ++c)
    if (!g.is_wall((int)c)) EXPECT_EQ(vec[c], oracle[c]) << c;
}
