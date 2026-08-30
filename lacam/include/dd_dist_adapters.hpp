/*
 * DD (Carrier-LaCAM) adapters over the shared lazy-BFS core
 * (skeleton-reuse refactor #2).  Full-cell index space over DDGrid,
 * DD sentinel INT_MAX/2.
 */
#pragma once

#include <climits>
#include <unordered_map>

#include "dd_carrier.hpp"
#include "lazy_dist.hpp"

struct DDLazyDist : LazyBfsField<DDGrid> {
  DDLazyDist(const DDGrid& g, int src)
      : LazyBfsField<DDGrid>(g, src, INT_MAX / 2)
  {
  }
};

// drop-in for the DD planner\'s old DistCache: per-goal cached fields with
// the legacy `.to(goal)[cell]` full-vector view (forces full expansion,
// same values as the old first-query-full-BFS) plus a lazy
// `dist(goal, cell)` for call sites that migrate.
struct DDDistCache {
  const DDGrid& g;
  std::unordered_map<int, DDLazyDist> by_src;
  explicit DDDistCache(const DDGrid& g_) : g(g_) {}

  DDLazyDist& field(int src)
  {
    auto it = by_src.find(src);
    if (it == by_src.end())
      it = by_src.emplace(src, DDLazyDist(g, src)).first;
    return it->second;
  }
  int dist(int src, int cell) { return field(src).get(cell); }
  const std::vector<int>& to(int src)
  {
    auto& f = field(src);
    f.settle_all();
    return f.d;
  }
};
