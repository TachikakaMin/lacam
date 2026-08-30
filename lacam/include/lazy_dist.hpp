/*
 * Shared lazy distance-field core (skeleton-reuse refactor #2, audit
 * 2026-08-30).
 *
 * This is the ORIGINAL DistTable resumable lazy BFS (c.f. Reverse
 * Resumable A*, AAAI AIIDE'05 — see dist_table.cpp) hoisted into a
 * topology-agnostic template so that the upstream lacam DistTable, the
 * TAPF dist table and the DD (Carrier-LaCAM) planner caches all share ONE
 * implementation instead of three near-identical BFS copies.
 *
 * Topology concept: `int neighbors(int cell, int out[MAX_DEG]) const` and
 * `size_t size() const`.  The unreachable sentinel is the CALLER's
 * convention (DistTable: K = vertex count; DD: INT_MAX/2) and is passed
 * in, never guessed.
 */
#pragma once

#include <climits>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include "dd_carrier.hpp"

template <typename Topology, int MAX_DEG = 4>
struct LazyBfsField {
  const Topology& g;
  const int unreachable;
  std::vector<int> d;
  std::deque<int> open;
  int n_expanded = 0;

  LazyBfsField(const Topology& g_, int src, int unreachable_v)
      : g(g_), unreachable(unreachable_v), d(g_.size(), unreachable_v)
  {
    d[src] = 0;
    open.push_back(src);
  }

  // resumable: expand until `target` is settled (popped), exactly the
  // original DistTable::get loop shape — settled-on-pop, early return.
  int get(int target)
  {
    if (d[target] < unreachable && settled(target)) return d[target];
    int nb[MAX_DEG];
    while (!open.empty()) {
      const int u = open.front();
      open.pop_front();
      ++n_expanded;
      const int du = d[u];
      const int n = g.neighbors(u, nb);
      for (int k = 0; k < n; ++k) {
        const int m = nb[k];
        if (du + 1 >= d[m]) continue;
        d[m] = du + 1;
        open.push_back(m);
      }
      if (u == target) return du;
    }
    return d[target];  // exhausted: exact value or the sentinel
  }

  int expanded() const { return n_expanded; }

 private:
  // a cell is settled once it can no longer improve: with unit weights,
  // anything still in OPEN has d >= front's d - 1; simplest exact rule
  // matching the original: only trust values after the cell was popped.
  // We track that implicitly: a value is final iff it is < the current
  // BFS frontier lower bound.  Frontier bound = d[open.front()].
  bool settled(int target) const
  {
    if (open.empty()) return true;
    return d[target] < d[open.front()];
  }
};

// ---- DD adapters (full-cell index space over DDGrid) ----

// per-source resumable field, DD sentinel INT_MAX/2
struct DDLazyDist : LazyBfsField<DDGrid> {
  DDLazyDist(const DDGrid& g, int src)
      : LazyBfsField<DDGrid>(g, src, INT_MAX / 2)
  {
  }
};

// drop-in for the DD planner's old DistCache: per-goal cached fields with
// the legacy `.to(goal)[cell]` full-vector view (forces full expansion,
// same behavior/perf as the old first-query-full-BFS) plus a lazy
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
    // full-vector view: settle everything once (legacy DD semantics)
    while (!f.open.empty()) {
      const int u = f.open.front();
      (void)f.get(u);
    }
    return f.d;
  }
};
