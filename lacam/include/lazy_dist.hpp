/*
 * Shared lazy distance-field core (skeleton-reuse refactor #2, audit
 * 2026-08-30).
 *
 * This is the ORIGINAL DistTable resumable lazy BFS (c.f. Reverse
 * Resumable A*, AAAI AIIDE\'05 — see the pre-refactor dist_table.cpp)
 * hoisted into a topology-agnostic template so the upstream lacam
 * DistTable, the TAPF dist table and the DD (Carrier-LaCAM) planner
 * caches all share ONE implementation instead of three near-identical
 * BFS copies.
 *
 * Semantics are the original\'s, verbatim: unit-weight BFS where an
 * assigned tentative value is already exact (wave property), so get()
 * returns immediately once the target has ANY value; otherwise the
 * frontier resumes until the target is popped.  The unreachable sentinel
 * is the CALLER\'s convention (DistTable: K = vertex count; DD:
 * INT_MAX/2) and is passed in, never guessed.
 *
 * Topology concept: `size_t size() const` and
 * `int neighbors(int cell, int out[MAX_DEG]) const`.
 */
#pragma once

#include <deque>
#include <vector>

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

  // original DistTable::get shape: tentative value = final (unit BFS);
  // else resume expansion until the target is popped.
  int get(int target)
  {
    if (d[target] < unreachable) return d[target];
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

  // drain the frontier completely (legacy full-vector views)
  void settle_all()
  {
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
    }
  }

  int expanded() const { return n_expanded; }
};
