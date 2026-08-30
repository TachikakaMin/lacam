/*
 * Shared LaCAM low-level kernel pieces.
 *
 * lacam_expand_constraint_vec(): the lazy constraint-tree expansion step
 * (vector-copy Constraint form) shared by planner.cpp, tapf_planner.cpp
 * and the carrier G1-conformance drain (dd_planner.cpp adapters).
 * focal_select_index(): FOCAL open-list selection shared by the same.
 * LacamNodeCore: high-level node fields + priority machinery shared by
 * the upstream Node and TAPFNode.
 *
 * PARENT SEMANTICS GLOSSARY (design.md section 10):
 *   - search parent (Node::parent): the node whose expansion CREATED this
 *     node; feeds guidance ancestry.  May be REWIRED by the TAPF rewrite
 *     (g-relax) for plan extraction — carrier guidance reads it only at
 *     node creation, so rewires never re-feed guidance (design 5.4a
 *     park-registry lesson).
 *   - macro edges (multi-step): stored beside the tree in
 *     TAPFPlanner::macro_edges with their trace and cost.
 */
#pragma once

#include <algorithm>
#include <memory>
#include <numeric>
#include <queue>
#include <vector>

#include "graph.hpp"

// vector-copy form driver (upstream Constraint semantics): children are
// heap-allocated, caller owns collection (upstream GC vector).  Candidate
// container is prepared (incl. shuffle) by the caller.
template <typename ConstraintT, typename Cand, typename Queue>
inline void lacam_expand_constraint_vec(ConstraintT* M, int agent,
                                        const Cand& candidates, Queue& q)
{
  for (auto u : candidates) q.push(new ConstraintT(M, agent, u));
}

// FOCAL open-list selection (upstream tapf_planner select_open_index
// shape, node-skeleton audit step 4): among viable nodes compute f_min,
// admit f <= focal_weight * f_min, pick the tie-break winner; fall back to
// the stack top (open.size()-1) when nothing qualifies.  Shared verbatim
// by tapf_planner (unsigned f, focal_better ties) and the DD planner
// (double g+h_adm, min-h ties).
template <typename NodeP, typename FVal, typename Viable, typename Better>
inline size_t focal_select_index(const std::vector<NodeP>& open,
                                 double focal_weight, FVal&& f_of,
                                 Viable&& viable, Better&& better)
{
  double f_min = -1;
  for (const auto& n : open) {
    if (!viable(n)) continue;
    const double f = f_of(n);
    if (f_min < 0 || f < f_min) f_min = f;
  }
  if (f_min < 0) return open.size() - 1;
  const double bound = focal_weight * f_min;
  size_t best = open.size();
  for (size_t idx = 0; idx < open.size(); ++idx) {
    const auto& n = open[idx];
    if (!viable(n)) continue;
    if (f_of(n) > bound + 1e-9) continue;
    if (best == open.size() || better(n, open[best])) best = idx;
  }
  return best == open.size() ? open.size() - 1 : best;
}

// LacamNodeCore: the high-level search node fields and priority machinery
// shared VERBATIM by the upstream Node (planner.cpp) and TAPFNode
// (tapf_planner.cpp) — previously duplicated twins (node-skeleton audit
// step 5).  Derived supplies the distance keyed however it likes (agent id
// upstream, task assignment for TAPF) via init_priorities_and_order.
// The carrier layer extends TAPFNode with a ShelfState member alongside
// this core (the former standalone DD Node was deleted at the v3
// integration cutover — design.md §10).
template <typename ConstraintT, typename Derived>
struct LacamNodeCore {
  const Config C;
  Derived* parent;
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<ConstraintT*> search_tree;

  LacamNodeCore(Config _C, Derived* _parent)
      : C(_C),
        parent(_parent),
        priorities(C.size(), 0),
        order(C.size(), 0),
        search_tree(std::queue<ConstraintT*>())
  {
    search_tree.push(new ConstraintT());
  }

  ~LacamNodeCore()
  {
    while (!search_tree.empty()) {
      delete search_tree.front();
      search_tree.pop();
    }
  }

  // verbatim upstream priority inheritance (PIBT-style) + order sort;
  // dist_of(i) = remaining distance of robot/agent i at C[i]
  template <typename DistOf>
  void init_priorities_and_order(DistOf&& dist_of)
  {
    const auto N = C.size();
    if (parent == nullptr) {
      for (size_t i = 0; i < N; ++i)
        priorities[i] = (float)dist_of(i) / N;
    } else {
      for (size_t i = 0; i < N; ++i) {
        if (dist_of(i) != 0) {
          priorities[i] = parent->priorities[i] + 1;
        } else {
          priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
        }
      }
    }
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int i, int j) { return priorities[i] > priorities[j]; });
  }
};
