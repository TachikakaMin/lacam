/*
 * Shared LaCAM low-level kernel pieces (skeleton-reuse #5 slice, audit
 * 2026-08-30).
 *
 * dd_expand_constraint(): the lazy operator-constraint tree step — pop is
 * done by the caller (loop shapes differ), this expands ONE constraint's
 * children according to the node's FROZEN variable order.  It is the
 * payload-generalized twin of the upstream step (planner.cpp low-level
 * search / tapf_planner.cpp): upstream fixes agent -> Vertex*, DD fixes
 * robot -> Op.  One implementation replaces the three copies that lived
 * in dd_planner.cpp (main loop + two enumeration test APIs); upstream can
 * adopt the same driver with a Vertex payload adapter.
 *
 * PARENT SEMANTICS GLOSSARY (audit prerequisite for deeper Node sharing;
 * also design.md §10):
 *   - search parent  (Node::parent): the node whose expansion CREATED this
 *     node.  Frozen at creation; feeds guidance ancestry (eta hysteresis,
 *     no-progress counters, livelock taboo).  NEVER rewired.
 *   - solution parent (parent_edge map): the best-known predecessor edge
 *     used for plan extraction; REWIRED by the LaCAM*-style g-relax and by
 *     multi-step macro edges (a trace of ops, not a single joint op).
 *   - guidance ancestry: reads the search parent's Guidance only (pure
 *     ordering); it must NOT follow solution-parent rewires, or identical
 *     configurations would receive history-dependent guidance again
 *     (park-registry lesson, design 5.4a).
 */
#pragma once

#include <memory>

// ConstraintT contract: parent/depth/who/what fields + kids (owning) —
// exactly the DD Constraint and (payload aside) the upstream one.
template <typename ConstraintT, typename Payload, typename LegalOps,
          typename Fifo>
inline void dd_expand_constraint(ConstraintT* c, int var,
                                 LegalOps&& legal_payloads, Fifo& fifo)
{
  for (const Payload& op : legal_payloads) {
    c->kids.push_back(std::make_unique<ConstraintT>());
    ConstraintT* k = c->kids.back().get();
    k->parent = c;
    k->depth = c->depth + 1;
    k->who = var;
    k->what = op;
    fifo.push_back(k);
  }
}
