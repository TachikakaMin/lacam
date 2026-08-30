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

// Two constraint representations coexist in this codebase and each gets a
// shared expansion driver (node-skeleton audit 2026-08-30):
//   - vector-copy form (upstream planner.cpp / tapf_planner.cpp Constraint:
//     who/where vectors copied per child)  -> lacam_expand_constraint_vec
//   - parent-chain form (DD Constraint: single who/what + owning kids)
//     -> dd_expand_constraint
// ConstraintT contract for dd_expand_constraint: parent/depth/who/what
// fields + kids (owning).
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

// pibt_recurse(): the PIBT reserve/recurse/backtrack frame — candidates in
// preference order, feasibility veto, tentative reservation, immediate
// recursion on an undecided occupant of a move destination, backtrack to
// the next candidate on failure, wait as the last resort.
//
// HONEST LINEAGE NOTE (node-skeleton audit 2026-08-30): this follows the
// funcPIBT control IDEA but is NOT semantically interchangeable with the
// upstream implementations; documented deltas:
//   - on occupant-recursion failure DD un-reserves and tries the next
//     candidate (upstream keeps the occupied_next reservation);
//   - the wait fallback returns true here (upstream reserves v_now and
//     returns false, letting the pusher retry);
//   - no LaCAM2 swap hook (DD covers head-ons via yield/wait-for instead).
// Upstream funcPIBT therefore does NOT call this frame yet; making it
// faithful (reservation-keeping + wait-fail + swap hook) is the recorded
// precondition for upstream adoption (debug.md P1-17 #6 后续).
// Domain decisions behind Ctx hooks:
//   candidates(robot) / feasible(robot, op) / fix / unfix
//   move_dest(op) -> destination cell or -1   (non-moves)
//   undecided_occupant(cell, robot) -> robot id or -1
//   wait_op()
// DD supplies the two-deck Carrier policy; upstream can adopt with a
// Vertex-payload context (deferred, same vector-form reason as the
// constraint driver — see design.md §10).
template <typename Ctx, typename OpT>
bool pibt_recurse(Ctx& ctx, const std::vector<int>& order, int i,
                  const std::vector<OpT>* forced, int depth_guard,
                  int max_depth)
{
  if (depth_guard > max_depth) return false;
  const int robot = order[i];
  if (ctx.decided(robot)) return true;

  std::vector<OpT> cand;
  if (forced && ctx.has_forced(*forced, robot)) {
    cand = {(*forced)[robot]};
  } else {
    cand = ctx.candidates(robot);
  }
  for (const OpT& op : cand) {
    if (!ctx.feasible(robot, op)) continue;
    const int dest = ctx.move_dest(op);
    const int occupant =
        dest >= 0 ? ctx.undecided_occupant(dest, robot) : -1;
    ctx.fix(robot, op);
    if (occupant >= 0) {
      int occ_pos = -1;
      for (size_t k2 = 0; k2 < order.size(); ++k2)
        if (order[k2] == occupant) occ_pos = (int)k2;
      if (occ_pos < 0 || !pibt_recurse<Ctx, OpT>(ctx, order, occ_pos, forced,
                                                 depth_guard + 1,
                                                 max_depth)) {
        ctx.unfix(robot, op);
        continue;
      }
    }
    return true;
  }
  const OpT w = ctx.wait_op();
  if (ctx.feasible(robot, w)) {
    ctx.fix(robot, w);
    return true;
  }
  return false;
}

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
