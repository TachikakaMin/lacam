// Oscillation-suppression unit tests (debug.md round-2 P2-13b/c/d).
//
// 13b — rho hysteresis (design 5.3(4) eta term): re-matching must prefer
// keeping a robot on its parent-assignment request cell unless another
// robot is closer by more than eta.  Without hysteresis, near-tie requests
// flip robots every node and the carriers turn around mid-corridor (the
// reversals jitter measured at 27392 total on the dev set).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <vector>

#include "gtest/gtest.h"

namespace {

// serve requests at (0,0) and (0,5); robots at (0,2)/(0,3) are symmetric
// up to 1 step.  Fresh greedy: r0->(0,0), r1->(0,5).
DDInstance make_two_serve_ins()
{
  DDInstance ins;
  ins.grid = DDGrid({"......", "......"});
  ins.robots.push_back(ins.grid.idx(0, 2));
  ins.robots.push_back(ins.grid.idx(0, 3));
  ins.target_starts.push_back(ins.grid.idx(0, 0));
  ins.target_goals.push_back(ins.grid.idx(1, 0));
  ins.shelves.push_back(ins.grid.idx(0, 0));
  ins.target_starts.push_back(ins.grid.idx(0, 5));
  ins.target_goals.push_back(ins.grid.idx(1, 5));
  ins.shelves.push_back(ins.grid.idx(0, 5));
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_oscillation, rho_fresh_greedy_is_nearest)
{
  auto ins = make_two_serve_ins();
  const auto X = initial_phys_config(ins);
  const auto fg = dd_match_free_goals(ins, X, nullptr);
  ASSERT_EQ(fg.size(), 2u);
  EXPECT_EQ(fg[0], ins.grid.idx(0, 0));
  EXPECT_EQ(fg[1], ins.grid.idx(0, 5));
}

TEST(dd_oscillation, rho_hysteresis_keeps_parent_assignment)
{
  auto ins = make_two_serve_ins();
  const auto X = initial_phys_config(ins);
  // parent had the SWAPPED matching (e.g. robots crossed while walking);
  // switching back would gain only 1 step per robot < eta -> must stick.
  std::vector<int> parent_fg = {ins.grid.idx(0, 5), ins.grid.idx(0, 0)};
  const auto fg = dd_match_free_goals(ins, X, &parent_fg);
  ASSERT_EQ(fg.size(), 2u);
  EXPECT_EQ(fg[0], ins.grid.idx(0, 5))
      << "hysteresis must keep r0 on its parent request";
  EXPECT_EQ(fg[1], ins.grid.idx(0, 0))
      << "hysteresis must keep r1 on its parent request";
}

TEST(dd_oscillation, rho_hysteresis_yields_to_large_gain)
{
  auto ins = make_two_serve_ins();
  const auto X = initial_phys_config(ins);
  // parent assignments point BOTH robots at the far-left request; the
  // gain from re-matching r1 to (0,5) is 3 steps > eta -> re-match wins.
  // (parent had r0 unassigned, r1 -> (0,0).)
  std::vector<int> parent_fg = {-1, ins.grid.idx(0, 0)};
  const auto fg = dd_match_free_goals(ins, X, &parent_fg);
  ASSERT_EQ(fg.size(), 2u);
  // r1 keeps (0,0) via hysteresis (d=3 vs r0's d=2: with bonus r1 wins);
  // r0 then takes (0,5): stable, no flip-flop, both requests served.
  EXPECT_EQ(fg[1], ins.grid.idx(0, 0));
  EXPECT_EQ(fg[0], ins.grid.idx(0, 5));
}

// 13c — path inertia: on RECOMPUTE, equal-cost ties must break toward the
// previous path (per-edge epsilon discount strictly below one base cost
// unit in total), so guidance routes stop flapping between mirror detours.
TEST(dd_oscillation, path_inertia_breaks_ties_toward_prev)
{
  DDGrid g({".....", ".....", "....."});
  std::vector<uint8_t> occ(g.size(), 0);
  occ[g.idx(1, 2)] = 1;  // block the straight route: two mirror detours
  const int src = g.idx(1, 0), dst = g.idx(1, 4);

  const auto fresh = dd_least_blocking_path(g, src, dst, occ, nullptr);
  ASSERT_FALSE(fresh.empty());

  // previous path = the detour through row 2 (mirror of row 0)
  const std::vector<int> prev = {g.idx(1, 0), g.idx(2, 1), g.idx(2, 2),
                                 g.idx(2, 3), g.idx(1, 4)};
  // NOTE: prev need not be a connected valid path for the bias to apply;
  // use the exact mirror detour cells
  const std::vector<int> prev_down = {g.idx(1, 0), g.idx(2, 0), g.idx(2, 1),
                                      g.idx(2, 2), g.idx(2, 3), g.idx(2, 4),
                                      g.idx(1, 4)};
  const auto biased = dd_least_blocking_path(g, src, dst, occ, &prev_down);
  ASSERT_FALSE(biased.empty());
  EXPECT_EQ(biased.size(), fresh.size()) << "inertia must not lengthen";
  bool through_row2 = false;
  for (int c : biased) through_row2 |= (c == g.idx(2, 2));
  EXPECT_TRUE(through_row2)
      << "tie must break toward the previous (row-2) detour";
}

TEST(dd_oscillation, path_inertia_never_beats_real_cost)
{
  DDGrid g({".....", ".....", "....."});
  std::vector<uint8_t> occ(g.size(), 0);
  occ[g.idx(1, 2)] = 1;
  occ[g.idx(2, 2)] = 1;  // row-2 detour now costs one blocked cell
  const int src = g.idx(1, 0), dst = g.idx(1, 4);
  const std::vector<int> prev_down = {g.idx(1, 0), g.idx(2, 0), g.idx(2, 1),
                                      g.idx(2, 2), g.idx(2, 3), g.idx(2, 4),
                                      g.idx(1, 4)};
  const auto biased = dd_least_blocking_path(g, src, dst, occ, &prev_down);
  ASSERT_FALSE(biased.empty());
  bool through_row0 = false;
  for (int c : biased) through_row0 |= (c == g.idx(0, 2));
  EXPECT_TRUE(through_row0)
      << "a strictly cheaper route must beat inertia";
}

// 13d — idle avoidance (design 5.4 free-unassigned rules): an UNASSIGNED
// free robot standing on an active least-blocking path must prefer
// stepping OFF the corridor over waiting in it (wait-in-corridor gets it
// pushed every step -> jitter); off the corridor it keeps the wait-first
// policy.
TEST(dd_oscillation, idle_escapes_active_path)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "...", "..."});
  ins.robots.push_back(ins.grid.idx(1, 0));  // r0: under o, gets serve
  ins.robots.push_back(ins.grid.idx(1, 1));  // r1: idle ON o's path
  ins.target_starts.push_back(ins.grid.idx(1, 0));
  ins.target_goals.push_back(ins.grid.idx(1, 2));
  ins.shelves.push_back(ins.grid.idx(1, 0));
  ins.finalize();
  const auto X = initial_phys_config(ins);
  for (int seed : {0, 1, 2}) {
    const auto ops = dd_root_joint_ops(ins, X, seed);
    ASSERT_EQ(ops.size(), 2u);
    EXPECT_EQ(ops[1].kind, Op::MOVE)
        << "idle robot must step off the active corridor (seed " << seed
        << ")";
    if (ops[1].kind == Op::MOVE) {
      EXPECT_TRUE(ops[1].to == ins.grid.idx(0, 1) ||
                  ops[1].to == ins.grid.idx(2, 1))
          << "escape must leave the protected path/goal cells";
    }
  }
}

TEST(dd_oscillation, idle_off_path_keeps_wait_first)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "...", "..."});
  ins.robots.push_back(ins.grid.idx(1, 0));
  ins.robots.push_back(ins.grid.idx(2, 0));  // idle OFF the path
  ins.target_starts.push_back(ins.grid.idx(1, 0));
  ins.target_goals.push_back(ins.grid.idx(1, 2));
  ins.shelves.push_back(ins.grid.idx(1, 0));
  ins.finalize();
  const auto X = initial_phys_config(ins);
  for (int seed : {0, 1, 2}) {
    const auto ops = dd_root_joint_ops(ins, X, seed);
    ASSERT_EQ(ops.size(), 2u);
    EXPECT_EQ(ops[1].kind, Op::WAIT)
        << "off-corridor idle keeps the wait-first policy (seed " << seed
        << ")";
  }
}

// P2-16a — min-cost rho (Hungarian) vs greedy nearest: the classic
// greedy-suboptimal fixture.  Requests at cells A=(0,0), B=(0,3);
// robots r0=(0,1), r1=(0,2).  Greedy (priority order, nearest robot):
// A takes r0 (d1), B takes r1 (d1) -> total 2 ... need asymmetric case:
// A=(0,0), B=(0,2); r0=(0,1), r1=(0,3).  Greedy: A->r0 (1), B->r1 (1),
// total 2 = optimal, still fine.  True separation: A=(0,1), B=(0,2);
// r0=(0,0), r1=(0,3): greedy A->r0(1) B->r1(1)=2 optimal again...
// Separation needs priority-order mismatch: HIGH-priority request far
// from everyone steals the only close robot of the LOW one.
//   serve S at (0,5) (priority 100), clear C at (0,0) (priority 50);
//   robots r0=(0,4), r1=(0,3).
//   greedy: S first -> r0 (d1); C -> r1 (d3). total 4.
//   optimal: S -> r0 (1), C -> r1 (3) ... identical.  For a real swap:
//   r0=(0,1), r1=(0,4):  greedy: S->r1 (1), C->r0 (1) total 2 optimal.
//   r0=(0,2), r1=(0,3):  greedy: S->r1 (2), C->r0 (2) total 4 optimal.
// Greedy-by-priority equals min-cost only when the cost matrix is a
// permutation-friendly Monge-ish; break it with THREE requests and TWO
// robots: priorities force greedy to serve the two HIGH requests with
// the two robots at large cost, while min-cost picks the same requests
// cheaper by swapping.
//   requests: S1 (0,0) prio 100, S2 (0,5) prio 100 (tie, order by index)
//   robots:   r0 (0,4), r1 (0,1)
//   greedy: S1 -> r1 (d1)?  nearest to S1 is r1 (1) -> S1:r1;
//           S2 -> r0 (1).  total 2 — optimal.  greedy nearest is optimal
//           for 2x2 unless the first pick blocks the second:
//   robots: r0 (0,2), r1 (0,3):
//           greedy S1 -> r0 (2), S2 -> r1 (2) = 4; optimal same.
//   The failing shape: S1 prio-first picks a robot that S2 NEEDS:
//   S1 (0,3), S2 (0,0); robots r0 (0,2), r1 (0,4).
//     greedy: S1 nearest = r1 (1) [r0 d1 too — tie, lower idx r0 wins!]
//     -> S1:r0 (1), S2 gets r1 (4). total 5.
//     optimal: S1:r1 (1), S2:r0 (2). total 3.  <-- separation
TEST(dd_oscillation, hungarian_rho_beats_greedy_on_crossing_fixture)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.robots.push_back(ins.grid.idx(0, 2));  // r0
  ins.robots.push_back(ins.grid.idx(0, 4));  // r1
  // two serve requests with equal priority at (0,3) and (0,0)
  ins.target_starts.push_back(ins.grid.idx(0, 3));
  ins.target_goals.push_back(ins.grid.idx(1, 3));
  ins.shelves.push_back(ins.grid.idx(0, 3));
  ins.target_starts.push_back(ins.grid.idx(0, 0));
  ins.target_goals.push_back(ins.grid.idx(1, 0));
  ins.shelves.push_back(ins.grid.idx(0, 0));
  ins.finalize();
  const auto X = initial_phys_config(ins);

  // greedy mode (DD_RHO_HUNGARIAN=0; min-cost is the default since the
  // dev A/B win: mk -10%, reversals -25%): request order (tie -> index)
  // lets S1 grab r0, pushing S2 to pay 4.
  setenv("DD_RHO_HUNGARIAN", "0", 1);
  const auto greedy = dd_match_free_goals(ins, X, nullptr);
  const int A = ins.grid.idx(0, 3), B = ins.grid.idx(0, 0);
  ASSERT_EQ(greedy.size(), 2u);
  EXPECT_EQ(greedy[0], A);
  EXPECT_EQ(greedy[1], B);

  // min-cost (default) must produce the crossing-free assignment
  // (total 3): r0 -> B (d2), r1 -> A (d1)
  unsetenv("DD_RHO_HUNGARIAN");
  const auto opt = dd_match_free_goals(ins, X, nullptr);
  ASSERT_EQ(opt.size(), 2u);
  EXPECT_EQ(opt[0], B) << "min-cost rho must swap the crossing pair";
  EXPECT_EQ(opt[1], A);
}

// P2-16d — placement score variant (design 5.6 NEW hypothesis, untested
// there): among nearest parking candidates, prefer the cell with the
// higher ESCAPE DEGREE (free upper-deck neighbors) so parked shelves do
// not wall themselves in.  Distance stays primary (the v2.2-rejected
// margin/corridor scoring traded distance and regressed d50).
TEST(dd_oscillation, placement_escape_degree_tiebreak)
{
  // o's path runs along row 1 (protected); the anon carrier hovers at
  // (0,2) OFF that path so the hover mask cannot reroute it.  Depth-1
  // parking candidates: (0,3) first in BFS order (down,up,right,left)
  // with escape degree 1 (boxed by the anon at (0,4) + hover), and (0,1)
  // with escape degree 2.  Default = nearest-first -> (0,3); the
  // DD_PLACE_ESCAPE tie-break must pick (0,1).
  DDInstance ins;
  ins.grid = DDGrid({".....", ".....", "....."});
  ins.robots.push_back(ins.grid.idx(0, 2));   // anon carrier
  ins.robots.push_back(ins.grid.idx(2, 4));   // spare robot
  ins.target_starts.push_back(ins.grid.idx(1, 0));
  ins.target_goals.push_back(ins.grid.idx(1, 4));
  ins.shelves.push_back(ins.grid.idx(1, 0));
  ins.shelves.push_back(ins.grid.idx(0, 4));  // boxes in (0,3)
  ins.shelves.push_back(ins.grid.idx(0, 2));  // the shelf r0 carries
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa[0] = KAPPA_ANON;
  X.anon_occ.erase(std::find(X.anon_occ.begin(), X.anon_occ.end(),
                             ins.grid.idx(0, 2)));

  unsetenv("DD_PLACE_ESCAPE");
  EXPECT_EQ(dd_parking_cell(ins, X, 0), ins.grid.idx(0, 3));

  setenv("DD_PLACE_ESCAPE", "1", 1);
  const int esc = dd_parking_cell(ins, X, 0);
  unsetenv("DD_PLACE_ESCAPE");
  EXPECT_EQ(esc, ins.grid.idx(0, 1))
      << "escape-degree tie-break must prefer the open cell";
}
