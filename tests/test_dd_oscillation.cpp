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
