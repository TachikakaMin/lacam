// Cross-deck wait-for graph (debug.md round-2 P2-15, design.md 5.5).
//
// Edges (one out-edge per robot, functional graph):
//   robot i -> robot j            if i's intended next cell is occupied by
//                                 robot j (lower deck);
//   robot i -> clear-assignee j   if i is a carrier whose next upper cell
//                                 holds a GROUNDED shelf and robot j is
//                                 rho-assigned to that shelf cell (i waits
//                                 for the shelf, the shelf waits for its
//                                 clearer);
// Cycles = structural deadlocks the PIBT push cannot resolve in one step.
// The livelock diversification taboos exactly the cycle members' rho
// pairs (targeted re-match) instead of the blanket parent taboo.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_waitfor, head_on_carrier_is_resolved_by_yield_before_cycle_detection)
{
  // two loaded carriers in a 1-wide corridor moving toward each other:
  // the fixed production yield rule parks one side before the residual
  // wait-for graph is built.
  DDInstance ins;
  ins.grid = DDGrid({"....", "####"});
  ins.robots.push_back(ins.grid.idx(0, 1));
  ins.robots.push_back(ins.grid.idx(0, 2));
  ins.target_starts.push_back(ins.grid.idx(0, 1));
  ins.target_goals.push_back(ins.grid.idx(0, 3));
  ins.shelves.push_back(ins.grid.idx(0, 1));
  ins.target_starts.push_back(ins.grid.idx(0, 2));
  ins.target_goals.push_back(ins.grid.idx(0, 0));
  ins.shelves.push_back(ins.grid.idx(0, 2));
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa[0] = 0;  // r0 carries b0 (start cells coincide)
  X.kappa[1] = 1;  // r1 carries b1
  const auto cyc = dd_waitfor_cycle_robots(ins, X);
  EXPECT_TRUE(cyc.empty());
}

TEST(dd_waitfor, cross_deck_cycle_detected)
{
  // r0 carries b0 toward a grounded anon shelf s; r1 is assigned to clear
  // s but its own next cell is r0's current cell: r0 -> (s) -> r1 -> r0.
  DDInstance ins;
  ins.grid = DDGrid({"....", "####"});
  ins.robots.push_back(ins.grid.idx(0, 1));  // r0, will carry b0
  ins.robots.push_back(ins.grid.idx(0, 0));  // r1, free clearer
  ins.target_starts.push_back(ins.grid.idx(0, 1));
  ins.target_goals.push_back(ins.grid.idx(0, 3));
  ins.shelves.push_back(ins.grid.idx(0, 1));
  ins.shelves.push_back(ins.grid.idx(0, 2));  // anon blocker s on b0's path
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa[0] = 0;
  // r1's request should be clear(s) at (0,2); its path to (0,2) goes
  // through r0's cell (0,1) in the 1-wide corridor -> cycle {0, 1}.
  const auto cyc = dd_waitfor_cycle_robots(ins, X);
  EXPECT_EQ(cyc, (std::vector<int>{0, 1}));
}

TEST(dd_waitfor, free_flow_has_no_cycle)
{
  DDInstance ins;
  ins.grid = DDGrid({"....", "...."});
  ins.robots.push_back(ins.grid.idx(0, 0));
  ins.robots.push_back(ins.grid.idx(1, 3));
  ins.target_starts.push_back(ins.grid.idx(0, 1));
  ins.target_goals.push_back(ins.grid.idx(0, 3));
  ins.shelves.push_back(ins.grid.idx(0, 1));
  ins.finalize();
  const auto X = initial_phys_config(ins);
  const auto cyc = dd_waitfor_cycle_robots(ins, X);
  EXPECT_TRUE(cyc.empty());
}
