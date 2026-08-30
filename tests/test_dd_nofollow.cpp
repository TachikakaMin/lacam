// no-following semantic toggle (debug.md round-2 P2-16c, design 3.4a /
// 8.4 alignment experiment): DD_NO_FOLLOWING=1 makes the validator
// BRaP-conservative — no robot may enter a cell being vacated this step
// (lower deck), and no shelf may enter an upper cell being vacated this
// step.  Used ONLY for the semantics-alignment A/B; default stays off.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <cstdlib>

#include "gtest/gtest.h"

namespace {

DDInstance convoy_ins()
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots.push_back(ins.grid.idx(0, 0));
  ins.robots.push_back(ins.grid.idx(0, 1));
  ins.target_starts.push_back(ins.grid.idx(0, 1));
  ins.target_goals.push_back(ins.grid.idx(0, 3));
  ins.shelves.push_back(ins.grid.idx(0, 1));
  ins.finalize();
  return ins;
}

DDInstance cycle2x2_ins()
{
  DDInstance ins;
  ins.grid = DDGrid({"..", ".."});
  ins.robots = {0, 1, 2, 3};
  ins.target_starts.push_back(0);
  ins.target_goals.push_back(1);
  for (int c = 0; c < 4; ++c) ins.shelves.push_back(c);
  ins.finalize();
  return ins;
}

struct NoFollowGuard {
  NoFollowGuard() { setenv("DD_NO_FOLLOWING", "1", 1); }
  ~NoFollowGuard() { unsetenv("DD_NO_FOLLOWING"); }
};

}  // namespace

TEST(dd_nofollow, lower_deck_following_rejected_under_toggle)
{
  auto ins = convoy_ins();
  const auto X = initial_phys_config(ins);
  // r0 follows r1: legal by default, illegal under the toggle
  std::vector<Op> ops = {Op::make_move(ins.grid.idx(0, 1)),
                         Op::make_move(ins.grid.idx(0, 2))};
  EXPECT_TRUE(apply_ops(ins, X, ops).has_value());
  {
    NoFollowGuard g;
    EXPECT_FALSE(apply_ops(ins, X, ops).has_value())
        << "lower-deck following must be rejected under DD_NO_FOLLOWING";
    // non-following moves stay legal
    std::vector<Op> solo = {Op::make_wait(),
                            Op::make_move(ins.grid.idx(0, 2))};
    EXPECT_TRUE(apply_ops(ins, X, solo).has_value());
  }
}

TEST(dd_nofollow, upper_deck_following_rejected_under_toggle)
{
  // two carriers in convoy: shelf0 enters the upper cell shelf1 vacates
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots.push_back(ins.grid.idx(0, 0));
  ins.robots.push_back(ins.grid.idx(0, 1));
  ins.target_starts.push_back(ins.grid.idx(0, 0));
  ins.target_goals.push_back(ins.grid.idx(0, 2));
  ins.shelves.push_back(ins.grid.idx(0, 0));
  ins.target_starts.push_back(ins.grid.idx(0, 1));
  ins.target_goals.push_back(ins.grid.idx(0, 3));
  ins.shelves.push_back(ins.grid.idx(0, 1));
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa[0] = 0;
  X.kappa[1] = 1;
  std::vector<Op> ops = {Op::make_move(ins.grid.idx(0, 1)),
                         Op::make_move(ins.grid.idx(0, 2))};
  EXPECT_TRUE(apply_ops(ins, X, ops).has_value());
  {
    NoFollowGuard g;
    EXPECT_FALSE(apply_ops(ins, X, ops).has_value())
        << "upper-deck (carried-shelf) following must be rejected";
  }
}

TEST(dd_nofollow, zero_empty_cycle_unsolvable_under_toggle)
{
  auto ins = cycle2x2_ins();
  // default semantics: rotation solves it
  DDStats st1;
  auto plan = solve_carrier_lacam(ins, 2.0, 0, &st1);
  EXPECT_FALSE(plan.empty());
  {
    NoFollowGuard g;
    DDStats st2;
    auto plan2 = solve_carrier_lacam(ins, 1.0, 0, &st2);
    EXPECT_TRUE(plan2.empty())
        << "proposition-2 instance must be UNSOLVABLE without following";
  }
}
