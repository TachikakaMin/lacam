// PROTECTED tests: Carrier-LaCAM planner (design.md sections 5, 6.5).
// Written BEFORE implementation (TDD RED).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

namespace {

DDInstance make_ins(const std::vector<std::string>& rows,
                    const std::vector<std::pair<int, int>>& robots,
                    const std::vector<std::pair<int, int>>& shelves,
                    const std::vector<std::pair<std::pair<int, int>,
                                                std::pair<int, int>>>& targets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  for (auto& q : robots) ins.robots.push_back(ins.grid.idx(q.first, q.second));
  for (auto& p : shelves)
    ins.shelves.push_back(ins.grid.idx(p.first, p.second));
  for (auto& t : targets) {
    ins.target_starts.push_back(ins.grid.idx(t.first.first, t.first.second));
    ins.target_goals.push_back(ins.grid.idx(t.second.first, t.second.second));
  }
  ins.finalize();
  return ins;
}

// replay plan through the validator: every step legal + final state is goal
::testing::AssertionResult plan_is_valid(const DDInstance& ins,
                                         const DDPlan& plan)
{
  if (plan.empty()) return ::testing::AssertionFailure() << "empty plan";
  auto s = initial_phys_config(ins);
  for (size_t t = 0; t < plan.size(); ++t) {
    auto nxt = apply_ops(ins, s, plan[t]);
    if (!nxt.has_value())
      return ::testing::AssertionFailure() << "illegal joint op at t=" << t;
    s = *nxt;
  }
  if (!is_dd_goal(ins, s))
    return ::testing::AssertionFailure() << "final state not goal";
  return ::testing::AssertionSuccess();
}

}  // namespace

TEST(dd_planner, single_robot_single_target)
{
  auto ins = make_ins({"....", "...."}, {{1, 0}}, {{0, 1}},
                      {{{0, 1}, {0, 3}}});
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_planner, single_robot_with_blocker)
{
  // theorem-1 style: blocker on the only short path must be cleared
  auto ins = make_ins({".....", "....."}, {{1, 0}},
                      {{0, 0}, {0, 1}, {0, 2}}, {{{0, 0}, {0, 4}}});
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_planner, cycle_rotation_zero_empty_cells)
{
  // Proposition 2 separation instance: requires convoy rotation.
  auto ins = make_ins(
      {"..", ".."}, {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{0, 1}, {1, 1}}, {{1, 1}, {1, 0}}, {{1, 0}, {0, 0}}});
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_planner, idle_robot_on_lift_cell)
{
  // idle robot parked under the target shelf: must move away first
  auto ins = make_ins({"...", "..."}, {{0, 1}, {2 - 2, 0}}, {{0, 1}},
                      {{{0, 1}, {0, 2}}});
  // robots: r0 AT the shelf cell (lower deck), r1 at (0,0)
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_planner, already_solved_returns_empty_but_valid_flagged)
{
  auto ins = make_ins({"..."}, {{0, 0}}, {{0, 1}}, {{{0, 1}, {0, 1}}});
  // start state is already goal: planner returns a trivial 0/1-step plan;
  // contract: non-"empty means failure" -> represent as plan with single
  // all-wait step
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  auto s = initial_phys_config(ins);
  auto nxt = apply_ops(ins, s, plan[0]);
  ASSERT_TRUE(nxt.has_value());
  EXPECT_TRUE(is_dd_goal(ins, *nxt));
}

TEST(dd_planner, two_targets_swap_positions)
{
  // b0 and b1 must swap cells (needs parking): classic hard-ish micro case
  auto ins = make_ins({"...", "..."}, {{1, 0}, {1, 2}},
                      {{0, 0}, {0, 2}},
                      {{{0, 0}, {0, 2}}, {{0, 2}, {0, 0}}});
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}
