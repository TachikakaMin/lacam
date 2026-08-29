// PROTECTED tests: B0 standalone rollout (design 8.1) and macro
// event-bounded rollout successors (design 7.1, D13).  TDD RED first.
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

TEST(dd_b0_rollout, solves_simple_carry)
{
  auto ins = make_ins({"....", "...."}, {{1, 0}}, {{0, 1}},
                      {{{0, 1}, {0, 3}}});
  DDStats st;
  auto plan = solve_carrier_rollout(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_b0_rollout, solves_blocker_clearing)
{
  auto ins = make_ins({".....", "....."}, {{1, 0}, {1, 4}},
                      {{0, 0}, {0, 1}, {0, 2}}, {{{0, 0}, {0, 4}}});
  DDStats st;
  auto plan = solve_carrier_rollout(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_b0_rollout, cycle_rotation_via_pibt_only)
{
  // PIBT immediate-recursion push chain must realize the 4-rotation
  // without any high-level search.
  auto ins = make_ins(
      {"..", ".."}, {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{0, 1}, {1, 1}}, {{1, 1}, {1, 0}}, {{1, 0}, {0, 0}}});
  DDStats st;
  auto plan = solve_carrier_rollout(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_b0_rollout, reports_failure_not_invalid_plan_when_stuck)
{
  // B0 has no search: a corridor case that needs backtracking may fail —
  // it must return an EMPTY plan (honest failure), never an invalid one.
  auto ins = make_ins(
      {"@.@", "@.@", "@.@", "..."},
      {{3, 0}, {3, 2}},
      {{3, 1}, {1, 1}},
      {{{3, 1}, {0, 1}}, {{1, 1}, {1, 1}}});
  DDStats st;
  auto plan = solve_carrier_rollout(ins, 2.0, 0, &st);
  if (!plan.empty()) EXPECT_TRUE(plan_is_valid(ins, plan));
}

TEST(dd_macro_rollout, macro_successors_used_and_plan_valid)
{
  // On a medium instance the searcher must insert macro (multi-step) edges
  // and still produce a validator-clean plan.  ddmapd-style layout.
  auto ins = make_ins(
      {"............", "............", "..@@........", "..@@........",
       "............", "............", "............", "............",
       "............", "............", "............", "............"},
      {{0, 0}, {0, 11}, {11, 0}, {11, 11}},
      {{2, 6}, {2, 7}, {3, 6}, {3, 7}, {8, 2}, {8, 3}, {9, 2}, {9, 3}},
      {{{2, 6}, {10, 10}}, {{8, 2}, {0, 5}}});
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
  EXPECT_GT(st.macro_successors, 0)
      << "macro rollout successors were never inserted (design 7.1)";
  EXPECT_GT(st.macro_steps, st.macro_successors)
      << "macro edges must contain multi-step traces";
}
