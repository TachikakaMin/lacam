// PROTECTED tests: goal-set instance layer + terminal condition
// (design_final 2.1/2.2/Prop 3; debug.md v4 WP-A, T1/T2).
// Written BEFORE implementation (TDD RED).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>

#include "gtest/gtest.h"

namespace {

// hand-built instance with per-target goal SETS (row/col pairs)
DDInstance make_set_ins(
    const std::vector<std::string>& rows,
    const std::vector<std::pair<int, int>>& robots,
    const std::vector<std::pair<int, int>>& shelves,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::vector<std::pair<int, int>>>& goal_sets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  for (auto& q : robots) ins.robots.push_back(ins.grid.idx(q.first, q.second));
  for (auto& p : shelves)
    ins.shelves.push_back(ins.grid.idx(p.first, p.second));
  for (size_t b = 0; b < starts.size(); ++b) {
    ins.target_starts.push_back(
        ins.grid.idx(starts[b].first, starts[b].second));
    std::vector<int> set;
    for (auto& g : goal_sets[b])
      set.push_back(ins.grid.idx(g.first, g.second));
    ins.target_goal_sets.push_back(set);
    ins.target_goals.push_back(set.front());  // provisional representative
  }
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_goalset, loader_parses_pool_list_and_singleton)
{
  const auto path = std::string(DD_TEST_DIR) + "/fixtures/dd_goalset.yaml";
  const auto ins = load_dd_instance(path);
  ASSERT_EQ(ins.n_targets(), 3u);
  ASSERT_EQ(ins.target_goal_sets.size(), 3u);
  // old singleton form -> {goal}
  EXPECT_EQ(ins.target_goal_sets[0],
            (std::vector<int>{ins.grid.idx(0, 4)}));
  // explicit list, given unsorted -> sorted unique
  EXPECT_EQ(ins.target_goal_sets[1],
            (std::vector<int>{ins.grid.idx(1, 3), ins.grid.idx(1, 4)}));
  // pool reference -> the shared pool, sorted
  EXPECT_EQ(ins.target_goal_sets[2],
            (std::vector<int>{ins.grid.idx(2, 3), ins.grid.idx(2, 4)}));
  // representative view = first element of the sorted set
  EXPECT_EQ(ins.target_goals[0], ins.grid.idx(0, 4));
  EXPECT_EQ(ins.target_goals[1], ins.grid.idx(1, 3));
  EXPECT_EQ(ins.target_goals[2], ins.grid.idx(2, 3));
}

TEST(dd_goalset, loader_old_format_yields_singleton_sets)
{
  const auto path = std::string(DD_TEST_DIR) + "/fixtures/dd_tiny.yaml";
  const auto ins = load_dd_instance(path);
  ASSERT_EQ(ins.target_goal_sets.size(), ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b)
    EXPECT_EQ(ins.target_goal_sets[b],
              (std::vector<int>{ins.target_goals[b]}));
}

TEST(dd_goalset, finalize_materializes_singletons_from_goals)
{
  // hand-built instances that only fill target_goals (all existing tests
  // and adapters do this) must keep working: finalize materializes sets
  DDInstance ins;
  ins.grid = DDGrid({"....", "...."});
  ins.robots.push_back(ins.grid.idx(1, 0));
  ins.shelves.push_back(ins.grid.idx(0, 0));
  ins.target_starts.push_back(ins.grid.idx(0, 0));
  ins.target_goals.push_back(ins.grid.idx(0, 3));
  ins.finalize();
  ASSERT_EQ(ins.target_goal_sets.size(), 1u);
  EXPECT_EQ(ins.target_goal_sets[0],
            (std::vector<int>{ins.grid.idx(0, 3)}));
}

TEST(dd_goalset, finalize_rejects_uncoverable_sets)
{
  // two targets, both eligible ONLY for the same single cell
  EXPECT_THROW(make_set_ins({"....", "...."}, {{1, 0}},
                            {{0, 0}, {0, 1}}, {{0, 0}, {0, 1}},
                            {{{0, 3}}, {{0, 3}}}),
               std::invalid_argument);
  // same via the YAML loader
  const auto path =
      std::string(DD_TEST_DIR) + "/fixtures/dd_goalset_uncoverable.yaml";
  EXPECT_THROW(load_dd_instance(path), std::invalid_argument);
}

TEST(dd_goalset, finalize_filters_unreachable_goals)
{
  // wall splits the grid; the right-side goal is unreachable and must be
  // filtered out, keeping the reachable one; representative follows.
  const std::vector<std::string> rows = {"..#..", "..#.."};
  auto ins = make_set_ins(rows, {{1, 0}}, {{0, 0}}, {{0, 0}},
                          {{{0, 4}, {0, 1}}});  // unreachable + reachable
  ASSERT_EQ(ins.target_goal_sets[0].size(), 1u);
  EXPECT_EQ(ins.target_goal_sets[0][0], ins.grid.idx(0, 1));
  EXPECT_EQ(ins.target_goals[0], ins.grid.idx(0, 1));
  // all goals unreachable -> loud failure (same rule as old singleton)
  EXPECT_THROW(make_set_ins(rows, {{1, 0}}, {{0, 0}}, {{0, 0}},
                            {{{0, 4}, {1, 4}}}),
               std::invalid_argument);
}

TEST(dd_goalset, is_dd_goal_accepts_any_eligible_cell)
{
  // two targets, shared 3-cell pool; both grounded on NON-representative
  // pool cells => goal (Prop 3); representative equality NOT required
  auto ins = make_set_ins(
      {".....", "....."}, {{1, 0}}, {{0, 0}, {0, 1}}, {{0, 0}, {0, 1}},
      {{{0, 2}, {0, 3}, {0, 4}}, {{0, 2}, {0, 3}, {0, 4}}});
  PhysConfig s = initial_phys_config(ins);
  s.target_pos = {ins.grid.idx(0, 3), ins.grid.idx(0, 4)};
  EXPECT_TRUE(is_dd_goal(ins, s));
  // a non-eligible cell is not a goal even when grounded
  s.target_pos = {ins.grid.idx(0, 3), ins.grid.idx(1, 4)};
  EXPECT_FALSE(is_dd_goal(ins, s));
  // carried on an eligible cell is not grounded -> not a goal (D10)
  s.target_pos = {ins.grid.idx(0, 3), ins.grid.idx(0, 4)};
  s.robots = {ins.grid.idx(0, 4)};
  s.kappa = {1};
  EXPECT_FALSE(is_dd_goal(ins, s));
}

TEST(dd_goalset, already_on_eligible_goal_is_trivially_solved)
{
  // target starts grounded ON a pool cell that is NOT the representative
  // (representative = sorted-first = (0,2)); new terminal semantics =>
  // trivially solved single all-wait plan.  Under fixed-goal semantics
  // the solver would move the shelf to the representative instead.
  auto ins = make_set_ins({".....", "....."}, {{1, 0}}, {{0, 3}},
                          {{0, 3}}, {{{0, 2}, {0, 3}}});
  DDStats st;
  const auto plan = solve_carrier_lacam(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());  // solved
  EXPECT_LE(plan.size(), 1u);  // trivially: one all-wait step
  auto s = initial_phys_config(ins);
  for (const auto& ops : plan) {
    auto nxt = apply_ops(ins, s, ops);
    ASSERT_TRUE(nxt.has_value());
    s = *nxt;
  }
  EXPECT_TRUE(is_dd_goal(ins, s));
}

TEST(dd_goalset, dynamic_first_solution_restarts_with_fixed_assignment)
{
  // The first solution may choose either eligible goal. Its terminal choice
  // becomes a singleton assignment for one automatic root restart.
  auto ins = make_set_ins({".....", "....."}, {{1, 0}}, {{0, 3}},
                          {{0, 3}}, {{{0, 2}, {0, 3}}});
  DDStats st;
  const auto plan = solve_carrier_lacam(ins, 1.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_EQ(st.assignment_restarts, 1);
  EXPECT_EQ(st.assignment_second_solved, 1);
  EXPECT_EQ(st.assignment_improvements, 0);
  EXPECT_GE(st.assignment_second_solution_ms, st.first_solution_ms);
}

TEST(dd_goalset, singleton_assignment_skips_second_search)
{
  auto ins = make_set_ins({".....", "....."}, {{1, 0}}, {{0, 3}},
                          {{0, 3}}, {{{0, 3}}});
  DDStats st;
  const auto plan = solve_carrier_lacam(ins, 1.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_EQ(st.assignment_restarts, 0);
  EXPECT_EQ(st.assignment_second_solved, 0);
  EXPECT_EQ(st.assignment_improvements, 0);
}

// review fix batch 2026-09-01 (TDD RED): two labeled targets referencing
// the SAME initial shelf cell would be one physical shelf with two
// identities — target_pos gets duplicate cells and upper-deck exclusivity
// is void. Neither the shelves-overlap check (one shelf entry) nor the
// covering matching (distinct goals) catches it; finalize must.
TEST(dd_goalset, finalize_rejects_duplicate_target_starts)
{
  EXPECT_THROW(make_set_ins({"....", "...."}, {{1, 0}},
                            {{0, 0}, {0, 1}}, {{0, 0}, {0, 0}},
                            {{{0, 2}}, {{0, 3}}}),
               std::invalid_argument);
}
