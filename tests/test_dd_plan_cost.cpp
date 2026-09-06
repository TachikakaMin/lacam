#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <limits>

#include "gtest/gtest.h"

TEST(dd_plan_cost, compares_ticks_before_work)
{
  const auto parallel_reference = PlanCost::from_values(31, 93);
  const auto old_plan = PlanCost::from_values(54, 90);
  EXPECT_LT(parallel_reference, old_plan);
  EXPECT_FALSE(old_plan < parallel_reference);

  EXPECT_LT(
      PlanCost::from_values(31, 90),
      PlanCost::from_values(31, 93));
  EXPECT_EQ(
      PlanCost::from_values(31, 93),
      PlanCost::from_values(31, 93));
}

TEST(dd_plan_cost, strict_order_is_transitive_without_epsilon)
{
  const auto a = PlanCost::from_values(7, 0.0);
  const auto b = PlanCost::from_values(7, 0.000001);
  const auto c = PlanCost::from_values(7, 0.000002);
  ASSERT_LT(a, b);
  ASSERT_LT(b, c);
  EXPECT_LT(a, c);
}

TEST(dd_plan_cost, addition_preserves_both_dimensions)
{
  const auto prefix = PlanCost::from_values(3, 1.25);
  const auto suffix = PlanCost::from_values(4, 2.5);
  const auto total = prefix + suffix;
  EXPECT_EQ(total.ticks, 7);
  EXPECT_DOUBLE_EQ(total.work_value(), 3.75);
}

TEST(dd_plan_cost, objective_contract_is_explicit)
{
  TAPFSearchConfig legacy;
  EXPECT_EQ(legacy.objective, TAPFObjective::LEGACY_WEIGHTED_WORK);
  EXPECT_EQ(legacy.stop_policy, TAPFStopPolicy::ANYTIME);

  TAPFSearchConfig carrier;
  carrier.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  EXPECT_EQ(carrier.objective, TAPFObjective::MAKESPAN_THEN_WORK);
}

TEST(dd_plan_cost,
     first_strict_improvement_stops_at_first_goal_below_bound)
{
  // This fixture's deterministic first solution has legacy work 40;
  // continuing anytime reaches work 38.  A bounded first-improvement pass
  // must return the former immediately instead of continuing to the latter.
  const TAPFInstance ins(
      "./assets/empty-8-8.map", std::vector<int>{0, 1, 8},
      std::vector<std::vector<int>>{
          {63, 62, 55}, {63, 62, 55}, {63, 62, 55}});
  ASSERT_TRUE(ins.is_valid());

  TAPFSearchConfig config;
  config.stop_policy = TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT;
  config.incumbent_init = PlanCost::legacy(100);
  TAPFStats stats;
  std::mt19937 mt(0);
  const auto solution = solve_tapf(
      ins, 0, nullptr, &mt, 0, &stats,
      /*anytime=*/true, /*force_full_assignment=*/false, config);

  ASSERT_FALSE(solution.empty());
  EXPECT_EQ(stats.solution_cost, 40u);
  EXPECT_EQ(stats.incumbent_updates, 1);
  EXPECT_LT(PlanCost::legacy(stats.solution_cost),
            config.incumbent_init);
}

TEST(dd_plan_cost, explicit_stop_policies_reject_inconsistent_bounds)
{
  const TAPFInstance ins(
      "./assets/empty-8-8.map", std::vector<int>{0},
      std::vector<std::vector<int>>{{1}});

  TAPFSearchConfig first_feasible;
  first_feasible.stop_policy = TAPFStopPolicy::FIRST_FEASIBLE;
  first_feasible.incumbent_init = PlanCost::legacy(2);
  EXPECT_THROW(
      TAPFPlanner(
          &ins, nullptr, nullptr, 0, 0, 0.001f, false, nullptr,
          first_feasible),
      std::invalid_argument);

  TAPFSearchConfig first_improvement;
  first_improvement.stop_policy =
      TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT;
  EXPECT_THROW(
      TAPFPlanner(
          &ins, nullptr, nullptr, 0, 0, 0.001f, false, nullptr,
          first_improvement),
      std::invalid_argument);
}

TEST(dd_plan_cost, unbounded_value_is_not_a_finite_incumbent)
{
  const auto bound = PlanCost::unbounded();
  EXPECT_FALSE(bound.is_bounded());
  EXPECT_TRUE(PlanCost::from_values(0, 0).is_bounded());
}

TEST(dd_plan_cost, carrier_entry_reports_executed_ticks_and_work)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.finalize();

  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 5.0, 0, &stats);
  ASSERT_FALSE(plan.empty());
  const auto cost = dd_plan_cost_probe(ins, plan);
  EXPECT_EQ(cost.ticks, static_cast<int64_t>(plan.size()));
  EXPECT_EQ(stats.best_makespan, cost.ticks);
  EXPECT_DOUBLE_EQ(stats.best_soc, cost.work_value());
  EXPECT_GE(stats.first_solution_makespan, stats.best_makespan);
  EXPECT_GE(stats.first_solution_ms, 0);
}

TEST(dd_plan_cost, two_pass_selection_prefers_shorter_plan)
{
  const auto first = PlanCost::from_values(54, 90);
  const auto second = PlanCost::from_values(31, 93);
  EXPECT_TRUE(dd_plan_cost_better_probe(second, first));
  EXPECT_FALSE(dd_plan_cost_better_probe(first, second));
}

TEST(dd_plan_cost, normalizes_to_the_first_goal_prefix)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.finalize();
  const DDPlan plan = {
      {Op::make_lift()},
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_drop()},
      {Op::make_lift()},
      {Op::make_wait()},
  };
  const auto normalized = dd_normalize_goal_prefix_probe(ins, plan);
  ASSERT_TRUE(normalized.has_value());
  EXPECT_EQ(normalized->size(), 3u);
  EXPECT_EQ(dd_plan_cost_probe(ins, plan).ticks, 3);
}

TEST(dd_plan_cost, initial_goal_normalizes_to_zero_ticks)
{
  DDInstance ins;
  ins.grid = DDGrid({".."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 0)};
  ins.finalize();
  const DDPlan one_wait = {{Op::make_wait()}};
  const auto normalized =
      dd_normalize_goal_prefix_probe(ins, one_wait);
  ASSERT_TRUE(normalized.has_value());
  EXPECT_TRUE(normalized->empty());
  EXPECT_EQ(dd_plan_cost_probe(ins, one_wait).ticks, 0);
}
