#include <dd_planner.hpp>

#include <algorithm>
#include <numeric>

#include "gtest/gtest.h"

namespace {

DDInstance line_instance(int width)
{
  DDInstance ins;
  ins.grid = DDGrid({std::string(width, '.')});
  return ins;
}

}  // namespace

TEST(dd_makespan_heuristic, grounded_target_already_at_goal_is_zero)
{
  auto ins = line_instance(2);
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 0)};
  ins.finalize();
  EXPECT_EQ(
      dd_makespan_lb_probe(ins, initial_phys_config(ins)),
      0);
}

TEST(dd_makespan_heuristic, carried_target_at_goal_still_needs_drop)
{
  auto ins = line_instance(2);
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 0)};
  ins.finalize();
  auto state = initial_phys_config(ins);
  state.kappa[0] = 0;
  EXPECT_EQ(dd_makespan_lb_probe(ins, state), 1);
}

TEST(dd_makespan_heuristic, approach_lift_move_drop_chain_is_counted)
{
  auto ins = line_instance(3);
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.finalize();
  EXPECT_EQ(
      dd_makespan_lb_probe(ins, initial_phys_config(ins)),
      3);
}

TEST(dd_makespan_heuristic, injective_matching_combines_bottleneck_and_work)
{
  auto ins = line_instance(4);
  ins.robots = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 2), ins.grid.idx(0, 2)};
  ins.target_goal_sets = {
      {ins.grid.idx(0, 2), ins.grid.idx(0, 3)},
      {ins.grid.idx(0, 2), ins.grid.idx(0, 3)},
  };
  ins.finalize();
  EXPECT_EQ(
      dd_makespan_lb_probe(ins, initial_phys_config(ins)),
      4);
}

TEST(dd_makespan_heuristic, mixed_tapf_time_uses_max_not_sum)
{
  TAPFInstance ins(
      "./assets/empty-8-8.map",
      {0, 1, 2},
      {{8}, {9}, {10}});
  ASSERT_TRUE(ins.is_valid());
  TAPFPlanner planner(&ins, nullptr, nullptr);
  EXPECT_EQ(planner.get_time_h_value(ins.starts), 1);
  EXPECT_EQ(planner.get_h_value(ins.starts), 3);
}

TEST(dd_makespan_heuristic, carrier_attach_installs_tick_and_work_bounds)
{
  auto ins = line_instance(3);
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.finalize();
  const TAPFInstance view(ins);
  TAPFSearchConfig config;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  TAPFPlanner planner(
      &view, nullptr, nullptr, 0, 0, 0.001f, false, nullptr, config);
  auto node = std::make_unique<TAPFNode>(
      view.starts, initial_shelf_state(view), planner.D, &view,
      std::vector<int>{-1}, TAPFAssignmentState(), nullptr);
  planner.attach_carrier_guidance(node.get());
  EXPECT_EQ(node->h.ticks, 3);
  EXPECT_DOUBLE_EQ(node->h.work_value(), 3);
  EXPECT_EQ(node->f, node->g + node->h);
}

TEST(dd_makespan_heuristic, ordinary_joint_wait_costs_one_tick)
{
  TAPFInstance ins(
      "./assets/empty-8-8.map",
      {0},
      {{0}});
  TAPFSearchConfig config;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  TAPFPlanner planner(
      &ins, nullptr, nullptr, 0, 0, 0.001f, false, nullptr, config);
  const auto shelf = initial_shelf_state(ins);
  auto from = std::make_unique<TAPFNode>(
      ins.starts, shelf, planner.D, &ins, std::vector<int>{0},
      TAPFAssignmentState(), nullptr);
  auto to = std::make_unique<TAPFNode>(
      ins.starts, shelf, planner.D, &ins, std::vector<int>{0},
      TAPFAssignmentState(), from.get());
  EXPECT_EQ(
      planner.get_edge_cost(from.get(), to.get()),
      PlanCost::from_values(1, 0));
}

TEST(dd_makespan_heuristic, macro_cost_ticks_equal_primitive_trace_length)
{
  auto ins = line_instance(3);
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.finalize();
  const TAPFInstance view(ins);
  TAPFSearchConfig config;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  TAPFPlanner planner(
      &view, nullptr, nullptr, 0, 0, 0.001f, false, nullptr, config);
  const auto rollout = planner.carrier_rollout(
      view.starts, initial_shelf_state(view), 8, 0, false);
  ASSERT_FALSE(rollout.ops.empty());
  EXPECT_EQ(
      rollout.cost.ticks,
      static_cast<int64_t>(rollout.ops.size()));
}
