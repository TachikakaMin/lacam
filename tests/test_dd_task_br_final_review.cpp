// PROTECTED final-review regressions.
// Added before implementation on 2026-09-03 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "../lacam/src/carrier_guidance.hpp"
#include "gtest/gtest.h"

TEST(dd_task_br_final_review, pair_kernel_uses_the_joint_recursion_core)
{
  // Cell ids:
  //   0 1 2
  //   3 4 5
  //
  // Upper occupancy:
  //   goal  T1 T0
  //   .     .  R
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {5};
  ins.shelves = {2, 1};
  ins.target_starts = {2, 1};
  ins.target_goal_sets = {{0}, {1}};
  ins.target_goals = {0, 1};
  ins.finalize();

  const PhysConfig X = initial_phys_config(ins);
  ASSERT_EQ(X.robots, (std::vector<int>{5}));
  ASSERT_EQ(X.target_pos, (std::vector<int>{2, 1}));
  ASSERT_TRUE(X.anon_occ.empty());
  ASSERT_EQ(X.kappa, (std::vector<int>{KAPPA_FREE}));

  const auto pair_effect =
      dd_pair_next_ready_effect_probe(ins, X, 0, 0, 256);

  const auto graph =
      dd_compile_single_root_graph_probe(ins, X, 0, 0, 256, 512);
  const auto ready = dd_ready_tasks_probe(ins, X, graph);

  ASSERT_TRUE(pair_effect.has_value());
  ASSERT_EQ(ready.size(), 1u);

  const TaskId expected{
      ShelfSelector{ShelfSelector::Kind::TARGET, 1},
      1,
      0,
  };

  EXPECT_EQ(*pair_effect, expected);
  EXPECT_EQ(graph.tasks[ready.front()].id, expected);
}

TEST(dd_task_br_final_review,
     reaching_goal_on_the_last_rollout_step_is_not_truncated)
{
  DDInstance ins;
  ins.grid = DDGrid({std::string(129, '.')});
  ins.robots = {0};
  ins.shelves = {0};
  ins.target_starts = {0};
  ins.target_goal_sets = {{128}};
  ins.target_goals = {128};
  ins.finalize();

  const PhysConfig X = initial_phys_config(ins);
  const UpperSignature upper =
      carrier_detail::make_upper_signature(X);
  DDDistCache upper_wall(ins.grid);

  const PairPlan plan = carrier_detail::pair_cost(
      ins, upper, 0, 128, upper_wall,
      /*alpha=*/1.0,
      /*gamma=*/1.0,
      /*delta=*/1.0);

  EXPECT_EQ(plan.direct_distance, 128);
  EXPECT_EQ(plan.rollout_steps, 128);
  EXPECT_TRUE(plan.reached_goal);
  EXPECT_FALSE(plan.truncated);
  EXPECT_FALSE(plan.stalled);
  EXPECT_TRUE(plan.exact);
  EXPECT_DOUBLE_EQ(plan.estimated_cost, 130.0);
}

TEST(dd_task_br_final_review, records_a_valid_zero_empty_rotation)
{
  // Cell ids and desired rotation:
  //
  //   0 1       0->1
  //   2 3       1->3
  //             3->2
  //             2->0
  //
  // Every cell initially contains a shelf and has a robot underneath it.
  DDInstance ins;
  ins.grid = DDGrid({"..", ".."});
  ins.robots = {0, 1, 2, 3};
  ins.shelves = {0, 1, 2, 3};
  ins.target_starts = {0};
  ins.target_goal_sets = {{1}};
  ins.target_goals = {1};
  ins.finalize();

  const PhysConfig X = initial_phys_config(ins);
  ASSERT_EQ(X.target_pos, (std::vector<int>{0}));
  ASSERT_EQ(X.anon_occ, (std::vector<int>{1, 2, 3}));

  const std::vector<int> tau{1};
  const std::vector<int> priority{1};
  const auto graph = dd_compile_joint_graph_probe(
      ins, X, &tau, &priority, 256, 512);

  const RotationCandidate expected{{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          0, 1},
      TaskId{
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 1},
          1, 3},
      TaskId{
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 3},
          3, 2},
      TaskId{
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 2},
          2, 0},
  }};

  EXPECT_TRUE(dd_ready_tasks_probe(ins, X, graph).empty());
  EXPECT_NE(std::find_if(
                graph.rotations.begin(), graph.rotations.end(),
                [&](const RotationCandidate& candidate) {
                  return candidate.cycle == expected.cycle;
                }),
            graph.rotations.end());

  for (const auto& candidate : graph.rotations) {
    ASSERT_GE(candidate.cycle.size(), 3u);
    for (size_t index = 0; index < candidate.cycle.size(); ++index)
      EXPECT_EQ(
          candidate.cycle[index].to,
          candidate.cycle[(index + 1) %
                          candidate.cycle.size()].from);
  }
}
