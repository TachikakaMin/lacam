#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

TEST(dd_task_br_blocked_restoration,
     dense_tail_defers_adjacent_goal_even_when_blocker_costs_two_steps)
{
  DDInstance ins;
  constexpr int width = 142;
  ins.grid = DDGrid({std::string(width, '.')});
  ins.robots = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, width - 1),
  };
  for (int col = 0; col < width; ++col)
    if (col != 2 && col != 70)
      ins.shelves.push_back(ins.grid.idx(0, col));
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, width - 1),
  };
  ins.target_goals = {
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 3),
  };
  ins.finalize();

  const auto guidance =
      dd_task_br_guidance_probe(ins, initial_phys_config(ins));
  ASSERT_NE(guidance.upper_epoch, nullptr);
  const auto& table = guidance.upper_epoch->pair_cost;
  ASSERT_EQ(table.size(), 2u);
  ASSERT_EQ(table[0].size(), 1u);
  ASSERT_EQ(table[1].size(), 1u);
  EXPECT_EQ(table[0][0].plan.direct_distance, 1);
  EXPECT_TRUE(table[0][0].plan.reached_goal);
  EXPECT_EQ(table[0][0].plan.rollout_steps, 2);
  EXPECT_TRUE(table[1][0].plan.truncated);
  EXPECT_FALSE(table[1][0].plan.reached_goal);
  EXPECT_GT(guidance.upper_epoch->target_priority[1],
            guidance.upper_epoch->target_priority[0]);
}
