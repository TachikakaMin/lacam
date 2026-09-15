#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(pair_congestion_assignment,
     longer_clear_goal_beats_a_shorter_shelf_congested_path)
{
  DDInstance ins;
  ins.grid = DDGrid({
      ".......",
      ".......",
      ".......",
      ".......",
      ".......",
  });
  ins.shelf_storage.assign(ins.grid.size(), 1);
  const int source = ins.grid.idx(2, 0);
  const int short_goal = ins.grid.idx(2, 6);
  const int clear_goal = ins.grid.idx(4, 6);
  ins.robots = {ins.grid.idx(4, 0)};
  ins.shelves = {
      source,
      ins.grid.idx(2, 2),
      ins.grid.idx(2, 3),
      ins.grid.idx(2, 4),
      ins.grid.idx(1, 3),
      ins.grid.idx(3, 3),
  };
  ins.target_starts = {source};
  ins.target_goals = {short_goal};
  ins.target_goal_sets = {{short_goal, clear_goal}};
  ins.finalize();

  const auto upper =
      carrier_detail::make_upper_signature(
          initial_phys_config(ins));
  DDDistCache distance(ins.grid);
  const auto assignment =
      carrier_detail::build_congestion_pair_assignment(
          ins, upper, distance,
          /*alpha=*/1, /*gamma=*/1, /*delta=*/1);

  ASSERT_EQ(assignment.tau.size(), 1u);
  EXPECT_EQ(assignment.tau[0], clear_goal);
  EXPECT_EQ(assignment.total_edges, 2);
  EXPECT_EQ(assignment.evaluated_edges, 2);
  EXPECT_EQ(assignment.hungarian_full_solves, 1);
  EXPECT_EQ(assignment.hungarian_forced_repairs, 0);
  EXPECT_EQ(assignment.prefix_refinements, 0);
  EXPECT_EQ(assignment.rollout_work_steps, 0);
}
