#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(pair_congestion_semantics,
     completed_estimates_are_not_labeled_as_exact_pair_cost)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.target_goal_sets = {{ins.grid.idx(0, 2)}};
  ins.finalize();

  const auto upper =
      carrier_detail::make_upper_signature(
          initial_phys_config(ins));
  DDDistCache distance(ins.grid);
  const auto assignment =
      carrier_detail::build_congestion_pair_assignment(
          ins, upper, distance, 1, 1, 1);

  ASSERT_EQ(assignment.table.size(), 1u);
  ASSERT_EQ(assignment.table[0].size(), 1u);
  EXPECT_TRUE(assignment.table[0][0].plan.exact);
  EXPECT_EQ(
      assignment.table[0][0].plan.bound_stage,
      PairBoundStage::HEURISTIC_ESTIMATE);
}
