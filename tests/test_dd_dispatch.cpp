#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

TEST(dd_dispatch,
     bottleneck_threshold_beats_a_lower_sum_but_later_assignment)
{
  const std::vector<std::vector<long long>> completion = {
      {10, 14},
      {1, 10},
  };
  const std::vector<std::vector<long long>> secondary = {
      {0, 4},
      {1, 10},
  };

  const auto assignment =
      carrier_detail::bottleneck_then_sum_assignment(
          completion, secondary);

  ASSERT_TRUE(assignment.feasible);
  EXPECT_EQ(assignment.bottleneck, 10);
  EXPECT_EQ(assignment.row_to_col, (std::vector<int>{0, 1}));
  EXPECT_EQ(assignment.secondary_cost, 10);
}

TEST(dd_dispatch,
     causal_tail_uses_the_longest_successor_chain_not_the_sum)
{
  ShelfTaskGraph graph;
  graph.tasks = {
      ShelfTask{
          TaskId{
              ShelfSelector{ShelfSelector::Kind::TARGET, 0}, 0, 1},
          {}, 1, StorageTransfer{1, {0, 1}}},
      ShelfTask{
          TaskId{
              ShelfSelector{ShelfSelector::Kind::TARGET, 1}, 2, 3},
          {}, 1, StorageTransfer{3, {2, 3}}},
      ShelfTask{
          TaskId{
              ShelfSelector{ShelfSelector::Kind::TARGET, 2}, 4, 5},
          {}, 1, StorageTransfer{5, {4, 5}}},
  };
  graph.predecessors = {{}, {0}, {0}};
  graph.successors = {{1, 2}, {}, {}};

  const auto tail =
      carrier_detail::task_critical_tail_ticks(graph);

  ASSERT_EQ(tail.size(), 3u);
  EXPECT_EQ(tail[0], 3);
  EXPECT_EQ(tail[1], 0);
  EXPECT_EQ(tail[2], 0);
}
