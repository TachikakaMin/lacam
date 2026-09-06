#include "../lacam/src/carrier_guidance.hpp"

#include <limits>
#include <stdexcept>
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

TEST(dd_dispatch,
     priority_is_a_low_order_term_after_physical_secondary_cost)
{
  EXPECT_EQ(
      carrier_detail::rho_priority_lex_cost(3, 11, 0), 33);
  EXPECT_EQ(
      carrier_detail::rho_priority_lex_cost(3, 11, 9), 42);
  EXPECT_LT(
      carrier_detail::rho_priority_lex_cost(3, 11, 9),
      carrier_detail::rho_priority_lex_cost(4, 11, 0));
}

TEST(dd_dispatch,
     priority_lex_cost_rejects_dispatch_sentinel_overflow)
{
  EXPECT_THROW(
      carrier_detail::rho_priority_lex_cost(
          std::numeric_limits<long long>::max(), 2, 1),
      std::overflow_error);
}
