#include "../lacam/src/carrier_assignment.hpp"

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

TEST(pair_incremental_invalid_dual,
     repair_fails_closed_instead_of_running_a_full_solve)
{
  PairAssignmentHungarianState saved;
  saved.row_count = 2;
  saved.column_count = 2;
  saved.row_to_column = {0, 1};
  saved.column_to_row = {0, 1};
  saved.row_dual = {0, -100};
  saved.column_dual = {0, 0};
  saved.valid = true;

  carrier_detail::IncrementalLongDoubleHungarian state;
  ASSERT_TRUE(state.restore(saved));

  const std::vector<std::vector<long double>> cost{
      {10, 0},
      {0, 10},
  };
  const auto cost_at = [&](int row, int column) {
    return cost[row][column];
  };

  EXPECT_THROW(
      (void)state.repair_rows({0}, cost_at),
      std::logic_error);
}
