#include "../lacam/include/tapf_assignment.hpp"

#include <array>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Core =
    tapf_assignment_detail::
        IncrementalHungarianState<long double>;
using Status =
    tapf_assignment_detail::
        IncrementalHungarianStatus;

struct FloatingArithmetic {
  bool normalize_reduced(
      long double& value, long double left,
      long double right, long double score) const
  {
    const long double tolerance =
        1e-12L *
        (1 + std::fabs(left) + std::fabs(right) +
         std::fabs(score));
    if (value < -tolerance) return false;
    if (value < 0) value = 0;
    return true;
  }

  bool normalize_slack(
      long double& value, long double delta) const
  {
    const long double tolerance =
        1e-12L * (1 + std::fabs(delta));
    if (value < -tolerance) return false;
    if (value < 0) value = 0;
    return true;
  }

  bool is_zero(long double value) const
  {
    return value == 0;
  }
};

struct NeverStop {
  bool operator()() const { return false; }
};

long double assignment_cost(
    const Core& state,
    const std::vector<std::vector<long double>>& cost,
    int real_rows)
{
  long double out = 0;
  for (int row = 0; row < real_rows; ++row)
    out += cost[row][state.row_to_column[row]];
  return out;
}

}  // namespace

TEST(incremental_hungarian_core,
     rectangular_repair_and_forced_row_share_one_core)
{
  constexpr int real_rows = 2;
  constexpr int columns = 3;
  std::vector<std::vector<long double>> cost{
      {1, 10, 9},
      {10, 1, 8},
  };
  const auto score = [&](int row, int column,
                         long double& out) {
    if (row >= real_rows) {
      out = 0;
      return true;
    }
    out = -cost[row][column];
    return true;
  };

  Core state;
  state.init(columns);
  ASSERT_EQ(
      state.solve_full(
          score, FloatingArithmetic{}, NeverStop{}),
      Status::OK);
  EXPECT_EQ(state.row_to_column[0], 0);
  EXPECT_EQ(state.row_to_column[1], 1);
  EXPECT_EQ(assignment_cost(state, cost, real_rows), 2);

  cost[0] = {20, 2, 3};
  ASSERT_EQ(
      state.repair_rows(
          real_rows, {0}, score,
          FloatingArithmetic{}, NeverStop{}),
      Status::OK);
  EXPECT_EQ(state.row_to_column[0], 2);
  EXPECT_EQ(state.row_to_column[1], 1);
  EXPECT_EQ(assignment_cost(state, cost, real_rows), 4);

  auto forced = state;
  const auto forced_score =
      [&](int row, int column, long double& out) {
        if (row == 0 && column != 1)
          return false;
        return score(row, column, out);
      };
  ASSERT_EQ(
      forced.repair_rows(
          real_rows, {0}, forced_score,
          FloatingArithmetic{}, NeverStop{}),
      Status::OK);
  EXPECT_EQ(forced.row_to_column[0], 1);
  EXPECT_EQ(forced.row_to_column[1], 2);
  EXPECT_EQ(assignment_cost(forced, cost, real_rows), 10);
}
