// PROTECTED additive rho assignment contract.
// Added before implementation on 2026-09-06 and intentionally observed RED.
#include <rho_assignment.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Matrix = std::vector<std::vector<RhoCost>>;

}  // namespace

TEST(dd_rho_additive, solves_negative_rectangular_matrix)
{
  const Matrix cost{
      {-5, 4, 0},
      {3, -7, 0},
  };

  const auto result = solve_rho_assignment_full(cost);

  ASSERT_TRUE(result.feasible);
  EXPECT_FALSE(result.overflow);
  EXPECT_EQ(result.objective, -12);
  EXPECT_EQ(result.row_to_col, (std::vector<int>{0, 1}));
}

TEST(dd_rho_additive, rejects_shape_without_complete_finite_matching)
{
  const Matrix too_narrow{
      {0},
      {0},
  };
  const Matrix forbidden{
      {0, kRhoAssignmentInf},
      {0, kRhoAssignmentInf},
  };

  EXPECT_FALSE(solve_rho_assignment_full(too_narrow).feasible);
  EXPECT_FALSE(solve_rho_assignment_full(forbidden).feasible);
}

TEST(dd_rho_additive, canonicalizes_multiple_optima_by_row_then_column)
{
  const Matrix cost{
      {0, 0, 5},
      {0, 0, 5},
  };

  const auto first = solve_rho_assignment_full(cost);
  const auto second = solve_rho_assignment_full(cost);

  ASSERT_TRUE(first.feasible);
  EXPECT_EQ(first.objective, 0);
  EXPECT_EQ(first.row_to_col, (std::vector<int>{0, 1}));
  EXPECT_EQ(second.row_to_col, first.row_to_col);
}

TEST(dd_rho_additive, row_shift_changes_objective_not_assignment)
{
  const Matrix original{
      {1, 8, 0, kRhoAssignmentInf},
      {7, 2, kRhoAssignmentInf, 0},
  };
  Matrix shifted = original;
  for (auto& value : shifted[0])
    if (value < kRhoAssignmentInf) value += 100;

  const auto before = solve_rho_assignment_full(original);
  const auto after = solve_rho_assignment_full(shifted);

  ASSERT_TRUE(before.feasible);
  ASSERT_TRUE(after.feasible);
  EXPECT_EQ(before.row_to_col, after.row_to_col);
  EXPECT_EQ(after.objective, before.objective + 100);
}

TEST(dd_rho_additive, idle_can_beat_a_weak_task_without_cardinality_first)
{
  // Columns are task A, task B, robot-0 idle, robot-1 idle.
  const Matrix cost{
      {-5, 100, 0, kRhoAssignmentInf},
      {-4, 100, kRhoAssignmentInf, 0},
  };

  const auto result = solve_rho_assignment_full(cost);

  ASSERT_TRUE(result.feasible);
  EXPECT_EQ(result.objective, -5);
  EXPECT_EQ(result.row_to_col, (std::vector<int>{0, 3}));
}

TEST(dd_rho_additive, min_sum_is_not_bottleneck_matching)
{
  const Matrix completion{
      {1, 6},
      {6, 9},
  };

  const auto result = solve_rho_assignment_full(completion);

  ASSERT_TRUE(result.feasible);
  EXPECT_EQ(result.objective, 10);
  EXPECT_EQ(result.row_to_col, (std::vector<int>{0, 1}));
  EXPECT_EQ(
      std::max(
          completion[0][result.row_to_col[0]],
          completion[1][result.row_to_col[1]]),
      9);
  EXPECT_EQ(std::max(completion[0][1], completion[1][0]), 6);
}

TEST(dd_rho_additive, urgency_is_a_finite_service_reward)
{
  const auto low =
      rho_execute_cost(/*approach=*/7, /*immediate_service=*/3,
                       /*continuity=*/1, /*mode=*/0,
                       /*urgency=*/2);
  const auto high =
      rho_execute_cost(/*approach=*/7, /*immediate_service=*/3,
                       /*continuity=*/1, /*mode=*/0,
                       /*urgency=*/6);

  ASSERT_TRUE(low.valid);
  ASSERT_TRUE(high.valid);
  EXPECT_EQ(low.total, 9);
  EXPECT_EQ(high.total, 5);
  EXPECT_EQ(low.total - high.total, 4);
  EXPECT_LT(high.total, low.total);
  EXPECT_GT(high.total, -kRhoAssignmentInf);
}

TEST(dd_rho_additive, immediate_service_excludes_successor_tail)
{
  const auto short_tail =
      rho_execute_cost(/*approach=*/4, /*immediate_service=*/3,
                       /*continuity=*/0, /*mode=*/0,
                       /*urgency=*/5);
  const auto long_tail =
      rho_execute_cost(/*approach=*/4, /*immediate_service=*/3,
                       /*continuity=*/0, /*mode=*/0,
                       /*urgency=*/5);

  ASSERT_TRUE(short_tail.valid);
  ASSERT_TRUE(long_tail.valid);
  EXPECT_EQ(short_tail.total, 2);
  EXPECT_EQ(long_tail.total, short_tail.total)
      << "successor critical tail is not an immediate service term";
}

TEST(dd_rho_additive, prepare_receives_only_bounded_partial_reward)
{
  const auto result =
      rho_prepare_cost(/*approach=*/8, /*continuity=*/1,
                       /*mode=*/0, /*urgency=*/5,
                       /*requested_prepare_reward=*/9);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.urgency_reward, 5);
  EXPECT_EQ(result.preparation_reward, 5);
  EXPECT_LE(result.preparation_reward, result.urgency_reward);
  EXPECT_EQ(result.total, 4);
}

TEST(dd_rho_additive, checked_arithmetic_reports_overflow)
{
  const RhoCost large = kRhoAssignmentInf - 1;
  Matrix cost(5, std::vector<RhoCost>(5, kRhoAssignmentInf));
  for (int row = 0; row < 5; ++row) cost[row][row] = large;

  const auto result = solve_rho_assignment_full(cost);

  EXPECT_FALSE(result.feasible);
  EXPECT_TRUE(result.overflow);
}

TEST(dd_rho_additive, empty_problem_is_feasible_and_canonical)
{
  const auto result = solve_rho_assignment_full(Matrix{});

  EXPECT_TRUE(result.feasible);
  EXPECT_FALSE(result.overflow);
  EXPECT_EQ(result.objective, 0);
  EXPECT_TRUE(result.row_to_col.empty());
}
