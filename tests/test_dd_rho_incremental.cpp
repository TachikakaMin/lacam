#include <rho_assignment.hpp>

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Matrix = std::vector<std::vector<RhoCost>>;

void expect_matches_full(
    const Matrix& cost, const RhoStateSolveResult& warm)
{
  const auto cold = solve_rho_assignment_full(cost);
  ASSERT_TRUE(cold.feasible);
  ASSERT_TRUE(warm.assignment.feasible);
  EXPECT_EQ(warm.assignment.objective, cold.objective);
  EXPECT_TRUE(validate_rho_assignment_state(cost, warm.state));
}

}  // namespace

TEST(dd_rho_incremental, one_changed_row_can_reassign_unchanged_rows)
{
  const Matrix parent = {
      {0, 100, 100},
      {0, 1, 2},
  };
  const Matrix child = {
      {100, 100, 0},
      {0, 1, 2},
  };
  const auto full = solve_rho_assignment_state_full(parent);
  ASSERT_TRUE(full.assignment.feasible);
  const auto repaired =
      repair_rho_assignment_state_rows(full.state, child, {0});
  ASSERT_TRUE(repaired.parent_valid);
  EXPECT_TRUE(repaired.used_parent);
  EXPECT_EQ(repaired.augmentations, 1);
  expect_matches_full(child, repaired);
  EXPECT_EQ(repaired.assignment.objective, 0);
  ASSERT_EQ(repaired.assignment.row_to_col.size(), 2u);
  EXPECT_EQ(repaired.assignment.row_to_col[0], 2);
  EXPECT_EQ(repaired.assignment.row_to_col[1], 0);
}

TEST(dd_rho_incremental, zero_changed_rows_reuses_valid_state)
{
  const Matrix cost = {
      {-4, 2, 0},
      {3, -1, 0},
  };
  const auto full = solve_rho_assignment_state_full(cost);
  ASSERT_TRUE(full.assignment.feasible);
  const auto reused =
      repair_rho_assignment_state_rows(full.state, cost, {});
  ASSERT_TRUE(reused.parent_valid);
  EXPECT_TRUE(reused.used_parent);
  EXPECT_EQ(reused.augmentations, 0);
  EXPECT_EQ(reused.assignment.objective, full.assignment.objective);
  EXPECT_EQ(reused.state.mate_l, full.state.mate_l);
  EXPECT_EQ(reused.state.mate_r, full.state.mate_r);
  EXPECT_EQ(reused.state.row_potential, full.state.row_potential);
  EXPECT_EQ(reused.state.column_potential, full.state.column_potential);
}

TEST(dd_rho_incremental, multiple_changed_rows_match_full_objective)
{
  const Matrix parent = {
      {0, 7, 9, 0},
      {8, 0, 3, 0},
      {4, 6, 0, 0},
  };
  const Matrix child = {
      {9, -3, 5, 0},
      {8, 0, 3, 0},
      {-2, 6, 8, 0},
  };
  const auto full = solve_rho_assignment_state_full(parent);
  ASSERT_TRUE(full.assignment.feasible);
  const auto repaired =
      repair_rho_assignment_state_rows(full.state, child, {2, 0, 2});
  ASSERT_TRUE(repaired.parent_valid);
  EXPECT_EQ(repaired.augmentations, 2);
  expect_matches_full(child, repaired);
}

TEST(dd_rho_incremental, repair_is_a_value_copy_and_does_not_mutate_parent)
{
  const Matrix parent_cost = {
      {0, 5, 0},
      {4, 0, 0},
  };
  const Matrix child_cost = {
      {7, -2, 0},
      {4, 0, 0},
  };
  const auto parent = solve_rho_assignment_state_full(parent_cost);
  ASSERT_TRUE(parent.assignment.feasible);
  const auto saved = parent.state;
  const auto child =
      repair_rho_assignment_state_rows(parent.state, child_cost, {0});
  ASSERT_TRUE(child.assignment.feasible);
  EXPECT_EQ(parent.state.mate_l, saved.mate_l);
  EXPECT_EQ(parent.state.mate_r, saved.mate_r);
  EXPECT_EQ(parent.state.row_potential, saved.row_potential);
  EXPECT_EQ(parent.state.column_potential, saved.column_potential);
  EXPECT_EQ(parent.state.objective, saved.objective);
  expect_matches_full(child_cost, child);
}

TEST(dd_rho_incremental, undeclared_row_change_invalidates_parent_state)
{
  const Matrix parent = {
      {0, 3, 0},
      {2, 0, 0},
  };
  const Matrix child = {
      {0, 3, 0},
      {-10, 0, 0},
  };
  const auto full = solve_rho_assignment_state_full(parent);
  ASSERT_TRUE(full.assignment.feasible);
  const auto invalid =
      repair_rho_assignment_state_rows(full.state, child, {});
  EXPECT_FALSE(invalid.parent_valid);
  EXPECT_FALSE(invalid.assignment.feasible);
}

TEST(dd_rho_incremental, invalid_negative_sentinel_reports_overflow)
{
  const Matrix cost = {{-kRhoAssignmentInf, 0}};
  const auto full = solve_rho_assignment_state_full(cost);
  EXPECT_TRUE(full.assignment.overflow);
  EXPECT_FALSE(full.assignment.feasible);
}

TEST(dd_rho_incremental, random_row_update_sequences_match_full_solver)
{
  std::mt19937 rng(20260906);
  std::uniform_int_distribution<int> rows_dist(1, 4);
  std::uniform_int_distribution<int> extra_cols_dist(0, 3);
  std::uniform_int_distribution<int> cost_dist(-20, 40);
  std::uniform_int_distribution<int> forbidden_dist(0, 9);

  for (int trial = 0; trial < 2000; ++trial) {
    const int rows = rows_dist(rng);
    const int cols = rows + extra_cols_dist(rng);
    Matrix cost(rows, std::vector<RhoCost>(cols, 0));
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col)
        cost[row][col] =
            forbidden_dist(rng) == 0
                ? kRhoAssignmentInf
                : cost_dist(rng);
      cost[row][row] = cost_dist(rng);
    }

    auto warm = solve_rho_assignment_state_full(cost);
    ASSERT_TRUE(warm.assignment.feasible) << "trial " << trial;
    expect_matches_full(cost, warm);

    for (int generation = 0; generation < 4; ++generation) {
      std::vector<int> changed;
      for (int row = 0; row < rows; ++row) {
        if ((rng() & 3U) == 0) continue;
        changed.push_back(row);
        for (int col = 0; col < cols; ++col)
          cost[row][col] =
              forbidden_dist(rng) == 0
                  ? kRhoAssignmentInf
                  : cost_dist(rng);
        cost[row][row] = cost_dist(rng);
      }
      if (changed.empty() && generation != 0) {
        changed.push_back(generation % rows);
        const int row = changed.front();
        for (int col = 0; col < cols; ++col)
          cost[row][col] = cost_dist(rng);
      }
      auto repaired =
          repair_rho_assignment_state_rows(
              warm.state, cost, changed);
      ASSERT_TRUE(repaired.parent_valid)
          << "trial " << trial << " generation " << generation;
      ASSERT_TRUE(repaired.assignment.feasible)
          << "trial " << trial << " generation " << generation;
      expect_matches_full(cost, repaired);
      warm = std::move(repaired);
    }
  }
}
