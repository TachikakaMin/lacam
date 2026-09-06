// PROTECTED differential oracle for exact canonical additive assignment.
#include <rho_assignment.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace {

struct BruteResult {
  std::vector<int> assignment;
  RhoCost objective = 0;
  bool feasible = false;
};

BruteResult brute_force(
    const std::vector<std::vector<RhoCost>>& cost)
{
  BruteResult best;
  const int rows = static_cast<int>(cost.size());
  const int columns =
      cost.empty() ? 0 : static_cast<int>(cost.front().size());
  if (rows == 0) {
    best.feasible = true;
    return best;
  }
  std::vector<int> choice(rows, -1);
  std::vector<uint8_t> used(columns, 0);
  const auto visit = [&](const auto& self, int row,
                         RhoCost objective) -> void {
    if (row == rows) {
      if (!best.feasible || objective < best.objective ||
          (objective == best.objective &&
           choice < best.assignment)) {
        best.assignment = choice;
        best.objective = objective;
        best.feasible = true;
      }
      return;
    }
    for (int column = 0; column < columns; ++column) {
      if (used[column] ||
          cost[row][column] >= kRhoAssignmentInf)
        continue;
      used[column] = 1;
      choice[row] = column;
      self(self, row + 1, objective + cost[row][column]);
      choice[row] = -1;
      used[column] = 0;
    }
  };
  visit(visit, 0, 0);
  return best;
}

}  // namespace

TEST(dd_rho_additive_canonical_random,
     full_solver_matches_exhaustive_lexicographic_oracle)
{
  std::mt19937 rng(20260906);
  std::uniform_int_distribution<int> value(-4, 7);
  std::bernoulli_distribution forbidden(0.18);

  for (int trial = 0; trial < 2000; ++trial) {
    const int rows = 1 + static_cast<int>(rng() % 3);
    const int columns =
        rows + static_cast<int>(rng() % (5 - rows));
    std::vector<std::vector<RhoCost>> cost(
        rows, std::vector<RhoCost>(columns, 0));
    for (auto& row : cost)
      for (auto& edge : row)
        edge = forbidden(rng) ? kRhoAssignmentInf : value(rng);

    const auto expected = brute_force(cost);
    const auto actual = solve_rho_assignment_full(cost);

    ASSERT_EQ(actual.feasible, expected.feasible)
        << "trial=" << trial;
    ASSERT_FALSE(actual.overflow) << "trial=" << trial;
    if (!expected.feasible) continue;
    EXPECT_EQ(actual.objective, expected.objective)
        << "trial=" << trial;
    EXPECT_EQ(actual.row_to_col, expected.assignment)
        << "trial=" << trial;
  }
}
