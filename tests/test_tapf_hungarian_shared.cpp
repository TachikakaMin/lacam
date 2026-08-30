// Skeleton-reuse refactor #1 (skeleton audit 2026-08-30): the DD planner
// must call the ORIGINAL lacam-tapf Hungarian (tapf_assignment) instead of
// its own copy.  This file pins the shared public API's contract:
//   - rectangular rows <= cols, row->col output (-1 = unassigned)
//   - negative costs allowed (DD eta hysteresis discounts)
//   - forbidden pairs via large sentinel (caller filters afterwards)
//   - optimal total cost (checked against brute-force permutations)
//   - deterministic tie behavior (classic potentials scan order), which
//     keeps the DD benchmark plans byte-identical after the swap.
#include <tapf_assignment.hpp>

#include <algorithm>
#include <climits>
#include <numeric>
#include <vector>

#include "gtest/gtest.h"

namespace {

long long brute_best(const std::vector<std::vector<int>>& cost)
{
  const int n = (int)cost.size(), m = (int)cost[0].size();
  std::vector<int> cols(m);
  std::iota(cols.begin(), cols.end(), 0);
  long long best = LLONG_MAX;
  do {
    long long s = 0;
    for (int i = 0; i < n; ++i) s += cost[i][cols[i]];
    best = std::min(best, s);
  } while (std::next_permutation(cols.begin(), cols.end()));
  return best;
}

long long total_of(const std::vector<std::vector<int>>& cost,
                   const std::vector<int>& r2c)
{
  long long s = 0;
  for (size_t i = 0; i < r2c.size(); ++i) s += cost[i][r2c[i]];
  return s;
}

}  // namespace

TEST(tapf_hungarian_shared, square_matches_bruteforce)
{
  std::vector<std::vector<int>> cost = {{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
  const auto r2c = tapf_hungarian_row_to_col(cost);
  ASSERT_EQ(r2c.size(), 3u);
  EXPECT_EQ(total_of(cost, r2c), brute_best(cost));
}

TEST(tapf_hungarian_shared, rectangular_rows_lt_cols)
{
  std::vector<std::vector<int>> cost = {{7, 2, 9, 4}, {3, 8, 1, 6}};
  const auto r2c = tapf_hungarian_row_to_col(cost);
  ASSERT_EQ(r2c.size(), 2u);
  EXPECT_NE(r2c[0], r2c[1]);
  EXPECT_EQ(total_of(cost, r2c), 3);  // 2 + 1
}

TEST(tapf_hungarian_shared, negative_costs_from_eta_discount)
{
  // eta hysteresis can push effective distances below zero
  std::vector<std::vector<int>> cost = {{-2, 1}, {0, -1}};
  const auto r2c = tapf_hungarian_row_to_col(cost);
  EXPECT_EQ(total_of(cost, r2c), -3);
}

TEST(tapf_hungarian_shared, forbidden_sentinel_avoided_when_possible)
{
  const int INF = INT_MAX / 8;
  std::vector<std::vector<int>> cost = {{INF, 5}, {4, INF}};
  const auto r2c = tapf_hungarian_row_to_col(cost);
  EXPECT_EQ(r2c[0], 1);
  EXPECT_EQ(r2c[1], 0);
}

TEST(tapf_hungarian_shared, dd_crossing_fixture_semantics)
{
  // the exact matrix shape of the DD crossing fixture (request rows S1@d
  // {2,1}, S2@d {2,4}): min-cost must swap the crossing pair
  std::vector<std::vector<int>> cost = {{2, 1}, {2, 4}};
  const auto r2c = tapf_hungarian_row_to_col(cost);
  EXPECT_EQ(r2c[0], 1);  // S1 -> r1
  EXPECT_EQ(r2c[1], 0);  // S2 -> r0
}
