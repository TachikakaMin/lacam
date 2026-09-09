// PROTECTED large fixed-goal upper-search regression.
// Captures the 20x20 case where PairCost finishes but BR upper guidance
// keeps revisiting clearance arrangements until its deadline.
#include <br_lacam_upper.hpp>
#include <dd_carrier.hpp>

#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_large_upper_regression,
     fixed_pair_cost_assignment_finishes_within_upper_budget)
{
  const auto ins = load_dd_instance(
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g20x20/"
      "brap_h20w20_a40_e100_B_seed0_pool.yaml");
  const std::vector<std::pair<int, int>> goal_coordinates{
      {0, 14},  {0, 15},  {10, 0},  {19, 4},  {19, 7},
      {2, 0},   {3, 0},   {4, 19},  {10, 19}, {15, 0},
      {19, 15}, {0, 6},   {15, 19}, {0, 16},  {0, 10},
      {3, 19},  {9, 0},   {11, 0},  {19, 3},  {19, 12},
      {19, 11}, {19, 13}, {0, 5},   {0, 11},  {9, 19},
      {13, 19}, {0, 9},   {5, 19},  {6, 0},   {19, 14},
      {13, 0},  {0, 13},  {6, 19},  {7, 19},  {19, 5},
      {14, 19}, {0, 12},  {11, 19}, {8, 19},  {12, 19}};
  ASSERT_EQ(goal_coordinates.size(), ins.n_targets());

  std::vector<int> tau;
  tau.reserve(goal_coordinates.size());
  for (const auto& [row, col] : goal_coordinates)
    tau.push_back(ins.grid.idx(row, col));

  Deadline deadline(2000);
  const auto result = solve_carrier_br_upper(
      ins, br_labeled_initial_state(ins), tau, &deadline);

  ASSERT_TRUE(result.solved())
      << "explored=" << result.stats.explored
      << " loops=" << result.stats.loop_count;
}
