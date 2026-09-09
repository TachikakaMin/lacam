// PROTECTED settled-target oscillation regression.
// Written after vacancy-aware guidance entered a two-state cycle near the
// end of brap_h10w10_a12_e8_R1_seed0.  Intentionally observed RED before
// the implementation fix on 2026-09-06.
#include <br_lacam_upper.hpp>
#include <dd_carrier.hpp>

#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_oscillation_regression,
     upper_search_finishes_without_repeatedly_disturbing_settled_targets)
{
  const auto ins = load_dd_instance(
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap/g10x10/"
      "brap_h10w10_a12_e8_R1_seed0.yaml");
  const std::vector<std::pair<int, int>> goal_coordinates{
      {1, 9}, {5, 3}, {8, 4}, {4, 2},
      {7, 8}, {0, 9}, {3, 3}, {9, 7},
      {3, 6}, {2, 7}, {3, 9}, {2, 9}};
  std::vector<int> tau;
  tau.reserve(goal_coordinates.size());
  for (const auto& [row, col] : goal_coordinates)
    tau.push_back(ins.grid.idx(row, col));

  Deadline deadline(2000);
  const auto result = solve_carrier_br_upper(
      ins, br_labeled_initial_state(ins), tau, &deadline);

  ASSERT_TRUE(result.solved());
}
