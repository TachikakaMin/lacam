// REGRESSION: completion-event replanning must remain practical on the
// 20x20 BRaP-pool quick cases; each event still certifies a full active
// frontier before the controller commits the first Drop.
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

namespace {

void expect_completion_event_replan_within_budget(
    const std::string& filename)
{
  const auto instance_path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g20x20/" +
      filename;
  const auto ins = load_dd_instance(instance_path);
  CarrierBRDStats brd;
  const auto result = solve_carrier_brd_result(
      ins, 10.0, 0, nullptr, &brd);
  EXPECT_TRUE(result.solved());
  EXPECT_EQ(brd.exit_reason, CarrierBRDExitReason::SOLVED);
  EXPECT_GE(brd.completion_events, 1);
}

}  // namespace

TEST(carrier_brd_completion_event_performance,
     h20_e10_seed0_stays_within_quick_budget)
{
  expect_completion_event_replan_within_budget(
      "brap_h20w20_a40_e10_B_seed0_pool.yaml");
}

TEST(carrier_brd_completion_event_performance,
     h20_e10_seed1_stays_within_quick_budget)
{
  expect_completion_event_replan_within_budget(
      "brap_h20w20_a40_e10_B_seed1_pool.yaml");
}
