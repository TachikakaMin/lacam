// REGRESSION: tasks whose endpoints are occupied by one another must reach
// LaCAM together; Drop legality is checked during the lower plan, not at
// dispatch admission.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

TEST(carrier_brd_endpoint_cycle,
     admits_a_storage_cycle_until_lacam_orders_the_drops)
{
  const auto instance_path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g10x10/"
      "brap_h10w10_a12_e3_B_seed1_pool.yaml";
  const auto ins = load_dd_instance(instance_path);
  DDStats stats;
  CarrierBRDStats brd;

  const auto result = solve_carrier_brd_result(
      ins, 5.0, 0, &stats, &brd);

  ASSERT_TRUE(result.solved());
  EXPECT_EQ(brd.exit_reason, CarrierBRDExitReason::SOLVED);
  EXPECT_TRUE(brd.raw_plan_valid);
  EXPECT_GT(brd.completion_events, 0);
}
