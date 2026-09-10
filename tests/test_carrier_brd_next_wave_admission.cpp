// REGRESSION: a next-wave continuation waits until its upper-route tail is
// physically clear; source readiness alone is not enough.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

TEST(carrier_brd_next_wave_admission,
     waits_for_current_wave_storage_before_early_continuation)
{
  const auto instance_path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g10x10/"
      "brap_h10w10_a12_e8_R1_seed0.yaml";
  const auto ins = load_dd_instance(instance_path);
  DDStats stats;
  CarrierBRDStats brd;

  const auto result = solve_carrier_brd_result(
      ins, 5.0, 0, &stats, &brd);

  ASSERT_TRUE(result.solved());
  EXPECT_EQ(brd.exit_reason, CarrierBRDExitReason::SOLVED);
  EXPECT_TRUE(brd.raw_plan_valid);
}
