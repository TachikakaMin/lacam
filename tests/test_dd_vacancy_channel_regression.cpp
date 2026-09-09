// PROTECTED vacancy-aware channel-map regression.
// Written after observing the 34 -> 43 tick regression on 2026-09-06
// and intentionally observed RED before the implementation fix.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

TEST(dd_vacancy_channel_regression,
     frozen_channel_case_does_not_regress_the_brd_baseline)
{
  const auto ins = load_dd_instance(
      std::string(DD_TEST_DIR) +
      "/../benchmark/viz_web/warehouse_block_suite/instances/"
      "warehouse_blocks_h20w20_b4_a1_d75_r8_t12_seed0.yaml");
  DDStats stats;
  CarrierBRDStats brd;
  const auto result =
      solve_carrier_brd_result(ins, 10.0, 0, &stats, &brd);

  ASSERT_TRUE(result.solved());
  const PlanCost cost = dd_plan_cost_probe(ins, result.plan);
  EXPECT_LE(cost.ticks, 34);
  EXPECT_LE(cost.work, 160000000);
  EXPECT_LE(brd.upper_transfers, 14);
}
