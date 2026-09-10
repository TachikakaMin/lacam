// REGRESSION: a matcher cutoff happens before a dispatch is committed, so
// it must not increment the dispatch-matcher accounting pair.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

TEST(carrier_brd_dispatch_timeout_telemetry,
     cutoff_matcher_does_not_outnumber_committed_dispatches)
{
  const auto instance_path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g40x40/"
      "brap_h40w40_a160_e400_R1_seed0.yaml";
  const auto ins = load_dd_instance(instance_path);
  DDStats stats;
  CarrierBRDStats brd;

  const auto result = solve_carrier_brd_result(
      ins, 10.0, 0, &stats, &brd);

  EXPECT_FALSE(result.solved());
  EXPECT_EQ(brd.exit_reason, CarrierBRDExitReason::DISPATCH_TIMEOUT);
  EXPECT_EQ(brd.match_calls, brd.dispatch_epochs);
}
