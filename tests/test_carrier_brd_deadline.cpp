// PROTECTED REGRESSION: BRD search plus mandatory cleanup must return within
// the caller's hard deadline even when runtime improvements make the former
// timeout trigger solvable.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <chrono>
#include <string>

#include "gtest/gtest.h"

TEST(carrier_brd_deadline,
     upper_timeout_includes_constraint_tree_cleanup)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g4x10/"
      "brap_h4w10_a5_e1_R1_seed0.yaml";
  const auto ins = load_dd_instance(path);
  CarrierBRDStats brd;

  const auto started = std::chrono::steady_clock::now();
  const auto result =
      solve_carrier_brd_result(ins, 3.0, 0, nullptr, &brd);
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started)
          .count();

  ASSERT_TRUE(
      result.status == DDSolveStatus::SOLVED ||
      result.status == DDSolveStatus::TIMEOUT);
  if (result.status == DDSolveStatus::SOLVED)
    EXPECT_EQ(brd.exit_reason, CarrierBRDExitReason::SOLVED);
  else
    EXPECT_EQ(brd.exit_reason, CarrierBRDExitReason::UPPER_TIMEOUT);
  EXPECT_LE(elapsed_ms, 3000.0);
}
