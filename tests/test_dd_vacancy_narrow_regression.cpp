// PROTECTED narrow single-vacancy carrier regression.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

TEST(dd_vacancy_narrow_regression,
     seed1_keeps_the_pre_commitment_solution_band)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g4x10/"
      "brap_h4w10_a5_e1_R1_seed1.yaml";
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto result =
      solve_carrier_lacam_result(ins, 10.0, 0, &stats);

  ASSERT_TRUE(result.solved());
  EXPECT_LE(stats.best_makespan, 389);
  EXPECT_LE(stats.best_soc, 769);
}
