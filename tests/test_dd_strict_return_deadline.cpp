// PROTECTED regression test: the strict solver deadline includes destruction
// of deferred search state, not only the pre-destruction plan timestamp.
// Written TDD RED, 2026-09-02.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <chrono>
#include <string>

#include "gtest/gtest.h"

TEST(dd_strict_return_deadline, successful_solver_call_returns_in_budget)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g6x10/"
      "brap_h6w10_a6_e1_B_seed1_pool.yaml";
  const auto ins = load_dd_instance(path);

  DDStats stats;
  const auto started = std::chrono::steady_clock::now();
  const auto plan = solve_carrier_lacam(ins, 10.0, 0, &stats);
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started)
          .count();

  ASSERT_FALSE(plan.empty());
  EXPECT_LE(elapsed_ms, 10000.0);
  ASSERT_GE(stats.deliverable_ms, 0);
  EXPECT_LE(stats.deliverable_ms, 10000.0);
  EXPECT_LE(elapsed_ms - stats.deliverable_ms, 100.0)
      << "deliverable_ms must be recorded after deferred planner cleanup";
}
