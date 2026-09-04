// PROTECTED v4.1 benchmark regressions (TDD RED), 2026-09-02.
// Both rows are successes in results_v3_round3_final under the same
// carrier/seed/following/unit-weight/strict-10s protocol.  Objective task
// merging must not turn either into a timeout.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

namespace {

::testing::AssertionResult valid_goal_plan(const DDInstance& ins,
                                           const DDPlan& plan)
{
  if (plan.empty()) return ::testing::AssertionFailure() << "empty plan";
  auto state = initial_phys_config(ins);
  for (size_t t = 0; t < plan.size(); ++t) {
    auto next = apply_ops(ins, state, plan[t]);
    if (!next.has_value())
      return ::testing::AssertionFailure() << "illegal step " << t;
    state = std::move(*next);
  }
  if (!is_dd_goal(ins, state))
    return ::testing::AssertionFailure() << "final state is not a goal";
  return ::testing::AssertionSuccess();
}

void expect_strict_gate_success(const std::string& relative_path)
{
  const auto path =
      std::string(DD_TEST_DIR) + "/../benchmark/instances_brap_pool/" +
      relative_path;
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 10.0, 0, &stats);
  ASSERT_FALSE(plan.empty())
      << "baseline-success row regressed: timed_out=" << stats.timed_out
      << " first_solution_ms=" << stats.first_solution_ms
      << " best_targets_done=" << stats.best_targets_done
      << " joint_shared_effects=" << stats.joint_shared_effects;
  EXPECT_TRUE(valid_goal_plan(ins, plan));
  EXPECT_GE(stats.deliverable_ms, 0);
  EXPECT_LE(stats.deliverable_ms, 10000);
}

}  // namespace

TEST(dd_objective_benchmark_regression,
     pool_seed1_remains_deliverable_within_strict_deadline)
{
  expect_strict_gate_success(
      "g10x10/brap_h10w10_a12_e3_B_seed1_pool.yaml");
}

TEST(dd_objective_benchmark_regression,
     singleton_seed0_remains_deliverable_within_strict_deadline)
{
  expect_strict_gate_success(
      "g10x10/brap_h10w10_a12_e3_R1_seed0.yaml");
}

TEST(dd_objective_benchmark_regression,
     roomy_pool_seed0_remains_deliverable_within_strict_deadline)
{
  expect_strict_gate_success(
      "g10x10/brap_h10w10_a12_e8_B_seed0_pool.yaml");
}

TEST(dd_objective_benchmark_regression,
     roomy_pool_seed1_remains_deliverable_within_strict_deadline)
{
  expect_strict_gate_success(
      "g10x10/brap_h10w10_a12_e8_B_seed1_pool.yaml");
}
