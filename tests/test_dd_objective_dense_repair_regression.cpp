// PROTECTED dense-plan repair regressions (TDD RED), 2026-09-02.
// These baseline-success rows still find a valid incumbent under v4.1,
// but their 150k+ raw steps must be compacted into a deliverable before
// the same strict 10s deadline.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

namespace {

void expect_dense_repair_gate_success(const std::string& relative_path)
{
  const auto path =
      std::string(DD_TEST_DIR) + "/../benchmark/instances_brap_pool/" +
      relative_path;
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 10.0, 0, &stats);
  ASSERT_FALSE(plan.empty())
      << "incumbent was not repaired in-budget: timed_out="
      << stats.timed_out << " first_solution_ms=" << stats.first_solution_ms
      << " first_solution_soc=" << stats.first_solution_soc;
  auto state = initial_phys_config(ins);
  for (size_t t = 0; t < plan.size(); ++t) {
    auto next = apply_ops(ins, state, plan[t]);
    ASSERT_TRUE(next.has_value()) << "illegal repaired step " << t;
    state = std::move(*next);
  }
  EXPECT_TRUE(is_dd_goal(ins, state));
  EXPECT_GE(stats.deliverable_ms, 0);
  EXPECT_LE(stats.deliverable_ms, 10000);
}

}  // namespace

TEST(dd_objective_dense_repair_regression,
     h10_e8_seed1_compacts_before_deadline)
{
  expect_dense_repair_gate_success(
      "g10x10/brap_h10w10_a12_e8_R1_seed1.yaml");
}

TEST(dd_objective_dense_repair_regression,
     h8_e2_seed0_compacts_before_deadline)
{
  expect_dense_repair_gate_success(
      "g8x10/brap_h8w10_a10_e2_R1_seed0.yaml");
}
