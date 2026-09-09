// PROTECTED carrier vacancy regressions from debug.md phase F.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <optional>
#include <string>

#include "gtest/gtest.h"

namespace {

struct PlanMetrics {
  long makespan = 0;
  long loaded_moves = 0;
  long free_moves = 0;
  long lift_drop = 0;
  long anon_moves = 0;

  long weighted_soc() const
  {
    return loaded_moves + free_moves + lift_drop + anon_moves;
  }
};

std::optional<PlanMetrics> replay_metrics(
    const DDInstance& ins, const DDPlan& plan)
{
  PlanMetrics metrics;
  auto state = initial_phys_config(ins);
  bool reached_goal = is_dd_goal(ins, state);
  for (const auto& ops : plan) {
    if (reached_goal) break;
    if (ops.size() != state.robots.size()) return std::nullopt;
    for (size_t robot = 0; robot < ops.size(); ++robot) {
      if (ops[robot].kind == Op::MOVE) {
        if (state.kappa[robot] == KAPPA_FREE) {
          ++metrics.free_moves;
        } else {
          ++metrics.loaded_moves;
          if (state.kappa[robot] == KAPPA_ANON)
            ++metrics.anon_moves;
        }
      } else if (
          ops[robot].kind == Op::LIFT ||
          ops[robot].kind == Op::DROP) {
        ++metrics.lift_drop;
      }
    }
    const auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    state = *next;
    ++metrics.makespan;
    reached_goal = is_dd_goal(ins, state);
  }
  if (!reached_goal) return std::nullopt;
  return metrics;
}

DDSolveResult solve_case(
    const std::string& relative_path, DDStats& stats)
{
  const auto path =
      std::string(DD_TEST_DIR) + "/../" + relative_path;
  return solve_carrier_lacam_result(
      load_dd_instance(path), 10.0, 0, &stats);
}

}  // namespace

TEST(dd_carrier_vacancy_regression,
     multi_target_pool_returns_to_the_pre_vacancy_soc_band)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g10x10/"
      "brap_h10w10_a12_e3_B_seed1_pool.yaml";
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto result =
      solve_carrier_lacam_result(ins, 10.0, 0, &stats);

  ASSERT_TRUE(result.solved());
  const auto metrics = replay_metrics(ins, result.plan);
  ASSERT_TRUE(metrics.has_value());
  EXPECT_LE(metrics->weighted_soc(), 1825)
      << "vacancy guidance must stay within 5% of the 1738 baseline";
}

TEST(dd_carrier_vacancy_regression,
     warehouse_single_target_avoids_free_robot_churn)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/viz_web/warehouse_block_suite/instances/"
      "warehouse_blocks_h20w20_b3_a1_d50_r8_t12_seed0.yaml";
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto result =
      solve_carrier_lacam_result(ins, 10.0, 0, &stats);

  ASSERT_TRUE(result.solved());
  const auto metrics = replay_metrics(ins, result.plan);
  ASSERT_TRUE(metrics.has_value());
  EXPECT_EQ(metrics->loaded_moves, 33);
  EXPECT_EQ(metrics->lift_drop, 24);
  EXPECT_LE(metrics->free_moves, 110);
}

TEST(dd_carrier_vacancy_regression,
     report_case_keeps_the_shared_vacancy_improvement)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g10x10/"
      "brap_h10w10_a1_e1_B_seed0_pool.yaml";
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto result =
      solve_carrier_lacam_result(ins, 10.0, 0, &stats);

  ASSERT_TRUE(result.solved());
  const auto metrics = replay_metrics(ins, result.plan);
  ASSERT_TRUE(metrics.has_value());
  EXPECT_LE(metrics->makespan, 49);
  EXPECT_LE(metrics->weighted_soc(), 101);
}
