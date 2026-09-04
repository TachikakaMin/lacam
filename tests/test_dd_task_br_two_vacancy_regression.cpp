// PROTECTED two-vacancy priority-progress regression.
// Added before implementation on 2026-09-03 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>

#include "gtest/gtest.h"

TEST(dd_task_br_two_vacancy_regression,
     priority_progress_keeps_seed1_deliverable)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g8x10/"
      "brap_h8w10_a10_e2_R1_seed1.yaml";
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 10.0, 0, &stats);

  ASSERT_FALSE(plan.empty())
      << "two-vacancy regression timed out: best_targets_done="
      << stats.best_targets_done
      << " joint_shared_effects=" << stats.joint_shared_effects;
  auto state = initial_phys_config(ins);
  for (const auto& ops : plan) {
    const auto next = apply_ops(ins, state, ops);
    ASSERT_TRUE(next.has_value());
    state = *next;
  }
  EXPECT_TRUE(is_dd_goal(ins, state));
  EXPECT_GE(stats.deliverable_ms, 0);
  EXPECT_LE(stats.deliverable_ms, 10000);
}
