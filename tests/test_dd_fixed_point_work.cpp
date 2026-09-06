#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <cstdlib>

#include "gtest/gtest.h"

namespace {

struct WeightEnvGuard {
  ~WeightEnvGuard()
  {
    unsetenv("DD_ALPHA");
    unsetenv("DD_BETA");
    unsetenv("DD_GAMMA");
    unsetenv("DD_DELTA");
  }
};

DDInstance make_micro_weight_case()
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_fixed_point_work, rejects_weights_outside_the_micro_unit_grid)
{
  WeightEnvGuard guard;
  setenv("DD_ALPHA", "0.0000004", 1);
  EXPECT_THROW(dd_load_soc_weights(), std::invalid_argument);
  EXPECT_THROW(
      PlanCost::from_values(0, 0.0000004),
      std::invalid_argument);
}

TEST(dd_fixed_point_work,
     search_and_final_replay_share_the_same_scaled_integer_work)
{
  WeightEnvGuard guard;
  setenv("DD_ALPHA", "0.000001", 1);
  setenv("DD_BETA", "0", 1);
  setenv("DD_GAMMA", "0.000001", 1);
  setenv("DD_DELTA", "0", 1);

  const auto ins = make_micro_weight_case();
  DDStats stats;
  const auto result = solve_carrier_lacam_result(
      ins, 2.0, 0, &stats);
  ASSERT_TRUE(result.solved());

  const auto replayed = dd_plan_cost_probe(ins, result.plan);
  ASSERT_TRUE(replayed.is_bounded());
  EXPECT_EQ(replayed.work, 3);
  EXPECT_EQ(stats.best_work_scaled, replayed.work);
  EXPECT_EQ(stats.first_solution_work_scaled, replayed.work);
  EXPECT_DOUBLE_EQ(stats.best_soc, replayed.work_value());
  EXPECT_DOUBLE_EQ(stats.first_solution_soc, replayed.work_value());
}
