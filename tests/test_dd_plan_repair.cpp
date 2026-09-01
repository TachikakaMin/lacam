#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <cstdlib>

#include "gtest/gtest.h"

namespace {

DDInstance make_repair_case()
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {ins.grid.idx(0, 0), ins.grid.idx(1, 2)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(1, 1)};
  ins.finalize();
  return ins;
}

bool valid(const DDInstance& ins, const DDPlan& plan)
{
  auto state = initial_phys_config(ins);
  for (const auto& ops : plan) {
    auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return false;
    state = *next;
  }
  return is_dd_goal(ins, state);
}

}  // namespace

TEST(dd_plan_repair, cuts_repeated_shelf_projection_and_repairs_robots)
{
  const auto ins = make_repair_case();
  const auto W = Op::make_wait();
  DDPlan plan = {
      {Op::make_move(ins.grid.idx(0, 1)), W},
      {Op::make_lift(), W},
      {W, W},
      {Op::make_drop(), W},
      {Op::make_lift(), W},
      {Op::make_move(ins.grid.idx(1, 1)), W},
      {Op::make_drop(), W},
  };
  ASSERT_TRUE(valid(ins, plan));

  DDPlanRepairStats stats;
  const auto repaired = repair_carrier_plan(ins, plan, &stats);
  ASSERT_TRUE(valid(ins, repaired));
  EXPECT_EQ(repaired.size(), 4u);
  EXPECT_EQ(stats.projected_loops, 1);
  EXPECT_EQ(stats.bridge_steps, 1);
  EXPECT_EQ(stats.steps_removed, 3);
  EXPECT_EQ(repaired[0][0].kind, Op::MOVE);
  EXPECT_EQ(repaired[1][0].kind, Op::LIFT);
}

TEST(dd_plan_repair, keeps_already_irreducible_plan)
{
  const auto ins = make_repair_case();
  const auto W = Op::make_wait();
  DDPlan plan = {
      {Op::make_move(ins.grid.idx(0, 1)), W},
      {Op::make_lift(), W},
      {Op::make_move(ins.grid.idx(1, 1)), W},
      {Op::make_drop(), W},
  };
  DDPlanRepairStats stats;
  const auto repaired = repair_carrier_plan(ins, plan, &stats);
  EXPECT_EQ(repaired, plan);
  EXPECT_EQ(stats.steps_removed, 0);
}

TEST(dd_plan_repair, cuts_exact_physical_state_loop)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 2)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();
  DDPlan plan = {
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_move(ins.grid.idx(0, 0))},
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_move(ins.grid.idx(0, 2))},
      {Op::make_lift()},
      {Op::make_move(ins.grid.idx(0, 3))},
      {Op::make_drop()},
  };
  ASSERT_TRUE(valid(ins, plan));

  DDPlanRepairStats stats;
  const auto repaired = repair_carrier_plan(ins, plan, &stats);
  EXPECT_TRUE(valid(ins, repaired));
  EXPECT_EQ(repaired.size(), 5u);
  EXPECT_EQ(stats.exact_loops, 1);
  EXPECT_EQ(stats.steps_removed, 2);
}

TEST(dd_plan_repair, multi_robot_fallback_projects_original_lower_path)
{
  DDInstance ins;
  ins.grid = DDGrid({"....", "...."});
  ins.robots = {
      ins.grid.idx(0, 0),
      ins.grid.idx(1, 0),
      ins.grid.idx(1, 3),
  };
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();
  const auto W = Op::make_wait();
  DDPlan plan = {
      {Op::make_move(ins.grid.idx(0, 1)), W, W},
      {Op::make_lift(), W, W},
      {W, Op::make_move(ins.grid.idx(1, 1)), W},
      {Op::make_drop(), W, W},
      {Op::make_lift(), W, W},
      {Op::make_move(ins.grid.idx(0, 2)), W, W},
      {Op::make_move(ins.grid.idx(0, 3)), W, W},
      {Op::make_drop(), W, W},
  };
  ASSERT_TRUE(valid(ins, plan));

  DDPlanRepairStats stats;
  const auto repaired = repair_carrier_plan(ins, plan, &stats);
  EXPECT_TRUE(valid(ins, repaired));
  EXPECT_EQ(repaired.size(), 6u);
  EXPECT_EQ(stats.projected_loops, 1);
  EXPECT_EQ(stats.bridge_steps, 2);
  EXPECT_EQ(stats.steps_removed, 2);
}

namespace {

// weighted SOC replay with explicit weights (independent arithmetic)
double plan_soc_w(const DDInstance& ins, const DDPlan& plan, double alpha,
                  double beta, double gamma, double delta)
{
  auto s = initial_phys_config(ins);
  double c = 0;
  for (const auto& ops : plan) {
    for (size_t i = 0; i < ops.size(); ++i) {
      if (ops[i].kind == Op::MOVE) {
        c += s.kappa[i] == KAPPA_FREE ? beta : alpha;
        if (s.kappa[i] == KAPPA_ANON) c += delta;
      } else if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) {
        c += gamma;
      }
    }
    auto next = apply_ops(ins, s, ops);
    if (!next.has_value()) return -1;
    s = *next;
  }
  return c;
}

}  // namespace

// review fix batch 2026-09-01: repair accepts bridges by STEP count while
// production candidate selection is by weighted SOC. A min-step bridge may
// move robots that originally waited. Pin the contract under adversarial
// weights (free moves expensive, lift/drop churn cheap): the returned plan
// never costs more than the raw plan.
TEST(dd_plan_repair, repaired_soc_never_exceeds_raw_under_any_weights)
{
  setenv("DD_ALPHA", "0.125", 1);
  setenv("DD_BETA", "8", 1);
  setenv("DD_GAMMA", "0.125", 1);
  setenv("DD_DELTA", "0.125", 1);

  const auto ins = make_repair_case();
  const auto W = Op::make_wait();
  // churn segment (cheap gamma) + free-robot walk (expensive beta)
  DDPlan plan = {
      {Op::make_move(ins.grid.idx(0, 1)), W},
      {Op::make_lift(), W},
      {W, Op::make_move(ins.grid.idx(1, 1))},
      {W, Op::make_move(ins.grid.idx(1, 0))},
      {Op::make_drop(), W},
      {Op::make_lift(), W},
      {W, Op::make_move(ins.grid.idx(1, 1))},
      {W, Op::make_move(ins.grid.idx(1, 2))},
      {Op::make_move(ins.grid.idx(1, 1)), W},
      {Op::make_drop(), W},
  };
  ASSERT_TRUE(valid(ins, plan));
  const double raw = plan_soc_w(ins, plan, 0.125, 8, 0.125, 0.125);
  ASSERT_GE(raw, 0);

  const auto repaired = repair_carrier_plan(ins, plan, nullptr);
  ASSERT_TRUE(valid(ins, repaired));
  EXPECT_LE(repaired.size(), plan.size());
  const double rep = plan_soc_w(ins, repaired, 0.125, 8, 0.125, 0.125);
  ASSERT_GE(rep, 0);
  EXPECT_LE(rep, raw + 1e-9);

  unsetenv("DD_ALPHA");
  unsetenv("DD_BETA");
  unsetenv("DD_GAMMA");
  unsetenv("DD_DELTA");
}
