#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <utils.hpp>

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

struct WeightEnvGuard {
  ~WeightEnvGuard()
  {
    unsetenv("DD_ALPHA");
    unsetenv("DD_BETA");
    unsetenv("DD_GAMMA");
    unsetenv("DD_DELTA");
  }
};

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

// Carrier-LaCAM v5 protected-test migration APPROVE (2026-09-05): a real
// generated repair must be accepted by the same production predicate used
// at the repair boundary.
TEST(dd_plan_repair, shorter_valid_repair_uses_shared_plan_cost_order)
{
  WeightEnvGuard guard;
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
  EXPECT_LT(repaired.size(), plan.size());
  const double rep = plan_soc_w(ins, repaired, 0.125, 8, 0.125, 0.125);
  ASSERT_GE(rep, 0);
  EXPECT_LT(rep, raw);
  EXPECT_TRUE(dd_repair_accepts_candidate_probe(ins, plan, repaired));
}

// The generator only constructs shorter bridges, so exercise the production
// acceptance predicate directly with two legal goal plans.  The five-tick
// candidate deliberately pays for two expensive free moves; strict
// makespan-first order must accept it over the six-tick low-work incumbent.
// Reversing the arguments proves that longer/lower-work is rejected.
TEST(dd_plan_repair,
     production_predicate_is_ticks_then_work_in_both_tradeoff_directions)
{
  WeightEnvGuard guard;
  setenv("DD_ALPHA", "1", 1);
  setenv("DD_BETA", "10", 1);
  setenv("DD_GAMMA", "1", 1);
  setenv("DD_DELTA", "0", 1);

  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();

  const auto W = Op::make_wait();
  const DDPlan shorter_high_work = {
      {Op::make_move(ins.grid.idx(0, 0))},
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_lift()},
      {Op::make_move(ins.grid.idx(0, 2))},
      {Op::make_drop()},
  };
  const DDPlan longer_low_work = {
      {W},
      {W},
      {W},
      {Op::make_lift()},
      {Op::make_move(ins.grid.idx(0, 2))},
      {Op::make_drop()},
  };
  ASSERT_TRUE(valid(ins, shorter_high_work));
  ASSERT_TRUE(valid(ins, longer_low_work));
  ASSERT_LT(shorter_high_work.size(), longer_low_work.size());
  ASSERT_GT(
      plan_soc_w(ins, shorter_high_work, 1, 10, 1, 0),
      plan_soc_w(ins, longer_low_work, 1, 10, 1, 0));

  EXPECT_TRUE(dd_repair_accepts_candidate_probe(
      ins, longer_low_work, shorter_high_work));
  EXPECT_FALSE(dd_repair_accepts_candidate_probe(
      ins, shorter_high_work, longer_low_work));
}

// 2026-09-02 R1 (debug.md §10, TDD RED): repair is part of the pass's
// 10s deadline.  With an exhausted deadline the repair must abort and
// return the RAW plan unchanged (still valid); with a fresh deadline the
// same plan must still be repaired.  Root cause: v3.0's longer raw
// incumbents made post-deadline repair balloon a borderline gate row to
// 14.4s wall against the strict 10s protocol.
TEST(dd_plan_repair, expired_deadline_aborts_to_raw_plan)
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

  Deadline expired(-1.0);  // deterministically past its budget (elapsed_ms truncates to whole ms)
  DDPlanRepairStats stats;
  const auto kept = repair_carrier_plan(ins, plan, &stats, &expired);
  EXPECT_EQ(kept.size(), plan.size()) << "expired deadline must return raw";
  EXPECT_EQ(stats.steps_removed, 0);
  ASSERT_TRUE(valid(ins, kept));

  Deadline fresh(5000.0);
  const auto repaired = repair_carrier_plan(ins, plan, nullptr, &fresh);
  EXPECT_LT(repaired.size(), plan.size())
      << "fresh deadline must still repair";
  ASSERT_TRUE(valid(ins, repaired));
}
