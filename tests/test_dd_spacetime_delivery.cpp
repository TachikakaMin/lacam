// PROTECTED TEST: Phase 8.4 commitment-aware delivery and repair.
// Written before implementation (TDD RED).
#include <dd_planner.hpp>

#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance carrier_line()
{
  DDInstance ins;
  ins.grid = DDGrid({".."});
  ins.robots = {0};
  ins.shelves = {0};
  ins.target_starts = {0};
  ins.target_goal_sets = {{1}};
  ins.finalize();
  return ins;
}

CarrierSpacetimeCommitment move_is_blocked_at_tick_one()
{
  CarrierSpacetimeCommitment commitment;
  commitment.frames.resize(5);
  commitment.lower_directed_edges.resize(4);
  commitment.frames[2].lower_vertices = {1};
  return commitment;
}

bool replay_with_commitment(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan,
    const CarrierSpacetimeCommitment& commitment)
{
  PhysConfig state = root;
  for (size_t tick = 0; tick < plan.size(); ++tick) {
    const auto next = apply_ops(
        ins, state, plan[tick], true, &commitment,
        static_cast<int64_t>(tick));
    if (!next.has_value()) return false;
    state = *next;
  }
  return is_dd_goal(ins, state);
}

}  // namespace

TEST(dd_spacetime_delivery,
     repair_preserves_a_wait_required_by_external_traffic)
{
  const DDInstance ins = carrier_line();
  const PhysConfig root = initial_phys_config(ins);
  const CarrierSpacetimeCommitment commitment =
      move_is_blocked_at_tick_one();
  const DDPlan plan = {
      {Op::make_lift()},
      {Op::make_wait()},
      {Op::make_move(1)},
      {Op::make_drop()},
  };
  ASSERT_TRUE(
      replay_with_commitment(ins, root, plan, commitment));

  DDPlanRepairStats stats;
  const DDPlan repaired = repair_carrier_plan(
      ins, root, plan, &stats, nullptr, &commitment, 0);
  EXPECT_EQ(repaired, plan);
}

TEST(dd_spacetime_delivery,
     cold_solver_returns_a_plan_valid_under_the_same_commitment)
{
  const DDInstance ins = carrier_line();
  const PhysConfig root = initial_phys_config(ins);
  const CarrierSpacetimeCommitment commitment =
      move_is_blocked_at_tick_one();

  const DDSolveResult result =
      solve_carrier_lacam_from_state_result(
          ins, root, 2.0, 0, nullptr, nullptr,
          &commitment, 0);
  ASSERT_TRUE(result.solved());
  EXPECT_TRUE(
      replay_with_commitment(
          ins, root, result.plan, commitment));
  EXPECT_GE(result.plan.size(), 4u);
}
