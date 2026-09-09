// PROTECTED TEST: persistent Carrier-LaCAM continuation contract,
// integration plan Phase 6. Written before implementation (TDD RED).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <optional>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_single_target_case(bool flexible_goals = false)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goal_sets = {
      flexible_goals
          ? std::vector<int>{
                ins.grid.idx(0, 3), ins.grid.idx(0, 4)}
          : std::vector<int>{ins.grid.idx(0, 4)}};
  ins.finalize();
  return ins;
}

std::optional<PhysConfig> replay_prefix(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan, size_t steps)
{
  if (steps > plan.size()) return std::nullopt;
  PhysConfig state = root;
  for (size_t step = 0; step < steps; ++step) {
    const auto next = apply_ops(ins, state, plan[step]);
    if (!next.has_value()) return std::nullopt;
    state = *next;
  }
  return state;
}

::testing::AssertionResult valid_goal_plan_from(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  const auto terminal =
      replay_prefix(ins, root, plan, plan.size());
  if (!terminal.has_value())
    return ::testing::AssertionFailure()
           << "plan is not legal from the supplied root";
  if (!is_dd_goal(ins, *terminal))
    return ::testing::AssertionFailure()
           << "plan does not reach a legal target goal";
  return ::testing::AssertionSuccess();
}

int terminal_target_cell(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  const auto terminal =
      replay_prefix(ins, root, plan, plan.size());
  if (!terminal.has_value() || terminal->target_pos.empty())
    return -1;
  return terminal->target_pos[0];
}

}  // namespace

TEST(dd_incremental_session,
     robot_only_prefix_hits_the_existing_root_tau_epoch)
{
  const DDInstance ins = make_single_target_case();
  const PhysConfig initial = initial_phys_config(ins);
  DDPlanningSession session(ins, initial, 0);

  DDStats first_stats;
  const auto first = session.solve(2.0, &first_stats);
  ASSERT_TRUE(first.solved());
  ASSERT_FALSE(first.plan.empty());
  const auto observed =
      replay_prefix(ins, initial, first.plan, 1);
  ASSERT_TRUE(observed.has_value());
  EXPECT_EQ(observed->target_pos, initial.target_pos);
  EXPECT_EQ(observed->anon_occ, initial.anon_occ);

  ASSERT_EQ(
      session.commit_prefix(1, *observed),
      DDCommitStatus::OK);
  DDStats warm_stats;
  const auto warm = session.solve(2.0, &warm_stats);

  ASSERT_TRUE(warm.solved());
  EXPECT_TRUE(valid_goal_plan_from(ins, *observed, warm.plan));
  EXPECT_GT(warm_stats.root_pair_cache_hits, 0);
  EXPECT_EQ(warm_stats.root_pair_cache_misses, 0);
}

TEST(dd_incremental_session,
     mismatched_observation_is_rejected_without_consuming_the_plan)
{
  const DDInstance ins = make_single_target_case();
  const PhysConfig initial = initial_phys_config(ins);
  DDPlanningSession session(ins, initial, 0);
  const auto first = session.solve(2.0);
  ASSERT_TRUE(first.solved());
  ASSERT_FALSE(first.plan.empty());
  const auto expected =
      replay_prefix(ins, initial, first.plan, 1);
  ASSERT_TRUE(expected.has_value());

  EXPECT_EQ(
      session.commit_prefix(1, initial),
      DDCommitStatus::STATE_MISMATCH);
  EXPECT_EQ(
      session.commit_prefix(1, *expected),
      DDCommitStatus::OK);
  EXPECT_TRUE(session.solve(2.0).solved());
}

TEST(dd_incremental_session,
     carried_target_keeps_the_goal_selected_before_the_prefix)
{
  const DDInstance ins = make_single_target_case(
      /*flexible_goals=*/true);
  const PhysConfig initial = initial_phys_config(ins);
  DDPlanningSession session(ins, initial, 0);
  const auto first = session.solve(2.0);
  ASSERT_TRUE(first.solved());
  const int selected_goal =
      terminal_target_cell(ins, initial, first.plan);
  ASSERT_NE(selected_goal, -1);

  size_t carried_steps = 0;
  std::optional<PhysConfig> carried;
  for (size_t steps = 1; steps <= first.plan.size(); ++steps) {
    const auto state =
        replay_prefix(ins, initial, first.plan, steps);
    ASSERT_TRUE(state.has_value());
    if (state->kappa[0] == 0 &&
        state->target_pos[0] != ins.target_starts[0]) {
      carried_steps = steps;
      carried = state;
      break;
    }
  }
  ASSERT_GT(carried_steps, 0u);
  ASSERT_TRUE(carried.has_value());

  ASSERT_EQ(
      session.commit_prefix(carried_steps, *carried),
      DDCommitStatus::OK);
  const auto warm = session.solve(2.0);
  ASSERT_TRUE(warm.solved());
  ASSERT_EQ(session.root_goal_commitment().count(0), 1u);
  EXPECT_EQ(
      session.root_goal_commitment().at(0),
      selected_goal);
  EXPECT_EQ(
      terminal_target_cell(ins, *carried, warm.plan),
      selected_goal);
}

TEST(dd_incremental_session,
     warm_and_cold_solve_share_the_same_goal_semantics)
{
  const DDInstance ins = make_single_target_case();
  const PhysConfig initial = initial_phys_config(ins);
  DDPlanningSession session(ins, initial, 7);
  const auto first = session.solve(2.0);
  ASSERT_TRUE(first.solved());
  const auto observed =
      replay_prefix(ins, initial, first.plan, 1);
  ASSERT_TRUE(observed.has_value());
  ASSERT_EQ(
      session.commit_prefix(1, *observed),
      DDCommitStatus::OK);

  const auto warm = session.solve(2.0);
  const auto cold = solve_carrier_lacam_from_state_result(
      ins, *observed, 2.0, 7);
  ASSERT_EQ(warm.status, cold.status);
  ASSERT_TRUE(warm.solved());
  ASSERT_TRUE(cold.solved());
  EXPECT_TRUE(valid_goal_plan_from(ins, *observed, warm.plan));
  EXPECT_TRUE(valid_goal_plan_from(ins, *observed, cold.plan));
  EXPECT_EQ(
      terminal_target_cell(ins, *observed, warm.plan),
      terminal_target_cell(ins, *observed, cold.plan));
}
