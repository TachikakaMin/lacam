// PROTECTED TEST: Code-Labyrinth normal arbitrary-root contract,
// design_final.md §28. Written before implementation (TDD RED).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <optional>
#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_target_case()
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();
  return ins;
}

std::optional<PhysConfig> replay_from(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  PhysConfig state = root;
  for (const auto& ops : plan) {
    const auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    state = *next;
  }
  return state;
}

std::vector<PhysConfig> replay_states_from(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  std::vector<PhysConfig> states{root};
  for (const auto& ops : plan) {
    const auto next = apply_ops(ins, states.back(), ops);
    if (!next.has_value()) return {};
    states.push_back(*next);
  }
  return states;
}

::testing::AssertionResult valid_goal_plan_from(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  const auto state = replay_from(ins, root, plan);
  if (!state.has_value())
    return ::testing::AssertionFailure()
           << "plan is not legal from the supplied physical root";
  if (!is_dd_goal(ins, *state))
    return ::testing::AssertionFailure()
           << "plan does not reach a target goal from the supplied root";
  return ::testing::AssertionSuccess();
}

PhysConfig carried_target_root(const DDInstance& ins)
{
  auto state = initial_phys_config(ins);
  state = *apply_ops(
      ins, state,
      {Op::make_move(ins.grid.idx(0, 1))});
  state = *apply_ops(ins, state, {Op::make_lift()});
  state = *apply_ops(
      ins, state,
      {Op::make_move(ins.grid.idx(0, 2))});
  return state;
}

}  // namespace

TEST(dd_arbitrary_root,
     invalid_public_root_returns_invalid_without_search)
{
  const DDInstance ins = make_target_case();
  PhysConfig invalid = initial_phys_config(ins);
  invalid.robots[0] = ins.grid.size();

  DDStats stats;
  stats.hl_nodes = 99;
  stats.guidance_builds = 99;
  stats.first_solution_ms = 42;
  DDPlan best_effort{{Op::make_wait()}};
  const auto result = solve_carrier_lacam_from_state_result(
      ins, invalid, 2.0, 0, &stats, &best_effort);

  EXPECT_EQ(result.status, DDSolveStatus::INVALID);
  EXPECT_TRUE(result.plan.empty());
  EXPECT_TRUE(best_effort.empty());
  EXPECT_EQ(stats.hl_nodes, 0);
  EXPECT_EQ(stats.guidance_builds, 0);
  EXPECT_EQ(stats.first_solution_ms, -1);
  EXPECT_FALSE(stats.timed_out);
}

TEST(dd_arbitrary_root,
     tapf_planner_rejects_invalid_and_shelf_free_physical_roots)
{
  const DDInstance carrier = make_target_case();
  const TAPFInstance carrier_view(carrier);
  TAPFSearchConfig invalid_config;
  invalid_config.initial_physical =
      initial_phys_config(carrier);
  invalid_config.initial_physical->robots[0] =
      carrier.grid.size();
  std::mt19937 mt(0);
  EXPECT_THROW(
      TAPFPlanner(
          &carrier_view, nullptr, &mt, 0, 0, 0.001f, true,
          nullptr, invalid_config),
      std::invalid_argument);

  DDInstance shelf_free;
  shelf_free.grid = DDGrid({".."});
  shelf_free.robots = {shelf_free.grid.idx(0, 0)};
  shelf_free.finalize();
  const TAPFInstance shelf_free_view(shelf_free);
  TAPFSearchConfig shelf_free_config;
  shelf_free_config.initial_physical =
      initial_phys_config(shelf_free);
  EXPECT_THROW(
      TAPFPlanner(
          &shelf_free_view, nullptr, &mt, 0, 0, 0.001f, true,
          nullptr, shelf_free_config),
      std::invalid_argument);
}

TEST(dd_arbitrary_root,
     grounded_root_uses_normal_task_br_rho_and_pibt_path)
{
  const DDInstance ins = make_target_case();
  const auto root = apply_ops(
      ins, initial_phys_config(ins),
      {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(root.has_value());
  ASSERT_TRUE(validate_phys_config_root(ins, *root).valid());

  DDStats stats;
  const auto result = solve_carrier_lacam_from_state_result(
      ins, *root, 2.0, 0, &stats);
  ASSERT_EQ(result.status, DDSolveStatus::SOLVED);
  EXPECT_TRUE(valid_goal_plan_from(ins, *root, result.plan));

  EXPECT_GT(stats.guidance_builds, 0);
  EXPECT_GT(stats.pair_cache_misses, 0);
  EXPECT_GT(stats.joint_task_nodes, 0);
  EXPECT_GT(
      stats.rho_match_calls_execute +
          stats.rho_match_calls_prepare,
      0);
  EXPECT_GT(stats.pibt_calls, 0);
}

TEST(dd_arbitrary_root,
     carried_target_root_is_shared_by_both_search_passes_and_finalization)
{
  const DDInstance ins = make_target_case();
  const PhysConfig root = carried_target_root(ins);
  ASSERT_TRUE(validate_phys_config_root(ins, root).valid());
  ASSERT_EQ(root.kappa[0], 0);

  DDStats stats;
  const auto result = solve_carrier_lacam_from_state_result(
      ins, root, 2.0, 0, &stats);
  ASSERT_EQ(result.status, DDSolveStatus::SOLVED);
  EXPECT_TRUE(valid_goal_plan_from(ins, root, result.plan));
  EXPECT_FALSE(
      valid_goal_plan_from(
          ins, initial_phys_config(ins), result.plan));

  EXPECT_EQ(stats.improvement_attempts, 1);
  EXPECT_GT(stats.reference_checkpoint_hits, 0);
  EXPECT_GE(stats.first_solution_ms, 0);
}

TEST(dd_arbitrary_root, carried_anonymous_shelf_root_is_supported)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {
      ins.grid.idx(1, 0), ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(1, 0), ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();
  const auto root = apply_ops(
      ins, initial_phys_config(ins),
      {Op::make_lift(), Op::make_wait()});
  ASSERT_TRUE(root.has_value());
  ASSERT_EQ(root->kappa[0], KAPPA_ANON);
  ASSERT_TRUE(validate_phys_config_root(ins, *root).valid());

  const auto result = solve_carrier_lacam_from_state_result(
      ins, *root, 2.0, 0);
  ASSERT_EQ(result.status, DDSolveStatus::SOLVED);
  EXPECT_TRUE(valid_goal_plan_from(ins, *root, result.plan));
}

TEST(dd_arbitrary_root,
     normalize_cost_replay_reference_fixed_goal_and_repair_use_root)
{
  const DDInstance ins = make_target_case();
  const PhysConfig root = carried_target_root(ins);
  const DDPlan goal_then_wait = {
      {Op::make_move(ins.grid.idx(0, 3))},
      {Op::make_drop()},
      {Op::make_wait()},
  };

  const auto normalized =
      dd_normalize_goal_prefix_probe(
          ins, root, goal_then_wait);
  ASSERT_TRUE(normalized.has_value());
  ASSERT_EQ(normalized->size(), 2u);
  EXPECT_TRUE(valid_goal_plan_from(ins, root, *normalized));
  EXPECT_FALSE(
      dd_normalize_goal_prefix_probe(
          ins, goal_then_wait)
          .has_value());

  const PlanCost cost =
      dd_plan_cost_probe(ins, root, goal_then_wait);
  EXPECT_TRUE(cost.is_bounded());
  EXPECT_EQ(cost.ticks, 2);
  EXPECT_FALSE(
      dd_plan_cost_probe(ins, goal_then_wait).is_bounded());

  const auto replayed = dd_replay_raw_prefix_probe(
      ins, root, goal_then_wait);
  ASSERT_TRUE(replayed.has_value());
  EXPECT_TRUE(is_dd_goal(ins, replayed->first));
  EXPECT_EQ(replayed->second.ticks, 3);

  const auto reference = dd_build_reference_plan_probe(
      ins, root, *normalized, 16);
  ASSERT_TRUE(reference.has_value());
  ASSERT_FALSE(reference->checkpoints.empty());
  EXPECT_EQ(reference->checkpoints.front().state, root);
  EXPECT_EQ(reference->checkpoints.front().suffix_cost.ticks, 2);

  const auto fixed = dd_fixed_goal_instance_from_plan_probe(
      ins, root, *normalized);
  ASSERT_TRUE(fixed.has_value());
  ASSERT_EQ(fixed->target_goal_sets.size(), 1u);
  EXPECT_EQ(
      fixed->target_goal_sets[0],
      (std::vector<int>{ins.grid.idx(0, 3)}));

  const DDPlan looping = {
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_move(ins.grid.idx(0, 2))},
      {Op::make_move(ins.grid.idx(0, 3))},
      {Op::make_drop()},
  };
  ASSERT_TRUE(valid_goal_plan_from(ins, root, looping));
  DDPlanRepairStats repair_stats;
  const auto repaired = repair_carrier_plan(
      ins, root, looping, &repair_stats);
  EXPECT_TRUE(valid_goal_plan_from(ins, root, repaired));
  EXPECT_EQ(repaired.size(), 2u);
  EXPECT_EQ(repair_stats.exact_loops, 1);

  const auto states = replay_states_from(ins, root, looping);
  ASSERT_EQ(states.size(), looping.size() + 1);
  const auto replay_repaired = repair_carrier_plan_from_replay(
      ins, root, looping, states);
  EXPECT_EQ(replay_repaired, repaired);
}

TEST(dd_arbitrary_root, old_entry_equals_new_entry_at_instance_root)
{
  const DDInstance ins = make_target_case();
  DDStats old_stats;
  DDStats new_stats;
  const auto old_result =
      solve_carrier_lacam_result(ins, 2.0, 0, &old_stats);
  const auto new_result =
      solve_carrier_lacam_from_state_result(
          ins, initial_phys_config(ins), 2.0, 0, &new_stats);

  EXPECT_EQ(old_result.status, new_result.status);
  EXPECT_EQ(old_result.plan, new_result.plan);
  EXPECT_EQ(old_stats.hl_nodes, new_stats.hl_nodes);
  EXPECT_EQ(old_stats.hl_expanded, new_stats.hl_expanded);
  EXPECT_EQ(old_stats.pibt_calls, new_stats.pibt_calls);
  EXPECT_EQ(
      old_stats.pair_cache_misses,
      new_stats.pair_cache_misses);
  EXPECT_EQ(
      old_stats.rho_match_calls_execute,
      new_stats.rho_match_calls_execute);
  EXPECT_EQ(
      old_stats.guidance_builds,
      new_stats.guidance_builds);
  EXPECT_EQ(
      old_stats.first_solution_makespan,
      new_stats.first_solution_makespan);
  EXPECT_EQ(old_stats.best_makespan, new_stats.best_makespan);
  EXPECT_EQ(
      old_stats.best_work_scaled,
      new_stats.best_work_scaled);
  EXPECT_EQ(
      old_stats.improvement_attempts,
      new_stats.improvement_attempts);
}
