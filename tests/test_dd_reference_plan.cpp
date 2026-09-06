#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <algorithm>
#include <optional>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_detour_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"....", "...."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 2)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();
  return ins;
}

DDPlan detour_plan(const DDInstance& ins)
{
  return {
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_move(ins.grid.idx(1, 1))},
      {Op::make_move(ins.grid.idx(1, 2))},
      {Op::make_move(ins.grid.idx(0, 2))},
      {Op::make_lift()},
      {Op::make_move(ins.grid.idx(0, 3))},
      {Op::make_drop()},
  };
}

DDPlan direct_prefix_to_shelf(const DDInstance& ins)
{
  return {
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_move(ins.grid.idx(0, 2))},
  };
}

std::optional<PhysConfig> replay_prefix(
    const DDInstance& ins, const DDPlan& plan)
{
  PhysConfig state = initial_phys_config(ins);
  for (const auto& ops : plan) {
    const auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return std::nullopt;
    state = *next;
  }
  return state;
}

bool valid_goal_plan(const DDInstance& ins, const DDPlan& plan)
{
  const auto state = replay_prefix(ins, plan);
  return state.has_value() && is_dd_goal(ins, *state);
}

}  // namespace

TEST(dd_reference_plan, exact_full_state_match_exposes_suffix)
{
  const DDInstance ins = make_detour_instance();
  const DDPlan reference_actions = detour_plan(ins);
  const auto reference =
      dd_build_reference_plan_probe(ins, reference_actions, 32);
  ASSERT_TRUE(reference.has_value());

  const auto checkpoint_state =
      replay_prefix(ins, direct_prefix_to_shelf(ins));
  ASSERT_TRUE(checkpoint_state.has_value());
  const auto checkpoint =
      dd_reference_checkpoint_probe(*reference, *checkpoint_state);
  ASSERT_TRUE(checkpoint.has_value());
  EXPECT_EQ(checkpoint->action_index, 4u);
  EXPECT_EQ(checkpoint->prefix_cost.ticks, 4);
  EXPECT_EQ(checkpoint->suffix_cost.ticks, 3);
  ASSERT_EQ(checkpoint->next_ops.size(), 1u);
  EXPECT_EQ(checkpoint->next_ops[0].kind, Op::LIFT);
}

TEST(dd_reference_plan, upper_layout_match_is_not_enough)
{
  // Protected-test fixture correction APPROVE:
  // independent GPT-5.6 Sol / high review 2026-09-06.  The previous
  // mutations accidentally selected other real checkpoints on the
  // reference path.  These states are physically legal, preserve the
  // compared upper layout, and are absent from the reference.
  const DDInstance ins = make_detour_instance();
  const auto reference =
      dd_build_reference_plan_probe(ins, detour_plan(ins), 32);
  ASSERT_TRUE(reference.has_value());
  auto state = replay_prefix(ins, direct_prefix_to_shelf(ins));
  ASSERT_TRUE(state.has_value());

  state->robots[0] = ins.grid.idx(1, 3);
  EXPECT_FALSE(
      dd_reference_checkpoint_probe(*reference, *state).has_value());

  DDInstance anon_ins;
  anon_ins.grid = DDGrid({"....."});
  anon_ins.robots = {anon_ins.grid.idx(0, 0)};
  anon_ins.shelves = {
      anon_ins.grid.idx(0, 3), anon_ins.grid.idx(0, 1)};
  anon_ins.target_starts = {anon_ins.grid.idx(0, 3)};
  anon_ins.target_goals = {anon_ins.grid.idx(0, 4)};
  anon_ins.finalize();
  const DDPlan anon_reference_actions = {
      {Op::make_move(anon_ins.grid.idx(0, 1))},
      {Op::make_move(anon_ins.grid.idx(0, 2))},
      {Op::make_move(anon_ins.grid.idx(0, 3))},
      {Op::make_lift()},
      {Op::make_move(anon_ins.grid.idx(0, 4))},
      {Op::make_drop()},
  };
  const auto anon_reference = dd_build_reference_plan_probe(
      anon_ins, anon_reference_actions, 32);
  ASSERT_TRUE(anon_reference.has_value());
  state = replay_prefix(
      anon_ins, DDPlan{{Op::make_move(anon_ins.grid.idx(0, 1))}});
  ASSERT_TRUE(state.has_value());
  ASSERT_EQ(state->anon_occ.size(), 1u);
  state->anon_occ.clear();
  state->kappa[0] = KAPPA_ANON;
  EXPECT_FALSE(
      dd_reference_checkpoint_probe(*anon_reference, *state).has_value());
}

TEST(dd_reference_plan, hash_collision_still_checks_full_state)
{
  const DDInstance ins = make_detour_instance();
  const auto built =
      dd_build_reference_plan_probe(ins, detour_plan(ins), 32);
  ASSERT_TRUE(built.has_value());
  ASSERT_FALSE(built->checkpoints.empty());

  PhysConfig other = initial_phys_config(ins);
  other.robots[0] = ins.grid.idx(1, 3);
  TAPFReferencePlan collision;
  collision.actions = built->actions;
  auto wrong = built->checkpoints.front();
  wrong.state_hash = phys_config_hash(other);
  collision.checkpoints.push_back(std::move(wrong));

  EXPECT_FALSE(
      dd_reference_checkpoint_probe(collision, other).has_value());
}

TEST(dd_reference_plan, repeated_state_selects_cheapest_suffix)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();
  const DDPlan actions = {
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_move(ins.grid.idx(0, 0))},
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_lift()},
      {Op::make_move(ins.grid.idx(0, 2))},
      {Op::make_move(ins.grid.idx(0, 3))},
      {Op::make_drop()},
  };
  ASSERT_TRUE(valid_goal_plan(ins, actions));

  const auto reference =
      dd_build_reference_plan_probe(ins, actions, 32);
  ASSERT_TRUE(reference.has_value());
  const auto checkpoint = dd_reference_checkpoint_probe(
      *reference, initial_phys_config(ins));
  ASSERT_TRUE(checkpoint.has_value());
  EXPECT_EQ(checkpoint->action_index, 2u);
  EXPECT_EQ(checkpoint->suffix_cost.ticks, 5);
}

TEST(dd_reference_plan, splice_requires_a_strict_raw_improvement)
{
  const DDInstance ins = make_detour_instance();
  const DDPlan reference_actions = detour_plan(ins);
  const auto reference =
      dd_build_reference_plan_probe(ins, reference_actions, 32);
  ASSERT_TRUE(reference.has_value());
  const PlanCost incumbent =
      dd_plan_cost_probe(ins, reference_actions);
  ASSERT_TRUE(incumbent.is_bounded());

  const auto improved = dd_reference_splice_probe(
      ins, *reference, direct_prefix_to_shelf(ins), incumbent);
  ASSERT_TRUE(improved.has_value());
  EXPECT_TRUE(valid_goal_plan(ins, *improved));
  const PlanCost improved_cost = dd_plan_cost_probe(ins, *improved);
  EXPECT_LT(improved_cost, incumbent);
  EXPECT_EQ(improved_cost.ticks, 5);

  EXPECT_FALSE(
      dd_reference_splice_probe(
          ins, *reference, direct_prefix_to_shelf(ins), improved_cost)
          .has_value())
      << "a reference suffix is an upper bound candidate, not a reason "
         "to accept a tie";
}

TEST(dd_reference_plan, action_hint_only_reorders_existing_candidates)
{
  const DDInstance ins = make_detour_instance();
  const auto reference =
      dd_build_reference_plan_probe(ins, detour_plan(ins), 32);
  ASSERT_TRUE(reference.has_value());
  const TAPFInstance view(ins);

  TAPFSearchConfig config;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  config.stop_policy = TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT;
  config.incumbent_init =
      dd_plan_cost_probe(ins, detour_plan(ins));
  config.reference_plan = &*reference;
  std::mt19937 mt(0);
  TAPFPlanner planner(
      &view, nullptr, &mt, 0, 0, 0.001f, true, nullptr, config);
  TAPFAssignmentState assignment_state;
  assignment_state.init(view.N, view.tasks.size());
  TAPFNode root(
      view.starts, initial_shelf_state(view), planner.D, &view,
      std::vector<int>(view.N, -1), assignment_state);

  std::vector<OpCand> candidates;
  planner.build_op_candidates(&root, 0, candidates);
  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_EQ(candidates.front().kind, Op::MOVE);
  EXPECT_EQ(candidates.front().v->index, ins.grid.idx(0, 1));

  const auto contains = [&](uint8_t kind, int cell) {
    return std::any_of(
        candidates.begin(), candidates.end(), [&](const OpCand& item) {
          return item.kind == kind && item.v->index == cell;
        });
  };
  EXPECT_TRUE(contains(Op::MOVE, ins.grid.idx(0, 1)));
  EXPECT_TRUE(contains(Op::MOVE, ins.grid.idx(1, 0)));
  EXPECT_TRUE(contains(Op::WAIT, ins.grid.idx(0, 0)));
}

TEST(dd_reference_plan, planner_accepts_exact_state_suffix_as_one_macro_edge)
{
  const DDInstance ins = make_detour_instance();
  const DDPlan reference_actions = detour_plan(ins);
  const auto reference =
      dd_build_reference_plan_probe(ins, reference_actions, 32);
  ASSERT_TRUE(reference.has_value());
  const TAPFInstance view(ins);

  TAPFSearchConfig config;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  config.stop_policy = TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT;
  config.macro_enabled = false;
  config.incumbent_init =
      dd_plan_cost_probe(ins, reference_actions);
  config.reference_plan = &*reference;
  TAPFStats stats;
  std::mt19937 mt(0);
  Deadline deadline(2000);
  TAPFPlanner planner(
      &view, &deadline, &mt, 0, 0, 0.001f, true, &stats, config);

  const auto solution = planner.solve();
  ASSERT_FALSE(solution.empty());
  const DDPlan plan =
      derive_carrier_ops(view, solution, planner.solution_shelves);
  ASSERT_TRUE(valid_goal_plan(ins, plan));
  EXPECT_LT(dd_plan_cost_probe(ins, plan), config.incumbent_init);
  EXPECT_EQ(dd_plan_cost_probe(ins, plan).ticks, 5);
  EXPECT_GE(stats.reference_checkpoint_hits, 1);
  EXPECT_GE(stats.reference_suffix_attempts, 1);
  EXPECT_EQ(stats.reference_suffix_accepted, 1);
}

TEST(dd_reference_plan, invalid_reference_suffix_never_becomes_a_solution)
{
  const DDInstance ins = make_detour_instance();
  auto reference =
      dd_build_reference_plan_probe(ins, detour_plan(ins), 32);
  ASSERT_TRUE(reference.has_value());
  ASSERT_GT(reference->actions.size(), 4u);
  reference->actions[4] = {
      Op::make_move(ins.grid.idx(1, 3))};
  const TAPFInstance view(ins);

  TAPFSearchConfig config;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  config.stop_policy = TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT;
  config.macro_enabled = false;
  config.incumbent_init = PlanCost::from_values(20, 1000);
  config.reference_plan = &*reference;
  TAPFStats stats;
  std::mt19937 mt(0);
  Deadline deadline(2000);
  TAPFPlanner planner(
      &view, &deadline, &mt, 0, 0, 0.001f, true, &stats, config);

  const auto solution = planner.solve();
  ASSERT_FALSE(solution.empty());
  const DDPlan plan =
      derive_carrier_ops(view, solution, planner.solution_shelves);
  EXPECT_TRUE(valid_goal_plan(ins, plan));
  EXPECT_EQ(stats.reference_suffix_accepted, 0);
}
