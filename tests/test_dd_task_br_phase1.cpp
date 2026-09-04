// PROTECTED Task-BR-PIBT Phase 1 tests.
// Written before implementation on 2026-09-03 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include "gtest/gtest.h"

namespace {

DDInstance phase1_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goal_sets = {
      {ins.grid.idx(0, 0), ins.grid.idx(0, 4)},
  };
  ins.target_goals = {ins.target_goal_sets[0].front()};
  ins.finalize();
  return ins;
}

void expect_same_pair_plan(const PairPlan& a, const PairPlan& b)
{
  EXPECT_DOUBLE_EQ(a.estimated_cost, b.estimated_cost);
  EXPECT_EQ(a.rollout_steps, b.rollout_steps);
  EXPECT_EQ(a.direct_distance, b.direct_distance);
  EXPECT_EQ(a.reached_goal, b.reached_goal);
  EXPECT_EQ(a.truncated, b.truncated);
  EXPECT_EQ(a.stalled, b.stalled);
}

void expect_same_task_graph(const ShelfTaskGraph& a,
                            const ShelfTaskGraph& b)
{
  ASSERT_EQ(a.tasks.size(), b.tasks.size());
  for (size_t i = 0; i < a.tasks.size(); ++i) {
    EXPECT_EQ(a.tasks[i].id, b.tasks[i].id);
    EXPECT_EQ(a.tasks[i].roots, b.tasks[i].roots);
    EXPECT_EQ(a.tasks[i].priority, b.tasks[i].priority);
  }
  EXPECT_EQ(a.predecessors, b.predecessors);
  EXPECT_EQ(a.successors, b.successors);
  EXPECT_EQ(a.paused_roots, b.paused_roots);
  EXPECT_EQ(a.effect_conflicts, b.effect_conflicts);
  EXPECT_EQ(a.candidate_backtracks, b.candidate_backtracks);
  ASSERT_EQ(a.rotations.size(), b.rotations.size());
  for (size_t i = 0; i < a.rotations.size(); ++i)
    EXPECT_EQ(a.rotations[i].cycle, b.rotations[i].cycle);
}

void expect_same_upper_epoch(const UpperEpochGuidance& a,
                             const UpperEpochGuidance& b)
{
  EXPECT_EQ(a.upper_signature, b.upper_signature);
  EXPECT_EQ(a.tau_guide, b.tau_guide);
  EXPECT_EQ(a.target_priority, b.target_priority);
  ASSERT_EQ(a.pair_cost.size(), b.pair_cost.size());
  for (size_t target = 0; target < a.pair_cost.size(); ++target) {
    ASSERT_EQ(a.pair_cost[target].size(), b.pair_cost[target].size());
    for (size_t option = 0; option < a.pair_cost[target].size();
         ++option) {
      EXPECT_EQ(a.pair_cost[target][option].goal,
                b.pair_cost[target][option].goal);
      expect_same_pair_plan(a.pair_cost[target][option].plan,
                            b.pair_cost[target][option].plan);
    }
  }
  expect_same_task_graph(a.task_graph, b.task_graph);
}

}  // namespace

TEST(dd_task_br_phase1, upper_signature_tracks_only_shelf_coordinates)
{
  const auto ins = phase1_instance();
  const auto X0 = initial_phys_config(ins);
  const auto U0 = dd_upper_signature_probe(X0);

  auto robot_only = X0;
  robot_only.robots[0] = ins.grid.idx(1, 4);
  EXPECT_EQ(dd_upper_signature_probe(robot_only), U0);

  const auto lifted = apply_ops(ins, X0, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  EXPECT_EQ(dd_upper_signature_probe(*lifted), U0);

  const auto dropped = apply_ops(ins, *lifted, {Op::make_drop()});
  ASSERT_TRUE(dropped.has_value());
  EXPECT_EQ(dd_upper_signature_probe(*dropped), U0);

  const auto moved =
      apply_ops(ins, *lifted, {Op::make_move(ins.grid.idx(0, 0))});
  ASSERT_TRUE(moved.has_value());
  EXPECT_NE(dd_upper_signature_probe(*moved), U0);
}

TEST(dd_task_br_phase1, anonymous_lift_and_drop_preserve_upper_signature)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.finalize();

  const auto X0 = initial_phys_config(ins);
  const auto U0 = dd_upper_signature_probe(X0);
  const auto lifted = apply_ops(ins, X0, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  EXPECT_EQ(dd_upper_signature_probe(*lifted), U0);
  const auto dropped = apply_ops(ins, *lifted, {Op::make_drop()});
  ASSERT_TRUE(dropped.has_value());
  EXPECT_EQ(dd_upper_signature_probe(*dropped), U0);
}

TEST(dd_task_br_phase1, pair_cost_and_tau_are_robot_position_invariant)
{
  const auto ins = phase1_instance();
  auto Xa = initial_phys_config(ins);
  auto Xb = Xa;
  Xb.robots[0] = ins.grid.idx(1, 4);

  ASSERT_EQ(dd_upper_signature_probe(Xa), dd_upper_signature_probe(Xb));
  for (const int goal : ins.target_goal_sets[0]) {
    expect_same_pair_plan(dd_pair_cost_probe(ins, Xa, 0, goal),
                          dd_pair_cost_probe(ins, Xb, 0, goal));
  }
  EXPECT_EQ(dd_tau_guide_probe(ins, Xa), dd_tau_guide_probe(ins, Xb));
}

TEST(dd_task_br_phase1, lift_changes_tau_lb_but_not_pair_guidance)
{
  const auto ins = phase1_instance();
  const auto grounded = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, grounded);
  const std::vector<Op> lift_ops = {Op::make_lift()};
  const auto lifted = apply_ops(ins, grounded, lift_ops);
  ASSERT_TRUE(lifted.has_value());
  const auto G1 =
      dd_task_br_guidance_probe(ins, *lifted, &grounded, &G0, &lift_ops);
  const std::vector<Op> drop_ops = {Op::make_drop()};
  const auto dropped = apply_ops(ins, *lifted, drop_ops);
  ASSERT_TRUE(dropped.has_value());
  const auto G2 =
      dd_task_br_guidance_probe(ins, *dropped, &*lifted, &G1, &drop_ops);

  EXPECT_EQ(dd_upper_signature_probe(grounded),
            dd_upper_signature_probe(*lifted));
  for (const int goal : ins.target_goal_sets[0]) {
    expect_same_pair_plan(dd_pair_cost_probe(ins, grounded, 0, goal),
                          dd_pair_cost_probe(ins, *lifted, 0, goal));
  }
  EXPECT_EQ(dd_tau_guide_probe(ins, grounded),
            dd_tau_guide_probe(ins, *lifted));
  EXPECT_NE(dd_tau_lb_probe(ins, grounded), dd_tau_lb_probe(ins, *lifted));
  ASSERT_NE(G0.upper_epoch, nullptr);
  ASSERT_NE(G1.upper_epoch, nullptr);
  ASSERT_NE(G2.upper_epoch, nullptr);
  EXPECT_EQ(G0.upper_epoch.get(), G1.upper_epoch.get());
  EXPECT_EQ(G1.upper_epoch.get(), G2.upper_epoch.get());
  expect_same_upper_epoch(*G0.upper_epoch, *G1.upper_epoch);
  expect_same_upper_epoch(*G1.upper_epoch, *G2.upper_epoch);
}

TEST(dd_task_br_phase1, loaded_move_invalidates_the_upper_epoch)
{
  const auto ins = phase1_instance();
  const auto grounded = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, grounded);
  const std::vector<Op> lift_ops = {Op::make_lift()};
  const auto lifted = apply_ops(ins, grounded, lift_ops);
  ASSERT_TRUE(lifted.has_value());
  const auto G1 =
      dd_task_br_guidance_probe(ins, *lifted, &grounded, &G0, &lift_ops);
  const std::vector<Op> move_ops = {
      Op::make_move(ins.grid.idx(0, 0))};
  const auto moved = apply_ops(ins, *lifted, move_ops);
  ASSERT_TRUE(moved.has_value());
  const auto G2 =
      dd_task_br_guidance_probe(ins, *moved, &*lifted, &G1, &move_ops);

  EXPECT_NE(dd_upper_signature_probe(*lifted),
            dd_upper_signature_probe(*moved));
  const auto before =
      dd_pair_cost_probe(ins, *lifted, 0, ins.grid.idx(0, 4));
  const auto after =
      dd_pair_cost_probe(ins, *moved, 0, ins.grid.idx(0, 4));
  EXPECT_NE(before.estimated_cost, after.estimated_cost);
  ASSERT_NE(G1.upper_epoch, nullptr);
  ASSERT_NE(G2.upper_epoch, nullptr);
  EXPECT_NE(G1.upper_epoch.get(), G2.upper_epoch.get());
  EXPECT_NE(G1.upper_epoch->upper_signature,
            G2.upper_epoch->upper_signature);
}

TEST(dd_task_br_phase1, anonymous_input_order_is_irrelevant)
{
  const auto ins = phase1_instance();
  auto a = initial_phys_config(ins);
  auto b = a;
  std::reverse(b.anon_occ.begin(), b.anon_occ.end());
  EXPECT_EQ(dd_upper_signature_probe(a), dd_upper_signature_probe(b));
  for (const int goal : ins.target_goal_sets[0])
    expect_same_pair_plan(dd_pair_cost_probe(ins, a, 0, goal),
                          dd_pair_cost_probe(ins, b, 0, goal));
  EXPECT_EQ(dd_tau_guide_probe(ins, a), dd_tau_guide_probe(ins, b));
}

TEST(dd_task_br_phase1,
     consecutive_anonymous_shifts_share_one_lift_drop_episode)
{
  const ShelfSelector anon{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 7};
  const ShelfSelector other_anon{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 9};
  constexpr double alpha = 2;
  constexpr double gamma = 3;
  constexpr double delta = 5;

  EXPECT_DOUBLE_EQ(
      dd_pair_episode_cost_probe(
          {anon, anon}, alpha, gamma, delta),
      2 * alpha + 2 * gamma + 2 * delta);
  EXPECT_DOUBLE_EQ(
      dd_pair_episode_cost_probe(
          {anon, other_anon}, alpha, gamma, delta),
      2 * alpha + 4 * gamma + 2 * delta);
}

TEST(dd_task_br_phase1, upper_epoch_cache_is_history_independent)
{
  const auto ins = phase1_instance();
  const auto query = initial_phys_config(ins);
  auto robot_only = query;
  robot_only.robots[0] = ins.grid.idx(1, 4);
  auto other_upper = query;
  other_upper.target_pos[0] = ins.grid.idx(1, 1);

  long hits_a = 0;
  const auto warm_a = dd_task_br_cached_guidance_probe(
      ins, query, {other_upper, robot_only}, &hits_a);
  long hits_b = 0;
  const auto warm_b = dd_task_br_cached_guidance_probe(
      ins, query, {robot_only, other_upper}, &hits_b);
  const auto fresh = dd_task_br_guidance_probe(ins, query);

  ASSERT_NE(warm_a.upper_epoch, nullptr);
  ASSERT_NE(warm_b.upper_epoch, nullptr);
  ASSERT_NE(fresh.upper_epoch, nullptr);
  EXPECT_GT(hits_a, 0);
  EXPECT_GT(hits_b, 0);
  expect_same_upper_epoch(*warm_a.upper_epoch, *fresh.upper_epoch);
  expect_same_upper_epoch(*warm_b.upper_epoch, *fresh.upper_epoch);
}

TEST(dd_task_br_phase1, tau_guide_is_globally_injective)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", ".....", "....."});
  ins.robots = {ins.grid.idx(2, 0)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  ins.target_starts = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  const int shared = ins.grid.idx(0, 1);
  const int alternate = ins.grid.idx(2, 4);
  ins.target_goal_sets = {
      {shared, alternate},
      {shared},
  };
  ins.target_goals = {shared, shared};
  ins.finalize();

  const auto tau =
      dd_tau_guide_probe(ins, initial_phys_config(ins));
  ASSERT_EQ(tau.size(), 2u);
  EXPECT_EQ(tau[0], alternate);
  EXPECT_EQ(tau[1], shared);
  EXPECT_NE(tau[0], tau[1]);
}

TEST(dd_task_br_phase1,
     settled_eligible_goal_is_kept_when_injective_matching_allows_it)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  ins.target_starts = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  const int settled = ins.grid.idx(0, 0);
  const int alternate = ins.grid.idx(0, 4);
  ins.target_goal_sets = {
      {settled, alternate},
      {settled, alternate},
  };
  ins.target_goals = {settled, alternate};
  ins.finalize();

  const auto tau =
      dd_tau_guide_probe(ins, initial_phys_config(ins));
  ASSERT_EQ(tau.size(), 2u);
  EXPECT_EQ(tau[0], settled);
  EXPECT_EQ(tau[1], alternate);
}

TEST(dd_task_br_phase1,
     settled_eligible_goal_reopens_when_global_matching_requires_it)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  ins.target_starts = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  const int settled = ins.grid.idx(0, 0);
  const int alternate = ins.grid.idx(0, 4);
  ins.target_goal_sets = {
      {settled, alternate},
      {settled},
  };
  ins.target_goals = {settled, settled};
  ins.finalize();

  const auto tau =
      dd_tau_guide_probe(ins, initial_phys_config(ins));
  ASSERT_EQ(tau.size(), 2u);
  EXPECT_EQ(tau[0], alternate);
  EXPECT_EQ(tau[1], settled);
}
