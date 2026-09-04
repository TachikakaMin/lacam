// PROTECTED Task-BR-PIBT production attach / Carrier-PIBT tests.
// Written before the production-path migration on 2026-09-03.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <memory>
#include <numeric>
#include <random>

#include "gtest/gtest.h"

namespace {

DDInstance line_instance(int width, const std::vector<int>& robots,
                         const std::vector<int>& shelves,
                         const std::vector<int>& target_starts,
                         const std::vector<std::vector<int>>& goal_sets)
{
  DDInstance ins;
  ins.grid = DDGrid({std::string(width, '.')});
  for (const int c : robots) ins.robots.push_back(ins.grid.idx(0, c));
  for (const int c : shelves) ins.shelves.push_back(ins.grid.idx(0, c));
  for (size_t b = 0; b < target_starts.size(); ++b) {
    ins.target_starts.push_back(ins.grid.idx(0, target_starts[b]));
    std::vector<int> goals;
    for (const int c : goal_sets[b])
      goals.push_back(ins.grid.idx(0, c));
    ins.target_goal_sets.push_back(goals);
    ins.target_goals.push_back(goals.front());
  }
  ins.finalize();
  return ins;
}

std::unique_ptr<TAPFNode> node_for(TAPFPlanner& planner,
                                   const TAPFInstance& view,
                                   const PhysConfig& X)
{
  Config config;
  for (const int cell : X.robots) config.push_back(view.G.U[cell]);
  ShelfState shelf;
  shelf.target_pos = X.target_pos;
  shelf.anon_occ = X.anon_occ;
  shelf.kappa = X.kappa;
  auto node = std::make_unique<TAPFNode>(
      config, shelf, planner.D, &view,
      std::vector<int>(view.N, -1), TAPFAssignmentState(), nullptr);
  node->h = 0;
  node->f = 0;
  return node;
}

std::vector<Op> preferred_ops(TAPFPlanner& planner,
                              const TAPFInstance& view,
                              const PhysConfig& X,
                              const CarrierGuidance& guidance)
{
  auto node = node_for(planner, view, X);
  node->guide = std::make_unique<CarrierGuidance>(guidance);
  node->order.resize(view.N);
  std::iota(node->order.begin(), node->order.end(), 0);
  node->constraint_order = node->order;
  planner.invalidate_carrier_scratch();
  TAPFConstraint root;
  EXPECT_TRUE(planner.get_new_config(node.get(), &root));
  EXPECT_TRUE(planner.apply_carrier_effects(node.get()));
  return planner.ops_scratch;
}

}  // namespace

TEST(dd_task_br_production, attach_uses_task_br_ready_only_guidance)
{
  const auto ins = line_instance(5, {3}, {0, 1, 2, 3}, {0}, {{4}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  auto node = node_for(planner, view, initial_phys_config(ins));

  planner.attach_carrier_guidance(node.get());

  ASSERT_NE(node->guide, nullptr);
  ASSERT_NE(node->guide->upper_epoch, nullptr);
  ASSERT_EQ(node->guide->upper_epoch->task_graph.tasks.size(), 4u);
  ASSERT_EQ(node->guide->ready_tasks.size(), 1u);
  ASSERT_EQ(node->guide->rho_task_id.size(), 1u);
  ASSERT_TRUE(node->guide->rho_task_id[0].has_value());
  const int ready = node->guide->ready_tasks[0];
  EXPECT_EQ(*node->guide->rho_task_id[0],
            node->guide->upper_epoch->task_graph.tasks[ready].id);
  EXPECT_EQ(node->guide->rho_ready_index[0], ready);
}

TEST(dd_task_br_production,
     attach_rejects_missing_or_nonreplayable_custody_anchor)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto X0 = initial_phys_config(ins);
  auto root = node_for(planner, view, X0);
  planner.attach_carrier_guidance(root.get());
  ASSERT_TRUE(root->guide->rho_task_id[0].has_value());

  const std::vector<Op> lift = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift);
  ASSERT_TRUE(X1.has_value());
  auto missing = node_for(planner, view, *X1);
  planner.attach_carrier_guidance(missing.get());
  ASSERT_EQ(missing->guide->custody_by_robot.size(), 1u);
  EXPECT_FALSE(missing->guide->custody_by_robot[0].has_value());

  const std::vector<Op> wrong = {Op::make_wait()};
  const auto wrong_next = apply_ops(ins, X0, wrong);
  ASSERT_TRUE(wrong_next.has_value());
  EXPECT_FALSE(*wrong_next == *X1);
  auto invalid = node_for(planner, view, *X1);
  planner.attach_carrier_guidance(
      invalid.get(), &X0, root->guide.get(), &wrong);
  ASSERT_EQ(invalid->guide->custody_by_robot.size(), 1u);
  EXPECT_FALSE(invalid->guide->custody_by_robot[0].has_value());
}

TEST(dd_task_br_production,
     carrier_pibt_executes_adjacent_tasks_and_drops_when_unbound)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);

  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  const auto O0 = preferred_ops(planner, view, X0, G0);
  ASSERT_EQ(O0.size(), 1u);
  EXPECT_EQ(O0[0], Op::make_lift());

  const auto X1 = apply_ops(ins, X0, O0);
  ASSERT_TRUE(X1.has_value());
  const auto G1 = dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &O0);
  ASSERT_TRUE(G1.custody_by_robot[0].has_value());
  const auto O1 = preferred_ops(planner, view, *X1, G1);
  ASSERT_EQ(O1.size(), 1u);
  EXPECT_EQ(O1[0], Op::make_move(ins.grid.idx(0, 1)));

  const auto X2 = apply_ops(ins, *X1, O1);
  ASSERT_TRUE(X2.has_value());
  const auto G2 = dd_task_br_guidance_probe(ins, *X2, &*X1, &G1, &O1);
  ASSERT_TRUE(G2.custody_by_robot[0].has_value());
  const auto O2 = preferred_ops(planner, view, *X2, G2);
  ASSERT_EQ(O2.size(), 1u);
  EXPECT_EQ(O2[0], Op::make_move(ins.grid.idx(0, 2)));

  const auto X3 = apply_ops(ins, *X2, O2);
  ASSERT_TRUE(X3.has_value());
  const auto G3 = dd_task_br_guidance_probe(ins, *X3, &*X2, &G2, &O2);
  EXPECT_FALSE(G3.custody_by_robot[0].has_value());
  const auto O3 = preferred_ops(planner, view, *X3, G3);
  ASSERT_EQ(O3.size(), 1u);
  EXPECT_EQ(O3[0], Op::make_drop());
}

TEST(dd_task_br_production, forced_unassigned_lift_prefers_drop_unbound)
{
  const auto ins = line_instance(3, {0}, {0, 1}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  const std::vector<Op> forced = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, forced);
  ASSERT_TRUE(X1.has_value());
  const auto G1 =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &forced);
  ASSERT_FALSE(G1.custody_by_robot[0].has_value());

  const auto preferred = preferred_ops(planner, view, *X1, G1);
  ASSERT_EQ(preferred.size(), 1u);
  EXPECT_EQ(preferred[0], Op::make_drop());
}

TEST(dd_task_br_production,
     idle_robot_on_current_task_footprint_steps_off_before_waiting)
{
  const auto ins = line_instance(3, {0, 1}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto X = initial_phys_config(ins);
  const auto guidance = dd_task_br_guidance_probe(ins, X);
  ASSERT_TRUE(guidance.rho_task_id[0].has_value());
  ASSERT_FALSE(guidance.rho_task_id[1].has_value());

  const auto ops = preferred_ops(planner, view, X, guidance);
  ASSERT_EQ(ops.size(), 2u);
  EXPECT_EQ(ops[0], Op::make_lift());
  EXPECT_EQ(ops[1], Op::make_move(ins.grid.idx(0, 2)));
}

TEST(dd_task_br_production,
     idle_robot_outside_current_footprint_keeps_wait_first)
{
  const auto ins = line_instance(5, {0, 4}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto X = initial_phys_config(ins);
  const auto guidance = dd_task_br_guidance_probe(ins, X);
  ASSERT_TRUE(guidance.rho_task_id[0].has_value());
  ASSERT_FALSE(guidance.rho_task_id[1].has_value());

  const auto ops = preferred_ops(planner, view, X, guidance);
  ASSERT_EQ(ops.size(), 2u);
  EXPECT_EQ(ops[0], Op::make_lift());
  EXPECT_EQ(ops[1], Op::make_wait());
}

TEST(dd_task_br_production,
     ready_lift_preference_is_independent_of_prior_lift_drop_episodes)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto initial = initial_phys_config(ins);
  auto current = initial;

  for (int episode = 0; episode < 4; ++episode) {
    const auto ready = dd_task_br_guidance_probe(ins, current);
    const auto lift = preferred_ops(planner, view, current, ready);
    ASSERT_EQ(lift.size(), 1u);
    EXPECT_EQ(lift[0], Op::make_lift()) << "episode " << episode;
    const auto loaded = apply_ops(ins, current, lift);
    ASSERT_TRUE(loaded.has_value());

    const auto unbound = dd_task_br_guidance_probe(ins, *loaded);
    const auto drop = preferred_ops(planner, view, *loaded, unbound);
    ASSERT_EQ(drop.size(), 1u);
    EXPECT_EQ(drop[0], Op::make_drop()) << "episode " << episode;
    const auto grounded = apply_ops(ins, *loaded, drop);
    ASSERT_TRUE(grounded.has_value());
    EXPECT_EQ(*grounded, initial);
    current = *grounded;
  }
}

TEST(dd_task_br_production,
     mixed_heuristic_adds_tapf_and_shelf_lb_exactly_once)
{
  TAPFInstance mixed(
      "./assets/empty-8-8.map", std::vector<int>{0},
      std::vector<std::vector<int>>{{63}});
  mixed.shelf_cells = {1};
  mixed.target_starts = {1};
  mixed.target_goals = {2};
  mixed.target_goal_sets = {{2}};
  ASSERT_TRUE(mixed.is_valid());

  DDInstance shelf_view;
  shelf_view.grid =
      DDGrid(std::vector<std::string>(8, std::string(8, '.')));
  shelf_view.robots = {0};
  shelf_view.shelves = {1};
  shelf_view.target_starts = {1};
  shelf_view.target_goals = {2};
  shelf_view.target_goal_sets = {{2}};
  shelf_view.finalize();

  std::mt19937 mt(0);
  TAPFPlanner planner(&mixed, nullptr, &mt);
  TAPFAssignmentState assignment_state;
  assignment_state.init(mixed.N, mixed.tasks.size());
  auto node = std::make_unique<TAPFNode>(
      mixed.starts, initial_shelf_state(mixed), planner.D, &mixed,
      std::vector<int>{0}, assignment_state, nullptr);
  const double tapf_h = planner.D.get(0, mixed.starts[0]);
  node->h = tapf_h;
  node->f = node->g + node->h;
  const double shelf_lb =
      dd_tau_lb_probe(shelf_view, initial_phys_config(shelf_view));
  const auto weights = dd_load_soc_weights();
  EXPECT_LE(shelf_lb, weights.alpha + 2 * weights.gamma);

  planner.attach_carrier_guidance(node.get());
  EXPECT_DOUBLE_EQ(node->h, tapf_h + shelf_lb);
  EXPECT_DOUBLE_EQ(node->f, node->g + node->h);
  const double attached_h = node->h;
  const double attached_f = node->f;
  planner.attach_carrier_guidance(node.get());
  EXPECT_DOUBLE_EQ(node->h, attached_h);
  EXPECT_DOUBLE_EQ(node->f, attached_f);
}

TEST(dd_task_br_production,
     zero_shelf_attach_is_a_bitwise_noop_for_tapf_priority_state)
{
  const TAPFInstance view(
      "./assets/empty-8-8.map", std::vector<int>{0, 7},
      std::vector<std::vector<int>>{{63}, {56}});
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  TAPFAssignmentState assignment_state;
  assignment_state.init(view.N, view.tasks.size());
  auto node = std::make_unique<TAPFNode>(
      view.starts, initial_shelf_state(view), planner.D, &view,
      std::vector<int>{0, 1}, assignment_state, nullptr);
  node->h = planner.D.get(0, view.starts[0]) +
            planner.D.get(1, view.starts[1]);
  node->f = node->g + node->h;
  const auto order = node->order;
  const auto constraint_order = node->constraint_order;
  const double h = node->h;
  const double f = node->f;

  planner.attach_carrier_guidance(node.get());

  EXPECT_EQ(node->guide, nullptr);
  EXPECT_EQ(node->order, order);
  EXPECT_EQ(node->constraint_order, constraint_order);
  EXPECT_DOUBLE_EQ(node->h, h);
  EXPECT_DOUBLE_EQ(node->f, f);
}

TEST(dd_task_br_production,
     rollout_refreshes_every_step_and_returns_terminal_anchor)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFStats stats;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &stats);
  const auto rollout = planner.carrier_rollout(
      view.starts, initial_shelf_state(view), 8, 0, false);

  ASSERT_TRUE(rollout.reached_goal);
  ASSERT_EQ(rollout.configs.size(), rollout.ops.size() + 1);
  ASSERT_EQ(rollout.shelves.size(), rollout.ops.size() + 1);
  int loaded_moves = 0;
  for (size_t step = 0; step < rollout.ops.size(); ++step)
    for (size_t robot = 0; robot < rollout.ops[step].size(); ++robot)
      loaded_moves +=
          rollout.ops[step][robot].kind == Op::MOVE &&
          rollout.shelves[step].kappa[robot] != KAPPA_FREE;
  EXPECT_GE(loaded_moves, 2);
  EXPECT_GE(stats.guidance_builds,
            (long)rollout.ops.size() + 1)
      << "every rollout state, including the terminal anchor, must attach";

  ASSERT_NE(rollout.terminal_guidance, nullptr);
  ASSERT_NE(rollout.terminal_guidance->upper_epoch, nullptr);
  ASSERT_EQ(rollout.terminal_guidance->custody_by_robot.size(), 1u);
  EXPECT_FALSE(
      rollout.terminal_guidance->custody_by_robot[0].has_value());
  EXPECT_TRUE(rollout.terminal_guidance->ready_tasks.empty());
}

TEST(dd_task_br_production,
     event_bounded_rollout_resumes_from_exact_terminal_anchor)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);

  const auto lift_chunk = planner.carrier_rollout(
      view.starts, initial_shelf_state(view), 8, 0, true);
  ASSERT_FALSE(lift_chunk.reached_goal);
  ASSERT_EQ(lift_chunk.ops.size(), 1u);
  ASSERT_EQ(lift_chunk.ops.front().size(), 1u);
  EXPECT_EQ(lift_chunk.ops.front().front().kind, Op::LIFT);
  ASSERT_NE(lift_chunk.terminal_guidance, nullptr);
  ASSERT_TRUE(
      lift_chunk.terminal_guidance->custody_by_robot[0].has_value());

  auto anchor = std::make_unique<TAPFNode>(
      lift_chunk.configs.back(), lift_chunk.shelves.back(), planner.D,
      &view, std::vector<int>(view.N, -1),
      TAPFAssignmentState(), nullptr);
  anchor->guide = std::make_unique<CarrierGuidance>(
      *lift_chunk.terminal_guidance);
  anchor->order = lift_chunk.terminal_order;
  anchor->constraint_order = anchor->order;
  anchor->h = lift_chunk.terminal_h;
  anchor->h_guidance = lift_chunk.terminal_h_guidance;

  const auto delivery_chunk = planner.carrier_rollout(
      anchor->C, anchor->shelf, 8, 0, true, anchor.get());
  ASSERT_TRUE(delivery_chunk.reached_goal);
  EXPECT_TRUE(delivery_chunk.shelf_moved);
  ASSERT_FALSE(delivery_chunk.ops.empty());
  EXPECT_EQ(delivery_chunk.ops.front().front().kind, Op::MOVE);
  EXPECT_EQ(delivery_chunk.shelves.back().target_pos[0],
            ins.target_goal_sets[0][0]);
  EXPECT_EQ(delivery_chunk.shelves.back().kappa[0], KAPPA_FREE);
}

TEST(dd_task_br_production,
     rewrite_installs_the_selected_candidate_edge_trace_atomically)
{
  const auto ins = line_instance(2, {0}, {0}, {0}, {{1}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto X0 = initial_phys_config(ins);
  const std::vector<Op> lift = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift);
  ASSERT_TRUE(X1.has_value());
  const std::vector<Op> drop = {Op::make_drop()};
  const auto X0_again = apply_ops(ins, *X1, drop);
  ASSERT_TRUE(X0_again.has_value());
  const auto X1_again = apply_ops(ins, *X0_again, lift);
  ASSERT_TRUE(X1_again.has_value());
  ASSERT_EQ(*X1_again, *X1);

  auto old_parent = node_for(planner, view, X0);
  auto new_parent = node_for(planner, view, X0);
  auto child = node_for(planner, view, *X1);
  old_parent->g = 4;
  new_parent->g = 0;
  child->g = 9;
  child->h = 3;
  child->f = 12;

  const std::vector<TransitionStep> old_trace = {
      TransitionStep{X0, lift, *X1},
      TransitionStep{*X1, drop, *X0_again},
      TransitionStep{*X0_again, lift, *X1_again},
  };
  const std::vector<TransitionStep> new_trace = {
      TransitionStep{X0, lift, *X1},
  };
  const auto old_edge = planner.register_outgoing_edge(
      old_parent.get(), child.get(), 5, old_trace);
  child->parent = old_parent.get();
  child->incoming_edge = old_edge;
  const auto candidate = planner.register_outgoing_edge(
      new_parent.get(), child.get(), 1, new_trace);

  std::vector<TAPFNode*> open;
  planner.rewrite(new_parent.get(), nullptr, open);

  EXPECT_EQ(child->parent, new_parent.get());
  EXPECT_EQ(child->incoming_edge, candidate);
  ASSERT_NE(child->incoming_edge, nullptr);
  EXPECT_EQ(child->incoming_edge->transition_trace.size(), 1u);
  EXPECT_DOUBLE_EQ(child->g, 1);
  EXPECT_DOUBLE_EQ(child->f, child->g + child->h);
  EXPECT_TRUE(child->guidance_stale);
}

TEST(dd_task_br_production,
     stale_child_refreshes_parent_chain_without_touching_search_costs)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto X0 = initial_phys_config(ins);
  const std::vector<Op> lift = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift);
  ASSERT_TRUE(X1.has_value());
  const std::vector<Op> move = {Op::make_move(ins.grid.idx(0, 1))};
  const auto X2 = apply_ops(ins, *X1, move);
  ASSERT_TRUE(X2.has_value());

  auto root = node_for(planner, view, X0);
  planner.attach_carrier_guidance(root.get());

  auto parent = node_for(planner, view, *X1);
  parent->parent = root.get();
  parent->incoming_edge = planner.register_outgoing_edge(
      root.get(), parent.get(), 1,
      {TransitionStep{X0, lift, *X1}});
  planner.attach_carrier_guidance(
      parent.get(), &X0, root->guide.get(), &lift);
  ASSERT_TRUE(parent->guide->custody_by_robot[0].has_value());

  auto child = node_for(planner, view, *X2);
  child->parent = parent.get();
  child->incoming_edge = planner.register_outgoing_edge(
      parent.get(), child.get(), 1,
      {TransitionStep{*X1, move, *X2}});
  planner.attach_carrier_guidance(
      child.get(), &*X1, parent->guide.get(), &move);
  ASSERT_TRUE(child->guide->custody_by_robot[0].has_value());
  const TaskId expected_child =
      child->guide->custody_by_robot[0]->task_id;

  parent->guide->custody_by_robot.assign(1, std::nullopt);
  child->guide->custody_by_robot.assign(1, std::nullopt);
  parent->guidance_stale = true;
  child->guidance_stale = true;
  const double parent_g = parent->g;
  const double parent_h = parent->h;
  const double parent_f = parent->f;
  const double child_g = child->g;
  const double child_h = child->h;
  const double child_f = child->f;
  const auto parent_constraints = parent->constraint_order;
  const auto child_constraints = child->constraint_order;

  planner.ensure_guidance_fresh(child.get());

  EXPECT_FALSE(parent->guidance_stale);
  EXPECT_FALSE(child->guidance_stale);
  ASSERT_TRUE(parent->guide->custody_by_robot[0].has_value());
  ASSERT_TRUE(child->guide->custody_by_robot[0].has_value());
  EXPECT_EQ(child->guide->custody_by_robot[0]->task_id,
            expected_child);
  EXPECT_DOUBLE_EQ(parent->g, parent_g);
  EXPECT_DOUBLE_EQ(parent->h, parent_h);
  EXPECT_DOUBLE_EQ(parent->f, parent_f);
  EXPECT_DOUBLE_EQ(child->g, child_g);
  EXPECT_DOUBLE_EQ(child->h, child_h);
  EXPECT_DOUBLE_EQ(child->f, child_f);
  EXPECT_EQ(parent->constraint_order, parent_constraints);
  EXPECT_EQ(child->constraint_order, child_constraints);
}
