// PROTECTED Task-BR-PIBT ready/rho/custody tests.
// Written before implementation on 2026-09-03 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <algorithm>
#include <numeric>
#include <set>

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

const TaskId& assigned_id(const CarrierGuidance& guide, int robot)
{
  EXPECT_TRUE(guide.rho_task_id[robot].has_value());
  return *guide.rho_task_id[robot];
}

void swap_task_graph_indices(ShelfTaskGraph& graph, int a, int b)
{
  ASSERT_GE(a, 0);
  ASSERT_GE(b, 0);
  ASSERT_LT(a, (int)graph.tasks.size());
  ASSERT_LT(b, (int)graph.tasks.size());
  const auto old = graph;
  std::vector<int> permutation(old.tasks.size());
  std::iota(permutation.begin(), permutation.end(), 0);
  std::swap(permutation[a], permutation[b]);
  for (size_t old_index = 0; old_index < old.tasks.size(); ++old_index) {
    const int new_index = permutation[old_index];
    graph.tasks[new_index] = old.tasks[old_index];
    graph.predecessors[new_index] = old.predecessors[old_index];
    graph.successors[new_index] = old.successors[old_index];
    for (int& predecessor : graph.predecessors[new_index])
      predecessor = permutation[predecessor];
    for (int& successor : graph.successors[new_index])
      successor = permutation[successor];
    std::sort(graph.predecessors[new_index].begin(),
              graph.predecessors[new_index].end());
    std::sort(graph.successors[new_index].begin(),
              graph.successors[new_index].end());
  }
}

}  // namespace

TEST(dd_task_br_execution, rho_sees_only_the_ready_leaf)
{
  const auto ins =
      line_instance(5, {3}, {0, 1, 2, 3}, {0}, {{4}});
  const auto X = initial_phys_config(ins);
  const auto guide = dd_task_br_guidance_probe(ins, X);

  ASSERT_TRUE(guide.upper_epoch != nullptr);
  ASSERT_EQ(guide.upper_epoch->task_graph.tasks.size(), 4u);
  ASSERT_EQ(guide.ready_tasks.size(), 1u);
  ASSERT_EQ(guide.rho_task_id.size(), 1u);
  ASSERT_TRUE(guide.rho_task_id[0].has_value());
  const auto& task =
      guide.upper_epoch->task_graph.tasks[guide.ready_tasks[0]];
  EXPECT_EQ(*guide.rho_task_id[0], task.id);
  EXPECT_EQ(task.id.from, ins.grid.idx(0, 3));
  EXPECT_EQ(task.id.to, ins.grid.idx(0, 4));
}

TEST(dd_task_br_execution,
     assigned_lift_wait_and_loaded_move_follow_three_stage_custody)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  const TaskId first = assigned_id(G0, 0);
  EXPECT_EQ(first.from, ins.grid.idx(0, 0));
  EXPECT_EQ(first.to, ins.grid.idx(0, 1));

  const std::vector<Op> lift_ops = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift_ops);
  ASSERT_TRUE(X1.has_value());
  const auto G1 =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &lift_ops);
  ASSERT_TRUE(G1.custody_by_robot[0].has_value());
  EXPECT_EQ(G1.custody_by_robot[0]->task_id, first);
  EXPECT_FALSE(G1.rho_task_id[0].has_value());

  const std::vector<Op> wait_ops = {Op::make_wait()};
  const auto Xwait = apply_ops(ins, *X1, wait_ops);
  ASSERT_TRUE(Xwait.has_value());
  const auto Gwait =
      dd_task_br_guidance_probe(ins, *Xwait, &*X1, &G1, &wait_ops);
  ASSERT_TRUE(Gwait.custody_by_robot[0].has_value());
  EXPECT_EQ(Gwait.custody_by_robot[0]->task_id, first);

  const std::vector<Op> move_ops = {
      Op::make_move(ins.grid.idx(0, 1))};
  const auto X2 = apply_ops(ins, *X1, move_ops);
  ASSERT_TRUE(X2.has_value());
  const auto G2 =
      dd_task_br_guidance_probe(ins, *X2, &*X1, &G1, &move_ops);
  ASSERT_TRUE(G2.custody_by_robot[0].has_value());
  const TaskId second = G2.custody_by_robot[0]->task_id;
  EXPECT_NE(second, first);
  EXPECT_EQ(second.from, ins.grid.idx(0, 1));
  EXPECT_EQ(second.to, ins.grid.idx(0, 2));
  EXPECT_FALSE(G2.rho_task_id[0].has_value());
}

TEST(dd_task_br_execution,
     carried_continuation_is_reserved_before_free_robot_matching)
{
  const auto ins = line_instance(4, {0, 3}, {0}, {0}, {{2}});
  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  ASSERT_TRUE(G0.rho_task_id[0].has_value());
  const std::vector<Op> lift = {
      Op::make_lift(), Op::make_wait()};
  const auto X1 = apply_ops(ins, X0, lift);
  ASSERT_TRUE(X1.has_value());
  const auto G1 =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &lift);
  ASSERT_TRUE(G1.custody_by_robot[0].has_value());

  const std::vector<Op> move = {
      Op::make_move(ins.grid.idx(0, 1)), Op::make_wait()};
  const auto X2 = apply_ops(ins, *X1, move);
  ASSERT_TRUE(X2.has_value());
  const auto G2 =
      dd_task_br_guidance_probe(ins, *X2, &*X1, &G1, &move);
  ASSERT_TRUE(G2.custody_by_robot[0].has_value());
  EXPECT_EQ(G2.custody_by_robot[0]->from, ins.grid.idx(0, 1));
  EXPECT_EQ(G2.custody_by_robot[0]->to, ins.grid.idx(0, 2));
  ASSERT_EQ(G2.rho_task_id.size(), 2u);
  EXPECT_FALSE(G2.rho_task_id[1].has_value())
      << "a free robot must not compete for the carried continuation";
}

TEST(dd_task_br_execution, drop_clears_custody)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  const std::vector<Op> lift_ops = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift_ops);
  ASSERT_TRUE(X1.has_value());
  const auto G1 =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &lift_ops);
  ASSERT_TRUE(G1.custody_by_robot[0].has_value());

  const std::vector<Op> drop_ops = {Op::make_drop()};
  const auto X2 = apply_ops(ins, *X1, drop_ops);
  ASSERT_TRUE(X2.has_value());
  const auto G2 =
      dd_task_br_guidance_probe(ins, *X2, &*X1, &G1, &drop_ops);
  EXPECT_FALSE(G2.custody_by_robot[0].has_value());
}

TEST(dd_task_br_execution,
     forced_deviation_move_discards_the_old_exact_custody)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "...", "..."});
  ins.robots = {ins.grid.idx(1, 1)};
  ins.shelves = {ins.grid.idx(1, 1)};
  ins.target_starts = {ins.grid.idx(1, 1)};
  ins.target_goal_sets = {{ins.grid.idx(1, 2)}};
  ins.target_goals = {ins.grid.idx(1, 2)};
  ins.finalize();

  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  const std::vector<Op> lift = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift);
  ASSERT_TRUE(X1.has_value());
  const auto G1 =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &lift);
  ASSERT_TRUE(G1.custody_by_robot[0].has_value());
  const TaskId old_id = G1.custody_by_robot[0]->task_id;
  EXPECT_EQ(old_id.to, ins.grid.idx(1, 2));

  const std::vector<Op> deviation = {
      Op::make_move(ins.grid.idx(0, 1))};
  const auto X2 = apply_ops(ins, *X1, deviation);
  ASSERT_TRUE(X2.has_value());
  const auto successors =
      dd_enumerate_node_successors(ins, *X1, 0);
  EXPECT_NE(std::find(successors.begin(), successors.end(), *X2),
            successors.end());

  const auto G2 =
      dd_task_br_guidance_probe(ins, *X2, &*X1, &G1, &deviation);
  if (G2.custody_by_robot[0].has_value()) {
    EXPECT_NE(G2.custody_by_robot[0]->task_id, old_id);
    EXPECT_EQ(G2.custody_by_robot[0]->from,
              ins.grid.idx(0, 1));
  }
}

TEST(dd_task_br_execution, no_transition_cannot_inject_custody)
{
  const auto ins = line_instance(3, {0}, {0}, {0}, {{2}});
  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  const std::vector<Op> lift_ops = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift_ops);
  ASSERT_TRUE(X1.has_value());

  const auto unattached = dd_task_br_guidance_probe(ins, *X1);
  ASSERT_EQ(unattached.custody_by_robot.size(), 1u);
  EXPECT_FALSE(unattached.custody_by_robot[0].has_value());

  const auto missing_ops =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, nullptr);
  EXPECT_FALSE(missing_ops.custody_by_robot[0].has_value());
}

TEST(dd_task_br_execution, forced_nonready_lift_is_loaded_but_unbound)
{
  // The ready task is anon 1->2, but the exhaustive tree forces the robot
  // at target 0 to Lift that unrelated, non-ready target.
  const auto ins = line_instance(3, {0}, {0, 1}, {0}, {{2}});
  const auto X0 = initial_phys_config(ins);
  const auto G0 = dd_task_br_guidance_probe(ins, X0);
  ASSERT_TRUE(G0.rho_task_id[0].has_value());
  EXPECT_EQ(G0.rho_task_id[0]->from, ins.grid.idx(0, 1));

  const std::vector<Op> forced_lift = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, forced_lift);
  ASSERT_TRUE(X1.has_value());
  ASSERT_EQ(X1->kappa[0], 0);
  const auto successors =
      dd_enumerate_node_successors(ins, X0, 0);
  EXPECT_NE(std::find(successors.begin(), successors.end(), *X1),
            successors.end())
      << "the exhaustive operator tree must retain the non-ready Lift";
  const auto G1 =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &forced_lift);
  ASSERT_EQ(G1.custody_by_robot.size(), 1u);
  EXPECT_FALSE(G1.custody_by_robot[0].has_value());
}

TEST(dd_task_br_execution, task_id_survives_vector_reordering)
{
  const auto ins = line_instance(5, {0}, {}, {}, {});
  const auto X = initial_phys_config(ins);
  const TaskId near{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 1}, 1, 2};
  const TaskId far{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 3}, 3, 4};
  ShelfTaskGraph first;
  first.tasks = {
      ShelfTask{near, {RootDemand{0, 10}}, 5},
      ShelfTask{far, {RootDemand{1, 11}}, 5},
  };
  first.predecessors = {{}, {}};
  first.successors = {{}, {}};
  const auto a =
      dd_match_ready_tasks_probe(ins, X, first, {0, 1}, nullptr);
  ASSERT_TRUE(a.rho_task_id[0].has_value());
  EXPECT_EQ(*a.rho_task_id[0], near);
  EXPECT_EQ(a.rho_ready_index[0], 0);

  ShelfTaskGraph reordered = first;
  std::swap(reordered.tasks[0], reordered.tasks[1]);
  const auto b = dd_match_ready_tasks_probe(
      ins, X, reordered, {0, 1}, &a.rho_task_id);
  ASSERT_TRUE(b.rho_task_id[0].has_value());
  EXPECT_EQ(*b.rho_task_id[0], near);
  EXPECT_EQ(b.rho_ready_index[0], 1);
}

TEST(dd_task_br_execution,
     loaded_wait_re_resolves_exact_custody_after_graph_reordering)
{
  const auto ins =
      line_instance(5, {3}, {0, 1, 2, 3}, {0}, {{4}});
  const auto X0 = initial_phys_config(ins);
  auto G0 = dd_task_br_guidance_probe(ins, X0);
  ASSERT_TRUE(G0.rho_task_id[0].has_value());
  const std::vector<Op> lift = {Op::make_lift()};
  const auto X1 = apply_ops(ins, X0, lift);
  ASSERT_TRUE(X1.has_value());
  auto G1 =
      dd_task_br_guidance_probe(ins, *X1, &X0, &G0, &lift);
  ASSERT_TRUE(G1.custody_by_robot[0].has_value());
  ASSERT_NE(G1.upper_epoch, nullptr);
  ASSERT_GE(G1.custody_by_robot[0]->current_task_index.value_or(-1), 0);
  ASSERT_GT(G1.upper_epoch->task_graph.tasks.size(), 1u);

  const TaskId exact = G1.custody_by_robot[0]->task_id;
  const int old_index =
      *G1.custody_by_robot[0]->current_task_index;
  const int other_index = old_index == 0 ? 1 : 0;
  auto reordered =
      std::make_shared<UpperEpochGuidance>(*G1.upper_epoch);
  swap_task_graph_indices(
      reordered->task_graph, old_index, other_index);
  ASSERT_NE(reordered->task_graph.tasks[old_index].id, exact);
  ASSERT_EQ(reordered->task_graph.tasks[other_index].id, exact);
  G1.upper_epoch = reordered;

  const std::vector<Op> wait = {Op::make_wait()};
  const auto X2 = apply_ops(ins, *X1, wait);
  ASSERT_TRUE(X2.has_value());
  const auto G2 =
      dd_task_br_guidance_probe(ins, *X2, &*X1, &G1, &wait);
  ASSERT_TRUE(G2.custody_by_robot[0].has_value());
  EXPECT_EQ(G2.custody_by_robot[0]->task_id, exact);
  ASSERT_TRUE(
      G2.custody_by_robot[0]->current_task_index.has_value());
  EXPECT_EQ(*G2.custody_by_robot[0]->current_task_index,
            other_index);
}

TEST(dd_task_br_execution, rho_lexicographic_cutoff_distance_then_switch)
{
  const auto ins = line_instance(7, {0}, {}, {}, {});
  const auto X = initial_phys_config(ins);
  const TaskId near{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 1}, 1, 2};
  const TaskId far{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 6}, 6, 5};

  ShelfTaskGraph priority_graph;
  priority_graph.tasks = {
      ShelfTask{near, {RootDemand{0, 10}}, 1},
      ShelfTask{far, {RootDemand{1, 11}}, 9},
  };
  priority_graph.predecessors = {{}, {}};
  priority_graph.successors = {{}, {}};
  const auto priority = dd_match_ready_tasks_probe(
      ins, X, priority_graph, {0, 1}, nullptr);
  ASSERT_TRUE(priority.rho_task_id[0].has_value());
  EXPECT_EQ(*priority.rho_task_id[0], far)
      << "higher-than-cutoff priority must own the scarce row";

  ShelfTaskGraph distance_graph = priority_graph;
  distance_graph.tasks[0].priority = 5;
  distance_graph.tasks[1].priority = 5;
  const std::vector<std::optional<TaskId>> previous = {far};
  const auto distance = dd_match_ready_tasks_probe(
      ins, X, distance_graph, {0, 1}, &previous);
  ASSERT_TRUE(distance.rho_task_id[0].has_value());
  EXPECT_EQ(*distance.rho_task_id[0], near)
      << "approach distance must beat the later switch penalty";
}

TEST(dd_task_br_execution,
     multirow_cutoff_assigns_all_mandatory_rows_then_nearest_tie)
{
  const auto ins = line_instance(12, {0, 11}, {}, {}, {});
  const auto X = initial_phys_config(ins);
  const TaskId mandatory{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 0}, 0, 1};
  const TaskId cutoff_near{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 10}, 10, 9};
  const TaskId cutoff_far{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 5}, 5, 6};
  const TaskId below_cutoff{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 7}, 7, 8};
  ShelfTaskGraph graph;
  graph.tasks = {
      ShelfTask{mandatory, {RootDemand{0, 20}}, 9},
      ShelfTask{cutoff_near, {RootDemand{1, 21}}, 8},
      ShelfTask{cutoff_far, {RootDemand{2, 22}}, 8},
      ShelfTask{below_cutoff, {RootDemand{3, 23}}, 1},
  };
  graph.predecessors = {{}, {}, {}, {}};
  graph.successors = {{}, {}, {}, {}};
  const std::vector<std::optional<TaskId>> previous = {
      std::nullopt, cutoff_far};

  const auto result = dd_match_ready_tasks_probe(
      ins, X, graph, {0, 1, 2, 3}, &previous);
  std::set<TaskId> assigned;
  for (const auto& id : result.rho_task_id)
    if (id.has_value()) assigned.insert(*id);
  ASSERT_EQ(assigned.size(), 2u);
  EXPECT_TRUE(assigned.count(mandatory))
      << "every row above the priority cutoff is mandatory";
  EXPECT_TRUE(assigned.count(cutoff_near))
      << "distance at the cutoff must beat the later switch penalty";
  EXPECT_FALSE(assigned.count(cutoff_far));
  EXPECT_FALSE(assigned.count(below_cutoff));
}
