// PROTECTED Task-BR-PIBT compiler tests.
// Written before implementation on 2026-09-03 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <cmath>
#include <unordered_set>

#include "gtest/gtest.h"

namespace {

DDInstance make_instance(
    const std::vector<std::string>& rows,
    const std::vector<std::pair<int, int>>& robots,
    const std::vector<std::pair<int, int>>& shelves,
    const std::vector<std::pair<std::pair<int, int>,
                                std::vector<std::pair<int, int>>>>& targets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  for (const auto& q : robots)
    ins.robots.push_back(ins.grid.idx(q.first, q.second));
  for (const auto& p : shelves)
    ins.shelves.push_back(ins.grid.idx(p.first, p.second));
  for (const auto& t : targets) {
    ins.target_starts.push_back(
        ins.grid.idx(t.first.first, t.first.second));
    std::vector<int> goals;
    for (const auto& g : t.second)
      goals.push_back(ins.grid.idx(g.first, g.second));
    ins.target_goal_sets.push_back(goals);
    ins.target_goals.push_back(goals.front());
  }
  ins.finalize();
  return ins;
}

bool adjacent(const DDGrid& grid, int from, int to)
{
  int neighbors[4];
  const int n = grid.neighbors(from, neighbors);
  for (int k = 0; k < n; ++k)
    if (neighbors[k] == to) return true;
  return false;
}

int find_effect(const ShelfTaskGraph& graph, int from, int to)
{
  for (size_t k = 0; k < graph.tasks.size(); ++k)
    if (graph.tasks[k].id.from == from && graph.tasks[k].id.to == to)
      return (int)k;
  return -1;
}

}  // namespace

TEST(dd_task_br_compiler, task_identity_is_the_exact_adjacent_effect)
{
  const ShelfSelector shelf{ShelfSelector::Kind::TARGET, 0};
  const TaskId a{shelf, 3, 4};
  const TaskId b{shelf, 3, 2};
  EXPECT_NE(a, b);
  std::unordered_set<TaskId, TaskIdHash> ids;
  ids.insert(a);
  ids.insert(b);
  EXPECT_EQ(ids.size(), 2u);

  const auto ins = make_instance(
      {"....."}, {{0, 0}},
      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
      {{{0, 0}, {{0, 4}}}});
  const auto graph =
      dd_compile_joint_graph_probe(ins, initial_phys_config(ins));
  ASSERT_FALSE(graph.tasks.empty());
  for (const auto& task : graph.tasks) {
    EXPECT_NE(task.id.from, task.id.to);
    EXPECT_NE(task.id.to, -1);
    EXPECT_FALSE(ins.grid.is_wall(task.id.from));
    EXPECT_FALSE(ins.grid.is_wall(task.id.to));
    EXPECT_TRUE(adjacent(ins.grid, task.id.from, task.id.to));
    EXPECT_EQ(task.id,
              (TaskId{task.id.shelf, task.id.from, task.id.to}));
  }
}

TEST(dd_task_br_compiler,
     requester_tries_next_candidate_and_failed_branch_rolls_back)
{
  // [anon][target][empty], target goal is the anonymous shelf's cell.
  // The preferred left candidate enters a two-shelf recursion cycle; the
  // requester must roll that branch back and choose the empty right cell.
  const auto ins = make_instance(
      {"..."}, {{0, 1}}, {{0, 0}, {0, 1}},
      {{{0, 1}, {{0, 0}}}});
  const auto X = initial_phys_config(ins);
  const auto graph =
      dd_compile_single_root_graph_probe(ins, X, 0, ins.grid.idx(0, 0));

  ASSERT_EQ(graph.tasks.size(), 1u);
  EXPECT_EQ(graph.tasks[0].id.shelf,
            (ShelfSelector{ShelfSelector::Kind::TARGET, 0}));
  EXPECT_EQ(graph.tasks[0].id.from, ins.grid.idx(0, 1));
  EXPECT_EQ(graph.tasks[0].id.to, ins.grid.idx(0, 2));
  EXPECT_TRUE(graph.predecessors[0].empty());
  ASSERT_EQ(graph.tasks[0].roots.size(), 1u);
  EXPECT_EQ(graph.tasks[0].roots[0],
            (RootDemand{0, ins.grid.idx(0, 0)}));
}

TEST(dd_task_br_compiler,
     recursion_cap_is_restarted_for_each_root_option)
{
  const auto ins = make_instance(
      {"...", "..."},
      {{1, 2}},
      {{0, 1}, {0, 0}, {1, 0}},
      {{{0, 1}, {{0, 0}}}});
  const auto X = initial_phys_config(ins);
  const std::vector<int> tau = {ins.grid.idx(0, 0)};
  const std::vector<int> priority = {1};
  const auto graph = dd_compile_joint_graph_probe(
      ins, X, &tau, &priority, 1, 16);

  ASSERT_TRUE(graph.paused_roots.empty());
  ASSERT_EQ(graph.tasks.size(), 1u);
  EXPECT_EQ(
      graph.tasks.front().id,
      (TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          ins.grid.idx(0, 1),
          ins.grid.idx(0, 2)}));
}

TEST(dd_task_br_compiler, one_empty_uses_generic_recursive_dependency_chain)
{
  const auto ins = make_instance(
      {"....."}, {{0, 0}},
      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
      {{{0, 0}, {{0, 4}}}});
  const auto X = initial_phys_config(ins);
  const auto graph = dd_compile_joint_graph_probe(ins, X);

  ASSERT_EQ(graph.tasks.size(), 4u);
  const int t34 = find_effect(graph, ins.grid.idx(0, 3),
                              ins.grid.idx(0, 4));
  const int t23 = find_effect(graph, ins.grid.idx(0, 2),
                              ins.grid.idx(0, 3));
  const int t12 = find_effect(graph, ins.grid.idx(0, 1),
                              ins.grid.idx(0, 2));
  const int t01 = find_effect(graph, ins.grid.idx(0, 0),
                              ins.grid.idx(0, 1));
  ASSERT_GE(t34, 0);
  ASSERT_GE(t23, 0);
  ASSERT_GE(t12, 0);
  ASSERT_GE(t01, 0);
  EXPECT_EQ(graph.predecessors[t34].size(), 0u);
  EXPECT_EQ(graph.predecessors[t23], std::vector<int>({t34}));
  EXPECT_EQ(graph.predecessors[t12], std::vector<int>({t23}));
  EXPECT_EQ(graph.predecessors[t01], std::vector<int>({t12}));

  const auto ready = dd_ready_tasks_probe(ins, X, graph);
  ASSERT_EQ(ready.size(), 1u);
  EXPECT_EQ(ready[0], t34);
}

TEST(dd_task_br_compiler, ready_requires_the_exact_physical_leaf)
{
  const auto ins = make_instance(
      {"....."}, {{0, 0}},
      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
      {{{0, 0}, {{0, 4}}}});
  auto X = initial_phys_config(ins);
  const auto graph = dd_compile_joint_graph_probe(ins, X);
  ASSERT_EQ(dd_ready_tasks_probe(ins, X, graph).size(), 1u);

  // Remove the selected anonymous shelf from its exact `from`.
  X.anon_occ.erase(
      std::find(X.anon_occ.begin(), X.anon_occ.end(), ins.grid.idx(0, 3)));
  EXPECT_TRUE(dd_ready_tasks_probe(ins, X, graph).empty());

  // A carried shelf without transition-derived custody is unrelated and
  // must not become a ready continuation merely because coordinates match.
  X = initial_phys_config(ins);
  X.anon_occ.erase(
      std::find(X.anon_occ.begin(), X.anon_occ.end(), ins.grid.idx(0, 3)));
  X.robots[0] = ins.grid.idx(0, 3);
  X.kappa[0] = KAPPA_ANON;
  EXPECT_TRUE(dd_ready_tasks_probe(ins, X, graph).empty());
}

TEST(dd_task_br_compiler, all_roots_are_compiled_or_explicitly_paused)
{
  const auto ins = make_instance(
      {"....", "...."}, {{0, 0}, {1, 0}},
      {{0, 0}, {1, 0}},
      {{{0, 0}, {{0, 3}}}, {{1, 0}, {{1, 3}}}});
  const auto X = initial_phys_config(ins);
  const auto full = dd_compile_joint_graph_probe(ins, X);
  EXPECT_TRUE(full.paused_roots.empty());
  std::unordered_set<int> roots;
  for (const auto& task : full.tasks)
    for (const auto& root : task.roots) roots.insert(root.target);
  EXPECT_EQ(roots, (std::unordered_set<int>{0, 1}));

  const auto exhausted =
      dd_compile_joint_graph_probe(ins, X, nullptr, nullptr,
                                   /*recursion_cap=*/0,
                                   /*backtrack_cap=*/0);
  EXPECT_TRUE(exhausted.tasks.empty());
  EXPECT_EQ(exhausted.paused_roots, std::vector<int>({0, 1}));
}

TEST(dd_task_br_compiler, unfinished_roots_are_not_truncated_at_legacy_cap)
{
  constexpr int root_count = 70;
  std::vector<std::string> rows(root_count, "...");
  std::vector<std::pair<int, int>> shelves;
  std::vector<std::pair<std::pair<int, int>,
                        std::vector<std::pair<int, int>>>> targets;
  for (int row = 0; row < root_count; ++row) {
    shelves.push_back({row, 0});
    targets.push_back({{row, 0}, {{row, 2}}});
  }
  const auto ins =
      make_instance(rows, {{0, 0}}, shelves, targets);
  const auto graph =
      dd_compile_joint_graph_probe(ins, initial_phys_config(ins));

  EXPECT_TRUE(graph.paused_roots.empty());
  std::unordered_set<int> represented;
  for (const auto& task : graph.tasks)
    for (const auto& root : task.roots)
      represented.insert(root.target);
  ASSERT_EQ(represented.size(), (size_t)root_count);
  for (int target = 0; target < root_count; ++target)
    EXPECT_EQ(represented.count(target), 1u);
}

TEST(dd_task_br_compiler, zero_empty_failure_is_finite_and_not_infeasible)
{
  const auto ins = make_instance(
      {"..."}, {{0, 0}}, {{0, 0}, {0, 1}, {0, 2}},
      {{{0, 0}, {{0, 2}}}});
  const auto X = initial_phys_config(ins);
  const auto graph = dd_compile_joint_graph_probe(ins, X);
  EXPECT_TRUE(dd_ready_tasks_probe(ins, X, graph).empty());
  const auto plan =
      dd_pair_cost_probe(ins, X, 0, ins.grid.idx(0, 2));
  EXPECT_TRUE(std::isfinite(plan.estimated_cost));
  EXPECT_TRUE(plan.stalled || plan.truncated);
}

TEST(dd_task_br_compiler, singleton_goal_assignment_is_natural)
{
  const auto ins = make_instance(
      {"...."}, {{0, 0}}, {{0, 1}},
      {{{0, 1}, {{0, 3}}}});
  const auto tau = dd_tau_guide_probe(ins, initial_phys_config(ins));
  ASSERT_EQ(tau.size(), 1u);
  EXPECT_EQ(tau[0], ins.grid.idx(0, 3));
}

TEST(dd_task_br_compiler, exact_effect_is_shared_across_root_demands)
{
  // B's own root move is exactly the blocker displacement needed by A.
  // The physical effect B:1->2 must be one graph node carrying both roots.
  const auto ins = make_instance(
      {"..."}, {{0, 0}, {0, 1}}, {{0, 0}, {0, 1}},
      {{{0, 0}, {{0, 1}}}, {{0, 1}, {{0, 2}}}});
  const auto graph =
      dd_compile_joint_graph_probe(ins, initial_phys_config(ins));
  const int shared =
      find_effect(graph, ins.grid.idx(0, 1), ins.grid.idx(0, 2));
  ASSERT_GE(shared, 0);
  EXPECT_EQ(graph.tasks[shared].id.shelf,
            (ShelfSelector{ShelfSelector::Kind::TARGET, 1}));
  EXPECT_EQ(graph.tasks[shared].roots,
            (std::vector<RootDemand>{
                RootDemand{0, ins.grid.idx(0, 1)},
                RootDemand{1, ins.grid.idx(0, 2)}}));
}

TEST(dd_task_br_compiler,
     shared_nonready_demand_propagates_to_the_deepest_leaf)
{
  ShelfTaskGraph graph;
  const TaskId leaf{
      ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 2}, 2, 3};
  const TaskId shared{
      ShelfSelector{ShelfSelector::Kind::TARGET, 1}, 1, 2};
  graph.tasks = {
      ShelfTask{leaf, {RootDemand{0, 10}}, 1},
      ShelfTask{shared, {RootDemand{0, 10}, RootDemand{1, 11}}, 1},
  };
  graph.predecessors = {{}, {0}};
  graph.successors = {{1}, {}};

  const auto propagated =
      dd_propagate_root_demands_probe(graph, {1, 9});
  ASSERT_EQ(propagated.tasks[0].roots.size(), 2u);
  EXPECT_EQ(propagated.tasks[0].roots,
            (std::vector<RootDemand>{
                RootDemand{0, 10}, RootDemand{1, 11}}));
  EXPECT_EQ(propagated.tasks[0].priority, 9);
  EXPECT_EQ(propagated.tasks[1].priority, 9);
}

TEST(dd_task_br_compiler,
     same_shelf_same_from_different_to_is_a_conflict)
{
  const ShelfSelector shelf{ShelfSelector::Kind::TARGET, 1};
  const TaskId left{shelf, 1, 0};
  const TaskId right{shelf, 1, 2};
  EXPECT_NE(left, right);
  EXPECT_TRUE(dd_task_effects_conflict_probe(left, right));
}

TEST(dd_task_br_compiler,
     destination_conflict_backtracks_the_high_root_to_its_alternative)
{
  // A prefers (1,2), but B is a dead-end root whose only move is also
  // (1,2). Joint DFS must retry A at (0,1), leaving both roots compiled.
  const auto ins = make_instance(
      {"#.##", "....", "##.#"}, {{1, 1}, {2, 2}},
      {{1, 1}, {2, 2}},
      {{{1, 1}, {{1, 3}}}, {{2, 2}, {{1, 2}}}});
  const std::vector<int> tau = {
      ins.grid.idx(1, 3), ins.grid.idx(1, 2)};
  const std::vector<int> priority = {2, 1};
  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins), &tau, &priority);

  EXPECT_TRUE(graph.paused_roots.empty());
  EXPECT_GE(find_effect(graph, ins.grid.idx(1, 1),
                        ins.grid.idx(0, 1)),
            0);
  EXPECT_GE(find_effect(graph, ins.grid.idx(2, 2),
                        ins.grid.idx(1, 2)),
            0);
  EXPECT_EQ(find_effect(graph, ins.grid.idx(1, 1),
                        ins.grid.idx(1, 2)),
            -1)
      << "failed preferred branch leaked through transactional rollback";
}

TEST(dd_task_br_compiler, target_blocker_uses_its_own_fixed_tau)
{
  // A moves through B's current cell. B has three empty vacate choices;
  // the one reaching B's own fixed tau must win.
  const auto ins = make_instance(
      {"...", "...", "..."}, {{1, 0}, {1, 1}},
      {{1, 0}, {1, 1}},
      {{{1, 0}, {{1, 2}}}, {{1, 1}, {{0, 1}}}});
  const std::vector<int> tau = {
      ins.grid.idx(1, 2), ins.grid.idx(0, 1)};
  const std::vector<int> priority = {2, 1};
  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins), &tau, &priority);
  const int blocker =
      find_effect(graph, ins.grid.idx(1, 1), ins.grid.idx(0, 1));
  ASSERT_GE(blocker, 0);
  EXPECT_EQ(graph.tasks[blocker].id.shelf,
            (ShelfSelector{ShelfSelector::Kind::TARGET, 1}));
}
