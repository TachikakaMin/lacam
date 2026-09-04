#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <optional>
#include <utility>
#include <vector>

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
  for (const auto& target : targets) {
    ins.target_starts.push_back(
        ins.grid.idx(target.first.first, target.first.second));
    std::vector<int> goals;
    for (const auto& goal : target.second)
      goals.push_back(ins.grid.idx(goal.first, goal.second));
    ins.target_goal_sets.push_back(goals);
    ins.target_goals.push_back(goals.front());
  }
  ins.finalize();
  return ins;
}

std::optional<TaskId> graph_next_ready_effect(
    const DDInstance& ins, const PhysConfig& X, int target, int goal,
    int recursion_cap)
{
  const auto graph = dd_compile_single_root_graph_probe(
      ins, X, target, goal, recursion_cap, 512);
  const auto ready = dd_ready_tasks_probe(ins, X, graph);
  if (ready.empty()) return std::nullopt;
  return graph.tasks[ready.front()].id;
}

void expect_kernel_matches_graph(const DDInstance& ins,
                                 const PhysConfig& X, int target,
                                 int goal, int recursion_cap = 256)
{
  EXPECT_EQ(dd_pair_next_ready_effect_probe(
                ins, X, target, goal, recursion_cap),
            graph_next_ready_effect(
                ins, X, target, goal, recursion_cap));
}

}  // namespace

TEST(dd_task_br_pair_kernel, matches_dependency_leaf_and_direct_effect)
{
  const auto chain = make_instance(
      {"....."}, {{0, 0}},
      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
      {{{0, 0}, {{0, 4}}}});
  expect_kernel_matches_graph(
      chain, initial_phys_config(chain), 0, chain.grid.idx(0, 4));

  const auto direct = make_instance(
      {"...."}, {{0, 1}}, {{0, 1}},
      {{{0, 1}, {{0, 3}}}});
  expect_kernel_matches_graph(
      direct, initial_phys_config(direct), 0, direct.grid.idx(0, 3));
}

TEST(dd_task_br_pair_kernel, matches_failed_branch_transactional_rollback)
{
  const auto ins = make_instance(
      {"..."}, {{0, 1}}, {{0, 0}, {0, 1}},
      {{{0, 1}, {{0, 0}}}});
  const auto X = initial_phys_config(ins);
  const auto effect = dd_pair_next_ready_effect_probe(
      ins, X, 0, ins.grid.idx(0, 0));

  expect_kernel_matches_graph(ins, X, 0, ins.grid.idx(0, 0));
  ASSERT_TRUE(effect.has_value());
  EXPECT_EQ(effect->shelf,
            (ShelfSelector{ShelfSelector::Kind::TARGET, 0}));
  EXPECT_EQ(effect->from, ins.grid.idx(0, 1));
  EXPECT_EQ(effect->to, ins.grid.idx(0, 2));
}

TEST(dd_task_br_pair_kernel, matches_cycle_stall_and_recursion_budget)
{
  const auto full = make_instance(
      {"..."}, {{0, 0}}, {{0, 0}, {0, 1}, {0, 2}},
      {{{0, 0}, {{0, 2}}}});
  const auto X = initial_phys_config(full);
  expect_kernel_matches_graph(full, X, 0, full.grid.idx(0, 2));
  EXPECT_FALSE(dd_pair_next_ready_effect_probe(
                   full, X, 0, full.grid.idx(0, 2))
                   .has_value());

  const auto chain = make_instance(
      {"....."}, {{0, 0}},
      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
      {{{0, 0}, {{0, 4}}}});
  for (int cap = 0; cap <= 5; ++cap)
    expect_kernel_matches_graph(
        chain, initial_phys_config(chain), 0,
        chain.grid.idx(0, 4), cap);
}

TEST(dd_task_br_pair_kernel,
     lazy_exact_matching_matches_full_matrix_and_skips_nontight_edges)
{
  DDInstance ins;
  ins.grid = DDGrid({"..............."});
  ins.robots = {ins.grid.idx(0, 7)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 7),
      ins.grid.idx(0, 14),
  };
  ins.target_starts = ins.shelves;
  const std::vector<int> shared_goals = ins.shelves;
  ins.target_goal_sets = {
      shared_goals, shared_goals, shared_goals};
  ins.target_goals = {
      shared_goals[0], shared_goals[1], shared_goals[2]};
  ins.finalize();

  const auto X = initial_phys_config(ins);
  const auto lazy = dd_lazy_tau_guide_probe(ins, X);
  EXPECT_EQ(lazy.tau, dd_tau_guide_probe(ins, X));
  EXPECT_EQ(lazy.total_edges, 9);
  EXPECT_LT(lazy.evaluated_edges, lazy.total_edges);

  for (size_t target = 0; target < lazy.tau.size(); ++target) {
    bool found = false;
    for (const auto& entry : lazy.table[target]) {
      if (entry.goal != lazy.tau[target]) continue;
      EXPECT_TRUE(entry.plan.exact);
      found = true;
    }
    EXPECT_TRUE(found);
  }
}

TEST(dd_task_br_pair_kernel,
     one_step_restoration_root_does_not_preempt_multistep_mission)
{
  const auto ins = make_instance(
      {"...................."}, {{0, 0}, {0, 4}},
      {{0, 0}, {0, 4}},
      {{{0, 0}, {{0, 1}}}, {{0, 4}, {{0, 19}}}});
  const auto guidance =
      dd_task_br_guidance_probe(ins, initial_phys_config(ins));

  ASSERT_NE(guidance.upper_epoch, nullptr);
  ASSERT_EQ(guidance.upper_epoch->target_priority.size(), 2u);
  EXPECT_GT(guidance.upper_epoch->target_priority[1],
            guidance.upper_epoch->target_priority[0]);
}

TEST(dd_task_br_pair_kernel,
     blocked_higher_pair_cost_mission_has_higher_priority_with_two_empty_cells)
{
  const auto ins = make_instance(
      {"....."}, {{0, 0}, {0, 2}},
      {{0, 0}, {0, 2}, {0, 3}},
      {{{0, 0}, {{0, 1}}}, {{0, 2}, {{0, 3}}}});
  const auto guidance =
      dd_task_br_guidance_probe(ins, initial_phys_config(ins));

  ASSERT_NE(guidance.upper_epoch, nullptr);
  ASSERT_EQ(guidance.upper_epoch->target_priority.size(), 2u);
  const auto& table = guidance.upper_epoch->pair_cost;
  ASSERT_EQ(table.size(), 2u);
  ASSERT_EQ(table[0].size(), 1u);
  ASSERT_EQ(table[1].size(), 1u);
  EXPECT_GT(table[1][0].plan.estimated_cost,
            table[0][0].plan.estimated_cost);
  EXPECT_GT(guidance.upper_epoch->target_priority[1],
            guidance.upper_epoch->target_priority[0]);
}
