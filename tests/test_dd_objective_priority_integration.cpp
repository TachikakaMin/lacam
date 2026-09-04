// PROTECTED integration test: per-pass base priority is frozen from root
// objective LB and reaches rho/task ordering.  Written TDD RED, 2026-09-02.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <set>

#include "gtest/gtest.h"

TEST(dd_objective_priority_integration, farther_root_owns_the_frontline_slot)
{
  DDInstance ins;
  ins.grid = DDGrid({".......", "......."});
  ins.robots = {ins.grid.idx(1, 0)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  ins.target_goals = {ins.grid.idx(0, 1), ins.grid.idx(0, 6)};
  ins.finalize();

  DDObjectiveBuildProbe probe;
  std::vector<int> rho;
  const auto tasks =
      dd_build_tasks(ins, initial_phys_config(ins), &rho, &probe);
  ASSERT_EQ(probe.effective_priority.size(), 2u);
  EXPECT_GT(probe.effective_priority[1], probe.effective_priority[0])
      << "the larger root LB must receive the higher frozen base priority";
  ASSERT_EQ(rho.size(), 1u);
  ASSERT_GE(rho[0], 0);
  ASSERT_LT(rho[0], (int)tasks.size());
  ASSERT_FALSE(tasks[rho[0]].roots.empty());
  EXPECT_EQ(tasks[rho[0]].roots.front().root_target, 1)
      << "task priority must reach rho before legacy 100/50 ordering";
}

TEST(dd_objective_priority_integration,
     free_assigned_carriers_keep_stable_v3_order)
{
  DDInstance ins;
  ins.grid = DDGrid({"..........", ".........."});
  ins.robots = {ins.grid.idx(1, 0), ins.grid.idx(0, 5)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 5)};
  ins.target_starts = {ins.grid.idx(0, 0), ins.grid.idx(0, 5)};
  ins.target_goals = {ins.grid.idx(0, 1), ins.grid.idx(0, 9)};
  ins.finalize();

  const auto order =
      dd_objective_robot_order_probe(ins, initial_phys_config(ins));
  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order[0], 0)
      << "free-assigned Carrier-PIBT keeps stable v3 order even when the "
         "later robot owns the harder task and is already at its pickup";
}

TEST(dd_objective_priority_integration,
     rho_reservation_replaces_only_the_last_truncated_slot)
{
  DDInstance ins;
  ins.grid = DDGrid({"..........", ".........."});
  ins.robots = {ins.grid.idx(1, 0), ins.grid.idx(1, 4)};
  ins.shelves = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 2), ins.grid.idx(0, 4)};
  ins.target_starts = ins.shelves;
  ins.target_goals = {
      ins.grid.idx(0, 1), ins.grid.idx(0, 3), ins.grid.idx(0, 9)};
  ins.finalize();

  std::vector<int> rho;
  const auto tasks =
      dd_build_tasks(ins, initial_phys_config(ins), &rho, nullptr);
  std::set<int> assigned_roots;
  for (const int task : rho)
    if (task >= 0 && task < (int)tasks.size() &&
        !tasks[task].roots.empty())
      assigned_roots.insert(tasks[task].roots.front().root_target);
  EXPECT_EQ(assigned_roots, (std::set<int>{0, 2}))
      << "legacy prefix [A,B] becomes [A,C] when only C owns the bounded "
         "highest-priority reservation";
}

TEST(dd_objective_priority_integration,
     loaded_labeled_carriers_use_remaining_distance_first)
{
  DDInstance ins;
  ins.grid = DDGrid({".........."});
  ins.robots = {ins.grid.idx(0, 5), ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 5)};
  ins.target_starts = ins.shelves;
  ins.target_goals = {ins.grid.idx(0, 4), ins.grid.idx(0, 6)};
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa = {1, 0};

  const auto order = dd_objective_robot_order_probe(ins, X);
  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order[0], 0)
      << "the one-step loaded mission precedes the harder four-step mission";
}

TEST(dd_objective_priority_integration,
     loaded_labeled_equal_distance_keeps_stable_v3_order)
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.robots = {ins.grid.idx(0, 4), ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 4)};
  ins.target_starts = ins.shelves;
  ins.target_goals = {ins.grid.idx(0, 1), ins.grid.idx(0, 5)};
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa = {1, 0};

  const auto order = dd_objective_robot_order_probe(ins, X);
  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order[0], 0)
      << "Carrier-PIBT does not consume task priority; equal-distance "
         "loaded carriers retain stable robot-id order";
}
