// PROTECTED joint-compiler progress regression.
// Added before implementation on 2026-09-03 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_task_br_joint_progress,
     higher_priority_root_progress_beats_a_smaller_shared_detour)
{
  // Cells:
  //   0 1      T0 at 0 wants 2, which is occupied by anonymous A2.
  //   2 3      T1 at 1 wants 3, which is empty.
  //
  // Two all-root candidates have the same aggregate remaining distance:
  //
  //   direct high root: A2 2->3, T0 0->2, T1 1->0
  //                     per-root residual [0, 2], three tasks
  //   shared detour:    T1 1->3, T0 0->1
  //                     per-root residual [2, 0], two tasks
  //
  // Root order is priority order, so the direct [0, 2] candidate must win
  // before graph-size/work tie-breaks are considered.
  DDInstance ins;
  ins.grid = DDGrid({"..", ".."});
  ins.robots = {0, 1};
  ins.shelves = {0, 1, 2};
  ins.target_starts = {0, 1};
  ins.target_goal_sets = {{2}, {3}};
  ins.target_goals = {2, 3};
  ins.finalize();

  const std::vector<int> tau = {2, 3};
  const std::vector<int> priority = {2, 1};
  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins), &tau, &priority);

  const TaskId high_direct{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0}, 0, 2};
  const TaskId high_detour{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0}, 0, 1};

  EXPECT_NE(
      std::find_if(
          graph.tasks.begin(), graph.tasks.end(),
          [&](const ShelfTask& task) { return task.id == high_direct; }),
      graph.tasks.end());
  EXPECT_EQ(
      std::find_if(
          graph.tasks.begin(), graph.tasks.end(),
          [&](const ShelfTask& task) { return task.id == high_detour; }),
      graph.tasks.end());
}
