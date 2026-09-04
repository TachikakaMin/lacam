// PROTECTED regression test: explicit reguide statistics must compare
// against a live snapshot of the old guidance.  Written TDD RED,
// 2026-09-02.
#include <dd_carrier.hpp>
#include <tapf_planner.hpp>

#include <random>

#include "gtest/gtest.h"

TEST(dd_reguide_stats, rho_change_count_uses_live_old_guidance)
{
  DDInstance dd;
  dd.grid = DDGrid({".....", "....."});
  dd.robots = {dd.grid.idx(1, 0), dd.grid.idx(1, 4)};
  dd.shelves = {dd.grid.idx(0, 1), dd.grid.idx(0, 3)};
  dd.target_starts = dd.shelves;
  dd.target_goals = {dd.grid.idx(0, 4), dd.grid.idx(0, 0)};
  dd.finalize();

  const TAPFInstance view(dd);
  std::mt19937 mt(0);
  TAPFStats stats;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &stats);
  auto node = std::make_unique<TAPFNode>(
      view.starts, initial_shelf_state(view), planner.D, &view,
      std::vector<int>(view.N, -1), TAPFAssignmentState(), nullptr);
  planner.attach_carrier_guidance(node.get());
  ASSERT_NE(node->guide, nullptr);
  const auto old_free_goal = node->guide->free_goal;
  const long before = stats.rho_change_builds;

  planner.attach_carrier_guidance(node.get(), /*reguide=*/true);

  ASSERT_NE(node->guide, nullptr);
  ASSERT_EQ(old_free_goal.size(), node->guide->free_goal.size());
  bool changed = false;
  for (size_t i = 0; i < old_free_goal.size(); ++i)
    changed |= old_free_goal[i] != node->guide->free_goal[i];
  EXPECT_EQ(stats.rho_change_builds - before, changed ? 1 : 0);
}
