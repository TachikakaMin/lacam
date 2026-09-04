// PROTECTED regression test: an in-flight committed custody episode must
// survive livelock/revisit re-guidance.  Written TDD RED, 2026-09-02.
#include <dd_carrier.hpp>
#include <tapf_planner.hpp>

#include <random>

#include "gtest/gtest.h"

TEST(dd_reguide_custody, committed_inflight_survives_livelock_reguide)
{
  DDInstance dd;
  dd.grid = DDGrid({"....."});
  dd.robots = {dd.grid.idx(0, 2)};
  dd.shelves = {dd.grid.idx(0, 0)};
  dd.target_starts = {dd.grid.idx(0, 0)};
  dd.target_goals = {dd.grid.idx(0, 4)};
  dd.finalize();

  const TAPFInstance view(dd);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);

  Config C{view.G.U[dd.grid.idx(0, 2)]};
  ShelfState S;
  S.target_pos = {dd.grid.idx(0, 2)};
  S.kappa = {0};
  auto node = std::make_unique<TAPFNode>(
      C, S, planner.D, &view, std::vector<int>(view.N, -1),
      TAPFAssignmentState(), nullptr);
  planner.attach_carrier_guidance(node.get());
  ASSERT_NE(node->guide, nullptr);
  ASSERT_EQ(node->guide->custody.size(), 1u);

  ManipulationTask committed;
  committed.shelf_target = 0;
  committed.from = dd.grid.idx(0, 0);
  committed.to = dd.grid.idx(0, 3);
  committed.to_committed = true;
  committed.root_target = 0;
  committed.root_goal = dd.grid.idx(0, 4);
  committed.roots = {{0, dd.grid.idx(0, 4)}};
  committed.priority = 50;
  committed.depth = 1;
  committed.id = 0x5a17u;
  node->guide->custody[0] = committed;

  planner.attach_carrier_guidance(node.get(), /*reguide=*/true);

  ASSERT_NE(node->guide, nullptr);
  ASSERT_EQ(node->guide->custody.size(), 1u);
  const auto& after = node->guide->custody[0];
  EXPECT_EQ(after.id, committed.id);
  EXPECT_EQ(after.shelf_target, committed.shelf_target);
  EXPECT_EQ(after.from, committed.from);
  EXPECT_EQ(after.to, committed.to);
  EXPECT_TRUE(after.to_committed);
  EXPECT_EQ(after.roots, committed.roots);
}
