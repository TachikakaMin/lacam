// PROTECTED regression test: a task recovered from the source rho binding
// immediately after Lift must become committed custody before objective
// conflict resolution.  Written TDD RED, 2026-09-02.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(dd_fresh_lift_claim, recovered_committed_drop_is_a_fixed_claim)
{
  DDInstance dd;
  dd.grid = DDGrid({"....."});
  dd.robots = {dd.grid.idx(0, 2)};
  dd.shelves = {dd.grid.idx(0, 2)};
  dd.target_starts = {dd.grid.idx(0, 2)};
  dd.target_goals = {dd.grid.idx(0, 4)};
  dd.finalize();

  PhysConfig state;
  state.robots = {dd.grid.idx(0, 2)};
  state.target_pos = {dd.grid.idx(0, 2)};
  state.kappa = {0};

  ManipulationTask task;
  task.shelf_target = 0;
  task.from = dd.grid.idx(0, 2);
  task.to = dd.grid.idx(0, 3);
  task.to_committed = true;
  task.root_target = 0;
  task.root_goal = dd.grid.idx(0, 4);
  task.roots = {{0, dd.grid.idx(0, 4)}};
  carrier_detail::normalize_task(task);

  CarrierGuidance source;
  source.tasks = {task};
  source.rho_task = {0};
  source.custody.resize(1);

  carrier_detail::Scratch scratch(dd.grid.size());
  carrier_detail::fill_occupancy(
      dd, state, scratch, std::vector<int>{dd.grid.idx(0, 4)});
  const auto custody = carrier_detail::recover_inflight_custody(
      dd, state, &source, scratch);

  ASSERT_EQ(custody.size(), 1u);
  ASSERT_NE(custody[0].id, 0u);
  EXPECT_TRUE(custody[0].to_committed);
  EXPECT_EQ(custody[0].to, dd.grid.idx(0, 3));
  EXPECT_EQ(carrier_detail::task_hard_claims(custody[0]),
            (std::vector<int>{dd.grid.idx(0, 2), dd.grid.idx(0, 3)}));
}
