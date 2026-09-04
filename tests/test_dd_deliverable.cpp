// PROTECTED v4.1 deliverable deadline test.  Written TDD RED, 2026-09-02.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

TEST(dd_deliverable, successful_plan_is_replayed_and_timestamped_in_budget)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {ins.grid.idx(1, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();

  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 1.0, 0, &stats);
  ASSERT_FALSE(plan.empty());
  EXPECT_GE(stats.deliverable_ms, 0);
  EXPECT_LE(stats.deliverable_ms, 1000)
      << "timestamp is after final C++ replay/SOC selection and must fit "
         "the owning deadline";
}
