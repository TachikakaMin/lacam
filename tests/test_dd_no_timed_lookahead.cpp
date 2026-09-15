#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

TEST(dd_no_timed_lookahead,
     lacam_solves_and_replays_without_a_bounded_transport_plan)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {ins.grid.idx(1, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.target_goal_sets = {{ins.grid.idx(0, 4)}};
  ins.finalize();

  const auto result = solve_carrier_lacam_result(ins, 5.0, 0);
  ASSERT_TRUE(result.solved());
  ASSERT_FALSE(result.plan.empty());
  auto physical = initial_phys_config(ins);
  for (const auto& ops : result.plan) {
    const auto next = apply_ops(ins, physical, ops);
    ASSERT_TRUE(next.has_value());
    for (size_t robot = 0; robot < ops.size(); ++robot) {
      if (ops[robot].kind == Op::DROP) {
        EXPECT_TRUE(ins.can_store_shelf(physical.robots[robot]));
      }
    }
    physical = *next;
  }
  EXPECT_TRUE(is_dd_goal(ins, physical));
}
