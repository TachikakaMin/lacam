// PROTECTED fixed-tau carrier diagnostic probe.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

namespace {

DDInstance make_two_goal_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.robots = {ins.grid.idx(1, 2)};
  ins.shelves = {ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 2)};
  ins.target_goals = {ins.grid.idx(0, 0)};
  ins.target_goal_sets = {{
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 4),
  }};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_fixed_tau_probe,
     singleton_view_reuses_carrier_search_and_reaches_override)
{
  const auto ins = make_two_goal_instance();
  const std::vector<int> tau{ins.grid.idx(0, 4)};
  DDStats stats;

  const auto result =
      dd_solve_carrier_lacam_fixed_tau_probe(
          ins, tau, 3.0, 0, &stats);

  ASSERT_TRUE(result.solved());
  auto state = initial_phys_config(ins);
  for (const auto& ops : result.plan) {
    const auto next = apply_ops(ins, state, ops);
    ASSERT_TRUE(next.has_value());
    state = *next;
  }
  EXPECT_TRUE(is_dd_goal(ins, state));
  EXPECT_EQ(state.target_pos, tau);
  EXPECT_EQ(stats.best_makespan,
            static_cast<long>(result.plan.size()));
}
