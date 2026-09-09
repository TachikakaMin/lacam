// PROTECTED regression: the final delivered-cost replay must not cross the
// hard deadline and then let the BRD controller report SOLVED.
#include <dd_planner.hpp>
#include <utils.hpp>

#include "gtest/gtest.h"

namespace {

DDInstance make_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.shelf_storage = {1, 0, 1};
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();
  return ins;
}

DDPlan make_deliverable_plan(const DDInstance& ins)
{
  return {
      {Op::make_lift()},
      {Op::make_move(ins.grid.idx(0, 1))},
      {Op::make_move(ins.grid.idx(0, 2))},
      {Op::make_drop()},
  };
}

}  // namespace

TEST(carrier_brd_final_cost_deadline,
     delivered_cost_replay_reports_cutoff)
{
  const auto ins = make_instance();
  const auto plan = make_deliverable_plan(ins);
  const Deadline expired(-1);
  bool cutoff = false;

  const auto cost = dd_plan_cost_deadline_probe(
      ins, plan, &expired, &cutoff);

  EXPECT_FALSE(cost.has_value());
  EXPECT_TRUE(cutoff);
}
