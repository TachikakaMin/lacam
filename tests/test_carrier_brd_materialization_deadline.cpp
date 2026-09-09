// PROTECTED regression: BRD task materialization and final plan replay must
// poll the same shared deadline inside their loops, not only after returning.
#include <br_lacam_upper.hpp>
#include <dd_planner.hpp>
#include <utils.hpp>

#include <utility>
#include <vector>

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

BRUpperSearchResult make_upper_result(
    const DDInstance& ins)
{
  const auto start = br_labeled_initial_state(ins);
  const auto successor = validate_complete_upper_action(
      ins, start,
      {BRUpperConstraintEntry::make_transfer(
          UpperShelfHandle{
              UpperShelfHandle::Kind::TARGET, 0},
          StorageTransfer{
              ins.grid.idx(0, 2),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
               ins.grid.idx(0, 2)}})});
  EXPECT_TRUE(successor.has_value());

  BRUpperSearchResult upper;
  upper.exit_reason = BRUpperExitReason::SOLVED;
  upper.states = {start, successor->state};
  upper.transitions = {successor->transition};
  return upper;
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

TEST(carrier_brd_materialization_deadline,
     frozen_compile_and_replay_report_cutoff)
{
  const auto ins = make_instance();
  const auto upper = make_upper_result(ins);
  const std::vector<int> tau{ins.grid.idx(0, 2)};

  const auto valid =
      compile_frozen_task_plan(ins, upper, tau);
  ASSERT_TRUE(valid.has_value());

  const Deadline expired(-1);
  bool compile_cutoff = false;
  const auto compiled = compile_frozen_task_plan(
      ins, upper, tau, &expired, &compile_cutoff);
  EXPECT_FALSE(compiled.has_value());
  EXPECT_TRUE(compile_cutoff);

  bool replay_cutoff = false;
  EXPECT_FALSE(replay_frozen_task_plan(
      ins, *valid, tau, nullptr, &expired, &replay_cutoff));
  EXPECT_TRUE(replay_cutoff);
}

TEST(carrier_brd_materialization_deadline,
     raw_replay_and_goal_prefix_report_cutoff)
{
  const auto ins = make_instance();
  const auto plan = make_deliverable_plan(ins);
  const Deadline expired(-1);

  bool replay_cutoff = false;
  EXPECT_FALSE(dd_replay_raw_prefix_deadline_probe(
      ins, plan, &expired, &replay_cutoff));
  EXPECT_TRUE(replay_cutoff);

  bool prefix_cutoff = false;
  const auto prefix = dd_normalize_goal_prefix_deadline_probe(
      ins, plan, &expired, &prefix_cutoff);
  EXPECT_FALSE(prefix.has_value());
  EXPECT_TRUE(prefix_cutoff);
}
