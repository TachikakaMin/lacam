// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P7.
// Completion-event controller tests written before implementation.
#include <br_lacam_upper.hpp>
#include <dd_planner.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Cell = std::pair<int, int>;

DDInstance make_instance(
    const std::vector<std::string>& rows,
    const std::vector<std::string>& storage,
    const std::vector<Cell>& robots,
    const std::vector<Cell>& shelves,
    const std::vector<std::pair<Cell, Cell>>& targets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (int r = 0; r < ins.grid.height; ++r)
    for (int c = 0; c < ins.grid.width; ++c)
      ins.shelf_storage[ins.grid.idx(r, c)] =
          storage[r][c] == 'S';
  for (const auto& [r, c] : robots)
    ins.robots.push_back(ins.grid.idx(r, c));
  for (const auto& [r, c] : shelves)
    ins.shelves.push_back(ins.grid.idx(r, c));
  for (const auto& [start, goal] : targets) {
    ins.target_starts.push_back(
        ins.grid.idx(start.first, start.second));
    ins.target_goals.push_back(
        ins.grid.idx(goal.first, goal.second));
  }
  ins.finalize();
  return ins;
}

UpperShelfHandle target(int id)
{
  return UpperShelfHandle{
      UpperShelfHandle::Kind::TARGET, id};
}

BRUpperConstraintEntry transfer(
    UpperShelfHandle shelf, int endpoint,
    std::vector<int> route)
{
  return BRUpperConstraintEntry::make_transfer(
      shelf, StorageTransfer{endpoint, std::move(route)});
}

FrozenTaskPlan one_wave_plan(
    const DDInstance& ins,
    const std::vector<BRUpperConstraintEntry>& action,
    const std::vector<int>& tau)
{
  const auto start = br_labeled_initial_state(ins);
  const auto successor =
      validate_complete_upper_action(ins, start, action);
  EXPECT_TRUE(successor.has_value());
  BRUpperSearchResult upper;
  upper.exit_reason = BRUpperExitReason::SOLVED;
  upper.states = {start, successor->state};
  upper.transitions = {successor->transition};
  const auto frozen =
      compile_frozen_task_plan(ins, upper, tau);
  EXPECT_TRUE(frozen.has_value());
  return *frozen;
}

PhysConfig replay(
    const DDInstance& ins, const DDPlan& plan)
{
  PhysConfig state = initial_phys_config(ins);
  for (const auto& ops : plan) {
    const auto next = apply_ops(ins, state, ops);
    EXPECT_TRUE(next.has_value());
    if (!next.has_value()) break;
    state = *next;
  }
  return state;
}

}  // namespace

TEST(carrier_brd_controller,
     initial_goal_returns_solved_empty_plan)
{
  const auto ins = make_instance(
      {"."}, {"S"}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {0, 0}}});
  CarrierBRDStats brd;

  const auto result =
      solve_carrier_brd_result(
          ins, 2.0, 0, nullptr, &brd);

  EXPECT_TRUE(result.solved());
  EXPECT_TRUE(result.plan.empty());
  EXPECT_EQ(brd.exit_reason, CarrierBRDExitReason::SOLVED);
  EXPECT_EQ(brd.dispatch_epochs, 0);
}

TEST(carrier_brd_controller,
     full_solver_uses_production_root_tau_and_delivers_goal)
{
  const auto ins = make_instance(
      {"...."}, {"S..S"}, {{0, 1}}, {{0, 0}},
      {{{0, 0}, {0, 3}}});
  const auto expected_tau =
      dd_lazy_tau_guide_probe(
          ins, initial_phys_config(ins))
          .tau;
  CarrierBRDStats brd;

  const auto result =
      solve_carrier_brd_result(
          ins, 5.0, 0, nullptr, &brd);

  ASSERT_TRUE(result.solved());
  EXPECT_EQ(brd.tau0, expected_tau);
  EXPECT_TRUE(is_dd_goal(ins, replay(ins, result.plan)));
  EXPECT_TRUE(brd.raw_plan_valid);
  EXPECT_EQ(brd.frozen_waves, 1);
  EXPECT_GE(brd.dispatch_epochs, 1);
}

TEST(carrier_brd_controller,
     every_completion_rematches_all_free_robots_and_pending_tasks)
{
  const auto ins = make_instance(
      {"...", "..."}, {"SSS", "SSS"},
      {{0, 0}}, {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 1}}});
  const std::vector<int> tau{
      ins.grid.idx(0, 1), ins.grid.idx(1, 1)};
  const auto frozen = one_wave_plan(
      ins,
      {
          transfer(
              target(0), tau[0],
              {ins.grid.idx(0, 0), tau[0]}),
          transfer(
              target(1), tau[1],
              {ins.grid.idx(1, 0), tau[1]}),
      },
      tau);
  CarrierBRDStats brd;

  const auto result = dd_execute_frozen_task_plan_probe(
      ins, frozen, tau, 5.0, 0, nullptr, &brd);

  ASSERT_TRUE(result.solved());
  ASSERT_EQ(brd.dispatches.size(), 2u);
  EXPECT_EQ(brd.dispatches[0].free_robots, 1);
  EXPECT_EQ(brd.dispatches[0].pending_tasks, 2);
  EXPECT_EQ(brd.dispatches[0].locked_pairs, 0);
  EXPECT_EQ(brd.dispatches[0].matcher_rows, 2);
  EXPECT_EQ(brd.dispatches[0].real_assignments, 1);
  EXPECT_EQ(brd.dispatches[1].free_robots, 1);
  EXPECT_EQ(brd.dispatches[1].pending_tasks, 1);
  EXPECT_EQ(brd.dispatches[1].locked_pairs, 0);
  EXPECT_EQ(brd.dispatches[1].matcher_rows, 1);
  EXPECT_EQ(brd.dispatches[1].real_assignments, 1);
  EXPECT_EQ(brd.completion_events, 2);
  EXPECT_TRUE(is_dd_goal(ins, replay(ins, result.plan)));
}

TEST(carrier_brd_controller,
     lifted_robot_task_pair_stays_locked_across_completion_event)
{
  const auto ins = make_instance(
      {".....", "....."}, {"SS...", "S...S"},
      {{0, 0}, {1, 0}}, {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 4}}});
  const std::vector<int> tau{
      ins.grid.idx(0, 1), ins.grid.idx(1, 4)};
  const auto frozen = one_wave_plan(
      ins,
      {
          transfer(
              target(0), tau[0],
              {ins.grid.idx(0, 0), tau[0]}),
          transfer(
              target(1), tau[1],
              {ins.grid.idx(1, 0), ins.grid.idx(1, 1),
               ins.grid.idx(1, 2), ins.grid.idx(1, 3),
               tau[1]}),
      },
      tau);
  CarrierBRDStats brd;

  const auto result = dd_execute_frozen_task_plan_probe(
      ins, frozen, tau, 5.0, 0, nullptr, &brd);

  ASSERT_TRUE(result.solved());
  ASSERT_GE(brd.dispatches.size(), 2u);
  EXPECT_EQ(brd.dispatches[0].locked_pairs, 0);
  EXPECT_EQ(brd.dispatches[0].pending_tasks, 2);
  EXPECT_EQ(brd.dispatches[1].locked_pairs, 1);
  EXPECT_EQ(brd.dispatches[1].pending_tasks, 0);
  EXPECT_EQ(brd.dispatches[1].free_robots, 1);
  EXPECT_EQ(brd.locked_pair_continuations, 1);
  EXPECT_TRUE(is_dd_goal(ins, replay(ins, result.plan)));
}
