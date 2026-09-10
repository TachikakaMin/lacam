// PROTECTED REGRESSION: complete lower-plan certification followed by
// earliest-Drop prefix execution and global replanning.
#include <br_lacam_upper.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>
#include <utils.hpp>

#include <optional>
#include <random>
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

FixedRobotTransfer locked_target(
    int robot, FrozenTaskId id, int target_id,
    const std::vector<int>& route)
{
  return FixedRobotTransfer{
      robot,
      id,
      target(target_id),
      ShelfSelector{
          ShelfSelector::Kind::TARGET, target_id},
      StorageTransfer{route.back(), route},
      FixedTransferStartMode::LOCKED_CARRYING};
}

CarrierEventContract carrying_contract(
    const PhysConfig& start,
    const std::vector<FixedRobotTransfer>& active)
{
  CarrierEventContract contract;
  contract.start = start;
  contract.active_transfers = active;
  for (const auto& fixed : active)
    contract.wave_ledger_snapshot.push_back(
        TaskLedgerEntry{
            fixed.task_id, ExecutionStatus::CARRYING,
            fixed.robot});
  return contract;
}

}  // namespace

TEST(carrier_brd_complete_replan_controller,
     commits_through_earliest_drop_then_replans_every_agent)
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
              {
                  ins.grid.idx(1, 0),
                  ins.grid.idx(1, 1),
                  ins.grid.idx(1, 2),
                  ins.grid.idx(1, 3),
                  tau[1],
              }),
      },
      tau);
  CarrierBRDStats brd;

  const auto result = dd_execute_frozen_task_plan_probe(
      ins, frozen, tau, 5.0, 0, nullptr, &brd);

  ASSERT_TRUE(result.solved());
  ASSERT_EQ(brd.completion_events, 2);
  ASSERT_EQ(brd.segments, 2);
  ASSERT_EQ(brd.dispatches.size(), 2u);
  EXPECT_EQ(brd.dispatches[1].locked_pairs, 1);

  PhysConfig state = initial_phys_config(ins);
  size_t first_drop_step = result.plan.size();
  for (size_t step = 0; step < result.plan.size(); ++step) {
    const auto next = apply_ops(ins, state, result.plan[step]);
    ASSERT_TRUE(next.has_value());
    if (first_drop_step == result.plan.size() &&
        std::any_of(
            result.plan[step].begin(), result.plan[step].end(),
            [](const Op& op) { return op.kind == Op::DROP; })) {
      first_drop_step = step;
      EXPECT_EQ(next->kappa[0], KAPPA_FREE);
      EXPECT_NE(next->kappa[1], KAPPA_FREE);
    }
    state = *next;
  }
  ASSERT_LT(first_drop_step + 1, result.plan.size());
  EXPECT_TRUE(is_dd_goal(ins, state));
}

TEST(carrier_brd_complete_replan_controller,
     incomplete_lower_search_does_not_return_a_legal_drop_prefix)
{
  const auto ins = make_instance(
      {"....."}, {"SSSSS"},
      {{0, 1}, {0, 4}}, {{0, 1}, {0, 4}},
      {{{0, 1}, {0, 2}}, {{0, 4}, {0, 0}}});
  const auto lifted = apply_ops(
      ins, initial_phys_config(ins),
      {Op::make_lift(), Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  const auto short_task = locked_target(
      0, FrozenTaskId{0, 0}, 0,
      {ins.grid.idx(0, 1), ins.grid.idx(0, 2)});
  const auto blocked_task = locked_target(
      1, FrozenTaskId{0, 1}, 1,
      {
          ins.grid.idx(0, 4),
          ins.grid.idx(0, 3),
          ins.grid.idx(0, 2),
          ins.grid.idx(0, 1),
          ins.grid.idx(0, 0),
      });
  const auto contract =
      carrying_contract(*lifted, {short_task, blocked_task});
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const auto moved = apply_ops(
      ins, *lifted,
      {
          Op::make_move(ins.grid.idx(0, 2)),
          Op::make_wait(),
      });
  ASSERT_TRUE(moved.has_value());
  ASSERT_TRUE(validate_carrier_event_transition(
      ins, contract, *lifted,
      {
          Op::make_move(ins.grid.idx(0, 2)),
          Op::make_wait(),
      },
      *moved));
  const auto early_drop = apply_ops(
      ins, *moved, {Op::make_drop(), Op::make_wait()});
  ASSERT_TRUE(early_drop.has_value());
  ASSERT_TRUE(validate_carrier_event_transition(
      ins, contract, *moved,
      {Op::make_drop(), Op::make_wait()}, *early_drop));

  const TAPFInstance view(ins);
  Deadline deadline(2000);
  std::mt19937 mt(0);
  TAPFStats stats;
  TAPFSearchConfig config;
  config.initial_physical = contract.start;
  config.event_contract = &contract;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  config.stop_policy = TAPFStopPolicy::FIRST_FEASIBLE;
  config.macro_enabled = false;
  TAPFPlanner planner(
      &view, &deadline, &mt, 0, 0, 0.001f,
      false, &stats, config);

  const auto solution = planner.solve();

  EXPECT_TRUE(solution.empty());
  EXPECT_FALSE(stats.timed_out);
  EXPECT_TRUE(planner.solution_shelves.empty());
}

TEST(carrier_brd_complete_replan_controller,
     completed_robot_cannot_lift_or_drop_again)
{
  const auto ins = make_instance(
      {".....", "....."}, {"SSSSS", "SSSSS"},
      {{0, 0}, {1, 0}}, {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 4}}});
  const auto short_task = locked_target(
      0, FrozenTaskId{0, 0}, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)});
  const auto long_task = locked_target(
      1, FrozenTaskId{0, 1}, 1,
      {
          ins.grid.idx(1, 0),
          ins.grid.idx(1, 1),
          ins.grid.idx(1, 2),
          ins.grid.idx(1, 3),
          ins.grid.idx(1, 4),
      });
  const auto lifted = apply_ops(
      ins, initial_phys_config(ins),
      {Op::make_lift(), Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  const auto contract =
      carrying_contract(*lifted, {short_task, long_task});
  auto state = *lifted;
  for (const auto& ops : std::vector<std::vector<Op>>{
           {
               Op::make_move(ins.grid.idx(0, 1)),
               Op::make_move(ins.grid.idx(1, 1)),
           },
           {
               Op::make_drop(),
               Op::make_move(ins.grid.idx(1, 2)),
           },
       }) {
    const auto next = apply_ops(ins, state, ops);
    ASSERT_TRUE(next.has_value());
    ASSERT_TRUE(validate_carrier_event_transition(
        ins, contract, state, ops, *next));
    state = *next;
  }
  ASSERT_EQ(
      carrier_event_phase_of(state, short_task).kind,
      CarrierTaskPhaseKind::COMPLETED);

  const std::vector<Op> relift{
      Op::make_lift(), Op::make_wait()};
  const auto relifted = apply_ops(ins, state, relift);
  ASSERT_TRUE(relifted.has_value());
  EXPECT_FALSE(validate_carrier_event_transition(
      ins, contract, state, relift, *relifted));

  EXPECT_FALSE(
      apply_ops(
          ins, state,
          {Op::make_drop(), Op::make_wait()})
          .has_value());
}
