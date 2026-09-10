// PROTECTED REGRESSION: Carrier-BRD complete-plan certification and
// earliest-Drop prefix execution.
#include <dd_carrier.hpp>
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

FixedRobotTransfer target_transfer(
    int robot, FrozenTaskId id, int target,
    const std::vector<int>& route,
    FixedTransferStartMode mode)
{
  return FixedRobotTransfer{
      robot,
      id,
      UpperShelfHandle{
          UpperShelfHandle::Kind::TARGET, target},
      ShelfSelector{
          ShelfSelector::Kind::TARGET, target},
      StorageTransfer{route.back(), route},
      mode};
}

CarrierEventContract contract_for(
    const PhysConfig& start,
    const std::vector<FixedRobotTransfer>& active,
    const std::vector<ExecutionStatus>& status)
{
  CarrierEventContract contract;
  contract.start = start;
  contract.active_transfers = active;
  for (size_t i = 0; i < active.size(); ++i) {
    const bool carrying =
        status[i] == ExecutionStatus::CARRYING;
    contract.wave_ledger_snapshot.push_back(
        TaskLedgerEntry{
            active[i].task_id, status[i],
            carrying
                ? std::optional<int>(active[i].robot)
                : std::nullopt});
  }
  return contract;
}

struct ContractRun {
  Solution solution;
  std::vector<std::vector<Op>> plan;
  PhysConfig final;
};

ContractRun run_contract(
    const DDInstance& ins,
    const CarrierEventContract& contract,
    double timeout_ms = 5000)
{
  const TAPFInstance view(ins);
  Deadline deadline(timeout_ms);
  std::mt19937 mt(0);
  TAPFSearchConfig config;
  config.initial_physical = contract.start;
  config.event_contract = &contract;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  config.stop_policy = TAPFStopPolicy::FIRST_FEASIBLE;
  config.macro_enabled = false;
  TAPFPlanner planner(
      &view, &deadline, &mt, 0, 0, 0.001f,
      false, nullptr, config);

  ContractRun out;
  out.solution = planner.solve();
  out.final = contract.start;
  if (out.solution.empty()) return out;
  EXPECT_EQ(
      planner.solution_shelves.size(),
      out.solution.size());
  out.plan = derive_carrier_ops(
      view, out.solution, planner.solution_shelves);
  for (const auto& ops : out.plan) {
    const auto next = apply_ops(ins, out.final, ops);
    EXPECT_TRUE(next.has_value());
    if (!next.has_value()) break;
    EXPECT_TRUE(validate_carrier_event_transition(
        ins, contract, out.final, ops, *next));
    out.final = *next;
  }
  return out;
}

}  // namespace

TEST(carrier_brd_complete_replan,
     carrying_move_may_leave_the_canonical_route)
{
  const auto ins = make_instance(
      {"...", "...", "..."},
      {"SSS", "SSS", "SSS"},
      {{1, 1}}, {{1, 1}},
      {{{1, 1}, {0, 2}}});
  const auto root = initial_phys_config(ins);
  const auto lifted =
      apply_ops(ins, root, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  const auto fixed = target_transfer(
      0, FrozenTaskId{0, 0}, 0,
      {
          ins.grid.idx(1, 1),
          ins.grid.idx(1, 2),
          ins.grid.idx(0, 2),
      },
      FixedTransferStartMode::LOCKED_CARRYING);
  const auto contract = contract_for(
      *lifted, {fixed}, {ExecutionStatus::CARRYING});
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const std::vector<Op> detour{
      Op::make_move(ins.grid.idx(2, 1))};
  const auto detoured = apply_ops(ins, *lifted, detour);
  ASSERT_TRUE(detoured.has_value());

  EXPECT_TRUE(validate_carrier_event_transition(
      ins, contract, *lifted, detour, *detoured));
  EXPECT_EQ(
      carrier_event_phase_of(*detoured, fixed).kind,
      CarrierTaskPhaseKind::CARRYING);

  const std::vector<Op> early_drop{Op::make_drop()};
  const auto dropped_at_source =
      apply_ops(ins, *lifted, early_drop);
  ASSERT_TRUE(dropped_at_source.has_value());
  EXPECT_FALSE(validate_carrier_event_transition(
      ins, contract, *lifted, early_drop,
      *dropped_at_source));
}

TEST(carrier_brd_complete_replan,
     lower_solve_certifies_all_active_transfers)
{
  const auto ins = make_instance(
      {".....", "....."},
      {"SS...", "S...S"},
      {{0, 0}, {1, 0}},
      {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 4}}});
  const auto root = initial_phys_config(ins);
  const auto short_task = target_transfer(
      0, FrozenTaskId{0, 0}, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
      FixedTransferStartMode::PROVISIONAL_FREE);
  const auto long_task = target_transfer(
      1, FrozenTaskId{0, 1}, 1,
      {
          ins.grid.idx(1, 0),
          ins.grid.idx(1, 1),
          ins.grid.idx(1, 2),
          ins.grid.idx(1, 3),
          ins.grid.idx(1, 4),
      },
      FixedTransferStartMode::PROVISIONAL_FREE);
  const auto contract = contract_for(
      root, {short_task, long_task},
      {ExecutionStatus::PENDING,
       ExecutionStatus::PENDING});
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const auto run = run_contract(ins, contract);

  ASSERT_FALSE(run.solution.empty());
  EXPECT_EQ(
      carrier_event_phase_of(run.final, short_task).kind,
      CarrierTaskPhaseKind::COMPLETED);
  EXPECT_EQ(
      carrier_event_phase_of(run.final, long_task).kind,
      CarrierTaskPhaseKind::COMPLETED);
}

TEST(carrier_brd_complete_replan,
     completed_robot_may_move_while_other_transfer_remains_active)
{
  const auto ins = make_instance(
      {".....", "....."},
      {"SSSSS", "SSSSS"},
      {{0, 0}, {1, 0}},
      {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 4}}});
  const auto root = initial_phys_config(ins);
  const auto short_task = target_transfer(
      0, FrozenTaskId{0, 0}, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
      FixedTransferStartMode::PROVISIONAL_FREE);
  const auto long_task = target_transfer(
      1, FrozenTaskId{0, 1}, 1,
      {
          ins.grid.idx(1, 0),
          ins.grid.idx(1, 1),
          ins.grid.idx(1, 2),
          ins.grid.idx(1, 3),
          ins.grid.idx(1, 4),
      },
      FixedTransferStartMode::PROVISIONAL_FREE);
  const auto contract = contract_for(
      root, {short_task, long_task},
      {ExecutionStatus::PENDING,
       ExecutionStatus::PENDING});

  auto state = root;
  for (const auto& ops : std::vector<std::vector<Op>>{
           {Op::make_lift(), Op::make_lift()},
           {Op::make_move(ins.grid.idx(0, 1)),
            Op::make_move(ins.grid.idx(1, 1))},
           {Op::make_drop(),
            Op::make_move(ins.grid.idx(1, 2))},
       }) {
    const auto next = apply_ops(ins, state, ops);
    ASSERT_TRUE(next.has_value());
    state = *next;
  }
  ASSERT_EQ(
      carrier_event_phase_of(state, short_task).kind,
      CarrierTaskPhaseKind::COMPLETED);
  ASSERT_EQ(
      carrier_event_phase_of(state, long_task).kind,
      CarrierTaskPhaseKind::CARRYING);

  const std::vector<Op> clear_the_way{
      Op::make_move(ins.grid.idx(0, 2)),
      Op::make_wait()};
  const auto next =
      apply_ops(ins, state, clear_the_way);
  ASSERT_TRUE(next.has_value());
  EXPECT_TRUE(validate_carrier_event_transition(
      ins, contract, state, clear_the_way, *next));
}

TEST(carrier_brd_complete_replan,
     opposing_route_hints_with_a_bypass_are_globally_solvable)
{
  const auto ins = make_instance(
      {"...", "...", "...", "...", "..."},
      {"SS.", "...", "...", "...", "S.S"},
      {{0, 1}, {4, 0}},
      {{0, 1}, {4, 0}},
      {{{0, 1}, {4, 2}}, {{4, 0}, {0, 0}}});
  auto root = initial_phys_config(ins);
  const auto lifted = apply_ops(
      ins, root, {Op::make_lift(), Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  const auto moved_once = apply_ops(
      ins, *lifted,
      {
          Op::make_move(ins.grid.idx(1, 1)),
          Op::make_move(ins.grid.idx(4, 1)),
      });
  ASSERT_TRUE(moved_once.has_value());
  const auto opposed = apply_ops(
      ins, *moved_once,
      {
          Op::make_move(ins.grid.idx(2, 1)),
          Op::make_move(ins.grid.idx(3, 1)),
      });
  ASSERT_TRUE(opposed.has_value());

  const auto down = target_transfer(
      0, FrozenTaskId{0, 0}, 0,
      {
          ins.grid.idx(0, 1),
          ins.grid.idx(1, 1),
          ins.grid.idx(2, 1),
          ins.grid.idx(3, 1),
          ins.grid.idx(4, 1),
          ins.grid.idx(4, 2),
      },
      FixedTransferStartMode::LOCKED_CARRYING);
  const auto up = target_transfer(
      1, FrozenTaskId{0, 1}, 1,
      {
          ins.grid.idx(4, 0),
          ins.grid.idx(4, 1),
          ins.grid.idx(3, 1),
          ins.grid.idx(2, 1),
          ins.grid.idx(1, 1),
          ins.grid.idx(1, 0),
          ins.grid.idx(0, 0),
      },
      FixedTransferStartMode::LOCKED_CARRYING);
  const auto contract = contract_for(
      *opposed, {down, up},
      {ExecutionStatus::CARRYING,
       ExecutionStatus::CARRYING});
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const auto run = run_contract(ins, contract);

  ASSERT_FALSE(run.solution.empty());
  EXPECT_EQ(
      carrier_event_phase_of(run.final, down).kind,
      CarrierTaskPhaseKind::COMPLETED);
  EXPECT_EQ(
      carrier_event_phase_of(run.final, up).kind,
      CarrierTaskPhaseKind::COMPLETED);
}
