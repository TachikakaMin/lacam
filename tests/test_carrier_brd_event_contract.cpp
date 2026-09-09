// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P5.
// Arbitrary physical-root and hard event-contract tests.
#include <dd_carrier.hpp>
#include <tapf_planner.hpp>

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
    std::vector<int> route,
    FixedTransferStartMode mode =
        FixedTransferStartMode::PROVISIONAL_FREE)
{
  return FixedRobotTransfer{
      robot,
      id,
      UpperShelfHandle{
          UpperShelfHandle::Kind::TARGET, target},
      ShelfSelector{
          ShelfSelector::Kind::TARGET, target},
      StorageTransfer{route.back(), std::move(route)},
      mode};
}

FixedRobotTransfer anonymous_transfer(
    int robot, FrozenTaskId id, int stable_id,
    std::vector<int> route,
    FixedTransferStartMode mode =
        FixedTransferStartMode::PROVISIONAL_FREE)
{
  const int source = route.front();
  return FixedRobotTransfer{
      robot,
      id,
      UpperShelfHandle{
          UpperShelfHandle::Kind::ANONYMOUS, stable_id},
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL, source},
      StorageTransfer{route.back(), std::move(route)},
      mode};
}

CarrierEventContract one_task_contract(
    const PhysConfig& start,
    const FixedRobotTransfer& fixed,
    ExecutionStatus status,
    std::optional<int> carrying_robot = std::nullopt)
{
  CarrierEventContract contract;
  contract.start = start;
  contract.wave_ledger_snapshot.push_back(
      TaskLedgerEntry{
          fixed.task_id, status, carrying_robot});
  contract.active_transfers.push_back(fixed);
  return contract;
}

}  // namespace

TEST(carrier_brd_event_contract,
     physical_root_accepts_target_and_anonymous_carrying)
{
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 0}, {0, 4}},
      {{0, 0}, {0, 4}}, {{{0, 0}, {0, 2}}});
  const auto initial = initial_phys_config(ins);
  const auto carrying = apply_ops(
      ins, initial, {Op::make_lift(), Op::make_lift()});
  ASSERT_TRUE(carrying.has_value());

  EXPECT_TRUE(validate_phys_config_root(ins, *carrying).valid());

  auto bad_size = *carrying;
  bad_size.kappa.pop_back();
  EXPECT_EQ(
      validate_phys_config_root(ins, bad_size).reason,
      PhysRootInvalidReason::VECTOR_SIZE);

  auto bad_target_anchor = *carrying;
  bad_target_anchor.target_pos[0] = ins.grid.idx(0, 1);
  EXPECT_EQ(
      validate_phys_config_root(ins, bad_target_anchor).reason,
      PhysRootInvalidReason::TARGET_CARRIER_MISMATCH);

  auto bad_cell = *carrying;
  bad_cell.robots[0] = ins.grid.size();
  EXPECT_EQ(
      validate_phys_config_root(ins, bad_cell).reason,
      PhysRootInvalidReason::INVALID_ROBOT_CELL);
}

TEST(carrier_brd_event_contract,
     target_phase_is_pure_approach_carrying_completed_or_invalid)
{
  const auto ins = make_instance(
      {"..."}, {"S.S"}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {0, 2}}});
  const FrozenTaskId id{0, 0};
  const auto fixed = target_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2)});
  const auto x0 = initial_phys_config(ins);
  EXPECT_EQ(
      carrier_event_phase_of(x0, fixed),
      (CarrierTaskPhase{
          CarrierTaskPhaseKind::APPROACH, -1}));

  const auto x1 = apply_ops(ins, x0, {Op::make_lift()});
  ASSERT_TRUE(x1.has_value());
  EXPECT_EQ(
      carrier_event_phase_of(*x1, fixed),
      (CarrierTaskPhase{
          CarrierTaskPhaseKind::CARRYING, 0}));
  const auto x2 = apply_ops(
      ins, *x1, {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(x2.has_value());
  EXPECT_EQ(
      carrier_event_phase_of(*x2, fixed),
      (CarrierTaskPhase{
          CarrierTaskPhaseKind::CARRYING, 1}));
  const auto x3 = apply_ops(
      ins, *x2, {Op::make_move(ins.grid.idx(0, 2))});
  ASSERT_TRUE(x3.has_value());
  EXPECT_EQ(
      carrier_event_phase_of(*x3, fixed),
      (CarrierTaskPhase{
          CarrierTaskPhaseKind::CARRYING, 2}));
  const auto x4 = apply_ops(ins, *x3, {Op::make_drop()});
  ASSERT_TRUE(x4.has_value());
  EXPECT_EQ(
      carrier_event_phase_of(*x4, fixed),
      (CarrierTaskPhase{
          CarrierTaskPhaseKind::COMPLETED, -1}));

  auto off_route = *x1;
  off_route.robots[0] = ins.grid.idx(0, 2);
  off_route.target_pos[0] = ins.grid.idx(0, 2);
  off_route.target_pos[0] = ins.grid.idx(0, 2);
  const auto wrong_route = target_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)});
  EXPECT_EQ(
      carrier_event_phase_of(off_route, wrong_route).kind,
      CarrierTaskPhaseKind::INVALID);
}

TEST(carrier_brd_event_contract,
     anonymous_phase_and_endpoint_carrying_root_are_valid)
{
  const auto ins = make_instance(
      {"..."}, {"S.S"}, {{0, 0}}, {{0, 0}}, {});
  const FrozenTaskId id{0, 0};
  auto fixed = anonymous_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2)});
  const auto x0 = initial_phys_config(ins);
  EXPECT_EQ(
      carrier_event_phase_of(x0, fixed).kind,
      CarrierTaskPhaseKind::APPROACH);
  const auto x1 = apply_ops(ins, x0, {Op::make_lift()});
  ASSERT_TRUE(x1.has_value());
  const auto x2 = apply_ops(
      ins, *x1, {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(x2.has_value());
  const auto x3 = apply_ops(
      ins, *x2, {Op::make_move(ins.grid.idx(0, 2))});
  ASSERT_TRUE(x3.has_value());
  EXPECT_EQ(
      carrier_event_phase_of(*x3, fixed),
      (CarrierTaskPhase{
          CarrierTaskPhaseKind::CARRYING, 2}));

  fixed.start_mode =
      FixedTransferStartMode::LOCKED_CARRYING;
  const auto contract = one_task_contract(
      *x3, fixed, ExecutionStatus::CARRYING, 0);
  EXPECT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const auto x4 = apply_ops(ins, *x3, {Op::make_drop()});
  ASSERT_TRUE(x4.has_value());
  EXPECT_EQ(
      carrier_event_phase_of(*x4, fixed).kind,
      CarrierTaskPhaseKind::COMPLETED);
}

TEST(carrier_brd_event_contract,
     ledger_and_active_transfers_must_be_bidirectionally_consistent)
{
  const auto ins = make_instance(
      {"..."}, {"S.S"}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {0, 2}}});
  const FrozenTaskId id{0, 0};
  const auto fixed = target_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2)});
  const auto x0 = initial_phys_config(ins);
  auto valid = one_task_contract(
      x0, fixed, ExecutionStatus::PENDING);
  EXPECT_TRUE(
      validate_carrier_event_contract(ins, valid).valid());

  auto duplicate = valid;
  duplicate.wave_ledger_snapshot.push_back(
      duplicate.wave_ledger_snapshot.front());
  EXPECT_EQ(
      validate_carrier_event_contract(ins, duplicate).reason,
      CarrierEventContractInvalidReason::INVALID_LEDGER);

  auto missing = valid;
  missing.wave_ledger_snapshot.clear();
  EXPECT_EQ(
      validate_carrier_event_contract(ins, missing).reason,
      CarrierEventContractInvalidReason::
          LEDGER_ACTIVE_MISMATCH);

  const auto lifted = apply_ops(ins, x0, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  auto unbound = valid;
  unbound.start = *lifted;
  EXPECT_EQ(
      validate_carrier_event_contract(ins, unbound).reason,
      CarrierEventContractInvalidReason::
          LEDGER_ACTIVE_MISMATCH);
}

TEST(carrier_brd_event_contract,
     transition_validator_rejects_wrong_lift_route_and_drop)
{
  const auto ins = make_instance(
      {"...", "..."}, {"S.S", "S.S"},
      {{0, 0}, {1, 2}}, {{0, 0}, {1, 2}},
      {{{0, 0}, {0, 2}}});
  const FrozenTaskId id{0, 0};
  const auto fixed = target_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2)});
  const auto x0 = initial_phys_config(ins);
  const auto contract = one_task_contract(
      x0, fixed, ExecutionStatus::PENDING);
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const std::vector<Op> wrong_lift{
      Op::make_wait(), Op::make_lift()};
  const auto wrong_lift_to =
      apply_ops(ins, x0, wrong_lift);
  ASSERT_TRUE(wrong_lift_to.has_value());
  EXPECT_FALSE(validate_carrier_event_transition(
      ins, contract, x0, wrong_lift, *wrong_lift_to));

  const std::vector<Op> lift{
      Op::make_lift(), Op::make_wait()};
  const auto x1 = apply_ops(ins, x0, lift);
  ASSERT_TRUE(x1.has_value());
  EXPECT_TRUE(validate_carrier_event_transition(
      ins, contract, x0, lift, *x1));

  const std::vector<Op> early_drop{
      Op::make_drop(), Op::make_wait()};
  const auto early_drop_to =
      apply_ops(ins, *x1, early_drop);
  ASSERT_TRUE(early_drop_to.has_value());
  EXPECT_FALSE(validate_carrier_event_transition(
      ins, contract, *x1, early_drop, *early_drop_to));

  const std::vector<Op> wrong_route{
      Op::make_move(ins.grid.idx(1, 0)),
      Op::make_wait()};
  const auto wrong_route_to =
      apply_ops(ins, *x1, wrong_route);
  ASSERT_TRUE(wrong_route_to.has_value());
  EXPECT_FALSE(validate_carrier_event_transition(
      ins, contract, *x1, wrong_route, *wrong_route_to));

  const std::vector<Op> correct_route{
      Op::make_move(ins.grid.idx(0, 1)),
      Op::make_wait()};
  const auto correct_route_to =
      apply_ops(ins, *x1, correct_route);
  ASSERT_TRUE(correct_route_to.has_value());
  EXPECT_TRUE(validate_carrier_event_transition(
      ins, contract, *x1, correct_route, *correct_route_to));
}

TEST(carrier_brd_event_contract,
     search_config_allows_normal_root_and_requires_exact_contract_pairing)
{
  const auto ins = make_instance(
      {"..."}, {"S.S"}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {0, 2}}});
  const TAPFInstance view(ins);
  const auto x0 = initial_phys_config(ins);
  const auto fixed = target_transfer(
      0, FrozenTaskId{0, 0}, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2)});
  const auto contract = one_task_contract(
      x0, fixed, ExecutionStatus::PENDING);
  std::mt19937 mt(0);

  TAPFSearchConfig root_without_contract;
  root_without_contract.initial_physical = x0;
  EXPECT_NO_THROW(
      TAPFPlanner(
          &view, nullptr, &mt, 0, 0, 0.001f, true, nullptr,
          root_without_contract));

  TAPFSearchConfig contract_without_root;
  contract_without_root.event_contract = &contract;
  EXPECT_THROW(
      TAPFPlanner(
          &view, nullptr, &mt, 0, 0, 0.001f, true, nullptr,
          contract_without_root),
      std::invalid_argument);

  TAPFSearchConfig mismatched;
  mismatched.event_contract = &contract;
  mismatched.initial_physical = x0;
  mismatched.initial_physical->robots[0] = ins.grid.idx(0, 1);
  EXPECT_THROW(
      TAPFPlanner(
          &view, nullptr, &mt, 0, 0, 0.001f, true, nullptr,
          mismatched),
      std::invalid_argument);

  auto invalid_contract = contract;
  invalid_contract.wave_ledger_snapshot.push_back(
      invalid_contract.wave_ledger_snapshot.front());
  TAPFSearchConfig invalid_contract_content;
  invalid_contract_content.event_contract =
      &invalid_contract;
  invalid_contract_content.initial_physical = x0;
  EXPECT_THROW(
      TAPFPlanner(
          &view, nullptr, &mt, 0, 0, 0.001f, true, nullptr,
          invalid_contract_content),
      std::invalid_argument);

  TAPFSearchConfig valid;
  valid.event_contract = &contract;
  valid.initial_physical = x0;
  EXPECT_NO_THROW(
      TAPFPlanner(
          &view, nullptr, &mt, 0, 0, 0.001f, true, nullptr,
          valid));

  EXPECT_NO_THROW(
      TAPFPlanner(&view, nullptr, &mt));
}
