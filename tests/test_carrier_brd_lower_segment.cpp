// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P6.
// Fixed-contract lower-segment and first-completion tests.
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

struct SegmentRun {
  Solution solution;
  std::vector<std::vector<Op>> plan;
  PhysConfig final;
  TAPFStats stats;
};

SegmentRun run_segment(
    const DDInstance& ins,
    const CarrierEventContract& contract,
    int seed = 0)
{
  const TAPFInstance view(ins);
  Deadline deadline(5000);
  std::mt19937 mt(seed);
  TAPFSearchConfig config;
  config.initial_physical = contract.start;
  config.event_contract = &contract;
  config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
  config.stop_policy = TAPFStopPolicy::FIRST_FEASIBLE;
  config.macro_enabled = false;
  TAPFStats stats;
  TAPFPlanner planner(
      &view, &deadline, &mt, 0, 0, 0.001f,
      false, &stats, config);
  SegmentRun out;
  out.solution = planner.solve();
  out.stats = stats;
  out.final = contract.start;
  if (!out.solution.empty()) {
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
  }
  return out;
}

}  // namespace

TEST(carrier_brd_lower_segment,
     single_transfer_runs_approach_lift_route_and_drop)
{
  const auto ins = make_instance(
      {"...."}, {"S..S"}, {{0, 1}}, {{0, 0}},
      {{{0, 0}, {0, 3}}});
  const FrozenTaskId id{0, 0};
  const auto fixed = target_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2), ins.grid.idx(0, 3)});
  CarrierEventContract contract;
  contract.start = initial_phys_config(ins);
  contract.wave_ledger_snapshot = {
      TaskLedgerEntry{
          id, ExecutionStatus::PENDING, std::nullopt}};
  contract.active_transfers = {fixed};
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const auto run = run_segment(ins, contract);

  ASSERT_FALSE(run.solution.empty());
  ASSERT_FALSE(run.plan.empty());
  EXPECT_EQ(
      carrier_event_phase_of(run.final, fixed).kind,
      CarrierTaskPhaseKind::COMPLETED);
  EXPECT_EQ(run.plan.back()[0].kind, Op::DROP);
}

TEST(carrier_brd_lower_segment,
     solve_stops_at_first_drop_without_waiting_for_wave_completion)
{
  const auto ins = make_instance(
      {".....", "....."}, {"SS...", "S...S"},
      {{0, 0}, {1, 0}}, {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 4}}});
  const FrozenTaskId short_id{0, 0};
  const FrozenTaskId long_id{0, 1};
  const auto short_task = target_transfer(
      0, short_id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)});
  const auto long_task = target_transfer(
      1, long_id, 1,
      {ins.grid.idx(1, 0), ins.grid.idx(1, 1),
       ins.grid.idx(1, 2), ins.grid.idx(1, 3),
       ins.grid.idx(1, 4)});
  CarrierEventContract contract;
  contract.start = initial_phys_config(ins);
  contract.wave_ledger_snapshot = {
      TaskLedgerEntry{
          short_id, ExecutionStatus::PENDING, std::nullopt},
      TaskLedgerEntry{
          long_id, ExecutionStatus::PENDING, std::nullopt},
  };
  contract.active_transfers = {short_task, long_task};
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const auto run = run_segment(ins, contract);

  ASSERT_FALSE(run.solution.empty());
  EXPECT_EQ(
      carrier_event_phase_of(run.final, short_task).kind,
      CarrierTaskPhaseKind::COMPLETED);
  EXPECT_EQ(
      carrier_event_phase_of(run.final, long_task).kind,
      CarrierTaskPhaseKind::CARRYING)
      << "the lower solve must return on the first completed task";
  EXPECT_EQ(run.plan.back()[0].kind, Op::DROP);
  EXPECT_NE(run.plan.back()[1].kind, Op::DROP);
}

TEST(carrier_brd_lower_segment,
     locked_carrying_root_continues_the_same_route)
{
  const auto ins = make_instance(
      {"...."}, {"S..S"}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {0, 3}}});
  const FrozenTaskId id{0, 0};
  auto fixed = target_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2), ins.grid.idx(0, 3)},
      FixedTransferStartMode::LOCKED_CARRYING);
  auto root = initial_phys_config(ins);
  const auto lifted = apply_ops(ins, root, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  const auto moved = apply_ops(
      ins, *lifted,
      {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(moved.has_value());
  CarrierEventContract contract;
  contract.start = *moved;
  contract.wave_ledger_snapshot = {
      TaskLedgerEntry{
          id, ExecutionStatus::CARRYING, 0}};
  contract.active_transfers = {fixed};
  ASSERT_TRUE(
      validate_carrier_event_contract(ins, contract).valid());

  const auto run = run_segment(ins, contract);

  ASSERT_FALSE(run.solution.empty());
  ASSERT_FALSE(run.plan.empty());
  EXPECT_EQ(run.plan.front()[0].kind, Op::MOVE);
  EXPECT_EQ(run.plan.front()[0].to, ins.grid.idx(0, 2));
  EXPECT_EQ(
      carrier_event_phase_of(run.final, fixed).kind,
      CarrierTaskPhaseKind::COMPLETED);
}

TEST(carrier_brd_lower_segment,
     fixed_contract_does_not_build_production_guidance)
{
  const auto ins = make_instance(
      {"..."}, {"S.S"}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {0, 2}}});
  const FrozenTaskId id{0, 0};
  const auto fixed = target_transfer(
      0, id, 0,
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2)});
  CarrierEventContract contract;
  contract.start = initial_phys_config(ins);
  contract.wave_ledger_snapshot = {
      TaskLedgerEntry{
          id, ExecutionStatus::PENDING, std::nullopt}};
  contract.active_transfers = {fixed};

  const auto run = run_segment(ins, contract);

  ASSERT_FALSE(run.solution.empty());
  EXPECT_EQ(run.stats.upper_epoch_builds, 0);
  EXPECT_EQ(run.stats.pair_cache_hits, 0);
  EXPECT_EQ(run.stats.pair_cache_misses, 0);
  EXPECT_EQ(run.stats.rho_match_calls_execute, 0);
  EXPECT_EQ(run.stats.rho_match_calls_prepare, 0);
}
