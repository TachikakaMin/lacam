// REGRESSION: one transfer may drop where another active transfer starts.
// The lower planner clears the source by Lift and enforces Drop legality.
#include <dd_carrier.hpp>
#include <tapf_planner.hpp>

#include "gtest/gtest.h"

TEST(carrier_brd_endpoint_source_overlap,
     pending_storage_cycle_is_a_valid_event_contract)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {ins.grid.idx(0, 0), ins.grid.idx(0, 3)};
  ins.shelves = {ins.grid.idx(0, 1), ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 1), ins.grid.idx(0, 2)};
  ins.target_goals = {ins.grid.idx(0, 2), ins.grid.idx(0, 1)};
  ins.finalize();

  const auto source_a = ins.grid.idx(0, 1);
  const auto source_b = ins.grid.idx(0, 2);
  CarrierEventContract contract;
  contract.start = initial_phys_config(ins);
  contract.wave_ledger_snapshot = {
      {FrozenTaskId{0, 0}, ExecutionStatus::PENDING, std::nullopt},
      {FrozenTaskId{0, 1}, ExecutionStatus::PENDING, std::nullopt},
  };
  contract.active_transfers = {
      {
          0,
          FrozenTaskId{0, 0},
          {UpperShelfHandle::Kind::TARGET, 0},
          {ShelfSelector::Kind::TARGET, 0},
          StorageTransfer{source_b, {source_a, source_b}},
          FixedTransferStartMode::PROVISIONAL_FREE,
      },
      {
          1,
          FrozenTaskId{0, 1},
          {UpperShelfHandle::Kind::TARGET, 1},
          {ShelfSelector::Kind::TARGET, 1},
          StorageTransfer{source_a, {source_b, source_a}},
          FixedTransferStartMode::PROVISIONAL_FREE,
      },
  };

  EXPECT_TRUE(validate_carrier_event_contract(ins, contract).valid());
}
