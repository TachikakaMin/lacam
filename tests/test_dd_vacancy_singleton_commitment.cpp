// PROTECTED singleton-goal compatibility contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(dd_vacancy_singleton_commitment,
     fixed_goal_target_never_needs_a_history_dependent_commitment)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = ins.shelves;
  ins.target_goal_sets = {{ins.grid.idx(0, 2)}};
  ins.finalize();
  const PhysConfig carried{
      {ins.grid.idx(0, 2)},
      {ins.grid.idx(0, 2)},
      {},
      {0},
  };
  UpperEpochGuidance previous_epoch;
  previous_epoch.upper_signature = UpperSignature{
      {ins.grid.idx(0, 1)}, {}};
  previous_epoch.tau_guide = {ins.grid.idx(0, 2)};
  const std::vector<TaskBRForcedEffect> forced{
      TaskBRForcedEffect{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          TaskBRForcedEffect::Kind::TRANSFER,
          StorageTransfer{
              ins.grid.idx(0, 2),
              {ins.grid.idx(0, 1), ins.grid.idx(0, 2)}},
          {RootDemand{0, ins.grid.idx(0, 2)}},
          1},
  };

  EXPECT_TRUE(
      carrier_detail::active_root_goal_commitments_for_epoch(
          ins, carrier_detail::make_upper_signature(carried),
          &previous_epoch, forced, {})
          .empty());
  EXPECT_TRUE(
      carrier_detail::carried_target_goal_commitments_for_epoch(
          ins, carried, &previous_epoch, {})
          .empty());
}
