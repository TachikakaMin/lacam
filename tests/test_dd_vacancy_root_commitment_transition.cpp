// PROTECTED adjacent-epoch root-goal commitment extraction contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_root_commitment_transition,
     keeps_unfinished_missions_and_ignores_stale_forced_roots)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.shelves = ins.target_starts;
  ins.target_goal_sets = {
      {ins.grid.idx(0, 2), ins.grid.idx(0, 3)},
      {ins.grid.idx(0, 2), ins.grid.idx(0, 3)},
  };
  ins.finalize();
  const UpperSignature upper{
      ins.target_starts,
      {},
  };
  UpperEpochGuidance previous_epoch;
  previous_epoch.upper_signature = upper;
  previous_epoch.tau_guide = {
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 3),
  };
  previous_epoch.root_goal_commitment = {
      {0, ins.grid.idx(0, 2)},
  };
  const std::vector<TaskBRForcedEffect> forced{
      TaskBRForcedEffect{
          ShelfSelector{ShelfSelector::Kind::TARGET, 1},
          TaskBRForcedEffect::Kind::TRANSFER,
          StorageTransfer{
              ins.grid.idx(0, 2),
              {ins.grid.idx(0, 1), ins.grid.idx(0, 2)}},
          {
              RootDemand{1, ins.grid.idx(0, 2)},
              RootDemand{1, ins.grid.idx(0, 3)},
          },
          1},
  };

  EXPECT_EQ(
      carrier_detail::active_root_goal_commitments_for_epoch(
          ins, upper, &previous_epoch, forced, {}),
      (RootGoalCommitment{
          {0, ins.grid.idx(0, 2)},
          {1, ins.grid.idx(0, 3)},
      }));
}

TEST(dd_vacancy_root_commitment_transition,
     releases_a_commitment_after_the_target_reaches_its_goal)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.shelves = ins.target_starts;
  ins.target_goal_sets = {{
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  }};
  ins.finalize();
  UpperEpochGuidance previous_epoch;
  previous_epoch.upper_signature = UpperSignature{
      {ins.grid.idx(0, 0)}, {}};
  previous_epoch.tau_guide = {ins.grid.idx(0, 2)};
  previous_epoch.root_goal_commitment = {
      {0, ins.grid.idx(0, 2)},
  };
  const UpperSignature reached{
      {ins.grid.idx(0, 2)}, {}};

  EXPECT_TRUE(
      carrier_detail::active_root_goal_commitments_for_epoch(
          ins, reached, &previous_epoch, {}, {})
          .empty());
}
