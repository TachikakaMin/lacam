// PROTECTED carried-target goal commitment contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(dd_vacancy_carried_goal_commitment,
     target_keeps_its_previous_goal_until_drop)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = ins.shelves;
  ins.target_goal_sets = {{
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  }};
  ins.finalize();
  const PhysConfig carried{
      {ins.grid.idx(0, 2)},
      {ins.grid.idx(0, 2)},
      {},
      {0},
  };
  UpperEpochGuidance previous_epoch;
  previous_epoch.tau_guide = {ins.grid.idx(0, 2)};

  EXPECT_EQ(
      carrier_detail::carried_target_goal_commitments_for_epoch(
          ins, carried, &previous_epoch, {}),
      (RootGoalCommitment{{0, ins.grid.idx(0, 2)}}));
}

TEST(dd_vacancy_carried_goal_commitment,
     grounded_target_does_not_create_a_new_commitment)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = ins.shelves;
  ins.target_goal_sets = {{
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  }};
  ins.finalize();
  auto grounded = initial_phys_config(ins);
  grounded.target_pos[0] = ins.grid.idx(0, 2);
  UpperEpochGuidance previous_epoch;
  previous_epoch.tau_guide = {ins.grid.idx(0, 2)};

  EXPECT_TRUE(
      carrier_detail::carried_target_goal_commitments_for_epoch(
          ins, grounded, &previous_epoch, {})
          .empty());
}
