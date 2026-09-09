// PROTECTED BR-upper low-level candidate legality regression.
// A transfer whose route is occupied in the current state cannot appear
// in any joint action accepted by validate_complete_upper_action().
#include <br_lacam_upper.hpp>
#include <dd_carrier.hpp>

#include <vector>

#include "gtest/gtest.h"

TEST(carrier_brd_candidate_legality,
     candidates_keep_wait_and_only_currently_clear_transfers)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();

  const auto state = br_labeled_initial_state(ins);
  const UpperShelfHandle target{
      UpperShelfHandle::Kind::TARGET, 0};
  const UpperShelfHandle anonymous{
      UpperShelfHandle::Kind::ANONYMOUS, 0};

  const auto target_candidates =
      br_upper_constraint_candidates(ins, state, target);
  ASSERT_EQ(target_candidates.size(), 1u);
  EXPECT_EQ(
      target_candidates.front().kind,
      BRUpperConstraintEntry::Kind::WAIT);

  const auto anonymous_candidates =
      br_upper_constraint_candidates(ins, state, anonymous);
  ASSERT_EQ(anonymous_candidates.size(), 2u);
  EXPECT_EQ(
      anonymous_candidates.front().kind,
      BRUpperConstraintEntry::Kind::WAIT);
  EXPECT_EQ(
      anonymous_candidates.back().kind,
      BRUpperConstraintEntry::Kind::TRANSFER);
  EXPECT_EQ(
      anonymous_candidates.back().transfer.endpoint,
      ins.grid.idx(0, 2));
}
