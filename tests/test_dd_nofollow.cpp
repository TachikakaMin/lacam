// Explicit no-following oracle variant used for BRaP conformance checks.
#include <dd_carrier.hpp>

#include "gtest/gtest.h"

namespace {

DDInstance convoy_ins()
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_nofollow, lower_deck_following_rejected_by_explicit_oracle)
{
  const auto ins = convoy_ins();
  const auto X = initial_phys_config(ins);
  const std::vector<Op> follow = {
      Op::make_move(ins.grid.idx(0, 1)),
      Op::make_move(ins.grid.idx(0, 2))};
  EXPECT_TRUE(apply_ops(ins, X, follow).has_value());
  EXPECT_FALSE(
      apply_ops(ins, X, follow, /*allow_following=*/false).has_value());

  const std::vector<Op> solo = {
      Op::make_wait(), Op::make_move(ins.grid.idx(0, 2))};
  EXPECT_TRUE(
      apply_ops(ins, X, solo, /*allow_following=*/false).has_value());
}

TEST(dd_nofollow, upper_deck_following_rejected_by_explicit_oracle)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 2), ins.grid.idx(0, 3)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 1)};
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa = {0, 1};
  const std::vector<Op> follow = {
      Op::make_move(ins.grid.idx(0, 1)),
      Op::make_move(ins.grid.idx(0, 2))};
  EXPECT_TRUE(apply_ops(ins, X, follow).has_value());
  EXPECT_FALSE(
      apply_ops(ins, X, follow, /*allow_following=*/false).has_value());
}

TEST(dd_nofollow, zero_empty_rotation_rejected_by_explicit_oracle)
{
  DDInstance ins;
  ins.grid = DDGrid({"..", ".."});
  ins.robots = {0, 1, 2, 3};
  ins.target_starts = {0};
  ins.target_goals = {1};
  ins.shelves = {0, 1, 2, 3};
  ins.finalize();
  auto X = initial_phys_config(ins);
  X.kappa = {0, KAPPA_ANON, KAPPA_ANON, KAPPA_ANON};
  X.anon_occ.clear();
  const std::vector<Op> rotate = {
      Op::make_move(1), Op::make_move(3),
      Op::make_move(0), Op::make_move(2)};
  EXPECT_TRUE(apply_ops(ins, X, rotate).has_value());
  EXPECT_FALSE(
      apply_ops(ins, X, rotate, /*allow_following=*/false).has_value());
}
