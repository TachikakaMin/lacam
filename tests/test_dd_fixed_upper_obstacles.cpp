// PROTECTED TEST: Phase 8 fixed upper-deck obstacle contract from
// design_final.md §28.7.1. Written before implementation (TDD RED).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <br_lacam_upper.hpp>

#include <algorithm>

#include "gtest/gtest.h"

namespace {

DDInstance make_detour_case()
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {ins.grid.idx(1, 0)};
  ins.shelves = {ins.grid.idx(1, 0)};
  ins.target_starts = {ins.grid.idx(1, 0)};
  ins.target_goals = {ins.grid.idx(1, 2)};
  ins.fixed_upper_cells = {ins.grid.idx(1, 1)};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_fixed_upper, finalize_rejects_dynamic_overlap_and_duplicates)
{
  DDInstance overlap;
  overlap.grid = DDGrid({"..."});
  overlap.robots = {overlap.grid.idx(0, 0)};
  overlap.shelves = {overlap.grid.idx(0, 1)};
  overlap.fixed_upper_cells = {overlap.grid.idx(0, 1)};
  EXPECT_THROW(overlap.finalize(), std::invalid_argument);

  DDInstance duplicate;
  duplicate.grid = DDGrid({"..."});
  duplicate.robots = {duplicate.grid.idx(0, 0)};
  duplicate.fixed_upper_cells = {
      duplicate.grid.idx(0, 1), duplicate.grid.idx(0, 1)};
  EXPECT_THROW(duplicate.finalize(), std::invalid_argument);
}

TEST(dd_fixed_upper, unloaded_robot_may_move_beneath_but_cannot_lift)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.fixed_upper_cells = {ins.grid.idx(0, 1)};
  ins.finalize();

  const auto moved = apply_ops(
      ins, initial_phys_config(ins),
      {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(moved.has_value())
      << "fixed upper occupancy must not become a lower-deck wall";
  EXPECT_FALSE(
      apply_ops(ins, *moved, {Op::make_lift()}).has_value())
      << "a fixed pod must never be interpreted as anonymous cargo";
  EXPECT_TRUE(moved->anon_occ.empty());
}

TEST(dd_fixed_upper, loaded_robot_and_arbitrary_root_cannot_overlap)
{
  const DDInstance ins = make_detour_case();
  const auto lifted = apply_ops(
      ins, initial_phys_config(ins), {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  EXPECT_FALSE(
      apply_ops(
          ins, *lifted,
          {Op::make_move(ins.grid.idx(1, 1))})
          .has_value())
      << "a carried shelf cannot enter fixed upper occupancy";

  PhysConfig overlapping = *lifted;
  overlapping.robots[0] = ins.grid.idx(1, 1);
  overlapping.target_pos[0] = ins.grid.idx(1, 1);
  EXPECT_EQ(
      validate_phys_config_root(ins, overlapping).reason,
      PhysRootInvalidReason::FIXED_UPPER_COLLISION);
}

TEST(dd_fixed_upper, planner_routes_carried_target_around_obstacle)
{
  const DDInstance ins = make_detour_case();
  DDStats stats;
  const auto result =
      solve_carrier_lacam_result(ins, 3.0, 0, &stats);
  ASSERT_EQ(result.status, DDSolveStatus::SOLVED);
  ASSERT_FALSE(result.plan.empty());

  PhysConfig state = initial_phys_config(ins);
  bool used_detour = false;
  for (const auto& ops : result.plan) {
    const auto next = apply_ops(ins, state, ops);
    ASSERT_TRUE(next.has_value());
    if (next->robots[0] < ins.grid.width) used_detour = true;
    if (next->kappa[0] != KAPPA_FREE) {
      EXPECT_FALSE(std::binary_search(
          ins.fixed_upper_cells.begin(),
          ins.fixed_upper_cells.end(),
          next->robots[0]));
    }
    state = *next;
  }
  EXPECT_TRUE(used_detour);
  EXPECT_TRUE(is_dd_goal(ins, state));
}

TEST(dd_fixed_upper,
     br_upper_search_ranks_the_first_transfer_with_fixed_upper_walls)
{
  // Cell layout:
  //
  //   target  .  fixed  goal
  //        . fixed   .    .
  //        .     .   .    .
  //
  // Without the upper-deck walls, 0 -> 1 looks shorter than 0 -> 4.
  // With the fixed pods treated as upper-deck walls, 0 -> 4 is the
  // shorter legal continuation to the goal.  This reaches the cached
  // upper-distance path inside the BR upper-domain search, rather than
  // only the public one-shot partial compiler.
  DDInstance ins;
  ins.grid = DDGrid({"....", "....", "...."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.fixed_upper_cells = {
      ins.grid.idx(0, 2), ins.grid.idx(1, 1)};
  ins.finalize();

  const auto result = solve_carrier_br_upper(
      ins, br_labeled_initial_state(ins),
      {ins.grid.idx(0, 3)}, nullptr);
  ASSERT_TRUE(result.solved());
  ASSERT_FALSE(result.transitions.empty());
  ASSERT_EQ(result.transitions.front().transfers.size(), 1u);
  const auto& first =
      result.transitions.front().transfers.front().transfer;
  EXPECT_EQ(first.route.front(), ins.grid.idx(0, 0));
  EXPECT_EQ(first.endpoint, ins.grid.idx(1, 0));
  EXPECT_EQ(
      result.stats.explored,
      static_cast<long>(result.states.size()))
      << "fixed upper walls must not create a phantom shortcut branch";
}
