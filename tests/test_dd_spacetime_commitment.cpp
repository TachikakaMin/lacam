// PROTECTED TEST: Phase 8.4 external spacetime commitment.
// Written before implementation (TDD RED).
#include <dd_carrier.hpp>
#include <instance.hpp>
#include <tapf_planner.hpp>

#include <vector>

#include "gtest/gtest.h"

namespace {

CarrierSpacetimeCommitment commitment_with_horizon(size_t horizon)
{
  CarrierSpacetimeCommitment commitment;
  commitment.frames.resize(horizon + 1);
  commitment.lower_directed_edges.resize(horizon);
  return commitment;
}

DDInstance line_instance(int width, int robot)
{
  DDInstance ins;
  ins.grid = DDGrid({std::string(width, '.')});
  ins.robots = {robot};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_spacetime_commitment,
     authoritative_transition_enforces_lower_and_reverse_edge)
{
  const DDInstance ins = line_instance(2, 0);
  const PhysConfig root = initial_phys_config(ins);

  auto lower = commitment_with_horizon(1);
  lower.frames[1].lower_vertices = {1};
  EXPECT_FALSE(
      apply_ops(
          ins, root, {Op::make_move(1)}, true, &lower, 0)
          .has_value());
  EXPECT_TRUE(
      apply_ops(
          ins, root, {Op::make_wait()}, true, &lower, 0)
          .has_value());

  auto reverse_edge = commitment_with_horizon(1);
  reverse_edge.lower_directed_edges[0] = {{1, 0}};
  EXPECT_FALSE(
      apply_ops(
          ins, root, {Op::make_move(1)}, true,
          &reverse_edge, 0)
          .has_value());
}

TEST(dd_spacetime_commitment,
     upper_frame_blocks_loaded_shelf_but_not_empty_robot)
{
  DDInstance carrier;
  carrier.grid = DDGrid({".."});
  carrier.robots = {0};
  carrier.shelves = {0};
  carrier.target_starts = {0};
  carrier.target_goal_sets = {{1}};
  carrier.finalize();

  const PhysConfig root = initial_phys_config(carrier);
  const auto lifted =
      apply_ops(carrier, root, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());

  auto commitment = commitment_with_horizon(2);
  commitment.frames[2].upper_vertices = {1};
  EXPECT_FALSE(
      apply_ops(
          carrier, *lifted, {Op::make_move(1)}, true,
          &commitment, 1)
          .has_value());

  const DDInstance empty = line_instance(2, 0);
  EXPECT_TRUE(
      apply_ops(
          empty, initial_phys_config(empty),
          {Op::make_move(1)}, true, &commitment, 1)
          .has_value());
}

TEST(dd_spacetime_commitment,
     tail_policy_releases_or_holds_the_last_frame)
{
  const DDInstance ins = line_instance(2, 0);
  const PhysConfig root = initial_phys_config(ins);

  auto release = commitment_with_horizon(1);
  release.frames[1].lower_vertices = {1};
  release.tail_policy =
      CarrierSpacetimeTailPolicy::RELEASE;
  EXPECT_TRUE(
      apply_ops(
          ins, root, {Op::make_move(1)}, true, &release, 1)
          .has_value());

  auto hold = release;
  hold.tail_policy =
      CarrierSpacetimeTailPolicy::HOLD_LAST;
  EXPECT_FALSE(
      apply_ops(
          ins, root, {Op::make_move(1)}, true, &hold, 1)
          .has_value());
}

TEST(dd_spacetime_commitment,
     time_aware_closed_allows_waiting_until_a_cell_is_released)
{
  DDInstance dd = line_instance(2, 0);
  TAPFInstance instance(dd);
  instance.tasks = {instance.G.U[1]};
  instance.allowed[0] = {true};
  ASSERT_TRUE(instance.is_valid());

  auto commitment = commitment_with_horizon(2);
  commitment.frames[1].lower_vertices = {1};

  TAPFSearchConfig config;
  config.objective =
      TAPFObjective::MAKESPAN_THEN_WORK;
  config.spacetime_commitment = &commitment;
  const Solution solution =
      solve_tapf(
          instance, 0, nullptr, nullptr, 0, nullptr, false,
          false, config);

  ASSERT_EQ(solution.size(), 3u);
  EXPECT_EQ(solution[0][0]->index, 0);
  EXPECT_EQ(solution[1][0]->index, 0);
  EXPECT_EQ(solution[2][0]->index, 1);
}

TEST(dd_spacetime_commitment,
     empty_commitment_preserves_the_direct_path)
{
  DDInstance dd = line_instance(2, 0);
  TAPFInstance instance(dd);
  instance.tasks = {instance.G.U[1]};
  instance.allowed[0] = {true};

  CarrierSpacetimeCommitment empty;
  TAPFSearchConfig config;
  config.objective =
      TAPFObjective::MAKESPAN_THEN_WORK;
  config.spacetime_commitment = &empty;
  const Solution solution =
      solve_tapf(
          instance, 0, nullptr, nullptr, 0, nullptr, false,
          false, config);

  ASSERT_EQ(solution.size(), 2u);
  EXPECT_EQ(solution[0][0]->index, 0);
  EXPECT_EQ(solution[1][0]->index, 1);
}
