// PROTECTED TEST: persistent external spacetime commitment contract.
// Written before implementation (TDD RED), integration Phase 8.5.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <cstdint>
#include <optional>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_line_case()
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {0};
  ins.shelves = {1};
  ins.target_starts = {1};
  ins.target_goal_sets = {{3}};
  ins.finalize();
  return ins;
}

CarrierSpacetimeCommitment block_first_approach()
{
  CarrierSpacetimeCommitment commitment;
  commitment.frames.resize(3);
  commitment.lower_directed_edges.resize(2);
  commitment.frames[1].lower_vertices = {1};
  return commitment;
}

std::optional<PhysConfig> replay_prefix(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan, size_t steps,
    const CarrierSpacetimeCommitment& commitment,
    int64_t origin)
{
  if (steps > plan.size()) return std::nullopt;
  PhysConfig state = root;
  for (size_t step = 0; step < steps; ++step) {
    const auto next = apply_ops(
        ins, state, plan[step], true, &commitment,
        origin + static_cast<int64_t>(step));
    if (!next.has_value()) return std::nullopt;
    state = *next;
  }
  return state;
}

}  // namespace

TEST(dd_spacetime_session,
     commit_prefix_advances_the_saved_commitment_origin)
{
  const DDInstance ins = make_line_case();
  const PhysConfig initial = initial_phys_config(ins);
  const auto commitment = block_first_approach();
  DDPlanningSession session(
      ins, initial, 0, commitment, 0);

  const auto first = session.solve(2.0);
  ASSERT_TRUE(first.solved());
  ASSERT_FALSE(first.plan.empty());
  EXPECT_EQ(first.plan.front()[0].kind, Op::WAIT);
  const auto observed = replay_prefix(
      ins, initial, first.plan, 1, commitment, 0);
  ASSERT_TRUE(observed.has_value());

  ASSERT_EQ(
      session.commit_prefix(1, *observed),
      DDCommitStatus::OK);
  EXPECT_EQ(session.commitment_time_origin(), 1);

  const auto warm = session.solve(2.0);
  ASSERT_TRUE(warm.solved());
  ASSERT_FALSE(warm.plan.empty());
  EXPECT_EQ(warm.plan.front()[0].kind, Op::MOVE);
  EXPECT_EQ(warm.plan.front()[0].to, 1);
}

TEST(dd_spacetime_session,
     rebase_requires_explicit_commitment_handling)
{
  const DDInstance ins = make_line_case();
  const PhysConfig initial = initial_phys_config(ins);
  const auto commitment = block_first_approach();
  DDPlanningSession session(
      ins, initial, 0, commitment, 0);

  EXPECT_EQ(
      session.rebase(initial),
      DDRebaseStatus::COMMITMENT_CONFIRMATION_REQUIRED);
  EXPECT_EQ(
      session.rebase_with_current_spacetime_commitment(
          initial, 2),
      DDRebaseStatus::OK);
  EXPECT_EQ(session.commitment_time_origin(), 2);

  CarrierSpacetimeCommitment replacement;
  EXPECT_EQ(
      session.rebase_with_spacetime_commitment(
          initial, replacement, 7),
      DDRebaseStatus::OK);
  EXPECT_EQ(session.commitment_time_origin(), 7);
  EXPECT_FALSE(session.has_spacetime_commitment());
}
