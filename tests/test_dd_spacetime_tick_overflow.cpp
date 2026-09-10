// PROTECTED TEST: absolute commitment ticks must fail before signed overflow.
// Written before the overflow fix (TDD RED), integration Phase 8.5.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <instance.hpp>
#include <tapf_planner.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

namespace {

DDInstance make_carrier_case()
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

CarrierSpacetimeCommitment one_frame_commitment()
{
  CarrierSpacetimeCommitment commitment;
  commitment.frames.resize(1);
  return commitment;
}

}  // namespace

TEST(dd_spacetime_tick_overflow,
     public_carrier_entry_rejects_the_maximum_origin)
{
  const DDInstance ins = make_carrier_case();
  const auto commitment = one_frame_commitment();
  const auto result = solve_carrier_lacam_from_state_result(
      ins, initial_phys_config(ins), 1.0, 0,
      nullptr, nullptr, &commitment,
      std::numeric_limits<int64_t>::max());
  EXPECT_EQ(result.status, DDSolveStatus::INVALID);
}

TEST(dd_spacetime_tick_overflow,
     persistent_session_rejects_the_maximum_origin)
{
  const DDInstance ins = make_carrier_case();
  const auto commitment = one_frame_commitment();
  EXPECT_THROW(
      DDPlanningSession(
          ins, initial_phys_config(ins), 0, commitment,
          std::numeric_limits<int64_t>::max()),
      std::invalid_argument);
}

TEST(dd_spacetime_tick_overflow,
     tapf_entry_rejects_the_maximum_origin)
{
  DDInstance dd;
  dd.grid = DDGrid({".."});
  dd.robots = {0};
  dd.finalize();
  TAPFInstance instance(dd);
  instance.tasks = {instance.G.U[1]};
  instance.allowed[0] = {true};

  const auto commitment = one_frame_commitment();
  TAPFSearchConfig config;
  config.spacetime_commitment = &commitment;
  config.commitment_time_origin =
      std::numeric_limits<int64_t>::max();
  EXPECT_THROW(
      solve_tapf(
          instance, 0, nullptr, nullptr, 0, nullptr,
          false, false, config),
      std::invalid_argument);
}
