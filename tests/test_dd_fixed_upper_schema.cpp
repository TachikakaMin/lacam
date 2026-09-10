// PROTECTED TEST: Phase 8 fixed upper-deck cells are part of the
// persistent Carrier schema. Written before implementation (TDD RED).
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <memory>
#include <random>

#include "gtest/gtest.h"

TEST(dd_fixed_upper_schema,
     persistent_cache_rejects_changed_fixed_upper_cells)
{
  DDInstance original;
  original.grid = DDGrid({"....."});
  original.robots = {0};
  original.shelves = {1};
  original.target_starts = {1};
  original.target_goal_sets = {{4}};
  original.finalize();

  const auto persistent =
      std::make_shared<TAPFCarrierPersistentState>(original);

  DDInstance changed = original;
  changed.fixed_upper_cells = {2};
  changed.finalize();

  const TAPFInstance view(changed);
  TAPFSearchConfig config;
  config.initial_physical = initial_phys_config(changed);
  config.carrier_persistent_state = persistent;
  std::mt19937 random(0);
  EXPECT_THROW(
      TAPFPlanner(
          &view, nullptr, &random, 0, 0, 0.001f, true,
          nullptr, config),
      std::invalid_argument);
}
