// PROTECTED TEST: persistent Carrier cache schema isolation,
// integration plan Phase 6. Written before implementation (TDD RED).
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <memory>
#include <random>

#include "gtest/gtest.h"

namespace {

DDInstance make_schema_case()
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {0};
  ins.shelves = {1, 2};
  ins.target_starts = {1};
  ins.target_goal_sets = {{2, 3}};
  ins.finalize();
  return ins;
}

void expect_rejected(
    const DDInstance& changed,
    const std::shared_ptr<TAPFCarrierPersistentState>& persistent)
{
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

}  // namespace

TEST(dd_incremental_schema,
     persistent_cache_rejects_changed_target_goal_schema)
{
  const DDInstance original = make_schema_case();
  const auto persistent =
      std::make_shared<TAPFCarrierPersistentState>(original);

  DDInstance changed = original;
  changed.target_goal_sets = {{0, 3}};
  changed.finalize();
  expect_rejected(changed, persistent);
}

TEST(dd_incremental_schema,
     persistent_cache_rejects_changed_storage_topology)
{
  const DDInstance original = make_schema_case();
  const auto persistent =
      std::make_shared<TAPFCarrierPersistentState>(original);

  DDInstance changed = original;
  changed.shelf_storage[0] = 0;
  changed.finalize();
  expect_rejected(changed, persistent);
}
