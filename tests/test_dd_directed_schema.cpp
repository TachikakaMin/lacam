// PROTECTED TEST: Phase 8.3 directedness is part of the persistent schema.
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <memory>
#include <random>

#include "gtest/gtest.h"

namespace {

DDInstance make_schema_case(bool directed)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  if (directed) {
    ins.grid.set_directed_edges(
        {{0, 1}, {1, 0}, {1, 2}, {2, 1}});
  } else {
    ins.grid.set_undirected_edges({{0, 1}, {1, 2}});
  }
  ins.robots = {0};
  ins.shelves = {1};
  ins.target_starts = {1};
  ins.target_goal_sets = {{2}};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_directed_schema,
     persistent_cache_rejects_changed_directedness)
{
  const DDInstance original = make_schema_case(false);
  const auto persistent =
      std::make_shared<TAPFCarrierPersistentState>(original);

  const DDInstance changed = make_schema_case(true);
  ASSERT_EQ(
      original.grid.out_neighbors,
      changed.grid.out_neighbors);
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
