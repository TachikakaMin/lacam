// PROTECTED TEST: Phase 8.2 explicit undirected adjacency.
// Written before implementation (TDD RED).
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_dist_adapters.hpp>
#include <dd_planner.hpp>
#include <instance.hpp>
#include <tapf_planner.hpp>

#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDGrid make_hub_grid()
{
  DDGrid grid({"......."});
  grid.set_undirected_edges(
      {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}});
  return grid;
}

DDInstance make_schema_case(
    const std::vector<std::pair<int, int>>& edges)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.grid.set_undirected_edges(edges);
  ins.robots = {4};
  ins.shelves = {1};
  ins.target_starts = {1};
  ins.target_goal_sets = {{2}};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_explicit_undirected_adjacency,
     degree_above_four_is_not_truncated)
{
  DDGrid grid = make_hub_grid();
  EXPECT_EQ(
      grid.outgoing(0),
      (std::vector<int>{1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(grid.incoming(0), grid.outgoing(0));

  DDLazyDist distance(grid, 0);
  for (int endpoint = 1; endpoint <= 6; ++endpoint)
    EXPECT_EQ(distance.get(endpoint), 1);

  DDInstance ins;
  ins.grid = grid;
  ins.robots = {0};
  ins.finalize();
  const PhysConfig root = initial_phys_config(ins);
  for (int endpoint = 1; endpoint <= 6; ++endpoint) {
    const auto next =
        apply_ops(ins, root, {Op::make_move(endpoint)});
    ASSERT_TRUE(next.has_value()) << endpoint;
    EXPECT_EQ(next->robots[0], endpoint);
  }
}

TEST(dd_explicit_undirected_adjacency,
     graph_and_transition_use_only_explicit_edges)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.grid.set_undirected_edges({{0, 2}});
  ins.robots = {0};
  ins.shelves = {2};
  ins.target_starts = {2};
  ins.target_goal_sets = {{0}};
  ins.finalize();

  const PhysConfig root = initial_phys_config(ins);
  EXPECT_TRUE(
      apply_ops(ins, root, {Op::make_move(2)}).has_value());
  EXPECT_FALSE(
      apply_ops(ins, root, {Op::make_move(1)}).has_value());

  const TAPFInstance view(ins);
  ASSERT_NE(view.G.U[0], nullptr);
  ASSERT_EQ(view.G.U[0]->neighbor.size(), 1u);
  EXPECT_EQ(view.G.U[0]->neighbor[0]->index, 2);
  ASSERT_NE(view.G.U[2], nullptr);
  ASSERT_EQ(view.G.U[2]->neighbor.size(), 1u);
  EXPECT_EQ(view.G.U[2]->neighbor[0]->index, 0);
  EXPECT_TRUE(view.G.U[1]->neighbor.empty());
}

TEST(dd_explicit_undirected_adjacency,
     default_rectangular_order_stays_down_up_right_left)
{
  DDGrid grid({"...", "...", "..."});
  const int center = grid.idx(1, 1);
  const std::vector<int> expected{
      grid.idx(2, 1), grid.idx(0, 1),
      grid.idx(1, 2), grid.idx(1, 0)};
  EXPECT_EQ(grid.outgoing(center), expected);
  EXPECT_EQ(grid.incoming(center), expected);
}

TEST(dd_explicit_undirected_adjacency,
     persistent_cache_rejects_changed_adjacency)
{
  const DDInstance original = make_schema_case(
      {{0, 1}, {0, 2}, {0, 3}, {0, 4}});
  const auto persistent =
      std::make_shared<TAPFCarrierPersistentState>(original);

  const DDInstance changed = make_schema_case(
      {{0, 1}, {0, 2}, {1, 3}, {0, 4}});
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
