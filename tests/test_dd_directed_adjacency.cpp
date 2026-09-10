// PROTECTED TEST: Phase 8.3 directed adjacency and reverse distance.
// Written before implementation (TDD RED).
#include <dd_carrier.hpp>
#include <dd_dist_adapters.hpp>
#include <instance.hpp>
#include <tapf_planner.hpp>

#include <climits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_directed_adjacency,
     move_graph_and_reverse_distance_respect_arc_direction)
{
  DDGrid grid({"..."});
  grid.set_directed_edges({{0, 1}, {1, 2}});
  EXPECT_EQ(grid.outgoing(0), (std::vector<int>{1}));
  EXPECT_TRUE(grid.incoming(0).empty());
  EXPECT_EQ(grid.incoming(1), (std::vector<int>{0}));
  EXPECT_EQ(grid.incoming(2), (std::vector<int>{1}));

  DDInstance ins;
  ins.grid = grid;
  ins.robots = {0};
  ins.finalize();
  const PhysConfig at_zero = initial_phys_config(ins);
  const auto at_one =
      apply_ops(ins, at_zero, {Op::make_move(1)});
  ASSERT_TRUE(at_one.has_value());
  EXPECT_TRUE(
      apply_ops(ins, *at_one, {Op::make_move(2)}).has_value());
  EXPECT_FALSE(
      apply_ops(ins, *at_one, {Op::make_move(0)}).has_value());

  DDDistCache distance(grid);
  EXPECT_EQ(distance.dist(2, 0), 2);
  EXPECT_EQ(distance.dist(0, 2), INT_MAX / 2);

  const TAPFInstance view(ins);
  ASSERT_EQ(view.G.U[0]->neighbor.size(), 1u);
  EXPECT_EQ(view.G.U[0]->neighbor[0]->index, 1);
  EXPECT_TRUE(view.G.U[0]->predecessor.empty());
  ASSERT_EQ(view.G.U[1]->predecessor.size(), 1u);
  EXPECT_EQ(view.G.U[1]->predecessor[0]->index, 0);
}

TEST(dd_directed_adjacency,
     finalize_checks_start_to_goal_reachability)
{
  DDInstance reachable;
  reachable.grid = DDGrid({".."});
  reachable.grid.set_directed_edges({{0, 1}});
  reachable.robots = {0};
  reachable.shelves = {0};
  reachable.target_starts = {0};
  reachable.target_goal_sets = {{1}};
  EXPECT_NO_THROW(reachable.finalize());

  DDInstance unreachable;
  unreachable.grid = DDGrid({".."});
  unreachable.grid.set_directed_edges({{0, 1}});
  unreachable.robots = {1};
  unreachable.shelves = {1};
  unreachable.target_starts = {1};
  unreachable.target_goal_sets = {{0}};
  EXPECT_THROW(unreachable.finalize(), std::invalid_argument);
}

TEST(dd_directed_adjacency,
     task_distance_and_planner_follow_forward_arcs)
{
  DDInstance dd;
  dd.grid = DDGrid({"..."});
  dd.grid.set_directed_edges({{0, 1}, {1, 2}});
  dd.robots = {0};
  dd.finalize();

  TAPFInstance instance(dd);
  instance.tasks = {instance.G.U[2]};
  instance.allowed[0] = {true};
  ASSERT_TRUE(instance.is_valid());
  const Solution solution = solve_tapf(instance);
  ASSERT_EQ(solution.size(), 3u);
  EXPECT_EQ(solution[0][0]->index, 0);
  EXPECT_EQ(solution[1][0]->index, 1);
  EXPECT_EQ(solution[2][0]->index, 2);
}
