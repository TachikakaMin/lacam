// PROTECTED TEST: Phase 8.2 task-agent PIBT must accept degree > 4.
// The Phase 8.2 RED baseline lacked explicit adjacency and fixed its
// candidate buffer to four moves plus WAIT.
#include <dd_carrier.hpp>
#include <instance.hpp>
#include <tapf_planner.hpp>

#include <random>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_explicit_high_degree_planner,
     task_agent_can_take_the_sixth_explicit_neighbor)
{
  DDInstance dd;
  dd.grid = DDGrid({"......."});
  dd.grid.set_undirected_edges(
      {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}});
  dd.robots = {0};
  dd.finalize();

  TAPFInstance instance(dd);
  instance.tasks = {instance.G.U[6]};
  instance.allowed[0] = {true};
  ASSERT_TRUE(instance.is_valid());

  std::mt19937 random(0);
  const Solution solution =
      solve_tapf(instance, 0, nullptr, &random);
  ASSERT_EQ(solution.size(), 2u);
  EXPECT_EQ(solution.front()[0]->index, 0);
  EXPECT_EQ(solution.back()[0]->index, 6);
}
