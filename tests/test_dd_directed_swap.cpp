// PROTECTED TEST: Phase 8.3 directed PIBT swap legality.
// Written before the implementation fix (TDD RED).
#include <dd_carrier.hpp>
#include <instance.hpp>
#include <tapf_planner.hpp>

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_directed_swap, forced_swap_never_uses_a_missing_reverse_arc)
{
  DDInstance dd;
  dd.grid = DDGrid({"......"});
  dd.grid.set_directed_edges(
      {{0, 1}, {0, 3}, {1, 2}, {3, 4}, {3, 5}});
  dd.robots = {0, 1};
  dd.finalize();

  TAPFInstance instance(dd);
  instance.tasks = {instance.G.U[2], instance.G.U[1]};
  instance.allowed = {{true, false}, {false, true}};
  ASSERT_TRUE(instance.is_valid());

  TAPFPlanner planner(&instance, nullptr, nullptr);
  TAPFNode node(
      instance.starts, initial_shelf_state(instance), planner.D,
      &instance, {0, 1}, TAPFAssignmentState(), nullptr);
  node.order = {0, 1};
  node.constraint_order = node.order;

  TAPFConstraint root;
  ASSERT_TRUE(planner.get_new_config(&node, &root));

  for (size_t robot = 0; robot < instance.N; ++robot) {
    const Vertex* from = planner.A[robot]->v_now;
    const Vertex* to = planner.A[robot]->v_next;
    ASSERT_NE(to, nullptr);
    if (from == to) continue;
    EXPECT_NE(
        std::find(from->neighbor.begin(), from->neighbor.end(), to),
        from->neighbor.end())
        << "robot " << robot << " moved along missing directed arc "
        << from->index << " -> " << to->index;
  }
}
