#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(Instance, initialize)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen_filename, map_filename, 3);

  ASSERT_EQ(size(ins.starts), 3);
  ASSERT_EQ(size(ins.goals), 3);
  ASSERT_EQ(ins.starts[0]->index, 203);
  ASSERT_EQ(ins.goals[0]->index, 583);
}

TEST(TAPFInstance, preserves_agent_target_cost_offsets)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{0, 1};
  const auto goals =
      std::vector<std::vector<int> >{{4, 5}, {4, 5}};
  const auto offsets =
      std::vector<std::vector<int> >{{7, 2}, {3, 9}};
  const auto ins = TAPFInstance(map_filename, starts, goals, offsets);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), 2);
  for (size_t task = 0; task < ins.tasks.size(); ++task) {
    if (ins.tasks[task]->index == 4) {
      ASSERT_EQ(ins.assignment_cost_offsets[0][task], 7);
      ASSERT_EQ(ins.assignment_cost_offsets[1][task], 3);
    } else if (ins.tasks[task]->index == 5) {
      ASSERT_EQ(ins.assignment_cost_offsets[0][task], 2);
      ASSERT_EQ(ins.assignment_cost_offsets[1][task], 9);
    } else {
      FAIL() << "unexpected TAPF target";
    }
  }
}

TEST(TAPFInstance, rejects_mismatched_cost_offsets)
{
  const auto ins = TAPFInstance(
      "./tests/assets/lifelong-task-small.map", std::vector<int>{0, 1},
      std::vector<std::vector<int> >{{4}, {5}},
      std::vector<std::vector<int> >{{7}, {3, 9}});

  ASSERT_FALSE(ins.is_valid());
}
