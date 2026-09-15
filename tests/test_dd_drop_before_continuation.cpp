#include "../lacam/src/carrier_guidance.hpp"
#include <dd_planner.hpp>

#include "gtest/gtest.h"

TEST(dd_drop_before_continuation,
     arrived_loaded_shelf_is_not_ready_for_the_next_task)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 2)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.target_goal_sets = {{ins.grid.idx(0, 4)}};
  ins.finalize();

  const TaskId completed{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 1),
  };
  const TaskId continuation{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 0),
  };

  ShelfTaskGraph current_graph;
  current_graph.tasks = {
      ShelfTask{
          continuation,
          {RootDemand{0, ins.grid.idx(0, 4)}},
          1,
      },
  };
  current_graph.predecessors = {{}};
  current_graph.successors = {{}};

  const PhysConfig previous{
      {ins.grid.idx(0, 2)},
      {ins.grid.idx(0, 2)},
      {},
      {0},
  };
  const PhysConfig arrived{
      {ins.grid.idx(0, 1)},
      {ins.grid.idx(0, 1)},
      {},
      {0},
  };

  auto previous_epoch = std::make_shared<UpperEpochGuidance>();
  previous_epoch->task_graph.tasks = {
      ShelfTask{
          completed,
          {RootDemand{0, ins.grid.idx(0, 4)}},
          1,
      },
  };
  previous_epoch->task_graph.predecessors = {{}};
  previous_epoch->task_graph.successors = {{}};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      carrier_detail::make_custody(
          previous_epoch->task_graph.tasks[0], 0),
  };

  const std::vector<Op> move = {
      Op::make_move(ins.grid.idx(0, 1)),
  };
  const auto recovered =
      carrier_detail::recover_task_br_custody(
          ins, arrived, current_graph, &previous,
          &previous_guidance, &move);
  auto custody = recovered.custody_by_robot;
  ASSERT_TRUE(custody[0].has_value());
  ASSERT_EQ(custody[0]->route_status, RouteStatus::ARRIVED);
  ASSERT_EQ(
      carrier_detail::custody_endpoint(*custody[0]),
      ins.grid.idx(0, 1));
  ASSERT_NE(arrived.kappa[0], KAPPA_FREE);

  const auto ready_before_drop =
      carrier_detail::ready_tasks_with_custody(
          ins, arrived, current_graph, custody,
          recovered.continuation_carrier);
  EXPECT_TRUE(ready_before_drop.empty());

  carrier_detail::bind_ready_continuations(
      ins, arrived, current_graph, ready_before_drop,
      recovered.continuation_carrier,
      recovered.previous_loaded_move_from, custody);
  ASSERT_TRUE(custody[0].has_value());
  EXPECT_EQ(
      carrier_detail::custody_endpoint(*custody[0]),
      ins.grid.idx(0, 1));
  EXPECT_EQ(custody[0]->route_status, RouteStatus::ARRIVED);

  const auto grounded =
      apply_ops(ins, arrived, {Op::make_drop()});
  ASSERT_TRUE(grounded.has_value());
  ASSERT_EQ(grounded->kappa[0], KAPPA_FREE);
  const std::vector<std::optional<Custody>> no_custody(1);
  const std::vector<uint8_t> no_continuation(1, 0);
  EXPECT_EQ(
      carrier_detail::ready_tasks_with_custody(
          ins, *grounded, current_graph, no_custody,
          no_continuation),
      (std::vector<int>{0}));
}
