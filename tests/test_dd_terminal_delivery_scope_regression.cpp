// Regression for over-broad terminal-delivery ordering.
// A free robot whose route merely passes through a target endpoint must not
// be treated as dispatching to pick up at that endpoint.
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <memory>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_terminal_delivery_scope_regression,
     route_through_endpoint_does_not_override_dispatch_priority)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {
      ins.grid.idx(1, 1),
      ins.grid.idx(0, 0),
  };
  ins.shelves = {
      ins.grid.idx(1, 1),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = ins.shelves;
  ins.target_goals = {
      ins.grid.idx(0, 1),
      ins.grid.idx(1, 2),
  };
  ins.target_goal_sets = {
      {ins.grid.idx(0, 1)},
      {ins.grid.idx(1, 2)},
  };
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.finalize();

  PhysConfig physical = initial_phys_config(ins);
  physical.kappa[0] = 0;

  const ShelfTask delivery{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          ins.grid.idx(1, 1), ins.grid.idx(0, 1)},
      {RootDemand{0, ins.grid.idx(0, 1)}},
      1,
      StorageTransfer{
          ins.grid.idx(0, 1),
          {ins.grid.idx(1, 1), ins.grid.idx(0, 1)}}};
  const ShelfTask pickup{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 1},
          ins.grid.idx(0, 2), ins.grid.idx(1, 2)},
      {RootDemand{1, ins.grid.idx(1, 2)}},
      100,
      StorageTransfer{
          ins.grid.idx(1, 2),
          {ins.grid.idx(0, 2), ins.grid.idx(1, 2)}}};

  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->tau_guide = ins.target_goals;
  epoch->task_graph.tasks = {pickup};

  CarrierGuidance guidance;
  guidance.upper_epoch = epoch;
  guidance.custody_by_robot.resize(ins.n_robots());
  guidance.custody_by_robot[0] =
      carrier_detail::make_custody(delivery, -1);
  carrier_detail::ensure_transfer_identity(
      *guidance.custody_by_robot[0], 0,
      phys_config_hash(physical));
  guidance.rho_task_id.resize(ins.n_robots());
  guidance.rho_task_id[1] = pickup.id;
  guidance.rho_ready_index = {-1, 0};

  carrier_detail::LowerDist lower(ins.grid);
  const auto order =
      carrier_detail::task_br_robot_order(
          physical, guidance, lower);

  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order.front(), 1)
      << "terminal delivery may override a dispatch that picks up at the "
         "endpoint, but not one whose route merely passes through it";
}
