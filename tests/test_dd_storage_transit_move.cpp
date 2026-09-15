#include "../lacam/src/carrier_guidance.hpp"

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance all_storage_line()
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.target_goal_sets = {{ins.grid.idx(0, 3)}};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_storage_transit_move,
     loaded_routes_may_cross_storage_cells_before_the_endpoint)
{
  const auto ins = all_storage_line();
  auto physical = initial_phys_config(ins);
  physical.kappa = {0};

  const std::vector<int> expected = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 1),
      ins.grid.idx(0, 2), ins.grid.idx(0, 3)};

  const auto rerouted = carrier_detail::reroute_to_endpoint(
      ins, physical, 0, ins.grid.idx(0, 3));
  EXPECT_EQ(rerouted.status, RouteStatus::OK);
  EXPECT_EQ(rerouted.route, expected);

  const auto distance = carrier_detail::transport_topology_distance(
      ins, ins.grid.idx(0, 0), ins.grid.idx(0, 3));
  EXPECT_EQ(distance[ins.grid.idx(0, 0)], 3);
}
