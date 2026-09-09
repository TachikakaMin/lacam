// PROTECTED static storage-topology route tests.
// Written after profiling showed PairCost re-running the same topological
// BFS on every rollout step.  Intentionally observed RED before the
// implementation on 2026-09-06.
#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

TEST(dd_storage_topology_routes,
     cached_routes_match_the_authoritative_topological_enumerator)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", ".@@@.", "....."});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (const auto& cell :
       std::vector<std::pair<int, int>>{
           {0, 0}, {0, 2}, {0, 4}, {2, 0}, {2, 2}, {2, 4}})
    ins.shelf_storage[ins.grid.idx(cell.first, cell.second)] = 1;
  ins.finalize();

  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  for (int source = 0; source < ins.grid.size(); ++source) {
    if (!ins.can_store_shelf(source)) continue;
    const auto expected =
        carrier_detail::reachable_storage_transfers(ins, source);
    const auto& cached =
        carrier_detail::storage_transfers_from_topology(
            topology, source);
    ASSERT_EQ(cached.size(), expected.size());
    for (size_t index = 0; index < expected.size(); ++index) {
      EXPECT_EQ(cached[index].endpoint, expected[index].endpoint);
      EXPECT_EQ(cached[index].route, expected[index].route);
    }
  }
}

TEST(dd_storage_topology_routes,
     vacancy_graph_keeps_only_the_nearest_exit_per_channel_entrance)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (const int col : {0, 2, 4})
    ins.shelf_storage[ins.grid.idx(0, col)] = 1;
  ins.finalize();

  const int source = ins.grid.idx(0, 0);
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto& exhaustive =
      carrier_detail::storage_transfers_from_topology(
          topology, source);
  ASSERT_EQ(exhaustive.size(), 2u);
  EXPECT_EQ(exhaustive[0].endpoint, ins.grid.idx(0, 2));
  EXPECT_EQ(exhaustive[1].endpoint, ins.grid.idx(0, 4));

  ASSERT_EQ(topology.outgoing[source].size(), 1u);
  EXPECT_EQ(
      topology.outgoing[source][0].endpoint,
      ins.grid.idx(0, 2));
  EXPECT_EQ(topology.outgoing[source][0].loaded_steps, 2);
}
