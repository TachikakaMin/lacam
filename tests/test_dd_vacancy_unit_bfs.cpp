// PROTECTED vacancy-potential unit-cost dispatch tests.
// Written after profiling showed priority-queue Dijkstra dominating dense
// storage PairCost.  Intentionally observed RED before the implementation
// on 2026-09-06.
#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_storage(
    const std::vector<std::string>& map,
    const std::vector<std::string>& storage)
{
  DDInstance ins;
  ins.grid = DDGrid(map);
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (int row = 0; row < ins.grid.height; ++row)
    for (int col = 0; col < ins.grid.width; ++col)
      ins.shelf_storage[ins.grid.idx(row, col)] =
          storage[row][col] == 'S';
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_vacancy_unit_bfs,
     selects_bfs_only_when_every_storage_transfer_has_unit_length)
{
  const auto dense = make_storage(
      {"....", "...."}, {"SSSS", "SSSS"});
  const auto dense_topology =
      carrier_detail::build_storage_transfer_topology(dense);
  EXPECT_TRUE(
      carrier_detail::vacancy_topology_uses_unit_cost_bfs(
          dense_topology));

  const auto channel = make_storage(
      {"....."}, {"S.S.S"});
  const auto channel_topology =
      carrier_detail::build_storage_transfer_topology(channel);
  EXPECT_FALSE(
      carrier_detail::vacancy_topology_uses_unit_cost_bfs(
          channel_topology));
}
