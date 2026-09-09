// PROTECTED vacancy-guidance churn telemetry contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

namespace {

ShelfTask make_task(
    const ShelfSelector& shelf, int from, int to,
    const RootDemand& root)
{
  return ShelfTask{
      TaskId{shelf, from, to},
      {root},
      1,
      StorageTransfer{to, {from, to}},
  };
}

ShelfTaskGraph make_previous_graph(
    const RootDemand& root0, const RootDemand& root1)
{
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 4},
          4, 3, root0),
      make_task(
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          0, 4, root0),
      make_task(
          ShelfSelector{ShelfSelector::Kind::TARGET, 1},
          1, 8, root1),
  };
  graph.predecessors = {{}, {0}, {}};
  graph.successors = {{1}, {}, {}};
  return graph;
}

ShelfTaskGraph make_current_graph(
    const RootDemand& root0, const RootDemand& root1)
{
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 4},
          4, 2, root0),
      make_task(
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          0, 4, root0),
      make_task(
          ShelfSelector{ShelfSelector::Kind::TARGET, 1},
          1, 8, root1),
  };
  graph.predecessors = {{}, {0}, {}};
  graph.successors = {{1}, {}, {}};
  return graph;
}

}  // namespace

TEST(dd_vacancy_churn_telemetry,
     compares_first_transfers_and_chain_jaccard_per_shared_root)
{
  const RootDemand root0{0, 9};
  const RootDemand root1{1, 8};
  const auto previous = make_previous_graph(root0, root1);
  const auto current = make_current_graph(root0, root1);

  const auto telemetry =
      carrier_detail::compare_vacancy_epoch_churn(
          previous, current);

  EXPECT_EQ(telemetry.first_transfer_comparisons, 2);
  EXPECT_EQ(telemetry.first_transfer_flips, 1);
  EXPECT_EQ(telemetry.chain_overlap_samples, 2);
  EXPECT_EQ(telemetry.chain_overlap_intersection, 2);
  EXPECT_EQ(telemetry.chain_overlap_union, 4);
}
