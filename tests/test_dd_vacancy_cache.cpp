// PROTECTED vacancy-potential cache tests.
// Written after PairCost repeatedly rebuilt the same potential and consumed
// the h20 quick-case deadline.  Intentionally observed RED before the
// implementation on 2026-09-06.
#include "../lacam/src/carrier_guidance.hpp"

#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_dense_line()
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  for (int col = 0; col < 4; ++col)
    ins.shelves.push_back(ins.grid.idx(0, col));
  ins.finalize();
  return ins;
}

carrier_detail::AbstractUpperState make_upper(
    const DDInstance& ins, const std::vector<int>& columns)
{
  UpperSignature signature;
  for (const int col : columns)
    signature.anon_pos.push_back(ins.grid.idx(0, col));
  return carrier_detail::make_abstract_upper_state(ins, signature);
}

carrier_detail::StorageTransferTopology make_weighted_topology(
    int cell_count)
{
  carrier_detail::StorageTransferTopology topology;
  topology.transfers.resize(cell_count);
  topology.outgoing.resize(cell_count);
  topology.reverse.resize(cell_count);
  const auto add_arc = [&](int source, int endpoint,
                           int loaded_steps) {
    const carrier_detail::StorageTransferArc arc{
        source, endpoint, loaded_steps};
    topology.outgoing[source].push_back(arc);
    topology.reverse[endpoint].push_back(arc);
  };
  const auto add_both = [&](int a, int b, int loaded_steps) {
    add_arc(a, b, loaded_steps);
    add_arc(b, a, loaded_steps);
  };
  add_both(0, 1, 1);
  add_both(1, 2, 4);
  add_both(2, 3, 1);
  add_both(3, 4, 3);
  add_both(4, 5, 1);
  add_both(0, 3, 5);
  add_both(1, 4, 5);
  add_both(2, 5, 5);
  return topology;
}

}  // namespace

TEST(dd_vacancy_cache, reuses_identical_upper_and_rebuilds_after_motion)
{
  const auto ins = make_dense_line();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::VacancyPotentialCache cache(ins.grid.size());

  const auto first_upper = make_upper(ins, {0, 1, 2, 3});
  bool cutoff = false;
  const auto& first =
      carrier_detail::cached_vacancy_potential(
          ins, first_upper, topology, cache, nullptr, &cutoff);
  ASSERT_FALSE(cutoff);
  const auto first_cost = first.cost;
  const auto& repeated =
      carrier_detail::cached_vacancy_potential(
          ins, first_upper, topology, cache, nullptr, &cutoff);
  ASSERT_FALSE(cutoff);
  EXPECT_EQ(repeated.cost, first_cost);
  EXPECT_EQ(cache.builds, 1);
  EXPECT_EQ(cache.hits, 1);

  const auto moved_upper = make_upper(ins, {0, 1, 2, 4});
  const auto& moved =
      carrier_detail::cached_vacancy_potential(
          ins, moved_upper, topology, cache, nullptr, &cutoff);
  ASSERT_FALSE(cutoff);
  const auto rebuilt =
      carrier_detail::build_vacancy_potential(
          ins, moved_upper, topology);
  EXPECT_NE(moved.cost, first_cost);
  EXPECT_EQ(moved.cost, rebuilt.cost);
  EXPECT_EQ(
      moved.next_vacancy_cell,
      rebuilt.next_vacancy_cell);
  EXPECT_EQ(
      moved.vacancy_source_cell,
      rebuilt.vacancy_source_cell);
  EXPECT_EQ(cache.builds, 2);
  EXPECT_EQ(cache.hits, 1);
  EXPECT_EQ(cache.incremental_updates, 1);
}

TEST(dd_vacancy_cache,
     incremental_source_swaps_match_full_rebuild_over_a_sequence)
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  for (int col = 0; col < 4; ++col)
    ins.shelves.push_back(ins.grid.idx(0, col));
  ins.finalize();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::VacancyPotentialCache cache(ins.grid.size());

  const std::vector<std::vector<int>> states{
      {0, 1, 2, 3},
      {0, 1, 2, 4},
      {0, 1, 4, 5},
      {0, 2, 4, 5},
      {1, 2, 4, 5},
  };
  bool cutoff = false;
  for (const auto& columns : states) {
    const auto upper = make_upper(ins, columns);
    const auto& incremental =
        carrier_detail::cached_vacancy_potential(
            ins, upper, topology, cache, nullptr, &cutoff);
    ASSERT_FALSE(cutoff);
    const auto rebuilt =
        carrier_detail::build_vacancy_potential(
            ins, upper, topology);
    EXPECT_EQ(incremental.cost, rebuilt.cost);
    EXPECT_EQ(
        incremental.next_vacancy_cell,
        rebuilt.next_vacancy_cell);
    EXPECT_EQ(
        incremental.vacancy_source_cell,
        rebuilt.vacancy_source_cell);
    EXPECT_EQ(
        incremental.settled_target_moves,
        rebuilt.settled_target_moves);
  }
  EXPECT_EQ(cache.incremental_updates, 4);
}

TEST(dd_vacancy_cache,
     transit_motion_does_not_invalidate_storage_vacancy_key)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  ins.shelf_storage[ins.grid.idx(0, 0)] = 1;
  ins.shelf_storage[ins.grid.idx(0, 3)] = 1;
  // The instance itself must start with shelves in storage.  Abstract upper
  // states may subsequently represent that shelf while it is in transit.
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.finalize();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::VacancyPotentialCache cache(ins.grid.size());
  bool cutoff = false;

  const auto first_upper = make_upper(ins, {1});
  (void)carrier_detail::cached_vacancy_potential(
      ins, first_upper, topology, cache, nullptr, &cutoff);
  ASSERT_FALSE(cutoff);
  const auto moved_upper = make_upper(ins, {2});
  (void)carrier_detail::cached_vacancy_potential(
      ins, moved_upper, topology, cache, nullptr, &cutoff);
  ASSERT_FALSE(cutoff);

  EXPECT_EQ(cache.builds, 1);
  EXPECT_EQ(cache.hits, 1);
  EXPECT_EQ(cache.incremental_updates, 0);
}

TEST(dd_vacancy_cache,
     weighted_source_moves_match_full_dijkstra_rebuild)
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  for (int col = 0; col < 3; ++col)
    ins.shelves.push_back(ins.grid.idx(0, col));
  ins.finalize();
  const auto topology =
      make_weighted_topology(ins.grid.size());
  carrier_detail::VacancyPotentialCache cache(ins.grid.size());

  std::vector<std::vector<int>> states;
  for (int a = 0; a < 4; ++a)
    for (int b = a + 1; b < 5; ++b)
      for (int c = b + 1; c < 6; ++c)
        states.push_back({a, b, c});
  bool cutoff = false;
  for (const auto& columns : states) {
    const auto upper = make_upper(ins, columns);
    const auto& incremental =
        carrier_detail::cached_vacancy_potential(
            ins, upper, topology, cache, nullptr, &cutoff);
    ASSERT_FALSE(cutoff);
    const auto rebuilt =
        carrier_detail::build_vacancy_potential(
            ins, upper, topology);
    EXPECT_EQ(incremental.cost, rebuilt.cost);
    EXPECT_EQ(
        incremental.next_vacancy_cell,
        rebuilt.next_vacancy_cell);
    EXPECT_EQ(
        incremental.vacancy_source_cell,
        rebuilt.vacancy_source_cell);
    EXPECT_EQ(
        incremental.settled_target_moves,
        rebuilt.settled_target_moves);
  }
  ASSERT_EQ(states.size(), 20);
  EXPECT_EQ(cache.incremental_updates, 19);
}
