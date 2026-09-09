// PROTECTED vacancy-aware goal-source regression tests.
// Written after the quick benchmark exposed BR-upper timeouts on
// 2026-09-06 and intentionally observed RED before the implementation fix.
#include "../lacam/src/carrier_guidance.hpp"

#include <br_lacam_upper.hpp>
#include <dd_carrier.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Cell = std::pair<int, int>;

DDInstance make_storage_line(const std::vector<int>& occupied)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  for (const int col : occupied)
    ins.shelves.push_back(ins.grid.idx(0, col));
  ins.finalize();
  return ins;
}

carrier_detail::AbstractUpperState make_anonymous_upper(
    const DDInstance& ins, const std::vector<int>& occupied)
{
  UpperSignature signature;
  for (const int col : occupied)
    signature.anon_pos.push_back(ins.grid.idx(0, col));
  std::sort(signature.anon_pos.begin(), signature.anon_pos.end());
  return carrier_detail::make_abstract_upper_state(ins, signature);
}

}  // namespace

TEST(dd_vacancy_goal_source_regression,
     protected_goal_is_not_a_vacancy_source_but_does_not_cut_the_graph)
{
  {
    const std::vector<int> occupied{0, 1, 3};
    const auto ins = make_storage_line(occupied);
    const auto upper = make_anonymous_upper(ins, occupied);
    const auto topology =
        carrier_detail::build_storage_transfer_topology(ins);
    const int protected_goal = ins.grid.idx(0, 2);
    const auto potential =
        carrier_detail::build_vacancy_potential(
            ins, upper, topology, {protected_goal});

    // Cell 2 is empty, but reserving it as a target goal means it must not
    // act as the zero-cost vacancy next to cell 1.  The usable vacancy is
    // cell 4, three pushes away.
    EXPECT_EQ(
        potential.anonymous_cost[ins.grid.idx(0, 1)].pushes, 3);
  }

  {
    const std::vector<int> occupied{0, 1, 2, 3};
    const auto ins = make_storage_line(occupied);
    const auto upper = make_anonymous_upper(ins, occupied);
    const auto topology =
        carrier_detail::build_storage_transfer_topology(ins);
    const int protected_goal = ins.grid.idx(0, 2);
    const auto potential =
        carrier_detail::build_vacancy_potential(
            ins, upper, topology, {protected_goal});

    // An occupied protected cell is still traversable by the abstract
    // vacancy propagation.  Treating it as a wall disconnects cells 0/1
    // and caused the upper search to fall back to endpoint-id ordering.
    const auto& cost =
        potential.anonymous_cost[ins.grid.idx(0, 0)];
    ASSERT_TRUE(cost.finite());
    EXPECT_EQ(cost.pushes, 4);
  }
}

TEST(dd_vacancy_goal_source_regression,
     warehouse_upper_search_no_longer_times_out_on_goal_barriers)
{
  const auto ins = load_dd_instance(
      std::string(DD_TEST_DIR) +
      "/../benchmark/viz_web/warehouse_block_suite/instances/"
      "warehouse_blocks_h20w20_b9_a1_d25_r8_t12_seed0.yaml");
  const std::vector<Cell> goal_coordinates{
      {15, 9}, {7, 11}, {5, 9}, {17, 11},
      {3, 9}, {16, 9}, {14, 11}, {5, 11},
      {12, 9}, {19, 11}, {2, 12}, {1, 9}};
  std::vector<int> tau;
  tau.reserve(goal_coordinates.size());
  for (const auto& [row, col] : goal_coordinates)
    tau.push_back(ins.grid.idx(row, col));

  Deadline deadline(1000);
  const auto result = solve_carrier_br_upper(
      ins, br_labeled_initial_state(ins), tau, &deadline);

  ASSERT_TRUE(result.solved());
}
