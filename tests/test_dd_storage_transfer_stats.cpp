// PROTECTED storage-transfer diagnostic regression.
// Written before implementation on 2026-09-04 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(dd_storage_transfer_stats,
     forced_transit_shelf_does_not_turn_an_aisle_into_a_vacancy)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.shelf_storage = {1, 0, 1};
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.target_goal_sets = {{ins.grid.idx(0, 2)}};
  ins.finalize();

  // The target was forced from storage 0 into transit 1 while another
  // shelf remains at storage 2.  One storage cell is physically exposed,
  // but there are still two shelves for two storage slots: no shelf can
  // be dropped without first completing/recovering a transfer.
  PhysConfig forced_transit;
  forced_transit.robots = {ins.grid.idx(0, 1)};
  forced_transit.target_pos = {ins.grid.idx(0, 1)};
  forced_transit.anon_occ = {ins.grid.idx(0, 2)};
  forced_transit.kappa = {0};
  const auto upper =
      carrier_detail::make_upper_signature(forced_transit);

  ASSERT_EQ(
      carrier_detail::upper_vacancy_count(ins, upper), 0u);
  EXPECT_TRUE(carrier_detail::zero_storage_vacancy_no_ready(
      ins, upper, /*ready_task_count=*/0, /*graph_task_count=*/1));
  EXPECT_FALSE(carrier_detail::zero_storage_vacancy_no_ready(
      ins, upper, /*ready_task_count=*/1, /*graph_task_count=*/1));
  EXPECT_FALSE(carrier_detail::zero_storage_vacancy_no_ready(
      ins, upper, /*ready_task_count=*/0, /*graph_task_count=*/0));

  auto one_vacancy = upper;
  one_vacancy.anon_pos.clear();
  ASSERT_EQ(
      carrier_detail::upper_vacancy_count(ins, one_vacancy), 1u);
  EXPECT_FALSE(carrier_detail::zero_storage_vacancy_no_ready(
      ins, one_vacancy,
      /*ready_task_count=*/0, /*graph_task_count=*/1));
}
