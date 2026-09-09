// PROTECTED in-flight graph-seeding scope contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(dd_vacancy_forced_effect_storage_scope,
     only_a_non_storage_carrying_source_needs_graph_seeding)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.shelf_storage = {1, 0, 1};

  EXPECT_FALSE(
      carrier_detail::in_flight_effect_requires_graph_seed(
          ins, ins.grid.idx(0, 0)));
  EXPECT_TRUE(
      carrier_detail::in_flight_effect_requires_graph_seed(
          ins, ins.grid.idx(0, 1)));
}
