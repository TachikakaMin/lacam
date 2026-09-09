// PROTECTED root-frontier forced-effect normalization contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_forced_effect_frontier_normalization,
     an_alternative_ready_transfer_for_the_same_roots_is_sufficient)
{
  const std::vector<RootDemand> roots{
      RootDemand{0, 8},
      RootDemand{1, 9},
  };
  const TaskBRForcedEffect forced{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      TaskBRForcedEffect::Kind::TRANSFER,
      StorageTransfer{3, {1, 2, 3}},
      roots,
      5};
  ShelfTaskGraph graph;
  graph.tasks.push_back(
      ShelfTask{
          TaskId{
              ShelfSelector{
                  ShelfSelector::Kind::TARGET, 1},
              4,
              5},
          roots,
          8,
          StorageTransfer{5, {4, 5}}});
  graph.predecessors.emplace_back();
  graph.successors.emplace_back();

  EXPECT_TRUE(
      carrier_detail::
          forced_effects_reproduced_by_unforced_frontier(
              {forced}, graph));

  graph.tasks[0].roots.pop_back();
  EXPECT_FALSE(
      carrier_detail::
          forced_effects_reproduced_by_unforced_frontier(
              {forced}, graph));
}
