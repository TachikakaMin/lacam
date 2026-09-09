// PROTECTED stale forced-effect priority normalization contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_forced_effect_priority_normalization,
     current_epoch_priority_does_not_make_an_exact_task_non_reproducible)
{
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, 0};
  const std::vector<RootDemand> roots{
      RootDemand{0, 8},
      RootDemand{1, 9},
  };
  const TaskBRForcedEffect forced{
      shelf,
      TaskBRForcedEffect::Kind::TRANSFER,
      StorageTransfer{3, {1, 2, 3}},
      roots,
      5};
  ShelfTaskGraph graph;
  graph.tasks.push_back(
      ShelfTask{
          TaskId{shelf, 1, 2},
          roots,
          8,
          forced.transfer});
  graph.predecessors.emplace_back();
  graph.successors.emplace_back();

  EXPECT_TRUE(
      carrier_detail::
          forced_effects_reproduced_by_unforced_prefix(
              {forced}, graph));
}
