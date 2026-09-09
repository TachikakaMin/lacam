// PROTECTED redundant in-flight forced-effect normalization contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

namespace {

TaskBRForcedEffect forced_transfer()
{
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, 0};
  return TaskBRForcedEffect{
      shelf,
      TaskBRForcedEffect::Kind::TRANSFER,
      StorageTransfer{3, {1, 2, 3}},
      {RootDemand{0, 8}, RootDemand{1, 9}},
      7};
}

ShelfTaskGraph reproduced_graph(
    const TaskBRForcedEffect& forced)
{
  ShelfTaskGraph graph;
  graph.tasks.push_back(
      ShelfTask{
          TaskId{
              forced.shelf,
              forced.transfer.route.front(),
              forced.transfer.route[1]},
          forced.roots,
          forced.priority,
          forced.transfer});
  graph.predecessors.emplace_back();
  graph.successors.emplace_back();
  return graph;
}

}  // namespace

TEST(dd_vacancy_forced_effect_redundancy,
     exact_unforced_graph_prefix_needs_no_forcing)
{
  const auto forced = forced_transfer();
  const std::vector<TaskBRForcedEffect> effects{forced};

  EXPECT_TRUE(
      carrier_detail::
          forced_effects_reproduced_by_unforced_prefix(
              effects, reproduced_graph(forced)));
}

TEST(dd_vacancy_forced_effect_redundancy,
     a_root_or_dependency_difference_keeps_the_effect)
{
  const auto forced = forced_transfer();
  const std::vector<TaskBRForcedEffect> effects{forced};
  auto different_roots = reproduced_graph(forced);
  different_roots.tasks[0].roots.pop_back();
  EXPECT_FALSE(
      carrier_detail::
          forced_effects_reproduced_by_unforced_prefix(
              effects, different_roots));

  auto dependent = reproduced_graph(forced);
  dependent.predecessors[0].push_back(1);
  EXPECT_FALSE(
      carrier_detail::
          forced_effects_reproduced_by_unforced_prefix(
              effects, dependent));
}
