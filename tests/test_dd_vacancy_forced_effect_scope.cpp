// PROTECTED in-flight forced-effect scope contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

namespace {

TaskBRForcedEffect forced_transfer(
    const ShelfSelector& shelf,
    const RootDemand& root)
{
  return TaskBRForcedEffect{
      shelf,
      TaskBRForcedEffect::Kind::TRANSFER,
      StorageTransfer{2, {1, 2}},
      {root},
      1};
}

ShelfTaskGraph compiled_graph(
    const ShelfSelector& shelf,
    std::vector<RootDemand> roots)
{
  ShelfTaskGraph graph;
  graph.tasks.push_back(
      ShelfTask{
          TaskId{shelf, 1, 2},
          std::move(roots),
          1,
          StorageTransfer{2, {1, 2}}});
  graph.predecessors.emplace_back();
  graph.successors.emplace_back();
  return graph;
}

}  // namespace

TEST(dd_vacancy_forced_effect_scope,
     isolated_single_root_transfer_uses_existing_custody_path)
{
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, 0};
  const RootDemand root{0, 9};
  const std::vector<TaskBRForcedEffect> forced{
      forced_transfer(shelf, root)};
  const auto graph = compiled_graph(shelf, {root});

  EXPECT_TRUE(
      carrier_detail::shared_forced_effects_for_compiled_graph(
          forced, graph)
          .empty());
}

TEST(dd_vacancy_forced_effect_scope,
     transfer_that_acquires_another_root_remains_forced)
{
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, 0};
  const RootDemand carried_root{0, 9};
  const RootDemand dependent_root{1, 8};
  const std::vector<TaskBRForcedEffect> forced{
      forced_transfer(shelf, carried_root)};
  const auto graph =
      compiled_graph(shelf, {carried_root, dependent_root});

  EXPECT_EQ(
      carrier_detail::shared_forced_effects_for_compiled_graph(
          forced, graph),
      forced);
}
