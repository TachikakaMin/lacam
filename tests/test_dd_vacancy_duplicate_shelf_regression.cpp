// PROTECTED joint Task-BR duplicate-shelf regression.
// Written after the quick benchmark timeout was traced to one graph
// containing two different transfers for target shelf 4.  Intentionally
// observed RED before the implementation fix on 2026-09-06.
#include "../lacam/src/carrier_guidance.hpp"

#include <br_lacam_upper.hpp>
#include <dd_carrier.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_duplicate_shelf_regression,
     joint_graph_reserves_one_complete_transfer_per_shelf)
{
  const auto ins = load_dd_instance(
      std::string(DD_TEST_DIR) +
      "/../benchmark/viz_web/warehouse_block_suite/instances/"
      "warehouse_blocks_h20w20_b9_a1_d25_r8_t12_seed0.yaml");
  const std::vector<std::pair<int, int>> goal_coordinates{
      {15, 9}, {7, 11}, {5, 9}, {17, 11},
      {3, 9}, {16, 9}, {14, 11}, {5, 11},
      {12, 9}, {19, 11}, {2, 12}, {1, 9}};
  std::vector<int> tau;
  tau.reserve(goal_coordinates.size());
  for (const auto& [row, col] : goal_coordinates)
    tau.push_back(ins.grid.idx(row, col));

  const auto labeled = br_labeled_initial_state(ins);
  const auto metadata = br_upper_root_metadata(labeled, tau);
  const auto upper_signature =
      project_labeled_upper_state(labeled);
  const auto upper =
      carrier_detail::make_abstract_upper_state(
          ins, upper_signature);
  std::vector<RootDemand> roots;
  for (size_t target = 0; target < tau.size(); ++target)
    if (labeled.target_pos[target] != tau[target])
      roots.push_back(RootDemand{(int)target, tau[target]});

  DDDistCache upper_wall(ins.grid);
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto graph = carrier_detail::compile_task_br_pibt(
      ins, upper, roots, tau, metadata.target_priority,
      upper_wall, topology,
      carrier_detail::TaskBRCompilerLimits{256, 512}, false);

  ASSERT_FALSE(graph.tasks.empty());
  std::map<ShelfSelector, int> task_count_by_shelf;
  for (const auto& task : graph.tasks)
    ++task_count_by_shelf[task.id.shelf];
  for (const auto& [shelf, count] : task_count_by_shelf)
    EXPECT_EQ(count, 1)
        << "shelf kind=" << static_cast<int>(shelf.kind)
        << " value=" << shelf.value;
}
