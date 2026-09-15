#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

namespace {

PairPlan only_plan(const DDInstance& ins)
{
  const auto upper =
      carrier_detail::make_upper_signature(
          initial_phys_config(ins));
  DDDistCache distance(ins.grid);
  const auto assignment =
      carrier_detail::build_congestion_pair_assignment(
          ins, upper, distance, 1, 1, 1);
  return assignment.table.at(0).at(0).plan;
}

DDInstance neighbor_count_instance(bool with_neighbor)
{
  DDInstance ins;
  ins.grid = DDGrid({
      ".....",
      ".....",
      ".....",
  });
  ins.shelf_storage.assign(ins.grid.size(), 1);
  const int source = ins.grid.idx(1, 0);
  ins.robots = {ins.grid.idx(2, 4)};
  ins.shelves = {source, ins.grid.idx(1, 2)};
  if (with_neighbor)
    ins.shelves.push_back(ins.grid.idx(0, 2));
  ins.target_starts = {source};
  ins.target_goals = {ins.grid.idx(1, 4)};
  ins.target_goal_sets = {{ins.grid.idx(1, 4)}};
  ins.finalize();
  return ins;
}

DDInstance two_target_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 4),
  };
  ins.target_starts = ins.shelves;
  ins.target_goals = {
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 3),
  };
  ins.target_goal_sets = {
      {ins.grid.idx(0, 1), ins.grid.idx(0, 3)},
      {ins.grid.idx(0, 1), ins.grid.idx(0, 3)},
  };
  ins.finalize();
  return ins;
}

}  // namespace

TEST(pair_congestion_core,
     equal_length_shortest_paths_choose_the_clear_route)
{
  DDInstance ins;
  ins.grid = DDGrid({
      "...",
      "...",
      "...",
  });
  ins.shelf_storage.assign(ins.grid.size(), 1);
  const int source = ins.grid.idx(0, 0);
  ins.robots = {ins.grid.idx(2, 0)};
  ins.shelves = {source, ins.grid.idx(0, 1)};
  ins.target_starts = {source};
  ins.target_goals = {ins.grid.idx(2, 2)};
  ins.target_goal_sets = {{ins.grid.idx(2, 2)}};
  ins.finalize();

  const PairPlan plan = only_plan(ins);
  EXPECT_EQ(plan.direct_distance, 4);
  EXPECT_DOUBLE_EQ(plan.estimated_cost, 6);
}

TEST(pair_congestion_core,
     a_shelf_beside_a_required_blocker_increases_the_estimate)
{
  const PairPlan isolated =
      only_plan(neighbor_count_instance(false));
  const PairPlan crowded =
      only_plan(neighbor_count_instance(true));

  EXPECT_DOUBLE_EQ(
      crowded.estimated_cost,
      isolated.estimated_cost + 1);
}

TEST(pair_congestion_core,
     one_hungarian_respects_multi_target_goal_commitments)
{
  const DDInstance ins = two_target_instance();
  const auto upper =
      carrier_detail::make_upper_signature(
          initial_phys_config(ins));
  DDDistCache free_distance(ins.grid);
  const auto free =
      carrier_detail::build_congestion_pair_assignment(
          ins, upper, free_distance, 1, 1, 1);
  EXPECT_EQ(
      free.tau,
      (std::vector<int>{
          ins.grid.idx(0, 1),
          ins.grid.idx(0, 3),
      }));
  EXPECT_EQ(free.hungarian_full_solves, 1);

  const RootGoalCommitment commitment{
      {0, ins.grid.idx(0, 3)},
  };
  DDDistCache committed_distance(ins.grid);
  const auto committed =
      carrier_detail::build_congestion_pair_assignment(
          ins, upper, committed_distance, 1, 1, 1,
          nullptr, &commitment);
  EXPECT_EQ(
      committed.tau,
      (std::vector<int>{
          ins.grid.idx(0, 3),
          ins.grid.idx(0, 1),
      }));
  EXPECT_EQ(committed.hungarian_full_solves, 1);
}

TEST(pair_congestion_core,
     production_upper_epoch_enters_task_br_without_pair_cost_rollout)
{
  const DDInstance ins = two_target_instance();
  const auto upper =
      carrier_detail::make_upper_signature(
          initial_phys_config(ins));
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  DDDistCache distance(ins.grid);
  const auto epoch =
      carrier_detail::build_task_br_upper_epoch(
          ins, upper, distance, topology, 1, 1, 1);

  EXPECT_EQ(epoch->pair_hungarian_full_solves, 1);
  EXPECT_EQ(epoch->pair_hungarian_forced_repairs, 0);
  EXPECT_EQ(epoch->pair_rollout_work_steps, 0);
  EXPECT_GT(epoch->task_graph.tasks.size(), 0u);
  for (const auto& row : epoch->pair_cost)
    for (const auto& entry : row)
      EXPECT_EQ(
          entry.plan.bound_stage,
          PairBoundStage::HEURISTIC_ESTIMATE);
}
