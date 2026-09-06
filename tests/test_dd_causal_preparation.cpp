#include "../lacam/src/carrier_guidance.hpp"

#include <dd_planner.hpp>

#include <algorithm>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance chain_instance(const std::vector<int>& robots)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  for (const int cell : robots)
    ins.robots.push_back(ins.grid.idx(0, cell));
  ins.shelves = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 1),
      ins.grid.idx(0, 2), ins.grid.idx(0, 3)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.target_goal_sets = {{ins.grid.idx(0, 4)}};
  ins.finalize();
  return ins;
}

std::vector<Op> preferred_ops(TAPFPlanner& planner,
                              const TAPFInstance& view,
                              const PhysConfig& physical,
                              const CarrierGuidance& guidance)
{
  Config config;
  for (const int cell : physical.robots)
    config.push_back(view.G.U[cell]);
  ShelfState shelf{
      physical.target_pos, physical.anon_occ, physical.kappa};
  auto node = std::make_unique<TAPFNode>(
      config, shelf, planner.D, &view,
      std::vector<int>(view.N, -1),
      TAPFAssignmentState(), nullptr);
  node->guide =
      std::make_unique<CarrierGuidance>(guidance);
  node->order.resize(view.N);
  std::iota(node->order.begin(), node->order.end(), 0);
  node->constraint_order = node->order;
  planner.invalidate_carrier_scratch();
  TAPFConstraint root;
  EXPECT_TRUE(planner.get_new_config(node.get(), &root));
  EXPECT_TRUE(planner.apply_carrier_effects(node.get()));
  return planner.ops_scratch;
}

}  // namespace

TEST(dd_causal_preparation,
     compiler_edges_wait_only_for_the_cell_each_consumer_enters)
{
  const auto ins = chain_instance({4});
  const auto graph =
      dd_compile_single_root_graph_probe(
          ins, initial_phys_config(ins), 0,
          ins.grid.idx(0, 4));

  ASSERT_EQ(graph.tasks.size(), 4u);
  ASSERT_EQ(graph.causal_edges.size(), 3u);
  for (const auto& edge : graph.causal_edges) {
    ASSERT_GE(edge.producer, 0);
    ASSERT_GE(edge.consumer, 0);
    ASSERT_LT(edge.producer, (int)graph.tasks.size());
    ASSERT_LT(edge.consumer, (int)graph.tasks.size());
    EXPECT_EQ(
        edge.kind, CausalEventKind::ENTER_CELL);
    EXPECT_EQ(
        edge.must_be_vacated,
        graph.tasks[edge.consumer].transfer.endpoint);
    EXPECT_NE(
        std::find(
            graph.predecessors[edge.consumer].begin(),
            graph.predecessors[edge.consumer].end(),
            edge.producer),
        graph.predecessors[edge.consumer].end());
  }
}

TEST(dd_causal_preparation,
     execute_and_prepare_are_dispatched_in_parallel_without_preferred_lift)
{
  const auto ins = chain_instance({3, 2});
  const auto physical = initial_phys_config(ins);
  const auto guidance =
      dd_task_br_guidance_probe(ins, physical);

  ASSERT_EQ(guidance.rho_mode.size(), 2u);
  ASSERT_EQ(
      guidance.rho_mode[0], DispatchMode::EXECUTE);
  ASSERT_EQ(
      guidance.rho_mode[1], DispatchMode::PREPARE);
  ASSERT_TRUE(guidance.rho_task_id[0].has_value());
  ASSERT_TRUE(guidance.rho_task_id[1].has_value());
  EXPECT_EQ(
      guidance.rho_task_id[0]->from,
      ins.grid.idx(0, 3));
  EXPECT_EQ(
      guidance.rho_task_id[1]->from,
      ins.grid.idx(0, 2));

  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto ops =
      preferred_ops(planner, view, physical, guidance);
  ASSERT_EQ(ops.size(), 2u);
  EXPECT_EQ(ops[0], Op::make_lift());
  EXPECT_NE(ops[1].kind, Op::LIFT);
}

TEST(dd_causal_preparation,
     a_single_robot_is_reserved_for_the_executable_predecessor)
{
  const auto ins = chain_instance({2});
  const auto physical = initial_phys_config(ins);
  const auto guidance =
      dd_task_br_guidance_probe(ins, physical);

  ASSERT_EQ(guidance.rho_mode.size(), 1u);
  ASSERT_TRUE(guidance.rho_task_id[0].has_value());
  EXPECT_EQ(
      guidance.rho_mode[0], DispatchMode::EXECUTE);
  EXPECT_EQ(
      guidance.rho_task_id[0]->from,
      ins.grid.idx(0, 3));

  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const auto ops =
      preferred_ops(planner, view, physical, guidance);
  ASSERT_EQ(ops.size(), 1u);
  EXPECT_NE(ops[0].kind, Op::LIFT);
}

TEST(dd_causal_preparation,
     vacate_condition_is_recomputed_after_an_unrelated_reoccupation)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 3)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 3)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.target_goal_sets = {{ins.grid.idx(0, 1)}};
  ins.finalize();
  ShelfTaskGraph graph;
  graph.tasks = {
      ShelfTask{
          TaskId{
              ShelfSelector{
                  ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
                  ins.grid.idx(0, 3)},
              ins.grid.idx(0, 3), ins.grid.idx(0, 2)},
          {}, 1,
          StorageTransfer{
              ins.grid.idx(0, 2),
              {ins.grid.idx(0, 3), ins.grid.idx(0, 2)}}},
      ShelfTask{
          TaskId{
              ShelfSelector{ShelfSelector::Kind::TARGET, 0},
              ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
          {}, 2,
          StorageTransfer{
              ins.grid.idx(0, 1),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}}},
  };
  graph.predecessors = {{}, {0}};
  graph.successors = {{1}, {}};
  graph.causal_edges = {
      CausalEdge{
          0, ins.grid.idx(0, 2), 1,
          CausalEventKind::ENTER_CELL}};

  PhysConfig empty = initial_phys_config(ins);
  empty.anon_occ = {ins.grid.idx(0, 3)};
  const auto released =
      carrier_detail::reconcile_execution_view(
          ins, empty, graph,
          std::vector<std::optional<Custody>>(1));
  ASSERT_EQ(released.causal_conditions.size(), 1u);
  EXPECT_TRUE(released.causal_conditions[0].fulfilled);

  PhysConfig reoccupied = empty;
  reoccupied.anon_occ = {ins.grid.idx(0, 2)};
  const auto blocked =
      carrier_detail::reconcile_execution_view(
          ins, reoccupied, graph,
          std::vector<std::optional<Custody>>(1));
  ASSERT_EQ(blocked.causal_conditions.size(), 1u);
  EXPECT_FALSE(blocked.causal_conditions[0].fulfilled);
}

TEST(dd_causal_preparation,
     current_empty_storage_is_distinct_from_static_storage_slack)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  ins.shelf_storage[ins.grid.idx(0, 0)] = 1;
  ins.shelf_storage[ins.grid.idx(0, 2)] = 1;
  ins.robots = {ins.grid.idx(1, 1)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 0)};
  ins.target_goal_sets = {{ins.grid.idx(0, 0)}};
  ins.finalize();

  UpperSignature upper;
  upper.target_pos = {ins.grid.idx(0, 0)};
  upper.anon_pos = {ins.grid.idx(1, 1)};
  EXPECT_EQ(
      carrier_detail::upper_vacancy_count(ins, upper), 0u);
  EXPECT_EQ(
      carrier_detail::empty_storage_cells(ins, upper),
      (std::vector<int>{ins.grid.idx(0, 2)}));
  const auto empty_transit =
      carrier_detail::empty_transit_cells(ins, upper);
  EXPECT_EQ(
      std::find(
          empty_transit.begin(), empty_transit.end(),
          ins.grid.idx(1, 1)),
      empty_transit.end());
}
