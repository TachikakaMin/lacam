// PROTECTED storage-aware Task-BR regression tests.
// Written before implementation on 2026-09-04 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"
#include <dd_planner.hpp>

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Cell = std::pair<int, int>;

DDInstance make_instance(
    const std::vector<std::string>& rows,
    const std::vector<std::string>& storage_rows,
    const std::vector<Cell>& robots,
    const std::vector<Cell>& shelves,
    const std::vector<std::pair<Cell, std::vector<Cell>>>& targets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  if (!storage_rows.empty()) {
    EXPECT_EQ(storage_rows.size(), rows.size());
    ins.shelf_storage.assign(ins.grid.size(), 0);
    for (int r = 0; r < ins.grid.height; ++r) {
      EXPECT_EQ(storage_rows[r].size(), rows[r].size());
      for (int c = 0; c < ins.grid.width; ++c)
        ins.shelf_storage[ins.grid.idx(r, c)] =
            storage_rows[r][c] == 'S';
    }
  }
  for (const auto& [r, c] : robots)
    ins.robots.push_back(ins.grid.idx(r, c));
  for (const auto& [r, c] : shelves)
    ins.shelves.push_back(ins.grid.idx(r, c));
  for (const auto& target : targets) {
    ins.target_starts.push_back(
        ins.grid.idx(target.first.first, target.first.second));
    std::vector<int> goals;
    for (const auto& [r, c] : target.second)
      goals.push_back(ins.grid.idx(r, c));
    ins.target_goal_sets.push_back(goals);
    ins.target_goals.push_back(goals.front());
  }
  ins.finalize();
  return ins;
}

int find_task(const ShelfTaskGraph& graph, const ShelfSelector& shelf,
              int from)
{
  for (size_t index = 0; index < graph.tasks.size(); ++index)
    if (graph.tasks[index].id.shelf == shelf &&
        graph.tasks[index].id.from == from)
      return (int)index;
  return -1;
}

void expect_valid_transfer(const DDInstance& ins,
                           const StorageTransfer& transfer)
{
  ASSERT_GE(transfer.endpoint, 0);
  ASSERT_TRUE(ins.can_store_shelf(transfer.endpoint));
  ASSERT_GE(transfer.route.size(), 2u);
  EXPECT_EQ(transfer.route.back(), transfer.endpoint);
  for (size_t index = 1; index < transfer.route.size(); ++index)
    EXPECT_TRUE(carrier_detail::adjacent_cells(
        ins.grid, transfer.route[index - 1], transfer.route[index]));
  for (size_t index = 1; index + 1 < transfer.route.size(); ++index)
    EXPECT_FALSE(ins.can_store_shelf(transfer.route[index]))
        << "a transfer may cross transit cells but may not pass through "
           "an intermediate storage endpoint";
}

}  // namespace

TEST(dd_storage_transfer, transit_cells_are_not_upper_vacancies)
{
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 0}}, {{0, 0}, {0, 2}},
      {{{0, 0}, {{0, 4}}}});
  const auto upper =
      carrier_detail::make_upper_signature(initial_phys_config(ins));
  EXPECT_EQ(carrier_detail::upper_vacancy_count(ins, upper), 1u);
}

TEST(dd_storage_transfer,
     occupied_endpoint_is_cleared_to_storage_before_requester_starts)
{
  // target at storage 0 wants occupied storage 2; the anonymous blocker
  // must commit to 2->3->4 before target commits to 0->1->2.
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 2}}, {{0, 0}, {0, 2}},
      {{{0, 0}, {{0, 2}}}});
  const auto graph = dd_compile_single_root_graph_probe(
      ins, initial_phys_config(ins), 0, ins.grid.idx(0, 2));

  const ShelfSelector target{ShelfSelector::Kind::TARGET, 0};
  const ShelfSelector blocker{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL, ins.grid.idx(0, 2)};
  const int target_task =
      find_task(graph, target, ins.grid.idx(0, 0));
  const int blocker_task =
      find_task(graph, blocker, ins.grid.idx(0, 2));
  ASSERT_GE(target_task, 0);
  ASSERT_GE(blocker_task, 0);

  EXPECT_EQ(graph.tasks[target_task].id.to, ins.grid.idx(0, 1));
  EXPECT_EQ(graph.tasks[target_task].transfer.endpoint,
            ins.grid.idx(0, 2));
  EXPECT_EQ(graph.tasks[target_task].transfer.route,
            (std::vector<int>{
                ins.grid.idx(0, 0), ins.grid.idx(0, 1),
                ins.grid.idx(0, 2)}));
  EXPECT_EQ(graph.tasks[blocker_task].id.to, ins.grid.idx(0, 3));
  EXPECT_EQ(graph.tasks[blocker_task].transfer.endpoint,
            ins.grid.idx(0, 4));
  EXPECT_EQ(graph.tasks[blocker_task].transfer.route,
            (std::vector<int>{
                ins.grid.idx(0, 2), ins.grid.idx(0, 3),
                ins.grid.idx(0, 4)}));
  ASSERT_EQ(graph.predecessors[target_task].size(), 1u);
  EXPECT_EQ(graph.predecessors[target_task][0], blocker_task);
  EXPECT_TRUE(graph.predecessors[blocker_task].empty());
}

TEST(dd_storage_transfer, pair_cost_executes_a_whole_cross_aisle_transfer)
{
  const auto ins = make_instance(
      {"...."}, {"S..S"}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {{0, 3}}}});
  const auto plan = dd_pair_cost_probe(
      ins, initial_phys_config(ins), 0, ins.grid.idx(0, 3));
  EXPECT_TRUE(plan.reached_goal);
  EXPECT_FALSE(plan.stalled);
  EXPECT_FALSE(plan.truncated);
  EXPECT_EQ(plan.rollout_steps, 3);

  const auto graph = dd_compile_single_root_graph_probe(
      ins, initial_phys_config(ins), 0, ins.grid.idx(0, 3));
  ASSERT_EQ(graph.tasks.size(), 1u);
  expect_valid_transfer(ins, graph.tasks[0].transfer);
  EXPECT_EQ(graph.tasks[0].transfer.route,
            (std::vector<int>{
                ins.grid.idx(0, 0), ins.grid.idx(0, 1),
                ins.grid.idx(0, 2), ins.grid.idx(0, 3)}));
}

TEST(dd_storage_transfer,
     custody_keeps_endpoint_when_new_epoch_no_longer_needs_blocker)
{
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 2}}, {{0, 0}, {0, 2}},
      {{{0, 0}, {{0, 2}}}});
  auto previous = initial_phys_config(ins);
  const auto lifted = apply_ops(ins, previous, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  previous = *lifted;

  const ShelfTask blocker_task{
      TaskId{
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 2)},
          ins.grid.idx(0, 2),
          ins.grid.idx(0, 3)},
      {RootDemand{0, ins.grid.idx(0, 2)}},
      1,
      StorageTransfer{
          ins.grid.idx(0, 4),
          {ins.grid.idx(0, 2), ins.grid.idx(0, 3),
           ins.grid.idx(0, 4)}}};
  auto previous_epoch = std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->task_graph.tasks = {blocker_task};
  previous_epoch->task_graph.predecessors = {{}};
  previous_epoch->task_graph.successors = {{}};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      carrier_detail::make_custody(blocker_task, 0)};

  const std::vector<Op> first_leg = {
      Op::make_move(ins.grid.idx(0, 3))};
  const auto in_aisle = apply_ops(ins, previous, first_leg);
  ASSERT_TRUE(in_aisle.has_value());
  const ShelfTaskGraph empty_new_graph;
  const auto recovered = carrier_detail::recover_task_br_custody(
      ins, *in_aisle, empty_new_graph, &previous, &previous_guidance,
      &first_leg);
  ASSERT_EQ(recovered.custody_by_robot.size(), 1u);
  ASSERT_TRUE(recovered.custody_by_robot[0].has_value());
  const auto& continued = *recovered.custody_by_robot[0];
  EXPECT_EQ(continued.transfer.endpoint, ins.grid.idx(0, 4));
  EXPECT_EQ(continued.transfer.route, blocker_task.transfer.route);
  EXPECT_EQ(continued.transfer_index, 1u);
  EXPECT_EQ(continued.task_id.from, ins.grid.idx(0, 3));
  EXPECT_EQ(continued.task_id.to, ins.grid.idx(0, 4));

  CarrierGuidance middle_guidance;
  middle_guidance.upper_epoch =
      std::make_shared<UpperEpochGuidance>();
  middle_guidance.custody_by_robot = recovered.custody_by_robot;
  const std::vector<Op> second_leg = {
      Op::make_move(ins.grid.idx(0, 4))};
  const auto at_endpoint = apply_ops(ins, *in_aisle, second_leg);
  ASSERT_TRUE(at_endpoint.has_value());
  const auto completed = carrier_detail::recover_task_br_custody(
      ins, *at_endpoint, empty_new_graph, &*in_aisle, &middle_guidance,
      &second_leg);
  EXPECT_FALSE(completed.custody_by_robot[0].has_value());
  EXPECT_TRUE(ins.can_store_shelf(at_endpoint->robots[0]));
}

TEST(dd_storage_transfer,
     zero_storage_vacancy_does_not_start_a_transit_dead_end)
{
  const auto ins = make_instance(
      {"..."}, {"S.S"}, {{0, 0}}, {{0, 0}, {0, 2}},
      {{{0, 0}, {{0, 2}}}});
  const auto X = initial_phys_config(ins);
  const auto upper = carrier_detail::make_upper_signature(X);
  ASSERT_EQ(carrier_detail::upper_vacancy_count(ins, upper), 0u);
  const auto graph = dd_compile_single_root_graph_probe(
      ins, X, 0, ins.grid.idx(0, 2));
  EXPECT_TRUE(graph.tasks.empty());
  const auto plan =
      dd_pair_cost_probe(ins, X, 0, ins.grid.idx(0, 2));
  EXPECT_TRUE(std::isfinite(plan.estimated_cost));
  EXPECT_TRUE(plan.stalled || plan.truncated);
}

TEST(dd_storage_transfer,
     incompatible_roots_cannot_reserve_the_same_storage_endpoint)
{
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 0}, {0, 4}},
      {{0, 0}, {0, 4}},
      {
          {{0, 0}, {{0, 0}, {0, 2}}},
          {{0, 4}, {{0, 2}, {0, 4}}},
      });
  const std::vector<int> tau = {
      ins.grid.idx(0, 2), ins.grid.idx(0, 2)};
  const std::vector<int> priority = {2, 1};
  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins), &tau, &priority);

  std::set<int> endpoints;
  for (const auto& task : graph.tasks) {
    expect_valid_transfer(ins, task.transfer);
    EXPECT_TRUE(endpoints.insert(task.transfer.endpoint).second)
        << "two accepted transfers reserved the same storage endpoint";
  }
  EXPECT_EQ(graph.tasks.size(), 1u);
  EXPECT_EQ(graph.paused_roots.size(), 1u);
}

TEST(dd_storage_transfer,
     forced_deviation_builds_recovery_from_the_actual_transit_cell)
{
  const auto ins = make_instance(
      {"....", "....", "...."},
      {"S..S", "....", "...S"},
      {{0, 0}}, {{0, 0}},
      {{{0, 0}, {{2, 3}}}});
  auto state = initial_phys_config(ins);
  auto next = apply_ops(ins, state, {Op::make_lift()});
  ASSERT_TRUE(next.has_value());
  state = *next;
  next = apply_ops(ins, state, {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(next.has_value());
  state = *next;
  next = apply_ops(ins, state, {Op::make_move(ins.grid.idx(0, 2))});
  ASSERT_TRUE(next.has_value());
  state = *next;

  Custody old;
  old.task_id = TaskId{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 3)};
  old.shelf = old.task_id.shelf;
  old.from = old.task_id.from;
  old.to = old.task_id.to;
  old.priority = 1;
  old.transfer = StorageTransfer{
      ins.grid.idx(0, 3),
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2), ins.grid.idx(0, 3)}};
  old.transfer_index = 2;
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_guidance.custody_by_robot = {old};

  const std::vector<Op> deviation = {
      Op::make_move(ins.grid.idx(1, 2))};
  const auto deviated = apply_ops(ins, state, deviation);
  ASSERT_TRUE(deviated.has_value());
  ASSERT_FALSE(ins.can_store_shelf(deviated->robots[0]));
  const ShelfTaskGraph empty_graph;
  const auto recovered = carrier_detail::recover_task_br_custody(
      ins, *deviated, empty_graph, &state, &previous_guidance,
      &deviation);
  ASSERT_TRUE(recovered.custody_by_robot[0].has_value());
  const auto& recovery = *recovered.custody_by_robot[0];
  expect_valid_transfer(ins, recovery.transfer);
  EXPECT_EQ(recovery.transfer.route.front(), deviated->robots[0]);
  EXPECT_EQ(recovery.task_id.from, deviated->robots[0]);
  EXPECT_EQ(recovery.task_id.to, recovery.transfer.route[1]);
  EXPECT_EQ(recovery.transfer_index, 0u);
}

TEST(dd_storage_transfer, legacy_map_degenerates_to_one_adjacent_leg)
{
  const auto ins = make_instance(
      {"..."}, {}, {{0, 0}}, {{0, 0}},
      {{{0, 0}, {{0, 2}}}});
  const auto graph = dd_compile_single_root_graph_probe(
      ins, initial_phys_config(ins), 0, ins.grid.idx(0, 2));
  ASSERT_EQ(graph.tasks.size(), 1u);
  EXPECT_EQ(graph.tasks[0].id.from, ins.grid.idx(0, 0));
  EXPECT_EQ(graph.tasks[0].id.to, ins.grid.idx(0, 1));
  EXPECT_EQ(graph.tasks[0].transfer.endpoint, ins.grid.idx(0, 1));
  EXPECT_EQ(graph.tasks[0].transfer.route,
            (std::vector<int>{
                ins.grid.idx(0, 0), ins.grid.idx(0, 1)}));
}

TEST(dd_storage_transfer,
     preferred_plan_rehomes_blocker_without_returning_to_its_origin)
{
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 2}}, {{0, 0}, {0, 2}},
      {{{0, 0}, {{0, 2}}}});
  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 2.0, 0, &stats);
  ASSERT_FALSE(plan.empty());

  auto state = initial_phys_config(ins);
  int episode_origin = -1;
  bool crossed_transit = false;
  for (const auto& ops : plan) {
    ASSERT_EQ(ops.size(), 1u);
    if (ops[0].kind == Op::LIFT) {
      episode_origin = state.robots[0];
      crossed_transit = false;
    }
    if (state.kappa[0] != KAPPA_FREE &&
        ops[0].kind == Op::MOVE &&
        !ins.can_store_shelf(ops[0].to))
      crossed_transit = true;
    if (ops[0].kind == Op::DROP) {
      ASSERT_TRUE(ins.can_store_shelf(state.robots[0]));
      if (crossed_transit)
        EXPECT_NE(state.robots[0], episode_origin)
            << "a storage transfer returned to the slot it just vacated";
      episode_origin = -1;
      crossed_transit = false;
    }
    const auto successor = apply_ops(ins, state, ops);
    ASSERT_TRUE(successor.has_value());
    state = *successor;
  }
  EXPECT_TRUE(is_dd_goal(ins, state));
  EXPECT_TRUE(std::binary_search(
      state.anon_occ.begin(), state.anon_occ.end(), ins.grid.idx(0, 4)));
}
