// PROTECTED storage-transfer claim and temporal-order regressions.
// Written before implementation on 2026-09-04 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"
#include <dd_planner.hpp>

#include <algorithm>
#include <optional>
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

int find_task(const ShelfTaskGraph& graph, const ShelfSelector& shelf)
{
  for (size_t index = 0; index < graph.tasks.size(); ++index)
    if (graph.tasks[index].id.shelf == shelf) return (int)index;
  return -1;
}

}  // namespace

TEST(dd_storage_transfer_claims,
     same_first_leg_with_different_endpoint_does_not_rebind_custody)
{
  const auto ins = make_instance(
      {".....", ".....", "....."},
      {"....S", "S....", "....S"},
      {{1, 0}}, {{1, 0}},
      {{{1, 0}, {{0, 4}}}});
  auto previous = initial_phys_config(ins);
  auto next = apply_ops(ins, previous, {Op::make_lift()});
  ASSERT_TRUE(next.has_value());
  previous = *next;
  next = apply_ops(
      ins, previous, {Op::make_move(ins.grid.idx(1, 1))});
  ASSERT_TRUE(next.has_value());
  previous = *next;

  Custody old;
  old.shelf =
      ShelfSelector{ShelfSelector::Kind::TARGET, 0};
  old.from = ins.grid.idx(1, 1);
  old.to = ins.grid.idx(1, 2);
  old.task_id = TaskId{old.shelf, old.from, old.to};
  old.roots = {RootDemand{0, ins.grid.idx(0, 4)}};
  old.priority = 7;
  old.transfer = StorageTransfer{
      ins.grid.idx(0, 4),
      {ins.grid.idx(1, 0), ins.grid.idx(1, 1),
       ins.grid.idx(1, 2), ins.grid.idx(1, 3),
       ins.grid.idx(0, 3), ins.grid.idx(0, 4)}};
  old.transfer_index = 1;

  const ShelfTask conflicting_task{
      TaskId{
          old.shelf, ins.grid.idx(1, 2), ins.grid.idx(1, 3)},
      {RootDemand{0, ins.grid.idx(2, 4)}},
      99,
      StorageTransfer{
          ins.grid.idx(2, 4),
          {ins.grid.idx(1, 2), ins.grid.idx(1, 3),
           ins.grid.idx(2, 3), ins.grid.idx(2, 4)}}};
  ShelfTaskGraph current_graph;
  current_graph.tasks = {conflicting_task};
  current_graph.predecessors = {{}};
  current_graph.successors = {{}};

  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_guidance.custody_by_robot = {old};
  const std::vector<Op> move = {
      Op::make_move(ins.grid.idx(1, 2))};
  const auto physical = apply_ops(ins, previous, move);
  ASSERT_TRUE(physical.has_value());

  const auto recovered = carrier_detail::recover_task_br_custody(
      ins, *physical, current_graph, &previous, &previous_guidance,
      &move);
  ASSERT_TRUE(recovered.custody_by_robot[0].has_value());
  const auto& custody = *recovered.custody_by_robot[0];
  EXPECT_EQ(custody.transfer.endpoint, ins.grid.idx(0, 4));
  EXPECT_EQ(custody.transfer, old.transfer);
  EXPECT_EQ(custody.transfer_index, 2u);
  EXPECT_FALSE(custody.current_task_index.has_value());
  ASSERT_EQ(custody.roots.size(), 1u);
  EXPECT_EQ(custody.roots[0].goal, ins.grid.idx(0, 4));
  EXPECT_EQ(custody.priority, 7);
}

TEST(dd_storage_transfer_claims,
     anonymous_selector_reanchors_to_each_verified_route_leg)
{
  const auto ins = make_instance(
      {"...."}, {"S..S"}, {{0, 0}}, {{0, 0}}, {});
  auto previous = initial_phys_config(ins);
  const auto lifted =
      apply_ops(ins, previous, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  previous = *lifted;

  Custody old;
  old.shelf = ShelfSelector{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
      ins.grid.idx(0, 0)};
  old.from = ins.grid.idx(0, 0);
  old.to = ins.grid.idx(0, 1);
  old.task_id = TaskId{old.shelf, old.from, old.to};
  old.transfer = StorageTransfer{
      ins.grid.idx(0, 3),
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2), ins.grid.idx(0, 3)}};

  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_guidance.custody_by_robot = {old};
  const std::vector<Op> move = {
      Op::make_move(ins.grid.idx(0, 1))};
  const auto physical = apply_ops(ins, previous, move);
  ASSERT_TRUE(physical.has_value());
  const ShelfTaskGraph empty_graph;

  const auto recovered = carrier_detail::recover_task_br_custody(
      ins, *physical, empty_graph, &previous, &previous_guidance,
      &move);
  ASSERT_TRUE(recovered.custody_by_robot[0].has_value());
  const auto& custody = *recovered.custody_by_robot[0];
  EXPECT_EQ(custody.shelf.kind,
            ShelfSelector::Kind::ANON_AT_EPOCH_CELL);
  EXPECT_EQ(custody.shelf.value, ins.grid.idx(0, 1));
  EXPECT_EQ(custody.task_id.shelf, custody.shelf);
  EXPECT_EQ(custody.task_id.from, ins.grid.idx(0, 1));
  EXPECT_EQ(custody.task_id.to, ins.grid.idx(0, 2));
}

TEST(dd_storage_transfer_claims,
     recovery_jointly_assigns_distinct_reachable_endpoints)
{
  const auto ins = make_instance(
      {".....", ".....", "....."},
      {"S...S", ".....", "....."},
      {{0, 0}, {0, 4}}, {{0, 0}, {0, 4}},
      {
          {{0, 0}, {{0, 0}}},
          {{0, 4}, {{0, 4}}},
      });
  PhysConfig previous;
  previous.robots = {
      ins.grid.idx(2, 1), ins.grid.idx(2, 2)};
  previous.target_pos = previous.robots;
  previous.kappa = {0, 1};
  const std::vector<Op> waits = {
      Op::make_wait(), Op::make_wait()};
  const auto physical = apply_ops(ins, previous, waits);
  ASSERT_TRUE(physical.has_value());
  const ShelfTaskGraph empty_graph;

  const auto recovered = carrier_detail::recover_task_br_custody(
      ins, *physical, empty_graph, &previous, nullptr, &waits);
  ASSERT_TRUE(recovered.custody_by_robot[0].has_value());
  ASSERT_TRUE(recovered.custody_by_robot[1].has_value());
  const std::set<int> endpoints = {
      recovered.custody_by_robot[0]->transfer.endpoint,
      recovered.custody_by_robot[1]->transfer.endpoint};
  EXPECT_EQ(
      endpoints,
      (std::set<int>{
          ins.grid.idx(0, 0), ins.grid.idx(0, 4)}));
}

TEST(dd_storage_transfer_claims,
     active_endpoint_claim_filters_ready_outside_cached_epoch)
{
  const auto ins = make_instance(
      {"......."}, {"S..S..S"}, {{0, 0}, {0, 1}},
      {{0, 0}, {0, 6}},
      {
          {{0, 0}, {{0, 3}, {0, 0}}},
          {{0, 6}, {{0, 3}, {0, 6}}},
      });
  PhysConfig physical;
  physical.robots = {ins.grid.idx(0, 1), ins.grid.idx(0, 0)};
  physical.target_pos = {
      ins.grid.idx(0, 1), ins.grid.idx(0, 6)};
  physical.kappa = {0, KAPPA_FREE};

  Custody active;
  active.shelf =
      ShelfSelector{ShelfSelector::Kind::TARGET, 0};
  active.from = ins.grid.idx(0, 1);
  active.to = ins.grid.idx(0, 2);
  active.task_id =
      TaskId{active.shelf, active.from, active.to};
  active.roots = {RootDemand{0, ins.grid.idx(0, 3)}};
  active.priority = 10;
  active.transfer = StorageTransfer{
      ins.grid.idx(0, 3),
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
       ins.grid.idx(0, 2), ins.grid.idx(0, 3)}};
  active.transfer_index = 1;

  const ShelfTask waiting{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 1},
          ins.grid.idx(0, 6), ins.grid.idx(0, 5)},
      {RootDemand{1, ins.grid.idx(0, 3)}},
      5,
      StorageTransfer{
          ins.grid.idx(0, 3),
          {ins.grid.idx(0, 6), ins.grid.idx(0, 5),
           ins.grid.idx(0, 4), ins.grid.idx(0, 3)}}};
  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->upper_signature =
      carrier_detail::make_upper_signature(physical);
  epoch->task_graph.tasks = {waiting};
  epoch->task_graph.predecessors = {{}};
  epoch->task_graph.successors = {{}};

  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = epoch;
  previous_guidance.custody_by_robot = {
      active, std::nullopt};
  const std::vector<Op> waits = {
      Op::make_wait(), Op::make_wait()};
  const auto with_claim =
      carrier_detail::build_task_br_guidance_from_upper_epoch(
          ins, physical, epoch, &physical, &previous_guidance,
          &waits);
  const auto without_claim =
      carrier_detail::build_task_br_guidance_from_upper_epoch(
          ins, physical, epoch);

  EXPECT_EQ(with_claim.upper_epoch.get(), epoch.get());
  EXPECT_EQ(without_claim.upper_epoch.get(), epoch.get());
  ASSERT_EQ(epoch->task_graph.tasks.size(), 1u);
  EXPECT_TRUE(with_claim.ready_tasks.empty());
  EXPECT_EQ(without_claim.ready_tasks, std::vector<int>({0}));
}

TEST(dd_storage_transfer_claims,
     dependency_chain_may_reuse_a_transit_hub_in_sequence)
{
  const auto ins = make_instance(
      {"@@@@@", "@@@@@", ".....", "@@.@@", "@@.@@"},
      {".....", ".....", "S...S", ".....", "..S.."},
      {{2, 4}}, {{2, 0}, {2, 4}},
      {{{2, 0}, {{2, 4}}}});
  const auto graph = dd_compile_single_root_graph_probe(
      ins, initial_phys_config(ins), 0, ins.grid.idx(2, 4));
  const ShelfSelector target{
      ShelfSelector::Kind::TARGET, 0};
  const ShelfSelector blocker{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
      ins.grid.idx(2, 4)};
  const int target_task = find_task(graph, target);
  const int blocker_task = find_task(graph, blocker);
  ASSERT_GE(target_task, 0);
  ASSERT_GE(blocker_task, 0);

  EXPECT_EQ(
      graph.tasks[target_task].transfer.endpoint,
      ins.grid.idx(2, 4));
  EXPECT_EQ(
      graph.tasks[blocker_task].transfer.endpoint,
      ins.grid.idx(4, 2));
  ASSERT_EQ(graph.predecessors[target_task].size(), 1u);
  EXPECT_EQ(graph.predecessors[target_task][0], blocker_task);
}
