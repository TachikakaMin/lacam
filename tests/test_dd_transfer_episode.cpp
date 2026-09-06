#include "../lacam/src/carrier_guidance.hpp"

#include <dd_planner.hpp>

#include <optional>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance line_storage_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  ins.shelf_storage[ins.grid.idx(0, 0)] = 1;
  ins.shelf_storage[ins.grid.idx(0, 3)] = 1;
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.target_goal_sets = {{ins.grid.idx(0, 3)}};
  ins.finalize();
  return ins;
}

Custody active_custody(const DDInstance& ins, int carrier,
                       const PhysConfig& anchor)
{
  const ShelfTask task{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
      {RootDemand{0, ins.grid.idx(0, 3)}},
      7,
      StorageTransfer{
          ins.grid.idx(0, 3),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
           ins.grid.idx(0, 2), ins.grid.idx(0, 3)}}};
  Custody custody = carrier_detail::make_custody(task, -1);
  custody.transfer_index = 1;
  custody.from = ins.grid.idx(0, 1);
  custody.to = ins.grid.idx(0, 2);
  custody.task_id =
      TaskId{custody.shelf, custody.from, custody.to};
  custody.preferred_leg = custody.task_id;
  custody.transfer_id.carrier = carrier;
  custody.transfer_id.anchor = phys_config_hash(anchor);
  return custody;
}

}  // namespace

TEST(dd_transfer_episode,
     zero_budget_refresh_and_wait_preserve_episode_identity)
{
  const auto ins = line_storage_instance();
  auto state = initial_phys_config(ins);
  auto next = apply_ops(ins, state, {Op::make_lift()});
  ASSERT_TRUE(next.has_value());
  state = *next;
  next = apply_ops(
      ins, state, {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(next.has_value());
  state = *next;

  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_guidance.custody_by_robot = {
      active_custody(ins, 0, state)};
  ASSERT_TRUE(carrier_detail::custody_physically_valid(
      ins, state, 0, *previous_guidance.custody_by_robot[0]));
  const TransferId expected_id =
      previous_guidance.custody_by_robot[0]->transfer_id;
  const int expected_endpoint =
      previous_guidance.custody_by_robot[0]->original_endpoint;
  const std::vector<Op> wait = {Op::make_wait()};
  const auto waited = apply_ops(ins, state, wait);
  ASSERT_TRUE(waited.has_value());
  const ShelfTaskGraph empty_graph;

  const auto blocked = carrier_detail::recover_task_br_custody(
      ins, *waited, empty_graph, &state, &previous_guidance, &wait,
      /*route_budget=*/0, /*force_route_refresh=*/true);

  ASSERT_TRUE(blocked.custody_by_robot[0].has_value());
  const auto& no_hint = *blocked.custody_by_robot[0];
  EXPECT_EQ(no_hint.transfer_id, expected_id);
  EXPECT_EQ(no_hint.original_endpoint, expected_endpoint);
  EXPECT_EQ(no_hint.route_status, RouteStatus::BUDGET_EXHAUSTED);
  EXPECT_FALSE(no_hint.preferred_leg.has_value());
  EXPECT_TRUE(carrier_detail::custody_physically_valid(
      ins, *waited, 0, no_hint));
  EXPECT_TRUE(carrier_detail::episode_active(no_hint));
  EXPECT_FALSE(carrier_detail::route_hint_usable(ins, *waited, 0, no_hint));

  CarrierGuidance blocked_guidance;
  blocked_guidance.upper_epoch =
      std::make_shared<UpperEpochGuidance>();
  blocked_guidance.custody_by_robot = blocked.custody_by_robot;
  const auto recovered = carrier_detail::recover_task_br_custody(
      ins, *waited, empty_graph, &*waited, &blocked_guidance, &wait,
      /*route_budget=*/32, /*force_route_refresh=*/true);

  ASSERT_TRUE(recovered.custody_by_robot[0].has_value());
  const auto& resumed = *recovered.custody_by_robot[0];
  EXPECT_EQ(resumed.transfer_id, expected_id);
  EXPECT_EQ(resumed.original_endpoint, expected_endpoint);
  EXPECT_EQ(resumed.route_status, RouteStatus::OK);
  ASSERT_TRUE(resumed.preferred_leg.has_value());
  EXPECT_EQ(resumed.preferred_leg->from, ins.grid.idx(0, 1));
  EXPECT_EQ(resumed.preferred_leg->to, ins.grid.idx(0, 2));
}

TEST(dd_transfer_episode,
     forced_deviation_reroutes_to_original_endpoint_without_new_episode)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", ".....", "....."});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  ins.shelf_storage[ins.grid.idx(0, 0)] = 1;
  ins.shelf_storage[ins.grid.idx(0, 4)] = 1;
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.target_goal_sets = {{ins.grid.idx(0, 4)}};
  ins.finalize();

  auto state = initial_phys_config(ins);
  auto next = apply_ops(ins, state, {Op::make_lift()});
  ASSERT_TRUE(next.has_value());
  state = *next;
  next = apply_ops(
      ins, state, {Op::make_move(ins.grid.idx(0, 1))});
  ASSERT_TRUE(next.has_value());
  state = *next;

  ShelfTask task{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
      {RootDemand{0, ins.grid.idx(0, 4)}},
      4,
      StorageTransfer{
          ins.grid.idx(0, 4),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
           ins.grid.idx(0, 2), ins.grid.idx(0, 3),
           ins.grid.idx(0, 4)}}};
  Custody custody = carrier_detail::make_custody(task, -1);
  custody.transfer_index = 1;
  custody.from = ins.grid.idx(0, 1);
  custody.to = ins.grid.idx(0, 2);
  custody.task_id =
      TaskId{custody.shelf, custody.from, custody.to};
  custody.preferred_leg = custody.task_id;
  custody.transfer_id.carrier = 0;
  custody.transfer_id.anchor = phys_config_hash(state);
  const TransferId expected_id = custody.transfer_id;

  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_guidance.custody_by_robot = {custody};
  ASSERT_TRUE(carrier_detail::custody_physically_valid(
      ins, state, 0, *previous_guidance.custody_by_robot[0]));
  const std::vector<Op> deviation = {
      Op::make_move(ins.grid.idx(1, 1))};
  const auto deviated = apply_ops(ins, state, deviation);
  ASSERT_TRUE(deviated.has_value());
  const ShelfTaskGraph empty_graph;

  const auto recovered = carrier_detail::recover_task_br_custody(
      ins, *deviated, empty_graph, &state, &previous_guidance,
      &deviation);

  ASSERT_TRUE(recovered.custody_by_robot[0].has_value());
  const auto& rerouted = *recovered.custody_by_robot[0];
  EXPECT_EQ(rerouted.transfer_id, expected_id);
  EXPECT_EQ(rerouted.original_endpoint, ins.grid.idx(0, 4));
  EXPECT_EQ(rerouted.transfer.endpoint, ins.grid.idx(0, 4));
  EXPECT_EQ(rerouted.rebind_reason, RebindReason::FORCED_DEVIATION);
  EXPECT_EQ(rerouted.route_status, RouteStatus::OK);
  ASSERT_TRUE(rerouted.preferred_leg.has_value());
  EXPECT_EQ(rerouted.preferred_leg->from, ins.grid.idx(1, 1));
  EXPECT_NE(rerouted.preferred_leg->to, ins.grid.idx(0, 1));
}

TEST(dd_transfer_episode,
     same_upper_epoch_reconciles_grounded_and_carried_execution_views)
{
  const auto ins = line_storage_instance();
  const auto grounded = initial_phys_config(ins);
  const ShelfTask task{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
      {RootDemand{0, ins.grid.idx(0, 3)}},
      7,
      StorageTransfer{
          ins.grid.idx(0, 3),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
           ins.grid.idx(0, 2), ins.grid.idx(0, 3)}}};
  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->upper_signature =
      carrier_detail::make_upper_signature(grounded);
  epoch->task_graph.tasks = {task};
  epoch->task_graph.predecessors = {{}};
  epoch->task_graph.successors = {{}};

  const auto grounded_guidance =
      carrier_detail::build_task_br_guidance_from_upper_epoch(
          ins, grounded, epoch);
  ASSERT_EQ(grounded_guidance.execution_view.tasks.size(), 1u);
  EXPECT_EQ(
      grounded_guidance.execution_view.tasks[0].state,
      ExecutionTaskState::PENDING);
  ASSERT_TRUE(grounded_guidance.rho_task_id[0].has_value());

  CarrierGuidance previous_guidance = grounded_guidance;
  const std::vector<Op> lift = {Op::make_lift()};
  const auto carried = apply_ops(ins, grounded, lift);
  ASSERT_TRUE(carried.has_value());
  const auto carried_guidance =
      carrier_detail::build_task_br_guidance_from_upper_epoch(
          ins, *carried, epoch, &grounded, &previous_guidance, &lift);

  EXPECT_EQ(
      grounded_guidance.upper_epoch.get(),
      carried_guidance.upper_epoch.get());
  ASSERT_EQ(carried_guidance.execution_view.tasks.size(), 1u);
  const auto& active = carried_guidance.execution_view.tasks[0];
  EXPECT_EQ(active.state, ExecutionTaskState::ACTIVE);
  EXPECT_EQ(active.carrier, 0);
  ASSERT_TRUE(active.transfer_id.has_value());
  ASSERT_TRUE(carried_guidance.custody_by_robot[0].has_value());
  EXPECT_EQ(
      *active.transfer_id,
      carried_guidance.custody_by_robot[0]->transfer_id);
  EXPECT_FALSE(carried_guidance.rho_task_id[0].has_value());
  ASSERT_EQ(carried_guidance.timed_transport.by_robot.size(), 1u);
  ASSERT_TRUE(
      carried_guidance.timed_transport.by_robot[0].has_value());
  EXPECT_EQ(
      carried_guidance.timed_transport.by_robot[0]->endpoint,
      ins.grid.idx(0, 3));
}

TEST(dd_transfer_episode,
     rho_reports_the_committed_endpoint_when_leg_ids_are_identical)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 4)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.finalize();
  const auto physical = initial_phys_config(ins);
  const TaskId shared_leg{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 1)},
      ins.grid.idx(0, 1), ins.grid.idx(0, 2)};
  ShelfTaskGraph graph;
  graph.tasks = {
      ShelfTask{
          shared_leg, {RootDemand{0, ins.grid.idx(0, 3)}}, 1,
          StorageTransfer{
              ins.grid.idx(0, 3),
              {ins.grid.idx(0, 1), ins.grid.idx(0, 2),
               ins.grid.idx(0, 3)}}},
      ShelfTask{
          shared_leg, {RootDemand{1, ins.grid.idx(0, 4)}}, 9,
          StorageTransfer{
              ins.grid.idx(0, 4),
              {ins.grid.idx(0, 1), ins.grid.idx(0, 2),
               ins.grid.idx(0, 3), ins.grid.idx(0, 4)}}},
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};

  const auto match = carrier_detail::match_ready_tasks(
      ins, physical, graph, {0, 1}, nullptr);

  ASSERT_TRUE(match.rho_task_id[0].has_value());
  EXPECT_EQ(*match.rho_task_id[0], shared_leg);
  ASSERT_TRUE(match.rho_transfer_key[0].has_value());
  EXPECT_EQ(
      match.rho_transfer_key[0]->endpoint,
      ins.grid.idx(0, 4));
  EXPECT_EQ(match.rho_ready_index[0], 1);
  EXPECT_FALSE(match.rho_task_id[1].has_value());
  EXPECT_FALSE(match.rho_transfer_key[1].has_value());
}
