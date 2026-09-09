#include "../lacam/src/carrier_guidance.hpp"

#include <dd_planner.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace {

Custody carried_task_custody(const DDInstance& ins, int target,
                             int robot, int source, int first_step,
                             int endpoint,
                             const std::vector<int>& route,
                             const PhysConfig& physical)
{
  const ShelfTask task{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, target},
          source, first_step},
      {RootDemand{target, endpoint}},
      10 - target,
      StorageTransfer{endpoint, route}};
  Custody custody = carrier_detail::make_custody(task, -1);
  carrier_detail::ensure_transfer_identity(
      custody, robot, phys_config_hash(physical));
  return custody;
}

}  // namespace

TEST(dd_timed_transport,
     shared_transit_first_step_is_serialized_without_deleting_a_job)
{
  DDInstance ins;
  ins.grid = DDGrid({"@@.@@", "@...@", "@@.@@"});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (const int cell : {
           ins.grid.idx(0, 2), ins.grid.idx(1, 1),
           ins.grid.idx(1, 3), ins.grid.idx(2, 2)})
    ins.shelf_storage[cell] = 1;
  ins.robots = {ins.grid.idx(1, 1), ins.grid.idx(1, 3)};
  ins.shelves = ins.robots;
  ins.target_starts = ins.robots;
  ins.target_goals = {
      ins.grid.idx(0, 2), ins.grid.idx(2, 2)};
  ins.target_goal_sets = {
      {ins.grid.idx(0, 2)}, {ins.grid.idx(2, 2)}};
  ins.finalize();

  PhysConfig physical = initial_phys_config(ins);
  physical.kappa = {0, 1};
  const int hub = ins.grid.idx(1, 2);
  const std::vector<std::optional<Custody>> custodies = {
      carried_task_custody(
          ins, 0, 0, ins.grid.idx(1, 1), hub,
          ins.grid.idx(0, 2),
          {ins.grid.idx(1, 1), hub, ins.grid.idx(0, 2)},
          physical),
      carried_task_custody(
          ins, 1, 1, ins.grid.idx(1, 3), hub,
          ins.grid.idx(2, 2),
          {ins.grid.idx(1, 3), hub, ins.grid.idx(2, 2)},
          physical),
  };

  const auto guidance =
      carrier_detail::build_bounded_joint_transport_guidance(
          ins, physical, custodies,
          /*horizon=*/4, /*expansions_per_job=*/64,
          /*frame_budget=*/4);

  ASSERT_EQ(guidance.by_robot.size(), 2u);
  ASSERT_TRUE(guidance.by_robot[0].has_value());
  ASSERT_TRUE(guidance.by_robot[1].has_value());
  EXPECT_EQ(guidance.by_robot[0]->status, RouteStatus::OK);
  EXPECT_EQ(guidance.by_robot[1]->status, RouteStatus::OK);
  const auto& a = guidance.by_robot[0]->cells;
  const auto& b = guidance.by_robot[1]->cells;
  ASSERT_GE(a.size(), 3u);
  ASSERT_GE(b.size(), 3u);
  EXPECT_NE(a[1] == physical.robots[0],
            b[1] == physical.robots[1]);
  for (size_t tick = 1; tick < std::min(a.size(), b.size()); ++tick) {
    EXPECT_NE(a[tick], b[tick]);
    EXPECT_FALSE(
        a[tick - 1] == b[tick] &&
        b[tick - 1] == a[tick]);
  }
  int selected_expansions = 0;
  for (const auto& hint : guidance.by_robot)
    if (hint.has_value())
      selected_expansions += hint->expansions;
  EXPECT_GT(guidance.frames_evaluated, 1);
  EXPECT_GT(guidance.expansions, selected_expansions)
      << "diagnostics must include every candidate frame, not only the "
         "selected frame";
}

TEST(dd_timed_transport,
     horizon_shortfall_returns_a_moving_prefix_toward_same_endpoint)
{
  DDInstance ins;
  ins.grid = DDGrid({"........"});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  ins.shelf_storage[ins.grid.idx(0, 0)] = 1;
  ins.shelf_storage[ins.grid.idx(0, 7)] = 1;
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 7)};
  ins.target_goal_sets = {{ins.grid.idx(0, 7)}};
  ins.finalize();

  PhysConfig physical = initial_phys_config(ins);
  physical.kappa = {0};
  const std::vector<std::optional<Custody>> custodies = {
      carried_task_custody(
          ins, 0, 0, ins.grid.idx(0, 0), ins.grid.idx(0, 1),
          ins.grid.idx(0, 7),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
           ins.grid.idx(0, 2), ins.grid.idx(0, 3),
           ins.grid.idx(0, 4), ins.grid.idx(0, 5),
           ins.grid.idx(0, 6), ins.grid.idx(0, 7)},
          physical),
  };

  const auto guidance =
      carrier_detail::build_bounded_joint_transport_guidance(
          ins, physical, custodies,
          /*horizon=*/3, /*expansions_per_job=*/64,
          /*frame_budget=*/2);

  ASSERT_EQ(guidance.by_robot.size(), 1u);
  ASSERT_TRUE(guidance.by_robot[0].has_value());
  const auto& hint = *guidance.by_robot[0];
  EXPECT_EQ(hint.status, RouteStatus::PREFIX);
  ASSERT_EQ(hint.cells.size(), 4u);
  EXPECT_EQ(hint.cells.front(), physical.robots[0]);
  EXPECT_NE(hint.cells[1], physical.robots[0]);
  EXPECT_NE(hint.cells.back(), ins.grid.idx(0, 7));
  EXPECT_EQ(hint.endpoint, ins.grid.idx(0, 7));
}

TEST(dd_timed_transport,
     opposing_corridor_jobs_use_the_passing_bay_without_edge_swaps)
{
  DDInstance ins;
  ins.grid = DDGrid({".......", "@@...@@"});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  ins.shelf_storage[ins.grid.idx(0, 0)] = 1;
  ins.shelf_storage[ins.grid.idx(0, 6)] = 1;
  ins.robots = {ins.grid.idx(0, 0), ins.grid.idx(0, 6)};
  ins.shelves = ins.robots;
  ins.target_starts = ins.robots;
  ins.target_goals = {
      ins.grid.idx(0, 6), ins.grid.idx(0, 0)};
  ins.target_goal_sets = {
      {ins.grid.idx(0, 6)}, {ins.grid.idx(0, 0)}};
  ins.finalize();

  PhysConfig physical = initial_phys_config(ins);
  physical.kappa = {0, 1};
  const std::vector<int> eastbound = {
      ins.grid.idx(0, 0), ins.grid.idx(0, 1),
      ins.grid.idx(0, 2), ins.grid.idx(0, 3),
      ins.grid.idx(0, 4), ins.grid.idx(0, 5),
      ins.grid.idx(0, 6)};
  auto westbound = eastbound;
  std::reverse(westbound.begin(), westbound.end());
  const std::vector<std::optional<Custody>> custodies = {
      carried_task_custody(
          ins, 0, 0, ins.grid.idx(0, 0), ins.grid.idx(0, 1),
          ins.grid.idx(0, 6), eastbound, physical),
      carried_task_custody(
          ins, 1, 1, ins.grid.idx(0, 6), ins.grid.idx(0, 5),
          ins.grid.idx(0, 0), westbound, physical),
  };

  const auto guidance =
      carrier_detail::build_bounded_joint_transport_guidance(
          ins, physical, custodies,
          /*horizon=*/12, /*expansions_per_job=*/512,
          /*frame_budget=*/4);

  ASSERT_TRUE(guidance.by_robot[0].has_value());
  ASSERT_TRUE(guidance.by_robot[1].has_value());
  ASSERT_EQ(guidance.by_robot[0]->status, RouteStatus::OK);
  ASSERT_EQ(guidance.by_robot[1]->status, RouteStatus::OK);
  const auto& a = guidance.by_robot[0]->cells;
  const auto& b = guidance.by_robot[1]->cells;
  bool used_passing_bay = false;
  for (size_t tick = 1; tick < std::min(a.size(), b.size()); ++tick) {
    EXPECT_NE(a[tick], b[tick]);
    EXPECT_FALSE(
        a[tick - 1] == b[tick] &&
        b[tick - 1] == a[tick]);
    used_passing_bay |=
        ins.grid.row(a[tick]) == 1 ||
        ins.grid.row(b[tick]) == 1;
  }
  EXPECT_TRUE(used_passing_bay);
}

TEST(dd_timed_transport,
     loaded_carrier_uses_empty_storage_pocket_instead_of_backing_down_corridor)
{
  DDInstance ins;
  ins.grid = DDGrid({"##.##", "....."});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  const int west = ins.grid.idx(1, 0);
  const int east = ins.grid.idx(1, 4);
  const int pocket = ins.grid.idx(0, 2);
  ins.shelf_storage[west] = 1;
  ins.shelf_storage[east] = 1;
  ins.shelf_storage[pocket] = 1;
  ins.robots = {ins.grid.idx(1, 1), ins.grid.idx(1, 2)};
  ins.shelves = {west, east};
  ins.target_starts = {west, east};
  ins.target_goals = {east, west};
  ins.target_goal_sets = {{east}, {west}};
  ins.finalize();

  PhysConfig physical = initial_phys_config(ins);
  physical.kappa = {0, 1};
  physical.target_pos = physical.robots;
  CarrierGuidance guidance;
  guidance.custody_by_robot = {
      carried_task_custody(
          ins, 0, 0, ins.grid.idx(1, 1),
          ins.grid.idx(1, 2), east,
          {ins.grid.idx(1, 1), ins.grid.idx(1, 2),
           ins.grid.idx(1, 3), east},
          physical),
      carried_task_custody(
          ins, 1, 1, ins.grid.idx(1, 2),
          ins.grid.idx(1, 1), west,
          {ins.grid.idx(1, 2), ins.grid.idx(1, 1), west},
          physical),
  };
  guidance.timed_transport.by_robot.resize(2);
  guidance.timed_transport.by_robot[0] = TimedRouteHint{
      RouteStatus::OK, east,
      {ins.grid.idx(1, 1), ins.grid.idx(1, 2),
       ins.grid.idx(1, 3), east},
      3, 0};
  guidance.timed_transport.by_robot[1] = TimedRouteHint{
      RouteStatus::OK, west,
      {ins.grid.idx(1, 2), ins.grid.idx(1, 3),
       ins.grid.idx(1, 4)},
      4, 0};

  for (const int seed : {0, 1, 2}) {
    const TAPFInstance view(ins);
    std::mt19937 mt(seed);
    TAPFStats stats;
    TAPFPlanner planner(
        &view, nullptr, &mt, 0, 0, 0.001f, true, &stats);
    Config config;
    for (const int cell : physical.robots)
      config.push_back(view.G.U[cell]);
    ShelfState shelf;
    shelf.target_pos = physical.target_pos;
    shelf.anon_occ = physical.anon_occ;
    shelf.kappa = physical.kappa;
    auto node = std::make_unique<TAPFNode>(
        config, shelf, planner.D, &view,
        std::vector<int>(view.N, -1),
        TAPFAssignmentState(), nullptr);
    node->guide =
        std::make_unique<CarrierGuidance>(guidance);
    node->order = {1, 0};
    node->constraint_order = node->order;
    planner.invalidate_carrier_scratch();
    TAPFConstraint root;
    ASSERT_TRUE(planner.get_new_config(node.get(), &root));
    ASSERT_TRUE(planner.apply_carrier_effects(node.get()));

    ASSERT_EQ(planner.ops_scratch.size(), 2u);
    EXPECT_EQ(planner.ops_scratch[0].kind, Op::MOVE);
    EXPECT_EQ(
        planner.ops_scratch[0].to, ins.grid.idx(1, 2));
    EXPECT_EQ(planner.ops_scratch[1].kind, Op::MOVE);
    EXPECT_EQ(planner.ops_scratch[1].to, pocket)
        << "an empty storage slot one cell off the aisle is a legal "
           "temporary passing pocket; the yielding loaded carrier should "
           "not be driven backward along the aisle (seed "
        << seed << ")";
  }
}

TEST(dd_timed_transport,
     paused_residual_is_part_of_the_all_target_frame_score)
{
  const auto sparse =
      carrier_detail::make_joint_transport_frame_score(
          /*planned_completion=*/{12},
          /*residual_completion=*/{50},
          /*predicted_work=*/5,
          /*stable_order=*/{0});
  const auto complete =
      carrier_detail::make_joint_transport_frame_score(
          /*planned_completion=*/{30},
          /*residual_completion=*/{},
          /*predicted_work=*/10,
          /*stable_order=*/{1});

  EXPECT_EQ(sparse.predicted_all_targets_ticks, 50);
  EXPECT_TRUE(
      carrier_detail::better_joint_transport_frame(
          complete, sparse))
      << "a frame cannot win by omitting paused work from its makespan";
}

TEST(dd_timed_transport,
     grounded_assignment_joins_the_pre_lift_reservation_frame)
{
  DDInstance ins;
  ins.grid = DDGrid({"@@.@@", "@...@", "@@.@@"});
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (const int cell : {
           ins.grid.idx(0, 2), ins.grid.idx(1, 1),
           ins.grid.idx(1, 3), ins.grid.idx(2, 2)})
    ins.shelf_storage[cell] = 1;
  ins.robots = {ins.grid.idx(1, 1), ins.grid.idx(1, 3)};
  ins.shelves = ins.robots;
  ins.target_starts = ins.robots;
  ins.target_goals = {
      ins.grid.idx(0, 2), ins.grid.idx(2, 2)};
  ins.target_goal_sets = {
      {ins.grid.idx(0, 2)}, {ins.grid.idx(2, 2)}};
  ins.finalize();

  PhysConfig physical = initial_phys_config(ins);
  physical.kappa = {0, KAPPA_FREE};
  const int hub = ins.grid.idx(1, 2);
  std::vector<std::optional<Custody>> custodies(2);
  custodies[0] = carried_task_custody(
      ins, 0, 0, ins.grid.idx(1, 1), hub,
      ins.grid.idx(0, 2),
      {ins.grid.idx(1, 1), hub, ins.grid.idx(0, 2)},
      physical);

  ShelfTaskGraph graph;
  graph.tasks = {
      ShelfTask{
          TaskId{
              ShelfSelector{ShelfSelector::Kind::TARGET, 1},
              ins.grid.idx(1, 3), hub},
          {RootDemand{1, ins.grid.idx(2, 2)}},
          9,
          StorageTransfer{
              ins.grid.idx(2, 2),
              {ins.grid.idx(1, 3), hub,
               ins.grid.idx(2, 2)}}},
  };
  graph.predecessors = {{}};
  graph.successors = {{}};
  ExecutionView view;
  view.tasks = {
      ExecutionTaskView{ExecutionTaskState::PENDING, -1, std::nullopt},
  };
  const std::vector<int> rho_ready_index = {-1, 0};
  const std::vector<DispatchMode> rho_mode = {
      DispatchMode::NONE, DispatchMode::EXECUTE};
  const std::vector<int> tau = {
      ins.grid.idx(0, 2), ins.grid.idx(2, 2)};
  const carrier_detail::JointTransportContext context{
      &graph, &view, &rho_ready_index, &rho_mode, &tau};

  const auto guidance =
      carrier_detail::build_bounded_joint_transport_guidance(
          ins, physical, custodies,
          /*horizon=*/4, /*expansions_per_job=*/64,
          /*frame_budget=*/4, &context);

  ASSERT_TRUE(guidance.by_robot[0].has_value());
  ASSERT_TRUE(guidance.by_robot[1].has_value());
  const auto& active = guidance.by_robot[0]->cells;
  const auto& assigned = guidance.by_robot[1]->cells;
  ASSERT_EQ(active.size(), assigned.size());
  EXPECT_EQ(assigned[0], physical.robots[1]);
  EXPECT_EQ(assigned[1], physical.robots[1])
      << "the grounded shelf remains at its source through approach/Lift";
  for (size_t tick = 1; tick < active.size(); ++tick) {
    EXPECT_NE(active[tick], assigned[tick]);
    EXPECT_FALSE(
        active[tick - 1] == assigned[tick] &&
        assigned[tick - 1] == active[tick]);
  }
}
