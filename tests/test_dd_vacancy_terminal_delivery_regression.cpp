// PROTECTED terminal-delivery regressions.
// Written after observing b11 reach one cell from its fixed goal, yield to
// an ordinary free robot, and then take a 126-tick detour in the 2026-09-07
// factorial report case.  Intentionally observed RED before the fix.
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

bool upper_cell_free_for_carried_target(
    const PhysConfig& physical, int target, int cell)
{
  for (size_t other = 0; other < physical.target_pos.size(); ++other)
    if (static_cast<int>(other) != target &&
        physical.target_pos[other] == cell)
      return false;
  return !std::binary_search(
      physical.anon_occ.begin(), physical.anon_occ.end(), cell);
}

}  // namespace

TEST(dd_vacancy_terminal_delivery_regression,
     adjacent_target_delivery_precedes_higher_priority_free_dispatch)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {
      ins.grid.idx(1, 0),
      ins.grid.idx(0, 1),
  };
  ins.shelves = {
      ins.grid.idx(1, 0),
      ins.grid.idx(1, 2),
  };
  ins.target_starts = ins.shelves;
  ins.target_goals = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 2),
  };
  ins.target_goal_sets = {
      {ins.grid.idx(0, 0)},
      {ins.grid.idx(0, 2)},
  };
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.finalize();

  PhysConfig physical = initial_phys_config(ins);
  physical.kappa[0] = 0;

  const ShelfTask delivery{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 0},
          ins.grid.idx(1, 0), ins.grid.idx(0, 0)},
      {RootDemand{0, ins.grid.idx(0, 0)}},
      1,
      StorageTransfer{
          ins.grid.idx(0, 0),
          {ins.grid.idx(1, 0), ins.grid.idx(0, 0)}}};
  const ShelfTask pickup{
      TaskId{
          ShelfSelector{ShelfSelector::Kind::TARGET, 1},
          ins.grid.idx(0, 0), ins.grid.idx(0, 2)},
      {RootDemand{1, ins.grid.idx(0, 2)}},
      100,
      StorageTransfer{
          ins.grid.idx(0, 2),
          {
              ins.grid.idx(0, 0),
              ins.grid.idx(0, 1),
              ins.grid.idx(0, 2)}}};

  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->tau_guide = ins.target_goals;
  epoch->task_graph.tasks = {pickup};

  CarrierGuidance guidance;
  guidance.upper_epoch = epoch;
  guidance.custody_by_robot.resize(ins.n_robots());
  guidance.custody_by_robot[0] =
      carrier_detail::make_custody(delivery, -1);
  carrier_detail::ensure_transfer_identity(
      *guidance.custody_by_robot[0], 0,
      phys_config_hash(physical));
  guidance.rho_task_id.resize(ins.n_robots());
  guidance.rho_task_id[1] = pickup.id;
  guidance.rho_ready_index = {-1, 0};

  carrier_detail::LowerDist lower(ins.grid);
  const auto order =
      carrier_detail::task_br_robot_order(
          physical, guidance, lower);

  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order.front(), 0)
      << "a carried target one move from its assigned goal must reserve "
         "that final move before ordinary free-robot dispatch";
}

TEST(dd_vacancy_terminal_delivery_regression,
     factorial_case_delivers_b11_when_its_goal_first_becomes_free)
{
  const auto path =
      std::string(DD_TEST_DIR) +
      "/../benchmark/viz_web/warehouse_case_proposal/factorial_suite/"
      "instances/g6_dmedium_ascarce_pcross_heavy_gsingleton_seed0.yaml";
  const auto ins = load_dd_instance(path);
  DDStats stats;
  const auto result =
      solve_carrier_lacam_result(ins, 10.0, 0, &stats);

  ASSERT_TRUE(result.solved());
  ASSERT_GT(ins.n_targets(), 11u);
  const int target = 11;
  const int goal = ins.target_goal_sets[target].front();
  auto physical = initial_phys_config(ins);
  bool saw_terminal_opportunity = false;
  for (const auto& ops : result.plan) {
    int carrier = -1;
    for (size_t robot = 0; robot < physical.kappa.size(); ++robot)
      if (physical.kappa[robot] == target) {
        carrier = static_cast<int>(robot);
        break;
      }
    if (!saw_terminal_opportunity && carrier >= 0 &&
        physical.target_pos[target] != goal &&
        upper_cell_free_for_carried_target(
            physical, target, goal) &&
        std::find(
            physical.robots.begin(), physical.robots.end(), goal) ==
            physical.robots.end()) {
      carrier_detail::LowerDist lower(ins.grid);
      if (lower.dist(goal, physical.robots[carrier]) == 1) {
        saw_terminal_opportunity = true;
        ASSERT_LT(carrier, static_cast<int>(ops.size()));
        EXPECT_EQ(ops[carrier].kind, Op::MOVE);
        EXPECT_EQ(ops[carrier].to, goal)
            << "b11 must not be diverted after reaching an open adjacent "
               "fixed goal";
      }
    }
    const auto next = apply_ops(ins, physical, ops);
    ASSERT_TRUE(next.has_value());
    physical = *next;
    if (is_dd_goal(ins, physical)) break;
  }

  EXPECT_TRUE(saw_terminal_opportunity);
  EXPECT_LE(dd_plan_cost_probe(ins, result.plan).ticks, 140);
}
