// PROTECTED adjacent-epoch continuity propagation contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_carrier.hpp>

#include <memory>
#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_continuity_transition,
     unrelated_loaded_move_preserves_previous_root_first_transfer)
{
  DDInstance ins;
  ins.grid = DDGrid({"....", "...."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(1, 1),
  };
  ins.target_starts = ins.shelves;
  ins.target_goal_sets = {
      {
          ins.grid.idx(0, 3),
          ins.grid.idx(1, 3),
      },
      {
          ins.grid.idx(0, 3),
          ins.grid.idx(1, 3),
      },
  };
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.finalize();

  auto previous = initial_phys_config(ins);
  previous.kappa[0] = 0;
  const std::vector<Op> executed_ops{
      Op::make_move(ins.grid.idx(0, 1))};
  const auto next = apply_ops(ins, previous, executed_ops);
  ASSERT_TRUE(next.has_value());

  const RootDemand stable_root{1, ins.grid.idx(1, 3)};
  const ShelfSelector stable_shelf{
      ShelfSelector::Kind::TARGET, 1};
  const TransferKey stable_key{
      stable_shelf, ins.grid.idx(1, 1), ins.grid.idx(1, 2)};

  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {
      ins.grid.idx(0, 3),
      ins.grid.idx(1, 3),
  };
  previous_epoch->target_priority.assign(
      ins.n_targets(), 0);
  previous_epoch->task_graph.tasks.push_back(
      ShelfTask{
          TaskId{
              stable_shelf,
              stable_key.source,
              stable_key.endpoint},
          {stable_root},
          0,
          StorageTransfer{
              stable_key.endpoint,
              {stable_key.source, stable_key.endpoint}}});
  previous_epoch->task_graph.predecessors.emplace_back();
  previous_epoch->task_graph.successors.emplace_back();

  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot.resize(
      ins.n_robots());

  DDDistCache distance(ins.grid);
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto guidance =
      carrier_detail::build_task_br_guidance(
          ins, *next, distance, topology, 1, 1, 1,
          &previous, &previous_guidance, &executed_ops);

  ASSERT_NE(guidance.upper_epoch, nullptr);
  const auto continuity =
      guidance.upper_epoch->transfer_continuity.find(
          stable_root);
  ASSERT_NE(
      continuity,
      guidance.upper_epoch->transfer_continuity.end());
  EXPECT_EQ(continuity->second, stable_key);
}
