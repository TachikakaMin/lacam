#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

TEST(dd_rho_single_assignment,
     deterministic_hungarian_needs_no_suffix_canonicalization)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 4),
  };
  ins.finalize();
  const auto physical = initial_phys_config(ins);

  ShelfTaskGraph graph;
  graph.tasks = {
      ShelfTask{
          TaskId{
              ShelfSelector{
                  ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
                  ins.grid.idx(0, 1)},
              ins.grid.idx(0, 1),
              ins.grid.idx(0, 2)},
          {RootDemand{0, ins.grid.idx(0, 2)}},
          1,
          StorageTransfer{
              ins.grid.idx(0, 2),
              {ins.grid.idx(0, 1), ins.grid.idx(0, 2)}}},
      ShelfTask{
          TaskId{
              ShelfSelector{
                  ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
                  ins.grid.idx(0, 3)},
              ins.grid.idx(0, 3),
              ins.grid.idx(0, 2)},
          {RootDemand{1, ins.grid.idx(0, 2)}},
          1,
          StorageTransfer{
              ins.grid.idx(0, 2),
              {ins.grid.idx(0, 3), ins.grid.idx(0, 2)}}},
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};

  const auto first =
      dd_match_ready_tasks_probe(
          ins, physical, graph, {0, 1}, nullptr);
  const auto second =
      dd_match_ready_tasks_probe(
          ins, physical, graph, {0, 1}, nullptr);

  EXPECT_EQ(first.status, RhoMatchStatus::OK);
  EXPECT_EQ(first.rho_task_id, second.rho_task_id);
  EXPECT_EQ(first.rho_ready_index, second.rho_ready_index);
  EXPECT_DOUBLE_EQ(first.telemetry.secondary_full_time_ms, 0);
  EXPECT_DOUBLE_EQ(first.telemetry.canonical_time_ms, 0);
}
