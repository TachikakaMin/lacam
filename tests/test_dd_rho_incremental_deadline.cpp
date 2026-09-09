// PROTECTED: deadline checks for incremental rho post-processing.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <chrono>
#include <thread>

#include "../lacam/src/carrier_guidance.hpp"
#include "gtest/gtest.h"

TEST(dd_rho_incremental_deadline,
     saved_state_validation_and_cost_obey_cutoff)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.finalize();

  ShelfTaskGraph graph;
  const ShelfSelector shelf{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
      ins.grid.idx(0, 2)};
  graph.tasks = {
      ShelfTask{
          TaskId{
              shelf,
              ins.grid.idx(0, 2),
              ins.grid.idx(0, 3)},
          {RootDemand{4, ins.grid.idx(0, 3)}},
          4,
          StorageTransfer{
              ins.grid.idx(0, 3),
              {ins.grid.idx(0, 2),
               ins.grid.idx(0, 3)}}}};
  graph.predecessors = {{}};
  graph.successors = {{}};

  const auto probe = dd_match_ready_tasks_probe(
      ins,
      initial_phys_config(ins),
      graph,
      {0},
      nullptr);
  ASSERT_TRUE(probe.rho_state.has_value());

  Deadline deadline(0);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(2));

  bool validation_cutoff = false;
  EXPECT_FALSE(
      carrier_detail::rho_incremental_state_valid(
          *probe.rho_state, &deadline,
          &validation_cutoff));
  EXPECT_TRUE(validation_cutoff);

  bool cost_cutoff = false;
  long long cost = -1;
  EXPECT_FALSE(
      carrier_detail::rho_incremental_matching_cost(
          *probe.rho_state, cost, &deadline,
          &cost_cutoff));
  EXPECT_TRUE(cost_cutoff);
}
