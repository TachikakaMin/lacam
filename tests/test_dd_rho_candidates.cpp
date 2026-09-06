// PROTECTED rho candidate-boundary diagnostics.
// Added before implementation on 2026-09-06 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance line_instance(int width, const std::vector<int>& robots)
{
  DDInstance ins;
  ins.grid = DDGrid({std::string(width, '.')});
  for (const int cell : robots)
    ins.robots.push_back(ins.grid.idx(0, cell));
  ins.finalize();
  return ins;
}

ShelfTask make_task(ShelfSelector shelf, int from, int to, int priority)
{
  return ShelfTask{
      TaskId{shelf, from, to},
      {RootDemand{priority, to}},
      priority,
      StorageTransfer{to, {from, to}}};
}

const RhoCandidateAudit* find_audit(
    const DDReadyMatchProbe& probe, int task_index)
{
  const auto it = std::find_if(
      probe.audit.begin(), probe.audit.end(),
      [&](const RhoCandidateAudit& item) {
        return item.task_index == task_index;
      });
  return it == probe.audit.end() ? nullptr : &*it;
}

}  // namespace

TEST(dd_rho_candidates,
     ordinary_low_priority_task_reaches_the_assignment_problem)
{
  const auto ins = line_instance(8, {0});
  const auto X = initial_phys_config(ins);
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 7)},
          ins.grid.idx(0, 7), ins.grid.idx(0, 6), 9),
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 1)},
          ins.grid.idx(0, 1), ins.grid.idx(0, 2), 1),
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};

  const auto probe =
      dd_match_ready_tasks_probe(ins, X, graph, {0, 1}, nullptr);

  ASSERT_EQ(probe.rho_task_id.size(), 1u);
  ASSERT_TRUE(probe.rho_task_id[0].has_value());
  EXPECT_EQ(*probe.rho_task_id[0], graph.tasks[0].id)
      << "the far task wins by the bottleneck objective";
  EXPECT_EQ(probe.rho_ready_index[0], 0);
  EXPECT_EQ(probe.telemetry.candidates_input, 2);
  EXPECT_EQ(probe.telemetry.candidates_after_key_dedupe, 2);
  EXPECT_EQ(probe.telemetry.candidates_after_shelf_preselect, 2);
  EXPECT_EQ(probe.telemetry.candidates_after_priority, 2);
  EXPECT_EQ(probe.telemetry.priority_filtered, 0);
  EXPECT_EQ(probe.telemetry.matrix_rows, 2);
  EXPECT_EQ(probe.telemetry.matrix_cols, 2);
  EXPECT_TRUE(probe.audit.empty());
}

TEST(dd_rho_candidates,
     duplicate_key_and_same_shelf_preselection_have_distinct_reasons)
{
  const auto ins = line_instance(6, {0});
  const auto X = initial_phys_config(ins);
  const ShelfSelector shelf{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
      ins.grid.idx(0, 2)};
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          shelf, ins.grid.idx(0, 2), ins.grid.idx(0, 3), 9),
      make_task(
          shelf, ins.grid.idx(0, 2), ins.grid.idx(0, 3), 1),
      make_task(
          shelf, ins.grid.idx(0, 2), ins.grid.idx(0, 1), 8),
  };
  graph.predecessors = {{}, {}, {}};
  graph.successors = {{}, {}, {}};

  const auto probe =
      dd_match_ready_tasks_probe(ins, X, graph, {0, 1, 2}, nullptr);

  EXPECT_EQ(probe.telemetry.candidates_input, 3);
  EXPECT_EQ(probe.telemetry.candidates_after_key_dedupe, 2);
  EXPECT_EQ(probe.telemetry.candidates_after_shelf_preselect, 1);
  EXPECT_EQ(probe.telemetry.candidates_after_priority, 1);
  const auto* duplicate = find_audit(probe, 1);
  ASSERT_NE(duplicate, nullptr);
  EXPECT_EQ(duplicate->reason, RhoDropReason::DUPLICATE_TRANSFER_KEY);
  const auto* same_shelf = find_audit(probe, 2);
  ASSERT_NE(same_shelf, nullptr);
  EXPECT_EQ(same_shelf->reason, RhoDropReason::SAME_SHELF_PRESELECTED);
}

TEST(dd_rho_candidates, telemetry_is_finite_and_fingerprints_are_stable)
{
  const auto ins = line_instance(5, {0, 4});
  const auto X = initial_phys_config(ins);
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 1)},
          ins.grid.idx(0, 1), ins.grid.idx(0, 2), 4),
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 3)},
          ins.grid.idx(0, 3), ins.grid.idx(0, 2), 4),
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};

  const auto first =
      dd_match_ready_tasks_probe(ins, X, graph, {0, 1}, nullptr);
  const auto second =
      dd_match_ready_tasks_probe(ins, X, graph, {0, 1}, nullptr);

  const auto& timing = first.telemetry;
  EXPECT_TRUE(std::isfinite(timing.candidate_time_ms));
  EXPECT_TRUE(std::isfinite(timing.matrix_time_ms));
  EXPECT_TRUE(std::isfinite(timing.bottleneck_time_ms));
  EXPECT_TRUE(std::isfinite(timing.secondary_full_time_ms));
  EXPECT_TRUE(std::isfinite(timing.canonical_time_ms));
  EXPECT_GE(timing.candidate_time_ms, 0);
  EXPECT_GE(timing.matrix_time_ms, 0);
  EXPECT_GE(timing.bottleneck_time_ms, 0);
  EXPECT_GE(timing.secondary_full_time_ms, 0);
  EXPECT_GE(timing.canonical_time_ms, 0);
  EXPECT_EQ(
      first.telemetry.column_identity_fingerprint,
      second.telemetry.column_identity_fingerprint);
  EXPECT_EQ(
      first.telemetry.column_value_fingerprint,
      second.telemetry.column_value_fingerprint);
  EXPECT_EQ(first.rho_task_id, second.rho_task_id);
  EXPECT_EQ(first.rho_transfer_key, second.rho_transfer_key);
  EXPECT_EQ(first.rho_ready_index, second.rho_ready_index);
}

TEST(dd_rho_candidates,
     no_reachable_task_has_an_explicit_hard_reason)
{
  DDInstance ins;
  ins.grid = DDGrid({"..@.."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.finalize();
  const auto X = initial_phys_config(ins);
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 1)},
          ins.grid.idx(0, 1), ins.grid.idx(0, 0), 4),
      make_task(
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 4)},
          ins.grid.idx(0, 4), ins.grid.idx(0, 3), 4),
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};

  const auto probe =
      dd_match_ready_tasks_probe(ins, X, graph, {0, 1}, nullptr);

  ASSERT_EQ(probe.rho_task_id.size(), 1u);
  ASSERT_TRUE(probe.rho_task_id[0].has_value());
  EXPECT_EQ(*probe.rho_task_id[0], graph.tasks[0].id);
  EXPECT_EQ(probe.telemetry.no_reachable_robot_filtered, 1);
  EXPECT_EQ(probe.telemetry.priority_filtered, 0);
  EXPECT_EQ(probe.telemetry.matrix_rows, 1);
  EXPECT_EQ(probe.telemetry.matrix_cols, 1);
  const auto* audit = find_audit(probe, 1);
  ASSERT_NE(audit, nullptr);
  EXPECT_EQ(audit->reason, RhoDropReason::NO_REACHABLE_ROBOT);
  EXPECT_EQ(audit->nearest_robot_distance, -1);
}
