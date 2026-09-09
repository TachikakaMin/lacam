// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P4.
// All-PENDING matcher tests written before implementation.
#include <dd_planner.hpp>
#include <utils.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance line_instance(
    const std::string& row,
    const std::vector<int>& robot_columns)
{
  DDInstance ins;
  ins.grid = DDGrid({row});
  for (const int column : robot_columns)
    ins.robots.push_back(ins.grid.idx(0, column));
  ins.finalize();
  return ins;
}

ShelfTask make_task(
    int from, int to, int priority,
    int selector_value = -1)
{
  if (selector_value < 0) selector_value = from;
  return ShelfTask{
      TaskId{
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              selector_value},
          from, to},
      {},
      priority,
      StorageTransfer{to, {from, to}}};
}

ShelfTaskGraph make_graph(std::vector<ShelfTask> tasks)
{
  ShelfTaskGraph graph;
  graph.tasks = std::move(tasks);
  graph.predecessors.resize(graph.tasks.size());
  graph.successors.resize(graph.tasks.size());
  return graph;
}

long real_assignment_count(const DDReadyMatchProbe& probe)
{
  return std::count_if(
      probe.rho_ready_index.begin(), probe.rho_ready_index.end(),
      [](int index) { return index >= 0; });
}

DDReadyMatchProbe match_all_pending(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph,
    const std::vector<int>& ready,
    const Deadline* deadline = nullptr,
    const std::vector<uint8_t>* eligible = nullptr)
{
  return dd_match_ready_tasks_probe(
      ins, physical, graph, ready, nullptr,
      DispatchMode::EXECUTE, nullptr,
      CandidateAdmission::KEEP_ALL_PENDING_ROWS,
      deadline, eligible);
}

}  // namespace

TEST(carrier_brd_matcher,
     explicit_default_admission_preserves_production_result)
{
  const auto ins = line_instance(".....", {0, 4});
  const auto physical = initial_phys_config(ins);
  const auto graph = make_graph(
      {
          make_task(
              ins.grid.idx(0, 1), ins.grid.idx(0, 2), 3),
          make_task(
              ins.grid.idx(0, 3), ins.grid.idx(0, 2), 1),
      });

  const auto implicit_default = dd_match_ready_tasks_probe(
      ins, physical, graph, {0, 1}, nullptr);
  const auto explicit_default = dd_match_ready_tasks_probe(
      ins, physical, graph, {0, 1}, nullptr,
      DispatchMode::EXECUTE, nullptr,
      CandidateAdmission::DROP_GLOBALLY_UNREACHABLE);

  EXPECT_EQ(implicit_default.status, RhoMatchStatus::OK);
  EXPECT_EQ(explicit_default.status, RhoMatchStatus::OK);
  EXPECT_EQ(
      implicit_default.rho_task_id,
      explicit_default.rho_task_id);
  EXPECT_EQ(
      implicit_default.rho_transfer_key,
      explicit_default.rho_transfer_key);
  EXPECT_EQ(
      implicit_default.rho_ready_index,
      explicit_default.rho_ready_index);
  EXPECT_EQ(
      implicit_default.telemetry.matrix_rows,
      explicit_default.telemetry.matrix_rows);
  EXPECT_EQ(
      implicit_default.telemetry.matrix_cols,
      explicit_default.telemetry.matrix_cols);
  EXPECT_EQ(
      implicit_default.telemetry.column_identity_fingerprint,
      explicit_default.telemetry.column_identity_fingerprint);
  EXPECT_EQ(
      implicit_default.telemetry.column_value_fingerprint,
      explicit_default.telemetry.column_value_fingerprint);
}

TEST(carrier_brd_matcher,
     all_pending_keeps_globally_unreachable_task_as_forced_dummy)
{
  const auto ins = line_instance("..@..", {0});
  const auto physical = initial_phys_config(ins);
  const auto graph = make_graph(
      {
          make_task(
              ins.grid.idx(0, 1), ins.grid.idx(0, 0), 1),
          make_task(
              ins.grid.idx(0, 4), ins.grid.idx(0, 3), 9),
      });

  const auto match =
      match_all_pending(ins, physical, graph, {0, 1});

  EXPECT_EQ(match.status, RhoMatchStatus::OK);
  EXPECT_EQ(match.telemetry.candidates_after_priority, 2);
  EXPECT_EQ(match.telemetry.no_reachable_robot_filtered, 0);
  EXPECT_EQ(match.telemetry.rows_without_finite_real_edge, 1);
  EXPECT_EQ(match.telemetry.maximum_real_cardinality, 1);
  EXPECT_EQ(match.telemetry.matrix_rows, 2);
  EXPECT_EQ(match.telemetry.matrix_cols, 2);
  EXPECT_EQ(match.telemetry.real_assignments, 1);
  EXPECT_EQ(real_assignment_count(match), 1);
  ASSERT_TRUE(match.rho_task_id[0].has_value());
  EXPECT_EQ(*match.rho_task_id[0], graph.tasks[0].id);
}

TEST(carrier_brd_matcher,
     all_pending_with_zero_eligible_robots_keeps_every_row)
{
  const auto ins = line_instance(".....", {0, 4});
  const auto physical = initial_phys_config(ins);
  const auto graph = make_graph(
      {
          make_task(
              ins.grid.idx(0, 1), ins.grid.idx(0, 2), 1),
          make_task(
              ins.grid.idx(0, 3), ins.grid.idx(0, 2), 1),
      });
  const std::vector<uint8_t> eligible{0, 0};

  const auto match = match_all_pending(
      ins, physical, graph, {0, 1}, nullptr, &eligible);

  EXPECT_EQ(match.status, RhoMatchStatus::OK);
  EXPECT_EQ(match.telemetry.matrix_rows, 2);
  EXPECT_EQ(match.telemetry.matrix_cols, 2);
  EXPECT_EQ(match.telemetry.rows_without_finite_real_edge, 2);
  EXPECT_EQ(match.telemetry.maximum_real_cardinality, 0);
  EXPECT_EQ(match.telemetry.real_assignments, 0);
  EXPECT_EQ(real_assignment_count(match), 0);
}

TEST(carrier_brd_matcher,
     hall_deficiency_uses_maximum_real_cardinality)
{
  const auto ins = line_instance("...@...", {0, 4, 6});
  const auto physical = initial_phys_config(ins);
  const auto graph = make_graph(
      {
          make_task(
              ins.grid.idx(0, 1), ins.grid.idx(0, 0), 3),
          make_task(
              ins.grid.idx(0, 2), ins.grid.idx(0, 1), 2),
          make_task(
              ins.grid.idx(0, 5), ins.grid.idx(0, 4), 1),
      });

  const auto match =
      match_all_pending(ins, physical, graph, {0, 1, 2});

  EXPECT_EQ(match.status, RhoMatchStatus::OK);
  EXPECT_EQ(match.telemetry.rows_without_finite_real_edge, 0);
  EXPECT_EQ(match.telemetry.maximum_real_cardinality, 2);
  EXPECT_EQ(match.telemetry.matrix_rows, 3);
  EXPECT_EQ(match.telemetry.matrix_cols, 4);
  EXPECT_EQ(match.telemetry.real_assignments, 2);
  EXPECT_EQ(real_assignment_count(match), 2);
}

TEST(carrier_brd_matcher,
     forced_dummy_row_does_not_change_actionable_assignment)
{
  const auto ins = line_instance("....@..", {0});
  const auto physical = initial_phys_config(ins);
  const auto actionable = make_graph(
      {
          make_task(
              ins.grid.idx(0, 1), ins.grid.idx(0, 2), 1),
          make_task(
              ins.grid.idx(0, 3), ins.grid.idx(0, 2), 2),
      });
  const auto with_forced_dummy = make_graph(
      {
          actionable.tasks[0],
          actionable.tasks[1],
          make_task(
              ins.grid.idx(0, 6), ins.grid.idx(0, 5), 999),
      });

  const auto baseline =
      match_all_pending(ins, physical, actionable, {0, 1});
  const auto extended = match_all_pending(
      ins, physical, with_forced_dummy, {0, 1, 2});

  ASSERT_EQ(baseline.status, RhoMatchStatus::OK);
  ASSERT_EQ(extended.status, RhoMatchStatus::OK);
  EXPECT_EQ(baseline.rho_task_id, extended.rho_task_id);
  EXPECT_EQ(
      baseline.rho_transfer_key,
      extended.rho_transfer_key);
  EXPECT_EQ(
      baseline.rho_ready_index,
      extended.rho_ready_index);
  EXPECT_EQ(baseline.telemetry.maximum_real_cardinality, 1);
  EXPECT_EQ(extended.telemetry.maximum_real_cardinality, 1);
  EXPECT_EQ(extended.telemetry.rows_without_finite_real_edge, 1);
}

TEST(carrier_brd_matcher,
     expired_budget_is_cutoff_not_a_zero_cardinality_result)
{
  const auto ins = line_instance(".....", {0});
  const auto physical = initial_phys_config(ins);
  const auto graph = make_graph(
      {
          make_task(
              ins.grid.idx(0, 1), ins.grid.idx(0, 2), 1),
      });
  const Deadline expired(-1);

  const auto match =
      match_all_pending(ins, physical, graph, {0}, &expired);

  EXPECT_EQ(match.status, RhoMatchStatus::CUTOFF);
  EXPECT_EQ(real_assignment_count(match), 0);
}
