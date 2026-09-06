#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance line_instance(
    int width, const std::vector<int>& robots)
{
  DDInstance ins;
  ins.grid = DDGrid({std::string(width, '.')});
  for (const int cell : robots)
    ins.robots.push_back(ins.grid.idx(0, cell));
  ins.finalize();
  return ins;
}

ShelfTask make_task(int cell, int to, int priority)
{
  const ShelfSelector shelf{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL, cell};
  return ShelfTask{
      TaskId{shelf, cell, to},
      {RootDemand{priority, to}},
      priority,
      StorageTransfer{to, {cell, to}}};
}

ShelfTaskGraph two_task_graph()
{
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(1, 2, 4),
      make_task(6, 5, 4),
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};
  return graph;
}

DDReadyMatchProbe warm_probe(
    const DDInstance& ins, const PhysConfig& state,
    const ShelfTaskGraph& graph,
    const DDReadyMatchProbe& parent,
    DispatchMode mode = DispatchMode::EXECUTE,
    bool transition_valid = true)
{
  EXPECT_TRUE(parent.rho_state.has_value());
  return dd_match_ready_tasks_probe(
      ins, state, graph, {0, 1},
      &parent.rho_task_id, &parent.rho_transfer_key,
      parent.rho_state.has_value() ? &*parent.rho_state : nullptr,
      nullptr, mode, transition_valid);
}

}  // namespace

TEST(dd_rho_shadow_integration,
     cold_call_builds_valid_shadow_without_changing_canonical_output)
{
  const auto ins = line_instance(8, {0, 7});
  const auto state = initial_phys_config(ins);
  const auto graph = two_task_graph();

  const auto probe =
      dd_match_ready_tasks_probe(
          ins, state, graph, {0, 1}, nullptr);
  const auto repeated =
      dd_match_ready_tasks_probe(
          ins, state, graph, {0, 1}, nullptr);

  ASSERT_TRUE(probe.rho_state.has_value());
  EXPECT_TRUE(validate_rho_assignment_state(
      probe.rho_state->optimum.real_cost,
      probe.rho_state->optimum));
  EXPECT_EQ(
      probe.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::NO_PARENT_STATE);
  EXPECT_EQ(probe.telemetry.incremental_full_solves, 1);
  EXPECT_EQ(probe.telemetry.incremental_repairs, 0);
  EXPECT_EQ(probe.telemetry.shadow_mismatches, 0);
  EXPECT_EQ(
      probe.telemetry.shadow_full_objective,
      probe.telemetry.shadow_incremental_objective);
  EXPECT_EQ(probe.rho_task_id, repeated.rho_task_id);
  EXPECT_EQ(probe.rho_transfer_key, repeated.rho_transfer_key);
  EXPECT_EQ(probe.rho_ready_index, repeated.rho_ready_index);
}

TEST(dd_rho_shadow_integration,
     unchanged_problem_reuses_zero_rows_by_value)
{
  const auto ins = line_instance(8, {0, 7});
  const auto state = initial_phys_config(ins);
  const auto graph = two_task_graph();
  const auto first =
      dd_match_ready_tasks_probe(
          ins, state, graph, {0, 1}, nullptr);
  const auto anchored = warm_probe(ins, state, graph, first);
  ASSERT_EQ(anchored.rho_task_id, first.rho_task_id);
  const auto second =
      warm_probe(ins, state, graph, anchored);

  ASSERT_TRUE(anchored.rho_state.has_value());
  ASSERT_TRUE(second.rho_state.has_value());
  EXPECT_EQ(
      second.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::NONE);
  EXPECT_EQ(second.telemetry.incremental_changed_rows, 0);
  EXPECT_EQ(second.telemetry.incremental_zero_row_reuses, 1);
  EXPECT_EQ(second.telemetry.incremental_repairs, 0);
  EXPECT_EQ(second.telemetry.incremental_augmentations, 0);
  EXPECT_EQ(second.telemetry.shadow_mismatches, 0);
  EXPECT_EQ(second.rho_task_id, anchored.rho_task_id);

  auto isolated = *second.rho_state;
  isolated.optimum.row_potential[0] += 17;
  EXPECT_NE(
      isolated.optimum.row_potential,
      anchored.rho_state->optimum.row_potential);
}

TEST(dd_rho_shadow_integration,
     one_robot_move_repairs_one_row_and_matches_cold_canonical)
{
  const auto ins = line_instance(8, {0, 7});
  const auto parent_state = initial_phys_config(ins);
  const auto graph = two_task_graph();
  const auto first = dd_match_ready_tasks_probe(
      ins, parent_state, graph, {0, 1}, nullptr);
  const auto anchored =
      warm_probe(ins, parent_state, graph, first);
  ASSERT_EQ(anchored.rho_task_id, first.rho_task_id);

  auto child_state = parent_state;
  child_state.robots[0] = ins.grid.idx(0, 1);
  const auto warm =
      warm_probe(ins, child_state, graph, anchored);
  const auto cold = dd_match_ready_tasks_probe(
      ins, child_state, graph, {0, 1}, nullptr);

  ASSERT_TRUE(warm.rho_state.has_value());
  EXPECT_EQ(
      warm.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::NONE);
  EXPECT_EQ(warm.telemetry.incremental_changed_rows, 1);
  EXPECT_EQ(warm.telemetry.incremental_repairs, 1);
  EXPECT_EQ(warm.telemetry.incremental_augmentations, 1);
  EXPECT_EQ(warm.telemetry.shadow_mismatches, 0);
  EXPECT_EQ(warm.rho_task_id, cold.rho_task_id);
  EXPECT_EQ(warm.rho_transfer_key, cold.rho_transfer_key);
  EXPECT_EQ(warm.rho_ready_index, cold.rho_ready_index);
}

TEST(dd_rho_shadow_integration,
     changed_priority_forces_exact_column_value_full_fallback)
{
  const auto ins = line_instance(8, {0, 7});
  const auto state = initial_phys_config(ins);
  const auto parent_graph = two_task_graph();
  const auto first = dd_match_ready_tasks_probe(
      ins, state, parent_graph, {0, 1}, nullptr);

  auto child_graph = parent_graph;
  child_graph.tasks[0].priority += 1;
  const auto second =
      warm_probe(ins, state, child_graph, first);

  EXPECT_EQ(
      second.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::COLUMN_VALUE_CHANGED);
  EXPECT_EQ(second.telemetry.incremental_full_solves, 1);
  EXPECT_EQ(second.telemetry.incremental_repairs, 0);
  EXPECT_EQ(second.telemetry.shadow_mismatches, 0);
}

TEST(dd_rho_shadow_integration,
     invalid_transition_and_mode_change_have_distinct_fallbacks)
{
  const auto ins = line_instance(8, {0, 7});
  const auto state = initial_phys_config(ins);
  const auto graph = two_task_graph();
  const auto first = dd_match_ready_tasks_probe(
      ins, state, graph, {0, 1}, nullptr);

  const auto stale = warm_probe(
      ins, state, graph, first,
      DispatchMode::EXECUTE, false);
  EXPECT_EQ(
      stale.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::STALE_OR_REWIRED_PARENT);
  EXPECT_EQ(stale.telemetry.incremental_full_solves, 1);

  const auto prepare = warm_probe(
      ins, state, graph, first,
      DispatchMode::PREPARE, true);
  EXPECT_EQ(
      prepare.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::MODE_CHANGED);
  EXPECT_EQ(prepare.telemetry.incremental_full_solves, 1);
  EXPECT_EQ(prepare.telemetry.shadow_mismatches, 0);
}
