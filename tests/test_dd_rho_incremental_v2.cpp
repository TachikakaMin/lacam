// PROTECTED: exact incremental rho contract for the production
// BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2 objective.
// Added before implementation on 2026-09-09 and intentionally observed RED.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance line_instance(
    int width, const std::vector<int>& robot_columns)
{
  DDInstance ins;
  ins.grid = DDGrid({std::string(width, '.')});
  for (const int column : robot_columns)
    ins.robots.push_back(ins.grid.idx(0, column));
  ins.finalize();
  return ins;
}

DDInstance grid_instance(
    const std::vector<std::string>& rows,
    const std::vector<std::pair<int, int>>& robots)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  for (const auto& robot : robots)
    ins.robots.push_back(
        ins.grid.idx(robot.first, robot.second));
  ins.finalize();
  return ins;
}

ShelfTask make_task(int from, int to, int priority = 4)
{
  const ShelfSelector shelf{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
      from};
  return ShelfTask{
      TaskId{shelf, from, to},
      {RootDemand{priority, to}},
      priority,
      StorageTransfer{to, {from, to}}};
}

ShelfTaskGraph two_line_tasks(const DDInstance& ins)
{
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ins.grid.idx(0, 1),
          ins.grid.idx(0, 2)),
      make_task(
          ins.grid.idx(0, 6),
          ins.grid.idx(0, 5)),
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};
  return graph;
}

DDReadyMatchProbe run_probe(
    const DDInstance& ins,
    const PhysConfig& physical,
    const ShelfTaskGraph& graph,
    const std::vector<int>& ready,
    const DDReadyMatchProbe* parent = nullptr,
    bool use_assignment_anchor = true,
    bool reuse_incremental_state = true)
{
  const std::vector<std::optional<TaskId>>* previous_ids =
      parent != nullptr && use_assignment_anchor
          ? &parent->rho_task_id
          : nullptr;
  const std::vector<std::optional<TransferKey>>* previous_keys =
      parent != nullptr && use_assignment_anchor
          ? &parent->rho_transfer_key
          : nullptr;
  const RhoIncrementalState* previous_state = nullptr;
  if (parent != nullptr && reuse_incremental_state) {
    if (!parent->rho_state.has_value()) {
      ADD_FAILURE() << "parent probe has no incremental rho state";
    } else {
      previous_state = &*parent->rho_state;
    }
  }
  return dd_match_ready_tasks_probe(
      ins,
      physical,
      graph,
      ready,
      previous_ids,
      DispatchMode::EXECUTE,
      previous_keys,
      CandidateAdmission::DROP_GLOBALLY_UNREACHABLE,
      nullptr,
      nullptr,
      previous_state);
}

void expect_same_exact_result(
    const DDReadyMatchProbe& warm,
    const DDReadyMatchProbe& cold)
{
  EXPECT_EQ(warm.status, cold.status);
  EXPECT_EQ(
      warm.telemetry.bottleneck,
      cold.telemetry.bottleneck);
  EXPECT_EQ(
      warm.telemetry.secondary_cost,
      cold.telemetry.secondary_cost);
  EXPECT_EQ(warm.rho_task_id, cold.rho_task_id);
  EXPECT_EQ(
      warm.rho_transfer_key,
      cold.rho_transfer_key);
  EXPECT_EQ(warm.rho_ready_index, cold.rho_ready_index);
}

}  // namespace

TEST(dd_rho_incremental_v2,
     unchanged_problem_reuses_zero_rows_exactly)
{
  const auto ins = line_instance(8, {0, 7});
  const auto physical = initial_phys_config(ins);
  const auto graph = two_line_tasks(ins);

  const auto first =
      run_probe(ins, physical, graph, {0, 1});
  ASSERT_TRUE(first.rho_state.has_value());
  EXPECT_EQ(
      first.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::NO_PARENT_STATE);
  EXPECT_EQ(first.telemetry.incremental_full_solves, 1);

  const auto anchored =
      run_probe(ins, physical, graph, {0, 1}, &first);
  ASSERT_TRUE(anchored.rho_state.has_value());
  const auto repeated =
      run_probe(ins, physical, graph, {0, 1}, &anchored);

  expect_same_exact_result(repeated, anchored);
  ASSERT_TRUE(repeated.rho_state.has_value());
  EXPECT_EQ(
      repeated.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::NONE);
  EXPECT_EQ(repeated.telemetry.incremental_changed_rows, 0);
  EXPECT_EQ(repeated.telemetry.incremental_zero_row_reuses, 1);
  EXPECT_EQ(repeated.telemetry.incremental_repairs, 0);
  EXPECT_EQ(repeated.telemetry.incremental_full_solves, 0);
}

TEST(dd_rho_incremental_v2,
     one_robot_move_repairs_one_row_and_matches_cold_full)
{
  const auto ins = line_instance(8, {0, 7});
  const auto root = initial_phys_config(ins);
  const auto graph = two_line_tasks(ins);
  const auto first = run_probe(ins, root, graph, {0, 1});
  const auto anchored =
      run_probe(ins, root, graph, {0, 1}, &first);
  ASSERT_TRUE(anchored.rho_state.has_value());

  auto child = root;
  child.robots[0] = ins.grid.idx(0, 1);
  const auto warm =
      run_probe(ins, child, graph, {0, 1}, &anchored);
  const auto cold = run_probe(
      ins,
      child,
      graph,
      {0, 1},
      &anchored,
      true,
      false);

  expect_same_exact_result(warm, cold);
  ASSERT_TRUE(warm.rho_state.has_value());
  EXPECT_EQ(
      warm.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::NONE);
  EXPECT_EQ(warm.telemetry.incremental_changed_rows, 1);
  EXPECT_EQ(warm.telemetry.incremental_repairs, 1);
  EXPECT_EQ(warm.telemetry.incremental_full_solves, 0);
}

TEST(dd_rho_incremental_v2,
     changed_candidate_model_falls_back_to_exact_full)
{
  const auto ins = line_instance(8, {0, 7});
  const auto physical = initial_phys_config(ins);
  const auto parent_graph = two_line_tasks(ins);
  const auto first =
      run_probe(ins, physical, parent_graph, {0, 1});
  const auto anchored = run_probe(
      ins, physical, parent_graph, {0, 1}, &first);
  ASSERT_TRUE(anchored.rho_state.has_value());

  auto child_graph = parent_graph;
  child_graph.tasks.push_back(
      make_task(
          ins.grid.idx(0, 3),
          ins.grid.idx(0, 4)));
  child_graph.predecessors.push_back({});
  child_graph.successors.push_back({});
  const auto warm = run_probe(
      ins, physical, child_graph, {0, 1, 2}, &anchored);
  const auto cold = run_probe(
      ins,
      physical,
      child_graph,
      {0, 1, 2},
      &anchored,
      true,
      false);

  expect_same_exact_result(warm, cold);
  ASSERT_TRUE(warm.rho_state.has_value());
  EXPECT_EQ(
      warm.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::COLUMN_MODEL_CHANGED);
  EXPECT_EQ(warm.telemetry.incremental_full_solves, 1);
  EXPECT_EQ(warm.telemetry.incremental_repairs, 0);
}

TEST(dd_rho_incremental_v2,
     invalid_parent_state_falls_back_without_changing_output)
{
  const auto ins = line_instance(8, {0, 7});
  const auto physical = initial_phys_config(ins);
  const auto graph = two_line_tasks(ins);
  const auto first =
      run_probe(ins, physical, graph, {0, 1});
  const auto anchored =
      run_probe(ins, physical, graph, {0, 1}, &first);
  ASSERT_TRUE(anchored.rho_state.has_value());

  auto corrupted = anchored;
  ASSERT_FALSE(
      corrupted.rho_state->hungarian.row_to_column.empty());
  corrupted.rho_state->hungarian.row_to_column[0] = -1;
  const auto warm =
      run_probe(ins, physical, graph, {0, 1}, &corrupted);
  const auto cold = run_probe(
      ins,
      physical,
      graph,
      {0, 1},
      &corrupted,
      true,
      false);

  expect_same_exact_result(warm, cold);
  ASSERT_TRUE(warm.rho_state.has_value());
  EXPECT_EQ(
      warm.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::INVALID_PARENT_STATE);
  EXPECT_EQ(warm.telemetry.incremental_full_solves, 1);
  EXPECT_EQ(warm.telemetry.incremental_repairs, 0);
}

TEST(dd_rho_incremental_v2,
     repaired_tied_optimum_keeps_robot_first_canonical_assignment)
{
  const auto ins = grid_instance(
      {"...", "...", "..."},
      {{1, 0}, {1, 2}});
  auto parent_physical = initial_phys_config(ins);
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ins.grid.idx(0, 1),
          ins.grid.idx(0, 0)),
      make_task(
          ins.grid.idx(2, 1),
          ins.grid.idx(2, 0)),
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};

  const auto parent = run_probe(
      ins,
      parent_physical,
      graph,
      {0, 1},
      nullptr,
      false);
  ASSERT_TRUE(parent.rho_state.has_value());
  ASSERT_EQ(parent.rho_ready_index.size(), 2u);
  EXPECT_EQ(parent.rho_ready_index[0], 0);
  EXPECT_EQ(parent.rho_ready_index[1], 1);

  auto child = parent_physical;
  child.robots[0] = ins.grid.idx(1, 1);
  const auto warm = run_probe(
      ins,
      child,
      graph,
      {0, 1},
      &parent,
      false,
      true);
  const auto cold = run_probe(
      ins,
      child,
      graph,
      {0, 1},
      nullptr,
      false);

  expect_same_exact_result(warm, cold);
  EXPECT_EQ(warm.rho_ready_index[0], 0);
  EXPECT_EQ(warm.rho_ready_index[1], 1);
  EXPECT_EQ(warm.telemetry.incremental_changed_rows, 1);
  EXPECT_EQ(warm.telemetry.incremental_repairs, 1);
}

TEST(dd_rho_incremental_v2,
     deterministic_random_row_updates_match_cold_full)
{
  const auto ins = line_instance(12, {0, 11});
  ShelfTaskGraph graph;
  graph.tasks = {
      make_task(
          ins.grid.idx(0, 3),
          ins.grid.idx(0, 4)),
      make_task(
          ins.grid.idx(0, 8),
          ins.grid.idx(0, 7)),
  };
  graph.predecessors = {{}, {}};
  graph.successors = {{}, {}};

  auto physical = initial_phys_config(ins);
  auto parent = run_probe(
      ins,
      physical,
      graph,
      {0, 1},
      nullptr,
      false);
  ASSERT_TRUE(parent.rho_state.has_value());

  std::mt19937 rng(20260909);
  for (int generation = 0; generation < 200; ++generation) {
    int first = static_cast<int>(rng() % 12);
    int second = static_cast<int>(rng() % 12);
    if (second == first)
      second = (second + 1) % 12;
    physical.robots[0] = ins.grid.idx(0, first);
    physical.robots[1] = ins.grid.idx(0, second);

    const auto warm = run_probe(
        ins,
        physical,
        graph,
        {0, 1},
        &parent,
        false,
        true);
    const auto cold = run_probe(
        ins,
        physical,
        graph,
        {0, 1},
        nullptr,
        false);
    expect_same_exact_result(warm, cold);
    ASSERT_TRUE(warm.rho_state.has_value())
        << "generation " << generation;
    EXPECT_EQ(
        warm.telemetry.incremental_fallback,
        RhoIncrementalFallbackReason::NONE)
        << "generation " << generation;
    EXPECT_EQ(warm.telemetry.incremental_full_solves, 0)
        << "generation " << generation;
    EXPECT_EQ(
        warm.telemetry.incremental_repairs +
            warm.telemetry.incremental_zero_row_reuses,
        1)
        << "generation " << generation;
    parent = warm;
  }
}
