#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"........"});
  ins.robots = {ins.grid.idx(0, 0), ins.grid.idx(0, 7)};
  ins.finalize();
  return ins;
}

ShelfTaskGraph graph(int first_priority = 4)
{
  ShelfTaskGraph out;
  out.tasks = {
      ShelfTask{
          TaskId{
              ShelfSelector{
                  ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 1},
              1, 2},
          {RootDemand{0, 2}},
          first_priority,
          StorageTransfer{2, {1, 2}}},
      ShelfTask{
          TaskId{
              ShelfSelector{
                  ShelfSelector::Kind::ANON_AT_EPOCH_CELL, 6},
              6, 5},
          {RootDemand{1, 5}},
          4,
          StorageTransfer{5, {6, 5}}},
  };
  out.predecessors = {{}, {}};
  out.successors = {{}, {}};
  return out;
}

DDReadyMatchProbe warm(
    const DDInstance& ins, const PhysConfig& state,
    const ShelfTaskGraph& tasks,
    const DDReadyMatchProbe& parent)
{
  return dd_match_ready_tasks_probe(
      ins, state, tasks, {0, 1},
      &parent.rho_task_id, &parent.rho_transfer_key,
      parent.rho_state.has_value() ? &*parent.rho_state : nullptr);
}

}  // namespace

TEST(dd_rho_shadow_telemetry,
     changed_row_distribution_is_known_only_for_reuse_attempts)
{
  const auto ins = instance();
  const auto state = initial_phys_config(ins);
  const auto tasks = graph();

  const auto cold = dd_match_ready_tasks_probe(
      ins, state, tasks, {0, 1}, nullptr);
  EXPECT_FALSE(cold.telemetry.incremental_changed_rows_valid);

  const auto anchored = warm(ins, state, tasks, cold);
  EXPECT_TRUE(anchored.telemetry.incremental_changed_rows_valid);
  const auto stable = warm(ins, state, tasks, anchored);
  EXPECT_TRUE(stable.telemetry.incremental_changed_rows_valid);
  EXPECT_EQ(stable.telemetry.incremental_changed_rows, 0);

  const auto changed_model =
      warm(ins, state, graph(5), stable);
  EXPECT_FALSE(
      changed_model.telemetry.incremental_changed_rows_valid);
  EXPECT_EQ(
      changed_model.telemetry.incremental_fallback,
      RhoIncrementalFallbackReason::COLUMN_VALUE_CHANGED);
}
