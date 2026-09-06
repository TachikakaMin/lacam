#include <tapf_planner.hpp>

#include <optional>
#include <vector>

#include "gtest/gtest.h"

namespace {

RhoColumnKey task_column(int value, int task_index)
{
  RhoColumnKey key;
  key.kind = RhoColumnKind::TASK;
  key.transfer.shelf =
      ShelfSelector{ShelfSelector::Kind::TARGET, value};
  key.transfer.source = value;
  key.transfer.endpoint = value + 10;
  key.task.shelf = key.transfer.shelf;
  key.task.from = key.transfer.source;
  key.task.to = key.transfer.endpoint;
  key.task_index = task_index;
  return key;
}

RhoColumnKey idle_column(int robot)
{
  RhoColumnKey key;
  key.kind = RhoColumnKind::OWN_IDLE;
  key.owner_robot = robot;
  return key;
}

RhoColumnModelVersion base_model()
{
  RhoColumnModelVersion model;
  model.ordered_columns = {
      task_column(1, 4),
      task_column(2, 7),
      idle_column(0),
      idle_column(1),
  };
  model.service = {3, 5, 0, 0};
  model.urgency = {8, 12, 0, 0};
  model.root_delay = {0, 0, 0, 0};
  model.endpoint_conflict_version = {11, 12, 0, 0};
  model.mode = DispatchMode::EXECUTE;
  model.mode_semantics_version = 1;
  model.objective_version = static_cast<uint32_t>(
      RhoObjectiveVersion::ADDITIVE_SERVICE_MINUS_DEFER_V1);
  model.scaling_version = 1;
  model.inf_version = 1;
  model.canonical_version = 1;
  rho_finalize_column_model(model);
  return model;
}

RhoRowFingerprint row_fingerprint(
    int position, int kappa, DispatchMode phase,
    std::vector<uint8_t> eligibility,
    std::optional<RhoColumnKey> anchor)
{
  RhoRowFingerprint row;
  row.robot_position = position;
  row.kappa = kappa;
  row.phase = phase;
  row.eligibility = std::move(eligibility);
  row.anchor_used = std::move(anchor);
  return row;
}

RhoNodeAssignmentState base_state()
{
  const std::vector<std::vector<RhoCost>> cost = {
      {-5, 4, 0, kRhoAssignmentInf},
      {3, -7, kRhoAssignmentInf, 0},
  };
  auto full = solve_rho_assignment_state_full(cost);
  EXPECT_TRUE(full.assignment.feasible);

  RhoNodeAssignmentState state;
  state.optimum = std::move(full.state);
  state.column_model = base_model();
  const auto old_anchor = task_column(2, 7);
  state.row_fingerprint = {
      row_fingerprint(
          20, KAPPA_FREE, DispatchMode::EXECUTE,
          {1, 1, 1, 0}, old_anchor),
      row_fingerprint(
          21, KAPPA_FREE, DispatchMode::EXECUTE,
          {1, 1, 0, 1}, std::nullopt),
  };
  state.mate = {
      task_column(1, 4),
      task_column(2, 7),
  };
  state.anchor_used = {
      old_anchor,
      std::nullopt,
  };
  return state;
}

}  // namespace

TEST(dd_rho_shadow_state,
     hash_collision_still_classifies_exact_value_change)
{
  auto parent = base_model();
  auto child = parent;
  child.urgency[0] += 1;
  // Simulate a collision: exact comparison must still inspect values.
  child.quick_hash = parent.quick_hash;
  EXPECT_FALSE(rho_column_models_exactly_equal(parent, child));
  EXPECT_EQ(
      classify_rho_column_model_change(parent, child),
      RhoIncrementalFallbackReason::COLUMN_VALUE_CHANGED);
}

TEST(dd_rho_shadow_state,
     classifies_structure_conflict_and_every_contract_version)
{
  const auto parent = base_model();

  auto changed = parent;
  changed.ordered_columns.push_back(idle_column(2));
  changed.service.push_back(0);
  changed.urgency.push_back(0);
  changed.root_delay.push_back(0);
  changed.endpoint_conflict_version.push_back(0);
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::SHAPE_CHANGED);

  changed = parent;
  changed.ordered_columns[0] = task_column(9, 4);
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::COLUMN_IDENTITY_CHANGED);

  changed = parent;
  changed.endpoint_conflict_version[0] += 1;
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::CONFLICT_CHANGED);

  changed = parent;
  changed.mode = DispatchMode::PREPARE;
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::MODE_CHANGED);

  changed = parent;
  ++changed.mode_semantics_version;
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::MODE_CHANGED);

  changed = parent;
  ++changed.objective_version;
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::OBJECTIVE_VERSION_CHANGED);

  changed = parent;
  ++changed.scaling_version;
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::SCALING_VERSION_CHANGED);

  changed = parent;
  ++changed.inf_version;
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::INF_VERSION_CHANGED);

  changed = parent;
  ++changed.canonical_version;
  rho_finalize_column_model(changed);
  EXPECT_EQ(
      classify_rho_column_model_change(parent, changed),
      RhoIncrementalFallbackReason::CANONICAL_VERSION_CHANGED);
}

TEST(dd_rho_shadow_state,
     row_fingerprint_tracks_custody_phase_eligibility_and_anchor)
{
  const auto anchor = task_column(1, 4);
  auto row = row_fingerprint(
      20, KAPPA_FREE, DispatchMode::EXECUTE,
      {1, 1, 1, 0}, anchor);

  auto changed = row;
  changed.robot_position += 1;
  EXPECT_NE(row, changed);
  changed = row;
  changed.kappa = 0;
  EXPECT_NE(row, changed);
  changed = row;
  changed.custody = anchor.transfer;
  EXPECT_NE(row, changed);
  changed = row;
  changed.phase = DispatchMode::PREPARE;
  EXPECT_NE(row, changed);
  changed = row;
  changed.eligibility[1] = 0;
  EXPECT_NE(row, changed);
  changed = row;
  changed.anchor_used = task_column(2, 7);
  EXPECT_NE(row, changed);
}

TEST(dd_rho_shadow_state,
     mate_and_anchor_are_separate_and_new_anchor_changes_row)
{
  auto parent = base_state();
  ASSERT_NE(parent.mate[0], parent.anchor_used[0]);

  auto current_rows = parent.row_fingerprint;
  current_rows[0].anchor_used = parent.mate[0];
  const auto decision = assess_rho_incremental_reuse(
      parent, parent.column_model, current_rows,
      /*transition_valid=*/true,
      /*parent_stale=*/false);
  EXPECT_TRUE(decision.may_repair);
  EXPECT_EQ(decision.fallback, RhoIncrementalFallbackReason::NONE);
  EXPECT_EQ(decision.changed_rows, std::vector<int>({0}));
}

TEST(dd_rho_shadow_state,
     lift_style_cross_row_eligibility_change_marks_every_affected_row)
{
  const auto parent = base_state();
  auto current_rows = parent.row_fingerprint;
  current_rows[0].kappa = 0;
  current_rows[0].eligibility[0] = 0;
  current_rows[1].eligibility[0] = 0;

  const auto decision = assess_rho_incremental_reuse(
      parent, parent.column_model, current_rows, true, false);
  EXPECT_TRUE(decision.may_repair);
  EXPECT_EQ(decision.changed_rows, std::vector<int>({0, 1}));
}

TEST(dd_rho_shadow_state,
     reuse_rejects_missing_stale_shape_changed_and_invalid_parent)
{
  const auto parent = base_state();
  const auto rows = parent.row_fingerprint;

  auto decision = assess_rho_incremental_reuse(
      std::nullopt, parent.column_model, rows, true, false);
  EXPECT_FALSE(decision.may_repair);
  EXPECT_EQ(
      decision.fallback,
      RhoIncrementalFallbackReason::NO_PARENT_STATE);

  decision = assess_rho_incremental_reuse(
      parent, parent.column_model, rows, false, false);
  EXPECT_FALSE(decision.may_repair);
  EXPECT_EQ(
      decision.fallback,
      RhoIncrementalFallbackReason::STALE_OR_REWIRED_PARENT);

  auto short_rows = rows;
  short_rows.pop_back();
  decision = assess_rho_incremental_reuse(
      parent, parent.column_model, short_rows, true, false);
  EXPECT_FALSE(decision.may_repair);
  EXPECT_EQ(
      decision.fallback,
      RhoIncrementalFallbackReason::SHAPE_CHANGED);

  auto invalid = parent;
  invalid.optimum.mate_l[0] = invalid.optimum.mate_l[1];
  decision = assess_rho_incremental_reuse(
      invalid, invalid.column_model, rows, true, false);
  EXPECT_FALSE(decision.may_repair);
  EXPECT_EQ(
      decision.fallback,
      RhoIncrementalFallbackReason::STATE_VALIDATION_FAILED);
}

TEST(dd_rho_shadow_state,
     carrier_guidance_owns_independent_execute_and_prepare_states)
{
  CarrierGuidance parent;
  parent.rho_execute_state = base_state();
  parent.rho_prepare_state = base_state();
  CarrierGuidance sibling = parent;

  ASSERT_TRUE(parent.rho_execute_state.has_value());
  ASSERT_TRUE(sibling.rho_execute_state.has_value());
  sibling.rho_execute_state->optimum.row_potential[0] += 9;
  sibling.rho_execute_state->mate[0] = task_column(2, 7);
  sibling.rho_prepare_state.reset();

  EXPECT_NE(
      sibling.rho_execute_state->optimum.row_potential,
      parent.rho_execute_state->optimum.row_potential);
  EXPECT_NE(
      sibling.rho_execute_state->mate,
      parent.rho_execute_state->mate);
  EXPECT_TRUE(parent.rho_prepare_state.has_value());
  EXPECT_FALSE(sibling.rho_prepare_state.has_value());
}

TEST(dd_rho_shadow_state, fallback_reasons_have_stable_names)
{
  EXPECT_STREQ(
      rho_incremental_fallback_reason_name(
          RhoIncrementalFallbackReason::NO_PARENT_STATE),
      "NO_PARENT_STATE");
  EXPECT_STREQ(
      rho_incremental_fallback_reason_name(
          RhoIncrementalFallbackReason::SHADOW_MISMATCH),
      "SHADOW_MISMATCH");
  EXPECT_STREQ(
      rho_incremental_fallback_reason_name(
          RhoIncrementalFallbackReason::INF_VERSION_CHANGED),
      "INF_VERSION_CHANGED");
}
