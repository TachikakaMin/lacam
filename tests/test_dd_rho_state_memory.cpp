#include <tapf_planner.hpp>

#include <vector>

#include "gtest/gtest.h"

TEST(dd_rho_state_memory,
     logical_payload_is_stable_under_copy_and_grows_with_matrix)
{
  RhoNodeAssignmentState state;
  state.optimum =
      solve_rho_assignment_state_full({{0, 1}}).state;
  state.column_model.ordered_columns.resize(2);
  state.column_model.service.resize(2);
  state.column_model.urgency.resize(2);
  state.column_model.root_delay.resize(2);
  state.column_model.endpoint_conflict_version.resize(2);
  state.row_fingerprint.resize(1);
  state.row_fingerprint[0].eligibility = {1, 1};
  state.mate.resize(1);
  state.anchor_used.resize(1);

  const size_t base =
      rho_node_assignment_state_payload_bytes(state);
  EXPECT_GT(base, sizeof(RhoNodeAssignmentState));
  const auto copied = state;
  EXPECT_EQ(
      rho_node_assignment_state_payload_bytes(copied), base);

  state.optimum.real_cost[0].resize(32, 0);
  state.optimum.columns = 32;
  state.optimum.mate_l.resize(32);
  state.optimum.mate_r.resize(32);
  state.optimum.row_potential.resize(32);
  state.optimum.column_potential.resize(32);
  state.column_model.ordered_columns.resize(32);
  state.column_model.service.resize(32);
  state.column_model.urgency.resize(32);
  state.column_model.root_delay.resize(32);
  state.column_model.endpoint_conflict_version.resize(32);
  state.row_fingerprint[0].eligibility.resize(32);
  EXPECT_GT(
      rho_node_assignment_state_payload_bytes(state), base);
}
