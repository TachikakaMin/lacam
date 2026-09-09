#include "../lacam/src/carrier_guidance.hpp"

#include <algorithm>
#include <array>
#include <random>

#include "gtest/gtest.h"

namespace {

void expect_same_pair_plan(
    const PairPlan& actual, const PairPlan& expected)
{
  EXPECT_DOUBLE_EQ(
      actual.estimated_cost, expected.estimated_cost);
  EXPECT_EQ(actual.rollout_steps, expected.rollout_steps);
  EXPECT_EQ(actual.direct_distance, expected.direct_distance);
  EXPECT_EQ(actual.reached_goal, expected.reached_goal);
  EXPECT_EQ(actual.truncated, expected.truncated);
  EXPECT_EQ(actual.stalled, expected.stalled);
  EXPECT_EQ(actual.exact, expected.exact);
  EXPECT_EQ(actual.cutoff, expected.cutoff);
  EXPECT_EQ(actual.bound_stage, expected.bound_stage);
}

const PairCostEntry& entry_for_goal(
    const PairCostTable& table, int target, int goal)
{
  const auto found = std::find_if(
      table[target].begin(), table[target].end(),
      [&](const PairCostEntry& entry) {
        return entry.goal == goal;
      });
  if (found == table[target].end())
    throw std::logic_error("missing PairCost entry");
  return *found;
}

DDInstance disconnected_pair_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"...#..."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.shelf_storage[3] = 0;
  ins.shelves = {0, 1, 4, 5};
  ins.target_starts = {0};
  ins.target_goals = {2};
  ins.target_goal_sets = {{0, 2}};
  ins.finalize();
  return ins;
}

DDInstance dense_pair_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.shelves = {0, 1, 2, 3, 4};
  ins.target_starts = {0, 1};
  ins.target_goals = {4, 5};
  ins.target_goal_sets.assign(
      2, std::vector<int>{0, 1, 2, 3, 4, 5});
  ins.finalize();
  return ins;
}

carrier_detail::LazyPairAssignment build_incremental(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& distance,
    const carrier_detail::StorageTransferTopology& topology,
    carrier_detail::PairCostDependencyContext& dependency_context,
    const UpperSignature* previous_upper = nullptr,
    const PairCostTable* previous_table = nullptr,
    const PairAssignmentHungarianState*
        previous_hungarian_state = nullptr,
    const RootGoalCommitment* commitments = nullptr,
    const RootGoalCommitment*
        previous_commitments = nullptr)
{
  carrier_detail::VacancyPotentialCache potential_cache(
      ins.grid.size());
  return carrier_detail::build_lazy_pair_cost_assignment(
      ins, upper, distance, topology, 1.0, 1.0, 1.0,
      nullptr, &potential_cache, commitments,
      &dependency_context, previous_upper, previous_table,
      previous_hungarian_state, previous_commitments);
}

void expect_incremental_matches_fresh(
    const DDInstance& ins, const UpperSignature& upper,
    const carrier_detail::StorageTransferTopology& topology,
    const carrier_detail::LazyPairAssignment& incremental,
    const RootGoalCommitment* commitments = nullptr)
{
  DDDistCache full_distance(ins.grid);
  const auto full = carrier_detail::build_pair_cost_table(
      ins, upper, full_distance, topology, 1.0, 1.0, 1.0);
  const auto expected_tau =
      commitments == nullptr
          ? carrier_detail::solve_tau_guide(
                ins, upper, full)
          : carrier_detail::
                solve_tau_guide_with_commitments(
                    ins, upper, full, *commitments);
  EXPECT_EQ(incremental.tau, expected_tau);

  carrier_detail::VacancyPotentialCache potential_cache(
      ins.grid.size());
  DDDistCache prefix_distance(ins.grid);
  for (size_t target = 0;
       target < incremental.table.size(); ++target) {
    for (const auto& entry : incremental.table[target]) {
      const auto& full_entry =
          entry_for_goal(
              full, static_cast<int>(target),
              entry.goal);
      if (entry.plan.exact) {
        EXPECT_EQ(
            entry.plan.bound_stage,
            PairBoundStage::EXACT);
        expect_same_pair_plan(
            entry.plan, full_entry.plan);
      } else if (
          entry.plan.bound_stage ==
          PairBoundStage::PREFIX_BOUND) {
        const auto prefix =
            carrier_detail::pair_cost_prefix_lower_bound(
                ins, upper, static_cast<int>(target),
                entry.goal, prefix_distance, topology,
                1.0, 1.0, 1.0, 8, nullptr,
                &potential_cache);
        expect_same_pair_plan(entry.plan, prefix);
      } else {
        ASSERT_EQ(
            entry.plan.bound_stage,
            PairBoundStage::CHEAP_BOUND);
        const auto cheap =
            carrier_detail::pair_cost_cheap_lower_bound(
                ins, upper, static_cast<int>(target),
                entry.goal, prefix_distance, 1.0, 1.0);
        expect_same_pair_plan(entry.plan, cheap);
      }
      if (!entry.plan.exact) {
        EXPECT_FALSE(std::isnan(
            entry.plan.estimated_cost));
        EXPECT_LE(
            entry.plan.estimated_cost,
            full_entry.plan.estimated_cost);
      }
    }
  }
}

}  // namespace

TEST(dd_pair_incremental,
     reuses_edges_when_a_disconnected_shelf_moves)
{
  const auto ins = disconnected_pair_instance();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::PairCostDependencyContext dependency_context;
  DDDistCache distance(ins.grid);
  const auto* left_distance =
      dependency_context.distances_to(
          ins, topology, 0, nullptr);
  ASSERT_NE(left_distance, nullptr);
  EXPECT_FALSE((*left_distance)[4].finite());
  EXPECT_FALSE((*left_distance)[6].finite());

  const UpperSignature first_upper{
      {0}, {1, 4, 5}};
  const auto first = build_incremental(
      ins, first_upper, distance, topology,
      dependency_context);
  ASSERT_EQ(first.table.size(), 1);
  ASSERT_EQ(first.table[0].size(), 2);
  for (const auto& entry : first.table[0])
    EXPECT_TRUE(entry.dependency.complete);

  const UpperSignature moved_upper{
      {0}, {1, 5, 6}};
  const auto moved = build_incremental(
      ins, moved_upper, distance, topology,
      dependency_context, &first_upper, &first.table,
      &first.hungarian_state);
  const auto changed =
      carrier_detail::upper_signature_changed_cells(
          ins, first_upper, moved_upper);
  ASSERT_EQ(first.table[0][1].dependency.cells.size(), 1);
  ASSERT_EQ(changed.size(), 1);
  EXPECT_EQ(
      first.table[0][1].dependency.cells[0] & changed[0],
      uint64_t{0});
  EXPECT_EQ(moved.reused_edges, 2);
  expect_incremental_matches_fresh(
      ins, moved_upper, topology, moved);

  const UpperSignature affected_upper{
      {0}, {2, 5, 6}};
  const auto affected = build_incremental(
      ins, affected_upper, distance, topology,
      dependency_context, &moved_upper, &moved.table,
      &moved.hungarian_state);
  EXPECT_EQ(affected.reused_edges, 2);
  EXPECT_EQ(
      entry_for_goal(affected.table, 0, 2)
          .plan.bound_stage,
      PairBoundStage::CHEAP_BOUND);
  expect_incremental_matches_fresh(
      ins, affected_upper, topology, affected);
}

TEST(dd_pair_incremental,
     nearby_state_sequence_matches_fresh_pair_cost_and_tau)
{
  const auto ins = dense_pair_instance();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::PairCostDependencyContext dependency_context;
  DDDistCache distance(ins.grid);
  const std::vector<UpperSignature> states{
      UpperSignature{{0, 1}, {2, 3, 4}},
      UpperSignature{{0, 1}, {3, 4, 5}},
      UpperSignature{{0, 1}, {2, 4, 5}},
      UpperSignature{{0, 1}, {2, 3, 5}},
      UpperSignature{{0, 1}, {2, 3, 4}},
  };

  carrier_detail::LazyPairAssignment previous;
  const UpperSignature* previous_upper = nullptr;
  long reused_edges = 0;
  for (const auto& upper : states) {
    const auto current = build_incremental(
        ins, upper, distance, topology,
        dependency_context, previous_upper,
        previous_upper != nullptr
            ? &previous.table
            : nullptr,
        previous_upper != nullptr
            ? &previous.hungarian_state
            : nullptr);
    expect_incremental_matches_fresh(
        ins, upper, topology, current);
    reused_edges += current.reused_edges;
    previous = current;
    previous_upper = &upper;
  }
  EXPECT_GT(reused_edges, 0);
}

TEST(dd_pair_incremental,
     randomized_single_shelf_moves_match_fresh_results)
{
  const auto ins = dense_pair_instance();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::PairCostDependencyContext dependency_context;
  DDDistCache distance(ins.grid);
  std::mt19937 rng(76081);

  UpperSignature upper{{0, 1}, {2, 3, 4}};
  int vacancy = 5;
  std::optional<UpperSignature> previous_upper;
  carrier_detail::LazyPairAssignment previous;
  long reused_edges = 0;
  for (int step = 0; step < 32; ++step) {
    const auto current = build_incremental(
        ins, upper, distance, topology,
        dependency_context,
        previous_upper.has_value()
            ? &*previous_upper
            : nullptr,
        previous_upper.has_value()
            ? &previous.table
            : nullptr,
        previous_upper.has_value()
            ? &previous.hungarian_state
            : nullptr);
    expect_incremental_matches_fresh(
        ins, upper, topology, current);
    reused_edges += current.reused_edges;
    previous_upper = upper;
    previous = current;

    const int shelf = static_cast<int>(rng() % 5);
    if (shelf < 2) {
      std::swap(upper.target_pos[shelf], vacancy);
    } else {
      std::swap(upper.anon_pos[shelf - 2], vacancy);
      std::sort(
          upper.anon_pos.begin(), upper.anon_pos.end());
    }
  }
  EXPECT_GT(reused_edges, 0);
}

TEST(dd_pair_incremental,
     commitment_changes_repair_only_affected_rows)
{
  const auto ins = dense_pair_instance();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::PairCostDependencyContext dependency_context;
  DDDistCache distance(ins.grid);
  const UpperSignature upper{{0, 1}, {2, 3, 4}};

  const auto unconstrained = build_incremental(
      ins, upper, distance, topology,
      dependency_context);
  ASSERT_TRUE(unconstrained.hungarian_state.valid);

  const RootGoalCommitment commitment{{0, 5}};
  const auto constrained = build_incremental(
      ins, upper, distance, topology,
      dependency_context, &upper,
      &unconstrained.table,
      &unconstrained.hungarian_state,
      &commitment);
  EXPECT_EQ(constrained.hungarian_full_solves, 0);
  EXPECT_GT(constrained.hungarian_row_repairs, 0);
  expect_incremental_matches_fresh(
      ins, upper, topology, constrained,
      &commitment);

  const auto released = build_incremental(
      ins, upper, distance, topology,
      dependency_context, &upper,
      &constrained.table,
      &constrained.hungarian_state,
      nullptr, &commitment);
  EXPECT_EQ(released.hungarian_full_solves, 0);
  EXPECT_GT(released.hungarian_row_repairs, 0);
  expect_incremental_matches_fresh(
      ins, upper, topology, released);
}

TEST(dd_pair_incremental,
     incremental_hungarian_row_repairs_match_full_solves)
{
  std::mt19937 rng(76082);
  constexpr long double INF = 1e60L;
  for (int trial = 0; trial < 80; ++trial) {
    const int rows = 1 + rng() % 6;
    const int columns = rows + rng() % 4;
    std::vector<std::vector<long double>> cost(
        rows, std::vector<long double>(columns));
    for (auto& row : cost)
      for (auto& entry : row)
        entry = rng() % 100;
    const auto cost_at = [&](int row, int column) {
      return cost[row][column];
    };

    carrier_detail::IncrementalLongDoubleHungarian state;
    auto actual =
        state.solve_full(rows, columns, cost_at);
    auto expected =
        carrier_detail::hungarian_long_double(cost);
    ASSERT_TRUE(actual.feasible);
    ASSERT_TRUE(expected.feasible);
    EXPECT_EQ(actual.cost, expected.cost);

    for (int step = 0; step < 20; ++step) {
      std::vector<int> changed_rows;
      const int change_count =
          1 + rng() % std::min(3, rows);
      while (static_cast<int>(
                 changed_rows.size()) <
             change_count) {
        const int row = rng() % rows;
        if (std::find(
                changed_rows.begin(),
                changed_rows.end(), row) ==
            changed_rows.end())
          changed_rows.push_back(row);
      }
      for (const int row : changed_rows)
        for (int column = 0;
             column < columns; ++column)
          if ((rng() % 3) == 0)
            cost[row][column] = rng() % 100;

      actual =
          state.repair_rows(
              changed_rows, cost_at);
      expected =
          carrier_detail::hungarian_long_double(
              cost);
      ASSERT_EQ(actual.feasible, expected.feasible);
      ASSERT_FALSE(actual.cutoff);
      EXPECT_EQ(actual.cost, expected.cost)
          << "trial=" << trial
          << " step=" << step;
    }

    carrier_detail::IncrementalLongDoubleHungarian
        restored;
    ASSERT_TRUE(restored.restore(state.snapshot()));
    const auto restored_result =
        restored.current_result(cost_at);
    EXPECT_EQ(
        restored_result.cost, expected.cost);

    for (int row = 0; row < rows; ++row) {
      for (int forced_column = 0;
           forced_column < columns;
           ++forced_column) {
        auto forced_state = restored;
        const auto forced_cost =
            [&](int candidate_row,
                int candidate_column) {
              if (candidate_row == row &&
                  candidate_column !=
                      forced_column)
                return INF;
              return cost[candidate_row]
                         [candidate_column];
            };
        const auto forced_actual =
            forced_state.repair_rows(
                {row}, forced_cost);
        auto forced_matrix = cost;
        for (int column = 0;
             column < columns; ++column)
          if (column != forced_column)
            forced_matrix[row][column] = INF;
        const auto forced_expected =
            carrier_detail::hungarian_long_double(
                forced_matrix);
        ASSERT_EQ(
            forced_actual.feasible,
            forced_expected.feasible);
        if (forced_actual.feasible) {
          EXPECT_EQ(
              forced_actual.row_to_col[row],
              forced_column);
          EXPECT_EQ(
              forced_actual.cost,
              forced_expected.cost);
        }
      }
    }
  }
}
