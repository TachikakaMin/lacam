#include "../lacam/src/carrier_guidance.hpp"

#include <algorithm>

#include "gtest/gtest.h"

namespace {

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

DDInstance staged_bound_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.shelves = {0, 1, 3, 4};
  ins.target_starts = {0};
  ins.target_goals = {0};
  ins.target_goal_sets = {{0, 2}};
  ins.finalize();
  return ins;
}

carrier_detail::LazyPairAssignment build_staged(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& distance,
    const carrier_detail::StorageTransferTopology& topology,
    carrier_detail::PairCostDependencyContext& dependency_context,
    const UpperSignature* previous_upper = nullptr,
    const PairCostTable* previous_table = nullptr,
    const PairAssignmentHungarianState*
        previous_hungarian_state = nullptr)
{
  carrier_detail::VacancyPotentialCache potential_cache(
      ins.grid.size());
  return carrier_detail::build_lazy_pair_cost_assignment(
      ins, upper, distance, topology, 1.0, 1.0, 1.0,
      nullptr, &potential_cache, nullptr,
      &dependency_context, previous_upper, previous_table,
      previous_hungarian_state);
}

}  // namespace

TEST(pair_staged_lower_bound,
     obvious_nonoptimal_edge_stays_at_cheap_bound)
{
  const auto ins = staged_bound_instance();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::PairCostDependencyContext dependency_context;
  DDDistCache distance(ins.grid);
  const UpperSignature upper{{0}, {1, 3, 4}};

  const auto result = build_staged(
      ins, upper, distance, topology,
      dependency_context);

  ASSERT_FALSE(result.cutoff);
  EXPECT_EQ(result.tau, std::vector<int>({0}));
  EXPECT_EQ(result.total_edges, 2);
  EXPECT_EQ(result.evaluated_edges, 1);
  EXPECT_EQ(result.prefix_refinements, 0);

  const auto& selected =
      entry_for_goal(result.table, 0, 0);
  const auto& alternative =
      entry_for_goal(result.table, 0, 2);
  EXPECT_TRUE(selected.plan.exact);
  EXPECT_EQ(
      selected.plan.bound_stage,
      PairBoundStage::EXACT);
  EXPECT_FALSE(alternative.plan.exact);
  EXPECT_EQ(
      alternative.plan.bound_stage,
      PairBoundStage::CHEAP_BOUND);

  DDDistCache full_distance(ins.grid);
  const auto full = carrier_detail::build_pair_cost_table(
      ins, upper, full_distance, topology,
      1.0, 1.0, 1.0);
  EXPECT_EQ(
      result.tau,
      carrier_detail::solve_tau_guide(
          ins, upper, full));
  EXPECT_LE(
      alternative.plan.estimated_cost,
      entry_for_goal(full, 0, 2)
          .plan.estimated_cost);
}

TEST(pair_staged_lower_bound,
     cheap_bound_survives_unrelated_single_shelf_move)
{
  const auto ins = staged_bound_instance();
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::PairCostDependencyContext dependency_context;
  DDDistCache distance(ins.grid);
  const UpperSignature first_upper{{0}, {1, 3, 4}};
  const auto first = build_staged(
      ins, first_upper, distance, topology,
      dependency_context);

  const UpperSignature moved_upper{{0}, {1, 2, 4}};
  const auto moved = build_staged(
      ins, moved_upper, distance, topology,
      dependency_context, &first_upper, &first.table,
      &first.hungarian_state);

  ASSERT_FALSE(moved.cutoff);
  EXPECT_EQ(moved.tau, std::vector<int>({0}));
  EXPECT_EQ(moved.reused_edges, 2);
  EXPECT_EQ(moved.prefix_refinements, 0);
  const auto& alternative =
      entry_for_goal(moved.table, 0, 2);
  EXPECT_FALSE(alternative.plan.exact);
  EXPECT_EQ(
      alternative.plan.bound_stage,
      PairBoundStage::CHEAP_BOUND);
}
