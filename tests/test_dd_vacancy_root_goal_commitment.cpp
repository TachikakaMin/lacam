// PROTECTED active-root goal commitment contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

namespace {

PairCostEntry cost_entry(int goal, double cost)
{
  PairPlan plan;
  plan.estimated_cost = cost;
  plan.exact = true;
  return PairCostEntry{goal, plan};
}

}  // namespace

TEST(dd_vacancy_root_goal_commitment,
     active_root_constraint_overrides_a_positive_pair_cost_gap)
{
  DDInstance ins;
  ins.grid = DDGrid({".."});
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_goals = ins.target_starts;
  ins.target_goal_sets = {
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
  };
  const UpperSignature upper{ins.target_starts, {}};
  PairCostTable table(2);
  table[0] = {
      cost_entry(ins.grid.idx(0, 0), 1),
      cost_entry(ins.grid.idx(0, 1), 0),
  };
  table[1] = {
      cost_entry(ins.grid.idx(0, 0), 0),
      cost_entry(ins.grid.idx(0, 1), 1),
  };
  const RootGoalCommitment commitment{
      {0, ins.grid.idx(0, 0)},
  };

  EXPECT_EQ(
      carrier_detail::solve_tau_guide(ins, upper, table),
      (std::vector<int>{
          ins.grid.idx(0, 1),
          ins.grid.idx(0, 0),
      }));
  EXPECT_EQ(
      carrier_detail::solve_tau_guide_with_commitments(
          ins, upper, table, commitment),
      (std::vector<int>{
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 1),
      }));
}

TEST(dd_vacancy_root_goal_commitment,
     lazy_certificate_matches_full_constrained_pair_cost)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_starts = ins.shelves;
  ins.target_goal_sets = {
      {ins.grid.idx(0, 2), ins.grid.idx(0, 3)},
      {ins.grid.idx(0, 2), ins.grid.idx(0, 3)},
  };
  ins.finalize();
  const auto upper =
      carrier_detail::make_upper_signature(
          initial_phys_config(ins));
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  DDDistCache full_distance(ins.grid);
  DDDistCache lazy_distance(ins.grid);
  const auto full = carrier_detail::build_pair_cost_table(
      ins, upper, full_distance, topology, 1, 1, 1);
  const RootGoalCommitment commitment{
      {0, ins.grid.idx(0, 3)},
  };
  const auto expected =
      carrier_detail::solve_tau_guide_with_commitments(
          ins, upper, full, commitment);
  const auto lazy =
      carrier_detail::build_lazy_pair_cost_assignment(
          ins, upper, lazy_distance, topology, 1, 1, 1,
          nullptr, nullptr, &commitment);

  EXPECT_EQ(lazy.tau, expected);
  ASSERT_EQ(lazy.table.size(), lazy.tau.size());
  for (size_t target = 0; target < lazy.tau.size(); ++target) {
    const auto selected = std::find_if(
        lazy.table[target].begin(),
        lazy.table[target].end(),
        [&](const PairCostEntry& entry) {
          return entry.goal == lazy.tau[target];
        });
    ASSERT_NE(selected, lazy.table[target].end());
    EXPECT_TRUE(selected->plan.exact);
  }
}
