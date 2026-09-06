#include "../lacam/src/carrier_guidance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <random>

#include "gtest/gtest.h"

namespace {

struct BruteAssignment {
  bool valid = false;
  long double primary = 0;
  int moved_away = 0;
  std::vector<int> tau;
};

bool better_assignment(const BruteAssignment& a,
                       const BruteAssignment& b)
{
  if (!b.valid) return a.valid;
  if (!a.valid) return false;
  if (a.primary != b.primary)
    return a.primary < b.primary;
  if (a.moved_away != b.moved_away)
    return a.moved_away < b.moved_away;
  return a.tau < b.tau;
}

BruteAssignment brute_tau_guide(
    const DDInstance& ins, const UpperSignature& upper,
    const PairCostTable& table)
{
  BruteAssignment best;
  std::vector<int> tau(ins.n_targets(), -1);
  std::vector<int> all_goals;
  for (const auto& goals : ins.target_goal_sets)
    all_goals.insert(all_goals.end(), goals.begin(), goals.end());
  std::sort(all_goals.begin(), all_goals.end());
  all_goals.erase(
      std::unique(all_goals.begin(), all_goals.end()),
      all_goals.end());
  std::vector<uint8_t> used(all_goals.size(), 0);
  std::function<void(size_t, long double, int)> visit =
      [&](size_t target, long double primary, int moved_away) {
        if (target == ins.n_targets()) {
          const BruteAssignment candidate{
              true, primary, moved_away, tau};
          if (better_assignment(candidate, best)) best = candidate;
          return;
        }
        for (const auto& entry : table[target]) {
          if (!std::isfinite(entry.plan.estimated_cost)) continue;
          const auto goal_it = std::lower_bound(
              all_goals.begin(), all_goals.end(), entry.goal);
          if (goal_it == all_goals.end() || *goal_it != entry.goal)
            continue;
          const size_t goal_index = goal_it - all_goals.begin();
          if (used[goal_index]) continue;
          used[goal_index] = 1;
          tau[target] = entry.goal;
          const bool moves_satisfied_target =
              std::binary_search(
                  ins.target_goal_sets[target].begin(),
                  ins.target_goal_sets[target].end(),
                  upper.target_pos[target]) &&
              upper.target_pos[target] != entry.goal;
          visit(
              target + 1,
              primary +
                  (long double)entry.plan.estimated_cost,
              moved_away + moves_satisfied_target);
          used[goal_index] = 0;
        }
      };
  visit(0, 0, 0);
  return best;
}

}  // namespace

TEST(dd_task_br_exact_matching,
     secondary_tie_break_never_overrides_positive_primary_gap)
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

  const UpperSignature upper{
      ins.target_starts,
      {},
  };
  PairCostTable table(2);
  auto entry = [](int goal, double cost) {
    PairPlan plan;
    plan.estimated_cost = cost;
    return PairCostEntry{goal, plan};
  };
  constexpr double epsilon = 1e-12;
  table[0] = {
      entry(ins.grid.idx(0, 0), epsilon),
      entry(ins.grid.idx(0, 1), 0.0),
  };
  table[1] = {
      entry(ins.grid.idx(0, 0), 0.0),
      entry(ins.grid.idx(0, 1), 0.0),
  };

  EXPECT_EQ(
      carrier_detail::solve_tau_guide(ins, upper, table),
      (std::vector<int>{
          ins.grid.idx(0, 1),
          ins.grid.idx(0, 0),
      }));
}

TEST(dd_task_br_exact_matching,
     exact_three_layer_order_matches_brute_force)
{
  std::mt19937 rng(934857);
  for (int trial = 0; trial < 300; ++trial) {
    const int target_count = 1 + (int)(rng() % 4);
    const int goal_count =
        target_count + (int)(rng() % 2);
    DDInstance ins;
    ins.grid = DDGrid(
        {std::string((size_t)goal_count, '.')});
    std::vector<int> goals(goal_count);
    for (int goal = 0; goal < goal_count; ++goal)
      goals[goal] = ins.grid.idx(0, goal);
    ins.target_starts.assign(
        goals.begin(), goals.begin() + target_count);
    ins.target_goals = ins.target_starts;
    ins.target_goal_sets.assign(target_count, goals);
    const UpperSignature upper{ins.target_starts, {}};

    PairCostTable table(target_count);
    for (int target = 0; target < target_count; ++target) {
      for (const int goal : goals) {
        PairPlan plan;
        plan.estimated_cost =
            (double)(rng() % 5);
        if (rng() % 5 == 0)
          plan.estimated_cost +=
              std::ldexp(
                  (double)(1 + rng() % 3), -40);
        table[target].push_back(
            PairCostEntry{goal, plan});
      }
    }

    const auto expected =
        brute_tau_guide(ins, upper, table);
    ASSERT_TRUE(expected.valid);
    EXPECT_EQ(
        carrier_detail::solve_tau_guide(
            ins, upper, table),
        expected.tau)
        << "trial=" << trial;
  }
}

TEST(dd_task_br_exact_matching,
     prefix_pair_cost_never_exceeds_full_pair_cost)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 0)};
  ins.target_goal_sets.resize(1);
  for (int cell = 0; cell < ins.grid.size(); ++cell)
    ins.target_goal_sets[0].push_back(cell);
  DDDistCache upper_wall(ins.grid);

  for (int target_cell = 0;
       target_cell < ins.grid.size(); ++target_cell) {
    for (int anon_mask = 0;
         anon_mask < (1 << ins.grid.size()); ++anon_mask) {
      if (anon_mask & (1 << target_cell)) continue;
      UpperSignature upper{{target_cell}, {}};
      for (int cell = 0; cell < ins.grid.size(); ++cell)
        if (anon_mask & (1 << cell))
          upper.anon_pos.push_back(cell);
      for (int goal = 0; goal < ins.grid.size(); ++goal) {
        const auto exact = carrier_detail::pair_cost(
            ins, upper, 0, goal, upper_wall,
            1.25, 2.5, 0.75);
        ASSERT_TRUE(
            std::isfinite(exact.estimated_cost));
        for (int prefix_cap = 0; prefix_cap <= 4;
             ++prefix_cap) {
          const auto lower =
              carrier_detail::pair_cost_prefix_lower_bound(
                  ins, upper, 0, goal, upper_wall,
                  1.25, 2.5, 0.75, prefix_cap);
          EXPECT_LE(
              lower.estimated_cost,
              exact.estimated_cost)
              << "target=" << target_cell
              << " mask=" << anon_mask
              << " goal=" << goal
              << " prefix=" << prefix_cap;
        }
      }
    }
  }
}

TEST(dd_task_br_exact_matching,
     eight_step_prefix_remains_a_valid_pair_cost_lower_bound)
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.shelves = {0, 1, 2, 3, 4};
  ins.target_starts = {0, 2};
  ins.target_goal_sets = {
      {0, 1, 2, 3, 4, 5},
      {0, 1, 2, 3, 4, 5},
  };
  ins.finalize();
  DDDistCache upper_wall(ins.grid);
  const UpperSignature upper{{0, 2}, {1, 3, 4}};

  for (int target = 0; target < 2; ++target)
    for (int goal = 0; goal < ins.grid.size(); ++goal) {
      const auto exact = carrier_detail::pair_cost(
          ins, upper, target, goal, upper_wall,
          1.25, 2.5, 0.75);
      const auto lower =
          carrier_detail::pair_cost_prefix_lower_bound(
              ins, upper, target, goal, upper_wall,
              1.25, 2.5, 0.75, 8);
      ASSERT_TRUE(std::isfinite(exact.estimated_cost));
      EXPECT_LE(lower.estimated_cost, exact.estimated_cost)
          << "target=" << target << " goal=" << goal;
    }
}

TEST(dd_task_br_exact_matching,
     lazy_prefix_assignment_matches_the_fully_evaluated_matrix)
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.shelves = {0, 1, 2, 3, 4};
  ins.target_starts = {0, 1, 2};
  ins.target_goal_sets.assign(
      3, std::vector<int>{0, 1, 2, 3, 4, 5});
  ins.finalize();

  std::mt19937 rng(728315);
  std::array<int, 6> cells{{0, 1, 2, 3, 4, 5}};
  for (int trial = 0; trial < 24; ++trial) {
    std::shuffle(cells.begin(), cells.end(), rng);
    const UpperSignature upper{
        {cells[0], cells[1], cells[2]},
        {cells[3], cells[4]}};
    DDDistCache lazy_distance(ins.grid);
    DDDistCache full_distance(ins.grid);
    const auto lazy =
        carrier_detail::build_lazy_pair_cost_assignment(
            ins, upper, lazy_distance, 1.0, 1.0, 1.0);
    const auto full = carrier_detail::build_pair_cost_table(
        ins, upper, full_distance, 1.0, 1.0, 1.0);
    const auto expected =
        carrier_detail::solve_tau_guide(ins, upper, full);
    EXPECT_EQ(lazy.tau, expected) << "trial=" << trial;
    ASSERT_EQ(lazy.table.size(), lazy.tau.size());
    for (size_t target = 0; target < lazy.tau.size(); ++target) {
      const auto selected = std::find_if(
          lazy.table[target].begin(), lazy.table[target].end(),
          [&](const PairCostEntry& entry) {
            return entry.goal == lazy.tau[target];
          });
      ASSERT_NE(selected, lazy.table[target].end());
      EXPECT_TRUE(selected->plan.exact);
    }
  }
}
