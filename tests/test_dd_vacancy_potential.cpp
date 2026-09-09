// PROTECTED vacancy-aware clearance tests.
// Written before implementation on 2026-09-06 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <br_lacam_upper.hpp>
#include <dd_carrier.hpp>
#include <utils.hpp>

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Cell = std::pair<int, int>;

DDInstance make_storage_instance(
    const std::vector<std::string>& rows,
    const std::vector<std::string>& storage_rows,
    const std::vector<Cell>& occupied)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (int r = 0; r < ins.grid.height; ++r)
    for (int c = 0; c < ins.grid.width; ++c)
      ins.shelf_storage[ins.grid.idx(r, c)] =
          storage_rows[r][c] == 'S';
  for (const auto& [r, c] : occupied)
    ins.shelves.push_back(ins.grid.idx(r, c));
  ins.finalize();
  return ins;
}

UpperSignature anonymous_upper(
    const DDInstance& ins, const std::vector<Cell>& occupied)
{
  UpperSignature upper;
  for (const auto& [r, c] : occupied)
    upper.anon_pos.push_back(ins.grid.idx(r, c));
  std::sort(upper.anon_pos.begin(), upper.anon_pos.end());
  return upper;
}

DDInstance load_report_case()
{
  return load_dd_instance(
      std::string(DD_TEST_DIR) +
      "/../benchmark/instances_brap_pool/g10x10/"
      "brap_h10w10_a1_e1_B_seed0_pool.yaml");
}

struct EmptyReservationView {
  bool destination_reserved(int) const { return false; }
  bool endpoint_reserved(int) const { return false; }
  bool has_distinct_endpoint_reservations() const { return false; }
  bool distinct_endpoint_reserved(int) const { return false; }
};

std::set<int> candidate_endpoints(
    const carrier_detail::OrderedShelfCandidates& candidates)
{
  std::set<int> endpoints;
  for (int index = 0; index < candidates.count; ++index)
    endpoints.insert(candidates.endpoints[index]);
  return endpoints;
}

}  // namespace

TEST(dd_vacancy_potential, dense_storage_distance_matches_vacancy_moves)
{
  std::vector<Cell> occupied;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      if (Cell{r, c} != Cell{2, 0})
        occupied.push_back({r, c});
  const auto ins = make_storage_instance(
      {"....", "....", "...."},
      {"SSSS", "SSSS", "SSSS"}, occupied);
  const auto upper_signature = anonymous_upper(ins, occupied);
  const auto upper = carrier_detail::make_abstract_upper_state(
      ins, upper_signature);
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto potential =
      carrier_detail::build_vacancy_potential(
          ins, upper, topology);

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) {
      const int distance = std::abs(r - 2) + c;
      const auto& cost = potential.cost[ins.grid.idx(r, c)];
      EXPECT_EQ(cost.pushes, distance);
      EXPECT_EQ(cost.loaded_steps, distance);
      EXPECT_EQ(cost.service_ticks, 3 * distance);
    }
}

TEST(dd_vacancy_potential,
     report_case_prefers_the_four_push_clearance_direction)
{
  const auto ins = load_report_case();
  const auto physical = initial_phys_config(ins);
  const auto upper_signature =
      carrier_detail::make_upper_signature(physical);
  const auto upper =
      carrier_detail::make_abstract_upper_state(ins, upper_signature);
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto potential =
      carrier_detail::build_vacancy_potential(
          ins, upper, topology);
  const auto cell = [&](int r, int c) {
    return ins.grid.idx(r, c);
  };

  EXPECT_EQ(potential.cost[cell(5, 2)].pushes, 4);
  EXPECT_EQ(potential.cost[cell(4, 3)].pushes, 6);

  DDDistCache upper_wall(ins.grid);
  const ShelfSelector target{ShelfSelector::Kind::TARGET, 0};
  const RootDemand root{0, cell(3, 0)};
  const auto candidates =
      carrier_detail::ordered_shelf_candidate_window(
          ins, upper, target, root, nullptr, true, upper_wall,
          potential, EmptyReservationView{});

  ASSERT_EQ(candidates.count, 4);
  EXPECT_EQ(candidates.endpoints[0], cell(5, 2));
  EXPECT_EQ(
      candidate_endpoints(candidates),
      (std::set<int>{
          cell(4, 3), cell(5, 2), cell(5, 4), cell(6, 3)}))
      << "vacancy-aware clearance may reorder candidates but must not "
         "change the legal candidate set";
}

TEST(dd_vacancy_potential,
     multiple_vacancies_use_the_nearest_source_and_break_ties_stably)
{
  const std::vector<Cell> occupied{{0, 1}, {0, 2}, {0, 3}};
  const auto ins = make_storage_instance(
      {"....."}, {"SSSSS"}, occupied);
  const auto upper = carrier_detail::make_abstract_upper_state(
      ins, anonymous_upper(ins, occupied));
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto first =
      carrier_detail::build_vacancy_potential(
          ins, upper, topology);
  const auto second =
      carrier_detail::build_vacancy_potential(
          ins, upper, topology);

  const int middle = ins.grid.idx(0, 2);
  EXPECT_EQ(first.cost[middle].pushes, 2);
  EXPECT_EQ(first.cost[middle].loaded_steps, 2);
  EXPECT_EQ(
      first.next_vacancy_cell[middle], ins.grid.idx(0, 1));
  EXPECT_EQ(first.cost, second.cost);
  EXPECT_EQ(first.next_vacancy_cell, second.next_vacancy_cell);
}

TEST(dd_vacancy_potential, channel_routes_charge_loaded_distance)
{
  const std::vector<Cell> occupied{{0, 0}, {0, 2}};
  const auto ins = make_storage_instance(
      {"....."}, {"S.S.S"}, occupied);
  const auto upper = carrier_detail::make_abstract_upper_state(
      ins, anonymous_upper(ins, occupied));
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto potential =
      carrier_detail::build_vacancy_potential(
          ins, upper, topology);

  const int left = ins.grid.idx(0, 0);
  const int middle = ins.grid.idx(0, 2);
  EXPECT_EQ(potential.cost[middle],
            (carrier_detail::ClearanceCost{4, 1, 2}));
  EXPECT_EQ(potential.cost[left],
            (carrier_detail::ClearanceCost{8, 2, 4}));
  EXPECT_EQ(potential.next_vacancy_cell[left], middle);
}

TEST(dd_vacancy_potential, no_vacancy_leaves_every_storage_cell_infinite)
{
  const std::vector<Cell> occupied{{0, 0}, {0, 1}, {0, 2}};
  const auto ins = make_storage_instance(
      {"..."}, {"SSS"}, occupied);
  const auto upper = carrier_detail::make_abstract_upper_state(
      ins, anonymous_upper(ins, occupied));
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto potential =
      carrier_detail::build_vacancy_potential(
          ins, upper, topology);

  for (int cell = 0; cell < ins.grid.size(); ++cell)
    EXPECT_FALSE(potential.cost[cell].finite());
}

TEST(dd_vacancy_potential,
     pair_cost_bounds_and_lazy_assignment_keep_their_contracts)
{
  const auto ins = load_report_case();
  const auto upper = carrier_detail::make_upper_signature(
      initial_phys_config(ins));
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  DDDistCache distance(ins.grid);
  for (const Cell goal : {Cell{3, 0}, Cell{4, 0}, Cell{5, 0}}) {
    const int goal_cell = ins.grid.idx(goal.first, goal.second);
    const auto exact = carrier_detail::pair_cost(
        ins, upper, 0, goal_cell, distance, topology,
        1.0, 1.0, 1.0);
    const auto lower =
        carrier_detail::pair_cost_prefix_lower_bound(
            ins, upper, 0, goal_cell, distance, topology,
            1.0, 1.0, 1.0, 8);
    ASSERT_TRUE(exact.exact);
    EXPECT_LE(lower.estimated_cost, exact.estimated_cost);
  }

  DDDistCache lazy_distance(ins.grid);
  DDDistCache full_distance(ins.grid);
  const auto lazy =
      carrier_detail::build_lazy_pair_cost_assignment(
          ins, upper, lazy_distance, topology, 1.0, 1.0, 1.0);
  const auto full = carrier_detail::build_pair_cost_table(
      ins, upper, full_distance, topology, 1.0, 1.0, 1.0);
  EXPECT_EQ(
      lazy.tau,
      carrier_detail::solve_tau_guide(ins, upper, full));
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

TEST(dd_vacancy_potential,
     report_case_fixed_goal_upper_plan_uses_at_most_twenty_one_transfers)
{
  const auto ins = load_report_case();
  const std::vector<int> tau{ins.grid.idx(3, 0)};
  Deadline deadline(5000);
  const auto result = solve_carrier_br_upper(
      ins, br_labeled_initial_state(ins), tau, &deadline);

  ASSERT_TRUE(result.solved());
  size_t transfers = 0;
  for (const auto& transition : result.transitions)
    transfers += transition.transfers.size();
  EXPECT_LE(transfers, 21u);
}
