// PROTECTED regression: every expensive BRD tau/upper subroutine must accept
// and propagate the shared deadline instead of relying only on outer checks.
#include <br_lacam_upper.hpp>
#include <dd_carrier.hpp>
#include <utils.hpp>

#include "../lacam/src/carrier_guidance.hpp"

#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Cell = std::pair<int, int>;

DDInstance make_instance(
    const std::vector<std::string>& rows,
    const std::vector<std::string>& storage,
    const std::vector<Cell>& shelves,
    const std::vector<std::pair<Cell, Cell>>& targets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  ins.shelf_storage.assign(ins.grid.size(), 0);
  for (int r = 0; r < ins.grid.height; ++r)
    for (int c = 0; c < ins.grid.width; ++c)
      ins.shelf_storage[ins.grid.idx(r, c)] =
          storage[r][c] == 'S';
  ins.robots = {ins.grid.idx(0, 0)};
  for (const auto& [r, c] : shelves)
    ins.shelves.push_back(ins.grid.idx(r, c));
  for (const auto& target : targets) {
    ins.target_starts.push_back(
        ins.grid.idx(target.first.first, target.first.second));
    ins.target_goals.push_back(
        ins.grid.idx(target.second.first, target.second.second));
  }
  ins.finalize();
  return ins;
}

UpperShelfHandle target(int id)
{
  return UpperShelfHandle{
      UpperShelfHandle::Kind::TARGET, id};
}

}  // namespace

TEST(carrier_brd_budget_probes,
     pair_cost_rollout_reports_expired_deadline)
{
  const auto ins = make_instance(
      {"....."}, {"S...S"}, {{0, 0}},
      {{{0, 0}, {0, 4}}});
  const auto upper =
      carrier_detail::make_upper_signature(
          initial_phys_config(ins));
  DDDistCache upper_wall(ins.grid);
  const Deadline expired(-1);

  const auto plan = carrier_detail::pair_cost(
      ins, upper, 0, ins.grid.idx(0, 4), upper_wall,
      1, 1, 1, &expired);

  EXPECT_TRUE(plan.cutoff);
  EXPECT_EQ(plan.rollout_steps, 0);
}

TEST(carrier_brd_budget_probes,
     canonical_hungarian_reports_expired_deadline)
{
  using carrier_detail::LexAssignmentCost;
  const std::vector<std::vector<LexAssignmentCost>> cost{
      {{1, 0, false}, {2, 0, false}},
      {{2, 0, false}, {1, 0, false}},
  };
  const Deadline expired(-1);

  const auto result =
      carrier_detail::hungarian_lexicographic(cost, &expired);

  EXPECT_TRUE(result.cutoff);
  EXPECT_FALSE(result.feasible);
}

TEST(carrier_brd_budget_probes,
     upper_partial_task_compiler_reports_expired_deadline)
{
  const auto ins = make_instance(
      {"....."}, {"S...S"}, {{0, 0}},
      {{{0, 0}, {0, 4}}});
  const auto state = br_labeled_initial_state(ins);
  const std::vector<int> tau{ins.grid.idx(0, 4)};
  const auto metadata = br_upper_root_metadata(state, tau);
  const Deadline expired(-1);
  bool cutoff = false;

  const auto successor = complete_partial_upper_action(
      ins, state, tau, metadata, {}, nullptr,
      &expired, &cutoff);

  EXPECT_FALSE(successor.has_value());
  EXPECT_TRUE(cutoff);
}
