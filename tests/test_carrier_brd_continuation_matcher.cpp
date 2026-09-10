// REGRESSION: a locked-only continuation still records a matcher pass.
#include <br_lacam_upper.hpp>
#include <dd_planner.hpp>

#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Cell = std::pair<int, int>;

DDInstance make_instance(
    const std::vector<std::string>& rows,
    const std::vector<std::string>& storage,
    const std::vector<Cell>& robots,
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
  for (const auto& [r, c] : robots)
    ins.robots.push_back(ins.grid.idx(r, c));
  for (const auto& [r, c] : shelves)
    ins.shelves.push_back(ins.grid.idx(r, c));
  for (const auto& [start, goal] : targets) {
    ins.target_starts.push_back(
        ins.grid.idx(start.first, start.second));
    ins.target_goals.push_back(
        ins.grid.idx(goal.first, goal.second));
  }
  ins.finalize();
  return ins;
}

UpperShelfHandle target(int id)
{
  return UpperShelfHandle{
      UpperShelfHandle::Kind::TARGET, id};
}

BRUpperConstraintEntry transfer(
    UpperShelfHandle shelf, int endpoint,
    std::vector<int> route)
{
  return BRUpperConstraintEntry::make_transfer(
      shelf, StorageTransfer{endpoint, std::move(route)});
}

FrozenTaskPlan one_wave_plan(
    const DDInstance& ins, const std::vector<int>& tau)
{
  const auto start = br_labeled_initial_state(ins);
  const auto successor = validate_complete_upper_action(
      ins, start,
      {
          transfer(
              target(0), tau[0],
              {ins.grid.idx(0, 0), tau[0]}),
          transfer(
              target(1), tau[1],
              {
                  ins.grid.idx(1, 0),
                  ins.grid.idx(1, 1),
                  ins.grid.idx(1, 2),
                  ins.grid.idx(1, 3),
                  tau[1],
              }),
      });
  EXPECT_TRUE(successor.has_value());
  if (!successor.has_value()) return {};

  BRUpperSearchResult upper;
  upper.exit_reason = BRUpperExitReason::SOLVED;
  upper.states = {start, successor->state};
  upper.transitions = {successor->transition};
  const auto frozen = compile_frozen_task_plan(ins, upper, tau);
  EXPECT_TRUE(frozen.has_value());
  return frozen.has_value() ? *frozen : FrozenTaskPlan{};
}

}  // namespace

TEST(carrier_brd_continuation_matcher,
     locked_only_continuation_still_calls_matcher)
{
  const auto ins = make_instance(
      {".....", "....."}, {"SS...", "S...S"},
      {{0, 0}, {1, 0}}, {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 4}}});
  const std::vector<int> tau{
      ins.grid.idx(0, 1), ins.grid.idx(1, 4)};
  const auto frozen = one_wave_plan(ins, tau);
  CarrierBRDStats brd;

  const auto result = dd_execute_frozen_task_plan_probe(
      ins, frozen, tau, 5.0, 0, nullptr, &brd);

  ASSERT_TRUE(result.solved());
  ASSERT_EQ(brd.dispatch_epochs, 2);
  ASSERT_EQ(brd.dispatches.size(), 2u);
  EXPECT_EQ(brd.match_calls, brd.dispatch_epochs);
}
