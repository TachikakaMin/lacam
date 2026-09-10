// REGRESSION: a completion event must expose causally ready tasks from later
// frozen upper waves to the next global lower replan.
#include <br_lacam_upper.hpp>
#include <dd_planner.hpp>

#include <optional>
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

FrozenTaskPlan two_wave_plan(
    const DDInstance& ins, const std::vector<int>& tau)
{
  const auto start = br_labeled_initial_state(ins);
  const auto first = validate_complete_upper_action(
      ins, start,
      {
          transfer(
              target(0), ins.grid.idx(0, 1),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}),
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
  EXPECT_TRUE(first.has_value());
  if (!first.has_value()) return {};

  const auto second = validate_complete_upper_action(
      ins, first->state,
      {
          transfer(
              target(0), tau[0],
              {ins.grid.idx(0, 1), tau[0]}),
          BRUpperConstraintEntry::make_wait(target(1)),
      });
  EXPECT_TRUE(second.has_value());
  if (!second.has_value()) return {};

  BRUpperSearchResult upper;
  upper.exit_reason = BRUpperExitReason::SOLVED;
  upper.states = {start, first->state, second->state};
  upper.transitions = {first->transition, second->transition};
  const auto frozen = compile_frozen_task_plan(ins, upper, tau);
  EXPECT_TRUE(frozen.has_value());
  return frozen.has_value() ? *frozen : FrozenTaskPlan{};
}

}  // namespace

TEST(carrier_brd_cross_wave_replan,
     next_wave_task_starts_before_an_unrelated_active_task_drops)
{
  const auto ins = make_instance(
      {".....", "....."}, {"SSS..", "S...S"},
      {{0, 0}, {1, 0}}, {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 2}}, {{1, 0}, {1, 4}}});
  const std::vector<int> tau{
      ins.grid.idx(0, 2), ins.grid.idx(1, 4)};
  const auto frozen = two_wave_plan(ins, tau);
  ASSERT_EQ(frozen.waves.size(), 2u);

  CarrierBRDStats brd;
  const auto result = dd_execute_frozen_task_plan_probe(
      ins, frozen, tau, 5.0, 0, nullptr, &brd);
  ASSERT_TRUE(result.solved());

  PhysConfig state = initial_phys_config(ins);
  std::optional<size_t> target0_second_lift;
  std::optional<size_t> target1_drop;
  for (size_t step = 0; step < result.plan.size(); ++step) {
    const auto next = apply_ops(ins, state, result.plan[step]);
    ASSERT_TRUE(next.has_value());
    for (size_t robot = 0; robot < state.kappa.size(); ++robot) {
      if (state.kappa[robot] == KAPPA_FREE &&
          next->kappa[robot] == 0)
        target0_second_lift = step;
      if (state.kappa[robot] == 1 &&
          next->kappa[robot] == KAPPA_FREE)
        target1_drop = step;
    }
    state = *next;
  }

  ASSERT_TRUE(target0_second_lift.has_value());
  ASSERT_TRUE(target1_drop.has_value());
  EXPECT_LT(*target0_second_lift, *target1_drop);
  EXPECT_TRUE(is_dd_goal(ins, state));
  EXPECT_GE(brd.dispatch_epochs, 2);
}
