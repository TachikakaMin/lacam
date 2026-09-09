// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P2.
// Partial Task-BR completion tests written before implementation.
#include <br_lacam_upper.hpp>

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
  return UpperShelfHandle{UpperShelfHandle::Kind::TARGET, id};
}

}  // namespace

TEST(carrier_brd_upper_partial,
     unconstrained_task_br_proposes_ready_blocker_only)
{
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 0}, {0, 2}},
      {{{0, 0}, {0, 2}}});
  const auto state = br_labeled_initial_state(ins);
  const std::vector<int> tau{ins.grid.idx(0, 2)};
  const auto metadata = br_upper_root_metadata(state, tau);
  BRUpperSearchStats stats;

  const auto successor = complete_partial_upper_action(
      ins, state, tau, metadata, {}, &stats);

  ASSERT_TRUE(successor.has_value());
  EXPECT_EQ(successor->state.target_pos[0], ins.grid.idx(0, 0));
  ASSERT_EQ(successor->state.anonymous_pos.size(), 1u);
  EXPECT_EQ(successor->state.anonymous_pos[0], ins.grid.idx(0, 4));
  ASSERT_EQ(successor->transition.transfers.size(), 1u);
  EXPECT_EQ(
      successor->transition.transfers[0].stable_shelf.kind,
      UpperShelfHandle::Kind::ANONYMOUS);
  EXPECT_EQ(stats.partial_compiler_calls, 1);
  EXPECT_EQ(stats.exact_oracle_calls, 1);
}

TEST(carrier_brd_upper_partial,
     forced_wait_is_reserved_while_other_root_can_progress)
{
  const auto ins = make_instance(
      {"...", "..."}, {"SSS", "SSS"}, {{0, 0}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{1, 0}, {1, 1}}});
  const auto state = br_labeled_initial_state(ins);
  const std::vector<int> tau{
      ins.grid.idx(0, 1), ins.grid.idx(1, 1)};
  const auto metadata = br_upper_root_metadata(state, tau);

  const auto successor = complete_partial_upper_action(
      ins, state, tau, metadata,
      {BRUpperConstraintEntry::make_wait(target(0))}, nullptr);

  ASSERT_TRUE(successor.has_value());
  EXPECT_EQ(successor->state.target_pos[0], ins.grid.idx(0, 0));
  EXPECT_EQ(successor->state.target_pos[1], ins.grid.idx(1, 1));
}

TEST(carrier_brd_upper_partial,
     forced_transfer_is_preserved_in_exact_successor)
{
  const auto ins = make_instance(
      {"...."}, {"SSSS"}, {{0, 0}}, {{{0, 0}, {0, 3}}});
  const auto state = br_labeled_initial_state(ins);
  const std::vector<int> tau{ins.grid.idx(0, 3)};
  const auto metadata = br_upper_root_metadata(state, tau);
  const StorageTransfer forced{
      ins.grid.idx(0, 1),
      {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}};

  const auto successor = complete_partial_upper_action(
      ins, state, tau, metadata,
      {BRUpperConstraintEntry::make_transfer(target(0), forced)},
      nullptr);

  ASSERT_TRUE(successor.has_value());
  ASSERT_EQ(successor->transition.transfers.size(), 1u);
  EXPECT_EQ(successor->transition.transfers[0].stable_shelf, target(0));
  EXPECT_EQ(successor->transition.transfers[0].transfer.route,
            forced.route);
}

TEST(carrier_brd_upper_partial,
     upper_search_uses_partial_completion_before_full_enumeration)
{
  const auto ins = make_instance(
      {"....."}, {"S...S"}, {{0, 0}}, {{{0, 0}, {0, 4}}});
  const auto result = solve_carrier_br_upper(
      ins, br_labeled_initial_state(ins),
      {ins.grid.idx(0, 4)}, nullptr);

  ASSERT_EQ(result.exit_reason, BRUpperExitReason::SOLVED);
  EXPECT_GT(result.stats.partial_compiler_calls, 0);
  EXPECT_GT(result.stats.exact_oracle_calls, 0);
}
