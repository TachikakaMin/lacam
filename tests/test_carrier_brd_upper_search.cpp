// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P2.
// Shared-kernel upper-search tests written before implementation.
#include <br_lacam_upper.hpp>

#include <unordered_set>
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

}  // namespace

TEST(carrier_brd_upper_search,
     shared_kernel_solves_one_full_transit_transfer)
{
  const auto ins = make_instance(
      {"....."}, {"S...S"}, {{0, 0}}, {{{0, 0}, {0, 4}}});
  const auto start = br_labeled_initial_state(ins);
  const std::vector<int> tau{ins.grid.idx(0, 4)};

  const auto result =
      solve_carrier_br_upper(ins, start, tau, nullptr);

  ASSERT_EQ(result.exit_reason, BRUpperExitReason::SOLVED);
  ASSERT_EQ(result.states.size(), 2u);
  ASSERT_EQ(result.transitions.size(), 1u);
  EXPECT_EQ(result.states.front(), start);
  EXPECT_EQ(result.states.back().target_pos, tau);
  ASSERT_EQ(result.transitions[0].transfers.size(), 1u);
  EXPECT_EQ(
      result.transitions[0].transfers[0].transfer.route,
      (std::vector<int>{
          ins.grid.idx(0, 0), ins.grid.idx(0, 1),
          ins.grid.idx(0, 2), ins.grid.idx(0, 3),
          ins.grid.idx(0, 4)}));
  EXPECT_GT(result.stats.exact_oracle_calls, 0);
}

TEST(carrier_brd_upper_search,
     blocker_requires_two_immutable_upper_edges)
{
  const auto ins = make_instance(
      {"....."}, {"S.S.S"}, {{0, 0}, {0, 2}},
      {{{0, 0}, {0, 2}}});
  const auto start = br_labeled_initial_state(ins);
  const std::vector<int> tau{ins.grid.idx(0, 2)};

  const auto result =
      solve_carrier_br_upper(ins, start, tau, nullptr);

  ASSERT_EQ(result.exit_reason, BRUpperExitReason::SOLVED);
  ASSERT_EQ(result.states.size(), 3u);
  ASSERT_EQ(result.transitions.size(), 2u);
  ASSERT_EQ(result.transitions[0].transfers.size(), 1u);
  EXPECT_EQ(
      result.transitions[0].transfers[0].stable_shelf.kind,
      UpperShelfHandle::Kind::ANONYMOUS);
  EXPECT_EQ(
      result.transitions[0].transfers[0].transfer.endpoint,
      ins.grid.idx(0, 4));
  ASSERT_EQ(result.transitions[1].transfers.size(), 1u);
  EXPECT_EQ(
      result.transitions[1].transfers[0].stable_shelf,
      (UpperShelfHandle{UpperShelfHandle::Kind::TARGET, 0}));
  EXPECT_EQ(result.states.back().target_pos, tau);
}

TEST(carrier_brd_upper_search,
     age_priority_and_variable_order_are_deterministic)
{
  const auto ins = make_instance(
      {"...."}, {"SSSS"}, {{0, 0}, {0, 1}, {0, 3}},
      {{{0, 0}, {0, 2}}, {{0, 1}, {0, 1}}});
  const auto root = br_labeled_initial_state(ins);
  const std::vector<int> tau{
      ins.grid.idx(0, 2), ins.grid.idx(0, 1)};
  const auto root_metadata =
      br_upper_root_metadata(root, tau);

  EXPECT_EQ(root_metadata.age, (std::vector<int>{0, 0}));
  EXPECT_EQ(root_metadata.target_priority,
            (std::vector<int>{2, 0}));
  ASSERT_EQ(root_metadata.variable_order.size(), 3u);
  EXPECT_EQ(
      root_metadata.variable_order[0],
      (UpperShelfHandle{UpperShelfHandle::Kind::TARGET, 0}));
  EXPECT_EQ(
      root_metadata.variable_order[1],
      (UpperShelfHandle{UpperShelfHandle::Kind::TARGET, 1}));
  EXPECT_EQ(
      root_metadata.variable_order[2],
      (UpperShelfHandle{UpperShelfHandle::Kind::ANONYMOUS, 0}));

  auto child = root;
  child.target_pos[1] = ins.grid.idx(0, 3);
  child.anonymous_pos[0] = ins.grid.idx(0, 1);
  const auto child_metadata =
      br_upper_child_metadata(child, tau, root_metadata);
  EXPECT_EQ(child_metadata.age, (std::vector<int>{1, 1}));
  EXPECT_EQ(child_metadata.target_priority,
            (std::vector<int>{2, 1}));
  EXPECT_EQ(
      child_metadata.variable_order[0],
      (UpperShelfHandle{UpperShelfHandle::Kind::TARGET, 0}));
  EXPECT_EQ(
      child_metadata.variable_order[1],
      (UpperShelfHandle{UpperShelfHandle::Kind::TARGET, 1}));
}

TEST(carrier_brd_upper_search,
     labeled_anonymous_positions_are_distinct_closed_keys)
{
  BRLabeledUpperState a;
  a.target_pos = {0};
  a.anonymous_pos = {1, 2};
  BRLabeledUpperState b = a;
  std::swap(b.anonymous_pos[0], b.anonymous_pos[1]);

  std::unordered_set<
      BRLabeledUpperState, BRLabeledUpperStateHash> closed;
  closed.insert(a);
  closed.insert(b);
  EXPECT_EQ(closed.size(), 2u);
}
