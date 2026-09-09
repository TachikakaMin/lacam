// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P2.
// Exact upper-domain tests written before implementation.
#include <br_lacam_upper.hpp>

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

using Cell = std::pair<int, int>;

DDInstance make_upper_instance(
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

UpperShelfHandle anonymous(int stable_id)
{
  return UpperShelfHandle{
      UpperShelfHandle::Kind::ANONYMOUS, stable_id};
}

BRUpperConstraintEntry wait(UpperShelfHandle shelf)
{
  return BRUpperConstraintEntry::make_wait(shelf);
}

BRUpperConstraintEntry transfer(
    UpperShelfHandle shelf, int endpoint, std::vector<int> route)
{
  return BRUpperConstraintEntry::make_transfer(
      shelf, StorageTransfer{endpoint, std::move(route)});
}

}  // namespace

TEST(carrier_brd_upper, exact_oracle_accepts_full_transit_route)
{
  const auto ins = make_upper_instance(
      {"....."}, {"S...S"}, {{0, 0}}, {{{0, 0}, {0, 4}}});
  const auto state = br_labeled_initial_state(ins);
  const auto result = validate_complete_upper_action(
      ins, state,
      {transfer(
          target(0), ins.grid.idx(0, 4),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
           ins.grid.idx(0, 2), ins.grid.idx(0, 3),
           ins.grid.idx(0, 4)})});

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state.target_pos,
            (std::vector<int>{ins.grid.idx(0, 4)}));
  ASSERT_EQ(result->transition.transfers.size(), 1u);
  EXPECT_EQ(result->transition.transfers[0].transfer.route.size(), 5u);
}

TEST(carrier_brd_upper, exhaustive_candidates_are_not_limited_to_four)
{
  const auto ins = make_upper_instance(
      {".....", ".....", ".....", ".....", "....."},
      {"S.S.S", ".....", "S.S.S", ".....", "S.S.S"},
      {{2, 2}}, {{{2, 2}, {0, 0}}});
  const auto state = br_labeled_initial_state(ins);
  const auto candidates =
      br_upper_constraint_candidates(ins, state, target(0));

  ASSERT_EQ(candidates.size(), 9u);
  EXPECT_EQ(
      std::count_if(
          candidates.begin(), candidates.end(),
          [](const BRUpperConstraintEntry& entry) {
            return entry.kind ==
                   BRUpperConstraintEntry::Kind::WAIT;
          }),
      1);
  std::set<int> endpoints;
  for (const auto& candidate : candidates)
    if (candidate.kind ==
        BRUpperConstraintEntry::Kind::TRANSFER)
      endpoints.insert(candidate.transfer.endpoint);
  EXPECT_EQ(endpoints.size(), 8u);
}

TEST(carrier_brd_upper, no_following_uses_wave_start_occupancy)
{
  const auto ins = make_upper_instance(
      {"..."}, {"SSS"}, {{0, 0}, {0, 1}},
      {{{0, 0}, {0, 2}}, {{0, 1}, {0, 1}}});
  const auto state = br_labeled_initial_state(ins);
  const auto result = validate_complete_upper_action(
      ins, state,
      {
          transfer(
              target(0), ins.grid.idx(0, 1),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}),
          transfer(
              target(1), ins.grid.idx(0, 2),
              {ins.grid.idx(0, 1), ins.grid.idx(0, 2)}),
      });

  EXPECT_FALSE(result.has_value())
      << "a shelf may not enter a source vacated in the same upper wave";
}

TEST(carrier_brd_upper, duplicate_endpoints_are_rejected)
{
  const auto ins = make_upper_instance(
      {"...", "...", "..."},
      {".S.", "S.S", "..."},
      {{1, 0}, {1, 2}},
      {{{1, 0}, {1, 0}}, {{1, 2}, {1, 2}}});
  const auto state = br_labeled_initial_state(ins);
  const auto result = validate_complete_upper_action(
      ins, state,
      {
          transfer(
              target(0), ins.grid.idx(0, 1),
              {ins.grid.idx(1, 0), ins.grid.idx(0, 0),
               ins.grid.idx(0, 1)}),
          transfer(
              target(1), ins.grid.idx(0, 1),
              {ins.grid.idx(1, 2), ins.grid.idx(0, 2),
               ins.grid.idx(0, 1)}),
      });

  EXPECT_FALSE(result.has_value());
}

TEST(carrier_brd_upper, non_simple_or_non_enumerated_route_is_rejected)
{
  const auto ins = make_upper_instance(
      {"....."}, {"S...S"}, {{0, 0}}, {{{0, 0}, {0, 4}}});
  const auto state = br_labeled_initial_state(ins);
  const auto result = validate_complete_upper_action(
      ins, state,
      {transfer(
          target(0), ins.grid.idx(0, 4),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
           ins.grid.idx(0, 0), ins.grid.idx(0, 1),
           ins.grid.idx(0, 2), ins.grid.idx(0, 3),
           ins.grid.idx(0, 4)})});

  EXPECT_FALSE(result.has_value());
}

TEST(carrier_brd_upper, exact_successor_set_matches_small_bruteforce)
{
  const auto ins = make_upper_instance(
      {"..."}, {"SSS"}, {{0, 0}, {0, 2}},
      {{{0, 0}, {0, 1}}});
  const auto state = br_labeled_initial_state(ins);
  const std::vector<std::vector<BRUpperConstraintEntry>> actions = {
      {wait(target(0)), wait(anonymous(0))},
      {transfer(
           target(0), ins.grid.idx(0, 1),
           {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}),
       wait(anonymous(0))},
      {wait(target(0)),
       transfer(
           anonymous(0), ins.grid.idx(0, 1),
           {ins.grid.idx(0, 2), ins.grid.idx(0, 1)})},
      {transfer(
           target(0), ins.grid.idx(0, 1),
           {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}),
       transfer(
           anonymous(0), ins.grid.idx(0, 1),
           {ins.grid.idx(0, 2), ins.grid.idx(0, 1)})},
  };

  std::set<std::pair<int, int>> successors;
  for (const auto& action : actions) {
    const auto result =
        validate_complete_upper_action(ins, state, action);
    if (result.has_value())
      successors.emplace(
          result->state.target_pos[0],
          result->state.anonymous_pos[0]);
  }
  EXPECT_EQ(
      successors,
      (std::set<std::pair<int, int>>{
          {ins.grid.idx(0, 0), ins.grid.idx(0, 2)},
          {ins.grid.idx(0, 1), ins.grid.idx(0, 2)},
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1)},
      }));
}

TEST(carrier_brd_upper, transition_payload_has_canonical_shelf_order)
{
  const auto ins = make_upper_instance(
      {"...", "..."}, {"SSS", "SSS"},
      {{0, 0}, {1, 2}}, {{{0, 0}, {0, 1}}});
  const auto state = br_labeled_initial_state(ins);
  const auto result = validate_complete_upper_action(
      ins, state,
      {
          transfer(
              anonymous(0), ins.grid.idx(1, 1),
              {ins.grid.idx(1, 2), ins.grid.idx(1, 1)}),
          transfer(
              target(0), ins.grid.idx(0, 1),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}),
      });

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->transition.transfers.size(), 2u);
  EXPECT_EQ(result->transition.transfers[0].stable_shelf, target(0));
  EXPECT_EQ(result->transition.transfers[1].stable_shelf, anonymous(0));
}

TEST(carrier_brd_upper, anonymous_stable_ids_follow_their_shelves)
{
  const auto ins = make_upper_instance(
      {"....."}, {"SSSSS"}, {{0, 4}, {0, 1}}, {});
  const auto state = br_labeled_initial_state(ins);
  ASSERT_EQ(state.anonymous_pos,
            (std::vector<int>{
                ins.grid.idx(0, 1), ins.grid.idx(0, 4)}));

  const auto result = validate_complete_upper_action(
      ins, state,
      {
          transfer(
              anonymous(0), ins.grid.idx(0, 0),
              {ins.grid.idx(0, 1), ins.grid.idx(0, 0)}),
          wait(anonymous(1)),
      });
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state.anonymous_pos,
            (std::vector<int>{
                ins.grid.idx(0, 0), ins.grid.idx(0, 4)}));

  BRLabeledUpperState labels_swapped = result->state;
  std::swap(
      labels_swapped.anonymous_pos[0],
      labels_swapped.anonymous_pos[1]);
  EXPECT_NE(labels_swapped, result->state);
}
