// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P3.
// Frozen task-wave tests written before implementation.
#include <br_lacam_upper.hpp>

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
  return UpperShelfHandle{UpperShelfHandle::Kind::TARGET, id};
}

UpperShelfHandle anonymous(int id)
{
  return UpperShelfHandle{UpperShelfHandle::Kind::ANONYMOUS, id};
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

BRUpperSearchResult make_result(
    std::vector<BRLabeledUpperState> states,
    std::vector<BRUpperTransition> transitions)
{
  BRUpperSearchResult result;
  result.exit_reason = BRUpperExitReason::SOLVED;
  result.states = std::move(states);
  result.transitions = std::move(transitions);
  return result;
}

}  // namespace

TEST(carrier_brd_frozen_tasks,
     one_task_per_transfer_preserves_route_and_canonical_metadata)
{
  const auto ins = make_instance(
      {"...", "..."}, {"S.S", "S.S"},
      {{0, 0}, {1, 2}},
      {{{0, 0}, {0, 2}}, {{1, 2}, {1, 0}}});
  const auto start = br_labeled_initial_state(ins);
  const auto successor = validate_complete_upper_action(
      ins, start,
      {
          transfer(
              target(1), ins.grid.idx(1, 0),
              {ins.grid.idx(1, 2), ins.grid.idx(1, 1),
               ins.grid.idx(1, 0)}),
          transfer(
              target(0), ins.grid.idx(0, 2),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
               ins.grid.idx(0, 2)}),
      });
  ASSERT_TRUE(successor.has_value());
  const std::vector<int> tau{
      ins.grid.idx(0, 2), ins.grid.idx(1, 0)};

  const auto plan = compile_frozen_task_plan(
      ins,
      make_result(
          {start, successor->state},
          {successor->transition}),
      tau);

  ASSERT_TRUE(plan.has_value());
  ASSERT_EQ(plan->waves.size(), 1u);
  const auto& wave = plan->waves[0];
  EXPECT_EQ(wave.expected_before, start);
  EXPECT_EQ(wave.expected_after, successor->state);
  ASSERT_EQ(wave.tasks.size(), 2u);

  const auto& first = wave.tasks[0];
  EXPECT_EQ(first.id, (FrozenTaskId{0, 0}));
  EXPECT_EQ(first.stable_shelf, target(0));
  EXPECT_EQ(
      first.task.id.shelf,
      (ShelfSelector{ShelfSelector::Kind::TARGET, 0}));
  EXPECT_EQ(first.task.id.from, ins.grid.idx(0, 0));
  EXPECT_EQ(first.task.id.to, ins.grid.idx(0, 1));
  EXPECT_EQ(
      first.task.transfer.route,
      (std::vector<int>{
          ins.grid.idx(0, 0), ins.grid.idx(0, 1),
          ins.grid.idx(0, 2)}));
  EXPECT_EQ(first.task.roots,
            (std::vector<RootDemand>{
                RootDemand{0, ins.grid.idx(0, 2)}}));
  EXPECT_EQ(first.task.priority, 1);

  const auto& second = wave.tasks[1];
  EXPECT_EQ(second.id, (FrozenTaskId{0, 1}));
  EXPECT_EQ(second.stable_shelf, target(1));
  EXPECT_EQ(second.task.id.to, ins.grid.idx(1, 1));
  EXPECT_EQ(
      second.task.transfer.route,
      (std::vector<int>{
          ins.grid.idx(1, 2), ins.grid.idx(1, 1),
          ins.grid.idx(1, 0)}));
  EXPECT_EQ(second.task.roots,
            (std::vector<RootDemand>{
                RootDemand{1, ins.grid.idx(1, 0)}}));
  EXPECT_EQ(second.task.priority, 1);
}

TEST(carrier_brd_frozen_tasks,
     repeated_physical_effects_keep_distinct_event_ids)
{
  const auto ins = make_instance(
      {".."}, {"SS"}, {{0, 0}},
      {{{0, 0}, {0, 1}}});
  const auto s0 = br_labeled_initial_state(ins);
  const auto e0 = validate_complete_upper_action(
      ins, s0,
      {transfer(
          target(0), ins.grid.idx(0, 1),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1)})});
  ASSERT_TRUE(e0.has_value());
  const auto e1 = validate_complete_upper_action(
      ins, e0->state,
      {transfer(
          target(0), ins.grid.idx(0, 0),
          {ins.grid.idx(0, 1), ins.grid.idx(0, 0)})});
  ASSERT_TRUE(e1.has_value());
  const auto e2 = validate_complete_upper_action(
      ins, e1->state,
      {transfer(
          target(0), ins.grid.idx(0, 1),
          {ins.grid.idx(0, 0), ins.grid.idx(0, 1)})});
  ASSERT_TRUE(e2.has_value());

  const auto plan = compile_frozen_task_plan(
      ins,
      make_result(
          {s0, e0->state, e1->state, e2->state},
          {e0->transition, e1->transition, e2->transition}),
      {ins.grid.idx(0, 1)});

  ASSERT_TRUE(plan.has_value());
  ASSERT_EQ(plan->waves.size(), 3u);
  const auto& first = plan->waves[0].tasks.at(0);
  const auto& repeated = plan->waves[2].tasks.at(0);
  EXPECT_EQ(first.task.id, repeated.task.id);
  EXPECT_EQ(first.task.transfer.route, repeated.task.transfer.route);
  EXPECT_NE(first.id, repeated.id);
  EXPECT_EQ(first.id, (FrozenTaskId{0, 0}));
  EXPECT_EQ(repeated.id, (FrozenTaskId{2, 0}));
}

TEST(carrier_brd_frozen_tasks,
     anonymous_identity_uses_each_wave_source_and_replays_shadow)
{
  const auto ins = make_instance(
      {"....."}, {"S.SSS"}, {{0, 0}, {0, 4}},
      {{{0, 0}, {0, 2}}});
  const auto s0 = br_labeled_initial_state(ins);
  const auto e0 = validate_complete_upper_action(
      ins, s0,
      {
          wait(target(0)),
          transfer(
              anonymous(0), ins.grid.idx(0, 3),
              {ins.grid.idx(0, 4), ins.grid.idx(0, 3)}),
      });
  ASSERT_TRUE(e0.has_value());
  const auto e1 = validate_complete_upper_action(
      ins, e0->state,
      {
          transfer(
              target(0), ins.grid.idx(0, 2),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1),
               ins.grid.idx(0, 2)}),
          wait(anonymous(0)),
      });
  ASSERT_TRUE(e1.has_value());
  const auto e2 = validate_complete_upper_action(
      ins, e1->state,
      {
          wait(target(0)),
          transfer(
              anonymous(0), ins.grid.idx(0, 4),
              {ins.grid.idx(0, 3), ins.grid.idx(0, 4)}),
      });
  ASSERT_TRUE(e2.has_value());
  const std::vector<int> tau{ins.grid.idx(0, 2)};

  const auto plan = compile_frozen_task_plan(
      ins,
      make_result(
          {s0, e0->state, e1->state, e2->state},
          {e0->transition, e1->transition, e2->transition}),
      tau);

  ASSERT_TRUE(plan.has_value());
  ASSERT_EQ(plan->waves.size(), 3u);
  const auto& anon_first = plan->waves[0].tasks.at(0);
  const auto& target_task = plan->waves[1].tasks.at(0);
  const auto& anon_second = plan->waves[2].tasks.at(0);
  EXPECT_EQ(anon_first.stable_shelf, anonymous(0));
  EXPECT_EQ(
      anon_first.task.id.shelf,
      (ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 4)}));
  EXPECT_EQ(
      anon_second.task.id.shelf,
      (ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 3)}));
  EXPECT_TRUE(anon_first.task.roots.empty());
  EXPECT_TRUE(anon_second.task.roots.empty());
  EXPECT_EQ(anon_first.task.priority, 0);
  EXPECT_EQ(anon_second.task.priority, 0);
  EXPECT_EQ(
      target_task.task.roots,
      (std::vector<RootDemand>{
          RootDemand{0, ins.grid.idx(0, 2)}}));
  EXPECT_EQ(target_task.task.priority, 1);

  BRLabeledUpperState replayed;
  EXPECT_TRUE(replay_frozen_task_plan(
      ins, *plan, tau, &replayed));
  EXPECT_EQ(replayed, e2->state);
  EXPECT_EQ(
      project_labeled_upper_state(replayed),
      (UpperSignature{
          {ins.grid.idx(0, 2)}, {ins.grid.idx(0, 4)}}));
}

TEST(carrier_brd_frozen_tasks,
     compile_rejects_payload_that_does_not_replay_to_recorded_state)
{
  const auto ins = make_instance(
      {"..."}, {"SSS"}, {{0, 0}},
      {{{0, 0}, {0, 2}}});
  const auto start = br_labeled_initial_state(ins);
  BRLabeledUpperState claimed_after = start;
  claimed_after.target_pos[0] = ins.grid.idx(0, 2);
  BRUpperTransition wrong_transition;
  wrong_transition.transfers.push_back(
      AppliedUpperTransfer{
          target(0),
          StorageTransfer{
              ins.grid.idx(0, 1),
              {ins.grid.idx(0, 0), ins.grid.idx(0, 1)}}});

  const auto plan = compile_frozen_task_plan(
      ins,
      make_result(
          {start, claimed_after}, {wrong_transition}),
      {ins.grid.idx(0, 2)});

  EXPECT_FALSE(plan.has_value());
}
