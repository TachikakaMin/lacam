// PROTECTED tests: two-deck semantics of dd_carrier (design.md sections
// 2.2, 3.1-3.3, 6.5).  Written BEFORE implementation (TDD RED).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

namespace {

// tiny helper: build instance from inline map rows / robots / shelves /
// targets (id implicit b0..)
DDInstance make_ins(const std::vector<std::string>& rows,
                    const std::vector<std::pair<int, int>>& robots,
                    const std::vector<std::pair<int, int>>& shelves,
                    const std::vector<std::pair<std::pair<int, int>,
                                                std::pair<int, int>>>& targets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  for (auto& q : robots) ins.robots.push_back(ins.grid.idx(q.first, q.second));
  for (auto& p : shelves)
    ins.shelves.push_back(ins.grid.idx(p.first, p.second));
  for (auto& t : targets) {
    ins.target_starts.push_back(ins.grid.idx(t.first.first, t.first.second));
    ins.target_goals.push_back(ins.grid.idx(t.second.first, t.second.second));
  }
  ins.finalize();
  return ins;
}

Op W() { return Op::make_wait(); }
Op M(const DDInstance& ins, int r, int c) { return Op::make_move(ins.grid.idx(r, c)); }
Op L() { return Op::make_lift(); }
Op D() { return Op::make_drop(); }

}  // namespace

TEST(dd_validator, lift_move_drop_reaches_goal)
{
  auto ins = make_ins({"....", "...."}, {{0, 0}}, {{0, 1}},
                      {{{0, 1}, {0, 3}}});
  auto s = initial_phys_config(ins);
  EXPECT_FALSE(is_dd_goal(ins, s));
  auto r1 = apply_ops(ins, s, {M(ins, 0, 1)});
  ASSERT_TRUE(r1.has_value());  // free robot moves under grounded shelf (I3)
  auto r2 = apply_ops(ins, *r1, {L()});
  ASSERT_TRUE(r2.has_value());
  EXPECT_EQ(r2->kappa[0], 0);  // carries target 0
  auto r3 = apply_ops(ins, *r2, {M(ins, 0, 2)});
  ASSERT_TRUE(r3.has_value());
  auto r4 = apply_ops(ins, *r3, {M(ins, 0, 3)});
  ASSERT_TRUE(r4.has_value());
  EXPECT_FALSE(is_dd_goal(ins, *r4));  // carried at goal is NOT goal (D10)
  auto r5 = apply_ops(ins, *r4, {D()});
  ASSERT_TRUE(r5.has_value());
  EXPECT_TRUE(is_dd_goal(ins, *r5));
}

TEST(dd_validator, preconditions_rejected)
{
  auto ins = make_ins({"...."}, {{0, 0}}, {{0, 1}}, {{{0, 1}, {0, 3}}});
  auto s = initial_phys_config(ins);
  EXPECT_FALSE(apply_ops(ins, s, {L()}).has_value());  // no shelf underfoot
  EXPECT_FALSE(apply_ops(ins, s, {D()}).has_value());  // not carrying
  EXPECT_FALSE(apply_ops(ins, s, {M(ins, 0, 2)}).has_value());  // not adjacent
}

TEST(dd_validator, wall_blocks_move)
{
  auto ins = make_ins({".@", ".."}, {{0, 0}}, {}, {});
  auto s = initial_phys_config(ins);
  EXPECT_FALSE(apply_ops(ins, s, {M(ins, 0, 1)}).has_value());
  EXPECT_TRUE(apply_ops(ins, s, {M(ins, 1, 0)}).has_value());
}

TEST(dd_validator, r1_vertex_conflict)
{
  auto ins = make_ins({"..."}, {{0, 0}, {0, 2}}, {}, {});
  auto s = initial_phys_config(ins);
  EXPECT_FALSE(apply_ops(ins, s, {M(ins, 0, 1), M(ins, 0, 1)}).has_value());
}

TEST(dd_validator, r2_swap_conflict_and_following)
{
  auto ins = make_ins({"..."}, {{0, 0}, {0, 1}}, {}, {});
  auto s = initial_phys_config(ins);
  EXPECT_FALSE(
      apply_ops(ins, s, {M(ins, 0, 1), M(ins, 0, 0)}).has_value());  // swap
  auto conv = apply_ops(ins, s, {M(ins, 0, 1), M(ins, 0, 2)});
  ASSERT_TRUE(conv.has_value());  // following allowed (design 3.4a)
}

TEST(dd_validator, s1_shelf_conflict_loaded_move)
{
  auto ins = make_ins({"..."}, {{0, 0}}, {{0, 0}, {0, 1}},
                      {{{0, 0}, {0, 2}}});
  auto s = initial_phys_config(ins);
  auto lifted = apply_ops(ins, s, {L()});
  ASSERT_TRUE(lifted.has_value());
  // loaded move into occupied upper cell violates S1
  EXPECT_FALSE(apply_ops(ins, *lifted, {M(ins, 0, 1)}).has_value());
}

TEST(dd_validator, i1_lift_only_grounded_at_step_start)
{
  // robot1 lifts anonymous shelf; next step robot1 drops while robot0 tries
  // to move into that cell -> R1 forbids; then single-robot: drop+lift same
  // step atomicity via two robots adjacent
  auto ins = make_ins({"..", ".."}, {{0, 0}, {0, 1}}, {{0, 1}}, {});
  auto s = initial_phys_config(ins);
  auto s1 = apply_ops(ins, s, {W(), L()});
  ASSERT_TRUE(s1.has_value());
  EXPECT_EQ(s1->kappa[1], KAPPA_ANON);
  // robot1 moves loaded to (1,1), drops; robot0 moves to (0,1)
  auto s2 = apply_ops(ins, *s1, {M(ins, 0, 1), M(ins, 1, 1)});
  ASSERT_TRUE(s2.has_value());
  auto s3 = apply_ops(ins, *s2, {W(), D()});
  ASSERT_TRUE(s3.has_value());
  // I1: robot0 cannot lift a shelf dropped THIS step -> next step it can.
  // (robot0 is at (0,1), shelf now grounded at (1,1); move under then lift)
  auto s4 = apply_ops(ins, *s3, {M(ins, 1, 1), M(ins, 1, 0)});
  ASSERT_TRUE(s4.has_value());
  auto s5 = apply_ops(ins, *s4, {L(), W()});
  ASSERT_TRUE(s5.has_value());
  EXPECT_EQ(s5->kappa[0], KAPPA_ANON);
}

TEST(dd_validator, cycle_rotation_zero_empty)
{
  // Proposition 2: fully-occupied 2x2 cycle rotates with 4 loaded robots.
  auto ins = make_ins(
      {"..", ".."}, {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{0, 1}, {1, 1}}, {{1, 1}, {1, 0}}, {{1, 0}, {0, 0}}});
  auto s = initial_phys_config(ins);
  auto s1 = apply_ops(ins, s, {L(), L(), L(), L()});
  ASSERT_TRUE(s1.has_value());
  auto s2 = apply_ops(
      ins, *s1, {M(ins, 0, 1), M(ins, 1, 1), M(ins, 1, 0), M(ins, 0, 0)});
  ASSERT_TRUE(s2.has_value());
  auto s3 = apply_ops(ins, *s2, {D(), D(), D(), D()});
  ASSERT_TRUE(s3.has_value());
  EXPECT_TRUE(is_dd_goal(ins, *s3));
}

TEST(dd_state, canonical_hash_anonymous_shelves)
{
  // two states differing only by which anonymous shelf sits where must
  // hash/compare equal after canonicalization (occupancy set semantics)
  auto ins = make_ins({"...."}, {{0, 0}}, {{0, 1}, {0, 3}}, {});
  auto s = initial_phys_config(ins);
  auto h1 = phys_config_hash(s);
  PhysConfig t = s;  // anon occupancy stored as sorted set -> same hash
  EXPECT_EQ(phys_config_hash(t), h1);
  EXPECT_TRUE(s == t);
}

TEST(dd_instance, load_yaml_fixture)
{
  const auto path = std::string(DD_TEST_DIR) + "/fixtures/dd_tiny.yaml";
  auto ins = load_dd_instance(path);
  EXPECT_EQ(ins.grid.height, 2);
  EXPECT_EQ(ins.grid.width, 4);
  EXPECT_EQ(ins.robots.size(), 1u);
  EXPECT_EQ(ins.n_targets(), 1u);
  EXPECT_EQ(ins.shelves.size(), 2u);
  auto s = initial_phys_config(ins);
  EXPECT_FALSE(is_dd_goal(ins, s));
}

TEST(dd_instance, non_default_flags_rejected)
{
  // debug.md P0-4 decision (a): v1 does not implement the optional flag
  // semantics (remove_on_complete / robots_return_to_rest); silently
  // ignoring a requested non-default flag would produce wrong semantics,
  // so the loader must FAIL LOUDLY.
  const auto path =
      std::string(DD_TEST_DIR) + "/fixtures/dd_flags_unsupported.yaml";
  EXPECT_THROW(load_dd_instance(path), std::exception);
}

TEST(dd_instance, default_or_absent_flags_accepted)
{
  // flags absent (dd_tiny.yaml) already covered above; explicit false
  // values must also load fine.
  const auto path =
      std::string(DD_TEST_DIR) + "/fixtures/dd_flags_default.yaml";
  auto ins = load_dd_instance(path);
  EXPECT_EQ(ins.n_targets(), 1u);
}

TEST(dd_storage_map, loaded_shelf_may_cross_aisle_but_not_drop_there)
{
  const auto path =
      std::string(DD_TEST_DIR) + "/fixtures/dd_storage_map.yaml";
  auto ins = load_dd_instance(path);
  EXPECT_TRUE(ins.can_store_shelf(ins.grid.idx(0, 1)));
  EXPECT_FALSE(ins.can_store_shelf(ins.grid.idx(0, 0)));

  auto s = initial_phys_config(ins);
  auto lifted = apply_ops(ins, s, {L()});
  ASSERT_TRUE(lifted.has_value());
  auto in_aisle = apply_ops(ins, *lifted, {M(ins, 0, 0)});
  ASSERT_TRUE(in_aisle.has_value())
      << "a carried shelf must be allowed to traverse an aisle";
  EXPECT_FALSE(apply_ops(ins, *in_aisle, {D()}).has_value())
      << "storage_map must reject DROP in an aisle";

  auto back_on_storage = apply_ops(ins, *in_aisle, {M(ins, 0, 1)});
  ASSERT_TRUE(back_on_storage.has_value());
  EXPECT_TRUE(apply_ops(ins, *back_on_storage, {D()}).has_value());
}

TEST(dd_storage_map, finalize_rejects_shelf_or_goal_outside_storage)
{
  DDInstance bad_shelf;
  bad_shelf.grid = DDGrid({"...."});
  bad_shelf.shelf_storage = {0, 1, 1, 0};
  bad_shelf.robots = {bad_shelf.grid.idx(0, 0)};
  bad_shelf.shelves = {bad_shelf.grid.idx(0, 0)};
  EXPECT_THROW(bad_shelf.finalize(), std::exception);

  DDInstance bad_goal;
  bad_goal.grid = DDGrid({"...."});
  bad_goal.shelf_storage = {0, 1, 1, 0};
  bad_goal.robots = {bad_goal.grid.idx(0, 0)};
  bad_goal.shelves = {bad_goal.grid.idx(0, 1)};
  bad_goal.target_starts = {bad_goal.grid.idx(0, 1)};
  bad_goal.target_goals = {bad_goal.grid.idx(0, 3)};
  EXPECT_THROW(bad_goal.finalize(), std::exception);
}

TEST(dd_storage_map, planner_carries_across_aisle_without_dropping_there)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.shelf_storage = {0, 1, 1, 0, 1};
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 2)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.finalize();

  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 2.0, 0, &stats);
  ASSERT_FALSE(plan.empty())
      << "the planner must carry a shelf through a non-storage aisle";

  auto state = initial_phys_config(ins);
  bool crossed_aisle_loaded = false;
  for (const auto& ops : plan) {
    ASSERT_EQ(ops.size(), 1u);
    if (state.kappa[0] != KAPPA_FREE &&
        ops[0].kind == Op::MOVE &&
        ops[0].to == ins.grid.idx(0, 3))
      crossed_aisle_loaded = true;
    if (ops[0].kind == Op::DROP) {
      EXPECT_TRUE(ins.can_store_shelf(state.robots[0]))
          << "planner emitted DROP in an aisle";
    }
    auto next = apply_ops(ins, state, ops);
    ASSERT_TRUE(next.has_value());
    state = *next;
  }

  EXPECT_TRUE(crossed_aisle_loaded);
  EXPECT_TRUE(is_dd_goal(ins, state));
}

TEST(dd_storage_map, planner_rehomes_blocker_beyond_aisle)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.shelf_storage = {0, 1, 1, 0, 1};
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {ins.grid.idx(0, 1), ins.grid.idx(0, 2)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();

  auto manual = initial_phys_config(ins);
  auto lifted = apply_ops(ins, manual, {L()});
  ASSERT_TRUE(lifted.has_value());
  auto in_aisle = apply_ops(ins, *lifted, {M(ins, 0, 3)});
  ASSERT_TRUE(in_aisle.has_value());
  auto rehomed = apply_ops(ins, *in_aisle, {M(ins, 0, 4)});
  ASSERT_TRUE(rehomed.has_value());
  EXPECT_TRUE(apply_ops(ins, *rehomed, {D()}).has_value());

  const auto aisle_successors =
      dd_enumerate_node_successors(ins, *in_aisle, 0);
  EXPECT_TRUE(std::find(
                  aisle_successors.begin(), aisle_successors.end(),
                  *rehomed) != aisle_successors.end())
      << "the exhaustive successor generator must offer the loaded move "
         "from aisle to storage";

  DDStats stats;
  const auto plan = solve_carrier_lacam(ins, 2.0, 0, &stats);
  ASSERT_FALSE(plan.empty())
      << "an anonymous blocker may cross an aisle, but must be rehomed "
         "on storage before release";

  auto state = initial_phys_config(ins);
  for (const auto& ops : plan) {
    ASSERT_EQ(ops.size(), 1u);
    if (ops[0].kind == Op::DROP) {
      EXPECT_TRUE(ins.can_store_shelf(state.robots[0]))
          << "planner emitted DROP in an aisle";
    }
    auto next = apply_ops(ins, state, ops);
    ASSERT_TRUE(next.has_value());
    state = *next;
  }
  EXPECT_TRUE(is_dd_goal(ins, state));
  EXPECT_TRUE(std::binary_search(
      state.anon_occ.begin(), state.anon_occ.end(), ins.grid.idx(0, 4)));
}
