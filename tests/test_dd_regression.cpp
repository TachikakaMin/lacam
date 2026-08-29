// PROTECTED regression tests for Carrier-LaCAM planner bugs found during
// dev-case runs (2026-08-29).  Written BEFORE the fixes (TDD RED).
//
// Bug A (PIBT lazy inheritance): the generator fixed a pusher onto an
// occupied cell without immediately recursing on the occupant; when the
// occupant later had no feasible op the WHOLE joint-op generation failed,
// wasting ~99% of high-level expansions on large instances
// (r2rM: 40k expansions -> 289 nodes in 5 s).
//
// Bug B (duplicate handling): reaching an EXPLORED configuration discarded
// the successor without re-pushing the existing node, losing LaCAM's
// revisit semantics and stalling dense instances
// (ddmapd d50: 24k duplicate hits, livelock).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

namespace {

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

::testing::AssertionResult plan_is_valid(const DDInstance& ins,
                                         const DDPlan& plan)
{
  if (plan.empty()) return ::testing::AssertionFailure() << "empty plan";
  auto s = initial_phys_config(ins);
  for (size_t t = 0; t < plan.size(); ++t) {
    auto nxt = apply_ops(ins, s, plan[t]);
    if (!nxt.has_value())
      return ::testing::AssertionFailure() << "illegal joint op at t=" << t;
    s = *nxt;
  }
  if (!is_dd_goal(ins, s))
    return ::testing::AssertionFailure() << "final state not goal";
  return ::testing::AssertionSuccess();
}

}  // namespace

TEST(dd_regression, pibt_pusher_backtracks_when_occupant_stuck)
{
  // Bug A micro-instance: loaded carrier's best move enters a dead-end cell
  // occupied by an idle robot whose only escape would be a swap.  Correct
  // PIBT backtracks to the carrier's next candidate instead of failing the
  // whole joint op.  The instance is solvable (carrier detours below).
  //   map:  . . .        robots: C at (0,0) carrying target from (0,0),
  //         . @ .                I at (0,1) idle (dead-end w.r.t. C's push)
  //   target: (0,0) -> (0,2)
  auto ins = make_ins({"...", ".@."}, {{0, 0}, {0, 1}},
                      {{0, 0}}, {{{0, 0}, {0, 2}}});
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
  // with immediate-recursion PIBT the unconstrained generator call must
  // never fail outright on this instance
  EXPECT_EQ(st.generator_failures, 0) << "whole-PIBT failures indicate lazy "
                                         "inheritance (Bug A)";
}

TEST(dd_regression, generator_throughput_dense_blocks)
{
  // Bug A at scale (shrunk): 12x12, 2x2 blocks at 30%, 6 robots.  With the
  // lazy-inheritance bug the node rate collapses (hundreds of failures per
  // node); fixed PIBT keeps failures well below expansions.
  auto ins = make_ins(
      {"............", "............", "..@@........", "..@@........",
       "............", "............", "............", "............",
       "............", "............", "............", "............"},
      {{0, 0}, {0, 11}, {11, 0}, {11, 11}, {5, 5}, {6, 6}},
      {{2, 6}, {2, 7}, {3, 6}, {3, 7}, {8, 2}, {8, 3}, {9, 2}, {9, 3}},
      {{{2, 6}, {10, 10}}, {{8, 2}, {0, 5}}});
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
  EXPECT_LT(st.generator_failures, st.hl_expanded / 2)
      << "generator failing on most expansions (Bug A)";
}

TEST(dd_regression, carried_shelf_positions_block_s1_in_generator)
{
  // Bug C: PIBT bookkeeping only pre-registered GROUNDED shelves as
  // upper-deck occupancy at t+1; a loaded robot could claim the cell under
  // another (not-yet-fixed) carrier's shelf, whose later WAIT then violated
  // S1 and the validator rejected the whole proposal (74% reject rate on
  // ddmapd d50).  Head-on carriers must resolve with ZERO validator rejects.
  auto ins = make_ins({"....", "...."}, {{0, 1}, {0, 2}},
                      {{0, 1}, {0, 2}},
                      {{{0, 1}, {0, 3}}, {{0, 2}, {0, 0}}});
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_TRUE(plan_is_valid(ins, plan));
  EXPECT_EQ(st.validator_rejects, 0)
      << "generator proposed S1-violating joint ops (Bug C)";
}

TEST(dd_regression, dev_case_ddmapd_16x16_d50_within_10s)
{
  // Protected dev case (dev_cases.txt line 6) — currently times out due to
  // Bug A + Bug B; must solve within the 10 s budget after the fixes.
  const auto path = std::string(DD_TEST_DIR) +
                    "/../benchmark/instances_standard/ddmapd/"
                    "ddmapd_h16w16r8d50t8_seed0.yaml";
  auto ins = load_dd_instance(path);
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 10.0, 0, &st);
  ASSERT_FALSE(plan.empty()) << "timed_out=" << st.timed_out
                             << " nodes=" << st.hl_nodes
                             << " dup=" << st.duplicate_configs;
  EXPECT_TRUE(plan_is_valid(ins, plan));
}
