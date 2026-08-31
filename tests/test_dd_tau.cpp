// PROTECTED tests: tau layer (design_final 5.3, debug.md v4 WP-B/C/D).
// WP-B (T6): the upper-wall distance field depends only on (walls, dest)
// — a single shared dest-keyed cache must return exactly the same fields
// as per-target caches (the property the T6 merge relies on).
#include <dd_carrier.hpp>
#include <dd_dist_adapters.hpp>
#include <dd_planner.hpp>

#include "gtest/gtest.h"

TEST(dd_tau_cache, shared_dest_keyed_cache_matches_fresh_caches)
{
  const DDGrid g({".#...", ".#.#.", ".....", "..#.."});
  DDDistCache shared(g);
  const std::vector<int> dests = {g.idx(0, 0), g.idx(0, 4), g.idx(2, 2),
                                  g.idx(3, 4)};
  // interleave queries on the shared cache, then compare against fresh
  // per-dest caches (no cross-dest contamination, identical values)
  for (const int d : dests) shared.to(d);
  for (const int d : dests) {
    DDDistCache fresh(g);
    EXPECT_EQ(shared.to(d), fresh.to(d)) << "dest " << d;
  }
  // spot-check a couple of exact wall-aware values
  EXPECT_EQ(shared.to(g.idx(0, 0))[g.idx(0, 2)], 6);  // around the wall
  EXPECT_EQ(shared.to(g.idx(2, 2))[g.idx(2, 2)], 0);
}

// ---- WP-C (T3): production tau matching (design_final 5.3/D17) ----

namespace {

DDInstance tau_ins(const std::vector<std::string>& rows,
                   const std::vector<std::pair<int, int>>& robots,
                   const std::vector<std::pair<int, int>>& starts,
                   const std::vector<std::vector<std::pair<int, int>>>& sets)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  for (auto& q : robots) ins.robots.push_back(ins.grid.idx(q.first, q.second));
  for (size_t b = 0; b < starts.size(); ++b) {
    ins.shelves.push_back(ins.grid.idx(starts[b].first, starts[b].second));
    ins.target_starts.push_back(
        ins.grid.idx(starts[b].first, starts[b].second));
    std::vector<int> set;
    for (auto& g : sets[b]) set.push_back(ins.grid.idx(g.first, g.second));
    ins.target_goal_sets.push_back(set);
    ins.target_goals.push_back(set.front());
  }
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_tau, singleton_degenerates_and_matches_root_h)
{
  // fixed-goal instance: tau must be the identity view and h_shelf must
  // equal the production root admissible h EXACTLY (same arithmetic)
  auto ins = tau_ins({".....", ".....", "....."}, {{2, 0}},
                     {{0, 1}, {1, 1}},
                     {{{0, 4}}, {{2, 4}}});
  const auto X = initial_phys_config(ins);
  double h = -1;
  const auto tau = dd_solve_tau(ins, X, nullptr, &h);
  ASSERT_EQ(tau.size(), 2u);
  EXPECT_EQ(tau[0], ins.target_goals[0]);
  EXPECT_EQ(tau[1], ins.target_goals[1]);
  EXPECT_DOUBLE_EQ(h, dd_root_admissible_h(ins));
}

TEST(dd_tau, injective_matching_resolves_contention)
{
  // T1 is eligible ONLY for P; T0 is eligible for {P, Q}.  tau must give
  // P to T1 and Q to T0 (row-wise greedy would hand P to T0 first).
  auto ins = tau_ins({".....", ".....", "....."}, {{2, 0}},
                     {{0, 0}, {0, 2}},
                     {{{0, 1}, {2, 4}}, {{0, 1}}});
  const int P = ins.grid.idx(0, 1), Q = ins.grid.idx(2, 4);
  const auto X = initial_phys_config(ins);
  const auto tau = dd_solve_tau(ins, X);
  ASSERT_EQ(tau.size(), 2u);
  EXPECT_EQ(tau[0], Q);
  EXPECT_EQ(tau[1], P);
}

TEST(dd_tau, h_equals_bruteforce_min_matching)
{
  // 3 targets over a 4-cell shared pool with walls: h_shelf must equal
  // the brute-force min over all injective assignments of
  // d_wall(p_b, g) + 2*gamma (all grounded off-goal, unit weights)
  const std::vector<std::string> rows = {"......", "..#...", "......"};
  auto ins = tau_ins(
      rows, {{2, 0}}, {{0, 0}, {0, 1}, {1, 1}},
      {{{0, 5}, {1, 5}, {2, 5}, {2, 3}},
       {{0, 5}, {1, 5}, {2, 5}, {2, 3}},
       {{0, 5}, {1, 5}, {2, 5}, {2, 3}}});
  const auto X = initial_phys_config(ins);
  double h = -1;
  const auto tau = dd_solve_tau(ins, X, nullptr, &h);
  ASSERT_EQ(tau.size(), 3u);
  // brute force over the 4 pool cells
  DDDistCache dc(ins.grid);
  const std::vector<int> pool = ins.target_goal_sets[0];
  double best = 1e18;
  for (size_t a = 0; a < pool.size(); ++a)
    for (size_t b = 0; b < pool.size(); ++b)
      for (size_t c = 0; c < pool.size(); ++c) {
        if (a == b || b == c || a == c) continue;
        const double v = dc.to(pool[a])[X.target_pos[0]] +
                         dc.to(pool[b])[X.target_pos[1]] +
                         dc.to(pool[c])[X.target_pos[2]] + 3 * 2;
        best = std::min(best, v);
      }
  EXPECT_DOUBLE_EQ(h, best);
  // and tau itself must realize an injective assignment of that cost
  double got = 3 * 2;
  got += dc.to(tau[0])[X.target_pos[0]];
  got += dc.to(tau[1])[X.target_pos[1]];
  got += dc.to(tau[2])[X.target_pos[2]];
  EXPECT_DOUBLE_EQ(got, best);
  EXPECT_TRUE(tau[0] != tau[1] && tau[1] != tau[2] && tau[0] != tau[2]);
}

TEST(dd_tau, hysteresis_is_tie_break_only)
{
  // equal-LB pool cells: parent pull decides; with an LB gap the primary
  // order must win regardless of the parent pull (design_final D17/D19)
  auto ins_eq = tau_ins({"...", "...", "..."}, {{1, 0}}, {{1, 1}},
                        {{{0, 2}, {2, 2}}});  // both d=2 from (1,1)
  const int A = ins_eq.grid.idx(0, 2), B = ins_eq.grid.idx(2, 2);
  const auto X_eq = initial_phys_config(ins_eq);
  const std::vector<int> pullB = {B};
  const auto tau_pull = dd_solve_tau(ins_eq, X_eq, &pullB);
  EXPECT_EQ(tau_pull[0], B);  // tie -> hysteresis follows the parent
  const std::vector<int> pullA = {A};
  const auto tau_pullA = dd_solve_tau(ins_eq, X_eq, &pullA);
  EXPECT_EQ(tau_pullA[0], A);

  auto ins_gap = tau_ins({"....", "....", "...."}, {{1, 0}}, {{1, 1}},
                         {{{1, 2}, {2, 3}}});  // d=1 vs d=3
  const int NEAR = ins_gap.grid.idx(1, 2), FAR = ins_gap.grid.idx(2, 3);
  const auto X_gap = initial_phys_config(ins_gap);
  const std::vector<int> pullFar = {FAR};
  const auto tau_gap = dd_solve_tau(ins_gap, X_gap, &pullFar);
  EXPECT_EQ(tau_gap[0], NEAR);  // primary LB order beats the pull
}

TEST(dd_tau, oplb_branches_carried_and_done)
{
  auto ins = tau_ins({".....", "....."}, {{0, 0}}, {{0, 1}, {0, 2}},
                     {{{1, 1}}, {{0, 2}, {1, 4}}});
  auto X = initial_phys_config(ins);
  // T1 grounded ON an eligible cell -> contributes 0; T0 grounded off
  // its goal (d=1) -> 1 + 2*gamma = 3
  double h = -1;
  dd_solve_tau(ins, X, nullptr, &h);
  EXPECT_DOUBLE_EQ(h, 3.0);
  // now T0 carried at its start (robot under it): opLB drops to 1 gamma
  X.robots = {ins.grid.idx(0, 1)};
  X.kappa = {0};
  double h2 = -1;
  dd_solve_tau(ins, X, nullptr, &h2);
  EXPECT_DOUBLE_EQ(h2, 2.0);  // d=1 + 1*gamma
}

TEST(dd_tau, taboo_excludes_pair_but_singletons_exempt)
{
  auto ins = tau_ins({".....", "....."}, {{1, 0}}, {{0, 0}, {0, 1}},
                     {{{0, 3}, {0, 4}}, {{1, 1}}});
  const auto X = initial_phys_config(ins);
  const auto tau0 = dd_solve_tau(ins, X);
  EXPECT_EQ(tau0[0], ins.grid.idx(0, 3));  // nearest
  // taboo (T0 -> (0,3)) diverts T0 to (0,4)
  const std::vector<std::pair<int, int>> tb = {{0, ins.grid.idx(0, 3)}};
  const auto tau1 = dd_solve_tau(ins, X, nullptr, nullptr, &tb);
  EXPECT_EQ(tau1[0], ins.grid.idx(0, 4));
  // taboo on a singleton row is exempt (would make the row infeasible)
  const std::vector<std::pair<int, int>> tb2 = {{1, ins.grid.idx(1, 1)}};
  const auto tau2 = dd_solve_tau(ins, X, nullptr, nullptr, &tb2);
  EXPECT_EQ(tau2[1], ins.grid.idx(1, 1));
}

// ---- WP-C (T5): PathCache dst invalidation ----

TEST(dd_tau_cache, pathcache_dst_change_is_a_miss)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", ".....", "....."});
  ins.robots.push_back(ins.grid.idx(2, 0));
  ins.shelves.push_back(ins.grid.idx(0, 0));
  ins.target_starts.push_back(ins.grid.idx(0, 0));
  ins.target_goal_sets.push_back(
      {ins.grid.idx(0, 4), ins.grid.idx(2, 4)});
  ins.target_goals.push_back(ins.grid.idx(0, 4));
  ins.finalize();
  const auto X = initial_phys_config(ins);
  long recomputes = -1;
  const auto path = dd_pathcache_dst_probe(
      ins, X, 0, ins.grid.idx(0, 4), ins.grid.idx(2, 4), &recomputes);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.back(), ins.grid.idx(2, 4));  // stale dst1 path is a bug
  EXPECT_EQ(recomputes, 2);                    // dst change == cache miss
}

// ---- WP-D (T4/T7): guidance consumers driven by tau ----

TEST(dd_tau_guidance, pool_instance_delivers_to_near_goals)
{
  // shared 3-cell pool; the sorted-first representative (0,0) is FAR
  // from both targets, the two other pool cells are adjacent to them.
  // tau-driven guidance must deliver to the near cells; representative-
  // driven guidance would push both toward (0,0) (contention + detour).
  auto ins = tau_ins(
      {"......", "......", "......"}, {{2, 0}},
      {{1, 2}, {1, 3}},
      {{{0, 0}, {1, 4}, {2, 3}}, {{0, 0}, {1, 4}, {2, 3}}});
  DDStats st;
  const auto plan = solve_carrier_lacam(ins, 5.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  auto s = initial_phys_config(ins);
  for (const auto& ops : plan) {
    auto nxt = apply_ops(ins, s, ops);
    ASSERT_TRUE(nxt.has_value());
    s = *nxt;
  }
  EXPECT_TRUE(is_dd_goal(ins, s));
  const int FARC = ins.grid.idx(0, 0);
  EXPECT_NE(s.target_pos[0], FARC);  // nobody hauled to the far cell
  EXPECT_NE(s.target_pos[1], FARC);
  EXPECT_LE(plan.size(), 14u);  // tau-direct horizon (near cells only)
}

TEST(dd_tau_guidance, frontier_movable_clear_outranks_chain_head)
{
  // 1-row corridor, single empty at (0,3): the only executable
  // manipulation is the blocker ADJACENT to the vacancy (c2), not the
  // chain head next to the target (c1).  With DD_CLEAR_FRONTIER=1 the
  // lone free robot must be matched to c2 (design_final 5.4/D20); the
  // default (0, singleton-parity gate) keeps the head-first order.
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots.push_back(ins.grid.idx(0, 4));
  for (int c : {0, 1, 2}) ins.shelves.push_back(ins.grid.idx(0, c));
  ins.target_starts.push_back(ins.grid.idx(0, 0));
  ins.target_goals.push_back(ins.grid.idx(0, 4));
  ins.finalize();
  const auto X = initial_phys_config(ins);
  const auto fg_default = dd_match_free_goals(ins, X, nullptr);
  ASSERT_EQ(fg_default.size(), 1u);
  EXPECT_EQ(fg_default[0], ins.grid.idx(0, 1));  // head-first (default)
  setenv("DD_CLEAR_FRONTIER", "1", 1);
  const auto fg = dd_match_free_goals(ins, X, nullptr);
  unsetenv("DD_CLEAR_FRONTIER");
  ASSERT_EQ(fg.size(), 1u);
  EXPECT_EQ(fg[0], ins.grid.idx(0, 2));  // movable frontier blocker c2
}
