// debug.md round-2 P0-2: park purity w.r.t. path-cache history.
//
// design.md 5.4a (v2.3): the park relation is a deterministic function of
// (X, D_b cache epoch) — NOT of X alone, because the asymmetric lazy
// invalidation (vacated cells keep the stale path) lets two different
// histories reach the same X with different cached least-blocking paths.
// Contract fixed by these tests:
//   1. determinism: identical (X, cache history) -> identical park set;
//   2. strict-invalidation probe mode is epoch-free:
//      warmed and fresh histories MUST agree;
//   3. characterization: under the default lazy policy the adversarial
//      occupied->vacated history CAN change the park set (known, documented
//      impurity — ordering-only, feasibility unaffected).  If this ever
//      starts failing because the default became epoch-free, upgrade
//      design.md 5.4a instead of deleting the assertion.
//
// Fixture (subagent-APPROVED rev 2): 4-row map with a wall band on row 1.
// o = target 0 runs (0,0)->(0,4) straight along row 0 (static distance 4,
// the SMALLEST, so o is sorted first and claims its path cells before b's
// self-goal claim — owner marking is first-come-first-served).  b's goal
// (0,2) sits on o's fresh path -> park[b]=1.  Warming the cache with (0,2)
// occupied forces o onto the detour row-2/3 corridor (occupied-cell
// penalty LAMBDA_BLK); after vacating, the lazy policy keeps the detour
// (all ITS cells unchanged) and b is NOT parked.  Note: b's static
// distance (3,2)->(0,2) is 7 around the wall band, strictly greater than
// o's 4.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_ins()
{
  DDInstance ins;
  ins.grid = DDGrid({".....", ".###.", ".....", "....."});
  ins.robots.push_back(ins.grid.idx(1, 0));
  ins.robots.push_back(ins.grid.idx(3, 4));
  // o = target 0: (0,0) -> (0,4), fresh path straight along row 0
  ins.target_starts.push_back(ins.grid.idx(0, 0));
  ins.target_goals.push_back(ins.grid.idx(0, 4));
  ins.shelves.push_back(ins.grid.idx(0, 0));
  // b = target 1: goal ON o's row-0 path at (0,2); start on row 3
  ins.target_starts.push_back(ins.grid.idx(3, 2));
  ins.target_goals.push_back(ins.grid.idx(0, 2));
  ins.shelves.push_back(ins.grid.idx(3, 2));
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_park_purity, deterministic_for_identical_history)
{
  auto ins = make_ins();
  const auto X = initial_phys_config(ins);
  for (bool strict : {false, true}) {
    const auto a = dd_compute_park(ins, X, /*warm_block_cell=*/-1, strict);
    const auto b = dd_compute_park(ins, X, /*warm_block_cell=*/-1, strict);
    EXPECT_EQ(a, b) << "park not deterministic (strict=" << strict << ")";
  }
}

TEST(dd_park_purity, fixture_actually_parks_on_fresh_path)
{
  auto ins = make_ins();
  const auto X = initial_phys_config(ins);
  std::vector<int> path;
  const auto park = dd_compute_park(ins, X, -1, false, &path);
  ASSERT_EQ(park.size(), 2u);
  // fresh least-blocking path for o runs through row 0 (anon shelf
  // penalizes row 2), hence crosses g_b and parks b.
  EXPECT_EQ(park[1], 1) << "fixture must trigger the park relation";
  bool crosses = false;
  for (int c : path) crosses |= (c == ins.grid.idx(0, 2));
  EXPECT_TRUE(crosses) << "fresh path expected through (0,2)";
}

TEST(dd_park_purity, strict_invalidation_is_epoch_free)
{
  auto ins = make_ins();
  const auto X = initial_phys_config(ins);
  const int block = ins.grid.idx(0, 2);
  const auto fresh = dd_compute_park(ins, X, -1, /*strict=*/true);
  const auto warmed = dd_compute_park(ins, X, block, /*strict=*/true);
  EXPECT_EQ(fresh, warmed)
      << "strict invalidation must make park independent of cache history";
}

TEST(dd_park_purity, default_lazy_policy_is_epoch_dependent_documented)
{
  auto ins = make_ins();
  const auto X = initial_phys_config(ins);
  const int block = ins.grid.idx(0, 2);
  const auto fresh = dd_compute_park(ins, X, -1, /*strict=*/false);
  const auto warmed = dd_compute_park(ins, X, block, /*strict=*/false);
  ASSERT_EQ(fresh.size(), 2u);
  ASSERT_EQ(warmed.size(), 2u);
  EXPECT_EQ(fresh[1], 1);
  EXPECT_EQ(warmed[1], 0)
      << "characterization: occupied->vacated history keeps the stale "
         "row-2 path and skips the park.  If the default policy became "
         "epoch-free, upgrade design.md 5.4a (subagent review) rather "
         "than weakening this test.";
}
