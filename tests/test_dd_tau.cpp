// PROTECTED tests: tau layer (design_final 5.3, debug.md v4 WP-B/C/D).
// WP-B (T6): the upper-wall distance field depends only on (walls, dest)
// — a single shared dest-keyed cache must return exactly the same fields
// as per-target caches (the property the T6 merge relies on).
#include <dd_carrier.hpp>
#include <dd_dist_adapters.hpp>

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
