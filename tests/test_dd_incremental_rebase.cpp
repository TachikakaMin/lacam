// PROTECTED TEST: dependency-safe PairCost reuse after an external dynamic
// state jump, integration plan Phase 6. Written before implementation (RED).
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <numeric>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance make_dense_rebase_case()
{
  DDInstance ins;
  ins.grid = DDGrid({"........"});
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.robots = {6, 7};
  ins.shelves = {0, 1, 2, 3};
  ins.target_starts = {0, 1};
  const std::vector<int> goals{0, 2, 4};
  ins.target_goal_sets.assign(2, goals);
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_incremental_rebase,
     one_moved_anonymous_shelf_repairs_only_affected_pair_edges)
{
  const DDInstance ins = make_dense_rebase_case();
  const PhysConfig initial = initial_phys_config(ins);
  DDPlanningSession session(ins, initial, 0);
  ASSERT_TRUE(session.solve(2.0).solved());

  PhysConfig observed = initial;
  observed.anon_occ = {3, 5};
  ASSERT_TRUE(validate_phys_config_root(ins, observed).valid());
  ASSERT_EQ(
      session.rebase(observed),
      DDRebaseStatus::OK);

  DDStats stats;
  ASSERT_TRUE(session.solve(2.0, &stats).solved());
  EXPECT_GT(stats.root_pair_edges_total, 0);
  EXPECT_GT(stats.root_pair_edges_reused, 0);
  EXPECT_GT(stats.root_pair_edges_evaluated, 0);
  EXPECT_LT(
      stats.root_pair_edges_evaluated,
      stats.root_pair_edges_total);
  EXPECT_EQ(
      stats.root_pair_edges_evaluated +
          stats.root_pair_edges_reused,
      stats.root_pair_edges_total);
}

TEST(dd_incremental_rebase,
     invalid_dynamic_root_is_rejected_without_losing_the_old_root)
{
  const DDInstance ins = make_dense_rebase_case();
  const PhysConfig initial = initial_phys_config(ins);
  DDPlanningSession session(ins, initial, 0);
  ASSERT_TRUE(session.solve(2.0).solved());

  PhysConfig invalid = initial;
  invalid.robots[0] = invalid.robots[1];
  EXPECT_EQ(
      session.rebase(invalid),
      DDRebaseStatus::INVALID_STATE);
  EXPECT_TRUE(session.solve(2.0).solved());
}
