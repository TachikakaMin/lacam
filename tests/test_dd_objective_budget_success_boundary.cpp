// PROTECTED regression: a successful final Objective-PIBT budget attempt.
// Added to close the independent-review coverage finding, 2026-09-02.
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

ManipulationTask committed_claim(int shelf, int from, int claim, int root)
{
  ManipulationTask t;
  t.shelf_target = shelf;
  t.from = from;
  t.to = claim;
  t.to_committed = true;
  t.root_target = root;
  t.root_goal = 2000 + root;
  t.roots = {DemandKey{root, t.root_goal}};
  return t;
}

DDObjectiveOptionProbe option(int root, double score,
                              std::vector<ManipulationTask> claims)
{
  DDObjectiveOptionProbe out;
  out.root_target = root;
  out.root_goal = 2000 + root;
  out.score = score;
  out.chain = std::move(claims);
  return out;
}

}  // namespace

TEST(dd_objective_budget_success_boundary,
     successful_sixteenth_attempt_is_committed)
{
  DDObjectiveResolveProbeInput in;
  in.options.resize(2);

  // The first three high-priority options claim 10, 11, and 12, so every
  // lower option conflicts.  The fourth claims only 10 and 11; therefore the
  // lower option claiming 12 succeeds on global attempt 16:
  // 3 * (1 high + 3 low) + 1 high + 3 low.
  for (int i = 0; i < 3; ++i)
    in.options[0].push_back(option(
        0, i,
        {committed_claim(0, 100 + 3 * i, 10, 0),
         committed_claim(0, 101 + 3 * i, 11, 0),
         committed_claim(0, 102 + 3 * i, 12, 0)}));
  in.options[0].push_back(
      option(0, 3,
             {committed_claim(0, 109, 10, 0),
              committed_claim(0, 110, 11, 0)}));
  for (int i = 0; i < 3; ++i)
    in.options[1].push_back(
        option(1, i, {committed_claim(1, 200 + i, 10 + i, 1)}));

  in.tentative = {0, 0};
  in.effective_priority = {100, 10};
  in.depth_cap = 4;
  in.reselect_cap = 16;

  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.selected, (std::vector<int>{3, 2}));
  EXPECT_EQ(out.yielded, (std::vector<uint8_t>{0, 0}));
  EXPECT_EQ(out.obj_default_resolutions, 0);
  EXPECT_EQ(out.obj_yields, 0);
  EXPECT_EQ(out.obj_reselect_requests, 4);
  EXPECT_EQ(out.obj_backtracks, 3);
  EXPECT_EQ(out.tasks.size(), 3u);
}
