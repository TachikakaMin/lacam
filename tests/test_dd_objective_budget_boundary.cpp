// PROTECTED regression: exact Objective-PIBT reselect-budget exhaustion.
// Written before the fix (TDD RED), 2026-09-02.
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
  t.root_goal = 1000 + root;
  t.roots = {DemandKey{root, t.root_goal}};
  return t;
}

DDObjectiveOptionProbe option(int root, double score,
                              ManipulationTask claim)
{
  DDObjectiveOptionProbe out;
  out.root_target = root;
  out.root_goal = 1000 + root;
  out.score = score;
  out.chain = {std::move(claim)};
  return out;
}

}  // namespace

TEST(dd_objective_budget_boundary,
     exact_reselect_cap_restores_all_unresolved_tentatives)
{
  DDObjectiveResolveProbeInput in;
  for (int i = 0; i < 4; ++i)
    in.options.resize(2),
        in.options[0].push_back(
            option(0, i, committed_claim(0, 100 + i, 10, 0)));
  for (int i = 0; i < 3; ++i)
    in.options[1].push_back(
        option(1, i, committed_claim(1, 200 + i, 10, 1)));

  in.tentative = {0, 0};
  in.effective_priority = {100, 10};
  in.depth_cap = 4;
  in.reselect_cap = 16;  // 4 high options * (1 + 3 low options)

  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.selected, in.tentative);
  EXPECT_EQ(out.yielded, (std::vector<uint8_t>{0, 0}));
  EXPECT_EQ(out.obj_yields, 0);
  EXPECT_EQ(out.obj_default_resolutions, 2);
  EXPECT_EQ(out.tasks.size(), 2u)
      << "budget exhaustion keeps the same compiler's tentative packages";
}
