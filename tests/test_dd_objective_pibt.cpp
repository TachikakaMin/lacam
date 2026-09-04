// PROTECTED tests: Carrier-LaCAM v4.1 Objective-PIBT.
// Written before implementation (TDD RED), 2026-09-02.
//
// These tests exercise stable protocol semantics through test-support
// probes backed by the same resolver/merge/progress functions used by
// production build_guidance().  They deliberately avoid pinning recursion
// layout or container choices.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <algorithm>
#include <set>
#include <vector>

#include "gtest/gtest.h"

namespace {

ManipulationTask task(int shelf, int from, int to, bool committed, int root,
                      int goal, int task_priority = 0)
{
  ManipulationTask t;
  t.shelf_target = shelf;
  t.from = from;
  t.to = to;
  t.to_committed = committed;
  t.root_target = root;  // compatibility view of the first demand
  t.root_goal = goal;
  t.roots = {DemandKey{root, goal}};
  t.task_priority = task_priority;
  return t;
}

DDObjectiveOptionProbe option(int root, int goal, double score,
                              std::vector<ManipulationTask> chain)
{
  DDObjectiveOptionProbe out;
  out.root_target = root;
  out.root_goal = goal;
  out.score = score;
  out.chain = std::move(chain);
  return out;
}

ManipulationTask committed_claim(int shelf, int from, int claim, int root,
                                 int priority = 0)
{
  return task(shelf, from, claim, true, root, 1000 + root, priority);
}

DDInstance pressure_instance()
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.robots = {ins.grid.idx(1, 2)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 4)};
  ins.target_starts = {ins.grid.idx(0, 0), ins.grid.idx(0, 4)};
  const std::vector<int> pool = {ins.grid.idx(0, 1), ins.grid.idx(0, 3)};
  ins.target_goal_sets = {pool, pool};
  ins.target_goals = {pool[0], pool[0]};
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_objective_tasks, physical_key_merges_roots_and_committed_semantics)
{
  auto advisory = task(-1, 7, 8, false, 0, 20, 30);
  auto committed = task(-1, 7, 9, true, 1, 21, 80);
  const auto merged =
      dd_merge_objective_tasks_probe({advisory, committed}, {30, 80});

  ASSERT_FALSE(merged.conflict);
  ASSERT_EQ(merged.tasks.size(), 1u);
  const auto& t = merged.tasks.front();
  EXPECT_TRUE(t.to_committed);
  EXPECT_EQ(t.to, 9);
  EXPECT_EQ(t.task_priority, 80);
  EXPECT_NE(t.id, 0u);
  EXPECT_EQ(t.roots, (std::vector<DemandKey>{{0, 20}, {1, 21}}));
  EXPECT_EQ(merged.tasks_merged, 1);

  // Identity ignores roots and advisory->committed drop refinement.
  const auto left =
      dd_merge_objective_tasks_probe({advisory}, {30}).tasks.front();
  const auto right =
      dd_merge_objective_tasks_probe({committed}, {80}).tasks.front();
  EXPECT_EQ(left.id, right.id);

  auto committed_elsewhere = committed;
  committed_elsewhere.to = 10;
  const auto conflict = dd_merge_objective_tasks_probe(
      {committed, committed_elsewhere}, {80, 80});
  EXPECT_TRUE(conflict.conflict)
      << "same physical task with two committed drops is a true conflict";
}

TEST(dd_objective_tasks, merge_keeps_v3_representative_depth_on_priority_ties)
{
  auto first = task(-1, 7, 8, false, 0, 20, 30);
  first.priority = 50;
  first.depth = 3;
  auto tied_later = task(-1, 7, 9, false, 1, 21, 80);
  tied_later.priority = 50;
  tied_later.depth = 1;

  auto merged =
      dd_merge_objective_tasks_probe({first, tied_later}, {30, 80});
  ASSERT_EQ(merged.tasks.size(), 1u);
  EXPECT_EQ(merged.tasks.front().depth, 3)
      << "v3 pool dedupe kept the first representative on equal priority";
  EXPECT_EQ(merged.tasks.front().priority, 50);
  EXPECT_EQ(merged.tasks.front().task_priority, 80);
  EXPECT_EQ(merged.tasks.front().roots,
            (std::vector<DemandKey>{{0, 20}, {1, 21}}));

  auto higher_later = tied_later;
  higher_later.priority = 60;
  higher_later.depth = 2;
  merged = dd_merge_objective_tasks_probe({first, higher_later}, {30, 80});
  ASSERT_EQ(merged.tasks.size(), 1u);
  EXPECT_EQ(merged.tasks.front().depth, 2)
      << "a strictly higher legacy priority replaces the representative";
  EXPECT_EQ(merged.tasks.front().priority, 60);
}

TEST(dd_objective_tasks, phase_t_order_keeps_the_v3_representative_root)
{
  auto root0 = task(-1, 7, 8, false, 0, 20, 30);
  root0.priority = 50;
  root0.depth = 1;
  auto root1 = task(-1, 7, 9, false, 1, 21, 80);
  root1.priority = 50;
  root1.depth = 1;

  DDObjectiveResolveProbeInput in;
  in.options = {
      {option(0, 20, 0, {root0})},
      {option(1, 21, 0, {root1})},
  };
  in.tentative = {0, 0};
  in.phase_t_order = {1, 0};
  in.effective_priority = {30, 80};

  const auto out = dd_resolve_objective_options_probe(in);
  ASSERT_EQ(out.tasks.size(), 1u);
  EXPECT_EQ(out.tasks.front().root_target, 1)
      << "equal-priority physical duplicates keep the first Phase-T "
         "emission as their v3 scheduling/pricing representative";
  EXPECT_EQ(out.tasks.front().root_goal, 21);
  EXPECT_EQ(out.tasks.front().roots,
            (std::vector<DemandKey>{{0, 20}, {1, 21}}));
  EXPECT_EQ(out.tasks.front().task_priority, 80);
}

TEST(dd_objective_tasks, selected_package_keeps_the_complete_chain)
{
  DDObjectiveResolveProbeInput in;
  in.options = {{
      option(0, 40, 0,
             {task(-1, 10, -1, false, 0, 40),
              task(-1, 11, -1, false, 0, 40),
              task(-1, 12, -1, false, 0, 40)}),
  }};
  in.tentative = {0};
  in.effective_priority = {10};

  const auto out = dd_resolve_objective_options_probe(in);
  ASSERT_EQ(out.selected, (std::vector<int>{0}));
  EXPECT_EQ(out.tasks.size(), 3u)
      << "a selected option emits its full <=CLEAR_CHAIN_K chain";
}

TEST(dd_objective_resolve, singleton_has_only_path_drop_negotiation_surface)
{
  DDObjectiveResolveProbeInput in;
  in.options = {{
      option(0, 40, 0, {committed_claim(0, 100, 10, 0)}),
  }};
  in.tentative = {0};
  in.effective_priority = {10};

  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.selected, (std::vector<int>{0}));
  EXPECT_EQ(out.obj_reselect_requests, 0);
  EXPECT_EQ(out.obj_yields, 0);
  ASSERT_EQ(out.tasks.size(), 1u);
  EXPECT_EQ(out.tasks[0].root_goal, 40);
}

TEST(dd_objective_resolve, lower_priority_adapts_without_evicting_higher)
{
  DDObjectiveResolveProbeInput in;
  in.options = {
      {option(0, 40, 0, {committed_claim(0, 100, 10, 0)})},
      {option(1, 41, 0, {committed_claim(1, 200, 10, 1)}),
       option(1, 41, 1, {committed_claim(1, 200, 11, 1)})},
  };
  in.tentative = {0, 0};
  in.effective_priority = {100, 10};

  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.selected, (std::vector<int>{0, 1}));
  EXPECT_EQ(out.yielded, (std::vector<uint8_t>{0, 0}));
  EXPECT_GE(out.obj_reselect_requests, 1);
}

TEST(dd_objective_resolve, inheritance_propagates_A_to_B_to_C)
{
  DDObjectiveResolveProbeInput in;
  in.options = {
      {option(0, 40, 0, {committed_claim(0, 100, 10, 0)})},
      {option(1, 41, 0, {committed_claim(1, 200, 10, 1)}),
       option(1, 41, 1, {committed_claim(1, 200, 20, 1)})},
      {option(2, 42, 0, {committed_claim(2, 300, 20, 2)}),
       option(2, 42, 1, {committed_claim(2, 300, 30, 2)})},
  };
  in.tentative = {0, 0, 0};
  in.effective_priority = {300, 200, 100};

  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.selected, (std::vector<int>{0, 1, 1}));
  EXPECT_EQ(out.yielded, (std::vector<uint8_t>{0, 0, 0}));
  EXPECT_GE(out.obj_inherit_depth_max, 2);
}

TEST(dd_objective_resolve, child_fail_backtracks_parent_then_final_yield)
{
  DDObjectiveResolveProbeInput backtrack;
  backtrack.options = {
      {option(0, 40, 0, {committed_claim(0, 100, 10, 0)}),
       option(0, 40, 1, {committed_claim(0, 100, 11, 0)})},
      {option(1, 41, 0, {committed_claim(1, 200, 10, 1)})},
  };
  backtrack.tentative = {0, 0};
  backtrack.effective_priority = {100, 10};
  const auto retried = dd_resolve_objective_options_probe(backtrack);
  EXPECT_EQ(retried.selected, (std::vector<int>{1, 0}));
  EXPECT_EQ(retried.yielded, (std::vector<uint8_t>{0, 0}));
  EXPECT_GE(retried.obj_backtracks, 1)
      << "B exhaustion returns FAIL so A tries its next option";

  DDObjectiveResolveProbeInput yield;
  yield.options = {
      {option(0, 40, 0, {committed_claim(0, 100, 10, 0)})},
      {option(1, 41, 0, {committed_claim(1, 200, 10, 1)})},
  };
  yield.tentative = {0, 0};
  yield.effective_priority = {100, 10};
  const auto adjudicated = dd_resolve_objective_options_probe(yield);
  EXPECT_EQ(adjudicated.selected[0], 0);
  EXPECT_EQ(adjudicated.yielded, (std::vector<uint8_t>{0, 1}));
  EXPECT_EQ(adjudicated.obj_yields, 1)
      << "only the top-level adjudicator yields the lower priority side";
}

TEST(dd_objective_resolve, in_flight_committed_claim_is_never_renegotiated)
{
  DDObjectiveResolveProbeInput in;
  in.options = {{
      option(0, 40, 0, {committed_claim(0, 100, 10, 0)}),
      option(0, 40, 1, {committed_claim(0, 100, 11, 0)}),
  }};
  in.tentative = {0};
  in.effective_priority = {1000};
  in.fixed_in_flight = {committed_claim(9, 900, 10, 9)};

  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.selected, (std::vector<int>{1}))
      << "even a higher-priority new demand adapts around in-flight custody";
  EXPECT_EQ(out.yielded, (std::vector<uint8_t>{0}));
}

TEST(dd_objective_resolve, budget_exhaustion_keeps_tentative_defaults)
{
  DDObjectiveResolveProbeInput in;
  in.options = {
      {option(0, 40, 0, {committed_claim(0, 100, 10, 0)})},
      {option(1, 41, 0, {committed_claim(1, 200, 10, 1)})},
  };
  in.tentative = {0, 0};
  in.effective_priority = {100, 10};
  in.depth_cap = 0;
  in.reselect_cap = 0;

  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.selected, in.tentative);
  EXPECT_EQ(out.yielded, (std::vector<uint8_t>{0, 0}));
  EXPECT_EQ(out.obj_default_resolutions, 2)
      << "the same compiler's tentative output is retained, not a legacy call";
}

TEST(dd_objective_resolve, parent_option_sticks_only_within_hysteresis)
{
  DDObjectiveResolveProbeInput sticky;
  sticky.options = {{
      option(0, 40, 0, {committed_claim(0, 100, 10, 0)}),
      option(0, 40, 1, {committed_claim(0, 100, 11, 0)}),
  }};
  sticky.tentative = {1};
  sticky.parent_selected = {1};
  sticky.effective_priority = {10};
  EXPECT_EQ(dd_resolve_objective_options_probe(sticky).selected[0], 1);

  sticky.options[0][1].score = 3;
  EXPECT_EQ(dd_resolve_objective_options_probe(sticky).selected[0], 0)
      << "a score gap above ASSIGNMENT_HYSTERESIS must beat stickiness";
}

TEST(dd_objective_priority, aging_is_per_target_temporary_and_resets)
{
  DDObjectiveProgressProbeInput progress;
  progress.parent_best_lb = {5, 5, 5, 5};
  progress.parent_no_progress = {23, 23, 23, 23};
  progress.parent_tau = {10, 10, 10, 10};
  progress.current_lb = {5, 4, 5, 5};
  progress.current_tau = {10, 10, 11, 10};
  progress.task_completed = {0, 0, 0, 1};
  progress.base_priority = {1, 4, 3, 2};
  const auto state = dd_objective_progress_probe(progress);

  EXPECT_EQ(state.no_progress, (std::vector<int>{24, 0, 0, 0}));
  EXPECT_EQ(state.best_lb, (std::vector<int>{5, 4, 5, 5}));
  EXPECT_EQ(state.aging, (std::vector<uint8_t>{1, 0, 0, 0}));
  EXPECT_GT(state.effective_priority[0], state.effective_priority[1]);

  // The temporary aging priority must affect this node's resolver order.
  DDObjectiveResolveProbeInput in;
  in.options = {
      {option(0, 40, 0, {committed_claim(0, 100, 10, 0)})},
      {option(1, 41, 0, {committed_claim(1, 200, 10, 1)})},
  };
  in.tentative = {0, 0};
  in.effective_priority = {state.effective_priority[0],
                           state.effective_priority[1]};
  const auto out = dd_resolve_objective_options_probe(in);
  EXPECT_EQ(out.yielded, (std::vector<uint8_t>{0, 1}));
}

TEST(dd_objective_tau, residual_pressure_repairs_matching_without_touching_h)
{
  const auto ins = pressure_instance();
  const auto X = initial_phys_config(ins);
  double h0 = -1, h1 = -1;
  const auto tau0 = dd_solve_tau(ins, X, nullptr, &h0);
  const auto tau1 =
      dd_solve_tau_with_pressure_probe(ins, X, {0.0, 10.0}, &h1);

  ASSERT_EQ(tau0.size(), 2u);
  ASSERT_EQ(tau1.size(), 2u);
  EXPECT_NE(tau0, tau1);
  EXPECT_NE(tau1[0], tau1[1])
      << "goal repair remains a globally feasible matching";
  EXPECT_DOUBLE_EQ(h0, h1)
      << "pressure/options/claims/priority never enter admissible h";
}
