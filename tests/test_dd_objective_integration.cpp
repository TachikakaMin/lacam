// PROTECTED integration tests: v4.1 Objective-PIBT is on the existing
// production build_guidance -> rho execution path.  Written before the
// integration change (TDD RED), 2026-09-02.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

namespace {

DDInstance labeled_ready_fixture()
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 4)};
  for (int c = 0; c < 4; ++c) ins.shelves.push_back(ins.grid.idx(0, c));
  ins.target_starts = {ins.grid.idx(0, 0), ins.grid.idx(0, 3)};
  ins.target_goals = {ins.grid.idx(0, 4), ins.grid.idx(0, 0)};
  ins.finalize();
  return ins;
}

DDInstance one_root_ready_fixture()
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 0)};
  for (int c = 0; c < 4; ++c) ins.shelves.push_back(ins.grid.idx(0, c));
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.finalize();
  return ins;
}

void expect_same_task(const ManipulationTask& a, const ManipulationTask& b)
{
  EXPECT_EQ(a.shelf_target, b.shelf_target);
  EXPECT_EQ(a.from, b.from);
  EXPECT_EQ(a.to, b.to);
  EXPECT_EQ(a.to_committed, b.to_committed);
  EXPECT_EQ(a.roots, b.roots);
  EXPECT_EQ(a.id, b.id);
}

}  // namespace

TEST(dd_objective_integration, production_pool_merges_cross_root_demands)
{
  const auto ins = labeled_ready_fixture();
  DDObjectiveBuildProbe probe;
  std::vector<int> rho;
  const auto tasks =
      dd_build_tasks(ins, initial_phys_config(ins), &rho, &probe);

  const int pickup = ins.grid.idx(0, 3);
  auto it = std::find_if(tasks.begin(), tasks.end(),
                         [&](const ManipulationTask& t) {
                           return t.from == pickup && t.to_committed;
                         });
  ASSERT_NE(it, tasks.end());
  EXPECT_EQ(it->roots,
            (std::vector<DemandKey>{{0, ins.grid.idx(0, 4)},
                                    {1, ins.grid.idx(0, 0)}}));
  ASSERT_EQ(probe.effective_priority.size(), 2u);
  EXPECT_EQ(it->task_priority,
            std::max(probe.effective_priority[0],
                     probe.effective_priority[1]));
  EXPECT_GE(probe.tasks_merged, 1);
  EXPECT_EQ(probe.selected_option.size(), 2u);
  ASSERT_EQ(rho.size(), ins.n_robots());
  ASSERT_GE(rho[0], 0);
  EXPECT_EQ(tasks[rho[0]].id, it->id);
}

TEST(dd_objective_integration, one_empty_default_is_same_compiler_output)
{
  const auto ins = one_root_ready_fixture();
  DDObjectiveBuildProbe probe;
  const auto selected =
      dd_build_tasks(ins, initial_phys_config(ins), nullptr, &probe);

  ASSERT_EQ(probe.tentative_tasks.size(), selected.size());
  for (size_t k = 0; k < selected.size(); ++k)
    expect_same_task(probe.tentative_tasks[k], selected[k]);
  EXPECT_EQ(probe.selected_option, (std::vector<int>{0}));
  EXPECT_EQ(probe.obj_reselect_requests, 0);
  EXPECT_EQ(probe.obj_yields, 0);
}

TEST(dd_objective_integration,
     conflict_free_production_build_generates_only_default_packages)
{
  DDInstance ins;
  ins.grid = DDGrid({".......", "......."});
  ins.robots = {ins.grid.idx(1, 0), ins.grid.idx(1, 6)};
  ins.shelves = {ins.grid.idx(0, 0), ins.grid.idx(0, 6)};
  ins.target_starts = {ins.grid.idx(0, 0), ins.grid.idx(0, 6)};
  ins.target_goals = {ins.grid.idx(0, 2), ins.grid.idx(0, 4)};
  ins.finalize();

  DDObjectiveBuildProbe probe;
  (void)dd_build_tasks(ins, initial_phys_config(ins), nullptr, &probe);
  ASSERT_EQ(probe.option_counts.size(), 2u);
  EXPECT_EQ(probe.option_counts, (std::vector<int>{1, 1}))
      << "route alternatives are a conflict-resolution surface, not "
         "unconditional per-node work";
}
