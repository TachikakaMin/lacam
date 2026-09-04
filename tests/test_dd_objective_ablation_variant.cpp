// PROTECTED structural-ablation contract (TDD RED), 2026-09-02.
// Each executable compiles the production inline resolver/merge code with
// exactly one structural CMake definition.  These variants are benchmark
// controls, never runtime strategy switches.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

namespace {

ManipulationTask task(int shelf, int from, int root, int goal,
                      int priority = 50)
{
  ManipulationTask t;
  t.shelf_target = shelf;
  t.from = from;
  t.to = from + 1;
  t.root_target = root;
  t.root_goal = goal;
  t.roots = {DemandKey{root, goal}};
  t.priority = priority;
  return t;
}

ObjectiveOption option(int root, int goal,
                       std::initializer_list<ManipulationTask> chain,
                       int score)
{
  ObjectiveOption o;
  o.root_target = root;
  o.root_goal = goal;
  o.chain = chain;
  o.score = score;
  return o;
}

}  // namespace

#if defined(DD_OBJECTIVE_FORCE_DEFAULT)
TEST(dd_objective_ablation_a, compile_time_variant_forces_tentative_defaults)
{
  const auto a0 = option(0, 10, {task(0, 3, 0, 10)}, 0);
  const auto a1 = option(0, 10, {task(0, 4, 0, 10)}, 1);
  const auto b0 = option(1, 11, {task(1, 3, 1, 11)}, 0);
  const auto b1 = option(1, 11, {task(1, 5, 1, 11)}, 1);

  const auto out = carrier_detail::resolve_objective_options(
      {{a0, a1}, {b0, b1}}, {0, 0}, {}, {20, 10}, {});

  EXPECT_EQ(out.selected, (std::vector<int>{0, 0}));
  EXPECT_EQ(out.obj_default_resolutions, 2);
  EXPECT_EQ(out.obj_reselect_requests, 0);
  EXPECT_EQ(out.obj_yields, 0);
}
#elif defined(DD_OBJECTIVE_NO_INHERIT)
TEST(dd_objective_ablation_b, compile_time_variant_adapts_without_push_chain)
{
  const auto a0 = option(0, 10, {task(0, 3, 0, 10)}, 0);
  const auto a1 = option(0, 10, {task(0, 4, 0, 10)}, 1);
  const auto b0 = option(1, 11, {task(1, 3, 1, 11)}, 0);
  const auto b1 = option(1, 11, {task(1, 5, 1, 11)}, 1);

  const auto out = carrier_detail::resolve_objective_options(
      {{a0, a1}, {b0, b1}}, {0, 0}, {}, {20, 10}, {});

  EXPECT_EQ(out.selected, (std::vector<int>{1, 0}))
      << "without inheritance, the active root adapts instead of pushing "
         "the lower-priority owner to its alternate";
  EXPECT_EQ(out.obj_reselect_requests, 0);
}
#elif defined(DD_OBJECTIVE_DROP_SECOND_ROOT)
TEST(dd_objective_ablation_c, compile_time_variant_drops_duplicate_root)
{
  auto first = task(7, 12, 0, 20, 40);
  auto second = task(7, 12, 1, 21, 80);

  const auto out =
      carrier_detail::merge_objective_tasks({first, second}, {40, 80});

  ASSERT_FALSE(out.conflict);
  ASSERT_EQ(out.tasks.size(), 1u);
  EXPECT_EQ(out.tasks.front().roots, (std::vector<DemandKey>{{0, 20}}));
  EXPECT_EQ(out.tasks.front().task_priority, 40);
  EXPECT_EQ(out.tasks_merged, 1);
}
#else
#error "compile this test with exactly one DD_OBJECTIVE_* ablation"
#endif
