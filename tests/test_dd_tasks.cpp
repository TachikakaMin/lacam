// PROTECTED tests: v3.0 step 1 — ManipulationTask pool + TaskId-bound rho
// (design_final v3.0 §3/§5; new.md §2/§4; debug.md invariants 19/23).
// Written BEFORE implementation (TDD RED).
//
// Step-1 scope: tasks exist as the SOURCE of requests (requests are the
// pickup projection), rho binds task pool indices, TaskId is stable
// across robot motion.  The frontier compiler (`to` of clear tasks,
// one-empty vacancy rule) is step 2 and is deliberately NOT pinned here.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

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

}  // namespace

TEST(dd_tasks, serve_task_carries_shelf_root_and_projection)
{
  // free path: the pool must contain a serve task MoveShelf(b0,
  // start -> assigned goal, root = b0 -> goal), and every request must be
  // the pickup projection of its task.
  auto ins = make_ins({"....", "...."}, {{1, 0}}, {{0, 1}},
                      {{{0, 1}, {0, 3}}});
  const auto X = initial_phys_config(ins);
  const auto tasks = dd_build_tasks(ins, X);
  ASSERT_FALSE(tasks.empty());
  int serve = -1;
  for (size_t k = 0; k < tasks.size(); ++k)
    if (tasks[k].priority == 100) serve = (int)k;
  ASSERT_GE(serve, 0) << "no serve task in the pool";
  EXPECT_EQ(tasks[serve].shelf_target, 0);
  EXPECT_EQ(tasks[serve].from, ins.grid.idx(0, 1));
  EXPECT_EQ(tasks[serve].to, ins.grid.idx(0, 3));  // assigned goal
  EXPECT_EQ(tasks[serve].root_target, 0);
  EXPECT_EQ(tasks[serve].root_goal, ins.grid.idx(0, 3));
  EXPECT_EQ(tasks[serve].depth, 0);
  EXPECT_NE(tasks[serve].id, 0u);
}

TEST(dd_tasks, clear_task_identifies_blocker_and_root)
{
  // blocked path: the first blocker on b0's SELECTED (least-blocking)
  // path must be compiled as a clear task rooted at b0's objective; the
  // blocker is anonymous, so the task identifies it by cell (equivalence
  // class), not by label.  Corridor topology forces the path through the
  // blocker (fixture change APPROVED by protected-test review
  // 2026-09-01: an open second row let the least-blocking path route
  // around the blocker, which is correct production behavior).
  auto ins = make_ins({"...."}, {{0, 0}}, {{0, 1}, {0, 2}},
                      {{{0, 1}, {0, 3}}});
  const auto X = initial_phys_config(ins);
  const auto tasks = dd_build_tasks(ins, X);
  ASSERT_FALSE(tasks.empty());
  int clear = -1;
  for (size_t k = 0; k < tasks.size(); ++k)
    if (tasks[k].from == ins.grid.idx(0, 2)) clear = (int)k;
  ASSERT_GE(clear, 0) << "no clear task for the blocker cell";
  EXPECT_EQ(tasks[clear].shelf_target, -1);  // anon: cell identity
  EXPECT_EQ(tasks[clear].root_target, 0);
  EXPECT_EQ(tasks[clear].root_goal, ins.grid.idx(0, 3));
  EXPECT_EQ(tasks[clear].priority, 50);  // chain head
  EXPECT_GE(tasks[clear].depth, 1);
  EXPECT_NE(tasks[clear].id, 0u);
}

TEST(dd_tasks, task_id_stable_across_robot_motion)
{
  // TaskId identifies (shelf, from, root) — it must NOT change when only
  // free-robot positions change (that is what makes hysteresis by task
  // identity meaningful across nodes).
  auto ins = make_ins({"....", "...."}, {{1, 0}}, {{0, 1}, {0, 2}},
                      {{{0, 1}, {0, 3}}});
  auto X = initial_phys_config(ins);
  const auto tasks_a = dd_build_tasks(ins, X);
  X.robots[0] = ins.grid.idx(1, 3);  // same shelves, robot elsewhere
  const auto tasks_b = dd_build_tasks(ins, X);
  ASSERT_EQ(tasks_a.size(), tasks_b.size());
  for (size_t k = 0; k < tasks_a.size(); ++k) {
    EXPECT_EQ(tasks_a[k].id, tasks_b[k].id) << "task " << k;
    EXPECT_EQ(tasks_a[k].from, tasks_b[k].from) << "task " << k;
  }
}

TEST(dd_tasks, rho_binds_task_and_requests_follow)
{
  // the free robot must be bound to a task pool index; free_goal and the
  // request index are derived views of that binding.
  auto ins = make_ins({"....", "...."}, {{1, 0}}, {{0, 1}},
                      {{{0, 1}, {0, 3}}});
  const auto X = initial_phys_config(ins);
  std::vector<int> rho_task;
  const auto tasks = dd_build_tasks(ins, X, &rho_task);
  ASSERT_FALSE(tasks.empty());
  ASSERT_EQ(rho_task.size(), ins.n_robots());
  ASSERT_GE(rho_task[0], 0) << "free robot not bound to any task";
  ASSERT_LT(rho_task[0], (int)tasks.size());
  const auto goals = dd_match_free_goals(ins, X, nullptr);
  EXPECT_EQ(goals[0], tasks[rho_task[0]].from)
      << "free_goal must be the bound task's pickup cell";
}
