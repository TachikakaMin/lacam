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

// ---- v3.0 step 2: frontier compiler (design_final §4.1 pseudo; new.md
// §2 one-empty rule; debug.md §7.2 test 5).  Written BEFORE the
// implementation (TDD RED). ----

TEST(dd_tasks, one_empty_ready_task_moves_vacancy_adjacent_shelf)
{
  // 1x5 corridor, upper deck: shelves on cells 0..3, single vacancy at
  // cell 4.  Target b0 = shelf@0 -> goal 4.  The head blocker (0,1) has
  // no adjacent free upper cell, so carrying it cannot even start; the
  // READY task must instead move the first vacancy-adjacent shelf on the
  // routing chain — anon@(0,3) — INTO the vacancy (15-puzzle semantics):
  // MoveShelf(anon@(0,3), (0,3) -> (0,4), root = b0 -> (0,4)).
  auto ins = make_ins({"....."}, {{0, 0}},
                      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
                      {{{0, 0}, {0, 4}}});
  const auto X = initial_phys_config(ins);
  std::vector<int> rho_task;
  const auto tasks = dd_build_tasks(ins, X, &rho_task);
  ASSERT_FALSE(tasks.empty());
  int ready = -1;
  for (size_t k = 0; k < tasks.size(); ++k)
    if (tasks[k].from == ins.grid.idx(0, 3)) ready = (int)k;
  ASSERT_GE(ready, 0) << "no task for the vacancy-adjacent shelf";
  EXPECT_EQ(tasks[ready].to, ins.grid.idx(0, 4))
      << "ready task must drop INTO the current vacancy";
  EXPECT_EQ(tasks[ready].shelf_target, -1);
  EXPECT_EQ(tasks[ready].root_target, 0);
  EXPECT_EQ(tasks[ready].root_goal, ins.grid.idx(0, 4));
  // no executable task may ask a robot to lift a shelf that cannot move:
  // every emitted pickup must either have an adjacent free upper cell or
  // be the vacancy-adjacent ready shelf itself.
  for (const auto& t : tasks) {
    if (t.from == ins.grid.idx(0, 3)) continue;  // the ready task
    int nb[4];
    const int n = ins.grid.neighbors(t.from, nb);
    bool can_start = false;
    for (int q = 0; q < n; ++q) {
      bool upper_free = true;
      for (int c : X.anon_occ) upper_free &= c != nb[q];
      for (int c : X.target_pos) upper_free &= c != nb[q];
      can_start |= upper_free;
    }
    EXPECT_TRUE(can_start) << "task pickup " << t.from
                           << " cannot start a carry (hover-lift bait)";
  }
  // and the free robot must be routed to an executable pickup
  ASSERT_GE(rho_task[0], 0);
  EXPECT_EQ(tasks[rho_task[0]].from, ins.grid.idx(0, 3));
}

TEST(dd_tasks, feasible_clear_head_gets_compiler_chosen_drop)
{
  // corridor with a movable blocker: the chain-head clear task must name
  // its drop cell (nearest free upper cell off the protected path when
  // one exists) instead of leaving the destination carrier-chosen.
  auto ins = make_ins({"...."}, {{0, 0}}, {{0, 1}, {0, 2}},
                      {{{0, 1}, {0, 3}}});
  const auto X = initial_phys_config(ins);
  const auto tasks = dd_build_tasks(ins, X);
  int clear = -1;
  for (size_t k = 0; k < tasks.size(); ++k)
    if (tasks[k].from == ins.grid.idx(0, 2)) clear = (int)k;
  ASSERT_GE(clear, 0);
  EXPECT_EQ(tasks[clear].to, ins.grid.idx(0, 0))
      << "chain-head clear must carry the compiler-chosen drop cell";
}

TEST(dd_tasks, unstartable_head_skips_drop_hint)
{
  // Regression pin (gate 2026-09-01, brap_h10w10_a12_e3_R1_seed1 lost to
  // guidance overhead): a chain-head blocker that cannot start a carry
  // (no adjacent free upper cell) outside the one-empty regime keeps
  // to == -1 — its drop cell is meaningless until it can move, and the
  // per-head hint BFS on protect-saturated boards is pure overhead.
  auto ins = make_ins({".....", "....."}, {{1, 0}},
                      {{0, 1}, {0, 2}, {0, 3}, {1, 1}, {1, 2}, {1, 3}},
                      {{{0, 1}, {0, 4}}});
  const auto X = initial_phys_config(ins);
  const auto tasks = dd_build_tasks(ins, X);
  int head = -1;
  for (size_t k = 0; k < tasks.size(); ++k)
    if (tasks[k].from == ins.grid.idx(0, 2) && tasks[k].priority == 50)
      head = (int)k;
  ASSERT_GE(head, 0);
  EXPECT_EQ(tasks[head].to, -1)
      << "unstartable head must not carry a compiler drop hint";
}

// ---- v3.0 step 3: execution-aware tau_guide (design_final §4.1/§5.1;
// new.md §3/§4; debug.md §7.2 test 3).  Written BEFORE the
// implementation (TDD RED). ----

namespace {

DDInstance make_pool_ins(const std::vector<std::string>& rows,
                         const std::vector<std::pair<int, int>>& robots,
                         const std::vector<std::pair<int, int>>& shelves,
                         std::pair<int, int> start,
                         const std::vector<std::pair<int, int>>& goals)
{
  DDInstance ins;
  ins.grid = DDGrid(rows);
  for (auto& q : robots) ins.robots.push_back(ins.grid.idx(q.first, q.second));
  for (auto& p : shelves)
    ins.shelves.push_back(ins.grid.idx(p.first, p.second));
  ins.target_starts.push_back(ins.grid.idx(start.first, start.second));
  std::vector<int> set;
  for (auto& g : goals) set.push_back(ins.grid.idx(g.first, g.second));
  ins.target_goal_sets.push_back(set);
  ins.target_goals.push_back(set.front());
  ins.finalize();
  return ins;
}

int pool_root_goal(const std::vector<ManipulationTask>& tasks)
{
  // every task in these single-target fixtures roots at b0; return the
  // root goal the compiler is currently serving (highest priority task)
  int best = -1, best_prio = -1;
  for (const auto& t : tasks)
    if (t.priority > best_prio) {
      best_prio = t.priority;
      best = t.root_goal;
    }
  return best;
}

}  // namespace

TEST(dd_tasks, robot_placement_flips_tau_guide_goal)
{
  // design_final §4.1 example, minimal form: b0 at (0,5) has goals
  // gL=(0,0) (lb cheaper by 1, but the corridor toward it — walls forbid
  // a detour under cells (0,0)/(0,1) — is blocked at (0,1), four cells
  // from the shelf) and gR=(0,11) (free path).  With the only robot on
  // the far right, realizing the left frontier costs 4 MORE than the
  // shelf approach itself (delta price 4 > lb gap 1 + eta 2): the
  // matching must flip to gR.  With the robot next to the left frontier
  // the delta is negative and lb + hysteresis keeps gL.  The target does
  // not move between the two states — only the robot.
  const std::vector<std::string> rows = {"............", "##.........."};
  auto far_ins = make_pool_ins(rows, {{1, 11}}, {{0, 5}, {0, 1}},
                               {0, 5}, {{0, 0}, {0, 11}});
  const auto far_tasks =
      dd_build_tasks(far_ins, initial_phys_config(far_ins));
  ASSERT_FALSE(far_tasks.empty());
  EXPECT_EQ(pool_root_goal(far_tasks), far_ins.grid.idx(0, 11))
      << "far robot: execution price must flip the root goal to gR";

  auto near_ins = make_pool_ins(rows, {{1, 2}}, {{0, 5}, {0, 1}},
                                {0, 5}, {{0, 0}, {0, 11}});
  const auto near_tasks =
      dd_build_tasks(near_ins, initial_phys_config(near_ins));
  ASSERT_FALSE(near_tasks.empty());
  EXPECT_EQ(pool_root_goal(near_tasks), near_ins.grid.idx(0, 0))
      << "near robot: lb-preferred gL must survive";
}

TEST(dd_tasks, execution_price_never_enters_admissible_h)
{
  // debug.md invariant 18 / §7.2 test 6: the admissible h is the
  // unrestricted LB matching — identical for both robot placements of
  // the flip fixture above (robots do not appear in tau_LB).
  const std::vector<std::string> rows = {"............", "##.........."};
  auto far_ins = make_pool_ins(rows, {{1, 11}}, {{0, 5}, {0, 1}},
                               {0, 5}, {{0, 0}, {0, 11}});
  auto near_ins = make_pool_ins(rows, {{1, 2}}, {{0, 5}, {0, 1}},
                                {0, 5}, {{0, 0}, {0, 11}});
  double h_far = -1, h_near = -1;
  dd_solve_tau(far_ins, initial_phys_config(far_ins), nullptr, &h_far);
  dd_solve_tau(near_ins, initial_phys_config(near_ins), nullptr, &h_near);
  EXPECT_DOUBLE_EQ(h_far, h_near);
  EXPECT_DOUBLE_EQ(h_far, 5 + 2);  // alpha*dist((0,5)->(0,0)) + 2*gamma
}

TEST(dd_tasks, custody_keeps_task_id_from_lift_through_drop)
{
  // debug.md §7.2 test 4: one manipulation = approach (bound via rho) ->
  // Lift -> carry (custody) -> Drop, all under ONE TaskId.  One-empty
  // fixture: the ready task anon@(0,3) -> (0,4) is the only executable
  // move; the robot approaches from (0,0).
  auto ins = make_ins({"....."}, {{0, 0}},
                      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
                      {{{0, 0}, {0, 4}}});
  const auto trace = dd_rollout_custody_trace(ins, 0, 32, 0);
  ASSERT_GE(trace.size(), 3u);
  // find the Lift boundary: FREE -> ANON
  int lift_at = -1;
  for (size_t t = 1; t < trace.size(); ++t)
    if (trace[t - 1].kappa == KAPPA_FREE && trace[t].kappa == KAPPA_ANON)
      lift_at = (int)t;
  ASSERT_GE(lift_at, 1) << "the robot never lifted the ready shelf";
  const uint64_t bound = trace[lift_at - 1].bound_id;
  ASSERT_NE(bound, 0u) << "approach phase must be bound to a task";
  // custody carries the SAME TaskId through every loaded step
  bool saw_carry = false;
  for (size_t t = lift_at; t < trace.size() && trace[t].kappa == KAPPA_ANON;
       ++t) {
    saw_carry = true;
    EXPECT_EQ(trace[t].custody_id, bound)
        << "custody id diverged from the bound task at step " << t;
  }
  EXPECT_TRUE(saw_carry);
}

// ---- 2026-09-02 R2 (debug.md §10): the EXECUTION half of the task loop.
// Written BEFORE implementation (TDD RED). ----

TEST(dd_tasks, one_empty_drop_lands_at_custody_task_to)
{
  // Invariant 22's behavior face: in the one-empty regime the executed
  // Drop must land at the custody task's CURRENT drop cell (the vacancy)
  // — deriving the carry waypoint from the task, not from an unrelated
  // parking choice.  The custody task's `to` is refined position-aware
  // every node (a stale compile-time `to` was falsified on dense boards;
  // the refinement is the same-task re-targeting design_final §6.1
  // allows), and in one-empty it must BE the vacancy.
  auto ins = make_ins({"....."}, {{0, 0}},
                      {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
                      {{{0, 0}, {0, 4}}});
  const auto trace = dd_rollout_custody_trace(ins, 0, 32, 0);
  ASSERT_GE(trace.size(), 3u);
  int lift_at = -1;
  for (size_t t = 1; t < trace.size() && lift_at < 0; ++t)
    if (trace[t - 1].kappa == KAPPA_FREE && trace[t].kappa == KAPPA_ANON)
      lift_at = (int)t;  // FIRST episode: the vacancy moves afterwards
  ASSERT_GE(lift_at, 1);
  // during the carry, the custody task's current drop cell is the vacancy
  int drop_step = -1;
  for (size_t t = lift_at; t < trace.size(); ++t) {
    if (trace[t].kappa != KAPPA_ANON) {
      drop_step = (int)t;
      break;
    }
    EXPECT_EQ(trace[t].custody_to, ins.grid.idx(0, 4))
        << "carry waypoint must be the custody task's vacancy at step "
        << t;
  }
  ASSERT_GE(drop_step, lift_at + 1) << "the carry never ended in a drop";
  // the executed Drop happened at the custody task's to: the robot's cell
  // when it became FREE again is the vacancy
  EXPECT_EQ(trace[drop_step].cell, ins.grid.idx(0, 4))
      << "Drop must land at task.to (the vacancy), not a parking cell";
}

TEST(dd_tasks, depth_orders_equal_priority_tasks_in_rho)
{
  // R2(c): dependency depth participates in rho's task ordering (design
  // §5.1 "task priority / dependency depth").  Priority already encodes
  // the chain position (50-emitted); the extra information depth carries
  // is the one-empty ready hop (+1).  Contract: within EQUAL priority
  // and equal pickup distance, the SHALLOWER task binds first.
  //
  // Deterministic one-empty fixture (3x4, single vacancy (2,3), robot on
  // its lower deck):
  //   b0 (0,1)->(0,3): head (0,2) unstartable -> READY task from (1,3),
  //       to (2,3), depth 2, priority 50 — emits FIRST (dist 2, index 0)
  //   b1 (2,1)->(2,3): head (2,2) vacancy-adjacent -> plain head task,
  //       depth 1, priority 50 — emits second
  //   robot (2,3): distance 1 to BOTH pickups ((1,3) and (2,2))
  // Without the depth key rho binds the deep ready task (emission order);
  // with it the depth-1 head must win.
  DDInstance ins;
  ins.grid = DDGrid({"....", "....", "...."});
  ins.robots = {ins.grid.idx(2, 3)};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      if (!(r == 2 && c == 3)) ins.shelves.push_back(ins.grid.idx(r, c));
  ins.target_starts = {ins.grid.idx(0, 1), ins.grid.idx(2, 1)};
  ins.target_goals = {ins.grid.idx(0, 3), ins.grid.idx(2, 3)};
  ins.finalize();
  const auto X = initial_phys_config(ins);
  std::vector<int> rho_task;
  const auto tasks = dd_build_tasks(ins, X, &rho_task);
  int heads50 = 0, min_depth = INT_MAX;
  for (const auto& t : tasks)
    if (t.priority == 50) {
      ++heads50;
      min_depth = std::min(min_depth, t.depth);
    }
  ASSERT_EQ(heads50, 2) << "fixture must produce two equal-priority heads";
  ASSERT_EQ(min_depth, 1);
  ASSERT_GE(rho_task[0], 0);
  EXPECT_EQ(tasks[rho_task[0]].depth, min_depth)
      << "equal priority + equal distance must bind the shallower task";
}
