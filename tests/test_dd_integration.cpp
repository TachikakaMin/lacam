// PROTECTED integration tests (debug.md v3): the DD/carrier mechanisms
// as INCREMENTAL extensions of the LaCAM-TAPF execution path.  Grows one
// WP at a time (WP1 instance layer, WP2 state/goal/cost, WP3 generator,
// WP4 guidance, WP5 entry adapters).
#include <lacam.hpp>

#include "gtest/gtest.h"

#include <dd_planner.hpp>

#ifndef DD_TEST_DIR
#define DD_TEST_DIR "./tests"
#endif

namespace {
const std::string kFix = std::string(DD_TEST_DIR) + "/fixtures/";
}

// ---------- WP1: instance / graph layer (mapping M1) ----------

TEST(dd_integration, graph_inline_rows_ctor_matches_file_semantics)
{
  // 3x4 grid, one wall at (1,2)
  const Graph g(std::vector<std::string>{"....", "..@.", "...."});
  EXPECT_EQ(g.width, 4);
  EXPECT_EQ(g.height, 3);
  EXPECT_EQ(g.size(), 11);          // 12 cells - 1 wall
  ASSERT_EQ((int)g.U.size(), 12);
  EXPECT_EQ(g.U[4 * 1 + 2], nullptr);  // wall cell
  ASSERT_NE(g.U[0], nullptr);

  // neighbor ORDER must match the .map file loader exactly (left, right,
  // y+1, y-1) — determinism contract for PIBT tie-breaking.
  const auto* v = g.U[4 * 1 + 1];  // (r=1,c=1): left (1,0), right wall,
                                   // down (2,1), up (0,1)
  ASSERT_NE(v, nullptr);
  ASSERT_EQ(v->neighbor.size(), 3u);
  EXPECT_EQ(v->neighbor[0]->index, 4 * 1 + 0);
  EXPECT_EQ(v->neighbor[1]->index, 4 * 2 + 1);
  EXPECT_EQ(v->neighbor[2]->index, 4 * 0 + 1);
}

TEST(dd_integration, tapf_instance_from_dd_instance)
{
  const auto dd = load_dd_instance(kFix + "dd_tiny.yaml");
  const TAPFInstance ins(dd);

  // graph: 2x4 wall-free
  EXPECT_EQ(ins.G.width, 4);
  EXPECT_EQ(ins.G.height, 2);
  EXPECT_EQ(ins.G.size(), 8);

  // robots -> starts (Config over the graph)
  ASSERT_EQ(ins.N, 1u);
  ASSERT_EQ(ins.starts.size(), 1u);
  EXPECT_EQ(ins.starts[0]->index, 4 * 1 + 0);

  // no agent tasks: carriers only (N x 0 compatibility matrix — the
  // pre-existing N == allowed.size() invariant is kept; subagent-APPROVEd)
  EXPECT_TRUE(ins.tasks.empty());
  ASSERT_EQ(ins.allowed.size(), ins.N);
  for (const auto& row : ins.allowed) EXPECT_TRUE(row.empty());

  // shelf layer copied (cell = Vertex::index encoding)
  ASSERT_EQ(ins.shelf_cells.size(), 2u);
  EXPECT_EQ(ins.shelf_cells[0], 4 * 0 + 1);
  EXPECT_EQ(ins.shelf_cells[1], 4 * 1 + 3);
  ASSERT_EQ(ins.target_starts.size(), 1u);
  EXPECT_EQ(ins.target_starts[0], 4 * 0 + 1);
  ASSERT_EQ(ins.target_goals.size(), 1u);
  EXPECT_EQ(ins.target_goals[0], 4 * 0 + 3);

  // DD-form validity: no tasks required when targets exist
  EXPECT_TRUE(ins.is_valid());
}

TEST(dd_integration, tapf_instance_without_tasks_or_targets_stays_invalid)
{
  // a shelf-free TAPF instance with zero tasks must remain INVALID —
  // the relaxed rule is scoped to carrier instances (targets present).
  const TAPFInstance ins("./assets/empty-8-8.map", std::vector<int>{0},
                         std::vector<std::vector<int>>{{}});
  EXPECT_FALSE(ins.is_valid());
}

// ---------- WP2: state / key / goal / cost (mapping M2, M5) ----------

namespace {
DDInstance tiny_dd(bool target_at_goal)
{
  DDInstance dd;
  dd.grid = DDGrid({"....", "...."});
  dd.robots = {dd.grid.idx(1, 0)};
  dd.shelves = {dd.grid.idx(0, 1), dd.grid.idx(1, 3)};
  dd.target_starts = {dd.grid.idx(0, 1)};
  dd.target_goals = {target_at_goal ? dd.grid.idx(0, 1) : dd.grid.idx(0, 3)};
  dd.finalize();
  return dd;
}
}  // namespace

TEST(dd_integration, initial_shelf_state_mirrors_instance)
{
  const TAPFInstance ins(tiny_dd(false));
  const auto S = initial_shelf_state(ins);
  ASSERT_EQ(S.target_pos.size(), 1u);
  EXPECT_EQ(S.target_pos[0], 4 * 0 + 1);
  ASSERT_EQ(S.anon_occ.size(), 1u);  // the non-target shelf
  EXPECT_EQ(S.anon_occ[0], 4 * 1 + 3);
  ASSERT_EQ(S.kappa.size(), 1u);
  EXPECT_EQ(S.kappa[0], KAPPA_FREE);

  // shelf-free instances carry an EMPTY layer (natural degradation)
  const TAPFInstance plain(
      "./assets/empty-8-8.map", std::vector<int>{0},
      std::vector<std::vector<int>>{{63}});
  const auto S0 = initial_shelf_state(plain);
  EXPECT_TRUE(S0.target_pos.empty());
  EXPECT_TRUE(S0.anon_occ.empty());
  EXPECT_TRUE(S0.kappa.empty());
}

TEST(dd_integration, carrier_goal_condition_requires_grounded)
{
  // D10: a target CARRIED on its goal cell is not a goal state
  const TAPFInstance ins(tiny_dd(false));
  TAPFPlanner planner(&ins, nullptr, nullptr);

  auto S = initial_shelf_state(ins);
  Config C = ins.starts;

  // not at goal yet
  EXPECT_FALSE(planner.is_goal_config(C, S));

  // grounded at goal -> goal
  S.target_pos[0] = ins.target_goals[0];
  EXPECT_TRUE(planner.is_goal_config(C, S));

  // carried at goal -> NOT goal
  S.kappa[0] = 0;
  EXPECT_FALSE(planner.is_goal_config(C, S));
}

TEST(dd_integration, solve_tapf_trivial_carrier_instance_single_step)
{
  // target already grounded at its goal: the root configuration is the
  // goal, so the ORIGINAL TAPF loop must return a size-1 solution.
  const TAPFInstance ins(tiny_dd(true));
  ASSERT_TRUE(ins.is_valid());
  std::mt19937 mt(0);
  const auto sol = solve_tapf(ins, 0, nullptr, &mt);
  ASSERT_EQ(sol.size(), 1u);
  EXPECT_TRUE(is_same_config(sol.front(), ins.starts));
}

// ---------- WP3: operator generator (mapping M3/M4) ----------

namespace {
// replay a derived op plan through the CONFORMANCE ORACLE (dd_carrier
// apply_ops) and require goal at the end.
void expect_oracle_valid_plan(const DDInstance& dd,
                              const std::vector<std::vector<Op>>& plan,
                              bool expect_goal = true)
{
  auto s = initial_phys_config(dd);
  for (const auto& ops : plan) {
    auto nxt = apply_ops(dd, s, ops);
    ASSERT_TRUE(nxt.has_value()) << "oracle rejected a step";
    s = *nxt;
  }
  if (expect_goal) EXPECT_TRUE(is_dd_goal(dd, s));
}

// solve a DD instance THROUGH the TAPF planner and return the derived
// per-timestep joint ops (empty on failure)
std::vector<std::vector<Op>> solve_dd_via_tapf(const DDInstance& dd, int seed,
                                               double limit_sec = 10.0)
{
  const TAPFInstance ins(dd);
  std::mt19937 mt(seed);
  Deadline dl(limit_sec * 1000);
  TAPFPlanner planner(&ins, &dl, &mt);
  const auto sol = planner.solve();
  if (sol.empty()) return {};
  EXPECT_EQ(planner.solution_shelves.size(), sol.size());
  return derive_carrier_ops(ins, sol, planner.solution_shelves);
}
}  // namespace

TEST(dd_integration, derive_carrier_ops_detects_lift_move_drop)
{
  const auto dd = tiny_dd(false);
  const TAPFInstance ins(dd);
  // hand-built two-step chain: robot walks (1,0)->(0,0), then (0,0)->(0,1)
  Solution sol;
  std::vector<ShelfState> shelves;
  Config c0 = ins.starts;
  Config c1{ins.G.U[0]};
  Config c2{ins.G.U[1]};
  auto s0 = initial_shelf_state(ins);
  auto s2 = s0;
  sol = {c0, c1, c2};
  shelves = {s0, s0, s2};
  auto plan = derive_carrier_ops(ins, sol, shelves);
  ASSERT_EQ(plan.size(), 2u);
  EXPECT_EQ(plan[0][0], Op::make_move(0));
  EXPECT_EQ(plan[1][0], Op::make_move(1));

  // lift in place: same cell, kappa FREE -> target 0
  auto s_lift = s0;
  s_lift.kappa[0] = 0;
  plan = derive_carrier_ops(ins, {c2, c2}, {s0, s_lift});
  ASSERT_EQ(plan.size(), 1u);
  EXPECT_EQ(plan[0][0], Op::make_lift());

  // drop in place: kappa target 0 -> FREE
  plan = derive_carrier_ops(ins, {c2, c2}, {s_lift, s0});
  ASSERT_EQ(plan.size(), 1u);
  EXPECT_EQ(plan[0][0], Op::make_drop());

  // pure wait
  plan = derive_carrier_ops(ins, {c2, c2}, {s0, s0});
  ASSERT_EQ(plan.size(), 1u);
  EXPECT_EQ(plan[0][0], Op::make_wait());
}

TEST(dd_integration, solve_tapf_carries_target_to_goal)
{
  // one robot, target (0,1) -> (0,3), free upper path: needs approach,
  // lift, two loaded moves, drop — pure operator-tree search, no guidance
  const auto dd = tiny_dd(false);
  const auto plan = solve_dd_via_tapf(dd, 0);
  ASSERT_FALSE(plan.empty());
  expect_oracle_valid_plan(dd, plan);
}

TEST(dd_integration, solve_tapf_clears_blocker_on_path)
{
  // anonymous blocker sits ON the target's only row-0 path: the robot
  // must relocate it (lift/carry/drop) before delivering the target
  DDInstance dd;
  dd.grid = DDGrid({"....", "...."});
  dd.robots = {dd.grid.idx(1, 0)};
  dd.shelves = {dd.grid.idx(0, 1), dd.grid.idx(0, 2)};
  dd.target_starts = {dd.grid.idx(0, 1)};
  dd.target_goals = {dd.grid.idx(0, 3)};
  dd.finalize();

  const auto plan = solve_dd_via_tapf(dd, 0);
  ASSERT_FALSE(plan.empty());
  expect_oracle_valid_plan(dd, plan);
}

TEST(dd_integration, solve_tapf_two_robots_two_targets)
{
  // two robots, two labeled targets crossing on a 3x4 grid
  DDInstance dd;
  dd.grid = DDGrid({"....", "....", "...."});
  dd.robots = {dd.grid.idx(1, 0), dd.grid.idx(1, 3)};
  dd.shelves = {dd.grid.idx(0, 0), dd.grid.idx(2, 3)};
  dd.target_starts = {dd.grid.idx(0, 0), dd.grid.idx(2, 3)};
  dd.target_goals = {dd.grid.idx(0, 3), dd.grid.idx(2, 0)};
  dd.finalize();

  const auto plan = solve_dd_via_tapf(dd, 0);
  ASSERT_FALSE(plan.empty());
  expect_oracle_valid_plan(dd, plan);
}

// ---------- WP4: guidance stack (mapping M6/M7/M8/M9) ----------

TEST(dd_integration, solve_tapf_m1_scale_fixture_within_10s)
{
  // design.md M1 exit criterion instance (20x20, 52 shelves, 10 robots,
  // 13 targets): needs the full guidance stack (requests/rho/paths);
  // blind operator search cannot touch this
  const auto dd =
      load_dd_instance(kFix + "m1_exit_20x20_s52_r10.yaml");
  const auto plan = solve_dd_via_tapf(dd, 0, 10.0);
  ASSERT_FALSE(plan.empty());
  expect_oracle_valid_plan(dd, plan);
}

// ---------- WP6 regression: allocation-history purity ----------
//
// The pre-integration planner arena-allocated its nodes, so pointer-keyed
// occupancy scratches were never fooled.  The integrated planner FREES
// rollout/B1 probe nodes mid-solve; the allocator then reuses their
// addresses for later nodes.  Guidance and the PIBT occupancy scratch
// must therefore never treat "same node address" as "same node" — the
// generation for a configuration X must be identical regardless of how
// many nodes lived and died before it (benchmark symptom: nondeterministic
// 40x makespan swings on dense ddmapd under different heap histories).
TEST(dd_integration, generation_is_pure_under_heap_churn)
{
  // dense-ish fixture: robot on a shelf cell, one blocker on the path
  DDInstance dd;
  dd.grid = DDGrid({".....", ".....", "....."});
  dd.robots = {dd.grid.idx(1, 1), dd.grid.idx(2, 4)};
  dd.shelves = {dd.grid.idx(1, 1), dd.grid.idx(1, 2), dd.grid.idx(1, 3)};
  dd.target_starts = {dd.grid.idx(1, 1)};
  dd.target_goals = {dd.grid.idx(1, 4)};
  dd.finalize();
  const TAPFInstance view(dd);

  // reference: fresh planner, single node for X0
  const auto reference = dd_root_joint_ops(dd, initial_phys_config(dd), 7);
  ASSERT_FALSE(reference.empty());

  // churned: same planner first processes a DIFFERENT configuration
  // (different occupancy!) through nodes that are then freed; the node
  // for X0 is allocated afterwards (typically at a recycled address).
  std::mt19937 mt(7);
  TAPFStats st;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &st);
  auto X_other = initial_phys_config(dd);
  // move the anonymous blockers elsewhere: occupancy differs from X0
  X_other.anon_occ = {dd.grid.idx(0, 0), dd.grid.idx(2, 0)};
  for (int burn = 0; burn < 3; ++burn) {
    Config C_o;
    for (const int cell : X_other.robots) C_o.push_back(view.G.U[cell]);
    ShelfState S_o;
    S_o.target_pos = X_other.target_pos;
    S_o.anon_occ = X_other.anon_occ;
    S_o.kappa = X_other.kappa;
    auto tmp = std::make_unique<TAPFNode>(C_o, S_o, planner.D, &view,
                                          std::vector<int>(2, -1),
                                          TAPFAssignmentState(), nullptr);
    planner.attach_carrier_guidance(tmp.get());
    TAPFConstraint root;
    ASSERT_TRUE(planner.get_new_config(tmp.get(), &root));
    // tmp destroyed here -> its address returns to the allocator
  }
  // now generate for X0 on the SAME planner (fresh node, likely at a
  // recycled address) — must match the fresh-planner reference exactly
  const auto X0 = initial_phys_config(dd);
  Config C0;
  for (const int cell : X0.robots) C0.push_back(view.G.U[cell]);
  ShelfState S0;
  S0.target_pos = X0.target_pos;
  S0.anon_occ = X0.anon_occ;
  S0.kappa = X0.kappa;
  auto node = std::make_unique<TAPFNode>(C0, S0, planner.D, &view,
                                         std::vector<int>(2, -1),
                                         TAPFAssignmentState(), nullptr);
  planner.attach_carrier_guidance(node.get());
  {
    std::mt19937 mt_ref(7);  // align the tie-breaker stream with the
    planner.MT = &mt_ref;    // fresh-planner reference call
    TAPFConstraint root;
    ASSERT_TRUE(planner.get_new_config(node.get(), &root));
    ASSERT_TRUE(planner.apply_carrier_effects(node.get()));
    planner.MT = &mt;
  }
  ASSERT_EQ(planner.ops_scratch.size(), reference.size());
  for (size_t i = 0; i < reference.size(); ++i)
    EXPECT_TRUE(planner.ops_scratch[i] == reference[i]) << "robot " << i;
}

// Every rollout state must attach from the immediately preceding real
// transition.  Same-U steps may share immutable upper-epoch data, but
// custody/ready/rho are always rebuilt; loaded Move recompiles the epoch.
TEST(dd_integration, rollout_steps_match_fresh_generation)
{
  DDInstance dd;
  dd.grid = DDGrid({"......", "......", "......"});
  dd.robots = {dd.grid.idx(0, 0), dd.grid.idx(2, 0)};
  dd.shelves = {dd.grid.idx(0, 2), dd.grid.idx(1, 3), dd.grid.idx(2, 2)};
  dd.target_starts = {dd.grid.idx(0, 2), dd.grid.idx(2, 2)};
  dd.target_goals = {dd.grid.idx(0, 5), dd.grid.idx(2, 5)};
  dd.finalize();
  const TAPFInstance view(dd);

  // rolling trace on ONE planner (probe nodes recycle heap addresses)
  TAPFStats st;
  TAPFPlanner roller(&view, nullptr, nullptr, 0, 0, 0.001f, true, &st);
  const auto C0 = view.starts;
  const auto S0 = initial_shelf_state(view);
  const auto r = roller.carrier_rollout(C0, S0, 12, 0, false);
  ASSERT_GE(r.ops.size(), 3u);
  ASSERT_NE(r.terminal_guidance, nullptr);

  auto physical = [](const Config& C, const ShelfState& S) {
    PhysConfig X;
    for (const auto* vertex : C) X.robots.push_back(vertex->index);
    X.target_pos = S.target_pos;
    X.anon_occ = S.anon_occ;
    X.kappa = S.kappa;
    return X;
  };
  auto expect_same_guidance = [](const CarrierGuidance& a,
                                 const CarrierGuidance& b) {
    ASSERT_NE(a.upper_epoch, nullptr);
    ASSERT_NE(b.upper_epoch, nullptr);
    EXPECT_EQ(a.upper_epoch->upper_signature,
              b.upper_epoch->upper_signature);
    EXPECT_EQ(a.upper_epoch->tau_guide, b.upper_epoch->tau_guide);
    EXPECT_EQ(a.upper_epoch->target_priority,
              b.upper_epoch->target_priority);
    ASSERT_EQ(a.upper_epoch->task_graph.tasks.size(),
              b.upper_epoch->task_graph.tasks.size());
    for (size_t task = 0;
         task < a.upper_epoch->task_graph.tasks.size(); ++task) {
      EXPECT_EQ(a.upper_epoch->task_graph.tasks[task].id,
                b.upper_epoch->task_graph.tasks[task].id);
      EXPECT_EQ(a.upper_epoch->task_graph.tasks[task].roots,
                b.upper_epoch->task_graph.tasks[task].roots);
      EXPECT_EQ(a.upper_epoch->task_graph.tasks[task].priority,
                b.upper_epoch->task_graph.tasks[task].priority);
    }
    EXPECT_EQ(a.upper_epoch->task_graph.predecessors,
              b.upper_epoch->task_graph.predecessors);
    EXPECT_EQ(a.upper_epoch->task_graph.successors,
              b.upper_epoch->task_graph.successors);
    EXPECT_EQ(a.ready_tasks, b.ready_tasks);
    EXPECT_EQ(a.rho_task_id, b.rho_task_id);
    EXPECT_EQ(a.rho_ready_index, b.rho_ready_index);
    ASSERT_EQ(a.custody_by_robot.size(),
              b.custody_by_robot.size());
    for (size_t robot = 0; robot < a.custody_by_robot.size(); ++robot) {
      ASSERT_EQ(a.custody_by_robot[robot].has_value(),
                b.custody_by_robot[robot].has_value());
      if (!a.custody_by_robot[robot].has_value()) continue;
      EXPECT_EQ(a.custody_by_robot[robot]->task_id,
                b.custody_by_robot[robot]->task_id);
      EXPECT_EQ(a.custody_by_robot[robot]->current_task_index,
                b.custody_by_robot[robot]->current_task_index);
      EXPECT_EQ(a.custody_by_robot[robot]->roots,
                b.custody_by_robot[robot]->roots);
      EXPECT_EQ(a.custody_by_robot[robot]->priority,
                b.custody_by_robot[robot]->priority);
    }
  };

  std::unique_ptr<CarrierGuidance> previous_guidance;
  PhysConfig previous_X;
  int loaded_moves = 0;
  for (size_t t = 0; t < r.ops.size(); ++t) {
    TAPFStats st2;
    TAPFPlanner fresh(&view, nullptr, nullptr, 0, 0, 0.001f, true, &st2);
    auto node = std::make_unique<TAPFNode>(
        r.configs[t], r.shelves[t], fresh.D, &view,
        std::vector<int>((int)view.N, -1), TAPFAssignmentState(), nullptr);
    if (t == 0) {
      fresh.attach_carrier_guidance(node.get());
    } else {
      fresh.attach_carrier_guidance(
          node.get(), &previous_X, previous_guidance.get(),
          &r.ops[t - 1]);
      for (size_t robot = 0; robot < view.N; ++robot) {
        const auto& previous_op = r.ops[t - 1][robot];
        if (previous_op.kind == Op::LIFT &&
            previous_guidance->rho_task_id[robot].has_value()) {
          ASSERT_TRUE(
              node->guide->custody_by_robot[robot].has_value());
          EXPECT_EQ(
              node->guide->custody_by_robot[robot]->task_id,
              *previous_guidance->rho_task_id[robot]);
        }
        if (previous_op.kind == Op::WAIT &&
            previous_X.kappa[robot] != KAPPA_FREE &&
            previous_guidance->custody_by_robot[robot].has_value()) {
          ASSERT_TRUE(
              node->guide->custody_by_robot[robot].has_value());
          EXPECT_EQ(
              node->guide->custody_by_robot[robot]->task_id,
              previous_guidance->custody_by_robot[robot]->task_id);
        }
        if (previous_op.kind == Op::MOVE &&
            previous_X.kappa[robot] != KAPPA_FREE) {
          ++loaded_moves;
          if (node->guide->custody_by_robot[robot].has_value() &&
              previous_guidance->custody_by_robot[robot].has_value())
            EXPECT_NE(
                node->guide->custody_by_robot[robot]->task_id,
                previous_guidance->custody_by_robot[robot]->task_id);
        }
        if (previous_op.kind == Op::DROP)
          EXPECT_FALSE(
              node->guide->custody_by_robot[robot].has_value());
      }
    }
    EXPECT_EQ(st2.guidance_builds, 1);
    previous_X = physical(r.configs[t], r.shelves[t]);
    previous_guidance =
        std::make_unique<CarrierGuidance>(*node->guide);

    TAPFConstraint root;
    ASSERT_TRUE(fresh.get_new_config(node.get(), &root)) << "step " << t;
    ASSERT_TRUE(fresh.apply_carrier_effects(node.get())) << "step " << t;
    for (size_t i = 0; i < view.N; ++i)
      EXPECT_TRUE(fresh.ops_scratch[i] == r.ops[t][i])
          << "step " << t << " robot " << i;
    Config generated(view.N, nullptr);
    for (const auto* agent : fresh.A)
      generated[agent->id] = agent->v_next;
    EXPECT_TRUE(is_same_config(generated, r.configs[t + 1]));
    EXPECT_EQ(fresh.shelf_next_scratch, r.shelves[t + 1]);
  }
  EXPECT_GE(loaded_moves, 2);

  TAPFStats terminal_stats;
  TAPFPlanner terminal(
      &view, nullptr, nullptr, 0, 0, 0.001f, true, &terminal_stats);
  auto terminal_node = std::make_unique<TAPFNode>(
      r.configs.back(), r.shelves.back(), terminal.D, &view,
      std::vector<int>((int)view.N, -1), TAPFAssignmentState(), nullptr);
  terminal.attach_carrier_guidance(
      terminal_node.get(), &previous_X, previous_guidance.get(),
      &r.ops.back());
  EXPECT_EQ(terminal_stats.guidance_builds, 1);
  expect_same_guidance(
      *terminal_node->guide, *r.terminal_guidance);
}

// Reservation semantics on carrier instances (design 5.4 / debug.md v3
// section 4 D1): with the upstream "keep the reservation on recursion
// failure" rule, one failed carrier push poisons every remaining
// candidate of the step, cascading into thousands of generator failures
// on dense instances — the SEARCH then returns plans an order of
// magnitude worse than its own guidance rollout (B0).  Anchor: on the
// dense d50 fixture, full search must not lose to plain B0 by more than
// 2x makespan (v5 evidence: search 76 vs B0 ~90; broken shape: 252+).
TEST(dd_integration, search_not_dominated_by_own_rollout_on_dense)
{
  const auto dd = load_dd_instance(kFix + "d50_16x16_r8_seed0.yaml");

  const auto search_plan = solve_dd_via_tapf(dd, 0, 10.0);
  ASSERT_FALSE(search_plan.empty());
  expect_oracle_valid_plan(dd, search_plan);

  const auto b0_plan = solve_carrier_rollout(dd, 10.0, 0);
  ASSERT_FALSE(b0_plan.empty());

  EXPECT_LE(search_plan.size(), 2 * b0_plan.size())
      << "search mk " << search_plan.size() << " vs B0 mk "
      << b0_plan.size();
}
