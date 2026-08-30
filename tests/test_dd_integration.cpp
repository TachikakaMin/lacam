// PROTECTED integration tests (debug.md v3): the DD/carrier mechanisms
// as INCREMENTAL extensions of the LaCAM-TAPF execution path.  Grows one
// WP at a time (WP1 instance layer, WP2 state/goal/cost, WP3 generator,
// WP4 guidance, WP5 entry adapters).
#include <lacam.hpp>

#include "gtest/gtest.h"

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
