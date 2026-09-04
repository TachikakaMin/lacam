// Carrier first-incumbent cost/accounting checks.
//
// Contract:
//  1. plans remain legal after macro extraction and output repair;
//  2. best_soc equals independent replay of the returned plan;
//  3. numeric objective inputs affect both search and reporting with the
//     same weights.
//
// Each carrier search pass stops at its first executable incumbent and
// repairs that plan structurally. Multi-goal inputs may use the remaining
// deadline for one fixed-assignment restart. The entry point intentionally
// does not promise the brute-force optimum or a duplicate-relax event.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>
#include <tapf_planner.hpp>

#include <functional>
#include <map>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

std::string key_of(const PhysConfig& X)
{
  std::string k;
  for (int v : X.robots) k += std::to_string(v) + ",";
  k += "|";
  for (int v : X.target_pos) k += std::to_string(v) + ",";
  k += "|";
  for (int v : X.anon_occ) k += std::to_string(v) + ",";
  k += "|";
  for (int v : X.kappa) k += std::to_string(v) + ",";
  return k;
}

double edge_cost(const PhysConfig& X, const std::vector<Op>& ops,
                 double alpha = 1, double beta = 1, double gamma = 1,
                 double delta = 1)
{
  double c = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
    if (ops[i].kind == Op::MOVE) {
      const bool loaded = X.kappa[i] != KAPPA_FREE;
      c += loaded ? alpha : beta;
      if (X.kappa[i] == KAPPA_ANON) c += delta;
    } else if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) {
      c += gamma;
    }
  }
  return c;
}

// NOTE: the unit-weight overloads below keep the original semantics
// (alpha=beta -> per-move 1 regardless of loadedness).

double brute_optimal_w(const DDInstance& ins, double a, double b2, double g2,
                       double d2, int cap = 3000000);

double brute_optimal(const DDInstance& ins, int cap = 3000000)
{
  return brute_optimal_w(ins, 1, 1, 1, 1, cap);
}

double brute_optimal_w(const DDInstance& ins, double A, double B, double G,
                       double D, int cap)
{
  const size_t R = ins.n_robots();
  auto X0 = initial_phys_config(ins);
  using PQE = std::pair<double, std::string>;
  std::map<std::string, double> dist;
  std::map<std::string, PhysConfig> cfg;
  std::priority_queue<PQE, std::vector<PQE>, std::greater<PQE>> pq;
  auto k0 = key_of(X0);
  dist[k0] = 0;
  cfg[k0] = X0;
  pq.push({0, k0});
  int expanded = 0;
  while (!pq.empty()) {
    auto [d, k] = pq.top();
    pq.pop();
    if (d > dist[k] + 1e-9) continue;
    const PhysConfig X = cfg[k];
    if (is_dd_goal(ins, X)) return d;
    if (++expanded > cap) return -1;
    std::vector<std::vector<Op>> raw(R);
    for (size_t i = 0; i < R; ++i) {
      raw[i].push_back(Op::make_wait());
      int nb[4];
      const int n = ins.grid.neighbors(X.robots[i], nb);
      for (int t = 0; t < n; ++t) raw[i].push_back(Op::make_move(nb[t]));
      raw[i].push_back(Op::make_lift());
      raw[i].push_back(Op::make_drop());
    }
    std::vector<Op> ops(R, Op::make_wait());
    std::function<void(size_t)> rec = [&](size_t i) {
      if (i == R) {
        auto nxt = apply_ops(ins, X, ops);
        if (!nxt.has_value()) return;
        auto nk = key_of(*nxt);
        const double nd = d + edge_cost(X, ops, A, B, G, D);
        auto it = dist.find(nk);
        if (it == dist.end() || nd < it->second - 1e-9) {
          dist[nk] = nd;
          cfg[nk] = *nxt;
          pq.push({nd, nk});
        }
        return;
      }
      for (const Op& op : raw[i]) {
        ops[i] = op;
        rec(i + 1);
      }
    };
    rec(0);
  }
  return -2;
}

DDInstance random_3x3(int seed)
{
  std::mt19937 rng(seed);
  DDInstance ins;
  ins.grid = DDGrid({"...", "...", "..."});
  std::vector<int> cells(9);
  for (int i = 0; i < 9; ++i) cells[i] = i;
  std::shuffle(cells.begin(), cells.end(), rng);
  ins.robots = {cells[0], cells[1]};
  const int nt = 1 + (int)(rng() % 2);
  std::vector<int> scells = {cells[2], cells[3], cells[4]};
  std::vector<int> gcands = {cells[5], cells[6]};
  for (int i = 0; i < nt; ++i) {
    ins.target_starts.push_back(scells[i]);
    ins.target_goals.push_back(gcands[i]);
  }
  for (int c : scells) ins.shelves.push_back(c);
  ins.finalize();
  return ins;
}

}  // namespace

TEST(dd_rewire, first_incumbent_family_is_valid_and_costed)
{
  int checked = 0;
  for (int seed = 0; seed < 12; ++seed) {
    auto ins = random_3x3(seed);
    const double opt = brute_optimal(ins);
    if (opt < 0) continue;  // infeasible or capped: skip
    ++checked;
    DDStats st;
    auto plan = solve_carrier_lacam(ins, 1.0, 0, &st);
    ASSERT_FALSE(plan.empty()) << "seed " << seed;
    PhysConfig X = initial_phys_config(ins);
    double replayed = 0;
    for (const auto& ops : plan) {
      replayed += edge_cost(X, ops);
      auto nxt = apply_ops(ins, X, ops);
      ASSERT_TRUE(nxt.has_value()) << "returned plan illegal at seed " << seed;
      X = *nxt;
    }
    EXPECT_TRUE(is_dd_goal(ins, X));
    EXPECT_NEAR(st.best_soc, replayed, 1e-6) << "seed " << seed;
    EXPECT_GE(st.best_soc + 1e-6, opt) << "seed " << seed;
    if (st.first_solution_soc >= 0) {
      EXPECT_LE(st.best_soc, st.first_solution_soc + 1e-6)
          << "output repair increased cost at seed " << seed;
    }
  }
  EXPECT_GE(checked, 6) << "family degenerated";
}

// Non-unit weights must be used consistently by the solver objective and
// the returned-plan accounting.
TEST(dd_rewire, weighted_objective_matches_replayed_plan_cost)
{
  setenv("DD_ALPHA", "2", 1);
  setenv("DD_BETA", "1", 1);
  setenv("DD_GAMMA", "5", 1);
  setenv("DD_DELTA", "3", 1);
  int checked = 0;
  for (int seed = 0; seed < 10; ++seed) {
    auto ins = random_3x3(seed);
    const double opt = brute_optimal_w(ins, 2, 1, 5, 3, 3000000);
    if (opt < 0) continue;
    ++checked;
    DDStats st;
    auto plan = solve_carrier_lacam(ins, 1.0, 0, &st);
    ASSERT_FALSE(plan.empty()) << "seed " << seed;
    auto X = initial_phys_config(ins);
    double replayed = 0;
    for (const auto& ops : plan) {
      replayed += edge_cost(X, ops, 2, 1, 5, 3);
      auto next = apply_ops(ins, X, ops);
      ASSERT_TRUE(next.has_value()) << "seed " << seed;
      X = *next;
    }
    EXPECT_TRUE(is_dd_goal(ins, X));
    EXPECT_NEAR(st.best_soc, replayed, 1e-6) << "seed " << seed;
    EXPECT_GE(st.best_soc + 1e-6, opt) << "seed " << seed;
  }
  unsetenv("DD_ALPHA");
  unsetenv("DD_BETA");
  unsetenv("DD_GAMMA");
  unsetenv("DD_DELTA");
  EXPECT_GE(checked, 5);
}

// review fix batch 2026-09-01 (TDD RED): DD_ALPHA..DD_DELTA are numeric
// objective inputs. Negative values break the non-negative edge-cost and
// admissible-LB assumptions (and bypass the matching-encoding overflow
// check, which only guards the upper bound); NaN/inf poison comparisons.
// The ONE shared parser must reject them loudly.
TEST(dd_weights, rejects_negative_and_non_finite_env_weights)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {ins.grid.idx(1, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();

  const char* keys[] = {"DD_ALPHA", "DD_BETA", "DD_GAMMA", "DD_DELTA"};
  const char* bad[] = {"-1", "nan", "inf", "-0.5"};
  for (const char* key : keys) {
    for (const char* value : bad) {
      setenv(key, value, 1);
      EXPECT_THROW(dd_root_admissible_h(ins), std::invalid_argument)
          << key << "=" << value;
      unsetenv(key);
    }
  }
  // valid non-unit weights still load (regression guard for the fix)
  setenv("DD_GAMMA", "2.5", 1);
  EXPECT_NO_THROW(dd_root_admissible_h(ins));
  unsetenv("DD_GAMMA");
}

// v3.0 step 3 (design_final §6.1, debug.md invariant 21; TDD RED): a
// duplicate node rewired to a cheaper parent must rebuild its guidance
// (hysteresis anchor, tasks, rho) instead of keeping the old parent's.
// Reparenting requires duplicate g-relaxations; in the stop-at-first
// production pass those are empirically absent (80-fixture scan, 68-case
// gate: zero), so the hook is exercised through the ANYTIME entry of the
// same solve loop, where dense shuffles relax heavily (deterministic:
// dense 3x4 fixture, solver seed 0).
TEST(dd_rewire, duplicate_rewire_rebuilds_guidance)
{
  std::mt19937 gen(0);
  DDInstance ins;
  ins.grid = DDGrid({"....", "....", "...."});
  std::vector<int> cells(12);
  for (int i = 0; i < 12; ++i) cells[i] = i;
  std::shuffle(cells.begin(), cells.end(), gen);
  ins.robots = {cells[0], cells[1]};
  for (int i = 2; i < 8; ++i) ins.shelves.push_back(cells[i]);  // 6 shelves
  ins.target_starts = {cells[2], cells[3]};
  ins.target_goals = {cells[8], cells[9]};
  ins.finalize();

  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  Deadline deadline(1000);
  TAPFStats st;
  TAPFSearchConfig cfg;  // anytime: keeps searching past the incumbent
  TAPFPlanner planner(&view, &deadline, &mt, 0, 0, 0.001f, true, &st, cfg);
  const auto sol = planner.solve();
  ASSERT_FALSE(sol.empty());
  ASSERT_GT(st.g_relaxed, 0) << "fixture stopped producing rewires";
  EXPECT_GT(st.rewire_guidance_rebuilds, 0)
      << "no duplicate-rewire guidance rebuild fired";
  // the rebuilt guidance must not corrupt the returned plan
  const auto plan = derive_carrier_ops(view, sol, planner.solution_shelves);
  auto X = initial_phys_config(ins);
  for (const auto& ops : plan) {
    auto nxt = apply_ops(ins, X, ops);
    ASSERT_TRUE(nxt.has_value());
    X = *nxt;
  }
  EXPECT_TRUE(is_dd_goal(ins, X));
}

// 2026-09-02 R3 (debug.md §10, TDD RED): rewrite() relaxes and reparents
// whole descendant chains, not just the duplicate-hit node.  Every node
// whose PARENT CHANGES during the relaxation must be marked
// guidance_stale so its next expansion re-anchors hysteresis/tasks/rho
// on the new parent (invariant 21).  Direct unit construction: C hangs
// under B_old; a cheaper route through B_new reparents C, which must
// come out stale.
TEST(dd_rewire, rewrite_marks_reparented_descendants_stale)
{
  DDInstance dd;
  dd.grid = DDGrid({"...", "..."});
  dd.robots = {dd.grid.idx(1, 0)};
  dd.shelves = {dd.grid.idx(0, 0)};
  dd.target_starts = {dd.grid.idx(0, 0)};
  dd.target_goals = {dd.grid.idx(0, 2)};
  dd.finalize();
  const TAPFInstance view(dd);
  std::mt19937 mt(0);
  TAPFStats st;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &st);

  const auto S0 = initial_shelf_state(view);
  auto mk_node = [&](Config c) {
    return new TAPFNode(c, S0, planner.D, &view,
                        std::vector<int>((int)view.N, -1),
                        TAPFAssignmentState(), nullptr);
  };
  Config c_root{view.G.U[dd.grid.idx(1, 0)]};
  Config c_mid{view.G.U[dd.grid.idx(1, 1)]};
  Config c_leaf{view.G.U[dd.grid.idx(1, 2)]};
  auto* root = mk_node(c_root);
  auto* b_old = mk_node(c_mid);
  auto* b_new = mk_node(c_mid);
  auto* leaf = mk_node(c_leaf);
  const PhysConfig X_root = initial_phys_config(dd);
  auto X_mid = X_root;
  X_mid.robots[0] = dd.grid.idx(1, 1);
  auto X_leaf = X_root;
  X_leaf.robots[0] = dd.grid.idx(1, 2);
  const std::vector<Op> move_mid = {
      Op::make_move(dd.grid.idx(1, 1))};
  const std::vector<Op> move_leaf = {
      Op::make_move(dd.grid.idx(1, 2))};
  ASSERT_EQ(apply_ops(dd, X_root, move_mid), X_mid);
  ASSERT_EQ(apply_ops(dd, X_mid, move_leaf), X_leaf);

  planner.attach_carrier_guidance(root);
  planner.attach_carrier_guidance(
      b_old, &X_root, root->guide.get(), &move_mid);
  planner.attach_carrier_guidance(
      b_new, &X_root, root->guide.get(), &move_mid);
  planner.attach_carrier_guidance(
      leaf, &X_mid, b_old->guide.get(), &move_leaf);
  b_old->parent = root;
  b_old->incoming_edge = planner.register_outgoing_edge(
      root, b_old, 1, {TransitionStep{X_root, move_mid, X_mid}});
  b_new->parent = root;
  b_new->incoming_edge = planner.register_outgoing_edge(
      root, b_new, 1, {TransitionStep{X_root, move_mid, X_mid}});
  leaf->parent = b_old;
  leaf->incoming_edge = planner.register_outgoing_edge(
      b_old, leaf, 1, {TransitionStep{X_mid, move_leaf, X_leaf}});
  const auto candidate = planner.register_outgoing_edge(
      b_new, leaf, 1, {TransitionStep{X_mid, move_leaf, X_leaf}});
  root->g = 0;
  b_old->g = 50;
  b_new->g = 1;
  leaf->g = 100;
  ASSERT_FALSE(leaf->guidance_stale);
  ASSERT_NE(leaf->guide, nullptr);

  std::vector<TAPFNode*> OPEN;
  planner.rewrite(b_new, nullptr, OPEN);
  EXPECT_EQ(leaf->parent, b_new) << "relaxation must reparent the leaf";
  EXPECT_EQ(leaf->incoming_edge, candidate);
  EXPECT_DOUBLE_EQ(leaf->g, 2);
  EXPECT_DOUBLE_EQ(leaf->f, leaf->g + leaf->h);
  EXPECT_TRUE(leaf->guidance_stale)
      << "a reparented descendant must be marked for guidance re-anchor";

  delete leaf;
  delete b_new;
  delete b_old;
  delete root;
}

// 2026-09-02 R4(a) (debug.md §10, TDD RED): atof silently parses "abc"
// as 0 — a valid-looking weight that silently zeroes the objective.  The
// ONE parser must reject strings that are not fully consumed as a finite
// non-negative number.
TEST(dd_weights, rejects_unparseable_weight_strings)
{
  DDInstance ins;
  ins.grid = DDGrid({"...", "..."});
  ins.robots = {ins.grid.idx(1, 0)};
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();
  const char* bad[] = {"abc", "1x", "", " ", "1.0.0"};
  for (const char* value : bad) {
    setenv("DD_ALPHA", value, 1);
    EXPECT_THROW(dd_root_admissible_h(ins), std::invalid_argument)
        << "DD_ALPHA='" << value << "' must be rejected";
    unsetenv("DD_ALPHA");
  }
  setenv("DD_ALPHA", "  2.5 ", 1);  // strtod skips leading spaces; allow
  EXPECT_NO_THROW(dd_root_admissible_h(ins));
  unsetenv("DD_ALPHA");
}

// 2026-09-02 round-3 S2 (debug.md §11, TDD RED): a relaxation that
// lowers a node's g through its UNCHANGED parent still means the
// upstream ancestry (and hence the guidance anchor chain) changed — the
// reviewer's leak: duplicate reparented+stale, but its child (parent
// pointer unchanged, g lowered) kept stale=0.  ANY relaxed node with a
// guide must come out stale.
TEST(dd_rewire, relaxed_child_with_unchanged_parent_is_stale)
{
  DDInstance dd;
  dd.grid = DDGrid({"...", "..."});
  dd.robots = {dd.grid.idx(1, 0)};
  dd.shelves = {dd.grid.idx(0, 0)};
  dd.target_starts = {dd.grid.idx(0, 0)};
  dd.target_goals = {dd.grid.idx(0, 2)};
  dd.finalize();
  const TAPFInstance view(dd);
  std::mt19937 mt(0);
  TAPFStats st;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &st);

  const auto S0 = initial_shelf_state(view);
  auto mk_node = [&](Config c) {
    return new TAPFNode(c, S0, planner.D, &view,
                        std::vector<int>((int)view.N, -1),
                        TAPFAssignmentState(), nullptr);
  };
  Config c_root{view.G.U[dd.grid.idx(1, 0)]};
  Config c_mid{view.G.U[dd.grid.idx(1, 1)]};
  Config c_leaf{view.G.U[dd.grid.idx(1, 2)]};
  auto* root = mk_node(c_root);
  auto* dup = mk_node(c_mid);
  auto* child = mk_node(c_leaf);
  const PhysConfig X_root = initial_phys_config(dd);
  auto X_mid = X_root;
  X_mid.robots[0] = dd.grid.idx(1, 1);
  auto X_leaf = X_root;
  X_leaf.robots[0] = dd.grid.idx(1, 2);
  const std::vector<Op> move_mid = {
      Op::make_move(dd.grid.idx(1, 1))};
  const std::vector<Op> move_leaf = {
      Op::make_move(dd.grid.idx(1, 2))};
  ASSERT_EQ(apply_ops(dd, X_root, move_mid), X_mid);
  ASSERT_EQ(apply_ops(dd, X_mid, move_leaf), X_leaf);

  planner.attach_carrier_guidance(root);
  planner.attach_carrier_guidance(
      dup, &X_root, root->guide.get(), &move_mid);
  planner.attach_carrier_guidance(
      child, &X_mid, dup->guide.get(), &move_leaf);
  dup->parent = root;
  dup->incoming_edge = planner.register_outgoing_edge(
      root, dup, 1, {TransitionStep{X_root, move_mid, X_mid}});
  child->parent = dup;
  child->incoming_edge = planner.register_outgoing_edge(
      dup, child, 1, {TransitionStep{X_mid, move_leaf, X_leaf}});
  root->g = 0;
  dup->g = 50;    // will be relaxed via root
  child->g = 100;  // will be relaxed via dup (parent unchanged)
  ASSERT_FALSE(child->guidance_stale);

  std::vector<TAPFNode*> OPEN;
  planner.rewrite(root, nullptr, OPEN);
  EXPECT_LT(dup->g, 50) << "dup must be relaxed";
  EXPECT_LT(child->g, 100) << "child must be relaxed through dup";
  EXPECT_EQ(child->parent, dup) << "child's parent pointer is unchanged";
  EXPECT_TRUE(child->guidance_stale)
      << "an ancestry change via relaxation must mark the child stale";

  delete child;
  delete dup;
  delete root;
}
