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
