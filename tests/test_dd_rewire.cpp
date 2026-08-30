// LaCAM*-style duplicate relax/rewire (debug.md round-2 P2-14,
// design.md 4.3).
//
// Contract:
//  1. property: on a deterministic family of tiny 3x3 instances the
//     solver's final best_soc equals the brute-force optimum (Dijkstra
//     over the FULL validator-accepted transition graph);
//  2. the relax machinery actually fires somewhere on the family
//     (stats.g_relaxed > 0 summed) — guards against the counter/logic
//     silently dying;
//  3. plans stay valid after rewiring (extract_plan follows the updated
//     parent edges; validated via apply_ops replay).
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

double edge_cost(const PhysConfig& X, const std::vector<Op>& ops)
{
  double c = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
    if (ops[i].kind == Op::MOVE) {
      c += 1;
      if (X.kappa[i] == KAPPA_ANON) c += 1;
    } else if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) {
      c += 1;
    }
  }
  return c;
}

double brute_optimal(const DDInstance& ins, int cap = 3000000)
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
        const double nd = d + edge_cost(X, ops);
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

TEST(dd_rewire, tiny_family_matches_brute_optimum_and_relax_fires)
{
  long relax_total = 0;
  int checked = 0;
  for (int seed = 0; seed < 12; ++seed) {
    auto ins = random_3x3(seed);
    const double opt = brute_optimal(ins);
    if (opt < 0) continue;  // infeasible or capped: skip
    ++checked;
    DDStats st;
    auto plan = solve_carrier_lacam(ins, 1.0, 0, &st);
    ASSERT_FALSE(plan.empty()) << "seed " << seed;
    relax_total += st.g_relaxed;
    EXPECT_NEAR(st.best_soc, opt, 1e-6)
        << "seed " << seed << ": solver best_soc " << st.best_soc
        << " != brute optimum " << opt;
    // replay the extracted plan through the validator: rewired parent
    // edges must still reconstruct a legal plan
    PhysConfig X = initial_phys_config(ins);
    for (const auto& ops : plan) {
      auto nxt = apply_ops(ins, X, ops);
      ASSERT_TRUE(nxt.has_value()) << "rewired plan illegal at seed " << seed;
      X = *nxt;
    }
    EXPECT_TRUE(is_dd_goal(ins, X));
  }
  EXPECT_GE(checked, 6) << "family degenerated";
  EXPECT_GT(relax_total, 0)
      << "duplicate g-relax never fired across the family — rewire "
         "machinery dead (or DDStats.g_relaxed not wired)";
}
