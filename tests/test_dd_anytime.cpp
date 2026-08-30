// PROTECTED tests: physical g, admissible h, FOCAL/anytime after first
// solution, f-pruning (design 5.7 / D5; debug.md P3).  TDD RED first.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <functional>
#include <map>
#include <queue>
#include <string>

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

double plan_soc(const DDInstance& ins, const DDPlan& plan)
{
  auto s = initial_phys_config(ins);
  double soc = 0;
  for (const auto& ops : plan) {
    for (size_t i = 0; i < ops.size(); ++i) {
      if (ops[i].kind == Op::MOVE) soc += 1;  // alpha=beta=1
      if (ops[i].kind == Op::MOVE && s.kappa[i] == KAPPA_ANON) soc += 1;  // d
      if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) soc += 1;  // g
    }
    auto nxt = apply_ops(ins, s, ops);
    if (!nxt.has_value()) ADD_FAILURE() << "illegal plan";
    s = *nxt;
  }
  return soc;
}

// brute-force optimal weighted SOC via uniform-cost search on full joint
// operator space (tiny instances only)
double optimal_soc(const DDInstance& ins)
{
  const size_t R = ins.n_robots();
  auto key_of = [](const PhysConfig& X) {
    std::string k;
    for (int v : X.robots) k += std::to_string(v) + ",";
    for (int v : X.target_pos) k += ";" + std::to_string(v);
    for (int v : X.anon_occ) k += "|" + std::to_string(v);
    for (int v : X.kappa) k += "^" + std::to_string(v);
    return k;
  };
  using QE = std::pair<double, std::string>;
  std::map<std::string, PhysConfig> configs;
  std::map<std::string, double> dist;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  auto X0 = initial_phys_config(ins);
  configs[key_of(X0)] = X0;
  dist[key_of(X0)] = 0;
  pq.push({0, key_of(X0)});
  while (!pq.empty()) {
    auto [d, k] = pq.top();
    pq.pop();
    if (d > dist[k] + 1e-9) continue;
    const PhysConfig X = configs[k];
    if (is_dd_goal(ins, X)) return d;
    // enumerate joint ops
    std::vector<std::vector<Op>> raw(R);
    for (size_t i = 0; i < R; ++i) {
      raw[i].push_back(Op::make_wait());
      int nb[4];
      const int n = ins.grid.neighbors(X.robots[i], nb);
      for (int kk = 0; kk < n; ++kk) raw[i].push_back(Op::make_move(nb[kk]));
      raw[i].push_back(Op::make_lift());
      raw[i].push_back(Op::make_drop());
    }
    std::vector<Op> ops(R, Op::make_wait());
    std::function<void(size_t, double)> rec = [&](size_t i, double c) {
      if (i == R) {
        auto nxt = apply_ops(ins, X, ops);
        if (!nxt.has_value()) return;
        const auto nk = key_of(*nxt);
        auto it = dist.find(nk);
        if (it == dist.end() || it->second > d + c + 1e-9) {
          dist[nk] = d + c;
          configs[nk] = *nxt;
          pq.push({d + c, nk});
        }
        return;
      }
      for (const Op& op : raw[i]) {
        double c2 = c;
        if (op.kind == Op::MOVE) {
          c2 += 1;
          if (X.kappa[i] == KAPPA_ANON) c2 += 1;
        }
        if (op.kind == Op::LIFT || op.kind == Op::DROP) c2 += 1;
        ops[i] = op;
        rec(i + 1, c2);
      }
    };
    rec(0, 0);
  }
  return -1;
}

}  // namespace

TEST(dd_anytime, first_solution_recorded_and_incumbent_improves_or_equal)
{
  auto ins = make_ins({".....", "....."}, {{1, 0}, {1, 4}},
                      {{0, 0}, {0, 1}, {0, 2}}, {{{0, 0}, {0, 4}}});
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 2.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_GE(st.first_solution_ms, 0) << "first-solution time not recorded";
  EXPECT_GT(st.first_solution_soc, 0);
  EXPECT_GT(st.best_soc, 0);
  EXPECT_LE(st.best_soc, st.first_solution_soc)
      << "anytime incumbent must never be worse than the first solution";
  EXPECT_GE(st.incumbent_updates, 1);
  const double soc = plan_soc(ins, plan);
  EXPECT_NEAR(soc, st.best_soc, 1e-6)
      << "returned plan must BE the best incumbent";
}

TEST(dd_anytime, anytime_reaches_optimal_on_tiny_instance)
{
  // tiny instance solvable optimally by brute force; with generous time the
  // anytime loop must reach the optimum (eventually-optimal behavior on a
  // finite space with f-pruning).
  auto ins = make_ins({"...", "..."}, {{1, 0}}, {{0, 1}},
                      {{{0, 1}, {0, 2}}});
  const double opt = optimal_soc(ins);
  ASSERT_GT(opt, 0);
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 3.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_NEAR(plan_soc(ins, plan), opt, 1e-6)
      << "anytime search failed to reach the brute-force optimum";
}

TEST(dd_anytime, admissible_h_never_exceeds_true_cost)
{
  // 4 tiny instances: brute-force optimum must dominate the reported root
  // admissible h (exposed via stats.f_pruned semantics -> we verify via the
  // solver: with time, best_soc >= h0 is implied; here we directly check
  // best_soc >= dd_root_admissible_h)
  std::vector<DDInstance> cases;
  cases.push_back(make_ins({"...", "..."}, {{1, 0}}, {{0, 1}},
                           {{{0, 1}, {0, 2}}}));
  cases.push_back(make_ins({"....", "...."}, {{1, 0}}, {{0, 1}},
                           {{{0, 1}, {0, 3}}}));
  cases.push_back(make_ins({"..", ".."}, {{0, 0}, {1, 1}},
                           {{0, 1}, {1, 0}},
                           {{{0, 1}, {1, 0}}, {{1, 0}, {0, 1}}}));
  cases.push_back(make_ins({".....", "....."}, {{1, 2}},
                           {{0, 1}, {0, 3}},
                           {{{0, 1}, {0, 4}}, {{0, 3}, {0, 0}}}));
  for (size_t ci = 0; ci < cases.size(); ++ci) {
    const auto& ins = cases[ci];
    const double h0 = dd_root_admissible_h(ins);
    const double opt = optimal_soc(ins);
    ASSERT_GE(opt, 0) << "case " << ci;
    EXPECT_LE(h0, opt + 1e-9)
        << "admissible h exceeds true optimum on case " << ci;
  }
}

TEST(dd_anytime, macro_disabled_after_first_solution)
{
  // Two-phase policy (ablation-驱动): macro rollout is a FEASIBILITY device
  // — greedy multi-step traces committed into the plan pad the cost at
  // scale (full-suite A/B: paper-suite makespan 2.5-3x worse with macro in
  // the improvement phase).  After the first solution the anytime phase
  // must therefore never insert macro successors.
  auto ins = make_ins(
      {"............", "............", "..@@........", "..@@........",
       "............", "............", "............", "............",
       "............", "............", "............", "............"},
      {{0, 0}, {0, 11}, {11, 0}, {11, 11}},
      {{2, 6}, {2, 7}, {3, 6}, {3, 7}, {8, 2}, {8, 3}, {9, 2}, {9, 3}},
      {{{2, 6}, {10, 10}}, {{8, 2}, {0, 5}}});
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 3.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_GT(st.macro_successors, 0)
      << "macro must still fire BEFORE the first solution";
  EXPECT_EQ(st.macro_after_first, 0)
      << "macro successors inserted during the anytime phase";
}
