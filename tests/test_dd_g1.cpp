// PROTECTED conformance tests for debug.md P0-1 (frozen constraint order)
// and P0-2 (G1: fully-constrained joint ops must be decided by the
// deterministic validator, and the constraint tree must enumerate EXACTLY
// the validator-accepted successor set).
//
// Oracle: brute-force enumeration over the FULL raw op space per robot
// (wait, all 4 moves, lift, drop — no precondition filtering), accepting
// whatever apply_ops accepts.  The production machinery must produce the
// same distinct successor set from a single node.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>
#include <functional>
#include <set>
#include <vector>

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

// canonical string key of a PhysConfig for set comparison
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

// brute force: all raw per-robot ops, validator decides
std::set<std::string> brute_force_successors(const DDInstance& ins,
                                             const PhysConfig& X)
{
  const size_t R = ins.n_robots();
  std::vector<std::vector<Op>> raw(R);
  for (size_t i = 0; i < R; ++i) {
    raw[i].push_back(Op::make_wait());
    int nb[4];
    const int n = ins.grid.neighbors(X.robots[i], nb);
    for (int k = 0; k < n; ++k) raw[i].push_back(Op::make_move(nb[k]));
    raw[i].push_back(Op::make_lift());
    raw[i].push_back(Op::make_drop());
  }
  std::set<std::string> out;
  std::vector<Op> ops(R, Op::make_wait());
  std::function<void(size_t)> rec = [&](size_t i) {
    if (i == R) {
      auto nxt = apply_ops(ins, X, ops);
      if (nxt.has_value()) out.insert(key_of(*nxt));
      return;
    }
    for (const Op& op : raw[i]) {
      ops[i] = op;
      rec(i + 1);
    }
  };
  rec(0);
  return out;
}

void expect_exact_successor_set(const DDInstance& ins, int seed)
{
  const auto X = initial_phys_config(ins);
  const auto oracle = brute_force_successors(ins, X);
  const auto produced_vec = dd_enumerate_node_successors(ins, X, seed);
  std::set<std::string> produced;
  for (const auto& s : produced_vec) produced.insert(key_of(s));

  std::vector<std::string> missing, extra;
  std::set_difference(oracle.begin(), oracle.end(), produced.begin(),
                      produced.end(), std::back_inserter(missing));
  std::set_difference(produced.begin(), produced.end(), oracle.begin(),
                      oracle.end(), std::back_inserter(extra));
  EXPECT_TRUE(missing.empty())
      << missing.size() << " validator-accepted successors NOT produced by "
      << "the constraint tree (G1/completeness violation), e.g.\n  "
      << (missing.empty() ? "" : missing.front());
  EXPECT_TRUE(extra.empty())
      << extra.size() << " produced successors are NOT validator-accepted, "
      << "e.g.\n  " << (extra.empty() ? "" : extra.front());
  // sanity: oracle non-trivial
  EXPECT_GT(oracle.size(), 1u);
}

}  // namespace

TEST(dd_g1_conformance, corridor_two_robots_one_target)
{
  // 2x3, two robots, one target mid-carry potential: op space 4^2..6^2
  auto ins = make_ins({"...", "..."}, {{0, 0}, {1, 2}}, {{0, 1}},
                      {{{0, 1}, {0, 2}}});
  for (int seed : {0, 1, 7}) expect_exact_successor_set(ins, seed);
}

TEST(dd_g1_conformance, zero_empty_cycle_four_loaded_robots)
{
  // Prop-2 cycle: every legal joint op (incl. the 4-rotation) must appear.
  auto ins = make_ins(
      {"..", ".."}, {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{0, 0}, {0, 1}, {1, 1}, {1, 0}},
      {{{0, 0}, {0, 1}}, {{0, 1}, {1, 1}}, {{1, 1}, {1, 0}}, {{1, 0}, {0, 0}}});
  for (int seed : {0, 3}) expect_exact_successor_set(ins, seed);
}

TEST(dd_g1_conformance, mixed_anon_and_target_with_carriers)
{
  // free robot under anon shelf + robot beside target: lift/drop branches
  auto ins = make_ins({"....", "...."}, {{0, 1}, {1, 2}},
                      {{0, 1}, {0, 2}, {1, 3}}, {{{0, 2}, {1, 0}}});
  for (int seed : {0, 5}) expect_exact_successor_set(ins, seed);
}

TEST(dd_g1_conformance, mid_carry_state_from_solver_step)
{
  // exercise a state with kappa != FREE: lift first via validator, then
  // compare successor sets from the carrying state.
  auto ins = make_ins({"...", "..."}, {{0, 1}, {1, 0}}, {{0, 1}, {0, 2}},
                      {{{0, 1}, {0, 0}}});
  auto X = initial_phys_config(ins);
  auto lifted = apply_ops(ins, X, {Op::make_lift(), Op::make_wait()});
  ASSERT_TRUE(lifted.has_value());
  const auto oracle = brute_force_successors(ins, *lifted);
  const auto produced_vec = dd_enumerate_node_successors(ins, *lifted, 0);
  std::set<std::string> produced;
  for (const auto& s : produced_vec) produced.insert(key_of(s));
  std::vector<std::string> missing;
  std::set_difference(oracle.begin(), oracle.end(), produced.begin(),
                      produced.end(), std::back_inserter(missing));
  EXPECT_TRUE(missing.empty())
      << missing.size() << " successors missing from carrying state";
  EXPECT_EQ(produced.size(), oracle.size());
}

TEST(dd_prop2, zero_empty_cycle_moves_require_following)
{
  // Proposition 2 same-instance separation (debug.md P2-10): on the fully
  // occupied cycle, EVERY validator-legal transition that contains a MOVE
  // has some robot entering a cell vacated in the SAME step (following).
  // Hence any no-following model (BRaP / 1-robust MAPF-DECOMP(PP)) admits
  // only lift/drop/wait transitions here -> shelves can never move -> the
  // instance is unsolvable for them, while Carrier-LaCAM solves it
  // (dd_planner.cycle_rotation_zero_empty_cells).
  auto ins = load_dd_instance(std::string(DD_TEST_DIR) +
                              "/fixtures/prop2_cycle_2x2.yaml");
  const auto X = initial_phys_config(ins);
  // lift everything first (carriers on every shelf), the interesting state:
  std::vector<Op> lifts(4, Op::make_lift());
  auto lifted = apply_ops(ins, X, lifts);
  ASSERT_TRUE(lifted.has_value());
  const size_t R = ins.n_robots();
  std::vector<std::vector<Op>> raw(R);
  for (size_t i = 0; i < R; ++i) {
    raw[i].push_back(Op::make_wait());
    int nb[4];
    const int n = ins.grid.neighbors(lifted->robots[i], nb);
    for (int k = 0; k < n; ++k) raw[i].push_back(Op::make_move(nb[k]));
    raw[i].push_back(Op::make_drop());
  }
  long with_move = 0, following_violations = 0;
  std::vector<Op> ops(R, Op::make_wait());
  std::function<void(size_t)> rec = [&](size_t i) {
    if (i == R) {
      auto nxt = apply_ops(ins, *lifted, ops);
      if (!nxt.has_value()) return;
      bool any_move = false;
      for (const Op& op : ops) any_move |= (op.kind == Op::MOVE);
      if (!any_move) return;
      ++with_move;
      // no-following check: every mover enters a cell someone vacates now
      for (size_t r2 = 0; r2 < R; ++r2) {
        if (ops[r2].kind != Op::MOVE) continue;
        bool vacated = false;
        for (size_t j = 0; j < R; ++j)
          if (lifted->robots[j] == ops[r2].to &&
              nxt->robots[j] != ops[r2].to)
            vacated = true;
        if (!vacated) ++following_violations;
      }
      return;
    }
    for (const Op& op : raw[i]) {
      ops[i] = op;
      rec(i + 1);
    }
  };
  rec(0);
  ASSERT_GT(with_move, 0) << "cycle admits no move transitions at all?";
  EXPECT_EQ(following_violations, 0)
      << "found a mover entering a non-vacated cell: the separation "
         "argument would be broken";
}
