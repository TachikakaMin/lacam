// PROTECTED tests: debug.md task-9 items — M1 exit-criterion literal case
// (design 10: "20x20 / 50 shelves / 10 robots 秒级首解"; the DD-MAPD 2x2
// block protocol yields multiples of 4, so 52 >= 50 shelves is used),
// Zobrist incremental hashing (design 6.1), and the dead-cell analysis.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <chrono>
#include <random>

#include "gtest/gtest.h"

TEST(dd_m1_exit, literal_case_first_solution_within_seconds)
{
  const auto path =
      std::string(DD_TEST_DIR) + "/fixtures/m1_exit_20x20_s52_r10.yaml";
  auto ins = load_dd_instance(path);
  ASSERT_EQ(ins.n_robots(), 10u);
  ASSERT_EQ(ins.shelves.size(), 52u);
  DDStats st;
  auto plan = solve_carrier_lacam(ins, 3.0, 0, &st);
  ASSERT_FALSE(plan.empty());
  EXPECT_GE(st.first_solution_ms, 0);
  EXPECT_LT(st.first_solution_ms, 2000.0)
      << "M1 exit criterion: first solution must arrive within seconds";
}

TEST(dd_zobrist, incremental_hash_matches_full_recompute)
{
  // property: for random legal transitions, the incrementally maintained
  // hash equals a from-scratch phys_config_hash of the successor.
  auto ins = load_dd_instance(std::string(DD_TEST_DIR) +
                              "/fixtures/m1_exit_20x20_s52_r10.yaml");
  auto s = initial_phys_config(ins);
  uint64_t inc = phys_config_hash(s);
  std::mt19937 rng(7);
  int applied = 0;
  for (int step = 0; step < 600 && applied < 60; ++step) {
    // one random actor per step (joint legality is near-certain), so the
    // property is exercised across many lift/move/drop transitions
    std::vector<Op> ops(ins.n_robots(), Op::make_wait());
    const size_t actor = rng() % ins.n_robots();
    {
      int nb[4];
      const int n = ins.grid.neighbors(s.robots[actor], nb);
      const int pick = (int)(rng() % (n + 2));
      if (pick < n)
        ops[actor] = Op::make_move(nb[pick]);
      else if (pick == n)
        ops[actor] = Op::make_lift();
      else
        ops[actor] = Op::make_drop();
    }
    auto nxt = apply_ops(ins, s, ops);
    if (!nxt.has_value()) continue;
    inc = phys_config_hash_incremental(ins, s, ops, inc);
    s = *nxt;
    ++applied;
    ASSERT_EQ(inc, phys_config_hash(s)) << "divergence at applied=" << applied;
  }
  ASSERT_GT(applied, 20) << "too few legal transitions exercised";
}

TEST(dd_dead_cell, wall_unreachable_goal_rejected_at_load)
{
  // design 5.6 dead-cell analysis, v1 semantics: both decks share the SAME
  // wall set, so a cell unreachable from a target's goal (walls-only BFS)
  // means the whole instance component is disconnected — the target can
  // NEVER reach its goal.  v1 therefore implements dead-cell analysis as an
  // instance-level feasibility rejection (finalize throws); the Sokoban
  // drop-pruning variant only matters once walls differ per deck (post-v1).
  DDInstance ins;
  ins.grid = DDGrid({"..@..", "..@..", "..@.."});  // two components
  ins.robots.push_back(ins.grid.idx(0, 0));
  ins.shelves.push_back(ins.grid.idx(1, 0));
  ins.target_starts.push_back(ins.grid.idx(1, 0));
  ins.target_goals.push_back(ins.grid.idx(1, 4));  // other component
  EXPECT_THROW(ins.finalize(), std::exception)
      << "goal in a different wall-component must be rejected at load";
}
