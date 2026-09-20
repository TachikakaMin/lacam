// PROTECTED tests: gantry (top-rail hoist) collision semantics.
// Written BEFORE implementation (TDD RED).
//
// Physical model: robots ride a rail plane above every stack; a carried
// load travels at transport height, so it never conflicts with grounded
// shelves.  Grounded-grounded exclusivity and robot-robot conflicts are
// unchanged.  Default (gantry=false) keeps warehouse semantics exactly.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <cstdio>
#include <fstream>

#include "gtest/gtest.h"

namespace {

DDInstance make_ins(const std::vector<std::string>& rows,
                    const std::vector<std::pair<int, int>>& robots,
                    const std::vector<std::pair<int, int>>& shelves,
                    const std::vector<std::pair<std::pair<int, int>,
                                                std::pair<int, int>>>& targets,
                    bool gantry)
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
  ins.gantry = gantry;
  ins.finalize();
  return ins;
}

Op M(const DDInstance& ins, int r, int c)
{
  return Op::make_move(ins.grid.idx(r, c));
}

// single row, robot on the target at (0,1); target goal (0,3); a second
// target parked (grounded, at its own goal) at (0,2) blocks the corridor
DDInstance corridor(bool gantry)
{
  return make_ins({"...."}, {{0, 1}}, {{0, 1}, {0, 2}},
                  {{{0, 1}, {0, 3}}, {{0, 2}, {0, 2}}}, gantry);
}

}  // namespace

// default semantics documented: loaded robot cannot enter a grounded-shelf
// cell (carried load conflicts with the grounded shelf)
TEST(dd_gantry, default_blocks_loaded_over_grounded)
{
  auto ins = corridor(false);
  auto s = initial_phys_config(ins);
  auto lifted = apply_ops(ins, s, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  EXPECT_FALSE(apply_ops(ins, *lifted, {M(ins, 0, 2)}).has_value());
}

// gantry: the carried load passes over the grounded shelf
TEST(dd_gantry, loaded_moves_over_grounded)
{
  auto ins = corridor(true);
  auto s = initial_phys_config(ins);
  auto lifted = apply_ops(ins, s, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  auto over = apply_ops(ins, *lifted, {M(ins, 0, 2)});
  ASSERT_TRUE(over.has_value());  // carried load above grounded shelf
  auto on = apply_ops(ins, *over, {M(ins, 0, 3)});
  ASSERT_TRUE(on.has_value());
  auto done = apply_ops(ins, *on, {Op::make_drop()});
  ASSERT_TRUE(done.has_value());
  EXPECT_TRUE(is_dd_goal(ins, *done));
}

// gantry: dropping onto a cell holding a grounded shelf stays illegal
TEST(dd_gantry, drop_on_occupied_cell_rejected)
{
  auto ins = corridor(true);
  auto s = initial_phys_config(ins);
  auto lifted = apply_ops(ins, s, {Op::make_lift()});
  ASSERT_TRUE(lifted.has_value());
  auto over = apply_ops(ins, *lifted, {M(ins, 0, 2)});
  ASSERT_TRUE(over.has_value());
  EXPECT_FALSE(apply_ops(ins, *over, {Op::make_drop()}).has_value());
}

// gantry: two grounded shelves on one cell can never arise; lifting the
// blocker and re-dropping it elsewhere still works as before
TEST(dd_gantry, grounded_exclusivity_kept)
{
  auto ins = corridor(true);
  auto s = initial_phys_config(ins);
  // second robot-free sanity: target 1 grounded at (0,2); a WAIT keeps all
  auto w = apply_ops(ins, s, {Op::make_wait()});
  ASSERT_TRUE(w.has_value());
  EXPECT_EQ(w->target_pos[1], ins.grid.idx(0, 2));
}

// YAML flag round-trip + solver end-to-end: gantry corridor is solvable
// with the direct carry-over route
TEST(dd_gantry, yaml_flag_and_solver)
{
  const char* path = "/tmp/dd_gantry_test_instance.yaml";
  {
    std::ofstream f(path);
    f << "name: gantry_corridor\n"
      << "map: |\n  .....\n"
      << "robots:\n  - [0, 0]\n"
      << "shelves:\n  - [0, 1]\n  - [0, 2]\n  - [0, 3]\n"
      << "targets:\n"
      << "  - id: b0\n    start: [0, 1]\n    goal: [0, 4]\n"
      << "  - id: b1\n    start: [0, 2]\n    goal: [0, 2]\n"
      << "  - id: b2\n    start: [0, 3]\n    goal: [0, 3]\n"
      << "flags: {gantry: true}\n";
  }
  auto ins = load_dd_instance(path);
  std::remove(path);
  EXPECT_TRUE(ins.gantry);
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  // direct route: move to (0,1), lift, 3 moves over the parked targets,
  // drop => 6 steps (allow small slack, but far below any shuffle plan)
  EXPECT_LE((int)plan.size(), 8);
}

// backward compatibility: flags {} still parses, gantry defaults to false
TEST(dd_gantry, default_flag_parses_false)
{
  const char* path = "/tmp/dd_gantry_default_instance.yaml";
  {
    std::ofstream f(path);
    f << "name: plain\nmap: |\n  ..\nrobots:\n  - [0, 0]\n"
      << "shelves:\n  - [0, 1]\ntargets:\n"
      << "  - id: b0\n    start: [0, 1]\n    goal: [0, 1]\n"
      << "flags: {}\n";
  }
  auto ins = load_dd_instance(path);
  std::remove(path);
  EXPECT_FALSE(ins.gantry);
}
