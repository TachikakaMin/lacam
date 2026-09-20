// PROTECTED tests: cold-start carrying (event-driven replanning support).
// Written BEFORE implementation (TDD RED).
//
// A gantry instance may declare that some robots already hold a target
// ("carrying"). Replanning after every completed drop then restarts the
// solver from the true mid-execution state: carried bricks keep their
// robot binding and goal, and participate in joint path planning.
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <cstdio>
#include <fstream>

#include "gtest/gtest.h"

namespace {

const char* write_yaml(const char* path, bool with_carrying)
{
  std::ofstream f(path);
  f << "name: carry_start\n"
    << "map: |\n  .....\n  .....\n"
    << "robots:\n  - [0, 2]\n  - [1, 0]\n"
    << "shelves:\n  - [0, 2]\n  - [1, 1]\n"
    << "targets:\n"
    << "  - id: b0\n    start: [0, 2]\n    goal: [0, 4]\n"
    << "  - id: b1\n    start: [1, 1]\n    goal: [1, 4]\n"
    << "flags: {gantry: true}\n";
  if (with_carrying) f << "carrying:\n  - [0, 0]\n";  // robot 0 holds b0
  return path;
}

}  // namespace

TEST(dd_carry_start, yaml_parses_and_initial_config)
{
  auto ins = load_dd_instance(
      write_yaml("/tmp/dd_carry_start1.yaml", true));
  std::remove("/tmp/dd_carry_start1.yaml");
  ASSERT_EQ(ins.carrying.size(), 2u);
  EXPECT_EQ(ins.carrying[0], 0);
  EXPECT_EQ(ins.carrying[1], KAPPA_FREE);
  auto s = initial_phys_config(ins);
  EXPECT_EQ(s.kappa[0], 0);
  EXPECT_EQ(s.kappa[1], KAPPA_FREE);
  EXPECT_EQ(s.target_pos[0], ins.grid.idx(0, 2));
  EXPECT_TRUE(validate_phys_config_root(ins, s).valid());
}

TEST(dd_carry_start, default_absent_all_free)
{
  auto ins = load_dd_instance(
      write_yaml("/tmp/dd_carry_start2.yaml", false));
  std::remove("/tmp/dd_carry_start2.yaml");
  auto s = initial_phys_config(ins);
  EXPECT_EQ(s.kappa[0], KAPPA_FREE);
  EXPECT_EQ(s.kappa[1], KAPPA_FREE);
}

// solver end-to-end: robot 0 starts mid-carry and must deliver b0 without
// parking it; robot 1 fetches b1 as usual
TEST(dd_carry_start, solver_finishes_from_mid_carry)
{
  auto ins = load_dd_instance(
      write_yaml("/tmp/dd_carry_start3.yaml", true));
  std::remove("/tmp/dd_carry_start3.yaml");
  auto plan = solve_carrier_lacam(ins, 5.0, 0);
  ASSERT_FALSE(plan.empty());
  // replay: b0 must never be dropped anywhere except its goal
  auto s = initial_phys_config(ins);
  int drops0 = 0;
  for (const auto& ops : plan) {
    auto nxt = apply_ops(ins, s, ops);
    ASSERT_TRUE(nxt.has_value());
    for (size_t i = 0; i < ops.size(); ++i)
      if (ops[i].kind == Op::DROP && s.kappa[i] == 0) {
        ++drops0;
        EXPECT_EQ(s.robots[i], ins.grid.idx(0, 4));
      }
    s = *nxt;
  }
  EXPECT_EQ(drops0, 1);  // delivered exactly once, no re-parking
  EXPECT_TRUE(is_dd_goal(ins, s));
  EXPECT_LE((int)plan.size(), 10);
}
