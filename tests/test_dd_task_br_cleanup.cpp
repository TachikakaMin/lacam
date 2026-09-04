// PROTECTED Task-BR-PIBT source-hygiene gate.
// The migration is incomplete while the legacy guidance pipeline remains
// compiled, publicly exposed, or registered as a test/ablation target.
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace {

std::string read_source(const std::string& relative)
{
  const std::string root =
      std::string(DD_TEST_DIR) + "/../";
  std::ifstream input(root + relative);
  EXPECT_TRUE(input.good()) << relative;
  std::ostringstream out;
  out << input.rdbuf();
  return out.str();
}

void expect_absent(
    const std::string& relative,
    const std::vector<std::string>& forbidden)
{
  const auto source = read_source(relative);
  for (const auto& token : forbidden)
    EXPECT_EQ(source.find(token), std::string::npos)
        << relative << " still contains legacy token: " << token;
}

}  // namespace

TEST(dd_task_br_cleanup, public_headers_expose_only_task_br_guidance)
{
  expect_absent(
      "lacam/include/tapf_planner.hpp",
      {
          "struct CarrierRequest",
          "struct DemandKey",
          "struct ManipulationTask",
          "struct ObjectiveOption",
          "selected_packages",
          "parking_cell",
          "target_park",
          "objective_no_progress",
          "std::set<TAPFNode*> neighbor",
          "macro_edges",
          "futile_clock",
          "lift_futile",
          "note_lift_cycle",
          "lift_on_cooldown",
          "bool reguide",
          "rollout_parent_guide",
          "rewire_rebuild",
          "tau_price_repairs",
          "obj_default_resolutions",
          "futile_lift_demotions",
      });
  expect_absent(
      "lacam/include/dd_planner.hpp",
      {
          "dd_solve_tau(",
          "dd_pathcache_dst_probe",
          "dd_enumerate_node_successors_reguided",
          "dd_compute_park",
          "dd_match_free_goals",
          "DDObjective",
          "dd_build_tasks",
          "DDCustodyStep",
          "dd_waitfor_cycle_robots",
          "tau_price_repairs",
          "obj_default_resolutions",
          "futile_lift_demotions",
      });
}

TEST(dd_task_br_cleanup, production_sources_contain_one_guidance_pipeline)
{
  expect_absent(
      "lacam/src/carrier_guidance.hpp",
      {
          "#if 0",
          "task_ident_hash",
          "ObjectiveOption",
          "ManipulationTask",
          "PathCache",
          "TauEngine",
          "build_guidance(",
          "waitfor_cycles",
          "compute_execution_prices",
          "ACTIVE_TARGET_CAP",
          "LIVELOCK_WINDOW",
          "target_park",
          "parking_cell",
          "selected_packages",
      });
  expect_absent(
      "lacam/src/tapf_planner.cpp",
      {
          "#if 0",
          "reguide",
          "rollout_parent_guide",
          "rewire_rebuild",
          "macro_edges",
          "note_lift_cycle",
          "lift_on_cooldown",
          "futile_lift",
          "push_moves_sorted_by",
          "node_from->neighbor",
      });
  expect_absent(
      "lacam/src/dd_planner.cpp",
      {
          "using carrier_detail::PathCache",
          "using carrier_detail::Scratch",
          "using carrier_detail::build_guidance",
          "using carrier_detail::waitfor_cycles",
          "dd_enumerate_node_successors_reguided",
          "dd_compute_park",
          "dd_match_free_goals",
          "DDObjective",
          "dd_build_tasks",
          "TauEngine",
          "compute_execution_prices",
      });
}

TEST(dd_task_br_cleanup, build_and_diagnostics_drop_objective_era_contracts)
{
  expect_absent(
      "CMakeLists.txt",
      {
          "DD_OBJECTIVE_FORCE_DEFAULT",
          "DD_OBJECTIVE_NO_INHERIT",
          "DD_OBJECTIVE_DROP_SECOND_ROOT",
          "test_dd_park_purity",
          "test_dd_waitfor",
          "test_dd_objective_pibt",
          "test_dd_objective_budget",
          "test_dd_objective_integration",
          "test_dd_objective_priority_integration",
          "test_dd_reguide_custody",
          "test_dd_reguide_stats",
          "test_dd_fresh_lift_claim",
          "add_objective_ablation_test",
      });
  expect_absent(
      "tools/dd_benchmark.cpp",
      {
          "obj_default_resolutions",
          "obj_reselect_requests",
          "obj_backtracks",
          "obj_yields",
          "tasks_merged",
          "tau_price_repairs",
          "futile_lift_demotions",
      });
  expect_absent(
      "benchmark/run_benchmark.py",
      {
          "obj_default_resolutions",
          "obj_reselect_requests",
          "obj_backtracks",
          "obj_yields",
          "tasks_merged",
          "tau_price_repairs",
          "futile_lift_demotions",
      });
}
