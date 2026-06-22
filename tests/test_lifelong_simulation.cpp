#include <filesystem>

#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(lifelong_simulation, direct_loaded_tapf_fixture_moves_to_goal)
{
  const auto ins = TAPFInstance("./tests/assets/lifelong-sim-small.map", {0},
                                std::vector<std::vector<int> >{{4, 5, 6}});
  ASSERT_TRUE(ins.is_valid());
  const auto solution = solve_tapf(ins);
  ASSERT_FALSE(solution.empty());
  ASSERT_GE(solution.size(), 2);
  ASSERT_TRUE(solution.back()[0]->index == 4 || solution.back()[0]->index == 5 ||
              solution.back()[0]->index == 6);
}

TEST(lifelong_simulation, small_event_driven_smoke_completes_task)
{
  auto config = LifelongSimulationConfig();
  config.map_filename = "./tests/assets/lifelong-sim-small.map";
  config.cache_filename =
      (std::filesystem::temp_directory_path() /
       "lacam_lifelong_sim_smoke_cache.bin")
          .string();
  config.num_agents = 1;
  config.horizon = 50;
  config.seed = 0;
  config.start_indexes = {3};
  config.planner_time_limit_sec = 0.2;
  config.task_config.goal_set_size = 3;
  config.task_config.release_interval = 100;
  config.task_config.outbound_probability = 1.0;
  config.debug = true;
  std::filesystem::remove(config.cache_filename);

  const auto metrics = run_lifelong_simulation(config);

  std::filesystem::remove(config.cache_filename);
  ASSERT_TRUE(metrics.valid) << metrics.error;
  ASSERT_GE(metrics.generated_tasks, 1);
  ASSERT_GE(metrics.completed_tasks, 1)
      << "generated=" << metrics.generated_tasks
      << " invocations=" << metrics.planner_invocations
      << " success=" << metrics.planner_success_count
      << " failure=" << metrics.planner_failure_count
      << " timeout=" << metrics.planner_timeout_count
      << " picked=" << metrics.final_picked_tasks
      << " assigned=" << metrics.final_assigned_tasks
      << " pending=" << metrics.final_pending_tasks;
  ASSERT_GE(metrics.planner_invocations, 1);
  ASSERT_GE(metrics.planner_success_count, 1);
  ASSERT_EQ(metrics.alternating_completed_tasks, 0);
  ASSERT_DOUBLE_EQ(metrics.alternating_throughput, 0);
}
