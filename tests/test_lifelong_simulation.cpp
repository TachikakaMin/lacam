#include <filesystem>

#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(lifelong_simulation, default_config_uses_scoring_planner_knobs)
{
  const auto config = LifelongSimulationConfig();
  EXPECT_EQ(config.max_shared_drop_goal_agents, 1);
  EXPECT_EQ(config.assignment_cost_mode,
            LIFELONG_ASSIGNMENT_COST_MILD_PICKUP_DELAY);

  const auto metrics = LifelongSimulationMetrics();
  EXPECT_EQ(metrics.max_shared_drop_goal_agents, 1);
  EXPECT_EQ(metrics.assignment_cost_mode,
            LIFELONG_ASSIGNMENT_COST_MILD_PICKUP_DELAY);
}

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

TEST(lifelong_simulation, planner_timeout_waits_and_replans_without_invalidating)
{
  auto config = LifelongSimulationConfig();
  config.map_filename = "./tests/assets/lifelong-sim-small.map";
  config.cache_filename =
      (std::filesystem::temp_directory_path() /
       "lacam_lifelong_sim_timeout_cache.bin")
          .string();
  config.num_agents = 1;
  config.horizon = 3;
  config.seed = 0;
  config.start_indexes = {3};
  config.planner_time_limit_sec = -0.001;
  config.task_config.goal_set_size = 3;
  config.task_config.release_interval = 100;
  config.task_config.outbound_probability = 1.0;
  std::filesystem::remove(config.cache_filename);

  const auto metrics = run_lifelong_simulation(config);

  std::filesystem::remove(config.cache_filename);
  ASSERT_TRUE(metrics.valid) << metrics.error;
  ASSERT_EQ(metrics.completed_tasks, 0);
  ASSERT_GE(metrics.planner_timeout_count + metrics.planner_failure_count +
                metrics.planner_empty_solution_count,
            1);
  ASSERT_GE(metrics.planner_invocations, 1);
}

TEST(lifelong_simulation, pickup_service_duration_delays_pickup_event)
{
  auto config = LifelongSimulationConfig();
  config.map_filename = "./tests/assets/lifelong-sim-small.map";
  config.cache_filename = "";
  config.num_agents = 1;
  config.horizon = 20;
  config.seed = 3;
  config.planner_time_limit_sec = 0.2;
  config.task_config.goal_set_size = 1;
  config.pickup_service_duration = 3;
  config.delivery_service_duration = 1;
  config.debug = true;

  const auto baseline = run_lifelong_simulation([&] {
    auto baseline_config = config;
    baseline_config.pickup_service_duration = 1;
    return baseline_config;
  }());
  const auto delayed = run_lifelong_simulation(config);

  ASSERT_TRUE(baseline.valid) << baseline.error;
  ASSERT_TRUE(delayed.valid) << delayed.error;
  ASSERT_FALSE(baseline.task_records.empty());
  ASSERT_FALSE(delayed.task_records.empty());
  ASSERT_GE(baseline.task_records.front().pickup_timestep, 0);
  ASSERT_EQ(delayed.task_records.front().pickup_timestep,
            baseline.task_records.front().pickup_timestep + 2);
}

TEST(lifelong_simulation,
     delivery_service_duration_delays_completion_event)
{
  auto config = LifelongSimulationConfig();
  config.map_filename = "./tests/assets/lifelong-sim-small.map";
  config.cache_filename = "";
  config.num_agents = 1;
  config.horizon = 30;
  config.seed = 3;
  config.planner_time_limit_sec = 0.2;
  config.task_config.goal_set_size = 1;
  config.pickup_service_duration = 1;
  config.delivery_service_duration = 3;
  config.debug = true;

  const auto baseline = run_lifelong_simulation([&] {
    auto baseline_config = config;
    baseline_config.delivery_service_duration = 1;
    return baseline_config;
  }());
  const auto delayed = run_lifelong_simulation(config);

  ASSERT_TRUE(baseline.valid) << baseline.error;
  ASSERT_TRUE(delayed.valid) << delayed.error;
  ASSERT_FALSE(baseline.task_records.empty());
  ASSERT_FALSE(delayed.task_records.empty());
  ASSERT_GE(baseline.task_records.front().completion_timestep, 0);
  ASSERT_EQ(delayed.task_records.front().completion_timestep,
            baseline.task_records.front().completion_timestep + 2);
}

TEST(lifelong_simulation,
     congestion_cost_mode_records_target_region_trace_metrics)
{
  auto config = LifelongSimulationConfig();
  config.map_filename = "./tests/assets/lifelong-sim-small.map";
  config.cache_filename =
      (std::filesystem::temp_directory_path() /
       "lacam_lifelong_sim_congestion_cache.bin")
          .string();
  config.num_agents = 2;
  config.horizon = 8;
  config.seed = 0;
  config.start_indexes = {3, 4};
  config.planner_time_limit_sec = 0.2;
  config.task_config.goal_set_size = 1;
  config.task_config.release_interval = 100;
  config.task_config.outbound_probability = 1.0;
  config.assignment_cost_mode = LIFELONG_ASSIGNMENT_COST_CONGESTION;
  config.debug = true;
  std::filesystem::remove(config.cache_filename);

  const auto metrics = run_lifelong_simulation(config);

  std::filesystem::remove(config.cache_filename);
  ASSERT_TRUE(metrics.valid) << metrics.error;
  ASSERT_GE(metrics.planner_invocations, 1);
  ASSERT_FALSE(metrics.planner_trace_records.empty());
  EXPECT_EQ(metrics.assignment_cost_mode,
            LIFELONG_ASSIGNMENT_COST_CONGESTION);
  const auto& trace = metrics.planner_trace_records.front();
  EXPECT_GE(trace.max_target_region_agent_count, 0);
  EXPECT_GE(trace.max_target_region_loaded_agent_count, 0);
  EXPECT_GE(trace.max_target_region_loaded_waiting_count, 0);
  EXPECT_GE(trace.max_target_region_loaded_stopped_count, 0);
  EXPECT_GE(trace.average_target_region_agent_count, 0);
  EXPECT_GE(trace.average_target_region_loaded_agent_count, 0);
}
