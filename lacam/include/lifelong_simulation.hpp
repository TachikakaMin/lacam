/*
 * Event-driven lifelong TAPF simulation loop.
 */
#pragma once

#include "lifelong_planning.hpp"
#include "tapf_planner.hpp"

struct LifelongSimulationConfig {
  std::string map_filename;
  std::string cache_filename;
  int num_agents = 1;
  int horizon = 1000;
  int seed = 0;
  std::vector<int> start_indexes;
  double planner_time_limit_sec = 2.0;
  LifelongTaskGeneratorConfig task_config;
  bool debug = false;
};

struct LifelongSimulationMetrics {
  std::string map_name;
  int num_agents = 0;
  int horizon = 0;
  int seed = 0;
  int generated_tasks = 0;
  int completed_tasks = 0;
  double throughput = 0;
  int final_pending_tasks = 0;
  int final_assigned_tasks = 0;
  int final_picked_tasks = 0;
  double average_task_completion_time = 0;
  double average_pickup_time = 0;
  double average_delivery_time = 0;
  int planner_invocations = 0;
  int planner_success_count = 0;
  int planner_timeout_count = 0;
  int planner_failure_count = 0;
  double average_planner_runtime = 0;
  double max_planner_runtime = 0;
  double total_simulation_runtime = 0;
  double average_agent_idle_time = 0;
  double average_agent_loaded_time = 0;
  double average_agent_unloaded_time = 0;
  int map_width = 0;
  int map_height = 0;
  std::vector<std::vector<int> > executed_path_indexes;
  bool valid = true;
  std::string error;
};

LifelongSimulationMetrics run_lifelong_simulation(
    const LifelongSimulationConfig& config);
