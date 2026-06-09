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

struct LifelongTaskVisualizationRecord {
  int task_id = -1;
  LifelongTaskType task_type = LifelongTaskType::OUTBOUND;
  int start_index = -1;
  std::vector<int> goal_indexes;
  int release_timestep = -1;
  int pickup_timestep = -1;
  int completion_timestep = -1;
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
  int planner_snapshot_infeasible_count = 0;
  int planner_invalid_instance_count = 0;
  int planner_empty_solution_count = 0;
  int first_empty_loaded_agents = -1;
  int first_empty_assigned_unloaded_agents = -1;
  int first_empty_idle_agents = -1;
  int first_empty_unique_target_count = -1;
  int first_empty_singleton_agents = -1;
  int first_empty_multi_goal_agents = -1;
  double average_planner_runtime = 0;
  double max_planner_runtime = 0;
  double total_simulation_runtime = 0;
  double average_agent_idle_time = 0;
  double average_agent_loaded_time = 0;
  double average_agent_unloaded_time = 0;
  int map_width = 0;
  int map_height = 0;
  std::vector<std::vector<int> > executed_path_indexes;
  std::vector<std::vector<int> > agent_task_ids_by_timestep;
  std::vector<std::vector<int> > agent_task_phases_by_timestep;
  std::vector<LifelongTaskVisualizationRecord> task_records;
  bool valid = true;
  std::string error;
};

LifelongSimulationMetrics run_lifelong_simulation(
    const LifelongSimulationConfig& config);
