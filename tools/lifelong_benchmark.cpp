#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <lacam.hpp>
#include <string>
#include <tuple>

namespace
{
void print_usage()
{
  std::cerr << "usage: lifelong_benchmark MAP NUM_AGENTS HORIZON SEED "
               "OUTPUT_CSV [CACHE] [TIME_LIMIT_SEC=2] [GOAL_SET_SIZE=3] "
               "[OUTBOUND_PROB=0.5] [RELEASE_INTERVAL=10] [DEBUG=0] "
               "[SCHEDULE_YAML] [ANYTIME=0] [MULTI_CARRY_CAPACITY=1] "
               "[FORCE_FULL_ASSIGNMENT=0] [SERVICE_COMMIT_AGENTS=0]\n";
}

bool should_write_header(const std::string& path)
{
  return path.empty() || !std::filesystem::exists(path) ||
         std::filesystem::file_size(path) == 0;
}

void write_csv_header(std::ostream& out)
{
  out << "map_name,num_agents,horizon,seed,generated_tasks,completed_tasks,"
         "throughput,alternating_completed_tasks,alternating_throughput,"
         "final_pending_tasks,final_assigned_tasks,"
         "final_picked_tasks,average_task_completion_time,"
         "average_pickup_time,average_delivery_time,planner_invocations,"
         "planner_success_count,planner_timeout_count,"
         "planner_partial_solution_count,planner_failure_count,"
         "planner_snapshot_infeasible_count,planner_invalid_instance_count,"
         "planner_empty_solution_count,"
         "average_planner_runtime,max_planner_runtime,total_planner_runtime,"
         "total_assignment_runtime,total_planner_search_runtime,"
         "total_simulation_runtime,"
         "average_agent_idle_time,average_agent_loaded_time,"
         "average_agent_unloaded_time,planner_force_full_assignment,"
         "multi_carry_capacity,"
         "average_carried_tasks,max_carried_tasks,"
         "average_loaded_distance_since_last_delivery,"
         "max_loaded_distance_since_last_delivery,pickup_while_loaded_count,"
         "delivery_events,average_tasks_carried_at_delivery,"
         "assignment_row_cache_requests,assignment_row_cache_hits,"
         "assignment_row_cache_hit_rate,valid,error\n";
}

void write_csv_row(std::ostream& out, const LifelongSimulationMetrics& m)
{
  out << m.map_name << "," << m.num_agents << "," << m.horizon << ","
      << m.seed << "," << m.generated_tasks << "," << m.completed_tasks
      << "," << m.throughput << "," << m.alternating_completed_tasks << ","
      << m.alternating_throughput << "," << m.final_pending_tasks << ","
      << m.final_assigned_tasks << "," << m.final_picked_tasks << ","
      << m.average_task_completion_time << "," << m.average_pickup_time << ","
      << m.average_delivery_time << "," << m.planner_invocations << ","
      << m.planner_success_count << "," << m.planner_timeout_count << ","
      << m.planner_partial_solution_count << ","
      << m.planner_failure_count << ","
      << m.planner_snapshot_infeasible_count << ","
      << m.planner_invalid_instance_count << ","
      << m.planner_empty_solution_count << "," << m.average_planner_runtime << ","
      << m.max_planner_runtime << "," << m.total_planner_runtime << ","
      << m.total_assignment_runtime << "," << m.total_planner_search_runtime
      << "," << m.total_simulation_runtime << ","
      << m.average_agent_idle_time << "," << m.average_agent_loaded_time << ","
      << m.average_agent_unloaded_time << ","
      << m.planner_force_full_assignment << ","
      << m.multi_carry_capacity << ","
      << m.average_carried_tasks << "," << m.max_carried_tasks << ","
      << m.average_loaded_distance_since_last_delivery << ","
      << m.max_loaded_distance_since_last_delivery << ","
      << m.pickup_while_loaded_count << "," << m.delivery_events << ","
      << m.average_tasks_carried_at_delivery << ","
      << m.assignment_row_cache_requests << ","
      << m.assignment_row_cache_hits << ","
      << m.assignment_row_cache_hit_rate << "," << m.valid << ","
      << '"' << m.error << '"' << "\n";
}

void write_trace_csv(const std::string& path,
                     const LifelongSimulationMetrics& metrics)
{
  if (path.empty()) return;
  std::ofstream out(path);
  out << "timestep,should_replan,event_happened,plan_finished,"
         "previous_planner_failed,idle_unloaded_with_pending,snapshot_feasible,"
         "instance_valid,solution_found,timed_out,partial_solution,"
         "planning_runtime_ms,"
         "assignment_time_ms,planner_search_time_ms,loaded_agents,"
         "assigned_unloaded_agents,idle_agents,pending_tasks,assigned_tasks,"
         "picked_tasks,completed_tasks,singleton_agents,multi_goal_agents,"
         "unique_target_count,total_goal_options,max_goal_options_per_agent,"
         "max_agents_per_target,duplicate_target_slots,agents_carrying_two_tasks,"
         "loaded_agents_with_pickup_options,"
         "loaded_agents_with_delivery_options,"
         "loaded_agents_with_both_pickup_delivery_options,"
         "loaded_pickup_goal_options,loaded_delivery_goal_options,"
         "loaded_wait_goal_options,max_loaded_pickup_options_per_agent,"
         "max_loaded_delivery_options_per_agent,"
         "root_loaded_pickup_assignments,"
         "root_loaded_delivery_assignments,root_loaded_wait_assignments,"
         "root_unloaded_pickup_assignments,root_unloaded_wait_assignments,"
         "root_initial_assignment_cost,final_loaded_pickup_assignments,"
         "final_loaded_delivery_assignments,final_loaded_wait_assignments,"
         "final_unloaded_pickup_assignments,final_unloaded_wait_assignments,"
         "max_loaded_distance_agent_pickup_options,"
         "max_loaded_distance_agent_delivery_options,"
         "max_loaded_distance_agent_wait_options,"
         "max_loaded_distance_agent_id,"
         "max_loaded_distance_agent_current_index,"
         "max_loaded_distance_agent_carried_tasks,"
         "max_loaded_distance_agent_root_target_index,"
         "max_loaded_distance_agent_final_target_index,"
         "max_loaded_distance_agent_root_target_type,"
         "max_loaded_distance_agent_final_target_type,"
         "average_carried_tasks_now,max_carried_tasks_now,"
         "average_loaded_distance_now,max_loaded_distance_now,"
         "hl_loop_iterations,hl_nodes_created,hl_nodes_explored,open_max_size,"
         "constraints_popped,constraints_generated,constraint_failures,"
         "pibt_calls,pibt_failures,pibt_recursions,assignment_calls,"
         "assignment_changes,final_assignment_changes,"
         "final_agent_assignment_changes,assignment_row_cache_requests,"
         "assignment_row_cache_hits,solution_depth,solution_cost,solution_h,"
         "service_satisfied_agents,service_satisfied_pickups,"
         "service_satisfied_deliveries,service_best_satisfied_agents\n";
  for (const auto& r : metrics.planner_trace_records) {
    out << r.timestep << "," << r.should_replan << "," << r.event_happened
        << "," << r.plan_finished << "," << r.previous_planner_failed << ","
        << r.idle_unloaded_with_pending << "," << r.snapshot_feasible << ","
        << r.instance_valid << "," << r.solution_found << "," << r.timed_out
        << "," << r.partial_solution << "," << r.planning_runtime_ms << ","
        << r.assignment_time_ms << ","
        << r.planner_search_time_ms << "," << r.loaded_agents << ","
        << r.assigned_unloaded_agents << "," << r.idle_agents << ","
        << r.pending_tasks << "," << r.assigned_tasks << "," << r.picked_tasks
        << "," << r.completed_tasks << "," << r.singleton_agents << ","
        << r.multi_goal_agents << "," << r.unique_target_count << ","
        << r.total_goal_options << "," << r.max_goal_options_per_agent << ","
        << r.max_agents_per_target << "," << r.duplicate_target_slots << ","
        << r.agents_carrying_two_tasks << ","
        << r.loaded_agents_with_pickup_options << ","
        << r.loaded_agents_with_delivery_options << ","
        << r.loaded_agents_with_both_pickup_delivery_options << ","
        << r.loaded_pickup_goal_options << ","
        << r.loaded_delivery_goal_options << ","
        << r.loaded_wait_goal_options << ","
        << r.max_loaded_pickup_options_per_agent << ","
        << r.max_loaded_delivery_options_per_agent << ","
        << r.root_loaded_pickup_assignments << ","
        << r.root_loaded_delivery_assignments << ","
        << r.root_loaded_wait_assignments << ","
        << r.root_unloaded_pickup_assignments << ","
        << r.root_unloaded_wait_assignments << ","
        << r.root_initial_assignment_cost << ","
        << r.final_loaded_pickup_assignments << ","
        << r.final_loaded_delivery_assignments << ","
        << r.final_loaded_wait_assignments << ","
        << r.final_unloaded_pickup_assignments << ","
        << r.final_unloaded_wait_assignments << ","
        << r.max_loaded_distance_agent_pickup_options << ","
        << r.max_loaded_distance_agent_delivery_options << ","
        << r.max_loaded_distance_agent_wait_options << ","
        << r.max_loaded_distance_agent_id << ","
        << r.max_loaded_distance_agent_current_index << ","
        << r.max_loaded_distance_agent_carried_tasks << ","
        << r.max_loaded_distance_agent_root_target_index << ","
        << r.max_loaded_distance_agent_final_target_index << ","
        << r.max_loaded_distance_agent_root_target_type << ","
        << r.max_loaded_distance_agent_final_target_type << ","
        << r.average_carried_tasks_now << "," << r.max_carried_tasks_now << ","
        << r.average_loaded_distance_now << "," << r.max_loaded_distance_now
        << "," << r.hl_loop_iterations << "," << r.hl_nodes_created << ","
        << r.hl_nodes_explored << "," << r.open_max_size << ","
        << r.constraints_popped << "," << r.constraints_generated << ","
        << r.constraint_failures << "," << r.pibt_calls << ","
        << r.pibt_failures << "," << r.pibt_recursions << ","
        << r.assignment_calls << "," << r.assignment_changes << ","
        << r.final_assignment_changes << ","
        << r.final_agent_assignment_changes << ","
        << r.assignment_row_cache_requests << ","
        << r.assignment_row_cache_hits << "," << r.solution_depth << ","
        << r.solution_cost << "," << r.solution_h << ","
        << r.service_satisfied_agents << ","
        << r.service_satisfied_pickups << ","
        << r.service_satisfied_deliveries << ","
        << r.service_best_satisfied_agents << "\n";
  }
}

void write_u32(std::ofstream& out, std::uint32_t value)
{
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

std::string binary_schedule_path(const std::string& output_path)
{
  return output_path + ".bin";
}

std::string binary_schedule_metadata_path(const std::string& binary_path)
{
  return std::filesystem::path(binary_path).filename().string();
}

const char* task_type_name(LifelongTaskType type)
{
  return type == LifelongTaskType::OUTBOUND ? "outbound" : "inbound";
}

const char* task_phase_name(int phase)
{
  if (phase == 1) return "assigned";
  if (phase == 2) return "loaded";
  return "idle";
}

void write_yaml_point(std::ostream& out, int index, int width)
{
  out << "{x: " << index / width << ", y: " << index % width << "}";
}

void write_schedule_binary(const LifelongSimulationMetrics& metrics,
                           const std::string& binary_path)
{
  std::ofstream out(binary_path, std::ios::binary);
  const char magic[8] = {'T', 'A', 'P', 'F', 'S', 'C', 'H', '1'};
  out.write(magic, sizeof(magic));
  write_u32(out, static_cast<std::uint32_t>(metrics.num_agents));
  const auto makespan =
      metrics.executed_path_indexes.empty()
          ? 0
          : static_cast<std::uint32_t>(metrics.executed_path_indexes.front().size() - 1);
  write_u32(out, makespan);
  for (const auto& path : metrics.executed_path_indexes) {
    auto changes =
        std::vector<std::tuple<std::uint32_t, std::uint32_t, std::uint32_t> >();
    auto last = -1;
    for (size_t t = 0; t < path.size(); ++t) {
      if (path[t] == last) continue;
      last = path[t];
      changes.emplace_back(static_cast<std::uint32_t>(t),
                           static_cast<std::uint32_t>(path[t] / metrics.map_width),
                           static_cast<std::uint32_t>(path[t] % metrics.map_width));
    }
    write_u32(out, static_cast<std::uint32_t>(changes.size()));
    for (const auto& [t, x, y] : changes) {
      write_u32(out, t);
      write_u32(out, x);
      write_u32(out, y);
    }
  }
}

void write_schedule_yaml(const LifelongSimulationMetrics& metrics,
                         const std::string& output_path)
{
  if (output_path.empty() || metrics.executed_path_indexes.empty()) return;
  if (std::filesystem::path(output_path).has_parent_path()) {
    std::filesystem::create_directories(std::filesystem::path(output_path).parent_path());
  }
  const auto binary_path = binary_schedule_path(output_path);
  write_schedule_binary(metrics, binary_path);

  std::ofstream out(output_path);
  const auto makespan = metrics.executed_path_indexes.front().size() - 1;
  out << "statistics:\n";
  out << "  makespan: " << makespan << "\n";
  out << "  completed_tasks: " << metrics.completed_tasks << "\n";
  out << "  generated_tasks: " << metrics.generated_tasks << "\n";
  out << "  throughput: " << metrics.throughput << "\n";
  out << "  alternating_completed_tasks: "
      << metrics.alternating_completed_tasks << "\n";
  out << "  alternating_throughput: " << metrics.alternating_throughput
      << "\n";
  out << "  multi_carry_capacity: " << metrics.multi_carry_capacity << "\n";
  out << "  average_carried_tasks: " << metrics.average_carried_tasks << "\n";
  out << "  max_carried_tasks: " << metrics.max_carried_tasks << "\n";
  out << "  pickup_while_loaded_count: "
      << metrics.pickup_while_loaded_count << "\n";
  out << "  delivery_events: " << metrics.delivery_events << "\n";
  out << "  assignment_row_cache_requests: "
      << metrics.assignment_row_cache_requests << "\n";
  out << "  assignment_row_cache_hits: "
      << metrics.assignment_row_cache_hits << "\n";
  out << "  assignment_row_cache_hit_rate: "
      << metrics.assignment_row_cache_hit_rate << "\n";
  out << "assignments:\n";
  for (size_t i = 0; i < metrics.executed_path_indexes.size(); ++i) {
    const auto final_index = metrics.executed_path_indexes[i].back();
    out << "  agent" << i << ":\n";
    out << "    x: " << final_index / metrics.map_width << "\n";
    out << "    y: " << final_index % metrics.map_width << "\n";
  }
  out << "schedule_binary:\n";
  out << "  format: tapf_sparse_schedule_v1\n";
  out << "  encoding: little_endian_u32\n";
  out << "  path: " << binary_schedule_metadata_path(binary_path) << "\n";
  out << "  agents: " << metrics.num_agents << "\n";
  out << "  makespan: " << makespan << "\n";
  out << "tasks:\n";
  for (const auto& task : metrics.task_records) {
    out << "  - id: " << task.task_id << "\n";
    out << "    type: " << task_type_name(task.task_type) << "\n";
    out << "    release: " << task.release_timestep << "\n";
    out << "    pickup: " << task.pickup_timestep << "\n";
    out << "    completion: " << task.completion_timestep << "\n";
    out << "    start: ";
    write_yaml_point(out, task.start_index, metrics.map_width);
    out << "\n";
    out << "    goals:\n";
    for (const auto goal : task.goal_indexes) {
      out << "      - ";
      write_yaml_point(out, goal, metrics.map_width);
      out << "\n";
    }
  }
  out << "agent_task_timeline:\n";
  for (size_t i = 0; i < metrics.agent_task_ids_by_timestep.size(); ++i) {
    out << "  agent" << i << ":\n";
    auto last_task = -2;
    auto last_phase = -1;
    auto last_assigned = -2;
    auto last_carried = std::vector<int>{-2};
    const auto& task_ids = metrics.agent_task_ids_by_timestep[i];
    const auto& phases = metrics.agent_task_phases_by_timestep[i];
    for (size_t t = 0; t < task_ids.size(); ++t) {
      const auto assigned =
          metrics.agent_assigned_task_ids_by_timestep.empty()
              ? -1
              : metrics.agent_assigned_task_ids_by_timestep[i][t];
      const auto carried =
          metrics.agent_carried_task_ids_by_timestep.empty()
              ? std::vector<int>()
              : metrics.agent_carried_task_ids_by_timestep[i][t];
      if (task_ids[t] == last_task && phases[t] == last_phase &&
          assigned == last_assigned && carried == last_carried) {
        continue;
      }
      last_task = task_ids[t];
      last_phase = phases[t];
      last_assigned = assigned;
      last_carried = carried;
      out << "    - t: " << t << "\n";
      out << "      task: " << task_ids[t] << "\n";
      out << "      phase: " << task_phase_name(phases[t]) << "\n";
      if (!metrics.agent_assigned_task_ids_by_timestep.empty()) {
        out << "      assigned_task: " << assigned << "\n";
      }
      if (!metrics.agent_carried_task_ids_by_timestep.empty()) {
        out << "      carried_tasks: [";
        for (size_t k = 0; k < carried.size(); ++k) {
          if (k > 0) out << ", ";
          out << carried[k];
        }
        out << "]\n";
      }
    }
  }
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 6) {
    print_usage();
    return 2;
  }

  auto config = LifelongSimulationConfig();
  config.map_filename = argv[1];
  config.num_agents = std::stoi(argv[2]);
  config.horizon = std::stoi(argv[3]);
  config.seed = std::stoi(argv[4]);
  const auto output_csv = std::string(argv[5]);
  config.cache_filename =
      argc >= 7 ? std::string(argv[6]) : output_csv + ".distcache";
  config.planner_time_limit_sec = argc >= 8 ? std::stod(argv[7]) : 2.0;
  config.task_config.goal_set_size = argc >= 9 ? std::stoi(argv[8]) : 3;
  config.task_config.outbound_probability =
      argc >= 10 ? std::stod(argv[9]) : 0.5;
  config.task_config.release_interval = argc >= 11 ? std::stoi(argv[10]) : 10;
  config.debug = argc >= 12 ? std::stoi(argv[11]) != 0 : false;
  const auto schedule_yaml = argc >= 13 ? std::string(argv[12]) : std::string();
  config.planner_anytime = argc >= 14 ? std::stoi(argv[13]) != 0 : false;
  config.multi_carry_capacity = argc >= 15 ? std::stoi(argv[14]) : 1;
  config.planner_force_full_assignment =
      argc >= 16 ? std::stoi(argv[15]) != 0 : false;
  config.service_commit_agents = argc >= 17 ? std::stoi(argv[16]) : 0;

  const auto metrics = run_lifelong_simulation(config);
  std::ofstream out(output_csv, std::ios::app);
  if (!out) {
    std::cerr << "failed to open output CSV: " << output_csv << "\n";
    return 1;
  }
  if (should_write_header(output_csv)) write_csv_header(out);
  write_csv_row(out, metrics);
  write_trace_csv(output_csv + ".trace.csv", metrics);
  write_schedule_yaml(metrics, schedule_yaml);

  std::cout << "valid=" << metrics.valid << "\n";
  std::cout << "generated_tasks=" << metrics.generated_tasks << "\n";
  std::cout << "completed_tasks=" << metrics.completed_tasks << "\n";
  std::cout << "throughput=" << metrics.throughput << "\n";
  std::cout << "alternating_completed_tasks="
            << metrics.alternating_completed_tasks << "\n";
  std::cout << "alternating_throughput=" << metrics.alternating_throughput
            << "\n";
  std::cout << "multi_carry_capacity=" << metrics.multi_carry_capacity << "\n";
  std::cout << "planner_force_full_assignment="
            << metrics.planner_force_full_assignment << "\n";
  std::cout << "average_carried_tasks=" << metrics.average_carried_tasks << "\n";
  std::cout << "max_carried_tasks=" << metrics.max_carried_tasks << "\n";
  std::cout << "pickup_while_loaded_count="
            << metrics.pickup_while_loaded_count << "\n";
  std::cout << "delivery_events=" << metrics.delivery_events << "\n";
  std::cout << "average_tasks_carried_at_delivery="
            << metrics.average_tasks_carried_at_delivery << "\n";
  std::cout << "assignment_row_cache_requests="
            << metrics.assignment_row_cache_requests << "\n";
  std::cout << "assignment_row_cache_hits="
            << metrics.assignment_row_cache_hits << "\n";
  std::cout << "assignment_row_cache_hit_rate="
            << metrics.assignment_row_cache_hit_rate << "\n";
  std::cout << "planner_invocations=" << metrics.planner_invocations << "\n";
  std::cout << "total_planner_runtime=" << metrics.total_planner_runtime
            << "\n";
  std::cout << "total_assignment_runtime="
            << metrics.total_assignment_runtime << "\n";
  std::cout << "total_planner_search_runtime="
            << metrics.total_planner_search_runtime << "\n";
  std::cout << "planner_success_count=" << metrics.planner_success_count << "\n";
  std::cout << "planner_timeout_count=" << metrics.planner_timeout_count << "\n";
  std::cout << "planner_partial_solution_count="
            << metrics.planner_partial_solution_count << "\n";
  std::cout << "planner_failure_count=" << metrics.planner_failure_count << "\n";
  std::cout << "planner_snapshot_infeasible_count="
            << metrics.planner_snapshot_infeasible_count << "\n";
  std::cout << "planner_invalid_instance_count="
            << metrics.planner_invalid_instance_count << "\n";
  std::cout << "planner_empty_solution_count="
            << metrics.planner_empty_solution_count << "\n";
  if (metrics.first_empty_loaded_agents >= 0) {
    std::cout << "first_empty_loaded_agents="
              << metrics.first_empty_loaded_agents << "\n";
    std::cout << "first_empty_assigned_unloaded_agents="
              << metrics.first_empty_assigned_unloaded_agents << "\n";
    std::cout << "first_empty_idle_agents="
              << metrics.first_empty_idle_agents << "\n";
    std::cout << "first_empty_unique_target_count="
              << metrics.first_empty_unique_target_count << "\n";
    std::cout << "first_empty_singleton_agents="
              << metrics.first_empty_singleton_agents << "\n";
    std::cout << "first_empty_multi_goal_agents="
              << metrics.first_empty_multi_goal_agents << "\n";
  }
  if (!metrics.valid) std::cout << "error=" << metrics.error << "\n";
  return metrics.valid ? 0 : 1;
}
