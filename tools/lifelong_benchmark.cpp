#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <lacam.hpp>
#include <tuple>

namespace
{
void print_usage()
{
  std::cerr << "usage: lifelong_benchmark MAP NUM_AGENTS HORIZON SEED "
               "OUTPUT_CSV [CACHE] [TIME_LIMIT_SEC=2] [GOAL_SET_SIZE=5] "
               "[OUTBOUND_PROB=0.5] [RELEASE_INTERVAL=10] [DEBUG=0] "
               "[SCHEDULE_YAML]\n";
}

bool should_write_header(const std::string& path)
{
  return path.empty() || !std::filesystem::exists(path) ||
         std::filesystem::file_size(path) == 0;
}

void write_csv_header(std::ostream& out)
{
  out << "map_name,num_agents,horizon,seed,generated_tasks,completed_tasks,"
         "throughput,final_pending_tasks,final_assigned_tasks,"
         "final_picked_tasks,average_task_completion_time,"
         "average_pickup_time,average_delivery_time,planner_invocations,"
         "planner_success_count,planner_timeout_count,planner_failure_count,"
         "planner_snapshot_infeasible_count,planner_invalid_instance_count,"
         "planner_empty_solution_count,"
         "average_planner_runtime,max_planner_runtime,total_simulation_runtime,"
         "average_agent_idle_time,average_agent_loaded_time,"
         "average_agent_unloaded_time,valid,error\n";
}

void write_csv_row(std::ostream& out, const LifelongSimulationMetrics& m)
{
  out << m.map_name << "," << m.num_agents << "," << m.horizon << ","
      << m.seed << "," << m.generated_tasks << "," << m.completed_tasks
      << "," << m.throughput << "," << m.final_pending_tasks << ","
      << m.final_assigned_tasks << "," << m.final_picked_tasks << ","
      << m.average_task_completion_time << "," << m.average_pickup_time << ","
      << m.average_delivery_time << "," << m.planner_invocations << ","
      << m.planner_success_count << "," << m.planner_timeout_count << ","
      << m.planner_failure_count << ","
      << m.planner_snapshot_infeasible_count << ","
      << m.planner_invalid_instance_count << ","
      << m.planner_empty_solution_count << "," << m.average_planner_runtime << ","
      << m.max_planner_runtime << "," << m.total_simulation_runtime << ","
      << m.average_agent_idle_time << "," << m.average_agent_loaded_time << ","
      << m.average_agent_unloaded_time << "," << m.valid << ","
      << '"' << m.error << '"' << "\n";
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
  config.task_config.goal_set_size = argc >= 9 ? std::stoi(argv[8]) : 5;
  config.task_config.outbound_probability =
      argc >= 10 ? std::stod(argv[9]) : 0.5;
  config.task_config.release_interval = argc >= 11 ? std::stoi(argv[10]) : 10;
  config.debug = argc >= 12 ? std::stoi(argv[11]) != 0 : false;
  const auto schedule_yaml = argc >= 13 ? std::string(argv[12]) : std::string();

  const auto metrics = run_lifelong_simulation(config);
  std::ofstream out(output_csv, std::ios::app);
  if (!out) {
    std::cerr << "failed to open output CSV: " << output_csv << "\n";
    return 1;
  }
  if (should_write_header(output_csv)) write_csv_header(out);
  write_csv_row(out, metrics);
  write_schedule_yaml(metrics, schedule_yaml);

  std::cout << "valid=" << metrics.valid << "\n";
  std::cout << "generated_tasks=" << metrics.generated_tasks << "\n";
  std::cout << "completed_tasks=" << metrics.completed_tasks << "\n";
  std::cout << "throughput=" << metrics.throughput << "\n";
  std::cout << "planner_invocations=" << metrics.planner_invocations << "\n";
  std::cout << "planner_success_count=" << metrics.planner_success_count << "\n";
  std::cout << "planner_timeout_count=" << metrics.planner_timeout_count << "\n";
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
