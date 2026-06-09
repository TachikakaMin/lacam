#include <filesystem>
#include <fstream>
#include <iostream>
#include <lacam.hpp>

namespace
{
void print_usage()
{
  std::cerr << "usage: lifelong_benchmark MAP NUM_AGENTS HORIZON SEED "
               "OUTPUT_CSV [CACHE] [TIME_LIMIT_SEC=2] [GOAL_SET_SIZE=5] "
               "[OUTBOUND_PROB=0.5] [RELEASE_INTERVAL=10] [DEBUG=0]\n";
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
      << m.planner_failure_count << "," << m.average_planner_runtime << ","
      << m.max_planner_runtime << "," << m.total_simulation_runtime << ","
      << m.average_agent_idle_time << "," << m.average_agent_loaded_time << ","
      << m.average_agent_unloaded_time << "," << m.valid << ","
      << '"' << m.error << '"' << "\n";
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

  const auto metrics = run_lifelong_simulation(config);
  std::ofstream out(output_csv, std::ios::app);
  if (!out) {
    std::cerr << "failed to open output CSV: " << output_csv << "\n";
    return 1;
  }
  if (should_write_header(output_csv)) write_csv_header(out);
  write_csv_row(out, metrics);

  std::cout << "valid=" << metrics.valid << "\n";
  std::cout << "generated_tasks=" << metrics.generated_tasks << "\n";
  std::cout << "completed_tasks=" << metrics.completed_tasks << "\n";
  std::cout << "throughput=" << metrics.throughput << "\n";
  std::cout << "planner_invocations=" << metrics.planner_invocations << "\n";
  std::cout << "planner_success_count=" << metrics.planner_success_count << "\n";
  std::cout << "planner_timeout_count=" << metrics.planner_timeout_count << "\n";
  std::cout << "planner_failure_count=" << metrics.planner_failure_count << "\n";
  if (!metrics.valid) std::cout << "error=" << metrics.error << "\n";
  return metrics.valid ? 0 : 1;
}
