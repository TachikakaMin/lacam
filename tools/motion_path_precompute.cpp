#include <chrono>
#include <filesystem>
#include <iostream>
#include <lacam.hpp>
#include <sstream>
#include <thread>

namespace
{
  std::vector<std::string> split_csv(const std::string& value)
  {
    auto result = std::vector<std::string>();
    auto input = std::stringstream(value);
    auto item = std::string();
    while (std::getline(input, item, ',')) result.push_back(item);
    return result;
  }

  bool parse_actions(const std::string& value, MotionActionSet& actions)
  {
    if (value == "all" || value == "paper") return true;
    actions = MotionActionSet{false, false, false, false, false, false, false};
    for (const auto& item : split_csv(value)) {
      if (item == "stay")
        actions.stay = true;
      else if (item == "forward")
        actions.forward = true;
      else if (item == "rotate_ccw")
        actions.rotate_ccw = true;
      else if (item == "rotate_cw")
        actions.rotate_cw = true;
      else if (item == "keep")
        actions.keep_speed = true;
      else if (item == "accelerate")
        actions.accelerate = true;
      else if (item == "decelerate")
        actions.decelerate = true;
      else
        return false;
    }
    return true;
  }

  bool parse_costs(const std::string& value, MotionActionCosts& costs)
  {
    const auto fields = split_csv(value);
    if (fields.size() != 7) return false;
    try {
      costs.stay = std::stoi(fields[0]);
      costs.forward = std::stoi(fields[1]);
      costs.rotate_ccw = std::stoi(fields[2]);
      costs.rotate_cw = std::stoi(fields[3]);
      costs.keep_speed = std::stoi(fields[4]);
      costs.accelerate = std::stoi(fields[5]);
      costs.decelerate = std::stoi(fields[6]);
    } catch (...) {
      return false;
    }
    return true;
  }
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3 || argc > 9) {
    std::cerr << "usage: motion_path_precompute MAP CACHE [MAX_SPEED=2] "
                 "[ROTATION_STEPS=2] [PATH_LENGTH=6] [ACTIONS=all] "
                 "[ACTION_COSTS=1,1,1,1,1,1,1] [WORKERS=auto]\n";
    return 2;
  }

  const auto map_path = std::filesystem::path(argv[1]);
  const auto cache_path = std::filesystem::path(argv[2]);
  auto parameters = MotionParameters();
  parameters.enabled = true;
  if (argc >= 4) parameters.max_speed = std::stoi(argv[3]);
  if (argc >= 5) parameters.rotation_steps = std::stoi(argv[4]);
  if (argc >= 6) parameters.lookahead_horizon = std::stoi(argv[5]);
  const auto actions = argc >= 7 ? std::string(argv[6]) : "all";
  const auto costs = argc >= 8 ? std::string(argv[7]) : "1,1,1,1,1,1,1";
  const auto workers =
      argc >= 9
          ? std::stoi(argv[8])
          : static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
  if (!parse_actions(actions, parameters.actions) ||
      !parse_costs(costs, parameters.costs)) {
    std::cerr << "invalid motion action list or costs\n";
    return 2;
  }

  try {
    const auto total_started = std::chrono::steady_clock::now();
    const auto graph_started = total_started;
    const auto graph = Graph(map_path.string());
    auto motion = MotionGraph(graph, parameters);
    const auto graph_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - graph_started)
                              .count();
    const auto paths_started = std::chrono::steady_clock::now();
    const auto loaded = motion.load_path_cache(cache_path);
    if (!loaded) {
      motion.precompute_path_candidates(workers);
      motion.save_path_cache(cache_path);
    }
    const auto paths_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - paths_started)
                              .count();
    const auto total_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - total_started)
                              .count();
    std::cout << "cache_status=" << (loaded ? "loaded" : "built") << "\n"
              << "map=" << map_path << "\n"
              << "cache=" << cache_path << "\n"
              << "motion_states=" << motion.size() << "\n"
              << "path_candidates=" << motion.path_candidate_count() << "\n"
              << "cache_bytes=" << std::filesystem::file_size(cache_path)
              << "\nworkers=" << workers << "\n"
              << "motion_graph_ms=" << graph_ms << "\n"
              << "path_cache_ms=" << paths_ms << "\n"
              << "elapsed_ms=" << total_ms << "\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
