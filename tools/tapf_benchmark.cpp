#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <lacam.hpp>
#include <sstream>
#include <tuple>
#include <vector>

namespace
{
  TAPFSearchMode parse_search_mode(const std::string& value)
  {
    if (value == "1" || value == "focal" || value == "FOCAL") {
      return TAPFSearchMode::FOCAL;
    }
    return TAPFSearchMode::DFS;
  }

  TAPFFocalTieBreak parse_focal_tie_break(const std::string& value)
  {
    if (value == "1" || value == "anti_wait") {
      return TAPFFocalTieBreak::ANTI_WAIT;
    }
    if (value == "2" || value == "anti_zigzag") {
      return TAPFFocalTieBreak::ANTI_ZIGZAG;
    }
    if (value == "3" || value == "anti_push") {
      return TAPFFocalTieBreak::ANTI_PUSH;
    }
    if (value == "4" || value == "anti_all") {
      return TAPFFocalTieBreak::ANTI_ALL;
    }
    return TAPFFocalTieBreak::H;
  }

  std::string search_mode_name(const TAPFSearchMode mode)
  {
    return mode == TAPFSearchMode::FOCAL ? "focal" : "dfs";
  }

  std::string focal_tie_break_name(const TAPFFocalTieBreak tie_break)
  {
    switch (tie_break) {
      case TAPFFocalTieBreak::ANTI_WAIT:
        return "anti_wait";
      case TAPFFocalTieBreak::ANTI_ZIGZAG:
        return "anti_zigzag";
      case TAPFFocalTieBreak::ANTI_PUSH:
        return "anti_push";
      case TAPFFocalTieBreak::ANTI_ALL:
        return "anti_all";
      case TAPFFocalTieBreak::H:
      default:
        return "h";
    }
  }

  std::vector<std::string> split_csv(const std::string& value)
  {
    auto result = std::vector<std::string>();
    auto input = std::stringstream(value);
    auto item = std::string();
    while (std::getline(input, item, ',')) result.push_back(item);
    return result;
  }

  bool parse_motion_actions(const std::string& value, MotionActionSet& actions)
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

  bool parse_motion_costs(const std::string& value, MotionActionCosts& costs)
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

  struct ValidationResult {
    bool start_valid = false;
    bool moves_valid = false;
    bool collision_free = false;
    bool goal_valid = false;
    bool unique_goal_assignment = false;

    bool valid_solution() const
    {
      return start_valid && moves_valid && collision_free && goal_valid &&
             unique_goal_assignment;
    }
  };

  ValidationResult validate_tapf_solution(const TAPFInstance& ins,
                                          const Solution& solution)
  {
    auto result = ValidationResult();
    if (solution.empty()) return result;

    result.start_valid = is_same_config(solution.front(), ins.starts);
    result.moves_valid = true;
    result.collision_free = true;
    for (size_t t = 1; t < solution.size(); ++t) {
      for (size_t i = 0; i < ins.N; ++i) {
        auto v_i_from = solution[t - 1][i];
        auto v_i_to = solution[t][i];
        if (v_i_from != v_i_to &&
            std::find(v_i_to->neighbor.begin(), v_i_to->neighbor.end(),
                      v_i_from) == v_i_to->neighbor.end()) {
          result.moves_valid = false;
        }
        for (size_t j = i + 1; j < ins.N; ++j) {
          auto v_j_from = solution[t - 1][j];
          auto v_j_to = solution[t][j];
          if (v_i_to == v_j_to) result.collision_free = false;
          if (v_i_from == v_j_to && v_i_to == v_j_from) {
            result.collision_free = false;
          }
        }
      }
    }

    auto used_tasks = std::vector<bool>(ins.tasks.size(), false);
    const auto& C = solution.back();
    result.goal_valid = true;
    result.unique_goal_assignment = true;
    for (size_t i = 0; i < ins.N; ++i) {
      auto matched = false;
      for (size_t j = 0; j < ins.tasks.size(); ++j) {
        if (!ins.allowed[i][j] || C[i] != ins.tasks[j]) {
          continue;
        }
        if (used_tasks[j]) result.unique_goal_assignment = false;
        used_tasks[j] = true;
        matched = true;
        break;
      }
      if (!matched) result.goal_valid = false;
    }
    return result;
  }

  ValidationResult validate_motion_solution(
      const TAPFInstance& ins, const MotionParameters& parameters,
      const Solution& solution, const MotionSolution& motion_solution)
  {
    auto result = ValidationResult();
    if (solution.empty() || solution.size() != motion_solution.size())
      return result;
    const auto motion = MotionGraph(ins.G, parameters);
    result.start_valid = is_same_config(solution.front(), ins.starts);
    result.moves_valid = true;
    result.collision_free = true;
    for (size_t i = 0; i < ins.N; ++i) {
      const auto heading =
          i < ins.start_headings.size() ? ins.start_headings[i] : 0;
      result.start_valid =
          result.start_valid && motion_solution.front()[i].id ==
                                    motion.state_id(ins.starts[i], heading);
    }
    for (size_t t = 1; t < motion_solution.size(); ++t) {
      auto occupied = std::vector<int>(ins.G.width * ins.G.height, -1);
      for (size_t i = 0; i < ins.N; ++i) {
        const auto edge = motion.transition(motion_solution[t - 1][i].id,
                                            motion_solution[t][i].id);
        if (edge == nullptr) {
          result.moves_valid = false;
          continue;
        }
        auto swept = edge->swept_cells;
        if (!parameters.follower_collisions && swept.size() > 1)
          swept.erase(swept.begin());
        for (const auto cell : swept) {
          if (occupied[cell] >= 0 && occupied[cell] != static_cast<int>(i))
            result.collision_free = false;
          occupied[cell] = i;
        }
      }
    }
    auto used_tasks = std::vector<bool>(ins.tasks.size(), false);
    result.goal_valid = true;
    result.unique_goal_assignment = true;
    for (size_t i = 0; i < ins.N; ++i) {
      auto matched = false;
      for (size_t task = 0; task < ins.tasks.size(); ++task) {
        const auto heading =
            task < ins.task_headings.size() ? ins.task_headings[task] : -1;
        if (!ins.allowed[i][task] ||
            !motion.is_goal(motion_solution.back()[i].id, ins.tasks[task],
                            heading))
          continue;
        if (used_tasks[task]) result.unique_goal_assignment = false;
        used_tasks[task] = true;
        matched = true;
        break;
      }
      if (!matched) result.goal_valid = false;
    }
    return result;
  }

  int get_tapf_sum_of_loss(const Solution& solution)
  {
    if (solution.empty()) return 0;
    int cost = 0;
    const auto N = solution.front().size();
    const auto T = solution.size();
    for (size_t i = 0; i < N; ++i) {
      const auto goal = solution.back()[i];
      for (size_t t = 1; t < T; ++t) {
        if (solution[t - 1][i] != goal || solution[t][i] != goal) ++cost;
      }
    }
    return cost;
  }

  void write_u32(std::ofstream& out, const uint32_t value)
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

  void write_schedule_binary(const TAPFInstance& ins, const Solution& solution,
                             const std::string& binary_path)
  {
    std::ofstream out(binary_path, std::ios::binary);
    const char magic[8] = {'T', 'A', 'P', 'F', 'S', 'C', 'H', '1'};
    out.write(magic, sizeof(magic));
    write_u32(out, static_cast<uint32_t>(ins.N));
    write_u32(out, static_cast<uint32_t>(get_makespan(solution)));
    for (size_t i = 0; i < ins.N; ++i) {
      auto changes = std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>();
      auto last = static_cast<Vertex*>(nullptr);
      for (size_t t = 0; t < solution.size(); ++t) {
        auto v = solution[t][i];
        if (last == v) continue;
        last = v;
        changes.emplace_back(static_cast<uint32_t>(t),
                             static_cast<uint32_t>(v->index / ins.G.width),
                             static_cast<uint32_t>(v->index % ins.G.width));
      }
      write_u32(out, static_cast<uint32_t>(changes.size()));
      for (const auto& [t, x, y] : changes) {
        write_u32(out, t);
        write_u32(out, x);
        write_u32(out, y);
      }
    }
  }

  void write_schedule_output(const TAPFInstance& ins, const Solution& solution,
                             const std::string& output_path,
                             const MotionSolution& motion_solution = {},
                             const MotionParameters& motion_parameters = {})
  {
    if (output_path.empty() || solution.empty()) return;
    const auto binary_path = binary_schedule_path(output_path);
    write_schedule_binary(ins, solution, binary_path);

    std::ofstream out(output_path);
    out << "statistics:\n";
    out << "  cost: " << get_sum_of_costs(solution) << "\n";
    out << "  makespan: " << get_makespan(solution) << "\n";
    out << "  sum_of_loss: " << get_tapf_sum_of_loss(solution) << "\n";
    out << "assignments:\n";
    const auto& final_config = solution.back();
    for (size_t i = 0; i < ins.N; ++i) {
      auto goal = final_config[i];
      out << "  agent" << i << ":\n";
      out << "    x: " << goal->index / ins.G.width << "\n";
      out << "    y: " << goal->index % ins.G.width << "\n";
    }
    if (motion_solution.size() == solution.size()) {
      out << "motion_parameters:\n";
      out << "  max_speed: " << motion_parameters.max_speed << "\n";
      out << "  rotation_steps: " << motion_parameters.rotation_steps << "\n";
      out << "motion_schedule:\n";
      for (size_t t = 0; t < motion_solution.size(); ++t) {
        out << "  - timestep: " << t << "\n";
        out << "    states:\n";
        for (const auto& state : motion_solution[t]) {
          out << "      - [" << state.location->index / ins.G.width << ", "
              << state.location->index % ins.G.width << ", " << state.heading
              << ", " << state.speed << ", " << state.omega << "]\n";
        }
      }
    }
    out << "schedule_binary:\n";
    out << "  format: tapf_sparse_schedule_v1\n";
    out << "  encoding: little_endian_u32\n";
    out << "  path: " << binary_schedule_metadata_path(binary_path) << "\n";
    out << "  agents: " << ins.N << "\n";
    out << "  makespan: " << get_makespan(solution) << "\n";
  }
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "usage: tapf_benchmark YAML MAP_DIR [TIME_LIMIT_SEC] "
                 "[SCHEDULE_YAML] [ANYTIME=1] [FULL_TA=0] [SEED=-1] "
                 "[SEARCH_MODE=dfs] [FOCAL_WEIGHT=1.5] "
                 "[FOCAL_TIE_BREAK=h] [MOTION=0] [MAX_SPEED=2] "
                 "[ROTATION_STEPS=2] [PATH_LENGTH=6] [ACTIONS=all] "
                 "[ACTION_COSTS=1,1,1,1,0,0,0] [FOLLOWER=1] "
                 "[MAP_DISTANCE_CACHE=] [MOTION_PATH_CACHE=]\n";
    return 2;
  }
  const auto yaml_filename = std::string(argv[1]);
  const auto map_dir = std::string(argv[2]);
  const auto time_limit_sec = argc >= 4 ? std::stod(argv[3]) : 30.0;
  const auto output_path = argc >= 5 ? std::string(argv[4]) : std::string();
  const auto anytime = argc >= 6 ? std::stoi(argv[5]) != 0 : true;
  const auto force_full_assignment =
      argc >= 7 ? std::stoi(argv[6]) != 0 : false;
  const auto seed = argc >= 8 ? std::stoi(argv[7]) : -1;
  auto search_config = TAPFSearchConfig();
  if (argc >= 9) search_config.mode = parse_search_mode(argv[8]);
  if (argc >= 10) search_config.focal_weight = std::stod(argv[9]);
  if (argc >= 11)
    search_config.focal_tie_break = parse_focal_tie_break(argv[10]);
  if (argc >= 12) search_config.motion.enabled = std::stoi(argv[11]) != 0;
  if (argc >= 13) search_config.motion.max_speed = std::stoi(argv[12]);
  if (argc >= 14) search_config.motion.rotation_steps = std::stoi(argv[13]);
  if (argc >= 15) search_config.motion.lookahead_horizon = std::stoi(argv[14]);
  const auto action_names = argc >= 16 ? std::string(argv[15]) : "all";
  const auto action_costs =
      argc >= 17 ? std::string(argv[16]) : "1,1,1,1,0,0,0";
  if (!parse_motion_actions(action_names, search_config.motion.actions) ||
      !parse_motion_costs(action_costs, search_config.motion.costs)) {
    std::cerr << "invalid motion action list or costs\n";
    return 2;
  }
  if (argc >= 18)
    search_config.motion.follower_collisions = std::stoi(argv[17]) != 0;
  const auto map_distance_cache_path =
      argc >= 19 ? std::filesystem::path(argv[18]) : std::filesystem::path();
  const auto motion_path_cache_path =
      argc >= 20 ? std::filesystem::path(argv[19]) : std::filesystem::path();

  const auto ins = TAPFInstance(yaml_filename, map_dir);
  if (!ins.is_valid()) {
    std::cout << "valid_instance=0\n";
    return 1;
  }

  auto map_distance_load_ms = 0.0;
  if (search_config.motion.enabled && !map_distance_cache_path.empty()) {
    const auto started = std::chrono::steady_clock::now();
    const auto map_path = std::filesystem::path(ins.source_map_filename);
    const auto expected = MapDistanceCacheMetadata{
        map_path.filename().string(), hash_file(map_path), ins.G.width,
        ins.G.height, ins.G.size()};
    auto task_vertex_ids = std::vector<int>();
    task_vertex_ids.reserve(ins.tasks.size());
    for (const auto task : ins.tasks) task_vertex_ids.push_back(task->id);
    auto rows = std::make_shared<MapDistanceRows>();
    if (!load_map_distance_rows(map_distance_cache_path, expected,
                                task_vertex_ids, *rows)) {
      std::cerr << "invalid or missing map distance cache: "
                << map_distance_cache_path << "\n";
      return 2;
    }
    search_config.map_distance_rows = std::move(rows);
    map_distance_load_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
  }

  auto motion_graph_preprocess_ms = 0.0;
  auto motion_path_load_ms = 0.0;
  auto precomputed_motion = std::shared_ptr<MotionGraph>();
  if (search_config.motion.enabled &&
      (search_config.map_distance_rows != nullptr ||
       !motion_path_cache_path.empty())) {
    const auto started = std::chrono::steady_clock::now();
    precomputed_motion =
        std::make_shared<MotionGraph>(ins.G, search_config.motion);
    motion_graph_preprocess_ms = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - started)
                                     .count();
    if (!motion_path_cache_path.empty()) {
      const auto load_started = std::chrono::steady_clock::now();
      if (!precomputed_motion->load_path_cache(motion_path_cache_path)) {
        std::cerr << "invalid or missing motion path cache: "
                  << motion_path_cache_path << "\n";
        return 2;
      }
      motion_path_load_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - load_started)
                                .count();
    }
  }

  auto deadline = Deadline(time_limit_sec * 1000);
  auto stats = TAPFStats();
  auto motion_solution = MotionSolution();
  auto MT = std::mt19937(seed);
  auto solution =
      solve_tapf(ins, 0, &deadline, seed >= 0 ? &MT : nullptr, 0, &stats,
                 anytime, force_full_assignment, search_config, nullptr,
                 nullptr, &motion_solution, precomputed_motion);
  const auto runtime_ms = deadline.elapsed_ms();
  const auto validation =
      search_config.motion.enabled
          ? validate_motion_solution(ins, search_config.motion, solution,
                                     motion_solution)
          : validate_tapf_solution(ins, solution);
  const auto objective_soc =
      search_config.motion.enabled
          ? stats.solution_parent_edge_cost
          : static_cast<unsigned>(get_sum_of_costs(solution));
  write_schedule_output(ins, solution, output_path, motion_solution,
                        search_config.motion);

  std::cout << "valid_instance=1\n";
  std::cout << "solved=" << !solution.empty() << "\n";
  std::cout << "valid_solution=" << validation.valid_solution() << "\n";
  std::cout << "start_valid=" << validation.start_valid << "\n";
  std::cout << "moves_valid=" << validation.moves_valid << "\n";
  std::cout << "collision_free=" << validation.collision_free << "\n";
  std::cout << "goal_valid=" << validation.goal_valid << "\n";
  std::cout << "unique_goal_assignment=" << validation.unique_goal_assignment
            << "\n";
  std::cout << "runtime_ms=" << runtime_ms << "\n";
  std::cout << "map_distance_cache=" << (!map_distance_cache_path.empty())
            << "\n";
  std::cout << "map_distance_load_ms=" << map_distance_load_ms << "\n";
  std::cout << "motion_graph_preprocess_ms=" << motion_graph_preprocess_ms
            << "\n";
  std::cout << "motion_path_cache=" << (!motion_path_cache_path.empty())
            << "\n";
  std::cout << "motion_path_load_ms=" << motion_path_load_ms << "\n";
  std::cout << "makespan=" << get_makespan(solution) << "\n";
  std::cout << "soc=" << objective_soc << "\n";
  std::cout << "sum_of_loss=" << get_tapf_sum_of_loss(solution) << "\n";
  std::cout << "hl_loop_iterations=" << stats.hl_loop_iterations << "\n";
  std::cout << "hl_nodes_created=" << stats.hl_nodes_created << "\n";
  std::cout << "hl_nodes_explored=" << stats.hl_nodes_explored << "\n";
  std::cout << "hl_max_depth=" << stats.hl_max_depth << "\n";
  std::cout << "motion_best_satisfied_agents="
            << stats.motion_best_satisfied_agents << "\n";
  std::cout << "hl_reinsertions=" << stats.hl_reinsertions << "\n";
  std::cout << "hl_duplicate_configs=" << stats.hl_duplicate_configs << "\n";
  std::cout << "open_max_size=" << stats.open_max_size << "\n";
  std::cout << "solution_depth=" << stats.solution_depth << "\n";
  std::cout << "constraints_popped=" << stats.constraints_popped << "\n";
  std::cout << "constraints_generated=" << stats.constraints_generated << "\n";
  std::cout << "constraint_failures=" << stats.constraint_failures << "\n";
  std::cout << "pibt_calls=" << stats.pibt_calls << "\n";
  std::cout << "pibt_failures=" << stats.pibt_failures << "\n";
  std::cout << "pibt_recursions=" << stats.pibt_recursions << "\n";
  std::cout << "assignment_calls=" << stats.assignment_calls << "\n";
  std::cout << "initial_assignment_cost=" << stats.initial_assignment_cost
            << "\n";
  std::cout << "assignment_changes=" << stats.assignment_changes << "\n";
  std::cout << "final_assignment_changes=" << stats.final_assignment_changes
            << "\n";
  std::cout << "final_agent_assignment_changes="
            << stats.final_agent_assignment_changes << "\n";
  std::cout << "anytime=" << anytime << "\n";
  std::cout << "force_full_assignment=" << force_full_assignment << "\n";
  std::cout << "seed=" << seed << "\n";
  std::cout << "search_mode=" << search_mode_name(search_config.mode) << "\n";
  std::cout << "focal_weight=" << search_config.focal_weight << "\n";
  std::cout << "focal_tie_break="
            << focal_tie_break_name(search_config.focal_tie_break) << "\n";
  std::cout << "solution_cost=" << stats.solution_cost << "\n";
  std::cout << "first_solution_cost=" << stats.first_solution_cost << "\n";
  std::cout << "first_solution_time_ms=" << stats.first_solution_time_ms
            << "\n";
  std::cout << "motion=" << search_config.motion.enabled << "\n";
  std::cout << "max_speed=" << search_config.motion.max_speed << "\n";
  std::cout << "rotation_steps=" << search_config.motion.rotation_steps << "\n";
  std::cout << "path_length=" << search_config.motion.lookahead_horizon << "\n";
  std::cout << "motion_actions=" << action_names << "\n";
  std::cout << "motion_action_costs=" << action_costs << "\n";
  std::cout << "follower_collisions="
            << search_config.motion.follower_collisions << "\n";
  std::cout << "incumbent_updates=" << stats.incumbent_updates << "\n";
  std::cout << "solution_parent_edge_cost=" << stats.solution_parent_edge_cost
            << "\n";
  std::cout << "anytime_cost_updates=" << stats.anytime_cost_updates << "\n";
  std::cout << "swap_checks=" << stats.swap_checks << "\n";
  std::cout << "swap_applied=" << stats.swap_applied << "\n";
  std::cout << "assignment_time_ms=" << stats.assignment_time_ms << "\n";
  std::cout << "timed_out=" << stats.timed_out << "\n";
  return solution.empty() || !validation.valid_solution() ? 1 : 0;
}
