#include <lacam.hpp>

#include <iostream>

namespace {
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
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "usage: tapf_benchmark YAML MAP_DIR [TIME_LIMIT_SEC]\n";
    return 2;
  }
  const auto yaml_filename = std::string(argv[1]);
  const auto map_dir = std::string(argv[2]);
  const auto time_limit_sec = argc >= 4 ? std::stod(argv[3]) : 30.0;

  const auto ins = TAPFInstance(yaml_filename, map_dir);
  if (!ins.is_valid()) {
    std::cout << "valid_instance=0\n";
    return 1;
  }

  auto deadline = Deadline(time_limit_sec * 1000);
  auto stats = TAPFStats();
  auto solution = solve_tapf(ins, 0, &deadline, nullptr, 0, &stats);
  const auto runtime_ms = deadline.elapsed_ms();
  const auto validation = validate_tapf_solution(ins, solution);

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
  std::cout << "makespan=" << get_makespan(solution) << "\n";
  std::cout << "soc=" << get_sum_of_costs(solution) << "\n";
  std::cout << "hl_loop_iterations=" << stats.hl_loop_iterations << "\n";
  std::cout << "hl_nodes_created=" << stats.hl_nodes_created << "\n";
  std::cout << "hl_nodes_explored=" << stats.hl_nodes_explored << "\n";
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
  std::cout << "assignment_changes=" << stats.assignment_changes << "\n";
  std::cout << "assignment_time_ms=" << stats.assignment_time_ms << "\n";
  std::cout << "timed_out=" << stats.timed_out << "\n";
  return solution.empty() || !validation.valid_solution() ? 1 : 0;
}
