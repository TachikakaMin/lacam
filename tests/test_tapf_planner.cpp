#include <lacam.hpp>

#include "gtest/gtest.h"

namespace {
bool is_tapf_feasible_solution(const TAPFInstance& ins,
                               const Solution& solution)
{
  if (solution.empty()) return false;
  if (!is_same_config(solution.front(), ins.starts)) return false;

  for (size_t t = 1; t < solution.size(); ++t) {
    for (size_t i = 0; i < ins.N; ++i) {
      auto v_i_from = solution[t - 1][i];
      auto v_i_to = solution[t][i];
      if (v_i_from != v_i_to &&
          std::find(v_i_to->neighbor.begin(), v_i_to->neighbor.end(),
                    v_i_from) == v_i_to->neighbor.end()) {
        return false;
      }

      for (size_t j = i + 1; j < ins.N; ++j) {
        auto v_j_from = solution[t - 1][j];
        auto v_j_to = solution[t][j];
        if (v_j_to == v_i_to) return false;
        if (v_j_to == v_i_from && v_j_from == v_i_to) return false;
      }
    }
  }

  auto used_tasks = std::vector<bool>(ins.tasks.size(), false);
  auto C = solution.back();
  for (size_t i = 0; i < ins.N; ++i) {
    auto matched = false;
    for (size_t j = 0; j < ins.tasks.size(); ++j) {
      if (used_tasks[j] || !ins.allowed[i][j] || C[i] != ins.tasks[j]) {
        continue;
      }
      used_tasks[j] = true;
      matched = true;
      break;
    }
    if (!matched) return false;
  }
  return true;
}
}  // namespace

TEST(tapf_planner, solve_shared_task_set)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto starts = std::vector<int>{
      8 * 0 + 0,
      8 * 0 + 1,
      8 * 1 + 0,
  };
  const auto tasks = std::vector<std::vector<int> >{
      {8 * 7 + 7, 8 * 7 + 6, 8 * 6 + 7},
      {8 * 7 + 7, 8 * 7 + 6, 8 * 6 + 7},
      {8 * 7 + 7, 8 * 7 + 6, 8 * 6 + 7},
  };
  const auto ins = TAPFInstance(map_filename, starts, tasks);

  ASSERT_TRUE(ins.is_valid());
  auto solution = solve_tapf(ins);
  ASSERT_TRUE(is_tapf_feasible_solution(ins, solution));
}

TEST(tapf_planner, solve_ita_cbs_yaml_fixture)
{
  const auto yaml_filename = "./third_party/ITA-CBS2/map_file/debug_cbs_data.yaml";
  const auto map_dir = "./third_party/ITA-CBS2/map_file";
  const auto ins = TAPFInstance(yaml_filename, map_dir);

  ASSERT_TRUE(ins.is_valid());
  auto solution = solve_tapf(ins);
  ASSERT_TRUE(is_tapf_feasible_solution(ins, solution));
}
