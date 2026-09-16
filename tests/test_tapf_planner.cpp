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

TEST(tapf_planner, goal_cost_offsets_steer_assignment)
{
  const auto map_filename = "./assets/empty-8-8.map";
  // two agents, one shared task at (0,2); each agent also offers its own
  // start cell as a high-cost hold goal
  const auto starts = std::vector<int>{8 * 0 + 0, 8 * 0 + 3};
  const auto tasks = std::vector<std::vector<int> >{
      {8 * 0 + 2, 8 * 0 + 0},
      {8 * 0 + 2, 8 * 0 + 3},
  };
  const auto costs = std::vector<std::vector<int> >{
      {0, 10000},
      {0, 10000},
  };
  const auto ins = TAPFInstance(map_filename, starts, tasks, costs);
  ASSERT_TRUE(ins.is_valid());

  auto D = TAPFDistTable(ins);
  const auto res = assign_tapf_tasks(ins, D, ins.starts);
  ASSERT_TRUE(res.feasible);
  // nearest agent (a1, dist 1) takes the shared task; a0 holds
  const auto task_a0 = res.agent_to_task[0];
  const auto task_a1 = res.agent_to_task[1];
  ASSERT_EQ(ins.tasks[task_a1], ins.G.U[8 * 0 + 2]);
  ASSERT_EQ(ins.tasks[task_a0], ins.G.U[8 * 0 + 0]);

  // the planner must terminate with agents on that optimal matching
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, nullptr,
                                   /*anytime=*/false);
  ASSERT_TRUE(is_tapf_feasible_solution(ins, solution));
  ASSERT_EQ(solution.back()[1], ins.G.U[8 * 0 + 2]);
  ASSERT_EQ(solution.back()[0], ins.G.U[8 * 0 + 0]);

  // biasing the shared task against a1 flips the assignment to a0
  const auto biased_costs = std::vector<std::vector<int> >{
      {0, 10000},
      {10000, 0},
  };
  const auto ins2 = TAPFInstance(map_filename, starts, tasks, biased_costs);
  auto D2 = TAPFDistTable(ins2);
  const auto res2 = assign_tapf_tasks(ins2, D2, ins2.starts);
  ASSERT_TRUE(res2.feasible);
  ASSERT_EQ(ins2.tasks[res2.agent_to_task[0]], ins2.G.U[8 * 0 + 2]);
  ASSERT_EQ(ins2.tasks[res2.agent_to_task[1]], ins2.G.U[8 * 0 + 3]);
  const auto solution2 = solve_tapf(ins2, 0, nullptr, nullptr, 0, nullptr,
                                    /*anytime=*/false);
  ASSERT_TRUE(is_tapf_feasible_solution(ins2, solution2));
  ASSERT_EQ(solution2.back()[0], ins2.G.U[8 * 0 + 2]);
  ASSERT_EQ(solution2.back()[1], ins2.G.U[8 * 0 + 3]);
}
