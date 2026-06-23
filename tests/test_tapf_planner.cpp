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

int vertex_occupancy(const Config& config, Vertex* vertex)
{
  return std::count(config.begin(), config.end(), vertex);
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

TEST(tapf_planner, assignment_uses_agent_target_cost_offsets)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{0, 6};
  const auto goals =
      std::vector<std::vector<int> >{{2, 4}, {2, 4}};
  const auto offsets =
      std::vector<std::vector<int> >{{10, 0}, {0, 10}};
  const auto ins = TAPFInstance(map_filename, starts, goals, offsets);
  auto distances = TAPFDistTable(ins);

  const auto assignment =
      assign_tapf_tasks(ins, distances, ins.starts);

  ASSERT_TRUE(assignment.feasible);
  ASSERT_EQ(ins.tasks[assignment.agent_to_task[0]]->index, 4);
  ASSERT_EQ(ins.tasks[assignment.agent_to_task[1]]->index, 2);

  auto state = TAPFAssignmentState();
  auto changed_agents = std::vector<int>{0, 1};
  const auto dynamic_assignment = assign_tapf_tasks_dynamic(
      ins, distances, ins.starts, state, changed_agents, true);
  ASSERT_TRUE(dynamic_assignment.feasible);
  ASSERT_EQ(ins.tasks[dynamic_assignment.agent_to_task[0]]->index, 4);
  ASSERT_EQ(ins.tasks[dynamic_assignment.agent_to_task[1]]->index, 2);
}

TEST(tapf_planner, assignment_uses_agent_target_distance_scales)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{0};
  const auto goals = std::vector<std::vector<int> >{{2, 4}};
  const auto offsets = std::vector<std::vector<int> >{{0, 0}};
  const auto scales = std::vector<std::vector<int> >{{10, 1}};
  const auto ins = TAPFInstance(map_filename, starts, goals, offsets, 1, scales);
  auto distances = TAPFDistTable(ins);

  const auto assignment = assign_tapf_tasks(ins, distances, ins.starts);

  ASSERT_TRUE(assignment.feasible);
  ASSERT_EQ(ins.tasks[assignment.agent_to_task[0]]->index, 4);
}

TEST(tapf_planner, unsolved_instance_returns_no_partial_path)
{
  const auto map_filename = "./tests/assets/2x1.map";
  const auto starts = std::vector<int>{0, 1};
  const auto goals = std::vector<std::vector<int> >{{1}, {0}};
  const auto ins = TAPFInstance(map_filename, starts, goals);

  ASSERT_TRUE(ins.is_valid());
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false, false,
                 TAPFSearchConfig(), &final_assignment);

  ASSERT_TRUE(solution.empty());
  ASSERT_TRUE(final_assignment.empty());
}

TEST(tapf_planner, normal_mode_rejects_duplicate_physical_goal_stack)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{0, 1};
  const auto goals = std::vector<std::vector<int> >{{2}, {2}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                true);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), starts.size());
  ASSERT_EQ(ins.tasks[0], ins.tasks[1]);
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, nullptr,
                                   false, false, TAPFSearchConfig());

  ASSERT_TRUE(solution.empty());
}

TEST(tapf_planner, service_mode_default_commits_first_real_service)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{0, 7};
  const auto goals =
      std::vector<std::vector<int> >{{2, 8}, {2, 8}};
  const auto ins = TAPFInstance(map_filename, starts, goals);

  ASSERT_TRUE(ins.is_valid());
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false, false,
                 search_config, &final_assignment);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(final_assignment.size(), starts.size());
  ASSERT_GT(stats.hl_nodes_created, 1);
  ASSERT_GT(stats.assignment_calls, 1);
  ASSERT_GT(stats.assignment_row_cache_requests, 0);
  ASSERT_GT(stats.assignment_row_cache_hits, 0);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
  ASSERT_GE(stats.service_best_satisfied_agents, 1);
}

TEST(tapf_planner,
     service_mode_allows_sequential_entries_to_one_physical_goal)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{0, 1};
  const auto goals = std::vector<std::vector<int> >{{2}, {2}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                true);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), starts.size());
  ASSERT_EQ(ins.tasks[0], ins.tasks[1]);
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto assignment_schedule = std::vector<std::vector<int> >();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = starts.size();
  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false, false,
                 search_config, &final_assignment, &assignment_schedule);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_best_satisfied_agents, 2);
  ASSERT_EQ(final_assignment.size(), starts.size());
  ASSERT_EQ(assignment_schedule.size(), solution.size());
  ASSERT_EQ(vertex_occupancy(solution.back(), ins.G.U[2]), 2)
      << "explicitly committing both services should prove the sequential "
         "shared-goal stack";
}

TEST(tapf_planner,
     service_mode_does_not_execute_simultaneous_entries_to_shared_goal)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{0, 2};
  const auto goals = std::vector<std::vector<int> >{{1}, {1}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                true);

  ASSERT_TRUE(ins.is_valid());
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = starts.size();
  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false, false,
                 search_config);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_best_satisfied_agents, 2);
  for (size_t t = 1; t < solution.size(); ++t) {
    auto entrants = 0;
    for (size_t i = 0; i < starts.size(); ++i) {
      if (solution[t - 1][i] != ins.G.U[1] && solution[t][i] == ins.G.U[1]) {
        ++entrants;
      }
    }
    ASSERT_LE(entrants, 1) << "two agents entered the shared service goal at t="
                           << t;
  }
}

TEST(tapf_planner, service_mode_does_not_stack_singleton_lifelong_service)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{0, 1};
  const auto goals = std::vector<std::vector<int> >{{1}, {1}};
  const auto task_keys =
      std::vector<std::vector<int> >{{1000000001}, {-2}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), starts.size());
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false, false,
                 search_config);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_satisfied_deliveries, 1);
  for (const auto& config : solution) {
    ASSERT_LE(vertex_occupancy(config, ins.G.U[1]), 1);
  }
}

TEST(tapf_planner, service_at_root_requires_a_committed_stay_transition)
{
  const auto map_filename = "./tests/assets/2x1.map";
  const auto ins = TAPFInstance(map_filename, std::vector<int>{0},
                                std::vector<std::vector<int> >{{0}});
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;

  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false, false,
                 search_config, &final_assignment);

  ASSERT_EQ(solution.size(), 2);
  ASSERT_EQ(solution[0][0], ins.G.U[0]);
  ASSERT_EQ(solution[1][0], ins.G.U[0]);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
  ASSERT_EQ(final_assignment.size(), 1);
}

TEST(tapf_planner, service_mode_allows_sequential_use_of_one_physical_goal)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{3, 9};
  const auto goals = std::vector<std::vector<int> >{{0}, {0}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                true);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), starts.size());
  ASSERT_EQ(ins.tasks[0], ins.tasks[1]);

  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = starts.size();
  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false, false,
                 search_config, &final_assignment);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_best_satisfied_agents, 2);
  ASSERT_EQ(final_assignment.size(), 2);
  ASSERT_NE(final_assignment[0], final_assignment[1]);
  ASSERT_EQ(ins.tasks[final_assignment[0]]->index, 0);
  ASSERT_EQ(ins.tasks[final_assignment[1]]->index, 0);
  // Both virtual task slots denote one physical vertex. The searched
  // continuation services them sequentially because only one agent may occupy
  // it per step; the returned execution prefix ends at the first service.
  ASSERT_EQ(std::count(solution.back().begin(), solution.back().end(),
                       ins.G.U[0]),
            1);
}
