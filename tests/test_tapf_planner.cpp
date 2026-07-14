#include <lacam.hpp>

#include "gtest/gtest.h"

namespace
{
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

TEST(tapf_planner, motion_mode_keeps_task_assignment_at_every_high_level_node)
{
  const auto ins = TAPFInstance("./tests/assets/5x1.map", {0}, {{4}}, {}, 1, {},
                                {}, false, {}, {}, {0}, {{0}});
  ASSERT_TRUE(ins.is_valid());
  auto config = TAPFSearchConfig();
  config.motion.enabled = true;
  config.motion.max_speed = 2;
  config.motion.rotation_steps = 2;
  config.motion.lookahead_horizon = 6;
  const auto all_pairs = build_map_distance_cache(ins.G, "5x1.map", 0, 2);
  auto goal_rows = std::make_shared<MapDistanceRows>();
  goal_rows->metadata = all_pairs.metadata;
  for (const auto task : ins.tasks) {
    goal_rows->row_by_vertex_id[task->id] = goal_rows->rows.size();
    goal_rows->rows.push_back(all_pairs.distances[task->id]);
  }
  config.map_distance_rows = goal_rows;
  auto deadline = Deadline(1000);
  auto random = std::mt19937(7);
  auto stats = TAPFStats();
  auto motion_solution = MotionSolution();
  const auto solution =
      solve_tapf(ins, 0, &deadline, &random, 0, &stats, false, true, config,
                 nullptr, nullptr, &motion_solution);
  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(solution.size(), motion_solution.size());
  EXPECT_EQ(solution.back()[0], ins.tasks[0]);
  EXPECT_EQ(motion_solution.back()[0].speed, 0);
  EXPECT_EQ(motion_solution.back()[0].heading, 0);
  EXPECT_EQ(stats.assignment_calls, stats.hl_nodes_created);

  const auto motion = MotionGraph(ins.G, config.motion);
  for (size_t t = 1; t < motion_solution.size(); ++t) {
    EXPECT_NE(motion.transition(motion_solution[t - 1][0].id,
                                motion_solution[t][0].id),
              nullptr);
  }
}

TEST(tapf_planner, motion_mode_prevents_swept_and_follower_collisions)
{
  const auto ins = TAPFInstance("./tests/assets/motion-5x1.yaml");
  ASSERT_TRUE(ins.is_valid());
  auto config = TAPFSearchConfig();
  config.motion.enabled = true;
  auto deadline = Deadline(1000);
  auto random = std::mt19937(9);
  auto motion_solution = MotionSolution();
  const auto solution =
      solve_tapf(ins, 0, &deadline, &random, 0, nullptr, false, true, config,
                 nullptr, nullptr, &motion_solution);
  ASSERT_FALSE(solution.empty());
  const auto motion = MotionGraph(ins.G, config.motion);
  for (size_t t = 1; t < motion_solution.size(); ++t) {
    auto occupied = std::vector<int>(ins.G.width * ins.G.height, -1);
    for (size_t i = 0; i < ins.N; ++i) {
      const auto edge = motion.transition(motion_solution[t - 1][i].id,
                                          motion_solution[t][i].id);
      ASSERT_NE(edge, nullptr);
      for (const auto cell : edge->swept_cells) {
        EXPECT_EQ(occupied[cell], -1);
        occupied[cell] = i;
      }
    }
  }
}

TEST(tapf_planner, solve_ita_cbs_yaml_fixture)
{
  const auto yaml_filename =
      "./third_party/ITA-CBS2/map_file/debug_cbs_data.yaml";
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
  const auto goals = std::vector<std::vector<int> >{{2, 4}, {2, 4}};
  const auto offsets = std::vector<std::vector<int> >{{10, 0}, {0, 10}};
  const auto ins = TAPFInstance(map_filename, starts, goals, offsets);
  auto distances = TAPFDistTable(ins);

  const auto assignment = assign_tapf_tasks(ins, distances, ins.starts);

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
  const auto ins =
      TAPFInstance(map_filename, starts, goals, offsets, 1, scales);
  auto distances = TAPFDistTable(ins);

  const auto assignment = assign_tapf_tasks(ins, distances, ins.starts);

  ASSERT_TRUE(assignment.feasible);
  ASSERT_EQ(ins.tasks[assignment.agent_to_task[0]]->index, 4);
}

TEST(tapf_planner, motion_assignment_cache_keys_full_motion_state)
{
  const auto ins = TAPFInstance("./tests/assets/5x1.map", {0}, {{4}});
  auto distances = TAPFDistTable(ins);
  auto state = TAPFAssignmentState();
  const auto changed_agents = std::vector<int>{0};

  const auto first = assign_tapf_tasks_dynamic(
      ins, distances, ins.starts, state, changed_agents, true, nullptr, {}, {},
      {}, {10}, [](int, int, Vertex*) { return 5; });
  ASSERT_TRUE(first.feasible);
  EXPECT_EQ(first.cost, 5);

  // The physical location is unchanged, but heading/speed/omega may produce a
  // different motion state and therefore a different weighted distance row.
  const auto second = assign_tapf_tasks_dynamic(
      ins, distances, ins.starts, state, changed_agents, true, nullptr, {}, {},
      {}, {11}, [](int, int, Vertex*) { return 9; });
  ASSERT_TRUE(second.feasible);
  EXPECT_EQ(second.cost, 9);
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
  const auto ins =
      TAPFInstance(map_filename, starts, goals, {}, 1, {}, {}, true);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), starts.size());
  ASSERT_EQ(ins.tasks[0], ins.tasks[1]);
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, nullptr, false,
                                   false, TAPFSearchConfig());

  ASSERT_TRUE(solution.empty());
}

TEST(tapf_planner, service_mode_default_commits_first_real_service)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{0, 7};
  const auto goals = std::vector<std::vector<int> >{{2, 8}, {2, 8}};
  const auto ins = TAPFInstance(map_filename, starts, goals);

  ASSERT_TRUE(ins.is_valid());
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config, &final_assignment);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(final_assignment.size(), starts.size());
  ASSERT_GT(stats.hl_nodes_created, 1);
  ASSERT_GT(stats.assignment_calls, 1);
  ASSERT_GT(stats.assignment_row_cache_requests, 0);
  ASSERT_GT(stats.assignment_row_cache_hits, 0);
  ASSERT_GE(stats.service_satisfied_agents, 1);
  ASSERT_GE(stats.service_best_satisfied_agents, 1);
}

TEST(tapf_planner, service_mode_default_cuts_at_first_service_completion)
{
  const auto map_filename = "./tests/assets/5x1.map";
  const auto starts = std::vector<int>{0, 4};
  const auto goals = std::vector<std::vector<int> >{{1}, {2}};
  const auto task_keys = std::vector<std::vector<int> >{{0}, {1}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys);

  ASSERT_TRUE(ins.is_valid());
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.pickup_service_duration = 2;
  search_config.delivery_service_duration = 2;
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_satisfied_agents, 1);
  auto agent1_entry = solution.size();
  for (size_t t = 1; t < solution.size(); ++t) {
    if (solution[t - 1][1] != ins.G.U[2] && solution[t][1] == ins.G.U[2]) {
      agent1_entry = t;
      break;
    }
  }
  ASSERT_LT(agent1_entry, solution.size());
  ASSERT_EQ(solution.size(), agent1_entry + 2)
      << "lifelong replanning consumes only the prefix ending at the first "
         "completed service; later started services continue via partial state";
  ASSERT_EQ(solution.back()[1], ins.G.U[2]);
}

TEST(tapf_planner, service_mode_allows_sequential_entries_to_one_physical_goal)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{3, 9};
  const auto goals = std::vector<std::vector<int> >{{0}, {0}};
  const auto ins =
      TAPFInstance(map_filename, starts, goals, {}, 1, {}, {}, true);

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
  ASSERT_EQ(vertex_occupancy(solution.back(), ins.G.U[0]), 1)
      << "explicitly committing both services should prove sequential service "
         "without stacking agents during an active service";
}

TEST(tapf_planner,
     service_mode_does_not_execute_simultaneous_entries_to_shared_goal)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{0, 2};
  const auto goals = std::vector<std::vector<int> >{{1}, {1}};
  const auto ins =
      TAPFInstance(map_filename, starts, goals, {}, 1, {}, {}, true);

  ASSERT_TRUE(ins.is_valid());
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = starts.size();
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

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

TEST(tapf_planner, service_mode_blocks_same_target_while_occupant_services)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{0, 1};
  const auto goals = std::vector<std::vector<int> >{{1}, {1}};
  const auto task_keys = std::vector<std::vector<int> >{{1000000001}, {-2}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), starts.size());
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_satisfied_deliveries, 1);
  ASSERT_EQ(vertex_occupancy(solution.back(), ins.G.U[1]), 1);
  for (size_t t = 1; t < solution.size(); ++t) {
    auto entrants = 0;
    for (size_t i = 0; i < starts.size(); ++i) {
      if (solution[t - 1][i] != ins.G.U[1] && solution[t][i] == ins.G.U[1]) {
        ++entrants;
      }
    }
    ASSERT_LE(entrants, 1);
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

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config, &final_assignment);

  ASSERT_EQ(solution.size(), 2);
  ASSERT_EQ(solution[0][0], ins.G.U[0]);
  ASSERT_EQ(solution[1][0], ins.G.U[0]);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
  ASSERT_EQ(final_assignment.size(), 1);
}

TEST(tapf_planner, service_duration_at_root_requires_n_stays)
{
  const auto map_filename = "./tests/assets/2x1.map";
  const auto ins = TAPFInstance(map_filename, std::vector<int>{0},
                                std::vector<std::vector<int> >{{0}});
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.pickup_service_duration = 3;
  search_config.delivery_service_duration = 3;

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config, &final_assignment);

  ASSERT_EQ(solution.size(), 4);
  for (size_t t = 0; t < solution.size(); ++t) {
    ASSERT_EQ(solution[t][0], ins.G.U[0]);
  }
  ASSERT_EQ(stats.service_satisfied_agents, 1);
  ASSERT_EQ(final_assignment.size(), 1);
}

TEST(tapf_planner, service_duration_blocks_vertex_until_complete)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{1, 0};
  const auto goals = std::vector<std::vector<int> >{{1}, {2}};
  const auto task_keys = std::vector<std::vector<int> >{{0}, {1}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys);
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = 1;
  search_config.pickup_service_duration = 3;
  search_config.delivery_service_duration = 3;

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_EQ(solution.size(), 4);
  for (size_t t = 1; t <= 3; ++t) {
    ASSERT_EQ(solution[t][0], ins.G.U[1]) << "service agent left at t=" << t;
    ASSERT_NE(solution[t][1], ins.G.U[1])
        << "transiting agent entered blocked service vertex at t=" << t;
  }
  ASSERT_EQ(stats.service_satisfied_agents, 1);
}

TEST(tapf_planner, unit_service_duration_blocks_vertex_for_committed_stay)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{1, 0};
  const auto goals = std::vector<std::vector<int> >{{1}, {2}};
  const auto task_keys = std::vector<std::vector<int> >{{0}, {1}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys);
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = 1;
  search_config.pickup_service_duration = 1;
  search_config.delivery_service_duration = 1;

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_EQ(solution.size(), 2);
  ASSERT_EQ(solution[1][0], ins.G.U[1]);
  ASSERT_NE(solution[1][1], ins.G.U[1])
      << "duration=1 still requires one committed service stay";
  ASSERT_EQ(stats.service_satisfied_agents, 1);
}

TEST(tapf_planner, initial_service_progress_requires_remaining_stays)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{1, 0};
  const auto goals = std::vector<std::vector<int> >{{1}, {2}};
  const auto task_keys = std::vector<std::vector<int> >{{0}, {1}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys);
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = 1;
  search_config.pickup_service_duration = 3;
  search_config.delivery_service_duration = 3;
  search_config.initial_service_assignments = {0, -1};
  search_config.initial_service_progress = {2, 0};

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_EQ(solution.size(), 2);
  ASSERT_EQ(solution[1][0], ins.G.U[1]);
  ASSERT_NE(solution[1][1], ins.G.U[1]);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
}

TEST(tapf_planner,
     optional_partial_service_continuation_uses_remaining_duration)
{
  const auto map_filename = "./tests/assets/2x1.map";
  const auto starts = std::vector<int>{0};
  const auto goals = std::vector<std::vector<int> >{{0}};
  const auto task_keys = std::vector<std::vector<int> >{{0}};
  const auto service_durations = std::vector<std::vector<int> >{{3}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys, service_durations);
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = 1;
  search_config.pickup_service_duration = 3;
  search_config.delivery_service_duration = 3;
  search_config.initial_optional_service_assignments = {0};
  search_config.initial_optional_service_remaining = {1};

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_EQ(solution.size(), 2);
  ASSERT_EQ(solution[0][0], ins.G.U[0]);
  ASSERT_EQ(solution[1][0], ins.G.U[0]);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
}

TEST(tapf_planner, optional_partial_service_can_be_abandoned)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{1};
  const auto goals = std::vector<std::vector<int> >{{1, 2}};
  const auto offsets = std::vector<std::vector<int> >{{100, 0}};
  const auto task_keys = std::vector<std::vector<int> >{{0, 1}};
  const auto service_durations = std::vector<std::vector<int> >{{3, 3}};
  const auto ins = TAPFInstance(map_filename, starts, goals, offsets, 1, {}, {},
                                false, task_keys, service_durations);
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = 1;
  search_config.pickup_service_duration = 3;
  search_config.delivery_service_duration = 3;
  search_config.initial_optional_service_assignments = {0};
  search_config.initial_optional_service_remaining = {2};

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config, &final_assignment);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(final_assignment.size(), 1);
  ASSERT_EQ(ins.tasks[final_assignment[0]], ins.G.U[2]);
  ASSERT_EQ(solution.back()[0], ins.G.U[2]);
}

TEST(tapf_planner, optional_partial_service_changes_root_assignment_cost)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{1};
  const auto goals = std::vector<std::vector<int> >{{1, 2}};
  const auto offsets = std::vector<std::vector<int> >{{0, 0}};
  const auto scales = std::vector<std::vector<int> >{{1, 1}};
  const auto task_keys = std::vector<std::vector<int> >{{0, 1}};
  const auto service_durations = std::vector<std::vector<int> >{{5, 1}};
  const auto ins = TAPFInstance(map_filename, starts, goals, offsets, 1, scales,
                                {}, false, task_keys, service_durations);

  auto no_partial_stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = 1;
  search_config.pickup_service_duration = 5;
  search_config.delivery_service_duration = 5;
  const auto no_partial_solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &no_partial_stats, false, false,
                 search_config);

  ASSERT_FALSE(no_partial_solution.empty());
  ASSERT_EQ(no_partial_stats.initial_assignment.size(), 1);
  ASSERT_EQ(ins.tasks[no_partial_stats.initial_assignment[0]], ins.G.U[2]);

  auto partial_stats = TAPFStats();
  search_config.initial_optional_service_assignments = {0};
  search_config.initial_optional_service_remaining = {1};
  const auto partial_solution = solve_tapf(
      ins, 0, nullptr, nullptr, 0, &partial_stats, false, false, search_config);

  ASSERT_FALSE(partial_solution.empty());
  ASSERT_EQ(partial_stats.initial_assignment.size(), 1);
  ASSERT_EQ(ins.tasks[partial_stats.initial_assignment[0]], ins.G.U[1]);
}

TEST(tapf_planner, service_duration_counts_consecutive_stays_after_arrival)
{
  const auto map_filename = "./tests/assets/2x1.map";
  const auto ins = TAPFInstance(map_filename, std::vector<int>{0},
                                std::vector<std::vector<int> >{{1}});
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.pickup_service_duration = 2;
  search_config.delivery_service_duration = 2;

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_EQ(solution.size(), 4);
  ASSERT_EQ(solution[0][0], ins.G.U[0]);
  ASSERT_EQ(solution[1][0], ins.G.U[1]);
  ASSERT_EQ(solution[2][0], ins.G.U[1]);
  ASSERT_EQ(solution[3][0], ins.G.U[1]);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
}

TEST(tapf_planner, service_duration_zero_completes_on_arrival)
{
  const auto map_filename = "./tests/assets/2x1.map";
  const auto ins = TAPFInstance(map_filename, std::vector<int>{0},
                                std::vector<std::vector<int> >{{1}});
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.pickup_service_duration = 0;
  search_config.delivery_service_duration = 0;

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_EQ(solution.size(), 2);
  ASSERT_EQ(solution[0][0], ins.G.U[0]);
  ASSERT_EQ(solution[1][0], ins.G.U[1]);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
}

TEST(tapf_planner, service_duration_zero_at_root_completes_without_stay)
{
  const auto map_filename = "./tests/assets/2x1.map";
  const auto ins = TAPFInstance(map_filename, std::vector<int>{1},
                                std::vector<std::vector<int> >{{1}});
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.pickup_service_duration = 0;
  search_config.delivery_service_duration = 0;

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config, &final_assignment);

  ASSERT_EQ(solution.size(), 1);
  ASSERT_EQ(solution[0][0], ins.G.U[1]);
  ASSERT_EQ(stats.service_satisfied_agents, 1);
  ASSERT_EQ(final_assignment.size(), 1);
}

TEST(tapf_planner, non_real_wait_target_does_not_use_service_duration)
{
  const auto map_filename = "./tests/assets/3x1.map";
  const auto starts = std::vector<int>{0, 2};
  const auto goals = std::vector<std::vector<int> >{{0}, {1}};
  const auto task_keys = std::vector<std::vector<int> >{{-2}, {1000000001}};
  const auto ins = TAPFInstance(map_filename, starts, goals, {}, 1, {}, {},
                                false, task_keys);
  auto stats = TAPFStats();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.pickup_service_duration = 4;
  search_config.delivery_service_duration = 1;

  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config);

  ASSERT_EQ(solution.size(), 3);
  ASSERT_EQ(solution[0][0], ins.G.U[0]);
  ASSERT_EQ(solution[1][1], ins.G.U[1]);
  ASSERT_EQ(solution[2][1], ins.G.U[1]);
  ASSERT_EQ(stats.service_satisfied_agents, 2);
  ASSERT_EQ(stats.service_satisfied_deliveries, 1);
}

TEST(tapf_planner, service_mode_allows_sequential_use_of_one_physical_goal)
{
  const auto map_filename = "./tests/assets/lifelong-task-small.map";
  const auto starts = std::vector<int>{3, 9};
  const auto goals = std::vector<std::vector<int> >{{0}, {0}};
  const auto ins =
      TAPFInstance(map_filename, starts, goals, {}, 1, {}, {}, true);

  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.tasks.size(), starts.size());
  ASSERT_EQ(ins.tasks[0], ins.tasks[1]);

  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = starts.size();
  const auto solution = solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, false,
                                   false, search_config, &final_assignment);

  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_best_satisfied_agents, 2);
  ASSERT_EQ(final_assignment.size(), 2);
  ASSERT_NE(final_assignment[0], final_assignment[1]);
  ASSERT_EQ(ins.tasks[final_assignment[0]]->index, 0);
  ASSERT_EQ(ins.tasks[final_assignment[1]]->index, 0);
  // Both virtual task slots denote one physical vertex. The searched
  // continuation services them sequentially because only one agent may occupy
  // it per step; the returned execution prefix ends at the first service.
  ASSERT_EQ(
      std::count(solution.back().begin(), solution.back().end(), ins.G.U[0]),
      1);
}
