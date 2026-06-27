#include <lacam.hpp>

#include "gtest/gtest.h"

namespace
{
  LifelongTask make_pending_task(int id, LifelongTaskType type, Vertex* start,
                                 const Vertices& goals)
  {
    auto task = LifelongTask();
    task.task_id = id;
    task.task_type = type;
    task.start = start;
    task.goal_set = goals;
    task.status = LifelongTaskStatus::PENDING;
    return task;
  }

  LifelongAgentState make_agent(int id, Vertex* location)
  {
    auto agent = LifelongAgentState();
    agent.agent_id = id;
    agent.current_location = location;
    agent.current_target = location;
    return agent;
  }

  struct LifelongSolveResult {
    Solution solution;
    std::vector<int> final_assignment;
    std::vector<int> target_indexes;
    bool applied = false;
  };

  LifelongSolveResult solve_snapshot(const std::string& map_filename,
                                     std::vector<LifelongAgentState>& agents,
                                     std::vector<LifelongTask>& tasks,
                                     const LifelongPlanningSnapshot& snapshot,
                                     int service_commit_agents = 1,
                                     TAPFStats* stats = nullptr)
  {
    const auto ins =
        build_lifelong_tapf_instance(map_filename, agents, snapshot);
    EXPECT_TRUE(ins.is_valid());
    auto result = LifelongSolveResult();
    auto search_config = TAPFSearchConfig();
    search_config.service_goal_mode = true;
    // Private wait columns keep the rectangular assignment feasible but are not
    // service events. Production lifelong planning commits one real service
    // cohort at a time.
    search_config.service_commit_agents = service_commit_agents;
    result.solution = solve_tapf(ins, 0, nullptr, nullptr, 0, stats, true, true,
                                 search_config, &result.final_assignment);
    for (const auto target : result.final_assignment) {
      result.target_indexes.push_back(ins.tasks[target]->index);
    }
    result.applied = apply_lifelong_solution_assignment(
        agents, tasks, snapshot, ins, result.final_assignment);
    return result;
  }

  int count_physical_task_columns(const TAPFInstance& instance,
                                  int vertex_index)
  {
    return static_cast<int>(std::count_if(
        instance.tasks.begin(), instance.tasks.end(), [&](const auto* task) {
          return task != nullptr && task->index == vertex_index;
        }));
  }

  bool has_private_wait_option(const LifelongPlanningSnapshot& snapshot,
                               size_t agent)
  {
    return agent < snapshot.goal_keys_by_agent.size() &&
           std::any_of(snapshot.goal_keys_by_agent[agent].begin(),
                       snapshot.goal_keys_by_agent[agent].end(),
                       [](const auto key) { return key < 0; });
  }
}  // namespace

TEST(lifelong_planning, unloaded_cost_includes_pickup_and_delivery)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);
  const auto agent = make_agent(0, graph.U[3]);
  const auto task =
      make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                        Vertices{graph.U[4], graph.U[5], graph.U[6]});

  ASSERT_EQ(lifelong_unloaded_assignment_cost(agent, task, distances), 7);
}

TEST(lifelong_planning, snapshot_contains_loaded_and_unloaded_cost_rows)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[1]),
      make_agent(1, graph.U[3]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].current_task_id = 20;

  auto picked = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                  Vertices{graph.U[0], graph.U[9]});
  picked.status = LifelongTaskStatus::PICKED;
  picked.picked_agent_id = 0;
  auto tasks = std::vector<LifelongTask>{
      picked,
      make_pending_task(21, LifelongTaskType::OUTBOUND, graph.U[2],
                        Vertices{graph.U[4], graph.U[5], graph.U[6]}),
  };

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::PENDING);
  ASSERT_FALSE(tasks[1].assigned_agent_id.has_value());

  auto loaded_goals = 0;
  auto unloaded_pickups = 0;
  for (size_t target = 0; target < ins.tasks.size(); ++target) {
    if (ins.allowed[0][target] &&
        (ins.tasks[target]->index == graph.U[0]->index ||
         ins.tasks[target]->index == graph.U[9]->index) &&
        ins.assignment_cost_offsets[0][target] == 0 &&
        ins.assignment_service_durations[0][target] == 1) {
      ++loaded_goals;
    }
    if (ins.allowed[1][target] &&
        ins.tasks[target]->index == graph.U[2]->index &&
        ins.assignment_cost_offsets[1][target] == 2 &&
        ins.assignment_distance_scales[1][target] == 1 &&
        ins.assignment_service_durations[1][target] == 1) {
      ++unloaded_pickups;
    }
  }
  ASSERT_EQ(loaded_goals, 10);
  ASSERT_EQ(unloaded_pickups, 1);
}

TEST(lifelong_planning, service_durations_are_encoded_separately_from_offsets)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[1]),
      make_agent(1, graph.U[3]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].carried_task_ids.push_back(20);

  auto picked = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                  Vertices{graph.U[0]});
  picked.status = LifelongTaskStatus::PICKED;
  picked.picked_agent_id = 0;
  auto tasks = std::vector<LifelongTask>{
      picked,
      make_pending_task(21, LifelongTaskType::OUTBOUND, graph.U[2],
                        Vertices{graph.U[4]}),
  };

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances, 1, 5, 4, 3);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_TRUE(ins.is_valid());
  auto saw_delivery = false;
  auto saw_pickup = false;
  for (size_t target = 0; target < ins.tasks.size(); ++target) {
    if (ins.allowed[0][target] &&
        ins.tasks[target]->index == graph.U[0]->index) {
      EXPECT_EQ(ins.assignment_cost_offsets[0][target], 0);
      EXPECT_EQ(ins.assignment_service_durations[0][target], 3);
      saw_delivery = true;
    }
    if (ins.allowed[1][target] &&
        ins.tasks[target]->index == graph.U[2]->index) {
      EXPECT_EQ(ins.assignment_cost_offsets[1][target], 2);
      EXPECT_EQ(ins.assignment_service_durations[1][target], 4);
      saw_pickup = true;
    }
  }
  ASSERT_TRUE(saw_delivery);
  ASSERT_TRUE(saw_pickup);
}

TEST(lifelong_planning, zero_service_duration_does_not_add_cost_offsets)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[1]),
      make_agent(1, graph.U[3]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].carried_task_ids.push_back(20);

  auto picked = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                  Vertices{graph.U[0]});
  picked.status = LifelongTaskStatus::PICKED;
  picked.picked_agent_id = 0;
  auto tasks = std::vector<LifelongTask>{
      picked,
      make_pending_task(21, LifelongTaskType::OUTBOUND, graph.U[2],
                        Vertices{graph.U[4]}),
  };

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances, 1, 5, 0, 0);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_TRUE(ins.is_valid());
  auto saw_delivery = false;
  auto saw_pickup = false;
  for (size_t target = 0; target < ins.tasks.size(); ++target) {
    if (ins.allowed[0][target] &&
        ins.tasks[target]->index == graph.U[0]->index) {
      EXPECT_EQ(ins.assignment_cost_offsets[0][target], 0);
      EXPECT_EQ(ins.assignment_service_durations[0][target], 0);
      saw_delivery = true;
    }
    if (ins.allowed[1][target] &&
        ins.tasks[target]->index == graph.U[2]->index) {
      EXPECT_EQ(ins.assignment_cost_offsets[1][target], 2);
      EXPECT_EQ(ins.assignment_service_durations[1][target], 0);
      saw_pickup = true;
    }
  }
  ASSERT_TRUE(saw_delivery);
  ASSERT_TRUE(saw_pickup);
}

TEST(lifelong_planning, planner_result_assigns_unloaded_task)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[3]),
      make_agent(1, graph.U[6]),
  };
  auto tasks = std::vector<LifelongTask>{
      make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                        Vertices{graph.U[4], graph.U[5], graph.U[6]}),
  };

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);
  auto tapf_distances = TAPFDistTable(ins);
  const auto initial_assignment =
      assign_tapf_tasks(ins, tapf_distances, ins.starts);
  ASSERT_TRUE(initial_assignment.feasible);
  const auto initially_selected_service_count =
      (ins.tasks[initial_assignment.agent_to_task[0]]->index ==
               graph.U[0]->index
           ? 1
           : 0) +
      (ins.tasks[initial_assignment.agent_to_task[1]]->index ==
               graph.U[0]->index
           ? 1
           : 0);
  ASSERT_EQ(initially_selected_service_count, 1);
  const auto result = solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_FALSE(result.solution.empty());
  ASSERT_EQ(result.final_assignment.size(), agents.size());
  ASSERT_EQ(result.target_indexes.size(), agents.size());
  ASSERT_TRUE(result.applied);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_TRUE(tasks[0].assigned_agent_id.has_value());
  const auto assigned_agent = *tasks[0].assigned_agent_id;
  ASSERT_EQ(agents[assigned_agent].current_task_id, 10);
  ASSERT_EQ(agents[assigned_agent].current_target, graph.U[0]);
  ASSERT_FALSE(agents[assigned_agent == 0 ? 1 : 0].current_task_id.has_value());
}

TEST(lifelong_planning, shared_pickup_start_has_one_slot_per_planning_round)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);
  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[3]),
      make_agent(1, graph.U[7]),
  };
  auto tasks = std::vector<LifelongTask>{
      make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                        Vertices{graph.U[4]}),
      make_pending_task(11, LifelongTaskType::OUTBOUND, graph.U[0],
                        Vertices{graph.U[6]}),
  };

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto result = solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_FALSE(result.solution.empty());
  ASSERT_TRUE(result.applied);
  const auto assigned_count =
      std::count_if(tasks.begin(), tasks.end(), [](const auto& task) {
        return task.status == LifelongTaskStatus::ASSIGNED;
      });
  ASSERT_EQ(assigned_count, 1);
}

TEST(lifelong_planning,
     partial_pickup_preference_preserves_shared_start_task_identity)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);
  auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[0])};
  agents[0].assigned_task_id = 11;
  agents[0].current_task_id = 11;
  agents[0].current_target = graph.U[0];

  auto partial_task = make_pending_task(11, LifelongTaskType::OUTBOUND,
                                        graph.U[0], Vertices{graph.U[9]});
  partial_task.status = LifelongTaskStatus::ASSIGNED;
  partial_task.assigned_agent_id = 0;
  auto tasks = std::vector<LifelongTask>{
      make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                        Vertices{graph.U[4]}),
      partial_task,
  };
  auto preferred =
      std::vector<std::unordered_map<int, int> >(agents.size());
  preferred[0][graph.U[0]->index] = 11;

  const auto snapshot = prepare_lifelong_planning_snapshot(
      agents, tasks, distances, 1, 5, 1, 1, std::vector<float>(),
      preferred);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::PENDING);
  ASSERT_EQ(snapshot.pending_task_id_by_start_index_by_agent[0].at(
                graph.U[0]->index),
            11);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);
  ASSERT_TRUE(ins.is_valid());
}

TEST(lifelong_planning, unloaded_wait_is_only_added_when_pickups_are_scarce)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);
  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[3]),
      make_agent(1, graph.U[7]),
  };

  {
    auto tasks = std::vector<LifelongTask>{
        make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                          Vertices{graph.U[4]}),
        make_pending_task(11, LifelongTaskType::OUTBOUND, graph.U[2],
                          Vertices{graph.U[6]}),
    };
    const auto snapshot =
        prepare_lifelong_planning_snapshot(agents, tasks, distances);

    ASSERT_TRUE(snapshot.feasible);
    ASSERT_FALSE(has_private_wait_option(snapshot, 0));
    ASSERT_FALSE(has_private_wait_option(snapshot, 1));
  }

  {
    auto tasks = std::vector<LifelongTask>{
        make_pending_task(20, LifelongTaskType::OUTBOUND, graph.U[0],
                          Vertices{graph.U[4]}),
        make_pending_task(21, LifelongTaskType::OUTBOUND, graph.U[0],
                          Vertices{graph.U[6]}),
    };
    const auto snapshot =
        prepare_lifelong_planning_snapshot(agents, tasks, distances);

    ASSERT_TRUE(snapshot.feasible);
    ASSERT_TRUE(has_private_wait_option(snapshot, 0));
    ASSERT_TRUE(has_private_wait_option(snapshot, 1));
  }
}

TEST(lifelong_planning, multi_carry_candidate_set_depends_on_load_count)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto pending = make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                                   Vertices{graph.U[4]});
  auto carried0 = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                    Vertices{graph.U[5]});
  carried0.status = LifelongTaskStatus::PICKED;
  carried0.picked_agent_id = 0;
  auto carried1 = make_pending_task(21, LifelongTaskType::INBOUND, graph.U[2],
                                    Vertices{graph.U[6]});
  carried1.status = LifelongTaskStatus::PICKED;
  carried1.picked_agent_id = 0;

  {
    auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[3])};
    auto tasks = std::vector<LifelongTask>{pending};
    const auto snapshot =
        prepare_lifelong_planning_snapshot(agents, tasks, distances, 2);
    ASSERT_TRUE(snapshot.feasible);
    ASSERT_TRUE(snapshot.pending_task_id_by_start_index_by_agent[0].count(
        graph.U[0]->index));
  }

  {
    auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[3])};
    agents[0].load_state = AgentLoadState::LOADED;
    agents[0].carried_task_ids = {20};
    agents[0].current_task_id = 20;
    auto tasks = std::vector<LifelongTask>{pending, carried0};
    const auto snapshot =
        prepare_lifelong_planning_snapshot(agents, tasks, distances, 2);
    ASSERT_TRUE(snapshot.feasible);
    ASSERT_TRUE(snapshot.pending_task_id_by_start_index_by_agent[0].count(
        graph.U[0]->index));
    ASSERT_NE(
        std::find(snapshot.goal_indexes_by_agent[0].begin(),
                  snapshot.goal_indexes_by_agent[0].end(), graph.U[5]->index),
        snapshot.goal_indexes_by_agent[0].end());
  }

  {
    auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[3])};
    agents[0].load_state = AgentLoadState::LOADED;
    agents[0].carried_task_ids = {20, 21};
    agents[0].current_task_id = 20;
    auto tasks = std::vector<LifelongTask>{pending, carried0, carried1};
    const auto snapshot =
        prepare_lifelong_planning_snapshot(agents, tasks, distances, 2);
    ASSERT_TRUE(snapshot.feasible);
    ASSERT_FALSE(snapshot.pending_task_id_by_start_index_by_agent[0].count(
        graph.U[0]->index));
    ASSERT_NE(
        std::find(snapshot.goal_indexes_by_agent[0].begin(),
                  snapshot.goal_indexes_by_agent[0].end(), graph.U[5]->index),
        snapshot.goal_indexes_by_agent[0].end());
  }
}

TEST(lifelong_planning, multi_carry_uses_per_pair_fixed_point_scales)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[3])};
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].carried_task_ids = {20};
  agents[0].current_task_id = 20;
  auto carried = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                   Vertices{graph.U[5]});
  carried.status = LifelongTaskStatus::PICKED;
  carried.picked_agent_id = 0;
  auto pending = make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                                   Vertices{graph.U[4]});
  auto tasks = std::vector<LifelongTask>{carried, pending};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances, 2);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_EQ(snapshot.common_cost_scale, 2);
  auto pickup_scale = -1;
  auto delivery_scale = -1;
  for (size_t target = 0; target < ins.tasks.size(); ++target) {
    if (!ins.allowed[0][target]) continue;
    if (ins.tasks[target]->index == graph.U[0]->index) {
      pickup_scale = ins.assignment_distance_scales[0][target];
    }
    if (ins.tasks[target]->index == graph.U[5]->index) {
      delivery_scale = ins.assignment_distance_scales[0][target];
    }
  }
  ASSERT_EQ(pickup_scale, 1);
  ASSERT_EQ(delivery_scale, 2);
}

TEST(lifelong_planning, loaded_distance_sets_priority_offset)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[3]),
      make_agent(1, graph.U[2]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].carried_task_ids = {20};
  agents[0].current_task_id = 20;
  agents[0].loaded_distance_since_last_delivery = 10;

  auto carried = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                   Vertices{graph.U[5]});
  carried.status = LifelongTaskStatus::PICKED;
  carried.picked_agent_id = 0;
  auto pending = make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                                   Vertices{graph.U[4]});
  auto tasks = std::vector<LifelongTask>{carried, pending};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances, 2);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_EQ(ins.agent_priority_offsets.size(), agents.size());
  ASSERT_FLOAT_EQ(ins.agent_priority_offsets[0], 10.0f);
  ASSERT_FLOAT_EQ(ins.agent_priority_offsets[1], 0.0f);
}

TEST(lifelong_planning, provided_priority_offsets_add_bounded_aging_bias)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[3]),
      make_agent(1, graph.U[2]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].carried_task_ids = {20};
  agents[0].current_task_id = 20;
  agents[0].loaded_distance_since_last_delivery = 10;

  auto carried = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                   Vertices{graph.U[5]});
  carried.status = LifelongTaskStatus::PICKED;
  carried.picked_agent_id = 0;
  auto pending = make_pending_task(10, LifelongTaskType::OUTBOUND, graph.U[0],
                                   Vertices{graph.U[4]});
  auto tasks = std::vector<LifelongTask>{carried, pending};
  const auto inherited_priorities = std::vector<float>{4.0f, 7.0f};

  const auto snapshot = prepare_lifelong_planning_snapshot(
      agents, tasks, distances, 2, 5, 1, 1, inherited_priorities);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_EQ(ins.agent_priority_offsets.size(), agents.size());
  ASSERT_FLOAT_EQ(ins.agent_priority_offsets[0], 12.0f);
  ASSERT_FLOAT_EQ(ins.agent_priority_offsets[1], 2.0f);
}

TEST(lifelong_planning, shared_loaded_drop_uses_distinct_service_slots)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[1]),
      make_agent(1, graph.U[2]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].current_task_id = 20;
  agents[1].load_state = AgentLoadState::LOADED;
  agents[1].current_task_id = 21;

  auto task0 = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                 Vertices{graph.U[4]});
  task0.status = LifelongTaskStatus::PICKED;
  task0.picked_agent_id = 0;
  auto task1 = make_pending_task(21, LifelongTaskType::INBOUND, graph.U[2],
                                 Vertices{graph.U[4], graph.U[6]});
  task1.status = LifelongTaskStatus::PICKED;
  task1.picked_agent_id = 1;
  auto tasks = std::vector<LifelongTask>{task0, task1};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto result = solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_FALSE(result.solution.empty());
  const auto service_count =
      (result.target_indexes[0] == graph.U[4]->index ? 1 : 0) +
      (result.target_indexes[1] == graph.U[4]->index ? 1 : 0);
  ASSERT_EQ(service_count, 2);
  ASSERT_TRUE(result.applied);
  ASSERT_EQ(agents[0].current_task_id, 20);
  ASSERT_EQ(agents[1].current_task_id, 21);
}

TEST(lifelong_planning, shared_loaded_drop_defaults_to_five_service_slots)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);
  constexpr auto kAgents = 6;
  auto agents = std::vector<LifelongAgentState>();
  auto tasks = std::vector<LifelongTask>();
  agents.reserve(kAgents);
  tasks.reserve(kAgents);
  for (auto i = 0; i < kAgents; ++i) {
    agents.push_back(make_agent(i, graph.U[i]));
    agents.back().load_state = AgentLoadState::LOADED;
    agents.back().current_task_id = 100 + i;
    auto task = make_pending_task(100 + i, LifelongTaskType::INBOUND,
                                  graph.U[i], Vertices{graph.U[9]});
    task.status = LifelongTaskStatus::PICKED;
    task.picked_agent_id = i;
    tasks.push_back(task);
  }

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(count_physical_task_columns(ins, graph.U[9]->index), 5);
}

TEST(lifelong_planning, shared_loaded_drop_service_slots_are_configurable)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);
  constexpr auto kAgents = 4;
  auto agents = std::vector<LifelongAgentState>();
  auto tasks = std::vector<LifelongTask>();
  agents.reserve(kAgents);
  tasks.reserve(kAgents);
  for (auto i = 0; i < kAgents; ++i) {
    agents.push_back(make_agent(i, graph.U[i]));
    agents.back().load_state = AgentLoadState::LOADED;
    agents.back().current_task_id = 200 + i;
    auto task = make_pending_task(200 + i, LifelongTaskType::INBOUND,
                                  graph.U[i], Vertices{graph.U[9]});
    task.status = LifelongTaskStatus::PICKED;
    task.picked_agent_id = i;
    tasks.push_back(task);
  }

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances, 1, 2);
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(count_physical_task_columns(ins, graph.U[9]->index), 2);
}

TEST(lifelong_planning, shared_singleton_drop_is_serviced_sequentially)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[1]),
      make_agent(1, graph.U[2]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].current_task_id = 20;
  agents[1].load_state = AgentLoadState::LOADED;
  agents[1].current_task_id = 21;

  auto task0 = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[1],
                                 Vertices{graph.U[4]});
  task0.status = LifelongTaskStatus::PICKED;
  task0.picked_agent_id = 0;
  auto task1 = make_pending_task(21, LifelongTaskType::INBOUND, graph.U[2],
                                 Vertices{graph.U[4]});
  task1.status = LifelongTaskStatus::PICKED;
  task1.picked_agent_id = 1;
  auto tasks = std::vector<LifelongTask>{task0, task1};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto result = solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_FALSE(result.solution.empty());
  const auto service_count =
      (result.target_indexes[0] == graph.U[4]->index ? 1 : 0) +
      (result.target_indexes[1] == graph.U[4]->index ? 1 : 0);
  ASSERT_EQ(service_count, 2);
  ASSERT_TRUE(result.applied);
  ASSERT_EQ(agents[0].current_task_id, 20);
  ASSERT_EQ(agents[1].current_task_id, 21);
}

TEST(lifelong_planning,
     full_service_proof_can_assign_two_agents_to_one_drop_location)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[3]),
      make_agent(1, graph.U[9]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].current_task_id = 20;
  agents[1].load_state = AgentLoadState::LOADED;
  agents[1].current_task_id = 21;

  auto task0 = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[3],
                                 Vertices{graph.U[0]});
  task0.status = LifelongTaskStatus::PICKED;
  task0.picked_agent_id = 0;
  auto task1 = make_pending_task(21, LifelongTaskType::INBOUND, graph.U[9],
                                 Vertices{graph.U[0]});
  task1.status = LifelongTaskStatus::PICKED;
  task1.picked_agent_id = 1;
  auto tasks = std::vector<LifelongTask>{task0, task1};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  auto stats = TAPFStats();
  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);
  auto final_assignment = std::vector<int>();
  auto assignment_schedule = std::vector<std::vector<int> >();
  auto search_config = TAPFSearchConfig();
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = agents.size();
  const auto solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, &stats, true, true, search_config,
                 &final_assignment, &assignment_schedule);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_TRUE(ins.is_valid());
  ASSERT_FALSE(solution.empty());
  ASSERT_EQ(stats.service_best_satisfied_agents, 2);
  ASSERT_EQ(final_assignment.size(), agents.size());
  ASSERT_EQ(assignment_schedule.size(), solution.size());
  ASSERT_EQ(std::count_if(solution.back().begin(), solution.back().end(),
                          [&](const auto* vertex) {
                            return vertex != nullptr &&
                                   vertex->index == graph.U[0]->index;
                          }),
            1)
      << "a multi-service commit should keep the prefix through all committed "
         "services without stacking agents during an active service";
}

TEST(lifelong_planning, loaded_drop_and_unloaded_pickup_compete_together)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{
      make_agent(0, graph.U[0]),
      make_agent(1, graph.U[2]),
  };
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].current_task_id = 20;

  auto picked = make_pending_task(20, LifelongTaskType::INBOUND, graph.U[0],
                                  Vertices{graph.U[1]});
  picked.status = LifelongTaskStatus::PICKED;
  picked.picked_agent_id = 0;
  auto pending = make_pending_task(21, LifelongTaskType::OUTBOUND, graph.U[1],
                                   Vertices{graph.U[6]});
  auto alternative = make_pending_task(22, LifelongTaskType::OUTBOUND,
                                       graph.U[3], Vertices{graph.U[7]});
  auto tasks = std::vector<LifelongTask>{picked, pending, alternative};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto result = solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_FALSE(result.solution.empty());
  ASSERT_EQ(result.target_indexes[0], graph.U[1]->index);
  ASSERT_EQ(result.target_indexes[1], graph.U[3]->index);
  ASSERT_TRUE(result.applied);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::PICKED);
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::PENDING);
  ASSERT_EQ(tasks[2].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_EQ(agents[1].current_task_id, 22);
}

TEST(lifelong_planning, replanning_can_switch_unpicked_task)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[2])};
  agents[0].current_task_id = 30;
  agents[0].current_target = graph.U[0];

  auto old_task = make_pending_task(30, LifelongTaskType::OUTBOUND, graph.U[0],
                                    Vertices{graph.U[6]});
  old_task.status = LifelongTaskStatus::ASSIGNED;
  old_task.assigned_agent_id = 0;
  auto new_task = make_pending_task(31, LifelongTaskType::OUTBOUND, graph.U[2],
                                    Vertices{graph.U[4]});
  auto tasks = std::vector<LifelongTask>{old_task, new_task};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto result = solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_TRUE(result.applied);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::PENDING);
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_EQ(agents[0].current_task_id, 31);
}

TEST(lifelong_planning, equal_cost_replanning_keeps_unpicked_task)
{
  const auto map_filename =
      std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[1])};
  agents[0].current_task_id = 30;
  agents[0].current_target = graph.U[0];

  auto old_task = make_pending_task(30, LifelongTaskType::OUTBOUND, graph.U[0],
                                    Vertices{graph.U[4]});
  old_task.status = LifelongTaskStatus::ASSIGNED;
  old_task.assigned_agent_id = 0;
  auto new_task = make_pending_task(31, LifelongTaskType::OUTBOUND, graph.U[2],
                                    Vertices{graph.U[6]});
  auto tasks = std::vector<LifelongTask>{old_task, new_task};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto result = solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_TRUE(result.applied);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::PENDING);
  ASSERT_EQ(agents[0].current_task_id, 30);
}
