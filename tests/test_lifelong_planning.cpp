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

LifelongSolveResult solve_snapshot(
    const std::string& map_filename,
    std::vector<LifelongAgentState>& agents, std::vector<LifelongTask>& tasks,
    const LifelongPlanningSnapshot& snapshot)
{
  const auto ins =
      build_lifelong_tapf_instance(map_filename, agents, snapshot);
  EXPECT_TRUE(ins.is_valid());
  auto result = LifelongSolveResult();
  result.solution =
      solve_tapf(ins, 0, nullptr, nullptr, 0, nullptr, true, true,
                 TAPFSearchConfig(), &result.final_assignment);
  for (const auto target : result.final_assignment) {
    result.target_indexes.push_back(ins.tasks[target]->index);
  }
  result.applied = apply_lifelong_solution_assignment(
      agents, tasks, snapshot, ins, result.final_assignment);
  return result;
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
  const auto map_filename = std::string("./tests/assets/lifelong-task-small.map");
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
        ins.assignment_cost_offsets[0][target] == 0) {
      ++loaded_goals;
    }
    if (ins.allowed[1][target] &&
        ins.tasks[target]->index == graph.U[2]->index &&
        ins.assignment_cost_offsets[1][target] == 6) {
      ++unloaded_pickups;
    }
  }
  ASSERT_EQ(loaded_goals, 2);
  ASSERT_EQ(unloaded_pickups, 1);
}

TEST(lifelong_planning, planner_result_assigns_unloaded_task)
{
  const auto map_filename = std::string("./tests/assets/lifelong-task-small.map");
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
  const auto ins =
      build_lifelong_tapf_instance(map_filename, agents, snapshot);
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
  const auto result =
      solve_snapshot(map_filename, agents, tasks, snapshot);

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

TEST(lifelong_planning, shared_loaded_drop_uses_alternative_drop)
{
  const auto map_filename = std::string("./tests/assets/lifelong-task-small.map");
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
  const auto result =
      solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_FALSE(result.solution.empty());
  const auto service_count =
      (result.target_indexes[0] == graph.U[4]->index ? 1 : 0) +
      (result.target_indexes[1] == graph.U[4]->index ? 1 : 0);
  ASSERT_EQ(service_count, 1);
  ASSERT_EQ(result.target_indexes[0], graph.U[4]->index);
  ASSERT_EQ(result.target_indexes[1], graph.U[6]->index);
  ASSERT_TRUE(result.applied);
  ASSERT_EQ(agents[0].current_task_id, 20);
  ASSERT_EQ(agents[1].current_task_id, 21);
}

TEST(lifelong_planning, shared_singleton_drop_defers_one_loaded_agent)
{
  const auto map_filename = std::string("./tests/assets/lifelong-task-small.map");
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
  const auto result =
      solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_FALSE(result.solution.empty());
  const auto service_count =
      (result.target_indexes[0] == graph.U[4]->index ? 1 : 0) +
      (result.target_indexes[1] == graph.U[4]->index ? 1 : 0);
  ASSERT_EQ(service_count, 1);
  ASSERT_TRUE(result.applied);
  ASSERT_EQ(agents[0].current_task_id, 20);
  ASSERT_EQ(agents[1].current_task_id, 21);
}

TEST(lifelong_planning, loaded_drop_and_unloaded_pickup_compete_together)
{
  const auto map_filename = std::string("./tests/assets/lifelong-task-small.map");
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
  auto alternative = make_pending_task(
      22, LifelongTaskType::OUTBOUND, graph.U[3], Vertices{graph.U[7]});
  auto tasks = std::vector<LifelongTask>{picked, pending, alternative};

  const auto snapshot =
      prepare_lifelong_planning_snapshot(agents, tasks, distances);
  const auto result =
      solve_snapshot(map_filename, agents, tasks, snapshot);

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
  const auto map_filename = std::string("./tests/assets/lifelong-task-small.map");
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
  const auto result =
      solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_TRUE(result.applied);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::PENDING);
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_EQ(agents[0].current_task_id, 31);
}

TEST(lifelong_planning, equal_cost_replanning_keeps_unpicked_task)
{
  const auto map_filename = std::string("./tests/assets/lifelong-task-small.map");
  const auto graph = Graph(map_filename);
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents =
      std::vector<LifelongAgentState>{make_agent(0, graph.U[1])};
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
  const auto result =
      solve_snapshot(map_filename, agents, tasks, snapshot);

  ASSERT_TRUE(result.applied);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::PENDING);
  ASSERT_EQ(agents[0].current_task_id, 30);
}
