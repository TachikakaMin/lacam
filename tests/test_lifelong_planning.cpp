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

TEST(lifelong_planning, assigns_unloaded_agents_to_task_starts_with_hungarian)
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
      assign_lifelong_tasks_for_replanning(agents, tasks, distances);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_TRUE(tasks[0].assigned_agent_id.has_value());
  const auto assigned_agent = *tasks[0].assigned_agent_id;
  ASSERT_EQ(snapshot.goal_indexes_by_agent[assigned_agent].size(), 1);
  ASSERT_EQ(snapshot.goal_indexes_by_agent[assigned_agent][0], graph.U[0]->index);
  ASSERT_EQ(agents[assigned_agent].current_target, graph.U[0]);

  const auto idle_agent = assigned_agent == 0 ? 1 : 0;
  ASSERT_FALSE(agents[idle_agent].current_task_id.has_value());
  ASSERT_EQ(snapshot.goal_indexes_by_agent[idle_agent][0],
            agents[idle_agent].current_location->index);

  const auto ins = build_lifelong_tapf_instance(map_filename, agents, snapshot);
  ASSERT_EQ(ins.N, 2);
  ASSERT_EQ(ins.starts[0]->index, agents[0].current_location->index);
}

TEST(lifelong_planning, loaded_agents_keep_picked_task_goal_set)
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
      assign_lifelong_tasks_for_replanning(agents, tasks, distances);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::PICKED);
  ASSERT_EQ(agents[0].current_task_id, 20);
  ASSERT_EQ(snapshot.assigned_task_ids_by_agent[0], 20);
  ASSERT_EQ(snapshot.goal_indexes_by_agent[0].size(), 2);
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_EQ(tasks[1].assigned_agent_id, 1);
  ASSERT_EQ(snapshot.goal_indexes_by_agent[1][0], graph.U[2]->index);
}

TEST(lifelong_planning, replanning_releases_and_switches_unpicked_assignment)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  const auto distances =
      build_map_distance_cache(graph, "lifelong-task-small.map", 1);

  auto agents = std::vector<LifelongAgentState>{make_agent(0, graph.U[2])};
  agents[0].current_task_id = 30;
  agents[0].current_target = graph.U[0];

  auto old_task = make_pending_task(30, LifelongTaskType::OUTBOUND, graph.U[0],
                                    Vertices{graph.U[4], graph.U[5], graph.U[6]});
  old_task.status = LifelongTaskStatus::ASSIGNED;
  old_task.assigned_agent_id = 0;
  auto new_task = make_pending_task(31, LifelongTaskType::OUTBOUND, graph.U[2],
                                    Vertices{graph.U[4], graph.U[5], graph.U[6]});
  auto tasks = std::vector<LifelongTask>{old_task, new_task};

  const auto snapshot =
      assign_lifelong_tasks_for_replanning(agents, tasks, distances);

  ASSERT_TRUE(snapshot.feasible);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::PENDING);
  ASSERT_FALSE(tasks[0].assigned_agent_id.has_value());
  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::ASSIGNED);
  ASSERT_EQ(agents[0].current_task_id, 31);
  ASSERT_EQ(snapshot.goal_indexes_by_agent[0][0], graph.U[2]->index);
}
