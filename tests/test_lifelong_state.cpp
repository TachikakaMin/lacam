#include <lacam.hpp>

#include "gtest/gtest.h"

namespace
{
LifelongTask make_assigned_task(int id, Vertex* start, const Vertices& goals,
                                int agent_id,
                                LifelongTaskType type =
                                    LifelongTaskType::OUTBOUND)
{
  auto task = LifelongTask();
  task.task_id = id;
  task.task_type = type;
  task.start = start;
  task.goal_set = goals;
  task.status = LifelongTaskStatus::ASSIGNED;
  task.assigned_agent_id = agent_id;
  return task;
}
}  // namespace

TEST(lifelong_state, pickup_and_completion_transitions)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto tasks = std::vector<LifelongTask>{
      make_assigned_task(3, graph.U[0], Vertices{graph.U[4], graph.U[5]}, 7),
  };
  auto agent = LifelongAgentState();
  agent.agent_id = 7;
  agent.current_location = graph.U[0];
  agent.current_task_id = 3;
  agent.current_target = graph.U[0];

  const auto pickup = try_pickup(agent, tasks, 4);
  ASSERT_TRUE(pickup.changed);
  ASSERT_EQ(agent.load_state, AgentLoadState::LOADED);
  ASSERT_EQ(agent.current_task_id, 3);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::PICKED);
  ASSERT_EQ(tasks[0].picked_agent_id, 7);
  ASSERT_EQ(tasks[0].pickup_timestep, 4);

  agent.current_location = graph.U[4];
  const auto completion = try_complete(agent, tasks, 9);
  ASSERT_TRUE(completion.changed);
  ASSERT_EQ(agent.load_state, AgentLoadState::UNLOADED);
  ASSERT_FALSE(agent.current_task_id.has_value());
  ASSERT_EQ(agent.current_target, agent.current_location);
  ASSERT_EQ(agent.completed_task_count, 1);
  ASSERT_EQ(agent.last_completed_task_type, LifelongTaskType::OUTBOUND);
  ASSERT_EQ(agent.alternating_completed_task_count, 0);
  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::COMPLETED);
  ASSERT_FALSE(tasks[0].assigned_agent_id.has_value());
  ASSERT_FALSE(tasks[0].picked_agent_id.has_value());
  ASSERT_EQ(tasks[0].completion_timestep, 9);

  auto agents = std::vector<LifelongAgentState>{agent};
  ASSERT_TRUE(check_lifelong_state_invariants(agents, tasks));
}

TEST(lifelong_state, counts_only_alternating_task_completions)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto agent = LifelongAgentState();
  agent.agent_id = 0;
  agent.current_location = graph.U[0];

  auto tasks = std::vector<LifelongTask>();
  const auto complete = [&](int id, LifelongTaskType type) {
    tasks.push_back(
        make_assigned_task(id, graph.U[0], Vertices{graph.U[4]}, 0, type));
    agent.current_task_id = id;
    ASSERT_TRUE(try_pickup(agent, tasks, id * 2).changed);
    agent.current_location = graph.U[4];
    ASSERT_TRUE(try_complete(agent, tasks, id * 2 + 1).changed);
    agent.current_location = graph.U[0];
  };

  complete(1, LifelongTaskType::OUTBOUND);
  ASSERT_EQ(agent.alternating_completed_task_count, 0);
  complete(2, LifelongTaskType::INBOUND);
  ASSERT_EQ(agent.alternating_completed_task_count, 1);
  complete(3, LifelongTaskType::INBOUND);
  ASSERT_EQ(agent.alternating_completed_task_count, 1);
  complete(4, LifelongTaskType::OUTBOUND);

  ASSERT_EQ(agent.completed_task_count, 4);
  ASSERT_EQ(agent.alternating_completed_task_count, 2);
  ASSERT_EQ(agent.last_completed_task_type, LifelongTaskType::OUTBOUND);
}

TEST(lifelong_state, release_unpicked_assignments_keeps_picked_tasks)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto tasks = std::vector<LifelongTask>{
      make_assigned_task(1, graph.U[0], Vertices{graph.U[4]}, 0),
      make_assigned_task(2, graph.U[2], Vertices{graph.U[5]}, 1),
  };
  tasks[1].status = LifelongTaskStatus::PICKED;
  tasks[1].picked_agent_id = 1;

  auto agents = std::vector<LifelongAgentState>(2);
  agents[0].agent_id = 0;
  agents[0].current_location = graph.U[1];
  agents[0].current_task_id = 1;
  agents[0].current_target = graph.U[0];
  agents[1].agent_id = 1;
  agents[1].current_location = graph.U[2];
  agents[1].load_state = AgentLoadState::LOADED;
  agents[1].current_task_id = 2;

  release_unpicked_assignments(agents, tasks);

  ASSERT_EQ(tasks[0].status, LifelongTaskStatus::PENDING);
  ASSERT_FALSE(tasks[0].assigned_agent_id.has_value());
  ASSERT_FALSE(agents[0].current_task_id.has_value());
  ASSERT_EQ(agents[0].current_target, agents[0].current_location);

  ASSERT_EQ(tasks[1].status, LifelongTaskStatus::PICKED);
  ASSERT_EQ(tasks[1].picked_agent_id, 1);
  ASSERT_EQ(agents[1].current_task_id, 2);
}

TEST(lifelong_state, invariants_reject_invalid_bindings)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto tasks = std::vector<LifelongTask>{
      make_assigned_task(1, graph.U[0], Vertices{graph.U[4]}, 0),
  };
  tasks[0].status = LifelongTaskStatus::PICKED;
  tasks[0].picked_agent_id = 0;

  auto agents = std::vector<LifelongAgentState>(1);
  agents[0].agent_id = 0;
  agents[0].current_location = graph.U[0];
  agents[0].load_state = AgentLoadState::UNLOADED;

  auto error = std::string();
  ASSERT_FALSE(check_lifelong_state_invariants(agents, tasks, &error));
  ASSERT_FALSE(error.empty());
}

TEST(lifelong_state, picked_task_start_can_be_reused)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto tasks = std::vector<LifelongTask>{
      make_assigned_task(1, graph.U[0], Vertices{graph.U[4]}, 0),
      make_assigned_task(2, graph.U[0], Vertices{graph.U[5]}, 1),
  };
  tasks[0].status = LifelongTaskStatus::PICKED;
  tasks[0].picked_agent_id = 0;

  auto agents = std::vector<LifelongAgentState>(2);
  agents[0].agent_id = 0;
  agents[0].current_location = graph.U[1];
  agents[0].load_state = AgentLoadState::LOADED;
  agents[0].current_task_id = 1;
  agents[1].agent_id = 1;
  agents[1].current_location = graph.U[2];
  agents[1].current_task_id = 2;

  ASSERT_TRUE(check_lifelong_state_invariants(agents, tasks));
}

TEST(lifelong_state, allows_two_unpicked_tasks_per_start)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto tasks = std::vector<LifelongTask>{
      make_assigned_task(1, graph.U[0], Vertices{graph.U[4]}, 0),
      make_assigned_task(2, graph.U[0], Vertices{graph.U[5]}, 1),
  };
  auto agents = std::vector<LifelongAgentState>{
      LifelongAgentState(), LifelongAgentState(), LifelongAgentState()};
  for (size_t i = 0; i < agents.size(); ++i) {
    agents[i].agent_id = static_cast<int>(i);
    agents[i].current_location = graph.U[i + 1];
  }
  agents[0].current_task_id = 1;
  agents[1].current_task_id = 2;

  ASSERT_TRUE(check_lifelong_state_invariants(agents, tasks));

  tasks.push_back(
      make_assigned_task(3, graph.U[0], Vertices{graph.U[6]}, 2));
  agents[2].current_task_id = 3;
  ASSERT_FALSE(check_lifelong_state_invariants(agents, tasks));
}
