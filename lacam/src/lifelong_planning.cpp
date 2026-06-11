#include "../include/lifelong_planning.hpp"

#include <algorithm>
#include <unordered_map>

namespace
{
constexpr int kIdleAssignmentCost = kTapfAssignmentInfCost / 4;

std::vector<size_t> collect_unloaded_agents(
    const std::vector<LifelongAgentState>& agents)
{
  auto ids = std::vector<size_t>();
  for (size_t i = 0; i < agents.size(); ++i) {
    if (agents[i].load_state == AgentLoadState::UNLOADED) ids.push_back(i);
  }
  return ids;
}

std::vector<size_t> collect_pending_tasks(const std::vector<LifelongTask>& tasks)
{
  auto ids = std::vector<size_t>();
  for (size_t i = 0; i < tasks.size(); ++i) {
    if (tasks[i].status == LifelongTaskStatus::PENDING) ids.push_back(i);
  }
  return ids;
}

std::vector<int> goal_indexes(const Vertices& vertices)
{
  auto indexes = std::vector<int>();
  indexes.reserve(vertices.size());
  for (auto v : vertices) indexes.push_back(v->index);
  return indexes;
}
}  // namespace

int lifelong_unloaded_assignment_cost(const LifelongAgentState& agent,
                                      const LifelongTask& task,
                                      const MapDistanceCache& distances)
{
  if (agent.current_location == nullptr || task.start == nullptr) {
    return kTapfAssignmentInfCost;
  }
  const auto pickup_cost = distances.get(agent.current_location, task.start);
  if (pickup_cost >= kMapDistanceInf) return kTapfAssignmentInfCost;

  auto delivery_cost = kMapDistanceInf;
  for (auto goal : task.goal_set) {
    delivery_cost = std::min(delivery_cost, distances.get(task.start, goal));
  }
  if (delivery_cost >= kMapDistanceInf) return kTapfAssignmentInfCost;
  return pickup_cost + delivery_cost;
}

int lifelong_loaded_cost(const LifelongAgentState& agent,
                         const LifelongTask& task,
                         const MapDistanceCache& distances)
{
  if (agent.current_location == nullptr) return kTapfAssignmentInfCost;
  auto cost = kMapDistanceInf;
  for (auto goal : task.goal_set) {
    cost = std::min(cost, distances.get(agent.current_location, goal));
  }
  return cost >= kMapDistanceInf ? kTapfAssignmentInfCost : cost;
}

LifelongPlanningSnapshot assign_lifelong_tasks_for_replanning(
    std::vector<LifelongAgentState>& agents, std::vector<LifelongTask>& tasks,
    const MapDistanceCache& distances)
{
  release_unpicked_assignments(agents, tasks);

  auto snapshot = LifelongPlanningSnapshot();
  snapshot.goal_indexes_by_agent.resize(agents.size());
  snapshot.assigned_task_ids_by_agent.resize(agents.size());

  for (size_t i = 0; i < agents.size(); ++i) {
    auto& agent = agents[i];
    if (agent.load_state == AgentLoadState::UNLOADED) {
      snapshot.goal_indexes_by_agent[i] = {agent.current_location->index};
      agent.current_target = agent.current_location;
      continue;
    }

    if (!agent.current_task_id.has_value()) {
      snapshot.feasible = false;
      continue;
    }
    auto* task = find_task_by_id(tasks, *agent.current_task_id);
    if (task == nullptr || task->status != LifelongTaskStatus::PICKED) {
      snapshot.feasible = false;
      continue;
    }
    snapshot.goal_indexes_by_agent[i] = goal_indexes(task->goal_set);
    snapshot.assigned_task_ids_by_agent[i] = task->task_id;
    snapshot.assignment_cost += lifelong_loaded_cost(agent, *task, distances);
  }

  const auto unloaded_agents = collect_unloaded_agents(agents);
  const auto pending_tasks = collect_pending_tasks(tasks);
  if (unloaded_agents.empty()) return snapshot;

  const auto dummy_cols = unloaded_agents.size();
  const auto total_cols = pending_tasks.size() + dummy_cols;
  auto cost = std::vector<std::vector<int> >(
      unloaded_agents.size(), std::vector<int>(total_cols, kIdleAssignmentCost));

  for (size_t row = 0; row < unloaded_agents.size(); ++row) {
    const auto agent_idx = unloaded_agents[row];
    for (size_t col = 0; col < pending_tasks.size(); ++col) {
      cost[row][col] = lifelong_unloaded_assignment_cost(
          agents[agent_idx], tasks[pending_tasks[col]], distances);
    }
  }

  const auto assignment = assign_hungarian_cost_matrix(cost);
  if (!assignment.feasible) {
    snapshot.feasible = false;
    return snapshot;
  }

  for (size_t row = 0; row < unloaded_agents.size(); ++row) {
    const auto agent_idx = unloaded_agents[row];
    const auto col = assignment.agent_to_task[row];
    if (col < 0 || col >= static_cast<int>(pending_tasks.size())) {
      continue;
    }
    const auto task_idx = pending_tasks[col];
    auto& task = tasks[task_idx];
    task.status = LifelongTaskStatus::ASSIGNED;
    task.assigned_agent_id = agents[agent_idx].agent_id;
    agents[agent_idx].current_task_id = task.task_id;
    agents[agent_idx].current_target = task.start;
    snapshot.goal_indexes_by_agent[agent_idx] = {task.start->index};
    snapshot.assigned_task_ids_by_agent[agent_idx] = task.task_id;
    snapshot.assignment_cost += cost[row][col];
  }

  return snapshot;
}

TAPFInstance build_lifelong_tapf_instance(
    const std::string& map_filename,
    const std::vector<LifelongAgentState>& agents,
    const LifelongPlanningSnapshot& snapshot)
{
  auto start_indexes = std::vector<int>();
  start_indexes.reserve(agents.size());
  for (const auto& agent : agents) start_indexes.push_back(agent.current_location->index);
  return TAPFInstance(map_filename, start_indexes, snapshot.goal_indexes_by_agent);
}
