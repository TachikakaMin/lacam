#include "../include/lifelong_planning.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

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

Vertices service_targets(const LifelongAgentState& agent,
                         const std::vector<LifelongTask>& tasks)
{
  if (!agent.current_task_id.has_value()) return Vertices();
  const auto* task = find_task_by_id(tasks, *agent.current_task_id);
  if (task == nullptr) return Vertices();
  if (agent.load_state == AgentLoadState::LOADED &&
      task->status == LifelongTaskStatus::PICKED) {
    return task->goal_set;
  }
  if (agent.load_state == AgentLoadState::UNLOADED &&
      task->status == LifelongTaskStatus::ASSIGNED) {
    return Vertices{task->start};
  }
  return Vertices();
}

int yield_assignment_penalty(size_t num_agents,
                             const MapDistanceCache& distances)
{
  const auto max_distance =
      static_cast<long long>(std::max(1, distances.metadata.traversable_count));
  const auto desired =
      (static_cast<long long>(num_agents) + 1) * (max_distance + 1);
  return static_cast<int>(
      std::min<long long>(desired, kTapfAssignmentInfCost / 2));
}

bool coordinate_physical_targets(
    std::vector<LifelongAgentState>& agents,
    const std::vector<LifelongTask>& tasks,
    const MapDistanceCache& distances, LifelongPlanningSnapshot& snapshot)
{
  if (agents.empty()) return true;

  auto service_by_agent = std::vector<Vertices>(agents.size());
  auto targets = Vertices();
  auto target_by_index = std::unordered_map<int, size_t>();
  auto parking_indexes = std::unordered_set<int>();

  auto add_target = [&](Vertex* target) {
    if (target == nullptr) return;
    if (target_by_index.find(target->index) != target_by_index.end()) return;
    target_by_index[target->index] = targets.size();
    targets.push_back(target);
  };

  for (size_t i = 0; i < agents.size(); ++i) {
    service_by_agent[i] = service_targets(agents[i], tasks);
    if (agents[i].current_task_id.has_value() && service_by_agent[i].empty()) {
      return false;
    }
    for (auto target : service_by_agent[i]) add_target(target);
  }
  for (const auto& agent : agents) {
    add_target(agent.current_location);
    parking_indexes.insert(agent.current_location->index);
  }

  auto cost = std::vector<std::vector<int> >(
      agents.size(),
      std::vector<int>(targets.size(), kTapfAssignmentInfCost));
  const auto yield_penalty =
      yield_assignment_penalty(agents.size(), distances);

  for (size_t i = 0; i < agents.size(); ++i) {
    for (auto target : service_by_agent[i]) {
      const auto col = target_by_index.at(target->index);
      const auto distance = distances.get(agents[i].current_location, target);
      if (distance < kMapDistanceInf) cost[i][col] = distance;
    }
    for (size_t col = 0; col < targets.size(); ++col) {
      if (parking_indexes.find(targets[col]->index) == parking_indexes.end()) {
        continue;
      }
      const auto distance =
          distances.get(agents[i].current_location, targets[col]);
      if (distance >= kMapDistanceInf) continue;
      cost[i][col] =
          std::min(cost[i][col], yield_penalty + distance);
    }
  }

  const auto assignment = assign_hungarian_cost_matrix(cost);
  if (!assignment.feasible) return false;

  for (size_t i = 0; i < agents.size(); ++i) {
    const auto col = assignment.agent_to_task[i];
    if (col < 0 || col >= static_cast<int>(targets.size())) return false;
    auto* target = targets[col];
    snapshot.goal_indexes_by_agent[i] = {target->index};
    agents[i].current_target = target;
  }
  return true;
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
  if (unloaded_agents.empty()) {
    snapshot.feasible =
        coordinate_physical_targets(agents, tasks, distances, snapshot);
    return snapshot;
  }

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

  snapshot.feasible =
      coordinate_physical_targets(agents, tasks, distances, snapshot);
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
