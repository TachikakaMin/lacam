#include "../include/lifelong_planning.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "../include/tapf_assignment.hpp"

namespace
{
std::vector<size_t> collect_pending_tasks(const std::vector<LifelongTask>& tasks)
{
  auto ids = std::vector<size_t>();
  for (size_t i = 0; i < tasks.size(); ++i) {
    if (tasks[i].status == LifelongTaskStatus::PENDING) ids.push_back(i);
  }
  return ids;
}

int task_delivery_cost(const LifelongTask& task,
                       const MapDistanceCache& distances)
{
  if (task.start == nullptr) return kTapfAssignmentInfCost;
  auto cost = kMapDistanceInf;
  for (auto goal : task.goal_set) {
    cost = std::min(cost, distances.get(task.start, goal));
  }
  return cost >= kMapDistanceInf ? kTapfAssignmentInfCost : cost;
}

int scaled_assignment_offset(int primary_cost, int cost_scale, int tie_cost = 0)
{
  const auto encoded = static_cast<long long>(primary_cost) * cost_scale +
                       tie_cost;
  return encoded >= kTapfAssignmentInfCost / 2
             ? kTapfAssignmentInfCost
             : static_cast<int>(encoded);
}

int deferred_assignment_offset(size_t num_agents,
                               const MapDistanceCache& distances,
                               int cost_scale)
{
  const auto max_distance =
      static_cast<long long>(
          std::max(1, distances.metadata.traversable_count - 1));
  const auto max_service_cost = 2 * max_distance;
  const auto desired =
      (static_cast<long long>(num_agents) + 1) *
      (max_service_cost + 1) * static_cast<long long>(cost_scale);
  const auto max_offset =
      static_cast<long long>(kTapfAssignmentInfCost / 2 - 1) -
      max_distance * cost_scale - 1;
  return static_cast<int>(std::max(0LL, std::min(desired, max_offset)));
}

void add_goal_option(LifelongPlanningSnapshot& snapshot, size_t agent,
                     Vertex* target, int cost_offset)
{
  if (target == nullptr || cost_offset >= kTapfAssignmentInfCost) return;
  auto& indexes = snapshot.goal_indexes_by_agent[agent];
  auto& offsets = snapshot.goal_cost_offsets_by_agent[agent];
  const auto iter = std::find(indexes.begin(), indexes.end(), target->index);
  if (iter == indexes.end()) {
    indexes.push_back(target->index);
    offsets.push_back(cost_offset);
  } else {
    const auto option = std::distance(indexes.begin(), iter);
    offsets[option] = std::min(offsets[option], cost_offset);
  }
  snapshot.target_by_index[target->index] = target;
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
  const auto delivery_cost = task_delivery_cost(task, distances);
  if (delivery_cost >= kTapfAssignmentInfCost) {
    return kTapfAssignmentInfCost;
  }
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

LifelongPlanningSnapshot prepare_lifelong_planning_snapshot(
    std::vector<LifelongAgentState>& agents, std::vector<LifelongTask>& tasks,
    const MapDistanceCache& distances)
{
  auto previous_task_ids =
      std::vector<std::optional<int> >(agents.size(), std::nullopt);
  for (size_t i = 0; i < agents.size(); ++i) {
    if (agents[i].load_state == AgentLoadState::UNLOADED) {
      previous_task_ids[i] = agents[i].current_task_id;
    }
  }

  release_unpicked_assignments(agents, tasks);

  auto snapshot = LifelongPlanningSnapshot();
  snapshot.goal_indexes_by_agent.resize(agents.size());
  snapshot.goal_cost_offsets_by_agent.resize(agents.size());
  const auto pending_tasks = collect_pending_tasks(tasks);
  const auto unloaded_count =
      std::count_if(agents.begin(), agents.end(), [](const auto& agent) {
        return agent.load_state == AgentLoadState::UNLOADED;
      });
  const auto needs_idle_targets = pending_tasks.size() < unloaded_count;
  const auto cost_scale = static_cast<int>(agents.size()) + 1;
  const auto deferred_offset =
      deferred_assignment_offset(agents.size(), distances, cost_scale);

  for (const auto task_idx : pending_tasks) {
    const auto& task = tasks[task_idx];
    if (task.start == nullptr || task.goal_set.empty()) {
      snapshot.feasible = false;
      continue;
    }
    const auto inserted = snapshot.pending_task_id_by_start_index.emplace(
        task.start->index, task.task_id);
    if (!inserted.second && inserted.first->second != task.task_id) {
      snapshot.feasible = false;
    }
  }

  for (size_t i = 0; i < agents.size(); ++i) {
    auto& agent = agents[i];
    if (agent.current_location == nullptr) {
      snapshot.feasible = false;
      continue;
    }

    if (agent.load_state == AgentLoadState::LOADED) {
      if (!agent.current_task_id.has_value()) {
        snapshot.feasible = false;
        continue;
      }
      const auto* task = find_task_by_id(tasks, *agent.current_task_id);
      if (task == nullptr || task->status != LifelongTaskStatus::PICKED ||
          task->picked_agent_id != agent.agent_id ||
          task->goal_set.empty()) {
        snapshot.feasible = false;
        continue;
      }
      for (auto goal : task->goal_set) add_goal_option(snapshot, i, goal, 0);
      // A private high-cost target lets overloaded drop regions be serviced
      // over multiple replans while all agents remain in one Hungarian solve.
      add_goal_option(snapshot, i, agent.current_location,
                      deferred_offset);
    } else {
      for (const auto task_idx : pending_tasks) {
        const auto& task = tasks[task_idx];
        const auto delivery_cost = task_delivery_cost(task, distances);
        if (delivery_cost >= kTapfAssignmentInfCost) continue;
        const auto switched = previous_task_ids[i].has_value() &&
                              *previous_task_ids[i] != task.task_id;
        add_goal_option(snapshot, i, task.start,
                        scaled_assignment_offset(
                            delivery_cost, cost_scale, switched ? 1 : 0));
      }
      if (needs_idle_targets) {
        add_goal_option(snapshot, i, agent.current_location,
                        deferred_offset +
                            (previous_task_ids[i].has_value() ? 1 : 0));
      }
    }
    if (snapshot.goal_indexes_by_agent[i].empty()) snapshot.feasible = false;
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
  for (const auto& agent : agents) {
    start_indexes.push_back(agent.current_location->index);
  }
  return TAPFInstance(map_filename, start_indexes,
                      snapshot.goal_indexes_by_agent,
                      snapshot.goal_cost_offsets_by_agent,
                      static_cast<int>(agents.size()) + 1);
}

bool apply_lifelong_solution_assignment(
    std::vector<LifelongAgentState>& agents, std::vector<LifelongTask>& tasks,
    const LifelongPlanningSnapshot& snapshot, const TAPFInstance& instance,
    const std::vector<int>& final_assignment)
{
  if (final_assignment.size() != agents.size()) return false;

  auto selected_task_ids =
      std::vector<std::optional<int> >(agents.size(), std::nullopt);
  auto used_task_ids = std::unordered_set<int>();
  for (size_t i = 0; i < agents.size(); ++i) {
    const auto target = final_assignment[i];
    if (target < 0 || target >= static_cast<int>(instance.tasks.size()) ||
        !instance.allowed[i][target]) {
      return false;
    }
    const auto target_index = instance.tasks[target]->index;
    const auto allowed = std::find(snapshot.goal_indexes_by_agent[i].begin(),
                                   snapshot.goal_indexes_by_agent[i].end(),
                                   target_index);
    if (allowed == snapshot.goal_indexes_by_agent[i].end()) return false;
    if (agents[i].load_state == AgentLoadState::LOADED) continue;

    const auto task_iter =
        snapshot.pending_task_id_by_start_index.find(target_index);
    if (task_iter == snapshot.pending_task_id_by_start_index.end()) continue;
    auto* task = find_task_by_id(tasks, task_iter->second);
    if (task == nullptr || task->status != LifelongTaskStatus::PENDING ||
        task->start == nullptr || task->start->index != target_index ||
        !used_task_ids.insert(task->task_id).second) {
      return false;
    }
    selected_task_ids[i] = task->task_id;
  }

  for (size_t i = 0; i < agents.size(); ++i) {
    const auto target_index = instance.tasks[final_assignment[i]]->index;
    const auto target_iter = snapshot.target_by_index.find(target_index);
    if (target_iter == snapshot.target_by_index.end()) return false;
    agents[i].current_target = target_iter->second;
    if (agents[i].load_state == AgentLoadState::LOADED) continue;

    agents[i].current_task_id.reset();
    if (!selected_task_ids[i].has_value()) continue;
    auto* task = find_task_by_id(tasks, *selected_task_ids[i]);
    if (task == nullptr) return false;
    task->status = LifelongTaskStatus::ASSIGNED;
    task->assigned_agent_id = agents[i].agent_id;
    agents[i].current_task_id = task->task_id;
    agents[i].current_target = task->start;
  }
  return true;
}
