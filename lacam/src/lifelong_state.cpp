#include "../include/lifelong_state.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>

LifelongTask* find_task_by_id(std::vector<LifelongTask>& tasks, int task_id)
{
  for (auto& task : tasks) {
    if (task.task_id == task_id) return &task;
  }
  return nullptr;
}

const LifelongTask* find_task_by_id(const std::vector<LifelongTask>& tasks,
                                    int task_id)
{
  for (const auto& task : tasks) {
    if (task.task_id == task_id) return &task;
  }
  return nullptr;
}

bool vertex_in_goal_set(Vertex* vertex, const Vertices& goal_set)
{
  return std::find(goal_set.begin(), goal_set.end(), vertex) != goal_set.end();
}

int carried_task_count(const LifelongAgentState& agent)
{
  return static_cast<int>(agent.carried_task_ids.size());
}

bool agent_is_loaded(const LifelongAgentState& agent)
{
  return !agent.carried_task_ids.empty();
}

void sync_agent_load_state(LifelongAgentState& agent)
{
  agent.load_state =
      agent.carried_task_ids.empty() ? AgentLoadState::UNLOADED
                                     : AgentLoadState::LOADED;
  if (agent.carried_task_ids.empty()) {
    agent.loaded_distance_since_last_delivery = 0;
    if (agent.assigned_task_id.has_value()) {
      agent.current_task_id = agent.assigned_task_id;
    } else {
      agent.current_task_id.reset();
    }
  } else if (agent.assigned_task_id.has_value()) {
    agent.current_task_id = agent.assigned_task_id;
  } else {
    agent.current_task_id = agent.carried_task_ids.front();
  }
}

void record_loaded_movement(LifelongAgentState& agent, Vertex* previous)
{
  if (!agent_is_loaded(agent) || previous == nullptr ||
      agent.current_location == nullptr || previous == agent.current_location) {
    return;
  }
  ++agent.loaded_distance_since_last_delivery;
}

void release_unpicked_assignments(std::vector<LifelongAgentState>& agents,
                                  std::vector<LifelongTask>& tasks)
{
  for (auto& task : tasks) {
    if (task.status != LifelongTaskStatus::ASSIGNED) continue;
    task.status = LifelongTaskStatus::PENDING;
    task.assigned_agent_id.reset();
  }
  for (auto& agent : agents) {
    if (agent.carried_task_ids.empty() &&
        agent.load_state == AgentLoadState::LOADED &&
        agent.current_task_id.has_value()) {
      agent.carried_task_ids.push_back(*agent.current_task_id);
    }
    agent.assigned_task_id.reset();
    if (!agent_is_loaded(agent)) agent.current_target = agent.current_location;
    sync_agent_load_state(agent);
  }
}

LifelongStateTransitionResult try_pickup(LifelongAgentState& agent,
                                         std::vector<LifelongTask>& tasks,
                                         int timestep,
                                         int multi_carry_capacity)
{
  const auto task_id =
      agent.assigned_task_id.has_value()
          ? agent.assigned_task_id
          : (!agent_is_loaded(agent) ? agent.current_task_id : std::nullopt);
  if (!task_id.has_value() ||
      carried_task_count(agent) >= multi_carry_capacity) {
    return LifelongStateTransitionResult();
  }
  auto* task = find_task_by_id(tasks, *task_id);
  if (task == nullptr || task->status != LifelongTaskStatus::ASSIGNED ||
      task->assigned_agent_id != agent.agent_id ||
      agent.current_location != task->start) {
    return LifelongStateTransitionResult();
  }

  task->status = LifelongTaskStatus::PICKED;
  task->picked_agent_id = agent.agent_id;
  task->pickup_timestep = timestep;
  task->assigned_agent_id.reset();
  agent.carried_task_ids.push_back(task->task_id);
  agent.assigned_task_id.reset();
  agent.current_target = nullptr;
  sync_agent_load_state(agent);
  return LifelongStateTransitionResult{true, "pickup"};
}

LifelongStateTransitionResult try_complete(LifelongAgentState& agent,
                                           std::vector<LifelongTask>& tasks,
                                           int timestep)
{
  if (!agent_is_loaded(agent) && agent.load_state == AgentLoadState::LOADED &&
      agent.current_task_id.has_value()) {
    agent.carried_task_ids.push_back(*agent.current_task_id);
  }
  if (!agent_is_loaded(agent)) {
    return LifelongStateTransitionResult();
  }
  if (agent.last_delivery_timestep == timestep) {
    return LifelongStateTransitionResult();
  }

  auto eligible = std::vector<LifelongTask*>();
  for (const auto task_id : agent.carried_task_ids) {
    auto* task = find_task_by_id(tasks, task_id);
    if (task != nullptr && task->status == LifelongTaskStatus::PICKED &&
        task->picked_agent_id == agent.agent_id &&
        vertex_in_goal_set(agent.current_location, task->goal_set)) {
      eligible.push_back(task);
    }
  }
  if (eligible.empty()) {
    return LifelongStateTransitionResult();
  }
  std::sort(eligible.begin(), eligible.end(), [](const auto* lhs,
                                                 const auto* rhs) {
    const auto lhs_pickup = lhs->pickup_timestep.value_or(
        std::numeric_limits<int>::max());
    const auto rhs_pickup = rhs->pickup_timestep.value_or(
        std::numeric_limits<int>::max());
    if (lhs_pickup != rhs_pickup) return lhs_pickup < rhs_pickup;
    return lhs->task_id < rhs->task_id;
  });
  auto* task = eligible.front();

  task->status = LifelongTaskStatus::COMPLETED;
  task->completion_timestep = timestep;
  task->assigned_agent_id.reset();
  task->picked_agent_id.reset();
  if (agent.last_completed_task_type.has_value() &&
      *agent.last_completed_task_type != task->task_type) {
    ++agent.alternating_completed_task_count;
  }
  agent.last_completed_task_type = task->task_type;
  agent.carried_task_ids.erase(
      std::remove(agent.carried_task_ids.begin(), agent.carried_task_ids.end(),
                  task->task_id),
      agent.carried_task_ids.end());
  agent.loaded_distance_since_last_delivery = 0;
  agent.last_delivery_timestep = timestep;
  agent.current_target =
      agent.carried_task_ids.empty() ? agent.current_location : nullptr;
  ++agent.completed_task_count;
  sync_agent_load_state(agent);
  return LifelongStateTransitionResult{true, "completion"};
}

bool check_lifelong_state_invariants(
    const std::vector<LifelongAgentState>& agents,
    const std::vector<LifelongTask>& tasks, std::string* error,
    int multi_carry_capacity)
{
  auto fail = [&](const std::string& message) {
    if (error != nullptr) *error = message;
    return false;
  };

  auto loaded_agent_by_task = std::unordered_map<int, int>();
  auto assigned_agent_ids = std::unordered_set<int>();
  for (const auto& agent : agents) {
    if (agent.current_location == nullptr) return fail("agent on null location");
    if (static_cast<int>(agent.carried_task_ids.size()) >
        multi_carry_capacity) {
      return fail("agent exceeds multi-carry capacity");
    }
    const auto legacy_loaded =
        agent.carried_task_ids.empty() &&
        agent.load_state == AgentLoadState::LOADED &&
        agent.current_task_id.has_value();
    if ((agent.load_state == AgentLoadState::LOADED) !=
        (!agent.carried_task_ids.empty() || legacy_loaded)) {
      return fail("agent load state disagrees with carried tasks");
    }
    if (agent.carried_task_ids.empty() && !legacy_loaded &&
        agent.loaded_distance_since_last_delivery != 0) {
      return fail("unloaded agent has loaded distance");
    }
    if (agent.assigned_task_id.has_value() &&
        !assigned_agent_ids.insert(agent.agent_id).second) {
      return fail("agent assigned to multiple tasks");
    }
    auto seen_carried = std::unordered_set<int>();
    for (const auto task_id : agent.carried_task_ids) {
      if (!seen_carried.insert(task_id).second) {
        return fail("agent carries duplicate task");
      }
      if (!loaded_agent_by_task.emplace(task_id, agent.agent_id).second) {
        return fail("task carried by multiple agents");
      }
    }
    if (legacy_loaded) {
      if (!loaded_agent_by_task.emplace(*agent.current_task_id,
                                        agent.agent_id).second) {
        return fail("task carried by multiple agents");
      }
    }
  }

  auto active_start_counts = std::unordered_map<int, int>();
  auto active_agent_bindings = std::unordered_set<int>();
  for (const auto& task : tasks) {
    if (task.status != LifelongTaskStatus::COMPLETED && task.start == nullptr) {
      return fail("unfinished task without start");
    }
    if (task.status == LifelongTaskStatus::PENDING ||
        task.status == LifelongTaskStatus::ASSIGNED) {
      if (++active_start_counts[task.start->index] >
          kLifelongTaskStartCapacity) {
        return fail("unpicked task start capacity exceeded");
      }
    }
    if (task.assigned_agent_id.has_value()) {
      if (task.status != LifelongTaskStatus::ASSIGNED) {
        return fail("non-assigned task has assigned agent");
      }
      if (!active_agent_bindings.insert(*task.assigned_agent_id).second) {
        return fail("agent assigned to multiple tasks");
      }
      const auto agent_iter = std::find_if(
          agents.begin(), agents.end(), [&](const auto& agent) {
            return agent.agent_id == *task.assigned_agent_id;
          });
      const auto legacy_assigned =
          agent_iter != agents.end() && agent_iter->carried_task_ids.empty() &&
          agent_iter->current_task_id == task.task_id;
      if (agent_iter == agents.end() ||
          (agent_iter->assigned_task_id != task.task_id &&
           !legacy_assigned)) {
        return fail("assigned task without matching agent");
      }
    }
    if (task.status == LifelongTaskStatus::PICKED) {
      if (!task.picked_agent_id.has_value()) {
        return fail("picked task without picked agent");
      }
      const auto iter = loaded_agent_by_task.find(task.task_id);
      if (iter == loaded_agent_by_task.end() ||
          iter->second != *task.picked_agent_id) {
        return fail("picked task without matching loaded agent");
      }
    }
    if (task.status == LifelongTaskStatus::COMPLETED) {
      if (task.assigned_agent_id.has_value() || task.picked_agent_id.has_value()) {
        return fail("completed task still bound to agent");
      }
      if (loaded_agent_by_task.find(task.task_id) != loaded_agent_by_task.end()) {
        return fail("completed task still carried");
      }
    }
  }
  return true;
}
