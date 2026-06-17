#include "../include/lifelong_state.hpp"

#include <algorithm>
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

void release_unpicked_assignments(std::vector<LifelongAgentState>& agents,
                                  std::vector<LifelongTask>& tasks)
{
  for (auto& task : tasks) {
    if (task.status != LifelongTaskStatus::ASSIGNED) continue;
    task.status = LifelongTaskStatus::PENDING;
    task.assigned_agent_id.reset();
  }
  for (auto& agent : agents) {
    if (agent.load_state != AgentLoadState::UNLOADED) continue;
    agent.current_task_id.reset();
    agent.current_target = agent.current_location;
  }
}

LifelongStateTransitionResult try_pickup(LifelongAgentState& agent,
                                         std::vector<LifelongTask>& tasks,
                                         int timestep)
{
  if (agent.load_state != AgentLoadState::UNLOADED ||
      !agent.current_task_id.has_value()) {
    return LifelongStateTransitionResult();
  }
  auto* task = find_task_by_id(tasks, *agent.current_task_id);
  if (task == nullptr || task->status != LifelongTaskStatus::ASSIGNED ||
      task->assigned_agent_id != agent.agent_id ||
      agent.current_location != task->start) {
    return LifelongStateTransitionResult();
  }

  task->status = LifelongTaskStatus::PICKED;
  task->picked_agent_id = agent.agent_id;
  task->pickup_timestep = timestep;
  agent.load_state = AgentLoadState::LOADED;
  agent.current_target = nullptr;
  return LifelongStateTransitionResult{true, "pickup"};
}

LifelongStateTransitionResult try_complete(LifelongAgentState& agent,
                                           std::vector<LifelongTask>& tasks,
                                           int timestep)
{
  if (agent.load_state != AgentLoadState::LOADED ||
      !agent.current_task_id.has_value()) {
    return LifelongStateTransitionResult();
  }
  auto* task = find_task_by_id(tasks, *agent.current_task_id);
  if (task == nullptr || task->status != LifelongTaskStatus::PICKED ||
      task->picked_agent_id != agent.agent_id ||
      !vertex_in_goal_set(agent.current_location, task->goal_set)) {
    return LifelongStateTransitionResult();
  }

  task->status = LifelongTaskStatus::COMPLETED;
  task->completion_timestep = timestep;
  task->assigned_agent_id.reset();
  task->picked_agent_id.reset();
  if (agent.last_completed_task_type.has_value() &&
      *agent.last_completed_task_type != task->task_type) {
    ++agent.alternating_completed_task_count;
  }
  agent.last_completed_task_type = task->task_type;
  agent.load_state = AgentLoadState::UNLOADED;
  agent.current_task_id.reset();
  agent.current_target = agent.current_location;
  ++agent.completed_task_count;
  return LifelongStateTransitionResult{true, "completion"};
}

bool check_lifelong_state_invariants(
    const std::vector<LifelongAgentState>& agents,
    const std::vector<LifelongTask>& tasks, std::string* error)
{
  auto fail = [&](const std::string& message) {
    if (error != nullptr) *error = message;
    return false;
  };

  auto loaded_agent_by_task = std::unordered_map<int, int>();
  for (const auto& agent : agents) {
    if (agent.current_location == nullptr) return fail("agent on null location");
    if (agent.load_state == AgentLoadState::LOADED) {
      if (!agent.current_task_id.has_value()) {
        return fail("loaded agent without task");
      }
      loaded_agent_by_task[*agent.current_task_id] = agent.agent_id;
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
      if (!active_agent_bindings.insert(*task.assigned_agent_id).second) {
        return fail("agent assigned to multiple tasks");
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
    }
  }
  return true;
}
