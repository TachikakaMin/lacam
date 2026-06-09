/*
 * Lifelong TAPF agent/task state transitions.
 */
#pragma once

#include <optional>

#include "lifelong_task.hpp"

enum class AgentLoadState {
  UNLOADED = 0,
  LOADED = 1,
};

struct LifelongAgentState {
  int agent_id = -1;
  Vertex* current_location = nullptr;
  AgentLoadState load_state = AgentLoadState::UNLOADED;
  std::optional<int> current_task_id;
  Vertex* current_target = nullptr;
  Config executed_path;
  int completed_task_count = 0;
};

struct LifelongStateTransitionResult {
  bool changed = false;
  std::string message;
};

LifelongTask* find_task_by_id(std::vector<LifelongTask>& tasks, int task_id);
const LifelongTask* find_task_by_id(const std::vector<LifelongTask>& tasks,
                                    int task_id);
bool vertex_in_goal_set(Vertex* vertex, const Vertices& goal_set);

void release_unpicked_assignments(std::vector<LifelongAgentState>& agents,
                                  std::vector<LifelongTask>& tasks);
LifelongStateTransitionResult try_pickup(LifelongAgentState& agent,
                                         std::vector<LifelongTask>& tasks,
                                         int timestep);
LifelongStateTransitionResult try_complete(LifelongAgentState& agent,
                                           std::vector<LifelongTask>& tasks,
                                           int timestep);
bool check_lifelong_state_invariants(
    const std::vector<LifelongAgentState>& agents,
    const std::vector<LifelongTask>& tasks, std::string* error = nullptr);
