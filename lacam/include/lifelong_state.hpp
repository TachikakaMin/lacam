/*
 * Lifelong TAPF agent/task state transitions.
 */
#pragma once

#include <optional>
#include <vector>

#include "lifelong_task.hpp"

enum class AgentLoadState {
  UNLOADED = 0,
  LOADED = 1,
};

struct LifelongAgentState {
  int agent_id = -1;
  Vertex* current_location = nullptr;
  AgentLoadState load_state = AgentLoadState::UNLOADED;
  std::optional<int> assigned_task_id;
  std::vector<int> carried_task_ids;
  std::optional<int> current_task_id;
  Vertex* current_target = nullptr;
  Config executed_path;
  int completed_task_count = 0;
  int loaded_distance_since_last_delivery = 0;
  int last_delivery_timestep = -1;
  std::optional<LifelongTaskType> last_completed_task_type;
  int alternating_completed_task_count = 0;
};

struct LifelongStateTransitionResult {
  bool changed = false;
  std::string message;
};

LifelongTask* find_task_by_id(std::vector<LifelongTask>& tasks, int task_id);
const LifelongTask* find_task_by_id(const std::vector<LifelongTask>& tasks,
                                    int task_id);
bool vertex_in_goal_set(Vertex* vertex, const Vertices& goal_set);
int carried_task_count(const LifelongAgentState& agent);
bool agent_is_loaded(const LifelongAgentState& agent);
void sync_agent_load_state(LifelongAgentState& agent);
void record_loaded_movement(LifelongAgentState& agent, Vertex* previous);

void release_unpicked_assignments(std::vector<LifelongAgentState>& agents,
                                  std::vector<LifelongTask>& tasks);
LifelongStateTransitionResult try_pickup(LifelongAgentState& agent,
                                         std::vector<LifelongTask>& tasks,
                                         int timestep,
                                         int multi_carry_capacity = 1);
LifelongStateTransitionResult try_complete(LifelongAgentState& agent,
                                           std::vector<LifelongTask>& tasks,
                                           int timestep);
bool check_lifelong_state_invariants(
    const std::vector<LifelongAgentState>& agents,
    const std::vector<LifelongTask>& tasks, std::string* error = nullptr,
    int multi_carry_capacity = 1);
