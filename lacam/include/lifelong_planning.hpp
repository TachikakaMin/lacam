/*
 * Conversion from lifelong TAPF state to one-shot TAPF planning inputs.
 */
#pragma once

#include "instance.hpp"
#include "lifelong_state.hpp"
#include "map_dist_cache.hpp"
#include "tapf_assignment.hpp"

struct LifelongPlanningSnapshot {
  std::vector<std::vector<int> > goal_indexes_by_agent;
  std::vector<std::optional<int> > assigned_task_ids_by_agent;
  int assignment_cost = 0;
  bool feasible = true;
};

int lifelong_unloaded_assignment_cost(const LifelongAgentState& agent,
                                      const LifelongTask& task,
                                      const MapDistanceCache& distances);
int lifelong_loaded_cost(const LifelongAgentState& agent,
                         const LifelongTask& task,
                         const MapDistanceCache& distances);
LifelongPlanningSnapshot assign_lifelong_tasks_for_replanning(
    std::vector<LifelongAgentState>& agents, std::vector<LifelongTask>& tasks,
    const MapDistanceCache& distances);
TAPFInstance build_lifelong_tapf_instance(
    const std::string& map_filename,
    const std::vector<LifelongAgentState>& agents,
    const LifelongPlanningSnapshot& snapshot);
