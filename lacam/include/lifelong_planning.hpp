/*
 * Conversion from lifelong TAPF state to one-shot TAPF planning inputs.
 */
#pragma once

#include <unordered_map>

#include "instance.hpp"
#include "lifelong_state.hpp"
#include "map_dist_cache.hpp"

struct LifelongPlanningSnapshot {
  std::vector<std::vector<int> > goal_indexes_by_agent;
  std::vector<std::vector<int> > goal_cost_offsets_by_agent;
  std::vector<std::unordered_map<int, int> >
      pending_task_id_by_start_index_by_agent;
  std::unordered_map<int, Vertex*> target_by_index;
  bool feasible = true;
};

int lifelong_unloaded_assignment_cost(const LifelongAgentState& agent,
                                      const LifelongTask& task,
                                      const MapDistanceCache& distances);
int lifelong_loaded_cost(const LifelongAgentState& agent,
                         const LifelongTask& task,
                         const MapDistanceCache& distances);
LifelongPlanningSnapshot prepare_lifelong_planning_snapshot(
    std::vector<LifelongAgentState>& agents, std::vector<LifelongTask>& tasks,
    const MapDistanceCache& distances);
TAPFInstance build_lifelong_tapf_instance(
    const std::string& map_filename,
    const std::vector<LifelongAgentState>& agents,
    const LifelongPlanningSnapshot& snapshot);
bool apply_lifelong_solution_assignment(
    std::vector<LifelongAgentState>& agents, std::vector<LifelongTask>& tasks,
    const LifelongPlanningSnapshot& snapshot, const TAPFInstance& instance,
    const std::vector<int>& final_assignment);
