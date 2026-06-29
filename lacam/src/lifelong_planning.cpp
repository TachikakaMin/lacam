#include "../include/lifelong_planning.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

#include "../include/tapf_assignment.hpp"

namespace
{
  constexpr int kDeliveryLocationKeyBase = 1000000000;
  constexpr int kPickupLocationKeyBase = 500000000;
  constexpr int kDeliveryLocationSlotStride = 1000000;
  constexpr int kCongestionRegionRadius = 2;

  std::vector<size_t> collect_pending_tasks(
      const std::vector<LifelongTask>& tasks)
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

  int lcm_upto(int value)
  {
    auto result = 1;
    for (int i = 2; i <= std::max(1, value); ++i) {
      result = std::lcm(result, i);
    }
    return result;
  }

  int deferred_assignment_offset(size_t num_agents,
                                 const MapDistanceCache& distances,
                                 int cost_scale)
  {
    const auto max_distance = static_cast<long long>(
        std::max(1, distances.metadata.traversable_count - 1));
    const auto max_service_cost = 2 * max_distance;
    const auto desired = (static_cast<long long>(num_agents) + 1) *
                         (max_service_cost + 1) *
                         static_cast<long long>(cost_scale);
    const auto max_offset =
        static_cast<long long>(kTapfAssignmentInfCost / 2 - 1) -
        max_distance * cost_scale - 1;
    return static_cast<int>(std::max(0LL, std::min(desired, max_offset)));
  }

  bool add_goal_option(LifelongPlanningSnapshot& snapshot, size_t agent,
                       Vertex* agent_location, Vertex* target,
                       int distance_scale, int cost_offset, int goal_key,
                       const MapDistanceCache& distances,
                       int service_duration = 0, bool force_replace = false)
  {
    if (agent_location == nullptr || target == nullptr || distance_scale <= 0 ||
        cost_offset >= kTapfAssignmentInfCost) {
      return false;
    }
    const auto root_distance = distances.get(agent_location, target);
    if (root_distance >= kMapDistanceInf) return false;
    const auto root_cost =
        static_cast<long long>(root_distance) * distance_scale + cost_offset;
    if (root_cost >= kTapfAssignmentInfCost) return false;

    auto& indexes = snapshot.goal_indexes_by_agent[agent];
    auto& offsets = snapshot.goal_cost_offsets_by_agent[agent];
    auto& scales = snapshot.goal_distance_scales_by_agent[agent];
    auto& keys = snapshot.goal_keys_by_agent[agent];
    auto& service_durations = snapshot.goal_service_durations_by_agent[agent];
    if (service_durations.size() < keys.size()) {
      service_durations.resize(keys.size(), 0);
    }
    const auto iter = std::find(keys.begin(), keys.end(), goal_key);
    if (iter == keys.end()) {
      indexes.push_back(target->index);
      offsets.push_back(cost_offset);
      scales.push_back(distance_scale);
      keys.push_back(goal_key);
      service_durations.push_back(std::max(0, service_duration));
      snapshot.target_by_index[target->index] = target;
      return true;
    } else {
      const auto option = std::distance(keys.begin(), iter);
      const auto old_root_cost =
          static_cast<long long>(root_distance) * scales[option] +
          offsets[option];
      if (force_replace || root_cost < old_root_cost) {
        offsets[option] = cost_offset;
        scales[option] = distance_scale;
        service_durations[option] = std::max(0, service_duration);
        snapshot.target_by_index[target->index] = target;
        return true;
      }
    }
    return false;
  }

  std::vector<const LifelongTask*> carried_tasks_for_agent(
      const LifelongAgentState& agent, const std::vector<LifelongTask>& tasks)
  {
    auto carried = std::vector<const LifelongTask*>();
    for (const auto task_id : agent.carried_task_ids) {
      const auto* task = find_task_by_id(tasks, task_id);
      if (task != nullptr) carried.push_back(task);
    }
    return carried;
  }

  int delivery_location_slot_key(int target_index, int slot)
  {
    return kDeliveryLocationKeyBase + slot * kDeliveryLocationSlotStride +
           target_index;
  }

  int pickup_location_key(int target_index)
  {
    return kPickupLocationKeyBase + target_index;
  }

  bool is_pickup_location_key(int key)
  {
    return key >= kPickupLocationKeyBase && key < kDeliveryLocationKeyBase;
  }

  bool is_delivery_location_key(int key)
  {
    return key >= kDeliveryLocationKeyBase;
  }

  int task_id_from_goal_option_key(int key)
  {
    return key >= 1000000 ? key / 1000000 - 1 : key;
  }

  bool vertex_index_in_goal_set(int vertex_index, const Vertices& goal_set)
  {
    return std::any_of(goal_set.begin(), goal_set.end(), [&](const auto* goal) {
      return goal != nullptr && goal->index == vertex_index;
    });
  }

  Vertex* representative_goal(Vertex* reference, const LifelongTask& task,
                              const MapDistanceCache& distances)
  {
    Vertex* best = nullptr;
    auto best_distance = kMapDistanceInf;
    for (auto goal : task.goal_set) {
      const auto distance = distances.get(reference, goal);
      if (distance < best_distance ||
          (distance == best_distance && best != nullptr &&
           std::make_tuple(goal->index / distances.metadata.width,
                           goal->index % distances.metadata.width,
                           goal->index) <
               std::make_tuple(best->index / distances.metadata.width,
                               best->index % distances.metadata.width,
                               best->index))) {
        best = goal;
        best_distance = distance;
      } else if (distance == best_distance && best == nullptr) {
        best = goal;
      }
    }
    return best_distance >= kMapDistanceInf ? nullptr : best;
  }

  int circle_cost(Vertex* reference,
                  const std::vector<const LifelongTask*>& tasks,
                  const MapDistanceCache& distances)
  {
    if (tasks.size() <= 1) return 0;
    auto representatives = Vertices();
    representatives.reserve(tasks.size());
    for (const auto* task : tasks) {
      auto* goal = representative_goal(reference, *task, distances);
      if (goal == nullptr) return kTapfAssignmentInfCost;
      representatives.push_back(goal);
    }
    std::sort(representatives.begin(), representatives.end(),
              [&](const auto* lhs, const auto* rhs) {
                return std::make_tuple(lhs->index / distances.metadata.width,
                                       lhs->index % distances.metadata.width,
                                       lhs->index) <
                       std::make_tuple(rhs->index / distances.metadata.width,
                                       rhs->index % distances.metadata.width,
                                       rhs->index);
              });
    auto total = 0LL;
    for (size_t i = 0; i < representatives.size(); ++i) {
      const auto* from = representatives[i];
      const auto* to = representatives[(i + 1) % representatives.size()];
      const auto distance =
          distances.get(const_cast<Vertex*>(from), const_cast<Vertex*>(to));
      if (distance >= kMapDistanceInf) return kTapfAssignmentInfCost;
      total += distance;
    }
    return total >= kTapfAssignmentInfCost ? kTapfAssignmentInfCost
                                           : static_cast<int>(total);
  }

  int scaled_static_cost(long long numerator, int common_scale, int denominator)
  {
    if (denominator <= 0 || numerator >= kTapfAssignmentInfCost) {
      return kTapfAssignmentInfCost;
    }
    const auto cost = numerator * common_scale / denominator;
    return cost >= kTapfAssignmentInfCost ? kTapfAssignmentInfCost
                                          : static_cast<int>(cost);
  }

  bool uses_mild_loaded_pickup_delay(int assignment_cost_mode)
  {
    return assignment_cost_mode == LIFELONG_ASSIGNMENT_COST_BASELINE ||
           assignment_cost_mode ==
               LIFELONG_ASSIGNMENT_COST_MILD_PICKUP_DELAY ||
           assignment_cost_mode == LIFELONG_ASSIGNMENT_COST_CONGESTION;
  }

  bool uses_local_congestion_cost(int assignment_cost_mode)
  {
    return assignment_cost_mode == LIFELONG_ASSIGNMENT_COST_BASELINE ||
           assignment_cost_mode == LIFELONG_ASSIGNMENT_COST_CONGESTION;
  }

  int add_cost_offset_penalty(int base_offset, long long penalty)
  {
    if (base_offset >= kTapfAssignmentInfCost) return kTapfAssignmentInfCost;
    if (penalty <= 0) return base_offset;
    const auto total = static_cast<long long>(base_offset) + penalty;
    return total >= kTapfAssignmentInfCost ? kTapfAssignmentInfCost
                                           : static_cast<int>(total);
  }

  bool vertex_in_local_region(Vertex* center, Vertex* candidate,
                              const MapDistanceCache& distances)
  {
    if (center == nullptr || candidate == nullptr) return false;
    return distances.get(center, candidate) <= kCongestionRegionRadius;
  }

  void increment_cell_count(std::vector<int>& counts, Vertex* vertex)
  {
    if (vertex == nullptr || vertex->index < 0 ||
        vertex->index >= static_cast<int>(counts.size())) {
      return;
    }
    ++counts[vertex->index];
  }

  int local_region_cell_count(Vertex* center,
                              const std::vector<int>& count_by_index)
  {
    if (center == nullptr) return 0;
    auto total = 0;
    auto seen = std::vector<int>();
    seen.reserve(16);
    auto add_vertex = [&](Vertex* vertex) {
      if (vertex == nullptr || vertex->index < 0 ||
          vertex->index >= static_cast<int>(count_by_index.size())) {
        return;
      }
      if (std::find(seen.begin(), seen.end(), vertex->index) != seen.end()) {
        return;
      }
      seen.push_back(vertex->index);
      total += count_by_index[vertex->index];
    };

    add_vertex(center);
    for (auto* first : center->neighbor) {
      add_vertex(first);
      if (kCongestionRegionRadius < 2 || first == nullptr) continue;
      for (auto* second : first->neighbor) {
        add_vertex(second);
      }
    }
    return total;
  }

  int local_congestion_count(
      Vertex* target, Vertex* excluded_agent_location,
      const std::vector<int>& current_agent_count_by_index,
      const std::vector<int>& created_target_count_by_index,
      const MapDistanceCache& distances, int assignment_cost_mode)
  {
    if (!uses_local_congestion_cost(assignment_cost_mode)) return 0;
    auto count =
        local_region_cell_count(target, current_agent_count_by_index) +
        local_region_cell_count(target, created_target_count_by_index);
    if (vertex_in_local_region(target, excluded_agent_location, distances)) {
      --count;
    }
    return std::max(0, count);
  }

  long long local_congestion_penalty(
      Vertex* target, Vertex* excluded_agent_location,
      const std::vector<int>& current_agent_count_by_index,
      const std::vector<int>& created_target_count_by_index,
      const MapDistanceCache& distances, int assignment_cost_mode,
      int common_scale)
  {
    const auto penalty_scale =
        assignment_cost_mode == LIFELONG_ASSIGNMENT_COST_BASELINE
            ? std::max(1, common_scale / 2)
            : std::max(1, common_scale);
    return static_cast<long long>(penalty_scale) *
           local_congestion_count(target, excluded_agent_location,
                                  current_agent_count_by_index,
                                  created_target_count_by_index, distances,
                                  assignment_cost_mode);
  }

  long long mild_loaded_pickup_delay_penalty(
      int carried_count, int root_pickup_distance, int pickup_service_duration,
      int assignment_cost_mode)
  {
    if (carried_count <= 0 ||
        !uses_mild_loaded_pickup_delay(assignment_cost_mode)) {
      return 0;
    }
    return static_cast<long long>(carried_count) *
           ((std::max(0, root_pickup_distance) +
             std::max(0, pickup_service_duration) + 1) /
            2);
  }

  void normalize_agent_task_state(std::vector<LifelongAgentState>& agents,
                                  const std::vector<LifelongTask>& tasks)
  {
    for (auto& agent : agents) {
      if (agent.carried_task_ids.empty() &&
          agent.load_state == AgentLoadState::LOADED &&
          agent.current_task_id.has_value()) {
        const auto* task = find_task_by_id(tasks, *agent.current_task_id);
        if (task != nullptr && task->status == LifelongTaskStatus::PICKED) {
          agent.carried_task_ids.push_back(task->task_id);
        }
      }
      if (!agent.assigned_task_id.has_value() &&
          agent.carried_task_ids.empty() && agent.current_task_id.has_value()) {
        const auto* task = find_task_by_id(tasks, *agent.current_task_id);
        if (task != nullptr && task->status == LifelongTaskStatus::ASSIGNED) {
          agent.assigned_task_id = task->task_id;
        }
      }
      sync_agent_load_state(agent);
    }
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
    const MapDistanceCache& distances, int multi_carry_capacity,
    int max_shared_drop_goal_agents, int pickup_service_duration,
    int delivery_service_duration,
    const std::vector<float>& agent_priority_offsets,
    const std::vector<std::unordered_map<int, int> >&
        preferred_pickup_task_id_by_start_index_by_agent,
    int assignment_cost_mode)
{
  normalize_agent_task_state(agents, tasks);
  auto previous_task_ids =
      std::vector<std::optional<int> >(agents.size(), std::nullopt);
  for (size_t i = 0; i < agents.size(); ++i) {
    previous_task_ids[i] = agents[i].assigned_task_id;
  }

  release_unpicked_assignments(agents, tasks);

  auto snapshot = LifelongPlanningSnapshot();
  snapshot.goal_indexes_by_agent.resize(agents.size());
  snapshot.goal_cost_offsets_by_agent.resize(agents.size());
  snapshot.goal_distance_scales_by_agent.resize(agents.size());
  snapshot.goal_service_durations_by_agent.resize(agents.size());
  snapshot.goal_keys_by_agent.resize(agents.size());
  snapshot.agent_priority_offsets.assign(agents.size(), 0.0f);
  const auto has_inherited_priorities = !agent_priority_offsets.empty();
  if (has_inherited_priorities && agent_priority_offsets.size() != agents.size()) {
    snapshot.feasible = false;
  }
  const auto has_preferred_pickups =
      !preferred_pickup_task_id_by_start_index_by_agent.empty();
  if (has_preferred_pickups &&
      preferred_pickup_task_id_by_start_index_by_agent.size() != agents.size()) {
    snapshot.feasible = false;
  }
  snapshot.pending_task_id_by_start_index_by_agent.resize(agents.size());
  snapshot.common_cost_scale = lcm_upto(multi_carry_capacity);
  const auto pending_tasks = collect_pending_tasks(tasks);
  const auto common_scale = snapshot.common_cost_scale;
  const auto deferred_offset =
      deferred_assignment_offset(agents.size(), distances, common_scale);
  const auto unloaded_count =
      std::count_if(agents.begin(), agents.end(), [](const auto& agent) {
        return agent.load_state == AgentLoadState::UNLOADED;
      });
  auto pending_start_indexes = std::unordered_set<int>();

  for (const auto task_idx : pending_tasks) {
    const auto& task = tasks[task_idx];
    if (task.start == nullptr || task.goal_set.empty()) {
      snapshot.feasible = false;
      continue;
    }
    pending_start_indexes.insert(task.start->index);
  }
  const auto needs_unloaded_wait_targets =
      pending_start_indexes.size() < static_cast<size_t>(unloaded_count);
  const auto cell_count =
      std::max(0, distances.metadata.width * distances.metadata.height);
  auto current_agent_count_by_index = std::vector<int>(cell_count, 0);
  auto created_target_count_by_index = std::vector<int>(cell_count, 0);
  for (const auto& agent : agents) {
    increment_cell_count(current_agent_count_by_index, agent.current_location);
  }

  for (size_t i = 0; i < agents.size(); ++i) {
    auto& agent = agents[i];
    if (agent.current_location == nullptr) {
      snapshot.feasible = false;
      continue;
    }
    if (max_shared_drop_goal_agents <= 0) {
      snapshot.feasible = false;
      continue;
    }

    const auto carried = carried_tasks_for_agent(agent, tasks);
    const auto carried_count = static_cast<int>(carried.size());
    if (carried_count != carried_task_count(agent)) {
      snapshot.feasible = false;
      continue;
    }
    if (carried_count > 0) {
      snapshot.agent_priority_offsets[i] =
          2.0f * static_cast<float>(agent.loaded_distance_since_last_delivery) /
          static_cast<float>(std::max<size_t>(1, agents.size()));
    }
    if (has_inherited_priorities && i < agent_priority_offsets.size()) {
      const auto aging_priority =
          std::min(2.0f, agent_priority_offsets[i] /
                             static_cast<float>(std::max<size_t>(1, agents.size())));
      snapshot.agent_priority_offsets[i] += aging_priority;
    }

    if (carried_count < multi_carry_capacity) {
      for (const auto task_idx : pending_tasks) {
        const auto& task = tasks[task_idx];
        const auto pickup_distance =
            distances.get(agent.current_location, task.start);
        if (pickup_distance >= kMapDistanceInf) continue;
        const auto delivery_cost = task_delivery_cost(task, distances);
        if (delivery_cost >= kTapfAssignmentInfCost) continue;
        auto pickup_set = carried;
        pickup_set.push_back(&task);
        const auto circle = circle_cost(task.start, pickup_set, distances);
        if (circle >= kTapfAssignmentInfCost) continue;
        const auto denominator = carried_count + 1;
        const auto delay_penalty = mild_loaded_pickup_delay_penalty(
            carried_count, pickup_distance, pickup_service_duration,
            assignment_cost_mode);
        const auto switched = previous_task_ids[i].has_value() &&
                              *previous_task_ids[i] != task.task_id;
        const auto static_cost =
            static_cast<long long>(delivery_cost) + circle +
            (carried_count > 0 ? agent.loaded_distance_since_last_delivery
                               : 0) +
            (switched ? 1 : 0) + delay_penalty * denominator;
        auto offset =
            scaled_static_cost(static_cost, common_scale, denominator);
        offset = add_cost_offset_penalty(
            offset, local_congestion_penalty(
                        task.start, agent.current_location,
                        current_agent_count_by_index,
                        created_target_count_by_index, distances,
                        assignment_cost_mode, common_scale));
        const auto distance_scale = common_scale / denominator;
        // A physical pickup has capacity one in the current planning round.
        // Tasks sharing it become available again after the event-driven
        // replan, so they are serviced sequentially rather than contending for
        // the same cell simultaneously.
        const auto pickup_key = pickup_location_key(task.start->index);
        auto force_pickup_task = false;
        if (has_preferred_pickups &&
            i < preferred_pickup_task_id_by_start_index_by_agent.size()) {
          const auto& preferred =
              preferred_pickup_task_id_by_start_index_by_agent[i];
          const auto iter = preferred.find(task.start->index);
          force_pickup_task =
              iter != preferred.end() && iter->second == task.task_id;
        }
        const auto has_pickup_option =
            std::find(snapshot.goal_keys_by_agent[i].begin(),
                      snapshot.goal_keys_by_agent[i].end(),
                      pickup_key) != snapshot.goal_keys_by_agent[i].end();
        if (add_goal_option(snapshot, i, agent.current_location, task.start,
                            distance_scale, offset, pickup_key, distances,
                            pickup_service_duration, force_pickup_task)) {
          snapshot
              .pending_task_id_by_start_index_by_agent[i][task.start->index] =
              task.task_id;
          if (!has_pickup_option) {
            increment_cell_count(created_target_count_by_index, task.start);
          }
        }
      }
    }

    if (carried_count > 0) {
      auto delivery_targets = std::unordered_set<int>();
      for (const auto* task : carried) {
        if (task->status != LifelongTaskStatus::PICKED ||
            task->picked_agent_id != agent.agent_id || task->goal_set.empty()) {
          snapshot.feasible = false;
          continue;
        }
        for (auto goal : task->goal_set) {
          if (!delivery_targets.insert(goal->index).second) continue;
          const auto circle = circle_cost(goal, carried, distances);
          if (circle >= kTapfAssignmentInfCost) continue;
          const auto static_cost = static_cast<long long>(circle);
          auto offset =
              scaled_static_cost(static_cost, common_scale, carried_count);
          offset = add_cost_offset_penalty(
              offset, local_congestion_penalty(
                          goal, agent.current_location,
                          current_agent_count_by_index,
                          created_target_count_by_index, distances,
                          assignment_cost_mode, common_scale));
          const auto distance_scale = common_scale / carried_count;
          auto added_delivery_target = false;
          for (auto slot = 0; slot < max_shared_drop_goal_agents; ++slot) {
            const auto goal_key = delivery_location_slot_key(goal->index, slot);
            added_delivery_target =
                add_goal_option(snapshot, i, agent.current_location, goal,
                                distance_scale, offset, goal_key, distances,
                                delivery_service_duration) ||
                added_delivery_target;
          }
          if (added_delivery_target) {
            increment_cell_count(created_target_count_by_index, goal);
          }
        }
      }
    }

    if (carried_count > 0 || needs_unloaded_wait_targets) {
      add_goal_option(snapshot, i, agent.current_location,
                      agent.current_location, common_scale, deferred_offset,
                      -static_cast<int>(i) - 1, distances);
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
  return TAPFInstance(
      map_filename, start_indexes, snapshot.goal_indexes_by_agent,
      snapshot.goal_cost_offsets_by_agent, snapshot.common_cost_scale,
      snapshot.goal_distance_scales_by_agent, snapshot.agent_priority_offsets,
      false, snapshot.goal_keys_by_agent,
      snapshot.goal_service_durations_by_agent);
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
    const auto target_key = target < static_cast<int>(instance.task_keys.size())
                                ? instance.task_keys[target]
                                : target_index;
    auto task_id = is_delivery_location_key(target_key)
                       ? -1
                       : task_id_from_goal_option_key(target_key);
    auto option = -1;
    for (size_t candidate = 0;
         candidate < snapshot.goal_indexes_by_agent[i].size(); ++candidate) {
      if (snapshot.goal_indexes_by_agent[i][candidate] == target_index &&
          candidate < snapshot.goal_keys_by_agent[i].size() &&
          snapshot.goal_keys_by_agent[i][candidate] == target_key) {
        option = static_cast<int>(candidate);
        break;
      }
    }
    if (option < 0) {
      return false;
    }
    if (target_key < 0) continue;
    if (is_pickup_location_key(target_key)) {
      const auto& pending_by_start =
          snapshot.pending_task_id_by_start_index_by_agent[i];
      const auto pending = pending_by_start.find(target_index);
      if (pending == pending_by_start.end()) return false;
      task_id = pending->second;
    }
    if (target_key < 1000000) {
      auto* task = find_task_by_id(tasks, task_id);
      if (task == nullptr || task->status != LifelongTaskStatus::PENDING ||
          task->start == nullptr || task->start->index != target_index ||
          !used_task_ids.insert(task->task_id).second) {
        return false;
      }
      selected_task_ids[i] = task->task_id;
      continue;
    }
    if (is_pickup_location_key(target_key)) {
      auto* task = find_task_by_id(tasks, task_id);
      if (task == nullptr || task->status != LifelongTaskStatus::PENDING ||
          task->start == nullptr || task->start->index != target_index ||
          !used_task_ids.insert(task->task_id).second) {
        return false;
      }
      selected_task_ids[i] = task->task_id;
      continue;
    }
    if (is_delivery_location_key(target_key)) {
      const auto valid_delivery = std::any_of(
          agents[i].carried_task_ids.begin(), agents[i].carried_task_ids.end(),
          [&](const int carried_task_id) {
            const auto* task = find_task_by_id(tasks, carried_task_id);
            return task != nullptr &&
                   task->status == LifelongTaskStatus::PICKED &&
                   vertex_index_in_goal_set(target_index, task->goal_set);
          });
      if (!valid_delivery) return false;
      continue;
    }
    auto* task = find_task_by_id(tasks, task_id);
    if (task == nullptr) return false;
    if (task->status == LifelongTaskStatus::PICKED &&
        std::find(agents[i].carried_task_ids.begin(),
                  agents[i].carried_task_ids.end(),
                  task->task_id) != agents[i].carried_task_ids.end() &&
        vertex_index_in_goal_set(target_index, task->goal_set)) {
      continue;
    }
    if (task->status != LifelongTaskStatus::PENDING || task->start == nullptr ||
        task->start->index != target_index ||
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

    agents[i].assigned_task_id.reset();
    if (!selected_task_ids[i].has_value()) {
      sync_agent_load_state(agents[i]);
      continue;
    }
    auto* task = find_task_by_id(tasks, *selected_task_ids[i]);
    if (task == nullptr) return false;
    task->status = LifelongTaskStatus::ASSIGNED;
    task->assigned_agent_id = agents[i].agent_id;
    agents[i].assigned_task_id = task->task_id;
    agents[i].current_target = task->start;
    sync_agent_load_state(agents[i]);
  }
  return true;
}
