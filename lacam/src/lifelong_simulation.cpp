#include "../include/lifelong_simulation.hpp"

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "../include/utils.hpp"

namespace
{
  constexpr int kDeliveryLocationKeyBase = 1000000000;
  constexpr int kPickupLocationKeyBase = 500000000;

  std::vector<LifelongAgentState> make_agents(
      const Graph& graph, int num_agents, std::mt19937& mt,
      const std::vector<int>& start_indexes)
  {
    auto agents = std::vector<LifelongAgentState>();
    for (int i = 0; i < num_agents; ++i) {
      Vertex* start = nullptr;
      if (!start_indexes.empty()) {
        if (static_cast<int>(start_indexes.size()) != num_agents) {
          throw std::runtime_error("start_indexes size must match num_agents");
        }
        const auto index = start_indexes[i];
        if (!graph.is_traversable(index)) {
          throw std::runtime_error("agent start index is not traversable");
        }
        start = graph.U[index];
      }
      auto agent = LifelongAgentState();
      agent.agent_id = i;
      agent.current_location = start;
      agent.current_target = start;
      agent.executed_path.push_back(start);
      agents.push_back(agent);
    }
    if (!start_indexes.empty()) return agents;

    auto vertices = graph.V;
    std::shuffle(vertices.begin(), vertices.end(), mt);
    if (static_cast<int>(vertices.size()) < num_agents) {
      throw std::runtime_error("not enough traversable starts for agents");
    }
    for (int i = 0; i < num_agents; ++i) {
      agents[i].current_location = vertices[i];
      agents[i].current_target = vertices[i];
      agents[i].executed_path[0] = vertices[i];
    }
    return agents;
  }

  std::vector<std::vector<int> > solution_to_indexes(const Solution& solution)
  {
    auto indexes = std::vector<std::vector<int> >();
    indexes.reserve(solution.size());
    for (const auto& config : solution) {
      auto row = std::vector<int>();
      row.reserve(config.size());
      for (auto v : config) row.push_back(v->index);
      indexes.push_back(row);
    }
    return indexes;
  }

  bool has_pending_task(const std::vector<LifelongTask>& tasks)
  {
    for (const auto& task : tasks) {
      if (task.status == LifelongTaskStatus::PENDING) return true;
    }
    return false;
  }

  bool has_idle_unloaded_agent(const std::vector<LifelongAgentState>& agents)
  {
    for (const auto& agent : agents) {
      if (!agent_is_loaded(agent) && !agent.assigned_task_id.has_value()) {
        return true;
      }
    }
    return false;
  }

  bool has_unfinished_work(const std::vector<LifelongAgentState>& agents,
                           const std::vector<LifelongTask>& tasks)
  {
    for (const auto& agent : agents) {
      if (agent_is_loaded(agent) || agent.assigned_task_id.has_value()) {
        return true;
      }
    }
    for (const auto& task : tasks) {
      if (task.status != LifelongTaskStatus::COMPLETED) return true;
    }
    return false;
  }

  bool agent_has_priority_target(const LifelongAgentState& agent,
                                 const std::vector<LifelongTask>& tasks)
  {
    if (agent.current_location == nullptr) return false;
    if (agent.assigned_task_id.has_value()) {
      const auto* task = find_task_by_id(tasks, *agent.assigned_task_id);
      if (task != nullptr && task->status == LifelongTaskStatus::ASSIGNED &&
          task->assigned_agent_id == agent.agent_id && task->start != nullptr) {
        return true;
      }
    }
    if (agent_is_loaded(agent)) {
      for (const auto task_id : agent.carried_task_ids) {
        const auto* task = find_task_by_id(tasks, task_id);
        if (task != nullptr && task->status == LifelongTaskStatus::PICKED &&
            task->picked_agent_id == agent.agent_id &&
            !task->goal_set.empty()) {
          return true;
        }
      }
    }
    return false;
  }

  bool agent_at_priority_target(const LifelongAgentState& agent,
                                const std::vector<LifelongTask>& tasks)
  {
    if (agent.current_location == nullptr) return false;
    if (agent.assigned_task_id.has_value()) {
      const auto* task = find_task_by_id(tasks, *agent.assigned_task_id);
      if (task != nullptr && task->status == LifelongTaskStatus::ASSIGNED &&
          task->assigned_agent_id == agent.agent_id &&
          task->start == agent.current_location) {
        return true;
      }
    }
    if (agent_is_loaded(agent)) {
      for (const auto task_id : agent.carried_task_ids) {
        const auto* task = find_task_by_id(tasks, task_id);
        if (task != nullptr && task->status == LifelongTaskStatus::PICKED &&
            task->picked_agent_id == agent.agent_id &&
            vertex_in_goal_set(agent.current_location, task->goal_set)) {
          return true;
        }
      }
    }
    return false;
  }

  void refresh_lifelong_priorities(
      std::vector<float>& priorities,
      const std::vector<LifelongAgentState>& agents,
      const std::vector<LifelongTask>& tasks, bool advance)
  {
    if (priorities.size() != agents.size()) {
      priorities.assign(agents.size(), 0.0f);
    }
    for (size_t i = 0; i < agents.size(); ++i) {
      if (!agent_has_priority_target(agents[i], tasks)) {
        priorities[i] = 0.0f;
        continue;
      }
      if (agent_at_priority_target(agents[i], tasks)) continue;
      if (advance) priorities[i] += 1.0f;
    }
  }

  void accumulate_agent_time(const std::vector<LifelongAgentState>& agents,
                             double& idle_time, double& loaded_time,
                             double& unloaded_time, double& carried_time,
                             double& loaded_distance_time,
                             int& max_carried_tasks,
                             int& max_loaded_distance_since_last_delivery)
  {
    for (const auto& agent : agents) {
      const auto carried = carried_task_count(agent);
      carried_time += carried;
      loaded_distance_time += agent.loaded_distance_since_last_delivery;
      max_carried_tasks = std::max(max_carried_tasks, carried);
      max_loaded_distance_since_last_delivery =
          std::max(max_loaded_distance_since_last_delivery,
                   agent.loaded_distance_since_last_delivery);
      if (agent_is_loaded(agent)) {
        loaded_time += 1;
      } else {
        unloaded_time += 1;
        if (!agent.assigned_task_id.has_value()) idle_time += 1;
      }
    }
  }

  bool check_motion_conflicts(const std::vector<Vertex*>& previous,
                              const std::vector<LifelongAgentState>& agents,
                              std::string* error)
  {
    auto occupied = std::unordered_set<int>();
    for (size_t i = 0; i < agents.size(); ++i) {
      const auto current = agents[i].current_location;
      if (current == nullptr) {
        if (error != nullptr) *error = "agent on null location";
        return false;
      }
      if (!occupied.insert(current->index).second) {
        if (error != nullptr) *error = "vertex conflict";
        return false;
      }
      for (size_t j = i + 1; j < agents.size(); ++j) {
        if (previous[i] == agents[j].current_location &&
            previous[j] == agents[i].current_location) {
          if (error != nullptr) *error = "edge swap conflict";
          return false;
        }
      }
    }
    return true;
  }

  int agent_task_phase(const LifelongAgentState& agent)
  {
    if (agent.assigned_task_id.has_value()) return 1;
    if (agent_is_loaded(agent)) return 2;
    return 0;
  }

  void append_agent_task_snapshot(LifelongSimulationMetrics& metrics,
                                  const std::vector<LifelongAgentState>& agents)
  {
    if (metrics.agent_task_ids_by_timestep.empty()) {
      metrics.agent_task_ids_by_timestep.resize(agents.size());
      metrics.agent_task_phases_by_timestep.resize(agents.size());
      metrics.agent_carried_task_ids_by_timestep.resize(agents.size());
      metrics.agent_assigned_task_ids_by_timestep.resize(agents.size());
    }
    for (size_t i = 0; i < agents.size(); ++i) {
      metrics.agent_task_ids_by_timestep[i].push_back(
          agents[i].current_task_id.value_or(-1));
      metrics.agent_task_phases_by_timestep[i].push_back(
          agent_task_phase(agents[i]));
      metrics.agent_carried_task_ids_by_timestep[i].push_back(
          agents[i].carried_task_ids);
      metrics.agent_assigned_task_ids_by_timestep[i].push_back(
          agents[i].assigned_task_id.value_or(-1));
    }
  }

  void overwrite_latest_agent_task_snapshot(
      LifelongSimulationMetrics& metrics,
      const std::vector<LifelongAgentState>& agents)
  {
    if (metrics.agent_task_ids_by_timestep.empty() ||
        metrics.agent_task_ids_by_timestep.front().empty()) {
      append_agent_task_snapshot(metrics, agents);
      return;
    }
    for (size_t i = 0; i < agents.size(); ++i) {
      metrics.agent_task_ids_by_timestep[i].back() =
          agents[i].current_task_id.value_or(-1);
      metrics.agent_task_phases_by_timestep[i].back() =
          agent_task_phase(agents[i]);
      metrics.agent_carried_task_ids_by_timestep[i].back() =
          agents[i].carried_task_ids;
      metrics.agent_assigned_task_ids_by_timestep[i].back() =
          agents[i].assigned_task_id.value_or(-1);
    }
  }

  void count_task_statuses(const std::vector<LifelongTask>& tasks, int& pending,
                           int& assigned, int& picked, int& completed)
  {
    pending = 0;
    assigned = 0;
    picked = 0;
    completed = 0;
    for (const auto& task : tasks) {
      switch (task.status) {
        case LifelongTaskStatus::PENDING:
          ++pending;
          break;
        case LifelongTaskStatus::ASSIGNED:
          ++assigned;
          break;
        case LifelongTaskStatus::PICKED:
          ++picked;
          break;
        case LifelongTaskStatus::COMPLETED:
          ++completed;
          break;
      }
    }
  }

  bool target_is_delivery_goal(const LifelongAgentState& agent,
                               const std::vector<LifelongTask>& tasks,
                               int target_index)
  {
    for (const auto task_id : agent.carried_task_ids) {
      const auto* task = find_task_by_id(tasks, task_id);
      if (task == nullptr || task->status != LifelongTaskStatus::PICKED) {
        continue;
      }
      for (auto goal : task->goal_set) {
        if (goal != nullptr && goal->index == target_index) return true;
      }
    }
    return false;
  }

  int task_id_from_assignment_key(int key)
  {
    return key >= 1000000 ? key / 1000000 - 1 : key;
  }

  bool apply_assignment_key_step(std::vector<LifelongAgentState>& agents,
                                 std::vector<LifelongTask>& tasks,
                                 const Graph& graph,
                                 const std::vector<int>& target_keys,
                                 const std::vector<int>& target_indexes)
  {
    if (target_keys.size() != agents.size() ||
        target_indexes.size() != agents.size()) {
      return false;
    }

    release_unpicked_assignments(agents, tasks);
    auto pickup_task_ids =
        std::vector<std::optional<int> >(agents.size(), std::nullopt);
    auto used_pickups = std::unordered_set<int>();
    for (size_t i = 0; i < agents.size(); ++i) {
      const auto target_index = target_indexes[i];
      if (target_index < 0 ||
          target_index >= static_cast<int>(graph.U.size())) {
        return false;
      }
      const auto key = target_keys[i];
      if (key < 0) continue;
      if (key >= kDeliveryLocationKeyBase) {
        const auto valid_delivery = std::any_of(
            agents[i].carried_task_ids.begin(),
            agents[i].carried_task_ids.end(), [&](const int carried_task_id) {
              const auto* carried = find_task_by_id(tasks, carried_task_id);
              return carried != nullptr &&
                     carried->status == LifelongTaskStatus::PICKED &&
                     vertex_in_goal_set(graph.U[target_index],
                                        carried->goal_set);
            });
        if (!valid_delivery) return false;
        continue;
      }
      if (key >= kPickupLocationKeyBase) return false;
      const auto task_id = task_id_from_assignment_key(key);
      auto* task = find_task_by_id(tasks, task_id);
      if (task == nullptr) return false;
      if (key < 1000000) {
        if (task->status != LifelongTaskStatus::PENDING ||
            task->start == nullptr || task->start->index != target_index ||
            !used_pickups.insert(task_id).second) {
          return false;
        }
        pickup_task_ids[i] = task_id;
        continue;
      }
      if (task->status != LifelongTaskStatus::PICKED ||
          std::find(agents[i].carried_task_ids.begin(),
                    agents[i].carried_task_ids.end(),
                    task_id) == agents[i].carried_task_ids.end() ||
          !vertex_in_goal_set(graph.U[target_index], task->goal_set)) {
        return false;
      }
    }

    for (size_t i = 0; i < agents.size(); ++i) {
      agents[i].current_target = graph.U[target_indexes[i]];
      agents[i].assigned_task_id.reset();
      if (pickup_task_ids[i].has_value()) {
        auto* task = find_task_by_id(tasks, *pickup_task_ids[i]);
        if (task == nullptr) return false;
        task->status = LifelongTaskStatus::ASSIGNED;
        task->assigned_agent_id = agents[i].agent_id;
        agents[i].assigned_task_id = task->task_id;
        agents[i].current_target = task->start;
      }
      sync_agent_load_state(agents[i]);
    }
    return true;
  }

  enum class LifelongTraceTargetType {
    NONE = 0,
    PICKUP = 1,
    DELIVERY = 2,
    WAIT = 3,
    OTHER = 4,
  };

  LifelongTraceTargetType classify_trace_target(
      const LifelongAgentState& agent, const std::vector<LifelongTask>& tasks,
      const LifelongPlanningSnapshot& snapshot, size_t agent_index,
      int target_index)
  {
    if (target_index < 0) return LifelongTraceTargetType::NONE;
    if (snapshot.pending_task_id_by_start_index_by_agent[agent_index].count(
            target_index)) {
      return LifelongTraceTargetType::PICKUP;
    }
    if (target_is_delivery_goal(agent, tasks, target_index)) {
      return LifelongTraceTargetType::DELIVERY;
    }
    if (agent.current_location != nullptr &&
        agent.current_location->index == target_index) {
      return LifelongTraceTargetType::WAIT;
    }
    return LifelongTraceTargetType::OTHER;
  }

  LifelongTraceTargetType classify_trace_assignment_target(
      const LifelongAgentState& agent, int target_key, int target_index)
  {
    if (target_key < 0) return LifelongTraceTargetType::WAIT;
    if (target_key < kDeliveryLocationKeyBase) {
      return LifelongTraceTargetType::PICKUP;
    }
    if (agent_is_loaded(agent)) return LifelongTraceTargetType::DELIVERY;
    return LifelongTraceTargetType::OTHER;
  }

  int encode_trace_target_type(LifelongTraceTargetType type)
  {
    return static_cast<int>(type);
  }

  std::vector<std::unordered_map<int, int> > partial_pickup_preferences(
      const std::vector<LifelongAgentState>& agents,
      const std::vector<LifelongTask>& tasks,
      const std::vector<bool>& service_active,
      const std::vector<int>& service_keys,
      const std::vector<int>& service_target_indexes,
      const std::vector<int>& service_progress, int pickup_service_duration)
  {
    auto preferred =
        std::vector<std::unordered_map<int, int> >(agents.size());
    for (size_t i = 0; i < agents.size(); ++i) {
      if (i >= service_active.size() || i >= service_keys.size() ||
          i >= service_target_indexes.size() || i >= service_progress.size() ||
          !service_active[i]) {
        continue;
      }
      const auto task_id = service_keys[i];
      const auto target_index = service_target_indexes[i];
      const auto remaining =
          std::max(0, pickup_service_duration - std::max(0, service_progress[i]));
      if (task_id < 0 || task_id >= kPickupLocationKeyBase ||
          target_index < 0 || remaining <= 0 ||
          agents[i].current_location == nullptr ||
          agents[i].current_location->index != target_index) {
        continue;
      }
      const auto* task = find_task_by_id(tasks, task_id);
      if (task == nullptr || task->start == nullptr ||
          task->start->index != target_index) {
        continue;
      }
      const auto valid_pending = task->status == LifelongTaskStatus::PENDING;
      const auto valid_assigned =
          task->status == LifelongTaskStatus::ASSIGNED &&
          task->assigned_agent_id.has_value() &&
          *task->assigned_agent_id == agents[i].agent_id;
      if (valid_pending || valid_assigned) preferred[i][target_index] = task_id;
    }
    return preferred;
  }

  void initialize_optional_partial_services(
      const TAPFInstance& ins, const LifelongPlanningSnapshot& snapshot,
      const std::vector<LifelongAgentState>& agents,
      const std::vector<bool>& service_active,
      const std::vector<int>& service_keys,
      const std::vector<int>& service_target_indexes,
      const std::vector<int>& service_progress, int pickup_service_duration,
      int delivery_service_duration, TAPFSearchConfig& search_config)
  {
    search_config.initial_optional_service_assignments.assign(agents.size(), -1);
    search_config.initial_optional_service_remaining.assign(agents.size(), 0);
    auto used_tasks = std::vector<bool>(ins.tasks.size(), false);
    for (size_t i = 0; i < agents.size(); ++i) {
      if (i >= service_active.size() || i >= service_keys.size() ||
          i >= service_target_indexes.size() || i >= service_progress.size() ||
          !service_active[i] || agents[i].current_location == nullptr) {
        continue;
      }
      const auto key = service_keys[i];
      const auto target_index = service_target_indexes[i];
      if (target_index < 0 ||
          agents[i].current_location->index != target_index) {
        continue;
      }
      const auto is_delivery = key >= kDeliveryLocationKeyBase;
      const auto is_pickup = key >= 0 && key < kPickupLocationKeyBase;
      if (!is_delivery && !is_pickup) continue;
      const auto duration =
          is_delivery ? delivery_service_duration : pickup_service_duration;
      const auto remaining =
          std::max(0, duration - std::max(0, service_progress[i]));
      if (remaining <= 0) continue;

      auto task_column = -1;
      if (is_pickup) {
        if (i >= snapshot.pending_task_id_by_start_index_by_agent.size()) {
          continue;
        }
        const auto& pending =
            snapshot.pending_task_id_by_start_index_by_agent[i];
        const auto iter = pending.find(target_index);
        if (iter == pending.end() || iter->second != key) continue;
        for (size_t j = 0; j < ins.tasks.size(); ++j) {
          const auto task_key =
              j < ins.task_keys.size() ? ins.task_keys[j] : -1;
          if (!used_tasks[j] && ins.allowed[i][j] &&
              ins.tasks[j]->index == target_index &&
              task_key >= kPickupLocationKeyBase &&
              task_key < kDeliveryLocationKeyBase) {
            task_column = static_cast<int>(j);
            break;
          }
        }
      } else {
        const auto carried_task_id = key - kDeliveryLocationKeyBase;
        if (std::find(agents[i].carried_task_ids.begin(),
                      agents[i].carried_task_ids.end(),
                      carried_task_id) == agents[i].carried_task_ids.end()) {
          continue;
        }
        for (size_t j = 0; j < ins.tasks.size(); ++j) {
          const auto task_key =
              j < ins.task_keys.size() ? ins.task_keys[j] : -1;
          if (!used_tasks[j] && ins.allowed[i][j] &&
              ins.tasks[j]->index == target_index &&
              task_key >= kDeliveryLocationKeyBase) {
            task_column = static_cast<int>(j);
            break;
          }
        }
      }
      if (task_column < 0) continue;
      used_tasks[task_column] = true;
      search_config.initial_optional_service_assignments[i] = task_column;
      search_config.initial_optional_service_remaining[i] = remaining;
    }
  }

  void accumulate_trace_assignment_type(LifelongPlannerTraceRecord& trace,
                                        const LifelongAgentState& agent,
                                        LifelongTraceTargetType type)
  {
    if (agent_is_loaded(agent)) {
      if (type == LifelongTraceTargetType::PICKUP) {
        ++trace.root_loaded_pickup_assignments;
      } else if (type == LifelongTraceTargetType::DELIVERY) {
        ++trace.root_loaded_delivery_assignments;
      } else if (type == LifelongTraceTargetType::WAIT) {
        ++trace.root_loaded_wait_assignments;
      }
    } else {
      if (type == LifelongTraceTargetType::PICKUP) {
        ++trace.root_unloaded_pickup_assignments;
      } else if (type == LifelongTraceTargetType::WAIT) {
        ++trace.root_unloaded_wait_assignments;
      }
    }
  }

  void accumulate_final_trace_assignment_type(LifelongPlannerTraceRecord& trace,
                                              const LifelongAgentState& agent,
                                              LifelongTraceTargetType type)
  {
    if (agent_is_loaded(agent)) {
      if (type == LifelongTraceTargetType::PICKUP) {
        ++trace.final_loaded_pickup_assignments;
      } else if (type == LifelongTraceTargetType::DELIVERY) {
        ++trace.final_loaded_delivery_assignments;
      } else if (type == LifelongTraceTargetType::WAIT) {
        ++trace.final_loaded_wait_assignments;
      }
    } else {
      if (type == LifelongTraceTargetType::PICKUP) {
        ++trace.final_unloaded_pickup_assignments;
      } else if (type == LifelongTraceTargetType::WAIT) {
        ++trace.final_unloaded_wait_assignments;
      }
    }
  }

  void finalize_metrics(LifelongSimulationMetrics& metrics,
                        const std::vector<LifelongAgentState>& agents,
                        const std::vector<LifelongTask>& tasks,
                        double total_planner_runtime, double idle_time,
                        double loaded_time, double unloaded_time,
                        double carried_time, double loaded_distance_time,
                        int max_carried_tasks,
                        int max_loaded_distance_since_last_delivery,
                        int delivery_carried_sum)
  {
    auto completion_sum = 0.0;
    auto pickup_sum = 0.0;
    auto delivery_sum = 0.0;
    auto pickup_count = 0;
    auto delivery_count = 0;

    for (const auto& task : tasks) {
      auto record = LifelongTaskVisualizationRecord();
      record.task_id = task.task_id;
      record.task_type = task.task_type;
      record.start_index = task.start == nullptr ? -1 : task.start->index;
      record.goal_indexes.reserve(task.goal_set.size());
      for (auto goal : task.goal_set)
        record.goal_indexes.push_back(goal->index);
      record.release_timestep = task.release_timestep;
      record.pickup_timestep = task.pickup_timestep.value_or(-1);
      record.completion_timestep = task.completion_timestep.value_or(-1);
      metrics.task_records.push_back(std::move(record));

      switch (task.status) {
        case LifelongTaskStatus::PENDING:
          ++metrics.final_pending_tasks;
          break;
        case LifelongTaskStatus::ASSIGNED:
          ++metrics.final_assigned_tasks;
          break;
        case LifelongTaskStatus::PICKED:
          ++metrics.final_picked_tasks;
          break;
        case LifelongTaskStatus::COMPLETED:
          ++metrics.completed_tasks;
          break;
      }
      if (task.pickup_timestep.has_value()) {
        pickup_sum += *task.pickup_timestep - task.release_timestep;
        ++pickup_count;
      }
      if (task.completion_timestep.has_value()) {
        completion_sum += *task.completion_timestep - task.release_timestep;
        if (task.pickup_timestep.has_value()) {
          delivery_sum += *task.completion_timestep - *task.pickup_timestep;
          ++delivery_count;
        }
      }
    }

    metrics.throughput =
        metrics.horizon > 0
            ? static_cast<double>(metrics.completed_tasks) / metrics.horizon
            : 0;
    for (const auto& agent : agents) {
      metrics.alternating_completed_tasks +=
          agent.alternating_completed_task_count;
    }
    metrics.alternating_throughput =
        metrics.horizon > 0
            ? static_cast<double>(metrics.alternating_completed_tasks) /
                  metrics.horizon
            : 0;
    metrics.average_task_completion_time =
        metrics.completed_tasks > 0 ? completion_sum / metrics.completed_tasks
                                    : 0;
    metrics.average_pickup_time =
        pickup_count > 0 ? pickup_sum / pickup_count : 0;
    metrics.average_delivery_time =
        delivery_count > 0 ? delivery_sum / delivery_count : 0;
    metrics.average_planner_runtime =
        metrics.planner_invocations > 0
            ? total_planner_runtime / metrics.planner_invocations
            : 0;
    metrics.total_planner_runtime = total_planner_runtime;
    metrics.total_planner_search_runtime =
        std::max(0.0, total_planner_runtime - metrics.total_assignment_runtime);
    const auto denom =
        static_cast<double>(std::max(1, metrics.horizon * metrics.num_agents));
    metrics.average_agent_idle_time = idle_time / denom;
    metrics.average_agent_loaded_time = loaded_time / denom;
    metrics.average_agent_unloaded_time = unloaded_time / denom;
    metrics.average_carried_tasks = carried_time / denom;
    metrics.max_carried_tasks = max_carried_tasks;
    metrics.average_loaded_distance_since_last_delivery =
        loaded_distance_time / denom;
    metrics.max_loaded_distance_since_last_delivery =
        max_loaded_distance_since_last_delivery;
    metrics.average_tasks_carried_at_delivery =
        metrics.delivery_events > 0
            ? static_cast<double>(delivery_carried_sum) /
                  metrics.delivery_events
            : 0;
    metrics.assignment_row_cache_hit_rate =
        metrics.assignment_row_cache_requests > 0
            ? static_cast<double>(metrics.assignment_row_cache_hits) /
                  metrics.assignment_row_cache_requests
            : 0;
  }
}  // namespace

LifelongSimulationMetrics run_lifelong_simulation(
    const LifelongSimulationConfig& config)
{
  auto metrics = LifelongSimulationMetrics();
  metrics.map_name =
      std::filesystem::path(config.map_filename).filename().string();
  metrics.num_agents = config.num_agents;
  metrics.horizon = config.horizon;
  metrics.seed = config.seed;
  metrics.multi_carry_capacity = config.multi_carry_capacity;
  metrics.max_shared_drop_goal_agents = config.max_shared_drop_goal_agents;
  metrics.assignment_cost_mode = config.assignment_cost_mode;
  metrics.planner_force_full_assignment = config.planner_force_full_assignment;

  const auto sim_start = Time::now();
  try {
    if (config.multi_carry_capacity <= 0) {
      throw std::invalid_argument("multi_carry_capacity must be positive");
    }
    if (config.max_shared_drop_goal_agents <= 0) {
      throw std::invalid_argument(
          "max_shared_drop_goal_agents must be positive");
    }
    if (config.assignment_cost_mode != LIFELONG_ASSIGNMENT_COST_BASELINE &&
        config.assignment_cost_mode !=
            LIFELONG_ASSIGNMENT_COST_MILD_PICKUP_DELAY) {
      throw std::invalid_argument("unsupported assignment_cost_mode");
    }
    auto graph = Graph(config.map_filename);
    metrics.map_width = graph.width;
    metrics.map_height = graph.height;
    auto mt = std::mt19937(config.seed);
    auto agents =
        make_agents(graph, config.num_agents, mt, config.start_indexes);
    auto tasks = std::vector<LifelongTask>();
    auto generator =
        LifelongTaskGenerator(&graph, config.task_config, config.seed);
    const auto cache_path =
        config.cache_filename.empty()
            ? std::filesystem::temp_directory_path() / "lacam_lifelong_dist.bin"
            : std::filesystem::path(config.cache_filename);
    const auto distances =
        load_or_build_map_distance_cache(config.map_filename, cache_path);

    auto plan = std::vector<std::vector<int> >();
    auto plan_assignment_keys = std::vector<std::vector<int> >();
    auto plan_assignment_target_indexes = std::vector<std::vector<int> >();
    auto plan_step = size_t(0);
    auto valid_plan = false;
    auto previous_planner_failed = false;
    auto total_planner_runtime = 0.0;
    auto idle_time = 0.0;
    auto loaded_time = 0.0;
    auto unloaded_time = 0.0;
    auto carried_time = 0.0;
    auto loaded_distance_time = 0.0;
    auto max_carried_tasks = 0;
    auto max_loaded_distance_since_last_delivery = 0;
    auto delivery_carried_sum = 0;
    auto service_active = std::vector<bool>(agents.size(), false);
    auto service_keys = std::vector<int>(agents.size(), -1);
    auto service_target_indexes = std::vector<int>(agents.size(), -1);
    auto service_progress = std::vector<int>(agents.size(), 0);
    auto inherited_priorities = std::vector<float>(agents.size(), 0.0f);
    auto pending_replan_from_arrival = false;
    append_agent_task_snapshot(metrics, agents);

    for (int t = 0; t < config.horizon; ++t) {
      auto released =
          generator.generate_for_timestep(t, config.num_agents, tasks);
      metrics.generated_tasks += static_cast<int>(released.size());
      tasks.insert(tasks.end(), released.begin(), released.end());

      auto process_arrivals = [&](int event_timestep,
                                  const std::vector<bool>* ready_agents) {
        auto changed = false;
        for (size_t i = 0; i < agents.size(); ++i) {
          if (ready_agents != nullptr &&
              (i >= ready_agents->size() || !(*ready_agents)[i])) {
            continue;
          }
          auto& agent = agents[i];
          const auto carried_before_pickup = carried_task_count(agent);
          auto pickup = try_pickup(agent, tasks, event_timestep,
                                   config.multi_carry_capacity);
          if (pickup.changed && carried_before_pickup > 0) {
            ++metrics.pickup_while_loaded_count;
          }
          const auto carried_before_delivery = carried_task_count(agent);
          auto completion = try_complete(agent, tasks, event_timestep);
          if (completion.changed) {
            ++metrics.delivery_events;
            delivery_carried_sum += carried_before_delivery;
          }
          if (pickup.changed || completion.changed) {
            service_active[i] = false;
            service_keys[i] = -1;
            service_target_indexes[i] = -1;
            service_progress[i] = 0;
            if (i < inherited_priorities.size()) {
              inherited_priorities[i] = 0.0f;
            }
          }
          changed = changed || pickup.changed || completion.changed;
        }
        return changed;
      };
      auto service_ready_agents = [&](const std::vector<Vertex*>& previous) {
        auto ready = std::vector<bool>(agents.size(), false);
        for (size_t i = 0; i < agents.size(); ++i) {
          if (agents[i].current_location == nullptr) {
            service_active[i] = false;
            service_keys[i] = -1;
            service_target_indexes[i] = -1;
            service_progress[i] = 0;
            continue;
          }
          auto key = -1;
          auto duration = 0;
          if (agents[i].assigned_task_id.has_value() &&
              carried_task_count(agents[i]) < config.multi_carry_capacity) {
            const auto* task =
                find_task_by_id(tasks, *agents[i].assigned_task_id);
            if (task != nullptr &&
                task->status == LifelongTaskStatus::ASSIGNED &&
                task->assigned_agent_id == agents[i].agent_id &&
                task->start == agents[i].current_location) {
              key = task->task_id;
              duration = config.pickup_service_duration;
            }
          }
          if (key < 0 && agent_is_loaded(agents[i])) {
            for (const auto task_id : agents[i].carried_task_ids) {
              const auto* task = find_task_by_id(tasks, task_id);
              if (task != nullptr &&
                  task->status == LifelongTaskStatus::PICKED &&
                  task->picked_agent_id == agents[i].agent_id &&
                  vertex_in_goal_set(agents[i].current_location,
                                     task->goal_set)) {
                key = kDeliveryLocationKeyBase + task->task_id;
                duration = config.delivery_service_duration;
                break;
              }
            }
          }
          const auto target_index = agents[i].current_location->index;
          if (key < 0) {
            service_active[i] = false;
            service_keys[i] = -1;
            service_target_indexes[i] = -1;
            service_progress[i] = 0;
            continue;
          }
          const auto same_service = service_active[i] &&
                                    service_keys[i] == key &&
                                    service_target_indexes[i] == target_index;
          if (!same_service) {
            service_active[i] = true;
            service_keys[i] = key;
            service_target_indexes[i] = target_index;
            service_progress[i] = 0;
          } else if (i < previous.size() && previous[i] != nullptr &&
                     previous[i]->index == target_index) {
            service_progress[i] = std::max(0, service_progress[i]) + 1;
          }
          ready[i] = service_progress[i] >= std::max(0, duration);
        }
        return ready;
      };
      const auto plan_finished = !valid_plan || plan_step + 1 >= plan.size();
      auto ready_at_t = service_ready_agents(std::vector<Vertex*>());
      const auto arrival_event_at_t = process_arrivals(t, &ready_at_t);
      auto event_happened =
          pending_replan_from_arrival || arrival_event_at_t;
      pending_replan_from_arrival = false;
      refresh_lifelong_priorities(inherited_priorities, agents, tasks, false);
      const auto should_replan =
          t == 0 || event_happened ||
          (plan_finished && has_unfinished_work(agents, tasks)) ||
          (previous_planner_failed && !valid_plan) ||
          (!valid_plan && has_idle_unloaded_agent(agents) &&
           has_pending_task(tasks));

      if (should_replan) {
        ++metrics.planner_invocations;
        const auto planning_start = Time::now();
        const auto preferred_partial_pickups = partial_pickup_preferences(
            agents, tasks, service_active, service_keys, service_target_indexes,
            service_progress, config.pickup_service_duration);
        auto snapshot = prepare_lifelong_planning_snapshot(
            agents, tasks, distances, config.multi_carry_capacity,
            config.max_shared_drop_goal_agents, config.pickup_service_duration,
            config.delivery_service_duration, inherited_priorities,
            preferred_partial_pickups, config.assignment_cost_mode);
        auto solution = Solution();
        auto final_assignment = std::vector<int>();
        auto assignment_schedule = std::vector<std::vector<int> >();
        auto next_plan = std::vector<std::vector<int> >();
        auto next_plan_assignment_keys = std::vector<std::vector<int> >();
        auto next_plan_assignment_target_indexes =
            std::vector<std::vector<int> >();
        auto stats = TAPFStats();
        auto trace = LifelongPlannerTraceRecord();
        trace.timestep = t;
        trace.should_replan = true;
        trace.event_happened = event_happened;
        trace.plan_finished = plan_finished;
        trace.previous_planner_failed = previous_planner_failed;
        trace.idle_unloaded_with_pending =
            has_idle_unloaded_agent(agents) && has_pending_task(tasks);
        trace.snapshot_feasible = snapshot.feasible;
        count_task_statuses(tasks, trace.pending_tasks, trace.assigned_tasks,
                            trace.picked_tasks, trace.completed_tasks);
        auto target_agent_counts = std::unordered_map<int, int>();
        if (snapshot.feasible) {
          auto ins = build_lifelong_tapf_instance(config.map_filename, agents,
                                                  snapshot);
          trace.instance_valid = ins.is_valid();
          if (ins.is_valid()) {
            auto loaded_count = 0;
            auto assigned_unloaded_count = 0;
            auto idle_count = 0;
            auto singleton_count = 0;
            auto multi_goal_count = 0;
            auto unique_targets = std::unordered_set<int>();
            auto total_carried = 0;
            auto max_carried_now = 0;
            auto loaded_distance_sum = 0.0;
            auto loaded_distance_count = 0;
            auto max_loaded_distance_now = 0;
            auto max_loaded_distance_agent_index = -1;
            auto priority_sum = 0.0;
            auto max_priority_offset_now = 0.0f;
            auto max_priority_offset_agent_id = -1;
            for (size_t i = 0; i < agents.size(); ++i) {
              const auto priority_offset = i < inherited_priorities.size()
                                               ? inherited_priorities[i]
                                               : 0.0f;
              priority_sum += priority_offset;
              if (priority_offset > max_priority_offset_now) {
                max_priority_offset_now = priority_offset;
                max_priority_offset_agent_id = agents[i].agent_id;
              }
              const auto carried = carried_task_count(agents[i]);
              total_carried += carried;
              max_carried_now = std::max(max_carried_now, carried);
              if (carried >= 2) ++trace.agents_carrying_two_tasks;
              if (agent_is_loaded(agents[i])) {
                ++loaded_count;
                loaded_distance_sum +=
                    agents[i].loaded_distance_since_last_delivery;
                ++loaded_distance_count;
                if (agents[i].loaded_distance_since_last_delivery >
                    max_loaded_distance_now) {
                  max_loaded_distance_now =
                      agents[i].loaded_distance_since_last_delivery;
                  max_loaded_distance_agent_index = static_cast<int>(i);
                  trace.max_loaded_distance_agent_id = agents[i].agent_id;
                  trace.max_loaded_distance_agent_current_index =
                      agents[i].current_location == nullptr
                          ? -1
                          : agents[i].current_location->index;
                  trace.max_loaded_distance_agent_carried_tasks = carried;
                }
              } else if (agents[i].assigned_task_id.has_value()) {
                ++assigned_unloaded_count;
              } else {
                ++idle_count;
              }
              const auto goal_options =
                  static_cast<int>(snapshot.goal_indexes_by_agent[i].size());
              auto pickup_options = 0;
              auto delivery_options = 0;
              auto wait_options = 0;
              trace.total_goal_options += goal_options;
              trace.max_goal_options_per_agent =
                  std::max(trace.max_goal_options_per_agent, goal_options);
              if (snapshot.goal_indexes_by_agent[i].size() == 1) {
                ++singleton_count;
              } else {
                ++multi_goal_count;
              }
              for (const auto target : snapshot.goal_indexes_by_agent[i]) {
                unique_targets.insert(target);
                ++target_agent_counts[target];
                const auto type = classify_trace_target(agents[i], tasks,
                                                        snapshot, i, target);
                if (type == LifelongTraceTargetType::PICKUP) {
                  ++pickup_options;
                } else if (type == LifelongTraceTargetType::DELIVERY) {
                  ++delivery_options;
                } else if (type == LifelongTraceTargetType::WAIT) {
                  ++wait_options;
                }
              }
              if (agent_is_loaded(agents[i])) {
                if (pickup_options > 0)
                  ++trace.loaded_agents_with_pickup_options;
                if (delivery_options > 0) {
                  ++trace.loaded_agents_with_delivery_options;
                }
                if (pickup_options > 0 && delivery_options > 0) {
                  ++trace.loaded_agents_with_both_pickup_delivery_options;
                }
                trace.loaded_pickup_goal_options += pickup_options;
                trace.loaded_delivery_goal_options += delivery_options;
                trace.loaded_wait_goal_options += wait_options;
                trace.max_loaded_pickup_options_per_agent = std::max(
                    trace.max_loaded_pickup_options_per_agent, pickup_options);
                trace.max_loaded_delivery_options_per_agent =
                    std::max(trace.max_loaded_delivery_options_per_agent,
                             delivery_options);
                if (static_cast<int>(i) == max_loaded_distance_agent_index) {
                  trace.max_loaded_distance_agent_pickup_options =
                      pickup_options;
                  trace.max_loaded_distance_agent_delivery_options =
                      delivery_options;
                  trace.max_loaded_distance_agent_wait_options = wait_options;
                }
              }
            }
            trace.loaded_agents = loaded_count;
            trace.assigned_unloaded_agents = assigned_unloaded_count;
            trace.idle_agents = idle_count;
            trace.singleton_agents = singleton_count;
            trace.multi_goal_agents = multi_goal_count;
            trace.unique_target_count = unique_targets.size();
            trace.average_carried_tasks_now =
                agents.empty()
                    ? 0
                    : static_cast<double>(total_carried) / agents.size();
            trace.max_carried_tasks_now = max_carried_now;
            trace.average_loaded_distance_now =
                loaded_distance_count > 0
                    ? loaded_distance_sum / loaded_distance_count
                    : 0;
            trace.max_loaded_distance_now = max_loaded_distance_now;
            trace.average_priority_offset_now =
                agents.empty() ? 0 : priority_sum / agents.size();
            trace.max_priority_offset_now = max_priority_offset_now;
            trace.max_priority_offset_agent_id = max_priority_offset_agent_id;
            for (const auto& [target, count] : target_agent_counts) {
              trace.max_agents_per_target =
                  std::max(trace.max_agents_per_target, count);
              if (count > 1) trace.duplicate_target_slots += count - 1;
            }
            auto deadline = Deadline(config.planner_time_limit_sec * 1000);
            auto planner_mt = std::mt19937(config.seed + t);
            auto search_config = TAPFSearchConfig();
            search_config.service_goal_mode = true;
            search_config.service_commit_agents =
                config.service_commit_agents > 0
                    ? std::min(config.num_agents, config.service_commit_agents)
                    : 0;
            search_config.pickup_service_duration =
                config.pickup_service_duration;
            search_config.delivery_service_duration =
                config.delivery_service_duration;
            initialize_optional_partial_services(
                ins, snapshot, agents, service_active, service_keys,
                service_target_indexes, service_progress,
                config.pickup_service_duration,
                config.delivery_service_duration, search_config);
            solution = solve_tapf(
                ins, 0, &deadline, &planner_mt, 0, &stats,
                config.planner_anytime, config.planner_force_full_assignment,
                search_config, &final_assignment, &assignment_schedule);
            trace.root_initial_assignment_cost = stats.initial_assignment_cost;
            for (size_t i = 0; i < stats.initial_assignment.size(); ++i) {
              const auto assignment = stats.initial_assignment[i];
              if (assignment < 0 ||
                  assignment >= static_cast<int>(ins.tasks.size())) {
                continue;
              }
              const auto target_key =
                  assignment < static_cast<int>(ins.task_keys.size())
                      ? ins.task_keys[assignment]
                      : ins.tasks[assignment]->index;
              const auto type = classify_trace_assignment_target(
                  agents[i], target_key, ins.tasks[assignment]->index);
              accumulate_trace_assignment_type(trace, agents[i], type);
              if (static_cast<int>(i) == max_loaded_distance_agent_index) {
                trace.max_loaded_distance_agent_root_target_type =
                    encode_trace_target_type(type);
                trace.max_loaded_distance_agent_root_target_index =
                    ins.tasks[assignment]->index;
              }
            }
            metrics.assignment_row_cache_requests +=
                stats.assignment_row_cache_requests;
            metrics.assignment_row_cache_hits +=
                stats.assignment_row_cache_hits;
            metrics.total_assignment_runtime += stats.assignment_time_ms;
            if (!solution.empty()) {
              for (size_t i = 0; i < final_assignment.size(); ++i) {
                const auto assignment = final_assignment[i];
                if (assignment < 0 ||
                    assignment >= static_cast<int>(ins.tasks.size())) {
                  continue;
                }
                const auto target_key =
                    assignment < static_cast<int>(ins.task_keys.size())
                        ? ins.task_keys[assignment]
                        : ins.tasks[assignment]->index;
                const auto type = classify_trace_assignment_target(
                    agents[i], target_key, ins.tasks[assignment]->index);
                accumulate_final_trace_assignment_type(trace, agents[i], type);
                if (static_cast<int>(i) == max_loaded_distance_agent_index) {
                  trace.max_loaded_distance_agent_final_target_type =
                      encode_trace_target_type(type);
                  trace.max_loaded_distance_agent_final_target_index =
                      ins.tasks[assignment]->index;
                }
              }
              const auto schedule_valid =
                  assignment_schedule.size() == solution.size() &&
                  !assignment_schedule.empty();
              if (schedule_valid) {
                next_plan_assignment_keys.reserve(assignment_schedule.size());
                next_plan_assignment_target_indexes.reserve(
                    assignment_schedule.size());
                for (const auto& step_assignment : assignment_schedule) {
                  auto keys = std::vector<int>();
                  auto indexes = std::vector<int>();
                  keys.reserve(step_assignment.size());
                  indexes.reserve(step_assignment.size());
                  for (size_t i = 0; i < step_assignment.size(); ++i) {
                    const auto assignment = step_assignment[i];
                    if (assignment < 0 ||
                        assignment >= static_cast<int>(ins.tasks.size())) {
                      keys.clear();
                      indexes.clear();
                      break;
                    }
                    auto key = ins.task_keys[assignment];
                    const auto index = ins.tasks[assignment]->index;
                    if (key >= kPickupLocationKeyBase &&
                        key < kDeliveryLocationKeyBase) {
                      const auto& pending_by_start =
                          snapshot.pending_task_id_by_start_index_by_agent[i];
                      const auto pending = pending_by_start.find(index);
                      if (pending == pending_by_start.end()) {
                        keys.clear();
                        indexes.clear();
                        break;
                      }
                      key = pending->second;
                    }
                    keys.push_back(key);
                    indexes.push_back(index);
                  }
                  if (keys.size() != agents.size()) break;
                  next_plan_assignment_keys.push_back(std::move(keys));
                  next_plan_assignment_target_indexes.push_back(
                      std::move(indexes));
                }
              }
              const auto translated =
                  next_plan_assignment_keys.size() == solution.size();
              if (translated && !assignment_schedule.empty() &&
                  apply_lifelong_solution_assignment(
                      agents, tasks, snapshot, ins,
                      assignment_schedule.front())) {
                next_plan = solution_to_indexes(solution);
              } else {
                solution.clear();
              }
            }
            if (solution.empty() && !stats.timed_out) {
              ++metrics.planner_empty_solution_count;
              if (metrics.first_empty_loaded_agents < 0) {
                metrics.first_empty_loaded_agents = loaded_count;
                metrics.first_empty_assigned_unloaded_agents =
                    assigned_unloaded_count;
                metrics.first_empty_idle_agents = idle_count;
                metrics.first_empty_unique_target_count = unique_targets.size();
                metrics.first_empty_singleton_agents = singleton_count;
                metrics.first_empty_multi_goal_agents = multi_goal_count;
              }
            }
          } else {
            ++metrics.planner_invalid_instance_count;
          }
        } else {
          ++metrics.planner_snapshot_infeasible_count;
        }
        const auto planning_runtime =
            std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() -
                                                                 planning_start)
                .count() /
            1000000.0;
        trace.solution_found = !solution.empty();
        trace.timed_out = stats.timed_out;
        trace.planning_runtime_ms = planning_runtime;
        trace.assignment_time_ms = stats.assignment_time_ms;
        trace.planner_search_time_ms =
            std::max(0.0, planning_runtime - stats.assignment_time_ms);
        trace.hl_loop_iterations = stats.hl_loop_iterations;
        trace.hl_nodes_created = stats.hl_nodes_created;
        trace.hl_nodes_explored = stats.hl_nodes_explored;
        trace.hl_reinsertions = stats.hl_reinsertions;
        trace.hl_duplicate_configs = stats.hl_duplicate_configs;
        trace.open_max_size = stats.open_max_size;
        trace.constraints_popped = stats.constraints_popped;
        trace.constraints_generated = stats.constraints_generated;
        trace.constraint_failures = stats.constraint_failures;
        trace.pibt_calls = stats.pibt_calls;
        trace.pibt_failures = stats.pibt_failures;
        trace.pibt_recursions = stats.pibt_recursions;
        trace.assignment_calls = stats.assignment_calls;
        trace.assignment_changes = stats.assignment_changes;
        trace.assignment_infeasible_count = stats.assignment_infeasible_count;
        trace.service_child_validation_failures =
            stats.service_child_validation_failures;
        trace.service_child_stack_validation_failures =
            stats.service_child_stack_validation_failures;
        trace.service_child_swap_validation_failures =
            stats.service_child_swap_validation_failures;
        trace.final_assignment_changes = stats.final_assignment_changes;
        trace.final_agent_assignment_changes =
            stats.final_agent_assignment_changes;
        trace.assignment_row_cache_requests =
            stats.assignment_row_cache_requests;
        trace.assignment_row_cache_hits = stats.assignment_row_cache_hits;
        trace.solution_depth = stats.solution_depth;
        trace.solution_cost = stats.solution_cost;
        trace.solution_h = stats.solution_h;
        trace.service_satisfied_agents = stats.service_satisfied_agents;
        trace.service_satisfied_pickups = stats.service_satisfied_pickups;
        trace.service_satisfied_deliveries = stats.service_satisfied_deliveries;
        trace.service_best_satisfied_agents =
            stats.service_best_satisfied_agents;
        metrics.planner_trace_records.push_back(trace);
        total_planner_runtime += planning_runtime;
        metrics.max_planner_runtime =
            std::max(metrics.max_planner_runtime, planning_runtime);

        if (!solution.empty()) {
          plan = next_plan;
          plan_assignment_keys = next_plan_assignment_keys;
          plan_assignment_target_indexes = next_plan_assignment_target_indexes;
          plan_step = 0;
          valid_plan = true;
          previous_planner_failed = false;
          ++metrics.planner_success_count;
        } else {
          valid_plan = false;
          previous_planner_failed = true;
          if (stats.timed_out) {
            ++metrics.planner_timeout_count;
          } else {
            ++metrics.planner_failure_count;
          }
        }
      }
      overwrite_latest_agent_task_snapshot(metrics, agents);

      auto previous = std::vector<Vertex*>();
      previous.reserve(agents.size());
      for (const auto& agent : agents)
        previous.push_back(agent.current_location);

      if (valid_plan && plan_step + 1 < plan.size()) {
        ++plan_step;
        for (size_t i = 0; i < agents.size(); ++i) {
          agents[i].current_location = graph.U[plan[plan_step][i]];
          record_loaded_movement(agents[i], previous[i]);
          agents[i].executed_path.push_back(agents[i].current_location);
        }
      } else {
        for (auto& agent : agents)
          agent.executed_path.push_back(agent.current_location);
      }

      auto ready_agents = service_ready_agents(previous);
      pending_replan_from_arrival =
          process_arrivals(t + 1, &ready_agents) ||
          pending_replan_from_arrival;
      if (valid_plan && plan_step + 1 < plan.size()) {
        if (plan_step >= plan_assignment_keys.size() ||
            plan_step >= plan_assignment_target_indexes.size() ||
            !apply_assignment_key_step(
                agents, tasks, graph, plan_assignment_keys[plan_step],
                plan_assignment_target_indexes[plan_step])) {
          valid_plan = false;
          previous_planner_failed = true;
        }
      }
      refresh_lifelong_priorities(inherited_priorities, agents, tasks, true);
      auto max_loaded_distance_now = 0;
      for (const auto& agent : agents) {
        if (agent_is_loaded(agent)) {
          max_loaded_distance_now =
              std::max(max_loaded_distance_now,
                       agent.loaded_distance_since_last_delivery);
        }
      }
      if (valid_plan && max_loaded_distance_now >= 500) {
        valid_plan = false;
      }

      accumulate_agent_time(agents, idle_time, loaded_time, unloaded_time,
                            carried_time, loaded_distance_time,
                            max_carried_tasks,
                            max_loaded_distance_since_last_delivery);
      append_agent_task_snapshot(metrics, agents);
      if (config.debug) {
        if (!check_motion_conflicts(previous, agents, &metrics.error) ||
            !check_lifelong_state_invariants(agents, tasks, &metrics.error,
                                             config.multi_carry_capacity)) {
          metrics.valid = false;
          break;
        }
      }
    }

    finalize_metrics(
        metrics, agents, tasks, total_planner_runtime, idle_time, loaded_time,
        unloaded_time, carried_time, loaded_distance_time, max_carried_tasks,
        max_loaded_distance_since_last_delivery, delivery_carried_sum);
    metrics.executed_path_indexes.reserve(agents.size());
    for (const auto& agent : agents) {
      auto path = std::vector<int>();
      path.reserve(agent.executed_path.size());
      for (auto v : agent.executed_path) path.push_back(v->index);
      metrics.executed_path_indexes.push_back(std::move(path));
    }
  } catch (const std::exception& e) {
    metrics.valid = false;
    metrics.error = e.what();
  }

  metrics.total_simulation_runtime =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() -
                                                           sim_start)
          .count() /
      1000000.0;
  return metrics;
}
