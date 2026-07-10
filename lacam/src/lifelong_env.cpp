#include "../include/lifelong_env.hpp"

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <unordered_set>

#include "../include/utils.hpp"

namespace
{
constexpr int kDeliveryLocationKeyBase = 1000000000;
constexpr int kPickupLocationKeyBase = 500000000;

std::vector<LifelongAgentState> make_env_agents(
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
          task->picked_agent_id == agent.agent_id && !task->goal_set.empty()) {
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

int task_id_from_assignment_key(int key)
{
  return key >= 1000000 ? key / 1000000 - 1 : key;
}

int agent_task_phase(const LifelongAgentState& agent)
{
  if (agent.assigned_task_id.has_value()) return 1;
  if (agent_is_loaded(agent)) return 2;
  return 0;
}

bool check_motion_conflicts(const std::vector<Vertex*>& previous,
                            const std::vector<Vertex*>& next,
                            std::string* error)
{
  auto occupied = std::unordered_set<int>();
  for (size_t i = 0; i < next.size(); ++i) {
    const auto current = next[i];
    if (current == nullptr) {
      if (error != nullptr) *error = "agent on null location";
      return false;
    }
    if (!occupied.insert(current->index).second) {
      if (error != nullptr) *error = "vertex conflict";
      return false;
    }
    for (size_t j = i + 1; j < next.size(); ++j) {
      if (previous[i] == next[j] && previous[j] == next[i]) {
        if (error != nullptr) *error = "edge swap conflict";
        return false;
      }
    }
  }
  return true;
}

bool is_legal_single_agent_move(Vertex* from, Vertex* to)
{
  if (from == nullptr || to == nullptr) return false;
  if (from == to) return true;
  return std::find(from->neighbor.begin(), from->neighbor.end(), to) !=
         from->neighbor.end();
}

std::vector<std::vector<int> > solution_to_indexes(const Solution& solution)
{
  auto indexes = std::vector<std::vector<int> >();
  indexes.reserve(solution.size());
  for (const auto& config : solution) {
    auto row = std::vector<int>();
    row.reserve(config.size());
    for (auto v : config) row.push_back(v->index);
    indexes.push_back(std::move(row));
  }
  return indexes;
}

std::vector<std::unordered_map<int, int> > partial_pickup_preferences(
    const std::vector<LifelongAgentState>& agents,
    const std::vector<LifelongTask>& tasks,
    const std::vector<bool>& service_active,
    const std::vector<int>& service_keys,
    const std::vector<int>& service_target_indexes,
    const std::vector<int>& service_progress, int pickup_service_duration)
{
  auto preferred = std::vector<std::unordered_map<int, int> >(agents.size());
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
    if (target_index < 0 || agents[i].current_location->index != target_index) {
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
      const auto& pending = snapshot.pending_task_id_by_start_index_by_agent[i];
      const auto iter = pending.find(target_index);
      if (iter == pending.end() || iter->second != key) continue;
      for (size_t j = 0; j < ins.tasks.size(); ++j) {
        const auto task_key = j < ins.task_keys.size() ? ins.task_keys[j] : -1;
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
        const auto task_key = j < ins.task_keys.size() ? ins.task_keys[j] : -1;
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

bool translate_assignment_schedule(
    const std::vector<std::vector<int> >& assignment_schedule,
    const TAPFInstance& instance, const LifelongPlanningSnapshot& snapshot,
    size_t num_agents, std::vector<std::vector<int> >& keys_by_step,
    std::vector<std::vector<int> >& indexes_by_step)
{
  keys_by_step.clear();
  indexes_by_step.clear();
  keys_by_step.reserve(assignment_schedule.size());
  indexes_by_step.reserve(assignment_schedule.size());
  for (const auto& step_assignment : assignment_schedule) {
    if (step_assignment.size() != num_agents) return false;
    auto keys = std::vector<int>();
    auto indexes = std::vector<int>();
    keys.reserve(step_assignment.size());
    indexes.reserve(step_assignment.size());
    for (size_t i = 0; i < step_assignment.size(); ++i) {
      const auto assignment = step_assignment[i];
      if (assignment < 0 ||
          assignment >= static_cast<int>(instance.tasks.size())) {
        return false;
      }
      auto key = instance.task_keys[assignment];
      const auto index = instance.tasks[assignment]->index;
      if (key >= kPickupLocationKeyBase && key < kDeliveryLocationKeyBase) {
        const auto& pending_by_start =
            snapshot.pending_task_id_by_start_index_by_agent[i];
        const auto pending = pending_by_start.find(index);
        if (pending == pending_by_start.end()) return false;
        key = pending->second;
      }
      keys.push_back(key);
      indexes.push_back(index);
    }
    keys_by_step.push_back(std::move(keys));
    indexes_by_step.push_back(std::move(indexes));
  }
  return keys_by_step.size() == assignment_schedule.size();
}

void reset_final_metric_fields(LifelongSimulationMetrics& metrics)
{
  metrics.completed_tasks = 0;
  metrics.throughput = 0;
  metrics.alternating_completed_tasks = 0;
  metrics.alternating_throughput = 0;
  metrics.final_pending_tasks = 0;
  metrics.final_assigned_tasks = 0;
  metrics.final_picked_tasks = 0;
  metrics.average_task_completion_time = 0;
  metrics.average_pickup_time = 0;
  metrics.average_delivery_time = 0;
  metrics.average_planner_runtime = 0;
  metrics.total_planner_runtime = 0;
  metrics.total_planner_search_runtime = 0;
  metrics.average_agent_idle_time = 0;
  metrics.average_agent_loaded_time = 0;
  metrics.average_agent_unloaded_time = 0;
  metrics.average_carried_tasks = 0;
  metrics.max_carried_tasks = 0;
  metrics.average_loaded_distance_since_last_delivery = 0;
  metrics.max_loaded_distance_since_last_delivery = 0;
  metrics.average_tasks_carried_at_delivery = 0;
  metrics.assignment_row_cache_hit_rate = 0;
  metrics.task_records.clear();
  metrics.executed_path_indexes.clear();
}

}  // namespace

const LifelongTask* LifelongEnvResetResult::find_task(int task_id) const
{
  return ::find_task_by_id(observation.tasks, task_id);
}

const LifelongTask* LifelongEnvStepResult::find_task(int task_id) const
{
  return ::find_task_by_id(observation.tasks, task_id);
}

LifelongEnvCore::LifelongEnvCore(LifelongSimulationConfig config)
    : config_(std::move(config))
{
}

void LifelongEnvCore::initialize_metrics()
{
  metrics_ = LifelongSimulationMetrics();
  metrics_.map_name =
      std::filesystem::path(config_.map_filename).filename().string();
  metrics_.num_agents = config_.num_agents;
  metrics_.horizon = config_.horizon;
  metrics_.seed = config_.seed;
  metrics_.multi_carry_capacity = config_.multi_carry_capacity;
  metrics_.max_shared_drop_goal_agents = config_.max_shared_drop_goal_agents;
  metrics_.assignment_cost_mode = config_.assignment_cost_mode;
  metrics_.planner_force_full_assignment =
      config_.planner_force_full_assignment;
  if (graph_ != nullptr) {
    metrics_.map_width = graph_->width;
    metrics_.map_height = graph_->height;
  }
}

LifelongEnvResetResult LifelongEnvCore::reset(int seed)
{
  config_.seed = seed;
  if (config_.multi_carry_capacity <= 0) {
    throw std::invalid_argument("multi_carry_capacity must be positive");
  }
  if (config_.max_shared_drop_goal_agents <= 0) {
    throw std::invalid_argument("max_shared_drop_goal_agents must be positive");
  }
  if (config_.assignment_cost_mode != LIFELONG_ASSIGNMENT_COST_BASELINE &&
      config_.assignment_cost_mode !=
          LIFELONG_ASSIGNMENT_COST_MILD_PICKUP_DELAY &&
      config_.assignment_cost_mode != LIFELONG_ASSIGNMENT_COST_CONGESTION) {
    throw std::invalid_argument("unsupported assignment_cost_mode");
  }

  graph_ = std::make_unique<Graph>(config_.map_filename);
  auto mt = std::mt19937(config_.seed);
  agents_ = make_env_agents(*graph_, config_.num_agents, mt,
                            config_.start_indexes);
  tasks_.clear();
  generator_ =
      std::make_unique<LifelongTaskGenerator>(graph_.get(), config_.task_config,
                                              config_.seed);
  const auto cache_path =
      config_.cache_filename.empty()
          ? std::filesystem::temp_directory_path() / "lacam_lifelong_dist.bin"
          : std::filesystem::path(config_.cache_filename);
  distances_ = std::make_unique<MapDistanceCache>(
      load_or_build_map_distance_cache(config_.map_filename, cache_path));
  service_active_.assign(agents_.size(), false);
  service_keys_.assign(agents_.size(), -1);
  service_target_indexes_.assign(agents_.size(), -1);
  service_progress_.assign(agents_.size(), 0);
  inherited_priorities_.assign(agents_.size(), 0.0f);
  timestep_ = 0;
  reset_done_ = true;
  previous_planner_failed_ = false;
  valid_plan_ = false;
  total_planner_runtime_ = 0;
  idle_time_ = 0;
  loaded_time_ = 0;
  unloaded_time_ = 0;
  carried_time_ = 0;
  loaded_distance_time_ = 0;
  max_carried_tasks_ = 0;
  max_loaded_distance_since_last_delivery_ = 0;
  delivery_carried_sum_ = 0;
  initialize_metrics();
  append_agent_task_snapshot();
  const auto released = release_tasks_for_timestep(0);
  auto result = LifelongEnvResetResult();
  result.observation = make_observation();
  result.info = make_info(released, false, false, true, false, true);
  return result;
}

int LifelongEnvCore::release_tasks_for_timestep(int timestep)
{
  auto released =
      generator_->generate_for_timestep(timestep, config_.num_agents, tasks_);
  const auto count = static_cast<int>(released.size());
  metrics_.generated_tasks += count;
  tasks_.insert(tasks_.end(), released.begin(), released.end());
  return count;
}

std::vector<bool> LifelongEnvCore::service_ready_agents(
    const std::vector<Vertex*>& previous)
{
  auto ready = std::vector<bool>(agents_.size(), false);
  for (size_t i = 0; i < agents_.size(); ++i) {
    if (agents_[i].current_location == nullptr) {
      service_active_[i] = false;
      service_keys_[i] = -1;
      service_target_indexes_[i] = -1;
      service_progress_[i] = 0;
      continue;
    }
    auto key = -1;
    auto duration = 0;
    if (agents_[i].assigned_task_id.has_value() &&
        carried_task_count(agents_[i]) < config_.multi_carry_capacity) {
      const auto* task = find_task_by_id(tasks_, *agents_[i].assigned_task_id);
      if (task != nullptr && task->status == LifelongTaskStatus::ASSIGNED &&
          task->assigned_agent_id == agents_[i].agent_id &&
          task->start == agents_[i].current_location) {
        key = task->task_id;
        duration = config_.pickup_service_duration;
      }
    }
    if (key < 0 && agent_is_loaded(agents_[i])) {
      for (const auto task_id : agents_[i].carried_task_ids) {
        const auto* task = find_task_by_id(tasks_, task_id);
        if (task != nullptr && task->status == LifelongTaskStatus::PICKED &&
            task->picked_agent_id == agents_[i].agent_id &&
            vertex_in_goal_set(agents_[i].current_location, task->goal_set)) {
          key = kDeliveryLocationKeyBase + task->task_id;
          duration = config_.delivery_service_duration;
          break;
        }
      }
    }
    const auto target_index = agents_[i].current_location->index;
    if (key < 0) {
      service_active_[i] = false;
      service_keys_[i] = -1;
      service_target_indexes_[i] = -1;
      service_progress_[i] = 0;
      continue;
    }
    const auto same_service =
        service_active_[i] && service_keys_[i] == key &&
        service_target_indexes_[i] == target_index;
    if (!same_service) {
      service_active_[i] = true;
      service_keys_[i] = key;
      service_target_indexes_[i] = target_index;
      service_progress_[i] = 0;
    } else if (i < previous.size() && previous[i] != nullptr &&
               previous[i]->index == target_index) {
      service_progress_[i] = std::max(0, service_progress_[i]) + 1;
    }
    ready[i] = service_progress_[i] >= std::max(0, duration);
  }
  return ready;
}

LifelongEnvCore::ArrivalResult LifelongEnvCore::process_arrivals(
    int event_timestep, const std::vector<bool>* ready_agents)
{
  auto result = ArrivalResult();
  for (size_t i = 0; i < agents_.size(); ++i) {
    if (ready_agents != nullptr &&
        (i >= ready_agents->size() || !(*ready_agents)[i])) {
      continue;
    }
    auto& agent = agents_[i];
    const auto carried_before_pickup = carried_task_count(agent);
    auto pickup =
        try_pickup(agent, tasks_, event_timestep, config_.multi_carry_capacity);
    if (pickup.changed && carried_before_pickup > 0) {
      ++metrics_.pickup_while_loaded_count;
    }
    const auto carried_before_delivery = carried_task_count(agent);
    auto completion = try_complete(agent, tasks_, event_timestep);
    if (completion.changed) {
      ++metrics_.delivery_events;
      delivery_carried_sum_ += carried_before_delivery;
    }
    if (pickup.changed || completion.changed) {
      service_active_[i] = false;
      service_keys_[i] = -1;
      service_target_indexes_[i] = -1;
      service_progress_[i] = 0;
      if (i < inherited_priorities_.size()) inherited_priorities_[i] = 0.0f;
    }
    result.changed = result.changed || pickup.changed || completion.changed;
    result.pickup = result.pickup || pickup.changed;
    result.completion = result.completion || completion.changed;
  }
  return result;
}

bool LifelongEnvCore::apply_assignment_frame(const std::vector<int>& keys,
                                             const std::vector<int>& targets,
                                             std::string* error)
{
  if (keys.empty() && targets.empty()) return true;
  if (keys.size() != agents_.size() || targets.size() != agents_.size()) {
    if (error != nullptr) *error = "assignment frame size mismatch";
    return false;
  }

  release_unpicked_assignments(agents_, tasks_);
  auto pickup_task_ids =
      std::vector<std::optional<int> >(agents_.size(), std::nullopt);
  auto used_pickups = std::unordered_set<int>();
  for (size_t i = 0; i < agents_.size(); ++i) {
    const auto target_index = targets[i];
    if (target_index < 0 ||
        target_index >= static_cast<int>(graph_->U.size()) ||
        graph_->U[target_index] == nullptr) {
      if (error != nullptr) *error = "invalid assignment target";
      return false;
    }
    const auto key = keys[i];
    if (key < 0) continue;
    if (key >= kDeliveryLocationKeyBase) {
      const auto valid_delivery = std::any_of(
          agents_[i].carried_task_ids.begin(),
          agents_[i].carried_task_ids.end(), [&](const int carried_task_id) {
            const auto* carried = find_task_by_id(tasks_, carried_task_id);
            return carried != nullptr &&
                   carried->status == LifelongTaskStatus::PICKED &&
                   vertex_in_goal_set(graph_->U[target_index],
                                      carried->goal_set);
          });
      if (!valid_delivery) {
        if (error != nullptr) *error = "invalid delivery assignment";
        return false;
      }
      continue;
    }
    if (key >= kPickupLocationKeyBase) {
      if (error != nullptr) *error = "untranslated pickup assignment key";
      return false;
    }
    const auto task_id = task_id_from_assignment_key(key);
    auto* task = find_task_by_id(tasks_, task_id);
    if (task == nullptr) {
      if (error != nullptr) *error = "assigned task not found";
      return false;
    }
    if (key < 1000000) {
      if (task->status != LifelongTaskStatus::PENDING ||
          task->start == nullptr || task->start->index != target_index ||
          !used_pickups.insert(task_id).second) {
        if (error != nullptr) *error = "invalid pickup assignment";
        return false;
      }
      pickup_task_ids[i] = task_id;
      continue;
    }
    if (task->status != LifelongTaskStatus::PICKED ||
        std::find(agents_[i].carried_task_ids.begin(),
                  agents_[i].carried_task_ids.end(),
                  task_id) == agents_[i].carried_task_ids.end() ||
        !vertex_in_goal_set(graph_->U[target_index], task->goal_set)) {
      if (error != nullptr) *error = "invalid carried task assignment";
      return false;
    }
  }

  for (size_t i = 0; i < agents_.size(); ++i) {
    agents_[i].current_target = graph_->U[targets[i]];
    agents_[i].assigned_task_id.reset();
    if (pickup_task_ids[i].has_value()) {
      auto* task = find_task_by_id(tasks_, *pickup_task_ids[i]);
      if (task == nullptr) return false;
      task->status = LifelongTaskStatus::ASSIGNED;
      task->assigned_agent_id = agents_[i].agent_id;
      agents_[i].assigned_task_id = task->task_id;
      agents_[i].current_target = task->start;
    }
    sync_agent_load_state(agents_[i]);
  }
  return true;
}

bool LifelongEnvCore::apply_motion(const std::vector<int>& next_indexes,
                                   std::vector<Vertex*>& previous,
                                   std::string* error)
{
  previous.clear();
  previous.reserve(agents_.size());
  for (const auto& agent : agents_) previous.push_back(agent.current_location);

  auto next = std::vector<Vertex*>();
  next.reserve(agents_.size());
  for (size_t i = 0; i < agents_.size(); ++i) {
    const auto index = next_indexes.empty()
                           ? agents_[i].current_location->index
                           : next_indexes[i];
    if (!graph_->is_traversable(index)) {
      if (error != nullptr) *error = "next location is not traversable";
      return false;
    }
    const auto target = graph_->U[index];
    if (!is_legal_single_agent_move(agents_[i].current_location, target)) {
      if (error != nullptr) *error = "next location is not adjacent";
      return false;
    }
    next.push_back(target);
  }
  if (!check_motion_conflicts(previous, next, error)) return false;

  for (size_t i = 0; i < agents_.size(); ++i) {
    agents_[i].current_location = next[i];
    record_loaded_movement(agents_[i], previous[i]);
    agents_[i].executed_path.push_back(agents_[i].current_location);
  }
  return true;
}

void LifelongEnvCore::append_agent_task_snapshot()
{
  if (metrics_.agent_task_ids_by_timestep.empty()) {
    metrics_.agent_task_ids_by_timestep.resize(agents_.size());
    metrics_.agent_task_phases_by_timestep.resize(agents_.size());
    metrics_.agent_carried_task_ids_by_timestep.resize(agents_.size());
    metrics_.agent_assigned_task_ids_by_timestep.resize(agents_.size());
  }
  for (size_t i = 0; i < agents_.size(); ++i) {
    metrics_.agent_task_ids_by_timestep[i].push_back(
        agents_[i].current_task_id.value_or(-1));
    metrics_.agent_task_phases_by_timestep[i].push_back(
        agent_task_phase(agents_[i]));
    metrics_.agent_carried_task_ids_by_timestep[i].push_back(
        agents_[i].carried_task_ids);
    metrics_.agent_assigned_task_ids_by_timestep[i].push_back(
        agents_[i].assigned_task_id.value_or(-1));
  }
}

void LifelongEnvCore::overwrite_latest_agent_task_snapshot()
{
  if (metrics_.agent_task_ids_by_timestep.empty() ||
      metrics_.agent_task_ids_by_timestep.front().empty()) {
    append_agent_task_snapshot();
    return;
  }
  for (size_t i = 0; i < agents_.size(); ++i) {
    metrics_.agent_task_ids_by_timestep[i].back() =
        agents_[i].current_task_id.value_or(-1);
    metrics_.agent_task_phases_by_timestep[i].back() =
        agent_task_phase(agents_[i]);
    metrics_.agent_carried_task_ids_by_timestep[i].back() =
        agents_[i].carried_task_ids;
    metrics_.agent_assigned_task_ids_by_timestep[i].back() =
        agents_[i].assigned_task_id.value_or(-1);
  }
}

void LifelongEnvCore::refresh_priorities(bool advance)
{
  if (inherited_priorities_.size() != agents_.size()) {
    inherited_priorities_.assign(agents_.size(), 0.0f);
  }
  for (size_t i = 0; i < agents_.size(); ++i) {
    if (!agent_has_priority_target(agents_[i], tasks_)) {
      inherited_priorities_[i] = 0.0f;
      continue;
    }
    if (agent_at_priority_target(agents_[i], tasks_)) continue;
    if (advance) inherited_priorities_[i] += 1.0f;
  }
}

void LifelongEnvCore::accumulate_agent_time()
{
  for (const auto& agent : agents_) {
    const auto carried = carried_task_count(agent);
    carried_time_ += carried;
    loaded_distance_time_ += agent.loaded_distance_since_last_delivery;
    max_carried_tasks_ = std::max(max_carried_tasks_, carried);
    max_loaded_distance_since_last_delivery_ =
        std::max(max_loaded_distance_since_last_delivery_,
                 agent.loaded_distance_since_last_delivery);
    if (agent_is_loaded(agent)) {
      loaded_time_ += 1;
    } else {
      unloaded_time_ += 1;
      if (!agent.assigned_task_id.has_value()) idle_time_ += 1;
    }
  }
}

void LifelongEnvCore::record_planner_result(const LifelongEnvAction& action)
{
  if (!action.planner_invoked && !action.planner_failed &&
      !action.planner_timed_out) {
    return;
  }
  ++metrics_.planner_invocations;
  total_planner_runtime_ += action.planner_runtime_ms;
  metrics_.max_planner_runtime =
      std::max(metrics_.max_planner_runtime, action.planner_runtime_ms);
  metrics_.total_assignment_runtime += action.assignment_runtime_ms;
  auto trace = action.planner_trace;
  metrics_.assignment_row_cache_requests +=
      trace.assignment_row_cache_requests;
  metrics_.assignment_row_cache_hits += trace.assignment_row_cache_hits;
  const auto has_planner_diagnostics =
      trace.should_replan || trace.timestep >= 0 || trace.snapshot_feasible ||
      trace.instance_valid;
  trace.timestep = timestep_;
  trace.planning_runtime_ms = action.planner_runtime_ms;
  trace.assignment_time_ms = action.assignment_runtime_ms;
  trace.planner_search_time_ms = action.planner_search_runtime_ms;
  trace.timed_out = action.planner_timed_out;
  trace.solution_found =
      !action.planner_failed && !action.initial_assignment_keys.empty();
  trace.should_replan = true;
  metrics_.planner_trace_records.push_back(trace);
  if (has_planner_diagnostics && !trace.snapshot_feasible) {
    ++metrics_.planner_snapshot_infeasible_count;
  } else if (has_planner_diagnostics && !trace.instance_valid) {
    ++metrics_.planner_invalid_instance_count;
  } else if (has_planner_diagnostics && action.planner_failed &&
             !action.planner_timed_out) {
    ++metrics_.planner_empty_solution_count;
  }
  if (action.planner_failed) {
    if (action.planner_timed_out) {
      ++metrics_.planner_timeout_count;
    } else {
      ++metrics_.planner_failure_count;
    }
  } else {
    ++metrics_.planner_success_count;
  }
}

LifelongEnvStepResult LifelongEnvCore::step(const LifelongEnvAction& action)
{
  auto result = LifelongEnvStepResult();
  if (!reset_done_) {
    result.info.valid = false;
    result.info.error = "step called before reset";
    result.truncated = true;
    return result;
  }
  if (timestep_ >= config_.horizon) {
    result.observation = make_observation();
    result.info = make_info(0, false, false, true, previous_planner_failed_,
                            false);
    result.info.done = true;
    result.truncated = true;
    return result;
  }
  if (!action.next_indexes.empty() &&
      action.next_indexes.size() != agents_.size()) {
    result.info.valid = false;
    result.info.error = "joint action size mismatch";
    result.truncated = true;
    result.observation = make_observation();
    return result;
  }

  auto completed_before = 0;
  for (const auto& task : tasks_) {
    if (task.status == LifelongTaskStatus::COMPLETED) ++completed_before;
  }

  record_planner_result(action);
  auto step_error = std::string();
  if (action.planner_invoked && action.planner_failed) {
    release_unpicked_assignments(agents_, tasks_);
  }
  if (action.commits_replan_assignment &&
      !apply_assignment_frame(action.initial_assignment_keys,
                              action.initial_assignment_targets,
                              &step_error)) {
    result.info.valid = false;
    result.info.error = step_error;
    result.truncated = config_.debug;
    result.observation = make_observation();
    return result;
  }
  overwrite_latest_agent_task_snapshot();

  auto previous = std::vector<Vertex*>();
  const auto wait_due_to_failure = action.planner_failed;
  const auto* motion_indexes =
      wait_due_to_failure ? nullptr : &action.next_indexes;
  if (!apply_motion(motion_indexes == nullptr ? std::vector<int>()
                                              : *motion_indexes,
                    previous, &step_error)) {
    result.info.valid = false;
    result.info.error = step_error;
    result.truncated = config_.debug;
    result.observation = make_observation();
    return result;
  }

  auto ready_agents = service_ready_agents(previous);
  auto arrivals = process_arrivals(timestep_ + 1, &ready_agents);
  if (!wait_due_to_failure && !action.assignment_keys.empty() &&
      !apply_assignment_frame(action.assignment_keys,
                              action.assignment_target_indexes, &step_error)) {
    previous_planner_failed_ = true;
    valid_plan_ = false;
  }

  refresh_priorities(true);
  valid_plan_ = !wait_due_to_failure && !action.plan_finished_after_step &&
                step_error.empty();
  previous_planner_failed_ = wait_due_to_failure || !step_error.empty();
  if (valid_plan_) {
    auto max_loaded_distance_now = 0;
    for (const auto& agent : agents_) {
      if (agent_is_loaded(agent)) {
        max_loaded_distance_now =
            std::max(max_loaded_distance_now,
                     agent.loaded_distance_since_last_delivery);
      }
    }
    if (max_loaded_distance_now >= 500) valid_plan_ = false;
  }

  accumulate_agent_time();
  ++timestep_;
  const auto released = timestep_ < config_.horizon
                            ? release_tasks_for_timestep(timestep_)
                            : 0;
  if (timestep_ < config_.horizon) {
    auto ready_at_t = service_ready_agents(std::vector<Vertex*>());
    const auto root_arrivals = process_arrivals(timestep_, &ready_at_t);
    arrivals.changed = arrivals.changed || root_arrivals.changed;
    arrivals.pickup = arrivals.pickup || root_arrivals.pickup;
    arrivals.completion = arrivals.completion || root_arrivals.completion;
    refresh_priorities(false);
  }
  append_agent_task_snapshot();

  if (config_.debug) {
    if (!step_error.empty()) {
      metrics_.valid = false;
      metrics_.error = step_error;
    } else if (!check_lifelong_state_invariants(
                   agents_, tasks_, &metrics_.error,
                   config_.multi_carry_capacity)) {
      metrics_.valid = false;
    }
  }

  auto completed_after = 0;
  for (const auto& task : tasks_) {
    if (task.status == LifelongTaskStatus::COMPLETED) ++completed_after;
  }
  result.reward = static_cast<double>(completed_after - completed_before);
  result.observation = make_observation();
  result.info =
      make_info(released, arrivals.pickup, arrivals.completion,
                action.plan_finished_after_step || !valid_plan_,
                previous_planner_failed_, false);
  if (!step_error.empty()) {
    result.info.valid = false;
    result.info.error = step_error;
  }
  result.truncated = timestep_ >= config_.horizon || !metrics_.valid;
  result.terminated = false;
  result.info.done = result.truncated || result.terminated;
  return result;
}

LifelongPlannerRequest LifelongEnvCore::make_planner_request() const
{
  auto request = LifelongPlannerRequest();
  request.timestep = timestep_;
  request.graph = graph_.get();
  request.distances = distances_.get();
  request.agents = agents_;
  request.tasks = tasks_;
  request.service_active = service_active_;
  request.service_keys = service_keys_;
  request.service_target_indexes = service_target_indexes_;
  request.service_progress = service_progress_;
  request.inherited_priorities = inherited_priorities_;
  request.config = config_;
  return request;
}

LifelongEnvInfo LifelongEnvCore::make_info(int released_task_count,
                                           bool pickup_event,
                                           bool completion_event,
                                           bool plan_finished,
                                           bool previous_failure,
                                           bool force_initial) const
{
  auto info = LifelongEnvInfo();
  info.released_task_count = released_task_count;
  info.pickup_event_happened = pickup_event;
  info.completion_event_happened = completion_event;
  info.event_happened = pickup_event || completion_event;
  info.plan_finished = plan_finished;
  info.previous_planner_failed = previous_failure;
  info.idle_unloaded_with_pending =
      has_idle_unloaded_agent(agents_) && has_pending_task(tasks_);
  info.valid = metrics_.valid;
  info.error = metrics_.error;

  if (force_initial) {
    info.needs_replan = true;
    info.replan_reason = LifelongReplanReason::INITIAL;
  } else if (info.event_happened) {
    info.needs_replan = true;
    info.replan_reason =
        pickup_event ? LifelongReplanReason::PICKUP
                     : LifelongReplanReason::COMPLETION;
  } else if (previous_failure && !valid_plan_) {
    info.needs_replan = true;
    info.replan_reason = LifelongReplanReason::PREVIOUS_FAILURE;
  } else if (plan_finished && has_unfinished_work(agents_, tasks_)) {
    info.needs_replan = true;
    info.replan_reason = LifelongReplanReason::PLAN_FINISHED;
  } else if (!valid_plan_ && info.idle_unloaded_with_pending) {
    info.needs_replan = true;
    info.replan_reason = LifelongReplanReason::IDLE_WITH_PENDING;
  }
  if (info.needs_replan && graph_ != nullptr && distances_ != nullptr) {
    info.planner_request = make_planner_request();
  }
  const auto current_metrics = finalized_metrics();
  info.latest_task_records_ready = !current_metrics.task_records.empty();
  info.done = timestep_ >= config_.horizon;
  return info;
}

LifelongEnvObservation LifelongEnvCore::make_observation() const
{
  auto obs = LifelongEnvObservation();
  obs.timestep = timestep_;
  obs.num_agents = static_cast<int>(agents_.size());
  const auto target_backlog =
      std::max(1, config_.task_config.backlog_multiplier) *
      std::max(1, config_.num_agents);
  obs.task_capacity = std::max<size_t>(
      tasks_.size(), static_cast<size_t>(target_backlog + config_.horizon + 1));
  obs.tasks = tasks_;
  obs.service_active = service_active_;
  obs.service_key = service_keys_;
  obs.service_target_index = service_target_indexes_;
  obs.service_progress = service_progress_;
  for (const auto& agent : agents_) {
    obs.agent_position_indexes.push_back(
        agent.current_location == nullptr ? -1 : agent.current_location->index);
    obs.agent_load_states.push_back(static_cast<int>(agent.load_state));
    obs.agent_assigned_task_ids.push_back(agent.assigned_task_id.value_or(-1));
    obs.agent_current_task_ids.push_back(agent.current_task_id.value_or(-1));
    obs.agent_current_target_indexes.push_back(
        agent.current_target == nullptr ? -1 : agent.current_target->index);
    obs.agent_carried_task_ids.push_back(agent.carried_task_ids);
    auto legal = std::vector<int>();
    if (agent.current_location != nullptr) {
      legal.push_back(agent.current_location->index);
      for (auto n : agent.current_location->neighbor) legal.push_back(n->index);
    }
    obs.legal_next_indexes.push_back(std::move(legal));
  }
  obs.task_ids.assign(obs.task_capacity, -1);
  obs.task_statuses.assign(obs.task_capacity, -1);
  obs.task_types.assign(obs.task_capacity, -1);
  obs.task_start_indexes.assign(obs.task_capacity, -1);
  obs.task_goal_indexes.assign(obs.task_capacity, std::vector<int>());
  obs.task_release_timesteps.assign(obs.task_capacity, -1);
  obs.task_pickup_timesteps.assign(obs.task_capacity, -1);
  obs.task_completion_timesteps.assign(obs.task_capacity, -1);
  obs.task_mask.assign(obs.task_capacity, false);
  for (size_t i = 0; i < tasks_.size() && i < obs.task_capacity; ++i) {
    const auto& task = tasks_[i];
    obs.task_ids[i] = task.task_id;
    obs.task_statuses[i] = static_cast<int>(task.status);
    obs.task_types[i] = static_cast<int>(task.task_type);
    obs.task_start_indexes[i] = task.start == nullptr ? -1 : task.start->index;
    for (auto goal : task.goal_set) {
      obs.task_goal_indexes[i].push_back(goal == nullptr ? -1 : goal->index);
    }
    obs.task_release_timesteps[i] = task.release_timestep;
    obs.task_pickup_timesteps[i] = task.pickup_timestep.value_or(-1);
    obs.task_completion_timesteps[i] =
        task.completion_timestep.value_or(-1);
    obs.task_mask[i] = true;
  }
  return obs;
}

LifelongSimulationMetrics LifelongEnvCore::finalized_metrics() const
{
  auto metrics = metrics_;
  reset_final_metric_fields(metrics);

  auto completion_sum = 0.0;
  auto pickup_sum = 0.0;
  auto delivery_sum = 0.0;
  auto pickup_count = 0;
  auto delivery_count = 0;
  for (const auto& task : tasks_) {
    auto record = LifelongTaskVisualizationRecord();
    record.task_id = task.task_id;
    record.task_type = task.task_type;
    record.start_index = task.start == nullptr ? -1 : task.start->index;
    for (auto goal : task.goal_set) {
      record.goal_indexes.push_back(goal == nullptr ? -1 : goal->index);
    }
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
  for (const auto& agent : agents_) {
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
          ? total_planner_runtime_ / metrics.planner_invocations
          : 0;
  metrics.total_planner_runtime = total_planner_runtime_;
  metrics.total_planner_search_runtime =
      std::max(0.0, total_planner_runtime_ - metrics.total_assignment_runtime);
  const auto denom =
      static_cast<double>(std::max(1, metrics.horizon * metrics.num_agents));
  metrics.average_agent_idle_time = idle_time_ / denom;
  metrics.average_agent_loaded_time = loaded_time_ / denom;
  metrics.average_agent_unloaded_time = unloaded_time_ / denom;
  metrics.average_carried_tasks = carried_time_ / denom;
  metrics.max_carried_tasks = max_carried_tasks_;
  metrics.average_loaded_distance_since_last_delivery =
      loaded_distance_time_ / denom;
  metrics.max_loaded_distance_since_last_delivery =
      max_loaded_distance_since_last_delivery_;
  metrics.average_tasks_carried_at_delivery =
      metrics.delivery_events > 0
          ? static_cast<double>(delivery_carried_sum_) /
                metrics.delivery_events
          : 0;
  metrics.assignment_row_cache_hit_rate =
      metrics.assignment_row_cache_requests > 0
          ? static_cast<double>(metrics.assignment_row_cache_hits) /
                metrics.assignment_row_cache_requests
          : 0;
  metrics.executed_path_indexes.reserve(agents_.size());
  for (const auto& agent : agents_) {
    auto path = std::vector<int>();
    path.reserve(agent.executed_path.size());
    for (auto v : agent.executed_path) path.push_back(v == nullptr ? -1 : v->index);
    metrics.executed_path_indexes.push_back(std::move(path));
  }
  return metrics;
}

LifelongSimulationMetrics LifelongEnvCore::metrics() const
{
  return finalized_metrics();
}

LacamTapfPolicy::LacamTapfPolicy(LifelongSimulationConfig config)
    : config_(std::move(config))
{
}

size_t LacamTapfPolicy::cached_step_count() const
{
  return cached_plan_.size();
}

LifelongEnvAction LacamTapfPolicy::wait_action(
    const LifelongEnvObservation& observation, bool failed,
    bool timed_out) const
{
  auto action = LifelongEnvAction();
  action.next_indexes = observation.agent_position_indexes;
  action.planner_failed = failed;
  action.planner_timed_out = timed_out;
  action.plan_finished_after_step = true;
  return action;
}

LifelongEnvAction LacamTapfPolicy::next_cached_action(
    const LifelongEnvObservation& observation)
{
  if (cached_plan_.empty() || next_plan_step_ >= cached_plan_.size()) {
    return wait_action(observation, true, false);
  }
  auto action = LifelongEnvAction();
  action.next_indexes = cached_plan_[next_plan_step_];
  if (next_plan_step_ + 1 < cached_plan_.size() &&
      next_plan_step_ < cached_assignment_keys_.size() &&
      next_plan_step_ < cached_assignment_target_indexes_.size()) {
    action.assignment_keys = cached_assignment_keys_[next_plan_step_];
    action.assignment_target_indexes =
        cached_assignment_target_indexes_[next_plan_step_];
  }
  action.plan_finished_after_step =
      next_plan_step_ + 1 >= cached_plan_.size();
  ++next_plan_step_;
  return action;
}

LifelongEnvAction LacamTapfPolicy::act(
    const LifelongEnvObservation& observation, const LifelongEnvInfo& info)
{
  if (info.needs_replan) return replan(observation, info);
  return next_cached_action(observation);
}

namespace
{
struct RolloutState {
  std::vector<LifelongAgentState> agents;
  std::vector<LifelongTask> tasks;
  std::vector<bool> service_active;
  std::vector<int> service_keys;
  std::vector<int> service_target_indexes;
  std::vector<int> service_progress;
};

// Mirrors LifelongEnvCore::service_ready_agents + process_arrivals on local
// copies: advance service dwell bookkeeping given the previous positions,
// then fire pickups/completions for agents whose service finished. Returns
// the number of completions this step.
int rollout_advance_services(RolloutState& s,
                             const LifelongSimulationConfig& config,
                             const std::vector<Vertex*>& previous,
                             int timestep)
{
  auto completions = 0;
  for (size_t i = 0; i < s.agents.size(); ++i) {
    auto& agent = s.agents[i];
    if (agent.current_location == nullptr) {
      s.service_active[i] = false;
      s.service_keys[i] = -1;
      s.service_target_indexes[i] = -1;
      s.service_progress[i] = 0;
      continue;
    }
    auto key = -1;
    auto duration = 0;
    if (agent.assigned_task_id.has_value() &&
        carried_task_count(agent) < config.multi_carry_capacity) {
      const auto* task = find_task_by_id(s.tasks, *agent.assigned_task_id);
      if (task != nullptr && task->status == LifelongTaskStatus::ASSIGNED &&
          task->assigned_agent_id == agent.agent_id &&
          task->start == agent.current_location) {
        key = task->task_id;
        duration = config.pickup_service_duration;
      }
    }
    if (key < 0 && agent_is_loaded(agent)) {
      for (const auto task_id : agent.carried_task_ids) {
        const auto* task = find_task_by_id(s.tasks, task_id);
        if (task != nullptr && task->status == LifelongTaskStatus::PICKED &&
            task->picked_agent_id == agent.agent_id &&
            vertex_in_goal_set(agent.current_location, task->goal_set)) {
          key = kDeliveryLocationKeyBase + task->task_id;
          duration = config.delivery_service_duration;
          break;
        }
      }
    }
    if (key < 0) {
      s.service_active[i] = false;
      s.service_keys[i] = -1;
      s.service_target_indexes[i] = -1;
      s.service_progress[i] = 0;
      continue;
    }
    const auto target_index = agent.current_location->index;
    const auto same_service = s.service_active[i] &&
                              s.service_keys[i] == key &&
                              s.service_target_indexes[i] == target_index;
    if (!same_service) {
      s.service_active[i] = true;
      s.service_keys[i] = key;
      s.service_target_indexes[i] = target_index;
      s.service_progress[i] = 0;
    } else if (i < previous.size() && previous[i] != nullptr &&
               previous[i]->index == target_index) {
      s.service_progress[i] = std::max(0, s.service_progress[i]) + 1;
    }
    if (s.service_progress[i] < std::max(0, duration)) continue;
    auto pickup =
        try_pickup(agent, s.tasks, timestep, config.multi_carry_capacity);
    auto completion = try_complete(agent, s.tasks, timestep);
    if (completion.changed) {
      ++completions;
      agent.loaded_distance_since_last_delivery = 0;
    }
    if (pickup.changed || completion.changed) {
      s.service_active[i] = false;
      s.service_keys[i] = -1;
      s.service_target_indexes[i] = -1;
      s.service_progress[i] = 0;
    }
  }
  return completions;
}

// One closed-loop planning step on the rollout copies: snapshot, solve,
// apply assignment + first configuration, advance services. Returns
// completions (or -1 if the pipeline failed).
int rollout_pipeline_step(RolloutState& s,
                          const LifelongSimulationConfig& config,
                          const Graph& graph,
                          const MapDistanceCache& distances,
                          const std::vector<float>& inherited_priorities,
                          int timestep, unsigned rng_salt)
{
  const auto preferred = partial_pickup_preferences(
      s.agents, s.tasks, s.service_active, s.service_keys,
      s.service_target_indexes, s.service_progress,
      config.pickup_service_duration);
  auto snapshot = prepare_lifelong_planning_snapshot(
      s.agents, s.tasks, distances, config.multi_carry_capacity,
      config.max_shared_drop_goal_agents, config.pickup_service_duration,
      config.delivery_service_duration, inherited_priorities, preferred,
      config.assignment_cost_mode);
  if (!snapshot.feasible) return -1;
  auto instance =
      build_lifelong_tapf_instance(config.map_filename, s.agents, snapshot);
  if (!instance.is_valid()) return -1;
  auto deadline = Deadline(50.0);
  auto mt = std::mt19937(config.seed + timestep + rng_salt);
  auto search_config = TAPFSearchConfig();
  search_config.mode = TAPFSearchMode::FOCAL;
  search_config.focal_tie_break = TAPFFocalTieBreak::H;
  search_config.service_goal_mode = true;
  search_config.service_commit_agents = 0;
  search_config.pickup_service_duration = config.pickup_service_duration;
  search_config.delivery_service_duration = config.delivery_service_duration;
  initialize_optional_partial_services(
      instance, snapshot, s.agents, s.service_active, s.service_keys,
      s.service_target_indexes, s.service_progress,
      config.pickup_service_duration, config.delivery_service_duration,
      search_config);
  auto stats = TAPFStats();
  auto final_assignment = std::vector<int>();
  auto assignment_schedule = std::vector<std::vector<int> >();
  const auto solution =
      solve_tapf(instance, 0, &deadline, &mt, 0, &stats, config.planner_anytime,
                 config.planner_force_full_assignment, search_config,
                 &final_assignment, &assignment_schedule);
  if (solution.empty() || solution.front().size() != s.agents.size()) {
    return -1;
  }
  if (assignment_schedule.empty() ||
      !apply_lifelong_solution_assignment(s.agents, s.tasks, snapshot,
                                          instance,
                                          assignment_schedule.front())) {
    return -1;
  }
  const auto& next =
      solution.size() >= 2 ? solution[1] : solution.front();
  auto previous = std::vector<Vertex*>(s.agents.size(), nullptr);
  for (size_t i = 0; i < s.agents.size(); ++i) {
    previous[i] = s.agents[i].current_location;
    auto v = next[i] != nullptr ? graph.U[next[i]->index] : nullptr;
    if (v != nullptr) {
      if (agent_is_loaded(s.agents[i]) && v != s.agents[i].current_location) {
        ++s.agents[i].loaded_distance_since_last_delivery;
      }
      s.agents[i].current_location = v;
    }
  }
  return rollout_advance_services(s, config, previous, timestep);
}
}  // namespace

LifelongEnvAction LacamTapfPolicy::replan(
    const LifelongEnvObservation& observation, const LifelongEnvInfo& info)
{
  cached_plan_.clear();
  cached_assignment_keys_.clear();
  cached_assignment_target_indexes_.clear();
  next_plan_step_ = 0;
  if (!info.planner_request.has_value() ||
      info.planner_request->graph == nullptr ||
      info.planner_request->distances == nullptr) {
    auto action = wait_action(observation, true, false);
    action.planner_invoked = true;
    return action;
  }

  auto action = LifelongEnvAction();
  action.planner_invoked = true;
  const auto planning_start = Time::now();
  auto request = *info.planner_request;
  auto agents = request.agents;
  auto tasks = request.tasks;
  const auto preferred_partial_pickups = partial_pickup_preferences(
      agents, tasks, request.service_active, request.service_keys,
      request.service_target_indexes, request.service_progress,
      config_.pickup_service_duration);
  auto snapshot = prepare_lifelong_planning_snapshot(
      agents, tasks, *request.distances, config_.multi_carry_capacity,
      config_.max_shared_drop_goal_agents, config_.pickup_service_duration,
      config_.delivery_service_duration, request.inherited_priorities,
      preferred_partial_pickups, config_.assignment_cost_mode);
  action.planner_trace.timestep = request.timestep;
  action.planner_trace.should_replan = true;
  action.planner_trace.event_happened = info.event_happened;
  action.planner_trace.plan_finished = info.plan_finished;
  action.planner_trace.previous_planner_failed = info.previous_planner_failed;
  action.planner_trace.idle_unloaded_with_pending =
      info.idle_unloaded_with_pending;
  action.planner_trace.snapshot_feasible = snapshot.feasible;
  count_task_statuses(tasks, action.planner_trace.pending_tasks,
                      action.planner_trace.assigned_tasks,
                      action.planner_trace.picked_tasks,
                      action.planner_trace.completed_tasks);

  auto solution = Solution();
  auto final_assignment = std::vector<int>();
  auto assignment_schedule = std::vector<std::vector<int> >();
  auto stats = TAPFStats();
  if (snapshot.feasible) {
    auto instance =
        build_lifelong_tapf_instance(config_.map_filename, agents, snapshot);
    action.planner_trace.instance_valid = instance.is_valid();
    if (instance.is_valid()) {
      auto deadline = Deadline(config_.planner_time_limit_sec * 1000);
      auto planner_mt = std::mt19937(config_.seed + request.timestep);
      auto search_config = TAPFSearchConfig();
      search_config.mode = TAPFSearchMode::FOCAL;
      search_config.focal_tie_break = TAPFFocalTieBreak::H;
      search_config.service_goal_mode = true;
      search_config.service_commit_agents =
          config_.service_commit_agents > 0
              ? std::min(config_.num_agents, config_.service_commit_agents)
              : 0;
      search_config.pickup_service_duration = config_.pickup_service_duration;
      search_config.delivery_service_duration =
          config_.delivery_service_duration;
      initialize_optional_partial_services(
          instance, snapshot, agents, request.service_active,
          request.service_keys, request.service_target_indexes,
          request.service_progress, config_.pickup_service_duration,
          config_.delivery_service_duration, search_config);
      // Closed-loop candidate selection: k solves with distinct RNG
      // (attempt 0 reproduces the single-solve behavior bit-for-bit);
      // each candidate's first step is scored by rolling the REAL
      // per-step pipeline (reassignment included) forward h steps on
      // state copies and counting completions. Deviate from attempt 0
      // only on a strictly better rollout.
      constexpr auto kRolloutCandidates = 12;
      constexpr auto kRolloutHorizon = 8;
      const auto budget_ms = config_.planner_time_limit_sec * 1000.0;
      auto elapsed_ms_now = [&]() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   Time::now() - planning_start)
                   .count() /
               1000000.0;
      };
      auto best_completions = -1;
      for (auto attempt = 0; attempt < kRolloutCandidates; ++attempt) {
        // Real-time budget gate: never start a new candidate past half the
        // per-invocation time limit, so the whole selection stays within it.
        if (attempt > 0 && elapsed_ms_now() > 0.5 * budget_ms) break;
        auto att_deadline = Deadline(std::max(
            10.0, std::min(100.0, budget_ms - elapsed_ms_now() - 100.0)));
        auto att_mt =
            std::mt19937(config_.seed + request.timestep + attempt * 1000003);
        auto att_solution = Solution();
        auto att_assignment = std::vector<int>();
        auto att_schedule = std::vector<std::vector<int> >();
        auto att_stats = TAPFStats();
        att_solution = solve_tapf(
            instance, 0, &att_deadline, &att_mt, 0, &att_stats,
            config_.planner_anytime, config_.planner_force_full_assignment,
            search_config, &att_assignment, &att_schedule);
        if (att_solution.empty()) {
          if (attempt == 0) stats = att_stats;
          continue;
        }
        auto completions = 0;
        if (att_schedule.empty() ||
            att_solution.front().size() != agents.size()) {
          completions = -1;
        } else {
          auto s = RolloutState{agents,
                                tasks,
                                request.service_active,
                                request.service_keys,
                                request.service_target_indexes,
                                request.service_progress};
          if (apply_lifelong_solution_assignment(s.agents, s.tasks, snapshot,
                                                 instance,
                                                 att_schedule.front())) {
            const auto& next = att_solution.size() >= 2
                                   ? att_solution[1]
                                   : att_solution.front();
            auto previous =
                std::vector<Vertex*>(s.agents.size(), nullptr);
            for (size_t i = 0; i < s.agents.size(); ++i) {
              previous[i] = s.agents[i].current_location;
              auto v = next[i] != nullptr
                           ? info.planner_request->graph->U[next[i]->index]
                           : nullptr;
              if (v != nullptr) {
                if (agent_is_loaded(s.agents[i]) &&
                    v != s.agents[i].current_location) {
                  ++s.agents[i].loaded_distance_since_last_delivery;
                }
                s.agents[i].current_location = v;
              }
            }
            completions = rollout_advance_services(s, config_, previous,
                                                   request.timestep);
            for (auto step = 1; step < kRolloutHorizon; ++step) {
              const auto more = rollout_pipeline_step(
                  s, config_, *info.planner_request->graph,
                  *request.distances, request.inherited_priorities,
                  request.timestep + step, 0);
              if (more < 0) break;
              completions += more;
            }
          } else {
            completions = -1;
          }
        }
        if (completions > best_completions ||
            (solution.empty() && !att_solution.empty())) {
          best_completions = completions;
          solution = std::move(att_solution);
          final_assignment = std::move(att_assignment);
          assignment_schedule = std::move(att_schedule);
          stats = att_stats;
        }
      }
      action.planner_trace.root_initial_assignment_cost =
          stats.initial_assignment_cost;
      action.planner_trace.solution_found = !solution.empty();
      action.planner_trace.timed_out = stats.timed_out;
      action.planner_trace.hl_loop_iterations = stats.hl_loop_iterations;
      action.planner_trace.hl_nodes_created = stats.hl_nodes_created;
      action.planner_trace.hl_nodes_explored = stats.hl_nodes_explored;
      action.planner_trace.assignment_calls = stats.assignment_calls;
      action.planner_trace.assignment_changes = stats.assignment_changes;
      action.planner_trace.assignment_infeasible_count =
          stats.assignment_infeasible_count;
      action.planner_trace.assignment_row_cache_requests =
          stats.assignment_row_cache_requests;
      action.planner_trace.assignment_row_cache_hits =
          stats.assignment_row_cache_hits;
      action.planner_trace.solution_depth = stats.solution_depth;
      action.planner_trace.solution_cost = stats.solution_cost;
      action.planner_trace.solution_h = stats.solution_h;
      action.planner_trace.service_satisfied_agents =
          stats.service_satisfied_agents;
      action.planner_trace.service_satisfied_pickups =
          stats.service_satisfied_pickups;
      action.planner_trace.service_satisfied_deliveries =
          stats.service_satisfied_deliveries;
      action.planner_trace.service_best_satisfied_agents =
          stats.service_best_satisfied_agents;
      if (!solution.empty()) {
        auto keys = std::vector<std::vector<int> >();
        auto targets = std::vector<std::vector<int> >();
        if (!assignment_schedule.empty() &&
            translate_assignment_schedule(assignment_schedule, instance,
                                          snapshot, agents.size(), keys,
                                          targets) &&
            apply_lifelong_solution_assignment(agents, tasks, snapshot,
                                               instance,
                                               assignment_schedule.front())) {
          cached_plan_ = solution_to_indexes(solution);
          cached_assignment_keys_ = std::move(keys);
          cached_assignment_target_indexes_ = std::move(targets);
        } else {
          solution.clear();
        }
      }
    }
  }

  const auto planning_runtime =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() -
                                                           planning_start)
          .count() /
      1000000.0;
  action.planner_runtime_ms = planning_runtime;
  action.assignment_runtime_ms = stats.assignment_time_ms;
  action.planner_search_runtime_ms =
      std::max(0.0, planning_runtime - stats.assignment_time_ms);
  action.planner_timed_out = stats.timed_out;
  if (cached_plan_.empty()) {
    action.next_indexes = observation.agent_position_indexes;
    action.planner_failed = true;
    action.plan_finished_after_step = true;
    return action;
  }

  const auto target_step = cached_plan_.size() > 1 ? size_t(1) : size_t(0);
  action.next_indexes = cached_plan_[target_step];
  if (target_step + 1 < cached_plan_.size() &&
      target_step < cached_assignment_keys_.size() &&
      target_step < cached_assignment_target_indexes_.size()) {
    action.assignment_keys = cached_assignment_keys_[target_step];
    action.assignment_target_indexes =
        cached_assignment_target_indexes_[target_step];
  }
  action.plan_finished_after_step = target_step + 1 >= cached_plan_.size();
  next_plan_step_ = target_step + 1;
  action.planner_invoked = true;
  action.planner_runtime_ms = planning_runtime;
  action.assignment_runtime_ms = stats.assignment_time_ms;
  action.planner_search_runtime_ms =
      std::max(0.0, planning_runtime - stats.assignment_time_ms);
  action.planner_trace.solution_found = true;
  action.planner_trace.snapshot_feasible = snapshot.feasible;
  action.planner_trace.instance_valid = true;
  action.commits_replan_assignment = true;
  if (!cached_assignment_keys_.empty()) {
    action.initial_assignment_keys = cached_assignment_keys_.front();
    action.initial_assignment_targets = cached_assignment_target_indexes_.front();
  }
  return action;
}

LifelongSimulationMetrics run_lifelong_simulation_via_env(
    const LifelongSimulationConfig& config)
{
  auto env = LifelongEnvCore(config);
  auto policy = LacamTapfPolicy(config);
  auto reset = env.reset(config.seed);
  auto observation = reset.observation;
  auto info = reset.info;
  while (!info.done) {
    auto action = policy.act(observation, info);
    auto step = env.step(action);
    observation = step.observation;
    info = step.info;
    if (step.truncated || step.terminated) break;
  }
  return env.metrics();
}
