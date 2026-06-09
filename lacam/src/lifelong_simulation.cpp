#include "../include/lifelong_simulation.hpp"

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <unordered_set>

#include "../include/utils.hpp"

namespace
{
std::vector<LifelongAgentState> make_agents(const Graph& graph, int num_agents,
                                            std::mt19937& mt,
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
    if (agent.load_state == AgentLoadState::UNLOADED &&
        !agent.current_task_id.has_value()) {
      return true;
    }
  }
  return false;
}

bool has_unfinished_work(const std::vector<LifelongAgentState>& agents,
                         const std::vector<LifelongTask>& tasks)
{
  for (const auto& agent : agents) {
    if (agent.load_state == AgentLoadState::LOADED ||
        agent.current_task_id.has_value()) {
      return true;
    }
  }
  for (const auto& task : tasks) {
    if (task.status != LifelongTaskStatus::COMPLETED) return true;
  }
  return false;
}

void accumulate_agent_time(const std::vector<LifelongAgentState>& agents,
                           double& idle_time, double& loaded_time,
                           double& unloaded_time)
{
  for (const auto& agent : agents) {
    if (agent.load_state == AgentLoadState::LOADED) {
      loaded_time += 1;
    } else {
      unloaded_time += 1;
      if (!agent.current_task_id.has_value()) idle_time += 1;
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

void finalize_metrics(LifelongSimulationMetrics& metrics,
                      const std::vector<LifelongTask>& tasks,
                      double total_planner_runtime, double idle_time,
                      double loaded_time, double unloaded_time)
{
  auto completion_sum = 0.0;
  auto pickup_sum = 0.0;
  auto delivery_sum = 0.0;
  auto pickup_count = 0;
  auto delivery_count = 0;

  for (const auto& task : tasks) {
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
      metrics.horizon > 0 ? static_cast<double>(metrics.completed_tasks) /
                                metrics.horizon
                          : 0;
  metrics.average_task_completion_time =
      metrics.completed_tasks > 0 ? completion_sum / metrics.completed_tasks : 0;
  metrics.average_pickup_time =
      pickup_count > 0 ? pickup_sum / pickup_count : 0;
  metrics.average_delivery_time =
      delivery_count > 0 ? delivery_sum / delivery_count : 0;
  metrics.average_planner_runtime =
      metrics.planner_invocations > 0
          ? total_planner_runtime / metrics.planner_invocations
          : 0;
  const auto denom =
      static_cast<double>(std::max(1, metrics.horizon * metrics.num_agents));
  metrics.average_agent_idle_time = idle_time / denom;
  metrics.average_agent_loaded_time = loaded_time / denom;
  metrics.average_agent_unloaded_time = unloaded_time / denom;
}
}  // namespace

LifelongSimulationMetrics run_lifelong_simulation(
    const LifelongSimulationConfig& config)
{
  auto metrics = LifelongSimulationMetrics();
  metrics.map_name = std::filesystem::path(config.map_filename).filename().string();
  metrics.num_agents = config.num_agents;
  metrics.horizon = config.horizon;
  metrics.seed = config.seed;

  const auto sim_start = Time::now();
  try {
    auto graph = Graph(config.map_filename);
    metrics.map_width = graph.width;
    metrics.map_height = graph.height;
    auto mt = std::mt19937(config.seed);
    auto agents = make_agents(graph, config.num_agents, mt, config.start_indexes);
    auto tasks = std::vector<LifelongTask>();
    auto generator = LifelongTaskGenerator(&graph, config.task_config, config.seed);
    const auto cache_path =
        config.cache_filename.empty()
            ? std::filesystem::temp_directory_path() / "lacam_lifelong_dist.bin"
            : std::filesystem::path(config.cache_filename);
    const auto distances =
        load_or_build_map_distance_cache(config.map_filename, cache_path);

    auto plan = std::vector<std::vector<int> >();
    auto plan_step = size_t(0);
    auto valid_plan = false;
    auto previous_planner_failed = false;
    auto total_planner_runtime = 0.0;
    auto idle_time = 0.0;
    auto loaded_time = 0.0;
    auto unloaded_time = 0.0;

    for (int t = 0; t < config.horizon; ++t) {
      auto released = generator.generate_for_timestep(t, config.num_agents, tasks);
      metrics.generated_tasks += static_cast<int>(released.size());
      tasks.insert(tasks.end(), released.begin(), released.end());

      auto process_arrivals = [&]() {
        auto changed = false;
        for (auto& agent : agents) {
          auto pickup = try_pickup(agent, tasks, t);
          auto completion = try_complete(agent, tasks, t);
          changed = changed || pickup.changed || completion.changed;
        }
        return changed;
      };
      auto event_happened = process_arrivals();

      const auto plan_finished = !valid_plan || plan_step + 1 >= plan.size();
      const auto should_replan =
          t == 0 || event_happened ||
          (plan_finished && has_unfinished_work(agents, tasks)) ||
          (previous_planner_failed && !valid_plan) ||
          (has_idle_unloaded_agent(agents) && has_pending_task(tasks));

      if (should_replan) {
        ++metrics.planner_invocations;
        const auto planning_start = Time::now();
        auto snapshot =
            assign_lifelong_tasks_for_replanning(agents, tasks, distances);
        auto solution = Solution();
        auto next_plan = std::vector<std::vector<int> >();
        auto stats = TAPFStats();
        if (snapshot.feasible) {
          auto ins = build_lifelong_tapf_instance(config.map_filename, agents,
                                                  snapshot);
          if (ins.is_valid()) {
            auto deadline = Deadline(config.planner_time_limit_sec * 1000);
            auto planner_mt = std::mt19937(config.seed + t);
            solution =
                solve_tapf(ins, 0, &deadline, &planner_mt, 0, &stats, false);
            if (!solution.empty()) next_plan = solution_to_indexes(solution);
          }
        }
        const auto planning_runtime =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Time::now() - planning_start)
                .count() /
            1000000.0;
        total_planner_runtime += planning_runtime;
        metrics.max_planner_runtime =
            std::max(metrics.max_planner_runtime, planning_runtime);

        if (!solution.empty()) {
          plan = next_plan;
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

      auto previous = std::vector<Vertex*>();
      previous.reserve(agents.size());
      for (const auto& agent : agents) previous.push_back(agent.current_location);

      if (valid_plan && plan_step + 1 < plan.size()) {
        ++plan_step;
        for (size_t i = 0; i < agents.size(); ++i) {
          agents[i].current_location = graph.U[plan[plan_step][i]];
          agents[i].executed_path.push_back(agents[i].current_location);
        }
      } else {
        for (auto& agent : agents) agent.executed_path.push_back(agent.current_location);
      }

      if (process_arrivals()) valid_plan = false;

      accumulate_agent_time(agents, idle_time, loaded_time, unloaded_time);
      if (config.debug) {
        if (!check_motion_conflicts(previous, agents, &metrics.error) ||
            !check_lifelong_state_invariants(agents, tasks, &metrics.error)) {
          metrics.valid = false;
          break;
        }
      }
    }

    finalize_metrics(metrics, tasks, total_planner_runtime, idle_time,
                     loaded_time, unloaded_time);
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
