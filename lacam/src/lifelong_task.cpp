#include "../include/lifelong_task.hpp"

#include <algorithm>
#include <queue>

namespace
{
bool is_tunnel_type(char type) { return type == 'i' || type == 'o'; }

bool is_unfinished(const LifelongTask& task)
{
  return task.status != LifelongTaskStatus::COMPLETED;
}

bool reserves_start(const LifelongTask& task)
{
  return task.status == LifelongTaskStatus::PENDING ||
         task.status == LifelongTaskStatus::ASSIGNED;
}

std::vector<Vertices> find_tunnels(const Graph& graph)
{
  auto tunnels = std::vector<Vertices>();
  auto visited = std::unordered_set<int>();

  for (auto start : graph.V) {
    if (!is_tunnel_type(graph.cell_type(start)) ||
        visited.find(start->index) != visited.end()) {
      continue;
    }

    auto tunnel = Vertices();
    auto open = std::queue<Vertex*>();
    open.push(start);
    visited.insert(start->index);
    while (!open.empty()) {
      auto current = open.front();
      open.pop();
      tunnel.push_back(current);
      for (auto neighbor : current->neighbor) {
        if (!is_tunnel_type(graph.cell_type(neighbor)) ||
            visited.find(neighbor->index) != visited.end()) {
          continue;
        }
        visited.insert(neighbor->index);
        open.push(neighbor);
      }
    }
    tunnels.push_back(std::move(tunnel));
  }
  return tunnels;
}

std::unordered_map<int, int> reserved_start_counts(
    const std::vector<LifelongTask>& tasks)
{
  auto reservations = std::unordered_map<int, int>();
  for (const auto& task : tasks) {
    if (task.start != nullptr && reserves_start(task)) {
      ++reservations[task.start->index];
    }
  }
  return reservations;
}

Vertex* sample_start(const Vertices& candidates,
                     const std::unordered_map<int, int>& start_reservations,
                     std::mt19937& mt)
{
  auto available = Vertices();
  for (auto v : candidates) {
    const auto iter = start_reservations.find(v->index);
    const auto reservations =
        iter == start_reservations.end() ? 0 : iter->second;
    if (reservations < kLifelongTaskStartCapacity) available.push_back(v);
  }
  if (available.empty()) return nullptr;
  auto dist = std::uniform_int_distribution<int>(
      0, static_cast<int>(available.size()) - 1);
  return available[dist(mt)];
}

Vertices sample_goal_set(const Vertices& candidates, int goal_set_size,
                         std::mt19937& mt)
{
  if (static_cast<int>(candidates.size()) < goal_set_size) return Vertices();
  auto shuffled = candidates;
  std::shuffle(shuffled.begin(), shuffled.end(), mt);
  return Vertices(shuffled.begin(), shuffled.begin() + goal_set_size);
}

Vertices sample_tunnel_goal_set(const std::vector<Vertices>& tunnels,
                                int goal_set_size, std::mt19937& mt)
{
  auto eligible = std::vector<const Vertices*>();
  for (const auto& tunnel : tunnels) {
    if (static_cast<int>(tunnel.size()) >= goal_set_size) {
      eligible.push_back(&tunnel);
    }
  }
  if (eligible.empty()) return Vertices();

  auto tunnel_dist = std::uniform_int_distribution<int>(
      0, static_cast<int>(eligible.size()) - 1);
  return sample_goal_set(*eligible[tunnel_dist(mt)], goal_set_size, mt);
}
}  // namespace

LifelongTaskGenerator::LifelongTaskGenerator(
    const Graph* _graph, LifelongTaskGeneratorConfig _config, int seed)
    : graph(_graph),
      config(_config),
      mt(seed),
      next_task_id(0),
      tunnel_vertices(),
      tunnels()
{
  if (graph == nullptr) {
    throw std::invalid_argument("LifelongTaskGenerator requires a graph");
  }
  if (config.goal_set_size <= 0) {
    throw std::invalid_argument("goal_set_size must be positive");
  }
  if (config.release_interval <= 0) {
    throw std::invalid_argument("release_interval must be positive");
  }
  if (config.backlog_multiplier <= 0) {
    throw std::invalid_argument("backlog_multiplier must be positive");
  }
  if (config.outbound_probability < 0.0 || config.outbound_probability > 1.0) {
    throw std::invalid_argument("outbound_probability must be in [0, 1]");
  }

  tunnels = find_tunnels(*graph);
  for (const auto& tunnel : tunnels) {
    tunnel_vertices.insert(tunnel_vertices.end(), tunnel.begin(), tunnel.end());
  }
}

int LifelongTaskGenerator::release_count(int timestep, int num_agents) const
{
  if (timestep < 0) return 0;
  if (timestep == 0) return config.backlog_multiplier * num_agents;
  return timestep % config.release_interval == 0 ? 1 : 0;
}

int LifelongTaskGenerator::release_count(
    int timestep, int num_agents, const std::vector<LifelongTask>& tasks) const
{
  if (timestep < 0) return 0;
  const auto target_backlog = config.backlog_multiplier * num_agents;
  auto unfinished_count = 0;
  for (const auto& task : tasks) {
    if (is_unfinished(task)) ++unfinished_count;
  }

  if (timestep == 0) {
    return std::max(0, target_backlog - unfinished_count);
  }

  auto count = timestep % config.release_interval == 0 ? 1 : 0;
  count += std::max(0, target_backlog - unfinished_count);
  return count;
}

std::vector<LifelongTask> LifelongTaskGenerator::generate(
    int timestep, int count, const std::vector<LifelongTask>& tasks)
{
  auto generated = std::vector<LifelongTask>();
  auto start_reservations = reserved_start_counts(tasks);
  for (int i = 0; i < count; ++i) {
    auto task =
        make_task(timestep, sample_task_type(), start_reservations);
    ++start_reservations[task.start->index];
    generated.push_back(task);
  }
  return generated;
}

std::vector<LifelongTask> LifelongTaskGenerator::generate_for_timestep(
    int timestep, int num_agents, const std::vector<LifelongTask>& tasks)
{
  return generate(timestep, release_count(timestep, num_agents, tasks), tasks);
}

std::optional<LifelongTask> LifelongTaskGenerator::try_make_task(
    int timestep, LifelongTaskType type,
    const std::unordered_map<int, int>& start_reservations)
{
  const auto starts = type == LifelongTaskType::OUTBOUND
                          ? graph->vertices_of_type('a')
                          : tunnel_vertices;
  auto start = sample_start(starts, start_reservations, mt);
  if (start == nullptr) {
    return std::nullopt;
  }
  auto goal_set =
      type == LifelongTaskType::OUTBOUND
          ? sample_tunnel_goal_set(tunnels, config.goal_set_size, mt)
          : sample_goal_set(graph->vertices_of_type('a'),
                            config.goal_set_size, mt);
  if (static_cast<int>(goal_set.size()) != config.goal_set_size) {
    return std::nullopt;
  }

  auto task = LifelongTask();
  task.task_id = next_task_id++;
  task.task_type = type;
  task.start = start;
  task.goal_set = goal_set;
  task.status = LifelongTaskStatus::PENDING;
  task.release_timestep = timestep;
  return task;
}

LifelongTask LifelongTaskGenerator::make_task(
    int timestep, LifelongTaskType type,
    const std::unordered_map<int, int>& start_reservations)
{
  auto task = try_make_task(timestep, type, start_reservations);
  if (task.has_value()) return *task;

  const auto alternate =
      type == LifelongTaskType::OUTBOUND ? LifelongTaskType::INBOUND
                                         : LifelongTaskType::OUTBOUND;
  task = try_make_task(timestep, alternate, start_reservations);
  if (task.has_value()) return *task;

  throw std::runtime_error("failed to generate task: no legal start/goal set");
}

LifelongTaskType LifelongTaskGenerator::sample_task_type()
{
  auto dist = std::uniform_real_distribution<double>(0.0, 1.0);
  return dist(mt) < config.outbound_probability ? LifelongTaskType::OUTBOUND
                                                : LifelongTaskType::INBOUND;
}
