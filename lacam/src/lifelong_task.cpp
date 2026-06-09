#include "../include/lifelong_task.hpp"

#include <algorithm>

namespace
{
bool is_unfinished(const LifelongTask& task)
{
  return task.status != LifelongTaskStatus::COMPLETED;
}

std::unordered_set<int> unfinished_start_indexes(
    const std::vector<LifelongTask>& tasks)
{
  auto used = std::unordered_set<int>();
  for (const auto& task : tasks) {
    if (task.start != nullptr && is_unfinished(task)) used.insert(task.start->index);
  }
  return used;
}

Vertex* sample_start(const Vertices& candidates,
                     const std::unordered_set<int>& used_starts,
                     std::mt19937& mt)
{
  auto available = Vertices();
  for (auto v : candidates) {
    if (used_starts.find(v->index) == used_starts.end()) available.push_back(v);
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
}  // namespace

LifelongTaskGenerator::LifelongTaskGenerator(
    const Graph* _graph, LifelongTaskGeneratorConfig _config, int seed)
    : graph(_graph), config(_config), mt(seed), next_task_id(0)
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
  if (config.outbound_probability < 0.0 || config.outbound_probability > 1.0) {
    throw std::invalid_argument("outbound_probability must be in [0, 1]");
  }
}

int LifelongTaskGenerator::release_count(int timestep, int num_agents) const
{
  if (timestep < 0) return 0;
  if (timestep == 0) return num_agents;
  return timestep % config.release_interval == 0 ? 1 : 0;
}

std::vector<LifelongTask> LifelongTaskGenerator::generate(
    int timestep, int count, const std::vector<LifelongTask>& tasks)
{
  auto generated = std::vector<LifelongTask>();
  auto used_starts = unfinished_start_indexes(tasks);
  for (int i = 0; i < count; ++i) {
    auto task = make_task(timestep, sample_task_type(), used_starts);
    used_starts.insert(task.start->index);
    generated.push_back(task);
  }
  return generated;
}

std::vector<LifelongTask> LifelongTaskGenerator::generate_for_timestep(
    int timestep, int num_agents, const std::vector<LifelongTask>& tasks)
{
  return generate(timestep, release_count(timestep, num_agents), tasks);
}

LifelongTask LifelongTaskGenerator::make_task(
    int timestep, LifelongTaskType type,
    const std::unordered_set<int>& used_starts)
{
  const auto start_type = type == LifelongTaskType::OUTBOUND ? 'a' : 'i';
  const auto goal_type = type == LifelongTaskType::OUTBOUND ? 'o' : 'a';
  const auto starts = graph->vertices_of_type(start_type);
  const auto goals = graph->vertices_of_type(goal_type);
  auto start = sample_start(starts, used_starts, mt);
  if (start == nullptr) {
    throw std::runtime_error("failed to generate task: no available start");
  }
  auto goal_set = sample_goal_set(goals, config.goal_set_size, mt);
  if (static_cast<int>(goal_set.size()) != config.goal_set_size) {
    throw std::runtime_error("failed to generate task: insufficient goals");
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

LifelongTaskType LifelongTaskGenerator::sample_task_type()
{
  auto dist = std::uniform_real_distribution<double>(0.0, 1.0);
  return dist(mt) < config.outbound_probability ? LifelongTaskType::OUTBOUND
                                                : LifelongTaskType::INBOUND;
}
