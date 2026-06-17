/*
 * Lifelong TAPF task model and random task generator.
 */
#pragma once

#include <optional>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "graph.hpp"

enum class LifelongTaskType {
  OUTBOUND = 0,
  INBOUND = 1,
};

enum class LifelongTaskStatus {
  PENDING = 0,
  ASSIGNED = 1,
  PICKED = 2,
  COMPLETED = 3,
};

constexpr int kLifelongTaskStartCapacity = 2;

struct LifelongTask {
  int task_id = -1;
  LifelongTaskType task_type = LifelongTaskType::OUTBOUND;
  Vertex* start = nullptr;
  Vertices goal_set;
  LifelongTaskStatus status = LifelongTaskStatus::PENDING;
  std::optional<int> assigned_agent_id;
  std::optional<int> picked_agent_id;
  int release_timestep = 0;
  std::optional<int> pickup_timestep;
  std::optional<int> completion_timestep;
};

struct LifelongTaskGeneratorConfig {
  int goal_set_size = 5;
  int release_interval = 10;
  int backlog_multiplier = 2;
  double outbound_probability = 0.5;
};

struct LifelongTaskGenerator {
  const Graph* graph;
  LifelongTaskGeneratorConfig config;
  std::mt19937 mt;
  int next_task_id;

  LifelongTaskGenerator(const Graph* graph,
                        LifelongTaskGeneratorConfig config,
                        int seed = 0);

  int release_count(int timestep, int num_agents) const;
  int release_count(int timestep, int num_agents,
                    const std::vector<LifelongTask>& tasks) const;
  std::vector<LifelongTask> generate(int timestep, int count,
                                     const std::vector<LifelongTask>& tasks);
  std::vector<LifelongTask> generate_for_timestep(
      int timestep, int num_agents, const std::vector<LifelongTask>& tasks);

 private:
  std::optional<LifelongTask> try_make_task(
      int timestep, LifelongTaskType type,
      const std::unordered_map<int, int>& start_reservations);
  LifelongTask make_task(int timestep, LifelongTaskType type,
                         const std::unordered_map<int, int>& start_reservations);
  LifelongTaskType sample_task_type();

  Vertices tunnel_vertices;
  std::vector<Vertices> tunnels;
};
