#include <queue>
#include <set>
#include <unordered_set>

#include <lacam.hpp>

#include "gtest/gtest.h"

namespace
{
bool all_goals_have_type(const Graph& graph, const LifelongTask& task,
                         char type)
{
  for (auto goal : task.goal_set) {
    if (graph.cell_type(goal) != type) return false;
  }
  return true;
}

bool is_tunnel_type(char type) { return type == 'i' || type == 'o'; }

bool all_goals_are_in_one_tunnel(const Graph& graph,
                                 const LifelongTask& task)
{
  if (task.goal_set.empty()) return false;

  auto reachable = std::unordered_set<int>();
  auto open = std::queue<Vertex*>();
  open.push(task.goal_set.front());
  reachable.insert(task.goal_set.front()->index);
  while (!open.empty()) {
    auto current = open.front();
    open.pop();
    for (auto neighbor : current->neighbor) {
      if (!is_tunnel_type(graph.cell_type(neighbor)) ||
          reachable.find(neighbor->index) != reachable.end()) {
        continue;
      }
      reachable.insert(neighbor->index);
      open.push(neighbor);
    }
  }

  for (auto goal : task.goal_set) {
    if (reachable.find(goal->index) == reachable.end()) return false;
  }
  return true;
}

std::set<int> goal_indexes(const LifelongTask& task)
{
  auto indexes = std::set<int>();
  for (auto goal : task.goal_set) indexes.insert(goal->index);
  return indexes;
}
}  // namespace

TEST(lifelong_task_generator, generates_outbound_tasks_in_valid_regions)
{
  const auto graph = Graph("./tests/assets/symbotic.map");
  auto config = LifelongTaskGeneratorConfig();
  config.outbound_probability = 1.0;
  auto generator = LifelongTaskGenerator(&graph, config, 7);

  const auto tasks = generator.generate(0, 3, {});

  ASSERT_EQ(tasks.size(), 3);
  for (const auto& task : tasks) {
    ASSERT_EQ(task.task_type, LifelongTaskType::OUTBOUND);
    ASSERT_EQ(task.status, LifelongTaskStatus::PENDING);
    ASSERT_EQ(task.release_timestep, 0);
    ASSERT_EQ(graph.cell_type(task.start), 'a');
    ASSERT_EQ(task.goal_set.size(), 5);
    for (auto goal : task.goal_set) {
      ASSERT_TRUE(is_tunnel_type(graph.cell_type(goal)));
    }
    ASSERT_TRUE(all_goals_are_in_one_tunnel(graph, task));
    ASSERT_EQ(goal_indexes(task).size(), 5);
  }
  ASSERT_NE(tasks[0].start, tasks[1].start);
  ASSERT_NE(tasks[1].start, tasks[2].start);
  ASSERT_NE(tasks[0].start, tasks[2].start);
}

TEST(lifelong_task_generator, generates_inbound_tasks_in_valid_regions)
{
  const auto graph = Graph("./tests/assets/symbotic.map");
  auto config = LifelongTaskGeneratorConfig();
  config.outbound_probability = 0.0;
  auto generator = LifelongTaskGenerator(&graph, config, 11);

  const auto tasks = generator.generate(10, 1, {});

  ASSERT_EQ(tasks.size(), 1);
  ASSERT_EQ(tasks.front().task_type, LifelongTaskType::INBOUND);
  ASSERT_TRUE(is_tunnel_type(graph.cell_type(tasks.front().start)));
  ASSERT_TRUE(all_goals_have_type(graph, tasks.front(), 'a'));
  ASSERT_EQ(goal_indexes(tasks.front()).size(), 5);
}

TEST(lifelong_task_generator, both_tunnel_types_support_both_task_types)
{
  const auto graph = Graph("./tests/assets/symbotic.map");

  auto outbound_config = LifelongTaskGeneratorConfig();
  outbound_config.outbound_probability = 1.0;
  auto outbound_generator =
      LifelongTaskGenerator(&graph, outbound_config, 19);
  const auto outbound_tasks = outbound_generator.generate(0, 100, {});
  auto outbound_goal_types = std::set<char>();
  for (const auto& task : outbound_tasks) {
    outbound_goal_types.insert(graph.cell_type(task.goal_set.front()));
    ASSERT_TRUE(all_goals_are_in_one_tunnel(graph, task));
  }
  ASSERT_EQ(outbound_goal_types, (std::set<char>{'i', 'o'}));

  auto inbound_config = LifelongTaskGeneratorConfig();
  inbound_config.outbound_probability = 0.0;
  auto inbound_generator = LifelongTaskGenerator(&graph, inbound_config, 23);
  const auto inbound_tasks = inbound_generator.generate(0, 50, {});
  auto inbound_start_types = std::set<char>();
  for (const auto& task : inbound_tasks) {
    inbound_start_types.insert(graph.cell_type(task.start));
  }
  ASSERT_EQ(inbound_start_types, (std::set<char>{'i', 'o'}));
}

TEST(lifelong_task_generator, release_schedule_matches_goal)
{
  const auto graph = Graph("./tests/assets/symbotic.map");
  auto generator =
      LifelongTaskGenerator(&graph, LifelongTaskGeneratorConfig(), 0);

  ASSERT_EQ(generator.release_count(0, 4), 8);
  ASSERT_EQ(generator.release_count(1, 4), 0);
  ASSERT_EQ(generator.release_count(9, 4), 0);
  ASSERT_EQ(generator.release_count(10, 4), 1);
  ASSERT_EQ(generator.release_count(990, 4), 1);
}

TEST(lifelong_task_generator, release_count_refills_backlog)
{
  const auto graph = Graph("./tests/assets/symbotic.map");
  auto generator =
      LifelongTaskGenerator(&graph, LifelongTaskGeneratorConfig(), 0);
  auto full_backlog = std::vector<LifelongTask>(8);
  auto partial_backlog = std::vector<LifelongTask>(8);
  partial_backlog[0].status = LifelongTaskStatus::COMPLETED;
  partial_backlog[1].status = LifelongTaskStatus::COMPLETED;

  ASSERT_EQ(generator.release_count(0, 4, {}), 8);
  ASSERT_EQ(generator.release_count(1, 4, full_backlog), 0);
  ASSERT_EQ(generator.release_count(10, 4, full_backlog), 1);
  ASSERT_EQ(generator.release_count(1, 4, partial_backlog), 2);
  ASSERT_EQ(generator.release_count(10, 4, partial_backlog), 3);
}

TEST(lifelong_task_generator, generate_for_timestep_refills_completed_tasks)
{
  const auto graph = Graph("./tests/assets/symbotic.map");
  auto config = LifelongTaskGeneratorConfig();
  config.outbound_probability = 1.0;
  auto generator = LifelongTaskGenerator(&graph, config, 17);

  auto initial = generator.generate_for_timestep(0, 4, {});
  ASSERT_EQ(initial.size(), 8);

  initial[0].status = LifelongTaskStatus::COMPLETED;
  initial[1].status = LifelongTaskStatus::COMPLETED;
  auto refill = generator.generate_for_timestep(1, 4, initial);

  ASSERT_EQ(refill.size(), 2);
}

TEST(lifelong_task_generator, fixed_seed_is_reproducible)
{
  const auto graph = Graph("./tests/assets/symbotic.map");
  auto config = LifelongTaskGeneratorConfig();
  config.outbound_probability = 1.0;
  auto generator_a = LifelongTaskGenerator(&graph, config, 123);
  auto generator_b = LifelongTaskGenerator(&graph, config, 123);

  const auto tasks_a = generator_a.generate(0, 2, {});
  const auto tasks_b = generator_b.generate(0, 2, {});

  ASSERT_EQ(tasks_a.size(), tasks_b.size());
  for (size_t i = 0; i < tasks_a.size(); ++i) {
    ASSERT_EQ(tasks_a[i].start->index, tasks_b[i].start->index);
    ASSERT_EQ(goal_indexes(tasks_a[i]), goal_indexes(tasks_b[i]));
  }
}

TEST(lifelong_task_generator, start_is_reused_after_pickup)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto config = LifelongTaskGeneratorConfig();
  config.goal_set_size = 3;
  config.outbound_probability = 1.0;
  auto generator = LifelongTaskGenerator(&graph, config, 5);

  auto existing = generator.generate(0, 4, {});
  auto start_counts = std::unordered_map<int, int>();
  for (const auto& task : existing) ++start_counts[task.start->index];
  ASSERT_EQ(start_counts.size(), 2);
  for (const auto& [start, count] : start_counts) ASSERT_EQ(count, 2);
  ASSERT_THROW(generator.generate(10, 1, existing), std::runtime_error);

  existing[0].status = LifelongTaskStatus::PICKED;
  auto generated = generator.generate(20, 1, existing);
  ASSERT_EQ(generated.front().start, existing[0].start);
}

TEST(lifelong_task_generator, falls_back_when_preferred_task_type_is_illegal)
{
  const auto graph = Graph("./tests/assets/lifelong-task-small.map");
  auto config = LifelongTaskGeneratorConfig();
  config.goal_set_size = 3;
  config.outbound_probability = 0.0;
  auto generator = LifelongTaskGenerator(&graph, config, 5);

  auto generated = generator.generate(0, 1, {});

  ASSERT_EQ(generated.front().task_type, LifelongTaskType::OUTBOUND);
  ASSERT_EQ(graph.cell_type(generated.front().start), 'a');
  ASSERT_TRUE(all_goals_are_in_one_tunnel(graph, generated.front()));
}

TEST(lifelong_task_generator, throws_when_no_legal_task_can_be_generated)
{
  const auto graph = Graph("./tests/assets/2x1.map");
  auto generator =
      LifelongTaskGenerator(&graph, LifelongTaskGeneratorConfig(), 0);

  ASSERT_THROW(generator.generate(0, 1, {}), std::runtime_error);
}
