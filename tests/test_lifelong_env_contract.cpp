#include <algorithm>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include <lacam.hpp>
#include <lifelong_env.hpp>

#include "gtest/gtest.h"

namespace
{
std::string cache_path(const std::string& name)
{
  return (std::filesystem::temp_directory_path() /
          ("lacam_lifelong_env_contract_" + name + ".bin"))
      .string();
}

void remove_cache_file(const std::string& path)
{
  if (path.empty()) return;
  auto ec = std::error_code();
  std::filesystem::remove(path, ec);
}

struct CacheFileGuard {
  explicit CacheFileGuard(const std::string& path) : path(path)
  {
    remove_cache_file(path);
  }

  ~CacheFileGuard() { remove_cache_file(path); }

  std::string path;
};

LifelongSimulationConfig small_env_config(const std::string& cache_name)
{
  auto config = LifelongSimulationConfig();
  config.map_filename = "./tests/assets/lifelong-sim-small.map";
  config.cache_filename = cache_path(cache_name);
  config.num_agents = 1;
  config.horizon = 50;
  config.seed = 0;
  config.start_indexes = {3};
  config.planner_time_limit_sec = 0.2;
  config.task_config.goal_set_size = 1;
  config.task_config.release_interval = 100;
  config.task_config.outbound_probability = 1.0;
  config.debug = true;
  return config;
}

LifelongSimulationConfig two_agent_corridor_config()
{
  auto config = LifelongSimulationConfig();
  config.map_filename = "./tests/assets/lifelong-corridor-3x1.map";
  config.cache_filename = "";
  config.num_agents = 2;
  config.horizon = 8;
  config.seed = 0;
  config.start_indexes = {0, 2};
  config.planner_time_limit_sec = 0.2;
  config.task_config.goal_set_size = 1;
  config.task_config.backlog_multiplier = 1;
  config.task_config.outbound_probability = 1.0;
  config.debug = true;
  return config;
}

LifelongSimulationConfig root_service_config()
{
  auto config = two_agent_corridor_config();
  config.num_agents = 1;
  config.horizon = 4;
  config.start_indexes = {0};
  config.pickup_service_duration = 0;
  config.delivery_service_duration = 0;
  return config;
}

const LifelongTask* find_observed_task(
    const LifelongEnvObservation& observation, int task_id)
{
  const auto it = std::find_if(
      observation.tasks.begin(), observation.tasks.end(),
      [&](const LifelongTask& task) { return task.task_id == task_id; });
  return it == observation.tasks.end() ? nullptr : &*it;
}
}  // namespace

TEST(lifelong_env_contract, reset_releases_t0_tasks_and_requests_replan)
{
  auto config = small_env_config("reset");
  const auto cache_guard = CacheFileGuard(config.cache_filename);
  LifelongEnvCore env(config);

  const auto reset = env.reset(config.seed);

  ASSERT_EQ(reset.observation.timestep, 0);
  ASSERT_EQ(reset.observation.agent_position_indexes.size(), 1);
  EXPECT_EQ(reset.observation.agent_position_indexes.front(), 3);
  EXPECT_FALSE(reset.observation.tasks.empty());
  EXPECT_TRUE(reset.info.needs_replan);
  EXPECT_EQ(reset.info.replan_reason, LifelongReplanReason::INITIAL);
  ASSERT_TRUE(reset.info.planner_request.has_value());
  EXPECT_EQ(reset.info.planner_request->timestep, 0);
  EXPECT_EQ(reset.info.planner_request->agents.size(), 1);
  EXPECT_EQ(reset.info.planner_request->tasks.size(),
            reset.observation.tasks.size());
}

TEST(lifelong_env_contract, policy_replans_once_then_serves_cached_step_action)
{
  auto config = small_env_config("cached_step");
  const auto cache_guard = CacheFileGuard(config.cache_filename);
  LifelongEnvCore env(config);
  LacamTapfPolicy policy(config);

  const auto reset = env.reset(config.seed);
  ASSERT_TRUE(reset.info.needs_replan);

  const auto first_action = policy.act(reset.observation, reset.info);
  EXPECT_TRUE(first_action.planner_invoked);
  EXPECT_TRUE(first_action.commits_replan_assignment);
  EXPECT_FALSE(first_action.planner_failed);
  ASSERT_EQ(first_action.next_indexes.size(),
            static_cast<size_t>(config.num_agents));

  auto step = env.step(first_action);
  ASSERT_TRUE(step.info.valid) << step.info.error;
  ASSERT_FALSE(step.terminated);
  ASSERT_FALSE(step.truncated) << step.info.error;
  EXPECT_EQ(step.observation.timestep, 1);
  EXPECT_EQ(env.metrics().planner_invocations, 1);
  EXPECT_EQ(env.metrics().planner_success_count, 1);
  ASSERT_FALSE(step.info.needs_replan);

  const auto cached_action = policy.act(step.observation, step.info);
  EXPECT_FALSE(cached_action.planner_invoked);
  EXPECT_FALSE(cached_action.commits_replan_assignment);
  EXPECT_FALSE(cached_action.planner_failed);
  ASSERT_EQ(cached_action.next_indexes.size(),
            static_cast<size_t>(config.num_agents));

  const auto cached_step = env.step(cached_action);
  ASSERT_TRUE(cached_step.info.valid) << cached_step.info.error;
  EXPECT_EQ(cached_step.observation.timestep, 2);
  EXPECT_EQ(env.metrics().planner_invocations, 1);
}

TEST(lifelong_env_contract, new_task_release_does_not_interrupt_cached_plan)
{
  auto config = small_env_config("release_no_replan");
  config.horizon = 20;
  config.task_config.release_interval = 1;
  const auto cache_guard = CacheFileGuard(config.cache_filename);
  LifelongEnvCore env(config);
  LacamTapfPolicy policy(config);

  const auto reset = env.reset(config.seed);
  auto step = env.step(policy.act(reset.observation, reset.info));
  ASSERT_TRUE(step.info.valid) << step.info.error;

  auto saw_release_without_replan = false;
  for (int guard = 0; guard < 10 && !step.terminated && !step.truncated;
       ++guard) {
    const auto action = policy.act(step.observation, step.info);
    step = env.step(action);
    ASSERT_TRUE(step.info.valid) << step.info.error;
    if (step.info.released_task_count > 0 && !step.info.event_happened &&
        !step.info.plan_finished) {
      EXPECT_FALSE(step.info.needs_replan);
      saw_release_without_replan = true;
      break;
    }
  }

  EXPECT_TRUE(saw_release_without_replan)
      << "A released task alone must stay pending until the cached one-shot "
         "plan reaches a normal replanning boundary.";
}

TEST(lifelong_env_contract,
     planner_failure_waits_one_step_and_requests_replan_again)
{
  auto config = small_env_config("planner_failure");
  config.horizon = 5;
  const auto cache_guard = CacheFileGuard(config.cache_filename);
  LifelongEnvCore env(config);

  const auto reset = env.reset(config.seed);
  ASSERT_TRUE(reset.info.needs_replan);
  const auto initial_positions = reset.observation.agent_position_indexes;

  auto failure = LifelongEnvAction();
  failure.next_indexes = initial_positions;
  failure.planner_invoked = true;
  failure.planner_failed = true;
  failure.planner_timed_out = true;

  const auto step = env.step(failure);

  ASSERT_TRUE(step.info.valid) << step.info.error;
  EXPECT_EQ(step.observation.timestep, 1);
  EXPECT_EQ(step.observation.agent_position_indexes, initial_positions);
  EXPECT_TRUE(step.info.needs_replan);
  EXPECT_EQ(step.info.replan_reason, LifelongReplanReason::PREVIOUS_FAILURE);
  EXPECT_GE(env.metrics().planner_timeout_count, 1);
  EXPECT_EQ(env.metrics().completed_tasks, 0);
}

TEST(lifelong_env_contract,
     assignment_schedule_frame_updates_bindings_and_targets)
{
  auto config = small_env_config("assignment_frame");
  const auto cache_guard = CacheFileGuard(config.cache_filename);
  LifelongEnvCore env(config);
  LacamTapfPolicy policy(config);

  const auto reset = env.reset(config.seed);
  const auto action = policy.act(reset.observation, reset.info);
  ASSERT_TRUE(action.commits_replan_assignment);
  ASSERT_EQ(action.initial_assignment_keys.size(),
            static_cast<size_t>(config.num_agents));
  ASSERT_EQ(action.initial_assignment_targets.size(),
            static_cast<size_t>(config.num_agents));

  const auto step = env.step(action);

  ASSERT_TRUE(step.info.valid) << step.info.error;
  ASSERT_EQ(step.observation.agent_assigned_task_ids.size(), 1);
  ASSERT_EQ(step.observation.agent_current_target_indexes.size(), 1);
  EXPECT_GE(step.observation.agent_current_target_indexes.front(), 0);

  const auto assigned_task_id =
      step.observation.agent_assigned_task_ids.front();
  ASSERT_GE(assigned_task_id, 0);
  const auto* assigned_task =
      find_observed_task(step.observation, assigned_task_id);
  ASSERT_NE(assigned_task, nullptr);
  EXPECT_EQ(assigned_task->status, LifelongTaskStatus::ASSIGNED);
  ASSERT_TRUE(assigned_task->assigned_agent_id.has_value());
  EXPECT_EQ(*assigned_task->assigned_agent_id, 0);
}

TEST(lifelong_env_contract, root_only_planner_solution_is_successful_action)
{
  const auto config = root_service_config();
  LifelongEnvCore env(config);
  LacamTapfPolicy policy(config);

  const auto reset = env.reset(config.seed);
  ASSERT_TRUE(reset.info.needs_replan);

  const auto action = policy.act(reset.observation, reset.info);

  EXPECT_TRUE(action.planner_invoked);
  EXPECT_TRUE(action.commits_replan_assignment);
  EXPECT_FALSE(action.planner_failed);
  EXPECT_EQ(action.next_indexes, reset.observation.agent_position_indexes);

  const auto step = env.step(action);

  ASSERT_TRUE(step.info.valid) << step.info.error;
  EXPECT_EQ(env.metrics().planner_success_count, 1);
  EXPECT_EQ(env.metrics().planner_failure_count, 0);
}

TEST(lifelong_env_contract, failed_replan_releases_unpicked_assignments)
{
  auto config = small_env_config("failed_replan_release");
  const auto cache_guard = CacheFileGuard(config.cache_filename);
  LifelongEnvCore env(config);
  LacamTapfPolicy policy(config);

  const auto reset = env.reset(config.seed);
  auto first_step = env.step(policy.act(reset.observation, reset.info));
  ASSERT_TRUE(first_step.info.valid) << first_step.info.error;
  ASSERT_EQ(first_step.observation.agent_assigned_task_ids.size(), 1);
  const auto assigned_task_id =
      first_step.observation.agent_assigned_task_ids.front();
  ASSERT_GE(assigned_task_id, 0);

  auto failure = LifelongEnvAction();
  failure.next_indexes = first_step.observation.agent_position_indexes;
  failure.planner_invoked = true;
  failure.planner_failed = true;
  failure.planner_timed_out = true;

  const auto failed_step = env.step(failure);

  ASSERT_TRUE(failed_step.info.valid) << failed_step.info.error;
  EXPECT_EQ(failed_step.observation.agent_assigned_task_ids.front(), -1);
  const auto* task = find_observed_task(failed_step.observation, assigned_task_id);
  ASSERT_NE(task, nullptr);
  EXPECT_EQ(task->status, LifelongTaskStatus::PENDING);
}

TEST(lifelong_env_contract, invalid_vertex_conflict_action_is_rejected)
{
  const auto config = two_agent_corridor_config();
  LifelongEnvCore env(config);
  const auto reset = env.reset(config.seed);
  ASSERT_EQ(reset.observation.agent_position_indexes,
            (std::vector<int>{0, 2}));

  auto vertex_conflict = LifelongEnvAction();
  vertex_conflict.next_indexes = {1, 1};

  const auto step = env.step(vertex_conflict);

  EXPECT_FALSE(step.info.valid);
  EXPECT_TRUE(step.truncated);
  EXPECT_NE(step.info.error.find("vertex conflict"), std::string::npos);
}

TEST(lifelong_env_contract, invalid_edge_swap_action_is_rejected)
{
  const auto config = two_agent_corridor_config();
  LifelongEnvCore env(config);
  const auto reset = env.reset(config.seed);
  ASSERT_EQ(reset.observation.agent_position_indexes,
            (std::vector<int>{0, 2}));

  auto move_to_center = LifelongEnvAction();
  move_to_center.next_indexes = {1, 2};
  auto after_center = env.step(move_to_center);
  ASSERT_TRUE(after_center.info.valid) << after_center.info.error;
  ASSERT_EQ(after_center.observation.agent_position_indexes,
            (std::vector<int>{1, 2}));

  auto edge_swap = LifelongEnvAction();
  edge_swap.next_indexes = {2, 1};

  const auto step = env.step(edge_swap);

  EXPECT_FALSE(step.info.valid);
  EXPECT_TRUE(step.truncated);
  EXPECT_NE(step.info.error.find("edge swap"), std::string::npos);
}

TEST(lifelong_env_contract, env_adapter_matches_legacy_simulation_metrics)
{
  auto config = small_env_config("adapter_metrics");
  config.horizon = 30;
  const auto cache_guard = CacheFileGuard(config.cache_filename);

  const auto legacy = run_lifelong_simulation(config);
  remove_cache_file(config.cache_filename);
  const auto env_metrics = run_lifelong_simulation_via_env(config);

  ASSERT_TRUE(legacy.valid) << legacy.error;
  ASSERT_TRUE(env_metrics.valid) << env_metrics.error;
  EXPECT_EQ(env_metrics.generated_tasks, legacy.generated_tasks);
  EXPECT_EQ(env_metrics.completed_tasks, legacy.completed_tasks);
  EXPECT_EQ(env_metrics.planner_invocations, legacy.planner_invocations);
  EXPECT_EQ(env_metrics.planner_success_count, legacy.planner_success_count);
  EXPECT_EQ(env_metrics.planner_timeout_count, legacy.planner_timeout_count);
  EXPECT_EQ(env_metrics.planner_failure_count, legacy.planner_failure_count);
  EXPECT_EQ(env_metrics.executed_path_indexes, legacy.executed_path_indexes);
  EXPECT_EQ(env_metrics.agent_assigned_task_ids_by_timestep,
            legacy.agent_assigned_task_ids_by_timestep);
  ASSERT_EQ(env_metrics.task_records.size(), legacy.task_records.size());
  for (size_t i = 0; i < legacy.task_records.size(); ++i) {
    EXPECT_EQ(env_metrics.task_records[i].task_id,
              legacy.task_records[i].task_id);
    EXPECT_EQ(env_metrics.task_records[i].pickup_timestep,
              legacy.task_records[i].pickup_timestep);
    EXPECT_EQ(env_metrics.task_records[i].completion_timestep,
              legacy.task_records[i].completion_timestep);
  }
}
