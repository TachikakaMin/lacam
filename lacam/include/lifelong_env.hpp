/*
 * Gymnasium-style lifelong TAPF environment core and LaCAM policy adapter.
 */
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifelong_simulation.hpp"

enum class LifelongReplanReason {
  NONE = 0,
  INITIAL = 1,
  PICKUP = 2,
  COMPLETION = 3,
  PLAN_FINISHED = 4,
  PREVIOUS_FAILURE = 5,
  IDLE_WITH_PENDING = 6,
};

struct LifelongPlannerRequest {
  int timestep = 0;
  const Graph* graph = nullptr;
  const MapDistanceCache* distances = nullptr;
  std::vector<LifelongAgentState> agents;
  std::vector<LifelongTask> tasks;
  std::vector<bool> service_active;
  std::vector<int> service_keys;
  std::vector<int> service_target_indexes;
  std::vector<int> service_progress;
  std::vector<float> inherited_priorities;
  LifelongSimulationConfig config;
};

struct LifelongEnvObservation {
  int timestep = 0;
  int num_agents = 0;
  size_t task_capacity = 0;
  std::vector<int> agent_position_indexes;
  std::vector<int> agent_load_states;
  std::vector<int> agent_assigned_task_ids;
  std::vector<int> agent_current_task_ids;
  std::vector<int> agent_current_target_indexes;
  std::vector<std::vector<int> > agent_carried_task_ids;
  std::vector<bool> service_active;
  std::vector<int> service_key;
  std::vector<int> service_target_index;
  std::vector<int> service_progress;
  std::vector<LifelongTask> tasks;
  std::vector<int> task_ids;
  std::vector<int> task_statuses;
  std::vector<int> task_types;
  std::vector<int> task_start_indexes;
  std::vector<std::vector<int> > task_goal_indexes;
  std::vector<int> task_release_timesteps;
  std::vector<int> task_pickup_timesteps;
  std::vector<int> task_completion_timesteps;
  std::vector<bool> task_mask;
  std::vector<std::vector<int> > legal_next_indexes;
};

struct LifelongEnvInfo {
  bool needs_replan = false;
  LifelongReplanReason replan_reason = LifelongReplanReason::NONE;
  std::optional<LifelongPlannerRequest> planner_request;
  int released_task_count = 0;
  bool event_happened = false;
  bool pickup_event_happened = false;
  bool completion_event_happened = false;
  bool plan_finished = false;
  bool previous_planner_failed = false;
  bool idle_unloaded_with_pending = false;
  bool valid = true;
  std::string error;
  bool invalid_action_converted_to_wait = false;
  bool latest_task_records_ready = false;
  bool done = false;
};

struct LifelongEnvAction {
  std::vector<int> next_indexes;
  std::vector<int> assignment_keys;
  std::vector<int> assignment_target_indexes;
  bool commits_replan_assignment = false;
  std::vector<int> initial_assignment_keys;
  std::vector<int> initial_assignment_targets;
  bool planner_invoked = false;
  bool planner_failed = false;
  bool planner_timed_out = false;
  bool plan_finished_after_step = false;
  double planner_runtime_ms = 0;
  double assignment_runtime_ms = 0;
  double planner_search_runtime_ms = 0;
  LifelongPlannerTraceRecord planner_trace;
};

struct LifelongEnvResetResult {
  LifelongEnvObservation observation;
  LifelongEnvInfo info;

  const LifelongTask* find_task(int task_id) const;
};

struct LifelongEnvStepResult {
  LifelongEnvObservation observation;
  double reward = 0;
  bool terminated = false;
  bool truncated = false;
  LifelongEnvInfo info;

  const LifelongTask* find_task(int task_id) const;
};

class LifelongEnvCore {
 public:
  explicit LifelongEnvCore(LifelongSimulationConfig config);

  LifelongEnvResetResult reset(int seed);
  LifelongEnvStepResult step(const LifelongEnvAction& action);
  LifelongSimulationMetrics metrics() const;
  int timestep() const { return timestep_; }

 private:
  struct ArrivalResult {
    bool changed = false;
    bool pickup = false;
    bool completion = false;
  };

  LifelongEnvObservation make_observation() const;
  LifelongEnvInfo make_info(int released_task_count, bool pickup_event,
                            bool completion_event, bool plan_finished,
                            bool previous_failure, bool force_initial) const;
  LifelongPlannerRequest make_planner_request() const;
  void initialize_metrics();
  int release_tasks_for_timestep(int timestep);
  std::vector<bool> service_ready_agents(
      const std::vector<Vertex*>& previous);
  ArrivalResult process_arrivals(int event_timestep,
                                 const std::vector<bool>* ready_agents);
  bool apply_assignment_frame(const std::vector<int>& keys,
                              const std::vector<int>& targets,
                              std::string* error);
  bool apply_motion(const std::vector<int>& next_indexes,
                    std::vector<Vertex*>& previous, std::string* error);
  void append_agent_task_snapshot();
  void overwrite_latest_agent_task_snapshot();
  void refresh_priorities(bool advance);
  void accumulate_agent_time();
  void record_planner_result(const LifelongEnvAction& action);
  LifelongSimulationMetrics finalized_metrics() const;

  LifelongSimulationConfig config_;
  LifelongSimulationMetrics metrics_;
  std::unique_ptr<Graph> graph_;
  std::unique_ptr<MapDistanceCache> distances_;
  std::unique_ptr<LifelongTaskGenerator> generator_;
  std::vector<LifelongAgentState> agents_;
  std::vector<LifelongTask> tasks_;
  std::vector<bool> service_active_;
  std::vector<int> service_keys_;
  std::vector<int> service_target_indexes_;
  std::vector<int> service_progress_;
  std::vector<float> inherited_priorities_;
  int timestep_ = 0;
  bool reset_done_ = false;
  bool previous_planner_failed_ = false;
  bool valid_plan_ = false;
  double total_planner_runtime_ = 0;
  double idle_time_ = 0;
  double loaded_time_ = 0;
  double unloaded_time_ = 0;
  double carried_time_ = 0;
  double loaded_distance_time_ = 0;
  int max_carried_tasks_ = 0;
  int max_loaded_distance_since_last_delivery_ = 0;
  int delivery_carried_sum_ = 0;
};

class LacamTapfPolicy {
 public:
  explicit LacamTapfPolicy(LifelongSimulationConfig config);

  LifelongEnvAction act(const LifelongEnvObservation& observation,
                        const LifelongEnvInfo& info);
  size_t cached_step_count() const;

 private:
  LifelongEnvAction wait_action(const LifelongEnvObservation& observation,
                                bool failed, bool timed_out) const;
  LifelongEnvAction next_cached_action(
      const LifelongEnvObservation& observation);
  LifelongEnvAction replan(const LifelongEnvObservation& observation,
                           const LifelongEnvInfo& info);

  LifelongSimulationConfig config_;
  std::vector<std::vector<int> > cached_plan_;
  std::vector<std::vector<int> > cached_assignment_keys_;
  std::vector<std::vector<int> > cached_assignment_target_indexes_;
  size_t next_plan_step_ = 0;
};

LifelongSimulationMetrics run_lifelong_simulation_via_env(
    const LifelongSimulationConfig& config);
