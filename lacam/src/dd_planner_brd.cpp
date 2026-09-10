// Carrier-BRD frozen-plan execution and solver entry.
// Split from the original dd_planner.cpp.

#include "dd_planner_internal.hpp"

using namespace dd_detail;

namespace {

struct CarrierBRDBudget {
  Deadline hard;
  Deadline search;

  explicit CarrierBRDBudget(double time_limit_sec)
      : hard(std::max(0.0, time_limit_sec) * 1000.0),
        search([&]() {
          const double total_ms =
              std::max(0.0, time_limit_sec) * 1000.0;
          // The controller owns the hard deadline and final replay.  The
          // event planner is destroyed before finalization, and measured
          // cleanup/replay stays well below a 500ms owner reserve.  Keep
          // most of a 10s benchmark budget available to completion-event
          // search while retaining time for strict final validation.
          const double reserve_ms =
              std::min(500.0, std::max(100.0, total_ms * 0.05));
          return std::max(0.0, total_ms - reserve_ms);
        }())
  {
  }
};

DDSolveStatus carrier_brd_status_for_exit(
    CarrierBRDExitReason reason)
{
  switch (reason) {
    case CarrierBRDExitReason::SOLVED:
      return DDSolveStatus::SOLVED;
    case CarrierBRDExitReason::UPPER_TIMEOUT:
    case CarrierBRDExitReason::DISPATCH_TIMEOUT:
    case CarrierBRDExitReason::SEGMENT_TIMEOUT:
    case CarrierBRDExitReason::SEARCH_TIMEOUT:
    case CarrierBRDExitReason::FINALIZATION_DEADLINE:
      return DDSolveStatus::TIMEOUT;
    case CarrierBRDExitReason::UPPER_EXHAUSTED:
    case CarrierBRDExitReason::DISPATCH_STUCK:
    case CarrierBRDExitReason::SEGMENT_EXHAUSTED:
      return DDSolveStatus::EXHAUSTED;
    case CarrierBRDExitReason::TAU_FAILED:
    case CarrierBRDExitReason::TASK_COMPILE_INVALID:
    case CarrierBRDExitReason::WAVE_START_MISMATCH:
    case CarrierBRDExitReason::SEGMENT_INVALID:
    case CarrierBRDExitReason::WAVE_END_MISMATCH:
    case CarrierBRDExitReason::FINAL_GOAL_MISMATCH:
    case CarrierBRDExitReason::FINAL_REPLAY_INVALID:
      return DDSolveStatus::INVALID;
  }
  return DDSolveStatus::INVALID;
}

DDSolveResult carrier_brd_failure(
    CarrierBRDExitReason reason, DDStats* stats,
    CarrierBRDStats* brd)
{
  brd->exit_reason = reason;
  const auto status = carrier_brd_status_for_exit(reason);
  if (stats != nullptr)
    stats->timed_out = status == DDSolveStatus::TIMEOUT;
  return DDSolveResult{status, {}};
}

bool carrier_brd_upper_matches(
    const PhysConfig& physical,
    const BRLabeledUpperState& labeled)
{
  return carrier_detail::make_upper_signature(physical) ==
         project_labeled_upper_state(labeled);
}

bool carrier_brd_all_robots_free(const PhysConfig& physical)
{
  return std::all_of(
      physical.kappa.begin(), physical.kappa.end(),
      [](int value) { return value == KAPPA_FREE; });
}

FixedRobotTransfer carrier_brd_fixed_transfer(
    const FrozenShelfTask& task, int robot,
    FixedTransferStartMode mode)
{
  FixedRobotTransfer fixed;
  fixed.robot = robot;
  fixed.task_id = task.id;
  fixed.stable_shelf = task.stable_shelf;
  fixed.shelf = task.task.id.shelf;
  fixed.transfer = task.task.transfer;
  fixed.start_mode = mode;
  return fixed;
}

bool carrier_brd_target_is_carried(
    const PhysConfig& physical, int target)
{
  return std::find(
             physical.kappa.begin(), physical.kappa.end(),
             target) != physical.kappa.end();
}

bool carrier_brd_task_source_is_grounded(
    const PhysConfig& physical, const FrozenShelfTask& task)
{
  if (task.task.transfer.route.empty()) return false;
  const int source = task.task.transfer.route.front();
  if (task.stable_shelf.kind == UpperShelfHandle::Kind::TARGET) {
    const int target = task.stable_shelf.stable_id;
    return target >= 0 &&
           target < static_cast<int>(physical.target_pos.size()) &&
           !carrier_brd_target_is_carried(physical, target) &&
           physical.target_pos[target] == source;
  }
  return std::binary_search(
      physical.anon_occ.begin(), physical.anon_occ.end(), source);
}

uint64_t carrier_brd_seed_mix(uint64_t hash, uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  value ^= value >> 31;
  return hash ^ (value + (hash << 6) + (hash >> 2));
}

int carrier_brd_segment_seed(
    int root_seed, size_t wave, size_t epoch,
    const CarrierEventContract& contract)
{
  uint64_t hash = carrier_brd_seed_mix(
      0x4252445345474d54ULL,
      static_cast<uint32_t>(root_seed));
  hash = carrier_brd_seed_mix(hash, wave);
  hash = carrier_brd_seed_mix(hash, epoch);
  for (const auto& fixed : contract.active_transfers) {
    hash = carrier_brd_seed_mix(hash, fixed.robot);
    hash = carrier_brd_seed_mix(hash, fixed.task_id.wave);
    hash = carrier_brd_seed_mix(hash, fixed.task_id.ordinal);
    hash = carrier_brd_seed_mix(
        hash, static_cast<uint64_t>(fixed.stable_shelf.kind));
    hash = carrier_brd_seed_mix(hash, fixed.stable_shelf.stable_id);
    hash = carrier_brd_seed_mix(hash, fixed.transfer.endpoint);
  }
  return static_cast<int>(
      static_cast<uint32_t>(hash ^ (hash >> 32)));
}

void carrier_brd_describe_frozen_plan(
    const FrozenTaskPlan& frozen, CarrierBRDStats* brd)
{
  brd->frozen_waves = static_cast<long>(frozen.waves.size());
  brd->frozen_tasks = 0;
  brd->max_wave_width = 0;
  brd->target_tasks = 0;
  brd->anonymous_tasks = 0;
  for (const auto& wave : frozen.waves) {
    brd->frozen_tasks += static_cast<long>(wave.tasks.size());
    brd->max_wave_width = std::max(
        brd->max_wave_width,
        static_cast<long>(wave.tasks.size()));
    for (const auto& task : wave.tasks) {
      if (task.stable_shelf.kind ==
          UpperShelfHandle::Kind::TARGET)
        ++brd->target_tasks;
      else
        ++brd->anonymous_tasks;
    }
  }
}

DDSolveResult execute_carrier_brd_frozen_plan(
    const DDInstance& ins, const FrozenTaskPlan& frozen,
    const std::vector<int>& tau, int seed,
    const Deadline* search_deadline,
    const Deadline* hard_deadline, DDStats* stats,
    CarrierBRDStats* brd)
{
  carrier_brd_describe_frozen_plan(frozen, brd);
  brd->tau0 = tau;
  bool frozen_replay_cutoff = false;
  if (!replay_frozen_task_plan(
          ins, frozen, tau, nullptr, search_deadline,
          &frozen_replay_cutoff)) {
    if (frozen_replay_cutoff)
      return carrier_brd_failure(
          is_expired(hard_deadline)
              ? CarrierBRDExitReason::FINALIZATION_DEADLINE
              : CarrierBRDExitReason::SEARCH_TIMEOUT,
          stats, brd);
    return carrier_brd_failure(
        CarrierBRDExitReason::TASK_COMPILE_INVALID, stats, brd);
  }

  PhysConfig physical = initial_phys_config(ins);
  if (!carrier_brd_upper_matches(
          physical, frozen.initial_labeled_state))
    return carrier_brd_failure(
        CarrierBRDExitReason::WAVE_START_MISMATCH, stats, brd);

  const TAPFInstance view(ins);
  DDPlan raw_plan;
  std::vector<std::optional<TaskId>>
      previous_provisional_task(ins.n_robots());
  std::vector<std::optional<TransferKey>>
      previous_provisional_key(ins.n_robots());

  auto fail = [&](CarrierBRDExitReason reason) {
    brd->raw_plan_steps =
        static_cast<long>(raw_plan.size());
    return carrier_brd_failure(reason, stats, brd);
  };

  std::vector<const FrozenShelfTask*> tasks;
  std::vector<TaskLedgerEntry> ledger;
  std::map<FrozenTaskId, size_t> ledger_index;
  for (const auto& wave : frozen.waves) {
    for (const auto& task : wave.tasks) {
      if (!ledger_index.emplace(task.id, ledger.size()).second)
        return fail(CarrierBRDExitReason::TASK_COMPILE_INVALID);
      tasks.push_back(&task);
      ledger.push_back(
          TaskLedgerEntry{
              task.id, ExecutionStatus::PENDING, std::nullopt});
    }
  }
  if (tasks.empty())
    return fail(CarrierBRDExitReason::TASK_COMPILE_INVALID);

  const auto all_tasks_completed = [&]() {
    return std::all_of(
        ledger.begin(), ledger.end(),
        [](const TaskLedgerEntry& entry) {
          return entry.status == ExecutionStatus::COMPLETED;
        });
  };
  const auto task_is_frontier = [&](size_t task_index) {
    const auto& task = *tasks[task_index];
    for (size_t previous = 0;
         previous < tasks.size(); ++previous) {
      if (tasks[previous]->stable_shelf == task.stable_shelf &&
          tasks[previous]->id < task.id &&
          ledger[previous].status != ExecutionStatus::COMPLETED)
        return false;
    }
    return true;
  };
  const auto earliest_incomplete_wave = [&]() {
    size_t wave = frozen.waves.size();
    for (size_t task_index = 0;
         task_index < ledger.size(); ++task_index) {
      if (ledger[task_index].status != ExecutionStatus::COMPLETED)
        wave = std::min(wave, tasks[task_index]->id.wave);
    }
    return wave;
  };
  const auto task_is_next_wave_continuation = [&](
      size_t task_index, size_t active_wave) {
    const auto& task = *tasks[task_index];
    if (task.id.wave != active_wave + 1) return false;
    for (size_t previous = 0;
         previous < tasks.size(); ++previous) {
      if (tasks[previous]->stable_shelf == task.stable_shelf &&
          tasks[previous]->id.wave == active_wave)
        return ledger[previous].status == ExecutionStatus::COMPLETED;
    }
    return false;
  };

  size_t epoch = 0;
  while (!all_tasks_completed()) {
    if (is_expired(search_deadline))
      return fail(CarrierBRDExitReason::SEARCH_TIMEOUT);
    const size_t active_wave = earliest_incomplete_wave();
    if (active_wave == frozen.waves.size())
      return fail(CarrierBRDExitReason::SEGMENT_INVALID);
    std::vector<FixedRobotTransfer> locked;
    std::set<int> locked_sources;
    std::set<int> used_sources;
    std::set<int> used_endpoints;
    size_t frontier_wave = frozen.waves.size();
    for (size_t task_index = 0;
         task_index < ledger.size(); ++task_index) {
      const auto& entry = ledger[task_index];
      if (entry.status != ExecutionStatus::CARRYING)
        continue;
      if (!entry.robot.has_value())
        return fail(CarrierBRDExitReason::SEGMENT_INVALID);
      const auto fixed = carrier_brd_fixed_transfer(
          *tasks[task_index], *entry.robot,
          FixedTransferStartMode::LOCKED_CARRYING);
      if (!used_sources.insert(
              fixed.transfer.route.front()).second ||
          !used_endpoints.insert(
              fixed.transfer.endpoint).second)
        return fail(CarrierBRDExitReason::SEGMENT_INVALID);
      locked_sources.insert(fixed.transfer.route.front());
      frontier_wave = std::min(
          frontier_wave, fixed.task_id.wave);
      locked.push_back(fixed);
    }
    brd->locked_pair_continuations +=
        static_cast<long>(locked.size());
    brd->locked_carriers_max = std::max(
        brd->locked_carriers_max,
        static_cast<long>(locked.size()));

    ShelfTaskGraph residual;
    std::vector<size_t> residual_to_ledger;
    std::vector<int> ready;
    const auto admit_pending_task = [&](
        size_t task_index, bool next_wave_continuation) {
      const auto& entry = ledger[task_index];
      const auto& task = *tasks[task_index];
      if (entry.status != ExecutionStatus::PENDING ||
          !task_is_frontier(task_index) ||
          !carrier_brd_task_source_is_grounded(physical, task))
        return;
      const int source = task.task.transfer.route.front();
      const int endpoint = task.task.transfer.endpoint;
      // A next-wave task may normally reuse an active task's source as its
      // endpoint: the contract and LaCAM still permit the Lift/Drop
      // handoff.  Defer only the continuation whose Drop cell is the source
      // of a robot that is still carrying.  Its predecessor has not
      // completed, so admitting it now turns a simple replan into one
      // coupled source-handoff certificate; the next Drop event will
      // rematch it immediately.
      if (next_wave_continuation &&
          locked_sources.count(endpoint) != 0)
        return;
      if (!used_sources.insert(source).second ||
          !used_endpoints.insert(endpoint).second)
        return;
      frontier_wave = std::min(
          frontier_wave, task.id.wave);
      residual_to_ledger.push_back(task_index);
      residual.tasks.push_back(task.task);
      ready.push_back(static_cast<int>(ready.size()));
    };
    // The earliest incomplete wave has priority for every free robot.  Only
    // once it has no source-grounded PENDING task may a causally-ready w+1
    // continuation consume a free robot.  This still breaks the old wave
    // barrier (a later task can start while earlier work is carrying), but
    // it prevents a future transfer from coupling with unpicked current work
    // in the same full-certificate search.
    for (size_t task_index = 0;
         task_index < ledger.size(); ++task_index) {
      if (tasks[task_index]->id.wave != active_wave)
        continue;
      admit_pending_task(task_index, false);
    }
    if (residual.tasks.empty()) {
      for (size_t task_index = 0;
           task_index < ledger.size(); ++task_index) {
        if (!task_is_next_wave_continuation(
                task_index, active_wave))
          continue;
        admit_pending_task(task_index, true);
      }
    }
    residual.predecessors.resize(residual.tasks.size());
    residual.successors.resize(residual.tasks.size());

    std::vector<uint8_t> eligible(ins.n_robots(), 0);
    long free_robot_count = 0;
    for (size_t robot = 0;
         robot < ins.n_robots(); ++robot) {
      if (physical.kappa[robot] != KAPPA_FREE)
        continue;
      eligible[robot] = 1;
      ++free_robot_count;
    }

    std::vector<std::optional<size_t>>
        provisional_by_robot(ins.n_robots());
    std::vector<FixedRobotTransfer> active = locked;
    long matcher_rows = 0;
    long real_assignments = 0;
    const auto match_started = Clock::now();
    auto match = carrier_detail::match_ready_tasks(
        ins, physical, residual, ready,
        &previous_provisional_task,
        &previous_provisional_key, &eligible,
        DispatchMode::EXECUTE, true,
        CandidateAdmission::KEEP_ALL_PENDING_ROWS,
        search_deadline);
    brd->match_ms +=
        std::chrono::duration<double, std::milli>(
            Clock::now() - match_started)
            .count();
    brd->match_max_rows = std::max(
        brd->match_max_rows,
        match.telemetry.matrix_rows);
    brd->match_rows_without_finite_real_edge +=
        match.telemetry.rows_without_finite_real_edge;
    brd->match_maximum_real_cardinality +=
        std::max(
            0L,
            match.telemetry.maximum_real_cardinality);
    brd->match_real_assignments +=
        match.telemetry.real_assignments;
    brd->match_max_cardinality_ms +=
        match.telemetry.maximum_real_cardinality_time_ms;
    brd->match_max_cardinality_cutoffs +=
        match.telemetry.maximum_real_cardinality_cutoff;
    if (match.telemetry.maximum_real_cardinality >= 0 &&
        match.telemetry.maximum_real_cardinality <
            std::min<long>(
                residual.tasks.size(), free_robot_count))
      ++brd->match_hall_deficient_calls;
    matcher_rows = match.telemetry.matrix_rows;
    real_assignments = match.telemetry.real_assignments;

    if (match.status == RhoMatchStatus::CUTOFF)
      return fail(CarrierBRDExitReason::DISPATCH_TIMEOUT);
    if (match.status != RhoMatchStatus::OK)
      return fail(CarrierBRDExitReason::TASK_COMPILE_INVALID);
    ++brd->match_calls;
    const auto& telemetry = match.telemetry;
    if (telemetry.invalid_filtered != 0 ||
        telemetry.duplicate_key_filtered != 0 ||
        telemetry.same_shelf_filtered != 0 ||
        telemetry.upstream_claim_filtered != 0 ||
        telemetry.mode_ineligible_filtered != 0 ||
        telemetry.no_reachable_robot_filtered != 0 ||
        telemetry.priority_filtered != 0 ||
        telemetry.matrix_rows !=
            static_cast<long>(residual.tasks.size()) ||
        telemetry.real_assignments !=
            telemetry.maximum_real_cardinality)
      return fail(CarrierBRDExitReason::TASK_COMPILE_INVALID);

    for (size_t robot = 0;
         robot < ins.n_robots(); ++robot) {
      const int residual_index =
          robot < match.rho_ready_index.size()
              ? match.rho_ready_index[robot]
              : -1;
      if (residual_index < 0) continue;
      if (residual_index >=
          static_cast<int>(residual_to_ledger.size()))
        return fail(CarrierBRDExitReason::TASK_COMPILE_INVALID);
      const size_t task_index =
          residual_to_ledger[residual_index];
      if (physical.kappa[robot] != KAPPA_FREE ||
          ledger[task_index].status !=
              ExecutionStatus::PENDING)
        return fail(CarrierBRDExitReason::TASK_COMPILE_INVALID);
      provisional_by_robot[robot] = task_index;
      active.push_back(carrier_brd_fixed_transfer(
          *tasks[task_index], static_cast<int>(robot),
          FixedTransferStartMode::PROVISIONAL_FREE));
      if (previous_provisional_key[robot].has_value() &&
          *previous_provisional_key[robot] !=
              carrier_detail::transfer_key(
                  tasks[task_index]->task))
        ++brd->provisional_reassignments;
    }
    ++brd->dispatch_epochs;
    brd->dispatches.push_back(
        CarrierBRDDispatchSnapshot{
            static_cast<long>(
                frontier_wave == frozen.waves.size()
                    ? 0
                    : frontier_wave),
            static_cast<long>(epoch),
            free_robot_count,
            static_cast<long>(residual.tasks.size()),
            static_cast<long>(locked.size()),
            matcher_rows,
            real_assignments});
    if (active.empty())
      return fail(CarrierBRDExitReason::DISPATCH_STUCK);
    std::sort(
        active.begin(), active.end(),
        [](const FixedRobotTransfer& a,
           const FixedRobotTransfer& b) {
          return a.task_id != b.task_id
                     ? a.task_id < b.task_id
                     : a.robot < b.robot;
        });

    CarrierEventContract contract;
    contract.start = physical;
    contract.wave_ledger_snapshot = ledger;
    contract.active_transfers = std::move(active);
    const auto contract_validation =
        validate_carrier_event_contract(ins, contract);
    if (!contract_validation.valid())
      return fail(CarrierBRDExitReason::SEGMENT_INVALID);

    DDPlan segment_plan;
    TAPFStats segment_stats;
    bool segment_timed_out = false;
    const auto segment_started = Clock::now();
    {
      std::mt19937 mt(carrier_brd_segment_seed(
          seed,
          frontier_wave == frozen.waves.size()
              ? 0
              : frontier_wave,
          epoch, contract));
      TAPFSearchConfig config;
      config.objective = TAPFObjective::MAKESPAN_THEN_WORK;
      config.macro_enabled = false;
      config.stop_policy = TAPFStopPolicy::FIRST_FEASIBLE;
      config.initial_physical = physical;
      config.event_contract = &contract;
      auto planner = std::make_unique<TAPFPlanner>(
          &view, search_deadline, &mt, 0, 0, 0.001f,
          false, &segment_stats, config);
      const auto solution = planner->solve();
      segment_timed_out = segment_stats.timed_out;
      map_stats(segment_stats, stats);
      brd->segment_nodes += segment_stats.hl_nodes_created;
      if (!solution.empty() &&
          planner->solution_shelves.size() == solution.size())
        segment_plan = plan_of(
            view, solution, planner->solution_shelves);
      const auto cleanup_started = Clock::now();
      planner.reset();
      brd->cleanup_ms +=
          std::chrono::duration<double, std::milli>(
              Clock::now() - cleanup_started)
              .count();
    }
    brd->segment_ms +=
        std::chrono::duration<double, std::milli>(
            Clock::now() - segment_started)
            .count();
    ++brd->segments;
    if (segment_plan.empty()) {
      ++brd->segment_failures;
      return fail(
          segment_timed_out || is_expired(search_deadline)
              ? CarrierBRDExitReason::SEGMENT_TIMEOUT
              : CarrierBRDExitReason::SEGMENT_EXHAUSTED);
    }
    if (is_expired(hard_deadline))
      return fail(CarrierBRDExitReason::FINALIZATION_DEADLINE);

    PhysConfig segment_state = physical;
    PhysConfig executed_state = physical;
    size_t executed_steps = 0;
    std::vector<FrozenTaskId> completed;
    bool segment_valid = true;
    for (size_t step = 0; step < segment_plan.size(); ++step) {
      if (is_expired(hard_deadline))
        return fail(CarrierBRDExitReason::FINALIZATION_DEADLINE);
      const auto next = apply_ops(ins, segment_state, segment_plan[step]);
      if (!next.has_value() ||
          !validate_carrier_event_transition_replayed_contract(
              ins, contract, segment_state, segment_plan[step], *next)) {
        segment_valid = false;
        break;
      }
      const bool in_executed_prefix = executed_steps == 0;
      std::vector<FrozenTaskId> completed_this_step;
      for (const auto& fixed : contract.active_transfers) {
        if (is_expired(hard_deadline))
          return fail(CarrierBRDExitReason::FINALIZATION_DEADLINE);
        const auto before = carrier_event_phase_of(segment_state, fixed);
        const auto after = carrier_event_phase_of(*next, fixed);
        if (before.kind != CarrierTaskPhaseKind::COMPLETED &&
            after.kind == CarrierTaskPhaseKind::COMPLETED)
          completed_this_step.push_back(fixed.task_id);
      }
      if (!completed_this_step.empty() && executed_steps == 0) {
        executed_steps = step + 1;
        executed_state = *next;
        completed = std::move(completed_this_step);
      }
      segment_state = *next;
      if (is_expired(hard_deadline))
        return fail(CarrierBRDExitReason::FINALIZATION_DEADLINE);
      if (in_executed_prefix &&
          brd->incidental_goal_prefix_ms < 0 &&
          is_dd_goal(ins, segment_state))
        brd->incidental_goal_prefix_ms = elapsed_ms(hard_deadline);
    }
    for (const auto& fixed : contract.active_transfers) {
      if (carrier_event_phase_of(segment_state, fixed).kind !=
          CarrierTaskPhaseKind::COMPLETED) {
        segment_valid = false;
        break;
      }
    }
    if (!segment_valid || executed_steps == 0 || completed.empty()) {
      ++brd->segment_failures;
      return fail(CarrierBRDExitReason::SEGMENT_INVALID);
    }

    raw_plan.insert(
        raw_plan.end(), segment_plan.begin(),
        segment_plan.begin() + executed_steps);
    physical = std::move(executed_state);

    std::set<FrozenTaskId> completed_set(completed.begin(), completed.end());
    for (const auto& fixed : contract.active_transfers) {
      const auto found = ledger_index.find(fixed.task_id);
      if (found == ledger_index.end())
        return fail(CarrierBRDExitReason::SEGMENT_INVALID);
      auto& entry = ledger[found->second];
      const auto phase = carrier_event_phase_of(physical, fixed);
      if (phase.kind == CarrierTaskPhaseKind::COMPLETED) {
        if (completed_set.count(fixed.task_id) == 0)
          return fail(CarrierBRDExitReason::SEGMENT_INVALID);
        entry.status = ExecutionStatus::COMPLETED;
        entry.robot.reset();
      } else if (phase.kind == CarrierTaskPhaseKind::CARRYING) {
        entry.status = ExecutionStatus::CARRYING;
        entry.robot = fixed.robot;
      } else if (
          phase.kind == CarrierTaskPhaseKind::APPROACH &&
          fixed.start_mode == FixedTransferStartMode::PROVISIONAL_FREE) {
        entry.status = ExecutionStatus::PENDING;
        entry.robot.reset();
      } else {
        return fail(CarrierBRDExitReason::SEGMENT_INVALID);
      }
    }

    ++brd->completion_events;
    brd->tasks_completed_per_event.push_back(
        static_cast<long>(completed.size()));
    std::fill(
        previous_provisional_task.begin(),
        previous_provisional_task.end(), std::nullopt);
    std::fill(
        previous_provisional_key.begin(),
        previous_provisional_key.end(), std::nullopt);
    for (size_t robot = 0;
         robot < provisional_by_robot.size(); ++robot) {
      if (!provisional_by_robot[robot].has_value() ||
          physical.kappa[robot] != KAPPA_FREE)
        continue;
      const size_t task_index = *provisional_by_robot[robot];
      if (ledger[task_index].status != ExecutionStatus::PENDING)
        continue;
      previous_provisional_task[robot] = tasks[task_index]->task.id;
      previous_provisional_key[robot] =
          carrier_detail::transfer_key(tasks[task_index]->task);
    }
    ++epoch;
  }

  const auto& final_upper = frozen.waves.back().expected_after;
  if (final_upper.target_pos != tau ||
      !carrier_brd_upper_matches(physical, final_upper) ||
      !carrier_brd_all_robots_free(physical) ||
      !is_dd_goal(ins, physical))
    return fail(
        CarrierBRDExitReason::FINAL_GOAL_MISMATCH);

  const auto replay_started = Clock::now();
  bool raw_replay_cutoff = false;
  const auto replayed = replay_raw_prefix(
      ins, raw_plan, hard_deadline, &raw_replay_cutoff);
  brd->replay_ms +=
      std::chrono::duration<double, std::milli>(
          Clock::now() - replay_started)
          .count();
  brd->raw_plan_steps =
      static_cast<long>(raw_plan.size());
  if (raw_replay_cutoff)
    return fail(
        CarrierBRDExitReason::FINALIZATION_DEADLINE);
  if (!replayed.has_value() ||
      !(replayed->first == physical))
    return fail(
        CarrierBRDExitReason::FINAL_REPLAY_INVALID);
  brd->raw_plan_valid = true;
  brd->raw_work_scaled = replayed->second.work;

  bool prefix_cutoff = false;
  const auto deliverable = normalize_goal_prefix(
      ins, raw_plan, hard_deadline, &prefix_cutoff);
  if (prefix_cutoff)
    return fail(
        CarrierBRDExitReason::FINALIZATION_DEADLINE);
  if (!deliverable.has_value())
    return fail(
        CarrierBRDExitReason::FINAL_REPLAY_INVALID);
  brd->goal_prefix_removed =
      static_cast<long>(
          raw_plan.size() - deliverable->size());
  if (is_expired(hard_deadline))
    return fail(
        CarrierBRDExitReason::FINALIZATION_DEADLINE);

  bool cost_cutoff = false;
  const auto delivered_cost = plan_cost_checked(
      ins, *deliverable, hard_deadline, &cost_cutoff);
  if (cost_cutoff || is_expired(hard_deadline))
    return fail(
        CarrierBRDExitReason::FINALIZATION_DEADLINE);
  if (!delivered_cost.has_value())
    return fail(
        CarrierBRDExitReason::FINAL_REPLAY_INVALID);

  brd->exit_reason = CarrierBRDExitReason::SOLVED;
  if (stats != nullptr) {
    const double solved_ms = elapsed_ms(hard_deadline);
    stats->first_solution_ms = solved_ms;
    stats->first_solution_makespan =
        delivered_cost->ticks;
    stats->first_solution_work_scaled =
        delivered_cost->work;
    stats->first_solution_soc =
        delivered_cost->work_value();
    stats->best_makespan = delivered_cost->ticks;
    stats->best_work_scaled = delivered_cost->work;
    stats->best_soc = delivered_cost->work_value();
    stats->deliverable_ms = solved_ms;
    stats->timed_out = false;
  }
  return DDSolveResult{
      DDSolveStatus::SOLVED, *deliverable};
}

}  // namespace

DDFinalizationStatus dd_classify_finalization_probe(
    bool replay_valid, double elapsed_ms, double limit_ms)
{
  if (!replay_valid) return DDFinalizationStatus::INVALID;
  if (elapsed_ms > limit_ms) return DDFinalizationStatus::DEADLINE;
  return DDFinalizationStatus::ACCEPT;
}

DDSolveResult solve_carrier_brd_result(
    const DDInstance& ins, double time_limit_sec, int seed,
    DDStats* stats, CarrierBRDStats* brd_stats)
{
  if (stats != nullptr) *stats = DDStats();
  CarrierBRDStats local_brd;
  CarrierBRDStats* brd =
      brd_stats != nullptr ? brd_stats : &local_brd;
  *brd = CarrierBRDStats();
  CarrierBRDBudget budget(time_limit_sec);

  const PhysConfig initial = initial_phys_config(ins);
  if (is_dd_goal(ins, initial)) {
    brd->exit_reason = CarrierBRDExitReason::SOLVED;
    brd->raw_plan_valid = true;
    brd->raw_work_scaled = 0;
    if (stats != nullptr) {
      const double solved_ms = elapsed_ms(&budget.hard);
      stats->first_solution_ms = solved_ms;
      stats->first_solution_makespan = 0;
      stats->first_solution_work_scaled = 0;
      stats->first_solution_soc = 0;
      stats->best_makespan = 0;
      stats->best_work_scaled = 0;
      stats->best_soc = 0;
      stats->deliverable_ms = solved_ms;
    }
    return DDSolveResult{DDSolveStatus::SOLVED, {}};
  }
  if (is_expired(&budget.search))
    return carrier_brd_failure(
        CarrierBRDExitReason::SEARCH_TIMEOUT, stats, brd);

  carrier_detail::LazyPairAssignment assignment;
  carrier_detail::VacancyPotentialCache potential_cache(
      ins.grid.size());
  const auto collect_tau_vacancy_telemetry = [&]() {
    brd->vacancy_potential_builds = potential_cache.builds;
    brd->vacancy_potential_unreachable_cells =
        potential_cache.unreachable_cells;
    brd->vacancy_potential_time_ms = potential_cache.build_ms;
    brd->clearance_first_choice_fallbacks =
        potential_cache.first_choice_fallbacks;
  };
  const auto tau_started = Clock::now();
  try {
    const auto weights = soc_weights_from_env();
    DDDistCache upper_wall(ins.grid);
    const auto storage_topology =
        carrier_detail::build_storage_transfer_topology(
            ins, &budget.search);
    assignment =
        carrier_detail::build_lazy_pair_cost_assignment(
            ins, carrier_detail::make_upper_signature(initial),
            upper_wall, storage_topology, weights.alpha,
            weights.gamma, weights.delta, &budget.search,
            &potential_cache);
  } catch (const std::exception&) {
    collect_tau_vacancy_telemetry();
    brd->tau_ms =
        std::chrono::duration<double, std::milli>(
            Clock::now() - tau_started)
            .count();
    return carrier_brd_failure(
        CarrierBRDExitReason::TAU_FAILED, stats, brd);
  }
  collect_tau_vacancy_telemetry();
  brd->tau_ms =
      std::chrono::duration<double, std::milli>(
          Clock::now() - tau_started)
          .count();
  brd->tau0 = assignment.tau;
  if (assignment.cutoff)
    return carrier_brd_failure(
        CarrierBRDExitReason::SEARCH_TIMEOUT, stats, brd);
  if (assignment.tau.size() != ins.n_targets())
    return carrier_brd_failure(
        CarrierBRDExitReason::TAU_FAILED, stats, brd);
  if (is_expired(&budget.search))
    return carrier_brd_failure(
        CarrierBRDExitReason::SEARCH_TIMEOUT, stats, brd);

  const auto upper_started = Clock::now();
  const auto upper = solve_carrier_br_upper(
      ins, br_labeled_initial_state(ins), assignment.tau,
      &budget.search);
  brd->upper_ms =
      std::chrono::duration<double, std::milli>(
          Clock::now() - upper_started)
          .count();
  brd->upper_nodes = upper.stats.explored;
  brd->upper_constraints = upper.stats.loop_count;
  brd->vacancy_potential_builds +=
      upper.stats.vacancy_potential_builds;
  brd->vacancy_potential_unreachable_cells +=
      upper.stats.vacancy_potential_unreachable_cells;
  brd->vacancy_potential_time_ms +=
      upper.stats.vacancy_potential_time_ms;
  brd->clearance_first_choice_fallbacks +=
      upper.stats.clearance_first_choice_fallbacks;
  brd->upper_steps =
      static_cast<long>(upper.transitions.size());
  for (const auto& transition : upper.transitions) {
    brd->upper_transfers +=
        static_cast<long>(transition.transfers.size());
    for (const auto& transfer : transition.transfers) {
      if (transfer.stable_shelf.kind !=
          UpperShelfHandle::Kind::ANONYMOUS)
        continue;
      ++brd->selected_clearance_pushes;
      if (!transfer.transfer.route.empty())
        brd->selected_clearance_loaded_steps +=
            static_cast<long>(
                transfer.transfer.route.size() - 1);
    }
  }
  if (!upper.solved()) {
    if (upper.exit_reason == BRUpperExitReason::TIMEOUT)
      return carrier_brd_failure(
          CarrierBRDExitReason::UPPER_TIMEOUT, stats, brd);
    if (upper.exit_reason == BRUpperExitReason::EXHAUSTED)
      return carrier_brd_failure(
          CarrierBRDExitReason::UPPER_EXHAUSTED, stats, brd);
    return carrier_brd_failure(
        CarrierBRDExitReason::TASK_COMPILE_INVALID, stats, brd);
  }
  if (is_expired(&budget.hard))
    return carrier_brd_failure(
        CarrierBRDExitReason::FINALIZATION_DEADLINE, stats, brd);

  const auto compile_started = Clock::now();
  bool compile_cutoff = false;
  const auto frozen = compile_frozen_task_plan(
      ins, upper, assignment.tau, &budget.search,
      &compile_cutoff);
  brd->task_compile_ms =
      std::chrono::duration<double, std::milli>(
          Clock::now() - compile_started)
          .count();
  if (compile_cutoff) {
    if (is_expired(&budget.hard))
      return carrier_brd_failure(
          CarrierBRDExitReason::FINALIZATION_DEADLINE, stats, brd);
    return carrier_brd_failure(
        CarrierBRDExitReason::SEARCH_TIMEOUT, stats, brd);
  }
  if (!frozen.has_value())
    return carrier_brd_failure(
        CarrierBRDExitReason::TASK_COMPILE_INVALID, stats, brd);
  if (is_expired(&budget.hard))
    return carrier_brd_failure(
        CarrierBRDExitReason::FINALIZATION_DEADLINE, stats, brd);
  if (is_expired(&budget.search))
    return carrier_brd_failure(
        CarrierBRDExitReason::SEARCH_TIMEOUT, stats, brd);

  return execute_carrier_brd_frozen_plan(
      ins, *frozen, assignment.tau, seed, &budget.search,
      &budget.hard, stats, brd);
}

DDSolveResult dd_execute_frozen_task_plan_probe(
    const DDInstance& ins, const FrozenTaskPlan& frozen,
    const std::vector<int>& tau, double time_limit_sec, int seed,
    DDStats* stats, CarrierBRDStats* brd_stats)
{
  if (stats != nullptr) *stats = DDStats();
  CarrierBRDStats local_brd;
  CarrierBRDStats* brd =
      brd_stats != nullptr ? brd_stats : &local_brd;
  *brd = CarrierBRDStats();
  CarrierBRDBudget budget(time_limit_sec);
  return execute_carrier_brd_frozen_plan(
      ins, frozen, tau, seed, &budget.search, &budget.hard,
      stats, brd);
}
