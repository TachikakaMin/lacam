// Part of carrier_guidance.hpp (internal, src/): Rho candidate scoring and ready-task matching.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_execution_view.hpp"

namespace carrier_detail {
struct RhoCandidate {
  int task_index = -1;
  TaskId id;
  TransferKey key;
  int priority = 0;
};

constexpr long long kRhoDispatchInf =
    std::numeric_limits<long long>::max() / 16;

inline long long rho_checked_dispatch_add(
    long long left, long long right)
{
  if (left < 0 || right < 0 ||
      static_cast<__int128>(left) + right >= kRhoDispatchInf)
    throw std::overflow_error(
        "rho dispatch completion overflow");
  return left + right;
}

inline long long rho_bottleneck_defer_delay(
    long long service, bool priority_frontier,
    bool continued_assignment)
{
  const long long base = std::max(1LL, service);
  const long long multiplier =
      1 + static_cast<long long>(priority_frontier) +
      static_cast<long long>(continued_assignment);
  const __int128 value =
      static_cast<__int128>(base) * multiplier;
  if (value >= kRhoDispatchInf)
    throw std::overflow_error(
        "rho bottleneck defer delay overflow");
  return static_cast<long long>(value);
}

inline long long rho_priority_lex_scale(
    __int128 nonnegative_priority_sum)
{
  if (nonnegative_priority_sum < 0 ||
      nonnegative_priority_sum + 1 >= kRhoDispatchInf)
    throw std::overflow_error(
        "rho priority scale overflow");
  return static_cast<long long>(nonnegative_priority_sum + 1);
}

inline long long rho_priority_lex_cost(
    long long physical_secondary, long long priority_scale,
    int deferred_priority)
{
  if (physical_secondary < 0 || priority_scale <= 0)
    throw std::invalid_argument(
        "rho priority lex cost requires non-negative inputs");
  const long long priority =
      static_cast<long long>(std::max(0, deferred_priority));
  const __int128 value =
      static_cast<__int128>(physical_secondary) *
          priority_scale +
      priority;
  if (value >= kRhoDispatchInf)
    throw std::overflow_error(
        "rho priority lex cost overflow");
  return static_cast<long long>(value);
}

inline uint64_t rho_fingerprint_mix(uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

inline void rho_fingerprint_add(uint64_t& hash, uint64_t value)
{
  hash ^= rho_fingerprint_mix(
      value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2));
}

inline void rho_fingerprint_add(
    uint64_t& hash, const ShelfSelector& shelf)
{
  rho_fingerprint_add(hash, static_cast<uint64_t>(shelf.kind));
  rho_fingerprint_add(
      hash, static_cast<uint64_t>(static_cast<uint32_t>(shelf.value)));
}

inline void rho_fingerprint_add(
    uint64_t& hash, const TransferKey& key)
{
  rho_fingerprint_add(hash, key.shelf);
  rho_fingerprint_add(
      hash, static_cast<uint64_t>(static_cast<uint32_t>(key.source)));
  rho_fingerprint_add(
      hash, static_cast<uint64_t>(static_cast<uint32_t>(key.endpoint)));
}

inline void rho_fingerprint_add(
    uint64_t& hash, const TaskId& id)
{
  rho_fingerprint_add(hash, id.shelf);
  rho_fingerprint_add(
      hash, static_cast<uint64_t>(static_cast<uint32_t>(id.from)));
  rho_fingerprint_add(
      hash, static_cast<uint64_t>(static_cast<uint32_t>(id.to)));
}

inline DDReadyMatchProbe match_ready_tasks(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph, const std::vector<int>& ready_tasks,
    const std::vector<std::optional<TaskId>>* previous_rho_task_id,
    const std::vector<std::optional<TransferKey>>*
        previous_rho_transfer_key = nullptr,
    const std::vector<uint8_t>* eligible_robot = nullptr,
    DispatchMode mode = DispatchMode::EXECUTE,
    bool collect_audit = false,
    CandidateAdmission admission =
        CandidateAdmission::DROP_GLOBALLY_UNREACHABLE,
    const Deadline* deadline = nullptr)
{
  const auto candidate_started = std::chrono::steady_clock::now();
  DDReadyMatchProbe out;
  const size_t robot_count = ins.n_robots();
  out.rho_task_id.resize(robot_count);
  out.rho_transfer_key.resize(robot_count);
  out.rho_ready_index.assign(robot_count, -1);
  out.telemetry.candidates_input =
      static_cast<long>(ready_tasks.size());
  out.telemetry.candidates_after_claims =
      static_cast<long>(ready_tasks.size());
  const auto clear_assignments = [&]() {
    std::fill(
        out.rho_task_id.begin(), out.rho_task_id.end(),
        std::nullopt);
    std::fill(
        out.rho_transfer_key.begin(),
        out.rho_transfer_key.end(), std::nullopt);
    std::fill(
        out.rho_ready_index.begin(),
        out.rho_ready_index.end(), -1);
  };
  const auto stop_cutoff = [&]() {
    out.status = RhoMatchStatus::CUTOFF;
    clear_assignments();
  };
  const auto stop_infeasible = [&]() {
    out.status = RhoMatchStatus::INFEASIBLE;
    clear_assignments();
  };
  if (is_expired(deadline)) {
    stop_cutoff();
    return out;
  }

  std::vector<int> free_robots;
  for (size_t robot = 0; robot < robot_count; ++robot)
    if (physical.kappa[robot] == KAPPA_FREE &&
        (eligible_robot == nullptr ||
         (robot < eligible_robot->size() &&
          (*eligible_robot)[robot])))
      free_robots.push_back((int)robot);

  out.telemetry.robot_row_fingerprints.resize(robot_count);
  for (size_t robot = 0; robot < robot_count; ++robot) {
    uint64_t fingerprint = rho_fingerprint_mix(robot + 1);
    rho_fingerprint_add(
        fingerprint,
        static_cast<uint64_t>(
            static_cast<uint32_t>(physical.robots[robot])));
    rho_fingerprint_add(
        fingerprint,
        static_cast<uint64_t>(
            static_cast<uint32_t>(physical.kappa[robot])));
    const bool eligible =
        physical.kappa[robot] == KAPPA_FREE &&
        (eligible_robot == nullptr ||
         (robot < eligible_robot->size() &&
          (*eligible_robot)[robot]));
    rho_fingerprint_add(fingerprint, eligible ? 1 : 0);
    if (previous_rho_transfer_key != nullptr &&
        robot < previous_rho_transfer_key->size() &&
        (*previous_rho_transfer_key)[robot].has_value()) {
      rho_fingerprint_add(fingerprint, 1);
      rho_fingerprint_add(
          fingerprint,
          *(*previous_rho_transfer_key)[robot]);
    } else if (previous_rho_task_id != nullptr &&
               robot < previous_rho_task_id->size() &&
               (*previous_rho_task_id)[robot].has_value()) {
      rho_fingerprint_add(fingerprint, 2);
      rho_fingerprint_add(
          fingerprint, *(*previous_rho_task_id)[robot]);
    } else {
      rho_fingerprint_add(fingerprint, 0);
    }
    out.telemetry.robot_row_fingerprints[robot] = fingerprint;
  }

  uint64_t mode_fingerprint = rho_fingerprint_mix(
      static_cast<uint64_t>(mode) + 1);
  for (const int robot : free_robots)
    rho_fingerprint_add(
        mode_fingerprint,
        static_cast<uint64_t>(static_cast<uint32_t>(robot)));
  out.telemetry.mode_or_conflict_fingerprint = mode_fingerprint;
  if (free_robots.empty() &&
      admission ==
          CandidateAdmission::DROP_GLOBALLY_UNREACHABLE) {
    out.telemetry.candidate_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - candidate_started)
            .count();
    return out;
  }

  std::vector<RhoCandidate> candidates;
  std::map<TransferKey, int> seen;
  std::map<ShelfSelector, int> selected_for_shelf;
  std::set<TransferKey> distinct_keys;
  const size_t free_count = free_robots.size();
  LowerDist lower_distance(ins.grid);
  const auto nearest_robot_distance = [&](const TaskId& id) {
    int nearest = INT_MAX / 4;
    for (const int robot : free_robots)
      nearest = std::min(
          nearest,
          lower_distance.dist(id.from, physical.robots[robot]));
    return nearest >= INT_MAX / 4 ? -1 : nearest;
  };
  const auto record_drop =
      [&](int task_index, const ShelfTask& task,
          const TransferKey& key, RhoDropReason reason) {
        switch (reason) {
          case RhoDropReason::INVALID_TASK:
            ++out.telemetry.invalid_filtered;
            break;
          case RhoDropReason::DUPLICATE_TRANSFER_KEY:
            ++out.telemetry.duplicate_key_filtered;
            break;
          case RhoDropReason::SAME_SHELF_PRESELECTED:
            ++out.telemetry.same_shelf_filtered;
            break;
          case RhoDropReason::UPSTREAM_TRANSFER_CLAIM:
            ++out.telemetry.upstream_claim_filtered;
            break;
          case RhoDropReason::MODE_INELIGIBLE:
            ++out.telemetry.mode_ineligible_filtered;
            break;
          case RhoDropReason::NO_REACHABLE_ROBOT:
            ++out.telemetry.no_reachable_robot_filtered;
            break;
        }
        if (!collect_audit) return;
        out.audit.push_back(
            RhoCandidateAudit{
                task_index,
                key,
                task.id,
                mode,
                task.priority,
                nearest_robot_distance(task.id),
                reason});
      };
  for (const int index : ready_tasks) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    if (index < 0 || index >= (int)graph.tasks.size()) {
      ++out.telemetry.invalid_filtered;
      if (collect_audit)
        out.audit.push_back(
            RhoCandidateAudit{
                index,
                TransferKey{},
                TaskId{},
                mode,
                0,
                -1,
                RhoDropReason::INVALID_TASK});
      continue;
    }
    const auto& task = graph.tasks[index];
    const TransferKey key = transfer_key(task);
    distinct_keys.insert(key);
    auto it = seen.find(key);
    if (it != seen.end()) {
      if (task.priority > candidates[it->second].priority) {
        const auto& replaced =
            graph.tasks[candidates[it->second].task_index];
        record_drop(
            candidates[it->second].task_index, replaced,
            candidates[it->second].key,
            RhoDropReason::DUPLICATE_TRANSFER_KEY);
        candidates[it->second].task_index = index;
        candidates[it->second].id = task.id;
        candidates[it->second].priority = task.priority;
      } else {
        record_drop(
            index, task, key,
            RhoDropReason::DUPLICATE_TRANSFER_KEY);
      }
      continue;
    }
    const auto selected =
        selected_for_shelf.find(task.id.shelf);
    if (selected != selected_for_shelf.end()) {
      auto& existing = candidates[selected->second];
      const bool replace =
          task.priority > existing.priority ||
          (task.priority == existing.priority &&
           (key < existing.key ||
            (key == existing.key &&
             index < existing.task_index)));
      if (replace) {
        const auto& replaced =
            graph.tasks[existing.task_index];
        record_drop(
            existing.task_index, replaced, existing.key,
            RhoDropReason::SAME_SHELF_PRESELECTED);
        seen.erase(existing.key);
        existing =
            RhoCandidate{index, task.id, key, task.priority};
        seen.emplace(key, selected->second);
      } else {
        record_drop(
            index, task, key,
            RhoDropReason::SAME_SHELF_PRESELECTED);
      }
      continue;
    }
    seen.emplace(key, (int)candidates.size());
    selected_for_shelf.emplace(
        task.id.shelf, (int)candidates.size());
    candidates.push_back(
        RhoCandidate{index, task.id, key, task.priority});
  }
  out.telemetry.candidates_after_key_dedupe =
      static_cast<long>(distinct_keys.size());
  out.telemetry.candidates_after_shelf_preselect =
      static_cast<long>(candidates.size());
  if (candidates.empty()) {
    out.telemetry.candidates_after_priority = 0;
    if (admission ==
        CandidateAdmission::KEEP_ALL_PENDING_ROWS) {
      // The completion-event controller deliberately rematches even when
      // the residual wave has no PENDING rows but a locked CARRYING pair
      // still exists.  This is the well-defined empty bipartite problem
      // M=0, K=0, not an uninitialized or infeasible matching.
      out.telemetry.maximum_real_cardinality = 0;
      out.telemetry.matrix_rows = 0;
      out.telemetry.matrix_cols =
          static_cast<long>(free_count);
    }
    out.telemetry.candidate_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - candidate_started)
            .count();
    return out;
  }
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const RhoCandidate& a, const RhoCandidate& b) {
                     if (a.priority != b.priority)
                       return a.priority > b.priority;
                     if (a.key != b.key) return a.key < b.key;
                     if (a.id != b.id) return a.id < b.id;
                     return a.task_index < b.task_index;
                   });

  if (admission ==
      CandidateAdmission::DROP_GLOBALLY_UNREACHABLE) {
    candidates.erase(
        std::remove_if(
            candidates.begin(), candidates.end(),
            [&](const RhoCandidate& candidate) {
              if (nearest_robot_distance(candidate.id) >= 0)
                return false;
              const auto& task =
                  graph.tasks[candidate.task_index];
              record_drop(
                  candidate.task_index, task, candidate.key,
                  RhoDropReason::NO_REACHABLE_ROBOT);
              return true;
            }),
        candidates.end());
  }

  const size_t task_count = candidates.size();
  out.telemetry.candidates_after_priority =
      static_cast<long>(task_count);
  out.telemetry.priority_filtered = 0;

  std::vector<std::vector<int>> real_distance(
      task_count, std::vector<int>(free_count, -1));
  std::vector<uint8_t> actionable(task_count, 0);
  std::vector<size_t> actionable_rows;
  for (size_t row = 0; row < task_count; ++row) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    for (size_t col = 0; col < free_count; ++col) {
      if (is_expired(deadline)) {
        stop_cutoff();
        return out;
      }
      const int distance = lower_distance.dist(
          candidates[row].id.from,
          physical.robots[free_robots[col]]);
      if (distance < INT_MAX / 4) {
        real_distance[row][col] = distance;
        actionable[row] = 1;
      }
    }
    if (actionable[row]) {
      actionable_rows.push_back(row);
    } else if (
        admission ==
        CandidateAdmission::KEEP_ALL_PENDING_ROWS) {
      ++out.telemetry.rows_without_finite_real_edge;
    }
  }

  size_t maximum_real_cardinality = 0;
  if (admission ==
      CandidateAdmission::KEEP_ALL_PENDING_ROWS) {
    const auto cardinality_started =
        std::chrono::steady_clock::now();
    std::vector<int> owner(free_count, -1);
    bool matching_cutoff = false;
    std::function<bool(size_t, std::vector<uint8_t>&)> augment =
        [&](size_t row, std::vector<uint8_t>& seen_columns) {
          if (is_expired(deadline)) {
            matching_cutoff = true;
            return false;
          }
          for (size_t col = 0; col < free_count; ++col) {
            if (is_expired(deadline)) {
              matching_cutoff = true;
              return false;
            }
            if (real_distance[row][col] < 0 ||
                seen_columns[col])
              continue;
            seen_columns[col] = 1;
            if (owner[col] < 0 ||
                augment(
                    static_cast<size_t>(owner[col]),
                    seen_columns)) {
              owner[col] = static_cast<int>(row);
              return true;
            }
            if (matching_cutoff) return false;
          }
          return false;
        };
    for (size_t row = 0; row < task_count; ++row) {
      if (is_expired(deadline)) {
        matching_cutoff = true;
        break;
      }
      std::vector<uint8_t> seen_columns(free_count, 0);
      if (augment(row, seen_columns))
        ++maximum_real_cardinality;
      if (matching_cutoff) break;
    }
    if (matching_cutoff) {
      out.telemetry.maximum_real_cardinality_time_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() -
              cardinality_started)
              .count();
      out.telemetry.maximum_real_cardinality_cutoff = true;
      stop_cutoff();
      return out;
    }
    out.telemetry.maximum_real_cardinality_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() -
            cardinality_started)
            .count();
    out.telemetry.maximum_real_cardinality =
        static_cast<long>(maximum_real_cardinality);
  }

  const size_t service_capacity =
      admission ==
              CandidateAdmission::KEEP_ALL_PENDING_ROWS
          ? maximum_real_cardinality
          : free_count;
  const size_t dummy_count =
      task_count > service_capacity
          ? task_count - service_capacity
          : 0;
  const size_t column_count = free_count + dummy_count;
  int priority_cutoff = std::numeric_limits<int>::min();
  if (admission ==
      CandidateAdmission::DROP_GLOBALLY_UNREACHABLE) {
    if (task_count > free_count)
      priority_cutoff =
          candidates[free_count - 1].priority;
  } else if (
      actionable_rows.size() > service_capacity &&
      service_capacity > 0) {
    priority_cutoff =
        candidates[
            actionable_rows[service_capacity - 1]]
            .priority;
  }
  out.telemetry.matrix_rows = static_cast<long>(task_count);
  out.telemetry.matrix_cols = static_cast<long>(column_count);

  uint64_t identity_fingerprint = rho_fingerprint_mix(0x52484f49ULL);
  rho_fingerprint_add(
      identity_fingerprint, static_cast<uint64_t>(mode));
  for (const int robot : free_robots)
    rho_fingerprint_add(
        identity_fingerprint,
        static_cast<uint64_t>(static_cast<uint32_t>(robot)));
  rho_fingerprint_add(identity_fingerprint, dummy_count);
  for (const auto& candidate : candidates) {
    rho_fingerprint_add(identity_fingerprint, candidate.key);
    rho_fingerprint_add(identity_fingerprint, candidate.id);
  }
  out.telemetry.column_identity_fingerprint =
      identity_fingerprint;
  out.telemetry.candidate_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - candidate_started)
          .count();

  const auto matrix_started = std::chrono::steady_clock::now();
  constexpr long long INF = kRhoDispatchInf;
  const long long switch_scale = (long long)free_count + 1;
  __int128 priority_sum = 0;
  if (admission ==
      CandidateAdmission::KEEP_ALL_PENDING_ROWS) {
    for (const size_t row : actionable_rows)
      priority_sum +=
          std::max(0, candidates[row].priority);
  } else {
    for (const auto& candidate : candidates)
      priority_sum += std::max(0, candidate.priority);
  }
  const long long priority_scale =
      rho_priority_lex_scale(priority_sum);
  const auto critical_tail = task_critical_tail_ticks(graph);
  const auto direct_target_candidate =
      [&](const RhoCandidate& candidate) {
        if (candidate.task_index < 0 ||
            candidate.task_index >= (int)graph.tasks.size() ||
            candidate.task_index >= (int)critical_tail.size() ||
            candidate.id.shelf.kind !=
                ShelfSelector::Kind::TARGET ||
            critical_tail[candidate.task_index] != 0)
          return false;
        const int target = candidate.id.shelf.value;
        if (target < 0 ||
            target >= (int)ins.target_goal_sets.size())
          return false;
        const auto& task = graph.tasks[candidate.task_index];
        const int endpoint = task.transfer.endpoint;
        const auto& goals = ins.target_goal_sets[target];
        if (!std::binary_search(
                goals.begin(), goals.end(), endpoint))
          return false;
        return std::any_of(
            task.roots.begin(), task.roots.end(),
            [&](const RootDemand& root) {
              return root.target == target &&
                     root.goal == endpoint;
            });
      };
  if (admission ==
      CandidateAdmission::KEEP_ALL_PENDING_ROWS) {
    out.telemetry.direct_target_phase =
        mode == DispatchMode::EXECUTE &&
        !actionable_rows.empty() &&
        std::all_of(
            actionable_rows.begin(), actionable_rows.end(),
            [&](size_t row) {
              return direct_target_candidate(candidates[row]);
            });
  } else {
    out.telemetry.direct_target_phase =
        mode == DispatchMode::EXECUTE &&
        std::all_of(
            candidates.begin(), candidates.end(),
            direct_target_candidate);
  }
  std::vector<std::vector<long long>> completion(
      task_count, std::vector<long long>(column_count, INF));
  std::vector<std::vector<long long>> cost(
      task_count, std::vector<long long>(column_count, INF));
  for (size_t row = 0; row < task_count; ++row) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    if (admission ==
            CandidateAdmission::KEEP_ALL_PENDING_ROWS &&
        !actionable[row]) {
      for (size_t col = free_count;
           col < column_count; ++col) {
        completion[row][col] = 0;
        cost[row][col] = 0;
      }
      continue;
    }
    const int task_index = candidates[row].task_index;
    const long long service =
        task_index >= 0 &&
                task_index < (int)graph.tasks.size()
            ? task_service_ticks(graph.tasks[task_index])
            : 1;
    const long long tail =
        task_index >= 0 &&
                task_index < (int)critical_tail.size()
            ? critical_tail[task_index]
            : 0;
    long long best_physical_completion = INF;
    long long best_real_approach = INF;
    for (size_t col = 0; col < free_count; ++col) {
      if (is_expired(deadline)) {
        stop_cutoff();
        return out;
      }
      const int robot = free_robots[col];
      const int distance = real_distance[row][col];
      if (distance < 0) continue;
      const bool switched =
          previous_rho_transfer_key != nullptr &&
                  robot <
                      (int)previous_rho_transfer_key->size() &&
                  (*previous_rho_transfer_key)[robot].has_value()
              ? *(*previous_rho_transfer_key)[robot] !=
                    candidates[row].key
              : previous_rho_task_id != nullptr &&
                    robot < (int)previous_rho_task_id->size() &&
                    (*previous_rho_task_id)[robot].has_value() &&
                    *(*previous_rho_task_id)[robot] !=
                        candidates[row].id;
      completion[row][col] = rho_checked_dispatch_add(
          rho_checked_dispatch_add(distance, service), tail);
      const __int128 physical_secondary =
          static_cast<__int128>(distance) * switch_scale +
          (switched ? 1 : 0);
      if (physical_secondary >= INF)
        throw std::overflow_error(
            "rho physical secondary cost overflow");
      cost[row][col] = rho_priority_lex_cost(
          static_cast<long long>(physical_secondary),
          priority_scale, 0);
      best_physical_completion =
          std::min(
              best_physical_completion,
              completion[row][col]);
      best_real_approach =
          std::min(best_real_approach, (long long)distance);
    }
    if (best_physical_completion < INF &&
        best_real_approach < INF) {
      bool continued_assignment = false;
      if (out.telemetry.direct_target_phase) {
        for (const int robot : free_robots) {
          if (previous_rho_transfer_key != nullptr &&
              robot <
                  (int)previous_rho_transfer_key->size() &&
              (*previous_rho_transfer_key)[robot].has_value()) {
            if (*(*previous_rho_transfer_key)[robot] ==
                candidates[row].key) {
              continued_assignment = true;
              break;
            }
            continue;
          }
          if (previous_rho_task_id != nullptr &&
              robot < (int)previous_rho_task_id->size() &&
              (*previous_rho_task_id)[robot].has_value() &&
              *(*previous_rho_task_id)[robot] ==
                  candidates[row].id) {
            continued_assignment = true;
            break;
          }
        }
      }
      const bool priority_frontier =
          out.telemetry.direct_target_phase &&
          (admission ==
                   CandidateAdmission::KEEP_ALL_PENDING_ROWS
               ? actionable_rows.size() > service_capacity
               : task_count > free_count) &&
          candidates[row].priority >= priority_cutoff;
      const long long defer_delay =
          out.telemetry.direct_target_phase
              ? rho_bottleneck_defer_delay(
                    service, priority_frontier,
                    continued_assignment)
              : std::max(1LL, service);
      for (size_t col = free_count; col < column_count; ++col) {
        if (is_expired(deadline)) {
          stop_cutoff();
          return out;
        }
        completion[row][col] =
            rho_checked_dispatch_add(
                best_physical_completion, defer_delay);
        const long long deferred_approach =
            rho_checked_dispatch_add(
                best_real_approach, defer_delay);
        const __int128 physical_secondary =
            static_cast<__int128>(deferred_approach) *
            switch_scale;
        if (physical_secondary >= INF)
          throw std::overflow_error(
              "rho deferred secondary cost overflow");
        cost[row][col] = rho_priority_lex_cost(
            static_cast<long long>(physical_secondary),
            priority_scale, candidates[row].priority);
      }
    }
  }

  uint64_t value_fingerprint = rho_fingerprint_mix(0x52484f56ULL);
  for (size_t row = 0; row < task_count; ++row) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    rho_fingerprint_add(
        value_fingerprint,
        static_cast<uint64_t>(
            static_cast<uint32_t>(candidates[row].priority)));
    for (size_t col = 0; col < column_count; ++col) {
      if (is_expired(deadline)) {
        stop_cutoff();
        return out;
      }
      rho_fingerprint_add(
          value_fingerprint,
          static_cast<uint64_t>(completion[row][col]));
      rho_fingerprint_add(
          value_fingerprint,
          static_cast<uint64_t>(cost[row][col]));
    }
  }
  out.telemetry.column_value_fingerprint = value_fingerprint;
  out.telemetry.matrix_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - matrix_started)
          .count();

  const auto bottleneck_started = std::chrono::steady_clock::now();
  const auto bottleneck =
      bottleneck_then_sum_assignment(
          completion, cost, deadline);
  out.telemetry.bottleneck_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - bottleneck_started)
          .count();
  if (bottleneck.cutoff) {
    stop_cutoff();
    return out;
  }
  if (!bottleneck.feasible) {
    stop_infeasible();
    return out;
  }
  for (size_t row = 0; row < task_count; ++row) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    for (size_t col = 0; col < column_count; ++col) {
      if (is_expired(deadline)) {
        stop_cutoff();
        return out;
      }
      if (completion[row][col] > bottleneck.bottleneck)
        cost[row][col] = INF;
    }
  }

  bool minimum_cost_cutoff = false;
  auto minimum_cost =
      [&](const std::vector<int>& rows,
          const std::vector<int>& cols) -> std::optional<long long> {
    if (is_expired(deadline)) {
      minimum_cost_cutoff = true;
      return std::nullopt;
    }
    if (rows.empty()) return 0;
    if (rows.size() > cols.size()) return std::nullopt;
    constexpr long double HINF = 1e60L;
    std::vector<std::vector<long double>> matrix(
        rows.size(), std::vector<long double>(cols.size(), HINF));
    for (size_t r = 0; r < rows.size(); ++r) {
      if (is_expired(deadline)) {
        minimum_cost_cutoff = true;
        return std::nullopt;
      }
      for (size_t c = 0; c < cols.size(); ++c) {
        if (is_expired(deadline)) {
          minimum_cost_cutoff = true;
          return std::nullopt;
        }
        if (cost[rows[r]][cols[c]] < INF)
          matrix[r][c] = (long double)cost[rows[r]][cols[c]];
      }
    }
    const auto assignment =
        hungarian_long_double(matrix, deadline);
    if (assignment.cutoff) {
      minimum_cost_cutoff = true;
      return std::nullopt;
    }
    if (!assignment.feasible) return std::nullopt;
    long long total = 0;
    for (size_t r = 0; r < rows.size(); ++r) {
      if (is_expired(deadline)) {
        minimum_cost_cutoff = true;
        return std::nullopt;
      }
      const int local_col = assignment.row_to_col[r];
      if (local_col < 0 ||
          cost[rows[r]][cols[local_col]] >= INF)
        return std::nullopt;
      total += cost[rows[r]][cols[local_col]];
    }
    return total;
  };

  std::vector<int> active_rows(task_count);
  std::iota(active_rows.begin(), active_rows.end(), 0);
  std::vector<int> active_cols(column_count);
  std::iota(active_cols.begin(), active_cols.end(), 0);
  const auto secondary_started = std::chrono::steady_clock::now();
  auto remaining_optimum = minimum_cost(active_rows, active_cols);
  out.telemetry.secondary_full_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - secondary_started)
          .count();
  if (minimum_cost_cutoff) {
    stop_cutoff();
    return out;
  }
  if (!remaining_optimum.has_value()) {
    stop_infeasible();
    return out;
  }

  const auto canonical_started = std::chrono::steady_clock::now();
  for (size_t real_col = 0; real_col < free_count; ++real_col) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    const auto col_it =
        std::find(active_cols.begin(), active_cols.end(), (int)real_col);
    if (col_it == active_cols.end()) continue;
    std::vector<int> row_options = active_rows;
    std::stable_sort(row_options.begin(), row_options.end(),
                     [&](int a, int b) {
                       if (candidates[a].id != candidates[b].id)
                         return candidates[a].id < candidates[b].id;
                       if (candidates[a].key != candidates[b].key)
                         return candidates[a].key < candidates[b].key;
                       return candidates[a].task_index <
                              candidates[b].task_index;
                     });
    bool fixed = false;
    for (const int row : row_options) {
      if (is_expired(deadline)) {
        stop_cutoff();
        return out;
      }
      if (cost[row][real_col] >= INF) continue;
      auto next_rows = active_rows;
      next_rows.erase(
          std::find(next_rows.begin(), next_rows.end(), row));
      auto next_cols = active_cols;
      next_cols.erase(
          std::find(next_cols.begin(), next_cols.end(),
                    (int)real_col));
      const auto suffix = minimum_cost(next_rows, next_cols);
      if (minimum_cost_cutoff) {
        stop_cutoff();
        return out;
      }
      if (!suffix.has_value() ||
          cost[row][real_col] + *suffix != *remaining_optimum)
        continue;
      const int robot = free_robots[real_col];
      out.rho_task_id[robot] = candidates[row].id;
      out.rho_transfer_key[robot] = candidates[row].key;
      out.rho_ready_index[robot] = candidates[row].task_index;
      active_rows = std::move(next_rows);
      active_cols = std::move(next_cols);
      *remaining_optimum -= cost[row][real_col];
      fixed = true;
      break;
    }
    if (fixed) continue;

    auto next_cols = active_cols;
    next_cols.erase(
        std::find(next_cols.begin(), next_cols.end(), (int)real_col));
    const auto suffix = minimum_cost(active_rows, next_cols);
    if (minimum_cost_cutoff) {
      stop_cutoff();
      return out;
    }
    if (suffix.has_value() && *suffix == *remaining_optimum) {
      active_cols = std::move(next_cols);
      continue;
    }
    throw std::logic_error(
        "match_ready_tasks: failed deterministic lexicographic refinement");
  }
  out.telemetry.canonical_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - canonical_started)
          .count();
  out.telemetry.real_assignments = static_cast<long>(
      std::count_if(
          out.rho_ready_index.begin(),
          out.rho_ready_index.end(),
          [](int task_index) { return task_index >= 0; }));
  if (admission ==
          CandidateAdmission::KEEP_ALL_PENDING_ROWS &&
      out.telemetry.real_assignments !=
          static_cast<long>(maximum_real_cardinality)) {
    stop_infeasible();
    return out;
  }
  return out;
}

}  // namespace carrier_detail
