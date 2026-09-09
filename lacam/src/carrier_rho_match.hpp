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

inline bool rho_incremental_same_model(
    const RhoIncrementalState& left,
    const RhoIncrementalState& right)
{
  return left.matrix_encoding_version ==
             right.matrix_encoding_version &&
         left.canonical_version == right.canonical_version &&
         left.objective_version == right.objective_version &&
         left.mode == right.mode &&
         left.admission == right.admission &&
         left.free_robots == right.free_robots &&
         left.candidate_task_indices ==
             right.candidate_task_indices &&
         left.candidate_priorities ==
             right.candidate_priorities &&
         left.candidate_ids == right.candidate_ids &&
         left.candidate_keys == right.candidate_keys &&
         left.task_count == right.task_count &&
         left.free_robot_count == right.free_robot_count &&
         left.dimension == right.dimension;
}

inline bool rho_incremental_score(
    const RhoIncrementalState& state, int row, int column,
    long long& score)
{
  if (row < 0 || row >= state.dimension ||
      column < 0 || column >= state.dimension ||
      row >=
          static_cast<int>(
              state.transposed_task_cost_rows.size()))
    return false;
  if (column >= state.task_count) {
    score = 0;
    return true;
  }
  const auto& values =
      state.transposed_task_cost_rows[row];
  if (values == nullptr ||
      column >= static_cast<int>(values->size()))
    return false;
  const long long cost = (*values)[column];
  if (cost < 0 || cost >= kRhoDispatchInf)
    return false;
  score = -cost;
  return true;
}

inline bool rho_incremental_matching_cost(
    const RhoIncrementalState& state, long long& total,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  total = 0;
  if (expired() ||
      !state.hungarian.structurally_valid())
    return false;
  __int128 exact_total = 0;
  for (int task = 0; task < state.task_count; ++task) {
    if (expired()) return false;
    const int row =
        state.hungarian.column_to_row[task];
    if (row < 0 || row >= state.dimension ||
        row >=
            static_cast<int>(
                state.transposed_task_cost_rows.size()) ||
        state.transposed_task_cost_rows[row] == nullptr ||
        task >=
            static_cast<int>(
                state.transposed_task_cost_rows[row]->size()))
      return false;
    const long long cost =
        (*state.transposed_task_cost_rows[row])[task];
    if (cost < 0 || cost >= kRhoDispatchInf)
      return false;
    exact_total += cost;
    if (exact_total >
        std::numeric_limits<long long>::max())
      return false;
  }
  total = static_cast<long long>(exact_total);
  return true;
}

inline bool rho_incremental_state_valid(
    const RhoIncrementalState& state,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired() || !state.valid ||
      state.matrix_encoding_version !=
          RhoIncrementalState::MATRIX_ENCODING_VERSION ||
      state.canonical_version !=
          RhoIncrementalState::CANONICAL_VERSION ||
      state.objective_version !=
          RhoObjectiveVersion::
              BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2 ||
      state.task_count <= 0 ||
      state.free_robot_count < 0 ||
      state.dimension < state.task_count ||
      state.free_robot_count > state.dimension ||
      state.free_robots.size() !=
          static_cast<size_t>(state.free_robot_count) ||
      state.candidate_task_indices.size() !=
          static_cast<size_t>(state.task_count) ||
      state.candidate_priorities.size() !=
          static_cast<size_t>(state.task_count) ||
      state.candidate_ids.size() !=
          static_cast<size_t>(state.task_count) ||
      state.candidate_keys.size() !=
          static_cast<size_t>(state.task_count) ||
      state.transposed_task_cost_rows.size() !=
          static_cast<size_t>(state.dimension) ||
      state.hungarian.dimension != state.dimension ||
      !state.hungarian.structurally_valid())
    return false;

  for (const auto& row :
       state.transposed_task_cost_rows) {
    if (expired()) return false;
    if (row == nullptr ||
        row->size() !=
            static_cast<size_t>(state.task_count))
      return false;
    for (const long long cost : *row) {
      if (expired()) return false;
      if (cost < 0)
        return false;
    }
  }

  for (int row = 0; row < state.dimension; ++row) {
    if (expired()) return false;
    for (int column = 0;
         column < state.dimension; ++column) {
      if (expired()) return false;
      long long score = 0;
      if (!rho_incremental_score(
              state, row, column, score))
        continue;
      const __int128 reduced =
          static_cast<__int128>(
              state.hungarian.row_dual[row]) +
          state.hungarian.column_dual[column] -
          score;
      if (reduced < 0)
        return false;
      if (state.hungarian.row_to_column[row] ==
              column &&
          reduced != 0)
        return false;
    }
    long long matched_score = 0;
    if (!rho_incremental_score(
            state, row,
            state.hungarian.row_to_column[row],
            matched_score))
      return false;
  }

  long long matching_cost = 0;
  bool matching_cutoff = false;
  const bool matching_valid =
      rho_incremental_matching_cost(
          state, matching_cost, deadline,
          &matching_cutoff);
  if (matching_cutoff && cutoff != nullptr)
    *cutoff = true;
  return matching_valid &&
         matching_cost == state.secondary_cost;
}

enum class RhoTightMatchingStatus : uint8_t {
  FEASIBLE = 0,
  INFEASIBLE = 1,
  CUTOFF = 2,
};

inline bool rho_incremental_tight_edge(
    const RhoIncrementalState& state, int row, int column)
{
  long long score = 0;
  if (!rho_incremental_score(
          state, row, column, score))
    return false;
  return static_cast<__int128>(
             state.hungarian.row_dual[row]) +
             state.hungarian.column_dual[column] -
             score ==
         0;
}

inline RhoTightMatchingStatus
rho_tight_perfect_matching(
    const RhoIncrementalState& state,
    const std::vector<uint8_t>& active_rows,
    const std::vector<uint8_t>& active_columns,
    const Deadline* deadline,
    std::vector<int>* row_to_column = nullptr)
{
  if (is_expired(deadline))
    return RhoTightMatchingStatus::CUTOFF;
  if (active_rows.size() !=
          static_cast<size_t>(state.dimension) ||
      active_columns.size() !=
          static_cast<size_t>(state.dimension))
    return RhoTightMatchingStatus::INFEASIBLE;
  const size_t row_count =
      static_cast<size_t>(
          std::count(
              active_rows.begin(), active_rows.end(), 1));
  if (row_count !=
      static_cast<size_t>(
          std::count(
              active_columns.begin(),
              active_columns.end(), 1)))
    return RhoTightMatchingStatus::INFEASIBLE;

  bool cutoff = false;
  std::vector<int> owner(state.dimension, -1);
  std::function<bool(int, std::vector<uint8_t>&)> augment =
      [&](int row, std::vector<uint8_t>& seen_columns) {
        if (is_expired(deadline)) {
          cutoff = true;
          return false;
        }
        for (int column = 0;
             column < state.dimension; ++column) {
          if (is_expired(deadline)) {
            cutoff = true;
            return false;
          }
          if (!active_columns[column] ||
              seen_columns[column] ||
              !rho_incremental_tight_edge(
                  state, row, column))
            continue;
          seen_columns[column] = 1;
          if (owner[column] < 0 ||
              augment(
                  owner[column], seen_columns)) {
            owner[column] = row;
            return true;
          }
          if (cutoff) return false;
        }
        return false;
      };

  for (int row = 0; row < state.dimension; ++row) {
    if (!active_rows[row]) continue;
    std::vector<uint8_t> seen_columns(
        state.dimension, 0);
    if (!augment(row, seen_columns))
      return cutoff
                 ? RhoTightMatchingStatus::CUTOFF
                 : RhoTightMatchingStatus::INFEASIBLE;
  }
  if (row_to_column != nullptr) {
    row_to_column->assign(state.dimension, -1);
    for (int column = 0;
         column < state.dimension; ++column)
      if (owner[column] >= 0)
        (*row_to_column)[owner[column]] = column;
  }
  return RhoTightMatchingStatus::FEASIBLE;
}

inline RhoTightMatchingStatus
rho_canonicalize_incremental_matching(
    RhoIncrementalState& state, const Deadline* deadline)
{
  std::vector<int> task_columns(state.task_count);
  std::iota(
      task_columns.begin(), task_columns.end(), 0);
  std::stable_sort(
      task_columns.begin(), task_columns.end(),
      [&](int left, int right) {
        if (state.candidate_ids[left] !=
            state.candidate_ids[right])
          return state.candidate_ids[left] <
                 state.candidate_ids[right];
        if (state.candidate_keys[left] !=
            state.candidate_keys[right])
          return state.candidate_keys[left] <
                 state.candidate_keys[right];
        return state.candidate_task_indices[left] <
               state.candidate_task_indices[right];
      });

  std::vector<uint8_t> active_rows(
      state.dimension, 1);
  std::vector<uint8_t> active_columns(
      state.dimension, 1);
  std::vector<int> canonical(
      state.dimension, -1);
  for (int robot_row = 0;
       robot_row < state.free_robot_count;
       ++robot_row) {
    if (is_expired(deadline))
      return RhoTightMatchingStatus::CUTOFF;
    bool fixed = false;
    for (const int task_column : task_columns) {
      if (is_expired(deadline))
        return RhoTightMatchingStatus::CUTOFF;
      if (!active_columns[task_column] ||
          !rho_incremental_tight_edge(
              state, robot_row, task_column))
        continue;
      active_rows[robot_row] = 0;
      active_columns[task_column] = 0;
      const auto status =
          rho_tight_perfect_matching(
              state, active_rows, active_columns,
              deadline);
      if (status == RhoTightMatchingStatus::CUTOFF)
        return status;
      if (status ==
          RhoTightMatchingStatus::FEASIBLE) {
        canonical[robot_row] = task_column;
        fixed = true;
        break;
      }
      active_rows[robot_row] = 1;
      active_columns[task_column] = 1;
    }
    if (fixed) continue;

    for (int padding_column = state.task_count;
         padding_column < state.dimension;
         ++padding_column) {
      if (is_expired(deadline))
        return RhoTightMatchingStatus::CUTOFF;
      if (!active_columns[padding_column] ||
          !rho_incremental_tight_edge(
              state, robot_row, padding_column))
        continue;
      active_rows[robot_row] = 0;
      active_columns[padding_column] = 0;
      const auto status =
          rho_tight_perfect_matching(
              state, active_rows, active_columns,
              deadline);
      if (status == RhoTightMatchingStatus::CUTOFF)
        return status;
      if (status ==
          RhoTightMatchingStatus::FEASIBLE) {
        canonical[robot_row] = padding_column;
        fixed = true;
        break;
      }
      active_rows[robot_row] = 1;
      active_columns[padding_column] = 1;
    }
    if (!fixed)
      return RhoTightMatchingStatus::INFEASIBLE;
  }

  std::vector<int> suffix;
  const auto suffix_status =
      rho_tight_perfect_matching(
          state, active_rows, active_columns,
          deadline, &suffix);
  if (suffix_status !=
      RhoTightMatchingStatus::FEASIBLE)
    return suffix_status;
  for (int row = 0; row < state.dimension; ++row)
    if (active_rows[row])
      canonical[row] = suffix[row];

  std::vector<int> reverse(
      state.dimension, -1);
  for (int row = 0; row < state.dimension; ++row) {
    if (is_expired(deadline))
      return RhoTightMatchingStatus::CUTOFF;
    const int column = canonical[row];
    if (column < 0 || column >= state.dimension ||
        reverse[column] >= 0 ||
        !rho_incremental_tight_edge(
            state, row, column))
      return RhoTightMatchingStatus::INFEASIBLE;
    reverse[column] = row;
  }
  state.hungarian.row_to_column =
      std::move(canonical);
  state.hungarian.column_to_row =
      std::move(reverse);
  return RhoTightMatchingStatus::FEASIBLE;
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
    const Deadline* deadline = nullptr,
    const RhoIncrementalState* previous_rho_state = nullptr)
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
  out.telemetry.bottleneck = bottleneck.bottleneck;
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

  if (task_count >
          static_cast<size_t>(
              std::numeric_limits<int>::max()) ||
      column_count >
          static_cast<size_t>(
              std::numeric_limits<int>::max()))
    throw std::overflow_error(
        "rho incremental assignment dimension overflow");

  RhoIncrementalState state;
  state.mode = mode;
  state.admission = admission;
  state.free_robots = free_robots;
  state.task_count = static_cast<int>(task_count);
  state.free_robot_count =
      static_cast<int>(free_count);
  state.dimension =
      static_cast<int>(column_count);
  state.bottleneck = bottleneck.bottleneck;
  state.candidate_task_indices.reserve(task_count);
  state.candidate_priorities.reserve(task_count);
  state.candidate_ids.reserve(task_count);
  state.candidate_keys.reserve(task_count);
  for (const auto& candidate : candidates) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    state.candidate_task_indices.push_back(
        candidate.task_index);
    state.candidate_priorities.push_back(
        candidate.priority);
    state.candidate_ids.push_back(candidate.id);
    state.candidate_keys.push_back(candidate.key);
  }

  const bool same_model =
      previous_rho_state != nullptr &&
      rho_incremental_same_model(
          *previous_rho_state, state);
  if (is_expired(deadline)) {
    stop_cutoff();
    return out;
  }
  state.transposed_task_cost_rows.reserve(
      column_count);
  std::vector<int> changed_rows;
  for (size_t column = 0;
       column < column_count; ++column) {
    if (is_expired(deadline)) {
      stop_cutoff();
      return out;
    }
    auto values =
        std::make_shared<std::vector<long long>>();
    values->reserve(task_count);
    for (size_t row = 0; row < task_count; ++row) {
      if (is_expired(deadline)) {
        stop_cutoff();
        return out;
      }
      values->push_back(cost[row][column]);
    }
    bool row_same =
        same_model &&
        column <
            previous_rho_state
                ->transposed_task_cost_rows.size() &&
        previous_rho_state
                ->transposed_task_cost_rows[column] != nullptr &&
        previous_rho_state
                ->transposed_task_cost_rows[column]
                ->size() == values->size();
    if (row_same) {
      for (size_t task = 0;
           task < values->size(); ++task) {
        if (is_expired(deadline)) {
          stop_cutoff();
          return out;
        }
        if ((*previous_rho_state
                  ->transposed_task_cost_rows[column])[task] !=
            (*values)[task]) {
          row_same = false;
          break;
        }
      }
    }
    if (row_same) {
      state.transposed_task_cost_rows.push_back(
          previous_rho_state
              ->transposed_task_cost_rows[column]);
    } else {
      state.transposed_task_cost_rows.push_back(
          std::move(values));
      if (same_model)
        changed_rows.push_back(
            static_cast<int>(column));
    }
  }
  out.telemetry.incremental_changed_rows =
      static_cast<long>(changed_rows.size());

  using IncrementalStatus =
      tapf_assignment_detail::
          IncrementalHungarianStatus;
  const auto score =
      [&](int row, int column, long long& value) {
        return rho_incremental_score(
            state, row, column, value);
      };
  const auto stop = [&]() {
    return is_expired(deadline);
  };
  const auto full_solve =
      [&](RhoIncrementalFallbackReason reason) {
        out.telemetry.incremental_fallback = reason;
        ++out.telemetry.incremental_full_solves;
        state.hungarian.init(state.dimension);
        const auto started =
            std::chrono::steady_clock::now();
        const auto status =
            state.hungarian.solve_full(
            score,
            tapf_assignment_detail::
                ExactIncrementalHungarianArithmetic<
                    long long>{},
            stop);
        out.telemetry.secondary_full_time_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() -
                started)
                .count();
        return status;
      };

  bool parent_validation_cutoff = false;
  const bool parent_state_valid =
      same_model &&
      rho_incremental_state_valid(
          *previous_rho_state, deadline,
          &parent_validation_cutoff);
  if (parent_validation_cutoff) {
    stop_cutoff();
    return out;
  }

  IncrementalStatus secondary_status =
      IncrementalStatus::OK;
  bool matching_is_already_canonical = false;
  if (previous_rho_state == nullptr) {
    secondary_status =
        full_solve(
            RhoIncrementalFallbackReason::
                NO_PARENT_STATE);
  } else if (!same_model) {
    secondary_status =
        full_solve(
            RhoIncrementalFallbackReason::
                COLUMN_MODEL_CHANGED);
  } else if (!parent_state_valid) {
    secondary_status =
        full_solve(
            RhoIncrementalFallbackReason::
                INVALID_PARENT_STATE);
  } else {
    out.telemetry.incremental_fallback =
        RhoIncrementalFallbackReason::NONE;
    state.hungarian =
        previous_rho_state->hungarian;
    if (changed_rows.empty()) {
      ++out.telemetry.incremental_zero_row_reuses;
      matching_is_already_canonical = true;
    } else {
      ++out.telemetry.incremental_repairs;
      const auto repair_started =
          std::chrono::steady_clock::now();
      secondary_status =
          state.hungarian.repair_rows(
              state.dimension, changed_rows, score,
              tapf_assignment_detail::
                  ExactIncrementalHungarianArithmetic<
                      long long>{},
              stop);
      out.telemetry.secondary_repair_time_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() -
              repair_started)
              .count();
      if (secondary_status ==
              IncrementalStatus::INVALID_DUAL ||
          secondary_status ==
              IncrementalStatus::INFEASIBLE) {
        secondary_status =
            full_solve(
                RhoIncrementalFallbackReason::
                    INVALID_PARENT_STATE);
      }
    }
  }
  if (secondary_status ==
      IncrementalStatus::CUTOFF) {
    stop_cutoff();
    return out;
  }
  if (secondary_status !=
      IncrementalStatus::OK) {
    stop_infeasible();
    return out;
  }

  const auto canonical_started = std::chrono::steady_clock::now();
  if (!matching_is_already_canonical) {
    const auto canonical_status =
        rho_canonicalize_incremental_matching(
            state, deadline);
    if (canonical_status ==
        RhoTightMatchingStatus::CUTOFF) {
      stop_cutoff();
      return out;
    }
    if (canonical_status !=
        RhoTightMatchingStatus::FEASIBLE)
      throw std::logic_error(
          "match_ready_tasks: failed exact tight-edge canonical refinement");
  }
  out.telemetry.canonical_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - canonical_started)
          .count();

  long long secondary_cost = 0;
  bool matching_cost_cutoff = false;
  if (!rho_incremental_matching_cost(
          state, secondary_cost, deadline,
          &matching_cost_cutoff)) {
    if (matching_cost_cutoff) {
      stop_cutoff();
      return out;
    }
    throw std::logic_error(
        "match_ready_tasks: invalid exact rho matching");
  }
  if (secondary_cost != bottleneck.secondary_cost)
    throw std::logic_error(
        "match_ready_tasks: incremental rho secondary optimum mismatch");
  state.secondary_cost = secondary_cost;
  state.valid = true;
  bool saved_state_cutoff = false;
  if (!rho_incremental_state_valid(
          state, deadline, &saved_state_cutoff)) {
    if (saved_state_cutoff) {
      stop_cutoff();
      return out;
    }
    throw std::logic_error(
        "match_ready_tasks: invalid saved incremental rho state");
  }
  out.telemetry.secondary_cost = secondary_cost;
  out.rho_state = state;

  for (size_t real_row = 0;
       real_row < free_count; ++real_row) {
    if (is_expired(deadline)) {
      stop_cutoff();
      out.rho_state.reset();
      return out;
    }
    const int task_column =
        state.hungarian.row_to_column[real_row];
    if (task_column < 0 ||
        task_column >= static_cast<int>(task_count))
      continue;
    const int robot = free_robots[real_row];
    out.rho_task_id[robot] =
        candidates[task_column].id;
    out.rho_transfer_key[robot] =
        candidates[task_column].key;
    out.rho_ready_index[robot] =
        candidates[task_column].task_index;
  }
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
