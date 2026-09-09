// Part of carrier_guidance.hpp (internal, src/): Pair-cost declarations, root-goal commitments, tau guide + lower bounds.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_assignment.hpp"

namespace carrier_detail {
struct StorageTransferTopology;
struct VacancyPotentialCache;
struct PairCostDependencyContext;

inline PairPlan pair_cost(const DDInstance& ins, const UpperSignature& upper,
                          int target, int goal, DDDistCache& upper_wall,
                          const StorageTransferTopology& storage_topology,
                          double alpha, double gamma, double delta,
                          const Deadline* deadline = nullptr,
                          VacancyPotentialCache* potential_cache = nullptr,
                          PairCostDependencyContext*
                              dependency_context = nullptr,
                          PairCostDependency* dependency = nullptr);
inline PairPlan pair_cost_prefix_lower_bound(
    const DDInstance& ins, const UpperSignature& upper, int target,
    int goal, DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    double alpha, double gamma, double delta, int prefix_cap,
    const Deadline* deadline = nullptr,
    VacancyPotentialCache* potential_cache = nullptr,
    PairCostDependencyContext*
        dependency_context = nullptr,
    PairCostDependency* dependency = nullptr);

inline PairCostTable build_pair_cost_table(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    double alpha, double gamma, double delta)
{
  PairCostTable table(ins.n_targets());
  for (size_t b = 0; b < ins.n_targets(); ++b)
    for (const int goal : ins.target_goal_sets[b])
      table[b].push_back(PairCostEntry{
          goal, pair_cost(
                    ins, upper, (int)b, goal, upper_wall,
                    storage_topology, alpha, gamma, delta)});
  return table;
}

struct RootGoalCommitmentMask {
  std::vector<int> fixed_goal;
  std::map<int, int> goal_owner;
};

inline RootGoalCommitmentMask make_root_goal_commitment_mask(
    const DDInstance& ins,
    const RootGoalCommitment* commitments)
{
  RootGoalCommitmentMask out;
  out.fixed_goal.assign(ins.n_targets(), -1);
  if (commitments == nullptr) return out;
  for (const auto& [target, goal] : *commitments) {
    if (target < 0 ||
        target >= static_cast<int>(ins.n_targets()) ||
        !eligible_goal(ins, target, goal))
      throw std::invalid_argument(
          "invalid root-goal commitment");
    const auto owner = out.goal_owner.emplace(goal, target);
    if (!owner.second && owner.first->second != target)
      throw std::invalid_argument(
          "root-goal commitments must be injective");
    out.fixed_goal[target] = goal;
  }
  return out;
}

inline bool root_goal_commitment_allows(
    const RootGoalCommitmentMask& mask,
    size_t target, int goal)
{
  if (target >= mask.fixed_goal.size()) return false;
  if (mask.fixed_goal[target] >= 0)
    return mask.fixed_goal[target] == goal;
  const auto owner = mask.goal_owner.find(goal);
  return owner == mask.goal_owner.end() ||
         owner->second == static_cast<int>(target);
}

inline std::vector<int> solve_tau_guide_impl(
    const DDInstance& ins, const UpperSignature& upper,
    const PairCostTable& table,
    const RootGoalCommitment* commitments,
    const Deadline* deadline = nullptr,
    bool* cutoff_out = nullptr)
{
  if (cutoff_out != nullptr) *cutoff_out = false;
  const auto cutoff = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff_out != nullptr) *cutoff_out = true;
    return true;
  };
  const size_t n = ins.n_targets();
  std::vector<int> tau(n, -1);
  const auto commitment_mask =
      make_root_goal_commitment_mask(ins, commitments);
  if (cutoff()) return {};
  if (n == 0) return tau;
  std::vector<int> goals;
  for (const auto& set : ins.target_goal_sets) {
    if (cutoff()) return {};
    goals.insert(goals.end(), set.begin(), set.end());
  }
  std::sort(goals.begin(), goals.end());
  goals.erase(std::unique(goals.begin(), goals.end()), goals.end());
  std::vector<std::vector<LexAssignmentCost>> lex_cost(
      n, std::vector<LexAssignmentCost>(
             goals.size(), LexAssignmentCost::infinity()));
  for (size_t b = 0; b < n; ++b) {
    if (cutoff()) return {};
    for (const auto& entry : table[b]) {
      if (cutoff()) return {};
      if (!root_goal_commitment_allows(
              commitment_mask, b, entry.goal))
        continue;
      const auto it = std::lower_bound(goals.begin(), goals.end(), entry.goal);
      if (it == goals.end() || *it != entry.goal ||
          !std::isfinite(entry.plan.estimated_cost))
        continue;
      const int goal_index = (int)(it - goals.begin());
      const bool moved_away =
          std::binary_search(ins.target_goal_sets[b].begin(),
                             ins.target_goal_sets[b].end(),
                             upper.target_pos[b]) &&
          upper.target_pos[b] != goals[goal_index];
      lex_cost[b][goal_index] = LexAssignmentCost{
          (long double)entry.plan.estimated_cost,
          moved_away ? 1 : 0,
          false};
    }
  }
  const auto optimum =
      hungarian_lexicographic(lex_cost, deadline);
  if (optimum.cutoff) {
    if (cutoff_out != nullptr) *cutoff_out = true;
    return {};
  }
  if (!optimum.feasible)
    throw std::logic_error(
        "solve_tau_guide: infeasible eligible-goal matching");

  // Exact tertiary assignment-vector tie: fix targets in id order to the
  // smallest goal that still admits the exact primary+secondary optimum.
  std::vector<uint8_t> used(goals.size(), 0);
  LexAssignmentCost prefix_cost;
  for (size_t b = 0; b < n; ++b) {
    if (cutoff()) return {};
    bool fixed = false;
    for (size_t j = 0; j < goals.size(); ++j) {
      if (cutoff()) return {};
      if (used[j] || lex_cost[b][j].infinite)
        continue;
      const size_t remaining_rows = n - b - 1;
      bool feasible = true;
      LexAssignmentCost remaining_opt;
      if (remaining_rows > 0) {
        std::vector<int> remaining_cols;
        for (size_t c = 0; c < goals.size(); ++c)
          if (!used[c] && c != j) remaining_cols.push_back((int)c);
        if (remaining_cols.size() < remaining_rows) {
          feasible = false;
        } else {
          std::vector<std::vector<LexAssignmentCost>> rem(
              remaining_rows,
              std::vector<LexAssignmentCost>(
                  remaining_cols.size(),
                  LexAssignmentCost::infinity()));
          for (size_t rr = 0; rr < remaining_rows; ++rr)
            for (size_t cc = 0; cc < remaining_cols.size(); ++cc)
              rem[rr][cc] =
                  lex_cost[b + 1 + rr][remaining_cols[cc]];
          const auto rem_assignment =
              hungarian_lexicographic(rem, deadline);
          if (rem_assignment.cutoff) {
            if (cutoff_out != nullptr) *cutoff_out = true;
            return {};
          }
          feasible = rem_assignment.feasible;
          if (feasible) remaining_opt = rem_assignment.cost;
        }
      }
      const LexAssignmentCost candidate_total =
          prefix_cost + lex_cost[b][j] + remaining_opt;
      if (!feasible || candidate_total != optimum.cost)
        continue;
      tau[b] = goals[j];
      used[j] = 1;
      prefix_cost += lex_cost[b][j];
      fixed = true;
      break;
    }
    if (!fixed)
      throw std::logic_error(
          "solve_tau_guide: failed exact assignment-vector tie");
  }
  return tau;
}

inline std::vector<int> solve_tau_guide(
    const DDInstance& ins, const UpperSignature& upper,
    const PairCostTable& table,
    const Deadline* deadline = nullptr,
    bool* cutoff_out = nullptr)
{
  return solve_tau_guide_impl(
      ins, upper, table, nullptr, deadline, cutoff_out);
}

inline std::vector<int> solve_tau_guide_with_commitments(
    const DDInstance& ins, const UpperSignature& upper,
    const PairCostTable& table,
    const RootGoalCommitment& commitments,
    const Deadline* deadline = nullptr,
    bool* cutoff_out = nullptr)
{
  return solve_tau_guide_impl(
      ins, upper, table, &commitments, deadline, cutoff_out);
}

struct LazyPairAssignment {
  PairCostTable table;
  std::vector<int> tau;
  PairAssignmentHungarianState hungarian_state;
  bool cutoff = false;
  long evaluated_edges = 0;
  long total_edges = 0;
  long reused_edges = 0;
  long prefix_refinements = 0;
  long hungarian_full_solves = 0;
  long hungarian_row_repairs = 0;
  long hungarian_forced_repairs = 0;
  long rollout_work_steps = 0;
  long rollout_truncations = 0;
  long rollout_stalls = 0;
};

inline void mark_pair_dependency_cell(
    PairCostDependency& dependency, size_t cell_count, int cell)
{
  if (cell < 0 || static_cast<size_t>(cell) >= cell_count) {
    dependency.complete = false;
    return;
  }
  const size_t word_count = (cell_count + 63) / 64;
  if (dependency.cells.size() != word_count)
    dependency.cells.assign(word_count, 0);
  dependency.cells[cell / 64] |=
      uint64_t{1} << (cell % 64);
}

inline PairPlan pair_cost_cheap_lower_bound(
    const DDInstance& ins, const UpperSignature& upper,
    int target, int goal, DDDistCache& upper_wall,
    double alpha, double gamma,
    PairCostDependency* dependency = nullptr)
{
  PairPlan out;
  out.exact = false;
  out.bound_stage = PairBoundStage::CHEAP_BOUND;
  if (dependency != nullptr) {
    dependency->cells.assign(
        (ins.grid.size() + 63) / 64, 0);
    dependency->vacancy_removal_cells.assign(
        dependency->cells.size(), 0);
    dependency->vacancy_thresholds.clear();
    dependency->complete = true;
  }
  if (target < 0 ||
      target >= static_cast<int>(upper.target_pos.size()) ||
      !eligible_goal(ins, target, goal)) {
    out.estimated_cost =
        std::numeric_limits<double>::infinity();
    out.stalled = true;
    out.exact = true;
    out.bound_stage = PairBoundStage::EXACT;
    return out;
  }

  const int position = upper.target_pos[target];
  if (dependency != nullptr)
    mark_pair_dependency_cell(
        *dependency, ins.grid.size(), position);
  const int distance = upper_wall.dist(goal, position);
  out.direct_distance =
      distance >= INT_MAX / 4 ? INT_MAX : distance;
  if (distance >= INT_MAX / 4) {
    out.estimated_cost =
        std::numeric_limits<double>::infinity();
    out.stalled = true;
    out.exact = true;
    out.bound_stage = PairBoundStage::EXACT;
  } else if (position == goal) {
    out.reached_goal = true;
    out.exact = true;
    out.bound_stage = PairBoundStage::EXACT;
  } else {
    out.estimated_cost =
        alpha * static_cast<double>(distance) +
        2.0 * gamma;
  }
  return out;
}

inline std::vector<uint64_t> upper_signature_changed_cells(
    const DDInstance& ins, const UpperSignature& previous,
    const UpperSignature& current)
{
  const size_t cell_count =
      static_cast<size_t>(ins.grid.size());
  std::vector<uint64_t> changed(
      (cell_count + 63) / 64, 0);
  const auto mark = [&](int cell) {
    if (cell >= 0 &&
        static_cast<size_t>(cell) < cell_count)
      changed[cell / 64] |=
          uint64_t{1} << (cell % 64);
  };
  if (previous.target_pos.size() !=
      current.target_pos.size()) {
    std::fill(
        changed.begin(), changed.end(),
        std::numeric_limits<uint64_t>::max());
    return changed;
  }
  for (size_t target = 0;
       target < current.target_pos.size(); ++target) {
    if (previous.target_pos[target] ==
        current.target_pos[target])
      continue;
    mark(previous.target_pos[target]);
    mark(current.target_pos[target]);
  }

  size_t previous_index = 0;
  size_t current_index = 0;
  while (previous_index < previous.anon_pos.size() ||
         current_index < current.anon_pos.size()) {
    if (current_index == current.anon_pos.size() ||
        (previous_index < previous.anon_pos.size() &&
         previous.anon_pos[previous_index] <
             current.anon_pos[current_index])) {
      mark(previous.anon_pos[previous_index++]);
    } else if (
        previous_index == previous.anon_pos.size() ||
        current.anon_pos[current_index] <
            previous.anon_pos[previous_index]) {
      mark(current.anon_pos[current_index++]);
    } else {
      ++previous_index;
      ++current_index;
    }
  }
  return changed;
}

inline bool pair_dependency_intersects(
    const PairCostDependency& dependency,
    const std::vector<uint64_t>& changed)
{
  if (!dependency.complete ||
      dependency.cells.size() != changed.size())
    return true;
  for (size_t word = 0; word < changed.size(); ++word)
    if ((dependency.cells[word] & changed[word]) != 0)
      return true;
  return false;
}

struct PairUpperSignatureDelta {
  std::vector<uint64_t> changed_cells;
  std::vector<uint64_t> vacancy_additions;
  std::vector<uint64_t> vacancy_removals;
};

inline PairUpperSignatureDelta pair_upper_signature_delta(
    const DDInstance& ins, const UpperSignature& previous,
    const UpperSignature& current)
{
  PairUpperSignatureDelta out;
  out.changed_cells =
      upper_signature_changed_cells(
          ins, previous, current);
  const size_t word_count =
      (static_cast<size_t>(ins.grid.size()) + 63) / 64;
  out.vacancy_additions.assign(word_count, 0);
  out.vacancy_removals.assign(word_count, 0);
  const auto previous_occupied =
      upper_occupancy_bitmap(ins, previous);
  const auto current_occupied =
      upper_occupancy_bitmap(ins, current);
  for (int cell = 0; cell < ins.grid.size(); ++cell) {
    if (!ins.can_store_shelf(cell) ||
        previous_occupied[cell] ==
            current_occupied[cell])
      continue;
    auto& changed =
        previous_occupied[cell]
            ? out.vacancy_additions
            : out.vacancy_removals;
    changed[cell / 64] |=
        uint64_t{1} << (cell % 64);
  }
  return out;
}

inline bool pair_dependency_intersects(
    const DDInstance& ins,
    const PairCostDependency& dependency,
    const PairUpperSignatureDelta& delta,
    const StorageTransferTopology& storage_topology,
    PairCostDependencyContext* dependency_context,
    const Deadline* deadline);

// Exact lazy assignment certificate.
//
// Every eligible edge starts at a cheap geometric lower bound L0(e), may
// advance to the existing eight-step prefix lower bound L1(e), and finally
// to exact deterministic PairCost C(e).  Both L0(e) and L1(e) are <= C(e),
// so the mixed matrix remains a lower-bound matrix throughout.  Each loop
// advances every edge selected by the current Hungarian solution until the
// selected assignment is exact.  For every still-lower-bound edge e,
// forcing e and solving the remaining injective assignment gives F_L(e), a
// lower bound on every exact assignment containing e.  We advance e by one
// stage on F_L(e) <= C* (equality included).  At termination every
// unrefined edge has F_L(e) > C*, hence no primary-optimal assignment can
// contain it; all primary-optimal edges are exact.  The exact secondary
// moved-away count and tertiary assignment-vector tie can then be solved on
// this mixed table without changing the fully evaluated result.
//
// PairPlan::exact=false remains only a branch-and-bound certificate.  It
// must never feed priorities, rollout diagnostics, or any consumer that
// interprets stalled/truncated/steps as a completed PairCost evaluation.
inline LazyPairAssignment build_lazy_pair_cost_assignment(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    double alpha, double gamma, double delta,
    const Deadline* deadline = nullptr,
    VacancyPotentialCache* potential_cache = nullptr,
    const RootGoalCommitment* commitments = nullptr,
    PairCostDependencyContext*
        dependency_context = nullptr,
    const UpperSignature* previous_upper = nullptr,
    const PairCostTable* previous_table = nullptr,
    const PairAssignmentHungarianState*
        previous_hungarian_state = nullptr,
    const RootGoalCommitment*
        previous_commitments = nullptr)
{
  LazyPairAssignment out;
  const auto cutoff = [&]() {
    if (!is_expired(deadline)) return false;
    out.cutoff = true;
    out.tau.clear();
    return true;
  };
  if (cutoff()) return out;
  const size_t target_count = ins.n_targets();
  const auto commitment_mask =
      make_root_goal_commitment_mask(ins, commitments);
  const auto previous_commitment_mask =
      make_root_goal_commitment_mask(
          ins, previous_commitments);
  out.table.resize(target_count);
  if (target_count == 0) return out;

  std::vector<int> goals;
  for (const auto& set : ins.target_goal_sets) {
    if (cutoff()) return out;
    goals.insert(goals.end(), set.begin(), set.end());
  }
  std::sort(goals.begin(), goals.end());
  goals.erase(std::unique(goals.begin(), goals.end()), goals.end());
  constexpr long double INF = 1e60L;
  std::vector<std::vector<long double>> cost(
      target_count, std::vector<long double>(goals.size(), INF));
  std::vector<std::vector<int>> entry_index(
      target_count, std::vector<int>(goals.size(), -1));
  const bool has_previous =
      previous_upper != nullptr &&
      previous_table != nullptr &&
      previous_table->size() == target_count;
  const bool has_previous_hungarian =
      has_previous &&
      previous_hungarian_state != nullptr &&
      previous_hungarian_state->valid &&
      previous_hungarian_state->row_count ==
          static_cast<int>(target_count) &&
      previous_hungarian_state->column_count ==
          static_cast<int>(goals.size());
  std::vector<uint8_t> assignment_row_changed(
      target_count,
      has_previous_hungarian ? 0 : 1);
  if (has_previous_hungarian) {
    for (size_t target = 0;
         target < target_count; ++target) {
      for (const int goal :
           ins.target_goal_sets[target]) {
        if (root_goal_commitment_allows(
                commitment_mask, target, goal) !=
            root_goal_commitment_allows(
                previous_commitment_mask,
                target, goal)) {
          assignment_row_changed[target] = 1;
          break;
        }
      }
    }
  }
  const PairUpperSignatureDelta upper_delta =
      has_previous
          ? pair_upper_signature_delta(
                ins, *previous_upper, upper)
          : PairUpperSignatureDelta{};

  for (size_t target = 0; target < target_count; ++target) {
    for (const int goal : ins.target_goal_sets[target]) {
      if (cutoff()) return out;
      const auto goal_it =
          std::lower_bound(goals.begin(), goals.end(), goal);
      if (goal_it == goals.end() || *goal_it != goal)
        throw std::logic_error(
            "build_lazy_pair_cost_assignment: missing goal");
      const size_t goal_index = goal_it - goals.begin();
      PairPlan plan;
      PairCostDependency dependency;
      bool reused = false;
      if (has_previous) {
        const auto& previous_row =
            (*previous_table)[target];
        const auto previous_entry = std::find_if(
            previous_row.begin(), previous_row.end(),
            [&](const PairCostEntry& entry) {
              return entry.goal == goal;
            });
        if (previous_entry != previous_row.end() &&
            !previous_entry->plan.cutoff &&
            !pair_dependency_intersects(
                ins, previous_entry->dependency,
                upper_delta, storage_topology,
                dependency_context, deadline)) {
          plan = previous_entry->plan;
          dependency = previous_entry->dependency;
          reused = true;
          ++out.reused_edges;
        }
      }
      if (!reused)
        assignment_row_changed[target] = 1;
      if (!reused)
        plan = pair_cost_cheap_lower_bound(
            ins, upper, (int)target, goal,
            upper_wall, alpha, gamma,
            &dependency);
      if (plan.cutoff) {
        out.cutoff = true;
        return out;
      }
      if (cutoff()) return out;
      entry_index[target][goal_index] =
          (int)out.table[target].size();
      out.table[target].push_back(
          PairCostEntry{
              goal, plan, std::move(dependency)});
      cost[target][goal_index] =
          root_goal_commitment_allows(
              commitment_mask, target, goal) &&
                  std::isfinite(plan.estimated_cost)
              ? (long double)plan.estimated_cost
              : INF;
      ++out.total_edges;
      out.evaluated_edges += plan.exact;
      if (!reused) {
        out.rollout_work_steps += plan.rollout_steps;
        if (plan.exact) {
          out.rollout_truncations += plan.truncated;
          out.rollout_stalls += plan.stalled;
        }
      }
    }
  }

  IncrementalLongDoubleHungarian
      incremental_hungarian;
  bool hungarian_ready =
      has_previous_hungarian &&
      incremental_hungarian.restore(
          *previous_hungarian_state);
  if (!hungarian_ready)
    std::fill(
        assignment_row_changed.begin(),
        assignment_row_changed.end(), 1);
  std::vector<int> changed_assignment_rows;
  std::vector<uint8_t> changed_assignment_row_mark(
      target_count, 0);
  const auto mark_assignment_row_changed =
      [&](size_t target) {
        if (target >= target_count ||
            changed_assignment_row_mark[target])
          return;
        changed_assignment_row_mark[target] = 1;
        changed_assignment_rows.push_back(
            static_cast<int>(target));
      };
  for (size_t target = 0;
       target < target_count; ++target)
    if (assignment_row_changed[target])
      mark_assignment_row_changed(target);
  const auto current_cost =
      [&](int target, int goal_index) {
        return cost[target][goal_index];
      };
  const auto solve_current_assignment = [&]() {
    LongDoubleAssignmentResult assignment;
    if (!hungarian_ready) {
      assignment =
          incremental_hungarian.solve_full(
              static_cast<int>(target_count),
              static_cast<int>(goals.size()),
              current_cost, deadline);
      ++out.hungarian_full_solves;
      hungarian_ready = assignment.feasible &&
                        !assignment.cutoff;
    } else if (!changed_assignment_rows.empty()) {
      assignment =
          incremental_hungarian.repair_rows(
              changed_assignment_rows,
              current_cost, deadline);
      out.hungarian_row_repairs +=
          changed_assignment_rows.size();
    } else {
      assignment =
          incremental_hungarian.current_result(
              current_cost, deadline);
    }
    for (const int row : changed_assignment_rows)
      changed_assignment_row_mark[row] = 0;
    changed_assignment_rows.clear();
    return assignment;
  };

  auto refine = [&](size_t target, size_t goal_index) {
    if (cutoff()) return false;
    const int index = entry_index[target][goal_index];
    if (index < 0) return false;
    auto& entry = out.table[target][index];
    if (entry.plan.exact) return false;
    const long double previous_cost =
        cost[target][goal_index];
    if (entry.plan.bound_stage ==
        PairBoundStage::CHEAP_BOUND) {
      entry.plan = pair_cost_prefix_lower_bound(
          ins, upper, (int)target, goals[goal_index],
          upper_wall, storage_topology,
          alpha, gamma, delta, 8, deadline,
          potential_cache, dependency_context,
          &entry.dependency);
      ++out.prefix_refinements;
    } else if (
        entry.plan.bound_stage ==
        PairBoundStage::PREFIX_BOUND) {
      entry.plan = pair_cost(
          ins, upper, (int)target, goals[goal_index],
          upper_wall, storage_topology,
          alpha, gamma, delta, deadline,
          potential_cache, dependency_context,
          &entry.dependency);
    } else {
      throw std::logic_error(
          "build_lazy_pair_cost_assignment: non-exact edge has exact stage");
    }
    if (entry.plan.cutoff) {
      out.cutoff = true;
      return false;
    }
    if (cutoff()) return false;
    out.rollout_work_steps += entry.plan.rollout_steps;
    if (entry.plan.exact) {
      ++out.evaluated_edges;
      out.rollout_truncations += entry.plan.truncated;
      out.rollout_stalls += entry.plan.stalled;
    }
    cost[target][goal_index] =
        root_goal_commitment_allows(
            commitment_mask, target, goals[goal_index]) &&
                std::isfinite(entry.plan.estimated_cost)
            ? (long double)entry.plan.estimated_cost
            : INF;
    if (cost[target][goal_index] !=
        previous_cost)
      mark_assignment_row_changed(target);
    return true;
  };

  auto forced_lower_bound =
      [&](size_t forced_target, size_t forced_goal) {
        if (cutoff()) return INF;
        if (forced_target >= target_count ||
            forced_goal >= goals.size() ||
            cost[forced_target][forced_goal] >= INF / 2)
          return INF;
        auto forced_hungarian =
            incremental_hungarian;
        const auto forced_cost =
            [&](int target, int goal_index) {
              if (target ==
                      static_cast<int>(
                          forced_target) &&
                  goal_index !=
                      static_cast<int>(
                          forced_goal))
                return INF;
              return cost[target][goal_index];
            };
        const auto assignment =
            forced_hungarian.repair_rows(
                {static_cast<int>(forced_target)},
                forced_cost, deadline);
        ++out.hungarian_forced_repairs;
        if (assignment.cutoff) {
          out.cutoff = true;
          return INF;
        }
        if (!assignment.feasible ||
            assignment.row_to_col[forced_target] !=
                static_cast<int>(forced_goal))
          return INF;
        return assignment.cost;
      };

  for (;;) {
    if (cutoff()) return out;
    const auto assignment =
        solve_current_assignment();
    if (assignment.cutoff) {
      out.cutoff = true;
      return out;
    }
    if (!assignment.feasible)
      throw std::logic_error(
          "build_lazy_pair_cost_assignment: infeasible matching");

    bool refined = false;
    for (size_t target = 0; target < target_count; ++target) {
      if (cutoff()) return out;
      const int goal_index = assignment.row_to_col[target];
      if (goal_index < 0)
        throw std::logic_error(
            "build_lazy_pair_cost_assignment: unassigned target");
      refined |= refine(target, (size_t)goal_index);
    }
    if (refined) continue;

    for (size_t target = 0;
         target < target_count && !refined;
         ++target) {
      for (size_t goal_index = 0; goal_index < goals.size();
           ++goal_index) {
        if (cutoff()) return out;
        const int index = entry_index[target][goal_index];
        if (index < 0 || out.table[target][index].plan.exact ||
            cost[target][goal_index] >= INF / 2)
          continue;
        if (forced_lower_bound(target, goal_index) <=
            assignment.cost)
          refined |= refine(target, goal_index);
        if (out.cutoff) return out;
        if (refined) break;
      }
    }
    if (refined) continue;
    break;
  }

  if (cutoff()) return out;
  out.hungarian_state =
      incremental_hungarian.snapshot();
  bool tau_cutoff = false;
  out.tau =
      commitments == nullptr
          ? solve_tau_guide(
                ins, upper, out.table, deadline, &tau_cutoff)
          : solve_tau_guide_with_commitments(
                ins, upper, out.table, *commitments,
                deadline, &tau_cutoff);
  if (tau_cutoff) {
    out.cutoff = true;
    out.tau.clear();
    return out;
  }
  if (cutoff()) return out;
  for (size_t target = 0; target < out.tau.size(); ++target) {
    const auto found = std::find_if(
        out.table[target].begin(), out.table[target].end(),
        [&](const PairCostEntry& entry) {
          return entry.goal == out.tau[target];
        });
    if (found == out.table[target].end() || !found->plan.exact)
      throw std::logic_error(
          "build_lazy_pair_cost_assignment: selected edge is not exact");
  }
  return out;
}

inline double solve_tau_lb(const DDInstance& ins, const PhysConfig& s,
                           DDDistCache& upper_wall, double alpha,
                           double gamma)
{
  const size_t n = ins.n_targets();
  if (n == 0) return 0;
  std::vector<int> goals;
  for (const auto& set : ins.target_goal_sets)
    goals.insert(goals.end(), set.begin(), set.end());
  std::sort(goals.begin(), goals.end());
  goals.erase(std::unique(goals.begin(), goals.end()), goals.end());
  constexpr long double INF = 1e60L;
  std::vector<uint8_t> carried(n, 0);
  for (const int k : s.kappa)
    if (k >= 0 && k < (int)n) carried[k] = 1;
  std::vector<std::vector<long double>> cost(
      n, std::vector<long double>(goals.size(), INF));
  for (size_t b = 0; b < n; ++b)
    for (size_t j = 0; j < goals.size(); ++j) {
      const int goal = goals[j];
      if (!eligible_goal(ins, (int)b, goal)) continue;
      const int d = upper_wall.dist(goal, s.target_pos[b]);
      if (d >= INT_MAX / 4) continue;
      long double v = alpha * (long double)d;
      if (carried[b])
        v += gamma;
      else if (s.target_pos[b] != goal)
        v += 2.0L * gamma;
      cost[b][j] = v;
    }
  const auto result = hungarian_long_double(cost);
  if (!result.feasible)
    throw std::logic_error("solve_tau_lb: infeasible eligible-goal matching");
  return (double)result.cost;
}

inline int64_t solve_tau_time_lb(const DDInstance& ins,
                                 const PhysConfig& s,
                                 DDDistCache& wall_distance)
{
  const size_t target_count = ins.n_targets();
  if (target_count == 0) return 0;
  if (ins.n_robots() == 0)
    throw std::logic_error(
        "solve_tau_time_lb: targets require at least one robot");

  std::vector<int> goals;
  for (const auto& set : ins.target_goal_sets)
    goals.insert(goals.end(), set.begin(), set.end());
  std::sort(goals.begin(), goals.end());
  goals.erase(std::unique(goals.begin(), goals.end()), goals.end());

  constexpr int64_t INF_T = std::numeric_limits<int64_t>::max() / 8;
  constexpr long double HINF = 1e60L;
  std::vector<uint8_t> carried(target_count, 0);
  for (const int k : s.kappa)
    if (k >= 0 && k < (int)target_count) carried[k] = 1;

  std::vector<std::vector<int64_t>> completion(
      target_count, std::vector<int64_t>(goals.size(), INF_T));
  std::vector<std::vector<int64_t>> necessary_work(
      target_count, std::vector<int64_t>(goals.size(), INF_T));
  std::vector<int64_t> thresholds;
  for (size_t target = 0; target < target_count; ++target) {
    int approach = INT_MAX / 4;
    for (const int robot_cell : s.robots)
      approach = std::min(
          approach,
          wall_distance.dist(s.target_pos[target], robot_cell));
    for (size_t goal_index = 0; goal_index < goals.size();
         ++goal_index) {
      const int goal = goals[goal_index];
      if (!eligible_goal(ins, (int)target, goal)) continue;
      const int distance =
          wall_distance.dist(goal, s.target_pos[target]);
      if (distance >= INT_MAX / 4) continue;

      int64_t ell = 0;
      int64_t work = 0;
      if (carried[target]) {
        ell = static_cast<int64_t>(distance) + 1;
        work = static_cast<int64_t>(distance) + 1;
      } else if (s.target_pos[target] != goal) {
        if (approach >= INT_MAX / 4) continue;
        ell = static_cast<int64_t>(approach) + 1 + distance + 1;
        work = static_cast<int64_t>(distance) + 2;
      }
      completion[target][goal_index] = ell;
      necessary_work[target][goal_index] = work;
      thresholds.push_back(ell);
    }
  }

  if (thresholds.empty())
    throw std::logic_error(
        "solve_tau_time_lb: no eligible completion bound");
  std::sort(thresholds.begin(), thresholds.end());
  thresholds.erase(
      std::unique(thresholds.begin(), thresholds.end()),
      thresholds.end());

  auto threshold_feasible = [&](int64_t threshold) {
    std::vector<std::vector<long double>> allowed(
        target_count,
        std::vector<long double>(goals.size(), HINF));
    for (size_t target = 0; target < target_count; ++target)
      for (size_t goal_index = 0; goal_index < goals.size();
           ++goal_index)
        if (completion[target][goal_index] <= threshold)
          allowed[target][goal_index] = 0;
    return hungarian_long_double(allowed).feasible;
  };

  size_t lo = 0;
  size_t hi = thresholds.size();
  while (lo < hi) {
    const size_t mid = lo + (hi - lo) / 2;
    if (threshold_feasible(thresholds[mid]))
      hi = mid;
    else
      lo = mid + 1;
  }
  if (lo == thresholds.size())
    throw std::logic_error(
        "solve_tau_time_lb: infeasible bottleneck matching");
  const int64_t bottleneck = thresholds[lo];

  std::vector<std::vector<long double>> work_matrix(
      target_count, std::vector<long double>(goals.size(), HINF));
  for (size_t target = 0; target < target_count; ++target)
    for (size_t goal_index = 0; goal_index < goals.size();
         ++goal_index)
      if (necessary_work[target][goal_index] < INF_T)
        work_matrix[target][goal_index] =
            necessary_work[target][goal_index];
  const auto work_assignment = hungarian_long_double(work_matrix);
  if (!work_assignment.feasible)
    throw std::logic_error(
        "solve_tau_time_lb: infeasible work matching");
  const int64_t work_bound = static_cast<int64_t>(
      std::ceil(
          work_assignment.cost /
          static_cast<long double>(ins.n_robots())));
  return std::max(bottleneck, work_bound);
}

}  // namespace carrier_detail
