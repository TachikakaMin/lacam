#include "../include/rho_assignment.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <limits>
#include <numeric>
#include <utility>

namespace {

using WideCost = RhoWideCost;

constexpr WideCost kWideInf = (static_cast<WideCost>(1) << 120);

struct WideAssignmentResult {
  std::vector<int> row_to_col;
  std::vector<WideCost> row_potential;
  std::vector<WideCost> column_potential;
  WideCost objective = 0;
  bool feasible = false;
};

bool finite_cost(RhoCost cost)
{
  return cost > -kRhoAssignmentInf &&
         cost < kRhoAssignmentInf;
}

bool invalid_negative_cost(RhoCost cost)
{
  return cost <= -kRhoAssignmentInf;
}

bool fits_rho_cost(WideCost value)
{
  return value >=
             static_cast<WideCost>(
                 std::numeric_limits<RhoCost>::min()) &&
         value <=
             static_cast<WideCost>(
                 std::numeric_limits<RhoCost>::max());
}

bool matrix_shape(
    const std::vector<std::vector<RhoCost>>& cost,
    int& row_count, int& column_count, bool& overflow)
{
  overflow = false;
  row_count = static_cast<int>(cost.size());
  column_count =
      cost.empty() ? 0 : static_cast<int>(cost.front().size());
  for (const auto& row : cost) {
    if (static_cast<int>(row.size()) != column_count)
      return false;
    for (const RhoCost value : row) {
      if (invalid_negative_cost(value)) {
        overflow = true;
        return false;
      }
    }
  }
  return true;
}

WideCost padded_edge(
    const std::vector<std::vector<RhoCost>>& cost,
    int real_rows, int row, int column)
{
  if (row >= real_rows) return 0;
  const RhoCost value = cost[row][column];
  return finite_cost(value)
             ? static_cast<WideCost>(value)
             : kWideInf;
}

bool guarded_potential(WideCost value)
{
  return value > -kWideInf && value < kWideInf;
}

bool state_objective(
    const std::vector<std::vector<RhoCost>>& cost,
    const RhoHungarianState& state, RhoCost& objective)
{
  WideCost total = 0;
  for (int row = 0; row < state.real_rows; ++row) {
    const int column = state.mate_l[row];
    if (column < 0 || column >= state.columns ||
        !finite_cost(cost[row][column]))
      return false;
    total += static_cast<WideCost>(cost[row][column]);
  }
  if (!fits_rho_cost(total)) return false;
  objective = static_cast<RhoCost>(total);
  return true;
}

bool validate_state_internal(
    const std::vector<std::vector<RhoCost>>& cost,
    const RhoHungarianState& state)
{
  int real_rows = 0;
  int columns = 0;
  bool overflow = false;
  if (!matrix_shape(cost, real_rows, columns, overflow) ||
      overflow || real_rows > columns ||
      (real_rows > 0 && columns == 0) || !state.valid ||
      state.real_rows != real_rows ||
      state.columns != columns ||
      state.real_cost != cost)
    return false;

  const int dimension = columns;
  if (static_cast<int>(state.mate_l.size()) != dimension ||
      static_cast<int>(state.mate_r.size()) != dimension ||
      static_cast<int>(state.row_potential.size()) != dimension ||
      static_cast<int>(state.column_potential.size()) != dimension)
    return false;

  for (int row = 0; row < dimension; ++row) {
    if (!guarded_potential(state.row_potential[row]))
      return false;
    const int column = state.mate_l[row];
    if (column < 0 || column >= dimension ||
        state.mate_r[column] != row)
      return false;
  }
  for (int column = 0; column < dimension; ++column) {
    if (!guarded_potential(state.column_potential[column]))
      return false;
    const int row = state.mate_r[column];
    if (row < 0 || row >= dimension ||
        state.mate_l[row] != column)
      return false;
  }

  for (int row = 0; row < dimension; ++row) {
    for (int column = 0; column < dimension; ++column) {
      const WideCost edge =
          padded_edge(cost, real_rows, row, column);
      if (edge >= kWideInf) continue;
      if (state.row_potential[row] +
              state.column_potential[column] >
          edge)
        return false;
    }
    const int matched_column = state.mate_l[row];
    const WideCost matched_edge =
        padded_edge(cost, real_rows, row, matched_column);
    if (matched_edge >= kWideInf ||
        state.row_potential[row] +
                state.column_potential[matched_column] !=
            matched_edge)
      return false;
  }

  RhoCost objective = 0;
  if (!state_objective(cost, state, objective) ||
      objective != state.objective)
    return false;
  WideCost dual_objective = 0;
  for (const WideCost value : state.row_potential)
    dual_objective += value;
  for (const WideCost value : state.column_potential)
    dual_objective += value;
  return dual_objective == static_cast<WideCost>(objective);
}

RhoAssignmentResult assignment_from_state(
    const std::vector<std::vector<RhoCost>>& cost,
    const RhoHungarianState& state)
{
  RhoAssignmentResult out;
  out.row_to_col.assign(cost.size(), -1);
  if (!state.valid) return out;
  for (int row = 0; row < state.real_rows; ++row)
    out.row_to_col[row] = state.mate_l[row];
  RhoCost objective = 0;
  if (!state_objective(cost, state, objective)) {
    out.overflow = true;
    return out;
  }
  out.objective = objective;
  out.feasible = true;
  return out;
}

bool augment_state_from_row(
    const std::vector<std::vector<RhoCost>>& cost,
    RhoHungarianState& state, int root)
{
  const int dimension = state.columns;
  if (root < 0 || root >= dimension ||
      state.mate_l[root] != -1)
    return false;

  std::vector<uint8_t> in_left(dimension, 0);
  std::vector<uint8_t> in_right(dimension, 0);
  std::vector<WideCost> slack(dimension, kWideInf);
  std::vector<int> slack_from(dimension, -1);
  std::deque<int> queue;
  in_left[root] = 1;
  queue.push_back(root);

  const auto augment_path = [&](int free_column) {
    int column = free_column;
    while (column >= 0) {
      const int row = slack_from[column];
      if (row < 0) return false;
      const int previous_column = state.mate_l[row];
      state.mate_l[row] = column;
      state.mate_r[column] = row;
      column = previous_column;
    }
    return true;
  };

  while (true) {
    while (!queue.empty()) {
      const int row = queue.front();
      queue.pop_front();
      for (int column = 0; column < dimension; ++column) {
        if (in_right[column]) continue;
        const WideCost edge =
            padded_edge(cost, state.real_rows, row, column);
        if (edge >= kWideInf) continue;
        const WideCost reduced =
            edge - state.row_potential[row] -
            state.column_potential[column];
        if (reduced < 0) return false;
        if (reduced < slack[column]) {
          slack[column] = reduced;
          slack_from[column] = row;
        }
        if (slack[column] != 0) continue;
        in_right[column] = 1;
        const int matched_row = state.mate_r[column];
        if (matched_row < 0)
          return augment_path(column);
        if (!in_left[matched_row]) {
          in_left[matched_row] = 1;
          queue.push_back(matched_row);
        }
      }
    }

    WideCost delta = kWideInf;
    for (int column = 0; column < dimension; ++column)
      if (!in_right[column])
        delta = std::min(delta, slack[column]);
    if (delta >= kWideInf || delta < 0) return false;

    for (int row = 0; row < dimension; ++row)
      if (in_left[row])
        state.row_potential[row] += delta;
    for (int column = 0; column < dimension; ++column) {
      if (in_right[column]) {
        state.column_potential[column] -= delta;
      } else if (slack[column] < kWideInf) {
        slack[column] -= delta;
      }
    }

    for (int column = 0; column < dimension; ++column) {
      if (in_right[column] || slack[column] != 0) continue;
      in_right[column] = 1;
      const int matched_row = state.mate_r[column];
      if (matched_row < 0)
        return augment_path(column);
      if (!in_left[matched_row]) {
        in_left[matched_row] = 1;
        queue.push_back(matched_row);
      }
    }
  }
}

RhoStateSolveResult solve_state_full_internal(
    const std::vector<std::vector<RhoCost>>& cost)
{
  RhoStateSolveResult out;
  out.assignment.row_to_col.assign(cost.size(), -1);
  int real_rows = 0;
  int columns = 0;
  bool overflow = false;
  if (!matrix_shape(cost, real_rows, columns, overflow)) {
    out.assignment.overflow = overflow;
    return out;
  }
  if (real_rows > columns ||
      (real_rows > 0 && columns == 0))
    return out;

  RhoHungarianState state;
  state.real_rows = real_rows;
  state.columns = columns;
  state.real_cost = cost;
  state.mate_l.assign(columns, -1);
  state.mate_r.assign(columns, -1);
  state.row_potential.assign(columns, 0);
  state.column_potential.assign(columns, 0);

  for (int row = 0; row < columns; ++row) {
    WideCost best = kWideInf;
    for (int column = 0; column < columns; ++column) {
      const WideCost edge =
          padded_edge(cost, real_rows, row, column);
      if (edge >= kWideInf) continue;
      best = std::min(
          best, edge - state.column_potential[column]);
    }
    if (best >= kWideInf) return out;
    state.row_potential[row] = best;
    if (!augment_state_from_row(cost, state, row))
      return out;
    ++out.augmentations;
  }

  state.valid = true;
  if (!state_objective(cost, state, state.objective)) {
    out.assignment.overflow = true;
    return out;
  }
  if (!validate_state_internal(cost, state))
    return out;
  out.state = std::move(state);
  out.assignment = assignment_from_state(cost, out.state);
  return out;
}

WideAssignmentResult solve_minimum_wide(
    const std::vector<std::vector<RhoCost>>& cost)
{
  WideAssignmentResult out;
  const size_t row_count = cost.size();
  const size_t column_count =
      cost.empty() ? 0 : cost.front().size();
  out.row_to_col.assign(row_count, -1);
  if (row_count == 0) {
    out.feasible = true;
    return out;
  }
  if (column_count == 0 || row_count > column_count)
    return out;
  for (const auto& row : cost)
    if (row.size() != column_count)
      return out;

  std::vector<WideCost> row_potential(row_count + 1, 0);
  std::vector<WideCost> column_potential(column_count + 1, 0);
  std::vector<int> column_mate(column_count + 1, 0);
  std::vector<int> predecessor(column_count + 1, 0);

  for (size_t next_row = 1; next_row <= row_count; ++next_row) {
    column_mate[0] = static_cast<int>(next_row);
    int current_column = 0;
    std::vector<WideCost> slack(column_count + 1, kWideInf);
    std::vector<uint8_t> used(column_count + 1, 0);
    do {
      used[current_column] = 1;
      const int current_row = column_mate[current_column];
      WideCost delta = kWideInf;
      int next_column = -1;
      for (size_t column = 1; column <= column_count; ++column) {
        if (used[column]) continue;
        const RhoCost edge =
            cost[current_row - 1][column - 1];
        if (finite_cost(edge)) {
          const WideCost reduced =
              static_cast<WideCost>(edge) -
              row_potential[current_row] -
              column_potential[column];
          if (reduced < slack[column]) {
            slack[column] = reduced;
            predecessor[column] = current_column;
          }
        }
        if (slack[column] < delta ||
            (slack[column] == delta &&
             slack[column] < kWideInf &&
             (next_column < 0 ||
              static_cast<int>(column) < next_column))) {
          delta = slack[column];
          next_column = static_cast<int>(column);
        }
      }
      if (next_column < 0 || delta >= kWideInf)
        return out;
      for (size_t column = 0; column <= column_count; ++column) {
        if (used[column]) {
          row_potential[column_mate[column]] += delta;
          column_potential[column] -= delta;
        } else if (slack[column] < kWideInf) {
          slack[column] -= delta;
        }
      }
      current_column = next_column;
    } while (column_mate[current_column] != 0);

    do {
      const int previous_column = predecessor[current_column];
      column_mate[current_column] =
          column_mate[previous_column];
      current_column = previous_column;
    } while (current_column != 0);
  }

  for (size_t column = 1; column <= column_count; ++column)
    if (column_mate[column] > 0)
      out.row_to_col[column_mate[column] - 1] =
          static_cast<int>(column) - 1;

  for (size_t row = 0; row < row_count; ++row) {
    const int column = out.row_to_col[row];
    if (column < 0 ||
        !finite_cost(cost[row][column]))
      return WideAssignmentResult{
          std::vector<int>(row_count, -1),
          std::vector<WideCost>(),
          std::vector<WideCost>(), 0, false};
    out.objective +=
        static_cast<WideCost>(cost[row][column]);
  }
  out.row_potential.assign(
      row_potential.begin() + 1, row_potential.end());
  out.column_potential.assign(
      column_potential.begin() + 1,
      column_potential.end());
  out.feasible = true;
  return out;
}

bool canonicalize_optimum(
    const std::vector<std::vector<RhoCost>>& cost,
    const WideAssignmentResult& optimum,
    std::vector<int>& canonical)
{
  const int original_row_count =
      static_cast<int>(cost.size());
  const int column_count =
      cost.empty() ? 0 : static_cast<int>(cost.front().size());
  // Rectangular assignment has unmatched columns.  A tight matching over
  // only the real rows need not preserve the optimum if it changes which
  // nonzero-potential columns are unmatched.  Complete the equality graph
  // with logical dummy rows: a dummy may cover exactly a zero-potential
  // column, which is the complementary-slackness condition for an
  // unmatched column.
  const int row_count = column_count;
  std::vector<int> row_to_col(row_count, -1);
  std::vector<int> column_to_row(column_count, -1);
  for (int row = 0; row < original_row_count; ++row) {
    const int column = optimum.row_to_col[row];
    if (column < 0 || column >= column_count ||
        column_to_row[column] >= 0)
      return false;
    row_to_col[row] = column;
    column_to_row[column] = row;
  }
  int dummy_row = original_row_count;
  for (int column = 0; column < column_count; ++column) {
    if (column_to_row[column] >= 0) continue;
    if (optimum.column_potential[column] != 0 ||
        dummy_row >= row_count)
      return false;
    row_to_col[dummy_row] = column;
    column_to_row[column] = dummy_row;
    ++dummy_row;
  }
  if (dummy_row != row_count) return false;

  std::vector<uint8_t> fixed_row(row_count, 0);
  std::vector<uint8_t> fixed_column(column_count, 0);
  const auto tight = [&](int row, int column) {
    if (row >= original_row_count)
      return optimum.column_potential[column] == 0;
    return
        finite_cost(cost[row][column]) &&
        static_cast<WideCost>(cost[row][column]) -
                optimum.row_potential[row] -
                optimum.column_potential[column] ==
            0;
  };

  for (int row = 0; row < original_row_count; ++row) {
    bool fixed = false;
    for (int candidate = 0; candidate < column_count;
         ++candidate) {
      if (fixed_column[candidate] ||
          !tight(row, candidate))
        continue;
      auto trial_rows = row_to_col;
      auto trial_columns = column_to_row;
      const int old_column = trial_rows[row];
      trial_rows[row] = -1;
      trial_columns[old_column] = -1;
      const int displaced = trial_columns[candidate];
      if (displaced >= 0) trial_rows[displaced] = -1;
      trial_rows[row] = candidate;
      trial_columns[candidate] = row;

      bool completed = displaced < 0;
      if (displaced >= 0) {
        std::vector<uint8_t> seen_row(row_count, 0);
        std::vector<uint8_t> seen_column(column_count, 0);
        seen_column[candidate] = 1;
        const std::function<bool(int)> augment =
            [&](int active_row) {
              if (active_row < 0 ||
                  fixed_row[active_row] ||
                  active_row == row ||
                  seen_row[active_row])
                return false;
              seen_row[active_row] = 1;
              for (int column = 0; column < column_count;
                   ++column) {
                if (fixed_column[column] ||
                    seen_column[column] ||
                    !tight(active_row, column))
                  continue;
                seen_column[column] = 1;
                const int other = trial_columns[column];
                if (other >= 0 && !augment(other)) continue;
                trial_rows[active_row] = column;
                trial_columns[column] = active_row;
                return true;
              }
              return false;
            };
        completed = augment(displaced);
      }
      if (!completed) continue;
      row_to_col = std::move(trial_rows);
      column_to_row = std::move(trial_columns);
      fixed_row[row] = 1;
      fixed_column[candidate] = 1;
      fixed = true;
      break;
    }
    if (!fixed) return false;
  }
  canonical.assign(
      row_to_col.begin(),
      row_to_col.begin() + original_row_count);
  return true;
}

RhoCostBreakdown invalid_breakdown()
{
  return RhoCostBreakdown{};
}

bool valid_nonnegative_term(RhoCost value)
{
  return value >= 0 && value < kRhoAssignmentInf;
}

RhoCostBreakdown make_cost_breakdown(
    RhoCost approach, RhoCost immediate_service,
    RhoCost continuity, RhoCost mode, RhoCost urgency,
    RhoCost preparation_reward, RhoCost applied_reward)
{
  if (!valid_nonnegative_term(approach) ||
      !valid_nonnegative_term(immediate_service) ||
      !valid_nonnegative_term(continuity) ||
      !valid_nonnegative_term(mode) ||
      !valid_nonnegative_term(urgency) ||
      !valid_nonnegative_term(preparation_reward) ||
      !valid_nonnegative_term(applied_reward) ||
      preparation_reward > urgency ||
      applied_reward > urgency)
    return invalid_breakdown();
  const WideCost total =
      static_cast<WideCost>(approach) +
      static_cast<WideCost>(immediate_service) +
      static_cast<WideCost>(continuity) +
      static_cast<WideCost>(mode) -
      static_cast<WideCost>(applied_reward);
  if (total <= -static_cast<WideCost>(kRhoAssignmentInf) ||
      total >= static_cast<WideCost>(kRhoAssignmentInf))
    return invalid_breakdown();
  RhoCostBreakdown out;
  out.approach = approach;
  out.immediate_service = immediate_service;
  out.continuity = continuity;
  out.mode = mode;
  out.urgency_reward = urgency;
  out.preparation_reward = preparation_reward;
  out.total = static_cast<RhoCost>(total);
  out.valid = true;
  return out;
}

}  // namespace

RhoCostBreakdown rho_execute_cost(
    RhoCost approach, RhoCost immediate_service,
    RhoCost continuity, RhoCost mode, RhoCost urgency)
{
  return make_cost_breakdown(
      approach, immediate_service, continuity, mode, urgency,
      0, urgency);
}

RhoCostBreakdown rho_prepare_cost(
    RhoCost approach, RhoCost continuity, RhoCost mode,
    RhoCost urgency, RhoCost requested_prepare_reward)
{
  if (!valid_nonnegative_term(requested_prepare_reward) ||
      !valid_nonnegative_term(urgency))
    return invalid_breakdown();
  const RhoCost bounded_reward =
      std::min(urgency, requested_prepare_reward);
  return make_cost_breakdown(
      approach, 0, continuity, mode, urgency,
      bounded_reward, bounded_reward);
}

RhoAssignmentResult solve_rho_assignment_full(
    const std::vector<std::vector<RhoCost>>& cost,
    RhoAssignmentTiming* timing)
{
  RhoAssignmentResult out;
  out.row_to_col.assign(cost.size(), -1);
  for (const auto& row : cost)
    for (const RhoCost value : row)
      if (invalid_negative_cost(value)) {
        out.overflow = true;
        return out;
      }

  const auto optimum_started =
      std::chrono::steady_clock::now();
  const auto optimum = solve_minimum_wide(cost);
  if (timing != nullptr)
    timing->optimum_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() -
            optimum_started)
            .count();
  if (!optimum.feasible) return out;
  if (!fits_rho_cost(optimum.objective)) {
    out.overflow = true;
    return out;
  }

  const auto canonical_started =
      std::chrono::steady_clock::now();
  if (!canonicalize_optimum(cost, optimum, out.row_to_col))
    return RhoAssignmentResult{
        std::vector<int>(cost.size(), -1), 0, false, false};
  if (timing != nullptr)
    timing->canonical_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() -
            canonical_started)
            .count();

  out.objective = static_cast<RhoCost>(optimum.objective);
  out.feasible = true;
  return out;
}

RhoStateSolveResult solve_rho_assignment_state_full(
    const std::vector<std::vector<RhoCost>>& cost)
{
  return solve_state_full_internal(cost);
}

RhoStateSolveResult repair_rho_assignment_state_rows(
    const RhoHungarianState& parent,
    const std::vector<std::vector<RhoCost>>& cost,
    const std::vector<int>& changed_rows)
{
  RhoStateSolveResult out;
  out.assignment.row_to_col.assign(cost.size(), -1);

  int real_rows = 0;
  int columns = 0;
  bool overflow = false;
  if (!matrix_shape(cost, real_rows, columns, overflow)) {
    out.assignment.overflow = overflow;
    return out;
  }
  if (real_rows > columns ||
      (real_rows > 0 && columns == 0) ||
      parent.real_rows != real_rows ||
      parent.columns != columns ||
      !validate_state_internal(parent.real_cost, parent))
    return out;

  std::vector<uint8_t> changed_mask(real_rows, 0);
  std::vector<int> normalized_rows;
  normalized_rows.reserve(changed_rows.size());
  for (const int row : changed_rows) {
    if (row < 0 || row >= real_rows) return out;
    if (changed_mask[row]) continue;
    changed_mask[row] = 1;
    normalized_rows.push_back(row);
  }
  std::sort(normalized_rows.begin(), normalized_rows.end());

  for (int row = 0; row < real_rows; ++row)
    if (!changed_mask[row] &&
        cost[row] != parent.real_cost[row])
      return out;

  out.parent_valid = true;
  out.used_parent = true;
  if (normalized_rows.empty()) {
    out.state = parent;
    out.assignment = assignment_from_state(cost, out.state);
    return out;
  }

  RhoHungarianState state = parent;
  state.real_cost = cost;
  for (const int row : normalized_rows) {
    const int column = state.mate_l[row];
    if (column < 0 || column >= columns ||
        state.mate_r[column] != row)
      return out;
    state.mate_l[row] = -1;
    state.mate_r[column] = -1;
  }

  for (const int row : normalized_rows) {
    WideCost best = kWideInf;
    for (int column = 0; column < columns; ++column) {
      const WideCost edge =
          padded_edge(cost, real_rows, row, column);
      if (edge >= kWideInf) continue;
      best = std::min(
          best, edge - state.column_potential[column]);
    }
    if (best >= kWideInf) {
      state.valid = false;
      out.state = std::move(state);
      return out;
    }
    state.row_potential[row] = best;
  }

  for (const int row : normalized_rows) {
    if (!augment_state_from_row(cost, state, row)) {
      state.valid = false;
      out.state = std::move(state);
      return out;
    }
    ++out.augmentations;
  }

  state.valid = true;
  if (!state_objective(cost, state, state.objective)) {
    state.valid = false;
    out.assignment.overflow = true;
    out.state = std::move(state);
    return out;
  }
  if (!validate_state_internal(cost, state)) {
    state.valid = false;
    out.state = std::move(state);
    return out;
  }

  out.state = std::move(state);
  out.assignment = assignment_from_state(cost, out.state);
  return out;
}

bool validate_rho_assignment_state(
    const std::vector<std::vector<RhoCost>>& cost,
    const RhoHungarianState& state)
{
  return validate_state_internal(cost, state);
}
