#include "../include/rho_assignment.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <numeric>
#include <utility>

namespace {

using WideCost = __int128_t;

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
