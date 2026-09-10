// Part of carrier_guidance.hpp (internal, src/): Pair/episode costs, forced-effect seeds, transfer continuity.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_task_compiler.hpp"

namespace carrier_detail {
struct PairCostDependencyContext {
  const std::vector<ClearanceCost>* distances_to(
      const DDInstance& ins,
      const StorageTransferTopology& storage_topology,
      int vacancy_cell, const Deadline* deadline)
  {
    const size_t cell_count =
        static_cast<size_t>(ins.grid.size());
    if (storage_topology.reverse.size() != cell_count ||
        vacancy_cell < 0 ||
        static_cast<size_t>(vacancy_cell) >= cell_count)
      return nullptr;
    if (distance_rows.size() != cell_count) {
      distance_rows.assign(cell_count, {});
      distance_ready.assign(cell_count, 0);
    }
    if (distance_ready[vacancy_cell])
      return &distance_rows[vacancy_cell];

    auto& distance = distance_rows[vacancy_cell];
    distance.assign(
        cell_count, ClearanceCost::infinity());
    using QueueKey = std::tuple<int, int, int, int>;
    std::priority_queue<
        QueueKey, std::vector<QueueKey>,
        std::greater<QueueKey>>
        queue;
    distance[vacancy_cell] = ClearanceCost::zero();
    queue.emplace(0, 0, 0, vacancy_cell);
    uint64_t operations = 0;
    while (!queue.empty()) {
      if (((++operations) & 255) == 0 &&
          is_expired(deadline)) {
        distance.clear();
        return nullptr;
      }
      const auto [
          service_ticks, pushes, loaded_steps, cell] =
          queue.top();
      queue.pop();
      const ClearanceCost settled{
          service_ticks, pushes, loaded_steps};
      if (settled != distance[cell]) continue;
      for (const auto& arc :
           storage_topology.reverse[cell]) {
        const ClearanceCost edge{
            arc.loaded_steps + 2, 1,
            arc.loaded_steps};
        const ClearanceCost candidate =
            settled + edge;
        if (!(candidate < distance[arc.source]))
          continue;
        distance[arc.source] = candidate;
        queue.emplace(
            candidate.service_ticks,
            candidate.pushes,
            candidate.loaded_steps,
            arc.source);
      }
    }
    distance_ready[vacancy_cell] = 1;
    return &distance;
  }

  void finalize(
      const DDInstance& ins,
      const StorageTransferTopology& storage_topology,
      const PairCostDependencyRecorder& recorder,
      PairCostDependency& dependency,
      const Deadline* deadline)
  {
    (void)storage_topology;
    dependency.cells = recorder.direct_cells;
    const size_t word_count =
        (static_cast<size_t>(ins.grid.size()) + 63) / 64;
    dependency.vacancy_removal_cells.assign(
        word_count, 0);
    dependency.vacancy_thresholds.clear();
    dependency.complete = false;
    if (!recorder.valid ||
        dependency.cells.size() != word_count)
      return;

    std::map<int, ClearanceCost> threshold_by_endpoint;
    for (const auto& read : recorder.vacancy_reads) {
      if (read.endpoint < 0 ||
          read.endpoint >= ins.grid.size())
        return;
      if (read.source_cell >= 0) {
        if (read.source_cell >= ins.grid.size() ||
            !ins.can_place_movable_shelf(
                read.source_cell))
          return;
        dependency.vacancy_removal_cells[
            read.source_cell / 64] |=
            uint64_t{1} <<
                (read.source_cell % 64);
      }
      const ClearanceCost threshold{
          read.service_ticks, read.pushes,
          read.loaded_steps};
      auto inserted =
          threshold_by_endpoint.emplace(
              read.endpoint, threshold);
      if (!inserted.second &&
          inserted.first->second < threshold)
        inserted.first->second = threshold;
    }

    for (const auto& [endpoint, threshold] :
         threshold_by_endpoint) {
      dependency.vacancy_thresholds.push_back(
          PairVacancyThreshold{
              endpoint, threshold.service_ticks,
              threshold.pushes, threshold.loaded_steps});
    }
    dependency.complete = !is_expired(deadline);
  }

  std::vector<std::vector<ClearanceCost>> distance_rows;
  std::vector<uint8_t> distance_ready;
};

inline bool pair_dependency_intersects(
    const DDInstance& ins,
    const PairCostDependency& dependency,
    const PairUpperSignatureDelta& delta,
    const StorageTransferTopology& storage_topology,
    PairCostDependencyContext* dependency_context,
    const Deadline* deadline)
{
  if (!dependency.complete ||
      dependency.cells.size() !=
          delta.changed_cells.size() ||
      dependency.vacancy_removal_cells.size() !=
          delta.vacancy_removals.size())
    return true;
  for (size_t word = 0;
       word < delta.changed_cells.size(); ++word) {
    if ((dependency.cells[word] &
         delta.changed_cells[word]) != 0 ||
        (dependency.vacancy_removal_cells[word] &
         delta.vacancy_removals[word]) != 0)
      return true;
  }

  bool has_additions = false;
  for (const uint64_t word : delta.vacancy_additions)
    has_additions |= word != 0;
  if (!has_additions ||
      dependency.vacancy_thresholds.empty())
    return false;
  if (dependency_context == nullptr) return true;

  for (int cell = 0; cell < ins.grid.size(); ++cell) {
    if (((delta.vacancy_additions[cell / 64] >>
          (cell % 64)) &
         uint64_t{1}) == 0)
      continue;
    const auto* distance =
        dependency_context->distances_to(
            ins, storage_topology, cell, deadline);
    if (distance == nullptr) return true;
    for (const auto& read :
         dependency.vacancy_thresholds) {
      if (read.endpoint < 0 ||
          read.endpoint >= ins.grid.size())
        return true;
      const ClearanceCost threshold{
          read.service_ticks, read.pushes,
          read.loaded_steps};
      if ((*distance)[read.endpoint] < threshold)
        return true;
    }
  }
  return false;
}

inline const PairPlan* selected_pair_plan(
    const PairCostTable& table, const std::vector<int>& tau, int target)
{
  if (target < 0 || target >= (int)table.size() ||
      target >= (int)tau.size())
    return nullptr;
  for (const auto& entry : table[target])
    if (entry.goal == tau[target]) return &entry.plan;
  return nullptr;
}

inline std::vector<int> target_priorities_from_pair_cost(
    const PairCostTable& table, const std::vector<int>& tau)
{
  std::vector<int> order(table.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    const PairPlan* plan_a = selected_pair_plan(table, tau, a);
    const PairPlan* plan_b = selected_pair_plan(table, tau, b);
    if ((plan_a != nullptr && !plan_a->exact) ||
        (plan_b != nullptr && !plan_b->exact))
      throw std::logic_error(
          "target priority cannot consume an inexact PairCost bound");
    const double ca =
        plan_a != nullptr
            ? plan_a->estimated_cost
            : std::numeric_limits<double>::infinity();
    const double cb =
        plan_b != nullptr
            ? plan_b->estimated_cost
            : std::numeric_limits<double>::infinity();
    return ca != cb ? ca > cb : a < b;
  });
  std::vector<int> priority(table.size(), 0);
  for (size_t rank = 0; rank < order.size(); ++rank)
    priority[order[rank]] = (int)order.size() - (int)rank;
  return priority;
}

inline std::vector<int> ready_tasks(const DDInstance& ins,
                                    const PhysConfig& physical,
                                    const ShelfTaskGraph& graph)
{
  const auto upper = make_upper_signature(physical);
  const auto occupied =
      upper_occupancy_bitmap(ins, upper);
  std::vector<uint8_t> carried_target(ins.n_targets(), 0);
  bool any_carried_anon = false;
  for (const int k : physical.kappa) {
    if (k >= 0 && k < (int)ins.n_targets()) carried_target[k] = 1;
    if (k == KAPPA_ANON) any_carried_anon = true;
  }
  std::vector<int> ready;
  for (size_t index = 0; index < graph.tasks.size(); ++index) {
    const auto& task = graph.tasks[index];
    if (!graph.predecessors[index].empty()) continue;
    bool shelf_grounded = false;
    if (task.id.shelf.kind == ShelfSelector::Kind::TARGET) {
      const int target = task.id.shelf.value;
      shelf_grounded =
          target >= 0 && target < (int)physical.target_pos.size() &&
          !carried_target[target] &&
          physical.target_pos[target] == task.id.from;
    } else {
      shelf_grounded =
          !any_carried_anon &&
          std::binary_search(physical.anon_occ.begin(),
                             physical.anon_occ.end(), task.id.from) &&
          task.id.shelf.value == task.id.from;
    }
    if (!shelf_grounded || occupied[task.id.to]) continue;
    ready.push_back((int)index);
  }
  std::stable_sort(ready.begin(), ready.end(), [&](int a, int b) {
    if (graph.tasks[a].priority != graph.tasks[b].priority)
      return graph.tasks[a].priority > graph.tasks[b].priority;
    return graph.tasks[a].id < graph.tasks[b].id;
  });
  return ready;
}

struct PairEpisodeCost {
  std::optional<ShelfSelector> open_shelf;
  double value = 0;

  void apply_shift(const ShelfSelector& shelf, double alpha,
                   double gamma, double delta)
  {
    if (!open_shelf.has_value() || *open_shelf != shelf) {
      if (open_shelf.has_value()) value += gamma;
      value += gamma;
      open_shelf = shelf;
    }
    value += alpha;
    if (shelf.kind == ShelfSelector::Kind::ANON_AT_EPOCH_CELL)
      value += delta;
  }

  double finish(double gamma)
  {
    if (open_shelf.has_value()) {
      value += gamma;
      open_shelf.reset();
    }
    return value;
  }
};

inline double pair_episode_cost(
    const std::vector<ShelfSelector>& shifted_shelves, double alpha,
    double gamma, double delta)
{
  PairEpisodeCost cost;
  for (const auto& shelf : shifted_shelves)
    cost.apply_shift(shelf, alpha, gamma, delta);
  return cost.finish(gamma);
}

inline PairPlan pair_cost_prefix_lower_bound(
    const DDInstance& ins, const UpperSignature& upper, int target,
    int goal, DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    double alpha, double gamma, double delta, int prefix_cap,
    const Deadline* deadline,
    VacancyPotentialCache* potential_cache,
    PairCostDependencyContext* dependency_context,
    PairCostDependency* dependency)
{
  PairPlan out;
  std::optional<PairCostDependencyRecorder> dependency_recorder;
  if (dependency != nullptr) {
    dependency->cells.clear();
    dependency->vacancy_removal_cells.clear();
    dependency->vacancy_thresholds.clear();
    dependency->complete = false;
    if (dependency_context != nullptr)
      dependency_recorder.emplace(ins.grid.size());
  }
  auto finish = [&]() {
    if (!out.cutoff && dependency != nullptr &&
        dependency_context != nullptr &&
        dependency_recorder.has_value())
      dependency_context->finalize(
          ins, storage_topology, *dependency_recorder,
          *dependency, deadline);
    out.bound_stage =
        out.exact ? PairBoundStage::EXACT
                  : PairBoundStage::PREFIX_BOUND;
    return out;
  };
  const auto cutoff = [&]() {
    if (!is_expired(deadline)) return false;
    out.cutoff = true;
    out.exact = false;
    return true;
  };
  if (cutoff()) return finish();
  if (!eligible_goal(ins, target, goal)) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return finish();
  }
  if (dependency_recorder.has_value())
    dependency_recorder->record_cell(
        upper.target_pos[target]);
  const int initial_distance =
      upper_wall.dist(goal, upper.target_pos[target]);
  out.direct_distance =
      initial_distance >= INT_MAX / 4 ? INT_MAX : initial_distance;
  if (initial_distance >= INT_MAX / 4) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return finish();
  }
  if (upper.target_pos[target] == goal) {
    out.reached_goal = true;
    return finish();
  }

  auto abstract = make_abstract_upper_state(
      ins, upper,
      dependency_recorder.has_value()
          ? &*dependency_recorder
          : nullptr);
  const TaskBRCompilerLimits limits{
      std::max(32, 4 * ins.grid.size()),
      std::max(64, 8 * ins.grid.size())};
  const RootDemand root{target, goal};
  const ShelfSelector target_shelf{
      ShelfSelector::Kind::TARGET, target};
  PairEpisodeCost episode_cost;
  SingleRootCompilerScratch compiler_scratch;
  const int effective_prefix_cap = std::max(0, prefix_cap);
  while (out.rollout_steps < effective_prefix_cap) {
    if (cutoff()) return finish();
    const auto next = compile_single_root_next_ready_effect(
        ins, abstract, root, upper_wall, storage_topology,
        limits, compiler_scratch, potential_cache, deadline);
    if (next.cutoff) {
      out.cutoff = true;
      out.exact = false;
      return finish();
    }
    if (cutoff()) return finish();
    if (!next.ready_effect.has_value() ||
        !next.ready_transfer.has_value()) {
      out.stalled = true;
      break;
    }
    const auto& effect = *next.ready_effect;
    const auto& transfer = *next.ready_transfer;
    const int legs = transfer.route_size - 1;
    if (legs <= 0 ||
        out.rollout_steps + legs > effective_prefix_cap)
      break;
    if (transfer.explicit_route.empty()) {
      episode_cost.apply_shift(
          effect.shelf, alpha, gamma, delta);
      abstract.move(effect.shelf, transfer.first_step);
      ++out.rollout_steps;
    } else {
      const auto& route = transfer.explicit_route;
      for (size_t index = 1;
           index < route.size(); ++index) {
        if (cutoff()) return finish();
        episode_cost.apply_shift(
            effect.shelf, alpha, gamma, delta);
        abstract.move(effect.shelf, route[index]);
        ++out.rollout_steps;
      }
    }
    if (abstract.position(target_shelf) == goal) {
      out.reached_goal = true;
      break;
    }
  }

  if (out.reached_goal || out.stalled) {
    out.estimated_cost = episode_cost.finish(gamma);
    if (!out.reached_goal) {
      const int remaining =
          upper_wall.dist(
              goal, abstract.position(target_shelf));
      if (remaining < INT_MAX / 4)
        out.estimated_cost +=
            alpha * (double)remaining + 2.0 * gamma;
      out.estimated_cost +=
          alpha * (double)(ins.grid.size() + 1) + 2.0 * gamma;
    }
    return finish();
  }

  const int remaining =
      upper_wall.dist(
          goal, abstract.position(target_shelf));
  out.estimated_cost = episode_cost.value;
  if (remaining < INT_MAX / 4)
    out.estimated_cost += alpha * (double)remaining;
  if (!episode_cost.open_shelf.has_value()) {
    out.estimated_cost += 2.0 * gamma;
  } else if (*episode_cost.open_shelf == target_shelf) {
    out.estimated_cost += gamma;
  } else {
    out.estimated_cost += 3.0 * gamma;
  }
  out.exact = false;
  return finish();
}

inline PairPlan pair_cost(const DDInstance& ins, const UpperSignature& upper,
                          int target, int goal, DDDistCache& upper_wall,
                          const StorageTransferTopology& storage_topology,
                          double alpha, double gamma, double delta,
                          const Deadline* deadline,
                          VacancyPotentialCache* potential_cache,
                          PairCostDependencyContext*
                              dependency_context,
                          PairCostDependency* dependency)
{
  PairPlan out;
  std::optional<PairCostDependencyRecorder> dependency_recorder;
  if (dependency != nullptr) {
    dependency->cells.clear();
    dependency->vacancy_removal_cells.clear();
    dependency->vacancy_thresholds.clear();
    dependency->complete = false;
    if (dependency_context != nullptr)
      dependency_recorder.emplace(ins.grid.size());
  }
  auto finish = [&]() {
    if (!out.cutoff && dependency != nullptr &&
        dependency_context != nullptr &&
        dependency_recorder.has_value())
      dependency_context->finalize(
          ins, storage_topology, *dependency_recorder,
          *dependency, deadline);
    out.bound_stage = PairBoundStage::EXACT;
    return out;
  };
  const auto cutoff = [&]() {
    if (!is_expired(deadline)) return false;
    out.cutoff = true;
    out.exact = false;
    return true;
  };
  if (cutoff()) return finish();
  if (!eligible_goal(ins, target, goal)) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return finish();
  }
  if (dependency_recorder.has_value())
    dependency_recorder->record_cell(
        upper.target_pos[target]);
  const int initial_distance =
      upper_wall.dist(goal, upper.target_pos[target]);
  out.direct_distance =
      initial_distance >= INT_MAX / 4 ? INT_MAX : initial_distance;
  if (initial_distance >= INT_MAX / 4) {
    out.estimated_cost = std::numeric_limits<double>::infinity();
    out.stalled = true;
    return finish();
  }
  if (upper.target_pos[target] == goal) {
    out.reached_goal = true;
    return finish();
  }

  auto abstract = make_abstract_upper_state(
      ins, upper,
      dependency_recorder.has_value()
          ? &*dependency_recorder
          : nullptr);
  const int step_cap = std::max(8, std::min(128, 2 * ins.grid.size()));
  const TaskBRCompilerLimits limits{
      std::max(32, 4 * ins.grid.size()),
      std::max(64, 8 * ins.grid.size())};
  const RootDemand root{target, goal};
  const ShelfSelector target_shelf{
      ShelfSelector::Kind::TARGET, target};
  PairEpisodeCost episode_cost;
  SingleRootCompilerScratch compiler_scratch;
  while (out.rollout_steps < step_cap) {
    if (cutoff()) return finish();
    if (abstract.position(target_shelf) == goal) {
      out.reached_goal = true;
      break;
    }
    const auto next = compile_single_root_next_ready_effect(
        ins, abstract, root, upper_wall, storage_topology,
        limits, compiler_scratch, potential_cache, deadline);
    if (next.cutoff) {
      out.cutoff = true;
      out.exact = false;
      return finish();
    }
    if (cutoff()) return finish();
    if (!next.ready_effect.has_value() ||
        !next.ready_transfer.has_value()) {
      out.stalled = true;
      break;
    }
    const auto& effect = *next.ready_effect;
    const auto& transfer = *next.ready_transfer;
    const int legs = transfer.route_size - 1;
    if (legs <= 0 || out.rollout_steps + legs > step_cap) {
      out.truncated = true;
      break;
    }
    if (transfer.explicit_route.empty()) {
      episode_cost.apply_shift(
          effect.shelf, alpha, gamma, delta);
      abstract.move(effect.shelf, transfer.first_step);
      ++out.rollout_steps;
    } else {
      const auto& route = transfer.explicit_route;
      for (size_t index = 1;
           index < route.size(); ++index) {
        if (cutoff()) return finish();
        episode_cost.apply_shift(
            effect.shelf, alpha, gamma, delta);
        abstract.move(effect.shelf, route[index]);
        ++out.rollout_steps;
      }
    }
    if (abstract.position(target_shelf) == goal) {
      out.reached_goal = true;
      break;
    }
  }
  if (!out.reached_goal && !out.stalled &&
      out.rollout_steps >= step_cap)
    out.truncated = true;
  out.estimated_cost += episode_cost.finish(gamma);
  if (!out.reached_goal) {
    const int remaining =
        upper_wall.dist(
            goal, abstract.position(target_shelf));
    if (remaining < INT_MAX / 4)
      out.estimated_cost += alpha * (double)remaining + 2.0 * gamma;
    const double finite_penalty =
        alpha * (double)(ins.grid.size() + 1) + 2.0 * gamma;
    if (out.stalled || out.truncated)
      out.estimated_cost += finite_penalty;
  }
  return finish();
}

inline PairPlan pair_cost_prefix_lower_bound(
    const DDInstance& ins, const UpperSignature& upper, int target,
    int goal, DDDistCache& upper_wall, double alpha,
    double gamma, double delta, int prefix_cap,
    const Deadline* deadline = nullptr)
{
  bool topology_cutoff = false;
  const auto storage_topology =
      build_storage_transfer_topology(
          ins, deadline, &topology_cutoff);
  if (topology_cutoff) {
    PairPlan out;
    out.cutoff = true;
    out.exact = false;
    out.bound_stage = PairBoundStage::PREFIX_BOUND;
    return out;
  }
  return pair_cost_prefix_lower_bound(
      ins, upper, target, goal, upper_wall,
      storage_topology, alpha, gamma, delta,
      prefix_cap, deadline);
}

inline PairPlan pair_cost(
    const DDInstance& ins, const UpperSignature& upper, int target,
    int goal, DDDistCache& upper_wall, double alpha,
    double gamma, double delta,
    const Deadline* deadline = nullptr)
{
  bool topology_cutoff = false;
  const auto storage_topology =
      build_storage_transfer_topology(
          ins, deadline, &topology_cutoff);
  if (topology_cutoff) {
    PairPlan out;
    out.cutoff = true;
    out.exact = false;
    return out;
  }
  return pair_cost(
      ins, upper, target, goal, upper_wall,
      storage_topology, alpha, gamma, delta, deadline);
}

// lower-deck distance provider: exact Manhattan on wall-free grids,
// shared lazy-BFS cache otherwise.
struct LowerDist {
  const DDGrid& g;
  bool wallfree;
  DDDistCache bfs;
  explicit LowerDist(const DDGrid& g_) : g(g_), bfs(g_)
  {
    wallfree = true;
    for (uint8_t w : g.wall) wallfree &= (w == 0);
  }
  int dist(int cell, int from)
  {
    if (wallfree)
      return std::abs(g.row(cell) - g.row(from)) +
             std::abs(g.col(cell) - g.col(from));
    return bfs.to(cell)[from];
  }
};

inline int task_index_by_id(const ShelfTaskGraph& graph, const TaskId& id)
{
  for (size_t i = 0; i < graph.tasks.size(); ++i)
    if (graph.tasks[i].id == id) return (int)i;
  return -1;
}

inline StorageTransfer normalized_transfer(const ShelfTask& task)
{
  if (task.transfer.route.size() >= 2 &&
      task.transfer.route.front() == task.id.from &&
      task.transfer.route[1] == task.id.to &&
      task.transfer.route.back() == task.transfer.endpoint)
    return task.transfer;
  return StorageTransfer{
      task.id.to, {task.id.from, task.id.to}};
}

inline TransferKey transfer_key(const ShelfTask& task)
{
  const auto transfer = normalized_transfer(task);
  return TransferKey{
      task.id.shelf, task.id.from, transfer.endpoint};
}

inline bool forced_effects_reproduced_by_unforced_prefix(
    const std::vector<TaskBRForcedEffect>& forced_effects,
    const ShelfTaskGraph& graph)
{
  size_t task_index = 0;
  for (const auto& forced : forced_effects) {
    if (forced.kind != TaskBRForcedEffect::Kind::TRANSFER ||
        forced.transfer.route.size() < 2 ||
        task_index >= graph.tasks.size() ||
        task_index >= graph.predecessors.size() ||
        !graph.predecessors[task_index].empty())
      return false;
    const auto& task = graph.tasks[task_index];
    const auto transfer = normalized_transfer(task);
    std::vector<RootDemand> forced_roots = forced.roots;
    std::sort(forced_roots.begin(), forced_roots.end());
    forced_roots.erase(
        std::unique(
            forced_roots.begin(), forced_roots.end()),
        forced_roots.end());
    std::vector<RootDemand> task_roots = task.roots;
    std::sort(task_roots.begin(), task_roots.end());
    task_roots.erase(
        std::unique(task_roots.begin(), task_roots.end()),
        task_roots.end());
    if (task.id.shelf != forced.shelf ||
        task.id.from != forced.transfer.route.front() ||
        task.id.to != forced.transfer.route[1] ||
        transfer.endpoint != forced.transfer.endpoint ||
        transfer.route != forced.transfer.route ||
        task_roots != forced_roots)
      return false;
    ++task_index;
  }
  return true;
}

inline bool forced_effects_reproduced_by_unforced_frontier(
    const std::vector<TaskBRForcedEffect>& forced_effects,
    const ShelfTaskGraph& graph)
{
  if (forced_effects_reproduced_by_unforced_prefix(
          forced_effects, graph))
    return true;
  std::vector<uint8_t> used(graph.tasks.size(), 0);
  for (const auto& forced : forced_effects) {
    if (forced.kind != TaskBRForcedEffect::Kind::TRANSFER ||
        forced.roots.empty())
      return false;
    std::vector<RootDemand> forced_roots = forced.roots;
    std::sort(forced_roots.begin(), forced_roots.end());
    forced_roots.erase(
        std::unique(
            forced_roots.begin(), forced_roots.end()),
        forced_roots.end());
    bool reproduced = false;
    for (size_t task_index = 0;
         task_index < graph.tasks.size(); ++task_index) {
      if (used[task_index] ||
          task_index >= graph.predecessors.size() ||
          !graph.predecessors[task_index].empty())
        continue;
      std::vector<RootDemand> task_roots =
          graph.tasks[task_index].roots;
      std::sort(task_roots.begin(), task_roots.end());
      task_roots.erase(
          std::unique(
              task_roots.begin(), task_roots.end()),
          task_roots.end());
      if (task_roots != forced_roots) continue;
      used[task_index] = 1;
      reproduced = true;
      break;
    }
    if (!reproduced) return false;
  }
  return true;
}

inline std::vector<TaskBRForcedEffect>
shared_forced_effects_for_compiled_graph(
    const std::vector<TaskBRForcedEffect>& forced_effects,
    const ShelfTaskGraph& graph)
{
  std::vector<TaskBRForcedEffect> out;
  for (const auto& forced : forced_effects) {
    if (forced.kind != TaskBRForcedEffect::Kind::TRANSFER ||
        forced.transfer.route.size() < 2) {
      out.push_back(forced);
      continue;
    }
    std::vector<RootDemand> forced_roots = forced.roots;
    std::sort(forced_roots.begin(), forced_roots.end());
    forced_roots.erase(
        std::unique(forced_roots.begin(), forced_roots.end()),
        forced_roots.end());
    if (forced_roots.size() != 1) {
      out.push_back(forced);
      continue;
    }
    const TransferKey key{
        forced.shelf, forced.transfer.route.front(),
        forced.transfer.endpoint};
    const auto task_it = std::find_if(
        graph.tasks.begin(), graph.tasks.end(),
        [&](const ShelfTask& task) {
          return transfer_key(task) == key;
        });
    if (task_it == graph.tasks.end()) {
      out.push_back(forced);
      continue;
    }
    const int task_index =
        static_cast<int>(task_it - graph.tasks.begin());
    bool shared = std::any_of(
        task_it->roots.begin(), task_it->roots.end(),
        [&](const RootDemand& root) {
          return !(root == forced_roots.front());
        });
    if (task_index <
            static_cast<int>(graph.predecessors.size()) &&
        !graph.predecessors[task_index].empty())
      shared = true;
    if (task_index <
            static_cast<int>(graph.successors.size()) &&
        !graph.successors[task_index].empty())
      shared = true;
    shared |= std::any_of(
        graph.causal_edges.begin(), graph.causal_edges.end(),
        [&](const CausalEdge& edge) {
          return edge.producer == task_index ||
                 edge.consumer == task_index;
        });
    shared |= std::any_of(
        graph.rotations.begin(), graph.rotations.end(),
        [&](const RotationCandidate& rotation) {
          return std::find(
                     rotation.cycle.begin(),
                     rotation.cycle.end(),
                     task_it->id) != rotation.cycle.end();
        });
    if (shared) out.push_back(forced);
  }
  return out;
}

inline RootTransferContinuity root_first_transfer_continuity(
    const ShelfTaskGraph& graph)
{
  RootTransferContinuity out;
  for (size_t index = 0; index < graph.tasks.size(); ++index) {
    const auto& task = graph.tasks[index];
    const TransferKey key = transfer_key(task);
    for (const auto& root : task.roots) {
      bool has_root_predecessor = false;
      if (index < graph.predecessors.size()) {
        for (const int predecessor :
             graph.predecessors[index]) {
          if (predecessor < 0 ||
              predecessor >=
                  static_cast<int>(graph.tasks.size()))
            continue;
          const auto& predecessor_roots =
              graph.tasks[predecessor].roots;
          if (std::binary_search(
                  predecessor_roots.begin(),
                  predecessor_roots.end(), root)) {
            has_root_predecessor = true;
            break;
          }
        }
      }
      if (has_root_predecessor) continue;
      const auto found = out.find(root);
      if (found == out.end() || key < found->second)
        out[root] = key;
    }
  }
  return out;
}

inline RootTransferContinuity next_epoch_transfer_continuity(
    const UpperEpochGuidance& previous_epoch)
{
  auto out =
      root_first_transfer_continuity(
          previous_epoch.task_graph);
  for (const auto& [root, protected_key] :
       previous_epoch.transfer_continuity) {
    const auto selected = out.find(root);
    if (selected != out.end() &&
        selected->second == protected_key)
      out.erase(selected);
  }
  return out;
}

inline std::vector<TaskBRForcedEffect>
canonical_fixed_goal_forced_effects(
    const DDInstance& ins, const UpperSignature& upper,
    const UpperEpochGuidance* previous_epoch,
    const StorageTransferTopology& storage_topology,
    const std::vector<TaskBRForcedEffect>& forced_effects)
{
  if (previous_epoch == nullptr || forced_effects.empty())
    return forced_effects;
  std::vector<int> tau;
  tau.reserve(ins.n_targets());
  for (const auto& goals : ins.target_goal_sets) {
    if (goals.size() != 1)
      return forced_effects;
    tau.push_back(goals.front());
  }
  std::vector<int> target_priority =
      previous_epoch->target_priority;
  if (target_priority.size() != ins.n_targets()) {
    target_priority.resize(ins.n_targets());
    for (size_t target = 0;
         target < ins.n_targets(); ++target)
      target_priority[target] =
          static_cast<int>(ins.n_targets() - target);
  }
  std::vector<RootDemand> roots;
  for (size_t target = 0; target < ins.n_targets(); ++target)
    if (target < upper.target_pos.size() &&
        upper.target_pos[target] != tau[target])
      roots.push_back(
          RootDemand{
              static_cast<int>(target), tau[target]});
  auto abstract =
      make_abstract_upper_state(ins, upper);
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache scratch_distance(upper_grid);
  TaskBRCompilerLimits compiler_limits{256, 512};
  if (upper_vacancy_count(ins, upper) > 2)
    compiler_limits.total_recursion_cap =
        compiler_limits.recursion_cap +
        compiler_limits.backtrack_cap;
  const auto unforced_graph = compile_task_br_pibt(
      ins, abstract, roots, tau, target_priority,
      scratch_distance, storage_topology,
      compiler_limits, false, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr);
  if (forced_effects_reproduced_by_unforced_frontier(
          forced_effects, unforced_graph))
    return {};
  const auto graph = compile_task_br_pibt(
      ins, abstract, roots, tau, target_priority,
      scratch_distance, storage_topology,
      compiler_limits, false, &forced_effects,
      nullptr, nullptr, nullptr, nullptr, nullptr);
  const auto shared_forced_effects =
      shared_forced_effects_for_compiled_graph(
          forced_effects, graph);
  std::vector<TaskBRForcedEffect> out;
  for (const auto& forced : shared_forced_effects) {
    std::vector<RootDemand> forced_roots = forced.roots;
    std::sort(forced_roots.begin(), forced_roots.end());
    forced_roots.erase(
        std::unique(forced_roots.begin(), forced_roots.end()),
        forced_roots.end());
    if (forced.kind != TaskBRForcedEffect::Kind::TRANSFER ||
        forced_roots.size() != 1 ||
        forced.transfer.route.size() < 2) {
      out.push_back(forced);
      continue;
    }
    const TransferKey key{
        forced.shelf, forced.transfer.route.front(),
        forced.transfer.endpoint};
    const auto task_it = std::find_if(
        graph.tasks.begin(), graph.tasks.end(),
        [&](const ShelfTask& task) {
          return transfer_key(task) == key;
        });
    if (task_it == graph.tasks.end()) continue;
    const int task_index =
        static_cast<int>(task_it - graph.tasks.begin());
    const bool shares_beyond_current_source =
        std::any_of(
            task_it->roots.begin(), task_it->roots.end(),
            [&](const RootDemand& root) {
              return !(root == forced_roots.front()) &&
                     root.goal != key.source;
            });
    if (!shares_beyond_current_source) continue;
    const bool unlocks_successor =
        (task_index <
             static_cast<int>(graph.successors.size()) &&
         !graph.successors[task_index].empty()) ||
        std::any_of(
            graph.causal_edges.begin(),
            graph.causal_edges.end(),
            [&](const CausalEdge& edge) {
              return edge.producer == task_index;
            });
    const bool participates_in_rotation =
        std::any_of(
            graph.rotations.begin(), graph.rotations.end(),
            [&](const RotationCandidate& rotation) {
              return std::find(
                         rotation.cycle.begin(),
                         rotation.cycle.end(),
                         task_it->id) != rotation.cycle.end();
            });
    if (unlocks_successor || participates_in_rotation)
      out.push_back(forced);
  }
  return out;
}

inline RootTransferContinuity successor_transfer_continuity(
    const ShelfTaskGraph& graph, const TaskId& completed_task,
    const std::vector<RootDemand>& roots)
{
  RootTransferContinuity out;
  const int completed_index =
      task_index_by_id(graph, completed_task);
  if (completed_index < 0 ||
      completed_index >=
          static_cast<int>(graph.successors.size()))
    return out;
  for (const auto& root : roots) {
    std::optional<TransferKey> selected;
    for (const int successor :
         graph.successors[completed_index]) {
      if (successor < 0 ||
          successor >= static_cast<int>(graph.tasks.size()))
        continue;
      const auto& task = graph.tasks[successor];
      if (!std::binary_search(
              task.roots.begin(), task.roots.end(), root))
        continue;
      const TransferKey key = transfer_key(task);
      if (!selected.has_value() || key < *selected)
        selected = key;
    }
    if (selected.has_value()) out.emplace(root, *selected);
  }
  return out;
}

}  // namespace carrier_detail
