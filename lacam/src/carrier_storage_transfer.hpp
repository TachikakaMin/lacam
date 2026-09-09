// Part of carrier_guidance.hpp (internal, src/): Task-BR compiler state, storage-transfer topology, vacancy potential.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_tau.hpp"

namespace carrier_detail {
struct PairVacancyDependencyRead {
  int endpoint = -1;
  int source_cell = -1;
  int service_ticks = 0;
  int pushes = 0;
  int loaded_steps = 0;
};

struct PairCostDependencyRecorder {
  explicit PairCostDependencyRecorder(size_t cell_count)
      : cell_count(cell_count),
        direct_cells((cell_count + 63) / 64, 0)
  {
  }

  void record_cell(int cell)
  {
    if (cell < 0 ||
        static_cast<size_t>(cell) >= cell_count) {
      valid = false;
      return;
    }
    direct_cells[cell / 64] |=
        uint64_t{1} << (cell % 64);
  }

  void record_vacancy_read(
      int endpoint, int source_cell, int service_ticks,
      int pushes, int loaded_steps)
  {
    if (endpoint < 0) {
      valid = false;
      return;
    }
    vacancy_reads.push_back(
        PairVacancyDependencyRead{
            endpoint, source_cell, service_ticks,
            pushes, loaded_steps});
  }

  size_t cell_count = 0;
  std::vector<uint64_t> direct_cells;
  std::vector<PairVacancyDependencyRead> vacancy_reads;
  bool valid = true;
};

struct TaskBRCompilerLimits {
  int recursion_cap = 256;
  int backtrack_cap = 512;
  // Negative preserves the legacy/probe default.  Production may impose a
  // tighter epoch-wide work cap without removing the clean local window that
  // each top-level root option receives.
  int total_recursion_cap = -1;
};

struct AbstractUpperState {
  std::vector<ShelfSelector> shelves;
  std::vector<int> positions;
  int target_count = 0;
  std::vector<int> anon_index_by_epoch_cell;
  std::vector<int> occupant;
  PairCostDependencyRecorder* dependency_recorder = nullptr;

  int shelf_index(const ShelfSelector& shelf) const
  {
    if (shelf.kind == ShelfSelector::Kind::TARGET)
      return shelf.value >= 0 && shelf.value < target_count
                 ? shelf.value
                 : -1;
    return shelf.value >= 0 &&
                   shelf.value < (int)anon_index_by_epoch_cell.size()
               ? anon_index_by_epoch_cell[shelf.value]
               : -1;
  }

  int position(const ShelfSelector& shelf) const
  {
    const int index = shelf_index(shelf);
    if (index < 0) {
      if (dependency_recorder != nullptr)
        dependency_recorder->valid = false;
      return -1;
    }
    const int cell = positions[index];
    if (dependency_recorder != nullptr)
      dependency_recorder->record_cell(cell);
    return cell;
  }

  bool empty(int cell) const
  {
    if (dependency_recorder != nullptr)
      dependency_recorder->record_cell(cell);
    return cell >= 0 && cell < (int)occupant.size() &&
           occupant[cell] < 0;
  }

  const ShelfSelector* shelf_at(int cell) const
  {
    if (dependency_recorder != nullptr)
      dependency_recorder->record_cell(cell);
    if (cell < 0 || cell >= (int)occupant.size()) return nullptr;
    const int index = occupant[cell];
    return index < 0 ? nullptr : &shelves[index];
  }

  void move(const ShelfSelector& shelf, int to)
  {
    const int index = shelf_index(shelf);
    if (index < 0) throw std::logic_error("unknown abstract shelf");
    const int from = positions[index];
    if (dependency_recorder != nullptr) {
      dependency_recorder->record_cell(from);
      dependency_recorder->record_cell(to);
    }
    if (from >= 0) occupant[from] = -1;
    positions[index] = to;
    occupant[to] = index;
  }
};

inline AbstractUpperState make_abstract_upper_state(
    const DDInstance& ins, const UpperSignature& upper,
    PairCostDependencyRecorder* dependency_recorder = nullptr)
{
  AbstractUpperState out;
  out.dependency_recorder = dependency_recorder;
  out.target_count = (int)upper.target_pos.size();
  out.anon_index_by_epoch_cell.assign(ins.grid.size(), -1);
  out.occupant.assign(ins.grid.size(), -1);
  auto add = [&](const ShelfSelector& shelf, int cell) {
    if (cell < 0 || cell >= ins.grid.size() || ins.grid.is_wall(cell))
      throw std::logic_error("abstract shelf on invalid cell");
    if (out.occupant[cell] >= 0)
      throw std::logic_error("duplicate shelf cell in upper projection");
    const int index = (int)out.shelves.size();
    out.shelves.push_back(shelf);
    out.positions.push_back(cell);
    if (shelf.kind == ShelfSelector::Kind::ANON_AT_EPOCH_CELL)
      out.anon_index_by_epoch_cell[shelf.value] = index;
    out.occupant[cell] = index;
  };
  for (size_t b = 0; b < upper.target_pos.size(); ++b)
    add(ShelfSelector{ShelfSelector::Kind::TARGET, (int)b},
        upper.target_pos[b]);
  for (const int cell : upper.anon_pos)
    add(ShelfSelector{ShelfSelector::Kind::ANON_AT_EPOCH_CELL, cell},
        cell);
  return out;
}

inline void add_root_demand(std::vector<RootDemand>& roots,
                            const RootDemand& root)
{
  roots.push_back(root);
  std::sort(roots.begin(), roots.end());
  roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
}

struct StorageTransferCandidate {
  int endpoint = -1;
  int first_step = -1;
  int route_size = 0;
  // Null means the exact adjacent route [from, first_step].  Non-null is
  // a read-only view owned by the surrounding candidate window.
  const std::vector<int>* explicit_route = nullptr;
};

struct ClearanceCost {
  static constexpr int INF = INT_MAX / 4;

  int service_ticks = INF;
  int pushes = INF;
  int loaded_steps = INF;

  static ClearanceCost zero()
  {
    return ClearanceCost{0, 0, 0};
  }

  static ClearanceCost infinity()
  {
    return ClearanceCost{};
  }

  bool finite() const { return service_ticks < INF; }

  bool operator==(const ClearanceCost& other) const
  {
    return service_ticks == other.service_ticks &&
           pushes == other.pushes &&
           loaded_steps == other.loaded_steps;
  }

  bool operator!=(const ClearanceCost& other) const
  {
    return !(*this == other);
  }

  bool operator<(const ClearanceCost& other) const
  {
    return std::tie(service_ticks, pushes, loaded_steps) <
           std::tie(
               other.service_ticks, other.pushes,
               other.loaded_steps);
  }
};

inline int clearance_cost_add_component(int a, int b)
{
  if (a >= ClearanceCost::INF || b >= ClearanceCost::INF)
    return ClearanceCost::INF;
  if (a > ClearanceCost::INF - b)
    return ClearanceCost::INF;
  return a + b;
}

inline ClearanceCost operator+(
    const ClearanceCost& a, const ClearanceCost& b)
{
  return ClearanceCost{
      clearance_cost_add_component(
          a.service_ticks, b.service_ticks),
      clearance_cost_add_component(a.pushes, b.pushes),
      clearance_cost_add_component(
          a.loaded_steps, b.loaded_steps)};
}

struct StorageTransferArc {
  int source = -1;
  int endpoint = -1;
  int loaded_steps = 0;
};

struct StorageTransferTopology {
  std::vector<std::vector<StorageTransfer>> transfers;
  std::vector<std::vector<StorageTransferArc>> outgoing;
  std::vector<std::vector<StorageTransferArc>> reverse;
};

struct VacancyPotential {
  std::vector<ClearanceCost> cost;
  std::vector<int> next_vacancy_cell;
  std::vector<int> vacancy_source_cell;
  std::vector<int> settled_target_moves;
  std::vector<ClearanceCost> anonymous_cost;
  std::vector<int> anonymous_next_vacancy_cell;
  std::vector<int> anonymous_vacancy_source_cell;
  std::vector<int> anonymous_settled_target_moves;
};

struct SelectedStorageTransfer {
  int endpoint = -1;
  int first_step = -1;
  int route_size = 0;
  // Empty means the exact adjacent route [from, first_step].  PairCost
  // copies a dynamic route only after this candidate becomes ready.
  std::vector<int> explicit_route;
};

inline SelectedStorageTransfer select_storage_transfer(
    const StorageTransferCandidate& candidate)
{
  SelectedStorageTransfer out;
  out.endpoint = candidate.endpoint;
  out.first_step = candidate.first_step;
  out.route_size = candidate.route_size;
  if (candidate.explicit_route != nullptr)
    out.explicit_route = *candidate.explicit_route;
  return out;
}

inline StorageTransfer materialize_storage_transfer(
    int from, const StorageTransferCandidate& candidate)
{
  if (candidate.explicit_route != nullptr)
    return StorageTransfer{
        candidate.endpoint, *candidate.explicit_route};
  return StorageTransfer{
      candidate.endpoint, {from, candidate.first_step}};
}

struct TaskBRCompilerState {
  ShelfTaskGraph graph;
  std::map<TransferKey, int> transfer_index;
  std::map<ShelfSelector, int> forced_transfer_index;
  std::map<ShelfSelector, TaskId> reserved_shelf_effect;
  std::map<int, TaskId> reserved_destination;
  std::map<int, TaskId> reserved_endpoint;
};

inline bool seed_task_br_forced_effects(
    const DDInstance& ins, const AbstractUpperState& upper,
    const std::vector<TaskBRForcedEffect>* forced_effects,
    TaskBRCompilerState& state)
{
  if (forced_effects == nullptr) return true;
  std::set<ShelfSelector> seen_shelves;
  for (const auto& forced : *forced_effects) {
    if (!seen_shelves.insert(forced.shelf).second) return false;
    const int from = upper.position(forced.shelf);
    if (from < 0) return false;
    if (forced.kind == TaskBRForcedEffect::Kind::WAIT) {
      const TaskId wait{forced.shelf, from, from};
      if (!state.reserved_shelf_effect.emplace(
              forced.shelf, wait).second)
        return false;
      continue;
    }
    if (forced.kind != TaskBRForcedEffect::Kind::TRANSFER ||
        forced.transfer.route.size() < 2 ||
        forced.transfer.route.front() != from ||
        forced.transfer.route.back() != forced.transfer.endpoint)
      return false;
    const int first_step = forced.transfer.route[1];
    const TaskId effect{forced.shelf, from, first_step};
    const TransferKey key{
        forced.shelf, from, forced.transfer.endpoint};
    if (!state.reserved_shelf_effect.emplace(
            forced.shelf, effect).second)
      return false;
    if (ins.can_store_shelf(first_step) &&
        !state.reserved_destination.emplace(
            first_step, effect).second)
      return false;
    if (forced.transfer.endpoint != first_step &&
        !state.reserved_endpoint.emplace(
            forced.transfer.endpoint, effect).second)
      return false;
    std::vector<RootDemand> roots = forced.roots;
    std::sort(roots.begin(), roots.end());
    roots.erase(
        std::unique(roots.begin(), roots.end()), roots.end());
    const int index = (int)state.graph.tasks.size();
    if (!state.transfer_index.emplace(key, index).second)
      return false;
    if (!state.forced_transfer_index.emplace(
            forced.shelf, index).second)
      return false;
    state.graph.tasks.push_back(
        ShelfTask{
            effect, std::move(roots), forced.priority,
            forced.transfer});
    state.graph.predecessors.emplace_back();
    state.graph.successors.emplace_back();
  }
  return true;
}

struct TaskBRCompilerUndo {
  enum class Kind {
    ERASE_EFFECT_INDEX,
    ERASE_SHELF_RESERVATION,
    ERASE_DESTINATION_RESERVATION,
    ERASE_ENDPOINT_RESERVATION,
    POP_GRAPH_TASK,
    POP_SUCCESSOR,
    POP_CAUSAL_EDGE,
    POP_ROTATION,
    RESTORE_TASK,
  };
  Kind kind = Kind::POP_GRAPH_TASK;
  TaskId task_id;
  ShelfSelector shelf;
  int index = -1;
  ShelfTask old_task;
  TransferKey transfer_key;
};

struct TaskBRCompilerTransaction {
  static constexpr bool records_rotations = true;

  TaskBRCompilerState& state;
  std::vector<TaskBRCompilerUndo> undo;

  explicit TaskBRCompilerTransaction(TaskBRCompilerState& state_)
      : state(state_)
  {
  }

  size_t checkpoint() const { return undo.size(); }

  void enter_recursion(const ShelfSelector&) {}
  void leave_recursion(const ShelfSelector&) {}

  bool recursion_cycle(
      const ShelfSelector& shelf,
      const std::vector<ShelfSelector>& recursion_stack) const
  {
    return std::find(
               recursion_stack.begin(), recursion_stack.end(), shelf) !=
           recursion_stack.end();
  }

  int find_transfer(const TransferKey& key) const
  {
    const auto found = state.transfer_index.find(key);
    return found == state.transfer_index.end() ? -1 : found->second;
  }

  int forced_transfer(const ShelfSelector& shelf) const
  {
    const auto found =
        state.forced_transfer_index.find(shelf);
    return found == state.forced_transfer_index.end()
               ? -1
               : found->second;
  }

  std::optional<TaskId> shelf_reservation(
      const ShelfSelector& shelf) const
  {
    const auto found = state.reserved_shelf_effect.find(shelf);
    return found == state.reserved_shelf_effect.end()
               ? std::nullopt
               : std::optional<TaskId>(found->second);
  }

  bool shelf_effect_conflicts(const ShelfSelector& shelf,
                              const TaskId&) const
  {
    const auto found = state.reserved_shelf_effect.find(shelf);
    // An exact TransferKey was merged before this check.  Any remaining
    // reservation for the shelf therefore represents a different complete
    // transfer, even when both routes happen to share the same first step.
    return found != state.reserved_shelf_effect.end();
  }

  bool destination_effect_conflicts(int cell,
                                    const TaskId& effect) const
  {
    const auto found = state.reserved_destination.find(cell);
    return found != state.reserved_destination.end() &&
           found->second != effect;
  }

  bool destination_reserved(int cell) const
  {
    return state.reserved_destination.count(cell) != 0;
  }

  bool endpoint_reserved(int cell) const
  {
    return state.reserved_destination.count(cell) != 0 ||
           state.reserved_endpoint.count(cell) != 0;
  }

  bool has_distinct_endpoint_reservations() const
  {
    return !state.reserved_endpoint.empty();
  }

  bool distinct_endpoint_reserved(int cell) const
  {
    return state.reserved_endpoint.count(cell) != 0;
  }

  bool endpoint_effect_conflicts(int cell,
                                 const TaskId& effect) const
  {
    const auto destination =
        state.reserved_destination.find(cell);
    if (destination != state.reserved_destination.end() &&
        destination->second != effect)
      return true;
    const auto found = state.reserved_endpoint.find(cell);
    return found != state.reserved_endpoint.end() &&
           found->second != effect;
  }

  bool distinct_endpoint_effect_conflicts(
      int cell, const TaskId& effect) const
  {
    const auto found = state.reserved_endpoint.find(cell);
    return found != state.reserved_endpoint.end() &&
           found->second != effect;
  }

  void reserve_shelf(const ShelfSelector& shelf, const TaskId& effect)
  {
    const auto inserted =
        state.reserved_shelf_effect.emplace(shelf, effect);
    if (inserted.second)
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::ERASE_SHELF_RESERVATION,
          TaskId(), shelf});
  }

  void reserve_destination(int cell, const TaskId& effect)
  {
    const auto inserted =
        state.reserved_destination.emplace(cell, effect);
    if (inserted.second)
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::ERASE_DESTINATION_RESERVATION,
          TaskId(), ShelfSelector(), cell});
  }

  void reserve_endpoint(int cell, const TaskId& effect)
  {
    const auto inserted = state.reserved_endpoint.emplace(cell, effect);
    if (inserted.second)
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::ERASE_ENDPOINT_RESERVATION,
          TaskId(), ShelfSelector(), cell});
  }

  void merge_task(int index, const RootDemand& root, int priority)
  {
    auto& task = state.graph.tasks[index];
    const bool has_root =
        std::binary_search(task.roots.begin(), task.roots.end(), root);
    const int merged_priority = std::max(task.priority, priority);
    if (has_root && merged_priority == task.priority) return;
    TaskBRCompilerUndo entry;
    entry.kind = TaskBRCompilerUndo::Kind::RESTORE_TASK;
    entry.index = index;
    entry.old_task = task;
    undo.push_back(std::move(entry));
    add_root_demand(task.roots, root);
    task.priority = merged_priority;
  }

  int add_task(const TransferKey& key, const TaskId& id,
               const StorageTransferCandidate& transfer,
               int from,
               const RootDemand& root, int predecessor,
               int must_be_vacated, int priority)
  {
    const auto found = state.transfer_index.find(key);
    if (found != state.transfer_index.end()) {
      merge_task(found->second, root, priority);
      return found->second;
    }

    const int index = (int)state.graph.tasks.size();
    state.transfer_index.emplace(key, index);
    TaskBRCompilerUndo erase_transfer;
    erase_transfer.kind =
        TaskBRCompilerUndo::Kind::ERASE_EFFECT_INDEX;
    erase_transfer.transfer_key = key;
    undo.push_back(std::move(erase_transfer));
    state.graph.tasks.push_back(
        ShelfTask{
            id, {root}, priority,
            materialize_storage_transfer(from, transfer)});
    state.graph.predecessors.emplace_back();
    state.graph.successors.emplace_back();
    undo.push_back(TaskBRCompilerUndo{
        TaskBRCompilerUndo::Kind::POP_GRAPH_TASK});
    if (predecessor >= 0) {
      state.graph.predecessors[index].push_back(predecessor);
      state.graph.successors[predecessor].push_back(index);
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::POP_SUCCESSOR,
          TaskId(), ShelfSelector(), predecessor});
      state.graph.causal_edges.push_back(
          CausalEdge{
              predecessor, must_be_vacated, index,
              CausalEventKind::ENTER_CELL});
      undo.push_back(TaskBRCompilerUndo{
          TaskBRCompilerUndo::Kind::POP_CAUSAL_EDGE});
    }
    return index;
  }

  void add_rotation(const RotationCandidate& rotation)
  {
    const auto duplicate = std::find_if(
        state.graph.rotations.begin(), state.graph.rotations.end(),
        [&](const RotationCandidate& existing) {
          return existing.cycle == rotation.cycle;
        });
    if (duplicate != state.graph.rotations.end()) return;
    state.graph.rotations.push_back(rotation);
    undo.push_back(TaskBRCompilerUndo{
        TaskBRCompilerUndo::Kind::POP_ROTATION});
  }

  void rollback(size_t checkpoint)
  {
    while (undo.size() > checkpoint) {
      auto entry = std::move(undo.back());
      undo.pop_back();
      switch (entry.kind) {
        case TaskBRCompilerUndo::Kind::ERASE_EFFECT_INDEX:
          state.transfer_index.erase(entry.transfer_key);
          break;
        case TaskBRCompilerUndo::Kind::ERASE_SHELF_RESERVATION:
          state.reserved_shelf_effect.erase(entry.shelf);
          break;
        case TaskBRCompilerUndo::Kind::ERASE_DESTINATION_RESERVATION:
          state.reserved_destination.erase(entry.index);
          break;
        case TaskBRCompilerUndo::Kind::ERASE_ENDPOINT_RESERVATION:
          state.reserved_endpoint.erase(entry.index);
          break;
        case TaskBRCompilerUndo::Kind::POP_GRAPH_TASK:
          state.graph.tasks.pop_back();
          state.graph.predecessors.pop_back();
          state.graph.successors.pop_back();
          break;
        case TaskBRCompilerUndo::Kind::POP_SUCCESSOR:
          state.graph.successors[entry.index].pop_back();
          break;
        case TaskBRCompilerUndo::Kind::POP_CAUSAL_EDGE:
          state.graph.causal_edges.pop_back();
          break;
        case TaskBRCompilerUndo::Kind::POP_ROTATION:
          state.graph.rotations.pop_back();
          break;
        case TaskBRCompilerUndo::Kind::RESTORE_TASK:
          state.graph.tasks[entry.index] = std::move(entry.old_task);
          break;
      }
    }
  }
};

struct TaskBRCompilerBudget {
  int recursion_calls = 0;
  long long total_recursion_calls = 0;
  int branch_calls = 0;
  bool recursion_exhausted = false;
  bool cutoff = false;
  long effect_conflicts = 0;
  long candidate_backtracks = 0;
  long first_choice_fallbacks = 0;
};

inline bool task_effects_conflict(const TaskId& a, const TaskId& b)
{
  if (a == b) return false;
  if (a.shelf == b.shelf) return true;
  return a.to == b.to;
}

inline bool adjacent_cells(const DDGrid& grid, int from, int to);

struct OrderedShelfCandidates {
  std::array<int, 4> endpoints{};
  std::array<int, 4> first_steps{};
  std::array<int, 4> route_sizes{};
  std::array<int, 4> route_slots{{-1, -1, -1, -1}};
  std::vector<std::vector<int>> explicit_routes;
  int count = 0;
  bool cutoff = false;

  StorageTransferCandidate candidate(int index) const
  {
    const int route_slot = route_slots[index];
    return StorageTransferCandidate{
        endpoints[index], first_steps[index], route_sizes[index],
        route_slot >= 0 ? &explicit_routes[route_slot] : nullptr};
  }
};

inline std::vector<StorageTransfer> reachable_storage_transfers(
    const DDInstance& ins, int from,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  std::vector<StorageTransfer> transfers;
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return transfers;
  if (from < 0 || from >= ins.grid.size() || ins.grid.is_wall(from))
    return transfers;

  std::vector<int> parent(ins.grid.size(), -2);
  std::deque<int> queue;
  parent[from] = -1;
  queue.push_back(from);
  std::map<int, StorageTransfer> route_by_endpoint;
  while (!queue.empty()) {
    if (expired()) return transfers;
    const int cell = queue.front();
    queue.pop_front();
    int raw_neighbors[4];
    const int count = ins.grid.neighbors(cell, raw_neighbors);
    std::array<int, 4> neighbors{};
    std::copy(raw_neighbors, raw_neighbors + count, neighbors.begin());
    std::sort(neighbors.begin(), neighbors.begin() + count);
    for (int index = 0; index < count; ++index) {
      if (expired()) return transfers;
      const int next = neighbors[index];
      if (next == from) continue;
      if (ins.can_store_shelf(next)) {
        if (route_by_endpoint.count(next) != 0) continue;
        std::vector<int> route;
        for (int cursor = cell; cursor >= 0; cursor = parent[cursor])
          route.push_back(cursor);
        std::reverse(route.begin(), route.end());
        route.push_back(next);
        route_by_endpoint.emplace(
            next, StorageTransfer{next, std::move(route)});
        continue;
      }
      // Discovery is topological.  A shelf occupying a transit cell is a
      // temporal blocker, not evidence that the endpoint is unreachable.
      if (parent[next] != -2) continue;
      parent[next] = cell;
      queue.push_back(next);
    }
  }
  transfers.reserve(route_by_endpoint.size());
  for (auto& entry : route_by_endpoint) {
    if (expired()) return {};
    transfers.push_back(std::move(entry.second));
  }
  return transfers;
}

inline std::vector<StorageTransfer> reachable_storage_transfers(
    const DDInstance& ins, const AbstractUpperState&, int from,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  return reachable_storage_transfers(
      ins, from, deadline, cutoff);
}

inline std::vector<StorageTransferArc>
nearest_channel_storage_arcs(
    const DDInstance& ins, int source,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  std::vector<StorageTransferArc> arcs;
  if (expired()) return arcs;
  if (source < 0 || source >= ins.grid.size() ||
      !ins.can_store_shelf(source))
    return arcs;

  int raw_neighbors[4];
  const int neighbor_count =
      ins.grid.neighbors(source, raw_neighbors);
  std::array<int, 4> neighbors{};
  std::copy(
      raw_neighbors, raw_neighbors + neighbor_count,
      neighbors.begin());
  std::sort(
      neighbors.begin(), neighbors.begin() + neighbor_count);

  // Keep every one-step storage move.  For each distinct channel entrance,
  // stop at the first storage layer reached by BFS and keep every endpoint
  // tied at that minimum distance.  The exhaustive transfer routes remain
  // in StorageTransferTopology::transfers; only the vacancy-potential graph
  // is sparse.
  std::map<int, int> loaded_steps_by_endpoint;
  for (int index = 0; index < neighbor_count; ++index) {
    if (expired()) return {};
    const int entrance = neighbors[index];
    if (ins.can_store_shelf(entrance)) {
      loaded_steps_by_endpoint[entrance] = 1;
      continue;
    }

    std::vector<uint8_t> seen(ins.grid.size(), 0);
    std::vector<int> distance(ins.grid.size(), -1);
    std::deque<int> queue;
    seen[entrance] = 1;
    distance[entrance] = 1;
    queue.push_back(entrance);
    int nearest_loaded_steps = ClearanceCost::INF;
    std::vector<int> nearest_endpoints;
    while (!queue.empty()) {
      if (expired()) return {};
      const int cell = queue.front();
      queue.pop_front();
      if (distance[cell] + 1 > nearest_loaded_steps)
        continue;

      int raw_next[4];
      const int next_count =
          ins.grid.neighbors(cell, raw_next);
      std::array<int, 4> next_cells{};
      std::copy(
          raw_next, raw_next + next_count, next_cells.begin());
      std::sort(
          next_cells.begin(), next_cells.begin() + next_count);
      for (int next_index = 0;
           next_index < next_count; ++next_index) {
        if (expired()) return {};
        const int next = next_cells[next_index];
        if (next == source) continue;
        if (ins.can_store_shelf(next)) {
          const int loaded_steps = distance[cell] + 1;
          if (loaded_steps < nearest_loaded_steps) {
            nearest_loaded_steps = loaded_steps;
            nearest_endpoints.clear();
          }
          if (loaded_steps == nearest_loaded_steps)
            nearest_endpoints.push_back(next);
          continue;
        }
        if (seen[next] ||
            distance[cell] + 1 >= nearest_loaded_steps)
          continue;
        seen[next] = 1;
        distance[next] = distance[cell] + 1;
        queue.push_back(next);
      }
    }
    std::sort(
        nearest_endpoints.begin(), nearest_endpoints.end());
    nearest_endpoints.erase(
        std::unique(
            nearest_endpoints.begin(), nearest_endpoints.end()),
        nearest_endpoints.end());
    for (const int endpoint : nearest_endpoints) {
      const auto found =
          loaded_steps_by_endpoint.find(endpoint);
      if (found == loaded_steps_by_endpoint.end() ||
          nearest_loaded_steps < found->second)
        loaded_steps_by_endpoint[endpoint] =
            nearest_loaded_steps;
    }
  }

  arcs.reserve(loaded_steps_by_endpoint.size());
  for (const auto& [endpoint, loaded_steps] :
       loaded_steps_by_endpoint)
    arcs.push_back(
        StorageTransferArc{
            source, endpoint, loaded_steps});
  return arcs;
}

inline StorageTransferTopology build_storage_transfer_topology(
    const DDInstance& ins, const Deadline* deadline = nullptr,
    bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  StorageTransferTopology topology;
  topology.transfers.resize(ins.grid.size());
  topology.outgoing.resize(ins.grid.size());
  topology.reverse.resize(ins.grid.size());
  for (int source = 0; source < ins.grid.size(); ++source) {
    if (expired()) return topology;
    if (!ins.can_store_shelf(source)) continue;
    bool transfer_cutoff = false;
    auto transfers = reachable_storage_transfers(
        ins, source, deadline, &transfer_cutoff);
    if (transfer_cutoff) {
      if (cutoff != nullptr) *cutoff = true;
      return topology;
    }
    bool arc_cutoff = false;
    const auto arcs = nearest_channel_storage_arcs(
        ins, source, deadline, &arc_cutoff);
    if (arc_cutoff) {
      if (cutoff != nullptr) *cutoff = true;
      return topology;
    }
    topology.transfers[source] = std::move(transfers);
    for (const auto& arc : arcs) {
      if (expired()) return topology;
      topology.outgoing[source].push_back(arc);
      topology.reverse[arc.endpoint].push_back(arc);
    }
  }
  const auto arc_order = [](const StorageTransferArc& a,
                            const StorageTransferArc& b) {
    return std::tie(a.source, a.endpoint, a.loaded_steps) <
           std::tie(b.source, b.endpoint, b.loaded_steps);
  };
  for (auto& arcs : topology.outgoing)
    std::sort(arcs.begin(), arcs.end(), arc_order);
  for (auto& arcs : topology.reverse)
    std::sort(arcs.begin(), arcs.end(), arc_order);
  return topology;
}

inline const std::vector<StorageTransfer>&
storage_transfers_from_topology(
    const StorageTransferTopology& topology, int source)
{
  static const std::vector<StorageTransfer> empty;
  if (source < 0 ||
      source >= (int)topology.transfers.size())
    return empty;
  return topology.transfers[source];
}

inline bool vacancy_topology_uses_unit_cost_bfs(
    const StorageTransferTopology& topology)
{
  for (const auto& arcs : topology.outgoing)
    for (const auto& arc : arcs)
      if (arc.loaded_steps != 1)
        return false;
  return true;
}

inline void build_vacancy_potential_layer(
    const DDInstance& ins, const AbstractUpperState& upper,
    const StorageTransferTopology& topology,
    const std::vector<uint8_t>* excluded_sources,
    const std::vector<uint8_t>* settled_target_at,
    std::vector<ClearanceCost>& cost,
    std::vector<int>& next_vacancy_cell,
    std::vector<int>& vacancy_source_cell,
    std::vector<int>& settled_target_moves,
    const Deadline* deadline, bool* cutoff)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return;
  uint64_t deadline_operations = 0;
  const auto periodic_expired = [&]() {
    ++deadline_operations;
    return (deadline_operations & 255) == 0 && expired();
  };
  cost.assign(
      ins.grid.size(), ClearanceCost::infinity());
  next_vacancy_cell.assign(ins.grid.size(), -1);
  vacancy_source_cell.assign(ins.grid.size(), -1);
  if (topology.reverse.size() !=
      (size_t)ins.grid.size())
    throw std::invalid_argument(
        "build_vacancy_potential: topology size mismatch");
  if (excluded_sources != nullptr &&
      excluded_sources->size() != (size_t)ins.grid.size())
    throw std::invalid_argument(
        "build_vacancy_potential: source-mask size mismatch");
  if (settled_target_at != nullptr &&
      settled_target_at->size() != (size_t)ins.grid.size())
    throw std::invalid_argument(
        "build_vacancy_potential: settled-target mask size mismatch");
  settled_target_moves.assign(
      ins.grid.size(), ClearanceCost::INF);
  std::vector<int> sources;
  sources.reserve(ins.grid.size());
  for (int cell = 0; cell < ins.grid.size(); ++cell) {
    if (periodic_expired()) return;
    // Vacancy-source discovery is accounted for by PairCost's separate
    // clearance-influence dependency.  Reading the raw occupancy here
    // avoids turning this implementation-wide scan into a false direct
    // dependency on every storage cell.
    if (!ins.can_store_shelf(cell) ||
        upper.occupant[cell] >= 0)
      continue;
    if (excluded_sources != nullptr &&
        (*excluded_sources)[cell])
      continue;
    cost[cell] = ClearanceCost::zero();
    vacancy_source_cell[cell] = cell;
    settled_target_moves[cell] = 0;
    sources.push_back(cell);
  }

  if (settled_target_at == nullptr &&
      vacancy_topology_uses_unit_cost_bfs(topology)) {
    std::deque<int> queue(sources.begin(), sources.end());
    const ClearanceCost unit_edge{3, 1, 1};
    while (!queue.empty()) {
      if (periodic_expired()) return;
      const int cell = queue.front();
      queue.pop_front();
      const ClearanceCost candidate = cost[cell] + unit_edge;
      for (const auto& arc : topology.reverse[cell]) {
        if (periodic_expired()) return;
        const int next = arc.source;
        const bool better = candidate < cost[next];
        const bool stable_tie =
            candidate == cost[next] &&
            (next_vacancy_cell[next] < 0 ||
             cell < next_vacancy_cell[next]);
        const bool same_path_root_change =
            candidate == cost[next] &&
            next_vacancy_cell[next] == cell &&
            vacancy_source_cell[next] !=
                vacancy_source_cell[cell];
        if (!better && !stable_tie &&
            !same_path_root_change)
          continue;
        cost[next] = candidate;
        settled_target_moves[next] = 0;
        next_vacancy_cell[next] = cell;
        vacancy_source_cell[next] =
            vacancy_source_cell[cell];
        queue.push_back(next);
      }
    }
    (void)expired();
    return;
  }

  using QueueKey = std::tuple<int, int, int, int, int>;
  std::priority_queue<
      QueueKey, std::vector<QueueKey>,
      std::greater<QueueKey>>
      queue;
  for (const int cell : sources)
    queue.emplace(0, 0, 0, 0, cell);
  while (!queue.empty()) {
    if (periodic_expired()) return;
    const auto [
        moved_settled_targets, service_ticks, pushes,
        loaded_steps, cell] =
        queue.top();
    queue.pop();
    const ClearanceCost settled{
        service_ticks, pushes, loaded_steps};
    if (settled != cost[cell] ||
        moved_settled_targets != settled_target_moves[cell])
      continue;
    for (const auto& arc : topology.reverse[cell]) {
      if (periodic_expired()) return;
      const ClearanceCost edge{
          arc.loaded_steps + 2, 1, arc.loaded_steps};
      const ClearanceCost candidate = settled + edge;
      const int next = arc.source;
      const int candidate_settled_targets =
          clearance_cost_add_component(
              moved_settled_targets,
              settled_target_at != nullptr &&
                      (*settled_target_at)[next]
                  ? 1
                  : 0);
      const auto candidate_key = std::tie(
          candidate_settled_targets, candidate.service_ticks,
          candidate.pushes, candidate.loaded_steps);
      const auto current_key = std::tie(
          settled_target_moves[next], cost[next].service_ticks,
          cost[next].pushes, cost[next].loaded_steps);
      const bool better = candidate_key < current_key;
      const bool stable_tie =
          candidate_key == current_key &&
          (next_vacancy_cell[next] < 0 ||
           cell < next_vacancy_cell[next]);
      const bool same_path_root_change =
          candidate_key == current_key &&
          next_vacancy_cell[next] == cell &&
          vacancy_source_cell[next] !=
              vacancy_source_cell[cell];
      if (!better && !stable_tie &&
          !same_path_root_change)
        continue;
      cost[next] = candidate;
      settled_target_moves[next] =
          candidate_settled_targets;
      next_vacancy_cell[next] = cell;
      vacancy_source_cell[next] =
          vacancy_source_cell[cell];
      queue.emplace(
          candidate_settled_targets,
          candidate.service_ticks, candidate.pushes,
          candidate.loaded_steps, next);
    }
  }
  (void)expired();
}

inline VacancyPotential build_vacancy_potential(
    const DDInstance& ins, const AbstractUpperState& upper,
    const StorageTransferTopology& topology,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  VacancyPotential potential;
  build_vacancy_potential_layer(
      ins, upper, topology, nullptr, nullptr, potential.cost,
      potential.next_vacancy_cell,
      potential.vacancy_source_cell,
      potential.settled_target_moves, deadline, cutoff);
  potential.anonymous_cost = potential.cost;
  potential.anonymous_next_vacancy_cell =
      potential.next_vacancy_cell;
  potential.anonymous_vacancy_source_cell =
      potential.vacancy_source_cell;
  potential.anonymous_settled_target_moves =
      potential.settled_target_moves;
  return potential;
}

inline VacancyPotential build_vacancy_potential(
    const DDInstance& ins, const AbstractUpperState& upper,
    const StorageTransferTopology& topology,
    const std::vector<int>& protected_goal_cells,
    const Deadline* deadline = nullptr, bool* cutoff = nullptr)
{
  VacancyPotential potential;
  std::vector<uint8_t> excluded_sources(ins.grid.size(), 0);
  for (const int cell : protected_goal_cells)
    if (cell >= 0 && cell < ins.grid.size())
      excluded_sources[cell] = 1;
  std::vector<uint8_t> settled_target_at(ins.grid.size(), 0);
  const size_t target_count = std::min(
      upper.target_count, (int)protected_goal_cells.size());
  for (size_t target = 0; target < target_count; ++target) {
    const ShelfSelector shelf{
        ShelfSelector::Kind::TARGET, (int)target};
    const int cell = upper.position(shelf);
    if (cell >= 0 &&
        cell == protected_goal_cells[target])
      settled_target_at[cell] = 1;
  }
  build_vacancy_potential_layer(
      ins, upper, topology, nullptr, &settled_target_at,
      potential.cost, potential.next_vacancy_cell,
      potential.vacancy_source_cell,
      potential.settled_target_moves, deadline, cutoff);
  if (cutoff != nullptr && *cutoff) return potential;
  build_vacancy_potential_layer(
      ins, upper, topology, &excluded_sources,
      &settled_target_at,
      potential.anonymous_cost,
      potential.anonymous_next_vacancy_cell,
      potential.anonymous_vacancy_source_cell,
      potential.anonymous_settled_target_moves,
      deadline, cutoff);
  return potential;
}

inline bool update_vacancy_potential_sources(
    const DDInstance& ins,
    const StorageTransferTopology& topology,
    const std::vector<uint64_t>& previous_occupancy,
    const std::vector<uint64_t>& occupancy,
    const VacancyPotential& previous,
    VacancyPotential& out,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return false;
  const size_t cell_count =
      static_cast<size_t>(ins.grid.size());
  const size_t word_count = (cell_count + 63) / 64;
  if (previous_occupancy.size() != word_count ||
      occupancy.size() != word_count ||
      topology.outgoing.size() != cell_count ||
      topology.reverse.size() != cell_count ||
      previous.cost.size() != cell_count ||
      previous.next_vacancy_cell.size() != cell_count ||
      previous.vacancy_source_cell.size() != cell_count ||
      previous.settled_target_moves.size() != cell_count)
    return false;

  uint64_t deadline_operations = 0;
  const auto periodic_expired = [&]() {
    ++deadline_operations;
    return (deadline_operations & 255) == 0 && expired();
  };
  const auto occupied = [](const std::vector<uint64_t>& key,
                           int cell) {
    return (key[cell / 64] >> (cell % 64)) & uint64_t{1};
  };

  std::vector<int> added_sources;
  std::vector<int> removed_sources;
  for (int cell = 0; cell < ins.grid.size(); ++cell) {
    if (periodic_expired()) return false;
    if (!ins.can_store_shelf(cell)) continue;
    const bool was_occupied =
        occupied(previous_occupancy, cell);
    const bool is_occupied = occupied(occupancy, cell);
    if (was_occupied == is_occupied) continue;
    if (was_occupied)
      added_sources.push_back(cell);
    else
      removed_sources.push_back(cell);
  }
  if (added_sources.empty() && removed_sources.empty()) {
    out = previous;
    return true;
  }

  out = previous;
  std::vector<uint8_t> removed(cell_count, 0);
  for (const int cell : removed_sources)
    removed[cell] = 1;

  for (int cell = 0; cell < ins.grid.size(); ++cell) {
    if (periodic_expired()) return false;
    const int source = out.vacancy_source_cell[cell];
    if (source < 0 || !removed[source]) continue;
    out.cost[cell] = ClearanceCost::infinity();
    out.next_vacancy_cell[cell] = -1;
    out.vacancy_source_cell[cell] = -1;
    out.settled_target_moves[cell] =
        ClearanceCost::INF;
  }

  using QueueKey = std::tuple<int, int, int, int>;
  std::priority_queue<
      QueueKey, std::vector<QueueKey>,
      std::greater<QueueKey>>
      queue;
  const auto enqueue = [&](int cell) {
    const auto& cost = out.cost[cell];
    queue.emplace(
        cost.service_ticks, cost.pushes,
        cost.loaded_steps, cell);
  };
  const auto relax = [&](int source, int downstream,
                         const ClearanceCost& candidate) {
    const bool better = candidate < out.cost[source];
    const bool stable_tie =
        candidate == out.cost[source] &&
        (out.next_vacancy_cell[source] < 0 ||
         downstream < out.next_vacancy_cell[source]);
    const bool same_path_root_change =
        candidate == out.cost[source] &&
        out.next_vacancy_cell[source] == downstream &&
        out.vacancy_source_cell[source] !=
            out.vacancy_source_cell[downstream];
    if (!better && !stable_tie &&
        !same_path_root_change)
      return;
    out.cost[source] = candidate;
    out.next_vacancy_cell[source] = downstream;
    out.vacancy_source_cell[source] =
        out.vacancy_source_cell[downstream];
    out.settled_target_moves[source] = 0;
    enqueue(source);
  };

  for (const int cell : added_sources) {
    out.cost[cell] = ClearanceCost::zero();
    out.next_vacancy_cell[cell] = -1;
    out.vacancy_source_cell[cell] = cell;
    out.settled_target_moves[cell] = 0;
    enqueue(cell);
  }

  for (int source = 0; source < ins.grid.size(); ++source) {
    if (periodic_expired()) return false;
    if (out.vacancy_source_cell[source] >= 0 ||
        !ins.can_store_shelf(source))
      continue;
    for (const auto& arc : topology.outgoing[source]) {
      if (periodic_expired()) return false;
      const int downstream = arc.endpoint;
      if (downstream < 0 ||
          downstream >= ins.grid.size() ||
          out.vacancy_source_cell[downstream] < 0)
        continue;
      const ClearanceCost edge{
          arc.loaded_steps + 2, 1, arc.loaded_steps};
      relax(source, downstream,
            edge + out.cost[downstream]);
    }
  }

  while (!queue.empty()) {
    if (periodic_expired()) return false;
    const auto [service_ticks, pushes, loaded_steps, cell] =
        queue.top();
    queue.pop();
    if (out.cost[cell] !=
        ClearanceCost{
            service_ticks, pushes, loaded_steps})
      continue;
    for (const auto& arc : topology.reverse[cell]) {
      if (periodic_expired()) return false;
      const ClearanceCost edge{
          arc.loaded_steps + 2, 1, arc.loaded_steps};
      relax(arc.source, cell, edge + out.cost[cell]);
    }
  }

  out.anonymous_cost = out.cost;
  out.anonymous_next_vacancy_cell =
      out.next_vacancy_cell;
  out.anonymous_vacancy_source_cell =
      out.vacancy_source_cell;
  out.anonymous_settled_target_moves =
      out.settled_target_moves;
  (void)expired();
  return cutoff == nullptr || !*cutoff;
}

}  // namespace carrier_detail
