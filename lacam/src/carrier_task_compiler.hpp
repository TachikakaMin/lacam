// Part of carrier_guidance.hpp (internal, src/): Single-root and joint task-BR compilation.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_shelf_candidates.hpp"

namespace carrier_detail {
struct SingleRootReadyResult {
  std::optional<TaskId> ready_effect;
  std::optional<SelectedStorageTransfer> ready_transfer;
  int recursion_calls = 0;
  bool recursion_exhausted = false;
  bool cutoff = false;
  long effect_conflicts = 0;
  long candidate_backtracks = 0;
};

struct SingleRootCompilerScratch {
  static constexpr bool records_rotations = false;

  enum class UndoKind { SHELF, DESTINATION, ENDPOINT };
  struct Undo {
    UndoKind kind = UndoKind::SHELF;
    int index = -1;
  };

  const AbstractUpperState* upper = nullptr;
  std::vector<int> reserved_shelf_to;
  std::vector<uint32_t> reserved_shelf_stamp;
  std::vector<int> reserved_destination_shelf;
  std::vector<uint32_t> reserved_destination_stamp;
  std::vector<int> reserved_endpoint_shelf;
  std::vector<uint32_t> reserved_endpoint_stamp;
  std::vector<uint32_t> active_shelf_stamp;
  std::vector<Undo> undo;
  std::vector<ShelfSelector> recursion_stack;
  std::vector<RotationCandidate> encountered_rotations;
  std::optional<TaskId> ready_effect;
  std::optional<SelectedStorageTransfer> ready_transfer;
  int distinct_endpoint_reservation_count = 0;
  size_t reservation_cell_count = 0;
  uint32_t generation = 0;

  void reset(const AbstractUpperState& upper_, size_t cell_count)
  {
    upper = &upper_;
    reserved_shelf_to.resize(upper_.shelves.size());
    reserved_shelf_stamp.resize(upper_.shelves.size(), 0);
    reserved_destination_shelf.resize(cell_count);
    reserved_destination_stamp.resize(cell_count, 0);
    active_shelf_stamp.resize(upper_.shelves.size(), 0);
    reservation_cell_count = cell_count;
    const size_t max_undo = 3 * upper_.shelves.size();
    if (undo.capacity() < max_undo)
      undo.reserve(max_undo);
    ++generation;
    if (generation == 0) {
      std::fill(
          reserved_shelf_stamp.begin(), reserved_shelf_stamp.end(), 0);
      std::fill(
          reserved_destination_stamp.begin(),
          reserved_destination_stamp.end(), 0);
      std::fill(
          reserved_endpoint_stamp.begin(),
          reserved_endpoint_stamp.end(), 0);
      std::fill(
          active_shelf_stamp.begin(), active_shelf_stamp.end(), 0);
      generation = 1;
    }
    undo.clear();
    recursion_stack.clear();
    encountered_rotations.clear();
    ready_effect.reset();
    ready_transfer.reset();
    distinct_endpoint_reservation_count = 0;
  }

  int shelf_index(const ShelfSelector& shelf) const
  {
    return upper == nullptr ? -1 : upper->shelf_index(shelf);
  }

  size_t checkpoint() const { return undo.size(); }

  void enter_recursion(const ShelfSelector& shelf)
  {
    const int index = shelf_index(shelf);
    if (index >= 0 && index < (int)active_shelf_stamp.size())
      active_shelf_stamp[index] = generation;
  }

  void leave_recursion(const ShelfSelector& shelf)
  {
    const int index = shelf_index(shelf);
    if (index >= 0 && index < (int)active_shelf_stamp.size())
      active_shelf_stamp[index] = 0;
  }

  bool recursion_cycle(
      const ShelfSelector& shelf,
      const std::vector<ShelfSelector>&) const
  {
    const int index = shelf_index(shelf);
    return index >= 0 &&
           index < (int)active_shelf_stamp.size() &&
           active_shelf_stamp[index] == generation;
  }

  int find_transfer(const TransferKey&) const { return -1; }

  int forced_transfer(const ShelfSelector&) const { return -1; }

  std::optional<TaskId> shelf_reservation(
      const ShelfSelector& shelf) const
  {
    const int index = shelf_index(shelf);
    if (index < 0 ||
        index >= (int)reserved_shelf_stamp.size() ||
        reserved_shelf_stamp[index] != generation)
      return std::nullopt;
    return TaskId{
        shelf, upper->positions[index], reserved_shelf_to[index]};
  }

  bool destination_reserved(int cell) const
  {
    return cell >= 0 &&
           cell < (int)reserved_destination_stamp.size() &&
           reserved_destination_stamp[cell] == generation;
  }

  bool endpoint_reserved(int cell) const
  {
    return cell >= 0 &&
           ((cell < (int)reserved_destination_stamp.size() &&
             reserved_destination_stamp[cell] == generation) ||
           (cell < (int)reserved_endpoint_stamp.size() &&
             reserved_endpoint_stamp[cell] == generation));
  }

  bool has_distinct_endpoint_reservations() const
  {
    return distinct_endpoint_reservation_count > 0;
  }

  bool distinct_endpoint_reserved(int cell) const
  {
    return cell >= 0 &&
           cell < (int)reserved_endpoint_stamp.size() &&
           reserved_endpoint_stamp[cell] == generation;
  }

  bool shelf_effect_conflicts(const ShelfSelector& shelf,
                              const TaskId&) const
  {
    const int index = shelf_index(shelf);
    return index >= 0 &&
           index < (int)reserved_shelf_stamp.size() &&
           reserved_shelf_stamp[index] == generation;
  }

  bool destination_effect_conflicts(int cell,
                                    const TaskId& effect) const
  {
    if (cell < 0 ||
        cell >= (int)reserved_destination_stamp.size() ||
        reserved_destination_stamp[cell] != generation)
      return false;
    const int index = reserved_destination_shelf[cell];
    return index != shelf_index(effect.shelf);
  }

  bool endpoint_effect_conflicts(int cell,
                                 const TaskId& effect) const
  {
    if (cell >= 0 &&
        cell < (int)reserved_destination_stamp.size() &&
        reserved_destination_stamp[cell] == generation) {
      const int index = reserved_destination_shelf[cell];
      if (index != shelf_index(effect.shelf)) return true;
    }
    if (cell < 0 ||
        cell >= (int)reserved_endpoint_stamp.size() ||
        reserved_endpoint_stamp[cell] != generation)
      return false;
    return reserved_endpoint_shelf[cell] !=
           shelf_index(effect.shelf);
  }

  bool distinct_endpoint_effect_conflicts(
      int cell, const TaskId& effect) const
  {
    if (cell < 0 ||
        cell >= (int)reserved_endpoint_stamp.size() ||
        reserved_endpoint_stamp[cell] != generation)
      return false;
    return reserved_endpoint_shelf[cell] !=
           shelf_index(effect.shelf);
  }

  void reserve_shelf(const ShelfSelector& shelf, const TaskId& effect)
  {
    const int index = shelf_index(shelf);
    if (index < 0 ||
        index >= (int)reserved_shelf_stamp.size() ||
        reserved_shelf_stamp[index] == generation)
      return;
    undo.push_back(Undo{UndoKind::SHELF, index});
    reserved_shelf_to[index] = effect.to;
    reserved_shelf_stamp[index] = generation;
  }

  void reserve_destination(int cell, const TaskId& effect)
  {
    if (cell < 0 ||
        cell >= (int)reserved_destination_stamp.size() ||
        reserved_destination_stamp[cell] == generation)
      return;
    const int index = shelf_index(effect.shelf);
    if (index < 0) return;
    undo.push_back(Undo{UndoKind::DESTINATION, cell});
    reserved_destination_shelf[cell] = index;
    reserved_destination_stamp[cell] = generation;
  }

  void reserve_endpoint(int cell, const TaskId& effect)
  {
    if (cell < 0 ||
        cell >= (int)reservation_cell_count ||
        (cell < (int)reserved_endpoint_stamp.size() &&
         reserved_endpoint_stamp[cell] == generation))
      return;
    if (reserved_endpoint_stamp.size() < reservation_cell_count) {
      reserved_endpoint_shelf.resize(reservation_cell_count);
      reserved_endpoint_stamp.resize(reservation_cell_count, 0);
    }
    if (cell >= (int)reserved_endpoint_stamp.size() ||
        reserved_endpoint_stamp[cell] == generation)
      return;
    const int index = shelf_index(effect.shelf);
    if (index < 0) return;
    undo.push_back(Undo{UndoKind::ENDPOINT, cell});
    reserved_endpoint_shelf[cell] = index;
    reserved_endpoint_stamp[cell] = generation;
    ++distinct_endpoint_reservation_count;
  }

  void merge_task(int, const RootDemand&, int) {}

  int add_task(const TransferKey&, const TaskId& id,
               const StorageTransferCandidate& transfer,
               int,
               const RootDemand& root, int predecessor,
               int must_be_vacated, int priority)
  {
    if (predecessor < 0 && !ready_effect.has_value()) {
      ready_effect = id;
      ready_transfer = select_storage_transfer(transfer);
    }
    return 0;
  }

  void rollback(size_t checkpoint)
  {
    while (undo.size() > checkpoint) {
      const Undo entry = undo.back();
      undo.pop_back();
      switch (entry.kind) {
        case UndoKind::SHELF:
          reserved_shelf_stamp[entry.index] = 0;
          break;
        case UndoKind::DESTINATION:
          reserved_destination_stamp[entry.index] = 0;
          break;
        case UndoKind::ENDPOINT:
          reserved_endpoint_stamp[entry.index] = 0;
          --distinct_endpoint_reservation_count;
          break;
      }
    }
  }
};

inline SingleRootReadyResult compile_single_root_next_ready_effect(
    const DDInstance& ins, const AbstractUpperState& upper,
    const RootDemand& root, DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const TaskBRCompilerLimits& limits,
    SingleRootCompilerScratch& scratch,
    VacancyPotentialCache* potential_cache,
    const Deadline* deadline = nullptr)
{
  TaskBRCompilerBudget budget;
  scratch.reset(upper, ins.grid.size());
  SingleRootReadyResult out;
  bool potential_cutoff = false;
  VacancyPotential local_potential;
  const VacancyPotential* vacancy_potential = nullptr;
  if (potential_cache != nullptr) {
    vacancy_potential = &cached_vacancy_potential(
        ins, upper, storage_topology, *potential_cache,
        deadline, &potential_cutoff);
  } else {
    local_potential = build_vacancy_potential(
        ins, upper, storage_topology, deadline,
        &potential_cutoff);
    vacancy_potential = &local_potential;
  }
  if (potential_cutoff) {
    out.cutoff = true;
    return out;
  }
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, root.target};
  const int result = resolve_shelf_task_br_pibt(
      ins, upper, shelf, root, 1, nullptr, true, upper_wall,
      storage_topology, *vacancy_potential, limits, budget, scratch,
      scratch.recursion_stack,
      scratch.encountered_rotations, nullptr, deadline);
  if (result >= 0) {
    out.ready_effect = scratch.ready_effect;
    out.ready_transfer = std::move(scratch.ready_transfer);
  }
  out.recursion_calls = budget.recursion_calls;
  out.recursion_exhausted = budget.recursion_exhausted;
  out.cutoff = budget.cutoff;
  out.effect_conflicts = budget.effect_conflicts;
  out.candidate_backtracks = budget.candidate_backtracks;
  if (potential_cache != nullptr) {
    potential_cache->recursion_calls +=
        budget.total_recursion_calls;
    potential_cache->candidate_backtracks +=
        budget.candidate_backtracks;
    potential_cache->first_choice_fallbacks +=
        budget.first_choice_fallbacks;
  }
  return out;
}

inline SingleRootReadyResult compile_single_root_next_ready_effect(
    const DDInstance& ins, const AbstractUpperState& upper,
    const RootDemand& root, DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const TaskBRCompilerLimits& limits,
    SingleRootCompilerScratch& scratch,
    const Deadline* deadline = nullptr)
{
  return compile_single_root_next_ready_effect(
      ins, upper, root, upper_wall, storage_topology, limits,
      scratch, nullptr, deadline);
}

inline SingleRootReadyResult compile_single_root_next_ready_effect(
    const DDInstance& ins, const AbstractUpperState& upper,
    const RootDemand& root, DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const TaskBRCompilerLimits& limits,
    const Deadline* deadline = nullptr)
{
  SingleRootCompilerScratch scratch;
  return compile_single_root_next_ready_effect(
      ins, upper, root, upper_wall, storage_topology, limits,
      scratch, nullptr, deadline);
}

inline SingleRootReadyResult compile_single_root_next_ready_effect(
    const DDInstance& ins, const AbstractUpperState& upper,
    const RootDemand& root, DDDistCache& upper_wall,
    const TaskBRCompilerLimits& limits,
    SingleRootCompilerScratch& scratch,
    const Deadline* deadline = nullptr)
{
  bool topology_cutoff = false;
  const auto storage_topology =
      build_storage_transfer_topology(
          ins, deadline, &topology_cutoff);
  if (topology_cutoff) {
    SingleRootReadyResult out;
    out.cutoff = true;
    return out;
  }
  return compile_single_root_next_ready_effect(
      ins, upper, root, upper_wall, storage_topology,
      limits, scratch, deadline);
}

inline SingleRootReadyResult compile_single_root_next_ready_effect(
    const DDInstance& ins, const AbstractUpperState& upper,
    const RootDemand& root, DDDistCache& upper_wall,
    const TaskBRCompilerLimits& limits,
    const Deadline* deadline = nullptr)
{
  SingleRootCompilerScratch scratch;
  return compile_single_root_next_ready_effect(
      ins, upper, root, upper_wall, limits, scratch, deadline);
}

inline void propagate_root_demands(ShelfTaskGraph& graph,
                                   const std::vector<int>& target_priority)
{
  for (size_t offset = graph.tasks.size(); offset-- > 0;) {
    for (const int predecessor : graph.predecessors[offset])
      for (const auto& root : graph.tasks[offset].roots)
        add_root_demand(graph.tasks[predecessor].roots, root);
  }
  for (auto& task : graph.tasks) {
    task.priority = 0;
    for (const auto& root : task.roots)
      if (root.target >= 0 &&
          root.target < (int)target_priority.size())
        task.priority =
            std::max(task.priority, target_priority[root.target]);
  }
}

struct JointCompileCandidate {
  TaskBRCompilerState state;
  std::vector<uint8_t> success;
  std::vector<int> paused;
  std::vector<long long> remaining_mission_by_root;
  long long remaining_mission_distance = 0;
  long long estimated_shelf_cost = 0;
  uint64_t stable_order = 0;
  bool valid = false;
};

inline bool better_joint_candidate(const JointCompileCandidate& candidate,
                                   const JointCompileCandidate& best,
                                   bool compare_full_root_progress = false)
{
  if (!best.valid) return true;
  if (candidate.success != best.success)
    return std::lexicographical_compare(
        best.success.begin(), best.success.end(), candidate.success.begin(),
        candidate.success.end());
  if (candidate.remaining_mission_distance !=
      best.remaining_mission_distance)
    return candidate.remaining_mission_distance <
           best.remaining_mission_distance;
  if (compare_full_root_progress &&
      candidate.remaining_mission_by_root !=
          best.remaining_mission_by_root)
    return std::lexicographical_compare(
        candidate.remaining_mission_by_root.begin(),
        candidate.remaining_mission_by_root.end(),
        best.remaining_mission_by_root.begin(),
        best.remaining_mission_by_root.end());
  const size_t compared_roots = std::min(
      candidate.remaining_mission_by_root.size(),
      best.remaining_mission_by_root.size());
  for (size_t root = 0; root < compared_roots; ++root) {
    const bool candidate_completes =
        candidate.remaining_mission_by_root[root] == 0;
    const bool best_completes =
        best.remaining_mission_by_root[root] == 0;
    if (candidate_completes != best_completes)
      return candidate_completes;
  }
  if (candidate.estimated_shelf_cost !=
      best.estimated_shelf_cost)
    return candidate.estimated_shelf_cost <
           best.estimated_shelf_cost;
  return candidate.stable_order < best.stable_order;
}

inline ShelfTaskGraph compile_task_br_pibt(
    const DDInstance& ins, const AbstractUpperState& upper,
    const std::vector<RootDemand>& requested_roots,
    const std::vector<int>& tau, const std::vector<int>& target_priority,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const TaskBRCompilerLimits& limits,
    bool single_root_mode,
    const std::vector<TaskBRForcedEffect>* forced_effects = nullptr,
    const Deadline* deadline = nullptr,
    bool* cutoff_out = nullptr,
    VacancyGuidanceTelemetry* telemetry = nullptr,
    const VacancyPotential* prepared_vacancy_potential = nullptr,
    const RootTransferContinuity* continuity = nullptr)
{
  if (cutoff_out != nullptr) *cutoff_out = false;
  if (is_expired(deadline)) {
    if (cutoff_out != nullptr) *cutoff_out = true;
    return ShelfTaskGraph();
  }
  TaskBRCompilerBudget budget;
  struct TelemetryCommit {
    VacancyGuidanceTelemetry* telemetry = nullptr;
    TaskBRCompilerBudget* budget = nullptr;
    ~TelemetryCommit()
    {
      if (telemetry != nullptr && budget != nullptr)
        telemetry->first_choice_fallbacks +=
            budget->first_choice_fallbacks;
    }
  } telemetry_commit{telemetry, &budget};

  VacancyPotential local_vacancy_potential;
  const VacancyPotential* vacancy_potential =
      prepared_vacancy_potential;
  if (vacancy_potential == nullptr) {
    const auto potential_started =
        std::chrono::steady_clock::now();
    bool potential_cutoff = false;
    local_vacancy_potential =
        single_root_mode
            ? build_vacancy_potential(
                  ins, upper, storage_topology, deadline,
                  &potential_cutoff)
            : build_vacancy_potential(
                  ins, upper, storage_topology, tau,
                  deadline, &potential_cutoff);
    if (telemetry != nullptr) {
      ++telemetry->potential_builds;
      telemetry->potential_time_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() -
              potential_started)
              .count();
      for (const auto& cost : local_vacancy_potential.cost)
        telemetry->potential_unreachable_cells +=
            !cost.finite();
    }
    if (potential_cutoff) {
      if (cutoff_out != nullptr) *cutoff_out = true;
      return ShelfTaskGraph();
    }
    vacancy_potential = &local_vacancy_potential;
  }
  std::vector<RootDemand> roots = requested_roots;
  std::stable_sort(roots.begin(), roots.end(), [&](const RootDemand& a,
                                                   const RootDemand& b) {
    const int pa = a.target >= 0 &&
                           a.target < (int)target_priority.size()
                       ? target_priority[a.target]
                       : 0;
    const int pb = b.target >= 0 &&
                           b.target < (int)target_priority.size()
                       ? target_priority[b.target]
                       : 0;
    return pa != pb ? pa > pb : a.target < b.target;
  });
  JointCompileCandidate best;
  size_t storage_cells = 0;
  for (int cell = 0; cell < ins.grid.size(); ++cell)
    storage_cells += ins.can_place_movable_shelf(cell);
  if (upper.shelves.size() > storage_cells)
    throw std::logic_error(
        "compile_task_br_pibt: shelves exceed storage cells");
  const size_t vacancy_count =
      storage_cells - upper.shelves.size();
  // With one vacancy, minimizing every non-zero residual makes a dense
  // displacement chain chase tiny distance improvements instead of
  // finishing a root.  Two or more vacancies provide enough independent
  // routing freedom for the full priority-ordered residual vector to be a
  // useful anti-oscillation tie-break.
  const bool compare_full_root_progress = vacancy_count >= 2;

  if (single_root_mode) {
    JointCompileCandidate candidate;
    candidate.valid = true;
    candidate.success.assign(roots.size(), 0);
    if (!seed_task_br_forced_effects(
            ins, upper, forced_effects, candidate.state))
      return ShelfTaskGraph();
    TaskBRCompilerTransaction transaction(candidate.state);
    if (!roots.empty() && limits.recursion_cap > 0) {
      std::vector<ShelfSelector> stack;
      std::vector<RotationCandidate> encountered_rotations;
      const ShelfSelector shelf{
          ShelfSelector::Kind::TARGET, roots.front().target};
      const int result = resolve_shelf_task_br_pibt(
          ins, upper, shelf, roots.front(),
          roots.front().target < (int)target_priority.size()
              ? target_priority[roots.front().target]
              : 0,
          &tau, true, upper_wall, storage_topology,
          *vacancy_potential, limits,
          budget, transaction, stack,
          encountered_rotations, nullptr, deadline, continuity);
      if (budget.cutoff) {
        if (cutoff_out != nullptr) *cutoff_out = true;
        return ShelfTaskGraph();
      }
      if (result >= 0) {
        candidate.success[0] = 1;
      } else {
        for (const auto& rotation : encountered_rotations)
          transaction.add_rotation(rotation);
        candidate.paused.push_back(roots.front().target);
      }
    } else {
      for (const auto& root : roots)
        candidate.paused.push_back(root.target);
    }
    best = std::move(candidate);
  } else if (limits.backtrack_cap <= 0 || limits.recursion_cap <= 0) {
    best.valid = true;
    best.success.assign(roots.size(), 0);
    if (!seed_task_br_forced_effects(
            ins, upper, forced_effects, best.state))
      return ShelfTaskGraph();
    for (const auto& root : roots) best.paused.push_back(root.target);
  } else {
    TaskBRCompilerState current;
    if (!seed_task_br_forced_effects(
            ins, upper, forced_effects, current))
      return ShelfTaskGraph();
    TaskBRCompilerTransaction transaction(current);
    std::vector<uint8_t> success(roots.size(), 0);
    std::vector<int> selected_root_to(roots.size(), -1);
    std::vector<int> paused;
    uint64_t candidate_sequence = 0;
    // The hard cap is needed when the first candidate windows fail to
    // produce a jointly executable graph.  Once every root has succeeded
    // in one candidate, however, the success vector is already
    // lexicographically maximal (§6.4); keep only a smaller soft window
    // for the remaining distance/work tie-breaks.
    const int soft_backtrack_cap =
        std::min(limits.backtrack_cap,
                 std::max(1, limits.recursion_cap / 2));
    auto all_roots_succeeded = [&]() {
      return best.valid && best.success.size() == roots.size() &&
             std::all_of(best.success.begin(), best.success.end(),
                         [](uint8_t value) { return value != 0; });
    };
    auto active_backtrack_cap = [&]() {
      return all_roots_succeeded() ? soft_backtrack_cap
                                   : limits.backtrack_cap;
    };
    auto record_candidate = [&](std::vector<int> candidate_paused) {
      if (is_expired(deadline)) {
        budget.cutoff = true;
        return;
      }
      JointCompileCandidate candidate;
      candidate.state = current;
      candidate.success = success;
      candidate.paused = std::move(candidate_paused);
      candidate.estimated_shelf_cost =
          (long long)current.graph.tasks.size();
      candidate.stable_order = candidate_sequence++;
      candidate.valid = true;
      candidate.remaining_mission_by_root.assign(roots.size(), 0);
      for (size_t root_index = 0;
           root_index < roots.size(); ++root_index) {
        if (!candidate.success[root_index]) continue;
        const int to = selected_root_to[root_index];
        const int distance =
            to >= 0
                ? upper_wall.dist(
                      roots[root_index].goal, to)
                : INT_MAX;
        const long long finite_distance =
            distance >= INT_MAX / 4
                ? (long long)ins.grid.size() + 1
                : distance;
        candidate.remaining_mission_by_root[root_index] =
            finite_distance;
        candidate.remaining_mission_distance +=
            finite_distance;
        candidate.estimated_shelf_cost += finite_distance;
      }
      if (better_joint_candidate(
              candidate, best, compare_full_root_progress)) {
        best = std::move(candidate);
      }
    };
    std::function<void(size_t)> compile_roots = [&](size_t k) {
      if (is_expired(deadline)) {
        budget.cutoff = true;
        return;
      }
      if (k == roots.size()) {
        record_candidate(paused);
        return;
      }
      if (budget.branch_calls >= active_backtrack_cap()) {
        auto candidate_paused = paused;
        for (size_t r = k; r < roots.size(); ++r)
          candidate_paused.push_back(roots[r].target);
        record_candidate(std::move(candidate_paused));
        return;
      }
      const auto& root = roots[k];
      const ShelfSelector shelf{
          ShelfSelector::Kind::TARGET, root.target};
      const int forced_index =
          transaction.forced_transfer(shelf);
      if (forced_index >= 0) {
        const size_t checkpoint = transaction.checkpoint();
        transaction.merge_task(
            forced_index, root,
            root.target < (int)target_priority.size()
                ? target_priority[root.target]
                : 0);
        success[k] = 1;
        selected_root_to[k] =
            current.graph.tasks[forced_index].transfer.endpoint;
        compile_roots(k + 1);
        selected_root_to[k] = -1;
        success[k] = 0;
        transaction.rollback(checkpoint);
        return;
      }
      const auto options = ordered_shelf_candidate_window(
          ins, upper, shelf, root, &tau, false, upper_wall,
          storage_topology, *vacancy_potential, transaction,
          continuity, deadline);
      if (options.cutoff) {
        budget.cutoff = true;
        return;
      }
      std::vector<RotationCandidate> failed_rotations;
      for (int option_index = 0;
           option_index < options.count; ++option_index) {
        if (is_expired(deadline)) {
          budget.cutoff = true;
          return;
        }
        if (budget.branch_calls >= active_backtrack_cap()) break;
        const auto transfer =
            options.candidate(option_index);
        if (transfer.route_size < 2) continue;
        const int to = transfer.first_step;
        ++budget.branch_calls;
        if (budget.recursion_calls >= limits.recursion_cap)
          budget.recursion_calls = 0;
        const size_t checkpoint = transaction.checkpoint();
        std::vector<ShelfSelector> stack;
        std::vector<RotationCandidate> option_rotations;
        const int result = resolve_shelf_task_br_pibt(
            ins, upper, shelf, root,
            root.target < (int)target_priority.size()
                ? target_priority[root.target]
                : 0,
            &tau, false, upper_wall, storage_topology,
            *vacancy_potential, limits,
            budget, transaction, stack, option_rotations,
            &transfer, deadline, continuity);
        if (budget.cutoff) return;
        const TaskId expected{
            shelf, upper.position(shelf), to};
        if (result >= 0 && current.graph.tasks[result].id == expected) {
          success[k] = 1;
          selected_root_to[k] = transfer.endpoint;
          compile_roots(k + 1);
          if (budget.cutoff) return;
          selected_root_to[k] = -1;
          success[k] = 0;
        } else {
          for (auto& rotation : option_rotations)
            add_unique_rotation_candidate(
                failed_rotations, std::move(rotation));
        }
        transaction.rollback(checkpoint);
      }
      if (budget.branch_calls < active_backtrack_cap()) {
        ++budget.branch_calls;
        const size_t checkpoint = transaction.checkpoint();
        for (const auto& rotation : failed_rotations)
          transaction.add_rotation(rotation);
        paused.push_back(root.target);
        compile_roots(k + 1);
        if (budget.cutoff) return;
        paused.pop_back();
        transaction.rollback(checkpoint);
      }
    };
    compile_roots(0);
  }

  if (budget.cutoff) {
    if (cutoff_out != nullptr) *cutoff_out = true;
    return ShelfTaskGraph();
  }
  if (!best.valid) {
    best.valid = true;
    best.success.assign(roots.size(), 0);
    for (const auto& root : roots) best.paused.push_back(root.target);
  }
  auto graph = std::move(best.state.graph);
  std::sort(best.paused.begin(), best.paused.end());
  best.paused.erase(std::unique(best.paused.begin(), best.paused.end()),
                    best.paused.end());
  graph.paused_roots = std::move(best.paused);
  graph.effect_conflicts = budget.effect_conflicts;
  graph.candidate_backtracks = budget.candidate_backtracks;
  propagate_root_demands(graph, target_priority);
  return graph;
}

inline ShelfTaskGraph compile_task_br_pibt(
    const DDInstance& ins, const AbstractUpperState& upper,
    const std::vector<RootDemand>& requested_roots,
    const std::vector<int>& tau,
    const std::vector<int>& target_priority,
    DDDistCache& upper_wall,
    const TaskBRCompilerLimits& limits,
    bool single_root_mode,
    const std::vector<TaskBRForcedEffect>* forced_effects = nullptr,
    const Deadline* deadline = nullptr,
    bool* cutoff_out = nullptr,
    VacancyGuidanceTelemetry* telemetry = nullptr,
    const RootTransferContinuity* continuity = nullptr)
{
  bool topology_cutoff = false;
  const auto storage_topology =
      build_storage_transfer_topology(
          ins, deadline, &topology_cutoff);
  if (topology_cutoff) {
    if (cutoff_out != nullptr) *cutoff_out = true;
    return ShelfTaskGraph();
  }
  return compile_task_br_pibt(
      ins, upper, requested_roots, tau, target_priority,
      upper_wall, storage_topology, limits, single_root_mode,
      forced_effects, deadline, cutoff_out, telemetry, nullptr,
      continuity);
}

}  // namespace carrier_detail
