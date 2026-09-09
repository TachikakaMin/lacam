// BR labeled-upper search (carrier BRD decomposition upper layer).
// Split from the original br_lacam_upper.cpp.

#include "br_lacam_upper_internal.hpp"

using namespace br_detail;

namespace {

bool adjacent(const DDGrid& grid, int from, int to)
{
  if (from < 0 || from >= grid.size() ||
      to < 0 || to >= grid.size())
    return false;
  int neighbors[4];
  const int count = grid.neighbors(from, neighbors);
  return std::find(neighbors, neighbors + count, to) !=
         neighbors + count;
}

carrier_detail::AbstractUpperState abstract_upper(
    const DDInstance& ins, const BRLabeledUpperState& state)
{
  UpperSignature projection;
  projection.target_pos = state.target_pos;
  projection.anon_pos = state.anonymous_pos;
  std::sort(projection.anon_pos.begin(), projection.anon_pos.end());
  return carrier_detail::make_abstract_upper_state(ins, projection);
}

bool route_is_enumerated(
    const DDInstance& ins,
    const carrier_detail::AbstractUpperState& upper, int from,
    const StorageTransfer& requested, const Deadline* deadline,
    bool* cutoff)
{
  bool transfer_cutoff = false;
  const auto transfers =
      carrier_detail::reachable_storage_transfers(
          ins, upper, from, deadline, &transfer_cutoff);
  if (transfer_cutoff) {
    if (cutoff != nullptr) *cutoff = true;
    return false;
  }
  return std::any_of(
      transfers.begin(), transfers.end(),
      [&](const StorageTransfer& candidate) {
        return candidate.endpoint == requested.endpoint &&
               candidate.route == requested.route;
      });
}

std::optional<UpperShelfHandle> stable_handle(
    const BRLabeledUpperState& state, const ShelfSelector& shelf)
{
  if (shelf.kind == ShelfSelector::Kind::TARGET) {
    if (shelf.value < 0 ||
        shelf.value >= (int)state.target_pos.size())
      return std::nullopt;
    return UpperShelfHandle{
        UpperShelfHandle::Kind::TARGET, shelf.value};
  }
  const auto found = std::find(
      state.anonymous_pos.begin(), state.anonymous_pos.end(),
      shelf.value);
  if (found == state.anonymous_pos.end()) return std::nullopt;
  return UpperShelfHandle{
      UpperShelfHandle::Kind::ANONYMOUS,
      (int)std::distance(state.anonymous_pos.begin(), found)};
}

std::optional<BRUpperSuccessor>
complete_partial_upper_action_with_cache(
    const DDInstance& ins, const BRLabeledUpperState& state,
    const std::vector<int>& tau,
    const BRUpperNodeMetadata& metadata,
    const std::vector<BRUpperConstraintEntry>& forced_constraints,
    BRUpperSearchStats* stats, DDDistCache& upper_wall,
    const carrier_detail::StorageTransferTopology& storage_topology,
    carrier_detail::VacancyGuidanceTelemetry* guidance_telemetry,
    const carrier_detail::VacancyPotential* vacancy_potential,
    const Deadline* deadline, bool* cutoff)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  if (stats != nullptr) ++stats->partial_compiler_calls;
  std::vector<TaskBRForcedEffect> forced_effects;
  forced_effects.reserve(forced_constraints.size());
  for (const auto& forced : forced_constraints) {
    TaskBRForcedEffect effect;
    effect.shelf = epoch_selector(state, forced.shelf);
    effect.kind =
        forced.kind == BRUpperConstraintEntry::Kind::WAIT
            ? TaskBRForcedEffect::Kind::WAIT
            : TaskBRForcedEffect::Kind::TRANSFER;
    effect.transfer = forced.transfer;
    forced_effects.push_back(std::move(effect));
  }

  std::vector<RootDemand> roots;
  for (size_t target = 0;
       target < state.target_pos.size() && target < tau.size();
       ++target)
    if (state.target_pos[target] != tau[target])
      roots.push_back(
          RootDemand{(int)target, tau[target]});
  auto upper = abstract_upper(ins, state);
  const carrier_detail::TaskBRCompilerLimits limits{256, 512};
  bool compiler_cutoff = false;
  const auto graph = carrier_detail::compile_task_br_pibt(
      ins, upper, roots, tau, metadata.target_priority,
      upper_wall, storage_topology, limits, false, &forced_effects,
      deadline, &compiler_cutoff, guidance_telemetry,
      vacancy_potential);
  if (compiler_cutoff) {
    if (cutoff != nullptr) *cutoff = true;
    return std::nullopt;
  }
  if (expired()) return std::nullopt;

  const size_t shelf_count =
      state.target_pos.size() + state.anonymous_pos.size();
  std::vector<std::optional<BRUpperConstraintEntry>> action(
      shelf_count);
  auto install = [&](const BRUpperConstraintEntry& entry) {
    const int index = handle_index(state, entry.shelf);
    if (index < 0 || action[index].has_value()) return false;
    action[index] = entry;
    return true;
  };
  for (const auto& forced : forced_constraints)
    if (!install(forced)) return std::nullopt;

  for (size_t task_index = 0;
       task_index < graph.tasks.size(); ++task_index) {
    if (task_index >= graph.predecessors.size() ||
        !graph.predecessors[task_index].empty())
      continue;
    const auto& task = graph.tasks[task_index];
    const auto handle = stable_handle(state, task.id.shelf);
    if (!handle.has_value()) return std::nullopt;
    const int index = handle_index(state, *handle);
    if (index < 0) return std::nullopt;
    const auto entry = BRUpperConstraintEntry::make_transfer(
        *handle, task.transfer);
    if (action[index].has_value()) {
      const auto& existing = *action[index];
      if (existing.kind != entry.kind ||
          existing.transfer.endpoint != entry.transfer.endpoint ||
          existing.transfer.route != entry.transfer.route)
        return std::nullopt;
      continue;
    }
    action[index] = entry;
  }

  std::vector<BRUpperConstraintEntry> complete;
  complete.reserve(shelf_count);
  for (size_t index = 0; index < shelf_count; ++index) {
    if (action[index].has_value()) {
      complete.push_back(*action[index]);
    } else {
      complete.push_back(
          BRUpperConstraintEntry::make_wait(
              handle_at(state, (int)index)));
    }
  }
  if (stats != nullptr) ++stats->exact_oracle_calls;
  auto successor = validate_complete_upper_action(
      ins, state, complete, deadline, cutoff);
  if (expired()) return std::nullopt;
  if (!successor.has_value() || successor->state == state)
    return std::nullopt;
  return successor;
}

BRUpperNodeMetadata make_metadata(
    const BRLabeledUpperState& state, const std::vector<int>& tau,
    const BRUpperNodeMetadata* parent)
{
  BRUpperNodeMetadata out;
  const size_t target_count = state.target_pos.size();
  out.age.assign(target_count, 0);
  out.target_priority.assign(target_count, 0);

  std::vector<int> active;
  std::vector<int> inactive;
  for (size_t target = 0; target < target_count; ++target) {
    const bool at_goal =
        target < tau.size() && state.target_pos[target] == tau[target];
    if (!at_goal) {
      out.age[target] =
          parent != nullptr && target < parent->age.size()
              ? parent->age[target] + 1
              : 0;
      active.push_back((int)target);
    } else {
      out.age[target] = 0;
      inactive.push_back((int)target);
    }
  }
  std::stable_sort(
      active.begin(), active.end(),
      [&](int a, int b) {
        return out.age[a] != out.age[b]
                   ? out.age[a] > out.age[b]
                   : a < b;
      });
  int priority = (int)target_count;
  for (const int target : active) {
    out.target_priority[target] = priority--;
    out.variable_order.push_back(
        UpperShelfHandle{
            UpperShelfHandle::Kind::TARGET, target});
  }
  for (const int target : inactive)
    out.variable_order.push_back(
        UpperShelfHandle{
            UpperShelfHandle::Kind::TARGET, target});
  for (size_t anonymous = 0;
       anonymous < state.anonymous_pos.size(); ++anonymous)
    out.variable_order.push_back(
        UpperShelfHandle{
            UpperShelfHandle::Kind::ANONYMOUS,
            (int)anonymous});
  return out;
}

struct BRUpperConstraint {
  std::vector<BRUpperConstraintEntry> entries;
  int depth = 0;

  BRUpperConstraint() = default;

  BRUpperConstraint(
      const BRUpperConstraint* parent,
      const BRUpperConstraintEntry& entry)
      : entries(parent->entries), depth(parent->depth + 1)
  {
    entries.push_back(entry);
  }
};

struct BRUpperNode {
  BRLabeledUpperState state;
  BRUpperNode* parent = nullptr;
  BRUpperTransition incoming_transition;
  BRUpperNodeMetadata metadata;
  std::optional<carrier_detail::VacancyPotential>
      vacancy_potential;
  std::queue<BRUpperConstraint*> search_tree;

  BRUpperNode(
      BRLabeledUpperState state_, BRUpperNode* parent_,
      BRUpperTransition incoming_transition_,
      BRUpperNodeMetadata metadata_)
      : state(std::move(state_)),
        parent(parent_),
        incoming_transition(std::move(incoming_transition_)),
        metadata(std::move(metadata_))
  {
    search_tree.push(new BRUpperConstraint());
  }

  ~BRUpperNode()
  {
    while (!search_tree.empty()) {
      delete search_tree.front();
      search_tree.pop();
    }
  }
};

struct BRUpperPath {
  std::vector<BRLabeledUpperState> states;
  std::vector<BRUpperTransition> transitions;
};

struct CarrierBRDomain {
  using Node = BRUpperNode;
  using Constraint = BRUpperConstraint;
  using Successor = BRUpperSuccessor;
  using Key = BRLabeledUpperState;
  using KeyHasher = BRLabeledUpperStateHash;
  using Result = BRUpperPath;

  const DDInstance& ins;
  const BRLabeledUpperState& start;
  const std::vector<int>& tau;
  const Deadline* deadline;
  BRUpperSearchStats& stats;
  DDDistCache upper_wall;
  carrier_detail::StorageTransferTopology storage_topology;
  carrier_detail::VacancyGuidanceTelemetry vacancy_telemetry;

  CarrierBRDomain(
      const DDInstance& ins_, const BRLabeledUpperState& start_,
      const std::vector<int>& tau_, const Deadline* deadline_,
      BRUpperSearchStats& stats_)
      : ins(ins_),
        start(start_),
        tau(tau_),
        deadline(deadline_),
        stats(stats_),
        upper_wall(ins_.grid),
        storage_topology(
            carrier_detail::build_storage_transfer_topology(
                ins_, deadline_))
  {
  }

  bool expired() const { return is_expired(deadline); }

  Node* make_root()
  {
    return new Node(
        start, nullptr, BRUpperTransition(),
        br_upper_root_metadata(start, tau));
  }

  Key node_key(const Node* node) const { return node->state; }

  Key successor_key(const Successor& successor) const
  {
    return successor.state;
  }

  bool is_goal(const Node* node) const
  {
    return node->state.target_pos == tau;
  }

  Result extract_result(const Node* node) const
  {
    Result out;
    while (node != nullptr) {
      out.states.push_back(node->state);
      if (node->parent != nullptr)
        out.transitions.push_back(node->incoming_transition);
      node = node->parent;
    }
    std::reverse(out.states.begin(), out.states.end());
    std::reverse(out.transitions.begin(), out.transitions.end());
    return out;
  }

  void expand_constraint(Node* node, Constraint* constraint)
  {
    if (constraint->depth >=
        (int)node->metadata.variable_order.size())
      return;
    const UpperShelfHandle shelf =
        node->metadata.variable_order[constraint->depth];
    const auto candidates = br_upper_constraint_candidates(
        ins, node->state, shelf, deadline);
    for (const auto& candidate : candidates)
      node->search_tree.push(
          new Constraint(constraint, candidate));
  }

  std::optional<Successor> generate_successor(
      Node* node, Constraint* constraint)
  {
    std::optional<Successor> successor;
    if (constraint->depth ==
        (int)node->metadata.variable_order.size()) {
      ++stats.exact_oracle_calls;
      successor = validate_complete_upper_action(
          ins, node->state, constraint->entries, deadline);
    } else {
      if (!node->vacancy_potential.has_value()) {
        const auto potential_started =
            std::chrono::steady_clock::now();
        bool potential_cutoff = false;
        auto potential =
            carrier_detail::build_vacancy_potential(
                ins, abstract_upper(ins, node->state),
                storage_topology, tau, deadline,
                &potential_cutoff);
        ++vacancy_telemetry.potential_builds;
        vacancy_telemetry.potential_time_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() -
                potential_started)
                .count();
        for (const auto& cost : potential.cost)
          vacancy_telemetry.potential_unreachable_cells +=
              !cost.finite();
        if (potential_cutoff) return std::nullopt;
        node->vacancy_potential = std::move(potential);
      }
      successor = complete_partial_upper_action_with_cache(
          ins, node->state, tau, node->metadata,
          constraint->entries, &stats, upper_wall,
          storage_topology, &vacancy_telemetry,
          &*node->vacancy_potential,
          deadline, nullptr);
    }
    if (!successor.has_value() ||
        successor->state == node->state)
      return std::nullopt;
    return successor;
  }

  Node* make_child(const Successor& successor, Node* parent)
  {
    return new Node(
        successor.state, parent, successor.transition,
        br_upper_child_metadata(
            successor.state, tau, parent->metadata));
  }
};

}  // namespace

int BRLabeledUpperState::position(UpperShelfHandle shelf) const
{
  if (shelf.kind == UpperShelfHandle::Kind::TARGET)
    return shelf.stable_id >= 0 &&
                   shelf.stable_id < (int)target_pos.size()
               ? target_pos[shelf.stable_id]
               : -1;
  return shelf.stable_id >= 0 &&
                 shelf.stable_id < (int)anonymous_pos.size()
             ? anonymous_pos[shelf.stable_id]
             : -1;
}

void BRLabeledUpperState::set_position(UpperShelfHandle shelf, int cell)
{
  if (shelf.kind == UpperShelfHandle::Kind::TARGET) {
    if (shelf.stable_id < 0 ||
        shelf.stable_id >= (int)target_pos.size())
      throw std::out_of_range("unknown target shelf handle");
    target_pos[shelf.stable_id] = cell;
    return;
  }
  if (shelf.stable_id < 0 ||
      shelf.stable_id >= (int)anonymous_pos.size())
    throw std::out_of_range("unknown anonymous shelf handle");
  anonymous_pos[shelf.stable_id] = cell;
}

size_t BRLabeledUpperStateHash::operator()(
    const BRLabeledUpperState& state) const
{
  uint64_t hash = 1469598103934665603ULL;
  auto add = [&](uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      hash ^= (value >> (8 * byte)) & 0xff;
      hash *= 1099511628211ULL;
    }
  };
  add(state.target_pos.size());
  for (const int cell : state.target_pos)
    add(static_cast<uint32_t>(cell));
  add(state.anonymous_pos.size());
  for (const int cell : state.anonymous_pos)
    add(static_cast<uint32_t>(cell));
  return static_cast<size_t>(hash);
}

BRUpperConstraintEntry BRUpperConstraintEntry::make_wait(
    UpperShelfHandle shelf)
{
  BRUpperConstraintEntry out;
  out.shelf = shelf;
  out.kind = Kind::WAIT;
  return out;
}

BRUpperConstraintEntry BRUpperConstraintEntry::make_transfer(
    UpperShelfHandle shelf, StorageTransfer transfer)
{
  BRUpperConstraintEntry out;
  out.shelf = shelf;
  out.kind = Kind::TRANSFER;
  out.transfer = std::move(transfer);
  return out;
}

BRLabeledUpperState br_labeled_initial_state(const DDInstance& ins)
{
  const PhysConfig physical = initial_phys_config(ins);
  return BRLabeledUpperState{
      physical.target_pos, physical.anon_occ};
}

BRUpperNodeMetadata br_upper_root_metadata(
    const BRLabeledUpperState& state, const std::vector<int>& tau)
{
  return make_metadata(state, tau, nullptr);
}

BRUpperNodeMetadata br_upper_child_metadata(
    const BRLabeledUpperState& state, const std::vector<int>& tau,
    const BRUpperNodeMetadata& parent)
{
  return make_metadata(state, tau, &parent);
}

std::vector<BRUpperConstraintEntry> br_upper_constraint_candidates(
    const DDInstance& ins, const BRLabeledUpperState& state,
    UpperShelfHandle shelf, const Deadline* deadline, bool* cutoff)
{
  std::vector<BRUpperConstraintEntry> out;
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return out;
  if (!valid_grounded_state(ins, state)) return out;
  const int from = state.position(shelf);
  if (from < 0) return out;
  out.push_back(BRUpperConstraintEntry::make_wait(shelf));
  const auto upper = abstract_upper(ins, state);
  bool transfer_cutoff = false;
  auto transfers = carrier_detail::reachable_storage_transfers(
      ins, upper, from, deadline, &transfer_cutoff);
  if (transfer_cutoff) {
    if (cutoff != nullptr) *cutoff = true;
    return {};
  }
  for (auto& transfer : transfers) {
    if (expired()) return {};
    const bool route_clear = std::all_of(
        std::next(transfer.route.begin()), transfer.route.end(),
        [&](int cell) { return upper.empty(cell); });
    if (!route_clear) continue;
    out.push_back(BRUpperConstraintEntry::make_transfer(
        shelf, std::move(transfer)));
  }
  return out;
}

std::optional<BRUpperSuccessor> validate_complete_upper_action(
    const DDInstance& ins, const BRLabeledUpperState& state,
    const std::vector<BRUpperConstraintEntry>& constraints,
    const Deadline* deadline, bool* cutoff)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto expired = [&]() {
    if (!is_expired(deadline)) return false;
    if (cutoff != nullptr) *cutoff = true;
    return true;
  };
  if (expired()) return std::nullopt;
  if (!valid_grounded_state(ins, state)) return std::nullopt;
  const size_t shelf_count =
      state.target_pos.size() + state.anonymous_pos.size();
  if (constraints.size() != shelf_count) return std::nullopt;

  std::vector<const BRUpperConstraintEntry*> by_shelf(
      shelf_count, nullptr);
  for (const auto& constraint : constraints) {
    if (expired()) return std::nullopt;
    const int index = handle_index(state, constraint.shelf);
    if (index < 0 || by_shelf[index] != nullptr) return std::nullopt;
    by_shelf[index] = &constraint;
  }
  if (std::any_of(
          by_shelf.begin(), by_shelf.end(),
          [](const auto* entry) { return entry == nullptr; }))
    return std::nullopt;

  std::vector<int> occupant(ins.grid.size(), -1);
  for (size_t index = 0; index < shelf_count; ++index) {
    if (expired()) return std::nullopt;
    const int cell = state.position(handle_at(state, (int)index));
    if (cell < 0 || cell >= ins.grid.size() ||
        occupant[cell] >= 0)
      return std::nullopt;
    occupant[cell] = (int)index;
  }

  const auto upper = abstract_upper(ins, state);
  BRUpperSuccessor result;
  result.state = state;
  std::set<int> endpoints;
  for (size_t index = 0; index < shelf_count; ++index) {
    if (expired()) return std::nullopt;
    const auto& constraint = *by_shelf[index];
    if (constraint.kind == BRUpperConstraintEntry::Kind::WAIT)
      continue;
    if (constraint.kind !=
        BRUpperConstraintEntry::Kind::TRANSFER)
      return std::nullopt;

    const int from = state.position(constraint.shelf);
    const auto& transfer = constraint.transfer;
    if (transfer.route.size() < 2 ||
        transfer.endpoint < 0 ||
        transfer.endpoint >= ins.grid.size() ||
        transfer.route.front() != from ||
        transfer.route.back() != transfer.endpoint ||
        !ins.can_store_shelf(transfer.endpoint))
      return std::nullopt;

    std::set<int> route_cells;
    for (size_t route_index = 0;
         route_index < transfer.route.size(); ++route_index) {
      if (expired()) return std::nullopt;
      const int cell = transfer.route[route_index];
      if (cell < 0 || cell >= ins.grid.size() ||
          ins.grid.is_wall(cell) ||
          !route_cells.insert(cell).second)
        return std::nullopt;
      if (route_index > 0 &&
          !adjacent(
              ins.grid, transfer.route[route_index - 1], cell))
        return std::nullopt;
      if (route_index > 0 && occupant[cell] >= 0)
        return std::nullopt;
    }
    if (!route_is_enumerated(
            ins, upper, from, transfer, deadline, cutoff))
      return std::nullopt;
    if (!endpoints.insert(transfer.endpoint).second)
      return std::nullopt;

    result.state.set_position(
        constraint.shelf, transfer.endpoint);
    result.transition.transfers.push_back(
        AppliedUpperTransfer{
            constraint.shelf, transfer});
  }

  std::sort(
      result.transition.transfers.begin(),
      result.transition.transfers.end(),
      [](const AppliedUpperTransfer& a,
         const AppliedUpperTransfer& b) {
        return a.stable_shelf < b.stable_shelf;
      });
  return result;
}

std::optional<BRUpperSuccessor> complete_partial_upper_action(
    const DDInstance& ins, const BRLabeledUpperState& state,
    const std::vector<int>& tau,
    const BRUpperNodeMetadata& metadata,
    const std::vector<BRUpperConstraintEntry>& forced_constraints,
    BRUpperSearchStats* stats, const Deadline* deadline,
    bool* cutoff)
{
  DDDistCache upper_wall(ins.grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(
          ins, deadline, cutoff);
  if (cutoff != nullptr && *cutoff)
    return std::nullopt;
  return complete_partial_upper_action_with_cache(
      ins, state, tau, metadata, forced_constraints,
      stats, upper_wall, storage_topology, nullptr, nullptr,
      deadline, cutoff);
}

BRUpperSearchResult solve_carrier_br_upper(
    const DDInstance& ins, const BRLabeledUpperState& start,
    const std::vector<int>& tau, const Deadline* deadline)
{
  BRUpperSearchResult result;
  if (!valid_grounded_state(ins, start) ||
      tau.size() != ins.n_targets()) {
    result.exit_reason = BRUpperExitReason::INVALID;
    return result;
  }
  for (size_t target = 0; target < tau.size(); ++target) {
    const auto& goals = ins.target_goal_sets[target];
    if (!std::binary_search(goals.begin(), goals.end(), tau[target])) {
      result.exit_reason = BRUpperExitReason::INVALID;
      return result;
    }
  }

  CarrierBRDomain domain(ins, start, tau, deadline, result.stats);
  const auto outcome = run_lacam_dfs_search(domain);
  result.stats.loop_count = (long)outcome.loop_count;
  result.stats.explored = (long)outcome.explored;
  result.stats.duplicate_pushes =
      (long)outcome.duplicate_pushes;
  result.stats.vacancy_potential_builds =
      domain.vacancy_telemetry.potential_builds;
  result.stats.vacancy_potential_unreachable_cells =
      domain.vacancy_telemetry.potential_unreachable_cells;
  result.stats.clearance_first_choice_fallbacks =
      domain.vacancy_telemetry.first_choice_fallbacks;
  result.stats.vacancy_potential_time_ms =
      domain.vacancy_telemetry.potential_time_ms;
  if (outcome.solution_found) {
    result.exit_reason = BRUpperExitReason::SOLVED;
    result.states = outcome.result.states;
    result.transitions = outcome.result.transitions;
  } else if (outcome.expired) {
    result.exit_reason = BRUpperExitReason::TIMEOUT;
  } else {
    result.exit_reason = BRUpperExitReason::EXHAUSTED;
  }
  return result;
}
