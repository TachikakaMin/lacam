// Part of carrier_guidance.hpp (internal, src/): Vacancy potential cache, shelf candidate ordering, task-BR PIBT resolve.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_storage_transfer.hpp"

namespace carrier_detail {
struct VacancyPotentialCache {
  explicit VacancyPotentialCache(size_t cell_count)
      : max_entries(std::max<size_t>(
            1, 262144 / std::max<size_t>(1, cell_count)))
  {
  }

  size_t max_entries = 1;
  std::map<std::vector<uint64_t>, VacancyPotential> entries;
  VacancyPotential uncached;
  std::vector<uint64_t> previous_key;
  const VacancyPotential* previous_potential = nullptr;
  long builds = 0;
  long hits = 0;
  long incremental_updates = 0;
  long unreachable_cells = 0;
  long recursion_calls = 0;
  long candidate_backtracks = 0;
  long first_choice_fallbacks = 0;
  double build_ms = 0;
};

struct VacancyGuidanceTelemetry {
  long potential_builds = 0;
  long potential_unreachable_cells = 0;
  long first_choice_fallbacks = 0;
  double potential_time_ms = 0;
  long first_transfer_comparisons = 0;
  long first_transfer_flips = 0;
  long chain_overlap_samples = 0;
  long chain_overlap_intersection = 0;
  long chain_overlap_union = 0;
  long upper_epoch_cache_evictions = 0;
};

inline std::vector<uint64_t> vacancy_potential_key(
    const DDInstance& ins,
    const AbstractUpperState& upper)
{
  std::vector<uint64_t> key(
      (upper.occupant.size() + 63) / 64, 0);
  for (size_t cell = 0; cell < upper.occupant.size(); ++cell)
    if (ins.can_place_movable_shelf(
            static_cast<int>(cell)) &&
        upper.occupant[cell] != AbstractUpperState::EMPTY)
      key[cell / 64] |= uint64_t{1} << (cell % 64);
  return key;
}

inline size_t vacancy_potential_key_difference(
    const std::vector<uint64_t>& a,
    const std::vector<uint64_t>& b)
{
  if (a.size() != b.size())
    return std::numeric_limits<size_t>::max();
  size_t difference = 0;
  for (size_t word = 0; word < a.size(); ++word)
    difference += static_cast<size_t>(
        __builtin_popcountll(a[word] ^ b[word]));
  return difference;
}

inline const VacancyPotential& cached_vacancy_potential(
    const DDInstance& ins, const AbstractUpperState& upper,
    const StorageTransferTopology& topology,
    VacancyPotentialCache& cache,
    const Deadline* deadline = nullptr,
    bool* cutoff = nullptr)
{
  if (cutoff != nullptr) *cutoff = false;
  const auto key = vacancy_potential_key(ins, upper);
  const auto found = cache.entries.find(key);
  if (found != cache.entries.end()) {
    ++cache.hits;
    cache.previous_key = key;
    cache.previous_potential = &found->second;
    return found->second;
  }

  const auto started = std::chrono::steady_clock::now();
  bool build_cutoff = false;
  VacancyPotential potential;
  const VacancyPotential* incremental_source = nullptr;
  const std::vector<uint64_t>* incremental_key = nullptr;
  constexpr size_t MAX_INCREMENTAL_CHANGES = 8;
  if (cache.previous_potential != nullptr &&
      vacancy_potential_key_difference(
          cache.previous_key, key) <=
          MAX_INCREMENTAL_CHANGES) {
    incremental_source = cache.previous_potential;
    incremental_key = &cache.previous_key;
  } else {
    size_t best_difference = MAX_INCREMENTAL_CHANGES + 1;
    for (const auto& entry : cache.entries) {
      const size_t difference =
          vacancy_potential_key_difference(
              entry.first, key);
      if (difference < best_difference) {
        best_difference = difference;
        incremental_key = &entry.first;
        incremental_source = &entry.second;
      }
    }
  }
  bool updated = false;
  if (incremental_source != nullptr &&
      incremental_key != nullptr) {
    updated = update_vacancy_potential_sources(
        ins, topology, *incremental_key, key,
        *incremental_source, potential,
        deadline, &build_cutoff);
    if (updated) ++cache.incremental_updates;
  }
  if (!updated && !build_cutoff)
    potential = build_vacancy_potential(
        ins, upper, topology, deadline, &build_cutoff);
  cache.build_ms +=
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started)
          .count();
  ++cache.builds;
  for (const auto& cost : potential.cost)
    cache.unreachable_cells += !cost.finite();
  if (build_cutoff) {
    if (cutoff != nullptr) *cutoff = true;
    cache.uncached = std::move(potential);
    cache.previous_key.clear();
    cache.previous_potential = nullptr;
    return cache.uncached;
  }
  if (cache.entries.size() < cache.max_entries) {
    const auto inserted =
        cache.entries.emplace(key, std::move(potential));
    cache.previous_key = key;
    cache.previous_potential = &inserted.first->second;
    return inserted.first->second;
  }
  cache.uncached = std::move(potential);
  cache.previous_key = key;
  cache.previous_potential = &cache.uncached;
  return cache.uncached;
}

inline int candidate_settled_target_moves(
    const AbstractUpperState& upper,
    const ShelfSelector& shelf,
    const StorageTransferCandidate& candidate,
    const std::vector<int>* tau,
    const VacancyPotential& potential,
    bool anonymous_goal_avoiding)
{
  const auto& downstream =
      anonymous_goal_avoiding &&
              !potential.anonymous_settled_target_moves.empty()
          ? potential.anonymous_settled_target_moves
          : potential.settled_target_moves;
  if (candidate.endpoint < 0 ||
      candidate.endpoint >= (int)downstream.size())
    return ClearanceCost::INF;
  int moved = downstream[candidate.endpoint];
  if (moved >= ClearanceCost::INF) return moved;
  if (tau != nullptr &&
      shelf.kind == ShelfSelector::Kind::TARGET &&
      shelf.value >= 0 && shelf.value < (int)tau->size() &&
      upper.position(shelf) == (*tau)[shelf.value])
    moved = clearance_cost_add_component(moved, 1);
  return moved;
}

inline ClearanceCost candidate_clearance_cost(
    const StorageTransferCandidate& candidate,
    const VacancyPotential& potential,
    bool anonymous_goal_avoiding = false)
{
  const auto& cost =
      anonymous_goal_avoiding &&
              !potential.anonymous_cost.empty()
          ? potential.anonymous_cost
          : potential.cost;
  if (candidate.endpoint < 0 ||
      candidate.endpoint >= (int)cost.size() ||
      candidate.route_size < 2)
    return ClearanceCost::infinity();
  const int loaded_steps = candidate.route_size - 1;
  return ClearanceCost{
             loaded_steps + 2, 1, loaded_steps} +
         cost[candidate.endpoint];
}

inline void record_pair_vacancy_dependency(
    const AbstractUpperState& upper, int endpoint,
    const VacancyPotential& potential,
    bool anonymous_goal_avoiding)
{
  auto* recorder = upper.dependency_recorder;
  if (recorder == nullptr) return;
  if (anonymous_goal_avoiding) {
    recorder->valid = false;
    return;
  }
  if (endpoint < 0 ||
      endpoint >= (int)potential.cost.size() ||
      endpoint >=
          (int)potential.vacancy_source_cell.size() ||
      endpoint >=
          (int)potential.settled_target_moves.size()) {
    recorder->valid = false;
    return;
  }
  const auto& cost = potential.cost[endpoint];
  const int settled =
      potential.settled_target_moves[endpoint];
  const int source =
      potential.vacancy_source_cell[endpoint];
  if ((cost.finite() && settled != 0) ||
      (!cost.finite() &&
       settled < ClearanceCost::INF) ||
      (cost.finite() && source < 0) ||
      (!cost.finite() && source >= 0)) {
    recorder->valid = false;
    return;
  }
  recorder->record_vacancy_read(
      endpoint, source,
      cost.service_ticks, cost.pushes,
      cost.loaded_steps);
}

inline PairCostTable build_pair_cost_table(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall, double alpha, double gamma,
    double delta)
{
  const auto storage_topology =
      build_storage_transfer_topology(ins);
  return build_pair_cost_table(
      ins, upper, upper_wall, storage_topology,
      alpha, gamma, delta);
}

inline LazyPairAssignment build_lazy_pair_cost_assignment(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall, double alpha, double gamma,
    double delta, const Deadline* deadline = nullptr)
{
  bool topology_cutoff = false;
  const auto storage_topology =
      build_storage_transfer_topology(
          ins, deadline, &topology_cutoff);
  if (topology_cutoff) {
    LazyPairAssignment out;
    out.cutoff = true;
    return out;
  }
  VacancyPotentialCache potential_cache(ins.grid.size());
  return build_lazy_pair_cost_assignment(
      ins, upper, upper_wall, storage_topology,
      alpha, gamma, delta, deadline, &potential_cache);
}

using RootTransferContinuity =
    std::map<RootDemand, TransferKey>;

inline int transfer_continuity_rank(
    const RootTransferContinuity* continuity,
    const RootDemand& root, const ShelfSelector& shelf,
    int source, int endpoint)
{
  if (continuity == nullptr) return 1;
  const auto found = continuity->find(root);
  if (found == continuity->end()) return 1;
  return found->second ==
                 TransferKey{shelf, source, endpoint}
             ? 0
             : 1;
}

template <typename CompilerContext>
inline OrderedShelfCandidates ordered_shelf_candidate_window(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const VacancyPotential& vacancy_potential,
    const CompilerContext& context,
    const RootTransferContinuity* continuity = nullptr,
    const Deadline* deadline = nullptr)
{
  if (is_expired(deadline)) {
    OrderedShelfCandidates out;
    out.cutoff = true;
    return out;
  }
  const int from = upper.position(shelf);
  const bool root_shelf =
      shelf.kind == ShelfSelector::Kind::TARGET &&
      shelf.value == root.target;
  const bool anonymous_goal_avoiding =
      !single_root_mode &&
      shelf.kind == ShelfSelector::Kind::ANON_AT_EPOCH_CELL;

  std::vector<int> neighbors = ins.grid.outgoing(from);
  const int neighbor_count =
      static_cast<int>(neighbors.size());
  if (ins.has_adjacent_storage_frontier(from)) {
    using DirectCandidateScore =
        std::array<int, 9>;
    std::vector<DirectCandidateScore> scores(neighbor_count);
    for (int index = 0; index < neighbor_count; ++index) {
      const int endpoint = neighbors[index];
      const int reserved =
          context.destination_reserved(endpoint) ||
                  (context.has_distinct_endpoint_reservations() &&
                   context.distinct_endpoint_reserved(endpoint))
              ? 1
              : 0;
      record_pair_vacancy_dependency(
          upper, endpoint, vacancy_potential,
          anonymous_goal_avoiding);
      const ClearanceCost clearance =
          candidate_clearance_cost(
              StorageTransferCandidate{
                  endpoint, endpoint, 2, nullptr},
              vacancy_potential,
              anonymous_goal_avoiding);
      const int settled_target_moves =
          candidate_settled_target_moves(
              upper, shelf,
              StorageTransferCandidate{
                  endpoint, endpoint, 2, nullptr},
              tau, vacancy_potential,
              anonymous_goal_avoiding);
      const int continuity_rank =
          transfer_continuity_rank(
              continuity, root, shelf, from, endpoint);
      if (root_shelf) {
        const int mission =
            upper_wall.dist(root.goal, endpoint);
        scores[index] = DirectCandidateScore{
            mission, settled_target_moves,
            clearance.service_ticks,
            clearance.pushes, clearance.loaded_steps,
            continuity_rank, reserved, endpoint, endpoint};
      } else if (
          !single_root_mode &&
          shelf.kind == ShelfSelector::Kind::TARGET &&
          tau != nullptr && shelf.value >= 0 &&
          shelf.value < (int)tau->size()) {
        const int mission =
            upper_wall.dist((*tau)[shelf.value], endpoint);
        scores[index] = DirectCandidateScore{
            settled_target_moves,
            clearance.service_ticks, clearance.pushes,
            clearance.loaded_steps, continuity_rank,
            reserved, mission,
            endpoint, endpoint};
      } else {
        int assigned_goal_interference = 0;
        if (!single_root_mode && tau != nullptr)
          assigned_goal_interference =
              std::count(tau->begin(), tau->end(), endpoint);
        scores[index] = DirectCandidateScore{
            assigned_goal_interference,
            settled_target_moves,
            clearance.service_ticks, clearance.pushes,
            clearance.loaded_steps, continuity_rank,
            reserved, endpoint, endpoint};
      }
    }
    for (int index = 1; index < neighbor_count; ++index) {
      const int endpoint = neighbors[index];
      const DirectCandidateScore score = scores[index];
      int insertion = index;
      while (insertion > 0 &&
             score < scores[insertion - 1]) {
        neighbors[insertion] = neighbors[insertion - 1];
        scores[insertion] = scores[insertion - 1];
        --insertion;
      }
      neighbors[insertion] = endpoint;
      scores[insertion] = score;
    }
    OrderedShelfCandidates out;
    out.count = neighbor_count;
    out.endpoints.resize(out.count);
    out.first_steps.resize(out.count);
    out.route_sizes.resize(out.count);
    out.route_slots.assign(out.count, -1);
    for (int index = 0; index < out.count; ++index) {
      const int endpoint = neighbors[index];
      out.endpoints[index] = endpoint;
      out.first_steps[index] = endpoint;
      out.route_sizes[index] = 2;
    }
    return out;
  }

  using CandidateScore =
      std::array<int, 9>;
  struct RankedCandidate {
    CandidateScore score;
    StorageTransfer transfer;
  };
  std::vector<RankedCandidate> ranked;
  const auto consider =
      [&](StorageTransfer transfer) {
    if (transfer.route.size() < 2) return;
    const int endpoint = transfer.endpoint;
    const int first_step = transfer.route[1];
    const int route_size = (int)transfer.route.size();
    const int reserved =
        context.destination_reserved(first_step) ||
                context.endpoint_reserved(endpoint)
            ? 1
            : 0;
    record_pair_vacancy_dependency(
        upper, endpoint, vacancy_potential,
        anonymous_goal_avoiding);
    const ClearanceCost clearance =
        candidate_clearance_cost(
            StorageTransferCandidate{
                endpoint, first_step, route_size,
                &transfer.route},
            vacancy_potential,
            anonymous_goal_avoiding);
    const int settled_target_moves =
        candidate_settled_target_moves(
            upper, shelf,
            StorageTransferCandidate{
                endpoint, first_step, route_size,
                &transfer.route},
            tau, vacancy_potential,
            anonymous_goal_avoiding);
    const int continuity_rank =
        transfer_continuity_rank(
            continuity, root, shelf, from, endpoint);
    CandidateScore score;
    if (root_shelf) {
      const int mission = upper_wall.dist(root.goal, endpoint);
      score = CandidateScore{
          mission, settled_target_moves,
          clearance.service_ticks,
          clearance.pushes, clearance.loaded_steps,
          continuity_rank, reserved, endpoint, first_step};
    } else if (!single_root_mode &&
               shelf.kind == ShelfSelector::Kind::TARGET &&
               tau != nullptr && shelf.value >= 0 &&
               shelf.value < (int)tau->size()) {
      const int own_goal = (*tau)[shelf.value];
      const int mission = upper_wall.dist(own_goal, endpoint);
      score = CandidateScore{
          settled_target_moves,
          clearance.service_ticks, clearance.pushes,
          clearance.loaded_steps, continuity_rank,
          reserved, mission,
          endpoint, first_step};
    } else {
      int assigned_goal_interference = 0;
      if (!single_root_mode && tau != nullptr)
        assigned_goal_interference =
              std::count(tau->begin(), tau->end(), endpoint);
      score = CandidateScore{
          assigned_goal_interference,
          settled_target_moves,
          clearance.service_ticks, clearance.pushes,
          clearance.loaded_steps, continuity_rank,
          reserved, endpoint, first_step};
    }

    auto insert = ranked.begin();
    while (insert != ranked.end() &&
           !(score < insert->score))
      ++insert;
    ranked.insert(
        insert, RankedCandidate{score, std::move(transfer)});
    // Preserve the rectangular four-candidate window, but grow it with
    // the actual local degree on general KMAP topology.
    const size_t candidate_limit =
        std::max<size_t>(4, ins.grid.outgoing(from).size());
    if (ranked.size() > candidate_limit)
      ranked.pop_back();
  };

  const auto& transfers =
      storage_transfers_from_topology(
          storage_topology, from);
  for (const auto& transfer : transfers) {
    if (is_expired(deadline)) {
      OrderedShelfCandidates out;
      out.cutoff = true;
      return out;
    }
    consider(transfer);
  }

  OrderedShelfCandidates out;
  out.count = static_cast<int>(ranked.size());
  out.endpoints.resize(out.count);
  out.first_steps.resize(out.count);
  out.route_sizes.resize(out.count);
  out.route_slots.assign(out.count, -1);
  out.explicit_routes.reserve(out.count);
  for (int index = 0; index < out.count; ++index) {
    auto& transfer = ranked[index].transfer;
    out.endpoints[index] = transfer.endpoint;
    out.first_steps[index] = transfer.route[1];
    out.route_sizes[index] = (int)transfer.route.size();
    if (transfer.route.size() > 2) {
      out.route_slots[index] =
          (int)out.explicit_routes.size();
      out.explicit_routes.push_back(
          std::move(transfer.route));
    }
  }
  return out;
}

template <typename CompilerContext>
inline OrderedShelfCandidates ordered_shelf_candidate_window(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const VacancyPotential& vacancy_potential,
    const CompilerContext& context,
    const RootTransferContinuity* continuity = nullptr,
    const Deadline* deadline = nullptr)
{
  bool topology_cutoff = false;
  const auto storage_topology =
      build_storage_transfer_topology(
          ins, deadline, &topology_cutoff);
  if (topology_cutoff) {
    OrderedShelfCandidates out;
    out.cutoff = true;
    return out;
  }
  return ordered_shelf_candidate_window(
      ins, upper, shelf, root, tau, single_root_mode, upper_wall,
      storage_topology, vacancy_potential, context, continuity,
      deadline);
}

inline std::vector<int> ordered_shelf_candidates(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const VacancyPotential& vacancy_potential,
    const std::map<int, TaskId>& reserved_destination)
{
  struct MapReservationView {
    const std::map<int, TaskId>& reservations;
    bool destination_reserved(int cell) const
    {
      return reservations.count(cell) != 0;
    }
    bool endpoint_reserved(int cell) const
    {
      return reservations.count(cell) != 0;
    }
    bool has_distinct_endpoint_reservations() const
    {
      return false;
    }
    bool distinct_endpoint_reserved(int) const { return false; }
  };
  const auto ordered = ordered_shelf_candidate_window(
      ins, upper, shelf, root, tau, single_root_mode, upper_wall,
      storage_topology, vacancy_potential,
      MapReservationView{reserved_destination});
  std::vector<int> endpoints;
  endpoints.reserve(ordered.count);
  for (int index = 0; index < ordered.count; ++index)
    endpoints.push_back(ordered.endpoints[index]);
  return endpoints;
}

inline std::vector<int> ordered_shelf_candidates(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const VacancyPotential& vacancy_potential,
    const std::map<int, TaskId>& reserved_destination)
{
  const auto storage_topology =
      build_storage_transfer_topology(ins);
  return ordered_shelf_candidates(
      ins, upper, shelf, root, tau, single_root_mode,
      upper_wall, storage_topology, vacancy_potential,
      reserved_destination);
}

inline std::vector<int> ordered_shelf_candidates(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const std::map<int, TaskId>& reserved_destination)
{
  const auto storage_topology =
      build_storage_transfer_topology(ins);
  const auto vacancy_potential =
      build_vacancy_potential(
          ins, upper, storage_topology);
  return ordered_shelf_candidates(
      ins, upper, shelf, root, tau, single_root_mode,
      upper_wall, storage_topology, vacancy_potential,
      reserved_destination);
}

template <typename CompilerContext>
inline std::optional<RotationCandidate> make_rotation_candidate(
    const std::vector<ShelfSelector>& recursion_stack,
    std::vector<ShelfSelector>::const_iterator cycle_begin,
    const CompilerContext& context)
{
  const size_t cycle_size =
      (size_t)std::distance(cycle_begin, recursion_stack.end());
  if (cycle_size < 3) return std::nullopt;

  RotationCandidate rotation;
  rotation.cycle.reserve(cycle_size);
  for (auto it = cycle_begin; it != recursion_stack.end(); ++it) {
    const auto effect = context.shelf_reservation(*it);
    if (!effect.has_value()) return std::nullopt;
    rotation.cycle.push_back(*effect);
  }
  for (size_t index = 0; index < rotation.cycle.size(); ++index)
    if (rotation.cycle[index].to !=
        rotation.cycle[(index + 1) % rotation.cycle.size()].from)
      return std::nullopt;

  const auto first = std::min_element(
      rotation.cycle.begin(), rotation.cycle.end());
  std::rotate(rotation.cycle.begin(), first, rotation.cycle.end());
  return rotation;
}

inline void add_unique_rotation_candidate(
    std::vector<RotationCandidate>& rotations,
    RotationCandidate rotation)
{
  const auto duplicate = std::find_if(
      rotations.begin(), rotations.end(),
      [&](const RotationCandidate& existing) {
        return existing.cycle == rotation.cycle;
      });
  if (duplicate == rotations.end())
    rotations.push_back(std::move(rotation));
}

template <typename CompilerContext>
inline int resolve_shelf_task_br_pibt(
    const DDInstance& ins, const AbstractUpperState& upper,
    const ShelfSelector& shelf, const RootDemand& root, int root_priority,
    const std::vector<int>* tau, bool single_root_mode,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const VacancyPotential& vacancy_potential,
    const TaskBRCompilerLimits& limits,
    TaskBRCompilerBudget& budget,
    CompilerContext& context,
    std::vector<ShelfSelector>& recursion_stack,
    std::vector<RotationCandidate>& encountered_rotations,
    const StorageTransferCandidate* forced_first_transfer = nullptr,
    const Deadline* deadline = nullptr,
    const RootTransferContinuity* continuity = nullptr)
{
  if (is_expired(deadline)) {
    budget.cutoff = true;
    return -1;
  }
  const bool total_recursion_exhausted =
      limits.total_recursion_cap >= 0 &&
      budget.total_recursion_calls >= limits.total_recursion_cap;
  if (budget.recursion_calls >= limits.recursion_cap ||
      total_recursion_exhausted) {
    budget.recursion_exhausted = true;
    return -1;
  }
  ++budget.recursion_calls;
  ++budget.total_recursion_calls;
  const int from = upper.position(shelf);
  if (from < 0) return -1;
  if constexpr (CompilerContext::records_rotations)
    recursion_stack.push_back(shelf);
  context.enter_recursion(shelf);
  auto leave_recursion = [&]() {
    context.leave_recursion(shelf);
    if constexpr (CompilerContext::records_rotations)
      recursion_stack.pop_back();
  };
  const int forced_index =
      context.forced_transfer(shelf);
  if (forced_index >= 0) {
    context.merge_task(
        forced_index, root, root_priority);
    leave_recursion();
    return forced_index;
  }
  OrderedShelfCandidates candidates;
  if (forced_first_transfer == nullptr) {
    candidates = ordered_shelf_candidate_window(
        ins, upper, shelf, root, tau, single_root_mode, upper_wall,
        storage_topology, vacancy_potential, context, continuity,
        deadline);
    if (candidates.cutoff) {
      budget.cutoff = true;
      leave_recursion();
      return -1;
    }
  }

  const int candidate_count =
      forced_first_transfer != nullptr ? 1 : candidates.count;
  const auto record_first_choice_fallback =
      [&](int candidate_index) {
        if (candidate_index == 0 && candidate_count > 1)
          ++budget.first_choice_fallbacks;
      };
  for (int candidate_index = 0;
       candidate_index < candidate_count; ++candidate_index) {
    if (is_expired(deadline)) {
      budget.cutoff = true;
      leave_recursion();
      return -1;
    }
    const StorageTransferCandidate transfer =
        forced_first_transfer != nullptr
            ? *forced_first_transfer
            : candidates.candidate(candidate_index);
    if (transfer.explicit_route == nullptr) {
      assert(
          transfer.route_size == 2 &&
          transfer.first_step == transfer.endpoint &&
          ins.can_place_movable_shelf(
              transfer.endpoint));
    } else {
      const auto& route = *transfer.explicit_route;
      bool route_valid =
          transfer.route_size >= 2 &&
          transfer.first_step >= 0 &&
          ins.can_place_movable_shelf(
              transfer.endpoint) &&
          transfer.route_size ==
              (int)route.size() &&
          route.front() == from &&
          route[1] == transfer.first_step &&
          route.back() == transfer.endpoint;
      for (size_t index = 1;
           index < route.size(); ++index) {
        if (is_expired(deadline)) {
          budget.cutoff = true;
          leave_recursion();
          return -1;
        }
        route_valid &= adjacent_cells(
            ins.grid, route[index - 1], route[index]);
      }
      for (size_t index = 1;
           index + 1 < route.size(); ++index) {
        if (is_expired(deadline)) {
          budget.cutoff = true;
          leave_recursion();
          return -1;
        }
        route_valid &= !ins.can_store_shelf(route[index]);
      }
      if (!route_valid) {
        record_first_choice_fallback(candidate_index);
        continue;
      }
    }

    const int to = transfer.first_step;
    const TaskId effect{shelf, from, to};
    const TransferKey key{shelf, from, transfer.endpoint};
    const int existing = context.find_transfer(key);
    if (existing >= 0) {
      context.merge_task(existing, root, root_priority);
      leave_recursion();
      return existing;
    }

    if (context.shelf_effect_conflicts(shelf, effect)) {
      ++budget.effect_conflicts;
      record_first_choice_fallback(candidate_index);
      continue;
    }
    const bool placement_destination =
        ins.can_place_movable_shelf(to);
    if (placement_destination &&
        context.destination_effect_conflicts(to, effect)) {
      ++budget.effect_conflicts;
      record_first_choice_fallback(candidate_index);
      continue;
    }
    const bool endpoint_conflict =
        transfer.endpoint == to
            ? context.has_distinct_endpoint_reservations() &&
                  context.distinct_endpoint_effect_conflicts(
                      transfer.endpoint, effect)
            : context.endpoint_effect_conflicts(
                  transfer.endpoint, effect);
    if (endpoint_conflict) {
      ++budget.effect_conflicts;
      record_first_choice_fallback(candidate_index);
      continue;
    }

    const size_t checkpoint = context.checkpoint();
    context.reserve_shelf(shelf, effect);
    if (placement_destination)
      context.reserve_destination(to, effect);
    if (transfer.endpoint != to)
      context.reserve_endpoint(transfer.endpoint, effect);
    int must_be_vacated = -1;
    if (transfer.explicit_route == nullptr) {
      if (!upper.empty(transfer.endpoint))
        must_be_vacated = transfer.endpoint;
    } else {
      for (size_t route_index = 1;
           route_index < transfer.explicit_route->size();
           ++route_index) {
        if (is_expired(deadline)) {
          budget.cutoff = true;
          leave_recursion();
          return -1;
        }
        const int route_cell =
            (*transfer.explicit_route)[route_index];
        if (!upper.empty(route_cell)) {
          must_be_vacated = route_cell;
          break;
        }
      }
    }
    int predecessor = -1;
    if (must_be_vacated >= 0) {
      const ShelfSelector* blocker_at =
          upper.shelf_at(must_be_vacated);
      if (blocker_at == nullptr) {
        ++budget.candidate_backtracks;
        record_first_choice_fallback(candidate_index);
        context.rollback(checkpoint);
        continue;
      }
      const ShelfSelector blocker = *blocker_at;
      if (context.recursion_cycle(blocker, recursion_stack)) {
        if constexpr (CompilerContext::records_rotations) {
          const auto cycle_begin = std::find(
              recursion_stack.begin(), recursion_stack.end(), blocker);
          if (const auto rotation = make_rotation_candidate(
                  recursion_stack, cycle_begin, context);
              rotation.has_value())
            add_unique_rotation_candidate(
                encountered_rotations, *rotation);
        }
        ++budget.candidate_backtracks;
        record_first_choice_fallback(candidate_index);
        context.rollback(checkpoint);
        continue;
      }
      predecessor = resolve_shelf_task_br_pibt(
          ins, upper, blocker, root, root_priority, tau, single_root_mode,
          upper_wall, storage_topology, vacancy_potential, limits,
          budget, context,
          recursion_stack,
          encountered_rotations, nullptr, deadline, continuity);
      if (predecessor < 0) {
        ++budget.candidate_backtracks;
        record_first_choice_fallback(candidate_index);
        context.rollback(checkpoint);
        continue;
      }
    }
    const int result = context.add_task(
        key, effect, transfer, from, root, predecessor,
        must_be_vacated, root_priority);
    leave_recursion();
    return result;
  }
  leave_recursion();
  return -1;
}

}  // namespace carrier_detail
