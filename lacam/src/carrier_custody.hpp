// Part of carrier_guidance.hpp (internal, src/): Epoch churn telemetry, custody + transfer claims, route hints.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_pair_cost.hpp"

namespace carrier_detail {
struct VacancyEpochChurnTelemetry {
  long first_transfer_comparisons = 0;
  long first_transfer_flips = 0;
  long chain_overlap_samples = 0;
  long chain_overlap_intersection = 0;
  long chain_overlap_union = 0;
};

inline VacancyEpochChurnTelemetry compare_vacancy_epoch_churn(
    const ShelfTaskGraph& previous,
    const ShelfTaskGraph& current)
{
  struct RootChain {
    std::optional<TransferKey> first_transfer;
    std::set<TransferKey> transfers;
  };
  const auto summarize = [](const ShelfTaskGraph& graph) {
    std::map<RootDemand, RootChain> roots;
    for (size_t index = 0; index < graph.tasks.size(); ++index) {
      const auto& task = graph.tasks[index];
      const TransferKey key = transfer_key(task);
      for (const auto& root : task.roots) {
        auto& chain = roots[root];
        chain.transfers.insert(key);
        bool has_root_predecessor = false;
        if (index < graph.predecessors.size()) {
          for (const int predecessor : graph.predecessors[index]) {
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
        if (!has_root_predecessor &&
            (!chain.first_transfer.has_value() ||
             key < *chain.first_transfer))
          chain.first_transfer = key;
      }
    }
    for (auto& [unused_root, chain] : roots) {
      (void)unused_root;
      if (!chain.first_transfer.has_value() &&
          !chain.transfers.empty())
        chain.first_transfer = *chain.transfers.begin();
    }
    return roots;
  };

  const auto previous_roots = summarize(previous);
  const auto current_roots = summarize(current);
  VacancyEpochChurnTelemetry out;
  for (const auto& [root, previous_chain] : previous_roots) {
    const auto found = current_roots.find(root);
    if (found == current_roots.end()) continue;
    const auto& current_chain = found->second;
    if (previous_chain.first_transfer.has_value() &&
        current_chain.first_transfer.has_value()) {
      ++out.first_transfer_comparisons;
      out.first_transfer_flips +=
          *previous_chain.first_transfer !=
          *current_chain.first_transfer;
    }

    ++out.chain_overlap_samples;
    auto previous_it = previous_chain.transfers.begin();
    auto current_it = current_chain.transfers.begin();
    while (previous_it != previous_chain.transfers.end() ||
           current_it != current_chain.transfers.end()) {
      if (current_it == current_chain.transfers.end() ||
          (previous_it != previous_chain.transfers.end() &&
           *previous_it < *current_it)) {
        ++out.chain_overlap_union;
        ++previous_it;
      } else if (
          previous_it == previous_chain.transfers.end() ||
          *current_it < *previous_it) {
        ++out.chain_overlap_union;
        ++current_it;
      } else {
        ++out.chain_overlap_intersection;
        ++out.chain_overlap_union;
        ++previous_it;
        ++current_it;
      }
    }
  }
  return out;
}

inline int custody_endpoint(const Custody& custody)
{
  return custody.original_endpoint >= 0
             ? custody.original_endpoint
             : custody.transfer.endpoint;
}

inline StorageTransfer normalized_transfer(const Custody& custody)
{
  StorageTransfer transfer = custody.transfer;
  transfer.endpoint = custody_endpoint(custody);
  return transfer;
}

inline void reanchor_anonymous_custody(Custody& custody)
{
  if (custody.shelf.kind !=
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL)
    return;
  custody.shelf.value = custody.from;
  custody.task_id =
      TaskId{custody.shelf, custody.from, custody.to};
}

inline void ensure_transfer_identity(Custody& custody, int robot,
                                     uint64_t anchor)
{
  if (custody.original_endpoint < 0)
    custody.original_endpoint = custody.transfer.endpoint;
  custody.transfer.endpoint = custody.original_endpoint;
  if (custody.transfer_id.key.source < 0) {
    int source = custody.from;
    if (!custody.transfer.route.empty())
      source = custody.transfer.route.front();
    custody.transfer_id.key =
        TransferKey{custody.shelf, source, custody.original_endpoint};
  }
  if (custody.transfer_id.carrier < 0)
    custody.transfer_id.carrier = robot;
  if (custody.transfer_id.anchor == 0)
    custody.transfer_id.anchor = anchor;
}

inline bool task_matches_active_transfer(
    const ShelfTask& task, const Custody& custody)
{
  if (!custody.transfer_id.valid()) return false;
  return transfer_key(task) == custody.transfer_id.key;
}

inline int compatible_task_index_by_custody(
    const ShelfTaskGraph& graph, const Custody& custody)
{
  for (size_t index = 0; index < graph.tasks.size(); ++index)
    if (task_matches_active_transfer(graph.tasks[index], custody))
      return (int)index;
  return -1;
}

struct ActiveTransferClaims {
  std::vector<uint8_t> endpoint;
  std::vector<uint8_t> transit;

  explicit ActiveTransferClaims(size_t cell_count = 0)
      : endpoint(cell_count, 0), transit(cell_count, 0)
  {
  }
};

inline bool transfer_conflicts_with_claims(
    const ActiveTransferClaims& claims,
    const StorageTransfer& transfer)
{
  if (transfer.endpoint < 0 ||
      transfer.endpoint >= (int)claims.endpoint.size() ||
      claims.endpoint[transfer.endpoint])
    return true;
  return false;
}

inline void add_transfer_claim(
    ActiveTransferClaims& claims,
    const StorageTransfer& transfer)
{
  if (transfer.endpoint >= 0 &&
      transfer.endpoint < (int)claims.endpoint.size())
    claims.endpoint[transfer.endpoint] = 1;
}

inline StorageTransfer remaining_transfer(const Custody& custody)
{
  const auto transfer = normalized_transfer(custody);
  if (custody.transfer_index >= transfer.route.size())
    return StorageTransfer{};
  return StorageTransfer{
      transfer.endpoint,
      std::vector<int>(
          transfer.route.begin() + custody.transfer_index,
          transfer.route.end())};
}

inline ActiveTransferClaims active_transfer_claims(
    int cell_count,
    const std::vector<std::optional<Custody>>& custody_by_robot)
{
  ActiveTransferClaims claims(cell_count);
  for (const auto& custody : custody_by_robot)
    if (custody.has_value())
      add_transfer_claim(claims, remaining_transfer(*custody));
  return claims;
}

inline Custody make_custody(const ShelfTask& task, int task_index)
{
  Custody out;
  out.task_id = task.id;
  out.current_task_index =
      task_index >= 0 ? std::optional<int>(task_index) : std::nullopt;
  out.shelf = task.id.shelf;
  out.from = task.id.from;
  out.to = task.id.to;
  out.roots = task.roots;
  out.priority = task.priority;
  out.transfer = normalized_transfer(task);
  out.transfer_index = 0;
  out.original_endpoint = out.transfer.endpoint;
  out.transfer_id.key = transfer_key(task);
  out.route_status = RouteStatus::OK;
  out.preferred_leg = task.id;
  return out;
}

inline bool task_matches_loaded_shelf(const PhysConfig& physical, int robot,
                                      const TaskId& id)
{
  if (robot < 0 || robot >= (int)physical.robots.size() ||
      robot >= (int)physical.kappa.size() ||
      physical.robots[robot] != id.from)
    return false;
  if (id.shelf.kind == ShelfSelector::Kind::TARGET)
    return physical.kappa[robot] == id.shelf.value;
  return physical.kappa[robot] == KAPPA_ANON;
}

inline bool adjacent_cells(const DDGrid& grid, int from, int to)
{
  return grid.has_edge(from, to);
}

struct RouteHintSearchResult {
  RouteStatus status = RouteStatus::NO_ROUTE;
  std::vector<int> route;
  int expansions = 0;
};

inline RouteHintSearchResult reroute_to_endpoint(
    const DDInstance& ins, const PhysConfig& physical, int robot,
    int endpoint, int expansion_budget = 4096,
    int discouraged_first_step = -1)
{
  RouteHintSearchResult out;
  if (robot < 0 || robot >= (int)physical.robots.size() ||
      robot >= (int)physical.kappa.size() ||
      physical.kappa[robot] == KAPPA_FREE || endpoint < 0 ||
      endpoint >= ins.grid.size() || ins.grid.is_wall(endpoint) ||
      !ins.can_place_movable_shelf(endpoint))
    return out;

  const int source = physical.robots[robot];
  if (source == endpoint) {
    out.status = RouteStatus::ARRIVED;
    out.route = {source};
    return out;
  }
  if (expansion_budget <= 0) {
    out.status = RouteStatus::BUDGET_EXHAUSTED;
    return out;
  }

  std::vector<uint8_t> occupied(ins.grid.size(), 0);
  for (const int cell : ins.fixed_upper_cells)
    occupied[cell] = 1;
  for (size_t target = 0; target < physical.target_pos.size(); ++target) {
    if ((int)target == physical.kappa[robot]) continue;
    const int cell = physical.target_pos[target];
    if (cell >= 0 && cell < (int)occupied.size()) occupied[cell] = 1;
  }
  for (const int cell : physical.anon_occ)
    if (cell >= 0 && cell < (int)occupied.size()) occupied[cell] = 1;
  occupied[source] = 0;

  auto search = [&](bool respect_occupancy,
                    int budget) -> RouteHintSearchResult {
    RouteHintSearchResult result;
    std::vector<int> parent(ins.grid.size(), -2);
    std::deque<int> queue;
    parent[source] = -1;
    queue.push_back(source);
    while (!queue.empty()) {
      if (budget >= 0 && result.expansions >= budget) {
        result.status = RouteStatus::BUDGET_EXHAUSTED;
        return result;
      }
      const int cell = queue.front();
      queue.pop_front();
      ++result.expansions;
      std::vector<int> neighbors = ins.grid.outgoing(cell);
      std::stable_sort(
          neighbors.begin(), neighbors.end(),
          [&](int a, int b) {
            if (cell == source) {
              const bool discouraged_a = a == discouraged_first_step;
              const bool discouraged_b = b == discouraged_first_step;
              if (discouraged_a != discouraged_b)
                return !discouraged_a;
            }
            return a < b;
          });
      for (const int next : neighbors) {
        if (parent[next] != -2) continue;
        if (next != endpoint && ins.can_store_shelf(next)) continue;
        if (respect_occupancy && occupied[next]) continue;
        parent[next] = cell;
        if (next == endpoint) {
          for (int cursor = endpoint; cursor >= 0;
               cursor = parent[cursor])
            result.route.push_back(cursor);
          std::reverse(result.route.begin(), result.route.end());
          result.status = RouteStatus::OK;
          return result;
        }
        queue.push_back(next);
      }
    }
    result.status = RouteStatus::NO_ROUTE;
    return result;
  };

  out = search(/*respect_occupancy=*/true, expansion_budget);
  if (out.status == RouteStatus::OK ||
      out.status == RouteStatus::BUDGET_EXHAUSTED)
    return out;
  const auto topology =
      search(/*respect_occupancy=*/false, /*budget=*/-1);
  out.status = topology.status == RouteStatus::OK
                   ? RouteStatus::TEMPORARILY_BLOCKED
                   : RouteStatus::NO_ROUTE;
  return out;
}

inline void install_route_hint(Custody& custody, int current,
                               const RouteHintSearchResult& hint)
{
  custody.transfer.endpoint = custody_endpoint(custody);
  custody.transfer.route = hint.route;
  custody.transfer_index = 0;
  custody.from = current;
  custody.route_status = hint.status;
  reanchor_anonymous_custody(custody);
  if (hint.route.size() >= 2) {
    custody.to = hint.route[1];
    custody.task_id =
        TaskId{custody.shelf, custody.from, custody.to};
    custody.preferred_leg = custody.task_id;
  } else {
    custody.to = -1;
    custody.task_id =
        TaskId{custody.shelf, custody.from, -1};
    custody.preferred_leg.reset();
  }
}

inline bool custody_physically_valid(const DDInstance& ins,
                                     const PhysConfig& physical, int robot,
                                     const Custody& custody)
{
  if (robot < 0 || robot >= (int)physical.robots.size() ||
      robot >= (int)physical.kappa.size() ||
      physical.robots[robot] != custody.from ||
      physical.kappa[robot] == KAPPA_FREE)
    return false;
  if (custody.shelf.kind == ShelfSelector::Kind::TARGET) {
    if (custody.shelf.value < 0 ||
        custody.shelf.value >= (int)physical.target_pos.size() ||
        physical.kappa[robot] != custody.shelf.value ||
        physical.target_pos[custody.shelf.value] != custody.from)
      return false;
  } else if (physical.kappa[robot] != KAPPA_ANON) {
    return false;
  }
  const int endpoint = custody_endpoint(custody);
  if (endpoint < 0 || endpoint >= ins.grid.size() ||
      !ins.can_place_movable_shelf(endpoint) ||
      custody.transfer.endpoint != endpoint)
    return false;
  if (custody.transfer_id.valid() &&
      (custody.transfer_id.carrier != robot ||
       custody.transfer_id.key.endpoint != endpoint))
    return false;
  return true;
}

inline bool episode_active(const Custody& custody)
{
  return custody.transfer_id.valid() &&
         custody_endpoint(custody) >= 0;
}

inline bool custody_arrived(const PhysConfig& physical, int robot,
                            const Custody& custody)
{
  return robot >= 0 &&
         robot < (int)physical.robots.size() &&
         physical.robots[robot] == custody_endpoint(custody) &&
         custody.route_status == RouteStatus::ARRIVED;
}

inline bool route_hint_usable(const DDInstance& ins,
                              const PhysConfig& physical, int robot,
                              const Custody& custody)
{
  if (!custody_physically_valid(ins, physical, robot, custody) ||
      (custody.route_status != RouteStatus::OK &&
       custody.route_status != RouteStatus::PREFIX) ||
      !custody.preferred_leg.has_value() ||
      custody.preferred_leg->shelf != custody.shelf ||
      custody.preferred_leg->from != custody.from ||
      custody.preferred_leg->to != custody.to ||
      !adjacent_cells(ins.grid, custody.from, custody.to))
    return false;
  const auto transfer = normalized_transfer(custody);
  if (transfer.route.size() < 2 ||
      transfer.route.back() != custody_endpoint(custody) ||
      custody.transfer_index + 1 >= transfer.route.size() ||
      transfer.route[custody.transfer_index] != custody.from ||
      transfer.route[custody.transfer_index + 1] != custody.to)
    return false;
  for (size_t index = 1; index < transfer.route.size(); ++index)
    if (!adjacent_cells(
            ins.grid, transfer.route[index - 1],
            transfer.route[index]))
      return false;

  for (size_t target = 0; target < physical.target_pos.size(); ++target)
    if ((int)target != physical.kappa[robot] &&
        physical.target_pos[target] == custody.to)
      return false;
  if (ins.is_fixed_upper_cell(custody.to))
    return false;
  return !std::binary_search(
      physical.anon_occ.begin(), physical.anon_occ.end(), custody.to);
}

inline std::vector<int> transport_topology_distance(
    const DDInstance& ins, int source, int endpoint)
{
  constexpr int INF = INT_MAX / 4;
  std::vector<int> distance(ins.grid.size(), INF);
  if (source < 0 || source >= ins.grid.size() ||
      endpoint < 0 || endpoint >= ins.grid.size() ||
      ins.grid.is_wall(source) || ins.grid.is_wall(endpoint) ||
      !ins.can_place_movable_shelf(endpoint))
    return distance;
  std::deque<int> queue;
  distance[endpoint] = 0;
  queue.push_back(endpoint);
  while (!queue.empty()) {
    const int cell = queue.front();
    queue.pop_front();
    for (const int next : ins.grid.outgoing(cell)) {
      if (distance[next] < INF) continue;
      if (next != source && ins.can_store_shelf(next)) continue;
      distance[next] = distance[cell] + 1;
      if (next != source) queue.push_back(next);
    }
  }
  return distance;
}

}  // namespace carrier_detail
