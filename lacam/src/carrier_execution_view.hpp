// Part of carrier_guidance.hpp (internal, src/): Execution view reconciliation, ready/preparable tasks, custody recovery.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_joint_transport.hpp"

namespace carrier_detail {
inline bool task_shelf_is_grounded(const DDInstance& ins,
                                   const PhysConfig& physical,
                                   const TaskId& id)
{
  if (id.shelf.kind == ShelfSelector::Kind::TARGET) {
    const int target = id.shelf.value;
    if (target < 0 || target >= (int)ins.n_targets() ||
        physical.target_pos[target] != id.from)
      return false;
    return std::find(physical.kappa.begin(), physical.kappa.end(),
                     target) == physical.kappa.end();
  }
  return id.shelf.value == id.from &&
         std::binary_search(physical.anon_occ.begin(),
                            physical.anon_occ.end(), id.from);
}

inline int carrier_of_task_shelf(const DDInstance& ins,
                                 const PhysConfig& physical,
                                 const TaskId& id)
{
  for (size_t robot = 0; robot < physical.kappa.size(); ++robot) {
    if (physical.robots[robot] != id.from) continue;
    if (id.shelf.kind == ShelfSelector::Kind::TARGET) {
      if (id.shelf.value >= 0 &&
          id.shelf.value < (int)ins.n_targets() &&
          physical.kappa[robot] == id.shelf.value)
        return (int)robot;
    } else if (physical.kappa[robot] == KAPPA_ANON &&
               id.shelf.value == id.from) {
      return (int)robot;
    }
  }
  return -1;
}

inline ExecutionView reconcile_execution_view(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph,
    const std::vector<std::optional<Custody>>& custody_by_robot)
{
  ExecutionView out;
  out.tasks.resize(graph.tasks.size());
  for (size_t index = 0; index < graph.tasks.size(); ++index) {
    const auto& task = graph.tasks[index];
    const TransferKey key = transfer_key(task);
    int shadow_carrier = -1;
    std::optional<TransferId> shadow_transfer;
    for (size_t robot = 0; robot < custody_by_robot.size(); ++robot) {
      if (!custody_by_robot[robot].has_value()) continue;
      const auto& custody = *custody_by_robot[robot];
      if (!custody.transfer_id.valid() ||
          custody.transfer_id.key.shelf != key.shelf)
        continue;
      if (custody.transfer_id.key == key) {
        out.tasks[index] = ExecutionTaskView{
            ExecutionTaskState::ACTIVE, (int)robot,
            custody.transfer_id};
        shadow_carrier = -1;
        break;
      }
      if (shadow_carrier < 0) {
        shadow_carrier = (int)robot;
        shadow_transfer = custody.transfer_id;
      }
    }
    if (out.tasks[index].state == ExecutionTaskState::ACTIVE)
      continue;
    if (shadow_carrier >= 0) {
      out.tasks[index] = ExecutionTaskView{
          ExecutionTaskState::SHADOWED, shadow_carrier,
          shadow_transfer};
      continue;
    }

    int physical_carrier = -1;
    if (task.id.shelf.kind == ShelfSelector::Kind::TARGET) {
      for (size_t robot = 0; robot < physical.kappa.size(); ++robot)
        if (physical.kappa[robot] == task.id.shelf.value) {
          physical_carrier = (int)robot;
          break;
        }
    } else {
      physical_carrier =
          carrier_of_task_shelf(ins, physical, task.id);
    }
    if (physical_carrier >= 0) {
      out.tasks[index] = ExecutionTaskView{
          ExecutionTaskState::SHADOWED, physical_carrier,
          std::nullopt};
      continue;
    }
    out.tasks[index].state =
        task_shelf_is_grounded(ins, physical, task.id)
            ? ExecutionTaskState::PENDING
            : ExecutionTaskState::FULFILLED;
  }
  const auto upper_occupied = upper_occupancy_bitmap(
      ins, make_upper_signature(physical));

  out.causal_conditions.resize(graph.causal_edges.size());
  for (size_t edge_index = 0;
       edge_index < graph.causal_edges.size(); ++edge_index) {
    const auto& edge = graph.causal_edges[edge_index];
    bool consumer_present = false;
    if (edge.consumer >= 0 &&
        edge.consumer < (int)graph.tasks.size() &&
        edge.must_be_vacated >= 0 &&
        edge.must_be_vacated < ins.grid.size()) {
      const auto& consumer = graph.tasks[edge.consumer];
      if (consumer.id.shelf.kind ==
          ShelfSelector::Kind::TARGET) {
        const int target = consumer.id.shelf.value;
        consumer_present =
            target >= 0 &&
            target < (int)physical.target_pos.size() &&
            physical.target_pos[target] ==
                edge.must_be_vacated;
      }
      if (!consumer_present &&
          edge.consumer < (int)out.tasks.size() &&
          out.tasks[edge.consumer].state ==
              ExecutionTaskState::ACTIVE) {
        const int carrier =
            out.tasks[edge.consumer].carrier;
        consumer_present =
            carrier >= 0 &&
            carrier < (int)physical.robots.size() &&
            physical.robots[carrier] ==
                edge.must_be_vacated;
      }
    }
    out.causal_conditions[edge_index].fulfilled =
        edge.must_be_vacated >= 0 &&
        edge.must_be_vacated < ins.grid.size() &&
        (!upper_occupied[edge.must_be_vacated] ||
         consumer_present);
  }
  return out;
}

inline bool task_dependencies_fulfilled(
    const ShelfTaskGraph& graph, int task_index,
    const ExecutionView* execution_view)
{
  if (task_index < 0 ||
      task_index >= (int)graph.predecessors.size())
    return false;
  if (graph.predecessors[task_index].empty()) return true;
  if (execution_view == nullptr) return false;
  std::set<int> represented_predecessors;
  for (size_t edge_index = 0;
       edge_index < graph.causal_edges.size(); ++edge_index) {
    const auto& edge = graph.causal_edges[edge_index];
    if (edge.consumer != task_index) continue;
    represented_predecessors.insert(edge.producer);
    if (edge_index >=
            execution_view->causal_conditions.size() ||
        !execution_view->causal_conditions[edge_index].fulfilled)
      return false;
  }
  for (const int predecessor : graph.predecessors[task_index])
    if (represented_predecessors.count(predecessor) == 0)
      return false;
  return !represented_predecessors.empty();
}

inline std::vector<int> ready_tasks_with_custody(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph,
    const std::vector<std::optional<Custody>>& custody_by_robot,
    const std::vector<uint8_t>& continuation_carrier,
    const ExecutionView* execution_view = nullptr,
    long* candidates_before_claims = nullptr,
    long* claims_filtered = nullptr)
{
  const auto upper = make_upper_signature(physical);
  const auto occupied =
      upper_occupancy_bitmap(ins, upper);

  std::unordered_map<TaskId, int, TaskIdHash> custody_owner;
  for (size_t robot = 0; robot < custody_by_robot.size(); ++robot)
    if (custody_by_robot[robot].has_value())
      custody_owner[custody_by_robot[robot]->task_id] = (int)robot;

  std::vector<int> ready;
  for (size_t index = 0; index < graph.tasks.size(); ++index) {
    const auto& task = graph.tasks[index];
    if (!task_dependencies_fulfilled(
            graph, (int)index, execution_view))
      continue;
    const bool occupied_placement =
        task.id.to >= 0 &&
        task.id.to < (int)occupied.size() &&
        ins.can_place_movable_shelf(task.id.to) &&
        occupied[task.id.to];
    if (task.id.to < 0 || task.id.to >= (int)occupied.size() ||
        occupied_placement || custody_owner.count(task.id))
      continue;

    bool shelf_available =
        task_shelf_is_grounded(ins, physical, task.id);
    if (!shelf_available) {
      const int carrier =
          carrier_of_task_shelf(ins, physical, task.id);
      const bool arrived_continuation =
          carrier >= 0 &&
          carrier < (int)custody_by_robot.size() &&
          custody_by_robot[carrier].has_value() &&
          custody_arrived(
              physical, carrier, *custody_by_robot[carrier]);
      shelf_available =
          carrier >= 0 &&
          carrier < (int)continuation_carrier.size() &&
          continuation_carrier[carrier] &&
          (carrier >= (int)custody_by_robot.size() ||
           !custody_by_robot[carrier].has_value() ||
           arrived_continuation);
    }
    if (shelf_available) ready.push_back((int)index);
  }
  std::stable_sort(ready.begin(), ready.end(), [&](int a, int b) {
    if (graph.tasks[a].priority != graph.tasks[b].priority)
      return graph.tasks[a].priority > graph.tasks[b].priority;
    return graph.tasks[a].id < graph.tasks[b].id;
  });
  if (candidates_before_claims != nullptr)
    *candidates_before_claims = static_cast<long>(ready.size());
  auto claims = active_transfer_claims(
      ins.grid.size(), custody_by_robot);
  std::vector<int> filtered;
  filtered.reserve(ready.size());
  for (const int index : ready) {
    const auto transfer =
        normalized_transfer(graph.tasks[index]);
    if (transfer_conflicts_with_claims(claims, transfer))
      continue;
    filtered.push_back(index);
    add_transfer_claim(claims, transfer);
  }
  if (claims_filtered != nullptr)
    *claims_filtered =
        static_cast<long>(ready.size() - filtered.size());
  return filtered;
}

inline std::vector<int> preparable_tasks_with_executors(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph,
    const ExecutionView& execution_view,
    const std::vector<std::optional<Custody>>& custody_by_robot,
    const std::vector<int>& executable_tasks,
    const std::vector<int>& executable_assignment)
{
  std::vector<uint8_t> is_executable(graph.tasks.size(), 0);
  for (const int index : executable_tasks)
    if (index >= 0 && index < (int)is_executable.size())
      is_executable[index] = 1;
  std::vector<uint8_t> has_executor(graph.tasks.size(), 0);
  for (size_t index = 0;
       index < execution_view.tasks.size() &&
       index < has_executor.size();
       ++index)
    has_executor[index] =
        execution_view.tasks[index].state ==
        ExecutionTaskState::ACTIVE;
  for (const int index : executable_assignment)
    if (index >= 0 && index < (int)has_executor.size())
      has_executor[index] = 1;

  std::vector<int> preparable;
  for (size_t index = 0; index < graph.tasks.size(); ++index) {
    if (is_executable[index] ||
        index >= execution_view.tasks.size() ||
        execution_view.tasks[index].state !=
            ExecutionTaskState::PENDING ||
        !task_shelf_is_grounded(
            ins, physical, graph.tasks[index].id) ||
        index >= graph.predecessors.size() ||
        graph.predecessors[index].empty())
      continue;

    bool has_unsatisfied_condition = false;
    bool all_have_executors = true;
    std::set<int> represented_predecessors;
    for (size_t edge_index = 0;
         edge_index < graph.causal_edges.size(); ++edge_index) {
      const auto& edge = graph.causal_edges[edge_index];
      if (edge.consumer != (int)index) continue;
      represented_predecessors.insert(edge.producer);
      const bool fulfilled =
          edge_index <
              execution_view.causal_conditions.size() &&
          execution_view.causal_conditions[edge_index].fulfilled;
      if (fulfilled) continue;
      has_unsatisfied_condition = true;
      bool producer_has_executor =
          edge.producer >= 0 &&
          edge.producer < (int)has_executor.size() &&
          has_executor[edge.producer];
      if (!producer_has_executor &&
          edge.producer >= 0 &&
          edge.producer < (int)graph.tasks.size() &&
          edge.producer < (int)execution_view.tasks.size()) {
        const auto& producer_view =
            execution_view.tasks[edge.producer];
        const int carrier = producer_view.carrier;
        const Custody* custody =
            carrier >= 0 &&
                    carrier < (int)custody_by_robot.size() &&
                    custody_by_robot[carrier].has_value()
                ? &*custody_by_robot[carrier]
                : nullptr;
        const auto producer_transfer =
            normalized_transfer(graph.tasks[edge.producer]);
        producer_has_executor =
            producer_view.state ==
                ExecutionTaskState::SHADOWED &&
            custody != nullptr &&
            custody->transfer_id.valid() &&
            custody->transfer_id.carrier == carrier &&
            carrier >= 0 &&
            carrier < (int)physical.robots.size() &&
            physical.robots[carrier] ==
                edge.must_be_vacated &&
            custody_physically_valid(
                ins, physical, carrier, *custody) &&
            task_matches_loaded_shelf(
                physical, carrier,
                graph.tasks[edge.producer].id) &&
            custody_endpoint(*custody) ==
                producer_transfer.endpoint &&
            custody_endpoint(*custody) !=
                edge.must_be_vacated &&
            custody->route_status != RouteStatus::ARRIVED &&
            route_hint_usable(
                ins, physical, carrier, *custody) &&
            custody->preferred_leg.has_value() &&
            custody->preferred_leg->from ==
                edge.must_be_vacated &&
            custody->preferred_leg->to !=
                edge.must_be_vacated;
      }
      if (!producer_has_executor)
        all_have_executors = false;
    }
    for (const int predecessor : graph.predecessors[index])
      if (represented_predecessors.count(predecessor) == 0)
        all_have_executors = false;
    if (has_unsatisfied_condition && all_have_executors)
      preparable.push_back((int)index);
  }
  std::stable_sort(
      preparable.begin(), preparable.end(),
      [&](int a, int b) {
        const auto& task_a = graph.tasks[a];
        const auto& task_b = graph.tasks[b];
        if (task_a.priority != task_b.priority)
          return task_a.priority > task_b.priority;
        const auto key_a = transfer_key(task_a);
        const auto key_b = transfer_key(task_b);
        return key_a != key_b ? key_a < key_b : a < b;
      });
  return preparable;
}

struct CustodyRecovery {
  std::vector<std::optional<Custody>> custody_by_robot;
  std::vector<uint8_t> continuation_carrier;
  std::vector<int> previous_loaded_move_from;
  bool transition_valid = false;
};

inline bool exact_ready_binding(const CarrierGuidance& guidance,
                                const TaskId& id,
                                const TransferKey* key,
                                int hinted_index,
                                int* task_index)
{
  if (guidance.upper_epoch == nullptr) return false;
  const auto& graph = guidance.upper_epoch->task_graph;
  const auto matches = [&](int index) {
    return index >= 0 && index < (int)graph.tasks.size() &&
           graph.tasks[index].id == id &&
           (key == nullptr ||
            transfer_key(graph.tasks[index]) == *key) &&
           std::find(
               guidance.ready_tasks.begin(),
               guidance.ready_tasks.end(), index) !=
               guidance.ready_tasks.end();
  };
  if (matches(hinted_index)) {
    if (task_index != nullptr) *task_index = hinted_index;
    return true;
  }
  for (const int index : guidance.ready_tasks)
    if (matches(index)) {
      if (task_index != nullptr) *task_index = index;
      return true;
    }
  return false;
}

inline std::optional<Custody> make_storage_recovery_custody(
    const DDInstance& ins, const PhysConfig& physical, int robot,
    const ShelfTaskGraph& current_graph,
    const std::optional<Custody>& previous_custody,
    const ActiveTransferClaims& claims, int route_budget = 4096,
    int discouraged_first_step = -1)
{
  if (robot < 0 || robot >= (int)physical.robots.size() ||
      robot >= (int)physical.kappa.size() ||
      physical.kappa[robot] == KAPPA_FREE ||
      ins.can_place_movable_shelf(
          physical.robots[robot]))
    return std::nullopt;

  if (previous_custody.has_value()) {
    Custody custody = *previous_custody;
    ensure_transfer_identity(
        custody, robot, phys_config_hash(physical));
    custody.from = physical.robots[robot];
    reanchor_anonymous_custody(custody);
    const int endpoint = custody_endpoint(custody);
    const auto hint = reroute_to_endpoint(
        ins, physical, robot, endpoint, route_budget,
        discouraged_first_step);
    install_route_hint(custody, physical.robots[robot], hint);
    custody.rebind_reason =
        hint.status == RouteStatus::OK ||
                hint.status == RouteStatus::PREFIX ||
                hint.status == RouteStatus::ARRIVED
            ? RebindReason::FORCED_DEVIATION
            : RebindReason::NO_ROUTE;
    const int index =
        compatible_task_index_by_custody(current_graph, custody);
    custody.current_task_index =
        index >= 0 ? std::optional<int>(index) : std::nullopt;
    if (index >= 0) {
      custody.roots = current_graph.tasks[index].roots;
      custody.priority = current_graph.tasks[index].priority;
    }
    if (!custody_physically_valid(ins, physical, robot, custody))
      return std::nullopt;
    return custody;
  }

  const auto upper_signature = make_upper_signature(physical);
  const auto upper =
      make_abstract_upper_state(ins, upper_signature);
  auto transfers = reachable_storage_transfers(
      ins, upper, physical.robots[robot]);
  transfers.erase(
      std::remove_if(
          transfers.begin(), transfers.end(),
          [&](const StorageTransfer& transfer) {
            return transfer.route.size() < 2 ||
                   !upper.empty(transfer.endpoint);
          }),
      transfers.end());
  if (transfers.empty()) return std::nullopt;
  std::stable_sort(
      transfers.begin(), transfers.end(),
      [](const StorageTransfer& a, const StorageTransfer& b) {
        return a.route.size() != b.route.size()
                   ? a.route.size() < b.route.size()
                   : a.endpoint < b.endpoint;
      });
  const auto selected = std::find_if(
      transfers.begin(), transfers.end(),
      [&](const StorageTransfer& transfer) {
        return !transfer_conflicts_with_claims(
            claims, transfer);
      });
  if (selected == transfers.end()) return std::nullopt;

  Custody custody;
  if (physical.kappa[robot] >= 0) {
    custody.shelf = ShelfSelector{
        ShelfSelector::Kind::TARGET, physical.kappa[robot]};
  } else {
    custody.shelf = ShelfSelector{
        ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
        physical.robots[robot]};
  }
  custody.transfer = *selected;
  custody.original_endpoint = selected->endpoint;
  custody.transfer_index = 0;
  custody.from = custody.transfer.route[0];
  custody.to = custody.transfer.route[1];
  reanchor_anonymous_custody(custody);
  custody.task_id =
      TaskId{custody.shelf, custody.from, custody.to};
  custody.preferred_leg = custody.task_id;
  custody.route_status = RouteStatus::OK;
  custody.rebind_reason = RebindReason::FORCED_DEVIATION;
  custody.transfer_id.key =
      TransferKey{
          custody.shelf, custody.from, custody.original_endpoint};
  custody.transfer_id.carrier = robot;
  custody.transfer_id.anchor = phys_config_hash(physical);
  const int index =
      compatible_task_index_by_custody(
          current_graph, custody);
  custody.current_task_index =
      index >= 0 ? std::optional<int>(index) : std::nullopt;
  if (index >= 0) {
    custody.roots = current_graph.tasks[index].roots;
    custody.priority = current_graph.tasks[index].priority;
  }
  if (!custody_physically_valid(ins, physical, robot, custody))
    return std::nullopt;
  return custody;
}

inline CustodyRecovery recover_task_br_custody(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& current_graph, const PhysConfig* previous_physical,
    const CarrierGuidance* previous_guidance,
    const std::vector<Op>* executed_ops, int route_budget = 4096,
    bool force_route_refresh = false)
{
  CustodyRecovery out;
  const size_t robot_count = ins.n_robots();
  out.custody_by_robot.resize(robot_count);
  out.continuation_carrier.assign(robot_count, 0);
  out.previous_loaded_move_from.assign(robot_count, -1);
  if (previous_physical == nullptr || executed_ops == nullptr ||
      previous_physical->robots.size() != robot_count ||
      previous_physical->kappa.size() != robot_count ||
      executed_ops->size() != robot_count)
    return out;
  const auto replayed = apply_ops(ins, *previous_physical, *executed_ops);
  if (!replayed.has_value() || !(*replayed == physical)) return out;
  out.transition_valid = true;

  for (size_t robot = 0; robot < robot_count; ++robot) {
    if (physical.kappa[robot] == KAPPA_FREE) continue;
    const auto& op = (*executed_ops)[robot];
    const int previous_kappa = previous_physical->kappa[robot];

    if (previous_kappa != KAPPA_FREE && op.kind == Op::MOVE) {
      out.continuation_carrier[robot] = 1;
      out.previous_loaded_move_from[robot] =
          previous_physical->robots[robot];
      const std::optional<Custody> previous_custody =
          previous_guidance != nullptr &&
                  robot <
                      previous_guidance->custody_by_robot.size()
              ? previous_guidance->custody_by_robot[robot]
              : std::nullopt;
      if (previous_custody.has_value()) {
        Custody custody = *previous_custody;
        ensure_transfer_identity(
            custody, (int)robot,
            phys_config_hash(*previous_physical));
        const bool followed_preferred_leg =
            previous_physical->robots[robot] == custody.from &&
            physical.robots[robot] == op.to &&
            ((custody.preferred_leg.has_value() &&
              custody.preferred_leg->from == custody.from &&
              custody.preferred_leg->to == op.to) ||
             (!custody.preferred_leg.has_value() &&
              custody.to == op.to));
        const int endpoint = custody_endpoint(custody);
        if (physical.robots[robot] == endpoint) {
          install_route_hint(
              custody, physical.robots[robot],
              RouteHintSearchResult{
                  RouteStatus::ARRIVED,
                  {physical.robots[robot]}, 0});
        } else if (followed_preferred_leg) {
          const auto transfer = normalized_transfer(custody);
          const size_t next_index = custody.transfer_index + 1;
          if (next_index + 1 < transfer.route.size() &&
              transfer.route[next_index] ==
                  physical.robots[robot]) {
            custody.transfer = transfer;
            custody.transfer_index = next_index;
            custody.from = transfer.route[next_index];
            custody.to = transfer.route[next_index + 1];
            reanchor_anonymous_custody(custody);
            custody.task_id =
                TaskId{custody.shelf, custody.from, custody.to};
            custody.preferred_leg = custody.task_id;
            custody.route_status = RouteStatus::OK;
          } else {
            const auto hint = reroute_to_endpoint(
                ins, physical, (int)robot, endpoint, route_budget);
            install_route_hint(
                custody, physical.robots[robot], hint);
            if (hint.status != RouteStatus::OK &&
                hint.status != RouteStatus::ARRIVED)
              custody.rebind_reason = RebindReason::NO_ROUTE;
          }
        } else {
          const auto hint = reroute_to_endpoint(
              ins, physical, (int)robot, endpoint, route_budget,
              previous_physical->robots[robot]);
          install_route_hint(
              custody, physical.robots[robot], hint);
          custody.rebind_reason =
              hint.status == RouteStatus::OK ||
                      hint.status == RouteStatus::PREFIX ||
                      hint.status == RouteStatus::ARRIVED
                  ? RebindReason::FORCED_DEVIATION
                  : RebindReason::NO_ROUTE;
        }
        const int current_index =
            compatible_task_index_by_custody(
                current_graph, custody);
        custody.current_task_index =
            current_index >= 0
                ? std::optional<int>(current_index)
                : std::nullopt;
        if (current_index >= 0) {
          custody.roots =
              current_graph.tasks[current_index].roots;
          custody.priority =
              current_graph.tasks[current_index].priority;
        }
        if (custody_physically_valid(
                ins, physical, (int)robot, custody)) {
          out.custody_by_robot[robot] = std::move(custody);
          out.continuation_carrier[robot] =
              custody_arrived(
                  physical, (int)robot,
                  *out.custody_by_robot[robot]);
        }
      }
      continue;
    }

    if (previous_kappa != KAPPA_FREE && op.kind == Op::WAIT &&
        previous_guidance != nullptr &&
        robot < previous_guidance->custody_by_robot.size() &&
        previous_guidance->custody_by_robot[robot].has_value()) {
      Custody custody =
          *previous_guidance->custody_by_robot[robot];
      ensure_transfer_identity(
          custody, (int)robot,
          phys_config_hash(*previous_physical));
      custody.from = physical.robots[robot];
      reanchor_anonymous_custody(custody);
      if (!custody_physically_valid(ins, physical, (int)robot, custody))
        continue;
      if (force_route_refresh ||
          !route_hint_usable(
              ins, physical, (int)robot, custody)) {
        const auto hint = reroute_to_endpoint(
            ins, physical, (int)robot, custody_endpoint(custody),
            route_budget);
        install_route_hint(custody, physical.robots[robot], hint);
        if (hint.status != RouteStatus::OK &&
            hint.status != RouteStatus::ARRIVED)
          custody.rebind_reason = RebindReason::NO_ROUTE;
      }
      const int current_index =
          compatible_task_index_by_custody(
              current_graph, custody);
      custody.current_task_index =
          current_index >= 0 ? std::optional<int>(current_index)
                             : std::nullopt;
      if (current_index >= 0) {
        const auto& current_task = current_graph.tasks[current_index];
        custody.roots = current_task.roots;
        custody.priority = current_task.priority;
      }
      out.custody_by_robot[robot] = std::move(custody);
      out.continuation_carrier[robot] =
          custody_arrived(
              physical, (int)robot,
              *out.custody_by_robot[robot]);
      continue;
    }

    if (previous_kappa == KAPPA_FREE && op.kind == Op::LIFT &&
        previous_guidance != nullptr &&
        robot < previous_guidance->rho_task_id.size() &&
        previous_guidance->rho_task_id[robot].has_value()) {
      const TaskId id = *previous_guidance->rho_task_id[robot];
      const TransferKey* key =
          robot < previous_guidance->rho_transfer_key.size() &&
                  previous_guidance->rho_transfer_key[robot].has_value()
              ? &*previous_guidance->rho_transfer_key[robot]
              : nullptr;
      const int hinted_index =
          robot < previous_guidance->rho_ready_index.size()
              ? previous_guidance->rho_ready_index[robot]
              : -1;
      int previous_index = -1;
      if (!exact_ready_binding(*previous_guidance, id, key,
                               hinted_index,
                               &previous_index) ||
          !task_matches_loaded_shelf(physical, (int)robot, id))
        continue;
      const auto& previous_task =
          previous_guidance->upper_epoch->task_graph.tasks[previous_index];
      Custody custody = make_custody(previous_task, -1);
      ensure_transfer_identity(
          custody, (int)robot,
          phys_config_hash(*previous_physical));
      const int current_index =
          compatible_task_index_by_custody(
              current_graph, custody);
      custody.current_task_index =
          current_index >= 0 ? std::optional<int>(current_index)
                             : std::nullopt;
      if (current_index >= 0) {
        custody.roots = current_graph.tasks[current_index].roots;
        custody.priority = current_graph.tasks[current_index].priority;
      }
      if (custody_physically_valid(ins, physical, (int)robot, custody))
        out.custody_by_robot[robot] = std::move(custody);
    }
  }

  auto claims = active_transfer_claims(
      ins.grid.size(), out.custody_by_robot);
  struct RecoveryCandidate {
    size_t robot = 0;
    int priority = 0;
    std::optional<Custody> previous_custody;
  };
  std::vector<RecoveryCandidate> recovery_candidates;
  for (size_t robot = 0; robot < robot_count; ++robot) {
    if (physical.kappa[robot] == KAPPA_FREE ||
        out.custody_by_robot[robot].has_value() ||
        ins.can_place_movable_shelf(
            physical.robots[robot]))
      continue;
    const std::optional<Custody> previous_custody =
        previous_guidance != nullptr &&
                robot < previous_guidance->custody_by_robot.size()
            ? previous_guidance->custody_by_robot[robot]
            : std::nullopt;
    recovery_candidates.push_back(
        RecoveryCandidate{
            robot,
            previous_custody.has_value()
                ? previous_custody->priority
                : 0,
            previous_custody});
  }
  std::stable_sort(
      recovery_candidates.begin(), recovery_candidates.end(),
      [](const RecoveryCandidate& a,
         const RecoveryCandidate& b) {
        return a.priority != b.priority
                   ? a.priority > b.priority
                   : a.robot < b.robot;
      });
  for (const auto& candidate : recovery_candidates) {
    const auto custody =
        make_storage_recovery_custody(
            ins, physical, (int)candidate.robot,
            current_graph, candidate.previous_custody,
            claims, route_budget,
            candidate.robot <
                    out.previous_loaded_move_from.size()
                ? out.previous_loaded_move_from[candidate.robot]
                : -1);
    if (!custody.has_value()) continue;
    out.custody_by_robot[candidate.robot] = custody;
    add_transfer_claim(
        claims, remaining_transfer(*custody));
    out.continuation_carrier[candidate.robot] = 0;
  }
  return out;
}

inline void bind_ready_continuations(
    const DDInstance& ins, const PhysConfig& physical,
    const ShelfTaskGraph& graph, const std::vector<int>& ready_tasks,
    const std::vector<uint8_t>& continuation_carrier,
    const std::vector<int>& previous_loaded_move_from,
    std::vector<std::optional<Custody>>& custody_by_robot)
{
  const bool suppress_immediate_reverse =
      !target_dense_upper_layout(
          ins, make_upper_signature(physical));
  for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
    if (robot >= continuation_carrier.size() ||
        !continuation_carrier[robot] ||
        physical.kappa[robot] == KAPPA_FREE ||
        (custody_by_robot[robot].has_value() &&
         !custody_arrived(
             physical, (int)robot,
             *custody_by_robot[robot])))
      continue;
    for (const int index : ready_tasks) {
      if (index < 0 || index >= (int)graph.tasks.size()) continue;
      const auto& task = graph.tasks[index];
      if (!task_matches_loaded_shelf(physical, (int)robot, task.id))
        continue;
      if (suppress_immediate_reverse &&
          robot < previous_loaded_move_from.size() &&
          previous_loaded_move_from[robot] >= 0 &&
          task.id.to == previous_loaded_move_from[robot])
        continue;
      Custody custody = make_custody(task, index);
      ensure_transfer_identity(
          custody, (int)robot, phys_config_hash(physical));
      custody_by_robot[robot] = std::move(custody);
      break;
    }
  }
}

}  // namespace carrier_detail
