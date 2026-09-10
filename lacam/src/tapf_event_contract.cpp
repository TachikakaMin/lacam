// Carrier event contract validation (phase/contract/transition).
// Split from the original tapf_planner.cpp.

#include "tapf_planner_internal.hpp"

using namespace tapf_detail;
using namespace carrier_detail;

CarrierTaskPhase carrier_event_phase_of(
    const PhysConfig& state,
    const FixedRobotTransfer& fixed)
{
  if (fixed.robot < 0 ||
      fixed.robot >= (int)state.robots.size() ||
      state.kappa.size() != state.robots.size() ||
      fixed.transfer.route.size() < 2 ||
      (fixed.shelf.kind ==
               ShelfSelector::Kind::ANON_AT_EPOCH_CELL &&
       fixed.transfer.route.front() != fixed.shelf.value) ||
      fixed.transfer.route.back() !=
          fixed.transfer.endpoint)
    return {};

  const int source = fixed.transfer.route.front();
  const int endpoint = fixed.transfer.endpoint;
  const int robot = fixed.robot;
  const int robot_cell = state.robots[robot];
  const int kappa = state.kappa[robot];
  const auto route_it = std::find(
      fixed.transfer.route.begin(),
      fixed.transfer.route.end(), robot_cell);
  const int route_index =
      route_it == fixed.transfer.route.end()
          ? -1
          : static_cast<int>(
                std::distance(
                    fixed.transfer.route.begin(), route_it));
  if (route_it != fixed.transfer.route.end() &&
      std::find(
          std::next(route_it), fixed.transfer.route.end(),
          robot_cell) != fixed.transfer.route.end())
    return {};

  if (fixed.shelf.kind == ShelfSelector::Kind::TARGET) {
    const int target = fixed.shelf.value;
    if (fixed.stable_shelf.kind !=
            UpperShelfHandle::Kind::TARGET ||
        fixed.stable_shelf.stable_id != target ||
        target < 0 ||
        target >= (int)state.target_pos.size())
      return {};
    const bool carried_elsewhere = std::any_of(
        state.kappa.begin(), state.kappa.end(),
        [&](int value) { return value == target; });
    if (kappa == target &&
        state.target_pos[target] == robot_cell &&
        route_index >= 0)
      return {
          CarrierTaskPhaseKind::CARRYING, route_index};
    if (kappa != KAPPA_FREE) return {};
    if (!carried_elsewhere &&
        state.target_pos[target] == endpoint &&
        endpoint != source)
      return {CarrierTaskPhaseKind::COMPLETED, -1};
    if (!carried_elsewhere &&
        state.target_pos[target] == source)
      return {CarrierTaskPhaseKind::APPROACH, -1};
    return {};
  }

  if (fixed.shelf.kind !=
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL ||
      fixed.stable_shelf.kind !=
          UpperShelfHandle::Kind::ANONYMOUS ||
      fixed.shelf.value != source)
    return {};
  const bool source_grounded = std::binary_search(
      state.anon_occ.begin(), state.anon_occ.end(), source);
  const bool endpoint_grounded = std::binary_search(
      state.anon_occ.begin(), state.anon_occ.end(), endpoint);
  if (kappa == KAPPA_ANON && !source_grounded &&
      route_index >= 0)
    return {CarrierTaskPhaseKind::CARRYING, route_index};
  if (kappa != KAPPA_FREE) return {};
  if (endpoint_grounded && !source_grounded &&
      endpoint != source)
    return {CarrierTaskPhaseKind::COMPLETED, -1};
  if (source_grounded)
    return {CarrierTaskPhaseKind::APPROACH, -1};
  return {};
}

CarrierEventContractValidation validate_carrier_event_contract(
    const DDInstance& ins, const CarrierEventContract& contract)
{
  if (!validate_phys_config_root(ins, contract.start).valid())
    return {
        CarrierEventContractInvalidReason::
            INVALID_PHYSICAL_ROOT};

  std::map<FrozenTaskId, const TaskLedgerEntry*> ledger;
  std::optional<size_t> wave;
  for (const auto& entry : contract.wave_ledger_snapshot) {
    if (!wave.has_value()) wave = entry.task_id.wave;
    if (entry.task_id.wave != *wave ||
        !ledger.emplace(entry.task_id, &entry).second)
      return {
          CarrierEventContractInvalidReason::INVALID_LEDGER};
    if (entry.status == ExecutionStatus::CARRYING) {
      if (!entry.robot.has_value() ||
          *entry.robot < 0 ||
          *entry.robot >= (int)ins.n_robots())
        return {
            CarrierEventContractInvalidReason::INVALID_LEDGER};
    } else if (entry.robot.has_value()) {
      return {
          CarrierEventContractInvalidReason::INVALID_LEDGER};
    }
  }

  std::set<int> robots;
  std::set<FrozenTaskId> tasks;
  std::set<UpperShelfHandle> shelves;
  std::set<int> sources;
  std::set<int> endpoints;
  std::map<FrozenTaskId, const FixedRobotTransfer*> active;
  for (const auto& fixed : contract.active_transfers) {
    if (fixed.robot < 0 ||
        fixed.robot >= (int)ins.n_robots() ||
        fixed.transfer.route.size() < 2 ||
        fixed.transfer.route.front() < 0 ||
        fixed.transfer.route.front() >= ins.grid.size() ||
        fixed.transfer.route.back() !=
            fixed.transfer.endpoint ||
        !ins.can_place_movable_shelf(
            fixed.transfer.endpoint))
      return {
          CarrierEventContractInvalidReason::
              INVALID_ACTIVE_TRANSFER};
    std::set<int> route_cells;
    for (size_t index = 0;
         index < fixed.transfer.route.size(); ++index) {
      const int cell = fixed.transfer.route[index];
      if (cell < 0 || cell >= ins.grid.size() ||
          ins.grid.is_wall(cell) ||
          ins.is_fixed_upper_cell(cell) ||
          !route_cells.insert(cell).second)
        return {
            CarrierEventContractInvalidReason::
                INVALID_ACTIVE_TRANSFER};
      if (index > 0) {
        if (!ins.grid.has_edge(
                fixed.transfer.route[index - 1], cell))
          return {
              CarrierEventContractInvalidReason::
                  INVALID_ACTIVE_TRANSFER};
      }
    }
    const int source = fixed.transfer.route.front();
    if (!robots.insert(fixed.robot).second ||
        !tasks.insert(fixed.task_id).second ||
        !shelves.insert(fixed.stable_shelf).second ||
        !sources.insert(source).second ||
        !endpoints.insert(fixed.transfer.endpoint).second ||
        !active.emplace(fixed.task_id, &fixed).second)
      return {
          CarrierEventContractInvalidReason::
              INVALID_ACTIVE_TRANSFER};
    if (fixed.shelf.kind == ShelfSelector::Kind::TARGET) {
      if (fixed.stable_shelf.kind !=
              UpperShelfHandle::Kind::TARGET ||
          fixed.stable_shelf.stable_id !=
              fixed.shelf.value)
        return {
            CarrierEventContractInvalidReason::
                INVALID_ACTIVE_TRANSFER};
    } else if (
        fixed.stable_shelf.kind !=
            UpperShelfHandle::Kind::ANONYMOUS ||
        fixed.shelf.value != source) {
      return {
          CarrierEventContractInvalidReason::
              INVALID_ACTIVE_TRANSFER};
    }

    const auto ledger_it = ledger.find(fixed.task_id);
    if (ledger_it == ledger.end())
      return {
          CarrierEventContractInvalidReason::
              LEDGER_ACTIVE_MISMATCH};
    const auto& entry = *ledger_it->second;
    const auto phase =
        carrier_event_phase_of(contract.start, fixed);
    if (fixed.start_mode ==
        FixedTransferStartMode::PROVISIONAL_FREE) {
      if (entry.status != ExecutionStatus::PENDING ||
          entry.robot.has_value() ||
          phase.kind != CarrierTaskPhaseKind::APPROACH)
        return {
            CarrierEventContractInvalidReason::
                LEDGER_ACTIVE_MISMATCH};
    } else {
      if (entry.status != ExecutionStatus::CARRYING ||
          !entry.robot.has_value() ||
          *entry.robot != fixed.robot ||
          phase.kind != CarrierTaskPhaseKind::CARRYING)
        return {
            CarrierEventContractInvalidReason::
                LEDGER_ACTIVE_MISMATCH};
    }
  }
  for (const int endpoint : endpoints)
    if (sources.count(endpoint) != 0)
      return {
          CarrierEventContractInvalidReason::
              INVALID_ACTIVE_TRANSFER};

  for (const auto& [task_id, entry] : ledger) {
    const auto active_it = active.find(task_id);
    if (entry->status == ExecutionStatus::CARRYING) {
      if (active_it == active.end() ||
          active_it->second->start_mode !=
              FixedTransferStartMode::LOCKED_CARRYING ||
          !entry->robot.has_value() ||
          active_it->second->robot != *entry->robot)
        return {
            CarrierEventContractInvalidReason::
                LEDGER_ACTIVE_MISMATCH};
    } else if (
        entry->status == ExecutionStatus::COMPLETED &&
        active_it != active.end()) {
      return {
          CarrierEventContractInvalidReason::
              LEDGER_ACTIVE_MISMATCH};
    }
  }

  for (size_t robot = 0;
       robot < contract.start.kappa.size(); ++robot) {
    if (contract.start.kappa[robot] == KAPPA_FREE)
      continue;
    const auto fixed_it = std::find_if(
        contract.active_transfers.begin(),
        contract.active_transfers.end(),
        [&](const FixedRobotTransfer& fixed) {
          return fixed.robot == (int)robot &&
                 fixed.start_mode ==
                     FixedTransferStartMode::
                         LOCKED_CARRYING;
        });
    if (fixed_it == contract.active_transfers.end())
      return {
          CarrierEventContractInvalidReason::
              UNBOUND_CARRYING_ROBOT};
  }
  return {};
}

bool validate_carrier_event_transition(
    const DDInstance& ins, const CarrierEventContract& contract,
    const PhysConfig& from, const std::vector<Op>& ops,
    const PhysConfig& to)
{
  if (!validate_carrier_event_contract(ins, contract).valid() ||
      !validate_phys_config_root(ins, from).valid() ||
      !validate_phys_config_root(ins, to).valid())
    return false;
  const auto replayed = apply_ops(ins, from, ops);
  if (!replayed.has_value() || !(*replayed == to) ||
      ops.size() != ins.n_robots())
    return false;

  std::vector<const FixedRobotTransfer*> by_robot(
      ins.n_robots(), nullptr);
  for (const auto& fixed : contract.active_transfers)
    by_robot[fixed.robot] = &fixed;

  for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
    const auto* fixed = by_robot[robot];
    if (fixed == nullptr) {
      if (from.kappa[robot] != KAPPA_FREE ||
          ops[robot].kind == Op::LIFT ||
          ops[robot].kind == Op::DROP)
        return false;
      continue;
    }
    const auto before =
        carrier_event_phase_of(from, *fixed);
    const auto after =
        carrier_event_phase_of(to, *fixed);
    if (before.kind == CarrierTaskPhaseKind::INVALID ||
        after.kind == CarrierTaskPhaseKind::INVALID ||
        before.kind == CarrierTaskPhaseKind::COMPLETED)
      return false;

    if (before.kind == CarrierTaskPhaseKind::APPROACH) {
      if (ops[robot].kind == Op::LIFT) {
        if (after.kind != CarrierTaskPhaseKind::CARRYING ||
            after.route_index != 0)
          return false;
      } else if (
          ops[robot].kind == Op::WAIT ||
          ops[robot].kind == Op::MOVE) {
        if (after.kind != CarrierTaskPhaseKind::APPROACH)
          return false;
      } else {
        return false;
      }
      continue;
    }

    if (ops[robot].kind == Op::WAIT) {
      if (after.kind != CarrierTaskPhaseKind::CARRYING ||
          after.route_index != before.route_index)
        return false;
    } else if (ops[robot].kind == Op::MOVE) {
      if (after.kind != CarrierTaskPhaseKind::CARRYING ||
          after.route_index != before.route_index + 1)
        return false;
    } else if (ops[robot].kind == Op::DROP) {
      if (before.route_index !=
              (int)fixed->transfer.route.size() - 1 ||
          after.kind != CarrierTaskPhaseKind::COMPLETED)
        return false;
    } else {
      return false;
    }
  }
  return true;
}

// out-of-line: CarrierEngine is an implementation type
