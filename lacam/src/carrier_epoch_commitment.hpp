// Part of carrier_guidance.hpp (internal, src/): Root demand seeds, priority commitments, epoch graph compilation.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_rho_match.hpp"

namespace carrier_detail {
inline std::vector<RootDemand>
filter_priority_commitment_for_tau(
    const std::vector<RootDemand>& candidates,
    const std::vector<int>& tau)
{
  std::vector<RootDemand> out;
  for (const auto& root : candidates)
    if (root.target >= 0 &&
        root.target < (int)tau.size() &&
        tau[root.target] == root.goal &&
        std::find_if(
            out.begin(), out.end(),
            [&](const RootDemand& existing) {
              return existing.target == root.target;
            }) == out.end())
      out.push_back(root);
  return out;
}

inline bool in_flight_effect_requires_graph_seed(
    const DDInstance& ins, int source)
{
  return source >= 0 && source < ins.grid.size() &&
         !ins.grid.is_wall(source) &&
         !ins.can_store_shelf(source);
}

inline std::vector<TaskBRForcedEffect>
in_flight_forced_effects_for_transition(
    const DDInstance& ins, const PhysConfig& physical,
    const PhysConfig* previous_physical,
    const CarrierGuidance* previous_guidance,
    const std::vector<Op>* executed_ops)
{
  std::vector<TaskBRForcedEffect> out;
  if (previous_physical == nullptr ||
      previous_guidance == nullptr ||
      executed_ops == nullptr ||
      previous_physical->robots.size() != ins.n_robots() ||
      previous_physical->kappa.size() != ins.n_robots() ||
      physical.robots.size() != ins.n_robots() ||
      physical.kappa.size() != ins.n_robots() ||
      executed_ops->size() != ins.n_robots())
    return out;
  const auto replayed =
      apply_ops(ins, *previous_physical, *executed_ops);
  if (!replayed.has_value() || !(*replayed == physical))
    return out;

  for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
    if (previous_physical->kappa[robot] == KAPPA_FREE ||
        physical.kappa[robot] == KAPPA_FREE ||
        ((*executed_ops)[robot].kind != Op::MOVE &&
         (*executed_ops)[robot].kind != Op::WAIT) ||
        robot >= previous_guidance->custody_by_robot.size() ||
        !previous_guidance->custody_by_robot[robot].has_value())
      continue;
    const auto& custody =
        *previous_guidance->custody_by_robot[robot];
    const int endpoint = custody_endpoint(custody);
    if (endpoint < 0 ||
        physical.robots[robot] == endpoint)
      continue;
    ShelfSelector shelf = custody.shelf;
    if (shelf.kind ==
        ShelfSelector::Kind::ANON_AT_EPOCH_CELL)
      shelf.value = physical.robots[robot];
    const bool carried_shelf_matches =
        (shelf.kind == ShelfSelector::Kind::TARGET &&
         physical.kappa[robot] == shelf.value) ||
        (shelf.kind ==
             ShelfSelector::Kind::ANON_AT_EPOCH_CELL &&
         physical.kappa[robot] == KAPPA_ANON);
    if (!carried_shelf_matches) continue;
    if (!in_flight_effect_requires_graph_seed(
            ins, physical.robots[robot]))
      continue;

    const int discouraged_first_step =
        (*executed_ops)[robot].kind == Op::MOVE
            ? previous_physical->robots[robot]
            : -1;
    const auto route = reroute_to_endpoint(
        ins, physical, (int)robot, endpoint, 4096,
        discouraged_first_step);
    if (route.status != RouteStatus::OK ||
        route.route.size() < 2 ||
        route.route.front() != physical.robots[robot] ||
        route.route.back() != endpoint)
      continue;

    std::vector<RootDemand> roots = custody.roots;
    std::sort(roots.begin(), roots.end());
    roots.erase(
        std::unique(roots.begin(), roots.end()),
        roots.end());
    out.push_back(
        TaskBRForcedEffect{
            shelf,
            TaskBRForcedEffect::Kind::TRANSFER,
            StorageTransfer{endpoint, route.route},
            std::move(roots),
            custody.priority});
  }
  std::sort(out.begin(), out.end());
  return out;
}

struct PriorityCommitmentRoot {
  RootDemand demand;
  int priority = 0;
};

struct PriorityCommitmentGroup {
  std::vector<PriorityCommitmentRoot> roots;
  TaskId task_id;
  size_t robot = 0;
  RootTransferContinuity continuity;
};

struct PriorityCommitmentSelection {
  std::vector<RootDemand> roots;
  RootTransferContinuity continuity;
};

inline RootGoalCommitment
active_root_goal_commitments_for_epoch(
    const DDInstance& ins, const UpperSignature& upper,
    const UpperEpochGuidance* previous_epoch,
    const std::vector<TaskBRForcedEffect>& forced_effects,
    const std::vector<PriorityCommitmentGroup>& completed_groups)
{
  RootGoalCommitment out;
  std::map<int, int> goal_owner;
  const auto add = [&](int target, int goal) {
    if (target < 0 ||
        target >= static_cast<int>(ins.n_targets()) ||
        target >= static_cast<int>(upper.target_pos.size()) ||
        ins.target_goal_sets[target].size() <= 1 ||
        !eligible_goal(ins, target, goal) ||
        upper.target_pos[target] == goal)
      return;
    const auto existing = out.find(target);
    if (existing != out.end()) return;
    const auto owner = goal_owner.find(goal);
    if (owner != goal_owner.end() && owner->second != target)
      return;
    out.emplace(target, goal);
    goal_owner.emplace(goal, target);
  };
  if (previous_epoch == nullptr) return out;

  for (const auto& [target, goal] :
       previous_epoch->root_goal_commitment)
    add(target, goal);

  const auto add_previous_mission =
      [&](const RootDemand& root) {
        if (root.target < 0 ||
            root.target >=
                static_cast<int>(
                    previous_epoch->tau_guide.size()) ||
            previous_epoch->tau_guide[root.target] != root.goal)
          return;
        add(root.target, root.goal);
      };
  for (const auto& forced : forced_effects)
    for (const auto& root : forced.roots)
      add_previous_mission(root);
  for (const auto& group : completed_groups)
    for (const auto& root : group.roots)
      add_previous_mission(root.demand);
  return out;
}

inline RootGoalCommitment
carried_target_goal_commitments_for_epoch(
    const DDInstance& ins, const PhysConfig& physical,
    const UpperEpochGuidance* previous_epoch,
    RootGoalCommitment commitments)
{
  if (previous_epoch == nullptr) return commitments;
  std::map<int, int> goal_owner;
  for (const auto& [target, goal] : commitments)
    goal_owner.emplace(goal, target);
  for (const int kappa : physical.kappa) {
    if (kappa < 0 ||
        kappa >= static_cast<int>(ins.n_targets()) ||
        kappa >=
            static_cast<int>(
                previous_epoch->tau_guide.size()) ||
        ins.target_goal_sets[kappa].size() <= 1 ||
        commitments.find(kappa) != commitments.end())
      continue;
    const int goal = previous_epoch->tau_guide[kappa];
    if (!eligible_goal(ins, kappa, goal))
      continue;
    const auto owner = goal_owner.find(goal);
    if (owner != goal_owner.end() && owner->second != kappa)
      continue;
    commitments.emplace(kappa, goal);
    goal_owner.emplace(goal, kappa);
  }
  return commitments;
}

inline RootTransferContinuity active_transfer_continuity_for_epoch(
    const RootTransferContinuity* previous,
    const PriorityCommitmentSelection& selection,
    const std::vector<int>& tau,
    const UpperSignature& upper)
{
  RootTransferContinuity out;
  const auto active = [&](const RootDemand& root) {
    return root.target >= 0 &&
           root.target < static_cast<int>(tau.size()) &&
           root.target <
               static_cast<int>(upper.target_pos.size()) &&
           tau[root.target] == root.goal &&
           upper.target_pos[root.target] != root.goal;
  };
  const auto shelf_still_at_source =
      [&](const TransferKey& key) {
        if (key.shelf.kind ==
            ShelfSelector::Kind::TARGET)
          return key.shelf.value >= 0 &&
                 key.shelf.value <
                     static_cast<int>(
                         upper.target_pos.size()) &&
                 upper.target_pos[key.shelf.value] ==
                     key.source;
        return key.shelf.value == key.source &&
               std::binary_search(
                   upper.anon_pos.begin(),
                   upper.anon_pos.end(),
                   key.shelf.value);
      };
  if (previous != nullptr)
    for (const auto& [root, key] : *previous)
      if (active(root) && shelf_still_at_source(key))
        out.emplace(root, key);
  for (const auto& [root, key] : selection.continuity)
    if (active(root) && shelf_still_at_source(key))
      out[root] = key;
  return out;
}

inline PriorityCommitmentSelection
select_priority_commitment_context_for_epoch(
    const std::vector<PriorityCommitmentGroup>& completed_groups,
    const std::vector<int>& tau, size_t active_root_count,
    const std::vector<int>& previous_commitment,
    size_t vacancy_count =
        std::numeric_limits<size_t>::max())
{
  PriorityCommitmentGroup best_group;
  bool have_best = false;
  for (const auto& group : completed_groups) {
    std::map<int, PriorityCommitmentRoot> unique_roots;
    for (const auto& root : group.roots) {
      if (root.demand.target < 0 ||
          root.demand.target >= (int)tau.size() ||
          tau[root.demand.target] != root.demand.goal)
        continue;
      auto inserted = unique_roots.emplace(
          root.demand.target, root);
      if (!inserted.second &&
          root.priority > inserted.first->second.priority)
        inserted.first->second = root;
    }
    PriorityCommitmentGroup filtered;
    filtered.task_id = group.task_id;
    filtered.robot = group.robot;
    for (const auto& [unused_target, root] : unique_roots) {
      (void)unused_target;
      filtered.roots.push_back(root);
      const auto continuity =
          group.continuity.find(root.demand);
      if (continuity != group.continuity.end())
        filtered.continuity.emplace(
            continuity->first, continuity->second);
    }
    std::stable_sort(
        filtered.roots.begin(), filtered.roots.end(),
        [](const auto& a, const auto& b) {
          return a.priority != b.priority
                     ? a.priority > b.priority
                     : a.demand.target < b.demand.target;
        });
    if (filtered.roots.empty()) continue;
    const bool better =
        !have_best ||
        filtered.roots.front().priority >
            best_group.roots.front().priority ||
        (filtered.roots.front().priority ==
             best_group.roots.front().priority &&
         (filtered.task_id < best_group.task_id ||
          (!(best_group.task_id < filtered.task_id) &&
           filtered.robot < best_group.robot)));
    if (better) {
      best_group = std::move(filtered);
      have_best = true;
    }
  }
  if (!have_best) return {};

  bool intersects_previous_collective = false;
  if (previous_commitment.size() > 1 &&
      best_group.roots.size() > 1)
    for (const auto& root : best_group.roots)
      if (std::find(
              previous_commitment.begin(),
              previous_commitment.end(),
              root.demand.target) != previous_commitment.end()) {
        intersects_previous_collective = true;
        break;
      }
  const bool collective =
      best_group.roots.size() * 2 > active_root_count ||
      (active_root_count > vacancy_count &&
       active_root_count - vacancy_count >= vacancy_count) ||
      intersects_previous_collective;
  const size_t selected_count =
      collective ? best_group.roots.size() : 1;
  PriorityCommitmentSelection selected;
  for (size_t i = 0; i < selected_count; ++i) {
    const auto& root = best_group.roots[i].demand;
    selected.roots.push_back(root);
    const auto continuity =
        best_group.continuity.find(root);
    if (continuity != best_group.continuity.end())
      selected.continuity.emplace(
          continuity->first, continuity->second);
  }
  return selected;
}

inline std::vector<RootDemand>
select_priority_commitment_for_epoch(
    const std::vector<PriorityCommitmentGroup>& completed_groups,
    const std::vector<int>& tau, size_t active_root_count,
    const std::vector<int>& previous_commitment,
    size_t vacancy_count =
        std::numeric_limits<size_t>::max())
{
  return select_priority_commitment_context_for_epoch(
             completed_groups, tau, active_root_count,
             previous_commitment, vacancy_count)
      .roots;
}

inline void compile_task_br_upper_epoch_graph(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const std::vector<int>& priority_commitment,
    const RootTransferContinuity& transfer_continuity,
    const std::vector<TaskBRForcedEffect>& forced_effects,
    UpperEpochGuidance& epoch,
    VacancyGuidanceTelemetry* telemetry = nullptr)
{
  epoch.priority_commitment.clear();
  epoch.transfer_continuity = transfer_continuity;
  epoch.forced_effects = forced_effects;
  epoch.target_priority =
      target_priorities_from_pair_cost(
          epoch.pair_cost, epoch.tau_guide);
  for (const int target : priority_commitment)
    if (target >= 0 && target < (int)ins.n_targets() &&
        target < (int)upper.target_pos.size() &&
        target < (int)epoch.tau_guide.size() &&
        upper.target_pos[target] != epoch.tau_guide[target] &&
        std::find(
            epoch.priority_commitment.begin(),
            epoch.priority_commitment.end(),
            target) == epoch.priority_commitment.end())
      epoch.priority_commitment.push_back(target);
  int promoted_priority =
      epoch.target_priority.empty()
          ? (int)epoch.priority_commitment.size() - 1
          : *std::max_element(
                epoch.target_priority.begin(),
                epoch.target_priority.end()) +
                (int)epoch.priority_commitment.size();
  for (const int target : epoch.priority_commitment)
    epoch.target_priority[target] = promoted_priority--;

  std::vector<RootDemand> roots;
  for (size_t target = 0; target < ins.n_targets(); ++target)
    if (target < upper.target_pos.size() &&
        target < epoch.tau_guide.size() &&
        upper.target_pos[target] != epoch.tau_guide[target])
      roots.push_back(
          RootDemand{(int)target, epoch.tau_guide[target]});
  auto abstract = make_abstract_upper_state(ins, upper);
  TaskBRCompilerLimits compiler_limits{256, 512};
  // With at most two vacancies, useful alternatives can each require a full
  // vacancy-chain recursion window; the branch cap still bounds the epoch.
  // Roomier layouts instead cap accumulated recursion so many failed root
  // options cannot repeatedly refresh the local 256-call window.
  if (upper_vacancy_count(ins, upper) > 2)
    compiler_limits.total_recursion_cap =
        compiler_limits.recursion_cap +
        compiler_limits.backtrack_cap;
  epoch.task_graph = compile_task_br_pibt(
      ins, abstract, roots, epoch.tau_guide,
      epoch.target_priority, upper_wall,
      storage_topology,
      compiler_limits, false,
      epoch.forced_effects.empty()
          ? nullptr
          : &epoch.forced_effects,
      nullptr, nullptr,
      telemetry, nullptr,
      epoch.transfer_continuity.empty()
          ? nullptr
          : &epoch.transfer_continuity);
}

inline std::shared_ptr<const UpperEpochGuidance>
build_task_br_upper_epoch(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    double alpha, double gamma, double delta,
    const std::vector<PriorityCommitmentGroup>*
        priority_commitment_groups = nullptr,
    size_t active_root_count = 0,
    const std::vector<int>*
        previous_priority_commitment = nullptr,
    VacancyGuidanceTelemetry* telemetry = nullptr,
    const RootTransferContinuity*
        previous_transfer_continuity = nullptr,
    const std::vector<TaskBRForcedEffect>*
        forced_effects = nullptr,
    const RootGoalCommitment*
        root_goal_commitment = nullptr,
    const UpperEpochGuidance*
        previous_pair_source = nullptr,
    PairCostDependencyContext*
        shared_dependency_context = nullptr)
{
  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->upper_signature = upper;
  VacancyPotentialCache potential_cache(ins.grid.size());
  PairCostDependencyContext local_dependency_context;
  auto* dependency_context =
      shared_dependency_context != nullptr
          ? shared_dependency_context
          : &local_dependency_context;
  auto assignment = build_lazy_pair_cost_assignment(
      ins, upper, upper_wall, storage_topology,
      alpha, gamma, delta, nullptr, &potential_cache,
      root_goal_commitment, dependency_context,
      previous_pair_source != nullptr
          ? &previous_pair_source->upper_signature
          : nullptr,
      previous_pair_source != nullptr
          ? &previous_pair_source->pair_cost
          : nullptr,
      previous_pair_source != nullptr
          ? &previous_pair_source->
                pair_assignment_hungarian
          : nullptr,
      previous_pair_source != nullptr
          ? &previous_pair_source->
                root_goal_commitment
          : nullptr);
  if (telemetry != nullptr) {
    telemetry->potential_builds += potential_cache.builds;
    telemetry->potential_unreachable_cells +=
        potential_cache.unreachable_cells;
    telemetry->first_choice_fallbacks +=
        potential_cache.first_choice_fallbacks;
    telemetry->potential_time_ms += potential_cache.build_ms;
  }
  epoch->pair_cost = std::move(assignment.table);
  epoch->pair_assignment_hungarian =
      std::move(assignment.hungarian_state);
  epoch->pair_edges_evaluated = assignment.evaluated_edges;
  epoch->pair_edges_total = assignment.total_edges;
  epoch->pair_edges_reused = assignment.reused_edges;
  epoch->pair_hungarian_full_solves =
      assignment.hungarian_full_solves;
  epoch->pair_hungarian_row_repairs =
      assignment.hungarian_row_repairs;
  epoch->pair_hungarian_forced_repairs =
      assignment.hungarian_forced_repairs;
  epoch->pair_rollout_work_steps = assignment.rollout_work_steps;
  epoch->pair_rollout_truncations =
      assignment.rollout_truncations;
  epoch->pair_rollout_stalls = assignment.rollout_stalls;
  epoch->tau_guide = std::move(assignment.tau);
  if (root_goal_commitment != nullptr)
    epoch->root_goal_commitment =
        *root_goal_commitment;
  PriorityCommitmentSelection selection;
  if (priority_commitment_groups != nullptr)
    selection =
        select_priority_commitment_context_for_epoch(
            *priority_commitment_groups,
            epoch->tau_guide, active_root_count,
            previous_priority_commitment != nullptr
                ? *previous_priority_commitment
                : std::vector<int>{},
            upper_vacancy_count(ins, upper));
  std::vector<int> priority_commitment;
  for (const auto& root : selection.roots)
    priority_commitment.push_back(root.target);
  const auto transfer_continuity =
      active_transfer_continuity_for_epoch(
          previous_transfer_continuity, selection,
          epoch->tau_guide, upper);
  compile_task_br_upper_epoch_graph(
      ins, upper, upper_wall, storage_topology,
      priority_commitment, transfer_continuity,
      forced_effects != nullptr
          ? *forced_effects
          : std::vector<TaskBRForcedEffect>{},
      *epoch,
      telemetry);
  return epoch;
}

inline std::shared_ptr<const UpperEpochGuidance>
rebuild_task_br_upper_epoch(
    const DDInstance& ins, const UpperSignature& upper,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    const UpperEpochGuidance& pair_source,
    const std::vector<int>& priority_commitment,
    const RootTransferContinuity& transfer_continuity,
    const std::vector<TaskBRForcedEffect>& forced_effects,
    VacancyGuidanceTelemetry* telemetry = nullptr)
{
  auto epoch =
      std::make_shared<UpperEpochGuidance>(pair_source);
  compile_task_br_upper_epoch_graph(
      ins, upper, upper_wall, storage_topology,
      priority_commitment, transfer_continuity,
      forced_effects, *epoch,
      telemetry);
  return epoch;
}

inline std::shared_ptr<const UpperEpochGuidance>
build_task_br_upper_epoch_for_tau(
    const DDInstance& ins, const UpperSignature& upper,
    const std::vector<int>& fixed_tau, DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    double alpha, double gamma, double delta,
    VacancyGuidanceTelemetry* telemetry = nullptr)
{
  if (fixed_tau.size() != ins.n_targets())
    throw std::invalid_argument(
        "build_task_br_upper_epoch_for_tau: tau size mismatch");
  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->upper_signature = upper;
  epoch->pair_cost = build_pair_cost_table(
      ins, upper, upper_wall, storage_topology,
      alpha, gamma, delta);
  for (const auto& row : epoch->pair_cost) {
    epoch->pair_edges_total += row.size();
    epoch->pair_edges_evaluated += row.size();
    for (const auto& entry : row) {
      epoch->pair_rollout_work_steps += entry.plan.rollout_steps;
      epoch->pair_rollout_truncations += entry.plan.truncated;
      epoch->pair_rollout_stalls += entry.plan.stalled;
    }
  }
  epoch->tau_guide = fixed_tau;
  for (size_t target = 0; target < ins.n_targets(); ++target) {
    if (!eligible_goal(ins, (int)target, fixed_tau[target]))
      throw std::invalid_argument(
          "build_task_br_upper_epoch_for_tau: ineligible goal");
  }
  compile_task_br_upper_epoch_graph(
      ins, upper, upper_wall, storage_topology, {}, {}, {},
      *epoch, telemetry);
  return epoch;
}

}  // namespace carrier_detail
