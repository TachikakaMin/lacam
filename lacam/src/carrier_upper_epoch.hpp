// Part of carrier_guidance.hpp (internal, src/): Upper-epoch cache and task-BR guidance construction.
// Chained include; see carrier_guidance.hpp for the module overview.
#pragma once

#include "carrier_epoch_commitment.hpp"

namespace carrier_detail {
struct UpperEpochCache {
  static constexpr size_t DEFAULT_CAPACITY = 256;

  struct Key {
    UpperSignature upper;
    std::vector<int> priority_commitment;
    RootTransferContinuity transfer_continuity;
    std::vector<TaskBRForcedEffect> forced_effects;
    RootGoalCommitment root_goal_commitment;

    bool operator<(const Key& other) const
    {
      if (upper != other.upper) return upper < other.upper;
      if (priority_commitment != other.priority_commitment)
        return priority_commitment <
               other.priority_commitment;
      if (transfer_continuity !=
          other.transfer_continuity)
        return transfer_continuity <
               other.transfer_continuity;
      if (forced_effects != other.forced_effects)
        return forced_effects < other.forced_effects;
      return root_goal_commitment <
             other.root_goal_commitment;
    }
  };

  struct ForcedEffectNormalizationKey {
    UpperSignature upper;
    std::vector<TaskBRForcedEffect> raw_forced_effects;
    std::vector<int> target_priority;

    bool operator<(
        const ForcedEffectNormalizationKey& other) const
    {
      if (upper != other.upper) return upper < other.upper;
      if (raw_forced_effects != other.raw_forced_effects)
        return raw_forced_effects <
               other.raw_forced_effects;
      if (target_priority != other.target_priority)
        return target_priority <
               other.target_priority;
      return false;
    }
  };

  explicit UpperEpochCache(size_t max_entries = DEFAULT_CAPACITY)
      : capacity(max_entries)
  {
    if (capacity == 0)
      throw std::invalid_argument(
          "UpperEpochCache capacity must be positive");
  }

  std::shared_ptr<const UpperEpochGuidance> lookup(
      const UpperSignature& signature,
      const std::vector<int>* priority_commitment = nullptr,
      const RootTransferContinuity* transfer_continuity = nullptr,
      const std::vector<TaskBRForcedEffect>*
          forced_effects = nullptr,
      const RootGoalCommitment*
          root_goal_commitment = nullptr)
  {
    const Key requested{
        signature,
        priority_commitment != nullptr
            ? *priority_commitment
            : std::vector<int>{},
        transfer_continuity != nullptr
            ? *transfer_continuity
            : RootTransferContinuity{},
        forced_effects != nullptr
            ? *forced_effects
            : std::vector<TaskBRForcedEffect>{},
        root_goal_commitment != nullptr
            ? *root_goal_commitment
            : RootGoalCommitment{}};
    const auto it = entries.find(requested);
    if (it == entries.end()) {
      ++misses;
      return nullptr;
    }
    ++hits;
    it->second.last_used = ++clock;
    return it->second.epoch;
  }

  bool lookup_forced_effect_normalization(
      const UpperSignature& upper,
      const std::vector<TaskBRForcedEffect>& raw_forced_effects,
      const std::vector<int>& target_priority,
      std::vector<TaskBRForcedEffect>& out)
  {
    const auto it = forced_effect_normalizations.find(
        ForcedEffectNormalizationKey{
            upper, raw_forced_effects, target_priority});
    if (it == forced_effect_normalizations.end())
      return false;
    it->second.last_used = ++normalization_clock;
    out = it->second.effects;
    return true;
  }

  void insert_forced_effect_normalization(
      const UpperSignature& upper,
      const std::vector<TaskBRForcedEffect>& raw_forced_effects,
      const std::vector<int>& target_priority,
      const std::vector<TaskBRForcedEffect>& effects)
  {
    const ForcedEffectNormalizationKey key{
        upper, raw_forced_effects, target_priority};
    const auto existing =
        forced_effect_normalizations.find(key);
    if (existing != forced_effect_normalizations.end()) {
      existing->second =
          ForcedEffectNormalizationEntry{
              effects, ++normalization_clock};
      return;
    }
    if (forced_effect_normalizations.size() == capacity) {
      const auto victim = std::min_element(
          forced_effect_normalizations.begin(),
          forced_effect_normalizations.end(),
          [](const auto& a, const auto& b) {
            return a.second.last_used <
                   b.second.last_used;
          });
      forced_effect_normalizations.erase(victim);
    }
    forced_effect_normalizations.emplace(
        key,
        ForcedEffectNormalizationEntry{
            effects, ++normalization_clock});
  }

  std::shared_ptr<const UpperEpochGuidance> peek_any(
      const UpperSignature& signature,
      const RootGoalCommitment*
          root_goal_commitment = nullptr)
  {
    const auto it = std::find_if(
        entries.begin(), entries.end(),
        [&](const auto& entry) {
          return entry.first.upper == signature &&
                 entry.first.root_goal_commitment ==
                     (root_goal_commitment != nullptr
                          ? *root_goal_commitment
                          : RootGoalCommitment{});
        });
    if (it == entries.end()) return nullptr;
    it->second.last_used = ++clock;
    return it->second.epoch;
  }

  std::shared_ptr<const UpperEpochGuidance> peek_most_recent()
  {
    if (entries.empty()) return nullptr;
    const auto it = std::max_element(
        entries.begin(), entries.end(),
        [](const auto& a, const auto& b) {
          return a.second.last_used <
                 b.second.last_used;
        });
    it->second.last_used = ++clock;
    return it->second.epoch;
  }

  void insert(
      const UpperSignature& signature,
      std::shared_ptr<const UpperEpochGuidance> epoch)
  {
    if (epoch == nullptr)
      throw std::invalid_argument(
          "UpperEpochCache cannot store a null epoch");
    const Key key{
        signature, epoch->priority_commitment,
        epoch->transfer_continuity,
        epoch->forced_effects,
        epoch->root_goal_commitment};
    const auto existing = entries.find(key);
    if (existing != entries.end()) {
      existing->second = Entry{std::move(epoch), ++clock};
      return;
    }
    if (entries.size() == capacity) {
      const auto victim = std::min_element(
          entries.begin(), entries.end(),
          [](const auto& a, const auto& b) {
            return a.second.last_used < b.second.last_used;
          });
      entries.erase(victim);
      ++evictions;
    }
    entries.emplace(
        key, Entry{std::move(epoch), ++clock});
  }

  size_t size() const { return entries.size(); }

  bool contains(const UpperSignature& signature) const
  {
    return std::find_if(
               entries.begin(), entries.end(),
               [&](const auto& entry) {
                 return entry.first.upper == signature;
               }) != entries.end();
  }

  long hits = 0;
  long misses = 0;
  long evictions = 0;
  PairCostDependencyContext pair_dependency_context;

 private:
  struct Entry {
    std::shared_ptr<const UpperEpochGuidance> epoch;
    uint64_t last_used = 0;
  };

  struct ForcedEffectNormalizationEntry {
    std::vector<TaskBRForcedEffect> effects;
    uint64_t last_used = 0;
  };

  size_t capacity;
  uint64_t clock = 0;
  uint64_t normalization_clock = 0;
  std::map<Key, Entry> entries;
  std::map<
      ForcedEffectNormalizationKey,
      ForcedEffectNormalizationEntry>
      forced_effect_normalizations;
};

inline CarrierGuidance build_task_br_guidance_from_upper_epoch(
    const DDInstance& ins, const PhysConfig& physical,
    std::shared_ptr<const UpperEpochGuidance> upper_epoch,
    const PhysConfig* previous_physical = nullptr,
    const CarrierGuidance* previous_guidance = nullptr,
    const std::vector<Op>* executed_ops = nullptr)
{
  if (upper_epoch == nullptr ||
      upper_epoch->upper_signature != make_upper_signature(physical))
    throw std::invalid_argument(
        "build_task_br_guidance_from_upper_epoch: epoch mismatch");
  CarrierGuidance out;
  out.upper_epoch = std::move(upper_epoch);
  auto recovered = recover_task_br_custody(
      ins, physical, out.upper_epoch->task_graph, previous_physical,
      previous_guidance, executed_ops);
  out.custody_by_robot = std::move(recovered.custody_by_robot);
  const auto preliminary_execution_view =
      reconcile_execution_view(
          ins, physical, out.upper_epoch->task_graph,
          out.custody_by_robot);
  long ready_before_claims = 0;
  long ready_claims_filtered = 0;
  out.ready_tasks = ready_tasks_with_custody(
      ins, physical, out.upper_epoch->task_graph,
      out.custody_by_robot, recovered.continuation_carrier,
      &preliminary_execution_view, &ready_before_claims,
      &ready_claims_filtered);
  bind_ready_continuations(
      ins, physical, out.upper_epoch->task_graph, out.ready_tasks,
      recovered.continuation_carrier,
      recovered.previous_loaded_move_from,
      out.custody_by_robot);
  out.execution_view = reconcile_execution_view(
      ins, physical, out.upper_epoch->task_graph,
      out.custody_by_robot);

  std::vector<int> grounded_ready;
  for (const int index : out.ready_tasks)
    if (index >= 0 &&
        index < (int)out.upper_epoch->task_graph.tasks.size() &&
        task_shelf_is_grounded(
            ins, physical,
            out.upper_epoch->task_graph.tasks[index].id))
      grounded_ready.push_back(index);
  const auto* previous_rho =
      recovered.transition_valid && previous_guidance != nullptr
          ? &previous_guidance->rho_task_id
          : nullptr;
  const auto* previous_rho_key =
      recovered.transition_valid && previous_guidance != nullptr
          ? &previous_guidance->rho_transfer_key
          : nullptr;
  auto rho = match_ready_tasks(
      ins, physical, out.upper_epoch->task_graph,
      grounded_ready, previous_rho, previous_rho_key);
  rho.telemetry.candidates_input = ready_before_claims;
  rho.telemetry.candidates_after_claims =
      static_cast<long>(grounded_ready.size());
  rho.telemetry.upstream_claim_filtered =
      ready_claims_filtered;
  out.rho_execute_telemetry = rho.telemetry;
  out.rho_task_id = std::move(rho.rho_task_id);
  out.rho_transfer_key = std::move(rho.rho_transfer_key);
  out.rho_ready_index = std::move(rho.rho_ready_index);
  out.rho_mode.assign(ins.n_robots(), DispatchMode::NONE);
  std::vector<uint8_t> eligible_for_preparation(
      ins.n_robots(), 1);
  for (size_t robot = 0; robot < ins.n_robots(); ++robot)
    if (robot < out.rho_task_id.size() &&
        out.rho_task_id[robot].has_value()) {
      out.rho_mode[robot] = DispatchMode::EXECUTE;
      eligible_for_preparation[robot] = 0;
    }
  out.preparable_tasks =
      preparable_tasks_with_executors(
          ins, physical, out.upper_epoch->task_graph,
          out.execution_view, out.custody_by_robot, grounded_ready,
          out.rho_ready_index);
  auto preparation = match_ready_tasks(
      ins, physical, out.upper_epoch->task_graph,
      out.preparable_tasks, previous_rho, previous_rho_key,
      &eligible_for_preparation, DispatchMode::PREPARE);
  out.rho_prepare_telemetry = preparation.telemetry;
  for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
    if (robot >= preparation.rho_task_id.size() ||
        !preparation.rho_task_id[robot].has_value())
      continue;
    out.rho_task_id[robot] =
        preparation.rho_task_id[robot];
    out.rho_transfer_key[robot] =
        preparation.rho_transfer_key[robot];
    out.rho_ready_index[robot] =
        preparation.rho_ready_index[robot];
    out.rho_mode[robot] = DispatchMode::PREPARE;
  }
  out.rho_mode_or_conflict_fingerprint =
      rho_fingerprint_mix(0x52484f4dULL);
  rho_fingerprint_add(
      out.rho_mode_or_conflict_fingerprint,
      out.rho_execute_telemetry.mode_or_conflict_fingerprint);
  rho_fingerprint_add(
      out.rho_mode_or_conflict_fingerprint,
      out.rho_prepare_telemetry.mode_or_conflict_fingerprint);
  out.rho_row_fingerprints.resize(ins.n_robots());
  for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
    uint64_t fingerprint = rho_fingerprint_mix(robot + 1);
    if (robot <
        out.rho_execute_telemetry.robot_row_fingerprints.size())
      rho_fingerprint_add(
          fingerprint,
          out.rho_execute_telemetry.robot_row_fingerprints[robot]);
    if (robot <
        out.rho_prepare_telemetry.robot_row_fingerprints.size())
      rho_fingerprint_add(
          fingerprint,
          out.rho_prepare_telemetry.robot_row_fingerprints[robot]);
    rho_fingerprint_add(
        fingerprint, static_cast<uint64_t>(out.rho_mode[robot]));
    out.rho_row_fingerprints[robot] = fingerprint;
    rho_fingerprint_add(
        out.rho_mode_or_conflict_fingerprint,
        static_cast<uint64_t>(out.rho_mode[robot]));
  }
  const auto timed_started =
      std::chrono::steady_clock::now();
  const JointTransportContext timed_context{
      &out.upper_epoch->task_graph,
      &out.execution_view,
      &out.rho_ready_index,
      &out.rho_mode,
      &out.upper_epoch->tau_guide};
  out.timed_transport =
      build_bounded_joint_transport_guidance(
          ins, physical, out.custody_by_robot,
          16, 256, 8, &timed_context);
  out.timed_transport.build_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - timed_started)
          .count();
  return out;
}

inline CarrierGuidance build_task_br_guidance(
    const DDInstance& ins, const PhysConfig& physical,
    DDDistCache& upper_wall,
    const StorageTransferTopology& storage_topology,
    double alpha, double gamma, double delta,
    const PhysConfig* previous_physical = nullptr,
    const CarrierGuidance* previous_guidance = nullptr,
    const std::vector<Op>* executed_ops = nullptr,
    UpperEpochCache* upper_epoch_cache = nullptr,
    VacancyGuidanceTelemetry* telemetry = nullptr)
{
  const auto upper = make_upper_signature(physical);
  std::shared_ptr<const UpperEpochGuidance> upper_epoch;
  const long cache_evictions_before =
      upper_epoch_cache != nullptr
          ? upper_epoch_cache->evictions
          : 0;

  bool transition_valid = false;
  if (previous_physical != nullptr && executed_ops != nullptr) {
    const auto replayed =
        apply_ops(ins, *previous_physical, *executed_ops);
    transition_valid = replayed.has_value() && *replayed == physical;
  }
  const auto raw_forced_effects =
      transition_valid
          ? in_flight_forced_effects_for_transition(
                ins, physical, previous_physical,
                previous_guidance, executed_ops)
          : std::vector<TaskBRForcedEffect>{};
  const UpperEpochGuidance* previous_epoch_for_forced =
      transition_valid &&
              previous_guidance != nullptr &&
              previous_guidance->upper_epoch != nullptr
          ? previous_guidance->upper_epoch.get()
          : nullptr;
  const bool fixed_goal_assignment = std::all_of(
      ins.target_goal_sets.begin(),
      ins.target_goal_sets.end(),
      [](const auto& goals) { return goals.size() == 1; });
  std::vector<TaskBRForcedEffect> forced_effects;
  if (upper_epoch_cache != nullptr &&
      previous_epoch_for_forced != nullptr &&
      !raw_forced_effects.empty() &&
      fixed_goal_assignment) {
    if (!upper_epoch_cache->
            lookup_forced_effect_normalization(
                upper, raw_forced_effects,
                previous_epoch_for_forced->target_priority,
                forced_effects)) {
      forced_effects =
          canonical_fixed_goal_forced_effects(
              ins, upper, previous_epoch_for_forced,
              storage_topology, raw_forced_effects);
      upper_epoch_cache->
          insert_forced_effect_normalization(
              upper, raw_forced_effects,
              previous_epoch_for_forced->target_priority,
              forced_effects);
    }
  } else {
    forced_effects =
        canonical_fixed_goal_forced_effects(
            ins, upper, previous_epoch_for_forced,
            storage_topology, raw_forced_effects);
  }
  const auto* forced_effects_key =
      forced_effects.empty() ? nullptr : &forced_effects;
  std::vector<PriorityCommitmentGroup>
      priority_commitment_groups;
  size_t priority_commitment_active_roots = 0;
  std::vector<int> previous_priority_commitment;
  RootTransferContinuity previous_transfer_continuity;
  const UpperEpochGuidance* previous_epoch_for_commitment =
      nullptr;
  if (transition_valid && previous_physical != nullptr &&
      previous_guidance != nullptr &&
      previous_guidance->upper_epoch != nullptr &&
      executed_ops != nullptr) {
    const auto& previous_epoch =
        *previous_guidance->upper_epoch;
    previous_epoch_for_commitment = &previous_epoch;
    previous_priority_commitment =
        previous_epoch.priority_commitment;
    if (!fixed_goal_assignment)
      previous_transfer_continuity =
          next_epoch_transfer_continuity(
              previous_epoch);
    size_t active_root_count = 0;
    for (size_t target = 0;
         target < previous_epoch.upper_signature.target_pos.size() &&
         target < previous_epoch.tau_guide.size();
         ++target)
      active_root_count +=
          previous_epoch.upper_signature.target_pos[target] !=
          previous_epoch.tau_guide[target];
    const bool dense_upper_layout =
        target_dense_upper_layout(
            ins, previous_epoch.upper_signature);
    for (size_t robot = 0; robot < ins.n_robots(); ++robot) {
      if (robot >= previous_physical->kappa.size() ||
          robot >= previous_physical->robots.size() ||
          robot >= physical.robots.size() ||
          robot >= executed_ops->size() ||
          robot >= previous_guidance->custody_by_robot.size() ||
          previous_physical->kappa[robot] == KAPPA_FREE ||
          (*executed_ops)[robot].kind != Op::MOVE ||
          !previous_guidance->custody_by_robot[robot].has_value())
        continue;
      const auto& custody =
          *previous_guidance->custody_by_robot[robot];
      if (previous_physical->robots[robot] != custody.from ||
          (*executed_ops)[robot].to != custody.to ||
          physical.robots[robot] != custody.to)
        continue;
      const auto transfer = normalized_transfer(custody);
      if (custody.transfer_index + 2 < transfer.route.size())
        continue;
      std::map<int, PriorityCommitmentRoot> group_roots;
      for (const auto& root : custody.roots) {
        if (root.target < 0 ||
            root.target >= (int)ins.n_targets() ||
            root.target >= (int)physical.target_pos.size() ||
            root.target >=
                (int)previous_epoch.tau_guide.size() ||
            previous_epoch.tau_guide[root.target] != root.goal ||
            physical.target_pos[root.target] == root.goal)
          continue;
        const bool flexible_goal =
            ins.target_goal_sets[root.target].size() > 1;
        const bool self_root =
            custody.shelf.kind == ShelfSelector::Kind::TARGET &&
            custody.shelf.value == root.target;
        if ((flexible_goal && !dense_upper_layout) ||
            (self_root && !flexible_goal))
          continue;
        const int previous_priority =
            root.target <
                    (int)previous_epoch.target_priority.size()
                ? previous_epoch.target_priority[root.target]
                : 0;
        auto inserted = group_roots.emplace(
            root.target,
            PriorityCommitmentRoot{root, previous_priority});
        if (!inserted.second)
          inserted.first->second.priority =
              std::max(
                  inserted.first->second.priority,
                  previous_priority);
      }
      std::vector<PriorityCommitmentRoot> ordered_group;
      for (const auto& [unused_target, root] : group_roots) {
        (void)unused_target;
        ordered_group.push_back(root);
      }
      std::stable_sort(
          ordered_group.begin(), ordered_group.end(),
          [](const auto& a, const auto& b) {
            return a.priority != b.priority
                       ? a.priority > b.priority
                       : a.demand.target < b.demand.target;
          });
      if (ordered_group.empty()) continue;
      std::vector<RootDemand> continuity_roots;
      continuity_roots.reserve(ordered_group.size());
      for (const auto& root : ordered_group)
        continuity_roots.push_back(root.demand);
      RootTransferContinuity continuity;
      if (!fixed_goal_assignment)
        continuity =
            successor_transfer_continuity(
                previous_epoch.task_graph,
                custody.task_id, continuity_roots);
      priority_commitment_groups.push_back(
          PriorityCommitmentGroup{
              std::move(ordered_group),
              custody.task_id,
              robot,
              std::move(continuity)});
    }
    if (!priority_commitment_groups.empty())
      priority_commitment_active_roots = active_root_count;
  }
  const auto root_goal_commitment =
      carried_target_goal_commitments_for_epoch(
          ins, physical, previous_epoch_for_commitment,
          active_root_goal_commitments_for_epoch(
              ins, upper, previous_epoch_for_commitment,
              forced_effects, priority_commitment_groups));
  const auto* root_goal_commitment_key =
      root_goal_commitment.empty()
          ? nullptr
          : &root_goal_commitment;
  if (transition_valid && previous_guidance != nullptr &&
      previous_guidance->upper_epoch != nullptr &&
      previous_guidance->upper_epoch->upper_signature == upper) {
    upper_epoch = previous_guidance->upper_epoch;
  } else if (upper_epoch_cache != nullptr) {
    bool insert_epoch = false;
    const auto pair_source =
        upper_epoch_cache->peek_any(
            upper, root_goal_commitment_key);
    if (pair_source != nullptr) {
      const auto selection =
          select_priority_commitment_context_for_epoch(
              priority_commitment_groups,
              pair_source->tau_guide,
              priority_commitment_active_roots,
              previous_priority_commitment,
              upper_vacancy_count(ins, upper));
      std::vector<int> priority_commitment;
      for (const auto& root : selection.roots)
        priority_commitment.push_back(root.target);
      const auto transfer_continuity =
          active_transfer_continuity_for_epoch(
              previous_transfer_continuity.empty()
                  ? nullptr
                  : &previous_transfer_continuity,
              selection, pair_source->tau_guide, upper);
      const auto* key =
          priority_commitment.empty()
              ? nullptr
              : &priority_commitment;
      const auto* continuity_key =
          transfer_continuity.empty()
              ? nullptr
              : &transfer_continuity;
      upper_epoch = upper_epoch_cache->lookup(
          upper, key, continuity_key, forced_effects_key,
          root_goal_commitment_key);
      if (upper_epoch == nullptr) {
        upper_epoch = rebuild_task_br_upper_epoch(
            ins, upper, upper_wall, storage_topology, *pair_source,
            priority_commitment, transfer_continuity,
            forced_effects,
            telemetry);
        insert_epoch = true;
      }
    } else {
      const auto nearby_pair_source =
          upper_epoch_cache->peek_most_recent();
      std::vector<int> requested_targets;
      for (const auto& group : priority_commitment_groups)
        for (const auto& root : group.roots)
          if (std::find(
                  requested_targets.begin(),
                  requested_targets.end(),
                  root.demand.target) ==
              requested_targets.end())
            requested_targets.push_back(root.demand.target);
      const auto* requested_key =
          requested_targets.empty()
              ? nullptr
              : &requested_targets;
      upper_epoch =
          upper_epoch_cache->lookup(
              upper, requested_key, nullptr,
              forced_effects_key,
              root_goal_commitment_key);
      if (upper_epoch != nullptr)
        throw std::logic_error(
            "UpperEpochCache peek/lookup disagreement");
      upper_epoch = build_task_br_upper_epoch(
          ins, upper, upper_wall, storage_topology,
          alpha, gamma, delta,
          priority_commitment_groups.empty()
              ? nullptr
              : &priority_commitment_groups,
          priority_commitment_active_roots,
          previous_priority_commitment.empty()
              ? nullptr
              : &previous_priority_commitment,
          telemetry,
          previous_transfer_continuity.empty()
              ? nullptr
              : &previous_transfer_continuity,
          forced_effects_key,
          root_goal_commitment_key,
          previous_epoch_for_commitment != nullptr
              ? previous_epoch_for_commitment
              : nearby_pair_source.get(),
          &upper_epoch_cache->pair_dependency_context);
      insert_epoch = true;
    }
    if (insert_epoch)
      upper_epoch_cache->insert(upper, upper_epoch);
  } else {
    upper_epoch = build_task_br_upper_epoch(
        ins, upper, upper_wall, storage_topology,
        alpha, gamma, delta,
        priority_commitment_groups.empty()
            ? nullptr
            : &priority_commitment_groups,
        priority_commitment_active_roots,
        previous_priority_commitment.empty()
            ? nullptr
            : &previous_priority_commitment,
        telemetry,
        previous_transfer_continuity.empty()
            ? nullptr
            : &previous_transfer_continuity,
        forced_effects_key,
        root_goal_commitment_key,
        previous_epoch_for_commitment,
        nullptr);
  }
  auto guidance = build_task_br_guidance_from_upper_epoch(
      ins, physical, std::move(upper_epoch), previous_physical,
      previous_guidance, executed_ops);
  if (telemetry != nullptr) {
    if (upper_epoch_cache != nullptr)
      telemetry->upper_epoch_cache_evictions +=
          upper_epoch_cache->evictions -
          cache_evictions_before;
    if (transition_valid && previous_guidance != nullptr &&
        previous_guidance->upper_epoch != nullptr &&
        guidance.upper_epoch != nullptr &&
        previous_guidance->upper_epoch->upper_signature !=
            guidance.upper_epoch->upper_signature) {
      const auto churn = compare_vacancy_epoch_churn(
          previous_guidance->upper_epoch->task_graph,
          guidance.upper_epoch->task_graph);
      telemetry->first_transfer_comparisons +=
          churn.first_transfer_comparisons;
      telemetry->first_transfer_flips +=
          churn.first_transfer_flips;
      telemetry->chain_overlap_samples +=
          churn.chain_overlap_samples;
      telemetry->chain_overlap_intersection +=
          churn.chain_overlap_intersection;
      telemetry->chain_overlap_union +=
          churn.chain_overlap_union;
    }
  }
  return guidance;
}

inline std::vector<int> task_br_robot_order(
    const PhysConfig& physical, const CarrierGuidance& guide,
    LowerDist& lower_distance)
{
  auto release_pending = [&](int robot) {
    return robot >= 0 &&
           robot < (int)guide.custody_by_robot.size() &&
           guide.custody_by_robot[robot].has_value() &&
           custody_arrived(
               physical, robot,
               *guide.custody_by_robot[robot]);
  };
  auto endpoint_contested_by_free_dispatch =
      [&](int carrier, int endpoint) {
        if (guide.upper_epoch == nullptr) return false;
        const auto& tasks =
            guide.upper_epoch->task_graph.tasks;
        for (size_t robot = 0;
             robot < physical.robots.size(); ++robot) {
          if (static_cast<int>(robot) == carrier ||
              robot >= physical.kappa.size() ||
              physical.kappa[robot] != KAPPA_FREE ||
              robot >= guide.rho_ready_index.size())
            continue;
          const int index = guide.rho_ready_index[robot];
          if (index < 0 ||
              index >= static_cast<int>(tasks.size()) ||
              lower_distance.dist(
                  endpoint, physical.robots[robot]) != 1)
            continue;
          if (tasks[index].id.from == endpoint)
            return true;
        }
        return false;
      };
  auto terminal_delivery_pending = [&](int robot) {
    if (robot < 0 ||
        robot >= static_cast<int>(physical.robots.size()) ||
        robot >= static_cast<int>(physical.kappa.size()) ||
        robot >= static_cast<int>(guide.custody_by_robot.size()) ||
        !guide.custody_by_robot[robot].has_value() ||
        guide.upper_epoch == nullptr)
      return false;
    const int target = physical.kappa[robot];
    if (target < 0 ||
        target >= static_cast<int>(
                      guide.upper_epoch->tau_guide.size()))
      return false;
    const auto& custody = *guide.custody_by_robot[robot];
    const int endpoint = custody_endpoint(custody);
    if (endpoint != guide.upper_epoch->tau_guide[target] ||
        physical.robots[robot] == endpoint ||
        lower_distance.dist(
            endpoint, physical.robots[robot]) != 1)
      return false;
    for (size_t other = 0;
         other < physical.target_pos.size(); ++other)
      if (static_cast<int>(other) != target &&
          physical.target_pos[other] == endpoint)
        return false;
    if (std::binary_search(
            physical.anon_occ.begin(),
            physical.anon_occ.end(), endpoint))
      return false;
    if (std::find(
            physical.robots.begin(),
            physical.robots.end(), endpoint) !=
        physical.robots.end())
      return false;
    return endpoint_contested_by_free_dispatch(
        robot, endpoint);
  };
  auto robot_class = [&](int robot) {
    const bool loaded = physical.kappa[robot] != KAPPA_FREE;
    const bool bound =
        robot < (int)guide.custody_by_robot.size() &&
        guide.custody_by_robot[robot].has_value();
    const bool assigned =
        robot < (int)guide.rho_task_id.size() &&
        guide.rho_task_id[robot].has_value();
    if (loaded && bound) return 0;
    if (loaded) return 1;
    if (assigned) return 2;
    return 3;
  };
  auto robot_priority = [&](int robot) {
    if (robot < (int)guide.custody_by_robot.size() &&
        guide.custody_by_robot[robot].has_value())
      return guide.custody_by_robot[robot]->priority;
    if (guide.upper_epoch != nullptr &&
        robot < (int)guide.rho_ready_index.size()) {
      const int index = guide.rho_ready_index[robot];
      if (index >= 0 &&
          index < (int)guide.upper_epoch->task_graph.tasks.size())
        return guide.upper_epoch->task_graph.tasks[index].priority;
    }
    return 0;
  };
  auto robot_distance = [&](int robot) {
    if (guide.upper_epoch == nullptr ||
        robot >= (int)guide.rho_ready_index.size())
      return 0;
    const int index = guide.rho_ready_index[robot];
    if (index < 0 ||
        index >= (int)guide.upper_epoch->task_graph.tasks.size())
      return 0;
    return lower_distance.dist(
        guide.upper_epoch->task_graph.tasks[index].id.from,
        physical.robots[robot]);
  };
  std::vector<int> order(physical.robots.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    const bool release_a = release_pending(a);
    const bool release_b = release_pending(b);
    if (release_a != release_b) return release_a;
    const bool delivery_a = terminal_delivery_pending(a);
    const bool delivery_b = terminal_delivery_pending(b);
    if (delivery_a != delivery_b) return delivery_a;
    const int priority_a = robot_priority(a);
    const int priority_b = robot_priority(b);
    if (priority_a != priority_b) return priority_a > priority_b;
    const int class_a = robot_class(a);
    const int class_b = robot_class(b);
    if (class_a != class_b) return class_a < class_b;
    const int distance_a = robot_distance(a);
    const int distance_b = robot_distance(b);
    return distance_a != distance_b ? distance_a < distance_b : a < b;
  });
  return order;
}

}  // namespace carrier_detail
