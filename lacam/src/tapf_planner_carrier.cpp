// Carrier guidance attachment, scratch state, and carrier rollout.
// Split from the original tapf_planner.cpp.

#include "tapf_planner_internal.hpp"

using namespace tapf_detail;
using namespace carrier_detail;

static void add_vacancy_guidance_telemetry(
    TAPFStats* stats,
    const VacancyGuidanceTelemetry& telemetry)
{
  if (stats == nullptr) return;
  stats->vacancy_potential_builds += telemetry.potential_builds;
  stats->vacancy_potential_unreachable_cells +=
      telemetry.potential_unreachable_cells;
  stats->clearance_first_choice_fallbacks +=
      telemetry.first_choice_fallbacks;
  stats->vacancy_potential_time_ms +=
      telemetry.potential_time_ms;
  stats->epoch_first_transfer_comparisons +=
      telemetry.first_transfer_comparisons;
  stats->epoch_first_transfer_flips +=
      telemetry.first_transfer_flips;
  stats->epoch_chain_overlap_samples +=
      telemetry.chain_overlap_samples;
  stats->epoch_chain_overlap_intersection +=
      telemetry.chain_overlap_intersection;
  stats->epoch_chain_overlap_union +=
      telemetry.chain_overlap_union;
  stats->upper_epoch_cache_evictions +=
      telemetry.upper_epoch_cache_evictions;
}

void TAPFPlanner::attach_carrier_root_guidance(TAPFNode* nd)
{
  if (search_config.carrier_root_guidance_output != nullptr)
    search_config.carrier_root_guidance_output->reset();

  const long cache_hits_before =
      stats != nullptr ? stats->pair_cache_hits : 0;
  const long cache_misses_before =
      stats != nullptr ? stats->pair_cache_misses : 0;
  const long edges_evaluated_before =
      stats != nullptr ? stats->pair_edges_evaluated : 0;
  const long edges_total_before =
      stats != nullptr ? stats->pair_edges_total : 0;
  const long edges_reused_before =
      stats != nullptr ? stats->pair_edges_reused : 0;

  const auto* continuation =
      search_config.carrier_root_continuation;
  if (continuation == nullptr) {
    attach_carrier_guidance(nd);
  } else {
    if (carrier == nullptr || dd_view == nullptr)
      throw std::invalid_argument(
          "carrier continuation requires a shelf layer");
    if (continuation->executed_prefix.empty())
      throw std::invalid_argument(
          "carrier continuation prefix must not be empty");

    std::vector<PhysConfig> states;
    states.reserve(continuation->executed_prefix.size() + 1);
    states.push_back(continuation->previous_physical);
    const bool has_spacetime_commitment =
        search_config.spacetime_commitment != nullptr &&
        !search_config.spacetime_commitment->empty();
    int64_t prefix_start_tick = 0;
    if (has_spacetime_commitment) {
      if (continuation->executed_prefix.size() >
          static_cast<uint64_t>(nd->absolute_tick))
        throw std::invalid_argument(
            "carrier continuation predates commitment origin");
      prefix_start_tick =
          nd->absolute_tick -
          static_cast<int64_t>(
              continuation->executed_prefix.size());
    }
    for (size_t step = 0;
         step < continuation->executed_prefix.size(); ++step) {
      const auto& ops =
          continuation->executed_prefix[step];
      const auto absolute_tick =
          checked_absolute_tick_add(
              prefix_start_tick, step);
      if (!absolute_tick.has_value())
        throw std::invalid_argument(
            "carrier continuation tick overflow");
      const auto next = apply_ops(
          *dd_view, states.back(), ops, true,
          has_spacetime_commitment
              ? search_config.spacetime_commitment
              : nullptr,
          *absolute_tick);
      if (!next.has_value())
        throw std::invalid_argument(
            "carrier continuation contains an invalid joint action");
      states.push_back(*next);
    }
    const PhysConfig root_physical = carrier->phys_view(nd);
    if (!(states.back() == root_physical))
      throw std::invalid_argument(
          "carrier continuation does not reach the search root");

    CarrierGuidance previous_guidance =
        continuation->previous_guidance;
    for (size_t step = 0;
         step < continuation->executed_prefix.size(); ++step) {
      const auto& ops = continuation->executed_prefix[step];
      if (step + 1 == continuation->executed_prefix.size()) {
        attach_carrier_guidance(
            nd, &states[step], &previous_guidance, &ops);
        break;
      }

      auto intermediate = std::make_unique<TAPFNode>(
          config_of_physical(*ins, states[step + 1]),
          shelf_of_physical(states[step + 1]), D, ins,
          std::vector<int>(N, -1), TAPFAssignmentState());
      const auto intermediate_tick =
          checked_absolute_tick_add(
              prefix_start_tick, step + 1);
      if (!intermediate_tick.has_value())
        throw std::invalid_argument(
            "carrier continuation tick overflow");
      intermediate->absolute_tick =
          *intermediate_tick;
      attach_carrier_guidance(
          intermediate.get(), &states[step],
          &previous_guidance, &ops);
      if (intermediate->guide == nullptr)
        throw std::logic_error(
            "carrier continuation lost root guidance");
      previous_guidance = *intermediate->guide;
    }
  }

  if (search_config.carrier_root_guidance_output != nullptr &&
      nd->guide != nullptr)
    *search_config.carrier_root_guidance_output =
        std::make_shared<CarrierGuidance>(*nd->guide);

  if (stats != nullptr) {
    stats->root_pair_cache_hits +=
        stats->pair_cache_hits - cache_hits_before;
    stats->root_pair_cache_misses +=
        stats->pair_cache_misses - cache_misses_before;
    stats->root_pair_edges_evaluated +=
        stats->pair_edges_evaluated -
        edges_evaluated_before;
    stats->root_pair_edges_total +=
        stats->pair_edges_total - edges_total_before;
    stats->root_pair_edges_reused +=
        stats->pair_edges_reused - edges_reused_before;
  }
}

void TAPFPlanner::attach_carrier_guidance(
    TAPFNode* nd, const PhysConfig* transition_previous_X,
    const CarrierGuidance* transition_previous_guidance,
    const std::vector<Op>* transition_executed_ops)
{
  if (search_config.event_contract != nullptr) {
    if (carrier == nullptr || dd_view == nullptr)
      throw std::logic_error(
          "event contract without carrier state");
    const bool initialize_node = nd->guide == nullptr;
    const auto guidance_started =
        std::chrono::steady_clock::now();
    const PhysConfig physical = carrier->phys_view(nd);
    std::vector<int> carrying;
    std::vector<int> provisional;
    std::vector<uint8_t> active(ins->N, 0);
    for (const auto& fixed :
         search_config.event_contract->active_transfers) {
      active[fixed.robot] = 1;
      const auto phase =
          carrier_event_phase_of(physical, fixed);
      if (phase.kind == CarrierTaskPhaseKind::CARRYING)
        carrying.push_back(fixed.robot);
      else
        provisional.push_back(fixed.robot);
    }
    std::sort(carrying.begin(), carrying.end());
    std::sort(provisional.begin(), provisional.end());
    nd->order.clear();
    nd->order.insert(
        nd->order.end(), carrying.begin(), carrying.end());
    nd->order.insert(
        nd->order.end(), provisional.begin(),
        provisional.end());
    for (size_t robot = 0; robot < ins->N; ++robot)
      if (!active[robot])
        nd->order.push_back(static_cast<int>(robot));
    if (initialize_node) nd->constraint_order = nd->order;
    nd->h_guidance = 0;
    nd->guide = std::make_unique<CarrierGuidance>();
    if (stats != nullptr) {
      ++stats->guidance_builds;
      stats->guidance_time_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() -
              guidance_started)
              .count();
    }
    return;
  }
  if (ins->target_starts.empty()) return;  // natural degradation
  auto& task_br_engine = *carrier;
  const auto& dd_instance = *dd_view;
  const PhysConfig physical = task_br_engine.phys_view(nd);
  const bool initialize_node = nd->guide == nullptr;
  const auto guidance_started = std::chrono::steady_clock::now();

  const PhysConfig* previous_physical = nullptr;
  const std::vector<Op>* transition_ops = nullptr;
  const CarrierGuidance* previous_guidance = nullptr;
  if (transition_previous_X != nullptr &&
      transition_previous_guidance != nullptr &&
      transition_executed_ops != nullptr) {
    const bool has_spacetime_commitment =
        search_config.spacetime_commitment != nullptr &&
        !search_config.spacetime_commitment->empty();
    const int64_t transition_tick =
        has_spacetime_commitment &&
                nd->absolute_tick > 0
            ? nd->absolute_tick - 1
            : 0;
    const auto replayed =
        apply_ops(
            dd_instance, *transition_previous_X,
            *transition_executed_ops, true,
            has_spacetime_commitment
                ? search_config.spacetime_commitment
                : nullptr,
            transition_tick);
    if (replayed.has_value() && *replayed == physical) {
      previous_physical = transition_previous_X;
      transition_ops = transition_executed_ops;
      previous_guidance = transition_previous_guidance;
    }
  }

  const std::vector<std::optional<TaskId>> previous_rho =
      previous_guidance != nullptr
          ? previous_guidance->rho_task_id
          : std::vector<std::optional<TaskId>>{};
  const std::vector<std::optional<TransferKey>>
      previous_rho_transfer_key =
          previous_guidance != nullptr
              ? previous_guidance->rho_transfer_key
              : std::vector<std::optional<TransferKey>>{};
  const std::vector<int> previous_tau =
      previous_guidance != nullptr && previous_guidance->upper_epoch != nullptr
          ? previous_guidance->upper_epoch->tau_guide
          : std::vector<int>{};
  const UpperSignature* previous_upper =
      previous_guidance != nullptr &&
              previous_guidance->upper_epoch != nullptr
          ? &previous_guidance->upper_epoch->upper_signature
          : nullptr;
  const long cache_hits_before = task_br_engine.task_br_cache.hits;
  const long cache_misses_before = task_br_engine.task_br_cache.misses;
  VacancyGuidanceTelemetry vacancy_telemetry;

  auto guide = std::make_unique<CarrierGuidance>(
      build_task_br_guidance(
          dd_instance, physical, task_br_engine.upper_wall,
          task_br_engine.storage_topology,
          weights.alpha, weights.gamma, weights.delta,
          previous_physical, previous_guidance, transition_ops,
          &task_br_engine.task_br_cache,
          &vacancy_telemetry));
  add_vacancy_guidance_telemetry(stats, vacancy_telemetry);

  long guidance_h = 0;
  if (guide->upper_epoch != nullptr) {
    const auto& tau = guide->upper_epoch->tau_guide;
    for (size_t target = 0;
         target < dd_instance.n_targets() && target < tau.size();
         ++target) {
      const int distance = task_br_engine.upper_wall.dist(
          tau[target], physical.target_pos[target]);
      if (distance < INT_MAX / 4 &&
          physical.target_pos[target] != tau[target])
        guidance_h += distance + 2;
    }
  }
  nd->h_guidance = guidance_h;

  nd->order = task_br_robot_order(
      physical, *guide, task_br_engine.lower);

  if (initialize_node) {
    nd->constraint_order = nd->order;
    const double shelf_lb = solve_tau_lb(
        dd_instance, physical, task_br_engine.upper_wall,
        weights.alpha, weights.gamma);
    nd->h += PlanCost::legacy(shelf_lb);
    if (search_config.objective ==
        TAPFObjective::MAKESPAN_THEN_WORK) {
      nd->h.ticks = std::max(
          nd->h.ticks,
          solve_tau_time_lb(
              dd_instance, physical, task_br_engine.upper_wall));
    }
    nd->f = nd->g + nd->h;
  }
  nd->guide = std::move(guide);

  if (stats != nullptr) {
    ++stats->guidance_builds;
    long cache_hits =
        task_br_engine.task_br_cache.hits - cache_hits_before;
    const long cache_misses =
        task_br_engine.task_br_cache.misses - cache_misses_before;
    // Adjacent robot-only transitions retain the exact immutable epoch
    // directly from the previous guidance and therefore do not perform a
    // map lookup. Report that production fast path as a cache hit as well;
    // otherwise a successful warm continuation misleadingly appears cold.
    if (cache_hits == 0 && cache_misses == 0 &&
        previous_guidance != nullptr &&
        previous_guidance->upper_epoch != nullptr &&
        nd->guide->upper_epoch ==
            previous_guidance->upper_epoch)
      cache_hits = 1;
    stats->pair_cache_hits += cache_hits;
    stats->pair_cache_misses += cache_misses;
    stats->upper_epoch_builds += cache_misses;

    if (cache_misses > 0 && nd->guide->upper_epoch != nullptr) {
      const auto& epoch = *nd->guide->upper_epoch;
      // Session telemetry reports unique PairCost edges that had to be
      // rebuilt for this epoch. The epoch's historical
      // pair_edges_evaluated counter also counts reused exact entries, so it
      // is not the changed-edge quantity needed by incremental callers.
      stats->pair_edges_evaluated +=
          epoch.pair_edges_total -
          epoch.pair_edges_reused;
      stats->pair_edges_total +=
          epoch.pair_edges_total;
      stats->pair_edges_reused +=
          epoch.pair_edges_reused;
      stats->pair_incremental_reuses +=
          epoch.pair_edges_reused;
      stats->pair_hungarian_full_solves +=
          epoch.pair_hungarian_full_solves;
      stats->pair_hungarian_row_repairs +=
          epoch.pair_hungarian_row_repairs;
      stats->pair_hungarian_forced_repairs +=
          epoch.pair_hungarian_forced_repairs;
      stats->pair_rollout_steps += epoch.pair_rollout_work_steps;
      stats->pair_rollout_truncations +=
          epoch.pair_rollout_truncations;
      stats->pair_rollout_stalls += epoch.pair_rollout_stalls;
      stats->joint_task_nodes += epoch.task_graph.tasks.size();
      for (const auto& predecessors : epoch.task_graph.predecessors)
        stats->joint_task_edges += predecessors.size();
      for (const auto& task : epoch.task_graph.tasks)
        stats->joint_shared_effects += task.roots.size() > 1;
      stats->joint_effect_conflicts +=
          epoch.task_graph.effect_conflicts;
      stats->joint_candidate_backtracks +=
          epoch.task_graph.candidate_backtracks;
      stats->joint_paused_roots +=
          epoch.task_graph.paused_roots.size();
    }
    stats->ready_task_count += nd->guide->ready_tasks.size();
    const auto accumulate_rho_telemetry =
        [&](const RhoMatchTelemetry& telemetry, bool execute) {
          if (telemetry.objective_version !=
              stats->rho_objective_version)
            throw std::logic_error(
                "mixed rho objective versions in one search");
          if (execute)
            ++stats->rho_match_calls_execute;
          else
            ++stats->rho_match_calls_prepare;
          stats->rho_candidates_input +=
              telemetry.candidates_input;
          stats->rho_candidates_after_claims +=
              telemetry.candidates_after_claims;
          stats->rho_candidates_after_key_dedupe +=
              telemetry.candidates_after_key_dedupe;
          stats->rho_candidates_after_shelf_preselect +=
              telemetry.candidates_after_shelf_preselect;
          stats->rho_candidates_after_priority +=
              telemetry.candidates_after_priority;
          stats->rho_invalid_filtered +=
              telemetry.invalid_filtered;
          stats->rho_duplicate_key_filtered +=
              telemetry.duplicate_key_filtered;
          stats->rho_same_shelf_filtered +=
              telemetry.same_shelf_filtered;
          stats->rho_upstream_claim_filtered +=
              telemetry.upstream_claim_filtered;
          stats->rho_mode_ineligible_filtered +=
              telemetry.mode_ineligible_filtered;
          stats->rho_no_reachable_robot_filtered +=
              telemetry.no_reachable_robot_filtered;
          stats->rho_priority_filtered +=
              telemetry.priority_filtered;
          stats->rho_matrix_rows_total += telemetry.matrix_rows;
          stats->rho_matrix_cols_total += telemetry.matrix_cols;
          stats->rho_matrix_max_rows = std::max(
              stats->rho_matrix_max_rows, telemetry.matrix_rows);
          stats->rho_incremental_full_solves +=
              telemetry.incremental_full_solves;
          stats->rho_incremental_repairs +=
              telemetry.incremental_repairs;
          stats->rho_incremental_zero_row_reuses +=
              telemetry.incremental_zero_row_reuses;
          stats->rho_incremental_changed_rows_total +=
              telemetry.incremental_changed_rows;
          stats->rho_candidate_time_ms +=
              telemetry.candidate_time_ms;
          stats->rho_matrix_time_ms += telemetry.matrix_time_ms;
          stats->rho_bottleneck_time_ms +=
              telemetry.bottleneck_time_ms;
          stats->rho_secondary_full_time_ms +=
              telemetry.secondary_full_time_ms;
          stats->rho_secondary_repair_time_ms +=
              telemetry.secondary_repair_time_ms;
          stats->rho_canonical_time_ms +=
              telemetry.canonical_time_ms;
        };
    accumulate_rho_telemetry(
        nd->guide->rho_execute_telemetry, true);
    accumulate_rho_telemetry(
        nd->guide->rho_prepare_telemetry, false);
    stats->timed_transport_expansions +=
        nd->guide->timed_transport.expansions;
    stats->timed_transport_frames +=
        nd->guide->timed_transport.frames_evaluated;
    stats->timed_transport_time_ms +=
        nd->guide->timed_transport.build_time_ms;

    if (previous_upper != nullptr &&
        nd->guide->upper_epoch != nullptr &&
        *previous_upper !=
            nd->guide->upper_epoch->upper_signature &&
        previous_tau.size() ==
            nd->guide->upper_epoch->tau_guide.size()) {
      for (size_t target = 0; target < previous_tau.size(); ++target)
        stats->tau_guide_changes_on_upper_move +=
            previous_tau[target] !=
            nd->guide->upper_epoch->tau_guide[target];
    }
    if (!previous_rho.empty() &&
        previous_rho.size() == nd->guide->rho_task_id.size()) {
      for (size_t robot = 0; robot < previous_rho.size(); ++robot) {
        const bool changed =
            previous_rho[robot] != nd->guide->rho_task_id[robot];
        stats->rho_repairs += changed;
        stats->rho_assignment_changes += changed;
      }
    }
    if (previous_guidance != nullptr) {
      const bool identity_same =
          previous_guidance->rho_execute_telemetry
                  .column_identity_fingerprint ==
              nd->guide->rho_execute_telemetry
                  .column_identity_fingerprint &&
          previous_guidance->rho_prepare_telemetry
                  .column_identity_fingerprint ==
              nd->guide->rho_prepare_telemetry
                  .column_identity_fingerprint;
      const bool value_same =
          previous_guidance->rho_execute_telemetry
                  .column_value_fingerprint ==
              nd->guide->rho_execute_telemetry
                  .column_value_fingerprint &&
          previous_guidance->rho_prepare_telemetry
                  .column_value_fingerprint ==
              nd->guide->rho_prepare_telemetry
                  .column_value_fingerprint;
      stats->rho_column_identity_same += identity_same;
      stats->rho_column_value_same += value_same;
      stats->rho_mode_or_conflict_same +=
          previous_guidance->rho_mode_or_conflict_fingerprint ==
          nd->guide->rho_mode_or_conflict_fingerprint;

      size_t changed_rows = 0;
      const size_t common_rows = std::min(
          previous_guidance->rho_row_fingerprints.size(),
          nd->guide->rho_row_fingerprints.size());
      for (size_t row = 0; row < common_rows; ++row)
        changed_rows +=
            previous_guidance->rho_row_fingerprints[row] !=
            nd->guide->rho_row_fingerprints[row];
      changed_rows +=
          std::max(
              previous_guidance->rho_row_fingerprints.size(),
              nd->guide->rho_row_fingerprints.size()) -
          common_rows;
      if (changed_rows == 0)
        ++stats->rho_changed_rows_0;
      else if (changed_rows == 1)
        ++stats->rho_changed_rows_1;
      else if (changed_rows == 2)
        ++stats->rho_changed_rows_2;
      else
        ++stats->rho_changed_rows_gt2;
    }
    if (!previous_rho_transfer_key.empty()) {
      std::map<TransferKey, int> previous_owner;
      std::map<TransferKey, int> current_owner;
      for (size_t robot = 0;
           robot < previous_rho_transfer_key.size(); ++robot)
        if (previous_rho_transfer_key[robot].has_value())
          previous_owner.emplace(
              *previous_rho_transfer_key[robot], (int)robot);
      for (size_t robot = 0;
           robot < nd->guide->rho_transfer_key.size(); ++robot)
        if (nd->guide->rho_transfer_key[robot].has_value())
          current_owner.emplace(
              *nd->guide->rho_transfer_key[robot], (int)robot);
      for (const auto& [key, owner] : current_owner) {
        const auto previous = previous_owner.find(key);
        if (previous != previous_owner.end() &&
            previous->second != owner)
          ++stats->owner_handoffs;
      }
    }
    if (previous_physical != nullptr && transition_ops != nullptr) {
      for (size_t robot = 0; robot < physical.kappa.size(); ++robot) {
        if (robot >= transition_ops->size()) continue;
        if ((*previous_physical).kappa[robot] != KAPPA_FREE &&
            (*transition_ops)[robot].kind == Op::MOVE &&
            robot < nd->guide->custody_by_robot.size() &&
            nd->guide->custody_by_robot[robot].has_value())
          ++stats->custody_continuations;
        if ((*transition_ops)[robot].kind != Op::WAIT ||
            previous_guidance == nullptr)
          continue;
        if (robot < previous_guidance->rho_mode.size() &&
            previous_guidance->rho_mode[robot] ==
                DispatchMode::PREPARE)
          ++stats->causal_waiting;
        if ((*previous_physical).kappa[robot] != KAPPA_FREE &&
            robot <
                previous_guidance->timed_transport.by_robot.size() &&
            previous_guidance->timed_transport.by_robot[robot]
                .has_value()) {
          const auto& hint =
              *previous_guidance->timed_transport.by_robot[robot];
          if ((hint.status == RouteStatus::OK ||
               hint.status == RouteStatus::PREFIX) &&
              hint.cells.size() >= 2 &&
              hint.cells.front() ==
                  (*previous_physical).robots[robot] &&
              hint.cells[1] == hint.cells.front())
            ++stats->traffic_waiting;
        }
      }
    }

    if (nd->guide->upper_epoch != nullptr) {
      const auto& upper =
          nd->guide->upper_epoch->upper_signature;
      if (zero_storage_vacancy_no_ready(
              dd_instance, upper, nd->guide->ready_tasks.size(),
              nd->guide->upper_epoch->task_graph.tasks.size()))
        ++stats->zero_empty_no_ready;
    }
    stats->guidance_time_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - guidance_started)
            .count();
  }
}

void TAPFPlanner::ensure_guidance_fresh(TAPFNode* nd)
{
  if (nd == nullptr || !nd->guidance_stale) return;
  if (search_config.event_contract != nullptr) {
    attach_carrier_guidance(nd);
    nd->guidance_stale = false;
    return;
  }
  if (ins->target_starts.empty()) {
    nd->guidance_stale = false;
    return;
  }
  if (nd->parent != nullptr) ensure_guidance_fresh(nd->parent);

  const PlanCost saved_g = nd->g;
  const PlanCost saved_h = nd->h;
  const PlanCost saved_f = nd->f;
  const auto saved_constraint_order = nd->constraint_order;

  if (nd->parent == nullptr || nd->incoming_edge == nullptr ||
      nd->parent->guide == nullptr ||
      nd->incoming_edge->transition_trace.empty()) {
    attach_carrier_guidance(nd);
  } else {
    const auto& trace = nd->incoming_edge->transition_trace;
    if (nd->incoming_edge->to != nd)
      throw std::logic_error(
          "ensure_guidance_fresh: incoming edge target mismatch");
    PhysConfig anchor_X =
        physical_state_of(nd->parent->C, nd->parent->shelf);
    CarrierGuidance anchor = *nd->parent->guide;
    for (size_t step = 0; step + 1 < trace.size(); ++step) {
      if (!(trace[step].previous_X == anchor_X))
        throw std::logic_error(
            "ensure_guidance_fresh: stale parent/trace anchor");
      const auto absolute_tick =
          checked_absolute_tick_add(
              nd->parent->absolute_tick, step);
      if (!absolute_tick.has_value())
        throw std::logic_error(
            "ensure_guidance_fresh: tick overflow");
      const auto replayed = apply_ops(
          *dd_view, anchor_X, trace[step].ops, true,
          search_config.spacetime_commitment,
          *absolute_tick);
      if (!replayed.has_value() ||
          !(*replayed == trace[step].next_X))
        throw std::logic_error(
            "ensure_guidance_fresh: invalid intermediate transition");
      VacancyGuidanceTelemetry vacancy_telemetry;
      anchor = build_task_br_guidance(
          *dd_view, *replayed, carrier->upper_wall,
          carrier->storage_topology, weights.alpha,
          weights.gamma, weights.delta, &anchor_X, &anchor,
          &trace[step].ops, &carrier->task_br_cache,
          &vacancy_telemetry);
      add_vacancy_guidance_telemetry(
          stats, vacancy_telemetry);
      anchor_X = *replayed;
      if (stats != nullptr) ++stats->guidance_builds;
    }
    const auto& last = trace.back();
    if (!(last.previous_X == anchor_X))
      throw std::logic_error(
          "ensure_guidance_fresh: final transition anchor mismatch");
    attach_carrier_guidance(
        nd, &anchor_X, &anchor, &last.ops);
  }

  nd->g = saved_g;
  nd->h = saved_h;
  nd->f = saved_f;
  nd->constraint_order = saved_constraint_order;
  nd->guidance_stale = false;
}


ShelfState initial_shelf_state(const TAPFInstance& ins)
{
  ShelfState S;
  if (ins.shelf_cells.empty() &&
      ins.fixed_upper_cells.empty())
    return S;  // shelf-free: empty layer
  S.target_pos = ins.target_starts;
  std::unordered_set<int> tset(ins.target_starts.begin(),
                               ins.target_starts.end());
  for (const int p : ins.shelf_cells)
    if (!tset.count(p)) S.anon_occ.push_back(p);
  std::sort(S.anon_occ.begin(), S.anon_occ.end());
  S.kappa.assign(ins.N, KAPPA_FREE);
  return S;
}

TAPFNode::TAPFNode(Config _C, ShelfState _shelf, TAPFDistTable& D,
                   const TAPFInstance* ins, std::vector<int> _assignment,
                   TAPFAssignmentState _assignment_state, TAPFNode* _parent)
    : LacamNodeCore<TAPFConstraint, TAPFNode>(_C, _parent),
      shelf(std::move(_shelf)),
      assignment(_assignment),
      assignment_state(_assignment_state),
      queued(false),
      g(0),
      h(0),
      f(0),
      depth(parent == nullptr ? 0 : parent->depth + 1),
      non_goal_waits(parent == nullptr ? 0 : parent->non_goal_waits),
      reversals(parent == nullptr ? 0 : parent->reversals),
      distance_increases(parent == nullptr ? 0 : parent->distance_increases),
      settled_pushes(parent == nullptr ? 0 : parent->settled_pushes)
{
  refresh_priority(D);
  refresh_search_metrics(D, ins);
  constraint_order = order;  // frozen for the node's lifetime (D11, M3)
}

void TAPFNode::discard_search_tree()
{
  while (!search_tree.empty()) search_tree.pop();
}

void TAPFNode::refresh_priority(TAPFDistTable& D)
{
  // shared core machinery, task-keyed distance (node-skeleton audit 5);
  // carrier agents (no instance task) contribute 0 here — their PIBT
  // ordering comes from the guidance layer (design 5.4, WP4)
  init_priorities_and_order([&](size_t i) {
    return assignment[i] >= 0 ? D.get(assignment[i], C[i]) : 0;
  });
}

void TAPFNode::refresh_search_metrics(TAPFDistTable& D,
                                      const TAPFInstance* ins)
{
  if (parent == nullptr) return;

  for (size_t i = 0; i < C.size(); ++i) {
    const auto task = assignment[i];
    if (task < 0) continue;  // carrier agent: no instance task
    const auto goal = ins->tasks[task];
    if (C[i] == parent->C[i] && C[i] != goal) ++non_goal_waits;
    if (parent->parent != nullptr && C[i] == parent->parent->C[i] &&
        C[i] != parent->C[i]) {
      ++reversals;
    }
    if (D.get(task, C[i]) > D.get(task, parent->C[i])) ++distance_increases;
  }

  for (size_t pusher = 0; pusher < C.size(); ++pusher) {
    if (C[pusher] == parent->C[pusher]) continue;
    for (size_t pushed = 0; pushed < C.size(); ++pushed) {
      if (pusher == pushed || C[pusher] != parent->C[pushed] ||
          C[pushed] == parent->C[pushed]) {
        continue;
      }
      if (assignment[pushed] < 0) continue;  // carrier agent
      const auto pushed_goal = ins->tasks[assignment[pushed]];
      if (parent->C[pushed] == pushed_goal) ++settled_pushes;
    }
  }
}

void TAPFPlanner::invalidate_carrier_scratch()
{
  carrier_scratch_node = nullptr;
}

void TAPFPlanner::refresh_carrier_scratch(const TAPFNode* S)
{
  if (carrier_scratch_node == S) return;
  carrier_scratch_node = S;
  if (S->shelf.kappa.empty()) return;  // no shelf layer: nothing to fill
  std::fill(carrier_grounded.begin(), carrier_grounded.end(), 0);
  for (const int p : dd_view->fixed_upper_cells)
    carrier_grounded[p] = CARRIER_GROUNDED_FIXED;
  for (const int p : S->shelf.anon_occ)
    carrier_grounded[p] = CARRIER_GROUNDED_ANON;
  std::vector<char> carried(ins->target_starts.size(), 0);
  for (const int k : S->shelf.kappa)
    if (k >= 0) carried[k] = 1;
  for (size_t b = 0; b < S->shelf.target_pos.size(); ++b)
    if (!carried[b]) carrier_grounded[S->shelf.target_pos[b]] = (int)b + 1;
}

bool TAPFPlanner::carrier_upper_taken(int cell) const
{
  // grounded shelves occupy their cell at t+1 (carrier_grounded != 0);
  // carried-shelf destination reservations live in the delta counters
  return carrier_grounded[cell] != 0 || carrier_upper_delta[cell] > 0;
}

void TAPFPlanner::carrier_upper_add(int cell)
{
  ++carrier_upper_delta[cell];
  carrier_upper_touched.push_back(cell);
}

void TAPFPlanner::carrier_upper_sub(int cell)
{
  --carrier_upper_delta[cell];  // touched entry stays; reset zeroes it
}

bool TAPFPlanner::spacetime_candidate_feasible(
    const Agent* agent, const Vertex* destination,
    uint8_t kind) const
{
  const auto* commitment =
      search_config.spacetime_commitment;
  if (commitment == nullptr || commitment->empty())
    return true;
  if (agent == nullptr || agent->v_now == nullptr ||
      destination == nullptr || carrier_scratch_node == nullptr)
    return false;

  const int64_t tick =
      carrier_scratch_node->absolute_tick;
  if (tick == std::numeric_limits<int64_t>::max())
    return false;
  if (commitment->blocks_lower(
          destination->index, tick + 1))
    return false;
  if (kind == Op::MOVE &&
      destination != agent->v_now &&
      commitment->has_lower_edge(
          destination->index, agent->v_now->index, tick))
    return false;

  bool upper_at_destination = false;
  if (!carrier_scratch_node->shelf.kappa.empty()) {
    const int kappa =
        carrier_scratch_node->shelf.kappa[agent->id];
    upper_at_destination =
        kind == Op::LIFT || kind == Op::DROP ||
        (kappa != KAPPA_FREE && kind != Op::LIFT);
  }
  return !upper_at_destination ||
         !commitment->blocks_upper(
             destination->index, tick + 1);
}

// per-kind preconditions of a FORCED op under a partial constraint
// (final arbitration is the oracle's; a wrong veto here only delays the
// combination to a deeper constraint — G1 keeps completeness)

std::vector<std::vector<Op>> derive_carrier_ops(
    const TAPFInstance& ins, const Solution& sol,
    const std::vector<ShelfState>& shelves)
{
  std::vector<std::vector<Op>> plan;
  if (sol.size() < 2) return plan;
  plan.reserve(sol.size() - 1);
  for (size_t t = 1; t < sol.size(); ++t) {
    std::vector<Op> ops(ins.N, Op::make_wait());
    for (size_t i = 0; i < ins.N; ++i) {
      if (sol[t][i] != sol[t - 1][i]) {
        ops[i] = Op::make_move(sol[t][i]->index);
        continue;
      }
      const int k_from =
          shelves[t - 1].kappa.empty() ? KAPPA_FREE : shelves[t - 1].kappa[i];
      const int k_to =
          shelves[t].kappa.empty() ? KAPPA_FREE : shelves[t].kappa[i];
      if (k_from == KAPPA_FREE && k_to != KAPPA_FREE)
        ops[i] = Op::make_lift();
      else if (k_from != KAPPA_FREE && k_to == KAPPA_FREE)
        ops[i] = Op::make_drop();
    }
    plan.push_back(std::move(ops));
  }
  return plan;
}

// physical cost of one joint op (design 2.3; solver weights)
static int64_t carrier_ops_work_scaled(
    const TAPFPlanner::Weights& w, const ShelfState& from,
    const std::vector<Op>& ops)
{
  int64_t work = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
    if (ops[i].kind == Op::MOVE) {
      add_scaled_work(
          work, from.kappa[i] == KAPPA_FREE
                    ? w.beta_scaled
                    : w.alpha_scaled);
      if (from.kappa[i] == KAPPA_ANON)
        add_scaled_work(work, w.delta_scaled);
    } else if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) {
      add_scaled_work(work, w.gamma_scaled);
    }
  }
  return work;
}

TAPFPlanner::CarrierRollout TAPFPlanner::carrier_rollout(const Config& C0,
                                                         const ShelfState& S0,
                                                         int max_steps,
                                                         int min_chunk,
                                                         bool stop_on_event,
                                                         const TAPFNode*
                                                             initial_anchor)
{
  CarrierRollout out;
  if (stats != nullptr) ++stats->rollout_calls;
  out.configs.push_back(C0);
  out.shelves.push_back(S0);
  std::unordered_set<uint64_t> local_seen;
  auto state_hash = [&](const Config& C, const ShelfState& S,
                        int64_t absolute_tick) {
    uint64_t hash =
        (uint64_t)ConfigHasher()(C) ^ shelf_layer_hash(S);
    if (search_config.spacetime_commitment != nullptr) {
      const int64_t phase =
          search_config.spacetime_commitment->phase_at(
              absolute_tick);
      if (phase != 0)
        hash ^= splitmix64(static_cast<uint64_t>(phase));
    }
    return hash;
  };
  const int64_t initial_tick =
      initial_anchor != nullptr
          ? initial_anchor->absolute_tick
          : search_config.commitment_time_origin;
  local_seen.insert(state_hash(C0, S0, initial_tick));
  auto make_rollout_node = [&](const Config& C, const ShelfState& S,
                               int64_t absolute_tick) {
    auto node = std::make_unique<TAPFNode>(
        C, S, D, ins, std::vector<int>(N, -1),
        TAPFAssignmentState(), nullptr);
    node->absolute_tick = absolute_tick;
    return node;
  };
  auto current = make_rollout_node(C0, S0, initial_tick);
  invalidate_carrier_scratch();
  if (initial_anchor != nullptr) {
    if (!is_same_config(initial_anchor->C, C0) ||
        !(initial_anchor->shelf == S0) ||
        initial_anchor->guide == nullptr ||
        initial_anchor->guidance_stale)
      throw std::invalid_argument(
          "carrier_rollout: invalid initial guidance anchor");
    current->guide =
        std::make_unique<CarrierGuidance>(*initial_anchor->guide);
    current->order = initial_anchor->order;
    current->constraint_order = initial_anchor->constraint_order;
    current->h = initial_anchor->h;
    current->f = current->g + current->h;
    current->h_guidance = initial_anchor->h_guidance;
  } else {
    attach_carrier_guidance(current.get());
  }
  if (current->guide != nullptr) {
    out.terminal_guidance =
        std::make_shared<CarrierGuidance>(*current->guide);
    out.terminal_order = current->order;
    out.terminal_h = current->h;
    out.terminal_h_guidance = current->h_guidance;
  }
  auto C_step = Config(N, nullptr);
  for (int step = 0; step < max_steps; ++step) {
    const Config& cur = out.configs.back();
    const ShelfState& curS = out.shelves.back();
    if (is_goal_config(cur, curS)) {
      out.reached_goal = true;
      return out;
    }
    const PhysConfig previous_X = carrier->phys_view(current.get());
    TAPFConstraint root;
    if (!get_new_config(current.get(), &root)) return out;
    if (!apply_carrier_effects(current.get())) return out;
    for (auto a : A) C_step[a->id] = a->v_next;
    const auto ops = ops_scratch;
    const auto next_tick_checked =
        checked_absolute_tick_add(
            current->absolute_tick, 1);
    if (!next_tick_checked.has_value()) return out;
    const int64_t next_tick = *next_tick_checked;
    if (!local_seen.insert(
             state_hash(
                 C_step, shelf_next_scratch, next_tick))
             .second) {
      if (stats != nullptr) ++stats->rollout_cycles;
      return out;
    }
    bool shelf_motion = false;
    for (size_t i = 0; i < ops.size(); ++i)
      shelf_motion |=
          ops[i].kind == Op::MOVE && !curS.kappa.empty() &&
          curS.kappa[i] != KAPPA_FREE;
    if (shelf_motion) {
      out.shelf_moved = true;
      if (stats != nullptr) ++stats->rollout_shelf_motion_steps;
    }
    out.cost += PlanCost::from_scaled(
        search_config.objective ==
                TAPFObjective::MAKESPAN_THEN_WORK
            ? 1
            : 0,
        carrier_ops_work_scaled(weights, curS, ops));
    out.ops.push_back(ops);
    out.configs.push_back(C_step);
    out.shelves.push_back(shelf_next_scratch);
    if (stats != nullptr) ++stats->macro_steps;

    auto next = make_rollout_node(
        C_step, shelf_next_scratch, next_tick);
    invalidate_carrier_scratch();
    attach_carrier_guidance(
        next.get(), &previous_X, current->guide.get(), &ops);
    if (next->guide != nullptr) {
      out.terminal_guidance =
          std::make_shared<CarrierGuidance>(*next->guide);
      out.terminal_order = next->order;
      out.terminal_h = next->h;
      out.terminal_h_guidance = next->h_guidance;
    }
    current = std::move(next);

    if (is_goal_config(out.configs.back(), out.shelves.back())) {
      out.reached_goal = true;
      return out;
    }
    if (stop_on_event && step >= min_chunk) {
      for (const Op& op : ops)
        if (op.kind == Op::LIFT || op.kind == Op::DROP) return out;
    }
  }
  return out;
}
