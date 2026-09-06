#include "../include/tapf_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <queue>
#include <unordered_set>

#include "../include/search_kernel.hpp"
#include "carrier_guidance.hpp"

namespace
{
  double focal_score(const TAPFNode* node, TAPFFocalTieBreak tie_break)
  {
    switch (tie_break) {
      case TAPFFocalTieBreak::ANTI_WAIT:
        return 8 * node->non_goal_waits + 4 * node->reversals +
               2 * node->distance_increases + node->settled_pushes;
      case TAPFFocalTieBreak::ANTI_ZIGZAG:
        return 8 * node->reversals + 4 * node->distance_increases +
               2 * node->settled_pushes + node->non_goal_waits;
      case TAPFFocalTieBreak::ANTI_PUSH:
        return 8 * node->settled_pushes + 4 * node->reversals +
               2 * node->distance_increases + node->non_goal_waits;
      case TAPFFocalTieBreak::ANTI_ALL:
        return 10 * node->settled_pushes + 6 * node->reversals +
               3 * node->non_goal_waits + 2 * node->distance_increases;
      case TAPFFocalTieBreak::H:
      default:
        return node->h.work_value();
    }
  }

  bool focal_better(const TAPFNode* a, const TAPFNode* b,
                    TAPFFocalTieBreak tie_break)
  {
    if (a->h != b->h) return a->h < b->h;
    const auto a_score = focal_score(a, tie_break);
    const auto b_score = focal_score(b, tie_break);
    if (a_score != b_score) return a_score < b_score;
    if (a->f != b->f) return a->f < b->f;
    if (a->g != b->g) return a->g > b->g;
    return a->depth < b->depth;
  }

  // CLOSED key = (Config, ShelfState) (design 6.1, mapping M2/M14): the
  // shelf hash is splitmix64-derived per (role, id, cell) and XORs to 0
  // for the empty layer, so shelf-free instances keep the exact original
  // ConfigHasher value and duplicate semantics.
  uint64_t splitmix64(uint64_t x)
  {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  }

  uint64_t shelf_layer_hash(const ShelfState& S)
  {
    uint64_t h = 0;
    for (size_t b = 0; b < S.target_pos.size(); ++b)
      h ^= splitmix64((1ULL << 40) ^ (b << 20) ^ (uint64_t)S.target_pos[b]);
    for (const int c : S.anon_occ)
      h ^= splitmix64((2ULL << 40) ^ (uint64_t)c);
    for (size_t i = 0; i < S.kappa.size(); ++i)
      h ^= splitmix64((3ULL << 40) ^ (i << 20) ^
                      (uint64_t)(S.kappa[i] + 2));
    return h;
  }

  PhysConfig physical_state_of(const Config& C, const ShelfState& S)
  {
    PhysConfig out;
    out.robots.reserve(C.size());
    for (const auto* vertex : C) out.robots.push_back(vertex->index);
    out.target_pos = S.target_pos;
    out.anon_occ = S.anon_occ;
    out.kappa = S.kappa;
    return out;
  }

  Config config_of_physical(const TAPFInstance& ins,
                            const PhysConfig& physical)
  {
    Config out;
    out.reserve(physical.robots.size());
    for (const int cell : physical.robots) out.push_back(ins.G.U[cell]);
    return out;
  }

  ShelfState shelf_of_physical(const PhysConfig& physical)
  {
    ShelfState out;
    out.target_pos = physical.target_pos;
    out.anon_occ = physical.anon_occ;
    out.kappa = physical.kappa;
    return out;
  }

  struct SearchKey {
    Config C;
    ShelfState S;
    bool operator==(const SearchKey& o) const
    {
      return C == o.C && S == o.S;
    }
  };

  struct SearchKeyHasher {
    size_t operator()(const SearchKey& k) const
    {
      return (size_t)ConfigHasher()(k.C) ^ (size_t)shelf_layer_hash(k.S);
    }
  };

  // guidance infrastructure lives in carrier_guidance.hpp (shared with
  // the carrier adapters in dd_planner.cpp)
  using namespace carrier_detail;

  constexpr int MACRO_CAP = 64;
  constexpr int MACRO_TARGET_LIMIT = 64;

  double focal_numeric_cost(const PlanCost& cost, TAPFObjective objective)
  {
    if (!cost.is_bounded()) return -1;
    return objective == TAPFObjective::MAKESPAN_THEN_WORK
               ? static_cast<double>(cost.ticks)
               : cost.work_value();
  }

  void add_scaled_work(int64_t& total, int64_t amount)
  {
    if (amount < 0 ||
        total > std::numeric_limits<int64_t>::max() - amount)
      throw std::overflow_error("fixed-point search work overflow");
    total += amount;
  }

  std::optional<PlanCost> reference_joint_cost(
      const SolverWeights& weights, const PhysConfig& state,
      const std::vector<Op>& ops, TAPFObjective objective)
  {
    if (ops.size() != state.robots.size() ||
        state.kappa.size() != state.robots.size())
      return std::nullopt;
    int64_t work = 0;
    for (size_t i = 0; i < ops.size(); ++i) {
      if (ops[i].kind == Op::MOVE) {
        add_scaled_work(
            work, state.kappa[i] == KAPPA_FREE
                      ? weights.beta_scaled
                      : weights.alpha_scaled);
        if (state.kappa[i] == KAPPA_ANON)
          add_scaled_work(work, weights.delta_scaled);
      } else if (ops[i].kind == Op::LIFT ||
                 ops[i].kind == Op::DROP) {
        add_scaled_work(work, weights.gamma_scaled);
      }
    }
    return PlanCost::from_scaled(
        objective == TAPFObjective::MAKESPAN_THEN_WORK ? 1 : 0,
        work);
  }
}  // namespace

// out-of-line: CarrierEngine is an implementation type
struct TAPFPlanner::CarrierEngine {
  // upper-deck wall distance (design_final 6.2/D21): ONE shared
  // dest-keyed cache — the field depends only on (walls, dest), so
  // per-target copies were redundant and would duplicate massively
  // under shared goal pools.
  DDDistCache upper_wall;
  LowerDist lower;
  UpperEpochCache task_br_cache;
  PhysConfig phys;  // scratch physical view of the node in processing

  explicit CarrierEngine(const DDInstance& dd)
      : upper_wall(dd.grid), lower(dd.grid)
  {
  }

  // physical view of a node (oracle coordinates)
  const PhysConfig& phys_view(const TAPFNode* nd)
  {
    phys.robots.resize(nd->C.size());
    for (size_t i = 0; i < nd->C.size(); ++i)
      phys.robots[i] = nd->C[i]->index;
    phys.target_pos = nd->shelf.target_pos;
    phys.anon_occ = nd->shelf.anon_occ;
    phys.kappa = nd->shelf.kappa;
    return phys;
  }
};

TAPFPlanner::~TAPFPlanner()
{
  for (auto* node : deferred_cleanup_nodes) delete node;
  for (auto a : A) delete a;
}

const TAPFReferenceCheckpoint*
TAPFPlanner::find_reference_checkpoint(
    const PhysConfig& state) const
{
  if (search_config.reference_plan == nullptr ||
      !reference_plan_valid)
    return nullptr;
  const uint64_t hash = phys_config_hash(state);
  const auto range = reference_checkpoint_index.equal_range(hash);
  const TAPFReferenceCheckpoint* best = nullptr;
  for (auto it = range.first; it != range.second; ++it) {
    const auto& checkpoint =
        search_config.reference_plan->checkpoints[it->second];
    if (!(checkpoint.state == state)) continue;
    if (best == nullptr ||
        checkpoint.suffix_cost < best->suffix_cost ||
        (checkpoint.suffix_cost == best->suffix_cost &&
         checkpoint.action_index > best->action_index))
      best = &checkpoint;
  }
  return best;
}

void TAPFPlanner::attach_carrier_guidance(
    TAPFNode* nd, const PhysConfig* transition_previous_X,
    const CarrierGuidance* transition_previous_guidance,
    const std::vector<Op>* transition_executed_ops)
{
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
    const auto replayed =
        apply_ops(dd_instance, *transition_previous_X,
                  *transition_executed_ops);
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

  auto guide = std::make_unique<CarrierGuidance>(
      build_task_br_guidance(
          dd_instance, physical, task_br_engine.upper_wall,
          weights.alpha, weights.gamma, weights.delta,
          previous_physical, previous_guidance, transition_ops,
          &task_br_engine.task_br_cache));

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
    const long cache_hits =
        task_br_engine.task_br_cache.hits - cache_hits_before;
    const long cache_misses =
        task_br_engine.task_br_cache.misses - cache_misses_before;
    stats->pair_cache_hits += cache_hits;
    stats->pair_cache_misses += cache_misses;
    stats->upper_epoch_builds += cache_misses;

    if (cache_misses > 0 && nd->guide->upper_epoch != nullptr) {
      const auto& epoch = *nd->guide->upper_epoch;
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
          const int objective_version =
              static_cast<int>(telemetry.objective_version);
          if (objective_version != 0) {
            if (stats->rho_objective_version == 0)
              stats->rho_objective_version = objective_version;
            else if (stats->rho_objective_version !=
                     objective_version)
              throw std::logic_error(
                  "mixed rho objective versions in one search");
          }
          stats->rho_task_assignments +=
              telemetry.task_assignments;
          stats->rho_idle_assignments +=
              telemetry.idle_assignments;
          stats->rho_candidate_time_ms +=
              telemetry.candidate_time_ms;
          stats->rho_matrix_time_ms += telemetry.matrix_time_ms;
          stats->rho_bottleneck_time_ms +=
              telemetry.bottleneck_time_ms;
          stats->rho_secondary_full_time_ms +=
              telemetry.secondary_full_time_ms;
          stats->rho_additive_full_time_ms +=
              telemetry.additive_full_time_ms;
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
      const auto replayed =
          apply_ops(*dd_view, anchor_X, trace[step].ops);
      if (!replayed.has_value() ||
          !(*replayed == trace[step].next_X))
        throw std::logic_error(
            "ensure_guidance_fresh: invalid intermediate transition");
      anchor = build_task_br_guidance(
          *dd_view, *replayed, carrier->upper_wall, weights.alpha,
          weights.gamma, weights.delta, &anchor_X, &anchor,
          &trace[step].ops, &carrier->task_br_cache);
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
  if (ins.shelf_cells.empty()) return S;  // shelf-free: empty layer
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

TAPFPlanner::TAPFPlanner(const TAPFInstance* _ins, const Deadline* _deadline,
                         std::mt19937* _MT, int _verbose, int _sticky_penalty,
                         float _restart_rate, bool _anytime, TAPFStats* _stats,
                         TAPFSearchConfig _search_config)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      sticky_penalty(_sticky_penalty),
      restart_rate(_restart_rate),
      anytime(_anytime),
      search_config(_search_config),
      force_full_assignment(false),
      stats(_stats),
      assignment_stats(TAPFAssignmentStats()),
      N(ins->N),
      V_size(ins->G.size()),
      weights(),
      D(TAPFDistTable(ins)),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr)),
      pibt_cand(N)
{
  if (search_config.stop_policy ==
          TAPFStopPolicy::FIRST_FEASIBLE &&
      search_config.incumbent_init.is_bounded())
    throw std::invalid_argument(
        "FIRST_FEASIBLE requires an unbounded external incumbent");
  if (search_config.stop_policy ==
          TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT &&
      !search_config.incumbent_init.is_bounded())
    throw std::invalid_argument(
        "FIRST_STRICT_IMPROVEMENT requires an external incumbent");
  if (search_config.reference_plan != nullptr) {
    reference_checkpoint_index.reserve(
        search_config.reference_plan->checkpoints.size() * 2);
    for (size_t i = 0;
         i < search_config.reference_plan->checkpoints.size(); ++i) {
      reference_checkpoint_index.emplace(
          search_config.reference_plan->checkpoints[i].state_hash, i);
    }
  }
  if (stats != nullptr) *stats = TAPFStats();
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);
  // Solver-objective weights default to one; optional numeric objective
  // inputs DD_ALPHA..DD_DELTA are read once.  Shelf-free instances never
  // evaluate the carrier cost term.
  carrier_detail::load_solver_weights(weights);
  // carrier layer (M4): the conformance-oracle view and the occupancy
  // scratch exist only when the instance HAS a shelf layer
  if (!ins->shelf_cells.empty()) {
    dd_view = std::make_unique<DDInstance>();
    dd_view->grid.height = ins->G.height;
    dd_view->grid.width = ins->G.width;
    dd_view->grid.wall.assign(ins->G.height * ins->G.width, 0);
    for (int c = 0; c < (int)dd_view->grid.wall.size(); ++c)
      dd_view->grid.wall[c] = ins->G.U[c] == nullptr ? 1 : 0;
    for (const auto* v : ins->starts) dd_view->robots.push_back(v->index);
    dd_view->shelves = ins->shelf_cells;
    dd_view->shelf_storage = ins->shelf_storage;
    dd_view->target_starts = ins->target_starts;
    dd_view->target_goals = ins->target_goals;
    dd_view->target_goal_sets = ins->target_goal_sets;  // T1: eligibility
    dd_view->finalize();
    const size_t n_cells = ins->G.U.size();
    carrier_grounded.assign(n_cells, 0);
    carrier_upper_delta.assign(n_cells, 0);
    carrier = std::make_unique<CarrierEngine>(*dd_view);
  }
  if (search_config.reference_plan != nullptr && dd_view != nullptr) {
    const auto& reference = *search_config.reference_plan;
    std::vector<PhysConfig> states;
    std::vector<PlanCost> steps;
    states.reserve(reference.actions.size() + 1);
    steps.reserve(reference.actions.size());
    states.push_back(initial_phys_config(*dd_view));
    bool valid = true;
    for (const auto& ops : reference.actions) {
      const auto step = reference_joint_cost(
          weights, states.back(), ops, search_config.objective);
      const auto next = apply_ops(*dd_view, states.back(), ops);
      if (!step.has_value() || !next.has_value()) {
        valid = false;
        break;
      }
      steps.push_back(*step);
      states.push_back(*next);
    }
    valid = valid && is_dd_goal(*dd_view, states.back());
    if (valid) {
      std::vector<PlanCost> prefix(states.size());
      std::vector<PlanCost> suffix(states.size());
      for (size_t i = 0; i < steps.size(); ++i)
        prefix[i + 1] = prefix[i] + steps[i];
      for (size_t i = steps.size(); i > 0; --i)
        suffix[i - 1] = steps[i - 1] + suffix[i];
      for (const auto& checkpoint : reference.checkpoints) {
        const size_t index = checkpoint.action_index;
        const bool next_matches =
            index < reference.actions.size()
                ? checkpoint.next_ops == reference.actions[index]
                : checkpoint.next_ops.empty();
        if (index >= states.size() ||
            checkpoint.state_hash != phys_config_hash(states[index]) ||
            !(checkpoint.state == states[index]) ||
            checkpoint.prefix_cost != prefix[index] ||
            checkpoint.suffix_cost != suffix[index] ||
            !next_matches) {
          valid = false;
          break;
        }
      }
    }
    reference_plan_valid = valid;
  }
}

Solution TAPFPlanner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart TAPF search");

  std::vector<TAPFNode*> OPEN;
  std::unordered_map<SearchKey, TAPFNode*, SearchKeyHasher> CLOSED;
  TAPFNode* S_goal = nullptr;
  auto C_new = Config(N, nullptr);
  // scratch lookup key reused across iterations (no per-iteration alloc
  // after warm-up; shelf part is empty for shelf-free instances)
  SearchKey lookup_key;

  auto push_open = [&](TAPFNode* node) {
    if (!node->queued && !node->search_tree.empty()) {
      OPEN.push_back(node);
      node->queued = true;
    }
  };

  auto erase_open = [&](const size_t index) {
    OPEN[index]->queued = false;
    OPEN.erase(OPEN.begin() + index);
  };

  // Pruning bound (M11): the incumbent's g, or an externally supplied
  // upper bound; -1 means no bound.
  auto current_bound = [&]() -> PlanCost {
    if (S_goal != nullptr) return S_goal->g;
    return search_config.incumbent_init;
  };

  auto select_open_index = [&]() -> size_t {
    if (search_config.mode == TAPFSearchMode::DFS ||
        (S_goal == nullptr &&
         !search_config.incumbent_init.is_bounded())) {
      return OPEN.size() - 1;
    }
    // shared FOCAL kernel (search_kernel.hpp) — same semantics as before
    return focal_select_index(
        OPEN, search_config.focal_weight,
        [&](const TAPFNode* n) {
          return focal_numeric_cost(n->f, search_config.objective);
        },
        [&](const TAPFNode* n) {
          const PlanCost b = current_bound();
          return !n->search_tree.empty() &&
                 (!b.is_bounded() || n->f < b);
        },
        [&](const TAPFNode* a, const TAPFNode* b) {
          return focal_better(a, b, search_config.focal_tie_break);
        });
  };

  auto initial_assignment_state = TAPFAssignmentState();
  initial_assignment_state.init(ins->N, ins->tasks.size());
  auto initial_agents = std::vector<int>(N, 0);
  std::iota(initial_agents.begin(), initial_agents.end(), 0);
  auto initial_assignment =
      assign_tapf_tasks_dynamic(*ins, D, ins->starts, initial_assignment_state,
                                initial_agents, true, &assignment_stats);
  if (!initial_assignment.feasible) return Solution();

  auto make_node_h = [&](const Config& config, double work_lb) {
    auto h = PlanCost::legacy(work_lb);
    if (search_config.objective ==
        TAPFObjective::MAKESPAN_THEN_WORK)
      h.ticks = get_time_h_value(config);
    return h;
  };

  auto S_init =
      new TAPFNode(ins->starts, initial_shelf_state(*ins), D, ins,
                   initial_assignment.agent_to_task, initial_assignment_state);
  S_init->h = make_node_h(ins->starts, initial_assignment.cost);
  S_init->f = S_init->g + S_init->h;
  attach_carrier_guidance(S_init);
  deepest_node = S_init;
  deepest_depth = 0;
  push_open(S_init);
  CLOSED[SearchKey{S_init->C, S_init->shelf}] = S_init;
  if (stats != nullptr) {
    stats->hl_nodes_created = 1;
    stats->open_max_size = 1;
  }

  auto accept_goal = [&](TAPFNode* candidate) {
    const bool improves =
        candidate != nullptr &&
        (S_goal == nullptr || candidate->g < S_goal->g) &&
        (!search_config.incumbent_init.is_bounded() ||
         candidate->g < search_config.incumbent_init);
    if (!improves) return false;
    if (stats != nullptr) {
      ++stats->incumbent_updates;
      if (stats->first_solution_ticks < 0) {
        stats->first_solution_cost =
            (unsigned)std::lround(candidate->g.work_value());
        stats->first_solution_g = candidate->g.work_value();
        stats->first_solution_ticks = candidate->g.ticks;
        stats->first_solution_work_scaled = candidate->g.work;
        stats->first_solution_work = candidate->g.work_value();
        stats->first_solution_time_ms = elapsed_ms(deadline);
      }
      ++stats->anytime_cost_updates;
    }
    S_goal = candidate;
    info(1, verbose, "elapsed:", elapsed_ms(deadline),
         "ms\tfound TAPF solution\tcost:", S_goal->g);
    return true;
  };

  auto try_reference_suffix = [&](TAPFNode* start) {
    if (search_config.stop_policy !=
            TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT ||
        search_config.reference_plan == nullptr ||
        dd_view == nullptr || !reference_plan_valid)
      return false;
    const PhysConfig start_state =
        physical_state_of(start->C, start->shelf);
    const auto* checkpoint =
        find_reference_checkpoint(start_state);
    if (checkpoint == nullptr) return false;
    if (stats != nullptr) ++stats->reference_checkpoint_hits;
    const auto& reference = *search_config.reference_plan;
    if (checkpoint->action_index >= reference.actions.size())
      return false;
    const PlanCost bound = current_bound();
    if (!bound.is_bounded() ||
        !(start->g + checkpoint->suffix_cost < bound))
      return false;
    if (stats != nullptr) ++stats->reference_suffix_attempts;

    std::vector<TransitionStep> trace;
    trace.reserve(
        reference.actions.size() - checkpoint->action_index);
    PhysConfig state = start_state;
    PlanCost suffix_cost;
    for (size_t action = checkpoint->action_index;
         action < reference.actions.size(); ++action) {
      if (is_expired(deadline)) return false;
      const auto step_cost = reference_joint_cost(
          weights, state, reference.actions[action],
          search_config.objective);
      if (!step_cost.has_value()) return false;
      const auto next =
          apply_ops(*dd_view, state, reference.actions[action]);
      if (!next.has_value()) return false;
      suffix_cost += *step_cost;
      trace.push_back(TransitionStep{
          state, reference.actions[action], *next});
      state = *next;
    }
    if (trace.empty() ||
        suffix_cost != checkpoint->suffix_cost ||
        !(start->g + suffix_cost < bound) ||
        !is_dd_goal(*dd_view, state))
      return false;

    Config terminal_config = config_of_physical(*ins, state);
    ShelfState terminal_shelf = shelf_of_physical(state);
    if (!is_goal_config(terminal_config, terminal_shelf))
      return false;
    SearchKey terminal_key{terminal_config, terminal_shelf};
    TAPFNode* terminal = nullptr;
    const auto existing = CLOSED.find(terminal_key);
    if (existing == CLOSED.end()) {
      terminal = new TAPFNode(
          terminal_config, terminal_shelf, D, ins,
          start->assignment, start->assignment_state, start);
      terminal->g = start->g + suffix_cost;
      terminal->h = PlanCost();
      terminal->f = terminal->g;
      terminal->incoming_edge = register_outgoing_edge(
          start, terminal, suffix_cost, trace);
      CLOSED.emplace(std::move(terminal_key), terminal);
      if (stats != nullptr) ++stats->hl_nodes_created;
    } else {
      terminal = existing->second;
      register_outgoing_edge(
          start, terminal, suffix_cost, trace);
      rewrite(start, S_goal, OPEN);
    }
    if (!accept_goal(terminal)) return false;
    if (stats != nullptr) ++stats->reference_suffix_accepted;
    return true;
  };

  const auto initial_lower_bound = S_init->h;
  // The caller owns the one search/finalization split.  Do not subtract a
  // second cleanup reserve here: doing so turned a 10 s controller budget
  // into an 8.5 s pass deadline and then a 7.65 s bounded-search cutoff.
  while (!OPEN.empty() && !is_expired(deadline)) {
    if (stats != nullptr) {
      ++stats->hl_loop_iterations;
      stats->open_max_size = std::max<int>(stats->open_max_size, OPEN.size());
    }
    const auto open_index = select_open_index();
    auto S = OPEN[open_index];

    if (S_goal != nullptr && S_goal->g <= initial_lower_bound) {
      break;
    }

    if (S->search_tree.empty()) {
      erase_open(open_index);
      continue;
    }

    if (S->guidance_stale) {
      ensure_guidance_fresh(S);
      if (stats != nullptr) ++stats->rewire_guidance_rebuilds;
    }

    {
      const PlanCost bound = current_bound();
      if (bound.is_bounded() && S->f >= bound) {
        if (stats != nullptr) ++stats->f_pruned;
        erase_open(open_index);
        continue;
      }
    }

    if (try_reference_suffix(S)) break;

    if (is_goal_config(S->C, S->shelf)) {
      accept_goal(S);
      if (search_config.stop_policy != TAPFStopPolicy::ANYTIME &&
          S_goal != nullptr)
        break;
      if (!anytime || deadline == nullptr ||
          (S_goal != nullptr && S_goal->g <= initial_lower_bound)) {
        break;
      }
      continue;
    }

    // macro successor probe (design 7.1/D13/D14, M10): on the node's
    // FIRST expansion, before any incumbent, within the carrier scale
    // regime, roll the unconstrained generator to the next lift/drop
    // event and push the terminal state as an extra DFS-top successor.
    // The node's constraint tree is untouched (completeness free);
    // structurally unreachable on shelf-free instances (h_guidance == 0).
    if (search_config.macro_enabled && S_goal == nullptr &&
        !search_config.incumbent_init.is_bounded() &&
        !S->macro_tried && S->h_guidance > 0 && ins->tasks.empty() &&
        (int)ins->target_starts.size() <= MACRO_TARGET_LIMIT) {
      S->macro_tried = true;
      auto r = carrier_rollout(S->C, S->shelf, MACRO_CAP, 0,
                               /*stop_on_event=*/true, S,
                               /*escape_preferred_self_loop=*/false);
      // rollout probes died: their addresses may be recycled by the
      // nodes created below — stale address-keyed scratches are poison
      invalidate_carrier_scratch();
      if (r.ops.size() >= 2) {
        lookup_key.C = r.configs.back();
        lookup_key.S = r.shelves.back();
        std::vector<TransitionStep> trace;
        trace.reserve(r.ops.size());
        for (size_t step = 0; step < r.ops.size(); ++step)
          trace.push_back(TransitionStep{
              physical_state_of(r.configs[step], r.shelves[step]),
              r.ops[step],
              physical_state_of(r.configs[step + 1],
                                r.shelves[step + 1])});
        auto macro_iter = CLOSED.find(lookup_key);
        TAPFNode* S_macro = nullptr;
        if (macro_iter == CLOSED.end()) {
          S_macro = new TAPFNode(
              r.configs.back(), r.shelves.back(), D, ins,
              std::vector<int>(N, -1), S->assignment_state, S);
          S_macro->g = S->g + r.cost;
          S_macro->h = r.terminal_h;
          S_macro->f = S_macro->g + S_macro->h;
          S_macro->h_guidance = r.terminal_h_guidance;
          if (r.terminal_guidance != nullptr)
            S_macro->guide = std::make_unique<CarrierGuidance>(
                *r.terminal_guidance);
          S_macro->order = r.terminal_order;
          S_macro->constraint_order = S_macro->order;
          S_macro->incoming_edge =
              register_outgoing_edge(S, S_macro, r.cost, trace);
          CLOSED[SearchKey{S_macro->C, S_macro->shelf}] = S_macro;
          push_open(S_macro);
          if (stats != nullptr) {
            ++stats->hl_nodes_created;
            ++stats->macro_successors;
            if (r.shelf_moved)
              ++stats->macro_shelf_motion_successors;
            else
              ++stats->macro_robot_only_successors;
            // macro_after_first stays 0 by construction: this block is
            // gated on S_goal == nullptr.
          }
        } else {
          S_macro = macro_iter->second;
          register_outgoing_edge(S, S_macro, r.cost, trace);
          rewrite(S, S_goal, OPEN);
          if (!S_macro->queued && !S_macro->search_tree.empty())
            push_open(S_macro);
          if (stats != nullptr) ++stats->hl_duplicate_configs;
        }
        continue;  // S stays queued; DFS tries the macro child first
      }
    }

    auto M = S->search_tree.front();
    S->search_tree.pop();
    if (stats != nullptr) ++stats->constraints_popped;
    if (M->depth < N) {
      auto i = S->constraint_order[M->depth];
      auto ops_cand = std::vector<OpCand>();
      build_op_candidates(S, i, ops_cand);
      lacam_expand_constraint_vec<TAPFConstraint>(M, i, ops_cand,
                                                  S->search_tree);
      if (stats != nullptr) stats->constraints_generated += ops_cand.size();
    }

    if (!get_new_config(S, M)) {
      delete M;
      if (stats != nullptr) ++stats->constraint_failures;
      continue;
    }
    const auto M_depth = M->depth;  // for the reject-counter split below
    delete M;

    for (auto a : A) C_new[a->id] = a->v_next;

    // carrier layer successor (M4): assemble the joint op and let the
    // conformance oracle arbitrate; fills shelf_next_scratch.  Trivially
    // true (layer copied) on shelf-free instances.
    if (!apply_carrier_effects(S)) {
      if (stats != nullptr) {
        if (M_depth == N)
          ++stats->carrier_g1_rejects;  // exhaustive-tree combos (G1)
        else
          ++stats->carrier_validator_rejects;
      }
      continue;
    }

    std::vector<Op> edge_ops;
    if (!S->shelf.kappa.empty()) {
      edge_ops = ops_scratch;
    } else {
      edge_ops.assign(N, Op::make_wait());
      for (const auto* agent : A)
        if (agent->v_next != agent->v_now)
          edge_ops[agent->id] =
              Op::make_move(agent->v_next->index);
    }
    const std::vector<TransitionStep> one_step_trace = {
        TransitionStep{
            physical_state_of(S->C, S->shelf), edge_ops,
            physical_state_of(C_new, shelf_next_scratch)}};

    lookup_key.C = C_new;
    lookup_key.S = shelf_next_scratch;
    auto iter = CLOSED.find(lookup_key);
    if (iter != CLOSED.end()) {
      auto S_known = iter->second;
      register_outgoing_edge(
          S, S_known, get_edge_cost(S, S_known), one_step_trace);
      rewrite(S, S_goal, OPEN);
      auto S_insert = S_known;
      if (MT != nullptr && get_random_float(MT) < restart_rate) {
        S_insert = S_init;
      }
      {
        const PlanCost b = current_bound();
        if ((!b.is_bounded() || S_insert->f < b) &&
            !S_insert->queued &&
            !S_insert->search_tree.empty()) {
          push_open(S_insert);
          if (stats != nullptr) ++stats->hl_reinsertions;
        }
      }
      if (stats != nullptr) ++stats->hl_duplicate_configs;
      continue;
    }

    auto changed_agents = std::vector<int>();
    changed_agents.reserve(N);
    for (int i = 0; i < N; ++i) {
      if (C_new[i] != S->C[i]) changed_agents.push_back(i);
    }

    auto assignment_state = S->assignment_state;
    auto assignment =
        assign_tapf_tasks_dynamic(*ins, D, C_new, assignment_state,
                                  changed_agents, force_full_assignment, &assignment_stats);
    if (!assignment.feasible) continue;
    if (stats != nullptr) {
      for (size_t i = 0; i < assignment.agent_to_task.size(); ++i) {
        if (assignment.agent_to_task[i] != S->assignment[i]) {
          ++stats->assignment_changes;
          break;
        }
      }
    }

    if (stats != nullptr) {
      bool shelf_motion = false;
      bool manipulation = false;
      for (size_t i = 0; i < ops_scratch.size(); ++i) {
        const auto kind = ops_scratch[i].kind;
        shelf_motion |=
            kind == Op::MOVE && !S->shelf.kappa.empty() &&
            S->shelf.kappa[i] != KAPPA_FREE;
        manipulation |= kind == Op::LIFT || kind == Op::DROP;
      }
      if (shelf_motion)
        ++stats->shelf_motion_successors;
      else if (manipulation)
        ++stats->manipulation_successors;
      else
        ++stats->robot_only_successors;
    }

    auto S_new = new TAPFNode(C_new, shelf_next_scratch, D, ins,
                              assignment.agent_to_task, assignment_state, S);
    const PlanCost edge_cost = get_edge_cost(S, S_new);
    S_new->g = S->g + edge_cost;
    S_new->h = make_node_h(C_new, assignment.cost);
    S_new->f = S_new->g + S_new->h;
    S_new->incoming_edge = register_outgoing_edge(
        S, S_new, edge_cost, one_step_trace);
    attach_carrier_guidance(
        S_new, &one_step_trace.front().previous_X, S->guide.get(),
        &edge_ops);
    CLOSED[SearchKey{S_new->C, S_new->shelf}] = S_new;
    if (deepest_node == nullptr || S_new->depth > deepest_depth) {
      deepest_node = S_new;
      deepest_depth = S_new->depth;
    }
    if (!S_new->shelf.kappa.empty()) {
      long done = 0;
      for (size_t b = 0; b < S_new->shelf.target_pos.size(); ++b) {
        bool carried = false;
        for (const int k : S_new->shelf.kappa) carried |= (k == (int)b);
        if (!carried &&
            std::binary_search(ins->target_goal_sets[b].begin(),
                               ins->target_goal_sets[b].end(),
                               S_new->shelf.target_pos[b]))
          ++done;
      }
      best_targets_done = std::max(best_targets_done, done);
    }
    {
      const PlanCost b = current_bound();
      if (!b.is_bounded() || S_new->f < b) {
        push_open(S_new);
      }
    }
    if (stats != nullptr) ++stats->hl_nodes_created;
  }

  auto solution = Solution();
  auto solution_nodes = std::vector<TAPFNode*>();
  solution_shelves.clear();
  if (S_goal != nullptr) {
    auto S = S_goal;
    while (S != nullptr) {
      solution_nodes.push_back(S);
      solution.push_back(S->C);
      solution_shelves.push_back(S->shelf);
      // Expand the selected immutable incoming trace.  The endpoint is
      // already present above, so append intermediate next states in
      // reverse order; the final vector reverse restores time order.
      if (S->parent != nullptr && S->incoming_edge != nullptr) {
        const auto& trace = S->incoming_edge->transition_trace;
        if (trace.size() > 1)
          for (size_t k = trace.size() - 1; k-- > 0;) {
            solution.push_back(
                config_of_physical(*ins, trace[k].next_X));
            solution_shelves.push_back(
                shelf_of_physical(trace[k].next_X));
          }
      }
      S = S->parent;
    }
    std::reverse(solution.begin(), solution.end());
    std::reverse(solution_nodes.begin(), solution_nodes.end());
    std::reverse(solution_shelves.begin(), solution_shelves.end());
  }

  if (stats != nullptr) {
    stats->hl_nodes_explored = CLOSED.size();
    stats->timed_out = solution.empty() && is_expired(deadline);
    stats->assignment_calls = assignment_stats.calls;
    stats->assignment_time_ms = assignment_stats.time_ms;
    if (!solution.empty()) {
      stats->solution_cost =
          (unsigned)std::lround(S_goal->g.work_value());
      stats->solution_ticks = S_goal->g.ticks;
      stats->solution_work_scaled = S_goal->g.work;
      stats->solution_work = S_goal->g.work_value();
      PlanCost parent_edge_cost;
      for (size_t step = 1; step < solution_nodes.size(); ++step) {
        parent_edge_cost +=
            get_edge_cost(solution_nodes[step - 1], solution_nodes[step]);
      }
      stats->solution_parent_edge_cost =
          (unsigned)std::lround(parent_edge_cost.work_value());
      stats->solution_depth = solution.size() - 1;
      for (size_t step = 1; step < solution_nodes.size(); ++step) {
        auto changed = false;
        const auto& prev = solution_nodes[step - 1]->assignment;
        const auto& curr = solution_nodes[step]->assignment;
        for (size_t i = 0; i < curr.size(); ++i) {
          if (curr[i] != prev[i]) {
            changed = true;
            ++stats->final_agent_assignment_changes;
          }
        }
        if (changed) ++stats->final_assignment_changes;
      }
    }
  }

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       solution.empty() ? (OPEN.empty() ? "no TAPF solution" : "failed")
                        : "TAPF solution found",
       "\texplored:", CLOSED.size());

  if (is_expired(deadline)) {
    for (auto p : CLOSED) p.second->discard_search_tree();
  }

  // best-effort chain to the deepest node (carrier debug aid, M16)
  best_effort_solution.clear();
  best_effort_shelves.clear();
  best_effort_tau.clear();
  if (S_goal == nullptr && deepest_node != nullptr &&
      !ins->shelf_cells.empty()) {
    if (deepest_node->guide != nullptr &&
        deepest_node->guide->upper_epoch != nullptr)
      best_effort_tau =
          deepest_node->guide->upper_epoch->tau_guide;
    for (const TAPFNode* S2 = deepest_node; S2 != nullptr; S2 = S2->parent) {
      best_effort_solution.push_back(S2->C);
      best_effort_shelves.push_back(S2->shelf);
      if (S2->parent != nullptr && S2->incoming_edge != nullptr) {
        const auto& trace = S2->incoming_edge->transition_trace;
        if (trace.size() > 1)
          for (size_t k = trace.size() - 1; k-- > 0;) {
            best_effort_solution.push_back(
                config_of_physical(*ins, trace[k].next_X));
            best_effort_shelves.push_back(
                shelf_of_physical(trace[k].next_X));
          }
      }
    }
    std::reverse(best_effort_solution.begin(), best_effort_solution.end());
    std::reverse(best_effort_shelves.begin(), best_effort_shelves.end());
  }

  if (search_config.defer_cleanup) {
    deferred_cleanup_nodes.reserve(deferred_cleanup_nodes.size() +
                                   CLOSED.size());
    for (auto p : CLOSED) deferred_cleanup_nodes.push_back(p.second);
  } else {
    for (auto p : CLOSED) delete p.second;
  }
  deepest_node = nullptr;  // owned by CLOSED; gone now

  return solution;
}

void TAPFPlanner::rewrite(TAPFNode* from, TAPFNode* goal,
                          std::vector<TAPFNode*>& OPEN)
{
  auto Q = std::queue<TAPFNode*>({from});
  while (!Q.empty()) {
    auto node_from = Q.front();
    Q.pop();
    for (const auto& edge : node_from->outgoing_edges) {
      if (edge == nullptr || edge->to == nullptr) continue;
      auto node_to = edge->to;
      const auto g = node_from->g + edge->physical_cost;
      if (g < node_to->g) {
        node_to->parent = node_from;
        node_to->incoming_edge = edge;
        node_to->g = g;
        node_to->f = node_to->g + node_to->h;
        node_to->guidance_stale = true;
        Q.push(node_to);
        if (stats != nullptr) {
          ++stats->anytime_cost_updates;
          ++stats->g_relaxed;
        }
        const PlanCost bound =
            goal != nullptr ? goal->g : search_config.incumbent_init;
        if ((!bound.is_bounded() || node_to->f < bound) &&
            !node_to->queued && !node_to->search_tree.empty()) {
          OPEN.push_back(node_to);
          node_to->queued = true;
        }
      }
    }
  }
}

SearchEdgeHandle TAPFPlanner::register_outgoing_edge(
    TAPFNode* from, TAPFNode* to, PlanCost physical_cost,
    const std::vector<TransitionStep>& transition_trace)
{
  if (from == nullptr || to == nullptr)
    throw std::invalid_argument(
        "register_outgoing_edge: null endpoint");
  if (!transition_trace.empty()) {
    const PhysConfig from_state =
        physical_state_of(from->C, from->shelf);
    const PhysConfig to_state =
        physical_state_of(to->C, to->shelf);
    if (!(transition_trace.front().previous_X == from_state) ||
        !(transition_trace.back().next_X == to_state))
      throw std::logic_error(
          "register_outgoing_edge: trace endpoint mismatch");
    for (size_t step = 0; step < transition_trace.size(); ++step) {
      if (step > 0 &&
          !(transition_trace[step - 1].next_X ==
            transition_trace[step].previous_X))
        throw std::logic_error(
            "register_outgoing_edge: discontinuous trace");
      if (dd_view != nullptr) {
        const auto replayed = apply_ops(
            *dd_view, transition_trace[step].previous_X,
            transition_trace[step].ops);
        if (!replayed.has_value() ||
            !(*replayed == transition_trace[step].next_X))
          throw std::logic_error(
              "register_outgoing_edge: unreplayable transition");
      }
    }
  }
  for (const auto& edge : from->outgoing_edges)
    if (edge != nullptr && edge->to == to &&
        edge->physical_cost == physical_cost &&
        edge->transition_trace == transition_trace)
      return edge;

  auto edge = std::make_shared<SearchEdge>();
  edge->to = to;
  edge->physical_cost = physical_cost;
  edge->transition_trace = transition_trace;
  SearchEdgeHandle handle = edge;
  from->outgoing_edges.push_back(handle);
  std::stable_sort(
      from->outgoing_edges.begin(), from->outgoing_edges.end(),
      [](const SearchEdgeHandle& a, const SearchEdgeHandle& b) {
        if (a->to != b->to)
          return std::less<const TAPFNode*>()(a->to, b->to);
        if (a->physical_cost != b->physical_cost)
          return a->physical_cost < b->physical_cost;
        return a->transition_trace.size() <
               b->transition_trace.size();
      });
  return handle;
}

PlanCost TAPFPlanner::get_edge_cost(const TAPFNode* from,
                                    const TAPFNode* to) const
{
  if (to != nullptr && to->parent == from &&
      to->incoming_edge != nullptr)
    return to->incoming_edge->physical_cost;
  int64_t work = 0;
  for (size_t i = 0; i < ins->N; ++i) {
    const auto task = to->assignment[i];
    if (task < 0) continue;  // carrier agent: physical term below
    const auto goal = ins->tasks[task];
    if (from->C[i] != goal || to->C[i] != goal)
      add_scaled_work(work, PlanCost::WORK_SCALE);
  }
  // physical carrier term (design 2.3, mapping M5): loaded/free moves,
  // lift/drop, anonymous-carry moves.  kappa is empty on shelf-free
  // instances, so this loop is structurally skipped there.
  for (size_t i = 0; i < from->shelf.kappa.size(); ++i) {
    const int k_from = from->shelf.kappa[i];
    const int k_to = to->shelf.kappa[i];
    if (from->C[i] != to->C[i]) {  // MOVE (kappa preserved by moves)
      if (k_from == KAPPA_FREE) {
        if (to->assignment[i] < 0)
          add_scaled_work(work, weights.beta_scaled);
      } else {
        add_scaled_work(work, weights.alpha_scaled);
        if (k_from == KAPPA_ANON)
          add_scaled_work(work, weights.delta_scaled);
      }
    } else if (k_from != k_to) {  // LIFT or DROP (same cell)
      add_scaled_work(work, weights.gamma_scaled);
    }
  }
  const int64_t ticks =
      search_config.objective == TAPFObjective::MAKESPAN_THEN_WORK ? 1 : 0;
  return PlanCost::from_scaled(ticks, work);
}

double TAPFPlanner::get_h_value(const Config& C)
{
  auto cost = 0.0;
  for (size_t i = 0; i < ins->N; ++i) {
    if (ins->allowed[i].empty()) continue;  // carrier agent: no task h
    auto best = D.K;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (!ins->allowed[i][j]) continue;
      best = std::min(best, D.get(j, C[i]));
    }
    cost += best < D.K ? best : D.K;
  }
  return cost;
}

int64_t TAPFPlanner::get_time_h_value(const Config& C)
{
  int64_t bound = 0;
  for (size_t i = 0; i < ins->N; ++i) {
    if (ins->allowed[i].empty()) continue;
    auto best = D.K;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (!ins->allowed[i][j]) continue;
      best = std::min(best, D.get(j, C[i]));
    }
    if (best < D.K) bound = std::max<int64_t>(bound, best);
  }
  return bound;
}

bool TAPFPlanner::is_goal_config(const Config& C, const ShelfState& S) const
{
  // agent-task part: identical to the original for every agent that HAS
  // an allowed task (is_valid guarantees that on shelf-free instances);
  // carrier agents (empty allowed row) are terminally unconstrained.
  auto used = std::vector<bool>(ins->tasks.size(), false);
  for (size_t i = 0; i < ins->N; ++i) {
    if (ins->allowed[i].empty()) continue;
    auto matched = false;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (used[j] || !ins->allowed[i][j] || C[i] != ins->tasks[j]) continue;
      used[j] = true;
      matched = true;
      break;
    }
    if (!matched) return false;
  }
  // carrier part (design_final 2.2/Prop 3, D10): every target grounded on
  // an ELIGIBLE goal cell (set membership; singleton == old equality)
  for (size_t b = 0; b < ins->target_goal_sets.size(); ++b) {
    const auto& set = ins->target_goal_sets[b];
    if (!std::binary_search(set.begin(), set.end(), S.target_pos[b]))
      return false;
  }
  for (const int k : S.kappa)
    if (k >= 0) return false;  // a carried target is not grounded
  return true;
}

void TAPFPlanner::build_op_candidates(TAPFNode* S, int i,
                                      std::vector<OpCand>& out)
{
  // operator candidates (design 5.2, M3): every vertex candidate is a
  // MOVE (or WAIT at the own cell); LIFT/DROP append — their guards are
  // structurally false on shelf-free instances, so the candidate set,
  // order and RNG consumption stay exactly the original there.
  auto C = S->C[i]->neighbor;
  C.push_back(S->C[i]);
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  out.clear();
  out.reserve(C.size() + 2);
  for (auto u : C)
    out.push_back(OpCand{u, (uint8_t)(u == S->C[i] ? Op::WAIT : Op::MOVE)});
  if (!S->shelf.kappa.empty()) {
    refresh_carrier_scratch(S);
    const int cell = S->C[i]->index;
    if (S->shelf.kappa[i] == KAPPA_FREE) {
      if (carrier_grounded[cell] != 0)
        out.push_back(OpCand{S->C[i], (uint8_t)Op::LIFT});
    } else {
      out.push_back(OpCand{S->C[i], (uint8_t)Op::DROP});
    }
  }

  const auto* checkpoint =
      find_reference_checkpoint(physical_state_of(S->C, S->shelf));
  if (checkpoint != nullptr &&
      checkpoint->next_ops.size() == S->C.size()) {
    const Op& wanted = checkpoint->next_ops[i];
    const int wanted_cell =
        wanted.kind == Op::MOVE ? wanted.to : S->C[i]->index;
    const auto it = std::find_if(
        out.begin(), out.end(), [&](const OpCand& candidate) {
          return candidate.kind == wanted.kind &&
                 candidate.v->index == wanted_cell;
        });
    if (it != out.end()) {
      std::rotate(out.begin(), it, std::next(it));
      if (stats != nullptr) ++stats->reference_action_hints;
    }
  }
}

bool TAPFPlanner::get_new_config(TAPFNode* S, TAPFConstraint* M)
{
  refresh_carrier_scratch(S);
  for (auto a : A) {
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    a->v_now = S->C[a->id];
    occupied_now[a->v_now->id] = a;
  }
  // reset carried-shelf reservations of the previous generation
  for (const int c : carrier_upper_touched) carrier_upper_delta[c] = 0;
  carrier_upper_touched.clear();

  // G1 (design 4.2, M4): with a shelf layer and a FULLY constrained joint
  // op, the conformance oracle is the sole arbiter — inline generator
  // checks must not stand in the way.  Shelf-free instances keep the
  // original inline checks as their (complete) arbiter.
  const bool oracle_decides =
      !S->shelf.kappa.empty() && M->depth == (int)ins->N;

  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];
    const auto l = M->where[k]->id;
    const auto kind =
        k < (int)M->ops.size() ? M->ops[k] : (uint8_t)Op::MOVE;

    if (!oracle_decides) {
      if (occupied_next[l] != nullptr) return false;
      auto l_pre = S->C[i]->id;
      if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
          occupied_next[l_pre]->id == occupied_now[l]->id)
        return false;
      if (!S->shelf.kappa.empty() &&
          !forced_op_feasible(S, i, M->where[k], kind))
        return false;
    }

    A[i]->v_next = M->where[k];
    A[i]->op_kind = kind;
    occupied_next[l] = A[i];
    // a carried shelf occupies the destination upper cell at t+1
    if (!S->shelf.kappa.empty() && S->shelf.kappa[i] != KAPPA_FREE &&
        kind != Op::LIFT)
      carrier_upper_add(M->where[k]->index);
  }

  for (auto k : S->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a, S->assignment)) return false;
  }
  return true;
}

bool TAPFPlanner::funcPIBT(Agent* ai, const std::vector<int>& assignment)
{
  if (stats != nullptr) ++stats->pibt_calls;
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();
  const auto task_id = assignment[i];
  // carrier role of this agent (KAPPA_FREE on shelf-free instances)
  const int kappa_i = carrier_scratch_node != nullptr &&
                              !carrier_scratch_node->shelf.kappa.empty()
                          ? carrier_scratch_node->shelf.kappa[i]
                          : KAPPA_FREE;
  const bool loaded = kappa_i != KAPPA_FREE;
  const CarrierGuidance* guide =
      carrier_scratch_node != nullptr ? carrier_scratch_node->guide.get()
                                      : nullptr;

  Agent* swap_agent = nullptr;
  // preference-ordered op candidates (per-agent buffer: recursion-safe)
  auto& cand = pibt_cand[i];
  cand.clear();

  if (task_id >= 0) {
    // ---- ORIGINAL task-agent candidate construction (unchanged) ----
    auto neighbor_agents = std::array<Agent*, 4>();
    auto neighbor_agent_count = 0u;

    for (auto u : ai->v_now->neighbor) {
      auto aj = occupied_now[u->id];
      if (aj != nullptr) neighbor_agents[neighbor_agent_count++] = aj;
    }

    for (size_t k = 0; k < K; ++k) {
      auto u = ai->v_now->neighbor[k];
      C_next[i][k] = u;
      if (MT != nullptr) tie_breakers[u->id] = get_random_float(MT);
    }
    C_next[i][K] = ai->v_now;
    if (MT != nullptr) tie_breakers[ai->v_now->id] = get_random_float(MT);

    auto get_hindrance = [&](Vertex* u) {
      auto count = 0u;
      for (auto n = 0u; n < neighbor_agent_count; ++n) {
        auto aj = neighbor_agents[n];
        if (aj->v_now == u) continue;
        const auto neighbor_task = assignment[aj->id];
        if (neighbor_task < 0) continue;  // carrier agent: no task field
        if (D.get(neighbor_task, u) < D.get(neighbor_task, aj->v_now)) {
          ++count;
        }
      }
      return count;
    };

    std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
              [&](Vertex* const v, Vertex* const u) {
                const auto dv = D.get(task_id, v);
                const auto du = D.get(task_id, u);
                if (dv != du) return dv < du;
                const auto hv = get_hindrance(v);
                const auto hu = get_hindrance(u);
                if (hv != hu) {
                  return hv < hu;
                }
                return tie_breakers[v->id] < tie_breakers[u->id];
              });

    swap_agent = swap_possible_and_required(ai, assignment);
    if (swap_agent != nullptr) {
      if (stats != nullptr) ++stats->swap_applied;
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
    }

    for (size_t k = 0; k < K + 1; ++k)
      cand.push_back({C_next[i][k], (uint8_t)(C_next[i][k] == ai->v_now
                                                  ? Op::WAIT
                                                  : Op::MOVE)});
  } else {
    // ---- carrier roles (design 5.4 candidate table, M7) ----
    auto& eng = *carrier;
    const int q = ai->v_now->index;

    auto append_candidate = [&](Vertex* vertex, uint8_t kind) {
      const auto duplicate =
          std::find_if(cand.begin(), cand.end(), [&](const auto& item) {
            return item.first == vertex && item.second == kind;
          });
      if (duplicate == cand.end()) cand.push_back({vertex, kind});
    };
    auto append_exact_move = [&](int cell) {
      for (auto* vertex : ai->v_now->neighbor)
        if (vertex->index == cell) {
          append_candidate(vertex, (uint8_t)Op::MOVE);
          return;
        }
    };
    auto append_all_moves = [&](auto&& score) {
      auto cells = std::vector<Vertex*>(ai->v_now->neighbor);
      std::stable_sort(cells.begin(), cells.end(),
                       [&](Vertex* a, Vertex* b) {
                         const auto score_a = score(a->index);
                         const auto score_b = score(b->index);
                         return score_a != score_b
                                    ? score_a < score_b
                                    : a->index < b->index;
                       });
      for (auto* vertex : cells)
        append_candidate(vertex, (uint8_t)Op::MOVE);
    };

    if (loaded) {
      const Custody* custody = nullptr;
      if (guide != nullptr &&
          i < (int)guide->custody_by_robot.size() &&
          guide->custody_by_robot[i].has_value())
        custody = &*guide->custody_by_robot[i];
      if (custody != nullptr && custody->from == q) {
        const bool at_endpoint =
            q == custody_endpoint(*custody) &&
            custody->route_status == RouteStatus::ARRIVED;
        if (at_endpoint && dd_view->can_store_shelf(q))
          append_candidate(ai->v_now, (uint8_t)Op::DROP);
        const TimedRouteHint* timed_hint = nullptr;
        if (guide != nullptr &&
            i < (int)guide->timed_transport.by_robot.size() &&
            guide->timed_transport.by_robot[i].has_value()) {
          const auto& candidate =
              *guide->timed_transport.by_robot[i];
          if ((candidate.status == RouteStatus::OK ||
               candidate.status == RouteStatus::PREFIX) &&
              candidate.endpoint == custody_endpoint(*custody) &&
              candidate.cells.size() >= 2 &&
              candidate.cells.front() == q)
            timed_hint = &candidate;
        }
        if (!at_endpoint && timed_hint != nullptr) {
          if (timed_hint->cells[1] == q)
            append_candidate(ai->v_now, (uint8_t)Op::WAIT);
          else
            append_exact_move(timed_hint->cells[1]);
        }
        if (!at_endpoint && custody->preferred_leg.has_value() &&
            custody->preferred_leg->from == q)
          append_exact_move(custody->preferred_leg->to);
        append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        if (!at_endpoint && dd_view->can_store_shelf(q))
          append_candidate(ai->v_now, (uint8_t)Op::DROP);
        append_all_moves([&](int cell) {
          return custody->preferred_leg.has_value() &&
                         cell == custody->preferred_leg->to
                     ? std::make_pair(0, cell)
                     : std::make_pair(1, cell);
        });
      } else {
        // Loaded-but-unbound is normally released in place.  A carrier on a
        // transit cell should already have transition-anchored recovery
        // custody.  If recovery cannot currently choose a legal storage
        // endpoint, prefer WAIT instead of greedily retargeting the shelf from
        // one aisle cell at a time.  Other legal moves remain in the operator
        // candidate set for completeness.
        const bool can_drop_here = dd_view->can_store_shelf(q);
        if (can_drop_here)
          append_candidate(ai->v_now, (uint8_t)Op::DROP);
        const bool recursively_displaced =
            occupied_next[ai->v_now->id] != nullptr &&
            occupied_next[ai->v_now->id] != ai;
        if (!can_drop_here) {
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
          append_all_moves([&](int cell) {
            return std::make_pair(
                dd_view->can_store_shelf(cell) ? 0 : 1, cell);
          });
        } else {
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        }
        if (can_drop_here && !recursively_displaced)
          append_all_moves(
              [](int cell) { return std::make_pair(0, cell); });
      }
    } else {
      const TaskId* assigned = nullptr;
      if (guide != nullptr && i < (int)guide->rho_task_id.size() &&
          guide->rho_task_id[i].has_value())
        assigned = &*guide->rho_task_id[i];
      if (assigned != nullptr) {
        const bool preparing =
            guide != nullptr &&
            i < (int)guide->rho_mode.size() &&
            guide->rho_mode[i] == DispatchMode::PREPARE;
        const bool exact_shelf_here =
            assigned->from == q &&
            ((assigned->shelf.kind == ShelfSelector::Kind::TARGET &&
              carrier_grounded[q] == assigned->shelf.value + 1) ||
             (assigned->shelf.kind ==
                  ShelfSelector::Kind::ANON_AT_EPOCH_CELL &&
              assigned->shelf.value == q &&
              carrier_grounded[q] == -1));
        if (exact_shelf_here && !preparing)
          append_candidate(ai->v_now, (uint8_t)Op::LIFT);
        if (exact_shelf_here && preparing)
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        append_all_moves([&](int cell) {
          return std::make_pair(
              eng.lower.dist(assigned->from, cell), cell);
        });
        append_candidate(ai->v_now, (uint8_t)Op::WAIT);
      } else {
        auto footprint = [&](int cell) {
          if (guide == nullptr || guide->upper_epoch == nullptr) return 0;
          for (const int index : guide->ready_tasks) {
            if (index < 0 ||
                index >=
                    (int)guide->upper_epoch->task_graph.tasks.size())
              continue;
            const auto& id =
                guide->upper_epoch->task_graph.tasks[index].id;
            if (id.from == cell || id.to == cell) return 1;
          }
          for (const auto& item : guide->custody_by_robot)
            if (item.has_value() &&
                (item->from == cell || item->to == cell))
              return 1;
          return 0;
        };
        if (footprint(q) != 0) {
          append_all_moves([&](int cell) {
            return std::make_pair(footprint(cell), cell);
          });
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        } else {
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
          append_all_moves([&](int cell) {
            return std::make_pair(footprint(cell), cell);
          });
        }
      }
    }

  }

  if (carrier_scratch_node != nullptr) {
    const auto* checkpoint = find_reference_checkpoint(
        physical_state_of(
            carrier_scratch_node->C,
            carrier_scratch_node->shelf));
    if (checkpoint != nullptr &&
        checkpoint->next_ops.size() ==
            carrier_scratch_node->C.size()) {
      const Op& wanted = checkpoint->next_ops[i];
      const int wanted_cell =
          wanted.kind == Op::MOVE ? wanted.to : ai->v_now->index;
      const auto it = std::find_if(
          cand.begin(), cand.end(), [&](const auto& candidate) {
            return candidate.second == wanted.kind &&
                   candidate.first->index == wanted_cell;
          });
      if (it != cand.end()) {
        std::rotate(cand.begin(), it, std::next(it));
        if (stats != nullptr) ++stats->reference_action_hints;
      }
    }
  }

  // ---- unified try loop ----
  // Reservation semantics are ROLE-dependent (design 5.4; debug.md v3
  // section 4 D1, regression `search_not_dominated_by_own_rollout_...`):
  //   task agents  — upstream shape verbatim (keep the reservation on a
  //                  failed push; wait fallback reserves and FAILS);
  //   carrier agents — the two-deck generator releases the reservation
  //                  and tries its next candidate (a failed push must not
  //                  poison the remaining candidates: S1 makes carrier
  //                  pushes fail far more often than task pushes), and
  //                  the wait fallback SUCCEEDS when feasible.
  const bool carrier_role =
      task_id < 0 && carrier_scratch_node != nullptr &&
      !carrier_scratch_node->shelf.kappa.empty();
  for (size_t k = 0; k < cand.size(); ++k) {
    auto u = cand[k].first;
    const uint8_t kind = cand[k].second;
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // carrier feasibility (M4); none of these fire for task agents
    if (kind == Op::MOVE && loaded && carrier_upper_taken(u->index))
      continue;  // S1
    if (kind == Op::LIFT && carrier_grounded[u->index] == 0) continue;
    if (kind == Op::DROP && kappa_i == KAPPA_ANON &&
        carrier_upper_taken(u->index))
      continue;

    occupied_next[u->id] = ai;
    ai->v_next = u;
    ai->op_kind = kind;
    if (loaded && kind != Op::LIFT) carrier_upper_add(u->index);

    if (ak != nullptr && ak != ai && ak->v_next == nullptr) {
      if (stats != nullptr) ++stats->pibt_recursions;
      if (!funcPIBT(ak, assignment)) {
        if (carrier_role) {
          // release and retry the next candidate (two-deck semantics)
          if (occupied_next[u->id] == ai) occupied_next[u->id] = nullptr;
          ai->v_next = nullptr;
          if (loaded && kind != Op::LIFT) carrier_upper_sub(u->index);
        }
        continue;
      }
    }

    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      swap_agent->op_kind = Op::MOVE;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    return true;
  }

  if (carrier_role) {
    // two-deck wait fallback: succeed when feasible, no forced reservation
    if (occupied_next[ai->v_now->id] == nullptr) {
      occupied_next[ai->v_now->id] = ai;
      ai->v_next = ai->v_now;
      ai->op_kind = Op::WAIT;
      if (loaded) carrier_upper_add(ai->v_now->index);
      return true;
    }
    if (stats != nullptr) ++stats->pibt_failures;
    return false;
  }

  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  ai->op_kind = Op::WAIT;
  if (loaded) carrier_upper_add(ai->v_now->index);
  if (stats != nullptr) ++stats->pibt_failures;
  return false;
}

Agent* TAPFPlanner::swap_possible_and_required(
    Agent* ai, const std::vector<int>& assignment)
{
  if (stats != nullptr) ++stats->swap_checks;
  const auto i = ai->id;
  // LaCAM2 swap reasoning is defined over instance-task distance fields;
  // carrier agents (no task) are covered by the yield/park machinery
  // instead (design 5.5, M8)
  if (assignment[i] < 0) return nullptr;
  if (C_next[i][0] == ai->v_now) return nullptr;

  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr && assignment[aj->id] >= 0 &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now, assignment) &&
      is_swap_possible(aj->v_now, ai->v_now, assignment)) {
    return aj;
  }

  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (assignment[ak->id] < 0) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0], assignment) &&
        is_swap_possible(C_next[i][0], ai->v_now, assignment)) {
      return ak;
    }
  }

  return nullptr;
}

bool TAPFPlanner::is_swap_required(const int pusher, const int puller,
                                   Vertex* v_pusher_origin,
                                   Vertex* v_puller_origin,
                                   const std::vector<int>& assignment)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  const auto pusher_task = assignment[pusher];
  const auto puller_task = assignment[puller];

  while (D.get(pusher_task, v_puller) < D.get(pusher_task, v_pusher)) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr &&
           assignment[a->id] >= 0 && ins->tasks[assignment[a->id]] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  return (D.get(puller_task, v_pusher) < D.get(puller_task, v_puller)) &&
         (D.get(pusher_task, v_pusher) == 0 ||
          D.get(pusher_task, v_puller) < D.get(pusher_task, v_pusher));
}

bool TAPFPlanner::is_swap_possible(Vertex* v_pusher_origin,
                                   Vertex* v_puller_origin,
                                   const std::vector<int>& assignment)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr &&
           assignment[a->id] >= 0 && ins->tasks[assignment[a->id]] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

// ---- carrier layer helpers (M4); all trivial with an empty layer ----

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
  for (const int p : S->shelf.anon_occ) carrier_grounded[p] = -1;
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

// per-kind preconditions of a FORCED op under a partial constraint
// (final arbitration is the oracle's; a wrong veto here only delays the
// combination to a deeper constraint — G1 keeps completeness)
bool TAPFPlanner::forced_op_feasible(const TAPFNode* S, int i, Vertex* v,
                                     uint8_t kind)
{
  const int kappa_i = S->shelf.kappa[i];
  switch (kind) {
    case Op::MOVE:
      if (kappa_i != KAPPA_FREE && carrier_upper_taken(v->index))
        return false;  // S1
      return true;
    case Op::LIFT:
      return kappa_i == KAPPA_FREE && carrier_grounded[S->C[i]->index] != 0;
    case Op::DROP:
      if (kappa_i == KAPPA_FREE) return false;
      if (dd_view != nullptr &&
          !dd_view->can_store_shelf(S->C[i]->index))
        return false;
      if (kappa_i == KAPPA_ANON && carrier_upper_taken(v->index))
        return false;  // another shelf occupies the cell at t+1
      return true;
    case Op::WAIT:
    default:
      return true;
  }
}

bool TAPFPlanner::apply_carrier_effects(const TAPFNode* S)
{
  if (S->shelf.kappa.empty()) {
    shelf_next_scratch = S->shelf;  // empty layer: carried over as-is
    return true;
  }
  // assemble the joint op from the agents' reservations
  ops_scratch.resize(N);
  for (const auto a : A) {
    switch (a->op_kind) {
      case Op::MOVE:
        ops_scratch[a->id] = a->v_next == a->v_now
                                 ? Op::make_wait()
                                 : Op::make_move(a->v_next->index);
        break;
      case Op::LIFT:
        ops_scratch[a->id] = Op::make_lift();
        break;
      case Op::DROP:
        ops_scratch[a->id] = Op::make_drop();
        break;
      case Op::WAIT:
      default:
        ops_scratch[a->id] = Op::make_wait();
        break;
    }
  }
  // conformance oracle = final arbiter (design 6.4, M4)
  const auto& phys = carrier->phys_view(S);
  const auto nxt = apply_ops(*dd_view, phys, ops_scratch);
  if (!nxt.has_value()) return false;
  shelf_next_scratch.target_pos = nxt->target_pos;
  shelf_next_scratch.anon_occ = nxt->anon_occ;
  shelf_next_scratch.kappa = nxt->kappa;
  return true;
}

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

std::optional<TAPFPlanner::CarrierRolloutStep>
TAPFPlanner::next_carrier_rollout_step(
    TAPFNode* node, bool escape_preferred_self_loop)
{
  const auto capture = [&]() {
    CarrierRolloutStep step;
    step.config.resize(N);
    for (const auto* agent : A)
      step.config[agent->id] = agent->v_next;
    step.shelf = shelf_next_scratch;
    step.ops = ops_scratch;
    return step;
  };
  const auto is_self_loop = [&](const CarrierRolloutStep& step) {
    return is_same_config(step.config, node->C) &&
           step.shelf == node->shelf;
  };

  TAPFConstraint root;
  if (!get_new_config(node, &root) ||
      !apply_carrier_effects(node))
    return std::nullopt;
  CarrierRolloutStep preferred = capture();
  if (!escape_preferred_self_loop ||
      !is_self_loop(preferred))
    return preferred;

  // The additive assignment deliberately permits idle.  A greedy rollout,
  // unlike the complete high-level search, otherwise has no opportunity to
  // visit the remaining legal operators.  Drain the same lazy low-level
  // constraint tree used by solve(); do not mutate the node's own tree.
  std::queue<TAPFConstraint*> constraints;
  std::vector<OpCand> candidates;
  if (N > 0) {
    const int robot = node->constraint_order[0];
    build_op_candidates(node, robot, candidates);
    lacam_expand_constraint_vec<TAPFConstraint>(
        &root, robot, candidates, constraints);
  }
  const auto clear_constraints = [&]() {
    while (!constraints.empty()) {
      delete constraints.front();
      constraints.pop();
    }
  };

  while (!constraints.empty() && !is_expired(deadline)) {
    TAPFConstraint* constraint = constraints.front();
    constraints.pop();
    if (constraint->depth < N) {
      const int robot =
          node->constraint_order[constraint->depth];
      build_op_candidates(node, robot, candidates);
      lacam_expand_constraint_vec<TAPFConstraint>(
          constraint, robot, candidates, constraints);
    }
    const bool generated =
        get_new_config(node, constraint) &&
        apply_carrier_effects(node);
    delete constraint;
    if (!generated) continue;
    CarrierRolloutStep alternative = capture();
    if (is_self_loop(alternative)) continue;
    clear_constraints();
    return alternative;
  }
  clear_constraints();
  return preferred;
}

TAPFPlanner::CarrierRollout TAPFPlanner::carrier_rollout(const Config& C0,
                                                         const ShelfState& S0,
                                                         int max_steps,
                                                         int min_chunk,
                                                         bool stop_on_event,
                                                         const TAPFNode*
                                                             initial_anchor,
                                                         bool
                                                             escape_preferred_self_loop)
{
  CarrierRollout out;
  if (stats != nullptr) ++stats->rollout_calls;
  out.configs.push_back(C0);
  out.shelves.push_back(S0);
  std::unordered_set<uint64_t> local_seen;
  auto state_hash = [&](const Config& C, const ShelfState& S) {
    return (uint64_t)ConfigHasher()(C) ^ shelf_layer_hash(S);
  };
  local_seen.insert(state_hash(C0, S0));
  auto make_rollout_node = [&](const Config& C, const ShelfState& S) {
    return std::make_unique<TAPFNode>(
        C, S, D, ins, std::vector<int>(N, -1),
        TAPFAssignmentState(), nullptr);
  };
  auto current = make_rollout_node(C0, S0);
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
    const auto generated =
        next_carrier_rollout_step(
            current.get(), escape_preferred_self_loop);
    if (!generated.has_value()) return out;
    C_step = generated->config;
    const auto& next_shelf = generated->shelf;
    const auto& ops = generated->ops;
    if (!local_seen.insert(state_hash(C_step, next_shelf)).second) {
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
    out.shelves.push_back(next_shelf);
    if (stats != nullptr) ++stats->macro_steps;

    auto next = make_rollout_node(C_step, next_shelf);
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

Solution solve_tapf(const TAPFInstance& ins, const int verbose,
                    const Deadline* deadline, std::mt19937* MT,
                    const int sticky_penalty, TAPFStats* stats, bool anytime,
                    bool force_full_assignment,
                    TAPFSearchConfig search_config)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tTAPF pre-processing");
  auto planner = TAPFPlanner(&ins, deadline, MT, verbose, sticky_penalty,
                             0.001f, anytime, stats, search_config);
  planner.force_full_assignment = force_full_assignment;
  return planner.solve();
}
