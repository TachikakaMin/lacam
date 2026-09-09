//
// Carrier (DD) entry adapters over the INTEGRATED LaCAM-TAPF planner
// (design.md v3 section 10; debug.md v3 WP5, mappings M11/M15/M16).
//
// There is exactly ONE solve loop in this codebase: TAPFPlanner::solve()
// in tapf_planner.cpp. This file converts DDInstance/PhysConfig to TAPF
// state types, runs the dynamic-to-fixed assignment refinement policy,
// implements B0/B1 on the shared generator/rollout, and re-exports test
// probes over the production guidance machinery (carrier_guidance.hpp).
//



#include "dd_planner_internal.hpp"

using namespace dd_detail;

const char* dd_improvement_exit_reason_name(
    DDImprovementExitReason reason)
{
  switch (reason) {
    case DDImprovementExitReason::NOT_ATTEMPTED:
      return "NOT_ATTEMPTED";
    case DDImprovementExitReason::NO_REMAINING_BUDGET:
      return "NO_REMAINING_BUDGET";
    case DDImprovementExitReason::STRICT_IMPROVEMENT:
      return "STRICT_IMPROVEMENT";
    case DDImprovementExitReason::SEARCH_CUTOFF:
      return "SEARCH_CUTOFF";
    case DDImprovementExitReason::SEARCH_EXHAUSTED:
      return "SEARCH_EXHAUSTED";
    case DDImprovementExitReason::CANDIDATE_REJECTED:
      return "CANDIDATE_REJECTED";
    case DDImprovementExitReason::FIXED_GOAL_SETUP_FAILED:
      return "FIXED_GOAL_SETUP_FAILED";
    case DDImprovementExitReason::REFERENCE_SUFFIX_ACCEPTED:
      return "REFERENCE_SUFFIX_ACCEPTED";
  }
  return "UNKNOWN";
}

const char* carrier_brd_exit_reason_name(
    CarrierBRDExitReason reason)
{
  switch (reason) {
    case CarrierBRDExitReason::SOLVED:
      return "SOLVED";
    case CarrierBRDExitReason::TAU_FAILED:
      return "TAU_FAILED";
    case CarrierBRDExitReason::UPPER_TIMEOUT:
      return "UPPER_TIMEOUT";
    case CarrierBRDExitReason::UPPER_EXHAUSTED:
      return "UPPER_EXHAUSTED";
    case CarrierBRDExitReason::TASK_COMPILE_INVALID:
      return "TASK_COMPILE_INVALID";
    case CarrierBRDExitReason::WAVE_START_MISMATCH:
      return "WAVE_START_MISMATCH";
    case CarrierBRDExitReason::DISPATCH_TIMEOUT:
      return "DISPATCH_TIMEOUT";
    case CarrierBRDExitReason::DISPATCH_STUCK:
      return "DISPATCH_STUCK";
    case CarrierBRDExitReason::SEGMENT_TIMEOUT:
      return "SEGMENT_TIMEOUT";
    case CarrierBRDExitReason::SEGMENT_EXHAUSTED:
      return "SEGMENT_EXHAUSTED";
    case CarrierBRDExitReason::SEGMENT_INVALID:
      return "SEGMENT_INVALID";
    case CarrierBRDExitReason::WAVE_END_MISMATCH:
      return "WAVE_END_MISMATCH";
    case CarrierBRDExitReason::FINAL_GOAL_MISMATCH:
      return "FINAL_GOAL_MISMATCH";
    case CarrierBRDExitReason::FINAL_REPLAY_INVALID:
      return "FINAL_REPLAY_INVALID";
    case CarrierBRDExitReason::SEARCH_TIMEOUT:
      return "SEARCH_TIMEOUT";
    case CarrierBRDExitReason::FINALIZATION_DEADLINE:
      return "FINALIZATION_DEADLINE";
  }
  return "UNKNOWN";
}

static DDSolveResult solve_carrier_lacam_from_state_result_impl(
    const DDInstance& ins, const PhysConfig& current,
    double time_limit_sec, int seed, DDStats* stats,
    DDPlan* best_effort,
    std::shared_ptr<TAPFCarrierPersistentState>
        carrier_persistent_state,
    const TAPFCarrierRootContinuation*
        carrier_root_continuation,
    std::shared_ptr<CarrierGuidance>*
        carrier_root_guidance_output)
{
  if (stats != nullptr) *stats = DDStats();
  if (best_effort != nullptr) best_effort->clear();
  if (ins.shelves.empty() ||
      !validate_phys_config_root(ins, current).valid())
    return DDSolveResult{DDSolveStatus::INVALID, {}};

  const TAPFInstance view(ins);
  const auto started = Clock::now();

  // One controller owns the verified incumbent and at most two calls to the
  // same TAPFPlanner::solve() implementation.  The first call stops at the
  // first deliverable plan.  The second call is bounded by that incumbent
  // and spends the remaining shared search budget on improvement.  Every
  // second pass receives a fresh singleton-goal view built from the
  // incumbent's actual terminal target positions.
  constexpr size_t MACRO_TARGET_LIMIT = 64;
  const bool use_macro = ins.n_targets() <= MACRO_TARGET_LIMIT;
  // A retained improvement planner may store a pointer to a copied fixed
  // assignment view. Keep that view alive through deferred destruction.
  std::unique_ptr<TAPFInstance> fixed_view_storage;
  std::optional<TAPFReferencePlan> first_reference;
  // Search trees can take seconds to destroy on dense cases.  Their
  // destruction is mandatory solver work and is completed before the
  // strict return timestamp.
  std::vector<std::unique_ptr<TAPFPlanner>> deferred_cleanup;
  deferred_cleanup.reserve(2);
  const double finalization_reserve_sec =
      std::min(1.5, std::max(0.25,
                            std::max(0.0, time_limit_sec) * 0.15));
  const double search_budget_sec =
      std::max(
          0.0,
          std::max(0.0, time_limit_sec) -
              finalization_reserve_sec);
  Deadline search_deadline(search_budget_sec * 1000.0);
  PlanCost cost = PlanCost::unbounded();
  double first_ms = -1, first_soc = -1;
  long first_makespan = -1;
  int64_t first_work_scaled = -1;
  long max_depth = 0, targets_done = 0;
  bool phase1_solved = false;
  DDPlan plan = run_search_attempt(
      view, ins, current, &search_deadline, seed, use_macro,
      TAPFStopPolicy::FIRST_FEASIBLE, PlanCost::unbounded(),
      nullptr, stats, best_effort, &phase1_solved, &cost, &first_ms,
      &first_makespan, &first_work_scaled, &first_soc,
      &max_depth, &targets_done,
      nullptr,
      &deferred_cleanup,
      std::move(carrier_persistent_state),
      carrier_root_continuation,
      carrier_root_guidance_output);
  // Do not accumulate two large search trees until finalization.  Pass 1's
  // tree no longer owns anything needed by the materialized incumbent.
  deferred_cleanup.clear();

  if (phase1_solved && is_expired(&search_deadline) &&
      stats != nullptr)
    stats->improvement_exit_reason =
        DDImprovementExitReason::NO_REMAINING_BUDGET;

  if (phase1_solved && !is_expired(&search_deadline)) {
    auto fixed =
        fixed_goal_instance_from_plan(ins, current, plan);
    if (!fixed.has_value()) {
      if (stats != nullptr)
        stats->improvement_exit_reason =
            DDImprovementExitReason::FIXED_GOAL_SETUP_FAILED;
    } else {
      const bool fixed_restart = has_dynamic_goal_sets(ins);
      fixed_view_storage =
          std::make_unique<TAPFInstance>(*fixed);
      first_reference = build_reference_plan(
          *fixed, current, plan, 256);
      const TAPFInstance* improvement_view =
          fixed_view_storage.get();
      const DDInstance* improvement_ins = &*fixed;
      if (stats != nullptr) {
        ++stats->improvement_attempts;
        if (fixed_restart) {
          ++stats->assignment_restarts;
          stats->assignment_first_soc = cost.work_value();
          stats->assignment_first_makespan = cost.ticks;
        }
      }
      PlanCost cost2 = PlanCost::unbounded();
      double second_ms = -1, second_soc = -1;
      long second_makespan = -1;
      bool phase2_solved = false;
      bool phase2_cutoff = false;
      DDPlan plan2 = run_search_attempt(
          *improvement_view, *improvement_ins, current,
          &search_deadline, seed,
          /*macro_enabled=*/false,
          TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT,
          cost,
          first_reference.has_value() ? &*first_reference : nullptr,
          stats, nullptr,
          &phase2_solved, &cost2, &second_ms,
          &second_makespan, nullptr, &second_soc, &max_depth,
          &targets_done, &phase2_cutoff, &deferred_cleanup);
      if (fixed_restart && phase2_solved && stats != nullptr) {
        // This diagnostic now means what its name says: the second search
        // itself produced a new, strictly-better candidate.  The retained
        // first-pass fallback is not counted as a second-pass solve.
        ++stats->assignment_second_solved;
        stats->assignment_second_solution_ms = second_ms;
        stats->assignment_second_soc = cost2.work_value();
        stats->assignment_second_makespan = cost2.ticks;
      }
      if (phase2_solved) {
        if (stats != nullptr) {
          ++stats->improvement_candidates;
          if (fixed_restart) {
            stats->assignment_second_soc = cost2.work_value();
            stats->assignment_second_makespan = cost2.ticks;
          }
        }
        if (cost2 < cost) {
          if (stats != nullptr) {
            ++stats->improvement_improvements;
            if (fixed_restart)
              ++stats->assignment_improvements;
          }
          plan = std::move(plan2);
          cost = cost2;
          if (stats != nullptr) {
            stats->improvement_exit_reason =
                stats->reference_suffix_accepted > 0
                    ? DDImprovementExitReason::
                          REFERENCE_SUFFIX_ACCEPTED
                    : DDImprovementExitReason::
                          STRICT_IMPROVEMENT;
          }
        } else if (stats != nullptr) {
          stats->improvement_exit_reason =
              DDImprovementExitReason::CANDIDATE_REJECTED;
        }
      } else if (stats != nullptr) {
        stats->improvement_exit_reason =
            second_ms >= 0
                ? DDImprovementExitReason::CANDIDATE_REJECTED
                : (phase2_cutoff
                       ? DDImprovementExitReason::SEARCH_CUTOFF
                       : DDImprovementExitReason::SEARCH_EXHAUSTED);
      }
    }
  }

  auto finish = [&](bool solved, DDPlan final_plan, PlanCost final_cost) {
    // A plan is not returned until all deferred solver-owned search state
    // has been destroyed.  fixed_view_storage remains alive across this
    // clear, preserving the pass-2 planner's instance pointer.
    deferred_cleanup.clear();
    DDSolveStatus status = solved ? DDSolveStatus::SOLVED
                                  : (stats != nullptr && stats->timed_out
                                         ? DDSolveStatus::TIMEOUT
                                         : DDSolveStatus::EXHAUSTED);
    if (solved) {
      const auto prefix =
          normalize_goal_prefix(ins, current, final_plan);
      const bool valid = prefix.has_value();
      if (valid) {
        final_plan = *prefix;
        final_cost = plan_cost(ins, current, final_plan);
      }
      const double deliverable_ms =
          std::chrono::duration<double, std::milli>(
              Clock::now() - started)
              .count();
      const auto finalization = dd_classify_finalization_probe(
          valid, deliverable_ms,
          std::max(0.0, time_limit_sec) * 1000.0);
      if (finalization != DDFinalizationStatus::ACCEPT) {
        final_plan.clear();
        final_cost = PlanCost::unbounded();
        status = finalization == DDFinalizationStatus::DEADLINE
                     ? DDSolveStatus::TIMEOUT
                     : DDSolveStatus::INVALID;
      } else if (stats != nullptr) {
        stats->deliverable_ms = deliverable_ms;
      }
    }
    if (stats != nullptr) {
      stats->first_solution_ms = first_ms;
      stats->first_solution_makespan = first_makespan;
      stats->first_solution_work_scaled = first_work_scaled;
      stats->first_solution_soc = first_soc;
      stats->best_makespan =
          status == DDSolveStatus::SOLVED ? final_cost.ticks : -1;
      stats->best_work_scaled =
          status == DDSolveStatus::SOLVED ? final_cost.work : -1;
      stats->best_soc =
          status == DDSolveStatus::SOLVED ? final_cost.work_value() : -1;
      stats->max_depth = max_depth;
      stats->best_targets_done = targets_done;
      // timed_out accumulated per pass above: an empty plan is a timeout
      // only if some pass actually expired; OPEN exhaustion and generator
      // failure report as a plain (non-timeout) failure.
      stats->timed_out = status == DDSolveStatus::TIMEOUT;
    }
    return DDSolveResult{status, std::move(final_plan)};
  };
  return finish(phase1_solved, std::move(plan), cost);
}

DDSolveResult solve_carrier_lacam_from_state_result(
    const DDInstance& ins, const PhysConfig& current,
    double time_limit_sec, int seed, DDStats* stats,
    DDPlan* best_effort)
{
  return solve_carrier_lacam_from_state_result_impl(
      ins, current, time_limit_sec, seed, stats, best_effort,
      nullptr, nullptr, nullptr);
}

struct DDPlanningSession::Impl {
  DDInstance instance;
  PhysConfig current;
  int seed = 0;
  std::shared_ptr<TAPFCarrierPersistentState> persistent;
  std::optional<TAPFCarrierRootContinuation> continuation;
  std::shared_ptr<CarrierGuidance> root_guidance;
  DDPlan last_plan;
  bool has_solved_plan = false;

  Impl(
      const DDInstance& ins, const PhysConfig& initial,
      int session_seed)
      : instance(ins),
        current(initial),
        seed(session_seed),
        persistent(
            std::make_shared<TAPFCarrierPersistentState>(instance))
  {
    if (instance.shelves.empty() ||
        !validate_phys_config_root(instance, current).valid())
      throw std::invalid_argument(
          "DDPlanningSession requires a valid Carrier root");
  }
};

DDPlanningSession::DDPlanningSession(
    const DDInstance& ins, const PhysConfig& initial, int seed)
    : impl_(std::make_unique<Impl>(ins, initial, seed))
{
}

DDPlanningSession::~DDPlanningSession() = default;
DDPlanningSession::DDPlanningSession(
    DDPlanningSession&&) noexcept = default;
DDPlanningSession& DDPlanningSession::operator=(
    DDPlanningSession&&) noexcept = default;

DDSolveResult DDPlanningSession::solve(
    double time_limit_sec, DDStats* stats,
    DDPlan* best_effort)
{
  if (impl_ == nullptr)
    throw std::logic_error(
        "DDPlanningSession is moved from");

  std::shared_ptr<CarrierGuidance> attached_root;
  const auto result =
      solve_carrier_lacam_from_state_result_impl(
          impl_->instance, impl_->current, time_limit_sec,
          impl_->seed, stats, best_effort, impl_->persistent,
          impl_->continuation.has_value()
              ? &*impl_->continuation
              : nullptr,
          &attached_root);
  if (attached_root != nullptr) {
    impl_->root_guidance = std::move(attached_root);
    impl_->continuation.reset();
  }
  impl_->has_solved_plan = result.solved();
  impl_->last_plan =
      result.solved() ? result.plan : DDPlan{};
  return result;
}

DDCommitStatus DDPlanningSession::commit_prefix(
    size_t executed_steps, const PhysConfig& observed)
{
  if (impl_ == nullptr)
    throw std::logic_error(
        "DDPlanningSession is moved from");
  if (!impl_->has_solved_plan ||
      impl_->root_guidance == nullptr)
    return DDCommitStatus::NO_SOLVED_PLAN;
  if (executed_steps == 0 ||
      executed_steps > impl_->last_plan.size())
    return DDCommitStatus::INVALID_PREFIX;

  PhysConfig replayed = impl_->current;
  for (size_t step = 0; step < executed_steps; ++step) {
    const auto next = apply_ops(
        impl_->instance, replayed,
        impl_->last_plan[step]);
    if (!next.has_value())
      return DDCommitStatus::INVALID_PREFIX;
    replayed = *next;
  }
  if (!(replayed == observed) ||
      !validate_phys_config_root(
           impl_->instance, observed)
           .valid())
    return DDCommitStatus::STATE_MISMATCH;

  TAPFCarrierRootContinuation next_continuation;
  next_continuation.previous_physical = impl_->current;
  next_continuation.previous_guidance =
      *impl_->root_guidance;
  next_continuation.executed_prefix.assign(
      impl_->last_plan.begin(),
      impl_->last_plan.begin() + executed_steps);

  impl_->current = observed;
  impl_->continuation = std::move(next_continuation);
  impl_->last_plan.clear();
  impl_->has_solved_plan = false;
  return DDCommitStatus::OK;
}

DDRebaseStatus DDPlanningSession::rebase(
    const PhysConfig& observed)
{
  if (impl_ == nullptr)
    throw std::logic_error(
        "DDPlanningSession is moved from");
  if (!validate_phys_config_root(
           impl_->instance, observed)
           .valid())
    return DDRebaseStatus::INVALID_STATE;

  impl_->current = observed;
  impl_->continuation.reset();
  impl_->root_guidance.reset();
  impl_->last_plan.clear();
  impl_->has_solved_plan = false;
  return DDRebaseStatus::OK;
}

const RootGoalCommitment&
DDPlanningSession::root_goal_commitment() const
{
  static const RootGoalCommitment empty;
  if (impl_ == nullptr || impl_->root_guidance == nullptr ||
      impl_->root_guidance->upper_epoch == nullptr)
    return empty;
  return impl_->root_guidance->upper_epoch
      ->root_goal_commitment;
}

DDSolveResult solve_carrier_lacam_result(
    const DDInstance& ins, double time_limit_sec, int seed,
    DDStats* stats, DDPlan* best_effort)
{
  return solve_carrier_lacam_from_state_result(
      ins, initial_phys_config(ins), time_limit_sec, seed,
      stats, best_effort);
}

DDSolveResult dd_solve_carrier_lacam_fixed_tau_probe(
    const DDInstance& ins, const std::vector<int>& tau,
    double time_limit_sec, int seed, DDStats* stats)
{
  if (tau.size() != ins.n_targets())
    throw std::invalid_argument(
        "fixed tau size does not match target count");

  DDInstance fixed = ins;
  std::unordered_set<int> assigned_goals;
  for (size_t target = 0; target < tau.size(); ++target) {
    if (!std::binary_search(
            ins.target_goal_sets[target].begin(),
            ins.target_goal_sets[target].end(), tau[target]))
      throw std::invalid_argument(
          "fixed tau contains an ineligible target goal");
    if (!assigned_goals.insert(tau[target]).second)
      throw std::invalid_argument(
          "fixed tau assigns multiple targets to one cell");
    fixed.target_goals[target] = tau[target];
    fixed.target_goal_sets[target] = {tau[target]};
  }
  fixed.finalize();
  return solve_carrier_lacam_result(
      fixed, time_limit_sec, seed, stats);
}

DDPlan solve_carrier_lacam(const DDInstance& ins, double time_limit_sec,
                           int seed, DDStats* stats, DDPlan* best_effort)
{
  return solve_carrier_lacam_result(
             ins, time_limit_sec, seed, stats, best_effort)
      .plan;
}

DDPlan solve_carrier_rollout(const DDInstance& ins, double time_limit_sec,
                             int seed, DDStats* stats)
{
  // B0 (design 8.1): the SHARED rollout core with no high-level search
  const TAPFInstance view(ins);
  if (stats != nullptr) *stats = DDStats();
  const auto t_start = Clock::now();
  std::mt19937 mt(seed);
  Deadline deadline(time_limit_sec * 1000);
  TAPFStats tstats;
  TAPFPlanner planner(&view, &deadline, &mt, 0, 0, 0.001f, true, &tstats);

  DDPlan plan;
  auto C = view.starts;
  auto S = initial_shelf_state(view);
  if (planner.is_goal_config(C, S)) {
    plan.push_back(std::vector<Op>(view.N, Op::make_wait()));
    return plan;
  }
  std::unordered_set<uint64_t> seen;
  seen.insert(state_hash(C, S));
  while (std::chrono::duration<double>(Clock::now() - t_start).count() <
         time_limit_sec) {
    auto r =
        planner.carrier_rollout(C, S, 512, 0, /*stop_on_event=*/false);
    for (auto& ops : r.ops) plan.push_back(std::move(ops));
    C = r.configs.back();
    S = r.shelves.back();
    if (r.reached_goal) {
      map_stats(tstats, stats);
      return plan;
    }
    if (r.ops.empty()) {
      map_stats(tstats, stats);
      if (stats != nullptr) ++stats->generator_failures;
      return {};  // stuck: honest failure (no search to recover)
    }
    if (!seen.insert(state_hash(C, S)).second) {
      map_stats(tstats, stats);
      return {};  // global cycle
    }
  }
  map_stats(tstats, stats);
  if (stats != nullptr) stats->timed_out = true;
  return {};
}

DDPlan solve_carrier_2stage(
    const DDInstance& ins, double time_limit_sec, int seed, DDStats* stats,
    std::vector<std::vector<int>>* fixed_paths_out)
{
  // B1 freezes both the root PairCost matching and one deterministic
  // shortest shelf path per target.  Its per-step execution still uses
  // the same Task-BR ready/rho/custody layer and Carrier-PIBT generator as
  // production; leaving a frozen path is an honest baseline failure.
  const TAPFInstance view(ins);
  if (stats != nullptr) *stats = DDStats();
  const auto started = Clock::now();
  std::mt19937 mt(seed);
  Deadline deadline(std::max(0.0, time_limit_sec) * 1000);
  TAPFStats tapf_stats;
  TAPFPlanner planner(
      &view, &deadline, &mt, 0, 0, 0.001f, true, &tapf_stats);
  DDDistCache upper_wall(ins.grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  LowerDist lower_distance(ins.grid);
  const auto weights = soc_weights_from_env();

  PhysConfig physical = initial_phys_config(ins);
  const std::vector<int> fixed_tau = tau_of(ins, physical);
  const auto initial_upper =
      carrier_detail::make_upper_signature(physical);
  std::vector<uint8_t> occupied(ins.grid.size(), 0);
  for (const int cell : initial_upper.target_pos) occupied[cell] = 1;
  for (const int cell : initial_upper.anon_pos) occupied[cell] = 1;

  auto shortest_least_blocking_path =
      [&](int src, int dst) -> std::vector<int> {
    using QueueItem = std::tuple<int, int, int>;
    const int inf = std::numeric_limits<int>::max() / 4;
    std::vector<std::pair<int, int>> distance(
        ins.grid.size(), {inf, inf});
    std::vector<int> previous(ins.grid.size(), -1);
    std::priority_queue<
        QueueItem, std::vector<QueueItem>, std::greater<QueueItem>>
        open;
    distance[src] = {0, 0};
    open.emplace(0, 0, src);
    while (!open.empty()) {
      const auto [steps, blockers, cell] = open.top();
      open.pop();
      if (distance[cell] != std::make_pair(steps, blockers)) continue;
      if (cell == dst) break;
      int raw_neighbors[4];
      const int count =
          ins.grid.neighbors(cell, raw_neighbors);
      std::vector<int> neighbors(
          raw_neighbors, raw_neighbors + count);
      std::sort(neighbors.begin(), neighbors.end());
      for (const int next : neighbors) {
        const std::pair<int, int> candidate{
            steps + 1,
            blockers +
                (occupied[next] && next != src ? 1 : 0)};
        if (candidate < distance[next] ||
            (candidate == distance[next] &&
             (previous[next] < 0 || cell < previous[next]))) {
          distance[next] = candidate;
          previous[next] = cell;
          open.emplace(candidate.first, candidate.second, next);
        }
      }
    }
    if (distance[dst].first >= inf) return {};
    std::vector<int> path;
    for (int cell = dst; cell >= 0; cell = previous[cell]) {
      path.push_back(cell);
      if (cell == src) break;
    }
    if (path.empty() || path.back() != src) return {};
    std::reverse(path.begin(), path.end());
    return path;
  };

  std::vector<std::vector<int>> fixed_paths(ins.n_targets());
  for (size_t target = 0; target < ins.n_targets(); ++target) {
    fixed_paths[target] = shortest_least_blocking_path(
        ins.target_starts[target], fixed_tau[target]);
    if (fixed_paths[target].empty()) return {};
  }
  if (fixed_paths_out != nullptr) *fixed_paths_out = fixed_paths;
  std::vector<size_t> fixed_index(ins.n_targets(), 0);

  auto at_fixed_goal = [&](const PhysConfig& state) {
    for (size_t target = 0; target < ins.n_targets(); ++target) {
      if (state.target_pos[target] != fixed_tau[target]) return false;
      if (std::find(state.kappa.begin(), state.kappa.end(),
                    (int)target) != state.kappa.end())
        return false;
    }
    return true;
  };
  if (at_fixed_goal(physical))
    return DDPlan{
        std::vector<Op>(ins.n_robots(), Op::make_wait())};

  carrier_detail::UpperEpochCache fixed_epoch_cache;
  std::optional<PhysConfig> previous_physical;
  std::optional<CarrierGuidance> previous_guidance;
  std::vector<Op> previous_ops;
  std::unordered_set<uint64_t> seen{
      phys_config_hash(physical)};
  DDPlan plan;

  while (std::chrono::duration<double>(
             Clock::now() - started)
             .count() < time_limit_sec) {
    const UpperSignature upper =
        carrier_detail::make_upper_signature(physical);
    auto epoch = fixed_epoch_cache.lookup(upper);
    if (epoch == nullptr) {
      epoch =
          carrier_detail::build_task_br_upper_epoch_for_tau(
              ins, upper, fixed_tau, upper_wall,
              storage_topology, weights.alpha, weights.gamma,
              weights.delta);
      fixed_epoch_cache.insert(upper, epoch);
    }
    CarrierGuidance guidance =
        carrier_detail::build_task_br_guidance_from_upper_epoch(
            ins, physical, std::move(epoch),
            previous_physical.has_value()
                ? &*previous_physical
                : nullptr,
            previous_guidance.has_value()
                ? &*previous_guidance
                : nullptr,
            previous_ops.empty() ? nullptr : &previous_ops);

    auto [config, shelf] = state_of(view, physical);
    auto node = std::make_unique<TAPFNode>(
        config, shelf, planner.D, &view,
        std::vector<int>((int)view.N, -1),
        TAPFAssignmentState(), nullptr);
    node->guide =
        std::make_unique<CarrierGuidance>(guidance);
    node->order = carrier_detail::task_br_robot_order(
        physical, guidance, lower_distance);
    node->constraint_order = node->order;
    planner.invalidate_carrier_scratch();
    TAPFConstraint root;
    if (!planner.get_new_config(node.get(), &root) ||
        !planner.apply_carrier_effects(node.get())) {
      map_stats(tapf_stats, stats);
      if (stats != nullptr) ++stats->generator_failures;
      return {};
    }

    Config next_config(view.N, nullptr);
    for (auto* agent : planner.A)
      next_config[agent->id] = agent->v_next;
    const PhysConfig next =
        phys_of(next_config, planner.shelf_next_scratch);
    std::vector<size_t> next_index = fixed_index;
    bool respects_fixed_paths = true;
    for (size_t target = 0; target < ins.n_targets(); ++target) {
      const auto& path = fixed_paths[target];
      const size_t index = fixed_index[target];
      if (next.target_pos[target] == path[index]) continue;
      if (index + 1 < path.size() &&
          next.target_pos[target] == path[index + 1]) {
        next_index[target] = index + 1;
        continue;
      }
      respects_fixed_paths = false;
      break;
    }
    if (!respects_fixed_paths) {
      map_stats(tapf_stats, stats);
      return {};
    }

    const auto ops = planner.ops_scratch;
    plan.push_back(ops);
    previous_physical = physical;
    previous_guidance = std::move(guidance);
    previous_ops = ops;
    physical = next;
    fixed_index = std::move(next_index);
    if (at_fixed_goal(physical)) {
      map_stats(tapf_stats, stats);
      return plan;
    }
    if (!seen.insert(phys_config_hash(physical)).second) {
      map_stats(tapf_stats, stats);
      return {};
    }
  }

  map_stats(tapf_stats, stats);
  if (stats != nullptr) stats->timed_out = true;
  return {};
}
