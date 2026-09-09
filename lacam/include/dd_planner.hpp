/*
 * dd_planner: carrier (two-deck) entry adapters and test-support probes
 * over the integrated LaCAM-TAPF planner (design.md v3 section 10).
 * The ONE solve loop lives in tapf_planner.cpp; nothing here searches.
 */
#pragma once

#include <optional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dd_carrier.hpp"
#include "tapf_planner.hpp"

using DDPlan = std::vector<std::vector<Op>>;  // per timestep, per robot

enum class DDSolveStatus {
  SOLVED = 0,
  EXHAUSTED = 1,
  TIMEOUT = 2,
  INVALID = 3,
};

enum class DDImprovementExitReason {
  NOT_ATTEMPTED = 0,
  NO_REMAINING_BUDGET = 1,
  STRICT_IMPROVEMENT = 2,
  SEARCH_CUTOFF = 3,
  SEARCH_EXHAUSTED = 4,
  CANDIDATE_REJECTED = 5,
  FIXED_GOAL_SETUP_FAILED = 6,
  REFERENCE_SUFFIX_ACCEPTED = 7,
};

const char* dd_improvement_exit_reason_name(
    DDImprovementExitReason reason);

struct DDSolveResult {
  DDSolveStatus status = DDSolveStatus::EXHAUSTED;
  DDPlan plan;

  bool solved() const { return status == DDSolveStatus::SOLVED; }
};

struct FrozenTaskPlan;
struct Deadline;

enum class CarrierBRDExitReason {
  SOLVED = 0,
  TAU_FAILED = 1,
  UPPER_TIMEOUT = 2,
  UPPER_EXHAUSTED = 3,
  TASK_COMPILE_INVALID = 4,
  WAVE_START_MISMATCH = 5,
  DISPATCH_TIMEOUT = 6,
  DISPATCH_STUCK = 7,
  SEGMENT_TIMEOUT = 8,
  SEGMENT_EXHAUSTED = 9,
  SEGMENT_INVALID = 10,
  WAVE_END_MISMATCH = 11,
  FINAL_GOAL_MISMATCH = 12,
  FINAL_REPLAY_INVALID = 13,
  SEARCH_TIMEOUT = 14,
  FINALIZATION_DEADLINE = 15,
};

const char* carrier_brd_exit_reason_name(
    CarrierBRDExitReason reason);

struct CarrierBRDDispatchSnapshot {
  long wave = -1;
  long epoch = -1;
  long free_robots = 0;
  long pending_tasks = 0;
  long locked_pairs = 0;
  long matcher_rows = 0;
  long real_assignments = 0;
};

struct CarrierBRDStats {
  CarrierBRDExitReason exit_reason =
      CarrierBRDExitReason::TAU_FAILED;
  std::vector<int> tau0;
  long frozen_waves = 0;
  long frozen_tasks = 0;
  long max_wave_width = 0;
  long target_tasks = 0;
  long anonymous_tasks = 0;
  long upper_nodes = 0;
  long upper_constraints = 0;
  long upper_steps = 0;
  long upper_transfers = 0;
  long dispatch_epochs = 0;
  long completion_events = 0;
  long locked_pair_continuations = 0;
  long locked_carriers_max = 0;
  long provisional_reassignments = 0;
  long match_calls = 0;
  long match_max_rows = 0;
  long match_rows_without_finite_real_edge = 0;
  long match_maximum_real_cardinality = 0;
  long match_real_assignments = 0;
  long match_hall_deficient_calls = 0;
  long match_max_cardinality_cutoffs = 0;
  long segments = 0;
  long segment_nodes = 0;
  long segment_failures = 0;
  long raw_plan_steps = 0;
  int64_t raw_work_scaled = -1;
  long goal_prefix_removed = 0;
  long vacancy_potential_builds = 0;
  long vacancy_potential_unreachable_cells = 0;
  long selected_clearance_pushes = 0;
  long selected_clearance_loaded_steps = 0;
  long clearance_first_choice_fallbacks = 0;
  bool raw_plan_valid = false;
  std::string guidance_version = "TASKBR_VACANCY_V1";
  double tau_ms = 0;
  double upper_ms = 0;
  double vacancy_potential_time_ms = 0;
  double task_compile_ms = 0;
  double match_ms = 0;
  double match_max_cardinality_ms = 0;
  double segment_ms = 0;
  double cleanup_ms = 0;
  double replay_ms = 0;
  double incidental_goal_prefix_ms = -1;
  std::vector<long> tasks_completed_per_event;
  std::vector<CarrierBRDDispatchSnapshot> dispatches;
};

struct DDPlanRepairStats {
  long exact_loops = 0;
  long projected_loops = 0;
  long bridge_steps = 0;
  long steps_removed = 0;
};

struct DDStats {
  long hl_nodes = 0;
  long hl_expanded = 0;
  long pibt_calls = 0;
  long validator_rejects = 0;  // PIBT PROPOSALS rejected (Bug C signal)
  long g1_rejects = 0;         // G1 full-constraint combos rejected (by design)
  long duplicate_configs = 0;
  // First-incumbent/baseline Carrier-PIBT calls that returned no joint op.
  // A bounded improvement pass is reported separately so adding anytime
  // search does not rewrite this long-standing health diagnostic.
  long generator_failures = 0;
  long max_depth = 0;
  long best_targets_done = 0;
  long macro_successors = 0;  // event-bounded rollout successors inserted
  long macro_steps = 0;       // total primitive steps inside macro edges
  long macro_after_first = 0; // macro successors inserted after an incumbent
                              // (must stay 0)
  long macro_shelf_motion_successors = 0;
  long macro_robot_only_successors = 0;
  long rollout_calls = 0;
  long rollout_cycles = 0;
  long rollout_shelf_motion_steps = 0;
  long robot_only_successors = 0;
  long manipulation_successors = 0;
  long shelf_motion_successors = 0;
  long upper_epoch_builds = 0;
  long pair_cache_hits = 0;
  long pair_cache_misses = 0;
  long root_pair_cache_hits = 0;
  long root_pair_cache_misses = 0;
  long pair_edges_evaluated = 0;
  long pair_edges_total = 0;
  long pair_edges_reused = 0;
  long root_pair_edges_evaluated = 0;
  long root_pair_edges_total = 0;
  long root_pair_edges_reused = 0;
  long pair_incremental_reuses = 0;
  long pair_hungarian_full_solves = 0;
  long pair_hungarian_row_repairs = 0;
  long pair_hungarian_forced_repairs = 0;
  long pair_rollout_steps = 0;
  long pair_rollout_truncations = 0;
  long pair_rollout_stalls = 0;
  long tau_guide_changes_on_upper_move = 0;
  long joint_task_nodes = 0;
  long joint_task_edges = 0;
  long joint_shared_effects = 0;
  long joint_effect_conflicts = 0;
  long joint_candidate_backtracks = 0;
  long joint_paused_roots = 0;
  long vacancy_potential_builds = 0;
  long vacancy_potential_unreachable_cells = 0;
  long clearance_first_choice_fallbacks = 0;
  double vacancy_potential_time_ms = 0;
  std::string guidance_version = "TASKBR_VACANCY_V1";
  long epoch_first_transfer_comparisons = 0;
  long epoch_first_transfer_flips = 0;
  long epoch_chain_overlap_samples = 0;
  long epoch_chain_overlap_intersection = 0;
  long epoch_chain_overlap_union = 0;
  long upper_epoch_cache_evictions = 0;
  long ready_task_count = 0;
  long rho_repairs = 0;
  long rho_match_calls_execute = 0;
  long rho_match_calls_prepare = 0;
  long rho_candidates_input = 0;
  long rho_candidates_after_claims = 0;
  long rho_candidates_after_key_dedupe = 0;
  long rho_candidates_after_shelf_preselect = 0;
  long rho_candidates_after_priority = 0;
  long rho_invalid_filtered = 0;
  long rho_duplicate_key_filtered = 0;
  long rho_same_shelf_filtered = 0;
  long rho_upstream_claim_filtered = 0;
  long rho_mode_ineligible_filtered = 0;
  long rho_no_reachable_robot_filtered = 0;
  long rho_priority_filtered = 0;
  long rho_matrix_rows_total = 0;
  long rho_matrix_cols_total = 0;
  long rho_matrix_max_rows = 0;
  RhoObjectiveVersion rho_objective_version =
      RhoObjectiveVersion::
          BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2;
  long rho_column_identity_same = 0;
  long rho_column_value_same = 0;
  long rho_mode_or_conflict_same = 0;
  long rho_changed_rows_0 = 0;
  long rho_changed_rows_1 = 0;
  long rho_changed_rows_2 = 0;
  long rho_changed_rows_gt2 = 0;
  long rho_assignment_changes = 0;
  double rho_candidate_time_ms = 0;
  double rho_matrix_time_ms = 0;
  double rho_bottleneck_time_ms = 0;
  double rho_secondary_full_time_ms = 0;
  double rho_canonical_time_ms = 0;
  long custody_continuations = 0;
  long timed_transport_expansions = 0;
  long timed_transport_frames = 0;
  long owner_handoffs = 0;
  long causal_waiting = 0;
  long traffic_waiting = 0;
  long zero_empty_no_ready = 0;
  long rewire_guidance_rebuilds = 0;
  long g_relaxed = 0;         // duplicate hits relaxed to a cheaper g
                              // (generic search-kernel diagnostic)
  long guidance_builds = 0;
  double tau_time_ms = 0;
  double guidance_time_ms = 0;
  double timed_transport_time_ms = 0;
  double deliverable_ms = -1;  // deferred cleanup + final replay complete
  // Incumbent and output-repair cost diagnostics.
  double first_solution_ms = -1;
  long first_solution_makespan = -1;
  int64_t first_solution_work_scaled = -1;
  double first_solution_soc = -1;  // weighted physical cost (a=b=g=d=1)
  long best_makespan = -1;
  int64_t best_work_scaled = -1;
  double best_soc = -1;
  long incumbent_updates = 0;
  long f_pruned = 0;
  long improvement_attempts = 0;
  long improvement_candidates = 0;
  long improvement_improvements = 0;
  long improvement_generator_failures = 0;
  long reference_plans_received = 0;
  long reference_plans_validated = 0;
  long reference_checkpoint_hits = 0;
  long reference_action_hints = 0;
  long reference_suffix_attempts = 0;
  long reference_suffix_accepted = 0;
  DDImprovementExitReason improvement_exit_reason =
      DDImprovementExitReason::NOT_ATTEMPTED;
  // Output normalization: exact-state loops plus shelf-projection loops
  // repaired by lower-deck robot paths.
  long exact_loops = 0;
  long projected_loops = 0;
  long bridge_steps = 0;
  long plan_steps_removed = 0;
  // After a dynamic-goal first solution, restart once from the root with
  // that solution's target->goal assignment fixed to singleton sets.
  long assignment_restarts = 0;
  long assignment_second_solved = 0;
  long assignment_improvements = 0;
  double assignment_second_solution_ms = -1;
  double assignment_first_soc = -1;
  double assignment_second_soc = -1;
  long assignment_first_makespan = -1;
  long assignment_second_makespan = -1;
  bool timed_out = false;
  PhysConfig deepest_config;  // config at max depth (debug)
  std::vector<int> deepest_tau;
};

// R4 (debug.md §10): the ONE objective-weight parser, exposed for tools.
// Reads DD_ALPHA..DD_DELTA (finite, non-negative, fully consumed strings;
// throws std::invalid_argument otherwise).  tools/dd_benchmark.cpp MUST
// use this instead of a private parser.
using DDSocWeights = SolverWeights;
DDSocWeights dd_load_soc_weights();
PlanCost dd_plan_cost_probe(const DDInstance& ins, const DDPlan& plan);
PlanCost dd_plan_cost_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan);
std::optional<PlanCost> dd_plan_cost_deadline_probe(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline, bool* cutoff);
bool dd_plan_cost_better_probe(const PlanCost& candidate,
                               const PlanCost& incumbent);
std::optional<DDPlan> dd_normalize_goal_prefix_probe(
    const DDInstance& ins, const DDPlan& plan);
std::optional<DDPlan> dd_normalize_goal_prefix_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan);
std::optional<DDPlan> dd_normalize_goal_prefix_deadline_probe(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline, bool* cutoff);
std::optional<std::pair<PhysConfig, PlanCost>>
dd_replay_raw_prefix_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan);
bool dd_replay_raw_prefix_deadline_probe(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline, bool* cutoff);
std::optional<TAPFReferencePlan> dd_build_reference_plan_probe(
    const DDInstance& ins, const DDPlan& plan,
    size_t max_checkpoints = 256);
std::optional<TAPFReferencePlan> dd_build_reference_plan_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan, size_t max_checkpoints = 256);
std::optional<DDInstance> dd_fixed_goal_instance_from_plan_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan);
std::optional<TAPFReferenceCheckpoint> dd_reference_checkpoint_probe(
    const TAPFReferencePlan& reference, const PhysConfig& state);
std::optional<DDPlan> dd_reference_splice_probe(
    const DDInstance& ins, const TAPFReferencePlan& reference,
    const DDPlan& prefix, const PlanCost& incumbent);

// Task-BR-PIBT Phase 1 probes. These are thin views over the same pure
// helpers used by production guidance; they do not run a second planner.
UpperSignature dd_upper_signature_probe(const PhysConfig& X);
PairPlan dd_pair_cost_probe(const DDInstance& ins, const PhysConfig& X,
                            int target, int goal);
struct DDLazyTauProbe {
  std::vector<int> tau;
  PairCostTable table;
  long evaluated_edges = 0;
  long total_edges = 0;
};
DDLazyTauProbe dd_lazy_tau_guide_probe(const DDInstance& ins,
                                       const PhysConfig& X);
std::optional<TaskId> dd_pair_next_ready_effect_probe(
    const DDInstance& ins, const PhysConfig& X, int target, int goal,
    int recursion_cap = 256);
double dd_pair_episode_cost_probe(
    const std::vector<ShelfSelector>& shifted_shelves, double alpha,
    double gamma, double delta);
std::vector<int> dd_tau_guide_probe(const DDInstance& ins,
                                    const PhysConfig& X);
double dd_tau_lb_probe(const DDInstance& ins, const PhysConfig& X);
int64_t dd_makespan_lb_probe(const DDInstance& ins, const PhysConfig& X);
ShelfTaskGraph dd_compile_single_root_graph_probe(
    const DDInstance& ins, const PhysConfig& X, int target, int goal,
    int recursion_cap = 256, int backtrack_cap = 512);
ShelfTaskGraph dd_compile_joint_graph_probe(
    const DDInstance& ins, const PhysConfig& X,
    const std::vector<int>* tau_override = nullptr,
    const std::vector<int>* priority_override = nullptr,
    int recursion_cap = 256, int backtrack_cap = 512);
std::vector<int> dd_ready_tasks_probe(const DDInstance& ins,
                                      const PhysConfig& X,
                                      const ShelfTaskGraph& graph);
ShelfTaskGraph dd_propagate_root_demands_probe(
    ShelfTaskGraph graph, const std::vector<int>& target_priority);
bool dd_task_effects_conflict_probe(const TaskId& a, const TaskId& b);
CarrierGuidance dd_task_br_guidance_probe(
    const DDInstance& ins, const PhysConfig& X,
    const PhysConfig* previous_X = nullptr,
    const CarrierGuidance* previous_guidance = nullptr,
    const std::vector<Op>* executed_ops = nullptr);
CarrierGuidance dd_task_br_cached_guidance_probe(
    const DDInstance& ins, const PhysConfig& X,
    const std::vector<PhysConfig>& warmups, long* cache_hits);
DDReadyMatchProbe dd_match_ready_tasks_probe(
    const DDInstance& ins, const PhysConfig& X,
    const ShelfTaskGraph& graph, const std::vector<int>& ready_tasks,
    const std::vector<std::optional<TaskId>>* previous_rho_task_id,
    DispatchMode mode = DispatchMode::EXECUTE,
    const std::vector<std::optional<TransferKey>>*
        previous_rho_transfer_key = nullptr,
    CandidateAdmission admission =
        CandidateAdmission::DROP_GLOBALLY_UNREACHABLE,
    const Deadline* deadline = nullptr,
    const std::vector<uint8_t>* eligible_robot = nullptr);

// Compatibility wrapper: authoritative callers that must distinguish a
// zero-tick success from failure use solve_carrier_lacam_result().
// On failure, if best_effort != nullptr it receives the action sequence to
// the deepest explored node (debug/rollout aid).
DDPlan solve_carrier_lacam(const DDInstance& ins, double time_limit_sec,
                           int seed, DDStats* stats = nullptr,
                           DDPlan* best_effort = nullptr);
DDSolveResult solve_carrier_lacam_result(
    const DDInstance& ins, double time_limit_sec, int seed,
    DDStats* stats = nullptr, DDPlan* best_effort = nullptr);
DDSolveResult solve_carrier_lacam_from_state_result(
    const DDInstance& ins, const PhysConfig& current,
    double time_limit_sec, int seed, DDStats* stats = nullptr,
    DDPlan* best_effort = nullptr);

enum class DDCommitStatus {
  OK = 0,
  NO_SOLVED_PLAN = 1,
  INVALID_PREFIX = 2,
  STATE_MISMATCH = 3,
};

enum class DDRebaseStatus {
  OK = 0,
  INVALID_STATE = 1,
};

// Persistent Carrier planning session. It retains only dependency-safe
// guidance/cache state and copied root continuation values; every solve still
// constructs a normal TAPFPlanner and calls the one TAPFPlanner::solve() loop.
class DDPlanningSession {
 public:
  DDPlanningSession(
      const DDInstance& ins, const PhysConfig& initial,
      int seed);
  ~DDPlanningSession();
  DDPlanningSession(DDPlanningSession&&) noexcept;
  DDPlanningSession& operator=(DDPlanningSession&&) noexcept;
  DDPlanningSession(const DDPlanningSession&) = delete;
  DDPlanningSession& operator=(const DDPlanningSession&) = delete;

  DDSolveResult solve(
      double time_limit_sec, DDStats* stats = nullptr,
      DDPlan* best_effort = nullptr);
  DDCommitStatus commit_prefix(
      size_t executed_steps, const PhysConfig& observed);
  DDRebaseStatus rebase(const PhysConfig& observed);
  const RootGoalCommitment& root_goal_commitment() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Diagnostic-only fixed-assignment seam.  It narrows each target's
// eligible set to the supplied tau, then calls the unchanged production
// carrier controller and TAPFPlanner search.
DDSolveResult dd_solve_carrier_lacam_fixed_tau_probe(
    const DDInstance& ins, const std::vector<int>& tau,
    double time_limit_sec, int seed, DDStats* stats = nullptr);

// Carrier BR-LaCAM decomposition baseline.  One upper shelf plan is
// compiled into immutable task waves.  Every lower solve stops at the
// first task-completion Drop; all still-free robots and all remaining
// PENDING tasks are then matched again, while already-Lifted pairs stay
// hard-locked.
DDSolveResult solve_carrier_brd_result(
    const DDInstance& ins, double time_limit_sec, int seed,
    DDStats* stats = nullptr, CarrierBRDStats* brd_stats = nullptr);

// Protected-test seam over the production completion-event controller.
// It skips only tau/upper compilation and executes the supplied verified
// frozen plan through the same matcher and TAPFPlanner::solve() path.
DDSolveResult dd_execute_frozen_task_plan_probe(
    const DDInstance& ins, const FrozenTaskPlan& frozen,
    const std::vector<int>& tau, double time_limit_sec, int seed,
    DDStats* stats = nullptr, CarrierBRDStats* brd_stats = nullptr);

// Test-visible finalization classifier shared by the production return
// path.  Invalid output is a correctness failure even when discovered
// near a deadline; only a valid output completed too late is DEADLINE.
enum class DDFinalizationStatus { ACCEPT, INVALID, DEADLINE };
DDFinalizationStatus dd_classify_finalization_probe(
    bool replay_valid, double elapsed_ms, double limit_ms);

// Remove exact physical-state loops, then remove loops in the grounded
// shelf projection.  A projected cut reconnects labeled robots on the
// lower deck while shelves remain fixed.  This is a semantics-preserving
// plan normalization, not a search option; invalid/non-improving repairs
// fall back to the original segment.
// R1 (debug.md §10): repair runs INSIDE the owning pass's deadline; when
// `deadline` is given and expires, the repair aborts and returns the raw
// plan unchanged.
DDPlan repair_carrier_plan(const DDInstance& ins, const DDPlan& plan,
                           DDPlanRepairStats* stats = nullptr,
                           const Deadline* deadline = nullptr);
DDPlan repair_carrier_plan(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan, DDPlanRepairStats* stats = nullptr,
    const Deadline* deadline = nullptr);
// Test-visible view of the exact production repair acceptance boundary.
// Both plans are replayed with the shared fixed-point weights; only a valid
// goal candidate with strictly smaller (ticks, work) is accepted.
bool dd_repair_accepts_candidate_probe(
    const DDInstance& ins, const DDPlan& incumbent,
    const DDPlan& candidate);
// Production fast path: `states` is the already-materialized physical
// replay parallel to `plan` (size = plan.size() + 1).  The repaired output
// is still replayed independently before acceptance; this only avoids
// replaying a very long raw incumbent twice.
DDPlan repair_carrier_plan_from_replay(
    const DDInstance& ins, const DDPlan& plan,
    const std::vector<PhysConfig>& states,
    DDPlanRepairStats* stats = nullptr,
    const Deadline* deadline = nullptr);
DDPlan repair_carrier_plan_from_replay(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan, const std::vector<PhysConfig>& states,
    DDPlanRepairStats* stats = nullptr,
    const Deadline* deadline = nullptr);

// B0 baseline (design 8.1) = Carrier-PIBT standalone: repeatedly apply the
// unconstrained generator from the current configuration until goal, dead
// end, or timeout.  No high-level search.  Shares the rollout core with the
// macro successor (design 7.1, D13).
DDPlan solve_carrier_rollout(const DDInstance& ins, double time_limit_sec,
                             int seed, DDStats* stats = nullptr);

// admissible SOC-style heuristic at the initial configuration (design
// 5.7 h_soc): test oracle for the production f-bound (the planner folds
// the same term into node h via attach_carrier_guidance).
double dd_root_admissible_h(const DDInstance& ins);

// B1 baseline (design 8.1): 2-stage — shelf paths fixed ONCE at the start
// configuration (stage 1), then executed by Carrier-PIBT with the fixed
// plan as a hard constraint (stage 2; requests derive from each target's
// next fixed waypoint).  Deliberately NOT complete: quantifies what the
// unified per-configuration replanning buys.  If fixed_paths != nullptr it
// receives the stage-1 plan (per target, cell sequence).
DDPlan solve_carrier_2stage(const DDInstance& ins, double time_limit_sec,
                            int seed, DDStats* stats = nullptr,
                            std::vector<std::vector<int>>* fixed_paths = nullptr);

// TEST SUPPORT (G1/completeness conformance, debug.md P0-1/P0-2):
// drain ONE node's operator-constraint tree through the production
// machinery (tree expansion + Carrier-PIBT + validator) and return every
// distinct successor configuration it can produce.  Used to compare against
// brute-force validator enumeration.
std::vector<PhysConfig> dd_enumerate_node_successors(const DDInstance& ins,
                                                     const PhysConfig& X,
                                                     int seed);

// TEST SUPPORT: run one unconstrained
// Carrier-PIBT step from X and return the chosen joint ops.
std::vector<Op> dd_root_joint_ops(const DDInstance& ins, const PhysConfig& X,
                                  int seed);
