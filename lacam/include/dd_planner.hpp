/*
 * dd_planner: carrier (two-deck) entry adapters and test-support probes
 * over the integrated LaCAM-TAPF planner (design.md v3 section 10).
 * The ONE solve loop lives in tapf_planner.cpp; nothing here searches.
 */
#pragma once

#include <optional>
#include <vector>

#include "dd_carrier.hpp"
#include "tapf_planner.hpp"

using DDPlan = std::vector<std::vector<Op>>;  // per timestep, per robot

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
  long generator_failures = 0;  // Carrier-PIBT returned no joint op
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
  long ready_task_count = 0;
  long rho_repairs = 0;
  long custody_continuations = 0;
  long zero_empty_no_ready = 0;
  long rewire_guidance_rebuilds = 0;
  long g_relaxed = 0;         // duplicate hits relaxed to a cheaper g
                              // (generic search-kernel diagnostic)
  long guidance_builds = 0;
  double tau_time_ms = 0;
  double guidance_time_ms = 0;
  double deliverable_ms = -1;  // deferred cleanup + final replay complete
  // Incumbent and output-repair cost diagnostics.
  double first_solution_ms = -1;
  double first_solution_soc = -1;  // weighted physical cost (a=b=g=d=1)
  double best_soc = -1;
  long incumbent_updates = 0;
  long f_pruned = 0;
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
struct DDSocWeights {
  double alpha = 1, beta = 1, gamma = 1, delta = 1;
};
DDSocWeights dd_load_soc_weights();

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
    const std::vector<std::optional<TaskId>>* previous_rho_task_id);

// returns empty plan on failure/timeout; a trivially-solved instance yields
// a single all-wait step (never an empty plan on success).
// On failure, if best_effort != nullptr it receives the action sequence to
// the deepest explored node (debug/rollout aid).
DDPlan solve_carrier_lacam(const DDInstance& ins, double time_limit_sec,
                           int seed, DDStats* stats = nullptr,
                           DDPlan* best_effort = nullptr);

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
struct Deadline;
DDPlan repair_carrier_plan(const DDInstance& ins, const DDPlan& plan,
                           DDPlanRepairStats* stats = nullptr,
                           const Deadline* deadline = nullptr);
// Production fast path: `states` is the already-materialized physical
// replay parallel to `plan` (size = plan.size() + 1).  The repaired output
// is still replayed independently before acceptance; this only avoids
// replaying a very long raw incumbent twice.
DDPlan repair_carrier_plan_from_replay(
    const DDInstance& ins, const DDPlan& plan,
    const std::vector<PhysConfig>& states,
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
