/*
 * LaCAM-style TAPF planner.
 */
#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "planner.hpp"
#include "tapf_assignment.hpp"

enum class TAPFSearchMode {
  DFS = 0,
  FOCAL = 1,
};

enum class TAPFFocalTieBreak {
  H = 0,
  ANTI_WAIT = 1,
  ANTI_ZIGZAG = 2,
  ANTI_PUSH = 3,
  ANTI_ALL = 4,
};

struct TAPFSearchConfig {
  TAPFSearchMode mode = TAPFSearchMode::DFS;
  TAPFFocalTieBreak focal_tie_break = TAPFFocalTieBreak::H;
  double focal_weight = 1.5;
  // Search-kernel controls. Carrier production uses one macro-assisted
  // first-incumbent pass; generic TAPF callers may still run anytime.
  bool macro_enabled = true;   // event-bounded rollout successors
  bool stop_at_first = false;
  // Carrier adapters may need to replay/finalize a found plan before the
  // large CLOSED tree is destroyed.  Cleanup is still owned by the planner
  // and runs in its destructor; only the lifetime ordering changes.
  bool defer_cleanup = false;
  double incumbent_init = -1;  // optional external upper bound
};

// skeleton dedup (node-skeleton audit 2026-08-30): TAPFConstraint was a
// byte-identical twin of planner.hpp's Constraint — now ONE type.
using TAPFConstraint = Constraint;

// Two-deck shelf layer of a search state (design.md 3.1, mapping M2).
// EMPTY vectors on shelf-free instances: every consumer loops over the
// data, so degradation to plain TAPF is structural, never a flag.
struct ShelfState {
  std::vector<int> target_pos;  // per target: current cell
  std::vector<int> anon_occ;    // SORTED cells of grounded anonymous shelves
  std::vector<int> kappa;       // per robot (empty when no shelf layer):
                                // KAPPA_FREE / KAPPA_ANON / target index

  bool operator==(const ShelfState& o) const
  {
    return target_pos == o.target_pos && anon_occ == o.anon_occ &&
           kappa == o.kappa;
  }
};

// root shelf state of an instance (empty layer for shelf-free TAPF)
ShelfState initial_shelf_state(const TAPFInstance& ins);

// ---- Task-BR-PIBT carrier guidance types (design_final §§2-5) ----
// These values are ordering metadata only and never enter SearchKey.
struct UpperSignature {
  std::vector<int> target_pos;
  std::vector<int> anon_pos;

  bool operator==(const UpperSignature& o) const
  {
    return target_pos == o.target_pos && anon_pos == o.anon_pos;
  }
  bool operator!=(const UpperSignature& o) const { return !(*this == o); }
  bool operator<(const UpperSignature& o) const
  {
    return target_pos != o.target_pos ? target_pos < o.target_pos
                                      : anon_pos < o.anon_pos;
  }
};

struct PairPlan {
  double estimated_cost = 0;
  int rollout_steps = 0;
  int direct_distance = -1;
  bool reached_goal = false;
  bool truncated = false;
  bool stalled = false;
  bool exact = true;
};

struct PairCostEntry {
  int goal = -1;
  PairPlan plan;
};

using PairCostTable = std::vector<std::vector<PairCostEntry>>;

struct ShelfSelector {
  enum class Kind : uint8_t { TARGET, ANON_AT_EPOCH_CELL };
  Kind kind = Kind::TARGET;
  int value = -1;

  bool operator==(const ShelfSelector& o) const
  {
    return kind == o.kind && value == o.value;
  }
  bool operator!=(const ShelfSelector& o) const { return !(*this == o); }
  bool operator<(const ShelfSelector& o) const
  {
    return kind != o.kind ? kind < o.kind : value < o.value;
  }
};

struct RootDemand {
  int target = -1;
  int goal = -1;

  bool operator==(const RootDemand& o) const
  {
    return target == o.target && goal == o.goal;
  }
  bool operator<(const RootDemand& o) const
  {
    return target != o.target ? target < o.target : goal < o.goal;
  }
};

struct TaskId {
  ShelfSelector shelf;
  int from = -1;
  int to = -1;

  bool operator==(const TaskId& o) const
  {
    return shelf == o.shelf && from == o.from && to == o.to;
  }
  bool operator!=(const TaskId& o) const { return !(*this == o); }
  bool operator<(const TaskId& o) const
  {
    if (shelf != o.shelf) return shelf < o.shelf;
    return from != o.from ? from < o.from : to < o.to;
  }
};

struct TaskIdHash {
  size_t operator()(const TaskId& id) const
  {
    auto mix = [](uint64_t x) {
      x += 0x9e3779b97f4a7c15ULL;
      x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
      x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
      return x ^ (x >> 31);
    };
    uint64_t h = mix((uint64_t)id.shelf.value + 3);
    h ^= mix(((uint64_t)id.shelf.kind << 60) ^
             (uint64_t)(uint32_t)id.from);
    h ^= mix(((uint64_t)1 << 58) ^ (uint64_t)(uint32_t)id.to);
    return (size_t)h;
  }
};

struct ShelfTask {
  TaskId id;
  std::vector<RootDemand> roots;
  int priority = 0;
};

struct RotationCandidate {
  std::vector<TaskId> cycle;
};

struct ShelfTaskGraph {
  std::vector<ShelfTask> tasks;
  std::vector<std::vector<int>> predecessors;
  std::vector<std::vector<int>> successors;
  std::vector<int> paused_roots;
  std::vector<RotationCandidate> rotations;
  long effect_conflicts = 0;
  long candidate_backtracks = 0;
};

struct Custody {
  TaskId task_id;
  std::optional<int> current_task_index;
  ShelfSelector shelf;
  int from = -1;
  int to = -1;
  std::vector<RootDemand> roots;
  int priority = 0;
};

struct UpperEpochGuidance {
  UpperSignature upper_signature;
  PairCostTable pair_cost;
  long pair_edges_evaluated = 0;
  long pair_edges_total = 0;
  long pair_rollout_work_steps = 0;
  long pair_rollout_truncations = 0;
  long pair_rollout_stalls = 0;
  std::vector<int> tau_guide;
  std::vector<int> priority_commitment;
  std::vector<int> target_priority;
  ShelfTaskGraph task_graph;
};

struct DDReadyMatchProbe {
  std::vector<std::optional<TaskId>> rho_task_id;
  std::vector<int> rho_ready_index;
};

struct CarrierGuidance {
  // Task-BR-PIBT guidance.  `upper_epoch` is immutable and may be shared
  // across robot-only transitions; all remaining fields are rebuilt from
  // the current physical state and one real parent transition.
  std::shared_ptr<const UpperEpochGuidance> upper_epoch;
  std::vector<int> ready_tasks;
  std::vector<std::optional<TaskId>> rho_task_id;
  std::vector<int> rho_ready_index;
  std::vector<std::optional<Custody>> custody_by_robot;
};

struct TAPFNode;

struct TransitionStep {
  PhysConfig previous_X;
  std::vector<Op> ops;
  PhysConfig next_X;

  bool operator==(const TransitionStep& o) const
  {
    return previous_X == o.previous_X && ops == o.ops &&
           next_X == o.next_X;
  }
};

struct SearchEdge {
  TAPFNode* to = nullptr;
  double physical_cost = 0;
  std::vector<TransitionStep> transition_trace;
};

using SearchEdgeHandle = std::shared_ptr<const SearchEdge>;

struct TAPFNode : LacamNodeCore<TAPFConstraint, TAPFNode> {
  const ShelfState shelf;  // two-deck layer (empty on shelf-free instances)
  // FROZEN variable order for constraint-tree expansion (design D11,
  // mapping M3): copied from `order` at creation.  Livelock handling (M9)
  // may perturb `order` (PIBT preference) but never this.
  std::vector<int> constraint_order;
  // Carrier guidance is null on shelf-free instances.
  std::unique_ptr<CarrierGuidance> guide;
  long h_guidance = 0;
  bool macro_tried = false;
  // Set when duplicate-hit relaxation selects a different incoming edge.
  // The exact transition trace re-anchors guidance lazily at expansion.
  bool guidance_stale = false;
  SearchEdgeHandle incoming_edge;
  std::vector<SearchEdgeHandle> outgoing_edges;
  std::vector<int> assignment;
  TAPFAssignmentState assignment_state;
  bool queued;
  double g;
  double h;
  double f;
  unsigned depth;
  unsigned non_goal_waits;
  unsigned reversals;
  unsigned distance_increases;
  unsigned settled_pushes;

  TAPFNode(Config _C, ShelfState _shelf, TAPFDistTable& D,
           const TAPFInstance* ins, std::vector<int> _assignment,
           TAPFAssignmentState _assignment_state, TAPFNode* _parent = nullptr);
  void discard_search_tree();
  void refresh_priority(TAPFDistTable& D);
  void refresh_search_metrics(TAPFDistTable& D, const TAPFInstance* ins);
};

struct TAPFStats {
  int hl_loop_iterations = 0;
  int hl_nodes_created = 0;
  int hl_nodes_explored = 0;
  int hl_reinsertions = 0;
  int hl_duplicate_configs = 0;
  int open_max_size = 0;
  int solution_depth = 0;
  int constraints_popped = 0;
  int constraints_generated = 0;
  int constraint_failures = 0;
  int pibt_calls = 0;
  int pibt_failures = 0;
  int pibt_recursions = 0;
  int assignment_calls = 0;
  int assignment_changes = 0;
  int final_assignment_changes = 0;
  int final_agent_assignment_changes = 0;
  int anytime_cost_updates = 0;
  int incumbent_updates = 0;
  int swap_checks = 0;
  int swap_applied = 0;
  // carrier layer (M4): joint ops rejected by the conformance-oracle
  // arbiter (0 on shelf-free instances — the arbiter is never invoked)
  int carrier_validator_rejects = 0;
  int carrier_g1_rejects = 0;  // fully-constrained combos the oracle
                               // rejected (expected: exhaustive trees)
  // macro rollout (design 7.1/D14, M10); all 0 on shelf-free instances
  long macro_successors = 0;
  long macro_steps = 0;
  long macro_after_first = 0;  // macro insertion is pre-incumbent only
  long macro_shelf_motion_successors = 0;
  long macro_robot_only_successors = 0;
  long rollout_calls = 0;
  long rollout_cycles = 0;
  long rollout_shelf_motion_steps = 0;
  // New-state transitions split by their highest-level physical effect.
  long robot_only_successors = 0;
  long manipulation_successors = 0;  // lift/drop, no shelf displacement
  long shelf_motion_successors = 0;
  // Task-BR-PIBT guidance diagnostics.
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
  long f_pruned = 0;    // nodes discarded by the incumbent/f bound
  long g_relaxed = 0;   // duplicate-hit g relaxations (rewrite propagation)
  long guidance_builds = 0;  // carrier guidance constructions (M6)
  double tau_time_ms = 0;
  double guidance_time_ms = 0;
  unsigned solution_cost = 0;
  unsigned first_solution_cost = 0;
  double first_solution_g = -1;  // exact double (carrier weighted soc)
  unsigned solution_parent_edge_cost = 0;
  double assignment_time_ms = 0;
  double first_solution_time_ms = 0;
  bool timed_out = false;
};

struct TAPFPlanner {
  const TAPFInstance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;
  const int sticky_penalty;
  const float restart_rate;
  const bool anytime;
  const TAPFSearchConfig search_config;
  bool force_full_assignment;
  TAPFStats* stats;
  TAPFAssignmentStats assignment_stats;

  const int N;  // number of agents
  const int V_size;
  // Physical cost weights (design 2.3/5.7, mapping M5): unit by default;
  // numeric objective inputs DD_ALPHA..DD_DELTA override them. Shelf-free
  // edge costs never read these (the carrier term loops over empty kappa).
  struct Weights {
    double alpha = 1, beta = 1, gamma = 1, delta = 1;
  };
  Weights weights;
  TAPFDistTable D;
  Candidates C_next;
  std::vector<float> tie_breakers;
  Agents A;
  Agents occupied_now;
  Agents occupied_next;

  // ---- carrier layer (M2/M4): allocated only when shelves exist ----
  // conformance-oracle view of the instance (final arbiter of every joint
  // op with a shelf layer, exactly the pre-integration architecture)
  std::unique_ptr<DDInstance> dd_view;
  // guidance engine (M6): distance caches, path cache, occupancy scratch
  struct CarrierEngine;
  std::unique_ptr<CarrierEngine> carrier;
  // per-node occupancy scratch: carrier_grounded doubles as the grounded
  // upper-deck occupancy at t+1 (0 none / -1 anon / b+1 target b);
  // delta counters hold tentative carried-shelf reservations
  const TAPFNode* carrier_scratch_node = nullptr;
  std::vector<int> carrier_grounded;
  std::vector<int> carrier_upper_delta;
  std::vector<int> carrier_upper_touched;
  ShelfState shelf_next_scratch;        // successor layer of the last gen
  std::vector<Op> ops_scratch;          // assembled joint op
  // funcPIBT op-candidate scratch, PER AGENT (funcPIBT recurses via
  // priority inheritance — a shared buffer would be clobbered mid-loop;
  // same recursion-safety shape as C_next)
  std::vector<std::vector<std::pair<Vertex*, uint8_t>>> pibt_cand;
  // solution shelf chain (parallel to the returned Solution; empty layers
  // on shelf-free instances) — consumed by the carrier adapters/tests
  std::vector<ShelfState> solution_shelves;
  std::vector<TAPFNode*> deferred_cleanup_nodes;
  // deepest explored node (carrier debug/best-effort, M16); the chain is
  // extracted into best_effort_* when a carrier solve ends without a plan
  const TAPFNode* deepest_node = nullptr;
  unsigned deepest_depth = 0;
  long best_targets_done = 0;
  Solution best_effort_solution;
  std::vector<ShelfState> best_effort_shelves;
  std::vector<int> best_effort_tau;
  // unconstrained rollout from a configuration (design 7.1/D13, M10):
  // the SHARED core of the macro successor and the B0 baseline.  Stops on
  // goal, lift/drop event (after min_chunk, when stop_on_event), step cap,
  // local cycle, or generator failure.
  struct CarrierRollout {
    std::vector<std::vector<Op>> ops;   // per executed step
    std::vector<Config> configs;       // states incl. start (ops.size()+1)
    std::vector<ShelfState> shelves;
    std::shared_ptr<const CarrierGuidance> terminal_guidance;
    std::vector<int> terminal_order;
    double terminal_h = 0;
    long terminal_h_guidance = 0;
    double cost = 0;
    bool reached_goal = false;
    bool shelf_moved = false;
  };
  CarrierRollout carrier_rollout(const Config& C0, const ShelfState& S0,
                                 int max_steps, int min_chunk,
                                 bool stop_on_event,
                                 const TAPFNode* initial_anchor = nullptr);

  TAPFPlanner(const TAPFInstance* _ins, const Deadline* _deadline,
              std::mt19937* _MT, int _verbose = 0, int _sticky_penalty = 0,
              float _restart_rate = 0.001f, bool _anytime = true,
              TAPFStats* _stats = nullptr,
              TAPFSearchConfig _search_config = TAPFSearchConfig());
  ~TAPFPlanner();  // out-of-line: CarrierEngine is defined in the .cpp
  Solution solve();
  // Build per-node Task-BR guidance.  A root attach has no transition
  // context; every non-root attach receives the exact adjacent transition.
  // Shelf LB and frozen constraint order are installed only on first attach.
  void attach_carrier_guidance(
      TAPFNode* nd,
      const PhysConfig* transition_previous_X = nullptr,
      const CarrierGuidance* transition_previous_guidance = nullptr,
      const std::vector<Op>* transition_executed_ops = nullptr);
  void ensure_guidance_fresh(TAPFNode* nd);
  // operator-candidate construction for the lazy constraint tree (M3):
  // the ONE production implementation, shared by solve() and the G1
  // conformance enumeration adapters.  rng consumption identical to the
  // pre-integration vertex shuffle on shelf-free instances.
  void build_op_candidates(TAPFNode* S, int i, std::vector<OpCand>& out);
  bool is_goal_config(const Config& C, const ShelfState& S) const;
  bool get_new_config(TAPFNode* S, TAPFConstraint* M);
  void rewrite(TAPFNode* from, TAPFNode* goal,
               std::vector<TAPFNode*>& OPEN);
  SearchEdgeHandle register_outgoing_edge(
      TAPFNode* from, TAPFNode* to, double physical_cost,
      const std::vector<TransitionStep>& transition_trace);
  double get_edge_cost(const TAPFNode* from, const TAPFNode* to) const;
  double get_h_value(const Config& C);
  // carrier helpers (M4); all no-ops / trivially true with an empty layer
  void refresh_carrier_scratch(const TAPFNode* S);
  // address-keyed scratches (occupancy + guidance occ view) must be
  // dropped whenever node addresses may be recycled (rollout probes)
  void invalidate_carrier_scratch();
  bool carrier_upper_taken(int cell) const;
  void carrier_upper_add(int cell);
  void carrier_upper_sub(int cell);
  bool forced_op_feasible(const TAPFNode* S, int i, Vertex* v, uint8_t kind);
  bool apply_carrier_effects(const TAPFNode* S);
  Agent* swap_possible_and_required(Agent* ai,
                                    const std::vector<int>& assignment);
  bool is_swap_required(const int pusher, const int puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin,
                        const std::vector<int>& assignment);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin,
                        const std::vector<int>& assignment);
  bool funcPIBT(Agent* ai, const std::vector<int>& assignment);
};

Solution solve_tapf(const TAPFInstance& ins, const int verbose = 0,
                    const Deadline* deadline = nullptr,
                    std::mt19937* MT = nullptr, const int sticky_penalty = 0,
                    TAPFStats* stats = nullptr, bool anytime = true,
                    bool force_full_assignment = false,
                    TAPFSearchConfig search_config = TAPFSearchConfig());

// Derive the per-timestep joint primitive ops from a solved carrier
// chain (M3): consecutive (Config, ShelfState) pairs uniquely determine
// WAIT / MOVE / LIFT / DROP per robot (kappa is part of the state).
std::vector<std::vector<Op>> derive_carrier_ops(
    const TAPFInstance& ins, const Solution& sol,
    const std::vector<ShelfState>& shelves);
