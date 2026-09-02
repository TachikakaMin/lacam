/*
 * LaCAM-style TAPF planner.
 */
#pragma once

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
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

// ---- carrier guidance (design 5.3/5.4a/5.5, mapping M6/M8) ----
// Per-node, ordering-only; NEVER part of the search key.  Absent (null)
// on shelf-free instances.
struct CarrierRequest {
  int cell = -1;     // shelf cell to lift (serve: target pos; clear: blocker)
  int priority = 0;  // higher first (100 serve; 50-k clear chain)
};

// Manipulation task (design_final v3.0 §3): one explicit shelf state
// change MoveShelf(s, from -> to, root = b -> g).  Labeled targets carry
// their index in shelf_target; anonymous shelves are identified by their
// current cell (equivalence-class semantics, custody tracked by the
// assigned robot's kappa episode).  Ordering-only guidance metadata:
// never part of the search key.
struct ManipulationTask {
  int shelf_target = -1;  // >=0: labeled target index; -1: anon (cell id)
  int from = -1;          // precise pickup cell (shelf's current cell)
  int to = -1;            // drop cell; -1 while the destination is still
                          // carrier-chosen parking (pre-frontier-compiler)
  int root_target = -1;   // which shelf-goal objective this serves
  int root_goal = -1;
  int priority = 0;       // rho ordering (100 serve; 50-k clear chain)
  int depth = 0;          // dependency depth from the root objective
  // Drop-cell semantics (R2): `to_committed` marks a COMPILED drop
  // commitment (one-empty ready tasks: to = the vacancy).  Un-committed
  // `to` values are advisory hints (guidance cost only); the task
  // delegates the drop choice to the carrier per node (parking).
  bool to_committed = false;
  uint64_t id = 0;        // stable TaskId: hash of (shelf, from, root);
                          // `to` excluded so identity survives frontier
                          // refinement of the drop cell
};

struct CarrierGuidance {
  std::vector<CarrierRequest> requests;
  // v3.0 task pool: requests are a PROJECTION of tasks (request k is
  // task k's pickup view); rho binds tasks, request indices follow.
  std::vector<ManipulationTask> tasks;
  // v3.0 §6 custody: the task a loaded-ANON robot is executing (copied at
  // Lift from the parent binding; kept while carrying; cleared on Drop).
  // id == 0 means no custody.  Labeled targets need no custody: kappa
  // already names the shelf and tau names its goal.
  std::vector<ManipulationTask> custody;
  std::vector<int> rho;           // robot -> request index (or -1)
  std::vector<int> rho_task;      // robot -> task pool index (or -1)
  std::vector<int> free_goal;     // per-robot request cell (or -1)
  std::vector<int> parking_cell;  // for parked/anon carriers
  std::vector<int> target_next;   // next cell on each target's path
  bool plan_bound = false;        // B1: fixed plan is a HARD constraint
  std::vector<uint8_t> target_park;  // design 5.4a park flags
  // shelf->goal matching (design_final 5.3, T3): per-target assigned
  // goal CELL.  Identity view on fixed-goal (singleton) instances.
  std::vector<int> tau;
};

struct TAPFNode : LacamNodeCore<TAPFConstraint, TAPFNode> {
  const ShelfState shelf;  // two-deck layer (empty on shelf-free instances)
  // FROZEN variable order for constraint-tree expansion (design D11,
  // mapping M3): copied from `order` at creation.  Livelock handling (M9)
  // may perturb `order` (PIBT preference) but never this.
  std::vector<int> constraint_order;
  // carrier guidance + livelock signals (M6/M9); guide is null and the
  // counters stay 0 on shelf-free instances
  std::unique_ptr<CarrierGuidance> guide;
  long h_guidance = 0;
  long best_h = 0;
  int no_progress = 0;
  int revisits = 0;
  bool macro_tried = false;
  // v3.0 §6.1: set when a duplicate-hit rewire reparents this node; the
  // guidance is re-anchored LAZILY at the next expansion (rebuilding on
  // every relax is unbounded in anytime searches).
  bool guidance_stale = false;
  std::set<TAPFNode*> neighbor;
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
  // Carrier assignment stability. Pair counts are more informative than
  // generic TAPF assignment_changes, whose carrier rows are all unbound.
  long tau_change_builds = 0;
  long tau_pair_changes = 0;
  long rho_change_builds = 0;
  long rho_pair_changes = 0;
  // v3.0 step 3 diagnostics: execution-price tau repairs and
  // duplicate-rewire guidance rebuilds (design_final §5.1/§6.1).
  long tau_price_repairs = 0;
  long rewire_guidance_rebuilds = 0;
  long f_pruned = 0;    // nodes discarded by the incumbent/f bound
  long g_relaxed = 0;   // duplicate-hit g relaxations (rewrite propagation)
  long guidance_builds = 0;  // carrier guidance constructions (M6)
  double tau_time_ms = 0;
  double guidance_time_ms = 0;
  long futile_lift_demotions = 0;
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
  // deepest explored node (carrier debug/best-effort, M16); the chain is
  // extracted into best_effort_* when a carrier solve ends without a plan
  const TAPFNode* deepest_node = nullptr;
  unsigned deepest_depth = 0;
  long best_targets_done = 0;
  Solution best_effort_solution;
  std::vector<ShelfState> best_effort_shelves;
  std::vector<int> best_effort_tau;
  // multi-step macro edges (M10), keyed (from, to): physical cost and the
  // INTERMEDIATE states (exclusive of endpoints) for extraction/rewrite.
  // Always empty on shelf-free instances.
  struct MacroEdge {
    double cost = 0;
    std::vector<Config> configs;
    std::vector<ShelfState> shelves;
  };
  std::map<std::pair<const TAPFNode*, const TAPFNode*>, MacroEdge>
      macro_edges;

  // unconstrained rollout from a configuration (design 7.1/D13, M10):
  // the SHARED core of the macro successor and the B0 baseline.  Stops on
  // goal, lift/drop event (after min_chunk, when stop_on_event), step cap,
  // local cycle, or generator failure.
  struct CarrierRollout {
    std::vector<std::vector<Op>> ops;   // per executed step
    std::vector<Config> configs;       // states incl. start (ops.size()+1)
    std::vector<ShelfState> shelves;
    double cost = 0;
    bool reached_goal = false;
    bool shelf_moved = false;
  };
  CarrierRollout carrier_rollout(const Config& C0, const ShelfState& S0,
                                 int max_steps, int min_chunk,
                                 bool stop_on_event);

  // Cross-node memory of repeated immediate lift/drop pairs.  Observation
  // and expiry are automatic; LIFT is only demoted in guidance, so the
  // exhaustive constraint tree is intact.
  long futile_clock = 0;
  std::unordered_map<uint64_t, std::pair<long, long>>
      lift_futile;  // key -> (recent count, last tick)
  void note_lift_cycle(const Config& before_lift_C,
                       const ShelfState& before_lift_S,
                       const Config& loaded_C, const ShelfState& loaded_S,
                       const Config& after_drop_C,
                       const ShelfState& after_drop_S);
  bool lift_on_cooldown(int cell) const;

  TAPFPlanner(const TAPFInstance* _ins, const Deadline* _deadline,
              std::mt19937* _MT, int _verbose = 0, int _sticky_penalty = 0,
              float _restart_rate = 0.001f, bool _anytime = true,
              TAPFStats* _stats = nullptr,
              TAPFSearchConfig _search_config = TAPFSearchConfig());
  ~TAPFPlanner();  // out-of-line: CarrierEngine is defined in the .cpp
  Solution solve();
  // guidance hook at node creation (M6/M9): builds requests/rho/park,
  // layers the PIBT order by carrier class, freezes constraint_order,
  // folds the admissible shelf h into node->h/f, and applies the
  // livelock diversification.  Immediate no-op without targets.
  // rollout_parent_guide supplies the eta-hysteresis ancestor for
  // parentless rollout probes.  rewire_rebuild (v3.0 §6.1, invariant 21)
  // re-anchors an EXISTING node's guidance on its NEW parent after a
  // duplicate-hit rewire: no taboo, no h/f or constraint_order changes.
  void attach_carrier_guidance(
      TAPFNode* nd, bool reguide = false,
      const CarrierGuidance* rollout_parent_guide = nullptr,
      bool rewire_rebuild = false);
  // operator-candidate construction for the lazy constraint tree (M3):
  // the ONE production implementation, shared by solve() and the G1
  // conformance enumeration adapters.  rng consumption identical to the
  // pre-integration vertex shuffle on shelf-free instances.
  void build_op_candidates(TAPFNode* S, int i, std::vector<OpCand>& out);
  bool is_goal_config(const Config& C, const ShelfState& S) const;
  bool get_new_config(TAPFNode* S, TAPFConstraint* M);
  void rewrite(TAPFNode* from, TAPFNode* goal,
               std::vector<TAPFNode*>& OPEN);
  double get_edge_cost(const TAPFNode* from, const TAPFNode* to) const;
  double get_h_value(const Config& C);
  // carrier helpers (M4); all no-ops / trivially true with an empty layer
  void refresh_carrier_scratch(const TAPFNode* S);
  // address-keyed scratches (occupancy + guidance occ view) must be
  // dropped whenever node addresses may be recycled (rollout probes)
  void invalidate_carrier_scratch();
  // path-cache diagnostics (engine-internal; 0 without a shelf layer)
  long carrier_path_recomputes() const;
  long carrier_path_cache_hits() const;
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
