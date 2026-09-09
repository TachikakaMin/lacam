/*
 * Shared TAPF/carrier data types (split from tapf_planner.hpp).
 */
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <type_traits>
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

enum class TAPFObjective {
  // Preserve the original LaCAM-TAPF scalar weighted-work objective.
  // Costs generated under this contract keep ticks at zero, so the one
  // PlanCost comparator naturally reduces to the historical ordering.
  LEGACY_WEIGHTED_WORK = 0,
  // Carrier-LaCAM v5 objective: minimize executed ticks, then weighted work.
  MAKESPAN_THEN_WORK = 1,
};

enum class TAPFStopPolicy {
  // Preserve the generic TAPF behavior: retain an incumbent and continue
  // while the caller's anytime/deadline contract permits.
  ANYTIME = 0,
  // Unbounded feasibility pass: stop at the first accepted goal.
  FIRST_FEASIBLE = 1,
  // Bounded improvement pass: accept only a goal strictly below the
  // external incumbent, then stop immediately.
  FIRST_STRICT_IMPROVEMENT = 2,
};

struct PlanCost {
  // Environment weights are accepted with up to micro-unit precision and
  // converted once to this exact integer domain.  Search ordering never uses
  // floating epsilon comparisons.
  static constexpr int64_t WORK_SCALE = 1000000;

  int64_t ticks = 0;
  int64_t work = 0;

  PlanCost() = default;
  PlanCost(int legacy_work)
      : PlanCost(from_values(0, static_cast<double>(legacy_work)))
  {
  }
  PlanCost(double legacy_work)
      : PlanCost(from_values(0, legacy_work))
  {
  }

  static PlanCost from_values(int64_t ticks, double work)
  {
    if (ticks < 0)
      throw std::invalid_argument("PlanCost ticks must be non-negative");
    if (!std::isfinite(work) || work < 0)
      throw std::invalid_argument(
          "PlanCost work must be finite and non-negative");
    const long double scaled =
        static_cast<long double>(work) * WORK_SCALE;
    if (scaled >
        static_cast<long double>(std::numeric_limits<int64_t>::max()))
      throw std::overflow_error("PlanCost work is out of range");
    const long double rounded = std::round(scaled);
    // Work is an exact fixed-point quantity.  The small tolerance only
    // absorbs binary floating conversion of a representable decimal such
    // as 0.1; it must never round a genuinely sub-micro coefficient.
    if (std::fabs(scaled - rounded) > 1e-6L)
      throw std::invalid_argument(
          "PlanCost work must be exactly representable at 1e-6 scale");
    return from_scaled(ticks, static_cast<int64_t>(rounded));
  }

  static PlanCost from_scaled(int64_t ticks, int64_t work)
  {
    if (ticks < 0 || work < 0)
      throw std::invalid_argument(
          "finite PlanCost components must be non-negative");
    return PlanCost(ticks, work, RawTag{});
  }

  static PlanCost legacy(double work) { return from_values(0, work); }

  static PlanCost unbounded()
  {
    return PlanCost(-1, -1, RawTag{});
  }

  bool is_bounded() const { return ticks >= 0; }

  double work_value() const
  {
    return is_bounded()
               ? static_cast<double>(work) /
                     static_cast<double>(WORK_SCALE)
               : -1.0;
  }

  // Compatibility reporting only.  Production ordering uses the operators
  // below; this conversion keeps historical scalar diagnostics/tests readable
  // while node costs migrate to PlanCost.
  operator double() const { return work_value(); }

  PlanCost& operator+=(const PlanCost& other)
  {
    *this = *this + other;
    return *this;
  }

  friend PlanCost operator+(const PlanCost& a, const PlanCost& b)
  {
    if (!a.is_bounded() || !b.is_bounded()) return unbounded();
    if (a.ticks > std::numeric_limits<int64_t>::max() - b.ticks ||
        a.work > std::numeric_limits<int64_t>::max() - b.work)
      throw std::overflow_error("PlanCost addition overflow");
    return from_scaled(a.ticks + b.ticks, a.work + b.work);
  }

  friend bool operator==(const PlanCost& a, const PlanCost& b)
  {
    return a.ticks == b.ticks && a.work == b.work;
  }
  friend bool operator!=(const PlanCost& a, const PlanCost& b)
  {
    return !(a == b);
  }
  friend bool operator<(const PlanCost& a, const PlanCost& b)
  {
    if (!a.is_bounded()) return false;
    if (!b.is_bounded()) return true;
    return a.ticks != b.ticks ? a.ticks < b.ticks : a.work < b.work;
  }
  friend bool operator>(const PlanCost& a, const PlanCost& b)
  {
    return b < a;
  }
  friend bool operator<=(const PlanCost& a, const PlanCost& b)
  {
    return !(b < a);
  }
  friend bool operator>=(const PlanCost& a, const PlanCost& b)
  {
    return !(a < b);
  }

  template <
      typename T,
      typename = std::enable_if_t<std::is_arithmetic<T>::value>>
  friend bool operator<(const PlanCost& a, T b)
  {
    return a < PlanCost(static_cast<double>(b));
  }
  template <
      typename T,
      typename = std::enable_if_t<std::is_arithmetic<T>::value>>
  friend bool operator<(T a, const PlanCost& b)
  {
    return PlanCost(static_cast<double>(a)) < b;
  }

  friend std::ostream& operator<<(std::ostream& os, const PlanCost& cost)
  {
    if (!cost.is_bounded()) return os << "unbounded";
    return os << '(' << cost.ticks << ',' << cost.work_value() << ')';
  }

 private:
  struct RawTag {};
  PlanCost(int64_t ticks_, int64_t work_, RawTag)
      : ticks(ticks_), work(work_)
  {
  }
};

struct TAPFReferenceCheckpoint {
  uint64_t state_hash = 0;
  PhysConfig state;
  size_t action_index = 0;
  PlanCost prefix_cost;
  PlanCost suffix_cost;
  std::vector<Op> next_ops;
};

struct TAPFReferencePlan {
  // Primitive joint actions from one authoritative, replayed incumbent.
  // Checkpoints point into this immutable sequence; no guidance metadata,
  // reservations, or task indices are retained across searches.
  std::vector<std::vector<Op>> actions;
  std::vector<TAPFReferenceCheckpoint> checkpoints;
};

struct SolverWeights {
  double alpha = 1;
  double beta = 1;
  double gamma = 1;
  double delta = 1;
  int64_t alpha_scaled = PlanCost::WORK_SCALE;
  int64_t beta_scaled = PlanCost::WORK_SCALE;
  int64_t gamma_scaled = PlanCost::WORK_SCALE;
  int64_t delta_scaled = PlanCost::WORK_SCALE;
};

struct CarrierEventContract;
struct CarrierGuidance;
struct TAPFCarrierPersistentState;
struct TAPFCarrierRootContinuation;

struct TAPFSearchConfig {
  TAPFSearchMode mode = TAPFSearchMode::DFS;
  TAPFFocalTieBreak focal_tie_break = TAPFFocalTieBreak::H;
  TAPFObjective objective = TAPFObjective::LEGACY_WEIGHTED_WORK;
  double focal_weight = 1.5;
  // Search-kernel controls. Carrier production uses one macro-assisted
  // first-incumbent pass and one bounded first-improvement pass; generic
  // TAPF callers may still run anytime.
  bool macro_enabled = true;   // event-bounded rollout successors
  TAPFStopPolicy stop_policy = TAPFStopPolicy::ANYTIME;
  // Carrier adapters may need to replay/finalize a found plan before the
  // large CLOSED tree is destroyed.  Cleanup is still owned by the planner
  // and runs in its destructor; only the lifetime ordering changes.
  bool defer_cleanup = false;
  PlanCost incumbent_init = PlanCost::unbounded();
  // Non-owning, immutable hint built from the verified first-pass plan.
  // It may only reorder existing operators or register a replayed suffix
  // through the ordinary SearchEdge trace path.
  const TAPFReferencePlan* reference_plan = nullptr;
  // Completion-event segments restart the same TAPF planner from the
  // controller's current full physical state.
  std::optional<PhysConfig> initial_physical;
  // Optional session-owned Carrier caches. Search nodes and search trees are
  // never retained across solves; only immutable distance/topology data and
  // dependency-safe upper-epoch state live here.
  std::shared_ptr<TAPFCarrierPersistentState>
      carrier_persistent_state;
  // Optional replayable continuation from the previous solved root to this
  // root. The planner validates and replays it through the ordinary
  // attach_carrier_guidance() path before opening the new search.
  const TAPFCarrierRootContinuation*
      carrier_root_continuation = nullptr;
  // Optional owning copy of the guidance attached to this search root.
  std::shared_ptr<CarrierGuidance>*
      carrier_root_guidance_output = nullptr;
  // Non-owning immutable hard contract for one segment.  Null preserves
  // production guidance and the instance root.
  const CarrierEventContract* event_contract = nullptr;
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

enum class PairBoundStage : uint8_t {
  CHEAP_BOUND = 0,
  PREFIX_BOUND = 1,
  EXACT = 2,
};

struct PairPlan {
  double estimated_cost = 0;
  int rollout_steps = 0;
  int direct_distance = -1;
  bool reached_goal = false;
  bool truncated = false;
  bool stalled = false;
  bool exact = true;
  bool cutoff = false;
  PairBoundStage bound_stage = PairBoundStage::EXACT;
};

struct PairVacancyThreshold {
  int endpoint = -1;
  int service_ticks = 0;
  int pushes = 0;
  int loaded_steps = 0;
};

// Conservative dynamic read-set for reusing a PairPlan across nearby upper
// states.  A complete dependency proves reuse is safe when direct reads and
// selected vacancy sources stay unchanged, and no new vacancy beats a saved
// endpoint threshold.
struct PairCostDependency {
  std::vector<uint64_t> cells;
  std::vector<uint64_t> vacancy_removal_cells;
  std::vector<PairVacancyThreshold> vacancy_thresholds;
  bool complete = false;
};

struct PairCostEntry {
  int goal = -1;
  PairPlan plan;
  PairCostDependency dependency;
};

using PairCostTable = std::vector<std::vector<PairCostEntry>>;

// Serializable primal/dual state for the PairCost assignment Hungarian.
// The next upper epoch can repair only rows whose PairCost entries changed.
struct PairAssignmentHungarianState {
  int row_count = 0;
  int column_count = 0;
  std::vector<int> row_to_column;
  std::vector<int> column_to_row;
  std::vector<long double> row_dual;
  std::vector<long double> column_dual;
  bool valid = false;
};

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

struct UpperShelfHandle {
  enum class Kind : uint8_t { TARGET = 0, ANONYMOUS = 1 };

  Kind kind = Kind::TARGET;
  int stable_id = -1;

  bool operator==(const UpperShelfHandle& o) const
  {
    return kind == o.kind && stable_id == o.stable_id;
  }
  bool operator!=(const UpperShelfHandle& o) const
  {
    return !(*this == o);
  }
  bool operator<(const UpperShelfHandle& o) const
  {
    return kind != o.kind ? kind < o.kind
                          : stable_id < o.stable_id;
  }
};

struct FrozenTaskId {
  size_t wave = 0;
  size_t ordinal = 0;

  bool operator==(const FrozenTaskId& o) const
  {
    return wave == o.wave && ordinal == o.ordinal;
  }
  bool operator!=(const FrozenTaskId& o) const
  {
    return !(*this == o);
  }
  bool operator<(const FrozenTaskId& o) const
  {
    return wave != o.wave ? wave < o.wave
                          : ordinal < o.ordinal;
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

// A real in-flight task may temporarily pin one target to the root goal
// that caused the task to start.  Uncommitted rows remain free to use the
// ordinary exact PairCost matching.
using RootGoalCommitment = std::map<int, int>;

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

struct TransferKey {
  ShelfSelector shelf;
  int source = -1;
  int endpoint = -1;

  bool operator==(const TransferKey& o) const
  {
    return shelf == o.shelf && source == o.source &&
           endpoint == o.endpoint;
  }
  bool operator!=(const TransferKey& o) const { return !(*this == o); }
  bool operator<(const TransferKey& o) const
  {
    if (shelf != o.shelf) return shelf < o.shelf;
    return source != o.source ? source < o.source
                              : endpoint < o.endpoint;
  }
};

struct TransferId {
  // A deterministic, branch-local episode identity.  `anchor` is the
  // physical-state hash at the transition that established the binding;
  // reroutes and leg changes preserve it, while a later Lift creates a new
  // anchor without introducing a process-global counter into ordering.
  TransferKey key;
  int carrier = -1;
  uint64_t anchor = 0;

  bool valid() const
  {
    return key.source >= 0 && key.endpoint >= 0 && carrier >= 0;
  }
  bool operator==(const TransferId& o) const
  {
    return key == o.key && carrier == o.carrier && anchor == o.anchor;
  }
  bool operator!=(const TransferId& o) const { return !(*this == o); }
  bool operator<(const TransferId& o) const
  {
    if (key != o.key) return key < o.key;
    return carrier != o.carrier ? carrier < o.carrier
                                : anchor < o.anchor;
  }
};

enum class RebindReason {
  NONE = 0,
  FORCED_DEVIATION = 1,
  NO_ROUTE = 2,
  ENDPOINT_UNAVAILABLE = 3,
};

enum class RouteStatus {
  OK = 0,
  PREFIX = 1,
  ARRIVED = 2,
  TEMPORARILY_BLOCKED = 3,
  BUDGET_EXHAUSTED = 4,
  NO_ROUTE = 5,
};

struct StorageTransfer {
  int endpoint = -1;
  std::vector<int> route;

  bool operator==(const StorageTransfer& o) const
  {
    // Endpoint identifies the transfer intent.  The route is a replaceable
    // execution hint and is intentionally excluded from identity.
    return endpoint == o.endpoint;
  }
  bool operator!=(const StorageTransfer& o) const
  {
    return !(*this == o);
  }
};

struct TaskBRForcedEffect {
  enum class Kind : uint8_t { WAIT = 0, TRANSFER = 1 };

  ShelfSelector shelf;
  Kind kind = Kind::WAIT;
  StorageTransfer transfer;
  std::vector<RootDemand> roots;
  int priority = 0;

  bool operator==(const TaskBRForcedEffect& other) const
  {
    return shelf == other.shelf && kind == other.kind &&
           transfer.endpoint == other.transfer.endpoint &&
           transfer.route == other.transfer.route &&
           roots == other.roots && priority == other.priority;
  }
  bool operator<(const TaskBRForcedEffect& other) const
  {
    if (shelf != other.shelf) return shelf < other.shelf;
    if (kind != other.kind)
      return static_cast<uint8_t>(kind) <
             static_cast<uint8_t>(other.kind);
    if (transfer.endpoint != other.transfer.endpoint)
      return transfer.endpoint < other.transfer.endpoint;
    if (transfer.route != other.transfer.route)
      return transfer.route < other.transfer.route;
    if (roots != other.roots) return roots < other.roots;
    return priority < other.priority;
  }
};

enum class ExecutionStatus : uint8_t {
  PENDING = 0,
  CARRYING = 1,
  COMPLETED = 2,
};

enum class FixedTransferStartMode : uint8_t {
  LOCKED_CARRYING = 0,
  PROVISIONAL_FREE = 1,
};

struct TaskLedgerEntry {
  FrozenTaskId task_id;
  ExecutionStatus status = ExecutionStatus::PENDING;
  std::optional<int> robot;
};

struct FixedRobotTransfer {
  int robot = -1;
  FrozenTaskId task_id;
  UpperShelfHandle stable_shelf;
  ShelfSelector shelf;
  StorageTransfer transfer;
  FixedTransferStartMode start_mode =
      FixedTransferStartMode::PROVISIONAL_FREE;
};

struct CarrierEventContract {
  PhysConfig start;
  std::vector<TaskLedgerEntry> wave_ledger_snapshot;
  std::vector<FixedRobotTransfer> active_transfers;
};

enum class CarrierTaskPhaseKind : uint8_t {
  APPROACH = 0,
  CARRYING = 1,
  COMPLETED = 2,
  INVALID = 3,
};

struct CarrierTaskPhase {
  CarrierTaskPhaseKind kind = CarrierTaskPhaseKind::INVALID;
  int route_index = -1;

  bool operator==(const CarrierTaskPhase& o) const
  {
    return kind == o.kind && route_index == o.route_index;
  }
  bool operator!=(const CarrierTaskPhase& o) const
  {
    return !(*this == o);
  }
};

enum class CarrierEventContractInvalidReason : uint8_t {
  NONE = 0,
  INVALID_PHYSICAL_ROOT = 1,
  INVALID_LEDGER = 2,
  INVALID_ACTIVE_TRANSFER = 3,
  LEDGER_ACTIVE_MISMATCH = 4,
  UNBOUND_CARRYING_ROBOT = 5,
};

struct CarrierEventContractValidation {
  CarrierEventContractInvalidReason reason =
      CarrierEventContractInvalidReason::NONE;

  bool valid() const
  {
    return reason == CarrierEventContractInvalidReason::NONE;
  }
};

CarrierTaskPhase carrier_event_phase_of(
    const PhysConfig& state,
    const FixedRobotTransfer& transfer);

CarrierEventContractValidation validate_carrier_event_contract(
    const DDInstance& ins, const CarrierEventContract& contract);

bool validate_carrier_event_transition(
    const DDInstance& ins, const CarrierEventContract& contract,
    const PhysConfig& from, const std::vector<Op>& ops,
    const PhysConfig& to);

struct ShelfTask {
  TaskId id;
  std::vector<RootDemand> roots;
  int priority = 0;
  StorageTransfer transfer;
};

struct RotationCandidate {
  std::vector<TaskId> cycle;
};

enum class CausalEventKind {
  ENTER_CELL = 0,
  ACQUIRE = 1,
  DROP = 2,
  ROBOT_AVAILABLE = 3,
};

struct CausalEdge {
  int producer = -1;
  int must_be_vacated = -1;
  int consumer = -1;
  CausalEventKind kind = CausalEventKind::ENTER_CELL;
};

struct ShelfTaskGraph {
  std::vector<ShelfTask> tasks;
  std::vector<std::vector<int>> predecessors;
  std::vector<std::vector<int>> successors;
  std::vector<CausalEdge> causal_edges;
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
  StorageTransfer transfer;
  size_t transfer_index = 0;
  TransferId transfer_id;
  int original_endpoint = -1;
  RebindReason rebind_reason = RebindReason::NONE;
  RouteStatus route_status = RouteStatus::OK;
  std::optional<TaskId> preferred_leg;
};

enum class ExecutionTaskState {
  ACTIVE = 0,
  FULFILLED = 1,
  PENDING = 2,
  SHADOWED = 3,
};

struct ExecutionTaskView {
  ExecutionTaskState state = ExecutionTaskState::PENDING;
  int carrier = -1;
  std::optional<TransferId> transfer_id;
};

struct CausalConditionView {
  bool fulfilled = false;
};

struct ExecutionView {
  std::vector<ExecutionTaskView> tasks;
  std::vector<CausalConditionView> causal_conditions;
};

struct TimedRouteHint {
  RouteStatus status = RouteStatus::NO_ROUTE;
  int endpoint = -1;
  std::vector<int> cells;
  int arrival_tick = -1;
  int expansions = 0;
};

struct JointTransportGuidance {
  std::vector<std::optional<TimedRouteHint>> by_robot;
  int expansions = 0;
  int frames_evaluated = 0;
  long long predicted_all_targets_ticks = 0;
  long long predicted_work = 0;
  double build_time_ms = 0;
};

struct UpperEpochGuidance {
  UpperSignature upper_signature;
  PairCostTable pair_cost;
  PairAssignmentHungarianState pair_assignment_hungarian;
  long pair_edges_evaluated = 0;
  long pair_edges_total = 0;
  long pair_edges_reused = 0;
  long pair_hungarian_full_solves = 0;
  long pair_hungarian_row_repairs = 0;
  long pair_hungarian_forced_repairs = 0;
  long pair_rollout_work_steps = 0;
  long pair_rollout_truncations = 0;
  long pair_rollout_stalls = 0;
  std::vector<int> tau_guide;
  RootGoalCommitment root_goal_commitment;
  std::vector<int> priority_commitment;
  std::map<RootDemand, TransferKey> transfer_continuity;
  std::vector<TaskBRForcedEffect> forced_effects;
  std::vector<int> target_priority;
  ShelfTaskGraph task_graph;
};

enum class DispatchMode {
  NONE = 0,
  EXECUTE = 1,
  PREPARE = 2,
};

enum class CandidateAdmission {
  DROP_GLOBALLY_UNREACHABLE = 0,
  KEEP_ALL_PENDING_ROWS = 1,
};

enum class RhoMatchStatus {
  OK = 0,
  CUTOFF = 1,
  INFEASIBLE = 2,
};

enum class RhoDropReason {
  INVALID_TASK = 0,
  DUPLICATE_TRANSFER_KEY = 1,
  SAME_SHELF_PRESELECTED = 2,
  UPSTREAM_TRANSFER_CLAIM = 3,
  MODE_INELIGIBLE = 4,
  NO_REACHABLE_ROBOT = 5,
};

enum class RhoObjectiveVersion : uint32_t {
  BOTTLENECK_SECONDARY_PRIORITY_V1 = 3,
  BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2 = 4,
};

inline const char* rho_objective_version_name(
    RhoObjectiveVersion version)
{
  switch (version) {
    case RhoObjectiveVersion::BOTTLENECK_SECONDARY_PRIORITY_V1:
      return "BOTTLENECK_SECONDARY_PRIORITY_V1";
    case RhoObjectiveVersion::
        BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2:
      return "BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2";
  }
  return "UNKNOWN";
}

enum class RhoIncrementalFallbackReason : uint8_t {
  NONE = 0,
  NO_PARENT_STATE = 1,
  COLUMN_MODEL_CHANGED = 2,
  INVALID_PARENT_STATE = 3,
};

// Exact node-local state for the production rho objective.  The Hungarian
// rows are the transposed original columns (real robots plus rho dummies);
// task columns are followed by zero-cost padding columns.  This makes a
// robot-only movement a row update while preserving the original rectangular
// task-to-robot/dummy assignment problem exactly.
struct RhoIncrementalState {
  static constexpr uint32_t MATRIX_ENCODING_VERSION = 1;
  static constexpr uint32_t CANONICAL_VERSION = 1;

  uint32_t matrix_encoding_version = MATRIX_ENCODING_VERSION;
  uint32_t canonical_version = CANONICAL_VERSION;
  RhoObjectiveVersion objective_version =
      RhoObjectiveVersion::
          BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2;
  DispatchMode mode = DispatchMode::NONE;
  CandidateAdmission admission =
      CandidateAdmission::DROP_GLOBALLY_UNREACHABLE;
  std::vector<int> free_robots;
  std::vector<int> candidate_task_indices;
  std::vector<int> candidate_priorities;
  std::vector<TaskId> candidate_ids;
  std::vector<TransferKey> candidate_keys;
  int task_count = 0;
  int free_robot_count = 0;
  int dimension = 0;
  long long bottleneck = 0;
  long long secondary_cost = 0;
  std::vector<
      std::shared_ptr<const std::vector<long long>>>
      transposed_task_cost_rows;
  tapf_assignment_detail::
      IncrementalHungarianState<long long> hungarian;
  bool valid = false;
};

struct RhoCandidateAudit {
  int task_index = -1;
  TransferKey key;
  TaskId id;
  DispatchMode mode = DispatchMode::NONE;
  int priority = 0;
  int nearest_robot_distance = -1;
  RhoDropReason reason = RhoDropReason::INVALID_TASK;
};

struct RhoMatchTelemetry {
  RhoObjectiveVersion objective_version =
      RhoObjectiveVersion::
          BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2;
  bool direct_target_phase = false;
  long candidates_input = 0;
  long candidates_after_claims = 0;
  long candidates_after_key_dedupe = 0;
  long candidates_after_shelf_preselect = 0;
  long candidates_after_priority = 0;
  long invalid_filtered = 0;
  long duplicate_key_filtered = 0;
  long same_shelf_filtered = 0;
  long upstream_claim_filtered = 0;
  long mode_ineligible_filtered = 0;
  long no_reachable_robot_filtered = 0;
  long priority_filtered = 0;
  long rows_without_finite_real_edge = 0;
  long maximum_real_cardinality = -1;
  long real_assignments = 0;
  double maximum_real_cardinality_time_ms = 0;
  bool maximum_real_cardinality_cutoff = false;
  long matrix_rows = 0;
  long matrix_cols = 0;
  double candidate_time_ms = 0;
  double matrix_time_ms = 0;
  double bottleneck_time_ms = 0;
  double secondary_full_time_ms = 0;
  double secondary_repair_time_ms = 0;
  double canonical_time_ms = 0;
  long long bottleneck = 0;
  long long secondary_cost = 0;
  RhoIncrementalFallbackReason incremental_fallback =
      RhoIncrementalFallbackReason::NO_PARENT_STATE;
  long incremental_full_solves = 0;
  long incremental_repairs = 0;
  long incremental_zero_row_reuses = 0;
  long incremental_changed_rows = 0;
  uint64_t column_identity_fingerprint = 0;
  uint64_t column_value_fingerprint = 0;
  uint64_t mode_or_conflict_fingerprint = 0;
  std::vector<uint64_t> robot_row_fingerprints;
};

struct DDReadyMatchProbe {
  RhoMatchStatus status = RhoMatchStatus::OK;
  std::vector<std::optional<TaskId>> rho_task_id;
  std::vector<std::optional<TransferKey>> rho_transfer_key;
  std::vector<int> rho_ready_index;
  RhoMatchTelemetry telemetry;
  std::vector<RhoCandidateAudit> audit;
  std::optional<RhoIncrementalState> rho_state;
};

struct CarrierGuidance {
  // Task-BR-PIBT guidance.  `upper_epoch` is immutable and may be shared
  // across robot-only transitions; all remaining fields are rebuilt from
  // the current physical state and one real parent transition.
  std::shared_ptr<const UpperEpochGuidance> upper_epoch;
  std::vector<int> ready_tasks;
  std::vector<int> preparable_tasks;
  std::vector<std::optional<TaskId>> rho_task_id;
  std::vector<std::optional<TransferKey>> rho_transfer_key;
  std::vector<int> rho_ready_index;
  std::vector<DispatchMode> rho_mode;
  RhoMatchTelemetry rho_execute_telemetry;
  RhoMatchTelemetry rho_prepare_telemetry;
  std::optional<RhoIncrementalState> rho_execute_state;
  std::optional<RhoIncrementalState> rho_prepare_state;
  uint64_t rho_mode_or_conflict_fingerprint = 0;
  std::vector<uint64_t> rho_row_fingerprints;
  std::vector<std::optional<Custody>> custody_by_robot;
  ExecutionView execution_view;
  JointTransportGuidance timed_transport;
};

struct TAPFCarrierRootContinuation {
  PhysConfig previous_physical;
  CarrierGuidance previous_guidance;
  std::vector<std::vector<Op>> executed_prefix;
};
