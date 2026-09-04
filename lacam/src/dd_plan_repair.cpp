#include "../include/dd_planner.hpp"
#include "../include/utils.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <queue>
#include <unordered_map>

#include "carrier_guidance.hpp"  // load_solver_weights: the ONE parser

namespace {

// Weighted-SOC view of the repair (review fix 2026-09-01): production
// candidate selection is by weighted SOC, so a repair may only be
// accepted when it does not increase that objective either.
struct RepairWeights {
  double alpha = 1, beta = 1, gamma = 1, delta = 1;
};

double joint_op_cost(const RepairWeights& w, const PhysConfig& from,
                     const std::vector<Op>& ops)
{
  double c = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
    if (ops[i].kind == Op::MOVE) {
      c += from.kappa[i] == KAPPA_FREE ? w.beta : w.alpha;
      if (from.kappa[i] == KAPPA_ANON) c += w.delta;
    } else if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) {
      c += w.gamma;
    }
  }
  return c;
}

struct IntVectorHasher {
  size_t operator()(const std::vector<int>& values) const
  {
    size_t h = values.size();
    for (const int v : values)
      h ^= static_cast<size_t>(v) + 0x9e3779b9U + (h << 6) + (h >> 2);
    return h;
  }
};

struct PhysConfigPtrHasher {
  size_t operator()(const PhysConfig* p) const
  {
    return PhysConfigHasher{}(*p);
  }
};

struct PhysConfigPtrEqual {
  bool operator()(const PhysConfig* a, const PhysConfig* b) const
  {
    return *a == *b;
  }
};

struct ShelfProjectionPtrHasher {
  size_t operator()(const PhysConfig* p) const
  {
    const IntVectorHasher hash;
    size_t h = hash(p->target_pos);
    const size_t a = hash(p->anon_occ);
    h ^= a + 0x9e3779b9U + (h << 6) + (h >> 2);
    return h;
  }
};

struct ShelfProjectionPtrEqual {
  bool operator()(const PhysConfig* a, const PhysConfig* b) const
  {
    return a->target_pos == b->target_pos && a->anon_occ == b->anon_occ;
  }
};

bool all_shelves_grounded(const PhysConfig& s)
{
  return std::all_of(s.kappa.begin(), s.kappa.end(),
                     [](int k) { return k == KAPPA_FREE; });
}

std::vector<int> distances_to(const DDGrid& grid, int goal)
{
  const int inf = grid.size() + 1;
  std::vector<int> dist(grid.size(), inf);
  std::queue<int> open;
  dist[goal] = 0;
  open.push(goal);
  while (!open.empty()) {
    const int u = open.front();
    open.pop();
    int nb[4];
    const int n = grid.neighbors(u, nb);
    for (int k = 0; k < n; ++k) {
      const int v = nb[k];
      if (dist[v] <= dist[u] + 1) continue;
      dist[v] = dist[u] + 1;
      open.push(v);
    }
  }
  return dist;
}

DDPlan single_robot_bridge(const DDGrid& grid, int start, int goal,
                           size_t max_steps)
{
  DDPlan out;
  const auto dist = distances_to(grid, goal);
  int at = start;
  while (at != goal && out.size() < max_steps) {
    int next = -1;
    int nb[4];
    const int n = grid.neighbors(at, nb);
    for (int k = 0; k < n; ++k)
      if (next < 0 || dist[nb[k]] < dist[next]) next = nb[k];
    if (next < 0 || dist[next] >= dist[at]) return {};
    out.push_back({Op::make_move(next)});
    at = next;
  }
  return at == goal && out.size() < max_steps ? out : DDPlan();
}

uint64_t pair_key(int a, int b)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
         static_cast<uint32_t>(b);
}

std::pair<int, int> decode_pair(uint64_t key)
{
  return {static_cast<int>(key >> 32),
          static_cast<int>(static_cast<uint32_t>(key))};
}

struct PairRecord {
  uint64_t parent = 0;
  int g = 0;
  bool has_parent = false;
};

struct PairOpen {
  int f;
  int g;
  uint64_t key;
};

struct PairOpenWorse {
  bool operator()(const PairOpen& a, const PairOpen& b) const
  {
    if (a.f != b.f) return a.f > b.f;
    return a.g < b.g;
  }
};

// Exact A* in the two-robot lower-deck configuration graph.  The heuristic
// is max of the independent wall distances, hence admissible and consistent.
std::optional<DDPlan> two_robot_bridge(
    const DDGrid& grid, const std::vector<int>& start,
    const std::vector<int>& goal, size_t max_steps)
{
  if (start == goal) return DDPlan();
  const auto d0 = distances_to(grid, goal[0]);
  const auto d1 = distances_to(grid, goal[1]);
  if (d0[start[0]] > grid.size() || d1[start[1]] > grid.size())
    return std::nullopt;

  const uint64_t start_key = pair_key(start[0], start[1]);
  const uint64_t goal_key = pair_key(goal[0], goal[1]);
  std::priority_queue<PairOpen, std::vector<PairOpen>, PairOpenWorse> open;
  std::unordered_map<uint64_t, PairRecord> records;
  records.emplace(start_key, PairRecord());
  open.push(
      PairOpen{std::max(d0[start[0]], d1[start[1]]), 0, start_key});

  while (!open.empty()) {
    const PairOpen top = open.top();
    open.pop();
    const auto rec_it = records.find(top.key);
    if (rec_it == records.end() || rec_it->second.g != top.g) continue;
    if (top.key == goal_key) break;
    if (static_cast<size_t>(top.g + 1) >= max_steps) continue;

    const auto [a, b] = decode_pair(top.key);
    int a_nb[4], b_nb[4];
    const int an = grid.neighbors(a, a_nb);
    const int bn = grid.neighbors(b, b_nb);
    int a_cand[5], b_cand[5];
    a_cand[0] = a;
    b_cand[0] = b;
    std::copy(a_nb, a_nb + an, a_cand + 1);
    std::copy(b_nb, b_nb + bn, b_cand + 1);
    for (int ai = 0; ai <= an; ++ai) {
      for (int bi = 0; bi <= bn; ++bi) {
        const int na = a_cand[ai], nb = b_cand[bi];
        if (na == nb || (na == b && nb == a)) continue;
        const int ng = top.g + 1;
        const int h = std::max(d0[na], d1[nb]);
        if (static_cast<size_t>(ng + h) >= max_steps) continue;
        const uint64_t next_key = pair_key(na, nb);
        auto [it, inserted] =
            records.emplace(next_key, PairRecord{top.key, ng, true});
        if (!inserted && it->second.g <= ng) continue;
        it->second = PairRecord{top.key, ng, true};
        open.push(PairOpen{ng + h, ng, next_key});
      }
    }
  }

  if (records.find(goal_key) == records.end()) return std::nullopt;
  std::vector<uint64_t> reverse_path;
  for (uint64_t key = goal_key; key != start_key;
       key = records.at(key).parent)
    reverse_path.push_back(key);
  std::reverse(reverse_path.begin(), reverse_path.end());

  DDPlan out;
  out.reserve(reverse_path.size());
  uint64_t previous = start_key;
  for (const uint64_t key : reverse_path) {
    const auto [a0, b0] = decode_pair(previous);
    const auto [a1, b1] = decode_pair(key);
    out.push_back(
        {a0 == a1 ? Op::make_wait() : Op::make_move(a1),
         b0 == b1 ? Op::make_wait() : Op::make_move(b1)});
    previous = key;
  }
  return out.size() < max_steps ? std::optional<DDPlan>(std::move(out))
                                : std::nullopt;
}

// Guaranteed fallback for any robot count: project the original segment to
// lower-deck configurations and erase exact robot-configuration loops.
DDPlan projected_original_bridge(const std::vector<PhysConfig>& states,
                                 size_t begin, size_t end)
{
  std::unordered_map<std::vector<int>, size_t, IntVectorHasher> last;
  last.reserve((end - begin + 1) * 2);
  for (size_t t = begin; t <= end; ++t) last[states[t].robots] = t;

  std::vector<size_t> kept;
  for (size_t t = begin; t <= end;) {
    kept.push_back(t);
    const size_t jump = last.at(states[t].robots);
    t = std::max(t, jump) + 1;
  }

  DDPlan out;
  if (kept.size() < 2) return out;
  out.reserve(kept.size() - 1);
  for (size_t k = 1; k < kept.size(); ++k) {
    const auto& from = states[kept[k - 1]].robots;
    const auto& to = states[kept[k]].robots;
    std::vector<Op> ops(from.size(), Op::make_wait());
    for (size_t i = 0; i < from.size(); ++i)
      if (from[i] != to[i]) ops[i] = Op::make_move(to[i]);
    out.push_back(std::move(ops));
  }
  return out;
}

std::optional<DDPlan> shortest_available_bridge(
    const DDInstance& ins, const std::vector<PhysConfig>& states,
    size_t begin, size_t end)
{
  const size_t max_steps = end - begin;
  const auto& start = states[begin].robots;
  const auto& goal = states[end].robots;
  if (start.size() == 1) {
    DDPlan path =
        single_robot_bridge(ins.grid, start[0], goal[0], max_steps);
    if (start == goal || !path.empty()) return path;
  } else if (start.size() == 2) {
    if (auto path = two_robot_bridge(ins.grid, start, goal, max_steps))
      return path;
  }

  DDPlan fallback = projected_original_bridge(states, begin, end);
  if (fallback.size() < max_steps) return fallback;
  return std::nullopt;
}

bool valid_goal_plan(const DDInstance& ins, const DDPlan& plan)
{
  PhysConfig state = initial_phys_config(ins);
  for (const auto& ops : plan) {
    auto next = apply_ops(ins, state, ops);
    if (!next.has_value()) return false;
    state = std::move(*next);
  }
  return is_dd_goal(ins, state);
}

}  // namespace

DDPlan repair_carrier_plan_impl(const DDInstance& ins, const DDPlan& plan,
                                const std::vector<PhysConfig>* replayed_states,
                                DDPlanRepairStats* stats,
                                const Deadline* deadline)
{
  if (stats != nullptr) *stats = DDPlanRepairStats();
  if (plan.size() < 2 || ins.n_robots() == 0) return plan;
  // R1 (debug.md §10): the repair belongs to the owning pass's 10s
  // deadline.  Checks are amortized (every EXPIRY_STRIDE steps in the
  // linear phases); any expiry aborts to the raw plan, which is valid by
  // precondition.
  constexpr size_t EXPIRY_STRIDE = 256;
  auto expired = [&]() { return is_expired(deadline); };
  if (expired()) return plan;

  std::vector<PhysConfig> owned_states;
  if (replayed_states == nullptr) {
    owned_states.reserve(plan.size() + 1);
    owned_states.push_back(initial_phys_config(ins));
    for (const auto& ops : plan) {
      auto next = apply_ops(ins, owned_states.back(), ops);
      if (!next.has_value()) return plan;
      owned_states.push_back(std::move(*next));
      if (owned_states.size() % EXPIRY_STRIDE == 0 && expired())
        return plan;
    }
  }
  const auto& states =
      replayed_states != nullptr ? *replayed_states : owned_states;
  if (states.size() != plan.size() + 1 ||
      !(states.front() == initial_phys_config(ins)))
    return plan;
  if (!is_dd_goal(ins, states.back())) return plan;

  // `states` is fully materialized and never reallocated below.  Key the
  // loop tables by pointers into that stable storage so the hash/equality
  // semantics stay exact without copying four vectors per timestep into
  // the unordered_map nodes (150k+ allocations on dense incumbents).
  std::unordered_map<const PhysConfig*, size_t, PhysConfigPtrHasher,
                     PhysConfigPtrEqual>
      last_exact;
  std::unordered_map<const PhysConfig*, size_t, ShelfProjectionPtrHasher,
                     ShelfProjectionPtrEqual>
      last_projection;
  last_exact.reserve(states.size() * 2);
  last_projection.reserve(states.size());
  for (size_t t = 0; t < states.size(); ++t) {
    last_exact[&states[t]] = t;
    if (all_shelves_grounded(states[t]))
      last_projection[&states[t]] = t;
  }

  // Weighted-SOC bookkeeping (review fix 2026-09-01): prefix sums of the
  // original per-step cost give O(1) replaced-segment cost; a bridge is
  // lower-deck-only with every shelf grounded, so its cost is beta per
  // MOVE.  Weight validation lives in the shared parser.
  RepairWeights weights;
  carrier_detail::load_solver_weights(weights);
  std::vector<double> prefix_cost(plan.size() + 1, 0.0);
  for (size_t t = 0; t < plan.size(); ++t)
    prefix_cost[t + 1] =
        prefix_cost[t] + joint_op_cost(weights, states[t], plan[t]);
  auto bridge_cost = [&](const DDPlan& bridge) {
    long moves = 0;
    for (const auto& ops : bridge)
      for (const auto& op : ops) moves += op.kind == Op::MOVE ? 1 : 0;
    return weights.beta * static_cast<double>(moves);
  };
  constexpr double SOC_EPS = 1e-9;
  double repaired_cost = 0;

  DDPlan repaired;
  repaired.reserve(plan.size());
  DDPlanRepairStats local;
  for (size_t t = 0; t < plan.size();) {
    if (repaired.size() % EXPIRY_STRIDE == 0 && expired()) return plan;
    const size_t exact = last_exact.at(&states[t]);
    if (exact > t) {
      ++local.exact_loops;
      local.steps_removed += static_cast<long>(exact - t);
      t = exact;
      continue;
    }

    if (all_shelves_grounded(states[t])) {
      const size_t projected =
          last_projection.at(&states[t]);
      if (projected > t) {
        auto bridge =
            shortest_available_bridge(ins, states, t, projected);
        if (bridge.has_value() &&
            bridge->size() < static_cast<size_t>(projected - t) &&
            bridge_cost(*bridge) <=
                prefix_cost[projected] - prefix_cost[t] + SOC_EPS) {
          ++local.projected_loops;
          local.bridge_steps += static_cast<long>(bridge->size());
          local.steps_removed += static_cast<long>(
              projected - t - bridge->size());
          repaired_cost += bridge_cost(*bridge);
          repaired.insert(repaired.end(),
                          std::make_move_iterator(bridge->begin()),
                          std::make_move_iterator(bridge->end()));
          t = projected;
          continue;
        }
      }
    }

    repaired_cost += prefix_cost[t + 1] - prefix_cost[t];
    repaired.push_back(plan[t]);
    ++t;
  }

  // The public solver uses an empty plan as failure, including on instances
  // that are already at goal.  Preserve its one-wait success convention.
  if (repaired.empty()) repaired = plan;
  if (expired()) return plan;  // R1: no budget for the final replay
  if (repaired.size() >= plan.size() ||
      repaired_cost > prefix_cost[plan.size()] + SOC_EPS ||
      !valid_goal_plan(ins, repaired))
    return plan;
  if (stats != nullptr) *stats = local;
  return repaired;
}

DDPlan repair_carrier_plan(const DDInstance& ins, const DDPlan& plan,
                           DDPlanRepairStats* stats,
                           const Deadline* deadline)
{
  return repair_carrier_plan_impl(ins, plan, nullptr, stats, deadline);
}

DDPlan repair_carrier_plan_from_replay(
    const DDInstance& ins, const DDPlan& plan,
    const std::vector<PhysConfig>& states, DDPlanRepairStats* stats,
    const Deadline* deadline)
{
  return repair_carrier_plan_impl(ins, plan, &states, stats, deadline);
}
