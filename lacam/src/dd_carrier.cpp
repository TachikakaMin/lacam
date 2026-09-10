// dd_carrier implementation: two-deck model + transition validator
// (design.md sections 2.2, 3.1-3.3).  The validator is the single
// implementation of the rule table (R1 R2 S1 I1 I2 I3; S2 implied by R2).
#include "../include/dd_carrier.hpp"
#include "../include/utils.hpp"

#include <algorithm>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

DDGrid::DDGrid(const std::vector<std::string>& rows)
{
  height = static_cast<int>(rows.size());
  width = height > 0 ? static_cast<int>(rows[0].size()) : 0;
  wall.assign(height * width, 0);
  for (int r = 0; r < height; ++r) {
    if (static_cast<int>(rows[r].size()) != width)
      throw std::invalid_argument("DDGrid: ragged map rows");
    for (int c = 0; c < width; ++c) {
      char ch = rows[r][c];
      wall[idx(r, c)] = is_map_wall_char(ch) ? 1 : 0;
    }
  }
  reset_rectangular_adjacency();
}

void DDGrid::reset_rectangular_adjacency()
{
  out_neighbors.assign(size(), {});
  in_neighbors.assign(size(), {});
  for (int v = 0; v < size(); ++v) {
    if (is_wall(v)) continue;
    const int r = row(v);
    const int c = col(v);
    auto& out = out_neighbors[v];
    if (r + 1 < height && !is_wall(idx(r + 1, c)))
      out.push_back(idx(r + 1, c));
    if (r - 1 >= 0 && !is_wall(idx(r - 1, c)))
      out.push_back(idx(r - 1, c));
    if (c + 1 < width && !is_wall(idx(r, c + 1)))
      out.push_back(idx(r, c + 1));
    if (c - 1 >= 0 && !is_wall(idx(r, c - 1)))
      out.push_back(idx(r, c - 1));
  }
  in_neighbors = out_neighbors;
  explicit_adjacency = false;
}

void DDGrid::set_undirected_adjacency(
    const std::vector<std::vector<int>>& adjacency)
{
  if ((int)adjacency.size() != size())
    throw std::invalid_argument(
        "DDGrid: adjacency size mismatch");
  for (int from = 0; from < size(); ++from) {
    if (is_wall(from) && !adjacency[from].empty())
      throw std::invalid_argument(
          "DDGrid: wall has adjacency");
    std::unordered_set<int> seen;
    for (const int to : adjacency[from]) {
      if (to < 0 || to >= size() || is_wall(to) || from == to)
        throw std::invalid_argument(
            "DDGrid: invalid adjacency edge");
      if (!seen.insert(to).second)
        throw std::invalid_argument(
            "DDGrid: duplicate adjacency edge");
    }
  }
  for (int from = 0; from < size(); ++from)
    for (const int to : adjacency[from])
      if (std::find(
              adjacency[to].begin(), adjacency[to].end(), from) ==
          adjacency[to].end())
        throw std::invalid_argument(
            "DDGrid: undirected adjacency is not symmetric");
  out_neighbors = adjacency;
  in_neighbors = adjacency;
  explicit_adjacency = true;
}

void DDGrid::set_undirected_edges(
    const std::vector<std::pair<int, int>>& edges)
{
  std::vector<std::vector<int>> adjacency(size());
  for (const auto& edge : edges) {
    if (edge.first < 0 || edge.first >= size() ||
        edge.second < 0 || edge.second >= size())
      throw std::invalid_argument(
          "DDGrid: invalid adjacency edge");
    adjacency[edge.first].push_back(edge.second);
    adjacency[edge.second].push_back(edge.first);
  }
  set_undirected_adjacency(adjacency);
}

void DDGrid::block_cell(int v)
{
  if (v < 0 || v >= size())
    throw std::invalid_argument("DDGrid: invalid blocked cell");
  wall[v] = 1;
  for (auto& outgoing : out_neighbors)
    outgoing.erase(
        std::remove(outgoing.begin(), outgoing.end(), v),
        outgoing.end());
  for (auto& incoming : in_neighbors)
    incoming.erase(
        std::remove(incoming.begin(), incoming.end(), v),
        incoming.end());
  out_neighbors[v].clear();
  in_neighbors[v].clear();
}

bool DDGrid::has_edge(int from, int to) const
{
  if (from < 0 || from >= size() || to < 0 || to >= size())
    return false;
  const auto& out = outgoing(from);
  return std::find(out.begin(), out.end(), to) != out.end();
}

int DDGrid::neighbors(int v, int out[4]) const
{
  const auto& dynamic = outgoing(v);
  if (dynamic.size() > 4)
    throw std::length_error(
        "DDGrid: fixed neighbor buffer cannot represent this topology");
  std::copy(dynamic.begin(), dynamic.end(), out);
  return static_cast<int>(dynamic.size());
}

void DDInstance::finalize()
{
  if (grid.size() == 0) throw std::invalid_argument("finalize: empty grid");
  if (shelf_storage.empty()) {
    shelf_storage.resize(grid.size(), 0);
    for (int v = 0; v < grid.size(); ++v)
      shelf_storage[v] = grid.is_wall(v) ? 0 : 1;
  } else if ((int)shelf_storage.size() != grid.size()) {
    throw std::invalid_argument(
        "finalize: shelf storage mask size mismatch");
  }
  for (int v = 0; v < grid.size(); ++v)
    if (shelf_storage[v] && grid.is_wall(v))
      throw std::invalid_argument(
          "finalize: storage cell overlaps a wall");
  auto check_cell = [&](int v, const char* what) {
    if (v < 0 || v >= grid.size() || grid.is_wall(v)) {
      std::ostringstream ss;
      ss << "finalize: invalid " << what << " cell " << v;
      throw std::invalid_argument(ss.str());
    }
  };
  for (const int cell : fixed_upper_cells) {
    check_cell(cell, "fixed upper");
    if (!can_store_shelf(cell))
      throw std::invalid_argument(
          "finalize: fixed upper cell is outside storage");
  }
  std::sort(fixed_upper_cells.begin(), fixed_upper_cells.end());
  if (std::adjacent_find(
          fixed_upper_cells.begin(), fixed_upper_cells.end()) !=
      fixed_upper_cells.end())
    throw std::invalid_argument(
        "finalize: duplicate fixed upper cell");

  adjacent_storage_frontier.assign(grid.size(), 0);
  for (int v = 0; v < grid.size(); ++v) {
    if (grid.is_wall(v)) continue;
    bool direct_frontier = true;
    for (const int neighbor : grid.outgoing(v))
      direct_frontier &=
          can_place_movable_shelf(neighbor);
    adjacent_storage_frontier[v] =
        direct_frontier ? 1 : 0;
  }
  std::unordered_set<int> seen_r, seen_s;
  for (int q : robots) {
    check_cell(q, "robot");
    if (!seen_r.insert(q).second)
      throw std::invalid_argument("finalize: robots overlap");
  }
  for (int p : shelves) {
    check_cell(p, "shelf");
    if (!can_place_movable_shelf(p))
      throw std::invalid_argument(
          is_fixed_upper_cell(p)
              ? "finalize: movable shelf overlaps fixed upper cell"
              : "finalize: shelf is outside storage");
    if (!seen_s.insert(p).second)
      throw std::invalid_argument("finalize: shelves overlap");
  }
  // goal-set layer (design_final 2.1, T1): materialize singleton sets when
  // the caller only filled target_goals (all pre-goal-set construction
  // paths); otherwise the sets are authoritative and target_goals becomes
  // the representative view (sorted-first), resynced below.
  if (target_goal_sets.empty() && !target_goals.empty()) {
    if (target_starts.size() != target_goals.size())
      throw std::invalid_argument("finalize: target starts/goals mismatch");
    for (const int g : target_goals) target_goal_sets.push_back({g});
  }
  if (target_starts.size() != target_goal_sets.size())
    throw std::invalid_argument("finalize: target starts/goals mismatch");
  std::unordered_set<int> seen_t;  // one physical shelf = one target label
  for (size_t b = 0; b < target_starts.size(); ++b) {
    if (!seen_s.count(target_starts[b]))
      throw std::invalid_argument("finalize: target start is not a shelf");
    if (!seen_t.insert(target_starts[b]).second)
      throw std::invalid_argument(
          "finalize: duplicate target start (two targets reference the "
          "same shelf)");
    if (target_goal_sets[b].empty())
      throw std::invalid_argument("finalize: empty target goal set");
    for (const int g : target_goal_sets[b]) {
      check_cell(g, "goal");
      if (!can_place_movable_shelf(g))
        throw std::invalid_argument(
            is_fixed_upper_cell(g)
                ? "finalize: target goal overlaps fixed upper cell"
                : "finalize: target goal is outside storage");
    }
  }

  // dead-cell / feasibility analysis (design 5.6, v1 form): both decks share
  // the wall set, so a goal outside its target's wall-component can never be
  // reached — reject at load instead of pruning at search time.
  {
    std::vector<int> comp(grid.size(), -1);
    int nc = 0;
    for (int v = 0; v < grid.size(); ++v) {
      if (grid.is_wall(v) || comp[v] >= 0) continue;
      std::vector<int> stack{v};
      comp[v] = nc;
      while (!stack.empty()) {
        int u = stack.back();
        stack.pop_back();
        for (const int neighbor : grid.outgoing(u))
          if (comp[neighbor] < 0) {
            comp[neighbor] = nc;
            stack.push_back(neighbor);
          }
      }
      ++nc;
    }
    for (size_t b = 0; b < target_starts.size(); ++b) {
      // filter each goal set to the start's wall component (unreachable
      // eligible cells can never be used); loud failure when none remain
      // (same condition/message as the old singleton rule)
      auto& set = target_goal_sets[b];
      set.erase(std::remove_if(set.begin(), set.end(),
                               [&](int g) {
                                 return comp[g] != comp[target_starts[b]];
                               }),
                set.end());
      if (set.empty())
        throw std::invalid_argument(
            "finalize: target goal unreachable from its start "
            "(different wall components)");
      std::sort(set.begin(), set.end());
      set.erase(std::unique(set.begin(), set.end()), set.end());
    }
  }
  // representative view: sorted-first of each set (== the goal for
  // singleton/old-format instances)
  target_goals.resize(target_goal_sets.size());
  for (size_t b = 0; b < target_goal_sets.size(); ++b)
    target_goals[b] = target_goal_sets[b].front();

  // covering matching check (design_final 2.1 loader contract, D15): an
  // injective target->goal assignment must exist (Kuhn's augmenting
  // paths; subsumes the old duplicate-fixed-goal rejection).
  {
    std::unordered_map<int, int> col_of;  // goal cell -> column id
    std::vector<std::vector<int>> adj(target_goal_sets.size());
    for (size_t b = 0; b < target_goal_sets.size(); ++b)
      for (const int g : target_goal_sets[b]) {
        const auto it = col_of.emplace(g, (int)col_of.size()).first;
        adj[b].push_back(it->second);
      }
    std::vector<int> match_row(col_of.size(), -1);  // column -> row
    std::vector<char> vis;
    std::function<bool(int)> aug = [&](int row) {
      for (const int c : adj[row]) {
        if (vis[c]) continue;
        vis[c] = 1;
        if (match_row[c] < 0 || aug(match_row[c])) {
          match_row[c] = row;
          return true;
        }
      }
      return false;
    };
    for (size_t b = 0; b < target_goal_sets.size(); ++b) {
      vis.assign(col_of.size(), 0);
      if (!aug((int)b))
        throw std::invalid_argument(
            "finalize: no covering goal matching over target goal sets "
            "(Hall violation)");
    }
  }
}

PhysConfig initial_phys_config(const DDInstance& ins)
{
  PhysConfig s;
  s.robots = ins.robots;
  s.target_pos = ins.target_starts;
  std::unordered_set<int> tset(ins.target_starts.begin(),
                               ins.target_starts.end());
  for (int p : ins.shelves)
    if (!tset.count(p)) s.anon_occ.push_back(p);
  std::sort(s.anon_occ.begin(), s.anon_occ.end());
  s.kappa.assign(ins.robots.size(), KAPPA_FREE);
  return s;
}

PhysRootValidation validate_phys_config_root(
    const DDInstance& ins, const PhysConfig& state)
{
  const size_t robot_count = ins.n_robots();
  const size_t target_count = ins.n_targets();
  if (state.robots.size() != robot_count ||
      state.kappa.size() != robot_count ||
      state.target_pos.size() != target_count)
    return {PhysRootInvalidReason::VECTOR_SIZE};

  std::unordered_set<int> robot_cells;
  for (const int cell : state.robots) {
    if (cell < 0 || cell >= ins.grid.size() ||
        ins.grid.is_wall(cell))
      return {PhysRootInvalidReason::INVALID_ROBOT_CELL};
    if (!robot_cells.insert(cell).second)
      return {PhysRootInvalidReason::ROBOT_COLLISION};
  }

  std::vector<int> target_carrier(target_count, -1);
  size_t carried_anonymous = 0;
  for (size_t robot = 0; robot < robot_count; ++robot) {
    const int kappa = state.kappa[robot];
    if (kappa == KAPPA_FREE) continue;
    if (kappa == KAPPA_ANON) {
      ++carried_anonymous;
      continue;
    }
    if (kappa < 0 || kappa >= (int)target_count)
      return {PhysRootInvalidReason::INVALID_KAPPA};
    if (target_carrier[kappa] >= 0)
      return {
          PhysRootInvalidReason::DUPLICATE_TARGET_CARRIER};
    target_carrier[kappa] = (int)robot;
  }

  std::unordered_set<int> upper_cells;
  for (size_t target = 0; target < target_count; ++target) {
    const int cell = state.target_pos[target];
    if (cell < 0 || cell >= ins.grid.size() ||
        ins.grid.is_wall(cell))
      return {PhysRootInvalidReason::INVALID_TARGET_CELL};
    if (ins.is_fixed_upper_cell(cell))
      return {PhysRootInvalidReason::FIXED_UPPER_COLLISION};
    const int carrier = target_carrier[target];
    if (carrier >= 0) {
      if (cell != state.robots[carrier])
        return {
            PhysRootInvalidReason::TARGET_CARRIER_MISMATCH};
    } else if (!ins.can_place_movable_shelf(cell)) {
      return {PhysRootInvalidReason::INVALID_TARGET_CELL};
    }
    if (!upper_cells.insert(cell).second)
      return {PhysRootInvalidReason::SHELF_COLLISION};
  }

  if (!std::is_sorted(
          state.anon_occ.begin(), state.anon_occ.end()) ||
      std::adjacent_find(
          state.anon_occ.begin(), state.anon_occ.end()) !=
          state.anon_occ.end())
    return {
        PhysRootInvalidReason::ANONYMOUS_ORDER_OR_DUPLICATE};
  for (const int cell : state.anon_occ) {
    if (ins.is_fixed_upper_cell(cell))
      return {PhysRootInvalidReason::FIXED_UPPER_COLLISION};
    if (cell < 0 || cell >= ins.grid.size() ||
        !ins.can_place_movable_shelf(cell))
      return {
          PhysRootInvalidReason::INVALID_ANONYMOUS_CELL};
    if (!upper_cells.insert(cell).second)
      return {PhysRootInvalidReason::SHELF_COLLISION};
  }
  for (size_t robot = 0; robot < robot_count; ++robot) {
    if (state.kappa[robot] != KAPPA_ANON) continue;
    if (ins.is_fixed_upper_cell(state.robots[robot]))
      return {PhysRootInvalidReason::FIXED_UPPER_COLLISION};
    if (!upper_cells.insert(state.robots[robot]).second)
      return {PhysRootInvalidReason::SHELF_COLLISION};
  }

  if (ins.shelves.size() < target_count ||
      state.anon_occ.size() + carried_anonymous !=
          ins.shelves.size() - target_count)
    return {PhysRootInvalidReason::SHELF_COUNT_MISMATCH};
  return {};
}

bool is_dd_goal(const DDInstance& ins, const PhysConfig& s)
{
  // Prop 3 (design_final 4.1): membership in the eligible set is exact
  // (S1 makes b -> p_b injective, so tau(b) := p_b witnesses a matching)
  for (size_t b = 0; b < ins.n_targets(); ++b) {
    const auto& set = ins.target_goal_sets[b];
    if (!std::binary_search(set.begin(), set.end(), s.target_pos[b]))
      return false;
  }
  for (int k : s.kappa)
    if (k >= 0) return false;  // carried target is not grounded (D10)
  return true;
}

std::optional<PhysConfig> apply_ops(const DDInstance& ins, const PhysConfig& s,
                                    const std::vector<Op>& ops,
                                    bool allow_following)
{
  const size_t R = ins.n_robots();
  if (ops.size() != R || s.robots.size() != R) return std::nullopt;

  // grounded shelf lookup at step start
  // cell -> target idx (grounded targets only)
  std::unordered_map<int, int> grounded_target;
  std::vector<bool> target_carried(ins.n_targets(), false);
  for (size_t i = 0; i < R; ++i)
    if (s.kappa[i] >= 0) target_carried[s.kappa[i]] = true;
  for (size_t b = 0; b < ins.n_targets(); ++b)
    if (!target_carried[b]) grounded_target[s.target_pos[b]] = (int)b;
  std::unordered_set<int> grounded_anon(s.anon_occ.begin(), s.anon_occ.end());

  PhysConfig nxt;
  nxt.robots.resize(R);
  nxt.target_pos = s.target_pos;
  nxt.kappa = s.kappa;
  std::vector<int> anon_next(s.anon_occ.begin(), s.anon_occ.end());

  // --- per-robot preconditions & effects ---
  std::vector<int> lifted_cells;
  for (size_t i = 0; i < R; ++i) {
    const int q = s.robots[i];
    const Op& op = ops[i];
    switch (op.kind) {
      case Op::WAIT:
        nxt.robots[i] = q;
        break;
      case Op::MOVE: {
        // adjacency + wall
        if (op.to < 0 || op.to >= ins.grid.size()) return std::nullopt;
        if (!ins.grid.has_edge(q, op.to)) return std::nullopt;
        nxt.robots[i] = op.to;
        if (s.kappa[i] >= 0) nxt.target_pos[s.kappa[i]] = op.to;
        break;
      }
      case Op::LIFT: {
        if (s.kappa[i] != KAPPA_FREE) return std::nullopt;
        auto it = grounded_target.find(q);
        if (it != grounded_target.end()) {
          nxt.kappa[i] = it->second;
        } else if (grounded_anon.count(q)) {
          nxt.kappa[i] = KAPPA_ANON;
          anon_next.erase(
              std::find(anon_next.begin(), anon_next.end(), q));
        } else {
          return std::nullopt;  // I1: nothing grounded here at step start
        }
        lifted_cells.push_back(q);
        nxt.robots[i] = q;
        break;
      }
      case Op::DROP: {
        if (s.kappa[i] == KAPPA_FREE) return std::nullopt;
        if (!ins.can_place_movable_shelf(q)) return std::nullopt;
        if (s.kappa[i] == KAPPA_ANON) anon_next.push_back(q);
        nxt.kappa[i] = KAPPA_FREE;
        nxt.robots[i] = q;
        break;
      }
      default:
        return std::nullopt;
    }
  }
  // I2 guard: two lifts at the same cell (impossible while robots are
  // distinct, but keep the validator authoritative)
  std::sort(lifted_cells.begin(), lifted_cells.end());
  if (std::adjacent_find(lifted_cells.begin(), lifted_cells.end()) !=
      lifted_cells.end())
    return std::nullopt;

  // --- R1: vertex conflict ---
  {
    std::unordered_set<int> occ;
    for (int v : nxt.robots)
      if (!occ.insert(v).second) return std::nullopt;
  }
  // --- R2: swap conflict (following allowed) ---
  {
    std::unordered_map<int, int> pos_of;  // old cell -> robot
    for (size_t i = 0; i < R; ++i) pos_of[s.robots[i]] = (int)i;
    for (size_t i = 0; i < R; ++i) {
      auto it = pos_of.find(nxt.robots[i]);
      if (it != pos_of.end() && it->second != (int)i &&
          nxt.robots[it->second] == s.robots[i])
        return std::nullopt;
    }
  }
  // Explicit BRaP-conservative oracle variant used by conformance tests.
  // Production calls use the physical default (following allowed).
  if (!allow_following) {
    // lower deck: robot i enters a cell some robot j (!= i) leaves
    std::unordered_map<int, int> was_at;  // old cell -> robot
    for (size_t i = 0; i < R; ++i) was_at[s.robots[i]] = (int)i;
    for (size_t i = 0; i < R; ++i) {
      if (nxt.robots[i] == s.robots[i]) continue;
      auto it = was_at.find(nxt.robots[i]);
      if (it != was_at.end() && it->second != (int)i) return std::nullopt;
    }
    // upper deck: shelf cell entered while being vacated this step
    std::unordered_set<int> shelf_was;  // occupied upper cells at t
    for (const int p : ins.fixed_upper_cells) shelf_was.insert(p);
    for (int p : s.anon_occ) shelf_was.insert(p);
    for (size_t b = 0; b < ins.n_targets(); ++b)
      shelf_was.insert(s.target_pos[b]);
    for (size_t i = 0; i < R; ++i)
      if (s.kappa[i] == KAPPA_ANON) shelf_was.insert(s.robots[i]);
    // next shelf cells that MOVED into a previously-occupied cell
    auto entered_occupied = [&](int now, int before) {
      return now != before && shelf_was.count(now) > 0;
    };
    for (size_t b = 0; b < ins.n_targets(); ++b) {
      // carried targets move with their carrier
      int before = s.target_pos[b];
      if (entered_occupied(nxt.target_pos[b], before)) return std::nullopt;
    }
    for (size_t i = 0; i < R; ++i) {
      if (nxt.kappa[i] == KAPPA_ANON && s.kappa[i] == KAPPA_ANON &&
          entered_occupied(nxt.robots[i], s.robots[i]))
        return std::nullopt;
    }
  }

  // --- S1: shelf vertex conflict at t+1 ---
  {
    std::unordered_set<int> upper(
        ins.fixed_upper_cells.begin(), ins.fixed_upper_cells.end());
    for (int p : anon_next)
      if (!upper.insert(p).second) return std::nullopt;
    std::vector<bool> carried_next(ins.n_targets(), false);
    for (size_t i = 0; i < R; ++i) {
      if (nxt.kappa[i] >= 0) carried_next[nxt.kappa[i]] = true;
      if (nxt.kappa[i] == KAPPA_ANON)
        if (!upper.insert(nxt.robots[i]).second) return std::nullopt;
    }
    for (size_t b = 0; b < ins.n_targets(); ++b)
      if (!upper.insert(nxt.target_pos[b]).second) return std::nullopt;
    (void)carried_next;
  }

  std::sort(anon_next.begin(), anon_next.end());
  nxt.anon_occ = std::move(anon_next);
  return nxt;
}

namespace {
// deterministic Zobrist keys derived on the fly (splitmix64): no tables,
// no grid-size coupling, stable across runs.
inline uint64_t zmix(uint64_t x)
{
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
inline uint64_t zkey_robot(size_t i, int cell)
{
  return zmix(0x1000000000ULL + (uint64_t)i * 1315423911ULL + (uint64_t)cell);
}
inline uint64_t zkey_target(size_t b, int cell)
{
  return zmix(0x2000000000ULL + (uint64_t)b * 2654435761ULL + (uint64_t)cell);
}
inline uint64_t zkey_anon(int cell)
{
  return zmix(0x3000000000ULL + (uint64_t)cell);
}
inline uint64_t zkey_kappa(size_t i, int k)
{
  return zmix(0x4000000000ULL + (uint64_t)i * 40503ULL +
              (uint64_t)(k + 3));
}
}  // namespace

uint64_t phys_config_hash(const PhysConfig& s)
{
  // Zobrist XOR over all state components (design 6.1); anon occupancy is
  // a set -> order-free XOR is canonical by construction.
  uint64_t h = 0x5851f42d4c957f2dULL;
  for (size_t i = 0; i < s.robots.size(); ++i)
    h ^= zkey_robot(i, s.robots[i]);
  for (size_t b = 0; b < s.target_pos.size(); ++b)
    h ^= zkey_target(b, s.target_pos[b]);
  for (int c : s.anon_occ) h ^= zkey_anon(c);
  for (size_t i = 0; i < s.kappa.size(); ++i)
    h ^= zkey_kappa(i, s.kappa[i]);
  return h;
}

uint64_t phys_config_hash_incremental(const DDInstance& ins,
                                      const PhysConfig& s,
                                      const std::vector<Op>& ops,
                                      uint64_t h)
{
  // mirrors apply_ops effects; caller guarantees ops is LEGAL for s.
  std::unordered_map<int, int> grounded_target;
  std::vector<bool> carried(ins.n_targets(), false);
  for (size_t i = 0; i < s.kappa.size(); ++i)
    if (s.kappa[i] >= 0) carried[s.kappa[i]] = true;
  for (size_t b = 0; b < ins.n_targets(); ++b)
    if (!carried[b]) grounded_target[s.target_pos[b]] = (int)b;
  std::unordered_set<int> grounded_anon(s.anon_occ.begin(), s.anon_occ.end());

  for (size_t i = 0; i < ops.size(); ++i) {
    const int q = s.robots[i];
    const int k = s.kappa[i];
    switch (ops[i].kind) {
      case Op::WAIT:
        break;
      case Op::MOVE: {
        const int v = ops[i].to;
        h ^= zkey_robot(i, q);
        h ^= zkey_robot(i, v);
        if (k >= 0) {
          h ^= zkey_target((size_t)k, q);
          h ^= zkey_target((size_t)k, v);
        }
        // carried ANON has no positional key (identity-free); nothing else
        break;
      }
      case Op::LIFT: {
        auto it = grounded_target.find(q);
        if (it != grounded_target.end()) {
          h ^= zkey_kappa(i, KAPPA_FREE);
          h ^= zkey_kappa(i, it->second);
        } else {
          // anon leaves the grounded set while carried
          h ^= zkey_anon(q);
          h ^= zkey_kappa(i, KAPPA_FREE);
          h ^= zkey_kappa(i, KAPPA_ANON);
        }
        (void)grounded_anon;
        break;
      }
      case Op::DROP: {
        if (k == KAPPA_ANON) {
          h ^= zkey_anon(q);
          h ^= zkey_kappa(i, KAPPA_ANON);
          h ^= zkey_kappa(i, KAPPA_FREE);
        } else {
          h ^= zkey_kappa(i, k);
          h ^= zkey_kappa(i, KAPPA_FREE);
        }
        break;
      }
    }
  }
  return h;
}
