/*
 * dd_carrier: two-deck physical configuration model for Carrier-LaCAM
 * (design.md sections 2-3, 6).  Self-contained: no dependency on lacam's
 * Graph/Instance.
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// kappa encoding (design 3.1): -1 free, -2 carrying anonymous shelf,
// >=0 carrying target with that index.
constexpr int KAPPA_FREE = -1;
constexpr int KAPPA_ANON = -2;

struct DDGrid {
  int height = 0;
  int width = 0;
  std::vector<uint8_t> wall;  // size h*w, 1 = wall
  // Authoritative topology. Rectangular maps materialize their legacy
  // down/up/right/left edges here; explicit maps replace them completely.
  std::vector<std::vector<int>> out_neighbors;
  std::vector<std::vector<int>> in_neighbors;
  bool explicit_adjacency = false;

  DDGrid() = default;
  explicit DDGrid(const std::vector<std::string>& rows);

  int idx(int r, int c) const { return r * width + c; }
  int row(int v) const { return v / width; }
  int col(int v) const { return v % width; }
  bool is_wall(int v) const { return wall[v] != 0; }
  int size() const { return height * width; }
  const std::vector<int>& outgoing(int v) const
  {
    return out_neighbors[v];
  }
  const std::vector<int>& incoming(int v) const
  {
    return in_neighbors[v];
  }
  bool uses_explicit_adjacency() const
  {
    return explicit_adjacency;
  }
  template <typename Visitor>
  void for_each_neighbor(int v, Visitor&& visit) const
  {
    for (const int neighbor : outgoing(v)) visit(neighbor);
  }
  void set_undirected_edges(
      const std::vector<std::pair<int, int>>& edges);
  void set_undirected_adjacency(
      const std::vector<std::vector<int>>& adjacency);
  void reset_rectangular_adjacency();
  void block_cell(int v);
  bool has_edge(int from, int to) const;
  // Legacy fixed-degree adapter retained for old tests and callers that
  // only use rectangular maps. It throws instead of truncating degree > 4.
  int neighbors(int v, int out[4]) const;
};

struct DDInstance {
  DDGrid grid;
  std::vector<int> robots;         // start cells (labeled, YAML order)
  std::vector<int> shelves;        // ALL shelf cells incl. target starts
  // Grounded-shelf/drop mask, size grid.size().  Empty before finalize()
  // means legacy behavior: every traversable cell is a storage cell.
  // Carried shelves may still MOVE through non-storage cells.
  std::vector<uint8_t> shelf_storage;
  // Per-cell immutable local transfer property computed by finalize():
  // every traversable neighbor is already a legal storage endpoint.
  std::vector<uint8_t> adjacent_storage_frontier;
  // Static upper-deck occupancy owned outside this Carrier session.
  // These cells are not movable shelves: unloaded robots may travel below
  // them, but carried shelves cannot enter and no shelf may be dropped there.
  // finalize() validates and sorts this vector.
  std::vector<int> fixed_upper_cells;
  std::vector<int> target_starts;  // by target index
  std::vector<int> target_goals;   // representative view: sorted-first of
                                   // the goal set (== the goal for
                                   // singleton/old-format instances)
  // eligible goal set per target (design_final 2.1, T1): sorted unique,
  // wall-component filtered by finalize().  Old fixed-goal instances are
  // singleton sets {target_goals[b]} (materialized by finalize when the
  // caller only filled target_goals).
  std::vector<std::vector<int>> target_goal_sets;
  std::string name;

  size_t n_robots() const { return robots.size(); }
  size_t n_targets() const { return target_starts.size(); }
  bool can_store_shelf(int v) const
  {
    return v >= 0 && v < grid.size() && !grid.is_wall(v) &&
           (shelf_storage.empty() || shelf_storage[v] != 0);
  }
  bool is_fixed_upper_cell(int v) const
  {
    return std::binary_search(
        fixed_upper_cells.begin(), fixed_upper_cells.end(), v);
  }
  bool can_place_movable_shelf(int v) const
  {
    return can_store_shelf(v) && !is_fixed_upper_cell(v);
  }
  bool has_adjacent_storage_frontier(int v) const
  {
    return v >= 0 && v < (int)adjacent_storage_frontier.size() &&
           adjacent_storage_frontier[v] != 0;
  }
  // consistency checks + derived data; call after filling fields
  void finalize();
};

DDInstance load_dd_instance(const std::string& yaml_path);

// robot primitive operator (design 3.2)
struct Op {
  enum Kind : uint8_t { WAIT, MOVE, LIFT, DROP } kind = WAIT;
  int to = -1;  // MOVE target cell

  static Op make_wait() { return Op{WAIT, -1}; }
  static Op make_move(int v) { return Op{MOVE, v}; }
  static Op make_lift() { return Op{LIFT, -1}; }
  static Op make_drop() { return Op{DROP, -1}; }
  bool operator==(const Op& o) const { return kind == o.kind && to == o.to; }
};

// physical configuration X = (Q^R, Q^B, kappa) (design 3.1)
struct PhysConfig {
  std::vector<int> robots;      // per robot cell
  std::vector<int> target_pos;  // per target cell (grounded or carried)
  std::vector<int> anon_occ;    // SORTED cells of grounded anonymous shelves
  std::vector<int> kappa;       // per robot: KAPPA_FREE/KAPPA_ANON/target idx

  bool operator==(const PhysConfig& o) const
  {
    return robots == o.robots && target_pos == o.target_pos &&
           anon_occ == o.anon_occ && kappa == o.kappa;
  }
};

PhysConfig initial_phys_config(const DDInstance& ins);

enum class PhysRootInvalidReason : uint8_t {
  NONE = 0,
  VECTOR_SIZE = 1,
  INVALID_ROBOT_CELL = 2,
  ROBOT_COLLISION = 3,
  INVALID_KAPPA = 4,
  DUPLICATE_TARGET_CARRIER = 5,
  INVALID_TARGET_CELL = 6,
  TARGET_CARRIER_MISMATCH = 7,
  INVALID_ANONYMOUS_CELL = 8,
  ANONYMOUS_ORDER_OR_DUPLICATE = 9,
  SHELF_COLLISION = 10,
  SHELF_COUNT_MISMATCH = 11,
  FIXED_UPPER_COLLISION = 12,
};

struct PhysRootValidation {
  PhysRootInvalidReason reason = PhysRootInvalidReason::NONE;

  bool valid() const
  {
    return reason == PhysRootInvalidReason::NONE;
  }
};

// Validate an arbitrary physical search root without reading Graph::U.
// Callers must do this before converting cell indices into Vertex*.
PhysRootValidation validate_phys_config_root(
    const DDInstance& ins, const PhysConfig& state);

// goal condition (design 2.2): every target grounded at its goal
bool is_dd_goal(const DDInstance& ins, const PhysConfig& s);

// two-deck transition validator (design 3.3 rule table).  Returns the
// successor configuration, or nullopt if any rule/precondition is violated.
std::optional<PhysConfig> apply_ops(const DDInstance& ins, const PhysConfig& s,
                                    const std::vector<Op>& ops,
                                    bool allow_following = true);

uint64_t phys_config_hash(const PhysConfig& s);

// Zobrist-style incremental hash (design 6.1): given the hash of s and a
// LEGAL joint op vector, returns the hash of apply_ops(ins, s, ops) without
// rehashing the whole configuration.
uint64_t phys_config_hash_incremental(const DDInstance& ins,
                                      const PhysConfig& s,
                                      const std::vector<Op>& ops,
                                      uint64_t hash_of_s);

struct PhysConfigHasher {
  uint64_t operator()(const PhysConfig& s) const
  {
    return phys_config_hash(s);
  }
};
