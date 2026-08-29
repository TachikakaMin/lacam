/*
 * dd_carrier: two-deck physical configuration model for Carrier-LaCAM
 * (design.md sections 2-3, 6).  Self-contained: no dependency on lacam's
 * Graph/Instance.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// kappa encoding (design 3.1): -1 free, -2 carrying anonymous shelf,
// >=0 carrying target with that index.
constexpr int KAPPA_FREE = -1;
constexpr int KAPPA_ANON = -2;

struct DDGrid {
  int height = 0;
  int width = 0;
  std::vector<uint8_t> wall;  // size h*w, 1 = wall

  DDGrid() = default;
  explicit DDGrid(const std::vector<std::string>& rows);

  int idx(int r, int c) const { return r * width + c; }
  int row(int v) const { return v / width; }
  int col(int v) const { return v % width; }
  bool is_wall(int v) const { return wall[v] != 0; }
  int size() const { return height * width; }
  // 4-neighborhood, walls excluded; returns count, fills out[0..3]
  int neighbors(int v, int out[4]) const;
};

struct DDInstance {
  DDGrid grid;
  std::vector<int> robots;         // start cells (labeled, YAML order)
  std::vector<int> shelves;        // ALL shelf cells incl. target starts
  std::vector<int> target_starts;  // by target index
  std::vector<int> target_goals;   // by target index
  std::string name;

  size_t n_robots() const { return robots.size(); }
  size_t n_targets() const { return target_starts.size(); }
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

// goal condition (design 2.2): every target grounded at its goal
bool is_dd_goal(const DDInstance& ins, const PhysConfig& s);

// two-deck transition validator (design 3.3 rule table).  Returns the
// successor configuration, or nullopt if any rule/precondition is violated.
std::optional<PhysConfig> apply_ops(const DDInstance& ins, const PhysConfig& s,
                                    const std::vector<Op>& ops);

uint64_t phys_config_hash(const PhysConfig& s);

struct PhysConfigHasher {
  uint64_t operator()(const PhysConfig& s) const
  {
    return phys_config_hash(s);
  }
};
