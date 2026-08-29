// dd_carrier implementation: two-deck model + transition validator
// (design.md sections 2.2, 3.1-3.3).  The validator is the single
// implementation of the rule table (R1 R2 S1 I1 I2 I3; S2 implied by R2).
#include "../include/dd_carrier.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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
      wall[idx(r, c)] = (ch == '@' || ch == 'T' || ch == '#') ? 1 : 0;
    }
  }
}

int DDGrid::neighbors(int v, int out[4]) const
{
  const int r = row(v), c = col(v);
  int n = 0;
  if (r + 1 < height && !is_wall(idx(r + 1, c))) out[n++] = idx(r + 1, c);
  if (r - 1 >= 0 && !is_wall(idx(r - 1, c))) out[n++] = idx(r - 1, c);
  if (c + 1 < width && !is_wall(idx(r, c + 1))) out[n++] = idx(r, c + 1);
  if (c - 1 >= 0 && !is_wall(idx(r, c - 1))) out[n++] = idx(r, c - 1);
  return n;
}

void DDInstance::finalize()
{
  if (grid.size() == 0) throw std::invalid_argument("finalize: empty grid");
  auto check_cell = [&](int v, const char* what) {
    if (v < 0 || v >= grid.size() || grid.is_wall(v)) {
      std::ostringstream ss;
      ss << "finalize: invalid " << what << " cell " << v;
      throw std::invalid_argument(ss.str());
    }
  };
  std::unordered_set<int> seen_r, seen_s;
  for (int q : robots) {
    check_cell(q, "robot");
    if (!seen_r.insert(q).second)
      throw std::invalid_argument("finalize: robots overlap");
  }
  for (int p : shelves) {
    check_cell(p, "shelf");
    if (!seen_s.insert(p).second)
      throw std::invalid_argument("finalize: shelves overlap");
  }
  if (target_starts.size() != target_goals.size())
    throw std::invalid_argument("finalize: target starts/goals mismatch");
  std::unordered_set<int> seen_g;
  for (size_t b = 0; b < target_starts.size(); ++b) {
    if (!seen_s.count(target_starts[b]))
      throw std::invalid_argument("finalize: target start is not a shelf");
    check_cell(target_goals[b], "goal");
    if (!seen_g.insert(target_goals[b]).second)
      throw std::invalid_argument("finalize: duplicate target goal");
  }
}

DDInstance load_dd_instance(const std::string& yaml_path)
{
  YAML::Node doc = YAML::LoadFile(yaml_path);
  DDInstance ins;
  if (doc["name"]) ins.name = doc["name"].as<std::string>();

  // debug.md P0-4: v1 does not implement optional flag semantics; fail
  // loudly on any non-default value instead of silently ignoring it.
  if (doc["flags"]) {
    for (const auto& kv : doc["flags"]) {
      bool value = false;
      try {
        value = kv.second.as<bool>();
      } catch (...) {
        value = true;  // non-boolean flag value: treat as unsupported
      }
      if (value) {
        throw std::invalid_argument(
            "load_dd_instance: unsupported non-default flag '" +
            kv.first.as<std::string>() + "' (v1 implements defaults only)");
      }
    }
  }

  std::vector<std::string> rows;
  {
    std::istringstream ss(doc["map"].as<std::string>());
    std::string line;
    while (std::getline(ss, line)) {
      // strip trailing whitespace/CR
      while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
        line.pop_back();
      if (!line.empty()) rows.push_back(line);
    }
  }
  ins.grid = DDGrid(rows);

  for (const auto& n : doc["robots"])
    ins.robots.push_back(ins.grid.idx(n[0].as<int>(), n[1].as<int>()));
  if (doc["shelves"])
    for (const auto& n : doc["shelves"])
      ins.shelves.push_back(ins.grid.idx(n[0].as<int>(), n[1].as<int>()));
  if (doc["targets"])
    for (const auto& t : doc["targets"]) {
      ins.target_starts.push_back(
          ins.grid.idx(t["start"][0].as<int>(), t["start"][1].as<int>()));
      ins.target_goals.push_back(
          ins.grid.idx(t["goal"][0].as<int>(), t["goal"][1].as<int>()));
    }
  ins.finalize();
  return ins;
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

bool is_dd_goal(const DDInstance& ins, const PhysConfig& s)
{
  for (size_t b = 0; b < ins.n_targets(); ++b)
    if (s.target_pos[b] != ins.target_goals[b]) return false;
  for (int k : s.kappa)
    if (k >= 0) return false;  // carried target is not grounded (D10)
  return true;
}

std::optional<PhysConfig> apply_ops(const DDInstance& ins, const PhysConfig& s,
                                    const std::vector<Op>& ops)
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
        int nb[4];
        const int n = ins.grid.neighbors(q, nb);
        bool adj = false;
        for (int k = 0; k < n; ++k) adj |= (nb[k] == op.to);
        if (!adj) return std::nullopt;
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
  // --- S1: shelf vertex conflict at t+1 ---
  {
    std::unordered_set<int> upper;
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

uint64_t phys_config_hash(const PhysConfig& s)
{
  // FNV-1a over all state components (anon_occ kept sorted -> canonical)
  uint64_t h = 1469598103934665603ULL;
  auto mix = [&h](uint64_t x) {
    h ^= x + 0x9e3779b97f4a7c15ULL;
    h *= 1099511628211ULL;
  };
  for (int v : s.robots) mix((uint64_t)v);
  mix(0xffffULL);
  for (int v : s.target_pos) mix((uint64_t)v);
  mix(0xfffeULL);
  for (int v : s.anon_occ) mix((uint64_t)v);
  mix(0xfffdULL);
  for (int v : s.kappa) mix((uint64_t)(v + 3));
  return h;
}
