#include "../include/dd_carrier.hpp"

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

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
      while (!line.empty() &&
             (line.back() == '\r' || line.back() == ' '))
        line.pop_back();
      if (!line.empty()) rows.push_back(line);
    }
  }
  ins.grid = DDGrid(rows);

  if (doc["storage_map"]) {
    std::vector<std::string> storage_rows;
    std::istringstream ss(doc["storage_map"].as<std::string>());
    std::string line;
    while (std::getline(ss, line)) {
      while (!line.empty() &&
             (line.back() == '\r' || line.back() == ' '))
        line.pop_back();
      if (!line.empty()) storage_rows.push_back(line);
    }
    if ((int)storage_rows.size() != ins.grid.height)
      throw std::invalid_argument(
          "load_dd_instance: storage_map height mismatch");
    ins.shelf_storage.assign(ins.grid.size(), 0);
    for (int r = 0; r < ins.grid.height; ++r) {
      if ((int)storage_rows[r].size() != ins.grid.width)
        throw std::invalid_argument(
            "load_dd_instance: storage_map width mismatch");
      for (int c = 0; c < ins.grid.width; ++c) {
        const char ch = storage_rows[r][c];
        if (ch == 'S' || ch == 's' || ch == '1')
          ins.shelf_storage[ins.grid.idx(r, c)] = 1;
        else if (ch != '.' && ch != '0' && ch != '-')
          throw std::invalid_argument(
              "load_dd_instance: invalid storage_map character");
      }
    }
  }

  for (const auto& n : doc["robots"])
    ins.robots.push_back(
        ins.grid.idx(n[0].as<int>(), n[1].as<int>()));
  if (doc["shelves"])
    for (const auto& n : doc["shelves"])
      ins.shelves.push_back(
          ins.grid.idx(n[0].as<int>(), n[1].as<int>()));
  // goal-set forms (design_final 2.1, T1): top-level `goal_pool` +
  // per-target `goals: [[r,c],...]` (explicit set) or `goals: pool`
  // (shared pool reference); old `goal: [r,c]` stays the singleton form.
  std::vector<int> pool;
  if (doc["goal_pool"])
    for (const auto& n : doc["goal_pool"])
      pool.push_back(
          ins.grid.idx(n[0].as<int>(), n[1].as<int>()));
  if (doc["targets"])
    for (const auto& t : doc["targets"]) {
      ins.target_starts.push_back(
          ins.grid.idx(
              t["start"][0].as<int>(),
              t["start"][1].as<int>()));
      std::vector<int> set;
      if (t["goals"]) {
        if (t["goals"].IsScalar()) {
          if (t["goals"].as<std::string>() != "pool")
            throw std::invalid_argument(
                "load_dd_instance: target `goals` must be a list or 'pool'");
          if (pool.empty())
            throw std::invalid_argument(
                "load_dd_instance: `goals: pool` without a goal_pool");
          set = pool;
        } else {
          for (const auto& n : t["goals"])
            set.push_back(
                ins.grid.idx(
                    n[0].as<int>(), n[1].as<int>()));
        }
      } else {
        set.push_back(
            ins.grid.idx(
                t["goal"][0].as<int>(),
                t["goal"][1].as<int>()));
      }
      ins.target_goal_sets.push_back(std::move(set));
    }
  ins.finalize();
  return ins;
}
