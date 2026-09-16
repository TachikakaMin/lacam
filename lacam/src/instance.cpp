#include "../include/instance.hpp"

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs_compat = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs_compat = std::experimental::filesystem;
#endif
#include <algorithm>
#include <cmath>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

Instance::Instance(const std::string& map_filename,
                   const std::vector<int>& start_indexes,
                   const std::vector<int>& goal_indexes)
    : G(map_filename),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size())
{
  for (auto k : start_indexes) starts.push_back(G.U[k]);
  for (auto k : goal_indexes) goals.push_back(G.U[k]);
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string& scen_filename,
                   const std::string& map_filename, const int _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // load start-goal pairs
  std::ifstream file(scen_filename);
  if (!file) {
    info(0, 0, scen_filename, " is not found");
    return;
  }
  std::string line;
  std::smatch results;

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_instance)) {
      auto x_s = std::stoi(results[1].str());
      auto y_s = std::stoi(results[2].str());
      auto x_g = std::stoi(results[3].str());
      auto y_g = std::stoi(results[4].str());
      if (x_s < 0 || G.width <= x_s || x_g < 0 || G.width <= x_g) continue;
      if (y_s < 0 || G.height <= y_s || y_g < 0 || G.height <= y_g) continue;
      auto s = G.U[G.width * y_s + x_s];
      auto g = G.U[G.width * y_g + x_g];
      if (s == nullptr || g == nullptr) continue;
      starts.push_back(s);
      goals.push_back(g);
    }

    if (starts.size() == N) break;
  }
}

Instance::Instance(const std::string& map_filename, std::mt19937* MT,
                   const int _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // random assignment
  const auto K = G.size();

  // set starts
  auto s_indexes = std::vector<int>(K);
  std::iota(s_indexes.begin(), s_indexes.end(), 0);
  std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
  int i = 0;
  while (true) {
    if (i >= K) return;
    starts.push_back(G.V[s_indexes[i]]);
    if (starts.size() == N) break;
    ++i;
  }

  // set goals
  auto g_indexes = std::vector<int>(K);
  std::iota(g_indexes.begin(), g_indexes.end(), 0);
  std::shuffle(g_indexes.begin(), g_indexes.end(), *MT);
  int j = 0;
  while (true) {
    if (j >= K) return;
    goals.push_back(G.V[g_indexes[j]]);
    if (goals.size() == N) break;
    ++j;
  }
}

bool Instance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != goals.size()) {
    info(1, verbose, "invalid N, check instance");
    return false;
  }
  return true;
}

TAPFInstance::TAPFInstance(const std::string& map_filename,
                           const std::vector<int>& start_indexes,
                           const std::vector<std::vector<int> >& task_indexes,
                           const std::vector<std::vector<int> >& task_costs)
    : G(map_filename),
      starts(Config()),
      tasks(Config()),
      allowed(std::vector<std::vector<bool> >()),
      goal_cost(std::vector<std::vector<int> >()),
      N(start_indexes.size())
{
  std::unordered_map<int, int> index_to_task;
  for (auto k : start_indexes) starts.push_back(G.U[k]);

  allowed.resize(N);
  goal_cost.resize(N);
  for (size_t i = 0; i < task_indexes.size(); ++i) {
    for (size_t g = 0; g < task_indexes[i].size(); ++g) {
      const auto k = task_indexes[i][g];
      const auto c = (i < task_costs.size() && g < task_costs[i].size())
                         ? task_costs[i][g]
                         : 0;
      if (index_to_task.find(k) == index_to_task.end()) {
        index_to_task[k] = tasks.size();
        tasks.push_back(G.U[k]);
        for (auto& row : allowed) row.push_back(false);
        for (auto& row : goal_cost) row.push_back(0);
      }
      const auto j = index_to_task[k];
      if (allowed[i][j]) {
        goal_cost[i][j] = std::min(goal_cost[i][j], c);  // duplicate entry
      } else {
        allowed[i][j] = true;
        goal_cost[i][j] = c;
      }
    }
  }
}

TAPFInstance::TAPFInstance(const YamlData& data)
    : TAPFInstance(data.map_filename, data.start_indexes, data.task_indexes,
                   data.task_costs)
{
  height_by_index = data.height_by_index;
  climb_cost = data.climb_cost;
  if (!height_by_index.empty()) {
    // drop edges with height difference > 1 (unclimbable cliffs)
    for (auto u : G.V) {
      auto& nb = u->neighbor;
      nb.erase(std::remove_if(nb.begin(), nb.end(),
                              [&](Vertex* m) {
                                return std::abs(height_by_index[u->index] -
                                                height_by_index[m->index]) > 1;
                              }),
               nb.end());
    }
  }
}

TAPFInstance::TAPFInstance(const std::string& yaml_filename,
                           const std::string& map_dir)
    : TAPFInstance(load_yaml(yaml_filename, map_dir))
{
}

TAPFInstance::YamlData TAPFInstance::load_yaml(
    const std::string& yaml_filename, const std::string& map_dir)
{
  auto config = YAML::LoadFile(yaml_filename);
  YamlData data;

  if (config["map"].IsScalar()) {
    fs_compat::path map_path(config["map"].as<std::string>());
    if (!map_dir.empty()) {
      map_path = fs_compat::path(map_dir) / map_path;
    } else if (map_path.is_relative()) {
      map_path = fs_compat::path(yaml_filename).parent_path() / map_path;
    }
    data.map_filename = map_path.string();
  } else {
    info(0, 0, "TAPF YAML inline map format is not supported");
    return data;
  }

  Graph graph(data.map_filename);
  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto r_s = start[0].as<int>();
    const auto c_s = start[1].as<int>();
    data.start_indexes.push_back(graph.width * r_s + c_s);

    data.task_indexes.push_back(std::vector<int>());
    data.task_costs.push_back(std::vector<int>());
    const auto& goals =
        node["potentialGoals"] ? node["potentialGoals"] : node["goal"];
    if (goals.IsSequence() && goals.size() > 0 && goals[0].IsSequence()) {
      for (const auto& goal : goals) {
        const auto r_g = goal[0].as<int>();
        const auto c_g = goal[1].as<int>();
        data.task_indexes.back().push_back(graph.width * r_g + c_g);
      }
    } else if (goals.IsSequence() && goals.size() == 2) {
      const auto r_g = goals[0].as<int>();
      const auto c_g = goals[1].as<int>();
      data.task_indexes.back().push_back(graph.width * r_g + c_g);
    }
    // optional per-goal assignment cost offsets, aligned with potentialGoals
    if (node["goalCosts"] && node["goalCosts"].IsSequence()) {
      for (const auto& c : node["goalCosts"]) {
        data.task_costs.back().push_back(c.as<int>());
      }
    }
  }

  // optional terrain: heights (row-major grid) + climbCost for +-1 steps
  if (config["heights"] && config["heights"].IsSequence()) {
    data.height_by_index.assign(graph.width * graph.height, 0);
    int r = 0;
    for (const auto& row : config["heights"]) {
      int c = 0;
      for (const auto& cell : row) {
        if (r < graph.height && c < graph.width) {
          data.height_by_index[graph.width * r + c] = cell.as<int>();
        }
        ++c;
      }
      ++r;
    }
  }
  if (config["climbCost"]) {
    data.climb_cost = std::max(1, config["climbCost"].as<int>());
  }

  return data;
}

bool TAPFInstance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != allowed.size()) {
    info(1, verbose, "invalid N, check TAPF instance");
    return false;
  }
  if (tasks.size() < N) {
    info(1, verbose, "TAPF expects at least one unique task per agent");
    return false;
  }
  for (size_t i = 0; i < N; ++i) {
    if (starts[i] == nullptr) {
      info(1, verbose, "invalid TAPF start");
      return false;
    }
    if (allowed[i].size() != tasks.size()) {
      info(1, verbose, "invalid TAPF compatibility matrix");
      return false;
    }
    auto any_allowed = false;
    for (size_t j = 0; j < tasks.size(); ++j) {
      if (tasks[j] == nullptr) {
        info(1, verbose, "invalid TAPF task");
        return false;
      }
      any_allowed = any_allowed || allowed[i][j];
    }
    if (!any_allowed) {
      info(1, verbose, "agent has no allowed TAPF task");
      return false;
    }
  }
  return true;
}
