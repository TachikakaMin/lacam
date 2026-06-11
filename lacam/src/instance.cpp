#include "../include/instance.hpp"

#include <filesystem>
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
                           const std::vector<std::vector<int> >& task_cost_offsets,
                           int task_distance_scale)
    : G(map_filename),
      starts(Config()),
      tasks(Config()),
      allowed(std::vector<std::vector<bool> >()),
      assignment_cost_offsets(std::vector<std::vector<int> >()),
      assignment_distance_scale(task_distance_scale),
      N(start_indexes.size())
{
  std::unordered_map<int, int> index_to_task;
  for (auto k : start_indexes) starts.push_back(G.U[k]);

  allowed.resize(N);
  assignment_cost_offsets.resize(N);
  const auto has_offsets = !task_cost_offsets.empty();
  if (task_indexes.size() != N ||
      (has_offsets && task_cost_offsets.size() != N)) {
    assignment_distance_scale = 0;
    return;
  }
  if (has_offsets) {
    for (size_t i = 0; i < N; ++i) {
      if (task_cost_offsets[i].size() != task_indexes[i].size()) {
        assignment_distance_scale = 0;
        return;
      }
    }
  }
  for (size_t i = 0; i < task_indexes.size(); ++i) {
    for (size_t option = 0; option < task_indexes[i].size(); ++option) {
      const auto k = task_indexes[i][option];
      if (index_to_task.find(k) == index_to_task.end()) {
        index_to_task[k] = tasks.size();
        tasks.push_back(G.U[k]);
        for (auto& row : allowed) row.push_back(false);
        for (auto& row : assignment_cost_offsets) row.push_back(0);
      }
      const auto task = index_to_task[k];
      const auto offset = has_offsets ? task_cost_offsets[i][option] : 0;
      if (!allowed[i][task]) {
        allowed[i][task] = true;
        assignment_cost_offsets[i][task] = offset;
      } else {
        assignment_cost_offsets[i][task] =
            std::min(assignment_cost_offsets[i][task], offset);
      }
    }
  }
}

TAPFInstance::TAPFInstance(const YamlData& data)
    : TAPFInstance(data.map_filename, data.start_indexes, data.task_indexes)
{
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
    std::filesystem::path map_path(config["map"].as<std::string>());
    if (!map_dir.empty()) {
      map_path = std::filesystem::path(map_dir) / map_path;
    } else if (map_path.is_relative()) {
      map_path = std::filesystem::path(yaml_filename).parent_path() / map_path;
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
  }

  return data;
}

bool TAPFInstance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != allowed.size()) {
    info(1, verbose, "invalid N, check TAPF instance");
    return false;
  }
  if (assignment_cost_offsets.size() != N) {
    info(1, verbose, "invalid TAPF assignment cost rows");
    return false;
  }
  if (assignment_distance_scale <= 0) {
    info(1, verbose, "invalid TAPF assignment distance scale");
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
    if (assignment_cost_offsets[i].size() != tasks.size()) {
      info(1, verbose, "invalid TAPF assignment cost matrix");
      return false;
    }
    auto any_allowed = false;
    for (size_t j = 0; j < tasks.size(); ++j) {
      if (tasks[j] == nullptr) {
        info(1, verbose, "invalid TAPF task");
        return false;
      }
      if (assignment_cost_offsets[i][j] < 0) {
        info(1, verbose, "negative TAPF assignment cost offset");
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
