#include "../include/instance.hpp"
#include "../include/filesystem_compat.hpp"

#include <yaml-cpp/yaml.h>

TAPFInstance::TAPFInstance(const YamlData& data)
    : TAPFInstance(
          data.map_filename, data.start_indexes,
          data.task_indexes)
{
}

TAPFInstance::TAPFInstance(
    const std::string& yaml_filename,
    const std::string& map_dir)
    : TAPFInstance(load_yaml(yaml_filename, map_dir))
{
}

TAPFInstance::YamlData TAPFInstance::load_yaml(
    const std::string& yaml_filename,
    const std::string& map_dir)
{
  auto config = YAML::LoadFile(yaml_filename);
  YamlData data;

  if (config["map"].IsScalar()) {
    lacam_filesystem::path map_path(
        config["map"].as<std::string>());
    if (!map_dir.empty()) {
      map_path =
          lacam_filesystem::path(map_dir) / map_path;
    } else if (map_path.is_relative()) {
      map_path =
          lacam_filesystem::path(yaml_filename)
              .parent_path() /
          map_path;
    }
    data.map_filename = map_path.string();
  } else {
    info(
        0, 0,
        "TAPF YAML inline map format is not supported");
    return data;
  }

  Graph graph(data.map_filename);
  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto r_s = start[0].as<int>();
    const auto c_s = start[1].as<int>();
    data.start_indexes.push_back(
        graph.width * r_s + c_s);

    data.task_indexes.push_back(std::vector<int>());
    const auto& goals =
        node["potentialGoals"] ? node["potentialGoals"]
                               : node["goal"];
    if (goals.IsSequence() && goals.size() > 0 &&
        goals[0].IsSequence()) {
      for (const auto& goal : goals) {
        const auto r_g = goal[0].as<int>();
        const auto c_g = goal[1].as<int>();
        data.task_indexes.back().push_back(
            graph.width * r_g + c_g);
      }
    } else if (
        goals.IsSequence() && goals.size() == 2) {
      const auto r_g = goals[0].as<int>();
      const auto c_g = goals[1].as<int>();
      data.task_indexes.back().push_back(
          graph.width * r_g + c_g);
    }
  }

  return data;
}
