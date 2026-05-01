/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

struct Instance {
  const Graph G;  // graph
  Config starts;  // initial configuration
  Config goals;   // goal configuration
  const uint N;   // number of agents

  // for testing
  Instance(const std::string& map_filename,
           const std::vector<int>& start_indexes,
           const std::vector<int>& goal_indexes);
  // for MAPF benchmark
  Instance(const std::string& scen_filename, const std::string& map_filename,
           const int _N = 1);
  // random instance generation
  Instance(const std::string& map_filename, std::mt19937* MT, const int _N = 1);
  ~Instance() {}

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

struct TAPFInstance {
 private:
  struct YamlData {
    std::string map_filename;
    std::vector<int> start_indexes;
    std::vector<std::vector<int> > task_indexes;
  };

  static YamlData load_yaml(const std::string& yaml_filename,
                            const std::string& map_dir);
  explicit TAPFInstance(const YamlData& data);

 public:
  const Graph G;  // graph
  Config starts;  // initial configuration
  Config tasks;   // unique task/goal locations
  std::vector<std::vector<bool> > allowed;  // agent-task compatibility
  const uint N;                           // number of agents

  TAPFInstance(const std::string& map_filename,
               const std::vector<int>& start_indexes,
               const std::vector<std::vector<int> >& task_indexes);
  TAPFInstance(const std::string& yaml_filename,
               const std::string& map_dir = "");
  ~TAPFInstance() {}

  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
