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
    std::vector<int> start_headings;
    std::vector<std::vector<int> > task_headings;
  };

  static YamlData load_yaml(const std::string& yaml_filename,
                            const std::string& map_dir);
  explicit TAPFInstance(const YamlData& data);

public:
  const Graph G;  // graph
  const std::string source_map_filename;
  Config starts;  // initial configuration
  Config tasks;   // unique task/goal locations
  // Cardinal headings use 0=east, 1=south, 2=west, 3=north.  A task heading
  // of -1 accepts any cardinal heading.  Starts default to east for backward
  // compatible programmatic construction.
  std::vector<int> start_headings;
  std::vector<int> task_headings;
  std::vector<int> task_keys;
  std::vector<std::vector<bool> > allowed;  // agent-task compatibility
  std::vector<std::vector<int> > assignment_cost_offsets;
  std::vector<std::vector<int> > assignment_distance_scales;
  std::vector<std::vector<int> > assignment_service_durations;
  std::vector<float> agent_priority_offsets;
  int assignment_distance_scale;
  const uint N;  // number of agents

  TAPFInstance(
      const std::string& map_filename, const std::vector<int>& start_indexes,
      const std::vector<std::vector<int> >& task_indexes,
      const std::vector<std::vector<int> >& task_cost_offsets = {},
      int task_distance_scale = 1,
      const std::vector<std::vector<int> >& task_distance_scales = {},
      const std::vector<float>& agent_priority_offsets = {},
      bool preserve_duplicate_tasks = false,
      const std::vector<std::vector<int> >& task_key_options = {},
      const std::vector<std::vector<int> >& task_service_durations = {},
      const std::vector<int>& initial_headings = {},
      const std::vector<std::vector<int> >& task_heading_options = {});
  TAPFInstance(const std::string& yaml_filename,
               const std::string& map_dir = "");
  ~TAPFInstance() {}

  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
