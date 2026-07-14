/* Discrete MAWPF warehouse motion model for LaCAM-TAPF. */
#pragma once

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "graph.hpp"

enum class MotionMoveAction {
  STAY = 0,
  FORWARD = 1,
  ROTATE_CCW = 2,
  ROTATE_CW = 3
};
enum class MotionSpeedAction { KEEP = 0, ACCELERATE = 1, DECELERATE = 2 };

struct MotionActionSet {
  bool stay = true;
  bool forward = true;
  bool rotate_ccw = true;
  bool rotate_cw = true;
  bool keep_speed = true;
  bool accelerate = true;
  bool decelerate = true;
};

struct MotionActionCosts {
  int stay = 1;
  int forward = 1;
  int rotate_ccw = 1;
  int rotate_cw = 1;
  int keep_speed = 0;
  int accelerate = 0;
  int decelerate = 0;
};

struct MotionParameters {
  bool enabled = false;
  int max_speed = 2;
  int rotation_steps = 2;
  int lookahead_horizon = 6;
  bool follower_collisions = true;
  MotionActionSet actions;
  MotionActionCosts costs;
};

struct MotionState {
  Vertex* location = nullptr;
  // Phase in [0, 4*T_rot); exact cardinal headings are multiples of T_rot.
  int heading = 0;
  int speed = 0;
  // Rotation commitment: -1 clockwise, 0 idle, +1 counter-clockwise.
  int omega = 0;
  int id = -1;
  bool operator==(const MotionState& o) const
  {
    return location == o.location && heading == o.heading && speed == o.speed &&
           omega == o.omega;
  }
};

using MotionConfig = std::vector<MotionState>;
using MotionSolution = std::vector<MotionConfig>;

struct MotionTransition {
  int to = -1;
  MotionMoveAction move = MotionMoveAction::STAY;
  MotionSpeedAction speed_change = MotionSpeedAction::KEEP;
  int cost = 1;
  // Geometric cells occupied during the timestep, including both endpoints.
  std::vector<int> swept_cells;
};

struct MotionConfigHasher {
  size_t operator()(const MotionConfig& config) const;
};

// Compact storage for all fixed-horizon paths that start at one motion state.
// Candidate k occupies states[k * horizon, (k + 1) * horizon).
struct MotionPathSet {
  std::vector<int> states;
  int stop_candidate = -1;
  const std::uint32_t* packed = nullptr;
  int packed_count = 0;
  int packed_horizon = 0;
  int packed_state_count = 0;

  int size() const { return packed_count; }
  bool empty() const { return size() == 0; }
  int state(int candidate, int timestep, int horizon) const
  {
    if (packed != nullptr) {
      const auto value =
          packed[static_cast<size_t>(candidate) * packed_horizon + timestep];
      if (value >= static_cast<std::uint32_t>(packed_state_count)) {
        throw std::out_of_range("invalid state in motion path cache");
      }
      return static_cast<int>(value);
    }
    return states.at(static_cast<size_t>(candidate) * horizon + timestep);
  }
};

class MotionGraph
{
public:
  static constexpr int kInf = std::numeric_limits<int>::max() / 8;
  MotionGraph(const Graph& graph, MotionParameters parameters);

  const MotionParameters& parameters() const { return parameters_; }
  int size() const { return static_cast<int>(states_.size()); }
  const MotionState& state(int id) const { return states_.at(id); }
  const std::vector<MotionTransition>& successors(int id) const
  {
    return outgoing_.at(id);
  }
  int state_id(Vertex* location, int cardinal_heading, int speed = 0,
               int omega = 0) const;
  int state_id_from_phase(Vertex* location, int heading_phase, int speed,
                          int omega) const;
  bool is_goal(int state_id, Vertex* goal, int cardinal_heading = -1) const;
  int distance(int state_id, int task_id, Vertex* goal,
               int cardinal_heading = -1);
  const MotionTransition* transition(int from, int to) const;
  int transition_cost(int from, int to) const;
  std::vector<int> stopping_path(int state_id, int horizon) const;
  const MotionPathSet& path_candidates(int state_id);
  void precompute_path_candidates(int workers = 1);
  bool load_path_cache(const std::filesystem::path& cache_path);
  void save_path_cache(const std::filesystem::path& cache_path) const;
  std::uint64_t path_cache_signature() const;
  std::uint64_t path_candidate_count() const;

private:
  const Graph& graph_;
  MotionParameters parameters_;
  int heading_phases_ = 4;
  std::vector<MotionState> states_;
  std::vector<std::vector<MotionTransition>> outgoing_;
  std::vector<std::vector<std::pair<int, int>>> reversed_;
  std::vector<int> key_to_state_;
  std::unordered_map<std::uint64_t, std::vector<int>> distance_cache_;
  std::vector<MotionPathSet> path_candidates_;
  std::vector<std::uint8_t> path_candidates_ready_;
  std::shared_ptr<const std::uint8_t> mapped_path_file_;
  const std::uint32_t* mapped_path_base_ = nullptr;
  std::vector<std::uint64_t> mapped_path_offsets_;
  std::uint64_t mapped_path_candidate_count_ = 0;

  std::uint64_t state_key(int vertex_id, int heading_phase, int speed,
                          int omega) const;
  bool line_is_clear(Vertex* from, int cardinal_heading, int cells,
                     Vertex** destination = nullptr) const;
  bool has_stopping_clearance(Vertex* location, int cardinal_heading,
                              int speed) const;
  int action_cost(MotionMoveAction move, MotionSpeedAction speed_change) const;
  void build_states();
  void build_transitions();
  void add_transition(int from, Vertex* destination, int heading_phase,
                      int speed, int omega, MotionMoveAction move,
                      MotionSpeedAction speed_change,
                      const std::vector<int>& swept_cells);
  std::vector<int> swept_cells(Vertex* from, Vertex* to) const;
  MotionPathSet build_path_candidates(int state_id) const;
};
