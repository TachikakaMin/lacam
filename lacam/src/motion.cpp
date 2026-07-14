#include "../include/motion.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <fstream>
#include <functional>
#include <map>
#include <queue>
#include <stdexcept>
#include <thread>

namespace
{
  int normalize(int value, int modulus)
  {
    value %= modulus;
    return value < 0 ? value + modulus : value;
  }
  int omega_index(int omega) { return omega + 1; }

  constexpr char kPathMagic[8] = {'M', 'O', 'T', 'P', 'A', 'T', 'H', '2'};
  constexpr std::uint32_t kPathVersion = 2;
  constexpr int kMaxPathCandidates = 500;

  void write_u32(std::ofstream& out, std::uint32_t value)
  {
    out.write(reinterpret_cast<const char*>(&value), sizeof(value));
  }

  void write_u64(std::ofstream& out, std::uint64_t value)
  {
    out.write(reinterpret_cast<const char*>(&value), sizeof(value));
  }

  std::uint32_t read_u32(std::ifstream& in)
  {
    auto value = std::uint32_t();
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
  }

  std::uint64_t read_u64(std::ifstream& in)
  {
    auto value = std::uint64_t();
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
  }

  void hash_mix(std::uint64_t& hash, std::uint64_t value)
  {
    hash ^= value;
    hash *= 1099511628211ULL;
  }
}  // namespace

size_t MotionConfigHasher::operator()(const MotionConfig& config) const
{
  auto seed = config.size();
  for (const auto& state : config) {
    seed ^= std::hash<int>()(state.id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

MotionGraph::MotionGraph(const Graph& graph, MotionParameters parameters)
    : graph_(graph), parameters_(parameters)
{
  if (parameters_.max_speed < 1 || parameters_.rotation_steps < 1 ||
      parameters_.lookahead_horizon < 1) {
    throw std::invalid_argument(
        "motion speed, rotation steps, and horizon must be positive");
  }
  const auto& c = parameters_.costs;
  if (c.stay < 0 || c.forward < 0 || c.rotate_ccw < 0 || c.rotate_cw < 0 ||
      c.keep_speed < 0 || c.accelerate < 0 || c.decelerate < 0) {
    throw std::invalid_argument("motion action costs must be nonnegative");
  }
  heading_phases_ = 4 * parameters_.rotation_steps;
  key_to_state_.assign(static_cast<size_t>(graph_.size()) * heading_phases_ *
                           (parameters_.max_speed + 1) * 3,
                       -1);
  build_states();
  build_transitions();
  path_candidates_.resize(states_.size());
  path_candidates_ready_.assign(states_.size(), 0);
}

std::uint64_t MotionGraph::state_key(int vertex_id, int heading_phase,
                                     int speed, int omega) const
{
  auto key = static_cast<std::uint64_t>(vertex_id);
  key = key * heading_phases_ + heading_phase;
  key = key * (parameters_.max_speed + 1) + speed;
  return key * 3u + omega_index(omega);
}

int MotionGraph::state_id(Vertex* location, int cardinal_heading, int speed,
                          int omega) const
{
  if (cardinal_heading < 0 || cardinal_heading >= 4) return -1;
  return state_id_from_phase(
      location, cardinal_heading * parameters_.rotation_steps, speed, omega);
}

int MotionGraph::state_id_from_phase(Vertex* location, int heading_phase,
                                     int speed, int omega) const
{
  if (location == nullptr || heading_phase < 0 ||
      heading_phase >= heading_phases_ || speed < 0 ||
      speed > parameters_.max_speed || omega < -1 || omega > 1)
    return -1;
  const auto key = state_key(location->id, heading_phase, speed, omega);
  return key >= key_to_state_.size() ? -1 : key_to_state_[key];
}

bool MotionGraph::line_is_clear(Vertex* from, int heading, int cells,
                                Vertex** destination) const
{
  if (from == nullptr || heading < 0 || heading >= 4 || cells < 0) return false;
  const int dx[4] = {1, 0, -1, 0};
  const int dy[4] = {0, 1, 0, -1};
  const auto x0 = from->index % graph_.width;
  const auto y0 = from->index / graph_.width;
  auto last = from;
  for (auto step = 1; step <= cells; ++step) {
    const auto x = x0 + dx[heading] * step;
    const auto y = y0 + dy[heading] * step;
    if (x < 0 || x >= graph_.width || y < 0 || y >= graph_.height) return false;
    last = graph_.U[graph_.width * y + x];
    if (last == nullptr) return false;
  }
  if (destination != nullptr) *destination = last;
  return true;
}

bool MotionGraph::has_stopping_clearance(Vertex* location, int heading,
                                         int speed) const
{
  return line_is_clear(location, heading, speed * (speed + 1) / 2);
}

void MotionGraph::build_states()
{
  for (auto location : graph_.V) {
    for (auto heading = 0; heading < heading_phases_; ++heading) {
      const auto aligned = heading % parameters_.rotation_steps == 0;
      if (aligned) {
        auto s = MotionState{location, heading, 0, 0,
                             static_cast<int>(states_.size())};
        key_to_state_[state_key(location->id, heading, 0, 0)] = s.id;
        states_.push_back(s);
        const auto cardinal = heading / parameters_.rotation_steps;
        for (auto speed = 1; speed <= parameters_.max_speed; ++speed) {
          if (!has_stopping_clearance(location, cardinal, speed)) continue;
          s = MotionState{location, heading, speed, 0,
                          static_cast<int>(states_.size())};
          key_to_state_[state_key(location->id, heading, speed, 0)] = s.id;
          states_.push_back(s);
        }
      } else {
        for (const auto omega : {-1, 1}) {
          auto s = MotionState{location, heading, 0, omega,
                               static_cast<int>(states_.size())};
          key_to_state_[state_key(location->id, heading, 0, omega)] = s.id;
          states_.push_back(s);
        }
      }
    }
  }
  outgoing_.resize(states_.size());
  reversed_.resize(states_.size());
}

int MotionGraph::action_cost(MotionMoveAction move,
                             MotionSpeedAction speed_change) const
{
  const auto& c = parameters_.costs;
  auto cost = move == MotionMoveAction::STAY
                  ? c.stay
                  : move == MotionMoveAction::FORWARD
                        ? c.forward
                        : move == MotionMoveAction::ROTATE_CCW ? c.rotate_ccw
                                                               : c.rotate_cw;
  cost += speed_change == MotionSpeedAction::KEEP
              ? c.keep_speed
              : speed_change == MotionSpeedAction::ACCELERATE ? c.accelerate
                                                              : c.decelerate;
  return std::max(1, cost);
}

std::vector<int> MotionGraph::swept_cells(Vertex* from, Vertex* to) const
{
  std::vector<int> cells;
  if (from == nullptr || to == nullptr) return cells;
  auto x = from->index % graph_.width;
  auto y = from->index / graph_.width;
  const auto x2 = to->index % graph_.width;
  const auto y2 = to->index / graph_.width;
  if (x != x2 && y != y2) return cells;
  const auto dx = x2 == x ? 0 : (x2 > x ? 1 : -1);
  const auto dy = y2 == y ? 0 : (y2 > y ? 1 : -1);
  while (true) {
    cells.push_back(graph_.width * y + x);
    if (x == x2 && y == y2) break;
    x += dx;
    y += dy;
  }
  return cells;
}

void MotionGraph::add_transition(int from, Vertex* destination, int heading,
                                 int speed, int omega, MotionMoveAction move,
                                 MotionSpeedAction speed_change,
                                 const std::vector<int>& cells)
{
  const auto to = state_id_from_phase(destination, heading, speed, omega);
  if (to < 0) return;
  const auto cost = action_cost(move, speed_change);
  outgoing_[from].push_back(
      MotionTransition{to, move, speed_change, cost, cells});
  reversed_[to].push_back({from, cost});
}

void MotionGraph::build_transitions()
{
  struct Movement {
    Vertex* destination;
    int heading;
    int omega;
    MotionMoveAction action;
    std::vector<int> cells;
  };
  for (const auto& current : states_) {
    std::vector<Movement> moves;
    const auto aligned = current.heading % parameters_.rotation_steps == 0;
    if (current.speed == 0) {
      if (current.omega == 0) {
        if (parameters_.actions.rotate_ccw) {
          const auto h = normalize(current.heading + 1, heading_phases_);
          moves.push_back({current.location, h,
                           h % parameters_.rotation_steps ? 1 : 0,
                           MotionMoveAction::ROTATE_CCW,
                           swept_cells(current.location, current.location)});
        }
        if (parameters_.actions.rotate_cw) {
          const auto h = normalize(current.heading - 1, heading_phases_);
          moves.push_back({current.location, h,
                           h % parameters_.rotation_steps ? -1 : 0,
                           MotionMoveAction::ROTATE_CW,
                           swept_cells(current.location, current.location)});
        }
        if (parameters_.actions.stay)
          moves.push_back({current.location, current.heading, 0,
                           MotionMoveAction::STAY,
                           swept_cells(current.location, current.location)});
      } else {
        const auto enabled = current.omega > 0 ? parameters_.actions.rotate_ccw
                                               : parameters_.actions.rotate_cw;
        if (enabled) {
          const auto h =
              normalize(current.heading + current.omega, heading_phases_);
          moves.push_back({current.location, h,
                           h % parameters_.rotation_steps ? current.omega : 0,
                           current.omega > 0 ? MotionMoveAction::ROTATE_CCW
                                             : MotionMoveAction::ROTATE_CW,
                           swept_cells(current.location, current.location)});
        }
      }
    }
    if (current.speed > 0 && aligned && parameters_.actions.forward) {
      Vertex* destination = nullptr;
      if (line_is_clear(current.location,
                        current.heading / parameters_.rotation_steps,
                        current.speed, &destination))
        moves.push_back({destination, current.heading, 0,
                         MotionMoveAction::FORWARD,
                         swept_cells(current.location, destination)});
    }
    for (const auto& move : moves) {
      const auto next_aligned = move.heading % parameters_.rotation_steps == 0;
      const auto can_accelerate = parameters_.actions.accelerate &&
                                  next_aligned && move.omega == 0 &&
                                  current.speed < parameters_.max_speed;
      if (move.action == MotionMoveAction::STAY && can_accelerate)
        add_transition(current.id, move.destination, move.heading,
                       current.speed + 1, 0, move.action,
                       MotionSpeedAction::ACCELERATE, move.cells);
      if (parameters_.actions.keep_speed)
        add_transition(current.id, move.destination, move.heading,
                       current.speed, move.omega, move.action,
                       MotionSpeedAction::KEEP, move.cells);
      if (move.action != MotionMoveAction::STAY && can_accelerate)
        add_transition(current.id, move.destination, move.heading,
                       current.speed + 1, 0, move.action,
                       MotionSpeedAction::ACCELERATE, move.cells);
      if (parameters_.actions.decelerate && current.speed > 0)
        add_transition(current.id, move.destination, move.heading,
                       current.speed - 1, 0, move.action,
                       MotionSpeedAction::DECELERATE, move.cells);
    }
  }
}

bool MotionGraph::is_goal(int id, Vertex* goal, int cardinal_heading) const
{
  if (id < 0 || id >= size() || goal == nullptr) return false;
  const auto& s = states_[id];
  if (s.location != goal || s.speed != 0 || s.omega != 0 ||
      s.heading % parameters_.rotation_steps != 0)
    return false;
  return cardinal_heading < 0 ||
         s.heading == cardinal_heading * parameters_.rotation_steps;
}

int MotionGraph::distance(int state_id, int task_id, Vertex* goal,
                          int cardinal_heading)
{
  if (state_id < 0 || state_id >= size() || goal == nullptr) return kInf;
  auto key =
      static_cast<std::uint64_t>(task_id + 1) * 5u + cardinal_heading + 1;
  auto it = distance_cache_.find(key);
  if (it == distance_cache_.end()) {
    std::vector<int> dist(states_.size(), kInf);
    using Item = std::pair<int, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    for (const auto& s : states_)
      if (is_goal(s.id, goal, cardinal_heading)) {
        dist[s.id] = 0;
        open.push({0, s.id});
      }
    while (!open.empty()) {
      const auto [d, to] = open.top();
      open.pop();
      if (d != dist[to]) continue;
      for (const auto& [from, cost] : reversed_[to])
        if (d + cost < dist[from]) {
          dist[from] = d + cost;
          open.push({dist[from], from});
        }
    }
    it = distance_cache_.emplace(key, std::move(dist)).first;
  }
  return it->second[state_id];
}

const MotionTransition* MotionGraph::transition(int from, int to) const
{
  if (from < 0 || from >= size()) return nullptr;
  const MotionTransition* best = nullptr;
  for (const auto& edge : outgoing_[from])
    if (edge.to == to && (best == nullptr || edge.cost < best->cost))
      best = &edge;
  return best;
}

int MotionGraph::transition_cost(int from, int to) const
{
  const auto edge = transition(from, to);
  return edge == nullptr ? kInf : edge->cost;
}

std::vector<int> MotionGraph::stopping_path(int state_id, int horizon) const
{
  std::vector<int> path;
  path.reserve(std::max(0, horizon));
  auto current = state_id;
  for (auto t = 0; t < horizon; ++t) {
    const auto& s = state(current);
    const MotionTransition* chosen = nullptr;
    for (const auto& edge : successors(current)) {
      const auto& next = state(edge.to);
      const auto preferred =
          s.omega != 0
              ? next.speed == 0 && (next.omega == s.omega || next.omega == 0)
              : s.speed > 0
                    ? next.speed == s.speed - 1 &&
                          edge.speed_change == MotionSpeedAction::DECELERATE
                    : edge.to == current;
      if (preferred && (chosen == nullptr || edge.cost < chosen->cost))
        chosen = &edge;
    }
    if (chosen == nullptr) break;
    current = chosen->to;
    path.push_back(current);
  }
  return path;
}

MotionPathSet MotionGraph::build_path_candidates(int state_id) const
{
  struct Candidate {
    std::vector<int> states;
    int cost = 0;
    int moves = 0;
  };

  const auto horizon = parameters_.lookahead_horizon;
  auto frontier = std::vector<Candidate>(1);
  for (auto depth = 0; depth < horizon; ++depth) {
    auto next_frontier = std::vector<Candidate>();
    for (const auto& prefix : frontier) {
      const auto from = prefix.states.empty() ? state_id : prefix.states.back();
      for (const auto& edge : successors(from)) {
        auto candidate = prefix;
        candidate.states.push_back(edge.to);
        candidate.cost += edge.cost;
        if (!prefix.states.empty() &&
            (state(from).location != state(edge.to).location ||
             state(from).heading != state(edge.to).heading)) {
          ++candidate.moves;
        }
        next_frontier.push_back(std::move(candidate));
      }
    }
    frontier = std::move(next_frontier);
    if (frontier.empty()) break;
  }

  auto best = std::map<int, Candidate>();
  for (auto& candidate : frontier) {
    if (candidate.states.size() != static_cast<size_t>(horizon)) continue;
    const auto endpoint = candidate.states.back();
    if (state(endpoint).speed != 0) continue;
    auto iter = best.find(endpoint);
    if (iter == best.end() || candidate.moves < iter->second.moves ||
        (candidate.moves == iter->second.moves &&
         candidate.cost < iter->second.cost)) {
      best[endpoint] = std::move(candidate);
    }
  }

  auto selected = std::vector<Candidate>();
  selected.reserve(std::min<size_t>(kMaxPathCandidates, best.size() + 1));
  for (auto& [endpoint, candidate] : best) {
    (void)endpoint;
    if (selected.size() >= kMaxPathCandidates - 1) break;
    selected.push_back(std::move(candidate));
  }

  auto stop = stopping_path(state_id, horizon);
  auto stop_candidate_index = -1;
  if (stop.size() == static_cast<size_t>(horizon)) {
    auto stop_candidate = Candidate();
    stop_candidate.states = std::move(stop);
    auto from = state_id;
    for (const auto to : stop_candidate.states) {
      stop_candidate.cost += transition_cost(from, to);
      from = to;
    }
    for (size_t k = 1; k < stop_candidate.states.size(); ++k) {
      if (state(stop_candidate.states[k - 1]).location !=
              state(stop_candidate.states[k]).location ||
          state(stop_candidate.states[k - 1]).heading !=
              state(stop_candidate.states[k]).heading) {
        ++stop_candidate.moves;
      }
    }
    stop_candidate_index = static_cast<int>(selected.size());
    selected.push_back(std::move(stop_candidate));
  }

  auto result = MotionPathSet();
  result.states.reserve(selected.size() * horizon);
  result.stop_candidate = stop_candidate_index;
  result.packed_count = static_cast<int>(selected.size());
  for (auto& candidate : selected) {
    result.states.insert(result.states.end(), candidate.states.begin(),
                         candidate.states.end());
  }
  return result;
}

const MotionPathSet& MotionGraph::path_candidates(int state_id)
{
  if (state_id < 0 || state_id >= size()) {
    throw std::out_of_range("invalid motion state for path candidates");
  }
  if (!path_candidates_ready_[state_id]) {
    if (mapped_path_base_ != nullptr &&
        mapped_path_offsets_.size() == states_.size() + 1) {
      const auto begin = mapped_path_offsets_[state_id];
      const auto end = mapped_path_offsets_[state_id + 1];
      if (end < begin || end - begin < 2) {
        throw std::runtime_error("corrupt motion path cache offsets");
      }
      const auto count = mapped_path_base_[begin];
      const auto stop = mapped_path_base_[begin + 1];
      const auto expected =
          std::uint64_t{2} +
          static_cast<std::uint64_t>(count) * parameters_.lookahead_horizon;
      if (count > kMaxPathCandidates || end - begin != expected ||
          (stop != std::numeric_limits<std::uint32_t>::max() &&
           stop >= count)) {
        throw std::runtime_error("corrupt motion path cache record");
      }
      auto& paths = path_candidates_[state_id];
      paths.stop_candidate = stop == std::numeric_limits<std::uint32_t>::max()
                                 ? -1
                                 : static_cast<int>(stop);
      paths.packed = mapped_path_base_ + begin + 2;
      paths.packed_count = static_cast<int>(count);
      paths.packed_horizon = parameters_.lookahead_horizon;
      paths.packed_state_count = size();
    } else {
      path_candidates_[state_id] = build_path_candidates(state_id);
    }
    path_candidates_ready_[state_id] = 1;
  }
  return path_candidates_[state_id];
}

void MotionGraph::precompute_path_candidates(int workers)
{
  const auto thread_count = std::max(
      1, std::min(size(), workers > 0
                              ? workers
                              : static_cast<int>(std::max(
                                    1u, std::thread::hardware_concurrency()))));
  auto next_state = std::atomic<int>(0);
  auto pool = std::vector<std::thread>();
  pool.reserve(thread_count);
  for (auto worker = 0; worker < thread_count; ++worker) {
    pool.emplace_back([&] {
      while (true) {
        const auto state_id = next_state.fetch_add(1);
        if (state_id >= size()) break;
        if (path_candidates_ready_[state_id]) continue;
        path_candidates_[state_id] = build_path_candidates(state_id);
        path_candidates_ready_[state_id] = 1;
      }
    });
  }
  for (auto& thread : pool) thread.join();
}

std::uint64_t MotionGraph::path_cache_signature() const
{
  auto hash = std::uint64_t{1469598103934665603ULL};
  hash_mix(hash, static_cast<std::uint64_t>(graph_.width));
  hash_mix(hash, static_cast<std::uint64_t>(graph_.height));
  hash_mix(hash, static_cast<std::uint64_t>(graph_.size()));
  for (const auto cell : graph_.cell_types) {
    hash_mix(hash, static_cast<unsigned char>(cell));
  }
  hash_mix(hash, static_cast<std::uint64_t>(parameters_.max_speed));
  hash_mix(hash, static_cast<std::uint64_t>(parameters_.rotation_steps));
  hash_mix(hash, static_cast<std::uint64_t>(parameters_.lookahead_horizon));
  const auto& a = parameters_.actions;
  const auto action_mask = static_cast<std::uint64_t>(a.stay) |
                           (static_cast<std::uint64_t>(a.forward) << 1) |
                           (static_cast<std::uint64_t>(a.rotate_ccw) << 2) |
                           (static_cast<std::uint64_t>(a.rotate_cw) << 3) |
                           (static_cast<std::uint64_t>(a.keep_speed) << 4) |
                           (static_cast<std::uint64_t>(a.accelerate) << 5) |
                           (static_cast<std::uint64_t>(a.decelerate) << 6);
  hash_mix(hash, action_mask);
  const auto& c = parameters_.costs;
  for (const auto cost : {c.stay, c.forward, c.rotate_ccw, c.rotate_cw,
                          c.keep_speed, c.accelerate, c.decelerate}) {
    hash_mix(hash, static_cast<std::uint64_t>(cost));
  }
  return hash;
}

std::uint64_t MotionGraph::path_candidate_count() const
{
  if (mapped_path_base_ != nullptr) return mapped_path_candidate_count_;
  auto count = std::uint64_t();
  for (size_t state_id = 0; state_id < path_candidates_.size(); ++state_id) {
    if (path_candidates_ready_[state_id]) {
      count += static_cast<std::uint64_t>(path_candidates_[state_id].size());
    }
  }
  return count;
}

bool MotionGraph::load_path_cache(const std::filesystem::path& cache_path)
{
  auto in = std::ifstream(cache_path, std::ios::binary);
  if (!in) return false;
  char magic[8];
  in.read(magic, sizeof(magic));
  if (!in ||
      !std::equal(std::begin(magic), std::end(magic), std::begin(kPathMagic)) ||
      read_u32(in) != kPathVersion || read_u64(in) != path_cache_signature() ||
      read_u32(in) != static_cast<std::uint32_t>(size()) ||
      read_u32(in) !=
          static_cast<std::uint32_t>(parameters_.lookahead_horizon)) {
    return false;
  }
  const auto total_candidates = read_u64(in);
  auto offsets = std::vector<std::uint64_t>(states_.size() + 1);
  for (auto& offset : offsets) offset = read_u64(in);
  const auto data_start = in.tellg();
  if (!in || data_start < 0 || offsets.front() != 0 ||
      !std::is_sorted(offsets.begin(), offsets.end())) {
    return false;
  }
  const auto expected_bytes = static_cast<std::uint64_t>(data_start) +
                              offsets.back() * sizeof(std::uint32_t);
  auto error = std::error_code();
  if (std::filesystem::file_size(cache_path, error) != expected_bytes ||
      error) {
    return false;
  }

  const auto descriptor = open(cache_path.c_str(), O_RDONLY);
  if (descriptor < 0) return false;
  const auto mapped =
      mmap(nullptr, expected_bytes, PROT_READ, MAP_PRIVATE, descriptor, 0);
  close(descriptor);
  if (mapped == MAP_FAILED) return false;
  auto mapping = std::shared_ptr<const std::uint8_t>(
      static_cast<const std::uint8_t*>(mapped),
      [expected_bytes](const std::uint8_t* address) {
        munmap(const_cast<std::uint8_t*>(address), expected_bytes);
      });
  mapped_path_file_ = std::move(mapping);
  mapped_path_base_ = reinterpret_cast<const std::uint32_t*>(
      mapped_path_file_.get() + static_cast<size_t>(data_start));
  mapped_path_offsets_ = std::move(offsets);
  mapped_path_candidate_count_ = total_candidates;
  path_candidates_.assign(states_.size(), MotionPathSet());
  path_candidates_ready_.assign(states_.size(), 0);
  return true;
}

void MotionGraph::save_path_cache(const std::filesystem::path& cache_path) const
{
  if (std::find(path_candidates_ready_.begin(), path_candidates_ready_.end(),
                0) != path_candidates_ready_.end()) {
    throw std::runtime_error("motion path cache is not fully precomputed");
  }
  if (cache_path.has_parent_path()) {
    std::filesystem::create_directories(cache_path.parent_path());
  }
  auto temporary = cache_path;
  temporary += ".tmp";
  auto out = std::ofstream(temporary, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to write motion path cache: " +
                             temporary.string());
  }
  out.write(kPathMagic, sizeof(kPathMagic));
  write_u32(out, kPathVersion);
  write_u64(out, path_cache_signature());
  write_u32(out, static_cast<std::uint32_t>(size()));
  write_u32(out, static_cast<std::uint32_t>(parameters_.lookahead_horizon));
  write_u64(out, path_candidate_count());
  const auto horizon = parameters_.lookahead_horizon;
  auto offsets = std::vector<std::uint64_t>(states_.size() + 1, 0);
  for (auto state_id = size_t{0}; state_id < path_candidates_.size();
       ++state_id) {
    offsets[state_id + 1] =
        offsets[state_id] + 2 +
        static_cast<std::uint64_t>(path_candidates_[state_id].size()) * horizon;
  }
  for (const auto offset : offsets) write_u64(out, offset);
  for (const auto& paths : path_candidates_) {
    write_u32(out, static_cast<std::uint32_t>(paths.size()));
    write_u32(out, paths.stop_candidate < 0
                       ? std::numeric_limits<std::uint32_t>::max()
                       : static_cast<std::uint32_t>(paths.stop_candidate));
    for (auto candidate = 0; candidate < paths.size(); ++candidate) {
      for (auto t = 0; t < horizon; ++t) {
        write_u32(out, static_cast<std::uint32_t>(
                           paths.state(candidate, t, horizon)));
      }
    }
  }
  out.close();
  if (!out) {
    throw std::runtime_error("failed to finish motion path cache: " +
                             temporary.string());
  }
  auto error = std::error_code();
  std::filesystem::rename(temporary, cache_path, error);
  if (error) {
    std::filesystem::remove(cache_path, error);
    error.clear();
    std::filesystem::rename(temporary, cache_path, error);
  }
  if (error) {
    throw std::runtime_error("failed to install motion path cache: " +
                             error.message());
  }
}
