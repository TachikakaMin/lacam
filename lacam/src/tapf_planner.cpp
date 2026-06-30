#include "../include/tapf_planner.hpp"

#include <limits>

namespace
{
  constexpr int kDeliveryLocationKeyBase = 1000000000;

  struct ServiceConfigKey {
    Config C;
    std::vector<int> service_assignment;
    std::vector<int> service_progress;
    std::vector<bool> service_committed;
    std::vector<bool> satisfied;

    bool operator==(const ServiceConfigKey& other) const
    {
      return C == other.C && service_assignment == other.service_assignment &&
             service_committed == other.service_committed &&
             service_progress == other.service_progress &&
             satisfied == other.satisfied;
    }
  };

  struct ServiceConfigKeyHasher {
    size_t operator()(const ServiceConfigKey& key) const
    {
      auto seed = ConfigHasher()(key.C);
      for (const auto task : key.service_assignment) {
        seed ^= std::hash<int>()(task) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      for (const auto progress : key.service_progress) {
        seed ^=
            std::hash<int>()(progress) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      for (const auto committed : key.service_committed) {
        seed ^= std::hash<int>()(committed ? 1 : 0) + 0x9e3779b9 +
                (seed << 6) + (seed >> 2);
      }
      for (const auto done : key.satisfied) {
        seed ^= std::hash<int>()(done ? 1 : 0) + 0x9e3779b9 + (seed << 6) +
                (seed >> 2);
      }
      return seed;
    }
  };

  bool is_open_viable(const TAPFNode* node, const TAPFNode* goal)
  {
    return !node->search_tree.empty() && (goal == nullptr || node->f < goal->g);
  }

  unsigned focal_score(const TAPFNode* node, TAPFFocalTieBreak tie_break)
  {
    switch (tie_break) {
      case TAPFFocalTieBreak::ANTI_WAIT:
        return 8 * node->non_goal_waits + 4 * node->reversals +
               2 * node->distance_increases + node->settled_pushes;
      case TAPFFocalTieBreak::ANTI_ZIGZAG:
        return 8 * node->reversals + 4 * node->distance_increases +
               2 * node->settled_pushes + node->non_goal_waits;
      case TAPFFocalTieBreak::ANTI_PUSH:
        return 8 * node->settled_pushes + 4 * node->reversals +
               2 * node->distance_increases + node->non_goal_waits;
      case TAPFFocalTieBreak::ANTI_ALL:
        return 10 * node->settled_pushes + 6 * node->reversals +
               3 * node->non_goal_waits + 2 * node->distance_increases;
      case TAPFFocalTieBreak::H:
      default:
        return node->h;
    }
  }

  bool focal_better(const TAPFNode* a, const TAPFNode* b,
                    TAPFFocalTieBreak tie_break)
  {
    if (a->h != b->h) return a->h < b->h;
    const auto a_score = focal_score(a, tie_break);
    const auto b_score = focal_score(b, tie_break);
    if (a_score != b_score) return a_score < b_score;
    if (a->f != b->f) return a->f < b->f;
    if (a->g != b->g) return a->g > b->g;
    return a->depth < b->depth;
  }

  bool is_delivery_task(const TAPFInstance* ins, int task)
  {
    return ins != nullptr && task >= 0 &&
           task < static_cast<int>(ins->task_keys.size()) &&
           ins->task_keys[task] >= kDeliveryLocationKeyBase;
  }

  int service_duration_for_task(const TAPFInstance* ins,
                                const TAPFSearchConfig& search_config, int task)
  {
    const auto duration = is_delivery_task(ins, task)
                              ? search_config.delivery_service_duration
                              : search_config.pickup_service_duration;
    return std::max(0, duration);
  }

  int assignment_service_duration_for_task(const TAPFInstance* ins, int agent,
                                           int task)
  {
    if (ins == nullptr || agent < 0 || task < 0 ||
        agent >= static_cast<int>(ins->assignment_service_durations.size()) ||
        task >= static_cast<int>(
                    ins->assignment_service_durations[agent].size())) {
      return 0;
    }
    return std::max(0, ins->assignment_service_durations[agent][task]);
  }

  bool is_real_service_task(const TAPFInstance* ins, int task)
  {
    return ins != nullptr && task >= 0 &&
           task < static_cast<int>(ins->tasks.size()) &&
           (task >= static_cast<int>(ins->task_keys.size()) ||
            ins->task_keys[task] >= 0);
  }
}  // namespace

TAPFConstraint::TAPFConstraint()
    : who(std::vector<int>()), where(Vertices()), depth(0)
{
}

TAPFConstraint::TAPFConstraint(TAPFConstraint* parent, int i, Vertex* v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
}

TAPFConstraint::~TAPFConstraint(){};

TAPFNode::TAPFNode(Config _C, TAPFDistTable& D, const TAPFInstance* ins,
                   std::vector<int> _assignment,
                   TAPFAssignmentState _assignment_state,
                   const TAPFSearchConfig& search_config,
                   const std::vector<int>& _service_assignment,
                   const std::vector<int>& _service_progress,
                   const std::vector<bool>& _service_committed,
                   const std::vector<bool>& _satisfied,
                   const std::vector<int>& _satisfied_assignment,
                   TAPFNode* _parent)
    : C(_C),
      parent(_parent),
      neighbor(std::set<TAPFNode*>()),
      assignment(_assignment),
      assignment_state(_assignment_state),
      service_assignment(C.size(), -1),
      service_progress(C.size(), 0),
      service_committed(C.size(), false),
      satisfied(C.size(), false),
      satisfied_assignment(C.size(), -1),
      queued(false),
      g(0),
      h(0),
      f(0),
      depth(parent == nullptr ? 0 : parent->depth + 1),
      non_goal_waits(parent == nullptr ? 0 : parent->non_goal_waits),
      reversals(parent == nullptr ? 0 : parent->reversals),
      distance_increases(parent == nullptr ? 0 : parent->distance_increases),
      settled_pushes(parent == nullptr ? 0 : parent->settled_pushes),
      priorities(C.size(), 0),
      order(C.size(), 0),
      search_tree(std::queue<TAPFConstraint*>())
{
  if (search_config.service_goal_mode) {
    if (_service_assignment.size() == C.size()) {
      service_assignment = _service_assignment;
    }
    if (_service_progress.size() == C.size()) {
      service_progress = _service_progress;
    }
    if (_service_committed.size() == C.size()) {
      service_committed = _service_committed;
    }
    if (_satisfied.size() == C.size()) satisfied = _satisfied;
    if (_satisfied_assignment.size() == C.size()) {
      satisfied_assignment = _satisfied_assignment;
    }
  }
  search_tree.push(new TAPFConstraint());
  if (parent != nullptr) parent->neighbor.insert(this);
  refresh_priority(D, ins, search_config);
  refresh_search_metrics(D, ins, search_config);
}

TAPFNode::~TAPFNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

void TAPFNode::discard_search_tree()
{
  while (!search_tree.empty()) search_tree.pop();
}

void TAPFNode::refresh_priority(TAPFDistTable& D, const TAPFInstance* ins,
                                const TAPFSearchConfig& search_config)
{
  const auto N = C.size();
  const auto opportunistic_service = search_config.service_goal_mode &&
                                     search_config.service_commit_agents > 0;
  if (parent == nullptr) {
    for (size_t i = 0; i < N; ++i) {
      const auto offset = ins == nullptr || ins->agent_priority_offsets.empty()
                              ? 0.0f
                              : ins->agent_priority_offsets[i];
      const auto task = search_config.service_goal_mode && satisfied[i] &&
                                i < satisfied_assignment.size() &&
                                satisfied_assignment[i] >= 0
                            ? satisfied_assignment[i]
                            : assignment[i];
      const auto d = D.get(task, C[i]);
      priorities[i] =
          opportunistic_service ? offset - (float)d / N : (float)d / N + offset;
    }
  } else {
    for (size_t i = 0; i < N; ++i) {
      const auto task = search_config.service_goal_mode && satisfied[i] &&
                                i < satisfied_assignment.size() &&
                                satisfied_assignment[i] >= 0
                            ? satisfied_assignment[i]
                            : assignment[i];
      if (D.get(task, C[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int i, int j) { return priorities[i] > priorities[j]; });
}

void TAPFNode::refresh_search_metrics(TAPFDistTable& D, const TAPFInstance* ins,
                                      const TAPFSearchConfig& search_config)
{
  if (parent == nullptr) return;

  for (size_t i = 0; i < C.size(); ++i) {
    if (search_config.service_goal_mode && satisfied[i]) continue;
    const auto task = assignment[i];
    const auto goal = ins->tasks[task];
    if (C[i] == parent->C[i] && C[i] != goal) ++non_goal_waits;
    if (parent->parent != nullptr && C[i] == parent->parent->C[i] &&
        C[i] != parent->C[i]) {
      ++reversals;
    }
    if (D.get(task, C[i]) > D.get(task, parent->C[i])) ++distance_increases;
  }

  for (size_t pusher = 0; pusher < C.size(); ++pusher) {
    if (C[pusher] == parent->C[pusher]) continue;
    for (size_t pushed = 0; pushed < C.size(); ++pushed) {
      if (pusher == pushed || C[pusher] != parent->C[pushed] ||
          C[pushed] == parent->C[pushed]) {
        continue;
      }
      const auto pushed_goal = ins->tasks[assignment[pushed]];
      if (search_config.service_goal_mode && parent->satisfied[pushed]) {
        continue;
      }
      if (parent->C[pushed] == pushed_goal) ++settled_pushes;
    }
  }
}

TAPFPlanner::TAPFPlanner(const TAPFInstance* _ins, const Deadline* _deadline,
                         std::mt19937* _MT, int _verbose, int _sticky_penalty,
                         float _restart_rate, bool _anytime, TAPFStats* _stats,
                         TAPFSearchConfig _search_config)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      sticky_penalty(_sticky_penalty),
      restart_rate(_restart_rate),
      anytime(_anytime),
      search_config(_search_config),
      force_full_assignment(false),
      stats(_stats),
      assignment_stats(TAPFAssignmentStats()),
      service_required_agents(std::vector<bool>(ins->N, false)),
      N(ins->N),
      V_size(ins->G.size()),
      D(TAPFDistTable(ins)),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr)),
      shared_goal_entry_counts(std::vector<int>(V_size, 0)),
      real_service_vertices(std::vector<bool>(V_size, false))
{
  if (stats != nullptr) *stats = TAPFStats();
  if (search_config.service_goal_mode) {
    for (size_t task = 0; task < ins->tasks.size(); ++task) {
      if (task >= ins->task_keys.size() ||
          ins->task_keys[task] < kDeliveryLocationKeyBase) {
        continue;
      }
      const auto goal = ins->tasks[task];
      if (goal != nullptr) real_service_vertices[goal->id] = true;
    }
  }
}

Solution TAPFPlanner::solve(std::vector<int>* final_assignment,
                            std::vector<std::vector<int> >* assignment_schedule)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart TAPF search");
  if (final_assignment != nullptr) final_assignment->clear();
  if (assignment_schedule != nullptr) assignment_schedule->clear();

  std::vector<TAPFNode*> OPEN;
  std::unordered_map<Config, TAPFNode*, ConfigHasher> CLOSED;
  std::unordered_map<ServiceConfigKey, TAPFNode*, ServiceConfigKeyHasher>
      CLOSED_SERVICE;
  TAPFNode* S_goal = nullptr;
  auto C_new = Config(N, nullptr);
  auto service_key = [](const TAPFNode* node) {
    auto service_assignment = node->service_assignment;
    auto service_progress = node->service_progress;
    auto service_committed = node->service_committed;
    for (size_t i = 0; i < node->satisfied.size(); ++i) {
      if (!node->satisfied[i]) continue;
      if (i < service_assignment.size()) service_assignment[i] = -1;
      if (i < service_progress.size()) service_progress[i] = 0;
      if (i < service_committed.size()) service_committed[i] = false;
    }
    return ServiceConfigKey{node->C, service_assignment, service_progress,
                            service_committed, node->satisfied};
  };
  struct ServiceState {
    std::vector<int> service_assignment;
    std::vector<int> service_progress;
    std::vector<bool> service_committed;
    std::vector<bool> satisfied;
    std::vector<int> satisfied_assignment;
    bool valid = true;
  };

  auto service_state_after_move = [&](const TAPFNode* parent, const Config& C) {
    auto state = ServiceState();
    state.service_assignment = std::vector<int>(N, -1);
    state.service_progress = std::vector<int>(N, 0);
    state.service_committed = std::vector<bool>(N, false);
    state.satisfied = std::vector<bool>(N, false);
    state.satisfied_assignment = std::vector<int>(N, -1);
    if (parent != nullptr) {
      state.service_assignment = parent->service_assignment;
      state.service_progress = parent->service_progress;
      state.service_committed = parent->service_committed;
      state.satisfied = parent->satisfied;
      state.satisfied_assignment = parent->satisfied_assignment;
    }
    for (size_t i = 0; i < N; ++i) {
      if (state.satisfied[i]) continue;
      if (parent == nullptr ||
          i >= static_cast<size_t>(parent->assignment.size())) {
        continue;
      }
      auto task = state.service_assignment[i] >= 0 ? state.service_assignment[i]
                                                   : parent->assignment[i];
      if (task >= 0 && task < static_cast<int>(ins->tasks.size()) &&
          C[i] == ins->tasks[task]) {
        if (!is_real_service_task(ins, task)) {
          state.satisfied[i] = true;
          state.satisfied_assignment[i] = task;
          state.service_assignment[i] = -1;
          state.service_progress[i] = 0;
          state.service_committed[i] = false;
          continue;
        }
        const auto starting_service = state.service_assignment[i] < 0;
        if (starting_service) {
          state.service_committed[i] = true;
        }
        const auto stayed_at_service =
            parent->C[i] == C[i] && parent->C[i] == ins->tasks[task];
        state.service_assignment[i] = task;
        const auto required_duration =
            service_duration_for_task(ins, search_config, task);
        if (stayed_at_service) {
          state.service_committed[i] = true;
          state.service_progress[i] =
              std::max(0, state.service_progress[i]) + 1;
        } else if (state.service_progress[i] < 0) {
          state.service_progress[i] = 0;
        }
        if (state.service_progress[i] >= required_duration) {
          state.satisfied[i] = true;
          state.satisfied_assignment[i] = task;
          state.service_assignment[i] = -1;
          state.service_progress[i] = 0;
          state.service_committed[i] = false;
        }
      } else if (state.service_assignment[i] >= 0) {
        if (i < state.service_committed.size() && state.service_committed[i]) {
          state.valid = false;
        } else {
          state.service_assignment[i] = -1;
          state.service_progress[i] = 0;
          if (i < state.service_committed.size()) {
            state.service_committed[i] = false;
          }
        }
      }
    }
    return state;
  };

  auto assignment_service_cost_state_for =
      [&](const Config& C, const std::vector<int>& service_assignment,
          const std::vector<int>& service_progress) {
        auto cost_state = TAPFAssignmentServiceCostState();
        cost_state.partial_task_by_agent.assign(N, -1);
        cost_state.partial_remaining_by_agent.assign(N, 0);
        for (auto i = 0; i < N; ++i) {
          if (i >= static_cast<int>(service_assignment.size()) ||
              i >= static_cast<int>(service_progress.size())) {
            continue;
          }
          const auto task = service_assignment[i];
          if (task < 0 || task >= static_cast<int>(ins->tasks.size()) ||
              C[i] != ins->tasks[task]) {
            continue;
          }
          const auto full_duration =
              assignment_service_duration_for_task(ins, i, task);
          cost_state.partial_task_by_agent[i] = task;
          cost_state.partial_remaining_by_agent[i] =
              std::max(0, full_duration - std::max(0, service_progress[i]));
        }
        return cost_state;
      };

  auto push_open = [&](TAPFNode* node) {
    if (!node->queued && !node->search_tree.empty()) {
      OPEN.push_back(node);
      node->queued = true;
    }
  };

  auto erase_open = [&](const size_t index) {
    OPEN[index]->queued = false;
    OPEN.erase(OPEN.begin() + index);
  };

  auto select_open_index = [&]() -> size_t {
    if (search_config.mode == TAPFSearchMode::DFS) {
      return OPEN.size() - 1;
    }

    auto f_min = std::numeric_limits<unsigned>::max();
    for (auto node : OPEN) {
      if (S_goal == nullptr || is_open_viable(node, S_goal)) {
        f_min = std::min(f_min, node->f);
      }
    }
    if (f_min == std::numeric_limits<unsigned>::max()) return OPEN.size() - 1;

    auto best = OPEN.size();
    const auto bound = search_config.focal_weight * static_cast<double>(f_min);
    for (size_t idx = 0; idx < OPEN.size(); ++idx) {
      auto node = OPEN[idx];
      if (S_goal != nullptr && !is_open_viable(node, S_goal)) continue;
      if (static_cast<double>(node->f) > bound + 1e-9) continue;
      if (best == OPEN.size() ||
          focal_better(node, OPEN[best], search_config.focal_tie_break)) {
        best = idx;
      }
    }
    return best == OPEN.size() ? OPEN.size() - 1 : best;
  };

  auto initial_assignment_state = TAPFAssignmentState();
  initial_assignment_state.init(ins->N, ins->tasks.size());
  auto initial_agents = std::vector<int>(N, 0);
  std::iota(initial_agents.begin(), initial_agents.end(), 0);
  auto initial_service_cost_state = TAPFAssignmentServiceCostState();
  initial_service_cost_state.partial_task_by_agent.assign(N, -1);
  initial_service_cost_state.partial_remaining_by_agent.assign(N, 0);
  if (search_config.service_goal_mode &&
      search_config.initial_optional_service_assignments.size() == N &&
      search_config.initial_optional_service_remaining.size() == N) {
    for (auto i = 0; i < N; ++i) {
      const auto task = search_config.initial_optional_service_assignments[i];
      if (task < 0 || task >= static_cast<int>(ins->tasks.size()) ||
          !ins->allowed[i][task] || ins->starts[i] != ins->tasks[task] ||
          !is_real_service_task(ins, task)) {
        continue;
      }
      initial_service_cost_state.partial_task_by_agent[i] = task;
      initial_service_cost_state.partial_remaining_by_agent[i] =
          std::max(0, search_config.initial_optional_service_remaining[i]);
    }
  }
  auto initial_assignment =
      assign_tapf_tasks_dynamic(*ins, D, ins->starts, initial_assignment_state,
                                initial_agents, true, &assignment_stats,
                                std::vector<int>(), std::vector<bool>(),
                                initial_service_cost_state);
  if (!initial_assignment.feasible) return Solution();
  if (stats != nullptr) {
    stats->initial_assignment = initial_assignment.agent_to_task;
    stats->initial_assignment_cost = initial_assignment.cost;
  }

  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);
  auto initial_satisfied = std::vector<bool>(N, false);
  auto initial_satisfied_assignment = std::vector<int>(N, -1);
  auto initial_service_assignment = std::vector<int>(N, -1);
  auto initial_service_progress = std::vector<int>(N, 0);
  auto initial_service_committed = std::vector<bool>(N, false);
  if (search_config.service_goal_mode) {
    if (search_config.initial_service_assignments.size() == N) {
      for (auto i = 0; i < N; ++i) {
        const auto task = search_config.initial_service_assignments[i];
        if (task < 0) continue;
        if (task >= static_cast<int>(ins->tasks.size()) ||
            !ins->allowed[i][task] || ins->starts[i] != ins->tasks[task]) {
          return Solution();
        }
        initial_assignment.agent_to_task[i] = task;
        service_required_agents[i] = true;
        if (is_real_service_task(ins, task) &&
            service_duration_for_task(ins, search_config, task) == 0) {
          initial_satisfied[i] = true;
          initial_satisfied_assignment[i] = task;
        } else {
          initial_service_assignment[i] = task;
          initial_service_committed[i] = true;
        }
      }
    }
    if (search_config.initial_service_progress.size() == N) {
      for (auto i = 0; i < N; ++i) {
        initial_service_progress[i] =
            std::max(0, search_config.initial_service_progress[i]);
      }
    }
    for (auto i = 0; i < N; ++i) {
      const auto task = initial_service_cost_state.partial_task_by_agent[i];
      if (task < 0 || initial_service_assignment[i] >= 0 ||
          initial_assignment.agent_to_task[i] != task ||
          ins->starts[i] != ins->tasks[task]) {
        continue;
      }
      const auto remaining =
          std::max(0,
                   initial_service_cost_state.partial_remaining_by_agent[i]);
      if (remaining == 0 ||
          service_duration_for_task(ins, search_config, task) == 0) {
        initial_satisfied[i] = true;
        initial_satisfied_assignment[i] = task;
        continue;
      }
      const auto duration = service_duration_for_task(ins, search_config, task);
      initial_service_assignment[i] = task;
      initial_service_progress[i] = std::max(0, duration - remaining);
      initial_service_committed[i] = false;
    }
    for (auto i = 0; i < N; ++i) {
      if (initial_service_assignment[i] >= 0) continue;
      const auto task = initial_assignment.agent_to_task[i];
      if (task < 0 || task >= static_cast<int>(ins->tasks.size()) ||
          ins->starts[i] != ins->tasks[task]) {
        continue;
      }
      if (is_real_service_task(ins, task)) {
        if (service_duration_for_task(ins, search_config, task) > 0) {
          initial_service_assignment[i] = task;
          initial_service_committed[i] = true;
        } else {
          initial_satisfied[i] = true;
          initial_satisfied_assignment[i] = task;
        }
      } else if (!is_real_service_task(ins, task)) {
        initial_satisfied[i] = true;
        initial_satisfied_assignment[i] = task;
      }
    }
  }

  auto S_init =
      new TAPFNode(ins->starts, D, ins, initial_assignment.agent_to_task,
                   initial_assignment_state, search_config,
                   initial_service_assignment, initial_service_progress,
                   initial_service_committed,
                   initial_satisfied, initial_satisfied_assignment);
  S_init->h = search_config.service_goal_mode ? get_h_value(S_init)
                                              : initial_assignment.cost;
  S_init->f = S_init->g + S_init->h;
  push_open(S_init);
  if (search_config.service_goal_mode) {
    CLOSED_SERVICE[service_key(S_init)] = S_init;
  } else {
    CLOSED[S_init->C] = S_init;
  }
  if (stats != nullptr) {
    stats->hl_nodes_created = 1;
    stats->open_max_size = 1;
  }
  const auto initial_lower_bound = S_init->h;
  const auto cleanup_reserve_ms =
      deadline == nullptr
          ? 0.0
          : std::min(1000.0, std::max(100.0, deadline->time_limit_ms * 0.1));
  const auto incumbent_search_limit_ms =
      deadline == nullptr
          ? 0.0
          : std::max(0.0, deadline->time_limit_ms - cleanup_reserve_ms);

  auto incumbent_search_expired = [&]() {
    return S_goal != nullptr && deadline != nullptr &&
           deadline->elapsed_ms() >= incumbent_search_limit_ms;
  };

  while (!OPEN.empty() && !is_expired(deadline) &&
         !incumbent_search_expired()) {
    if (stats != nullptr) {
      ++stats->hl_loop_iterations;
      stats->open_max_size = std::max<int>(stats->open_max_size, OPEN.size());
    }
    const auto open_index = select_open_index();
    auto S = OPEN[open_index];

    if (S_goal != nullptr && S_goal->g <= initial_lower_bound) {
      break;
    }

    if (S->search_tree.empty()) {
      erase_open(open_index);
      continue;
    }

    if (S_goal != nullptr && S->f >= S_goal->g) {
      erase_open(open_index);
      continue;
    }

    if (is_goal_node(S)) {
      if (S_goal == nullptr || S->g < S_goal->g) {
        if (stats != nullptr) {
          ++stats->incumbent_updates;
          if (stats->first_solution_cost == 0) {
            stats->first_solution_cost = S->g;
            stats->first_solution_time_ms = elapsed_ms(deadline);
          }
        }
        S_goal = S;
        if (stats != nullptr) ++stats->anytime_cost_updates;
        info(1, verbose, "elapsed:", elapsed_ms(deadline),
             "ms\tfound TAPF solution\tcost:", S_goal->g);
      }
      if (!anytime || deadline == nullptr || S_goal->g <= initial_lower_bound) {
        break;
      }
      continue;
    }

    auto M = S->search_tree.front();
    S->search_tree.pop();
    if (stats != nullptr) ++stats->constraints_popped;
    const auto constrained_agents = N;
    if (M->depth < constrained_agents) {
      auto i = S->order[M->depth];
      auto C = S->C[i]->neighbor;
      C.push_back(S->C[i]);
      if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
      for (auto u : C) {
        S->search_tree.push(new TAPFConstraint(M, i, u));
        if (stats != nullptr) ++stats->constraints_generated;
      }
    }

    if (!get_new_config(S, M)) {
      delete M;
      if (stats != nullptr) ++stats->constraint_failures;
      continue;
    }
    delete M;

    for (auto a : A) C_new[a->id] = a->v_next;
    if (search_config.service_goal_mode &&
        !validate_service_child_config(S, C_new, S->assignment,
                                       S->service_assignment, S->satisfied,
                                       S->satisfied_assignment)) {
      if (stats != nullptr) {
        ++stats->service_child_validation_failures;
        ++stats->service_child_stack_validation_failures;
      }
      continue;
    }

    if (!search_config.service_goal_mode) {
      auto iter = CLOSED.find(C_new);
      if (iter != CLOSED.end()) {
        auto S_known = iter->second;
        S->neighbor.insert(S_known);
        rewrite(S, S_known, S_goal, OPEN);
        auto S_insert = S_known;
        if (MT != nullptr && get_random_float(MT) < restart_rate) {
          S_insert = S_init;
        }
        if ((S_goal == nullptr || S_insert->f < S_goal->g) &&
            !S_insert->queued && !S_insert->search_tree.empty()) {
          push_open(S_insert);
          if (stats != nullptr) ++stats->hl_reinsertions;
        }
        if (stats != nullptr) ++stats->hl_duplicate_configs;
        continue;
      }
    }

    auto changed_agents = std::vector<int>();
    auto changed_agent_flags = std::vector<bool>(N, false);
    changed_agents.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      if (C_new[i] != S->C[i]) {
        changed_agents.push_back(i);
        changed_agent_flags[i] = true;
      }
    }

    auto satisfied = std::vector<bool>();
    auto satisfied_assignment = std::vector<int>();
    auto service_assignment = std::vector<int>();
    auto service_progress = std::vector<int>();
    auto service_committed = std::vector<bool>();
    if (search_config.service_goal_mode) {
      auto service_state = service_state_after_move(S, C_new);
      if (!service_state.valid) continue;
      service_assignment = service_state.service_assignment;
      service_progress = service_state.service_progress;
      service_committed = service_state.service_committed;
      satisfied = service_state.satisfied;
      satisfied_assignment = service_state.satisfied_assignment;
      for (size_t i = 0; i < N; ++i) {
        const auto service_changed =
            i >= S->service_assignment.size() ||
            i >= S->service_progress.size() || i >= S->satisfied.size() ||
            i >= S->satisfied_assignment.size() ||
            i >= S->service_committed.size() ||
            service_assignment[i] != S->service_assignment[i] ||
            service_progress[i] != S->service_progress[i] ||
            service_committed[i] != S->service_committed[i] ||
            satisfied[i] != S->satisfied[i] ||
            satisfied_assignment[i] != S->satisfied_assignment[i];
        if (service_changed && !changed_agent_flags[i]) {
          changed_agents.push_back(i);
          changed_agent_flags[i] = true;
        }
      }
    }

    auto assignment_state = S->assignment_state;
    auto fixed_task_by_agent = std::vector<int>();
    if (search_config.service_goal_mode) {
      fixed_task_by_agent.assign(N, -1);
      for (size_t i = 0; i < N; ++i) {
        if (i < service_assignment.size() && service_assignment[i] >= 0 &&
            (i >= satisfied.size() || !satisfied[i])) {
          fixed_task_by_agent[i] = service_assignment[i];
        } else if (i < satisfied.size() && satisfied[i] &&
                   i < satisfied_assignment.size()) {
          fixed_task_by_agent[i] = satisfied_assignment[i];
        }
      }
    }
    const auto service_cost_state =
        search_config.service_goal_mode
            ? assignment_service_cost_state_for(C_new, service_assignment,
                                                service_progress)
            : TAPFAssignmentServiceCostState();
    auto assignment = assign_tapf_tasks_dynamic(
        *ins, D, C_new, assignment_state, changed_agents, force_full_assignment,
        &assignment_stats, fixed_task_by_agent, std::vector<bool>(),
        service_cost_state);
    if (!assignment.feasible) {
      if (stats != nullptr) ++stats->assignment_infeasible_count;
      continue;
    }
    auto stack_failure = false;
    auto swap_failure = false;
    if (!validate_service_child_config(
            S, C_new, assignment.agent_to_task, service_assignment, satisfied,
            satisfied_assignment, &stack_failure, &swap_failure)) {
      if (stats != nullptr) {
        ++stats->service_child_validation_failures;
        if (stack_failure) ++stats->service_child_stack_validation_failures;
        if (swap_failure) ++stats->service_child_swap_validation_failures;
      }
      continue;
    }
    if (stats != nullptr) {
      for (size_t i = 0; i < assignment.agent_to_task.size(); ++i) {
        if (assignment.agent_to_task[i] != S->assignment[i]) {
          ++stats->assignment_changes;
          break;
        }
      }
    }
    auto S_new =
        new TAPFNode(C_new, D, ins, assignment.agent_to_task, assignment_state,
                     search_config, service_assignment, service_progress,
                     service_committed, satisfied, satisfied_assignment, S);
    S_new->g = S->g + get_edge_cost(S, S_new);
    S_new->h =
        search_config.service_goal_mode ? get_h_value(S_new) : assignment.cost;
    S_new->f = S_new->g + S_new->h;
    if (search_config.service_goal_mode) {
      const auto key = service_key(S_new);
      auto iter = CLOSED_SERVICE.find(key);
      if (iter != CLOSED_SERVICE.end()) {
        auto S_known = iter->second;
        S->neighbor.erase(S_new);
        delete S_new;
        S->neighbor.insert(S_known);
        rewrite(S, S_known, S_goal, OPEN);
        auto S_insert = S_known;
        if (MT != nullptr && get_random_float(MT) < restart_rate) {
          S_insert = S_init;
        }
        if ((S_goal == nullptr || S_insert->f < S_goal->g) &&
            !S_insert->queued && !S_insert->search_tree.empty()) {
          push_open(S_insert);
          if (stats != nullptr) ++stats->hl_reinsertions;
        }
        if (stats != nullptr) ++stats->hl_duplicate_configs;
        continue;
      }
      CLOSED_SERVICE[key] = S_new;
    } else {
      CLOSED[S_new->C] = S_new;
    }
    if (S_goal == nullptr || S_new->f < S_goal->g) {
      push_open(S_new);
    }
    if (stats != nullptr) ++stats->hl_nodes_created;
  }

  auto solution = Solution();
  auto solution_nodes = std::vector<TAPFNode*>();
  auto S_return = S_goal;
  if (S_return != nullptr) {
    auto S = S_return;
    while (S != nullptr) {
      solution_nodes.push_back(S);
      solution.push_back(S->C);
      S = S->parent;
    }
    std::reverse(solution.begin(), solution.end());
    std::reverse(solution_nodes.begin(), solution_nodes.end());
  }

  if (search_config.service_goal_mode && solution_nodes.size() > 1) {
    auto required_total = 0;
    for (const auto required : service_required_agents) {
      if (required) ++required_total;
    }
    for (size_t step = 1; step < solution_nodes.size(); ++step) {
      if (required_total > 0) {
        auto required_reached = 0;
        for (size_t i = 0; i < solution_nodes[step]->satisfied.size(); ++i) {
          if (i < service_required_agents.size() &&
              service_required_agents[i] &&
              solution_nodes[step]->satisfied[i]) {
            ++required_reached;
          }
        }
        if (required_reached == required_total) {
          solution.resize(step + 1);
          solution_nodes.resize(step + 1);
          break;
        }
        continue;
      }
      auto real_services = 0;
      for (size_t i = 0; i < solution_nodes[step]->satisfied.size(); ++i) {
        if (!solution_nodes[step]->satisfied[i]) continue;
        const auto task = solution_nodes[step]->satisfied_assignment[i];
        if (task >= 0 && task < static_cast<int>(ins->task_keys.size()) &&
            ins->task_keys[task] >= 0) {
          ++real_services;
        }
      }
      const auto required_real_services =
          search_config.service_commit_agents > 1
              ? std::min<int>(ins->N, search_config.service_commit_agents)
              : 1;
      if (real_services < required_real_services) {
        continue;
      }
      solution.resize(step + 1);
      solution_nodes.resize(step + 1);
      break;
    }
    if (!solution_nodes.empty()) S_return = solution_nodes.back();
  }

  if (assignment_schedule != nullptr) {
    assignment_schedule->reserve(solution_nodes.size());
    for (const auto* node : solution_nodes) {
      assignment_schedule->push_back(node->assignment);
    }
  }

  if (stats != nullptr) {
    stats->hl_nodes_explored =
        search_config.service_goal_mode ? CLOSED_SERVICE.size() : CLOSED.size();
    stats->timed_out = S_goal == nullptr && is_expired(deadline);
    stats->assignment_calls = assignment_stats.calls;
    stats->assignment_time_ms = assignment_stats.time_ms;
    stats->assignment_row_cache_requests = assignment_stats.row_cache_requests;
    stats->assignment_row_cache_hits = assignment_stats.row_cache_hits;
    if (!solution.empty()) {
      stats->solution_cost = S_return->g;
      stats->solution_h = S_return->h;
      if (search_config.service_goal_mode) {
        stats->service_satisfied_agents = std::count(
            S_return->satisfied.begin(), S_return->satisfied.end(), true);
        for (size_t i = 0; i < S_return->satisfied.size(); ++i) {
          if (!S_return->satisfied[i]) continue;
          const auto task = i < S_return->satisfied_assignment.size()
                                ? S_return->satisfied_assignment[i]
                                : -1;
          if (task < 0 || task >= static_cast<int>(ins->task_keys.size())) {
            continue;
          }
          if (ins->task_keys[task] >= kDeliveryLocationKeyBase) {
            ++stats->service_satisfied_deliveries;
          } else if (ins->task_keys[task] >= 0) {
            ++stats->service_satisfied_pickups;
          }
        }
      }
      for (size_t step = 1; step < solution_nodes.size(); ++step) {
        stats->solution_parent_edge_cost +=
            get_edge_cost(solution_nodes[step - 1], solution_nodes[step]);
      }
      stats->solution_depth = solution.size() - 1;
      for (size_t step = 1; step < solution_nodes.size(); ++step) {
        auto changed = false;
        const auto& prev = solution_nodes[step - 1]->assignment;
        const auto& curr = solution_nodes[step]->assignment;
        for (size_t i = 0; i < curr.size(); ++i) {
          if (curr[i] != prev[i]) {
            changed = true;
            ++stats->final_agent_assignment_changes;
          }
        }
        if (changed) ++stats->final_assignment_changes;
      }
    }
    if (search_config.service_goal_mode) {
      const auto& closed = CLOSED_SERVICE;
      for (const auto& item : closed) {
        const auto node = item.second;
        if (node == nullptr) continue;
        stats->service_best_satisfied_agents = std::max<int>(
            stats->service_best_satisfied_agents,
            std::count(node->satisfied.begin(), node->satisfied.end(), true));
      }
    }
  }
  if (final_assignment != nullptr) {
    if (S_return == nullptr) {
      final_assignment->clear();
    } else if (search_config.service_goal_mode && !solution_nodes.empty()) {
      // This output is applied at the current simulation state. Later
      // assignments belong to later path steps and are exposed separately via
      // assignment_schedule.
      *final_assignment = solution_nodes.front()->assignment;
    } else {
      *final_assignment = S_return->assignment;
    }
  }

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       solution.empty() ? (OPEN.empty() ? "no TAPF solution" : "failed")
                        : "TAPF solution found",
       "\texplored:",
       search_config.service_goal_mode ? CLOSED_SERVICE.size() : CLOSED.size());

  if (deadline != nullptr &&
      deadline->elapsed_ms() >= incumbent_search_limit_ms) {
    for (auto p : CLOSED) p.second->discard_search_tree();
    for (auto p : CLOSED_SERVICE) p.second->discard_search_tree();
  }

  for (auto a : A) delete a;
  if (search_config.service_goal_mode) {
    for (auto p : CLOSED_SERVICE) delete p.second;
  } else {
    for (auto p : CLOSED) delete p.second;
  }

  return solution;
}

void TAPFPlanner::rewrite(TAPFNode* from, TAPFNode* to, TAPFNode* goal,
                          std::vector<TAPFNode*>& OPEN)
{
  auto Q = std::queue<TAPFNode*>({from});
  while (!Q.empty()) {
    auto node_from = Q.front();
    Q.pop();
    for (auto node_to : node_from->neighbor) {
      const auto g = node_from->g + get_edge_cost(node_from, node_to);
      if (g < node_to->g) {
        node_to->g = g;
        node_to->f = node_to->g + node_to->h;
        node_to->parent = node_from;
        Q.push(node_to);
        if (stats != nullptr) ++stats->anytime_cost_updates;
        if (goal != nullptr && node_to->f < goal->g && !node_to->queued &&
            !node_to->search_tree.empty()) {
          OPEN.push_back(node_to);
          node_to->queued = true;
        }
      }
    }
  }
}

unsigned TAPFPlanner::get_edge_cost(const TAPFNode* from,
                                    const TAPFNode* to) const
{
  auto cost = 0u;
  for (size_t i = 0; i < ins->N; ++i) {
    if (search_config.service_goal_mode &&
        (agent_satisfied(from, i) || agent_satisfied(to, i))) {
      continue;
    }
    const auto goal = assigned_goal(to->assignment, i);
    if (from->C[i] != goal || to->C[i] != goal) ++cost;
  }
  return cost;
}

unsigned TAPFPlanner::get_h_value(const Config& C)
{
  auto cost = 0u;
  for (size_t i = 0; i < ins->N; ++i) {
    auto best = D.K;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (!ins->allowed[i][j]) continue;
      best = std::min(best, D.get(j, C[i]));
    }
    cost += best < D.K ? best : D.K;
  }
  return cost;
}

unsigned TAPFPlanner::get_h_value(const TAPFNode* node)
{
  if (node == nullptr) return 0;
  auto cost = 0u;
  for (size_t i = 0; i < ins->N; ++i) {
    if (search_config.service_goal_mode && agent_satisfied(node, i)) continue;
    const auto task = search_config.service_goal_mode &&
                              i < node->service_assignment.size() &&
                              node->service_assignment[i] >= 0
                          ? node->service_assignment[i]
                          : node->assignment[i];
    if (task < 0 || task >= static_cast<int>(ins->tasks.size()) ||
        !ins->allowed[i][task]) {
      cost += D.K;
      continue;
    }
    const auto d = D.get(task, node->C[i]);
    if (d >= D.K) {
      cost += D.K;
      continue;
    }
    const auto scale = ins->assignment_distance_scales.empty()
                           ? ins->assignment_distance_scale
                           : ins->assignment_distance_scales[i][task];
    const auto offset = ins->assignment_cost_offsets[i][task];
    auto value = static_cast<long long>(d) * scale + offset;
    if (search_config.service_goal_mode) {
      auto service_steps = assignment_service_duration_for_task(ins, i, task);
      if (i < node->service_assignment.size() &&
          node->service_assignment[i] == task &&
          i < node->service_progress.size() &&
          node->C[i] == ins->tasks[task]) {
        service_steps =
            std::max(0, service_steps - std::max(0, node->service_progress[i]));
      }
      value += static_cast<long long>(service_steps) * scale;
    }
    cost += value > std::numeric_limits<unsigned>::max()
                ? std::numeric_limits<unsigned>::max()
                : static_cast<unsigned>(value);
  }
  return cost;
}

bool TAPFPlanner::agent_satisfied(const TAPFNode* node, int agent) const
{
  if (node == nullptr || agent < 0 ||
      agent >= static_cast<int>(node->assignment.size())) {
    return false;
  }
  if (!search_config.service_goal_mode) {
    const auto goal = assigned_goal(node->assignment, agent);
    return goal != nullptr && node->C[agent] == goal;
  }
  return agent < static_cast<int>(node->satisfied.size()) &&
         node->satisfied[agent];
}

Vertex* TAPFPlanner::assigned_goal(const std::vector<int>& assignment,
                                   int agent) const
{
  if (agent < 0 || agent >= static_cast<int>(assignment.size())) return nullptr;
  const auto task = assignment[agent];
  if (task < 0 || task >= static_cast<int>(ins->tasks.size())) return nullptr;
  return ins->tasks[task];
}

Vertex* TAPFPlanner::service_goal(const TAPFNode* node, int agent) const
{
  if (node == nullptr) return nullptr;
  return service_goal_for_state(node->assignment, node->service_assignment,
                                node->satisfied, node->satisfied_assignment,
                                agent);
}

Vertex* TAPFPlanner::service_goal_for_state(
    const std::vector<int>& assignment,
    const std::vector<int>& service_assignment,
    const std::vector<bool>& satisfied,
    const std::vector<int>& satisfied_assignment, int agent) const
{
  if (agent < 0 || agent >= static_cast<int>(assignment.size())) {
    return nullptr;
  }
  if (search_config.service_goal_mode &&
      agent < static_cast<int>(satisfied.size()) && satisfied[agent]) {
    return nullptr;
  }
  auto task = assignment[agent];
  if (search_config.service_goal_mode &&
      agent < static_cast<int>(service_assignment.size()) &&
      service_assignment[agent] >= 0 &&
      (agent >= static_cast<int>(satisfied.size()) || !satisfied[agent])) {
    task = service_assignment[agent];
  }
  if (task < 0 || task >= static_cast<int>(ins->tasks.size())) {
    return nullptr;
  }
  return ins->tasks[task];
}

bool TAPFPlanner::agent_has_service_option_at(int agent, Vertex* vertex) const
{
  if (!search_config.service_goal_mode || vertex == nullptr || agent < 0 ||
      agent >= static_cast<int>(ins->allowed.size())) {
    return false;
  }
  const auto& allowed = ins->allowed[agent];
  for (size_t task = 0; task < allowed.size(); ++task) {
    if (!allowed[task] || task >= ins->tasks.size() ||
        ins->tasks[task] != vertex) {
      continue;
    }
    if (task < ins->task_keys.size() && ins->task_keys[task] >= 1000000000) {
      return true;
    }
  }
  return false;
}

bool TAPFPlanner::can_share_service_goal(const TAPFNode* node, int agent,
                                         Vertex* vertex) const
{
  if (!search_config.service_goal_mode || node == nullptr ||
      vertex == nullptr) {
    return false;
  }
  return service_goal(node, agent) == vertex;
}

bool TAPFPlanner::can_share_service_goal_for_state(
    const std::vector<int>& assignment,
    const std::vector<int>& service_assignment,
    const std::vector<bool>& satisfied,
    const std::vector<int>& satisfied_assignment, int agent,
    Vertex* vertex) const
{
  if (!search_config.service_goal_mode || vertex == nullptr) return false;
  return service_goal_for_state(assignment, service_assignment, satisfied,
                                satisfied_assignment, agent) == vertex;
}

bool TAPFPlanner::can_reserve_next(const TAPFNode* node, Agent* agent,
                                   Vertex* vertex)
{
  if (agent == nullptr || vertex == nullptr) return false;
  auto in_active_blocking_service = [&](const Agent* candidate) {
    if (candidate == nullptr || node == nullptr) return false;
    const auto id = candidate->id;
    if (id < 0 || id >= static_cast<int>(node->service_assignment.size()) ||
        id >= static_cast<int>(node->satisfied.size()) ||
        node->service_assignment[id] < 0 || node->satisfied[id]) {
      return false;
    }
    const auto task = node->service_assignment[id];
    const auto progress = id < static_cast<int>(node->service_progress.size())
                              ? node->service_progress[id]
                              : 0;
    return service_duration_for_task(ins, search_config, task) >
           std::max(0, progress);
  };
  const auto representative = occupied_next[vertex->id];
  if (representative != nullptr && representative != agent &&
      (in_active_blocking_service(representative) ||
       in_active_blocking_service(agent))) {
    return false;
  }
  if (representative != nullptr &&
      (!can_share_service_goal(node, agent->id, vertex) ||
       !can_share_service_goal(node, representative->id, vertex))) {
    return false;
  }
  if (can_share_service_goal(node, agent->id, vertex) &&
      agent->v_now != vertex && shared_goal_entry_counts[vertex->id] > 0) {
    return false;
  }
  auto current_occupant = occupied_now[vertex->id];
  if (current_occupant != nullptr && current_occupant->v_next == agent->v_now) {
    return false;
  }
  return true;
}

void TAPFPlanner::reserve_next(const TAPFNode* node, Agent* agent,
                               Vertex* vertex)
{
  if (agent == nullptr || vertex == nullptr) return;
  if (can_share_service_goal(node, agent->id, vertex) &&
      agent->v_now != vertex) {
    ++shared_goal_entry_counts[vertex->id];
  }
  if (occupied_next[vertex->id] == nullptr) {
    occupied_next[vertex->id] = agent;
  }
  agent->v_next = vertex;
}

bool TAPFPlanner::validate_service_child_config(
    const TAPFNode* parent, const Config& C, const std::vector<int>& assignment,
    const std::vector<int>& service_assignment,
    const std::vector<bool>& satisfied,
    const std::vector<int>& satisfied_assignment, bool* stack_failure,
    bool* swap_failure) const
{
  if (stack_failure != nullptr) *stack_failure = false;
  if (swap_failure != nullptr) *swap_failure = false;
  if (!search_config.service_goal_mode) return true;
  if (C.size() != ins->N || assignment.size() != ins->N) return false;

  auto occupancy = std::vector<std::vector<int> >(V_size);
  auto entrants = std::vector<int>(V_size, 0);
  for (size_t i = 0; i < C.size(); ++i) {
    if (C[i] == nullptr) return false;
    occupancy[C[i]->id].push_back(i);
    if (parent != nullptr && i < parent->C.size() && parent->C[i] != C[i] &&
        can_share_service_goal_for_state(assignment, service_assignment,
                                         satisfied, satisfied_assignment, i,
                                         C[i])) {
      ++entrants[C[i]->id];
    }
  }

  for (size_t v = 0; v < occupancy.size(); ++v) {
    const auto& agents = occupancy[v];
    if (agents.size() <= 1) continue;
    if (entrants[v] > 1) {
      if (stack_failure != nullptr) *stack_failure = true;
      return false;
    }
    auto vertex = ins->G.U[v];
    for (const auto agent : agents) {
      if (!can_share_service_goal_for_state(assignment, service_assignment,
                                            satisfied, satisfied_assignment,
                                            agent, vertex)) {
        if (stack_failure != nullptr) *stack_failure = true;
        return false;
      }
    }
  }

  if (parent != nullptr) {
    for (size_t i = 0; i < C.size(); ++i) {
      for (size_t j = i + 1; j < C.size(); ++j) {
        if (parent->C[i] == C[j] && parent->C[j] == C[i] && C[i] != C[j]) {
          if (swap_failure != nullptr) *swap_failure = true;
          return false;
        }
      }
    }
  }
  return true;
}

int TAPFPlanner::distance_to_assigned_goal(const TAPFNode* node, int agent,
                                           Vertex* v)
{
  if (node == nullptr || v == nullptr) return D.K;
  const auto goal = service_goal(node, agent);
  if (goal == nullptr) return D.K;
  const auto task =
      agent >= 0 && agent < static_cast<int>(node->service_assignment.size()) &&
              node->service_assignment[agent] >= 0 &&
              !agent_satisfied(node, agent)
          ? node->service_assignment[agent]
          : (agent >= 0 && agent < static_cast<int>(node->assignment.size())
                 ? node->assignment[agent]
                 : -1);
  if (task < 0 || task >= static_cast<int>(ins->tasks.size())) return D.K;
  return D.get(task, v);
}

int TAPFPlanner::distance_to_assigned_goal(const std::vector<int>& assignment,
                                           int agent, Vertex* v)
{
  if (v == nullptr) return D.K;
  const auto task = agent >= 0 && agent < static_cast<int>(assignment.size())
                        ? assignment[agent]
                        : -1;
  if (task < 0 || task >= static_cast<int>(ins->tasks.size())) return D.K;
  return D.get(task, v);
}

bool TAPFPlanner::is_goal_node(const TAPFNode* node) const
{
  if (node == nullptr || node->assignment.size() != ins->N) return false;
  if (search_config.service_goal_mode) {
    auto reached = 0;
    auto real_services = 0;
    auto required_reached = 0;
    auto required_total = 0;
    for (size_t i = 0; i < ins->N; ++i) {
      if (i < service_required_agents.size() && service_required_agents[i]) {
        ++required_total;
      }
      if (!agent_satisfied(node, i)) continue;
      ++reached;
      if (i < service_required_agents.size() && service_required_agents[i]) {
        ++required_reached;
      }
      const auto task = i < node->satisfied_assignment.size()
                            ? node->satisfied_assignment[i]
                            : -1;
      if (task < 0 || task >= static_cast<int>(ins->tasks.size()) ||
          !ins->allowed[i][task]) {
        return false;
      }
      if (task >= static_cast<int>(ins->task_keys.size()) ||
          ins->task_keys[task] >= 0) {
        ++real_services;
      }
    }
    if (required_total > 0) return required_reached == required_total;
    const auto required =
        search_config.service_commit_agents > 0
            ? std::min<int>(ins->N, search_config.service_commit_agents)
            : 1;
    return real_services >= required;
  }
  for (size_t i = 0; i < ins->N; ++i) {
    const auto task = node->assignment[i];
    if (task < 0 || task >= static_cast<int>(ins->tasks.size()) ||
        !ins->allowed[i][task] || !agent_satisfied(node, i)) {
      return false;
    }
  }
  return true;
}

bool TAPFPlanner::get_new_config(TAPFNode* S, TAPFConstraint* M)
{
  std::fill(shared_goal_entry_counts.begin(), shared_goal_entry_counts.end(),
            0);
  for (auto a : A) {
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    a->v_now = S->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];
    if (M->where[k] == nullptr) continue;

    if (!can_reserve_next(S, A[i], M->where[k])) return false;
    reserve_next(S, A[i], M->where[k]);
  }

  if (search_config.service_goal_mode) {
    for (auto k : S->order) {
      if (k >= static_cast<int>(S->service_assignment.size()) ||
          k >= static_cast<int>(S->satisfied.size()) ||
          k >= static_cast<int>(S->service_committed.size()) ||
          S->service_assignment[k] < 0 || S->satisfied[k] ||
          !S->service_committed[k]) {
        continue;
      }
      auto a = A[k];
      if (a->v_next != nullptr && a->v_next != a->v_now) return false;
      if (a->v_next == nullptr) {
        if (!can_reserve_next(S, a, a->v_now)) return false;
        reserve_next(S, a, a->v_now);
      }
    }
  }

  if (search_config.service_goal_mode) {
    for (auto k : S->order) {
      if (k >= static_cast<int>(S->service_assignment.size()) ||
          k >= static_cast<int>(S->satisfied.size()) ||
          k >= static_cast<int>(S->service_committed.size()) ||
          S->service_assignment[k] < 0 || S->satisfied[k] ||
          S->service_committed[k]) {
        continue;
      }
      auto a = A[k];
      if (a == nullptr || a->v_next != nullptr) continue;
      const auto goal = service_goal(S, k);
      if (goal == nullptr || goal != a->v_now) continue;
      if (!can_reserve_next(S, a, a->v_now)) continue;
      reserve_next(S, a, a->v_now);
    }
  }

  if (search_config.service_goal_mode) {
    auto direct_entries = 0;
    constexpr auto kMaxDirectServiceEntries = 100;
    for (auto k : S->order) {
      if (direct_entries >= kMaxDirectServiceEntries) break;
      auto a = A[k];
      if (S->satisfied[k] || a->v_next != nullptr) continue;
      if (k >= static_cast<int>(S->assignment.size())) continue;
      const auto task = S->assignment[k];
      if (task < 0 || task >= static_cast<int>(ins->tasks.size())) continue;
      if (task >= static_cast<int>(ins->task_keys.size()) ||
          ins->task_keys[task] < kDeliveryLocationKeyBase) {
        continue;
      }
      auto goal = ins->tasks[task];
      if (goal == nullptr || D.get(task, a->v_now) != 1) continue;
      if (occupied_now[goal->id] != nullptr) continue;
      if (!can_reserve_next(S, a, goal)) continue;
      reserve_next(S, a, goal);
      ++direct_entries;
    }
  }

  for (auto k : S->order) {
    auto a = A[k];
    if (search_config.service_goal_mode && S->satisfied[k]) continue;
    if (a->v_next == nullptr && !funcPIBT(a, S)) return false;
  }
  for (auto k : S->order) {
    auto a = A[k];
    if (!search_config.service_goal_mode || !S->satisfied[k] ||
        a->v_next != nullptr) {
      continue;
    }
    if (!funcPIBT(a, S)) return false;
  }
  return true;
}

bool TAPFPlanner::funcPIBT(Agent* ai, const TAPFNode* node)
{
  if (stats != nullptr) ++stats->pibt_calls;
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();
  auto neighbor_agents = std::array<Agent*, 4>();
  auto neighbor_agent_count = 0u;

  for (auto u : ai->v_now->neighbor) {
    auto aj = occupied_now[u->id];
    if (aj != nullptr) neighbor_agents[neighbor_agent_count++] = aj;
  }

  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr) tie_breakers[u->id] = get_random_float(MT);
  }
  C_next[i][K] = ai->v_now;
  if (MT != nullptr) tie_breakers[ai->v_now->id] = get_random_float(MT);

  auto get_hindrance = [&](Vertex* u) {
    auto count = 0u;
    for (auto n = 0u; n < neighbor_agent_count; ++n) {
      auto aj = neighbor_agents[n];
      if (aj->v_now == u) continue;
      if (distance_to_assigned_goal(node, aj->id, u) <
          distance_to_assigned_goal(node, aj->id, aj->v_now)) {
        ++count;
      }
    }
    return count;
  };

  auto should_leave_foreign_service = false;
  if (search_config.service_goal_mode &&
      ai->v_now->id < static_cast<int>(real_service_vertices.size()) &&
      real_service_vertices[ai->v_now->id] &&
      !can_share_service_goal(node, i, ai->v_now)) {
    const auto task = i < static_cast<int>(node->assignment.size())
                          ? node->assignment[i]
                          : -1;
    const auto task_key =
        task >= 0 && task < static_cast<int>(ins->task_keys.size())
            ? ins->task_keys[task]
            : -1;
    if (task_key < 1000000000) {
      should_leave_foreign_service = true;
    }
    for (auto n = 0u; n < neighbor_agent_count; ++n) {
      auto aj = neighbor_agents[n];
      if (aj == nullptr || agent_satisfied(node, aj->id)) continue;
      if (service_goal(node, aj->id) == ai->v_now ||
          agent_has_service_option_at(aj->id, ai->v_now)) {
        should_leave_foreign_service = true;
        break;
      }
    }
  }

  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              if (should_leave_foreign_service) {
                const auto v_leaves = v != ai->v_now;
                const auto u_leaves = u != ai->v_now;
                if (v_leaves != u_leaves) return v_leaves;
              }
              const auto dv = distance_to_assigned_goal(node, i, v);
              const auto du = distance_to_assigned_goal(node, i, u);
              if (dv != du) return dv < du;
              const auto hv = get_hindrance(v);
              const auto hu = get_hindrance(u);
              if (hv != hu) {
                return hv < hu;
              }
              return tie_breakers[v->id] < tie_breakers[u->id];
            });

  auto swap_agent = swap_possible_and_required(ai, node);
  if (swap_agent != nullptr) {
    if (stats != nullptr) ++stats->swap_applied;
    std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
  }

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];
    if (!can_reserve_next(node, ai, u)) continue;

    auto& ak = occupied_now[u->id];
    auto next_snapshot = std::vector<Vertex*>(N, nullptr);
    for (const auto a : A) next_snapshot[a->id] = a->v_next;
    const auto occupied_next_snapshot = occupied_next;
    const auto shared_goal_entry_snapshot = shared_goal_entry_counts;
    auto restore_candidate = [&]() {
      for (const auto a : A) a->v_next = next_snapshot[a->id];
      occupied_next = occupied_next_snapshot;
      shared_goal_entry_counts = shared_goal_entry_snapshot;
    };
    reserve_next(node, ai, u);

    if (ak != nullptr && ak != ai && ak->v_next == nullptr) {
      if (can_share_service_goal(node, ai->id, u) &&
          can_share_service_goal(node, ak->id, u)) {
        reserve_next(node, ak, u);
      } else {
        if (stats != nullptr) ++stats->pibt_recursions;
        if (!funcPIBT(ak, node)) {
          restore_candidate();
          continue;
        }
      }
    }

    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    return true;
  }

  if (!can_reserve_next(node, ai, ai->v_now)) {
    if (stats != nullptr) ++stats->pibt_failures;
    return false;
  }
  reserve_next(node, ai, ai->v_now);
  if (stats != nullptr) ++stats->pibt_failures;
  return false;
}

Agent* TAPFPlanner::swap_possible_and_required(Agent* ai, const TAPFNode* node)
{
  if (stats != nullptr) ++stats->swap_checks;
  const auto i = ai->id;
  if (C_next[i][0] == ai->v_now) return nullptr;

  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now, node) &&
      is_swap_possible(aj->v_now, ai->v_now, node)) {
    return aj;
  }

  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0], node) &&
        is_swap_possible(C_next[i][0], ai->v_now, node)) {
      return ak;
    }
  }

  return nullptr;
}

bool TAPFPlanner::is_swap_required(const int pusher, const int puller,
                                   Vertex* v_pusher_origin,
                                   Vertex* v_puller_origin,
                                   const TAPFNode* node)
{
  if (node == nullptr || agent_satisfied(node, pusher)) return false;
  const auto puller_is_free = agent_satisfied(node, puller);
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  const auto& assignment = node->assignment;
  const auto pusher_task = assignment[pusher];
  const auto puller_task = assignment[puller];

  while (D.get(pusher_task, v_puller) < D.get(pusher_task, v_pusher)) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher || (u->neighbor.size() == 1 && a != nullptr &&
                            !agent_satisfied(node, a->id) &&
                            ins->tasks[assignment[a->id]] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  return (puller_is_free ||
          D.get(puller_task, v_pusher) < D.get(puller_task, v_puller)) &&
         (D.get(pusher_task, v_pusher) == 0 ||
          D.get(pusher_task, v_puller) < D.get(pusher_task, v_pusher));
}

bool TAPFPlanner::is_swap_possible(Vertex* v_pusher_origin,
                                   Vertex* v_puller_origin,
                                   const TAPFNode* node)
{
  if (node == nullptr) return false;
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  const auto& assignment = node->assignment;
  while (v_puller != v_pusher_origin) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher || (u->neighbor.size() == 1 && a != nullptr &&
                            !agent_satisfied(node, a->id) &&
                            ins->tasks[assignment[a->id]] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

Solution solve_tapf(const TAPFInstance& ins, const int verbose,
                    const Deadline* deadline, std::mt19937* MT,
                    const int sticky_penalty, TAPFStats* stats, bool anytime,
                    bool force_full_assignment, TAPFSearchConfig search_config,
                    std::vector<int>* final_assignment,
                    std::vector<std::vector<int> >* assignment_schedule)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tTAPF pre-processing");
  auto planner = TAPFPlanner(&ins, deadline, MT, verbose, sticky_penalty,
                             0.001f, anytime, stats, search_config);
  planner.force_full_assignment = force_full_assignment;
  return planner.solve(final_assignment, assignment_schedule);
}
