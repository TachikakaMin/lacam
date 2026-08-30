#include "../include/tapf_planner.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <unordered_set>

#include "../include/search_kernel.hpp"

namespace
{
  bool is_open_viable(const TAPFNode* node, const TAPFNode* goal)
  {
    return !node->search_tree.empty() &&
           (goal == nullptr || node->f < goal->g);
  }

  double focal_score(const TAPFNode* node, TAPFFocalTieBreak tie_break)
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

  // CLOSED key = (Config, ShelfState) (design 6.1, mapping M2/M14): the
  // shelf hash is splitmix64-derived per (role, id, cell) and XORs to 0
  // for the empty layer, so shelf-free instances keep the exact original
  // ConfigHasher value and duplicate semantics.
  uint64_t splitmix64(uint64_t x)
  {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  }

  uint64_t shelf_layer_hash(const ShelfState& S)
  {
    uint64_t h = 0;
    for (size_t b = 0; b < S.target_pos.size(); ++b)
      h ^= splitmix64((1ULL << 40) ^ (b << 20) ^ (uint64_t)S.target_pos[b]);
    for (const int c : S.anon_occ)
      h ^= splitmix64((2ULL << 40) ^ (uint64_t)c);
    for (size_t i = 0; i < S.kappa.size(); ++i)
      h ^= splitmix64((3ULL << 40) ^ (i << 20) ^
                      (uint64_t)(S.kappa[i] + 2));
    return h;
  }

  struct SearchKey {
    Config C;
    ShelfState S;
    bool operator==(const SearchKey& o) const
    {
      return C == o.C && S == o.S;
    }
  };

  struct SearchKeyHasher {
    size_t operator()(const SearchKey& k) const
    {
      return (size_t)ConfigHasher()(k.C) ^ (size_t)shelf_layer_hash(k.S);
    }
  };
}  // namespace


ShelfState initial_shelf_state(const TAPFInstance& ins)
{
  ShelfState S;
  if (ins.shelf_cells.empty()) return S;  // shelf-free: empty layer
  S.target_pos = ins.target_starts;
  std::unordered_set<int> tset(ins.target_starts.begin(),
                               ins.target_starts.end());
  for (const int p : ins.shelf_cells)
    if (!tset.count(p)) S.anon_occ.push_back(p);
  std::sort(S.anon_occ.begin(), S.anon_occ.end());
  S.kappa.assign(ins.N, KAPPA_FREE);
  return S;
}

TAPFNode::TAPFNode(Config _C, ShelfState _shelf, TAPFDistTable& D,
                   const TAPFInstance* ins, std::vector<int> _assignment,
                   TAPFAssignmentState _assignment_state, TAPFNode* _parent)
    : LacamNodeCore<TAPFConstraint, TAPFNode>(_C, _parent),
      shelf(std::move(_shelf)),
      neighbor(std::set<TAPFNode*>()),
      assignment(_assignment),
      assignment_state(_assignment_state),
      queued(false),
      g(0),
      h(0),
      f(0),
      depth(parent == nullptr ? 0 : parent->depth + 1),
      non_goal_waits(parent == nullptr ? 0 : parent->non_goal_waits),
      reversals(parent == nullptr ? 0 : parent->reversals),
      distance_increases(parent == nullptr ? 0 : parent->distance_increases),
      settled_pushes(parent == nullptr ? 0 : parent->settled_pushes)
{
  if (parent != nullptr) parent->neighbor.insert(this);
  refresh_priority(D);
  refresh_search_metrics(D, ins);
  constraint_order = order;  // frozen for the node's lifetime (D11, M3)
}

void TAPFNode::discard_search_tree()
{
  while (!search_tree.empty()) search_tree.pop();
}

void TAPFNode::refresh_priority(TAPFDistTable& D)
{
  // shared core machinery, task-keyed distance (node-skeleton audit 5);
  // carrier agents (no instance task) contribute 0 here — their PIBT
  // ordering comes from the guidance layer (design 5.4, WP4)
  init_priorities_and_order([&](size_t i) {
    return assignment[i] >= 0 ? D.get(assignment[i], C[i]) : 0;
  });
}

void TAPFNode::refresh_search_metrics(TAPFDistTable& D,
                                      const TAPFInstance* ins)
{
  if (parent == nullptr) return;

  for (size_t i = 0; i < C.size(); ++i) {
    const auto task = assignment[i];
    if (task < 0) continue;  // carrier agent: no instance task
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
      if (assignment[pushed] < 0) continue;  // carrier agent
      const auto pushed_goal = ins->tasks[assignment[pushed]];
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
      N(ins->N),
      V_size(ins->G.size()),
      weights(),
      D(TAPFDistTable(ins)),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
  if (stats != nullptr) *stats = TAPFStats();
  // solver-objective weights (design 5.7, round-2 P2-16b semantics): unit
  // by default; DD_SOLVER_WEIGHTS=1 folds DD_ALPHA..DD_DELTA into g.
  // Shelf-free instances never evaluate the carrier cost term.
  if (const char* e = std::getenv("DD_SOLVER_WEIGHTS")) {
    if (std::atoi(e) != 0) {
      if (const char* a = std::getenv("DD_ALPHA")) weights.alpha = atof(a);
      if (const char* b = std::getenv("DD_BETA")) weights.beta = atof(b);
      if (const char* c = std::getenv("DD_GAMMA")) weights.gamma = atof(c);
      if (const char* d = std::getenv("DD_DELTA")) weights.delta = atof(d);
    }
  }
  // carrier layer (M4): the conformance-oracle view and the occupancy
  // scratch exist only when the instance HAS a shelf layer
  if (!ins->shelf_cells.empty()) {
    dd_view = std::make_unique<DDInstance>();
    dd_view->grid.height = ins->G.height;
    dd_view->grid.width = ins->G.width;
    dd_view->grid.wall.assign(ins->G.height * ins->G.width, 0);
    for (int c = 0; c < (int)dd_view->grid.wall.size(); ++c)
      dd_view->grid.wall[c] = ins->G.U[c] == nullptr ? 1 : 0;
    for (const auto* v : ins->starts) dd_view->robots.push_back(v->index);
    dd_view->shelves = ins->shelf_cells;
    dd_view->target_starts = ins->target_starts;
    dd_view->target_goals = ins->target_goals;
    dd_view->finalize();
    const size_t n_cells = ins->G.U.size();
    carrier_grounded.assign(n_cells, 0);
    carrier_upper_base.assign(n_cells, 0);
    carrier_upper_delta.assign(n_cells, 0);
  }
}

Solution TAPFPlanner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart TAPF search");

  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  std::vector<TAPFNode*> OPEN;
  std::unordered_map<SearchKey, TAPFNode*, SearchKeyHasher> CLOSED;
  TAPFNode* S_goal = nullptr;
  auto C_new = Config(N, nullptr);
  // scratch lookup key reused across iterations (no per-iteration alloc
  // after warm-up; shelf part is empty for shelf-free instances)
  SearchKey lookup_key;

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
    if (search_config.mode == TAPFSearchMode::DFS || S_goal == nullptr) {
      return OPEN.size() - 1;
    }
    // shared FOCAL kernel (search_kernel.hpp) — same semantics as before
    return focal_select_index(
        OPEN, search_config.focal_weight,
        [](const TAPFNode* n) { return static_cast<double>(n->f); },
        [&](const TAPFNode* n) { return is_open_viable(n, S_goal); },
        [&](const TAPFNode* a, const TAPFNode* b) {
          return focal_better(a, b, search_config.focal_tie_break);
        });
  };

  auto initial_assignment_state = TAPFAssignmentState();
  initial_assignment_state.init(ins->N, ins->tasks.size());
  auto initial_agents = std::vector<int>(N, 0);
  std::iota(initial_agents.begin(), initial_agents.end(), 0);
  auto initial_assignment =
      assign_tapf_tasks_dynamic(*ins, D, ins->starts, initial_assignment_state,
                                initial_agents, true, &assignment_stats);
  if (!initial_assignment.feasible) return Solution();

  auto S_init =
      new TAPFNode(ins->starts, initial_shelf_state(*ins), D, ins,
                   initial_assignment.agent_to_task, initial_assignment_state);
  S_init->h = initial_assignment.cost;
  S_init->f = S_init->g + S_init->h;
  push_open(S_init);
  CLOSED[SearchKey{S_init->C, S_init->shelf}] = S_init;
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

    if (is_goal_config(S->C, S->shelf)) {
      if (S_goal == nullptr || S->g < S_goal->g) {
        if (stats != nullptr) {
          ++stats->incumbent_updates;
          if (stats->first_solution_cost == 0) {
            stats->first_solution_cost = (unsigned)std::lround(S->g);
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
    if (M->depth < N) {
      auto i = S->constraint_order[M->depth];
      auto C = S->C[i]->neighbor;
      C.push_back(S->C[i]);
      if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
      // operator candidates (design 5.2, M3): every vertex candidate is a
      // MOVE (or WAIT at the own cell); LIFT/DROP append — their guards
      // are structurally false on shelf-free instances, so the candidate
      // set, order and RNG consumption stay exactly the original there.
      auto ops_cand = std::vector<OpCand>();
      ops_cand.reserve(C.size() + 2);
      for (auto u : C)
        ops_cand.push_back(OpCand{
            u, (uint8_t)(u == S->C[i] ? Op::WAIT : Op::MOVE)});
      if (!S->shelf.kappa.empty()) {
        refresh_carrier_scratch(S);
        const int cell = S->C[i]->index;
        if (S->shelf.kappa[i] == KAPPA_FREE) {
          if (carrier_grounded[cell] != 0)
            ops_cand.push_back(OpCand{S->C[i], (uint8_t)Op::LIFT});
        } else {
          ops_cand.push_back(OpCand{S->C[i], (uint8_t)Op::DROP});
        }
      }
      lacam_expand_constraint_vec<TAPFConstraint>(M, i, ops_cand,
                                                  S->search_tree);
      if (stats != nullptr) stats->constraints_generated += ops_cand.size();
    }

    if (!get_new_config(S, M)) {
      delete M;
      if (stats != nullptr) ++stats->constraint_failures;
      continue;
    }
    delete M;

    for (auto a : A) C_new[a->id] = a->v_next;

    // carrier layer successor (M4): assemble the joint op and let the
    // conformance oracle arbitrate; fills shelf_next_scratch.  Trivially
    // true (layer copied) on shelf-free instances.
    if (!apply_carrier_effects(S)) {
      if (stats != nullptr) ++stats->carrier_validator_rejects;
      continue;
    }

    lookup_key.C = C_new;
    lookup_key.S = shelf_next_scratch;
    auto iter = CLOSED.find(lookup_key);
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

    auto changed_agents = std::vector<int>();
    changed_agents.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      if (C_new[i] != S->C[i]) changed_agents.push_back(i);
    }

    auto assignment_state = S->assignment_state;
    auto assignment =
        assign_tapf_tasks_dynamic(*ins, D, C_new, assignment_state,
                                  changed_agents, force_full_assignment, &assignment_stats);
    if (!assignment.feasible) continue;
    if (stats != nullptr) {
      for (size_t i = 0; i < assignment.agent_to_task.size(); ++i) {
        if (assignment.agent_to_task[i] != S->assignment[i]) {
          ++stats->assignment_changes;
          break;
        }
      }
    }

    auto S_new = new TAPFNode(C_new, shelf_next_scratch, D, ins,
                              assignment.agent_to_task, assignment_state, S);
    S_new->g = S->g + get_edge_cost(S, S_new);
    S_new->h = assignment.cost;
    S_new->f = S_new->g + S_new->h;
    CLOSED[SearchKey{S_new->C, S_new->shelf}] = S_new;
    if (S_goal == nullptr || S_new->f < S_goal->g) {
      push_open(S_new);
    }
    if (stats != nullptr) ++stats->hl_nodes_created;
  }

  auto solution = Solution();
  auto solution_nodes = std::vector<TAPFNode*>();
  solution_shelves.clear();
  if (S_goal != nullptr) {
    auto S = S_goal;
    while (S != nullptr) {
      solution_nodes.push_back(S);
      solution.push_back(S->C);
      solution_shelves.push_back(S->shelf);
      S = S->parent;
    }
    std::reverse(solution.begin(), solution.end());
    std::reverse(solution_nodes.begin(), solution_nodes.end());
    std::reverse(solution_shelves.begin(), solution_shelves.end());
  }

  if (stats != nullptr) {
    stats->hl_nodes_explored = CLOSED.size();
    stats->timed_out = solution.empty() && is_expired(deadline);
    stats->assignment_calls = assignment_stats.calls;
    stats->assignment_time_ms = assignment_stats.time_ms;
    if (!solution.empty()) {
      stats->solution_cost = (unsigned)std::lround(S_goal->g);
      double parent_edge_cost = 0;
      for (size_t step = 1; step < solution_nodes.size(); ++step) {
        parent_edge_cost +=
            get_edge_cost(solution_nodes[step - 1], solution_nodes[step]);
      }
      stats->solution_parent_edge_cost = (unsigned)std::lround(parent_edge_cost);
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
  }

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       solution.empty() ? (OPEN.empty() ? "no TAPF solution" : "failed")
                        : "TAPF solution found",
       "\texplored:", CLOSED.size());

  if (deadline != nullptr && deadline->elapsed_ms() >= incumbent_search_limit_ms) {
    for (auto p : CLOSED) p.second->discard_search_tree();
  }

  for (auto a : A) delete a;
  for (auto p : CLOSED) delete p.second;

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

double TAPFPlanner::get_edge_cost(const TAPFNode* from,
                                  const TAPFNode* to) const
{
  auto cost = 0.0;
  for (size_t i = 0; i < ins->N; ++i) {
    const auto task = to->assignment[i];
    if (task < 0) continue;  // carrier agent: physical term below
    const auto goal = ins->tasks[task];
    if (from->C[i] != goal || to->C[i] != goal) ++cost;
  }
  // physical carrier term (design 2.3, mapping M5): loaded/free moves,
  // lift/drop, anonymous-carry moves.  kappa is empty on shelf-free
  // instances, so this loop is structurally skipped there.
  for (size_t i = 0; i < from->shelf.kappa.size(); ++i) {
    const int k_from = from->shelf.kappa[i];
    const int k_to = to->shelf.kappa[i];
    if (from->C[i] != to->C[i]) {  // MOVE (kappa preserved by moves)
      if (k_from == KAPPA_FREE) {
        if (to->assignment[i] < 0) cost += weights.beta;
      } else {
        cost += weights.alpha;
        if (k_from == KAPPA_ANON) cost += weights.delta;
      }
    } else if (k_from != k_to) {  // LIFT or DROP (same cell)
      cost += weights.gamma;
    }
  }
  return cost;
}

double TAPFPlanner::get_h_value(const Config& C)
{
  auto cost = 0.0;
  for (size_t i = 0; i < ins->N; ++i) {
    if (ins->allowed[i].empty()) continue;  // carrier agent: no task h
    auto best = D.K;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (!ins->allowed[i][j]) continue;
      best = std::min(best, D.get(j, C[i]));
    }
    cost += best < D.K ? best : D.K;
  }
  return cost;
}

bool TAPFPlanner::is_goal_config(const Config& C, const ShelfState& S) const
{
  // agent-task part: identical to the original for every agent that HAS
  // an allowed task (is_valid guarantees that on shelf-free instances);
  // carrier agents (empty allowed row) are terminally unconstrained.
  auto used = std::vector<bool>(ins->tasks.size(), false);
  for (size_t i = 0; i < ins->N; ++i) {
    if (ins->allowed[i].empty()) continue;
    auto matched = false;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (used[j] || !ins->allowed[i][j] || C[i] != ins->tasks[j]) continue;
      used[j] = true;
      matched = true;
      break;
    }
    if (!matched) return false;
  }
  // carrier part (design 2.2, D10): every target grounded at its goal
  for (size_t b = 0; b < ins->target_goals.size(); ++b)
    if (S.target_pos[b] != ins->target_goals[b]) return false;
  for (const int k : S.kappa)
    if (k >= 0) return false;  // a carried target is not grounded
  return true;
}

bool TAPFPlanner::get_new_config(TAPFNode* S, TAPFConstraint* M)
{
  refresh_carrier_scratch(S);
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
  // reset carried-shelf reservations of the previous generation
  for (const int c : carrier_upper_touched) carrier_upper_delta[c] = 0;
  carrier_upper_touched.clear();

  // G1 (design 4.2, M4): with a shelf layer and a FULLY constrained joint
  // op, the conformance oracle is the sole arbiter — inline generator
  // checks must not stand in the way.  Shelf-free instances keep the
  // original inline checks as their (complete) arbiter.
  const bool oracle_decides =
      !S->shelf.kappa.empty() && M->depth == (int)ins->N;

  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];
    const auto l = M->where[k]->id;
    const auto kind =
        k < (int)M->ops.size() ? M->ops[k] : (uint8_t)Op::MOVE;

    if (!oracle_decides) {
      if (occupied_next[l] != nullptr) return false;
      auto l_pre = S->C[i]->id;
      if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
          occupied_next[l_pre]->id == occupied_now[l]->id)
        return false;
      if (!S->shelf.kappa.empty() &&
          !forced_op_feasible(S, i, M->where[k], kind))
        return false;
    }

    A[i]->v_next = M->where[k];
    A[i]->op_kind = kind;
    occupied_next[l] = A[i];
    // a carried shelf occupies the destination upper cell at t+1
    if (!S->shelf.kappa.empty() && S->shelf.kappa[i] != KAPPA_FREE &&
        kind != Op::LIFT)
      carrier_upper_add(M->where[k]->index);
  }

  for (auto k : S->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a, S->assignment)) return false;
  }
  return true;
}

bool TAPFPlanner::funcPIBT(Agent* ai, const std::vector<int>& assignment)
{
  if (stats != nullptr) ++stats->pibt_calls;
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();
  const auto task_id = assignment[i];
  // carrier role of this agent (KAPPA_FREE on shelf-free instances)
  const int kappa_i =
      cur_shelf != nullptr && !cur_shelf->kappa.empty() ? cur_shelf->kappa[i]
                                                        : KAPPA_FREE;
  const bool loaded = kappa_i != KAPPA_FREE;
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
      const auto neighbor_task = assignment[aj->id];
      if (neighbor_task < 0) continue;  // carrier agent: no task field
      if (D.get(neighbor_task, u) < D.get(neighbor_task, aj->v_now)) {
        ++count;
      }
    }
    return count;
  };

  // remaining-distance key: instance task (original semantics) — carrier
  // agents get their direction from the guidance layer (M6/M7)
  auto dist_of = [&](Vertex* v) -> int {
    return task_id >= 0 ? (int)D.get(task_id, v) : 0;
  };

  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              const auto dv = dist_of(v);
              const auto du = dist_of(u);
              if (dv != du) return dv < du;
              const auto hv = get_hindrance(v);
              const auto hu = get_hindrance(u);
              if (hv != hu) {
                return hv < hu;
              }
              return tie_breakers[v->id] < tie_breakers[u->id];
            });

  auto swap_agent = swap_possible_and_required(ai, assignment);
  if (swap_agent != nullptr) {
    if (stats != nullptr) ++stats->swap_applied;
    std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
  }

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // S1 (M4): a loaded robot cannot take its shelf into an occupied
    // upper cell (never triggers with an empty shelf layer)
    if (loaded && u != ai->v_now && carrier_upper_taken(u->index)) continue;

    occupied_next[u->id] = ai;
    ai->v_next = u;
    ai->op_kind = u == ai->v_now ? Op::WAIT : Op::MOVE;
    if (loaded) carrier_upper_add(u->index);

    if (ak != nullptr && ak != ai && ak->v_next == nullptr) {
      if (stats != nullptr) ++stats->pibt_recursions;
      if (!funcPIBT(ak, assignment)) continue;
    }

    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      swap_agent->op_kind = Op::MOVE;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    return true;
  }

  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  ai->op_kind = Op::WAIT;
  if (loaded) carrier_upper_add(ai->v_now->index);
  if (stats != nullptr) ++stats->pibt_failures;
  return false;
}

Agent* TAPFPlanner::swap_possible_and_required(
    Agent* ai, const std::vector<int>& assignment)
{
  if (stats != nullptr) ++stats->swap_checks;
  const auto i = ai->id;
  // LaCAM2 swap reasoning is defined over instance-task distance fields;
  // carrier agents (no task) are covered by the yield/park machinery
  // instead (design 5.5, M8)
  if (assignment[i] < 0) return nullptr;
  if (C_next[i][0] == ai->v_now) return nullptr;

  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr && assignment[aj->id] >= 0 &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now, assignment) &&
      is_swap_possible(aj->v_now, ai->v_now, assignment)) {
    return aj;
  }

  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (assignment[ak->id] < 0) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0], assignment) &&
        is_swap_possible(C_next[i][0], ai->v_now, assignment)) {
      return ak;
    }
  }

  return nullptr;
}

bool TAPFPlanner::is_swap_required(const int pusher, const int puller,
                                   Vertex* v_pusher_origin,
                                   Vertex* v_puller_origin,
                                   const std::vector<int>& assignment)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  const auto pusher_task = assignment[pusher];
  const auto puller_task = assignment[puller];

  while (D.get(pusher_task, v_puller) < D.get(pusher_task, v_pusher)) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr &&
           assignment[a->id] >= 0 && ins->tasks[assignment[a->id]] == u)) {
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

  return (D.get(puller_task, v_pusher) < D.get(puller_task, v_puller)) &&
         (D.get(pusher_task, v_pusher) == 0 ||
          D.get(pusher_task, v_puller) < D.get(pusher_task, v_pusher));
}

bool TAPFPlanner::is_swap_possible(Vertex* v_pusher_origin,
                                   Vertex* v_puller_origin,
                                   const std::vector<int>& assignment)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr &&
           assignment[a->id] >= 0 && ins->tasks[assignment[a->id]] == u)) {
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

// ---- carrier layer helpers (M4); all trivial with an empty layer ----

void TAPFPlanner::refresh_carrier_scratch(const TAPFNode* S)
{
  if (carrier_scratch_node == S) return;
  carrier_scratch_node = S;
  cur_shelf = &S->shelf;
  if (S->shelf.kappa.empty()) return;  // no shelf layer: nothing to fill
  std::fill(carrier_grounded.begin(), carrier_grounded.end(), 0);
  std::fill(carrier_upper_base.begin(), carrier_upper_base.end(), 0);
  for (const int p : S->shelf.anon_occ) {
    carrier_grounded[p] = -1;
    carrier_upper_base[p] = 1;
  }
  std::vector<char> carried(ins->target_starts.size(), 0);
  for (const int k : S->shelf.kappa)
    if (k >= 0) carried[k] = 1;
  for (size_t b = 0; b < S->shelf.target_pos.size(); ++b) {
    if (carried[b]) continue;
    carrier_grounded[S->shelf.target_pos[b]] = (int)b + 1;
    carrier_upper_base[S->shelf.target_pos[b]] = 1;
  }
}

bool TAPFPlanner::carrier_upper_taken(int cell) const
{
  return carrier_upper_base[cell] + carrier_upper_delta[cell] > 0;
}

void TAPFPlanner::carrier_upper_add(int cell)
{
  ++carrier_upper_delta[cell];
  carrier_upper_touched.push_back(cell);
}

// per-kind preconditions of a FORCED op under a partial constraint
// (final arbitration is the oracle's; a wrong veto here only delays the
// combination to a deeper constraint — G1 keeps completeness)
bool TAPFPlanner::forced_op_feasible(const TAPFNode* S, int i, Vertex* v,
                                     uint8_t kind)
{
  const int kappa_i = S->shelf.kappa[i];
  switch (kind) {
    case Op::MOVE:
      if (kappa_i != KAPPA_FREE && carrier_upper_taken(v->index))
        return false;  // S1
      return true;
    case Op::LIFT:
      return kappa_i == KAPPA_FREE && carrier_grounded[S->C[i]->index] != 0;
    case Op::DROP:
      if (kappa_i == KAPPA_FREE) return false;
      if (kappa_i == KAPPA_ANON && carrier_upper_taken(v->index))
        return false;  // another shelf occupies the cell at t+1
      return true;
    case Op::WAIT:
    default:
      return true;
  }
}

bool TAPFPlanner::apply_carrier_effects(const TAPFNode* S)
{
  if (S->shelf.kappa.empty()) {
    shelf_next_scratch = S->shelf;  // empty layer: carried over as-is
    return true;
  }
  // assemble the joint op from the agents' reservations
  ops_scratch.resize(N);
  for (const auto a : A) {
    switch (a->op_kind) {
      case Op::MOVE:
        ops_scratch[a->id] = a->v_next == a->v_now
                                 ? Op::make_wait()
                                 : Op::make_move(a->v_next->index);
        break;
      case Op::LIFT:
        ops_scratch[a->id] = Op::make_lift();
        break;
      case Op::DROP:
        ops_scratch[a->id] = Op::make_drop();
        break;
      case Op::WAIT:
      default:
        ops_scratch[a->id] = Op::make_wait();
        break;
    }
  }
  // conformance oracle = final arbiter (design 6.4, M4)
  PhysConfig phys;
  phys.robots.resize(N);
  for (int i = 0; i < N; ++i) phys.robots[i] = S->C[i]->index;
  phys.target_pos = S->shelf.target_pos;
  phys.anon_occ = S->shelf.anon_occ;
  phys.kappa = S->shelf.kappa;
  const auto nxt = apply_ops(*dd_view, phys, ops_scratch);
  if (!nxt.has_value()) return false;
  shelf_next_scratch.target_pos = nxt->target_pos;
  shelf_next_scratch.anon_occ = nxt->anon_occ;
  shelf_next_scratch.kappa = nxt->kappa;
  return true;
}

std::vector<std::vector<Op>> derive_carrier_ops(
    const TAPFInstance& ins, const Solution& sol,
    const std::vector<ShelfState>& shelves)
{
  std::vector<std::vector<Op>> plan;
  if (sol.size() < 2) return plan;
  plan.reserve(sol.size() - 1);
  for (size_t t = 1; t < sol.size(); ++t) {
    std::vector<Op> ops(ins.N, Op::make_wait());
    for (size_t i = 0; i < ins.N; ++i) {
      if (sol[t][i] != sol[t - 1][i]) {
        ops[i] = Op::make_move(sol[t][i]->index);
        continue;
      }
      const int k_from =
          shelves[t - 1].kappa.empty() ? KAPPA_FREE : shelves[t - 1].kappa[i];
      const int k_to =
          shelves[t].kappa.empty() ? KAPPA_FREE : shelves[t].kappa[i];
      if (k_from == KAPPA_FREE && k_to != KAPPA_FREE)
        ops[i] = Op::make_lift();
      else if (k_from != KAPPA_FREE && k_to == KAPPA_FREE)
        ops[i] = Op::make_drop();
    }
    plan.push_back(std::move(ops));
  }
  return plan;
}

Solution solve_tapf(const TAPFInstance& ins, const int verbose,
                    const Deadline* deadline, std::mt19937* MT,
                    const int sticky_penalty, TAPFStats* stats, bool anytime,
                    bool force_full_assignment,
                    TAPFSearchConfig search_config)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tTAPF pre-processing");
  auto planner = TAPFPlanner(&ins, deadline, MT, verbose, sticky_penalty,
                             0.001f, anytime, stats, search_config);
  planner.force_full_assignment = force_full_assignment;
  return planner.solve();
}
