#include "../include/tapf_planner.hpp"

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
                   std::vector<int> _assignment, TAPFNode* _parent)
    : C(_C),
      parent(_parent),
      assignment(_assignment),
      priorities(C.size(), 0),
      order(C.size(), 0),
      search_tree(std::queue<TAPFConstraint*>())
{
  search_tree.push(new TAPFConstraint());
  const auto N = C.size();

  if (parent == nullptr) {
    for (size_t i = 0; i < N; ++i) {
      priorities[i] = (float)D.get(assignment[i], C[i]) / N;
    }
  } else {
    for (size_t i = 0; i < N; ++i) {
      if (D.get(assignment[i], C[i]) != 0) {
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

TAPFNode::~TAPFNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

TAPFPlanner::TAPFPlanner(const TAPFInstance* _ins, const Deadline* _deadline,
                         std::mt19937* _MT, int _verbose,
                         int _sticky_penalty, TAPFStats* _stats)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      sticky_penalty(_sticky_penalty),
      stats(_stats),
      assignment_stats(TAPFAssignmentStats()),
      N(ins->N),
      V_size(ins->G.size()),
      D(TAPFDistTable(ins)),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
  if (stats != nullptr) *stats = TAPFStats();
}

Solution TAPFPlanner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline),
       "ms\tstart TAPF search");

  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  std::stack<TAPFNode*> OPEN;
  std::unordered_map<Config, TAPFNode*, ConfigHasher> CLOSED;
  std::vector<TAPFConstraint*> GC;

  auto initial_assignment =
      assign_tapf_tasks(*ins, D, ins->starts, std::vector<int>(),
                        sticky_penalty, &assignment_stats);
  if (!initial_assignment.feasible) return Solution();

  auto S = new TAPFNode(ins->starts, D, ins, initial_assignment.agent_to_task);
  OPEN.push(S);
  CLOSED[S->C] = S;
  if (stats != nullptr) {
    stats->hl_nodes_created = 1;
    stats->open_max_size = 1;
  }

  int loop_cnt = 0;
  std::vector<Config> solution;

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;
    if (stats != nullptr) {
      ++stats->hl_loop_iterations;
      stats->open_max_size =
          std::max<int>(stats->open_max_size, OPEN.size());
    }
    S = OPEN.top();

    if (is_goal_config(S->C)) {
      while (S != nullptr) {
        solution.push_back(S->C);
        S = S->parent;
      }
      std::reverse(solution.begin(), solution.end());
      if (stats != nullptr) stats->solution_depth = solution.size() - 1;
      break;
    }

    if (S->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    auto M = S->search_tree.front();
    GC.push_back(M);
    S->search_tree.pop();
    if (stats != nullptr) ++stats->constraints_popped;
    if (M->depth < N) {
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
      if (stats != nullptr) ++stats->constraint_failures;
      continue;
    }

    auto C = Config(N, nullptr);
    for (auto a : A) C[a->id] = a->v_next;

    auto assignment =
        assign_tapf_tasks(*ins, D, C, S->assignment, sticky_penalty,
                          &assignment_stats);
    if (!assignment.feasible) continue;
    if (stats != nullptr) {
      for (size_t i = 0; i < assignment.agent_to_task.size(); ++i) {
        if (assignment.agent_to_task[i] != S->assignment[i]) {
          ++stats->assignment_changes;
          break;
        }
      }
    }

    auto iter = CLOSED.find(C);
    if (iter != CLOSED.end()) {
      iter->second->assignment = assignment.agent_to_task;
      OPEN.push(iter->second);
      if (stats != nullptr) {
        ++stats->hl_duplicate_configs;
        ++stats->hl_reinsertions;
      }
      continue;
    }

    auto S_new = new TAPFNode(C, D, ins, assignment.agent_to_task, S);
    OPEN.push(S_new);
    CLOSED[S_new->C] = S_new;
    if (stats != nullptr) ++stats->hl_nodes_created;
  }

  if (stats != nullptr) {
    stats->hl_nodes_explored = CLOSED.size();
    stats->timed_out = solution.empty() && is_expired(deadline);
    stats->assignment_calls = assignment_stats.calls;
    stats->assignment_time_ms = assignment_stats.time_ms;
  }

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       solution.empty() ? (OPEN.empty() ? "no TAPF solution" : "failed")
                        : "TAPF solution found",
       "\tloop_itr:", loop_cnt, "\texplored:", CLOSED.size());

  for (auto a : A) delete a;
  for (auto M : GC) delete M;
  for (auto p : CLOSED) delete p.second;

  return solution;
}

bool TAPFPlanner::is_goal_config(const Config& C) const
{
  auto used = std::vector<bool>(ins->tasks.size(), false);
  for (size_t i = 0; i < ins->N; ++i) {
    auto matched = false;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (used[j] || !ins->allowed[i][j] || C[i] != ins->tasks[j]) continue;
      used[j] = true;
      matched = true;
      break;
    }
    if (!matched) return false;
  }
  return true;
}

bool TAPFPlanner::get_new_config(TAPFNode* S, TAPFConstraint* M)
{
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
    const auto l = M->where[k]->id;

    if (occupied_next[l] != nullptr) return false;
    auto l_pre = S->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    A[i]->v_next = M->where[k];
    occupied_next[l] = A[i];
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

  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr) tie_breakers[u->id] = get_random_float(MT);
  }
  C_next[i][K] = ai->v_now;

  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(task_id, v) + tie_breakers[v->id] <
                     D.get(task_id, u) + tie_breakers[u->id];
            });

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    occupied_next[u->id] = ai;
    ai->v_next = u;

    if (ak == nullptr || u == ai->v_now) return true;
    if (ak->v_next == nullptr) {
      if (stats != nullptr) ++stats->pibt_recursions;
      if (!funcPIBT(ak, assignment)) continue;
    }
    return true;
  }

  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  if (stats != nullptr) ++stats->pibt_failures;
  return false;
}

Solution solve_tapf(const TAPFInstance& ins, const int verbose,
                    const Deadline* deadline, std::mt19937* MT,
                    const int sticky_penalty, TAPFStats* stats)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline),
       "ms\tTAPF pre-processing");
  auto planner =
      TAPFPlanner(&ins, deadline, MT, verbose, sticky_penalty, stats);
  return planner.solve();
}
