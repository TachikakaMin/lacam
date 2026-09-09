#include "../include/planner.hpp"

#include "../include/search_kernel.hpp"

Constraint::Constraint() : who(std::vector<int>()), where(Vertices()), depth(0)
{
}

Constraint::Constraint(Constraint* parent, int i, Vertex* v)
    : who(parent->who), where(parent->where), ops(parent->ops),
      depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
  // ops deliberately not extended: plain-vertex chains never read kinds
  // (readers fall back to MOVE when ops is shorter than depth)
}

Constraint::Constraint(Constraint* parent, int i, OpCand c)
    : who(parent->who), where(parent->where), ops(parent->ops),
      depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(c.v);
  ops.push_back(c.kind);
}

Constraint::~Constraint(){};

Node::Node(Config _C, DistTable& D, Node* _parent)
    : LacamNodeCore<Constraint, Node>(_C, _parent)
{
  init_priorities_and_order([&](size_t i) { return D.get(i, C[i]); });
}

namespace {

struct MapfLacamDomain {
  using Node = ::Node;
  using Constraint = ::Constraint;
  using Successor = Config;
  using Key = Config;
  using KeyHasher = ConfigHasher;
  using Result = Solution;

  Planner& planner;

  explicit MapfLacamDomain(Planner& planner_) : planner(planner_) {}

  bool expired() const { return is_expired(planner.deadline); }

  Node* make_root()
  {
    return new Node(planner.ins->starts, planner.D);
  }

  Key node_key(const Node* node) const { return node->C; }

  Key successor_key(const Successor& successor) const { return successor; }

  bool is_goal(const Node* node) const
  {
    return is_same_config(node->C, planner.ins->goals);
  }

  Result extract_result(const Node* node) const
  {
    Result solution;
    while (node != nullptr) {
      solution.push_back(node->C);
      node = node->parent;
    }
    std::reverse(solution.begin(), solution.end());
    return solution;
  }

  void expand_constraint(Node* node, Constraint* constraint)
  {
    if (constraint->depth >= planner.N) return;
    const int agent = node->order[constraint->depth];
    auto candidates = node->C[agent]->neighbor;
    candidates.push_back(node->C[agent]);
    if (planner.MT != nullptr)
      std::shuffle(candidates.begin(), candidates.end(), *planner.MT);
    lacam_expand_constraint_vec<Constraint>(
        constraint, agent, candidates, node->search_tree);
  }

  std::optional<Successor> generate_successor(
      Node* node, Constraint* constraint)
  {
    if (!planner.get_new_config(node, constraint)) return std::nullopt;
    Successor successor(planner.N, nullptr);
    for (auto* agent : planner.A)
      successor[agent->id] = agent->v_next;
    return successor;
  }

  Node* make_child(const Successor& successor, Node* parent)
  {
    return new Node(successor, planner.D, parent);
  }

  void before_search_object_cleanup()
  {
    for (auto*& agent : planner.A) {
      delete agent;
      agent = nullptr;
    }
  }
};

}  // namespace

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, int _verbose)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
}

Solution Planner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  MapfLacamDomain domain(*this);
  auto outcome = run_lacam_dfs_search(domain);

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       outcome.solution_found
           ? "solution found"
           : (outcome.open_exhausted ? "no solution" : "failed"),
       "\tloop_itr:", outcome.loop_count,
       "\texplored:", outcome.explored);
  return outcome.result;
}

bool Planner::get_new_config(Node* S, Constraint* M)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    // set occupied now
    a->v_now = S->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];        // agent
    const auto l = M->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto l_pre = S->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    // set occupied_next
    A[i]->v_next = M->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : S->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
  }
  return true;
}

bool Planner::funcPIBT(Agent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort, note: K + 1 is sufficient
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts with constraints
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // empty or stay
    if (ak == nullptr || u == ai->v_now) return true;

    // priority inheritance
    if (ak->v_next == nullptr && !funcPIBT(ak)) continue;

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Solution solve(const Instance& ins, const int verbose, const Deadline* deadline,
               std::mt19937* MT)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tpre-processing");
  auto planner = Planner(&ins, deadline, MT, verbose);
  return planner.solve();
}
