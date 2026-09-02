#include "../include/tapf_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <unordered_set>

#include "../include/search_kernel.hpp"
#include "carrier_guidance.hpp"

namespace
{
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

  // guidance infrastructure lives in carrier_guidance.hpp (shared with
  // the carrier adapters in dd_planner.cpp)
  using namespace carrier_detail;

  constexpr int MACRO_CAP = 64;
  constexpr int MACRO_TARGET_LIMIT = 64;
  constexpr int GUIDANCE_REFRESH_STEPS = 8;
  constexpr long FUTILE_REPEAT_LIMIT = 3;
}  // namespace

// out-of-line: CarrierEngine is an implementation type
struct TAPFPlanner::CarrierEngine {
  // upper-deck wall distance (design_final 6.2/D21): ONE shared
  // dest-keyed cache — the field depends only on (walls, dest), so
  // per-target copies were redundant and would duplicate massively
  // under shared goal pools.
  DDDistCache upper_wall;
  LowerDist lower;
  PathCache paths;
  Scratch sc;
  TauEngine tau_engine;  // shelf->goal matching state (T3)
  PhysConfig phys;  // scratch physical view of the node in processing

  explicit CarrierEngine(const DDInstance& dd)
      : upper_wall(dd.grid), lower(dd.grid), sc(dd.grid.size())
  {
  }

  // physical view of a node (oracle coordinates)
  const PhysConfig& phys_view(const TAPFNode* nd)
  {
    phys.robots.resize(nd->C.size());
    for (size_t i = 0; i < nd->C.size(); ++i)
      phys.robots[i] = nd->C[i]->index;
    phys.target_pos = nd->shelf.target_pos;
    phys.anon_occ = nd->shelf.anon_occ;
    phys.kappa = nd->shelf.kappa;
    return phys;
  }
};

TAPFPlanner::~TAPFPlanner()
{
  for (auto a : A) delete a;
}

void TAPFPlanner::attach_carrier_guidance(
    TAPFNode* nd, bool reguide, const CarrierGuidance* rollout_parent_guide,
    bool rewire_rebuild)
{
  if (ins->target_starts.empty()) return;  // natural degradation
  auto& eng = *carrier;
  const auto& dd = *dd_view;
  const auto& phys = eng.phys_view(nd);
  const TAPFNode* par = nd->parent;
  const auto tau_started = std::chrono::steady_clock::now();
  // creation = first attach: folds h and freezes constraint_order.
  // reguide = livelock diversification (own-guide anchor + taboo).
  // rewire_rebuild = duplicate-hit re-anchor on the NEW parent (v3.0
  // §6.1): fresh tasks/rho/hysteresis, no taboo, no h/order changes.
  const bool creation = !reguide && !rewire_rebuild;

  // tau FIRST (design_final 5.3/5.6, T3/T4): every guidance consumer
  // below (guidance-h, paths, park, order, PIBT) reads the ASSIGNED
  // goal.  Hysteresis follows the parent/rollout guide (own guide when
  // re-guiding, the NEW parent when rebuilding after a rewire) —
  // independent of the livelock state; diversification under livelock
  // happens via the tau taboo below, never via h.
  const CarrierGuidance* tau_hyst = nullptr;
  if (rewire_rebuild && par != nullptr && par->guide != nullptr)
    tau_hyst = par->guide.get();
  else if (reguide && nd->guide != nullptr)
    tau_hyst = nd->guide.get();
  else if (par != nullptr && par->guide != nullptr)
    tau_hyst = par->guide.get();
  else
    tau_hyst = rollout_parent_guide;
  bool target_drop_boundary = false;
  if (par != nullptr && !par->shelf.kappa.empty()) {
    for (size_t i = 0; i < nd->shelf.kappa.size(); ++i)
      target_drop_boundary |= par->shelf.kappa[i] >= 0 &&
                              nd->shelf.kappa[i] == KAPPA_FREE;
  }
  const bool preserve_tau =
      tau_hyst != nullptr && !reguide && !target_drop_boundary;
  double h_shelf_tau = 0;
  auto tau_vec = solve_tau(
      dd, phys, eng.upper_wall, eng.tau_engine, weights.alpha,
      weights.gamma,
      tau_hyst != nullptr && !tau_hyst->tau.empty() ? &tau_hyst->tau
                                                    : nullptr,
      nullptr, &h_shelf_tau, preserve_tau);

  // guidance-h (design 5.5 livelock signal; ordering only): sum of
  // ASSIGNED-goal distances (+2 lift/drop proxy) over unsettled targets
  long hg = 0;
  {
    std::vector<char> carried(dd.n_targets(), 0);
    for (int k : phys.kappa)
      if (k >= 0) carried[k] = 1;
    for (size_t b = 0; b < dd.n_targets(); ++b) {
      const bool done =
          !carried[b] && phys.target_pos[b] == tau_vec[b];
      if (done) continue;
      const auto& d = eng.upper_wall.to(tau_vec[b]);
      hg += d[phys.target_pos[b]] + 2;
    }
  }
  nd->h_guidance = hg;
  if (creation) {
    nd->best_h = par ? std::min(par->best_h, hg) : hg;
    nd->no_progress = (par != nullptr && hg > 0 && hg >= par->best_h)
                          ? par->no_progress + 1
                          : 0;
  }
  const bool livelock =
      reguide || (!rewire_rebuild && nd->no_progress > 0 &&
                  nd->no_progress % LIVELOCK_WINDOW == 0);
  std::vector<std::pair<int, int>> taboo;
  if (livelock) {
    const CarrierGuidance* src =
        reguide ? nd->guide.get() : (par != nullptr ? par->guide.get() : nullptr);
    if (src != nullptr) {
      if (!reguide) {
        // wait-for refinement (M9): taboo only the cycle members' rho
        // pairs when a structural cycle exists; blanket taboo otherwise
        eng.sc.occ_node = nullptr;
        fill_occupancy(dd, phys, eng.sc, tau_vec);
        const auto cyc = waitfor_cycles(dd, phys, *src, eng.lower, eng.sc);
        for (const int i : cyc)
          if (src->rho[i] >= 0) taboo.emplace_back(i, src->free_goal[i]);
      }
      if (taboo.empty()) {
        for (size_t i = 0; i < ins->N; ++i)
          if (src->rho[i] >= 0)
            taboo.emplace_back((int)i, src->free_goal[i]);
      }
    }
    // Tau repair is local and task-boundary based. Retargeting every shelf
    // at once breaks in-flight robot/shelf episodes, so one unfinished,
    // grounded multi-goal row is released per livelock epoch. The matching
    // repair may move an alternating cycle, but all unrelated pairs remain
    // stable through the parent hysteresis. Carried rows are locked inside
    // solve_tau; singleton rows remain structurally exempt.
    std::vector<std::pair<int, int>> tau_taboo;
    const size_t epoch =
        static_cast<size_t>(reguide ? std::max(0, nd->revisits / 8)
                                    : std::max(0, nd->no_progress /
                                                     LIVELOCK_WINDOW));
    for (size_t offset = 0; offset < tau_vec.size(); ++offset) {
      const size_t b = (epoch + offset) % tau_vec.size();
      const bool carried =
          std::find(phys.kappa.begin(), phys.kappa.end(), (int)b) !=
          phys.kappa.end();
      if (carried || dd.target_goal_sets[b].size() <= 1 ||
          phys.target_pos[b] == tau_vec[b])
        continue;
      tau_taboo.emplace_back((int)b, tau_vec[b]);
      break;
    }
    if (!tau_taboo.empty())
      tau_vec = solve_tau(
          dd, phys, eng.upper_wall, eng.tau_engine, weights.alpha,
          weights.gamma,
          tau_hyst != nullptr && !tau_hyst->tau.empty() ? &tau_hyst->tau
                                                        : nullptr,
          &tau_taboo, nullptr, false);
  }
  if (stats != nullptr) {
    stats->tau_time_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - tau_started)
            .count();
    if (tau_hyst != nullptr && tau_hyst->tau.size() == tau_vec.size()) {
      long changed = 0;
      for (size_t b = 0; b < tau_vec.size(); ++b)
        changed += tau_hyst->tau[b] != tau_vec[b];
      if (changed > 0) {
        ++stats->tau_change_builds;
        stats->tau_pair_changes += changed;
      }
    }
  }
  const CarrierGuidance* parent_guide =
      (!livelock && par != nullptr) ? par->guide.get() : nullptr;
  if (parent_guide == nullptr && !livelock)
    parent_guide = rollout_parent_guide;  // parentless rollout probes
  const auto guidance_started = std::chrono::steady_clock::now();
  auto guide = std::make_unique<CarrierGuidance>(
      build_guidance(dd, phys, tau_vec, eng.upper_wall, eng.lower,
                     eng.paths, eng.sc, nd, livelock ? &taboo : nullptr,
                     parent_guide));
  if (stats != nullptr) ++stats->guidance_builds;

  // ---- v3.0 §5.1 execution-price repair (ordering-only) ----
  // One light feedback round via the SHARED compute_execution_prices;
  // prices never touch the admissible h (invariant 18); row-wise (>256)
  // and all-singleton regimes skip the round entirely.
  if ((int)dd.n_targets() <= ASSIGNMENT_EXACT_LIMIT &&
      !eng.tau_engine.all_singleton) {
    std::vector<std::pair<int, double>> price;
    if (compute_execution_prices(dd, phys, *guide, tau_vec,
                                 eng.upper_wall, eng.lower, weights.alpha,
                                 weights.beta, price)) {
      auto tau1 = solve_tau(
          dd, phys, eng.upper_wall, eng.tau_engine, weights.alpha,
          weights.gamma,
          tau_hyst != nullptr && !tau_hyst->tau.empty() ? &tau_hyst->tau
                                                        : nullptr,
          nullptr, nullptr, false, &price);
      if (tau1 != tau_vec) {
        tau_vec = std::move(tau1);
        guide = std::make_unique<CarrierGuidance>(build_guidance(
            dd, phys, tau_vec, eng.upper_wall, eng.lower, eng.paths,
            eng.sc, nd, livelock ? &taboo : nullptr, parent_guide));
        if (stats != nullptr) {
          ++stats->tau_price_repairs;
          ++stats->guidance_builds;
        }
      }
    }
  }
  nd->guide = std::move(guide);
  if (stats != nullptr) {
    stats->guidance_time_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - guidance_started)
            .count();
    if (tau_hyst != nullptr &&
        tau_hyst->free_goal.size() == nd->guide->free_goal.size()) {
      long changed = 0;
      for (size_t i = 0; i < nd->guide->free_goal.size(); ++i)
        changed += tau_hyst->free_goal[i] != nd->guide->free_goal[i];
      if (changed > 0) {
        ++stats->rho_change_builds;
        stats->rho_pair_changes += changed;
      }
    }
  }

  // PIBT order: class layering (design 5.4 N.order): loaded-target >
  // loaded-anon/parked > free-assigned > free-idle; within class by
  // remaining distance then id (stable)
  const auto& g = *nd->guide;
  auto cls = [&](int i) {
    const int k = phys.kappa[i];
    if (k >= 0) return g.target_park[k] ? 1 : 0;
    if (k == KAPPA_ANON) return 1;
    if (g.rho[i] >= 0) return 2;
    return 3;
  };
  auto rem = [&](int i) -> int {
    const int k = phys.kappa[i];
    if (k >= 0) {
      const auto& d = eng.upper_wall.to(g.tau[k]);
      return d[phys.robots[i]];
    }
    return 0;
  };
  std::iota(nd->order.begin(), nd->order.end(), 0);
  std::stable_sort(nd->order.begin(), nd->order.end(), [&](int a, int b) {
    const int ca = cls(a), cb = cls(b);
    if (ca != cb) return ca < cb;
    const int ra = rem(a), rb = rem(b);
    return ra < rb;
  });
  if (livelock && MT != nullptr) {
    // shuffle whole then re-sort by class: randomizes within classes only;
    // the FROZEN constraint_order is never touched (D11)
    std::shuffle(nd->order.begin(), nd->order.end(), *MT);
    std::stable_sort(nd->order.begin(), nd->order.end(),
                     [&](int a, int b) { return cls(a) < cls(b); });
  }
  if (creation) {
    nd->constraint_order = nd->order;  // frozen at creation (D11, M3)
    // admissible shelf h (design_final 4.3/5.3): the LB-matching value
    // from solve_tau — bit-identical to the old per-target formula on
    // singleton instances (forced assignment, same arithmetic)
    nd->h += h_shelf_tau;
    nd->f = nd->g + nd->h;
  }
}


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
      occupied_next(Agents(V_size, nullptr)),
      pibt_cand(N)
{
  if (stats != nullptr) *stats = TAPFStats();
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);
  // Solver-objective weights default to one; optional numeric objective
  // inputs DD_ALPHA..DD_DELTA are read once.  Shelf-free instances never
  // evaluate the carrier cost term.
  carrier_detail::load_solver_weights(weights);
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
    dd_view->target_goal_sets = ins->target_goal_sets;  // T1: eligibility
    dd_view->finalize();
    const size_t n_cells = ins->G.U.size();
    carrier_grounded.assign(n_cells, 0);
    carrier_upper_delta.assign(n_cells, 0);
    carrier = std::make_unique<CarrierEngine>(*dd_view);
  }
}

Solution TAPFPlanner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart TAPF search");

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

  // Pruning bound (M11): the incumbent's g, or an externally supplied
  // upper bound; -1 means no bound.
  auto current_bound = [&]() -> double {
    if (S_goal != nullptr) return S_goal->g;
    return search_config.incumbent_init;
  };

  auto select_open_index = [&]() -> size_t {
    if (search_config.mode == TAPFSearchMode::DFS ||
        (S_goal == nullptr && search_config.incumbent_init < 0)) {
      return OPEN.size() - 1;
    }
    // shared FOCAL kernel (search_kernel.hpp) — same semantics as before
    return focal_select_index(
        OPEN, search_config.focal_weight,
        [](const TAPFNode* n) { return static_cast<double>(n->f); },
        [&](const TAPFNode* n) {
          const double b = current_bound();
          return !n->search_tree.empty() && (b < 0 || n->f < b);
        },
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
  attach_carrier_guidance(S_init);
  deepest_node = S_init;
  deepest_depth = 0;
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

    // v3.0 §6.1 lazy re-anchor: this node was reparented by a rewire
    // since its guidance was built; rebuild from the CURRENT parent
    // before expanding under stale hysteresis/tasks/rho.
    if (S->guidance_stale && S->guide != nullptr) {
      S->guidance_stale = false;
      attach_carrier_guidance(S, /*reguide=*/false, nullptr,
                              /*rewire_rebuild=*/true);
      if (stats != nullptr) ++stats->rewire_guidance_rebuilds;
    }

    {
      const double bound = current_bound();
      if (bound >= 0 && S->f >= bound) {
        if (stats != nullptr) ++stats->f_pruned;
        erase_open(open_index);
        continue;
      }
    }

    if (is_goal_config(S->C, S->shelf)) {
      const bool improves =
          (S_goal == nullptr || S->g < S_goal->g) &&
          (search_config.incumbent_init < 0 ||
           S->g < search_config.incumbent_init);
      if (improves) {
        if (stats != nullptr) {
          ++stats->incumbent_updates;
          if (stats->first_solution_cost == 0) {
            stats->first_solution_cost = (unsigned)std::lround(S->g);
            stats->first_solution_g = S->g;
            stats->first_solution_time_ms = elapsed_ms(deadline);
          }
        }
        S_goal = S;
        if (stats != nullptr) ++stats->anytime_cost_updates;
        info(1, verbose, "elapsed:", elapsed_ms(deadline),
             "ms\tfound TAPF solution\tcost:", S_goal->g);
      }
      if (search_config.stop_at_first && S_goal != nullptr) break;
      if (!anytime || deadline == nullptr ||
          (S_goal != nullptr && S_goal->g <= initial_lower_bound)) {
        break;
      }
      continue;
    }

    // macro successor probe (design 7.1/D13/D14, M10): on the node's
    // FIRST expansion, before any incumbent, within the carrier scale
    // regime, roll the unconstrained generator to the next lift/drop
    // event and push the terminal state as an extra DFS-top successor.
    // The node's constraint tree is untouched (completeness free);
    // structurally unreachable on shelf-free instances (h_guidance == 0).
    if (search_config.macro_enabled && S_goal == nullptr &&
        !S->macro_tried && S->h_guidance > 0 && ins->tasks.empty() &&
        (int)ins->target_starts.size() <= MACRO_TARGET_LIMIT) {
      S->macro_tried = true;
      auto r = carrier_rollout(S->C, S->shelf, MACRO_CAP, 0,
                               /*stop_on_event=*/true);
      // rollout probes died: their addresses may be recycled by the
      // nodes created below — stale address-keyed scratches are poison
      invalidate_carrier_scratch();
      if (r.ops.size() >= 2) {
        lookup_key.C = r.configs.back();
        lookup_key.S = r.shelves.back();
        if (CLOSED.find(lookup_key) == CLOSED.end()) {
          auto S_macro =
              new TAPFNode(r.configs.back(), r.shelves.back(), D, ins,
                           std::vector<int>(N, -1), S->assignment_state, S);
          S_macro->g = S->g + r.cost;
          S_macro->h = 0;
          S_macro->f = S_macro->g + S_macro->h;
          attach_carrier_guidance(S_macro);
          CLOSED[SearchKey{S_macro->C, S_macro->shelf}] = S_macro;
          MacroEdge edge;
          edge.cost = r.cost;
          edge.configs.assign(r.configs.begin() + 1, r.configs.end() - 1);
          edge.shelves.assign(r.shelves.begin() + 1, r.shelves.end() - 1);
          macro_edges[{S, S_macro}] = std::move(edge);
          push_open(S_macro);
          if (stats != nullptr) {
            ++stats->hl_nodes_created;
            ++stats->macro_successors;
            if (r.shelf_moved)
              ++stats->macro_shelf_motion_successors;
            else
              ++stats->macro_robot_only_successors;
            // macro_after_first stays 0 by construction: this block is
            // gated on S_goal == nullptr.
          }
          continue;  // S stays queued; DFS tries the macro child first
        }
      }
    }

    auto M = S->search_tree.front();
    S->search_tree.pop();
    if (stats != nullptr) ++stats->constraints_popped;
    if (M->depth < N) {
      auto i = S->constraint_order[M->depth];
      auto ops_cand = std::vector<OpCand>();
      build_op_candidates(S, i, ops_cand);
      lacam_expand_constraint_vec<TAPFConstraint>(M, i, ops_cand,
                                                  S->search_tree);
      if (stats != nullptr) stats->constraints_generated += ops_cand.size();
    }

    if (!get_new_config(S, M)) {
      delete M;
      if (stats != nullptr) ++stats->constraint_failures;
      continue;
    }
    const auto M_depth = M->depth;  // for the reject-counter split below
    delete M;

    for (auto a : A) C_new[a->id] = a->v_next;

    // carrier layer successor (M4): assemble the joint op and let the
    // conformance oracle arbitrate; fills shelf_next_scratch.  Trivially
    // true (layer copied) on shelf-free instances.
    if (!apply_carrier_effects(S)) {
      if (stats != nullptr) {
        if (M_depth == N)
          ++stats->carrier_g1_rejects;  // exhaustive-tree combos (G1)
        else
          ++stats->carrier_validator_rejects;
      }
      continue;
    }

    lookup_key.C = C_new;
    lookup_key.S = shelf_next_scratch;
    auto iter = CLOSED.find(lookup_key);
    if (iter != CLOSED.end()) {
      auto S_known = iter->second;
      S->neighbor.insert(S_known);
      // v3.0 §6.1 / R3: rewrite() itself marks every REPARENTED node
      // (S_known and propagated descendants) guidance_stale; the lazy
      // rebuild happens at each node's next expansion.
      rewrite(S, S_goal, OPEN);
      auto S_insert = S_known;
      if (MT != nullptr && get_random_float(MT) < restart_rate) {
        S_insert = S_init;
      }
      {
        const double b = current_bound();
        if ((b < 0 || S_insert->f < b) && !S_insert->queued &&
            !S_insert->search_tree.empty()) {
          push_open(S_insert);
          if (stats != nullptr) ++stats->hl_reinsertions;
        }
      }
      if (stats != nullptr) ++stats->hl_duplicate_configs;
      // carrier livelock diversification on heavy revisiting (design 5.5
      // signal B, M9): re-match rho with the node's pairs tabooed and
      // reshuffle within classes; allow a fresh macro probe.  Ordering
      // only; never reached without a guidance layer.
      if (S_known->guide != nullptr) {
        ++S_known->revisits;
        if (S_known->revisits % 8 == 0) {
          S_known->macro_tried = false;
          attach_carrier_guidance(S_known, /*reguide=*/true);
        }
      }
      continue;
    }

    auto changed_agents = std::vector<int>();
    changed_agents.reserve(N);
    for (int i = 0; i < N; ++i) {
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

    if (stats != nullptr) {
      bool shelf_motion = false;
      bool manipulation = false;
      for (size_t i = 0; i < ops_scratch.size(); ++i) {
        const auto kind = ops_scratch[i].kind;
        shelf_motion |=
            kind == Op::MOVE && !S->shelf.kappa.empty() &&
            S->shelf.kappa[i] != KAPPA_FREE;
        manipulation |= kind == Op::LIFT || kind == Op::DROP;
      }
      if (shelf_motion)
        ++stats->shelf_motion_successors;
      else if (manipulation)
        ++stats->manipulation_successors;
      else
        ++stats->robot_only_successors;
    }

    auto S_new = new TAPFNode(C_new, shelf_next_scratch, D, ins,
                              assignment.agent_to_task, assignment_state, S);
    S_new->g = S->g + get_edge_cost(S, S_new);
    S_new->h = assignment.cost;
    S_new->f = S_new->g + S_new->h;
    attach_carrier_guidance(S_new);
    if (!S_new->shelf.kappa.empty() && S->parent != nullptr)
      note_lift_cycle(S->parent->C, S->parent->shelf, S->C, S->shelf,
                      S_new->C, S_new->shelf);
    CLOSED[SearchKey{S_new->C, S_new->shelf}] = S_new;
    if (deepest_node == nullptr || S_new->depth > deepest_depth) {
      deepest_node = S_new;
      deepest_depth = S_new->depth;
    }
    if (!S_new->shelf.kappa.empty()) {
      long done = 0;
      for (size_t b = 0; b < S_new->shelf.target_pos.size(); ++b) {
        bool carried = false;
        for (const int k : S_new->shelf.kappa) carried |= (k == (int)b);
        if (!carried &&
            std::binary_search(ins->target_goal_sets[b].begin(),
                               ins->target_goal_sets[b].end(),
                               S_new->shelf.target_pos[b]))
          ++done;
      }
      best_targets_done = std::max(best_targets_done, done);
    }
    {
      const double b = current_bound();
      if (b < 0 || S_new->f < b) {
        push_open(S_new);
      }
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
      // expand a multi-step macro edge into its executed intermediates
      // (reverse order here; the final reverse restores time order)
      if (S->parent != nullptr) {
        const auto it = macro_edges.find({S->parent, S});
        if (it != macro_edges.end()) {
          const auto& e = it->second;
          for (size_t k = e.configs.size(); k-- > 0;) {
            solution.push_back(e.configs[k]);
            solution_shelves.push_back(e.shelves[k]);
          }
        }
      }
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

  // best-effort chain to the deepest node (carrier debug aid, M16)
  best_effort_solution.clear();
  best_effort_shelves.clear();
  best_effort_tau.clear();
  if (S_goal == nullptr && deepest_node != nullptr &&
      !ins->shelf_cells.empty()) {
    if (deepest_node->guide != nullptr)
      best_effort_tau = deepest_node->guide->tau;
    for (const TAPFNode* S2 = deepest_node; S2 != nullptr; S2 = S2->parent) {
      best_effort_solution.push_back(S2->C);
      best_effort_shelves.push_back(S2->shelf);
      if (S2->parent != nullptr) {
        const auto it = macro_edges.find({S2->parent, S2});
        if (it != macro_edges.end()) {
          for (size_t k = it->second.configs.size(); k-- > 0;) {
            best_effort_solution.push_back(it->second.configs[k]);
            best_effort_shelves.push_back(it->second.shelves[k]);
          }
        }
      }
    }
    std::reverse(best_effort_solution.begin(), best_effort_solution.end());
    std::reverse(best_effort_shelves.begin(), best_effort_shelves.end());
  }

  for (auto p : CLOSED) delete p.second;
  deepest_node = nullptr;  // owned by CLOSED; gone now

  return solution;
}

void TAPFPlanner::rewrite(TAPFNode* from, TAPFNode* goal,
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
        // R3 (debug.md §10, invariant 21): ANY node reparented by the
        // relaxation — the duplicate-hit node and every propagated
        // descendant alike — must re-anchor its guidance (hysteresis,
        // tasks, rho) on the new parent.  Lazy: flag here, rebuild at
        // the node's next expansion.
        if (node_to->parent != node_from && node_to->guide != nullptr)
          node_to->guidance_stale = true;
        node_to->parent = node_from;
        Q.push(node_to);
        if (stats != nullptr) {
          ++stats->anytime_cost_updates;
          ++stats->g_relaxed;
        }
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
  // multi-step macro edge (M10): the stored trace cost is authoritative
  // (the one-step formula below would under-count a multi-cell jump).
  // The map is always empty on shelf-free instances.
  if (!macro_edges.empty()) {
    const auto it = macro_edges.find({from, to});
    if (it != macro_edges.end()) return it->second.cost;
  }
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
  // carrier part (design_final 2.2/Prop 3, D10): every target grounded on
  // an ELIGIBLE goal cell (set membership; singleton == old equality)
  for (size_t b = 0; b < ins->target_goal_sets.size(); ++b) {
    const auto& set = ins->target_goal_sets[b];
    if (!std::binary_search(set.begin(), set.end(), S.target_pos[b]))
      return false;
  }
  for (const int k : S.kappa)
    if (k >= 0) return false;  // a carried target is not grounded
  return true;
}

void TAPFPlanner::build_op_candidates(TAPFNode* S, int i,
                                      std::vector<OpCand>& out)
{
  // operator candidates (design 5.2, M3): every vertex candidate is a
  // MOVE (or WAIT at the own cell); LIFT/DROP append — their guards are
  // structurally false on shelf-free instances, so the candidate set,
  // order and RNG consumption stay exactly the original there.
  auto C = S->C[i]->neighbor;
  C.push_back(S->C[i]);
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  out.clear();
  out.reserve(C.size() + 2);
  for (auto u : C)
    out.push_back(OpCand{u, (uint8_t)(u == S->C[i] ? Op::WAIT : Op::MOVE)});
  if (!S->shelf.kappa.empty()) {
    refresh_carrier_scratch(S);
    const int cell = S->C[i]->index;
    if (S->shelf.kappa[i] == KAPPA_FREE) {
      if (carrier_grounded[cell] != 0)
        out.push_back(OpCand{S->C[i], (uint8_t)Op::LIFT});
    } else {
      out.push_back(OpCand{S->C[i], (uint8_t)Op::DROP});
    }
  }
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

// Anonymous shelves are interchangeable and share one identity per cell.
static inline uint64_t lift_futile_key(int shelf_id, int cell)
{
  return ((uint64_t)(uint32_t)(shelf_id + 1) << 32) | (uint32_t)cell;
}

void TAPFPlanner::note_lift_cycle(
    const Config& before_lift_C, const ShelfState& before_lift_S,
    const Config& loaded_C, const ShelfState& loaded_S,
    const Config& after_drop_C, const ShelfState& after_drop_S)
{
  if (loaded_S.kappa.empty()) return;
  ++futile_clock;
  const long window = std::max<long>(64, 8L * V_size);
  for (size_t i = 0; i < loaded_S.kappa.size(); ++i) {
    const int shelf = loaded_S.kappa[i];
    if (shelf == KAPPA_FREE ||
        before_lift_S.kappa[i] != KAPPA_FREE ||
        after_drop_S.kappa[i] != KAPPA_FREE)
      continue;
    if (before_lift_C[i] != loaded_C[i] ||
        loaded_C[i] != after_drop_C[i])
      continue;
    auto& e =
        lift_futile[lift_futile_key(shelf >= 0 ? shelf : -1,
                                    loaded_C[i]->index)];
    if (futile_clock - e.second > window) e.first = 0;
    ++e.first;
    e.second = futile_clock;
  }
}

bool TAPFPlanner::lift_on_cooldown(int cell) const
{
  if (lift_futile.empty()) return false;
  const int g = carrier_grounded[cell];  // 0 none / -1 anon / b+1
  if (g == 0) return false;
  const auto it =
      lift_futile.find(lift_futile_key(g > 0 ? g - 1 : -1, cell));
  if (it == lift_futile.end()) return false;
  const long window = std::max<long>(64, 8L * V_size);
  return it->second.first >= FUTILE_REPEAT_LIMIT &&
         futile_clock - it->second.second <= window;
}

bool TAPFPlanner::funcPIBT(Agent* ai, const std::vector<int>& assignment)
{
  if (stats != nullptr) ++stats->pibt_calls;
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();
  const auto task_id = assignment[i];
  // carrier role of this agent (KAPPA_FREE on shelf-free instances)
  const int kappa_i = carrier_scratch_node != nullptr &&
                              !carrier_scratch_node->shelf.kappa.empty()
                          ? carrier_scratch_node->shelf.kappa[i]
                          : KAPPA_FREE;
  const bool loaded = kappa_i != KAPPA_FREE;
  const CarrierGuidance* guide =
      carrier_scratch_node != nullptr ? carrier_scratch_node->guide.get()
                                      : nullptr;

  Agent* swap_agent = nullptr;
  // preference-ordered op candidates (per-agent buffer: recursion-safe)
  auto& cand = pibt_cand[i];
  cand.clear();

  if (task_id >= 0) {
    // ---- ORIGINAL task-agent candidate construction (unchanged) ----
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

    std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
              [&](Vertex* const v, Vertex* const u) {
                const auto dv = D.get(task_id, v);
                const auto du = D.get(task_id, u);
                if (dv != du) return dv < du;
                const auto hv = get_hindrance(v);
                const auto hu = get_hindrance(u);
                if (hv != hu) {
                  return hv < hu;
                }
                return tie_breakers[v->id] < tie_breakers[u->id];
              });

    swap_agent = swap_possible_and_required(ai, assignment);
    if (swap_agent != nullptr) {
      if (stats != nullptr) ++stats->swap_applied;
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
    }

    for (size_t k = 0; k < K + 1; ++k)
      cand.push_back({C_next[i][k], (uint8_t)(C_next[i][k] == ai->v_now
                                                  ? Op::WAIT
                                                  : Op::MOVE)});
  } else {
    // ---- carrier roles (design 5.4 candidate table, M7) ----
    auto& eng = *carrier;
    auto push_moves_sorted_by = [&](auto&& dist_of_cell, bool s1_filter) {
      auto cells = std::vector<Vertex*>(ai->v_now->neighbor);
      if (MT != nullptr) std::shuffle(cells.begin(), cells.end(), *MT);
      std::stable_sort(cells.begin(), cells.end(),
                       [&](Vertex* a, Vertex* b) {
                         return dist_of_cell(a->index) < dist_of_cell(b->index);
                       });
      for (auto* v : cells) {
        if (s1_filter && carrier_upper_taken(v->index)) continue;
        cand.push_back({v, (uint8_t)Op::MOVE});
      }
    };
    const int q = ai->v_now->index;

    if (kappa_i >= 0 && guide != nullptr && !guide->target_park[kappa_i]) {
      // loaded with an unparked target
      const int b = kappa_i;
      const auto& dgoal = eng.upper_wall.to(guide->tau[b]);
      if (q == guide->tau[b]) {
        cand.push_back({ai->v_now, (uint8_t)Op::DROP});
        cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
      } else if (guide->plan_bound) {
        // B1 hard constraint: only the fixed plan's next cell or waiting
        const int nxt_cell = guide->target_next[b];
        if (nxt_cell >= 0 && !carrier_upper_taken(nxt_cell))
          for (auto* v : ai->v_now->neighbor)
            if (v->index == nxt_cell) {
              cand.push_back({v, (uint8_t)Op::MOVE});
              break;
            }
        cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
      } else {
        const int nxt_cell = guide->target_next[b];
        if (nxt_cell >= 0 && !carrier_upper_taken(nxt_cell))
          for (auto* v : ai->v_now->neighbor)
            if (v->index == nxt_cell) {
              cand.push_back({v, (uint8_t)Op::MOVE});
              break;
            }
        push_moves_sorted_by([&](int c) { return dgoal[c]; }, true);
        if (cand.size() >= 2)  // dedupe the path-head candidate
          for (size_t a = 1; a < cand.size(); ++a)
            if (cand[a].first == cand[0].first &&
                cand[a].second == cand[0].second) {
              cand.erase(cand.begin() + a);
              break;
            }
        // Drop as a yield when the path head is STRUCTURALLY blocked
        const bool structurally_blocked =
            nxt_cell >= 0 && carrier_grounded[nxt_cell] != 0;
        if (structurally_blocked) {
          cand.push_back({ai->v_now, (uint8_t)Op::DROP});
          cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
        } else {
          cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
          cand.push_back({ai->v_now, (uint8_t)Op::DROP});
        }
      }
    } else if (kappa_i == KAPPA_ANON ||
               (kappa_i >= 0 && guide != nullptr &&
                guide->target_park[kappa_i])) {
      // clear duty: carry to the drop cell and drop.  R2 (invariant 23):
      // an ANON carrier executing a task with a COMMITTED drop (one-empty
      // ready: to = the vacancy, kept fresh per node by build_guidance)
      // carries to task.to; un-committed tasks delegate the drop to the
      // per-node parking choice (carrier-chosen, position-fresh — the
      // semantics that keep dense boards healthy, d50 bound test).
      int park = guide != nullptr ? guide->parking_cell[i] : -1;
      if (kappa_i == KAPPA_ANON && guide != nullptr &&
          guide->custody.size() > (size_t)i &&
          guide->custody[i].id != 0 && guide->custody[i].to_committed &&
          guide->custody[i].to >= 0)
        park = guide->custody[i].to;
      if (park == q) {
        cand.push_back({ai->v_now, (uint8_t)Op::DROP});
        cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
      } else {
        if (park >= 0)
          push_moves_sorted_by(
              [&](int c) { return eng.lower.dist(park, c); }, true);
        else
          push_moves_sorted_by([](int) { return 0; }, true);
        cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
        cand.push_back({ai->v_now, (uint8_t)Op::DROP});
      }
    } else {
      // free robot
      const int goal = guide != nullptr ? guide->free_goal[i] : -1;
      bool lift_here = goal >= 0 && q == goal && carrier_grounded[q] != 0;
      bool lift_demoted = false;
      if (lift_here) {
        if (lift_on_cooldown(q)) {
          lift_demoted = true;
          if (stats != nullptr) ++stats->futile_lift_demotions;
        }
        if (!lift_demoted) cand.push_back({ai->v_now, (uint8_t)Op::LIFT});
      }
      if (goal >= 0) {
        push_moves_sorted_by(
            [&](int c) { return eng.lower.dist(goal, c); }, false);
        cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
        if (lift_demoted) cand.push_back({ai->v_now, (uint8_t)Op::LIFT});
      } else if (eng.sc.protect[q] != 0) {
        // idle ON an active corridor: step off first, wait, then
        // protected moves (still pushable)
        push_moves_sorted_by(
            [&](int c) { return eng.sc.protect[c] ? 1 : 0; }, false);
        size_t first_prot = 0;
        while (first_prot < cand.size() &&
               !(cand[first_prot].second == Op::MOVE &&
                 eng.sc.protect[cand[first_prot].first->index]))
          ++first_prot;
        cand.insert(cand.begin() + first_prot,
                    {ai->v_now, (uint8_t)Op::WAIT});
      } else {
        cand.push_back({ai->v_now, (uint8_t)Op::WAIT});
        push_moves_sorted_by([](int) { return 0; }, false);
      }
    }
  }

  // ---- unified try loop ----
  // Reservation semantics are ROLE-dependent (design 5.4; debug.md v3
  // section 4 D1, regression `search_not_dominated_by_own_rollout_...`):
  //   task agents  — upstream shape verbatim (keep the reservation on a
  //                  failed push; wait fallback reserves and FAILS);
  //   carrier agents — the two-deck generator releases the reservation
  //                  and tries its next candidate (a failed push must not
  //                  poison the remaining candidates: S1 makes carrier
  //                  pushes fail far more often than task pushes), and
  //                  the wait fallback SUCCEEDS when feasible.
  const bool carrier_role =
      task_id < 0 && carrier_scratch_node != nullptr &&
      !carrier_scratch_node->shelf.kappa.empty();
  for (size_t k = 0; k < cand.size(); ++k) {
    auto u = cand[k].first;
    const uint8_t kind = cand[k].second;
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // carrier feasibility (M4); none of these fire for task agents
    if (kind == Op::MOVE && loaded && carrier_upper_taken(u->index))
      continue;  // S1
    if (kind == Op::LIFT && carrier_grounded[u->index] == 0) continue;
    if (kind == Op::DROP && kappa_i == KAPPA_ANON &&
        carrier_upper_taken(u->index))
      continue;

    occupied_next[u->id] = ai;
    ai->v_next = u;
    ai->op_kind = kind;
    if (loaded && kind != Op::LIFT) carrier_upper_add(u->index);

    if (ak != nullptr && ak != ai && ak->v_next == nullptr) {
      if (stats != nullptr) ++stats->pibt_recursions;
      if (!funcPIBT(ak, assignment)) {
        if (carrier_role) {
          // release and retry the next candidate (two-deck semantics)
          if (occupied_next[u->id] == ai) occupied_next[u->id] = nullptr;
          ai->v_next = nullptr;
          if (loaded && kind != Op::LIFT) carrier_upper_sub(u->index);
        }
        continue;
      }
    }

    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      swap_agent->op_kind = Op::MOVE;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    return true;
  }

  if (carrier_role) {
    // two-deck wait fallback: succeed when feasible, no forced reservation
    if (occupied_next[ai->v_now->id] == nullptr) {
      occupied_next[ai->v_now->id] = ai;
      ai->v_next = ai->v_now;
      ai->op_kind = Op::WAIT;
      if (loaded) carrier_upper_add(ai->v_now->index);
      return true;
    }
    if (stats != nullptr) ++stats->pibt_failures;
    return false;
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

long TAPFPlanner::carrier_path_recomputes() const
{
  return carrier != nullptr ? carrier->paths.recomputes : 0;
}

long TAPFPlanner::carrier_path_cache_hits() const
{
  return carrier != nullptr ? carrier->paths.hits : 0;
}

void TAPFPlanner::invalidate_carrier_scratch()
{
  carrier_scratch_node = nullptr;
  if (carrier != nullptr) carrier->sc.occ_node = nullptr;
}

void TAPFPlanner::refresh_carrier_scratch(const TAPFNode* S)
{
  if (carrier_scratch_node == S) return;
  carrier_scratch_node = S;
  if (S->shelf.kappa.empty()) return;  // no shelf layer: nothing to fill
  std::fill(carrier_grounded.begin(), carrier_grounded.end(), 0);
  for (const int p : S->shelf.anon_occ) carrier_grounded[p] = -1;
  std::vector<char> carried(ins->target_starts.size(), 0);
  for (const int k : S->shelf.kappa)
    if (k >= 0) carried[k] = 1;
  for (size_t b = 0; b < S->shelf.target_pos.size(); ++b)
    if (!carried[b]) carrier_grounded[S->shelf.target_pos[b]] = (int)b + 1;
}

bool TAPFPlanner::carrier_upper_taken(int cell) const
{
  // grounded shelves occupy their cell at t+1 (carrier_grounded != 0);
  // carried-shelf destination reservations live in the delta counters
  return carrier_grounded[cell] != 0 || carrier_upper_delta[cell] > 0;
}

void TAPFPlanner::carrier_upper_add(int cell)
{
  ++carrier_upper_delta[cell];
  carrier_upper_touched.push_back(cell);
}

void TAPFPlanner::carrier_upper_sub(int cell)
{
  --carrier_upper_delta[cell];  // touched entry stays; reset zeroes it
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
  const auto& phys = carrier->phys_view(S);
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

// physical cost of one joint op (design 2.3; solver weights)
static double carrier_ops_cost(const TAPFPlanner::Weights& w,
                               const ShelfState& from,
                               const std::vector<Op>& ops)
{
  double c = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
    if (ops[i].kind == Op::MOVE) {
      c += from.kappa[i] == KAPPA_FREE ? w.beta : w.alpha;
      if (from.kappa[i] == KAPPA_ANON) c += w.delta;
    } else if (ops[i].kind == Op::LIFT || ops[i].kind == Op::DROP) {
      c += w.gamma;
    }
  }
  return c;
}

TAPFPlanner::CarrierRollout TAPFPlanner::carrier_rollout(const Config& C0,
                                                         const ShelfState& S0,
                                                         int max_steps,
                                                         int min_chunk,
                                                         bool stop_on_event)
{
  CarrierRollout out;
  if (stats != nullptr) ++stats->rollout_calls;
  out.configs.push_back(C0);
  out.shelves.push_back(S0);
  std::unordered_set<uint64_t> local_seen;
  auto state_hash = [&](const Config& C, const ShelfState& S) {
    return (uint64_t)ConfigHasher()(C) ^ shelf_layer_hash(S);
  };
  local_seen.insert(state_hash(C0, S0));
  std::unique_ptr<TAPFNode> prev_node;
  auto C_step = Config(N, nullptr);
  for (int step = 0; step < max_steps; ++step) {
    const Config& cur = out.configs.back();
    const ShelfState& curS = out.shelves.back();
    if (is_goal_config(cur, curS)) {
      out.reached_goal = true;
      return out;
    }
    auto node = std::make_unique<TAPFNode>(cur, curS, D, ins,
                                           std::vector<int>(N, -1),
                                           TAPFAssignmentState(), nullptr);
    // probe nodes recycle heap addresses: address-keyed scratches MUST be
    // invalidated every step (pre-integration rollout lesson; guarded by
    // dd_integration.rollout_steps_match_fresh_generation)
    invalidate_carrier_scratch();
    if (step % GUIDANCE_REFRESH_STEPS == 0 || prev_node == nullptr ||
        prev_node->guide == nullptr) {
      attach_carrier_guidance(
          node.get(), false,
          prev_node != nullptr ? prev_node->guide.get() : nullptr);
    } else {
      node->guide = std::move(prev_node->guide);
      node->order = prev_node->order;
      node->constraint_order = node->order;
    }
    TAPFConstraint root;
    if (!get_new_config(node.get(), &root)) return out;   // stuck: honest
    if (!apply_carrier_effects(node.get())) return out;   // oracle reject
    for (auto a : A) C_step[a->id] = a->v_next;
    const auto ops = ops_scratch;  // copy (assembled by carrier effects)
    if (!local_seen.insert(state_hash(C_step, shelf_next_scratch)).second) {
      if (stats != nullptr) ++stats->rollout_cycles;
      return out;  // local cycle
    }
    bool shelf_motion = false;
    for (size_t i = 0; i < ops.size(); ++i)
      shelf_motion |=
          ops[i].kind == Op::MOVE && !curS.kappa.empty() &&
          curS.kappa[i] != KAPPA_FREE;
    if (shelf_motion) {
      out.shelf_moved = true;
      if (stats != nullptr) ++stats->rollout_shelf_motion_steps;
    }
    out.cost += carrier_ops_cost(weights, curS, ops);
    out.ops.push_back(ops);
    out.configs.push_back(C_step);
    out.shelves.push_back(shelf_next_scratch);
    if (stats != nullptr) ++stats->macro_steps;
    if (out.configs.size() >= 3) {
      const size_t T = out.configs.size();
      note_lift_cycle(out.configs[T - 3], out.shelves[T - 3],
                      out.configs[T - 2], out.shelves[T - 2],
                      out.configs[T - 1], out.shelves[T - 1]);
    }
    prev_node = std::move(node);
    if (is_goal_config(out.configs.back(), out.shelves.back())) {
      out.reached_goal = true;
      return out;
    }
    if (stop_on_event && step >= min_chunk) {
      for (const Op& op : ops)
        if (op.kind == Op::LIFT || op.kind == Op::DROP) return out;
    }
  }
  return out;
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
