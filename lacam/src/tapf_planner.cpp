// Integrated TAPF planner: construction, high-level search, edges.
// Split from the original tapf_planner.cpp.




#include "tapf_planner_internal.hpp"

using namespace tapf_detail;
using namespace carrier_detail;

TAPFPlanner::~TAPFPlanner()
{
  for (auto* node : deferred_cleanup_nodes) delete node;
  for (auto a : A) delete a;
}

const TAPFReferenceCheckpoint*
TAPFPlanner::find_reference_checkpoint(
    const PhysConfig& state) const
{
  if (search_config.reference_plan == nullptr ||
      !reference_plan_valid)
    return nullptr;
  const uint64_t hash = phys_config_hash(state);
  const auto range = reference_checkpoint_index.equal_range(hash);
  const TAPFReferenceCheckpoint* best = nullptr;
  for (auto it = range.first; it != range.second; ++it) {
    const auto& checkpoint =
        search_config.reference_plan->checkpoints[it->second];
    if (!(checkpoint.state == state)) continue;
    if (best == nullptr ||
        checkpoint.suffix_cost < best->suffix_cost ||
        (checkpoint.suffix_cost == best->suffix_cost &&
         checkpoint.action_index > best->action_index))
      best = &checkpoint;
  }
  return best;
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
  if (search_config.stop_policy ==
          TAPFStopPolicy::FIRST_FEASIBLE &&
      search_config.incumbent_init.is_bounded())
    throw std::invalid_argument(
        "FIRST_FEASIBLE requires an unbounded external incumbent");
  if (search_config.stop_policy ==
          TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT &&
      !search_config.incumbent_init.is_bounded())
    throw std::invalid_argument(
        "FIRST_STRICT_IMPROVEMENT requires an external incumbent");
  if (search_config.reference_plan != nullptr) {
    reference_checkpoint_index.reserve(
        search_config.reference_plan->checkpoints.size() * 2);
    for (size_t i = 0;
         i < search_config.reference_plan->checkpoints.size(); ++i) {
      reference_checkpoint_index.emplace(
          search_config.reference_plan->checkpoints[i].state_hash, i);
    }
  }
  if (stats != nullptr) *stats = TAPFStats();
  if (stats != nullptr &&
      search_config.reference_plan != nullptr)
    ++stats->reference_plans_received;
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
    dd_view->shelf_storage = ins->shelf_storage;
    dd_view->target_starts = ins->target_starts;
    dd_view->target_goals = ins->target_goals;
    dd_view->target_goal_sets = ins->target_goal_sets;  // T1: eligibility
    dd_view->finalize();
    const size_t n_cells = ins->G.U.size();
    carrier_grounded.assign(n_cells, 0);
    carrier_upper_delta.assign(n_cells, 0);
    carrier = std::make_unique<CarrierEngine>(*dd_view);
  }
  if (search_config.initial_physical.has_value()) {
    if (dd_view == nullptr)
      throw std::invalid_argument(
          "carrier physical root requires a shelf layer");
    const auto validation = validate_phys_config_root(
        *dd_view, *search_config.initial_physical);
    if (!validation.valid())
      throw std::invalid_argument(
          "invalid carrier physical root");
  }
  if (search_config.event_contract != nullptr) {
    if (!search_config.initial_physical.has_value())
      throw std::invalid_argument(
          "event contract requires a carrier physical root");
    if (!(*search_config.initial_physical ==
          search_config.event_contract->start))
      throw std::invalid_argument(
          "event contract start does not match initial_physical");
    const auto validation = validate_carrier_event_contract(
        *dd_view, *search_config.event_contract);
    if (!validation.valid())
      throw std::invalid_argument(
          "invalid carrier event contract");
  }
  if (search_config.reference_plan != nullptr && dd_view != nullptr) {
    const auto& reference = *search_config.reference_plan;
    std::vector<PhysConfig> states;
    std::vector<PlanCost> steps;
    states.reserve(reference.actions.size() + 1);
    steps.reserve(reference.actions.size());
    states.push_back(
        search_config.initial_physical.has_value()
            ? *search_config.initial_physical
            : initial_phys_config(*dd_view));
    bool valid = true;
    for (const auto& ops : reference.actions) {
      const auto step = reference_joint_cost(
          weights, states.back(), ops, search_config.objective);
      const auto next = apply_ops(*dd_view, states.back(), ops);
      if (!step.has_value() || !next.has_value()) {
        valid = false;
        break;
      }
      steps.push_back(*step);
      states.push_back(*next);
    }
    valid = valid && is_dd_goal(*dd_view, states.back());
    if (valid) {
      std::vector<PlanCost> prefix(states.size());
      std::vector<PlanCost> suffix(states.size());
      for (size_t i = 0; i < steps.size(); ++i)
        prefix[i + 1] = prefix[i] + steps[i];
      for (size_t i = steps.size(); i > 0; --i)
        suffix[i - 1] = steps[i - 1] + suffix[i];
      for (const auto& checkpoint : reference.checkpoints) {
        const size_t index = checkpoint.action_index;
        const bool next_matches =
            index < reference.actions.size()
                ? checkpoint.next_ops == reference.actions[index]
                : checkpoint.next_ops.empty();
        if (index >= states.size() ||
            checkpoint.state_hash != phys_config_hash(states[index]) ||
            !(checkpoint.state == states[index]) ||
            checkpoint.prefix_cost != prefix[index] ||
            checkpoint.suffix_cost != suffix[index] ||
            !next_matches) {
          valid = false;
          break;
        }
      }
    }
    reference_plan_valid = valid;
    if (stats != nullptr && reference_plan_valid)
      ++stats->reference_plans_validated;
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
  auto current_bound = [&]() -> PlanCost {
    if (S_goal != nullptr) return S_goal->g;
    return search_config.incumbent_init;
  };

  auto select_open_index = [&]() -> size_t {
    if (search_config.mode == TAPFSearchMode::DFS ||
        (S_goal == nullptr &&
         !search_config.incumbent_init.is_bounded())) {
      return OPEN.size() - 1;
    }
    // shared FOCAL kernel (search_kernel.hpp) — same semantics as before
    return focal_select_index(
        OPEN, search_config.focal_weight,
        [&](const TAPFNode* n) {
          return focal_numeric_cost(n->f, search_config.objective);
        },
        [&](const TAPFNode* n) {
          const PlanCost b = current_bound();
          return !n->search_tree.empty() &&
                 (!b.is_bounded() || n->f < b);
        },
        [&](const TAPFNode* a, const TAPFNode* b) {
          return focal_better(a, b, search_config.focal_tie_break);
        });
  };

  auto initial_assignment_state = TAPFAssignmentState();
  initial_assignment_state.init(ins->N, ins->tasks.size());
  Config root_config = ins->starts;
  ShelfState root_shelf = initial_shelf_state(*ins);
  if (search_config.initial_physical.has_value()) {
    const auto& root = *search_config.initial_physical;
    root_config = config_of_physical(*ins, root);
    root_shelf = shelf_of_physical(root);
  }
  auto initial_agents = std::vector<int>(N, 0);
  std::iota(initial_agents.begin(), initial_agents.end(), 0);
  auto initial_assignment =
      assign_tapf_tasks_dynamic(*ins, D, root_config, initial_assignment_state,
                                initial_agents, true, &assignment_stats);
  if (!initial_assignment.feasible) return Solution();

  auto make_node_h = [&](const Config& config, double work_lb) {
    auto h = PlanCost::legacy(work_lb);
    if (search_config.objective ==
        TAPFObjective::MAKESPAN_THEN_WORK)
      h.ticks = get_time_h_value(config);
    return h;
  };

  auto S_init =
      new TAPFNode(root_config, root_shelf, D, ins,
                   initial_assignment.agent_to_task, initial_assignment_state);
  S_init->h = make_node_h(root_config, initial_assignment.cost);
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

  auto accept_goal = [&](TAPFNode* candidate) {
    const bool improves =
        candidate != nullptr &&
        (S_goal == nullptr || candidate->g < S_goal->g) &&
        (!search_config.incumbent_init.is_bounded() ||
         candidate->g < search_config.incumbent_init);
    if (!improves) return false;
    if (stats != nullptr) {
      ++stats->incumbent_updates;
      if (stats->first_solution_ticks < 0) {
        stats->first_solution_cost =
            (unsigned)std::lround(candidate->g.work_value());
        stats->first_solution_g = candidate->g.work_value();
        stats->first_solution_ticks = candidate->g.ticks;
        stats->first_solution_work_scaled = candidate->g.work;
        stats->first_solution_work = candidate->g.work_value();
        stats->first_solution_time_ms = elapsed_ms(deadline);
      }
      ++stats->anytime_cost_updates;
    }
    S_goal = candidate;
    info(1, verbose, "elapsed:", elapsed_ms(deadline),
         "ms\tfound TAPF solution\tcost:", S_goal->g);
    return true;
  };

  auto try_reference_suffix = [&](TAPFNode* start) {
    if (search_config.stop_policy !=
            TAPFStopPolicy::FIRST_STRICT_IMPROVEMENT ||
        search_config.reference_plan == nullptr ||
        dd_view == nullptr || !reference_plan_valid)
      return false;
    const PhysConfig start_state =
        physical_state_of(start->C, start->shelf);
    const auto* checkpoint =
        find_reference_checkpoint(start_state);
    if (checkpoint == nullptr) return false;
    if (stats != nullptr) ++stats->reference_checkpoint_hits;
    const auto& reference = *search_config.reference_plan;
    if (checkpoint->action_index >= reference.actions.size())
      return false;
    const PlanCost bound = current_bound();
    if (!bound.is_bounded() ||
        !(start->g + checkpoint->suffix_cost < bound))
      return false;
    if (stats != nullptr) ++stats->reference_suffix_attempts;

    std::vector<TransitionStep> trace;
    trace.reserve(
        reference.actions.size() - checkpoint->action_index);
    PhysConfig state = start_state;
    PlanCost suffix_cost;
    for (size_t action = checkpoint->action_index;
         action < reference.actions.size(); ++action) {
      if (is_expired(deadline)) return false;
      const auto step_cost = reference_joint_cost(
          weights, state, reference.actions[action],
          search_config.objective);
      if (!step_cost.has_value()) return false;
      const auto next =
          apply_ops(*dd_view, state, reference.actions[action]);
      if (!next.has_value()) return false;
      suffix_cost += *step_cost;
      trace.push_back(TransitionStep{
          state, reference.actions[action], *next});
      state = *next;
    }
    if (trace.empty() ||
        suffix_cost != checkpoint->suffix_cost ||
        !(start->g + suffix_cost < bound) ||
        !is_dd_goal(*dd_view, state))
      return false;

    Config terminal_config = config_of_physical(*ins, state);
    ShelfState terminal_shelf = shelf_of_physical(state);
    if (!is_goal_config(terminal_config, terminal_shelf))
      return false;
    SearchKey terminal_key{terminal_config, terminal_shelf};
    TAPFNode* terminal = nullptr;
    const auto existing = CLOSED.find(terminal_key);
    if (existing == CLOSED.end()) {
      terminal = new TAPFNode(
          terminal_config, terminal_shelf, D, ins,
          start->assignment, start->assignment_state, start);
      terminal->g = start->g + suffix_cost;
      terminal->h = PlanCost();
      terminal->f = terminal->g;
      terminal->incoming_edge = register_outgoing_edge(
          start, terminal, suffix_cost, trace);
      CLOSED.emplace(std::move(terminal_key), terminal);
      if (stats != nullptr) ++stats->hl_nodes_created;
    } else {
      terminal = existing->second;
      register_outgoing_edge(
          start, terminal, suffix_cost, trace);
      rewrite(start, S_goal, OPEN);
    }
    if (!accept_goal(terminal)) return false;
    if (stats != nullptr) ++stats->reference_suffix_accepted;
    return true;
  };

  const auto initial_lower_bound = S_init->h;
  // The caller owns the one search/finalization split.  Do not subtract a
  // second cleanup reserve here: doing so turned a 10 s controller budget
  // into an 8.5 s pass deadline and then a 7.65 s bounded-search cutoff.
  while (!OPEN.empty() && !is_expired(deadline)) {
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

    if (S->guidance_stale) {
      ensure_guidance_fresh(S);
      if (stats != nullptr) ++stats->rewire_guidance_rebuilds;
    }

    {
      const PlanCost bound = current_bound();
      if (bound.is_bounded() && S->f >= bound) {
        if (stats != nullptr) ++stats->f_pruned;
        erase_open(open_index);
        continue;
      }
    }

    if (try_reference_suffix(S)) break;

    if (is_goal_config(S->C, S->shelf)) {
      accept_goal(S);
      if (search_config.stop_policy != TAPFStopPolicy::ANYTIME &&
          S_goal != nullptr)
        break;
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
        !search_config.incumbent_init.is_bounded() &&
        !S->macro_tried && S->h_guidance > 0 && ins->tasks.empty() &&
        (int)ins->target_starts.size() <= MACRO_TARGET_LIMIT) {
      S->macro_tried = true;
      auto r = carrier_rollout(S->C, S->shelf, MACRO_CAP, 0,
                               /*stop_on_event=*/true, S);
      // rollout probes died: their addresses may be recycled by the
      // nodes created below — stale address-keyed scratches are poison
      invalidate_carrier_scratch();
      if (r.ops.size() >= 2) {
        lookup_key.C = r.configs.back();
        lookup_key.S = r.shelves.back();
        std::vector<TransitionStep> trace;
        trace.reserve(r.ops.size());
        for (size_t step = 0; step < r.ops.size(); ++step)
          trace.push_back(TransitionStep{
              physical_state_of(r.configs[step], r.shelves[step]),
              r.ops[step],
              physical_state_of(r.configs[step + 1],
                                r.shelves[step + 1])});
        auto macro_iter = CLOSED.find(lookup_key);
        TAPFNode* S_macro = nullptr;
        if (macro_iter == CLOSED.end()) {
          S_macro = new TAPFNode(
              r.configs.back(), r.shelves.back(), D, ins,
              std::vector<int>(N, -1), S->assignment_state, S);
          S_macro->g = S->g + r.cost;
          S_macro->h = r.terminal_h;
          S_macro->f = S_macro->g + S_macro->h;
          S_macro->h_guidance = r.terminal_h_guidance;
          if (r.terminal_guidance != nullptr)
            S_macro->guide = std::make_unique<CarrierGuidance>(
                *r.terminal_guidance);
          S_macro->order = r.terminal_order;
          S_macro->constraint_order = S_macro->order;
          S_macro->incoming_edge =
              register_outgoing_edge(S, S_macro, r.cost, trace);
          CLOSED[SearchKey{S_macro->C, S_macro->shelf}] = S_macro;
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
        } else {
          S_macro = macro_iter->second;
          register_outgoing_edge(S, S_macro, r.cost, trace);
          rewrite(S, S_goal, OPEN);
          if (!S_macro->queued && !S_macro->search_tree.empty())
            push_open(S_macro);
          if (stats != nullptr) ++stats->hl_duplicate_configs;
        }
        continue;  // S stays queued; DFS tries the macro child first
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

    std::vector<Op> edge_ops;
    if (!S->shelf.kappa.empty()) {
      edge_ops = ops_scratch;
    } else {
      edge_ops.assign(N, Op::make_wait());
      for (const auto* agent : A)
        if (agent->v_next != agent->v_now)
          edge_ops[agent->id] =
              Op::make_move(agent->v_next->index);
    }
    const std::vector<TransitionStep> one_step_trace = {
        TransitionStep{
            physical_state_of(S->C, S->shelf), edge_ops,
            physical_state_of(C_new, shelf_next_scratch)}};

    lookup_key.C = C_new;
    lookup_key.S = shelf_next_scratch;
    auto iter = CLOSED.find(lookup_key);
    if (iter != CLOSED.end()) {
      auto S_known = iter->second;
      register_outgoing_edge(
          S, S_known, get_edge_cost(S, S_known), one_step_trace);
      rewrite(S, S_goal, OPEN);
      auto S_insert = S_known;
      if (MT != nullptr && get_random_float(MT) < restart_rate) {
        S_insert = S_init;
      }
      {
        const PlanCost b = current_bound();
        if ((!b.is_bounded() || S_insert->f < b) &&
            !S_insert->queued &&
            !S_insert->search_tree.empty()) {
          push_open(S_insert);
          if (stats != nullptr) ++stats->hl_reinsertions;
        }
      }
      if (stats != nullptr) ++stats->hl_duplicate_configs;
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
    const PlanCost edge_cost = get_edge_cost(S, S_new);
    S_new->g = S->g + edge_cost;
    S_new->h = make_node_h(C_new, assignment.cost);
    S_new->f = S_new->g + S_new->h;
    S_new->incoming_edge = register_outgoing_edge(
        S, S_new, edge_cost, one_step_trace);
    attach_carrier_guidance(
        S_new, &one_step_trace.front().previous_X, S->guide.get(),
        &edge_ops);
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
      const PlanCost b = current_bound();
      if (!b.is_bounded() || S_new->f < b) {
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
      // Expand the selected immutable incoming trace.  The endpoint is
      // already present above, so append intermediate next states in
      // reverse order; the final vector reverse restores time order.
      if (S->parent != nullptr && S->incoming_edge != nullptr) {
        const auto& trace = S->incoming_edge->transition_trace;
        if (trace.size() > 1)
          for (size_t k = trace.size() - 1; k-- > 0;) {
            solution.push_back(
                config_of_physical(*ins, trace[k].next_X));
            solution_shelves.push_back(
                shelf_of_physical(trace[k].next_X));
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
      stats->solution_cost =
          (unsigned)std::lround(S_goal->g.work_value());
      PlanCost parent_edge_cost;
      for (size_t step = 1; step < solution_nodes.size(); ++step) {
        parent_edge_cost +=
            get_edge_cost(solution_nodes[step - 1], solution_nodes[step]);
      }
      stats->solution_parent_edge_cost =
          (unsigned)std::lround(parent_edge_cost.work_value());
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

  if (is_expired(deadline)) {
    for (auto p : CLOSED) p.second->discard_search_tree();
  }

  // best-effort chain to the deepest node (carrier debug aid, M16)
  best_effort_solution.clear();
  best_effort_shelves.clear();
  best_effort_tau.clear();
  if (S_goal == nullptr && deepest_node != nullptr &&
      !ins->shelf_cells.empty()) {
    if (deepest_node->guide != nullptr &&
        deepest_node->guide->upper_epoch != nullptr)
      best_effort_tau =
          deepest_node->guide->upper_epoch->tau_guide;
    for (const TAPFNode* S2 = deepest_node; S2 != nullptr; S2 = S2->parent) {
      best_effort_solution.push_back(S2->C);
      best_effort_shelves.push_back(S2->shelf);
      if (S2->parent != nullptr && S2->incoming_edge != nullptr) {
        const auto& trace = S2->incoming_edge->transition_trace;
        if (trace.size() > 1)
          for (size_t k = trace.size() - 1; k-- > 0;) {
            best_effort_solution.push_back(
                config_of_physical(*ins, trace[k].next_X));
            best_effort_shelves.push_back(
                shelf_of_physical(trace[k].next_X));
          }
      }
    }
    std::reverse(best_effort_solution.begin(), best_effort_solution.end());
    std::reverse(best_effort_shelves.begin(), best_effort_shelves.end());
  }

  if (search_config.defer_cleanup) {
    deferred_cleanup_nodes.reserve(deferred_cleanup_nodes.size() +
                                   CLOSED.size());
    for (auto p : CLOSED) deferred_cleanup_nodes.push_back(p.second);
  } else {
    for (auto p : CLOSED) delete p.second;
  }
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
    for (const auto& edge : node_from->outgoing_edges) {
      if (edge == nullptr || edge->to == nullptr) continue;
      auto node_to = edge->to;
      const auto g = node_from->g + edge->physical_cost;
      if (g < node_to->g) {
        node_to->parent = node_from;
        node_to->incoming_edge = edge;
        node_to->g = g;
        node_to->f = node_to->g + node_to->h;
        node_to->guidance_stale = true;
        Q.push(node_to);
        if (stats != nullptr) {
          ++stats->anytime_cost_updates;
          ++stats->g_relaxed;
        }
        const PlanCost bound =
            goal != nullptr ? goal->g : search_config.incumbent_init;
        if ((!bound.is_bounded() || node_to->f < bound) &&
            !node_to->queued && !node_to->search_tree.empty()) {
          OPEN.push_back(node_to);
          node_to->queued = true;
        }
      }
    }
  }
}

SearchEdgeHandle TAPFPlanner::register_outgoing_edge(
    TAPFNode* from, TAPFNode* to, PlanCost physical_cost,
    const std::vector<TransitionStep>& transition_trace)
{
  if (from == nullptr || to == nullptr)
    throw std::invalid_argument(
        "register_outgoing_edge: null endpoint");
  if (!transition_trace.empty()) {
    const PhysConfig from_state =
        physical_state_of(from->C, from->shelf);
    const PhysConfig to_state =
        physical_state_of(to->C, to->shelf);
    if (!(transition_trace.front().previous_X == from_state) ||
        !(transition_trace.back().next_X == to_state))
      throw std::logic_error(
          "register_outgoing_edge: trace endpoint mismatch");
    for (size_t step = 0; step < transition_trace.size(); ++step) {
      if (step > 0 &&
          !(transition_trace[step - 1].next_X ==
            transition_trace[step].previous_X))
        throw std::logic_error(
            "register_outgoing_edge: discontinuous trace");
      if (dd_view != nullptr) {
        const auto replayed = apply_ops(
            *dd_view, transition_trace[step].previous_X,
            transition_trace[step].ops);
        if (!replayed.has_value() ||
            !(*replayed == transition_trace[step].next_X))
          throw std::logic_error(
              "register_outgoing_edge: unreplayable transition");
      }
    }
  }
  for (const auto& edge : from->outgoing_edges)
    if (edge != nullptr && edge->to == to &&
        edge->physical_cost == physical_cost &&
        edge->transition_trace == transition_trace)
      return edge;

  auto edge = std::make_shared<SearchEdge>();
  edge->to = to;
  edge->physical_cost = physical_cost;
  edge->transition_trace = transition_trace;
  SearchEdgeHandle handle = edge;
  from->outgoing_edges.push_back(handle);
  std::stable_sort(
      from->outgoing_edges.begin(), from->outgoing_edges.end(),
      [](const SearchEdgeHandle& a, const SearchEdgeHandle& b) {
        if (a->to != b->to)
          return std::less<const TAPFNode*>()(a->to, b->to);
        if (a->physical_cost != b->physical_cost)
          return a->physical_cost < b->physical_cost;
        return a->transition_trace.size() <
               b->transition_trace.size();
      });
  return handle;
}

PlanCost TAPFPlanner::get_edge_cost(const TAPFNode* from,
                                    const TAPFNode* to) const
{
  if (to != nullptr && to->parent == from &&
      to->incoming_edge != nullptr)
    return to->incoming_edge->physical_cost;
  int64_t work = 0;
  for (size_t i = 0; i < ins->N; ++i) {
    const auto task = to->assignment[i];
    if (task < 0) continue;  // carrier agent: physical term below
    const auto goal = ins->tasks[task];
    if (from->C[i] != goal || to->C[i] != goal)
      add_scaled_work(work, PlanCost::WORK_SCALE);
  }
  // physical carrier term (design 2.3, mapping M5): loaded/free moves,
  // lift/drop, anonymous-carry moves.  kappa is empty on shelf-free
  // instances, so this loop is structurally skipped there.
  for (size_t i = 0; i < from->shelf.kappa.size(); ++i) {
    const int k_from = from->shelf.kappa[i];
    const int k_to = to->shelf.kappa[i];
    if (from->C[i] != to->C[i]) {  // MOVE (kappa preserved by moves)
      if (k_from == KAPPA_FREE) {
        if (to->assignment[i] < 0)
          add_scaled_work(work, weights.beta_scaled);
      } else {
        add_scaled_work(work, weights.alpha_scaled);
        if (k_from == KAPPA_ANON)
          add_scaled_work(work, weights.delta_scaled);
      }
    } else if (k_from != k_to) {  // LIFT or DROP (same cell)
      add_scaled_work(work, weights.gamma_scaled);
    }
  }
  const int64_t ticks =
      search_config.objective == TAPFObjective::MAKESPAN_THEN_WORK ? 1 : 0;
  return PlanCost::from_scaled(ticks, work);
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
