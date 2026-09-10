// PIBT step generation, swap handling, forced-op feasibility.
// Split from the original tapf_planner.cpp.

#include "tapf_planner_internal.hpp"

using namespace tapf_detail;
using namespace carrier_detail;

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

int64_t TAPFPlanner::get_time_h_value(const Config& C)
{
  int64_t bound = 0;
  for (size_t i = 0; i < ins->N; ++i) {
    if (ins->allowed[i].empty()) continue;
    auto best = D.K;
    for (size_t j = 0; j < ins->tasks.size(); ++j) {
      if (!ins->allowed[i][j]) continue;
      best = std::min(best, D.get(j, C[i]));
    }
    if (best < D.K) bound = std::max<int64_t>(bound, best);
  }
  return bound;
}

bool TAPFPlanner::is_goal_config(const Config& C, const ShelfState& S) const
{
  if (search_config.event_contract != nullptr) {
    const PhysConfig physical = physical_state_of(C, S);
    bool newly_completed = false;
    for (const auto& fixed :
         search_config.event_contract->active_transfers) {
      const auto start_phase = carrier_event_phase_of(
          search_config.event_contract->start, fixed);
      const auto phase =
          carrier_event_phase_of(physical, fixed);
      if (phase.kind == CarrierTaskPhaseKind::INVALID)
        return false;
      if (start_phase.kind !=
              CarrierTaskPhaseKind::COMPLETED &&
          phase.kind == CarrierTaskPhaseKind::COMPLETED)
        newly_completed = true;
    }
    return newly_completed;
  }
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
      if (carrier_grounded[cell] != 0 &&
          carrier_grounded[cell] != CARRIER_GROUNDED_FIXED)
        out.push_back(OpCand{S->C[i], (uint8_t)Op::LIFT});
    } else {
      out.push_back(OpCand{S->C[i], (uint8_t)Op::DROP});
    }
  }

  const auto* checkpoint =
      find_reference_checkpoint(
          physical_state_of(S->C, S->shelf),
          S->absolute_tick);
  if (checkpoint != nullptr &&
      checkpoint->next_ops.size() == S->C.size()) {
    const Op& wanted = checkpoint->next_ops[i];
    const int wanted_cell =
        wanted.kind == Op::MOVE ? wanted.to : S->C[i]->index;
    const auto it = std::find_if(
        out.begin(), out.end(), [&](const OpCand& candidate) {
          return candidate.kind == wanted.kind &&
                 candidate.v->index == wanted_cell;
        });
    if (it != out.end()) {
      std::rotate(out.begin(), it, std::next(it));
      if (stats != nullptr) ++stats->reference_action_hints;
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

    if (!spacetime_candidate_feasible(
            A[i], M->where[k], kind))
      return false;

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
    std::vector<Agent*> neighbor_agents;
    neighbor_agents.reserve(K);

    for (auto u : ai->v_now->neighbor) {
      auto aj = occupied_now[u->id];
      if (aj != nullptr) neighbor_agents.push_back(aj);
    }

    C_next[i].resize(K + 1);
    for (size_t k = 0; k < K; ++k) {
      auto u = ai->v_now->neighbor[k];
      C_next[i][k] = u;
      if (MT != nullptr) tie_breakers[u->id] = get_random_float(MT);
    }
    C_next[i][K] = ai->v_now;
    if (MT != nullptr) tie_breakers[ai->v_now->id] = get_random_float(MT);

    auto get_hindrance = [&](Vertex* u) {
      auto count = 0u;
      for (auto* aj : neighbor_agents) {
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
    const int q = ai->v_now->index;

    auto append_candidate = [&](Vertex* vertex, uint8_t kind) {
      const auto duplicate =
          std::find_if(cand.begin(), cand.end(), [&](const auto& item) {
            return item.first == vertex && item.second == kind;
          });
      if (duplicate == cand.end()) cand.push_back({vertex, kind});
    };
    auto append_exact_move = [&](int cell) {
      for (auto* vertex : ai->v_now->neighbor)
        if (vertex->index == cell) {
          append_candidate(vertex, (uint8_t)Op::MOVE);
          return;
        }
    };
    auto append_all_moves = [&](auto&& score) {
      auto cells = std::vector<Vertex*>(ai->v_now->neighbor);
      std::stable_sort(cells.begin(), cells.end(),
                       [&](Vertex* a, Vertex* b) {
                         const auto score_a = score(a->index);
                         const auto score_b = score(b->index);
                         return score_a != score_b
                                    ? score_a < score_b
                                    : a->index < b->index;
                       });
      for (auto* vertex : cells)
        append_candidate(vertex, (uint8_t)Op::MOVE);
    };

    if (search_config.event_contract != nullptr) {
      const auto* fixed = [&]() -> const FixedRobotTransfer* {
        for (const auto& candidate :
             search_config.event_contract->active_transfers)
          if (candidate.robot == i) return &candidate;
        return nullptr;
      }();
      if (fixed == nullptr) {
        append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        append_all_moves(
            [](int cell) { return std::make_pair(0, cell); });
      } else {
        const auto phase = carrier_event_phase_of(
            eng.phys_view(carrier_scratch_node), *fixed);
        if (phase.kind == CarrierTaskPhaseKind::APPROACH) {
          if (q == fixed->transfer.route.front())
            append_candidate(
                ai->v_now, (uint8_t)Op::LIFT);
          append_all_moves([&](int cell) {
            return std::make_pair(
                eng.lower.dist(
                    fixed->transfer.route.front(), cell),
                cell);
          });
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        } else if (
            phase.kind == CarrierTaskPhaseKind::CARRYING) {
          const int last =
              (int)fixed->transfer.route.size() - 1;
          if (phase.route_index == last) {
            append_candidate(
                ai->v_now, (uint8_t)Op::DROP);
          } else if (
              phase.route_index >= 0 &&
              phase.route_index < last) {
            append_exact_move(
                fixed->transfer.route[
                    phase.route_index + 1]);
          }
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        } else {
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        }
      }
    } else if (loaded) {
      const Custody* custody = nullptr;
      if (guide != nullptr &&
          i < (int)guide->custody_by_robot.size() &&
          guide->custody_by_robot[i].has_value())
        custody = &*guide->custody_by_robot[i];
      if (custody != nullptr && custody->from == q) {
        const bool at_endpoint =
            q == custody_endpoint(*custody) &&
            custody->route_status == RouteStatus::ARRIVED;
        if (at_endpoint &&
            dd_view->can_place_movable_shelf(q))
          append_candidate(ai->v_now, (uint8_t)Op::DROP);
        // The timed router normally keeps carried shelves in aisles.  When
        // another loaded route claims this aisle cell and our own hint would
        // retreat, expose an adjacent empty storage cell as a one-step
        // passing pocket before following that retreat.
        const TimedRouteHint* timed_hint = nullptr;
        if (guide != nullptr &&
            i < (int)guide->timed_transport.by_robot.size() &&
            guide->timed_transport.by_robot[i].has_value()) {
          const auto& candidate =
              *guide->timed_transport.by_robot[i];
          if ((candidate.status == RouteStatus::OK ||
               candidate.status == RouteStatus::PREFIX) &&
              candidate.endpoint == custody_endpoint(*custody) &&
              candidate.cells.size() >= 2 &&
              candidate.cells.front() == q)
            timed_hint = &candidate;
        }
        const Agent* pusher = occupied_next[ai->v_now->id];
        const bool recursively_displaced_by_loaded_carrier =
            pusher != nullptr && pusher != ai &&
            carrier_scratch_node != nullptr &&
            pusher->id <
                (int)carrier_scratch_node->shelf.kappa.size() &&
            carrier_scratch_node->shelf.kappa[pusher->id] !=
                KAPPA_FREE;
        bool planned_displacement_by_loaded_carrier = false;
        if (guide != nullptr &&
            carrier_scratch_node != nullptr) {
          for (size_t other = 0;
               other < carrier_scratch_node->shelf.kappa.size() &&
               other < guide->timed_transport.by_robot.size();
               ++other) {
            if (other == (size_t)i ||
                carrier_scratch_node->shelf.kappa[other] ==
                    KAPPA_FREE ||
                !guide->timed_transport.by_robot[other]
                     .has_value())
              continue;
            const auto& other_hint =
                *guide->timed_transport.by_robot[other];
            if ((other_hint.status == RouteStatus::OK ||
                 other_hint.status == RouteStatus::PREFIX) &&
                other_hint.cells.size() >= 2 &&
                other_hint.cells.front() ==
                    carrier_scratch_node->C[other]->index &&
                other_hint.cells[1] == q) {
              planned_displacement_by_loaded_carrier = true;
              break;
            }
          }
        }
        const bool timed_hint_retreats =
            timed_hint != nullptr &&
            timed_hint->cells[1] != q &&
            eng.lower.dist(
                custody_endpoint(*custody),
                timed_hint->cells[1]) >
                eng.lower.dist(
                    custody_endpoint(*custody), q);
        const bool needs_loaded_passing_pocket =
            !dd_view->can_store_shelf(q) &&
            (recursively_displaced_by_loaded_carrier ||
             (planned_displacement_by_loaded_carrier &&
              timed_hint_retreats));
        if (!at_endpoint && needs_loaded_passing_pocket) {
          std::vector<Vertex*> passing_pockets;
          for (auto* vertex : ai->v_now->neighbor) {
            if (!dd_view->can_store_shelf(vertex->index) ||
                occupied_now[vertex->id] != nullptr ||
                carrier_upper_taken(vertex->index))
              continue;
            passing_pockets.push_back(vertex);
          }
          std::sort(
              passing_pockets.begin(), passing_pockets.end(),
              [](const Vertex* a, const Vertex* b) {
                return a->index < b->index;
              });
          for (auto* vertex : passing_pockets)
            append_candidate(vertex, (uint8_t)Op::MOVE);
        }
        if (!at_endpoint && timed_hint != nullptr) {
          if (timed_hint->cells[1] == q)
            append_candidate(ai->v_now, (uint8_t)Op::WAIT);
          else
            append_exact_move(timed_hint->cells[1]);
        }
        if (!at_endpoint && custody->preferred_leg.has_value() &&
            custody->preferred_leg->from == q)
          append_exact_move(custody->preferred_leg->to);
        append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        if (!at_endpoint &&
            dd_view->can_place_movable_shelf(q))
          append_candidate(ai->v_now, (uint8_t)Op::DROP);
        append_all_moves([&](int cell) {
          return custody->preferred_leg.has_value() &&
                         cell == custody->preferred_leg->to
                     ? std::make_pair(0, cell)
                     : std::make_pair(1, cell);
        });
      } else {
        // Loaded-but-unbound is normally released in place.  A carrier on a
        // transit cell should already have transition-anchored recovery
        // custody.  If recovery cannot currently choose a legal storage
        // endpoint, prefer WAIT instead of greedily retargeting the shelf from
        // one aisle cell at a time.  Other legal moves remain in the operator
        // candidate set for completeness.
        const bool can_drop_here =
            dd_view->can_place_movable_shelf(q);
        if (can_drop_here)
          append_candidate(ai->v_now, (uint8_t)Op::DROP);
        const bool recursively_displaced =
            occupied_next[ai->v_now->id] != nullptr &&
            occupied_next[ai->v_now->id] != ai;
        if (!can_drop_here) {
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
          append_all_moves([&](int cell) {
            return std::make_pair(
                dd_view->can_place_movable_shelf(cell) ? 0 : 1,
                cell);
          });
        } else {
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        }
        if (can_drop_here && !recursively_displaced)
          append_all_moves(
              [](int cell) { return std::make_pair(0, cell); });
      }
    } else {
      const TaskId* assigned = nullptr;
      if (guide != nullptr && i < (int)guide->rho_task_id.size() &&
          guide->rho_task_id[i].has_value())
        assigned = &*guide->rho_task_id[i];
      if (assigned != nullptr) {
        const bool preparing =
            guide != nullptr &&
            i < (int)guide->rho_mode.size() &&
            guide->rho_mode[i] == DispatchMode::PREPARE;
        const bool exact_shelf_here =
            assigned->from == q &&
            ((assigned->shelf.kind == ShelfSelector::Kind::TARGET &&
              carrier_grounded[q] == assigned->shelf.value + 1) ||
             (assigned->shelf.kind ==
                  ShelfSelector::Kind::ANON_AT_EPOCH_CELL &&
              assigned->shelf.value == q &&
              carrier_grounded[q] ==
                  CARRIER_GROUNDED_ANON));
        if (exact_shelf_here && !preparing)
          append_candidate(ai->v_now, (uint8_t)Op::LIFT);
        if (exact_shelf_here && preparing)
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        append_all_moves([&](int cell) {
          return std::make_pair(
              eng.lower.dist(assigned->from, cell), cell);
        });
        append_candidate(ai->v_now, (uint8_t)Op::WAIT);
      } else {
        auto footprint = [&](int cell) {
          if (guide == nullptr || guide->upper_epoch == nullptr) return 0;
          for (const int index : guide->ready_tasks) {
            if (index < 0 ||
                index >=
                    (int)guide->upper_epoch->task_graph.tasks.size())
              continue;
            const auto& id =
                guide->upper_epoch->task_graph.tasks[index].id;
            if (id.from == cell || id.to == cell) return 1;
          }
          for (const auto& item : guide->custody_by_robot)
            if (item.has_value() &&
                (item->from == cell || item->to == cell))
              return 1;
          return 0;
        };
        if (footprint(q) != 0) {
          append_all_moves([&](int cell) {
            return std::make_pair(footprint(cell), cell);
          });
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
        } else {
          append_candidate(ai->v_now, (uint8_t)Op::WAIT);
          append_all_moves([&](int cell) {
            return std::make_pair(footprint(cell), cell);
          });
        }
      }
    }

  }

  if (carrier_scratch_node != nullptr) {
    const auto* checkpoint = find_reference_checkpoint(
        physical_state_of(
            carrier_scratch_node->C,
            carrier_scratch_node->shelf),
        carrier_scratch_node->absolute_tick);
    if (checkpoint != nullptr &&
        checkpoint->next_ops.size() ==
            carrier_scratch_node->C.size()) {
      const Op& wanted = checkpoint->next_ops[i];
      const int wanted_cell =
          wanted.kind == Op::MOVE ? wanted.to : ai->v_now->index;
      const auto it = std::find_if(
          cand.begin(), cand.end(), [&](const auto& candidate) {
            return candidate.second == wanted.kind &&
                   candidate.first->index == wanted_cell;
          });
      if (it != cand.end()) {
        std::rotate(cand.begin(), it, std::next(it));
        if (stats != nullptr) ++stats->reference_action_hints;
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
    if (!spacetime_candidate_feasible(ai, u, kind))
      continue;
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // carrier feasibility (M4); none of these fire for task agents
    if (kind == Op::MOVE && loaded && carrier_upper_taken(u->index))
      continue;  // S1
    if (kind == Op::LIFT &&
        (carrier_grounded[u->index] == 0 ||
         carrier_grounded[u->index] ==
             CARRIER_GROUNDED_FIXED))
      continue;
    if (kind == Op::DROP &&
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
        occupied_next[ai->v_now->id] == nullptr &&
        spacetime_candidate_feasible(
            swap_agent, ai->v_now, (uint8_t)Op::MOVE)) {
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

  const auto can_move_into_pusher_origin = [&](const Agent* candidate) {
    return std::find(
               candidate->v_now->neighbor.begin(),
               candidate->v_now->neighbor.end(),
               ai->v_now) != candidate->v_now->neighbor.end();
  };

  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr && assignment[aj->id] >= 0 &&
      can_move_into_pusher_origin(aj) &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now, assignment) &&
      is_swap_possible(aj->v_now, ai->v_now, assignment)) {
    return aj;
  }

  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (assignment[ak->id] < 0) continue;
    if (can_move_into_pusher_origin(ak) &&
        is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0], assignment) &&
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
      return kappa_i == KAPPA_FREE &&
             carrier_grounded[S->C[i]->index] != 0 &&
             carrier_grounded[S->C[i]->index] !=
                 CARRIER_GROUNDED_FIXED;
    case Op::DROP:
      if (kappa_i == KAPPA_FREE) return false;
      if (dd_view != nullptr &&
          !dd_view->can_place_movable_shelf(
              S->C[i]->index))
        return false;
      if (carrier_upper_taken(v->index))
        return false;  // another shelf occupies the cell at t+1
      return true;
    case Op::WAIT:
    default:
      return true;
  }
}

bool TAPFPlanner::apply_carrier_effects(const TAPFNode* S)
{
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
  if (S->shelf.kappa.empty()) {
    shelf_next_scratch = S->shelf;  // empty layer: carried over as-is
    if (search_config.spacetime_commitment == nullptr)
      return true;
    PhysConfig phys;
    phys.robots.reserve(S->C.size());
    for (const auto* vertex : S->C)
      phys.robots.push_back(vertex->index);
    phys.kappa.assign(S->C.size(), KAPPA_FREE);
    return apply_ops(
               *dd_view, phys, ops_scratch, true,
               search_config.spacetime_commitment,
               S->absolute_tick)
        .has_value();
  }
  // conformance oracle = final arbiter (design 6.4, M4)
  const auto& phys = carrier->phys_view(S);
  const auto nxt = apply_ops(
      *dd_view, phys, ops_scratch, true,
      search_config.spacetime_commitment,
      S->absolute_tick);
  if (!nxt.has_value()) return false;
  if (search_config.event_contract != nullptr &&
      !validate_carrier_event_transition(
          *dd_view, *search_config.event_contract,
          phys, ops_scratch, *nxt))
    return false;
  shelf_next_scratch.target_pos = nxt->target_pos;
  shelf_next_scratch.anon_occ = nxt->anon_occ;
  shelf_next_scratch.kappa = nxt->kappa;
  return true;
}
