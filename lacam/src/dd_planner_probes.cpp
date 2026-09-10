// Test-support probes of the production machinery (protected suites).
// Split from the original dd_planner.cpp.

#include "dd_planner_internal.hpp"

using namespace dd_detail;

DDSocWeights dd_load_soc_weights()
{
  // R4: the ONE parser (carrier_detail::load_solver_weights) behind a
  // public face for tools; planner and reporting weights always agree.
  DDSocWeights w;
  carrier_detail::load_solver_weights(w);
  return w;
}

PlanCost dd_plan_cost_probe(const DDInstance& ins, const DDPlan& plan)
{
  return plan_cost(ins, plan);
}

PlanCost dd_plan_cost_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  return plan_cost(ins, root, plan);
}

std::optional<PlanCost> dd_plan_cost_deadline_probe(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline, bool* cutoff)
{
  return plan_cost_checked(ins, plan, deadline, cutoff);
}

bool dd_plan_cost_better_probe(const PlanCost& candidate,
                               const PlanCost& incumbent)
{
  return candidate < incumbent;
}

std::optional<DDPlan> dd_normalize_goal_prefix_probe(
    const DDInstance& ins, const DDPlan& plan)
{
  return normalize_goal_prefix(ins, plan);
}

std::optional<DDPlan> dd_normalize_goal_prefix_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  return normalize_goal_prefix(ins, root, plan);
}

std::optional<DDPlan> dd_normalize_goal_prefix_deadline_probe(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline, bool* cutoff)
{
  return normalize_goal_prefix(ins, plan, deadline, cutoff);
}

bool dd_replay_raw_prefix_deadline_probe(
    const DDInstance& ins, const DDPlan& plan,
    const Deadline* deadline, bool* cutoff)
{
  return replay_raw_prefix(ins, plan, deadline, cutoff).has_value();
}

std::optional<std::pair<PhysConfig, PlanCost>>
dd_replay_raw_prefix_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  return replay_raw_prefix(ins, root, plan);
}

std::optional<TAPFReferencePlan> dd_build_reference_plan_probe(
    const DDInstance& ins, const DDPlan& plan,
    size_t max_checkpoints)
{
  return build_reference_plan(ins, plan, max_checkpoints);
}

std::optional<TAPFReferencePlan> dd_build_reference_plan_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan, size_t max_checkpoints)
{
  return build_reference_plan(
      ins, root, plan, max_checkpoints);
}

std::optional<DDInstance> dd_fixed_goal_instance_from_plan_probe(
    const DDInstance& ins, const PhysConfig& root,
    const DDPlan& plan)
{
  return fixed_goal_instance_from_plan(ins, root, plan);
}

std::optional<TAPFReferenceCheckpoint> dd_reference_checkpoint_probe(
    const TAPFReferencePlan& reference, const PhysConfig& state)
{
  const auto* checkpoint =
      find_reference_checkpoint(reference, state);
  if (checkpoint == nullptr) return std::nullopt;
  return *checkpoint;
}

std::optional<DDPlan> dd_reference_splice_probe(
    const DDInstance& ins, const TAPFReferencePlan& reference,
    const DDPlan& prefix, const PlanCost& incumbent)
{
  const auto replayed = replay_raw_prefix(ins, prefix);
  if (!replayed.has_value()) return std::nullopt;
  const auto* checkpoint =
      find_reference_checkpoint(reference, replayed->first);
  if (checkpoint == nullptr ||
      checkpoint->action_index >= reference.actions.size())
    return std::nullopt;
  if (!(replayed->second + checkpoint->suffix_cost < incumbent))
    return std::nullopt;

  DDPlan stitched = prefix;
  stitched.insert(
      stitched.end(),
      reference.actions.begin() + checkpoint->action_index,
      reference.actions.end());
  const auto normalized = normalize_goal_prefix(ins, stitched);
  if (!normalized.has_value() ||
      !(plan_cost(ins, *normalized) < incumbent))
    return std::nullopt;
  return normalized;
}

UpperSignature dd_upper_signature_probe(const PhysConfig& X)
{
  return carrier_detail::make_upper_signature(X);
}

PairPlan dd_pair_cost_probe(const DDInstance& ins, const PhysConfig& X,
                            int target, int goal)
{
  const auto w = dd_load_soc_weights();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  return carrier_detail::pair_cost(
      ins, carrier_detail::make_upper_signature(X), target, goal, upper_wall,
      storage_topology, w.alpha, w.gamma, w.delta);
}

DDLazyTauProbe dd_lazy_tau_guide_probe(const DDInstance& ins,
                                       const PhysConfig& X)
{
  const auto w = dd_load_soc_weights();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto result = carrier_detail::build_lazy_pair_cost_assignment(
      ins, carrier_detail::make_upper_signature(X), upper_wall,
      storage_topology, w.alpha, w.gamma, w.delta);
  DDLazyTauProbe out;
  out.tau = result.tau;
  out.table = result.table;
  out.evaluated_edges = result.evaluated_edges;
  out.total_edges = result.total_edges;
  return out;
}

std::optional<TaskId> dd_pair_next_ready_effect_probe(
    const DDInstance& ins, const PhysConfig& X, int target, int goal,
    int recursion_cap)
{
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto abstract =
      carrier_detail::make_abstract_upper_state(ins, upper);
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  return carrier_detail::compile_single_root_next_ready_effect(
             ins, abstract, RootDemand{target, goal}, upper_wall,
             storage_topology,
             carrier_detail::TaskBRCompilerLimits{
                 recursion_cap, 512})
      .ready_effect;
}

double dd_pair_episode_cost_probe(
    const std::vector<ShelfSelector>& shifted_shelves, double alpha,
    double gamma, double delta)
{
  return carrier_detail::pair_episode_cost(
      shifted_shelves, alpha, gamma, delta);
}

std::vector<int> dd_tau_guide_probe(const DDInstance& ins,
                                    const PhysConfig& X)
{
  const auto w = dd_load_soc_weights();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto table = carrier_detail::build_pair_cost_table(
      ins, upper, upper_wall, storage_topology,
      w.alpha, w.gamma, w.delta);
  return carrier_detail::solve_tau_guide(ins, upper, table);
}

double dd_tau_lb_probe(const DDInstance& ins, const PhysConfig& X)
{
  const auto w = dd_load_soc_weights();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  return carrier_detail::solve_tau_lb(
      ins, X, upper_wall, w.alpha, w.gamma);
}

int64_t dd_makespan_lb_probe(const DDInstance& ins, const PhysConfig& X)
{
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache wall_distance(upper_grid);
  return carrier_detail::solve_tau_time_lb(
      ins, X, wall_distance);
}

ShelfTaskGraph dd_compile_single_root_graph_probe(
    const DDInstance& ins, const PhysConfig& X, int target, int goal,
    int recursion_cap, int backtrack_cap)
{
  const auto upper = carrier_detail::make_upper_signature(X);
  auto abstract =
      carrier_detail::make_abstract_upper_state(ins, upper);
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  std::vector<int> tau(ins.n_targets(), -1);
  std::vector<int> priority(ins.n_targets(), 0);
  if (target >= 0 && target < (int)ins.n_targets()) {
    tau[target] = goal;
    priority[target] = 1;
  }
  return carrier_detail::compile_task_br_pibt(
      ins, abstract, {RootDemand{target, goal}}, tau, priority, upper_wall,
      storage_topology,
      carrier_detail::TaskBRCompilerLimits{recursion_cap, backtrack_cap},
      true);
}

ShelfTaskGraph dd_compile_joint_graph_probe(
    const DDInstance& ins, const PhysConfig& X,
    const std::vector<int>* tau_override,
    const std::vector<int>* priority_override,
    int recursion_cap, int backtrack_cap)
{
  const auto w = dd_load_soc_weights();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto upper = carrier_detail::make_upper_signature(X);
  const auto table = carrier_detail::build_pair_cost_table(
      ins, upper, upper_wall, storage_topology,
      w.alpha, w.gamma, w.delta);
  const auto tau = tau_override != nullptr
                       ? *tau_override
                       : carrier_detail::solve_tau_guide(ins, upper, table);
  const auto priority =
      priority_override != nullptr
          ? *priority_override
          : carrier_detail::target_priorities_from_pair_cost(
                table, tau);
  std::vector<RootDemand> roots;
  for (size_t b = 0; b < ins.n_targets(); ++b)
    if (upper.target_pos[b] != tau[b])
      roots.push_back(RootDemand{(int)b, tau[b]});
  auto abstract =
      carrier_detail::make_abstract_upper_state(ins, upper);
  return carrier_detail::compile_task_br_pibt(
      ins, abstract, roots, tau, priority, upper_wall,
      storage_topology,
      carrier_detail::TaskBRCompilerLimits{recursion_cap, backtrack_cap},
      false);
}

std::vector<int> dd_ready_tasks_probe(const DDInstance& ins,
                                      const PhysConfig& X,
                                      const ShelfTaskGraph& graph)
{
  return carrier_detail::ready_tasks(ins, X, graph);
}

ShelfTaskGraph dd_propagate_root_demands_probe(
    ShelfTaskGraph graph, const std::vector<int>& target_priority)
{
  carrier_detail::propagate_root_demands(graph, target_priority);
  return graph;
}

bool dd_task_effects_conflict_probe(const TaskId& a, const TaskId& b)
{
  return carrier_detail::task_effects_conflict(a, b);
}

CarrierGuidance dd_task_br_guidance_probe(
    const DDInstance& ins, const PhysConfig& X,
    const PhysConfig* previous_X,
    const CarrierGuidance* previous_guidance,
    const std::vector<Op>* executed_ops)
{
  const auto weights = dd_load_soc_weights();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  return carrier_detail::build_task_br_guidance(
      ins, X, upper_wall, storage_topology,
      weights.alpha, weights.gamma, weights.delta,
      previous_X, previous_guidance, executed_ops);
}

CarrierGuidance dd_task_br_cached_guidance_probe(
    const DDInstance& ins, const PhysConfig& X,
    const std::vector<PhysConfig>& warmups, long* cache_hits)
{
  const auto weights = dd_load_soc_weights();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache upper_wall(upper_grid);
  const auto storage_topology =
      carrier_detail::build_storage_transfer_topology(ins);
  carrier_detail::UpperEpochCache cache;
  for (const auto& warmup : warmups)
    (void)carrier_detail::build_task_br_guidance(
        ins, warmup, upper_wall, storage_topology,
        weights.alpha, weights.gamma, weights.delta,
        nullptr, nullptr, nullptr, &cache);
  auto result = carrier_detail::build_task_br_guidance(
      ins, X, upper_wall, storage_topology,
      weights.alpha, weights.gamma, weights.delta,
      nullptr, nullptr, nullptr, &cache);
  if (cache_hits != nullptr) *cache_hits = cache.hits;
  return result;
}

DDReadyMatchProbe dd_match_ready_tasks_probe(
    const DDInstance& ins, const PhysConfig& X,
    const ShelfTaskGraph& graph, const std::vector<int>& ready_tasks,
    const std::vector<std::optional<TaskId>>* previous_rho_task_id,
    DispatchMode mode,
    const std::vector<std::optional<TransferKey>>*
        previous_rho_transfer_key,
    CandidateAdmission admission,
    const Deadline* deadline,
    const std::vector<uint8_t>* eligible_robot,
    const RhoIncrementalState* previous_rho_state)
{
  return carrier_detail::match_ready_tasks(
      ins, X, graph, ready_tasks, previous_rho_task_id,
      previous_rho_transfer_key, eligible_robot, mode, true,
      admission, deadline, previous_rho_state);
}

double dd_root_admissible_h(const DDInstance& ins)
{
  // Keep the admissible lower bound independent from tau_guide.
  const SocWeights w = soc_weights_from_env();
  const DDGrid upper_grid = make_upper_deck_grid(ins);
  DDDistCache uw(upper_grid);
  const auto X = initial_phys_config(ins);
  return carrier_detail::solve_tau_lb(
      ins, X, uw, w.alpha, w.gamma);
}

// ===================================================================
// TEST SUPPORT (protected suites): probes of the PRODUCTION machinery
// ===================================================================

namespace {

// drain one node's operator-constraint tree through the production
// expansion + generation (G1 conformance, debug.md P0-1/P0-2/P0-5 lineage)
std::vector<PhysConfig> drain_node(const TAPFInstance& view,
                                   const PhysConfig& X, int seed)
{
  std::mt19937 mt(seed);
  TAPFStats tstats;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &tstats);
  auto [C, S] = state_of(view, X);
  auto node = std::make_unique<TAPFNode>(C, S, planner.D, &view,
                                         std::vector<int>((int)view.N, -1),
                                         TAPFAssignmentState(), nullptr);
  planner.invalidate_carrier_scratch();
  planner.attach_carrier_guidance(node.get());

  std::vector<PhysConfig> out;
  std::unordered_set<uint64_t> seen;
  const int R = (int)view.N;
  std::vector<OpCand> cand;
  std::vector<TAPFConstraint*> popped;  // delete after the drain
  while (!node->search_tree.empty()) {
    auto M = node->search_tree.front();
    node->search_tree.pop();
    popped.push_back(M);
    if (M->depth < R) {
      const int i = node->constraint_order[M->depth];
      planner.build_op_candidates(node.get(), i, cand);
      lacam_expand_constraint_vec<TAPFConstraint>(M, i, cand,
                                                  node->search_tree);
    }
    if (!planner.get_new_config(node.get(), M)) continue;
    if (!planner.apply_carrier_effects(node.get())) continue;
    Config C_new(view.N, nullptr);
    for (auto a : planner.A) C_new[a->id] = a->v_next;
    auto nxt = phys_of(C_new, planner.shelf_next_scratch);
    if (seen.insert(phys_config_hash(nxt)).second) out.push_back(nxt);
  }
  for (auto* M : popped) delete M;
  return out;
}

}  // namespace

std::vector<PhysConfig> dd_enumerate_node_successors(const DDInstance& ins,
                                                     const PhysConfig& X,
                                                     int seed)
{
  const TAPFInstance view(ins);
  return drain_node(view, X, seed);
}

std::vector<Op> dd_root_joint_ops(const DDInstance& ins, const PhysConfig& X,
                                  int seed)
{
  const TAPFInstance view(ins);
  std::mt19937 mt(seed);
  TAPFStats tstats;
  TAPFPlanner planner(&view, nullptr, &mt, 0, 0, 0.001f, true, &tstats);
  auto [C, S] = state_of(view, X);
  auto node = std::make_unique<TAPFNode>(C, S, planner.D, &view,
                                         std::vector<int>((int)view.N, -1),
                                         TAPFAssignmentState(), nullptr);
  planner.attach_carrier_guidance(node.get());
  TAPFConstraint root;
  if (!planner.get_new_config(node.get(), &root)) return {};
  if (!planner.apply_carrier_effects(node.get())) return {};
  return planner.ops_scratch;
}
