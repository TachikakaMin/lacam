#include "../lacam/src/carrier_guidance.hpp"
#include <dd_planner.hpp>

#include <sstream>
#include <unordered_map>

#include "gtest/gtest.h"

namespace {

TaskId target_effect(int target, int from, int to)
{
  return TaskId{
      ShelfSelector{ShelfSelector::Kind::TARGET, target},
      from,
      to,
  };
}

}  // namespace

TEST(dd_task_br_audit,
     equal_root_success_prefers_the_smaller_joint_task_graph)
{
  carrier_detail::JointCompileCandidate best;
  best.valid = true;
  best.success = {1, 1};
  best.estimated_shelf_cost = 2;
  best.state.graph.tasks = {
      ShelfTask{target_effect(0, 0, 1), {}, 2},
      ShelfTask{target_effect(1, 2, 3), {}, 1},
  };

  carrier_detail::JointCompileCandidate candidate;
  candidate.valid = true;
  candidate.success = {1, 1};
  candidate.estimated_shelf_cost = 1;
  candidate.state.graph.tasks = {
      ShelfTask{target_effect(0, 0, 1), {}, 2},
  };

  EXPECT_TRUE(
      carrier_detail::better_joint_candidate(candidate, best));
  EXPECT_FALSE(
      carrier_detail::better_joint_candidate(best, candidate));
}

TEST(dd_task_br_audit,
     exact_candidate_tie_keeps_stable_effect_enumeration_order)
{
  carrier_detail::JointCompileCandidate first;
  first.valid = true;
  first.success = {1};
  first.estimated_shelf_cost = 3;
  first.stable_order = 4;
  first.state.graph.tasks = {
      ShelfTask{target_effect(0, 1, 2), {}, 1},
  };

  carrier_detail::JointCompileCandidate later;
  later.valid = true;
  later.success = {1};
  later.estimated_shelf_cost = 3;
  later.stable_order = 5;
  later.state.graph.tasks = {
      ShelfTask{target_effect(0, 1, 0), {}, 1},
  };

  EXPECT_TRUE(
      carrier_detail::better_joint_candidate(first, later));
  EXPECT_FALSE(
      carrier_detail::better_joint_candidate(later, first));
}

TEST(dd_task_br_audit,
     equal_task_count_prefers_lower_estimated_shelf_cost)
{
  carrier_detail::JointCompileCandidate best;
  best.valid = true;
  best.success = {1};
  best.estimated_shelf_cost = 2;
  best.state.graph.tasks = {
      ShelfTask{target_effect(0, 1, 0), {}, 1},
  };

  carrier_detail::JointCompileCandidate candidate;
  candidate.valid = true;
  candidate.success = {1};
  candidate.estimated_shelf_cost = 0;
  candidate.state.graph.tasks = {
      ShelfTask{target_effect(0, 1, 2), {}, 1},
  };

  EXPECT_TRUE(
      carrier_detail::better_joint_candidate(candidate, best));
  EXPECT_FALSE(
      carrier_detail::better_joint_candidate(best, candidate));
}

TEST(dd_task_br_audit,
     joint_compiler_keeps_the_toward_goal_option_on_equal_task_count)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();

  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins));
  ASSERT_EQ(graph.tasks.size(), 1);
  EXPECT_EQ(
      graph.tasks.front().id,
      target_effect(
          0, ins.grid.idx(0, 1), ins.grid.idx(0, 2)));
}

TEST(dd_task_br_audit,
     joint_compiler_keeps_toward_goal_dependency_chain_on_cost_tie)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.robots = {
      ins.grid.idx(1, 0),
      ins.grid.idx(1, 4),
  };
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.finalize();

  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins));
  ASSERT_EQ(graph.tasks.size(), 2);
  const auto ready = dd_ready_tasks_probe(
      ins, initial_phys_config(ins), graph);
  ASSERT_EQ(ready.size(), 1);
  EXPECT_EQ(
      graph.tasks[ready.front()].id,
      (TaskId{
          ShelfSelector{
              ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
              ins.grid.idx(0, 1)},
          ins.grid.idx(0, 1),
          ins.grid.idx(1, 1)}));
}

TEST(dd_task_br_audit,
     forward_dependency_chain_beats_immediate_root_reversal)
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 4),
  };
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 5)};
  ins.finalize();

  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins));
  const auto expected =
      target_effect(
          0, ins.grid.idx(0, 1), ins.grid.idx(0, 2));
  EXPECT_NE(
      std::find_if(
          graph.tasks.begin(), graph.tasks.end(),
          [&](const ShelfTask& task) {
            return task.id == expected;
          }),
      graph.tasks.end());
}

TEST(dd_task_br_audit,
     anonymous_blocker_avoids_assigned_goal_before_cell_id_tie)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  const UpperSignature upper{
      {ins.grid.idx(0, 2)},
      {ins.grid.idx(0, 1)},
  };
  const ShelfSelector anonymous{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
      ins.grid.idx(0, 1),
  };
  const RootDemand root{0, ins.grid.idx(0, 0)};
  const std::vector<int> tau = {ins.grid.idx(0, 0)};
  DDDistCache upper_wall(ins.grid);
  const std::map<int, TaskId> reservations;

  const auto candidates =
      carrier_detail::ordered_shelf_candidates(
          ins, carrier_detail::make_abstract_upper_state(
                   ins, upper),
          anonymous, root, &tau, false, upper_wall,
          reservations);
  ASSERT_EQ(candidates.size(), 2);
  EXPECT_EQ(candidates.front(), ins.grid.idx(0, 2));
}

TEST(dd_task_br_audit,
     blocker_clearing_rollout_reaches_goal_without_upper_cycle)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.robots = {
      ins.grid.idx(1, 0),
      ins.grid.idx(1, 4),
  };
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.finalize();
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);

  const auto rollout = planner.carrier_rollout(
      view.starts, initial_shelf_state(view), 64, 0, false);
  std::ostringstream trace;
  for (size_t step = 0; step < rollout.configs.size(); ++step) {
    trace << "\n" << step << ": target="
          << rollout.shelves[step].target_pos[0]
          << " anon=";
    for (const int cell : rollout.shelves[step].anon_occ)
      trace << cell << ",";
    trace << " robots=";
    for (const auto* vertex : rollout.configs[step])
      trace << vertex->index << ",";
    trace << " kappa=";
    for (const int shelf : rollout.shelves[step].kappa)
      trace << shelf << ",";
    if (step < rollout.ops.size()) {
      trace << " ops=";
      for (const auto& op : rollout.ops[step])
        trace << (int)op.kind << "@" << op.to << ",";
    }
  }
  EXPECT_TRUE(rollout.reached_goal) << trace.str();
}

TEST(dd_task_br_audit,
     roomy_loaded_move_keeps_reverse_ready_without_auto_binding)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.finalize();

  const TaskId completed{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 2)},
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 3),
  };
  const TaskId reverse{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 3)},
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
  };
  ShelfTaskGraph current_graph;
  current_graph.tasks = {
      ShelfTask{
          reverse,
          {RootDemand{0, ins.grid.idx(0, 1)}},
          1,
      },
  };
  current_graph.predecessors = {{}};
  current_graph.successors = {{}};
  const PhysConfig previous{
      {ins.grid.idx(0, 2)},
      {ins.grid.idx(0, 0)},
      {},
      {KAPPA_ANON},
  };
  const PhysConfig current{
      {ins.grid.idx(0, 3)},
      {ins.grid.idx(0, 0)},
      {},
      {KAPPA_ANON},
  };
  EXPECT_FALSE(
      carrier_detail::target_dense_upper_layout(
          ins, carrier_detail::make_upper_signature(current)));

  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->task_graph.tasks = {
      ShelfTask{
          completed,
          {RootDemand{0, ins.grid.idx(0, 1)}},
          1,
      },
  };
  previous_epoch->task_graph.predecessors = {{}};
  previous_epoch->task_graph.successors = {{}};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      carrier_detail::make_custody(
          previous_epoch->task_graph.tasks[0], 0),
  };
  const std::vector<Op> move = {
      Op::make_move(ins.grid.idx(0, 3)),
  };
  const auto recovered =
      carrier_detail::recover_task_br_custody(
          ins, current, current_graph, &previous,
          &previous_guidance, &move);
  auto custody = recovered.custody_by_robot;
  const auto ready =
      carrier_detail::ready_tasks_with_custody(
          ins, current, current_graph, custody,
          recovered.continuation_carrier);
  ASSERT_EQ(ready, (std::vector<int>{0}));
  carrier_detail::bind_ready_continuations(
      ins, current, current_graph, ready,
      recovered.continuation_carrier,
      recovered.previous_loaded_move_from, custody);
  EXPECT_FALSE(custody[0].has_value());

  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const Config config = {
      view.G.U[current.robots[0]],
  };
  const ShelfState shelf{
      current.target_pos, current.anon_occ, current.kappa};
  auto node = std::make_unique<TAPFNode>(
      config, shelf, planner.D, &view,
      std::vector<int>{-1}, TAPFAssignmentState(), nullptr);
  auto current_epoch =
      std::make_shared<UpperEpochGuidance>();
  current_epoch->upper_signature =
      carrier_detail::make_upper_signature(current);
  current_epoch->task_graph = current_graph;
  node->guide = std::make_unique<CarrierGuidance>();
  node->guide->upper_epoch = current_epoch;
  node->guide->ready_tasks = ready;
  node->guide->custody_by_robot = custody;
  node->order = {0};
  node->constraint_order = {0};

  TAPFConstraint root;
  TAPFConstraint forced(
      &root, 0,
      OpCand{
          view.G.U[ins.grid.idx(0, 2)],
          (uint8_t)Op::MOVE});
  ASSERT_TRUE(planner.get_new_config(node.get(), &forced));
  ASSERT_TRUE(planner.apply_carrier_effects(node.get()));
  ASSERT_EQ(planner.ops_scratch.size(), 1u);
  EXPECT_EQ(
      planner.ops_scratch[0],
      Op::make_move(ins.grid.idx(0, 2)));
}

TEST(dd_task_br_audit,
     dense_loaded_move_may_bind_its_exact_reverse)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 2)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_goals = {
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 0),
  };
  ins.finalize();

  const TaskId completed{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 2)},
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 3),
  };
  const TaskId reverse{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 3)},
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
  };
  ShelfTaskGraph current_graph;
  current_graph.tasks = {
      ShelfTask{
          reverse,
          {RootDemand{0, ins.grid.idx(0, 1)}},
          2,
      },
  };
  current_graph.predecessors = {{}};
  current_graph.successors = {{}};
  const PhysConfig previous{
      {ins.grid.idx(0, 2)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 1),
      },
      {},
      {KAPPA_ANON},
  };
  const PhysConfig current{
      {ins.grid.idx(0, 3)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 1),
      },
      {},
      {KAPPA_ANON},
  };
  EXPECT_TRUE(
      carrier_detail::target_dense_upper_layout(
          ins, carrier_detail::make_upper_signature(current)));

  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->task_graph.tasks = {
      ShelfTask{
          completed,
          {RootDemand{0, ins.grid.idx(0, 1)}},
          2,
      },
  };
  previous_epoch->task_graph.predecessors = {{}};
  previous_epoch->task_graph.successors = {{}};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      carrier_detail::make_custody(
          previous_epoch->task_graph.tasks[0], 0),
  };
  const std::vector<Op> move = {
      Op::make_move(ins.grid.idx(0, 3)),
  };
  const auto recovered =
      carrier_detail::recover_task_br_custody(
          ins, current, current_graph, &previous,
          &previous_guidance, &move);
  auto custody = recovered.custody_by_robot;
  const auto ready =
      carrier_detail::ready_tasks_with_custody(
          ins, current, current_graph, custody,
          recovered.continuation_carrier);
  ASSERT_EQ(ready, (std::vector<int>{0}));
  carrier_detail::bind_ready_continuations(
      ins, current, current_graph, ready,
      recovered.continuation_carrier,
      recovered.previous_loaded_move_from, custody);
  ASSERT_TRUE(custody[0].has_value());
  EXPECT_EQ(custody[0]->task_id, reverse);
}

TEST(dd_task_br_audit,
     assigned_free_robot_yields_until_loaded_unbound_robot_drops)
{
  DDInstance ins;
  ins.grid = DDGrid({".....", "....."});
  ins.robots = {
      ins.grid.idx(1, 0),
      ins.grid.idx(1, 4),
  };
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 4)};
  ins.finalize();
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFPlanner planner(&view, nullptr, &mt);
  const TaskId ready_id =
      target_effect(
          0, ins.grid.idx(0, 1), ins.grid.idx(0, 2));
  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->task_graph.tasks = {
      ShelfTask{ready_id, {RootDemand{0, ins.grid.idx(0, 4)}}, 1},
  };
  epoch->task_graph.predecessors = {{}};
  epoch->task_graph.successors = {{}};
  CarrierGuidance guidance;
  guidance.upper_epoch = epoch;
  guidance.ready_tasks = {0};
  guidance.rho_task_id = {std::nullopt, ready_id};
  guidance.rho_ready_index = {-1, 0};
  guidance.custody_by_robot.resize(2);
  const PhysConfig physical{
      {ins.grid.idx(0, 3), ins.grid.idx(0, 4)},
      {ins.grid.idx(0, 1)},
      {ins.grid.idx(1, 1)},
      {KAPPA_ANON, KAPPA_FREE},
  };
  Config config{
      view.G.U[physical.robots[0]],
      view.G.U[physical.robots[1]],
  };
  ShelfState shelf{
      physical.target_pos, physical.anon_occ, physical.kappa};
  auto node = std::make_unique<TAPFNode>(
      config, shelf, planner.D, &view, std::vector<int>{-1, -1},
      TAPFAssignmentState(), nullptr);
  node->guide =
      std::make_unique<CarrierGuidance>(guidance);
  carrier_detail::LowerDist lower(ins.grid);
  node->order = carrier_detail::task_br_robot_order(
      physical, guidance, lower);
  node->constraint_order = node->order;
  ASSERT_EQ(node->order, (std::vector<int>{1, 0}));

  TAPFConstraint root;
  ASSERT_TRUE(planner.get_new_config(node.get(), &root));
  ASSERT_TRUE(planner.apply_carrier_effects(node.get()));
  ASSERT_EQ(planner.ops_scratch.size(), 2);
  EXPECT_EQ(planner.ops_scratch[0], Op::make_drop());
  EXPECT_FALSE(
      planner.ops_scratch[1] ==
      Op::make_move(ins.grid.idx(0, 3)));
}

TEST(dd_task_br_audit,
     task_priority_precedes_loaded_free_phase_in_robot_order)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  carrier_detail::LowerDist lower(ins.grid);

  PhysConfig physical;
  physical.robots = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 2),
  };
  physical.kappa = {
      KAPPA_ANON,
      KAPPA_FREE,
  };

  auto epoch = std::make_shared<UpperEpochGuidance>();
  epoch->task_graph.tasks = {
      ShelfTask{
          target_effect(0, ins.grid.idx(0, 1),
                        ins.grid.idx(0, 2)),
          {RootDemand{0, ins.grid.idx(0, 2)}},
          9,
      },
  };
  CarrierGuidance guidance;
  guidance.upper_epoch = epoch;
  guidance.rho_task_id = {
      std::nullopt,
      epoch->task_graph.tasks[0].id,
  };
  guidance.rho_ready_index = {-1, 0};
  guidance.custody_by_robot.resize(2);

  EXPECT_EQ(
      carrier_detail::task_br_robot_order(
          physical, guidance, lower),
      (std::vector<int>{1, 0}));
}

TEST(dd_task_br_audit,
     failed_cycle_branches_do_not_leak_rotation_candidates)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();

  const auto graph = dd_compile_joint_graph_probe(
      ins, initial_phys_config(ins));
  EXPECT_TRUE(graph.rotations.empty());
}

TEST(dd_task_br_audit,
     upper_epoch_cache_evicts_the_least_recently_used_signature)
{
  carrier_detail::UpperEpochCache cache(2);
  const UpperSignature a{{0}, {}};
  const UpperSignature b{{1}, {}};
  const UpperSignature c{{2}, {}};
  auto epoch_a = std::make_shared<UpperEpochGuidance>();
  auto epoch_b = std::make_shared<UpperEpochGuidance>();
  auto epoch_c = std::make_shared<UpperEpochGuidance>();

  cache.insert(a, epoch_a);
  cache.insert(b, epoch_b);
  EXPECT_EQ(cache.lookup(a), epoch_a);
  cache.insert(c, epoch_c);

  EXPECT_EQ(cache.size(), 2);
  EXPECT_TRUE(cache.contains(a));
  EXPECT_FALSE(cache.contains(b));
  EXPECT_TRUE(cache.contains(c));
}

TEST(dd_task_br_audit,
     loaded_blocker_roots_commit_the_next_upper_epoch_priority)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {ins.grid.idx(0, 0)},
      {},
      {KAPPA_ANON},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId blocker_effect{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 1)},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {ins.grid.idx(0, 3)};
  previous_epoch->target_priority = {1};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          blocker_effect,
          std::nullopt,
          blocker_effect.shelf,
          blocker_effect.from,
          blocker_effect.to,
          {RootDemand{0, ins.grid.idx(0, 3)}},
          1,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->priority_commitment,
      (std::vector<int>{0}));
  ASSERT_EQ(guidance.upper_epoch->target_priority.size(), 1u);
  EXPECT_GT(guidance.upper_epoch->target_priority[0], 1);
}

TEST(dd_task_br_audit,
     shared_blocker_at_half_of_active_roots_commits_only_the_highest_root)
{
  DDInstance ins;
  ins.grid = DDGrid({"........"});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
      ins.grid.idx(0, 7),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
      ins.grid.idx(0, 7),
  };
  ins.target_goals = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 1),
  };
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 5),
          ins.grid.idx(0, 6),
          ins.grid.idx(0, 7),
      },
      {},
      {KAPPA_ANON},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId blocker_effect{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 1)},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 1),
  };
  previous_epoch->target_priority = {1, 4, 3, 2};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          blocker_effect,
          std::nullopt,
          blocker_effect.shelf,
          blocker_effect.from,
          blocker_effect.to,
          {
              RootDemand{0, ins.grid.idx(0, 4)},
              RootDemand{1, ins.grid.idx(0, 3)},
          },
          2,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->priority_commitment,
      (std::vector<int>{1}));
}

TEST(dd_task_br_audit,
     shared_blocker_for_strict_majority_commits_all_roots_in_priority_order)
{
  DDInstance ins;
  ins.grid = DDGrid({"......."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
  };
  ins.target_goals = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
  };
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 5),
          ins.grid.idx(0, 6),
      },
      {},
      {KAPPA_ANON},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId blocker_effect{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 1)},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
  };
  previous_epoch->target_priority = {1, 3, 2};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          blocker_effect,
          std::nullopt,
          blocker_effect.shelf,
          blocker_effect.from,
          blocker_effect.to,
          {
              RootDemand{0, ins.grid.idx(0, 4)},
              RootDemand{1, ins.grid.idx(0, 3)},
          },
          3,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->priority_commitment,
      (std::vector<int>{1, 0}));
  ASSERT_EQ(guidance.upper_epoch->target_priority.size(), 3u);
  EXPECT_GT(
      guidance.upper_epoch->target_priority[1],
      guidance.upper_epoch->target_priority[0]);
  EXPECT_GT(
      guidance.upper_epoch->target_priority[0],
      guidance.upper_epoch->target_priority[2]);
}

TEST(dd_task_br_audit,
     shared_blocker_under_upper_vacancy_pressure_commits_all_roots)
{
  DDInstance ins;
  ins.grid = DDGrid({".........."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
      ins.grid.idx(0, 7),
      ins.grid.idx(0, 8),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
      ins.grid.idx(0, 7),
      ins.grid.idx(0, 8),
  };
  ins.target_goals = {
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
      ins.grid.idx(0, 7),
  };
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 4),
          ins.grid.idx(0, 5),
          ins.grid.idx(0, 6),
          ins.grid.idx(0, 7),
          ins.grid.idx(0, 8),
      },
      {},
      {KAPPA_ANON},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId blocker_effect{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 1)},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = ins.target_goals;
  previous_epoch->target_priority = {1, 6, 5, 4, 3, 2};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          blocker_effect,
          std::nullopt,
          blocker_effect.shelf,
          blocker_effect.from,
          blocker_effect.to,
          {
              RootDemand{0, ins.target_goals[0]},
              RootDemand{1, ins.target_goals[1]},
          },
          6,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->priority_commitment,
      (std::vector<int>{1, 0}));
}

TEST(dd_task_br_audit,
     commitment_filter_drops_roots_whose_new_tau_changed)
{
  const std::vector<RootDemand> candidates = {
      RootDemand{0, 10},
      RootDemand{1, 11},
      RootDemand{2, 12},
  };
  const std::vector<int> new_tau = {10, 21, 12};

  EXPECT_EQ(
      carrier_detail::filter_priority_commitment_for_tau(
          candidates, new_tau),
      (std::vector<RootDemand>{
          RootDemand{0, 10},
          RootDemand{2, 12},
      }));
}

TEST(dd_task_br_audit,
     collective_commitment_renews_on_an_intersecting_shared_effect)
{
  DDInstance ins;
  ins.grid = DDGrid({"........"});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
      ins.grid.idx(0, 7),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 5),
      ins.grid.idx(0, 6),
      ins.grid.idx(0, 7),
  };
  ins.target_goals = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 1),
  };
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 5),
          ins.grid.idx(0, 6),
          ins.grid.idx(0, 7),
      },
      {},
      {KAPPA_ANON},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId blocker_effect{
      ShelfSelector{
          ShelfSelector::Kind::ANON_AT_EPOCH_CELL,
          ins.grid.idx(0, 1)},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 1),
  };
  previous_epoch->priority_commitment = {1, 0, 2};
  previous_epoch->target_priority = {6, 7, 5, 4};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          blocker_effect,
          std::nullopt,
          blocker_effect.shelf,
          blocker_effect.from,
          blocker_effect.to,
          {
              RootDemand{0, ins.grid.idx(0, 4)},
              RootDemand{1, ins.grid.idx(0, 3)},
          },
          7,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->priority_commitment,
      (std::vector<int>{1, 0}));
  ASSERT_EQ(guidance.upper_epoch->target_priority.size(), 4u);
  EXPECT_GT(
      guidance.upper_epoch->target_priority[1],
      guidance.upper_epoch->target_priority[0]);
  EXPECT_GT(
      guidance.upper_epoch->target_priority[0],
      guidance.upper_epoch->target_priority[2]);
}

TEST(dd_task_br_audit,
     tau_filter_runs_before_completed_group_selection)
{
  const TaskId stale_effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 3}, 30, 31};
  const TaskId valid_effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 4}, 40, 41};
  const std::vector<carrier_detail::PriorityCommitmentGroup> groups = {
      carrier_detail::PriorityCommitmentGroup{
          {
              carrier_detail::PriorityCommitmentRoot{
                  RootDemand{0, 10}, 9},
          },
          stale_effect,
          0,
      },
      carrier_detail::PriorityCommitmentGroup{
          {
              carrier_detail::PriorityCommitmentRoot{
                  RootDemand{1, 11}, 8},
          },
          valid_effect,
          1,
      },
  };

  EXPECT_EQ(
      carrier_detail::select_priority_commitment_for_epoch(
          groups, {20, 11}, 2, {}),
      (std::vector<RootDemand>{RootDemand{1, 11}}));
}

TEST(dd_task_br_audit,
     filtered_intersection_cannot_renew_collective_commitment)
{
  const TaskId effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 3}, 30, 31};
  const std::vector<carrier_detail::PriorityCommitmentGroup> groups = {
      carrier_detail::PriorityCommitmentGroup{
          {
              carrier_detail::PriorityCommitmentRoot{
                  RootDemand{0, 10}, 4},
              carrier_detail::PriorityCommitmentRoot{
                  RootDemand{1, 11}, 5},
          },
          effect,
          0,
      },
  };

  EXPECT_EQ(
      carrier_detail::select_priority_commitment_for_epoch(
          groups, {10, 21, 12}, 4, {1, 2}),
      (std::vector<RootDemand>{RootDemand{0, 10}}));
}

TEST(dd_task_br_audit,
     root_shelf_move_recomputes_priority_without_self_commitment)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {ins.grid.idx(0, 1)},
      {},
      {0},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId root_effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {ins.grid.idx(0, 3)};
  previous_epoch->target_priority = {1};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          root_effect,
          std::nullopt,
          root_effect.shelf,
          root_effect.from,
          root_effect.to,
          {RootDemand{0, ins.grid.idx(0, 3)}},
          1,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_TRUE(guidance.upper_epoch->priority_commitment.empty());
  ASSERT_EQ(guidance.upper_epoch->target_priority.size(), 1u);
  EXPECT_EQ(guidance.upper_epoch->target_priority[0], 1);
}

TEST(dd_task_br_audit,
     roomy_multi_goal_root_move_recomputes_without_priority_commitment)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goal_sets = {{
      ins.grid.idx(0, 3),
      ins.grid.idx(0, 4),
  }};
  ins.target_goals = {ins.grid.idx(0, 3)};
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {ins.grid.idx(0, 1)},
      {},
      {0},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId root_effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {ins.grid.idx(0, 3)};
  previous_epoch->target_priority = {1};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          root_effect,
          std::nullopt,
          root_effect.shelf,
          root_effect.from,
          root_effect.to,
          {RootDemand{0, ins.grid.idx(0, 3)}},
          1,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->tau_guide,
      (std::vector<int>{ins.grid.idx(0, 3)}));
  EXPECT_TRUE(guidance.upper_epoch->priority_commitment.empty());
}

TEST(dd_task_br_audit,
     dense_multi_goal_root_move_keeps_commitment_when_tau_stays_stable)
{
  DDInstance ins;
  ins.grid = DDGrid({"...."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 3),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 3),
  };
  ins.target_goal_sets = {
      {
          ins.grid.idx(0, 2),
          ins.grid.idx(0, 3),
      },
      {ins.grid.idx(0, 0)},
  };
  ins.target_goals = {
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 0),
  };
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 0)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 3),
      },
      {ins.grid.idx(0, 2)},
      {0},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 1)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());
  EXPECT_TRUE(
      carrier_detail::target_dense_upper_layout(
          ins, carrier_detail::make_upper_signature(*current)));

  const TaskId root_effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {
      ins.grid.idx(0, 2),
      ins.grid.idx(0, 0),
  };
  previous_epoch->target_priority = {2, 1};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          root_effect,
          std::nullopt,
          root_effect.shelf,
          root_effect.from,
          root_effect.to,
          {RootDemand{0, ins.grid.idx(0, 2)}},
          2,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->tau_guide,
      (std::vector<int>{
          ins.grid.idx(0, 2),
          ins.grid.idx(0, 0),
      }));
  EXPECT_EQ(
      guidance.upper_epoch->priority_commitment,
      (std::vector<int>{0}));
}

TEST(dd_task_br_audit,
     roomy_multi_goal_blocker_recomputes_without_priority_commitment)
{
  DDInstance ins;
  ins.grid = DDGrid({"......"});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_goal_sets = {
      {
          ins.grid.idx(0, 4),
          ins.grid.idx(0, 5),
      },
      {ins.grid.idx(0, 3)},
  };
  ins.target_goals = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
  };
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 1),
      },
      {},
      {1},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId blocker_effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 1},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
  };
  previous_epoch->target_priority = {2, 1};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          blocker_effect,
          std::nullopt,
          blocker_effect.shelf,
          blocker_effect.from,
          blocker_effect.to,
          {RootDemand{0, ins.grid.idx(0, 4)}},
          2,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  ASSERT_EQ(guidance.upper_epoch->tau_guide.size(), 2u);
  EXPECT_EQ(
      guidance.upper_epoch->tau_guide[0],
      ins.grid.idx(0, 4));
  EXPECT_TRUE(guidance.upper_epoch->priority_commitment.empty());
}

TEST(dd_task_br_audit,
     labeled_blocker_for_another_root_commits_that_root_mission)
{
  DDInstance ins;
  ins.grid = DDGrid({"....."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_starts = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.target_goals = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
  };
  ins.finalize();

  PhysConfig previous{
      {ins.grid.idx(0, 1)},
      {
          ins.grid.idx(0, 0),
          ins.grid.idx(0, 1),
      },
      {},
      {1},
  };
  const std::vector<Op> ops = {
      Op::make_move(ins.grid.idx(0, 2)),
  };
  const auto current = apply_ops(ins, previous, ops);
  ASSERT_TRUE(current.has_value());

  const TaskId blocker_effect{
      ShelfSelector{ShelfSelector::Kind::TARGET, 1},
      ins.grid.idx(0, 1),
      ins.grid.idx(0, 2),
  };
  auto previous_epoch =
      std::make_shared<UpperEpochGuidance>();
  previous_epoch->upper_signature =
      carrier_detail::make_upper_signature(previous);
  previous_epoch->tau_guide = {
      ins.grid.idx(0, 4),
      ins.grid.idx(0, 3),
  };
  previous_epoch->target_priority = {2, 1};
  CarrierGuidance previous_guidance;
  previous_guidance.upper_epoch = previous_epoch;
  previous_guidance.custody_by_robot = {
      Custody{
          blocker_effect,
          std::nullopt,
          blocker_effect.shelf,
          blocker_effect.from,
          blocker_effect.to,
          {RootDemand{0, ins.grid.idx(0, 4)}},
          2,
      },
  };

  const auto guidance = dd_task_br_guidance_probe(
      ins, *current, &previous, &previous_guidance, &ops);
  ASSERT_NE(guidance.upper_epoch, nullptr);
  EXPECT_EQ(
      guidance.upper_epoch->priority_commitment,
      (std::vector<int>{0}));
  ASSERT_EQ(guidance.upper_epoch->target_priority.size(), 2u);
  EXPECT_GT(
      guidance.upper_epoch->target_priority[0],
      guidance.upper_epoch->target_priority[1]);
}

TEST(dd_task_br_audit,
     real_solve_registers_multistep_edge_to_closed_duplicate)
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {
      ins.grid.idx(0, 0),
      ins.grid.idx(0, 1),
  };
  ins.shelves = {ins.grid.idx(0, 0)};
  ins.target_starts = {ins.grid.idx(0, 0)};
  ins.target_goals = {ins.grid.idx(0, 2)};
  ins.finalize();
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFStats stats;
  TAPFSearchConfig config;
  config.macro_enabled = true;
  config.stop_at_first = true;
  config.defer_cleanup = true;
  TAPFPlanner planner(
      &view, nullptr, &mt, 0, 0, 0.0f, false, &stats, config);

  const auto solution = planner.solve();
  ASSERT_FALSE(solution.empty());
  ASSERT_GT(stats.macro_successors, 0);
  ASSERT_GT(stats.hl_duplicate_configs, 0);

  std::unordered_map<const TAPFNode*, size_t> incoming_count;
  size_t multistep_edges = 0;
  for (const auto* from : planner.deferred_cleanup_nodes)
    for (const auto& edge : from->outgoing_edges) {
      ++incoming_count[edge->to];
      multistep_edges += edge->transition_trace.size() >= 2;
    }
  EXPECT_GT(
      multistep_edges,
      static_cast<size_t>(stats.macro_successors));

  bool closed_duplicate_witness = false;
  for (const auto* from : planner.deferred_cleanup_nodes)
    for (const auto& edge : from->outgoing_edges)
      closed_duplicate_witness |=
          edge->transition_trace.size() >= 2 &&
          incoming_count[edge->to] >= 2;
  EXPECT_TRUE(closed_duplicate_witness);
}

TEST(dd_task_br_audit,
     rewrite_reinserts_relaxed_node_under_external_incumbent)
{
  DDInstance ins;
  ins.grid = DDGrid({".."});
  ins.robots = {ins.grid.idx(0, 0)};
  ins.finalize();
  const TAPFInstance view(ins);
  std::mt19937 mt(0);
  TAPFSearchConfig config;
  config.incumbent_init = 10;
  TAPFPlanner planner(
      &view, nullptr, &mt, 0, 0, 0.001f, true, nullptr, config);
  const auto shelf = initial_shelf_state(view);
  auto from = std::make_unique<TAPFNode>(
      Config{view.G.U[ins.grid.idx(0, 0)]}, shelf, planner.D, &view,
      std::vector<int>{-1}, TAPFAssignmentState(), nullptr);
  auto relaxed = std::make_unique<TAPFNode>(
      Config{view.G.U[ins.grid.idx(0, 1)]}, shelf, planner.D, &view,
      std::vector<int>{-1}, TAPFAssignmentState(), nullptr);
  from->g = 0;
  relaxed->g = 100;
  relaxed->h = 2;
  relaxed->f = relaxed->g + relaxed->h;
  planner.register_outgoing_edge(from.get(), relaxed.get(), 1, {});

  std::vector<TAPFNode*> open;
  planner.rewrite(from.get(), nullptr, open);

  ASSERT_EQ(open.size(), 1);
  EXPECT_EQ(open.front(), relaxed.get());
  EXPECT_TRUE(relaxed->queued);
  EXPECT_DOUBLE_EQ(relaxed->f, 3);
}
