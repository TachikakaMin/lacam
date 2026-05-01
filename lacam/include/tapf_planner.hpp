/*
 * LaCAM-style TAPF planner.
 */
#pragma once

#include "planner.hpp"
#include "tapf_assignment.hpp"

struct TAPFConstraint {
  std::vector<int> who;
  Vertices where;
  const int depth;
  TAPFConstraint();
  TAPFConstraint(TAPFConstraint* parent, int i, Vertex* v);
  ~TAPFConstraint();
};

struct TAPFNode {
  const Config C;
  TAPFNode* parent;
  std::vector<int> assignment;
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<TAPFConstraint*> search_tree;

  TAPFNode(Config _C, TAPFDistTable& D, const TAPFInstance* ins,
           std::vector<int> _assignment, TAPFNode* _parent = nullptr);
  ~TAPFNode();
};

struct TAPFStats {
  int hl_loop_iterations = 0;
  int hl_nodes_created = 0;
  int hl_nodes_explored = 0;
  int hl_reinsertions = 0;
  int hl_duplicate_configs = 0;
  int open_max_size = 0;
  int solution_depth = 0;
  int constraints_popped = 0;
  int constraints_generated = 0;
  int constraint_failures = 0;
  int pibt_calls = 0;
  int pibt_failures = 0;
  int pibt_recursions = 0;
  int assignment_calls = 0;
  int assignment_changes = 0;
  double assignment_time_ms = 0;
  bool timed_out = false;
};

struct TAPFPlanner {
  const TAPFInstance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;
  const int sticky_penalty;
  TAPFStats* stats;
  TAPFAssignmentStats assignment_stats;

  const int N;
  const int V_size;
  TAPFDistTable D;
  Candidates C_next;
  std::vector<float> tie_breakers;
  Agents A;
  Agents occupied_now;
  Agents occupied_next;

  TAPFPlanner(const TAPFInstance* _ins, const Deadline* _deadline,
              std::mt19937* _MT, int _verbose = 0,
              int _sticky_penalty = 0, TAPFStats* _stats = nullptr);
  Solution solve();
  bool is_goal_config(const Config& C) const;
  bool get_new_config(TAPFNode* S, TAPFConstraint* M);
  bool funcPIBT(Agent* ai, const std::vector<int>& assignment);
};

Solution solve_tapf(const TAPFInstance& ins, const int verbose = 0,
                    const Deadline* deadline = nullptr,
                    std::mt19937* MT = nullptr,
                    const int sticky_penalty = 0,
                    TAPFStats* stats = nullptr);
