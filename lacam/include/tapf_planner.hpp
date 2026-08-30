/*
 * LaCAM-style TAPF planner.
 */
#pragma once

#include <set>
#include <vector>

#include "planner.hpp"
#include "tapf_assignment.hpp"

enum class TAPFSearchMode {
  DFS = 0,
  FOCAL = 1,
};

enum class TAPFFocalTieBreak {
  H = 0,
  ANTI_WAIT = 1,
  ANTI_ZIGZAG = 2,
  ANTI_PUSH = 3,
  ANTI_ALL = 4,
};

struct TAPFSearchConfig {
  TAPFSearchMode mode = TAPFSearchMode::DFS;
  TAPFFocalTieBreak focal_tie_break = TAPFFocalTieBreak::H;
  double focal_weight = 1.5;
};

// skeleton dedup (node-skeleton audit 2026-08-30): TAPFConstraint was a
// byte-identical twin of planner.hpp's Constraint — now ONE type.
using TAPFConstraint = Constraint;

struct TAPFNode {
  const Config C;
  TAPFNode* parent;
  std::set<TAPFNode*> neighbor;
  std::vector<int> assignment;
  TAPFAssignmentState assignment_state;
  bool queued;
  unsigned g;
  unsigned h;
  unsigned f;
  unsigned depth;
  unsigned non_goal_waits;
  unsigned reversals;
  unsigned distance_increases;
  unsigned settled_pushes;
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<TAPFConstraint*> search_tree;

  TAPFNode(Config _C, TAPFDistTable& D, const TAPFInstance* ins,
           std::vector<int> _assignment, TAPFAssignmentState _assignment_state,
           TAPFNode* _parent = nullptr);
  ~TAPFNode();
  void discard_search_tree();
  void refresh_priority(TAPFDistTable& D);
  void refresh_search_metrics(TAPFDistTable& D, const TAPFInstance* ins);
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
  int final_assignment_changes = 0;
  int final_agent_assignment_changes = 0;
  int anytime_cost_updates = 0;
  int incumbent_updates = 0;
  int swap_checks = 0;
  int swap_applied = 0;
  unsigned solution_cost = 0;
  unsigned first_solution_cost = 0;
  unsigned solution_parent_edge_cost = 0;
  double assignment_time_ms = 0;
  double first_solution_time_ms = 0;
  bool timed_out = false;
};

struct TAPFPlanner {
  const TAPFInstance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;
  const int sticky_penalty;
  const float restart_rate;
  const bool anytime;
  const TAPFSearchConfig search_config;
  bool force_full_assignment;
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
              std::mt19937* _MT, int _verbose = 0, int _sticky_penalty = 0,
              float _restart_rate = 0.001f, bool _anytime = true,
              TAPFStats* _stats = nullptr,
              TAPFSearchConfig _search_config = TAPFSearchConfig());
  Solution solve();
  bool is_goal_config(const Config& C) const;
  bool get_new_config(TAPFNode* S, TAPFConstraint* M);
  void rewrite(TAPFNode* from, TAPFNode* to, TAPFNode* goal,
               std::vector<TAPFNode*>& OPEN);
  unsigned get_edge_cost(const TAPFNode* from, const TAPFNode* to) const;
  unsigned get_h_value(const Config& C);
  Agent* swap_possible_and_required(Agent* ai,
                                    const std::vector<int>& assignment);
  bool is_swap_required(const int pusher, const int puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin,
                        const std::vector<int>& assignment);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin,
                        const std::vector<int>& assignment);
  bool funcPIBT(Agent* ai, const std::vector<int>& assignment);
};

Solution solve_tapf(const TAPFInstance& ins, const int verbose = 0,
                    const Deadline* deadline = nullptr,
                    std::mt19937* MT = nullptr, const int sticky_penalty = 0,
                    TAPFStats* stats = nullptr, bool anytime = true,
                    bool force_full_assignment = false,
                    TAPFSearchConfig search_config = TAPFSearchConfig());
