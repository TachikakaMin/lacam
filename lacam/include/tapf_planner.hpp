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
  bool service_goal_mode = false;
  int service_commit_agents = 0;
  int pickup_service_duration = 1;
  int delivery_service_duration = 1;
};

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
  std::set<TAPFNode*> neighbor;
  std::vector<int> assignment;
  TAPFAssignmentState assignment_state;
  std::vector<int> service_assignment;
  std::vector<int> service_progress;
  std::vector<bool> satisfied;
  std::vector<int> satisfied_assignment;
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
           const TAPFSearchConfig& search_config,
           const std::vector<int>& _service_assignment = std::vector<int>(),
           const std::vector<int>& _service_progress = std::vector<int>(),
           const std::vector<bool>& _satisfied = std::vector<bool>(),
           const std::vector<int>& _satisfied_assignment = std::vector<int>(),
           TAPFNode* _parent = nullptr);
  ~TAPFNode();
  void discard_search_tree();
  void refresh_priority(TAPFDistTable& D, const TAPFInstance* ins,
                        const TAPFSearchConfig& search_config);
  void refresh_search_metrics(TAPFDistTable& D, const TAPFInstance* ins,
                              const TAPFSearchConfig& search_config);
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
  int assignment_infeasible_count = 0;
  int service_child_validation_failures = 0;
  int service_child_stack_validation_failures = 0;
  int service_child_swap_validation_failures = 0;
  int final_assignment_changes = 0;
  int final_agent_assignment_changes = 0;
  int anytime_cost_updates = 0;
  int incumbent_updates = 0;
  int swap_checks = 0;
  int swap_applied = 0;
  int initial_assignment_cost = 0;
  unsigned solution_cost = 0;
  unsigned solution_h = 0;
  unsigned first_solution_cost = 0;
  unsigned solution_parent_edge_cost = 0;
  double assignment_time_ms = 0;
  long assignment_row_cache_requests = 0;
  long assignment_row_cache_hits = 0;
  double first_solution_time_ms = 0;
  bool timed_out = false;
  int service_satisfied_agents = 0;
  int service_satisfied_pickups = 0;
  int service_satisfied_deliveries = 0;
  int service_best_satisfied_agents = 0;
  std::vector<int> initial_assignment;
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
  std::vector<bool> service_required_agents;

  const int N;
  const int V_size;
  TAPFDistTable D;
  Candidates C_next;
  std::vector<float> tie_breakers;
  Agents A;
  Agents occupied_now;
  Agents occupied_next;
  std::vector<int> shared_goal_entry_counts;
  std::vector<bool> real_service_vertices;

  TAPFPlanner(const TAPFInstance* _ins, const Deadline* _deadline,
              std::mt19937* _MT, int _verbose = 0, int _sticky_penalty = 0,
              float _restart_rate = 0.001f, bool _anytime = true,
              TAPFStats* _stats = nullptr,
              TAPFSearchConfig _search_config = TAPFSearchConfig());
  Solution solve(
      std::vector<int>* final_assignment = nullptr,
      std::vector<std::vector<int> >* assignment_schedule = nullptr);
  bool agent_satisfied(const TAPFNode* node, int agent) const;
  Vertex* assigned_goal(const std::vector<int>& assignment, int agent) const;
  Vertex* service_goal(const TAPFNode* node, int agent) const;
  Vertex* service_goal_for_state(
      const std::vector<int>& assignment,
      const std::vector<int>& service_assignment,
      const std::vector<bool>& satisfied,
      const std::vector<int>& satisfied_assignment, int agent) const;
  bool agent_has_service_option_at(int agent, Vertex* vertex) const;
  bool can_share_service_goal(const TAPFNode* node, int agent,
                              Vertex* vertex) const;
  bool can_share_service_goal_for_state(
      const std::vector<int>& assignment,
      const std::vector<int>& service_assignment,
      const std::vector<bool>& satisfied,
      const std::vector<int>& satisfied_assignment, int agent,
      Vertex* vertex) const;
  bool can_reserve_next(const TAPFNode* node, Agent* agent,
                        Vertex* vertex);
  void reserve_next(const TAPFNode* node, Agent* agent, Vertex* vertex);
  bool validate_service_child_config(
      const TAPFNode* parent, const Config& C,
      const std::vector<int>& assignment,
      const std::vector<int>& service_assignment,
      const std::vector<bool>& satisfied,
      const std::vector<int>& satisfied_assignment,
      bool* stack_failure = nullptr, bool* swap_failure = nullptr) const;
  int distance_to_assigned_goal(const TAPFNode* node, int agent, Vertex* v);
  int distance_to_assigned_goal(const std::vector<int>& assignment, int agent,
                                Vertex* v);
  bool is_goal_node(const TAPFNode* node) const;
  bool get_new_config(TAPFNode* S, TAPFConstraint* M);
  void rewrite(TAPFNode* from, TAPFNode* to, TAPFNode* goal,
               std::vector<TAPFNode*>& OPEN);
  unsigned get_edge_cost(const TAPFNode* from, const TAPFNode* to) const;
  unsigned get_h_value(const Config& C);
  unsigned get_h_value(const TAPFNode* node);
  Agent* swap_possible_and_required(Agent* ai, const TAPFNode* node);
  bool is_swap_required(const int pusher, const int puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin,
                        const TAPFNode* node);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin,
                        const TAPFNode* node);
  bool funcPIBT(Agent* ai, const TAPFNode* node);
};

Solution solve_tapf(const TAPFInstance& ins, const int verbose = 0,
                    const Deadline* deadline = nullptr,
                    std::mt19937* MT = nullptr, const int sticky_penalty = 0,
                    TAPFStats* stats = nullptr, bool anytime = true,
                    bool force_full_assignment = false,
                    TAPFSearchConfig search_config = TAPFSearchConfig(),
                    std::vector<int>* final_assignment = nullptr,
                    std::vector<std::vector<int> >* assignment_schedule =
                        nullptr);
