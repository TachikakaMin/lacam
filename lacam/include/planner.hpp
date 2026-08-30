/*
 * LaCAM algorithm
 */
#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

// operator-constraint candidate (design 5.2, mapping M3): a vertex plus
// the primitive-operator kind.  Plain vertex candidates (upstream MAPF /
// shelf-free TAPF) map to MOVE-or-stay and never read the kind.
struct OpCand {
  Vertex* v;
  uint8_t kind;  // Op::Kind
};

// low-level search node
struct Constraint {
  std::vector<int> who;
  Vertices where;
  std::vector<uint8_t> ops;  // Op::Kind per fixed agent (M3); MOVE default
  const int depth;
  Constraint();
  Constraint(Constraint* parent, int i, Vertex* v);  // who and where
  Constraint(Constraint* parent, int i, OpCand c);   // + operator kind
  ~Constraint();
};

#include "search_kernel.hpp"

// high-level search node — fields/priority machinery live in the shared
// LacamNodeCore (node-skeleton audit step 5)
struct Node : LacamNodeCore<Constraint, Node> {
  Node(Config _C, DistTable& D, Node* _parent = nullptr);
};
using Nodes = std::vector<Node*>;

// PIBT agent
struct Agent {
  const int id;
  Vertex* v_now;    // current location
  Vertex* v_next;   // next location
  uint8_t op_kind;  // Op::Kind of the reserved action (carrier layer, M3);
                    // upstream MAPF never reads it
  Agent(int _id) : id(_id), v_now(nullptr), v_next(nullptr), op_kind(0) {}
};
using Agents = std::vector<Agent*>;

// next location candidates, for saving memory allocation
using Candidates = std::vector<std::array<Vertex*, 5> >;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  DistTable D;
  Candidates C_next;                // next location candidates
  std::vector<float> tie_breakers;  // random values, used in PIBT
  Agents A;
  Agents occupied_now;   // for quick collision checking
  Agents occupied_next;  // for quick collision checking

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          int _verbose = 0);
  Solution solve();
  bool get_new_config(Node* S, Constraint* M);
  bool funcPIBT(Agent* ai);
};

// main function
Solution solve(const Instance& ins, const int verbose = 0,
               const Deadline* deadline = nullptr, std::mt19937* MT = nullptr);
