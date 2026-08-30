/*
 * distance table with lazy evaluation, using BFS
 * (skeleton-reuse refactor #2: thin adapter over the shared LazyBfsField
 *  core in lazy_dist.hpp — semantics unchanged, sentinel = K)
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "lazy_dist.hpp"
#include "utils.hpp"

// free-vertex id space over Graph (grid maps: degree <= 4)
struct GraphIdTopology {
  const Graph& G;
  size_t size() const { return G.V.size(); }
  int neighbors(int id, int out[4]) const
  {
    const auto& nb = G.V[id]->neighbor;
    const int n = (int)nb.size();
    for (int k = 0; k < n; ++k) out[k] = nb[k]->id;
    return n;
  }
};

struct DistTable {
  const int K;  // number of vertices
  GraphIdTopology topo;
  std::vector<LazyBfsField<GraphIdTopology> > fields;  // one per agent goal

  int get(int i, int v_id);   // agent, vertex-id
  int get(int i, Vertex* v);  // agent, vertex

  DistTable(const Instance& ins);
  DistTable(const Instance* ins);

  void setup(const Instance* ins);  // initialization
};
