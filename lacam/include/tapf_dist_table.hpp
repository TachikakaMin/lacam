/*
 * TAPF distance table with lazy evaluation, indexed by task-id and
 * vertex-id.  (skeleton-reuse refactor #2: thin adapter over the shared
 * LazyBfsField core — semantics unchanged, sentinel = K)
 */
#pragma once

#include "dist_table.hpp"
#include "instance.hpp"

struct TAPFDistTable {
  const int K;  // number of vertices
  GraphIdTopology topo;
  std::vector<LazyBfsField<GraphIdTopology> > fields;  // one per task

  int get(int task_id, int v_id);
  int get(int task_id, Vertex* v);

  TAPFDistTable(const TAPFInstance& ins);
  TAPFDistTable(const TAPFInstance* ins);

  void setup(const TAPFInstance* ins);
};
