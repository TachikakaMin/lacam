/*
 * TAPF distance table with lazy evaluation, indexed by task-id and vertex-id.
 */
#pragma once

#include "instance.hpp"

struct TAPFDistTable {
  const int K;  // number of vertices
  std::vector<std::vector<int> > table;
  std::vector<std::queue<Vertex*> > OPEN;

  int get(int task_id, int v_id);
  int get(int task_id, Vertex* v);

  TAPFDistTable(const TAPFInstance& ins);
  TAPFDistTable(const TAPFInstance* ins);

  void setup(const TAPFInstance* ins);
};
