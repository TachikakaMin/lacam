/*
 * TAPF distance table with lazy evaluation, indexed by task-id and vertex-id.
 * With terrain heights, +-1 height moves cost climb_cost (lazy Dijkstra);
 * otherwise unit-cost BFS (identical to the original behavior).
 */
#pragma once

#include <queue>

#include "instance.hpp"

struct TAPFDistTable {
  const int K;  // number of vertices
  std::vector<std::vector<int> > table;
  std::vector<std::queue<Vertex*> > OPEN;
  // weighted mode (terrain): lazy Dijkstra queues, (dist, vertex)
  using PQItem = std::pair<int, Vertex*>;
  std::vector<std::priority_queue<PQItem, std::vector<PQItem>,
                                  std::greater<PQItem> > >
      OPEN_W;
  const std::vector<int>* heights = nullptr;  // grid-indexed terrain
  int climb_cost = 1;
  bool weighted = false;
  std::vector<std::vector<bool> > closed;  // settled flags (weighted mode)

  int get(int task_id, int v_id);
  int get(int task_id, Vertex* v);

  TAPFDistTable(const TAPFInstance& ins);
  TAPFDistTable(const TAPFInstance* ins);

  void setup(const TAPFInstance* ins);
};
