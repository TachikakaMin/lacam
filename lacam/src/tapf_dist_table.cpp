#include "../include/tapf_dist_table.hpp"

TAPFDistTable::TAPFDistTable(const TAPFInstance& ins)
    : K(ins.G.V.size()), table(ins.tasks.size(), std::vector<int>(K, K))
{
  setup(&ins);
}

TAPFDistTable::TAPFDistTable(const TAPFInstance* ins)
    : K(ins->G.V.size()), table(ins->tasks.size(), std::vector<int>(K, K))
{
  setup(ins);
}

void TAPFDistTable::setup(const TAPFInstance* ins)
{
  weighted = !ins->height_by_index.empty() && ins->climb_cost > 1;
  if (weighted) {
    heights = &ins->height_by_index;
    climb_cost = ins->climb_cost;
    const int INF = K * climb_cost;  // > any real path cost
    for (auto& row : table) row.assign(K, INF);
    OPEN_W.resize(ins->tasks.size());
    closed.assign(ins->tasks.size(), std::vector<bool>(K, false));
    for (size_t i = 0; i < ins->tasks.size(); ++i) {
      auto n = ins->tasks[i];
      table[i][n->id] = 0;
      OPEN_W[i].push({0, n});
    }
    return;
  }
  for (size_t i = 0; i < ins->tasks.size(); ++i) {
    OPEN.push_back(std::queue<Vertex*>());
    auto n = ins->tasks[i];
    OPEN[i].push(n);
    table[i][n->id] = 0;
  }
}

int TAPFDistTable::get(int task_id, int v_id)
{
  if (weighted) {
    // lazy Dijkstra: resume until v_id is settled
    if (closed[task_id][v_id]) return table[task_id][v_id];
    auto& pq = OPEN_W[task_id];
    while (!pq.empty()) {
      auto [d_n, n] = pq.top();
      pq.pop();
      if (closed[task_id][n->id]) continue;
      closed[task_id][n->id] = true;
      for (auto& m : n->neighbor) {
        const int step =
            ((*heights)[n->index] != (*heights)[m->index]) ? climb_cost : 1;
        if (d_n + step < table[task_id][m->id]) {
          table[task_id][m->id] = d_n + step;
          pq.push({d_n + step, m});
        }
      }
      if (n->id == v_id) return d_n;
    }
    return K * climb_cost;
  }

  if (table[task_id][v_id] < K) return table[task_id][v_id];

  while (!OPEN[task_id].empty()) {
    auto n = OPEN[task_id].front();
    OPEN[task_id].pop();
    const int d_n = table[task_id][n->id];
    for (auto& m : n->neighbor) {
      const int d_m = table[task_id][m->id];
      if (d_n + 1 >= d_m) continue;
      table[task_id][m->id] = d_n + 1;
      OPEN[task_id].push(m);
    }
    if (n->id == v_id) return d_n;
  }
  return K;
}

int TAPFDistTable::get(int task_id, Vertex* v) { return get(task_id, v->id); }
