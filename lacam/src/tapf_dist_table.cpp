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
  for (size_t i = 0; i < ins->tasks.size(); ++i) {
    OPEN.push_back(std::queue<Vertex*>());
    auto n = ins->tasks[i];
    OPEN[i].push(n);
    table[i][n->id] = 0;
  }
}

int TAPFDistTable::get(int task_id, int v_id)
{
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
