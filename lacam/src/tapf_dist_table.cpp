#include "../include/tapf_dist_table.hpp"

TAPFDistTable::TAPFDistTable(const TAPFInstance& ins)
    : K(ins.G.V.size()), topo{ins.G}
{
  setup(&ins);
}

TAPFDistTable::TAPFDistTable(const TAPFInstance* ins)
    : K(ins->G.V.size()), topo{ins->G}
{
  setup(ins);
}

void TAPFDistTable::setup(const TAPFInstance* ins)
{
  fields.reserve(ins->tasks.size());
  for (size_t i = 0; i < ins->tasks.size(); ++i)
    fields.emplace_back(topo, ins->tasks[i]->id, K);
}

int TAPFDistTable::get(int task_id, int v_id)
{
  return fields[task_id].get(v_id);
}

int TAPFDistTable::get(int task_id, Vertex* v) { return get(task_id, v->id); }
