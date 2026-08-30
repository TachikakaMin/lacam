#include "../include/dist_table.hpp"

DistTable::DistTable(const Instance& ins)
    : K(ins.G.V.size()), topo{ins.G}
{
  setup(&ins);
}

DistTable::DistTable(const Instance* ins)
    : K(ins->G.V.size()), topo{ins->G}
{
  setup(ins);
}

void DistTable::setup(const Instance* ins)
{
  fields.reserve(ins->N);
  for (size_t i = 0; i < ins->N; ++i)
    fields.emplace_back(topo, ins->goals[i]->id, K);
}

int DistTable::get(int i, int v_id) { return fields[i].get(v_id); }

int DistTable::get(int i, Vertex* v) { return get(i, v->id); }
