#include <iostream>
#include <numeric>
#include <lacam.hpp>
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  TAPFInstance ins(argv[1], "");
  TAPFDistTable distances(ins);
  TAPFAssignmentState state; state.init(ins.N, ins.tasks.size());
  std::vector<int> agents(ins.N); std::iota(agents.begin(), agents.end(), 0);
  auto result=assign_tapf_tasks_dynamic(ins, distances, ins.starts, state, agents, true);
  std::cout << "{\"feasible\":" << (result.feasible ? "true" : "false") << ",\"cost\":" << result.cost << ",\"goals\":[";
  for (size_t i=0;i<ins.N;++i) {
    if (i) std::cout << ",";
    auto vertex=ins.tasks[result.agent_to_task[i]];
    std::cout << "[" << vertex->index/ins.G.width << "," << vertex->index%ins.G.width << "]";
  }
  std::cout << "]}\n";
}
