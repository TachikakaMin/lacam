#!/usr/bin/env python3
"""Build search-queue corrections; keep every dynamic assignment result untouched."""
import concurrent.futures,difflib,hashlib,json,subprocess
from pathlib import Path
repo=Path(__file__).resolve().parents[1];root=repo/'build/results/ita_optimization_500';variants=root/'variants';base=(root/'original/tapf_planner.cpp').read_text();base_h=(root/'original/tapf_planner.hpp').read_text();fast=(variants/'fast_checks/tapf_planner.cpp').read_text().replace('#include "tapf_planner.hpp"','#include "../include/tapf_planner.hpp"',1);fast_h=(variants/'fast_checks/tapf_planner.hpp').read_text()
def restart(s,h):
 old='''      if (MT != nullptr && get_random_float(MT) < restart_rate) {
        S_insert = S_init;
      }
      if ((S_goal == nullptr || S_insert->f < S_goal->g) &&
          !S_insert->queued && !S_insert->search_tree.empty()) {
        push_open(S_insert);'''
 new='''      const bool restart_selected = MT != nullptr && get_random_float(MT) < restart_rate;
      if (restart_selected) S_insert = S_init;
      if ((S_goal == nullptr || S_insert->f < S_goal->g) &&
          !S_insert->search_tree.empty() && (!S_insert->queued || restart_selected)) {
        if (S_insert->queued) {
          auto position = std::find(OPEN.begin(), OPEN.end(), S_insert);
          if (position != OPEN.end()) OPEN.erase(position);
          S_insert->queued = false;
        }
        push_open(S_insert);'''
 assert old in s;return s.replace(old,new),h

def stack(s,h):
 assert '  bool queued;' in h
 h=h.replace('  bool queued;', '  int queued;  // Number of OPEN references to this node.')
 s=s.replace('      queued(false),','      queued(0),')
 s=s.replace('    if (!node->queued && !node->search_tree.empty()) {', '    if (!node->search_tree.empty()) {')
 s=s.replace('      node->queued = true;', '      ++node->queued;')
 s=s.replace('    OPEN[index]->queued = false;', '    --OPEN[index]->queued;')
 s=s.replace('          !S_insert->queued && !S_insert->search_tree.empty()) {', '          !S_insert->search_tree.empty()) {')
 s=s.replace('          node_to->queued = true;', '          ++node_to->queued;')
 return s,h

sources={'restart_only':restart(base,base_h),'restart_fast':restart(fast,fast_h),'native_stack':stack(base,base_h),'stack_fast':stack(fast,fast_h)}
def build(item):
 name,(s,h)=item;folder=variants/name;folder.mkdir(exist_ok=True)
 assert 'assignment.agent_to_task =' not in s
 assert s.count('assign_tapf_tasks_dynamic(')==base.count('assign_tapf_tasks_dynamic(')
 (folder/'tapf_planner.hpp').write_text(h);(folder/'tapf_planner.cpp').write_text(s.replace('#include "../include/tapf_planner.hpp"','#include "tapf_planner.hpp"',1))
 patch=''.join(difflib.unified_diff(base.splitlines(True),s.splitlines(True),fromfile='a/lacam/src/tapf_planner.cpp',tofile='b/lacam/src/tapf_planner.cpp'))+''.join(difflib.unified_diff(base_h.splitlines(True),h.splitlines(True),fromfile='a/lacam/include/tapf_planner.hpp',tofile='b/lacam/include/tapf_planner.hpp'));(folder/'change.patch').write_text(patch)
 compile=['/usr/bin/c++','-O3','-DNDEBUG','-Wall','-mtune=native','-march=native','-I'+str(repo/'lacam/include'),'-I/home/yimin/miniconda3/envs/lacam-tapf/include','-c',str(folder/'tapf_planner.cpp'),'-o',str(folder/'planner.o')]
 link=['/usr/bin/c++','-O3',str(repo/'build/CMakeFiles/tapf_benchmark.dir/tools/tapf_benchmark.cpp.o'),str(folder/'planner.o'),str(repo/'build/lacam/liblacam.a'),'-L/home/yimin/miniconda3/envs/lacam-tapf/lib','-Wl,-rpath,/home/yimin/miniconda3/envs/lacam-tapf/lib','-lyaml-cpp','-o',str(folder/'tapf_benchmark')]
 with (folder/'build.log').open('w') as f:
  subprocess.run(compile,stdout=f,stderr=f,check=True);subprocess.run(link,stdout=f,stderr=f,check=True)
 (folder/'provenance.json').write_text(json.dumps({'compile':compile,'link':link,'binary_sha256':hashlib.sha256((folder/'tapf_benchmark').read_bytes()).hexdigest(),'dynamic_ta_result_overridden':False},indent=2));return name
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
 for name in pool.map(build,sources.items()):print(name,flush=True)
