#!/usr/bin/env python3
"""Instrument private dynamic-TA builds. Trace runs are not benchmark results."""
import argparse
import concurrent.futures
import json
import subprocess
from pathlib import Path

repo = Path(__file__).resolve().parents[1]
root = repo / 'build/results/ita_optimization_500'
p = argparse.ArgumentParser()
p.add_argument('variants', nargs='+')
args = p.parse_args()

instrument = r'''
  std::ofstream trace;
  if (const char* path = std::getenv("TAPF_SEARCH_TRACE")) trace.open(path);
  unsigned long trace_duplicate_queued = 0, trace_restarts = 0;
  unsigned long trace_restart_applied = 0, trace_restart_blocked = 0;
  TAPFNode* trace_best = S_init;
  auto trace_node = [&](TAPFNode* node, const char* event) {
    if (!trace.is_open()) return;
    unsigned reached = 0;
    for (size_t i=0; i<N; ++i) reached += node->C[i] == ins->tasks[node->assignment[i]];
    trace << "{\"event\":\"" << event << "\",\"elapsed_ms\":" << elapsed_ms(deadline)
          << ",\"nodes\":" << stats->hl_nodes_created << ",\"depth\":" << node->depth
          << ",\"h\":" << node->h << ",\"assignment_changes\":" << stats->assignment_changes
          << ",\"assigned_reached\":" << reached << ",\"open_size\":" << OPEN.size()
          << ",\"duplicate_queued\":" << trace_duplicate_queued
          << ",\"restart_selected\":" << trace_restarts
          << ",\"restart_applied\":" << trace_restart_applied
          << ",\"restart_blocked\":" << trace_restart_blocked << ",\"positions\":[";
    for (size_t i=0; i<N; ++i) {
      if (i) trace << ',';
      trace << '[' << node->C[i]->index / ins->G.width << ',' << node->C[i]->index % ins->G.width << ']';
    }
    trace << "],\"targets\":[";
    for (size_t i=0; i<N; ++i) {
      if (i) trace << ',';
      auto v=ins->tasks[node->assignment[i]];
      trace << '[' << v->index / ins->G.width << ',' << v->index % ins->G.width << ']';
    }
    trace << "]}\n";
  };
  trace_node(S_init, "initial");
'''

def build(name):
    source = root / 'variants' / name
    dest = root / 'variants' / (name + '_trace')
    dest.mkdir(exist_ok=True)
    s = (source / 'tapf_planner.cpp').read_text()
    s = s.replace('#include "../include/tapf_planner.hpp"', '#include "tapf_planner.hpp"', 1)
    s = '#include <fstream>\n#include <cstdlib>\n' + s
    anchor = '  const auto initial_lower_bound = S_init->h;'
    assert anchor in s
    s = s.replace(anchor, instrument + '\n' + anchor, 1)
    s = s.replace('      auto S_known = iter->second;', '      auto S_known = iter->second;\n      if (S_known->queued) ++trace_duplicate_queued;', 1)
    s = s.replace('      auto S_insert = S_known;', '      auto S_insert = S_known;\n      bool trace_restart = false;', 1)
    s = s.replace('        S_insert = S_init;', '        S_insert = S_init;\n        trace_restart = true; ++trace_restarts;', 1)
    s = s.replace('      if (restart_selected) S_insert = S_init;', '      if (restart_selected) { S_insert = S_init; trace_restart = true; ++trace_restarts; }', 1)
    s = s.replace('        push_open(S_insert);', '        push_open(S_insert);\n        if (trace_restart) ++trace_restart_applied;', 1)
    s = s.replace('      if (stats != nullptr) ++stats->hl_duplicate_configs;', '      if (trace_restart) {\n        trace_restart_blocked = trace_restarts - trace_restart_applied;\n        if (trace_restarts <= 12) trace_node(S_insert, "restart");\n      }\n      if (stats != nullptr) ++stats->hl_duplicate_configs;', 1)
    s = s.replace('        S_goal = S;', '        S_goal = S;\n        trace_node(S, "solution");', 1)
    anchor = '    if (stats != nullptr) ++stats->hl_nodes_created;'
    assert anchor in s
    s = s.replace(anchor, anchor + '\n    if (S_new->h < trace_best->h) trace_best = S_new;\n    if (stats->hl_nodes_created % 1000 == 0) trace_node(S_new, "sample");', 1)
    s = s.replace('  auto solution = Solution();', '  trace_node(trace_best, "end_best_h");\n  trace.close();\n  auto solution = Solution();', 1)
    (dest / 'tapf_planner.cpp').write_text(s)
    header = source / 'tapf_planner.hpp'
    if not header.exists():
        header = root / 'original/tapf_planner.hpp'
    (dest / 'tapf_planner.hpp').write_bytes(header.read_bytes())
    provenance = json.loads((source / 'provenance.json').read_text())
    # Some older provenance files use different keys; construct the two commands explicitly.
    compile_cmd = ['/usr/bin/c++', '-O3', '-DNDEBUG', '-Wall', '-mtune=native', '-march=native', '-I'+str(repo/'lacam/include'), '-I/home/yimin/miniconda3/envs/lacam-tapf/include', '-c', str(dest/'tapf_planner.cpp'), '-o', str(dest/'planner.o')]
    link_cmd = ['/usr/bin/c++', '-O3', str(repo/'build/CMakeFiles/tapf_benchmark.dir/tools/tapf_benchmark.cpp.o'), str(dest/'planner.o'), str(repo/'build/lacam/liblacam.a'), '-L/home/yimin/miniconda3/envs/lacam-tapf/lib', '-Wl,-rpath,/home/yimin/miniconda3/envs/lacam-tapf/lib', '-lyaml-cpp', '-o', str(dest/'tapf_benchmark')]
    with (dest/'build.log').open('w') as log:
        subprocess.run(compile_cmd, stdout=log, stderr=log, check=True)
        subprocess.run(link_cmd, stdout=log, stderr=log, check=True)
    (dest/'provenance.json').write_text(json.dumps({'source_variant': name, 'diagnostic_only': True, 'compile':compile_cmd,'link':link_cmd}, indent=2))
    print(name, flush=True)

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    list(pool.map(build, args.variants))
