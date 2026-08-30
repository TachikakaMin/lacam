// Carrier-LaCAM benchmark driver.
// usage: dd_benchmark INSTANCE.yaml TIME_LIMIT_SEC PLAN_OUT [SEED] [MODE]
//   MODE: lacam (default) | b0 (rollout-only baseline) | b1 (2-stage)
//
// stdout metrics: solved, makespan, loaded_moves, free_moves, lift_drop,
//                 weighted_soc (alpha=beta=gamma=delta=1), runtime_ms,
//                 hl_nodes, hl_expanded, pibt_calls, validator_rejects,
//                 duplicate_configs, timed_out
// PLAN_OUT: one line per timestep; per-robot tokens joined by ';':
//           'w' | 'm R C' | 'l' | 'd'   (robot order = YAML robots order)
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <set>
#include <map>

int main(int argc, char** argv)
{
  if (argc < 4) {
    std::cerr << "usage: dd_benchmark INSTANCE.yaml TIME_LIMIT_SEC PLAN_OUT "
                 "[SEED]\n";
    return 2;
  }
  const std::string yaml_path = argv[1];
  const double time_limit_sec = std::stod(argv[2]);
  const std::string plan_out = argv[3];
  const int seed = argc >= 5 ? std::stoi(argv[4]) : 0;
  const std::string mode = argc >= 6 ? argv[5] : "lacam";

  DDInstance ins;
  try {
    ins = load_dd_instance(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "failed to load instance: " << e.what() << "\n";
    std::cout << "valid_instance=0\n";
    return 1;
  }
  std::cout << "valid_instance=1\n";

  const auto t0 = std::chrono::steady_clock::now();
  DDStats stats;
  DDPlan best_effort;
  DDPlan plan;
  if (mode == "b0")
    plan = solve_carrier_rollout(ins, time_limit_sec, seed, &stats);
  else if (mode == "b1")
    plan = solve_carrier_2stage(ins, time_limit_sec, seed, &stats);
  else
    plan = solve_carrier_lacam(
        ins, time_limit_sec, seed, &stats,
        std::getenv("DD_DEBUG_DUMP") ? &best_effort : nullptr);
  if (plan.empty() && !best_effort.empty()) {
    std::ofstream out(plan_out + std::string(".best_effort"));
    for (const auto& ops : best_effort) {
      for (size_t i = 0; i < ops.size(); ++i) {
        if (i) out << ";";
        switch (ops[i].kind) {
          case Op::WAIT: out << "w"; break;
          case Op::MOVE:
            out << "m " << ins.grid.row(ops[i].to) << " "
                << ins.grid.col(ops[i].to);
            break;
          case Op::LIFT: out << "l"; break;
          case Op::DROP: out << "d"; break;
        }
      }
      out << "\n";
    }
  }
  const double runtime_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t0)
          .count();

  long loaded_moves = 0, free_moves = 0, lift_drop = 0, anon_moves = 0;
  bool valid = !plan.empty();
  if (valid) {
    // replay through the validator: belt-and-braces before reporting
    auto s = initial_phys_config(ins);
    std::ofstream out(plan_out);
    for (const auto& ops : plan) {
      for (size_t i = 0; i < ops.size(); ++i) {
        if (i) out << ";";
        switch (ops[i].kind) {
          case Op::WAIT:
            out << "w";
            break;
          case Op::MOVE:
            out << "m " << ins.grid.row(ops[i].to) << " "
                << ins.grid.col(ops[i].to);
            if (s.kappa[i] == KAPPA_FREE) {
              ++free_moves;
            } else {
              ++loaded_moves;
              if (s.kappa[i] == KAPPA_ANON) ++anon_moves;
            }
            break;
          case Op::LIFT:
            out << "l";
            ++lift_drop;
            break;
          case Op::DROP:
            out << "d";
            ++lift_drop;
            break;
        }
      }
      out << "\n";
      auto nxt = apply_ops(ins, s, ops);
      if (!nxt.has_value()) {
        valid = false;
        break;
      }
      s = *nxt;
    }
    if (valid && !is_dd_goal(ins, s)) valid = false;
  }

  // cost weights (design 2.3): alpha*loaded + beta*free + gamma*liftdrop
  // + delta*anon; defaults all 1, overridable via env (DD_ALPHA etc.)
  auto envd = [](const char* k, double dflt) {
    const char* v = std::getenv(k);
    return v ? std::atof(v) : dflt;
  };
  const double alpha = envd("DD_ALPHA", 1.0), beta = envd("DD_BETA", 1.0),
               gamma = envd("DD_GAMMA", 1.0), delta = envd("DD_DELTA", 1.0);
  const double weighted_soc = alpha * loaded_moves + beta * free_moves +
                              gamma * lift_drop + delta * anon_moves;

  std::cout << "solved=" << (valid ? 1 : 0) << "\n";
  std::cout << "makespan=" << (valid ? plan.size() : 0) << "\n";
  std::cout << "loaded_moves=" << loaded_moves << "\n";
  std::cout << "free_moves=" << free_moves << "\n";
  std::cout << "lift_drop=" << lift_drop << "\n";
  std::cout << "anon_moves=" << anon_moves << "\n";
  std::cout << "weighted_soc=" << weighted_soc << "\n";
  std::cout << "runtime_ms=" << runtime_ms << "\n";
  std::cout << "hl_nodes=" << stats.hl_nodes << "\n";
  std::cout << "hl_expanded=" << stats.hl_expanded << "\n";
  std::cout << "pibt_calls=" << stats.pibt_calls << "\n";
  std::cout << "validator_rejects=" << stats.validator_rejects << "\n";
  std::cout << "g1_rejects=" << stats.g1_rejects << "\n";
  std::cout << "generator_failures=" << stats.generator_failures << "\n";
  std::cout << "max_depth=" << stats.max_depth << "\n";
  std::cout << "guidance_builds=" << stats.guidance_builds << "\n";
  std::cout << "path_recomputes=" << stats.path_recomputes << "\n";
  std::cout << "path_cache_hits=" << stats.path_cache_hits << "\n";
  std::cout << "best_targets_done=" << stats.best_targets_done << "\n";
  std::cout << "duplicate_configs=" << stats.duplicate_configs << "\n";
  std::cout << "timed_out=" << (stats.timed_out ? 1 : 0) << "\n";
  std::cout << "seed=" << seed << "\n";
  std::cout << "mode=" << mode << "\n";
  std::cout << "first_solution_ms=" << stats.first_solution_ms << "\n";
  std::cout << "first_solution_soc=" << stats.first_solution_soc << "\n";
  std::cout << "best_soc=" << stats.best_soc << "\n";
  std::cout << "incumbent_updates=" << stats.incumbent_updates << "\n";
  if (!valid && std::getenv("DD_DEBUG_DUMP") &&
      !stats.deepest_config.robots.empty()) {
    const auto& X = stats.deepest_config;
    std::cerr << "--- deepest config (depth=" << stats.max_depth << ") ---\n";
    // upper occupancy set
    std::set<int> upper(X.anon_occ.begin(), X.anon_occ.end());
    for (size_t b = 0; b < ins.n_targets(); ++b) upper.insert(X.target_pos[b]);
    std::map<int, int> robot_at;
    for (size_t i = 0; i < ins.n_robots(); ++i)
      robot_at[X.robots[i]] = (int)i;
    for (size_t b = 0; b < ins.n_targets(); ++b) {
      bool carried = false;
      for (int k : X.kappa) carried |= (k == (int)b);
      const int p = X.target_pos[b], gcell = ins.target_goals[b];
      if (!carried && p == gcell) continue;
      std::cerr << "target b" << b << " at (" << ins.grid.row(p) << ","
                << ins.grid.col(p) << ") goal (" << ins.grid.row(gcell) << ","
                << ins.grid.col(gcell) << ") carried=" << carried << "\n";
      const int r0 = std::max(0, ins.grid.row(gcell) - 3);
      const int r1 = std::min(ins.grid.height - 1, ins.grid.row(gcell) + 3);
      const int c0 = std::max(0, ins.grid.col(gcell) - 3);
      const int c1 = std::min(ins.grid.width - 1, ins.grid.col(gcell) + 3);
      for (int r = r0; r <= r1; ++r) {
        std::string line;
        for (int c = c0; c <= c1; ++c) {
          const int v = ins.grid.idx(r, c);
          char ch = '.';
          if (ins.grid.is_wall(v)) ch = '@';
          else if (v == gcell) ch = upper.count(v) ? 'X' : 'G';
          else if (v == p) ch = 'B';
          else if (upper.count(v)) ch = '#';
          if (robot_at.count(v) && ch == '.') ch = 'r';
          line += ch;
        }
        std::cerr << "  " << line << "\n";
      }
    }
    for (size_t i = 0; i < ins.n_robots(); ++i)
      std::cerr << "robot " << i << " at (" << ins.grid.row(X.robots[i])
                << "," << ins.grid.col(X.robots[i]) << ") kappa="
                << X.kappa[i] << "\n";
  }
  return 0;
}
