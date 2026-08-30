// PROTECTED characterization tests (debug.md v3 WP0, design.md v3.0
// semantic invariant): pin the EXACT pre-integration behavior of the
// LaCAM-TAPF planner (solve_tapf) and the upstream MAPF planner (solve)
// on shelf-free instances.  The DD/carrier integration must keep every
// assertion here green WITHOUT touching this file — natural degradation,
// no flags, no fallbacks.
//
// Golden values were captured from HEAD 9861a22 (2026-08-30, pre-
// integration) and double-run verified.  Determinism tiers:
//   - deadline-free runs (first-solution search): the full trajectory is
//     deterministic -> pin size + solution hash + cost + loop iterations
//     + nodes created;
//   - 60s-deadline anytime runs: the post-first-solution phase reads
//     pointer-ordered neighbor sets (run-to-run variance in iters), but
//     the OUTCOME is stable -> pin size + cost only.
#include <lacam.hpp>

#include "gtest/gtest.h"

namespace {

uint64_t fnv(uint64_t h, uint64_t x)
{
  for (int k = 0; k < 8; ++k) {
    h ^= (x >> (8 * k)) & 0xff;
    h *= 1099511628211ULL;
  }
  return h;
}

uint64_t solution_hash(const Solution& sol)
{
  uint64_t h = 1469598103934665603ULL;
  h = fnv(h, sol.size());
  for (const auto& C : sol)
    for (const auto* v : C) h = fnv(h, (uint64_t)v->index);
  return h;
}

struct Golden {
  int seed;              // -1 = MT nullptr
  size_t size;
  uint64_t hash;
  unsigned cost;
  int iters;
  int nodes;
};

void expect_deterministic_run(const TAPFInstance& ins, const Golden& g)
{
  TAPFStats st;
  std::mt19937 mt(g.seed);
  std::mt19937* mtp = g.seed >= 0 ? &mt : nullptr;
  const auto sol = solve_tapf(ins, 0, nullptr, mtp, 0, &st);
  EXPECT_EQ(sol.size(), g.size) << "seed " << g.seed;
  EXPECT_EQ(solution_hash(sol), g.hash) << "seed " << g.seed;
  EXPECT_EQ(st.solution_cost, g.cost) << "seed " << g.seed;
  EXPECT_EQ(st.hl_loop_iterations, g.iters) << "seed " << g.seed;
  EXPECT_EQ(st.hl_nodes_created, g.nodes) << "seed " << g.seed;
}

void expect_anytime_outcome(const TAPFInstance& ins, int seed,
                            TAPFSearchMode mode, size_t size, unsigned cost)
{
  TAPFStats st;
  TAPFSearchConfig cfg;
  cfg.mode = mode;
  std::mt19937 mt(seed);
  Deadline dl(60 * 1000);
  const auto sol = solve_tapf(ins, 0, &dl, &mt, 0, &st, true, false, cfg);
  EXPECT_EQ(sol.size(), size) << "seed " << seed;
  EXPECT_EQ(st.solution_cost, cost) << "seed " << seed;
}

TAPFInstance instance_A()
{
  return TAPFInstance(
      "./assets/empty-8-8.map", std::vector<int>{0, 1, 8},
      std::vector<std::vector<int>>{
          {63, 62, 55}, {63, 62, 55}, {63, 62, 55}});
}

TAPFInstance instance_B()
{
  return TAPFInstance("./third_party/ITA-CBS2/map_file/debug_cbs_data.yaml",
                      "./third_party/ITA-CBS2/map_file");
}

TAPFInstance instance_C()
{
  Graph g("./assets/random-32-32-10.map");
  std::vector<int> starts;
  std::vector<std::vector<int>> tasks;
  std::mt19937 gen(7);
  std::vector<int> cells;
  for (auto* v : g.V) cells.push_back(v->index);
  std::shuffle(cells.begin(), cells.end(), gen);
  for (int i = 0; i < 12; ++i) starts.push_back(cells[i]);
  for (int i = 0; i < 12; ++i) {
    std::vector<int> t;
    for (int j = 0; j < 3; ++j) t.push_back(cells[100 + 3 * i + j]);
    t.push_back(cells[200 + (i / 2)]);
    tasks.push_back(t);
  }
  return TAPFInstance("./assets/random-32-32-10.map", starts, tasks);
}

}  // namespace

TEST(tapf_compat, empty8_shared_tasks_deterministic)
{
  const auto ins = instance_A();
  ASSERT_TRUE(ins.is_valid());
  expect_deterministic_run(ins, {-1, 15, 10672109841913456188ULL, 40, 15, 15});
  expect_deterministic_run(ins, {0, 15, 14259379848777366309ULL, 40, 15, 15});
  expect_deterministic_run(ins, {1, 15, 15740135639527816946ULL, 40, 15, 15});
  expect_deterministic_run(ins, {2, 15, 15571108187103813012ULL, 40, 15, 15});
}

TEST(tapf_compat, empty8_shared_tasks_anytime_outcome)
{
  const auto ins = instance_A();
  expect_anytime_outcome(ins, 0, TAPFSearchMode::DFS, 14, 38);
  expect_anytime_outcome(ins, 0, TAPFSearchMode::FOCAL, 14, 38);
}

TEST(tapf_compat, itacbs_yaml_deterministic)
{
  const auto ins = instance_B();
  ASSERT_TRUE(ins.is_valid());
  expect_deterministic_run(ins, {-1, 8, 11891090403137052935ULL, 53, 8, 8});
  expect_deterministic_run(ins, {0, 6, 11163886842914532641ULL, 36, 6, 6});
  expect_deterministic_run(ins, {1, 10, 4028115162436958223ULL, 42, 10, 10});
}

TEST(tapf_compat, itacbs_yaml_anytime_outcome)
{
  const auto ins = instance_B();
  expect_anytime_outcome(ins, 1, TAPFSearchMode::FOCAL, 6, 33);
}

TEST(tapf_compat, random32_programmatic_deterministic)
{
  const auto ins = instance_C();
  ASSERT_TRUE(ins.is_valid());
  expect_deterministic_run(ins, {-1, 29, 9843472546437004456ULL, 130, 29, 29});
  expect_deterministic_run(ins, {0, 29, 15878261995596361413ULL, 134, 29, 29});
  expect_deterministic_run(ins, {1, 29, 16698401964857785370ULL, 132, 29, 29});
  expect_deterministic_run(ins, {2, 29, 3847445436325743918ULL, 130, 29, 29});
}

TEST(tapf_compat, random32_programmatic_anytime_outcome)
{
  const auto ins = instance_C();
  expect_anytime_outcome(ins, 0, TAPFSearchMode::DFS, 29, 134);
  expect_anytime_outcome(ins, 2, TAPFSearchMode::FOCAL, 29, 130);
}

TEST(tapf_compat, upstream_mapf_solve_deterministic)
{
  const auto ins = Instance("./assets/random-32-32-10-random-1.scen",
                            "./assets/random-32-32-10.map", 30);
  {
    std::mt19937 mt(0);
    const auto sol = solve(ins, 0, nullptr, &mt);
    EXPECT_EQ(sol.size(), 54u);
    EXPECT_EQ(solution_hash(sol), 5108782387747977564ULL);
  }
  {
    std::mt19937 mt(1);
    const auto sol = solve(ins, 0, nullptr, &mt);
    EXPECT_EQ(sol.size(), 54u);
    EXPECT_EQ(solution_hash(sol), 13080592691313221996ULL);
  }
}
