#include "../include/instance.hpp"

#include <unordered_map>

Instance::Instance(const std::string& map_filename,
                   const std::vector<int>& start_indexes,
                   const std::vector<int>& goal_indexes)
    : G(map_filename),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size())
{
  for (auto k : start_indexes) starts.push_back(G.U[k]);
  for (auto k : goal_indexes) goals.push_back(G.U[k]);
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string& scen_filename,
                   const std::string& map_filename, const int _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // load start-goal pairs
  std::ifstream file(scen_filename);
  if (!file) {
    info(0, 0, scen_filename, " is not found");
    return;
  }
  std::string line;
  std::smatch results;

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_instance)) {
      auto x_s = std::stoi(results[1].str());
      auto y_s = std::stoi(results[2].str());
      auto x_g = std::stoi(results[3].str());
      auto y_g = std::stoi(results[4].str());
      if (x_s < 0 || G.width <= x_s || x_g < 0 || G.width <= x_g) continue;
      if (y_s < 0 || G.height <= y_s || y_g < 0 || G.height <= y_g) continue;
      auto s = G.U[G.width * y_s + x_s];
      auto g = G.U[G.width * y_g + x_g];
      if (s == nullptr || g == nullptr) continue;
      starts.push_back(s);
      goals.push_back(g);
    }

    if (starts.size() == N) break;
  }
}

Instance::Instance(const std::string& map_filename, std::mt19937* MT,
                   const int _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // random assignment
  const auto K = G.size();

  // set starts
  auto s_indexes = std::vector<int>(K);
  std::iota(s_indexes.begin(), s_indexes.end(), 0);
  std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
  int i = 0;
  while (true) {
    if (i >= K) return;
    starts.push_back(G.V[s_indexes[i]]);
    if (starts.size() == N) break;
    ++i;
  }

  // set goals
  auto g_indexes = std::vector<int>(K);
  std::iota(g_indexes.begin(), g_indexes.end(), 0);
  std::shuffle(g_indexes.begin(), g_indexes.end(), *MT);
  int j = 0;
  while (true) {
    if (j >= K) return;
    goals.push_back(G.V[g_indexes[j]]);
    if (goals.size() == N) break;
    ++j;
  }
}

bool Instance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != goals.size()) {
    info(1, verbose, "invalid N, check instance");
    return false;
  }
  return true;
}

TAPFInstance::TAPFInstance(const std::string& map_filename,
                           const std::vector<int>& start_indexes,
                           const std::vector<std::vector<int> >& task_indexes)
    : G(map_filename),
      starts(Config()),
      tasks(Config()),
      allowed(std::vector<std::vector<bool> >()),
      N(start_indexes.size())
{
  std::unordered_map<int, int> index_to_task;
  for (auto k : start_indexes) starts.push_back(G.U[k]);

  allowed.resize(N);
  for (size_t i = 0; i < task_indexes.size(); ++i) {
    for (auto k : task_indexes[i]) {
      if (index_to_task.find(k) == index_to_task.end()) {
        index_to_task[k] = tasks.size();
        tasks.push_back(G.U[k]);
        for (auto& row : allowed) row.push_back(false);
      }
      allowed[i][index_to_task[k]] = true;
    }
  }
}

namespace {
// DDGrid wall bitmap -> map rows for the shared Graph builder
std::vector<std::string> dd_grid_rows(const DDGrid& g)
{
  std::vector<std::string> rows(g.height, std::string(g.width, '.'));
  for (int r = 0; r < g.height; ++r)
    for (int c = 0; c < g.width; ++c)
      if (g.is_wall(g.idx(r, c))) rows[r][c] = '@';
  return rows;
}
}  // namespace

TAPFInstance::TAPFInstance(const DDInstance& dd)
    : G(dd_grid_rows(dd.grid)),
      starts(Config()),
      tasks(Config()),
      allowed(std::vector<std::vector<bool> >(dd.robots.size())),
      N(dd.robots.size()),
      shelf_cells(dd.shelves),
      shelf_storage(dd.shelf_storage),
      fixed_upper_cells(dd.fixed_upper_cells),
      target_starts(dd.target_starts),
      target_goals(dd.target_goals),
      target_goal_sets(dd.target_goal_sets)
{
  for (auto cell : dd.robots) starts.push_back(G.U[cell]);
}

bool TAPFInstance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != allowed.size()) {
    info(1, verbose, "invalid N, check TAPF instance");
    return false;
  }
  // carrier form (design.md v3, M1): agents may have NO instance tasks
  // when the instance declares rearrangement targets — the goal condition
  // then quantifies over targets, not agent tasks.
  const auto carrier_form =
      tasks.empty() &&
      (!target_starts.empty() || !fixed_upper_cells.empty());
  if (!carrier_form && tasks.size() < N) {
    info(1, verbose, "TAPF expects at least one unique task per agent");
    return false;
  }
  for (size_t i = 0; i < N; ++i) {
    if (starts[i] == nullptr) {
      info(1, verbose, "invalid TAPF start");
      return false;
    }
    if (allowed[i].size() != tasks.size()) {
      info(1, verbose, "invalid TAPF compatibility matrix");
      return false;
    }
    auto any_allowed = false;
    for (size_t j = 0; j < tasks.size(); ++j) {
      if (tasks[j] == nullptr) {
        info(1, verbose, "invalid TAPF task");
        return false;
      }
      any_allowed = any_allowed || allowed[i][j];
    }
    if (!any_allowed && !carrier_form) {
      info(1, verbose, "agent has no allowed TAPF task");
      return false;
    }
  }
  return true;
}
