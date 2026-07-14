// Build this helper against the authors' MAWPF library.  It intentionally uses
// their random-instance constructor so both solvers receive identical starts,
// goal cells, and uniformly sampled headings.
#include <fstream>
#include <instance.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  if (argc != 8 && argc != 9) {
    std::cerr << "usage: mawpf_instance_dump MAP N SEED L VMAX TROT OUTPUT "
                 "[labeled]\n";
    return 2;
  }
  const auto map = std::string(argv[1]);
  const auto agents = std::stoi(argv[2]);
  const auto seed = std::stoi(argv[3]);
  const auto horizon = std::stoi(argv[4]);
  const auto max_speed = std::stoi(argv[5]);
  const auto rotation_steps = std::stoi(argv[6]);
  const auto output = std::string(argv[7]);
  const auto labeled = argc == 9 && std::string(argv[8]) == "labeled";
  const auto instance =
      Instance(horizon, max_speed, rotation_steps, map, agents, seed);
  if (!instance.is_valid()) return 1;

  auto out = std::ofstream(output);
  out << "map: " << map << "\n";
  out << "agents:\n";
  for (auto i = 0; i < agents; ++i) {
    const auto start = instance.starts[i];
    out << "  - start: [" << start->y << ", " << start->x << ", "
        << start->dir / rotation_steps << "]\n";
    out << "    potentialGoals:\n";
    // LaCAM-TAPF retains assignment: every high-level node may choose any of
    // the same physical goal states used by the labeled MAWPF instance.
    const auto first_goal = labeled ? i : 0;
    const auto last_goal = labeled ? i + 1 : agents;
    for (auto j = first_goal; j < last_goal; ++j) {
      const auto goal = instance.goals[j];
      out << "      - [" << goal->y << ", " << goal->x << ", "
          << goal->dir / rotation_steps << "]\n";
    }
  }
  return out ? 0 : 1;
}
