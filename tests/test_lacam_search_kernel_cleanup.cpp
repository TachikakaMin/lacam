// PROTECTED REGRESSION: consumed low-level constraints must not accumulate
// until post-deadline cleanup.
#include <search_kernel.hpp>

#include <optional>
#include <queue>

#include "gtest/gtest.h"

namespace {

struct CleanupProbe {
  int created = 0;
  int destroyed = 0;
  int deadline_probes = 0;
  int destroyed_at_second_probe = -1;
};

struct ProbeConstraint {
  CleanupProbe* probe = nullptr;

  explicit ProbeConstraint(CleanupProbe* probe_) : probe(probe_)
  {
    ++probe->created;
  }

  ~ProbeConstraint()
  {
    ++probe->destroyed;
  }
};

struct ProbeNode {
  int state = 0;
  std::queue<ProbeConstraint*> search_tree;

  explicit ProbeNode(CleanupProbe* probe)
  {
    search_tree.push(new ProbeConstraint(probe));
    search_tree.push(new ProbeConstraint(probe));
    search_tree.push(new ProbeConstraint(probe));
  }

  ~ProbeNode()
  {
    while (!search_tree.empty()) {
      delete search_tree.front();
      search_tree.pop();
    }
  }
};

struct CleanupProbeDomain {
  using Node = ProbeNode;
  using Constraint = ProbeConstraint;
  using Successor = int;
  using Key = int;
  using KeyHasher = std::hash<int>;
  using Result = int;

  CleanupProbe probe;

  bool expired()
  {
    ++probe.deadline_probes;
    if (probe.deadline_probes == 1) return false;
    if (probe.deadline_probes == 2)
      probe.destroyed_at_second_probe = probe.destroyed;
    return true;
  }

  Node* make_root() { return new Node(&probe); }

  Key node_key(const Node* node) const { return node->state; }

  Key successor_key(const Successor& successor) const
  {
    return successor;
  }

  bool is_goal(const Node*) const { return false; }

  Result extract_result(const Node*) const { return 0; }

  void expand_constraint(Node*, Constraint*) {}

  std::optional<Successor> generate_successor(
      Node*, Constraint*)
  {
    return std::nullopt;
  }

  Node* make_child(const Successor&, Node*) { return nullptr; }
};

}  // namespace

TEST(lacam_search_kernel_cleanup,
     consumed_constraint_is_destroyed_before_next_deadline_probe)
{
  CleanupProbeDomain domain;

  const auto outcome = run_lacam_dfs_search(domain);

  EXPECT_TRUE(outcome.expired);
  EXPECT_EQ(domain.probe.destroyed_at_second_probe, 1);
  EXPECT_EQ(domain.probe.created, 3);
  EXPECT_EQ(domain.probe.destroyed, domain.probe.created);
}
