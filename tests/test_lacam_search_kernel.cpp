// PROTECTED TEST: Carrier BR-LaCAM decomposition baseline, plan.md P1.
// This test was written before the shared DFS kernel implementation.
#include <search_kernel.hpp>

#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

namespace {

struct FakeConstraint {
  int action = -1;
  int depth = 0;
};

struct FakeNode {
  int state = -1;
  FakeNode* parent = nullptr;
  std::queue<FakeConstraint*> search_tree;

  FakeNode(int state_, FakeNode* parent_)
      : state(state_), parent(parent_)
  {
    search_tree.push(new FakeConstraint());
  }

  ~FakeNode()
  {
    while (!search_tree.empty()) {
      delete search_tree.front();
      search_tree.pop();
    }
  }
};

struct FakeDomain {
  using Node = FakeNode;
  using Constraint = FakeConstraint;
  using Successor = int;
  using Key = int;
  using KeyHasher = std::hash<int>;
  using Result = std::vector<int>;

  bool expired() const { return false; }

  Node* make_root() { return new Node(0, nullptr); }

  Key node_key(const Node* node) const { return node->state; }

  Key successor_key(const Successor& successor) const
  {
    return successor;
  }

  bool is_goal(const Node* node) const { return node->state == 3; }

  Result extract_result(const Node* node) const
  {
    Result path;
    while (node != nullptr) {
      path.push_back(node->state);
      node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  void expand_constraint(Node* node, Constraint* constraint)
  {
    if (node->state != 0 || constraint->depth != 0) return;
    node->search_tree.push(new Constraint{1, 1});
    node->search_tree.push(new Constraint{2, 1});
  }

  std::optional<Successor> generate_successor(
      Node* node, Constraint* constraint)
  {
    if (node->state == 0 && constraint->depth == 0) return 1;
    if (node->state == 0 && constraint->action == 1) return 1;
    if (node->state == 0 && constraint->action == 2) return 2;
    if (node->state == 2) return 3;
    return std::nullopt;
  }

  Node* make_child(const Successor& successor, Node* parent)
  {
    return new Node(successor, parent);
  }
};

}  // namespace

TEST(lacam_search_kernel,
     one_dfs_loop_handles_constraint_growth_duplicates_and_path_extraction)
{
  FakeDomain domain;
  const auto outcome = run_lacam_dfs_search(domain);

  ASSERT_TRUE(outcome.solution_found);
  EXPECT_EQ(outcome.result, (std::vector<int>{0, 2, 3}));
  EXPECT_EQ(outcome.explored, 4u);
  EXPECT_EQ(outcome.duplicate_pushes, 1u);
  EXPECT_FALSE(outcome.open_exhausted);
  EXPECT_FALSE(outcome.expired);
}
