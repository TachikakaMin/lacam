#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>

#include "motion.hpp"

namespace
{
  const MotionTransition* find_transition(const MotionGraph& motion, int from,
                                          MotionMoveAction move,
                                          MotionSpeedAction speed)
  {
    for (const auto& edge : motion.successors(from)) {
      if (edge.move == move && edge.speed_change == speed) return &edge;
    }
    return nullptr;
  }
}  // namespace

TEST(Motion, PaperDefaultsAccelerateMoveAndStop)
{
  const auto graph = Graph("./tests/assets/5x1.map");
  auto parameters = MotionParameters();
  parameters.enabled = true;
  parameters.max_speed = 2;
  parameters.rotation_steps = 2;
  const auto motion = MotionGraph(graph, parameters);

  const auto start = motion.state_id(graph.U[0], 0);
  ASSERT_GE(start, 0);
  const auto accelerate = find_transition(motion, start, MotionMoveAction::STAY,
                                          MotionSpeedAction::ACCELERATE);
  ASSERT_NE(accelerate, nullptr);
  EXPECT_EQ(motion.state(accelerate->to).location, graph.U[0]);
  EXPECT_EQ(motion.state(accelerate->to).speed, 1);

  const auto decelerate =
      find_transition(motion, accelerate->to, MotionMoveAction::FORWARD,
                      MotionSpeedAction::DECELERATE);
  ASSERT_NE(decelerate, nullptr);
  EXPECT_EQ(motion.state(decelerate->to).location, graph.U[1]);
  EXPECT_EQ(motion.state(decelerate->to).speed, 0);
  EXPECT_EQ(decelerate->swept_cells, (std::vector<int>{0, 1}));
}

TEST(Motion, RotationUsesConfiguredNumberOfTimesteps)
{
  const auto graph = Graph("./tests/assets/5x1.map");
  auto parameters = MotionParameters();
  parameters.enabled = true;
  parameters.rotation_steps = 3;
  const auto motion = MotionGraph(graph, parameters);
  auto state = motion.state_id(graph.U[2], 0);
  for (auto step = 1; step <= 3; ++step) {
    const auto edge = find_transition(
        motion, state, MotionMoveAction::ROTATE_CCW, MotionSpeedAction::KEEP);
    ASSERT_NE(edge, nullptr);
    state = edge->to;
    EXPECT_EQ(motion.state(state).heading, step);
    EXPECT_EQ(motion.state(state).omega, step == 3 ? 0 : 1);
  }
}

TEST(Motion, ActionAvailabilityAndCostsAreConfigurable)
{
  const auto graph = Graph("./tests/assets/5x1.map");
  auto parameters = MotionParameters();
  parameters.enabled = true;
  parameters.actions.rotate_cw = false;
  parameters.costs.rotate_ccw = 7;
  parameters.costs.keep_speed = 2;
  const auto motion = MotionGraph(graph, parameters);
  const auto state = motion.state_id(graph.U[2], 0);
  EXPECT_EQ(find_transition(motion, state, MotionMoveAction::ROTATE_CW,
                            MotionSpeedAction::KEEP),
            nullptr);
  const auto ccw = find_transition(motion, state, MotionMoveAction::ROTATE_CCW,
                                   MotionSpeedAction::KEEP);
  ASSERT_NE(ccw, nullptr);
  EXPECT_EQ(ccw->cost, 9);
}

TEST(Motion, DistanceAccountsForHeadingAndRotationCost)
{
  const auto graph = Graph("./tests/assets/5x1.map");
  auto parameters = MotionParameters();
  parameters.enabled = true;
  parameters.rotation_steps = 2;
  auto motion = MotionGraph(graph, parameters);
  const auto east = motion.state_id(graph.U[0], 0);
  const auto west = motion.state_id(graph.U[0], 2);
  const auto east_cost = motion.distance(east, 0, graph.U[4], 0);
  const auto west_cost = motion.distance(west, 0, graph.U[4], 0);
  EXPECT_LT(east_cost, MotionGraph::kInf);
  EXPECT_GT(west_cost, east_cost);
}

TEST(Motion, StoppingClearanceExcludesUnsafeFastState)
{
  const auto graph = Graph("./tests/assets/5x1.map");
  auto parameters = MotionParameters();
  parameters.enabled = true;
  parameters.max_speed = 2;
  const auto motion = MotionGraph(graph, parameters);
  EXPECT_GE(motion.state_id(graph.U[1], 0, 2), 0);
  EXPECT_EQ(motion.state_id(graph.U[2], 0, 2), -1);
}

TEST(Motion, PrecomputedPathCacheRoundTripsAndRejectsWrongParameters)
{
  const auto graph = Graph("./tests/assets/5x1.map");
  auto parameters = MotionParameters();
  parameters.enabled = true;
  parameters.lookahead_horizon = 3;
  auto original = MotionGraph(graph, parameters);
  original.precompute_path_candidates(2);
  ASSERT_GT(original.path_candidate_count(), 0u);

  const auto suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto cache = std::filesystem::temp_directory_path() /
                     ("motion-path-cache-" + std::to_string(suffix) + ".bin");
  original.save_path_cache(cache);

  auto loaded = MotionGraph(graph, parameters);
  ASSERT_TRUE(loaded.load_path_cache(cache));
  EXPECT_EQ(loaded.path_candidate_count(), original.path_candidate_count());
  for (auto state_id = 0; state_id < original.size(); ++state_id) {
    const auto& expected = original.path_candidates(state_id);
    const auto& actual = loaded.path_candidates(state_id);
    EXPECT_EQ(actual.stop_candidate, expected.stop_candidate);
    ASSERT_EQ(actual.size(), expected.size());
    for (auto candidate = 0; candidate < expected.size(); ++candidate) {
      for (auto t = 0; t < parameters.lookahead_horizon; ++t) {
        EXPECT_EQ(actual.state(candidate, t, parameters.lookahead_horizon),
                  expected.state(candidate, t, parameters.lookahead_horizon));
      }
    }
  }

  parameters.lookahead_horizon = 4;
  auto incompatible = MotionGraph(graph, parameters);
  EXPECT_FALSE(incompatible.load_path_cache(cache));
  std::filesystem::remove(cache);
}
