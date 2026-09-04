// PROTECTED joint-compiler aggregate-progress regression.
// Added before implementation on 2026-09-03 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(dd_task_br_joint_aggregate,
     aggregate_progress_precedes_per_root_tie_break)
{
  carrier_detail::JointCompileCandidate concentrated;
  concentrated.valid = true;
  concentrated.success = {1, 1};
  concentrated.remaining_mission_by_root = {0, 100};
  concentrated.remaining_mission_distance = 100;
  concentrated.estimated_shelf_cost = 1;

  carrier_detail::JointCompileCandidate balanced;
  balanced.valid = true;
  balanced.success = {1, 1};
  balanced.remaining_mission_by_root = {1, 1};
  balanced.remaining_mission_distance = 2;
  balanced.estimated_shelf_cost = 2;

  EXPECT_TRUE(
      carrier_detail::better_joint_candidate(balanced, concentrated));
  EXPECT_FALSE(
      carrier_detail::better_joint_candidate(concentrated, balanced));
}
