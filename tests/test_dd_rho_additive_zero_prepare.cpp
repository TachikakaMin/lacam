// PROTECTED regression: PREPARE with no realized benefit receives no defer
// reward, even when the task's full EXECUTE urgency is positive.
#include <rho_assignment.hpp>

#include "gtest/gtest.h"

TEST(dd_rho_additive_zero_prepare,
     zero_prepare_benefit_does_not_consume_execute_urgency)
{
  const auto result =
      rho_prepare_cost(/*approach=*/8, /*continuity=*/1,
                       /*mode=*/0, /*urgency=*/5,
                       /*requested_prepare_reward=*/0);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.urgency_reward, 5);
  EXPECT_EQ(result.preparation_reward, 0);
  EXPECT_EQ(result.total, 9);
}
