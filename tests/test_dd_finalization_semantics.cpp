// PROTECTED regression test: correctness rejection and deadline rejection
// are distinct finalization outcomes.  Written TDD RED, 2026-09-02.
#include <dd_planner.hpp>

#include "gtest/gtest.h"

TEST(dd_finalization_semantics, invalid_replay_is_not_reported_as_timeout)
{
  EXPECT_EQ(dd_classify_finalization_probe(false, 9000.0, 10000.0),
            DDFinalizationStatus::INVALID);
  EXPECT_EQ(dd_classify_finalization_probe(true, 10001.0, 10000.0),
            DDFinalizationStatus::DEADLINE);
  EXPECT_EQ(dd_classify_finalization_probe(true, 9999.0, 10000.0),
            DDFinalizationStatus::ACCEPT);
}
