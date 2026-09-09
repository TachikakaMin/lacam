// PROTECTED fixed-goal forced-effect normalization cache contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <vector>

#include "gtest/gtest.h"

TEST(dd_vacancy_forced_normalization_cache,
     production_key_reuses_only_matching_canonical_effects)
{
  carrier_detail::UpperEpochCache cache(2);
  const UpperSignature upper{{1}, {}};
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, 0};
  const std::vector<TaskBRForcedEffect> raw{
      TaskBRForcedEffect{
          shelf,
          TaskBRForcedEffect::Kind::TRANSFER,
          StorageTransfer{2, {1, 2}},
          {RootDemand{0, 9}},
          1}};
  const std::vector<TaskBRForcedEffect> canonical;
  const std::vector<int> priority{3};
  std::vector<TaskBRForcedEffect> found;

  EXPECT_FALSE(
      cache.lookup_forced_effect_normalization(
          upper, raw, priority, found));
  cache.insert_forced_effect_normalization(
      upper, raw, priority, canonical);
  ASSERT_TRUE(
      cache.lookup_forced_effect_normalization(
          upper, raw, priority, found));
  EXPECT_EQ(found, canonical);

  const UpperSignature different_upper{{2}, {}};
  EXPECT_FALSE(
      cache.lookup_forced_effect_normalization(
          different_upper, raw, priority, found));

  auto different_raw = raw;
  different_raw.front().transfer =
      StorageTransfer{3, {1, 3}};
  EXPECT_FALSE(
      cache.lookup_forced_effect_normalization(
          upper, different_raw, priority, found));

  const std::vector<int> different_priority{4};
  EXPECT_FALSE(
      cache.lookup_forced_effect_normalization(
          upper, raw, different_priority, found));
}
