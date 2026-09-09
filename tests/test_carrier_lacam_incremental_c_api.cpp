// PROTECTED TEST: persistent Carrier-LaCAM C ABI continuation contract,
// integration plan Phase 6. Written before implementation (TDD RED).
#include "../lacam/interface/carrier_lacam_jna.h"

#include <cstdint>
#include <memory>
#include <string>

#include "gtest/gtest.h"

namespace {

struct HandleDeleter {
  void operator()(void* handle) const
  {
    carrier_lacam_destroy(handle);
  }
};

using Handle = std::unique_ptr<void, HandleDeleter>;

Handle configured_handle()
{
  Handle handle(carrier_lacam_create(0));
  if (handle == nullptr) return handle;
  const uint8_t wall[] = {0, 0, 0, 0};
  const uint8_t storage[] = {1, 1, 1, 1};
  if (carrier_lacam_set_grid(
          handle.get(), 1, 4, wall, 4, storage, 4) !=
      CARRIER_LACAM_OK)
    return {};
  const int robots[] = {0, 3};
  const int shelves[] = {1};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {2};
  if (carrier_lacam_set_entities(
          handle.get(), 2, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1) !=
      CARRIER_LACAM_OK)
    return {};
  const int target_cells[] = {1};
  const int kappa[] = {-1, -1};
  if (carrier_lacam_set_state(
          handle.get(), robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2) != CARRIER_LACAM_OK)
    return {};
  return handle;
}

}  // namespace

TEST(carrier_lacam_incremental_c_api,
     commit_requires_a_solved_plan_and_valid_prefix_length)
{
  Handle handle = configured_handle();
  ASSERT_NE(handle, nullptr);
  const int robots[] = {1, 3};
  const int target_cells[] = {1};
  const int kappa[] = {-1, -1};

  EXPECT_EQ(
      carrier_lacam_commit_prefix(
          handle.get(), 1, robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2),
      CARRIER_LACAM_INVALID_STATE);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_commit_prefix(
          handle.get(), -1, robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2),
      CARRIER_LACAM_INVALID_ARGUMENT);
  EXPECT_EQ(
      carrier_lacam_commit_prefix(
          handle.get(), 99, robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2),
      CARRIER_LACAM_INVALID_ARGUMENT);
}

TEST(carrier_lacam_incremental_c_api,
     mismatch_is_atomic_and_success_invalidates_the_old_plan)
{
  Handle handle = configured_handle();
  ASSERT_NE(handle, nullptr);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);

  const int original_robots[] = {0, 3};
  const int observed_robots[] = {1, 3};
  const int target_cells[] = {1};
  const int kappa[] = {-1, -1};
  EXPECT_EQ(
      carrier_lacam_commit_prefix(
          handle.get(), 1, original_robots, 2,
          target_cells, 1, nullptr, 0, kappa, 2),
      CARRIER_LACAM_INVALID_STATE);
  ASSERT_EQ(
      carrier_lacam_commit_prefix(
          handle.get(), 1, observed_robots, 2,
          target_cells, 1, nullptr, 0, kappa, 2),
      CARRIER_LACAM_OK);

  EXPECT_EQ(carrier_lacam_get_status(handle.get()),
            CARRIER_LACAM_INVALID_STATE);
  EXPECT_EQ(carrier_lacam_get_timestep_count(handle.get()), -1);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  EXPECT_GT(carrier_lacam_get_pair_cache_hits(handle.get()), 0);
  EXPECT_EQ(
      carrier_lacam_get_root_pair_cache_misses(handle.get()), 0);
  EXPECT_STREQ(carrier_lacam_last_error(handle.get()), "");
}
