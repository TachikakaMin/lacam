// PROTECTED TEST: C ABI external spacetime commitment and origin.
// Written before implementation (TDD RED), integration Phase 8.5.
#include "../lacam/interface/carrier_lacam_jna.h"

#include <cstdint>
#include <memory>

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
  const uint8_t walls[] = {0, 0, 0, 0};
  const uint8_t storage[] = {1, 1, 1, 1};
  if (carrier_lacam_set_grid(
          handle.get(), 1, 4, walls, 4, storage, 4) !=
      CARRIER_LACAM_OK)
    return {};

  const int lower_offsets[] = {0, 0, 1, 1};
  const int lower_cells[] = {1};
  const int upper_offsets[] = {0, 0, 0, 0};
  if (carrier_lacam_set_spacetime_commitment(
          handle.get(), 3,
          lower_offsets, 4, lower_cells, 1,
          upper_offsets, 4, nullptr, 0,
          nullptr, nullptr, nullptr, 0,
          CARRIER_LACAM_SPACETIME_RELEASE, 0) !=
      CARRIER_LACAM_OK)
    return {};

  const int robots[] = {0};
  const int shelves[] = {1};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {3};
  if (carrier_lacam_set_entities(
          handle.get(), 1, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1) !=
      CARRIER_LACAM_OK)
    return {};
  const int targets[] = {1};
  const int kappa[] = {-1};
  if (carrier_lacam_set_state(
          handle.get(), robots, 1, targets, 1,
          nullptr, 0, kappa, 1) != CARRIER_LACAM_OK)
    return {};
  return handle;
}

}  // namespace

TEST(carrier_lacam_spacetime_c_api,
     setter_is_independent_and_commit_advances_origin)
{
  Handle handle = configured_handle();
  ASSERT_NE(handle, nullptr);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  ASSERT_GT(
      carrier_lacam_get_timestep_count(handle.get()), 0);
  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 0, 0),
      CARRIER_LACAM_ACTION_WAIT);

  const int robots[] = {0};
  const int targets[] = {1};
  const int kappa[] = {-1};
  ASSERT_EQ(
      carrier_lacam_commit_prefix(
          handle.get(), 1, robots, 1, targets, 1,
          nullptr, 0, kappa, 1),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_commitment_time_origin(handle.get()),
      1);

  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 0, 0),
      CARRIER_LACAM_ACTION_MOVE);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(
          handle.get(), 0, 0),
      1);
}

TEST(carrier_lacam_spacetime_c_api,
     ordinary_rebase_cannot_silently_keep_a_commitment)
{
  Handle handle = configured_handle();
  ASSERT_NE(handle, nullptr);
  const int robots[] = {0};
  const int targets[] = {1};
  const int kappa[] = {-1};

  EXPECT_EQ(
      carrier_lacam_rebase_state(
          handle.get(), robots, 1, targets, 1,
          nullptr, 0, kappa, 1),
      CARRIER_LACAM_INVALID_STATE);
  EXPECT_EQ(
      carrier_lacam_rebase_state_with_current_spacetime_commitment(
          handle.get(), 2, robots, 1, targets, 1,
          nullptr, 0, kappa, 1),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_commitment_time_origin(handle.get()),
      2);
}
