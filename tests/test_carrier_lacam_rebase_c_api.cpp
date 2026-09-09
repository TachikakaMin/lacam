// PROTECTED TEST: dependency-safe Carrier-LaCAM C ABI rebase contract,
// integration plan Phase 6. Written before implementation (TDD RED).
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

Handle configured_rebase_handle()
{
  Handle handle(carrier_lacam_create(0));
  if (handle == nullptr) return handle;

  const uint8_t wall[] = {0, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t storage[] = {1, 1, 1, 1, 1, 1, 1, 1};
  if (carrier_lacam_set_grid(
          handle.get(), 1, 8, wall, 8, storage, 8) !=
      CARRIER_LACAM_OK)
    return {};

  const int robots[] = {6, 7};
  const int shelves[] = {0, 1, 2, 3};
  const int target_shelves[] = {0, 1};
  const int goal_offsets[] = {0, 3, 6};
  const int goals[] = {0, 2, 4, 0, 2, 4};
  if (carrier_lacam_set_entities(
          handle.get(), 2, robots, 4, shelves, 2,
          target_shelves, goal_offsets, 3, goals, 6) !=
      CARRIER_LACAM_OK)
    return {};

  const int target_cells[] = {0, 1};
  const int anonymous_cells[] = {2, 3};
  const int kappa[] = {-1, -1};
  if (carrier_lacam_set_state(
          handle.get(), robots, 2, target_cells, 2,
          anonymous_cells, 2, kappa, 2) !=
      CARRIER_LACAM_OK)
    return {};
  return handle;
}

}  // namespace

TEST(carrier_lacam_rebase_c_api,
     valid_dynamic_rebase_invalidates_the_old_plan_and_reuses_edges)
{
  Handle handle = configured_rebase_handle();
  ASSERT_NE(handle, nullptr);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);

  const int robots[] = {6, 7};
  const int target_cells[] = {0, 1};
  const int anonymous_cells[] = {3, 5};
  const int kappa[] = {-1, -1};
  ASSERT_EQ(
      carrier_lacam_rebase_state(
          handle.get(), robots, 2, target_cells, 2,
          anonymous_cells, 2, kappa, 2),
      CARRIER_LACAM_OK);

  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_INVALID_STATE);
  EXPECT_EQ(
      carrier_lacam_get_timestep_count(handle.get()), -1);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  EXPECT_GT(
      carrier_lacam_get_changed_pair_edges(handle.get()), 0);
  EXPECT_GT(
      carrier_lacam_get_reused_pair_edges(handle.get()), 0);
  EXPECT_EQ(
      carrier_lacam_get_changed_pair_edges(handle.get()) +
          carrier_lacam_get_reused_pair_edges(handle.get()),
      carrier_lacam_get_total_pair_edges(handle.get()));
}

TEST(carrier_lacam_rebase_c_api,
     invalid_dynamic_rebase_is_atomic)
{
  Handle handle = configured_rebase_handle();
  ASSERT_NE(handle, nullptr);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  const int old_timesteps =
      carrier_lacam_get_timestep_count(handle.get());
  ASSERT_GE(old_timesteps, 0);

  const int invalid_robots[] = {6, 6};
  const int target_cells[] = {0, 1};
  const int anonymous_cells[] = {2, 3};
  const int kappa[] = {-1, -1};
  EXPECT_EQ(
      carrier_lacam_rebase_state(
          handle.get(), invalid_robots, 2,
          target_cells, 2, anonymous_cells, 2,
          kappa, 2),
      CARRIER_LACAM_INVALID_ARGUMENT);

  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_timestep_count(handle.get()),
      old_timesteps);
}
