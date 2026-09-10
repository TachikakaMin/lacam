// PROTECTED TEST: Phase 8.6 fixed upper-deck C ABI bridge.
// Written before implementation (TDD RED).
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

Handle configured_grid()
{
  Handle handle(carrier_lacam_create(0));
  EXPECT_NE(handle, nullptr);
  const uint8_t wall[] = {0, 0, 0};
  const uint8_t storage[] = {1, 1, 1};
  EXPECT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 3, wall, 3, storage, 3),
      CARRIER_LACAM_OK);
  return handle;
}

}  // namespace

TEST(carrier_lacam_fixed_upper_c_api,
     fixed_upper_cells_are_applied_to_the_entity_schema)
{
  auto handle = configured_grid();
  const int fixed_upper[] = {1};
  ASSERT_EQ(
      carrier_lacam_set_fixed_upper_cells(
          handle.get(), fixed_upper, 1),
      CARRIER_LACAM_OK);

  const int robots[] = {0};
  const int shelves[] = {2};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {2};
  EXPECT_EQ(
      carrier_lacam_set_entities(
          handle.get(), 1, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1),
      CARRIER_LACAM_OK)
      << carrier_lacam_last_error(handle.get());
}

TEST(carrier_lacam_fixed_upper_c_api,
     movable_shelf_overlap_is_rejected_by_existing_schema_validation)
{
  auto handle = configured_grid();
  const int fixed_upper[] = {2};
  ASSERT_EQ(
      carrier_lacam_set_fixed_upper_cells(
          handle.get(), fixed_upper, 1),
      CARRIER_LACAM_OK);

  const int robots[] = {0};
  const int shelves[] = {2};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {1};
  EXPECT_EQ(
      carrier_lacam_set_entities(
          handle.get(), 1, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1),
      CARRIER_LACAM_INVALID_ARGUMENT);
}
