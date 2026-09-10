// PROTECTED TEST: Phase 8.3 directed CSR C ABI.
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

}  // namespace

TEST(carrier_lacam_directed_adjacency_c_api,
     directed_csr_allows_forward_plan_without_reverse_arc)
{
  Handle handle(carrier_lacam_create(0));
  ASSERT_NE(handle, nullptr);
  const uint8_t wall[] = {0, 0, 0};
  const uint8_t storage[] = {1, 1, 1};
  ASSERT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 3, wall, 3, storage, 3),
      CARRIER_LACAM_OK);
  const int offsets[] = {0, 1, 2, 2};
  const int destinations[] = {1, 2};
  ASSERT_EQ(
      carrier_lacam_set_directed_adjacency(
          handle.get(), offsets, 4, destinations, 2),
      CARRIER_LACAM_OK);

  const int robots[] = {0};
  const int shelves[] = {1};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {2};
  ASSERT_EQ(
      carrier_lacam_set_entities(
          handle.get(), 1, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1),
      CARRIER_LACAM_OK);
  const int targets[] = {1};
  const int kappa[] = {-1};
  ASSERT_EQ(
      carrier_lacam_set_state(
          handle.get(), robots, 1, targets, 1,
          nullptr, 0, kappa, 1),
      CARRIER_LACAM_OK);

  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK)
      << carrier_lacam_last_error(handle.get());
  EXPECT_EQ(
      carrier_lacam_get_action_destination(handle.get(), 0, 0),
      1);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(handle.get(), 2, 0),
      2);
}
