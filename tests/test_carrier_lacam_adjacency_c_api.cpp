// PROTECTED TEST: Phase 8.2 C ABI for explicit undirected CSR adjacency.
// Written before implementation (TDD RED).
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

Handle configured_long_edge_case()
{
  Handle handle(carrier_lacam_create(0));
  if (handle == nullptr) return handle;
  const uint8_t wall[] = {0, 0, 0};
  const uint8_t storage[] = {1, 1, 1};
  if (carrier_lacam_set_grid(
          handle.get(), 1, 3, wall, 3, storage, 3) !=
      CARRIER_LACAM_OK)
    return {};
  const int offsets[] = {0, 1, 1, 2};
  const int destinations[] = {2, 0};
  if (carrier_lacam_set_undirected_adjacency(
          handle.get(), offsets, 4, destinations, 2) !=
      CARRIER_LACAM_OK)
    return {};
  const int robots[] = {0};
  const int shelves[] = {2};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {0};
  if (carrier_lacam_set_entities(
          handle.get(), 1, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1) !=
      CARRIER_LACAM_OK)
    return {};
  const int targets[] = {2};
  const int kappa[] = {-1};
  if (carrier_lacam_set_state(
          handle.get(), robots, 1, targets, 1,
          nullptr, 0, kappa, 1) != CARRIER_LACAM_OK)
    return {};
  return handle;
}

}  // namespace

TEST(carrier_lacam_adjacency_c_api,
     symmetric_csr_replaces_coordinate_adjacency)
{
  Handle handle = configured_long_edge_case();
  ASSERT_NE(handle, nullptr);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK)
      << carrier_lacam_last_error(handle.get());
  ASSERT_EQ(carrier_lacam_get_timestep_count(handle.get()), 4);
  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 0, 0),
      CARRIER_LACAM_ACTION_MOVE);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(handle.get(), 0, 0),
      2);
  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 2, 0),
      CARRIER_LACAM_ACTION_MOVE);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(handle.get(), 2, 0),
      0);
}

TEST(carrier_lacam_adjacency_c_api,
     asymmetric_or_malformed_csr_is_rejected)
{
  Handle handle(carrier_lacam_create(0));
  ASSERT_NE(handle, nullptr);
  const uint8_t wall[] = {0, 0, 0};
  const uint8_t storage[] = {1, 1, 1};
  ASSERT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 3, wall, 3, storage, 3),
      CARRIER_LACAM_OK);

  const int asymmetric_offsets[] = {0, 1, 1, 1};
  const int asymmetric_destinations[] = {2};
  EXPECT_EQ(
      carrier_lacam_set_undirected_adjacency(
          handle.get(), asymmetric_offsets, 4,
          asymmetric_destinations, 1),
      CARRIER_LACAM_INVALID_ARGUMENT);
  EXPECT_FALSE(
      std::string(carrier_lacam_last_error(handle.get())).empty());

  const int malformed_offsets[] = {0, 2, 1, 2};
  const int destinations[] = {2, 0};
  EXPECT_EQ(
      carrier_lacam_set_undirected_adjacency(
          handle.get(), malformed_offsets, 4,
          destinations, 2),
      CARRIER_LACAM_INVALID_ARGUMENT);
}
