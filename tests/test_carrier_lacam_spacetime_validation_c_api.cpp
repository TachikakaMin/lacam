// PROTECTED TEST: hostile C ABI spacetime input must fail closed.
// Written before the overflow fix (TDD RED), integration Phase 8.5.
#include "../lacam/interface/carrier_lacam_jna.h"

#include <cstdint>
#include <limits>
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

TEST(carrier_lacam_spacetime_validation_c_api,
     maximum_edge_tick_is_rejected_without_indexing)
{
  Handle handle(carrier_lacam_create(0));
  ASSERT_NE(handle, nullptr);
  const uint8_t walls[] = {0, 0};
  const uint8_t storage[] = {1, 1};
  ASSERT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 2, walls, 2, storage, 2),
      CARRIER_LACAM_OK);

  const int offsets[] = {0, 0, 0};
  const int edge_ticks[] = {
      std::numeric_limits<int>::max()};
  const int edge_from[] = {0};
  const int edge_to[] = {1};
  EXPECT_EQ(
      carrier_lacam_set_spacetime_commitment(
          handle.get(), 2,
          offsets, 3, nullptr, 0,
          offsets, 3, nullptr, 0,
          edge_ticks, edge_from, edge_to, 1,
          CARRIER_LACAM_SPACETIME_RELEASE, 0),
      CARRIER_LACAM_INVALID_ARGUMENT);
}
