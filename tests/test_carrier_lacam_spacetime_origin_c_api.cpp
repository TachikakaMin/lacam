// PROTECTED TEST: C ABI rejects a commitment origin that cannot advance.
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

TEST(carrier_lacam_spacetime_origin_c_api,
     maximum_origin_is_rejected_by_the_setter)
{
  Handle handle(carrier_lacam_create(0));
  ASSERT_NE(handle, nullptr);
  const uint8_t walls[] = {0, 0};
  const uint8_t storage[] = {1, 1};
  ASSERT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 2, walls, 2, storage, 2),
      CARRIER_LACAM_OK);
  const int offsets[] = {0, 0};
  EXPECT_EQ(
      carrier_lacam_set_spacetime_commitment(
          handle.get(), 1,
          offsets, 2, nullptr, 0,
          offsets, 2, nullptr, 0,
          nullptr, nullptr, nullptr, 0,
          CARRIER_LACAM_SPACETIME_RELEASE,
          std::numeric_limits<int64_t>::max()),
      CARRIER_LACAM_INVALID_ARGUMENT);
}
