// PROTECTED REGRESSION TEST: no allocation failure may escape the
// Carrier-LaCAM extern "C" boundary.
#include "../lacam/interface/carrier_lacam_jna.h"

#include <atomic>
#include <cstdlib>
#include <new>
#include <string>

#include "gtest/gtest.h"

namespace {

std::atomic<int> allocations_to_fail{0};

bool should_fail_allocation()
{
  int remaining =
      allocations_to_fail.load(std::memory_order_relaxed);
  while (remaining > 0) {
    if (allocations_to_fail.compare_exchange_weak(
            remaining, remaining - 1,
            std::memory_order_relaxed))
      return true;
  }
  return false;
}

void* allocate_or_throw(std::size_t size)
{
  if (should_fail_allocation()) throw std::bad_alloc();
  if (void* memory = std::malloc(size == 0 ? 1 : size))
    return memory;
  throw std::bad_alloc();
}

}  // namespace

void* operator new(std::size_t size)
{
  return allocate_or_throw(size);
}

void* operator new[](std::size_t size)
{
  return allocate_or_throw(size);
}

void operator delete(void* memory) noexcept
{
  std::free(memory);
}

void operator delete[](void* memory) noexcept
{
  std::free(memory);
}

void operator delete(void* memory, std::size_t) noexcept
{
  std::free(memory);
}

void operator delete[](void* memory, std::size_t) noexcept
{
  std::free(memory);
}

TEST(carrier_lacam_exception_boundary,
     error_reporting_allocation_failure_does_not_escape)
{
  void* handle = carrier_lacam_create(0);
  ASSERT_NE(handle, nullptr);

  int status = -1;
  bool exception_escaped = false;
  allocations_to_fail.store(2, std::memory_order_relaxed);
  try {
    status = carrier_lacam_set_grid(
        handle, 0, 0, nullptr, 0, nullptr, 0);
  } catch (...) {
    exception_escaped = true;
  }
  allocations_to_fail.store(0, std::memory_order_relaxed);

  EXPECT_FALSE(exception_escaped);
  EXPECT_EQ(status, CARRIER_LACAM_INTERNAL_ERROR);
  ASSERT_NE(carrier_lacam_last_error(handle), nullptr);
  EXPECT_FALSE(
      std::string(carrier_lacam_last_error(handle)).empty());
  carrier_lacam_destroy(handle);
}
