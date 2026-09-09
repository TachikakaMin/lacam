// PROTECTED: Phase 7 incremental rho telemetry across the C ABI.
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
  const uint8_t wall[] = {0, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t storage[] = {1, 1, 1, 1, 1, 1, 1, 1};
  if (carrier_lacam_set_grid(
          handle.get(), 1, 8, wall, 8, storage, 8) !=
      CARRIER_LACAM_OK)
    return {};
  const int robots[] = {0, 7};
  const int shelves[] = {1, 6};
  const int target_shelves[] = {0, 1};
  const int goal_offsets[] = {0, 1, 2};
  const int goals[] = {2, 5};
  if (carrier_lacam_set_entities(
          handle.get(), 2, robots, 2, shelves, 2,
          target_shelves, goal_offsets, 3, goals, 2) !=
      CARRIER_LACAM_OK)
    return {};
  const int target_cells[] = {1, 6};
  const int kappa[] = {-1, -1};
  if (carrier_lacam_set_state(
          handle.get(), robots, 2, target_cells, 2,
          nullptr, 0, kappa, 2) != CARRIER_LACAM_OK)
    return {};
  return handle;
}

}  // namespace

TEST(carrier_lacam_rho_metrics_c_api,
     solve_exports_incremental_counts_and_time_distribution)
{
  Handle handle = configured_handle();
  ASSERT_NE(handle, nullptr);
  EXPECT_EQ(
      carrier_lacam_get_rho_incremental_full_solves(
          handle.get()),
      -1);

  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  EXPECT_GT(
      carrier_lacam_get_rho_incremental_full_solves(
          handle.get()),
      0);
  EXPECT_GE(
      carrier_lacam_get_rho_incremental_repairs(
          handle.get()),
      0);
  EXPECT_GE(
      carrier_lacam_get_rho_incremental_zero_row_reuses(
          handle.get()),
      0);
  EXPECT_GE(
      carrier_lacam_get_rho_incremental_changed_rows(
          handle.get()),
      0);
  EXPECT_GE(
      carrier_lacam_get_rho_bottleneck_ms(handle.get()), 0);
  EXPECT_GE(
      carrier_lacam_get_rho_secondary_full_ms(handle.get()), 0);
  EXPECT_GE(
      carrier_lacam_get_rho_secondary_repair_ms(handle.get()), 0);
  EXPECT_GE(
      carrier_lacam_get_rho_canonical_ms(handle.get()), 0);
}
