// PROTECTED stale continuity-key normalization contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

TEST(dd_vacancy_continuity_stale_key,
     drops_only_keys_whose_shelf_has_left_the_recorded_source)
{
  const RootDemand moved_root{0, 9};
  const RootDemand stable_root{1, 8};
  const ShelfSelector moved_shelf{
      ShelfSelector::Kind::TARGET, 0};
  const ShelfSelector stable_shelf{
      ShelfSelector::Kind::TARGET, 1};
  const carrier_detail::RootTransferContinuity previous{
      {moved_root, TransferKey{moved_shelf, 1, 2}},
      {stable_root, TransferKey{stable_shelf, 5, 6}},
  };
  const UpperSignature upper{
      {2, 5},
      {},
  };
  const std::vector<int> tau{9, 8};

  EXPECT_EQ(
      carrier_detail::active_transfer_continuity_for_epoch(
          &previous, {}, tau, upper),
      (carrier_detail::RootTransferContinuity{
          {stable_root,
           TransferKey{stable_shelf, 5, 6}},
      }));
}
