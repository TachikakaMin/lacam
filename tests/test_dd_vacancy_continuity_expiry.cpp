// PROTECTED one-epoch transfer-continuity lifetime contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include "gtest/gtest.h"

namespace {

UpperEpochGuidance epoch_with_first_transfer(
    const RootDemand& root, const TransferKey& key)
{
  UpperEpochGuidance epoch;
  epoch.task_graph.tasks.push_back(
      ShelfTask{
          TaskId{key.shelf, key.source, key.endpoint},
          {root},
          1,
          StorageTransfer{
              key.endpoint,
              {key.source, key.endpoint}}});
  epoch.task_graph.predecessors.emplace_back();
  epoch.task_graph.successors.emplace_back();
  return epoch;
}

}  // namespace

TEST(dd_vacancy_continuity_expiry,
     a_protected_choice_does_not_renew_itself)
{
  const RootDemand root{0, 9};
  const TransferKey key{
      ShelfSelector{ShelfSelector::Kind::TARGET, 0},
      1,
      2};
  auto epoch = epoch_with_first_transfer(root, key);
  epoch.transfer_continuity.emplace(root, key);

  EXPECT_TRUE(
      carrier_detail::
          next_epoch_transfer_continuity(epoch)
              .empty());
}

TEST(dd_vacancy_continuity_expiry,
     a_naturally_changed_choice_can_protect_the_next_epoch)
{
  const RootDemand root{0, 9};
  const ShelfSelector shelf{
      ShelfSelector::Kind::TARGET, 0};
  const TransferKey selected{shelf, 1, 2};
  auto epoch = epoch_with_first_transfer(root, selected);
  epoch.transfer_continuity.emplace(
      root, TransferKey{shelf, 1, 3});

  EXPECT_EQ(
      carrier_detail::
          next_epoch_transfer_continuity(epoch),
      (carrier_detail::RootTransferContinuity{
          {root, selected},
      }));
}
