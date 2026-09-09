// PROTECTED vacancy-chain continuity ordering contract.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_carrier.hpp>
#include <utils.hpp>

#include <set>
#include <vector>

#include "gtest/gtest.h"

namespace {

struct OneReservationView {
  int reserved = -1;

  bool destination_reserved(int cell) const
  {
    return cell == reserved;
  }
  bool endpoint_reserved(int cell) const
  {
    return cell == reserved;
  }
  bool has_distinct_endpoint_reservations() const { return false; }
  bool distinct_endpoint_reserved(int) const { return false; }
};

DDInstance symmetric_line()
{
  DDInstance ins;
  ins.grid = DDGrid({"..."});
  ins.robots = {ins.grid.idx(0, 1)};
  ins.shelves = {ins.grid.idx(0, 1)};
  ins.target_starts = {ins.grid.idx(0, 1)};
  ins.target_goals = {ins.grid.idx(0, 1)};
  ins.shelf_storage.assign(ins.grid.size(), 1);
  ins.finalize();
  return ins;
}

std::set<int> endpoints(
    const carrier_detail::OrderedShelfCandidates& candidates)
{
  std::set<int> out;
  for (int index = 0; index < candidates.count; ++index)
    out.insert(candidates.endpoints[index]);
  return out;
}

}  // namespace

TEST(dd_vacancy_continuity,
     exact_q_tie_prefers_previous_transfer_before_reservation)
{
  const auto ins = symmetric_line();
  const auto upper_signature =
      carrier_detail::make_upper_signature(initial_phys_config(ins));
  const auto upper =
      carrier_detail::make_abstract_upper_state(ins, upper_signature);
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto potential =
      carrier_detail::build_vacancy_potential(ins, upper, topology);
  DDDistCache distance(ins.grid);

  const int left = ins.grid.idx(0, 0);
  const int middle = ins.grid.idx(0, 1);
  const int right = ins.grid.idx(0, 2);
  const ShelfSelector target{
      ShelfSelector::Kind::TARGET, 0};
  const RootDemand tied_root{0, middle};
  const carrier_detail::RootTransferContinuity continuity{
      {tied_root, TransferKey{target, middle, right}}};

  const auto candidates =
      carrier_detail::ordered_shelf_candidate_window(
          ins, upper, target, tied_root, nullptr, true, distance,
          topology, potential, OneReservationView{right},
          &continuity);

  ASSERT_EQ(candidates.count, 2);
  EXPECT_EQ(
      endpoints(candidates),
      (std::set<int>{left, right}))
      << "continuity may reorder candidates but must not change the set";
  EXPECT_EQ(candidates.endpoints[0], right)
      << "an exact-Q continuity match must beat a reservation tie-break";
}

TEST(dd_vacancy_continuity,
     continuity_never_overrides_a_strictly_better_q_score)
{
  const auto ins = symmetric_line();
  const auto upper_signature =
      carrier_detail::make_upper_signature(initial_phys_config(ins));
  const auto upper =
      carrier_detail::make_abstract_upper_state(ins, upper_signature);
  const auto topology =
      carrier_detail::build_storage_transfer_topology(ins);
  const auto potential =
      carrier_detail::build_vacancy_potential(ins, upper, topology);
  DDDistCache distance(ins.grid);

  const int left = ins.grid.idx(0, 0);
  const int middle = ins.grid.idx(0, 1);
  const int right = ins.grid.idx(0, 2);
  const ShelfSelector target{
      ShelfSelector::Kind::TARGET, 0};
  const RootDemand left_root{0, left};
  const carrier_detail::RootTransferContinuity continuity{
      {left_root, TransferKey{target, middle, right}}};

  const auto candidates =
      carrier_detail::ordered_shelf_candidate_window(
          ins, upper, target, left_root, nullptr, true, distance,
          topology, potential, OneReservationView{}, &continuity);

  ASSERT_EQ(candidates.count, 2);
  EXPECT_EQ(candidates.endpoints[0], left)
      << "continuity is a tie-break after Q, not a weighted penalty";
}
