// Shared helpers for br_lacam_upper.cpp / br_lacam_replay.cpp.
// NOT part of the public API.
#pragma once

#include "../include/br_lacam_upper.hpp"

#include "carrier_guidance.hpp"

#include "../include/search_kernel.hpp"
#include "../include/utils.hpp"

#include <algorithm>
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>

namespace br_detail
{

inline int handle_index(const BRLabeledUpperState& state, UpperShelfHandle shelf)
{
  if (shelf.kind == UpperShelfHandle::Kind::TARGET)
    return shelf.stable_id >= 0 &&
                   shelf.stable_id < (int)state.target_pos.size()
               ? shelf.stable_id
               : -1;
  return shelf.stable_id >= 0 &&
                 shelf.stable_id < (int)state.anonymous_pos.size()
             ? (int)state.target_pos.size() + shelf.stable_id
             : -1;
}

inline UpperShelfHandle handle_at(const BRLabeledUpperState& state, int index)
{
  const int target_count = (int)state.target_pos.size();
  if (index < target_count)
    return UpperShelfHandle{
        UpperShelfHandle::Kind::TARGET, index};
  return UpperShelfHandle{
      UpperShelfHandle::Kind::ANONYMOUS, index - target_count};
}

inline bool valid_grounded_state(const DDInstance& ins,
                          const BRLabeledUpperState& state)
{
  if (state.target_pos.size() != ins.n_targets()) return false;
  if (ins.shelves.size() < ins.n_targets()) return false;
  if (state.anonymous_pos.size() !=
      ins.shelves.size() - ins.n_targets())
    return false;
  std::vector<uint8_t> occupied(ins.grid.size(), 0);
  auto add = [&](int cell) {
    if (!ins.can_store_shelf(cell) || occupied[cell]) return false;
    occupied[cell] = 1;
    return true;
  };
  for (const int cell : state.target_pos)
    if (!add(cell)) return false;
  for (const int cell : state.anonymous_pos)
    if (!add(cell)) return false;
  return true;
}

inline bool same_transfer(
    const StorageTransfer& a, const StorageTransfer& b)
{
  return a.endpoint == b.endpoint && a.route == b.route;
}

inline ShelfSelector epoch_selector(
    const BRLabeledUpperState& state, UpperShelfHandle shelf)
{
  if (shelf.kind == UpperShelfHandle::Kind::TARGET)
    return ShelfSelector{
        ShelfSelector::Kind::TARGET, shelf.stable_id};
  const int cell = state.position(shelf);
  return ShelfSelector{
      ShelfSelector::Kind::ANON_AT_EPOCH_CELL, cell};
}

}  // namespace br_detail
