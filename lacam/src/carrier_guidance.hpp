/*
 * Carrier guidance infrastructure (design 5.3/5.4a/5.5/6.2; mapping
 * M6/M8/M9) shared by the integrated TAPF planner (tapf_planner.cpp) and
 * the carrier entry/test-support adapters (dd_planner.cpp).  Internal
 * header (src/): NOT part of the public API.
 *
 * Everything here operates on the conformance-oracle view (DDInstance /
 * PhysConfig, identical cell-index encoding as Vertex::index) and is
 * ordering-only; none of it executes on shelf-free instances.  Called
 * from TAPFPlanner::attach_carrier_guidance (production) and from the
 * carrier adapters/test probes in dd_planner.cpp.
 */
#pragma once



#include "carrier_upper_epoch.hpp"

// Session-owned Carrier state that is safe to retain across independent
// TAPFPlanner instances. Search nodes, parent pointers, OPEN/CLOSED, and
// planner-local scratch are deliberately excluded.
struct TAPFCarrierPersistentState {
  struct Schema {
    int grid_height = 0;
    int grid_width = 0;
    std::vector<uint8_t> grid_wall;
    std::vector<uint8_t> shelf_storage;
    size_t robot_count = 0;
    size_t shelf_count = 0;
    std::vector<int> target_starts;
    std::vector<std::vector<int>> target_goal_sets;

    explicit Schema(const DDInstance& dd)
        : grid_height(dd.grid.height),
          grid_width(dd.grid.width),
          grid_wall(dd.grid.wall),
          shelf_storage(dd.shelf_storage),
          robot_count(dd.n_robots()),
          shelf_count(dd.shelves.size()),
          target_starts(dd.target_starts),
          target_goal_sets(dd.target_goal_sets)
    {
    }

    bool matches(const DDInstance& dd) const
    {
      return grid_height == dd.grid.height &&
             grid_width == dd.grid.width &&
             grid_wall == dd.grid.wall &&
             shelf_storage == dd.shelf_storage &&
             robot_count == dd.n_robots() &&
             shelf_count == dd.shelves.size() &&
             target_starts == dd.target_starts &&
             target_goal_sets == dd.target_goal_sets;
    }
  };

  Schema schema;
  DDDistCache upper_wall;
  carrier_detail::StorageTransferTopology storage_topology;
  carrier_detail::LowerDist lower;
  carrier_detail::UpperEpochCache task_br_cache;

  explicit TAPFCarrierPersistentState(const DDInstance& dd)
      : schema(dd),
        upper_wall(dd.grid),
        storage_topology(
            carrier_detail::build_storage_transfer_topology(dd)),
        lower(dd.grid)
  {
  }

  bool compatible_with(const DDInstance& dd) const
  {
    return schema.matches(dd);
  }
};
