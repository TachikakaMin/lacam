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
