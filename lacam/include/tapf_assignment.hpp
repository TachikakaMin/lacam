/*
 * Assignment helper for TAPF.
 */
#pragma once

#include "tapf_dist_table.hpp"

struct TAPFAssignmentResult {
  std::vector<int> agent_to_task;
  int cost;
  bool feasible;
};

struct TAPFAssignmentStats {
  int calls = 0;
  double time_ms = 0;
};

TAPFAssignmentResult assign_tapf_tasks(
    const TAPFInstance& ins, TAPFDistTable& D, const Config& C,
    const std::vector<int>& previous_assignment = std::vector<int>(),
    const int sticky_penalty = 0,
    TAPFAssignmentStats* stats = nullptr);
