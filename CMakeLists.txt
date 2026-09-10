cmake_minimum_required(VERSION 3.16)
project(lacam-project CXX)

option(
  CARRIER_LACAM_PORTABLE_ONLY
  "Build only the YAML-free Carrier-LaCAM shared library"
  OFF)
if(CARRIER_LACAM_PORTABLE_ONLY)
  set(CARRIER_LACAM_BUILD_NATIVE OFF)
endif()
add_subdirectory(./lacam)
if(CARRIER_LACAM_PORTABLE_ONLY)
  return()
endif()

add_subdirectory(./third_party/argparse)
add_subdirectory(./third_party/googletest)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_executable(main main.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_link_libraries(main lacam argparse)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8)
  target_include_directories(main BEFORE PRIVATE ./compat)
endif()

add_executable(tapf_benchmark tools/tapf_benchmark.cpp)
target_compile_features(tapf_benchmark PUBLIC cxx_std_17)
target_link_libraries(tapf_benchmark lacam)

add_executable(dd_benchmark tools/dd_benchmark.cpp)
target_compile_features(dd_benchmark PUBLIC cxx_std_17)
target_link_libraries(dd_benchmark lacam)

# test
set(TEST_MAIN_FUNC ./third_party/googletest/googletest/src/gtest_main.cc)
set(TEST_ALL_SRC ${TEST_MAIN_FUNC})
macro(add_test name target)
  add_executable(${name} ${target} ${TEST_MAIN_FUNC})
  target_link_libraries(${name} lacam gtest ${YAML_CPP_LIBRARIES})
  target_include_directories(${name} PRIVATE ${YAML_CPP_INCLUDE_DIRS})
  target_link_directories(${name} PRIVATE ${YAML_CPP_LIBRARY_DIRS})
  list(APPEND TEST_ALL_SRC ${target})
endmacro(add_test)

add_compile_definitions(DD_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests")
add_test(test_graph ./tests/test_graph.cpp)
add_test(test_instance ./tests/test_instance.cpp)
add_test(test_dist_table ./tests/test_dist_table.cpp)
add_test(test_planner ./tests/test_planner.cpp)
add_test(test_tapf_planner ./tests/test_tapf_planner.cpp)
add_test(test_tapf_compat ./tests/test_tapf_compat.cpp)
add_test(test_dd_integration ./tests/test_dd_integration.cpp)
add_test(test_post_processing ./tests/test_post_processing.cpp)
add_test(test_dd_carrier ./tests/test_dd_carrier.cpp)
add_test(test_dd_fixed_upper_obstacles
         ./tests/test_dd_fixed_upper_obstacles.cpp)
add_test(test_dd_fixed_upper_schema
         ./tests/test_dd_fixed_upper_schema.cpp)
add_test(test_dd_explicit_undirected_adjacency
         ./tests/test_dd_explicit_undirected_adjacency.cpp)
add_test(test_dd_explicit_high_degree_planner
         ./tests/test_dd_explicit_high_degree_planner.cpp)
add_test(test_dd_directed_adjacency
         ./tests/test_dd_directed_adjacency.cpp)
add_test(test_dd_directed_swap
         ./tests/test_dd_directed_swap.cpp)
add_test(test_dd_directed_schema
         ./tests/test_dd_directed_schema.cpp)
add_test(test_dd_spacetime_commitment
         ./tests/test_dd_spacetime_commitment.cpp)
add_test(test_dd_spacetime_delivery
         ./tests/test_dd_spacetime_delivery.cpp)
add_test(test_dd_spacetime_session
         ./tests/test_dd_spacetime_session.cpp)
add_test(test_dd_spacetime_tick_overflow
         ./tests/test_dd_spacetime_tick_overflow.cpp)
add_test(test_dd_planner ./tests/test_dd_planner.cpp)
add_test(test_dd_arbitrary_root ./tests/test_dd_arbitrary_root.cpp)
add_test(test_dd_incremental_session
         ./tests/test_dd_incremental_session.cpp)
add_test(test_dd_incremental_rebase
         ./tests/test_dd_incremental_rebase.cpp)
add_test(test_dd_incremental_schema
         ./tests/test_dd_incremental_schema.cpp)
add_executable(
  test_carrier_lacam_c_api
  ./tests/test_carrier_lacam_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_adjacency_c_api
  ./tests/test_carrier_lacam_adjacency_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_adjacency_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_directed_adjacency_c_api
  ./tests/test_carrier_lacam_directed_adjacency_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_directed_adjacency_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_spacetime_c_api
  ./tests/test_carrier_lacam_spacetime_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_spacetime_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_fixed_upper_c_api
  ./tests/test_carrier_lacam_fixed_upper_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_fixed_upper_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_spacetime_validation_c_api
  ./tests/test_carrier_lacam_spacetime_validation_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_spacetime_validation_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_spacetime_origin_c_api
  ./tests/test_carrier_lacam_spacetime_origin_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_spacetime_origin_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_incremental_c_api
  ./tests/test_carrier_lacam_incremental_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_incremental_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_rebase_c_api
  ./tests/test_carrier_lacam_rebase_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_rebase_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_rho_metrics_c_api
  ./tests/test_carrier_lacam_rho_metrics_c_api.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_rho_metrics_c_api
  carrier_lacam gtest)
add_executable(
  test_carrier_lacam_exception_boundary
  ./tests/test_carrier_lacam_exception_boundary.cpp
  ${TEST_MAIN_FUNC})
target_link_libraries(
  test_carrier_lacam_exception_boundary
  carrier_lacam gtest)
add_test(test_dd_regression ./tests/test_dd_regression.cpp)
add_test(test_dd_g1 ./tests/test_dd_g1.cpp)
add_test(test_dd_rollout ./tests/test_dd_rollout.cpp)
add_test(test_dd_anytime ./tests/test_dd_anytime.cpp)
add_test(test_dd_m2 ./tests/test_dd_m2.cpp)
add_test(test_dd_golden ./tests/test_dd_golden.cpp)
add_test(test_dd_rewire ./tests/test_dd_rewire.cpp)
add_test(test_dd_nofollow ./tests/test_dd_nofollow.cpp)
add_test(test_dd_goalset ./tests/test_dd_goalset.cpp)
add_test(test_dd_task_br_phase1 ./tests/test_dd_task_br_phase1.cpp)
add_test(test_dd_task_br_pair_kernel ./tests/test_dd_task_br_pair_kernel.cpp)
add_test(test_dd_task_br_final_review ./tests/test_dd_task_br_final_review.cpp)
add_test(test_dd_task_br_joint_progress ./tests/test_dd_task_br_joint_progress.cpp)
add_test(test_dd_task_br_joint_aggregate ./tests/test_dd_task_br_joint_aggregate.cpp)
add_test(test_dd_task_br_exact_matching ./tests/test_dd_task_br_exact_matching.cpp)
add_test(test_dd_task_br_audit ./tests/test_dd_task_br_audit.cpp)
add_test(test_dd_task_br_dense_tail ./tests/test_dd_task_br_dense_tail.cpp)
add_test(test_dd_task_br_blocked_restoration ./tests/test_dd_task_br_blocked_restoration.cpp)
add_test(test_dd_task_br_compiler ./tests/test_dd_task_br_compiler.cpp)
add_test(test_dd_task_br_execution ./tests/test_dd_task_br_execution.cpp)
add_test(test_dd_task_br_production ./tests/test_dd_task_br_production.cpp)
add_test(test_dd_task_br_cleanup ./tests/test_dd_task_br_cleanup.cpp)
add_test(test_dd_objective_benchmark_regression ./tests/test_dd_objective_benchmark_regression.cpp)
add_test(test_dd_objective_dense_repair_regression ./tests/test_dd_objective_dense_repair_regression.cpp)
add_test(test_dd_task_br_two_vacancy_regression ./tests/test_dd_task_br_two_vacancy_regression.cpp)
add_test(test_dd_storage_transfer ./tests/test_dd_storage_transfer.cpp)
add_test(test_dd_vacancy_potential
         ./tests/test_dd_vacancy_potential.cpp)
add_test(test_dd_vacancy_channel_regression
         ./tests/test_dd_vacancy_channel_regression.cpp)
add_test(test_dd_vacancy_goal_source_regression
         ./tests/test_dd_vacancy_goal_source_regression.cpp)
add_test(test_dd_vacancy_duplicate_shelf_regression
         ./tests/test_dd_vacancy_duplicate_shelf_regression.cpp)
add_test(test_dd_vacancy_oscillation_regression
         ./tests/test_dd_vacancy_oscillation_regression.cpp)
add_test(test_dd_vacancy_large_upper_regression
         ./tests/test_dd_vacancy_large_upper_regression.cpp)
add_test(test_dd_vacancy_upper_cache
         ./tests/test_dd_vacancy_upper_cache.cpp)
add_test(test_carrier_brd_candidate_legality
         ./tests/test_carrier_brd_candidate_legality.cpp)
add_test(test_dd_vacancy_cache
         ./tests/test_dd_vacancy_cache.cpp)
add_test(test_dd_pair_incremental
         ./tests/test_dd_pair_incremental.cpp)
add_test(test_pair_staged_lower_bound
         ./tests/test_pair_staged_lower_bound.cpp)
add_test(test_incremental_hungarian_core
         ./tests/test_incremental_hungarian_core.cpp)
add_test(test_pair_incremental_invalid_dual
         ./tests/test_pair_incremental_invalid_dual.cpp)
add_test(test_dd_storage_topology_routes
         ./tests/test_dd_storage_topology_routes.cpp)
add_test(test_dd_vacancy_unit_bfs
         ./tests/test_dd_vacancy_unit_bfs.cpp)
add_test(test_dd_vacancy_churn_telemetry
         ./tests/test_dd_vacancy_churn_telemetry.cpp)
add_test(test_dd_fixed_tau_probe
         ./tests/test_dd_fixed_tau_probe.cpp)
add_test(test_dd_vacancy_continuity
         ./tests/test_dd_vacancy_continuity.cpp)
add_test(test_dd_vacancy_continuity_transition
         ./tests/test_dd_vacancy_continuity_transition.cpp)
add_test(test_dd_vacancy_continuity_fixed_goal_scope
         ./tests/test_dd_vacancy_continuity_fixed_goal_scope.cpp)
add_test(test_dd_vacancy_continuity_stale_key
         ./tests/test_dd_vacancy_continuity_stale_key.cpp)
add_test(test_dd_vacancy_continuity_expiry
         ./tests/test_dd_vacancy_continuity_expiry.cpp)
add_test(test_dd_vacancy_inflight_forced_effect
         ./tests/test_dd_vacancy_inflight_forced_effect.cpp)
add_test(test_dd_vacancy_forced_effect_scope
         ./tests/test_dd_vacancy_forced_effect_scope.cpp)
add_test(test_dd_vacancy_forced_effect_redundancy
         ./tests/test_dd_vacancy_forced_effect_redundancy.cpp)
add_test(test_dd_vacancy_forced_effect_priority_normalization
         ./tests/test_dd_vacancy_forced_effect_priority_normalization.cpp)
add_test(test_dd_vacancy_forced_effect_frontier_normalization
         ./tests/test_dd_vacancy_forced_effect_frontier_normalization.cpp)
add_test(test_dd_vacancy_forced_effect_storage_scope
         ./tests/test_dd_vacancy_forced_effect_storage_scope.cpp)
add_test(test_dd_vacancy_forced_normalization_cache
         ./tests/test_dd_vacancy_forced_normalization_cache.cpp)
add_test(test_dd_vacancy_root_goal_commitment
         ./tests/test_dd_vacancy_root_goal_commitment.cpp)
add_test(test_dd_vacancy_root_commitment_transition
         ./tests/test_dd_vacancy_root_commitment_transition.cpp)
add_test(test_dd_vacancy_carried_goal_commitment
         ./tests/test_dd_vacancy_carried_goal_commitment.cpp)
add_test(test_dd_vacancy_singleton_commitment
         ./tests/test_dd_vacancy_singleton_commitment.cpp)
add_test(test_dd_vacancy_narrow_regression
         ./tests/test_dd_vacancy_narrow_regression.cpp)
add_test(test_dd_vacancy_terminal_delivery_regression
         ./tests/test_dd_vacancy_terminal_delivery_regression.cpp)
add_test(test_dd_terminal_delivery_scope_regression
         ./tests/test_dd_terminal_delivery_scope_regression.cpp)
add_test(test_dd_carrier_vacancy_regression
         ./tests/test_dd_carrier_vacancy_regression.cpp)
add_test(test_dd_storage_transfer_claims ./tests/test_dd_storage_transfer_claims.cpp)
add_test(test_dd_storage_transfer_stats ./tests/test_dd_storage_transfer_stats.cpp)
add_test(test_dd_deliverable ./tests/test_dd_deliverable.cpp)
add_test(test_dd_strict_return_deadline ./tests/test_dd_strict_return_deadline.cpp)
add_test(test_dd_finalization_semantics ./tests/test_dd_finalization_semantics.cpp)
add_test(test_dd_plan_repair ./tests/test_dd_plan_repair.cpp)
add_test(test_dd_plan_cost ./tests/test_dd_plan_cost.cpp)
add_test(test_dd_fixed_point_work ./tests/test_dd_fixed_point_work.cpp)
add_test(test_dd_makespan_heuristic ./tests/test_dd_makespan_heuristic.cpp)
add_test(test_dd_reference_plan ./tests/test_dd_reference_plan.cpp)
add_test(test_dd_transfer_episode ./tests/test_dd_transfer_episode.cpp)
add_test(test_dd_timed_transport ./tests/test_dd_timed_transport.cpp)
add_test(test_dd_causal_preparation ./tests/test_dd_causal_preparation.cpp)
add_test(test_dd_dispatch ./tests/test_dd_dispatch.cpp)
add_test(test_dd_rho_candidates ./tests/test_dd_rho_candidates.cpp)
add_test(test_dd_rho_incremental_v2
         ./tests/test_dd_rho_incremental_v2.cpp)
add_test(test_dd_rho_incremental_deadline
         ./tests/test_dd_rho_incremental_deadline.cpp)
add_test(test_tapf_hungarian_shared ./tests/test_tapf_hungarian_shared.cpp)
add_test(test_lazy_dist ./tests/test_lazy_dist.cpp)
add_test(test_lacam_search_kernel ./tests/test_lacam_search_kernel.cpp)
add_test(test_lacam_search_kernel_cleanup
         ./tests/test_lacam_search_kernel_cleanup.cpp)
add_test(test_carrier_brd_upper ./tests/test_carrier_brd_upper.cpp)
add_test(test_carrier_brd_upper_search
         ./tests/test_carrier_brd_upper_search.cpp)
add_test(test_carrier_brd_upper_partial
         ./tests/test_carrier_brd_upper_partial.cpp)
add_test(test_carrier_brd_frozen_tasks
         ./tests/test_carrier_brd_frozen_tasks.cpp)
add_test(test_carrier_brd_matcher
         ./tests/test_carrier_brd_matcher.cpp)
add_test(test_carrier_brd_event_contract
         ./tests/test_carrier_brd_event_contract.cpp)
add_test(test_carrier_brd_lower_segment
         ./tests/test_carrier_brd_lower_segment.cpp)
add_test(test_carrier_brd_controller
         ./tests/test_carrier_brd_controller.cpp)
add_test(test_carrier_brd_deadline
         ./tests/test_carrier_brd_deadline.cpp)
add_test(test_carrier_brd_budget_probes
         ./tests/test_carrier_brd_budget_probes.cpp)
add_test(test_carrier_brd_materialization_deadline
         ./tests/test_carrier_brd_materialization_deadline.cpp)
add_test(test_carrier_brd_final_cost_deadline
         ./tests/test_carrier_brd_final_cost_deadline.cpp)

add_executable(test_all ${TEST_ALL_SRC})
target_link_libraries(test_all lacam gtest ${YAML_CPP_LIBRARIES})
target_include_directories(test_all PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_directories(test_all PRIVATE ${YAML_CPP_LIBRARY_DIRS})
