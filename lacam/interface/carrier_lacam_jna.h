#ifndef CARRIER_LACAM_JNA_H
#define CARRIER_LACAM_JNA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum CarrierLacamStatus {
  CARRIER_LACAM_OK = 0,
  CARRIER_LACAM_INVALID_ARGUMENT = 1,
  CARRIER_LACAM_INVALID_STATE = 2,
  CARRIER_LACAM_EXHAUSTED = 3,
  CARRIER_LACAM_TIMEOUT = 4,
  CARRIER_LACAM_CANCELLED = 5,
  CARRIER_LACAM_INTERNAL_ERROR = 6,
};

enum CarrierLacamActionKind {
  CARRIER_LACAM_ACTION_WAIT = 0,
  CARRIER_LACAM_ACTION_MOVE = 1,
  CARRIER_LACAM_ACTION_LIFT = 2,
  CARRIER_LACAM_ACTION_DROP = 3,
};

int carrier_lacam_abi_version(void);

void* carrier_lacam_create(int seed);
void carrier_lacam_destroy(void* handle);
int carrier_lacam_reset(void* handle);

int carrier_lacam_set_grid(
    void* handle, int height, int width,
    const uint8_t* wall_mask, int wall_mask_count,
    const uint8_t* storage_mask, int storage_mask_count);

// Optional explicit undirected topology in CSR form. Call after set_grid
// and before set_entities. offsets has grid_cell_count + 1 entries;
// destinations preserves per-cell neighbor order and must be symmetric.
int carrier_lacam_set_undirected_adjacency(
    void* handle,
    const int* offsets, int offset_count,
    const int* destinations, int destination_count);

int carrier_lacam_set_directed_adjacency(
    void* handle,
    const int* offsets, int offset_count,
    const int* destinations, int destination_count);

// target_shelf_indices maps each target identity to one entry in shelf_cells.
// goal_offsets has target_count + 1 entries and indexes goal_cells.
int carrier_lacam_set_entities(
    void* handle,
    int robot_count, const int* robot_cells,
    int shelf_count, const int* shelf_cells,
    int target_count, const int* target_shelf_indices,
    const int* goal_offsets, int goal_offset_count,
    const int* goal_cells, int goal_cell_count);

// kappa uses the Carrier encoding: -1 free, -2 anonymous shelf,
// and non-negative target identity.
int carrier_lacam_set_state(
    void* handle,
    const int* robot_cells, int robot_cell_count,
    const int* target_cells, int target_cell_count,
    const int* anonymous_cells, int anonymous_cell_count,
    const int* kappa, int kappa_count);

int carrier_lacam_solve(void* handle, int timeout_ms);
int carrier_lacam_commit_prefix(
    void* handle, int executed_steps,
    const int* observed_robot_cells,
    int observed_robot_cell_count,
    const int* observed_target_cells,
    int observed_target_cell_count,
    const int* observed_anonymous_cells,
    int observed_anonymous_cell_count,
    const int* observed_kappa,
    int observed_kappa_count);

int carrier_lacam_rebase_state(
    void* handle,
    const int* observed_robot_cells,
    int observed_robot_cell_count,
    const int* observed_target_cells,
    int observed_target_cell_count,
    const int* observed_anonymous_cells,
    int observed_anonymous_cell_count,
    const int* observed_kappa,
    int observed_kappa_count);

// Before the first completed solve, status is INVALID_STATE. A solved
// zero-timestep plan is OK with timestep_count == 0; failed solves also have
// no actions and are distinguished by this status.
int carrier_lacam_get_status(void* handle);
int carrier_lacam_get_timestep_count(void* handle);
int carrier_lacam_get_robot_count(void* handle);
int carrier_lacam_get_action_kind(
    void* handle, int timestep, int robot);
int carrier_lacam_get_action_destination(
    void* handle, int timestep, int robot);

double carrier_lacam_get_first_solution_ms(void* handle);
double carrier_lacam_get_deliverable_ms(void* handle);
int64_t carrier_lacam_get_makespan(void* handle);
int64_t carrier_lacam_get_work_scaled(void* handle);
int64_t carrier_lacam_get_pair_cache_hits(void* handle);
int64_t carrier_lacam_get_root_pair_cache_misses(void* handle);
int64_t carrier_lacam_get_changed_pair_edges(void* handle);
int64_t carrier_lacam_get_total_pair_edges(void* handle);
int64_t carrier_lacam_get_reused_pair_edges(void* handle);
int64_t carrier_lacam_get_rho_incremental_full_solves(void* handle);
int64_t carrier_lacam_get_rho_incremental_repairs(void* handle);
int64_t carrier_lacam_get_rho_incremental_zero_row_reuses(
    void* handle);
int64_t carrier_lacam_get_rho_incremental_changed_rows(void* handle);
double carrier_lacam_get_rho_bottleneck_ms(void* handle);
double carrier_lacam_get_rho_secondary_full_ms(void* handle);
double carrier_lacam_get_rho_secondary_repair_ms(void* handle);
double carrier_lacam_get_rho_canonical_ms(void* handle);

// The returned pointer remains valid until the next call on this handle.
// Callers crossing JNA should copy it immediately.
const char* carrier_lacam_last_error(void* handle);

#ifdef __cplusplus
}
#endif

#endif
