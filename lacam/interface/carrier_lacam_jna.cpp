#include "carrier_lacam_jna.h"

#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>
#include <limits>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

static_assert(
    static_cast<int>(Op::WAIT) == CARRIER_LACAM_ACTION_WAIT,
    "C ABI WAIT value must match Op::Kind");
static_assert(
    static_cast<int>(Op::MOVE) == CARRIER_LACAM_ACTION_MOVE,
    "C ABI MOVE value must match Op::Kind");
static_assert(
    static_cast<int>(Op::LIFT) == CARRIER_LACAM_ACTION_LIFT,
    "C ABI LIFT value must match Op::Kind");
static_assert(
    static_cast<int>(Op::DROP) == CARRIER_LACAM_ACTION_DROP,
    "C ABI DROP value must match Op::Kind");

struct CarrierLacamContext {
  explicit CarrierLacamContext(int value) : seed(value) {}

  int seed = 0;
  std::optional<DDGrid> grid;
  std::vector<uint8_t> storage_mask;
  std::optional<DDInstance> instance;
  std::optional<PhysConfig> state;
  std::unique_ptr<DDPlanningSession> session;

  bool has_result = false;
  int result_status = CARRIER_LACAM_INVALID_STATE;
  DDPlan plan;
  DDStats stats;
  std::string last_error;
  const char* static_error = nullptr;

  void clear_result()
  {
    has_result = false;
    result_status = CARRIER_LACAM_INVALID_STATE;
    plan.clear();
    stats = DDStats();
  }

  void clear_problem()
  {
    grid.reset();
    storage_mask.clear();
    instance.reset();
    state.reset();
    session.reset();
    clear_result();
  }

  void clear_error() noexcept
  {
    last_error.clear();
    static_error = nullptr;
  }

  bool set_error(
      const char* message,
      const char* allocation_fallback) noexcept
  {
    try {
      last_error.assign(message != nullptr ? message : "");
      static_error = nullptr;
      return true;
    } catch (...) {
      last_error.clear();
      static_error = allocation_fallback;
      return false;
    }
  }

  const char* error_message() const noexcept
  {
    return static_error != nullptr
               ? static_error
               : last_error.c_str();
  }
};

CarrierLacamContext* context_of(void* handle)
{
  return static_cast<CarrierLacamContext*>(handle);
}

int fail(
    CarrierLacamContext& context, int status,
    const char* message) noexcept
{
  return context.set_error(message, message)
             ? status
             : CARRIER_LACAM_INTERNAL_ERROR;
}

int fail_exception(
    CarrierLacamContext& context, int status,
    const char* message,
    const char* allocation_fallback) noexcept
{
  context.set_error(message, allocation_fallback);
  return status;
}

template <typename T>
std::vector<T> copy_array(
    const T* data, int count, const char* name)
{
  if (count < 0)
    throw std::invalid_argument(
        std::string(name) + " count is negative");
  if (count == 0) return {};
  if (data == nullptr)
    throw std::invalid_argument(
        std::string(name) + " is null");
  return std::vector<T>(data, data + count);
}

template <typename Function>
int guarded_status(void* handle, Function&& function) noexcept
{
  if (handle == nullptr) return CARRIER_LACAM_INVALID_ARGUMENT;
  auto& context = *context_of(handle);
  try {
    return function(context);
  } catch (const std::invalid_argument& error) {
    return fail_exception(
        context, CARRIER_LACAM_INVALID_ARGUMENT,
        error.what(), "invalid native argument");
  } catch (const std::bad_alloc&) {
    return fail_exception(
        context, CARRIER_LACAM_INTERNAL_ERROR,
        "native allocation failed",
        "native allocation failed");
  } catch (const std::exception& error) {
    return fail_exception(
        context, CARRIER_LACAM_INTERNAL_ERROR,
        error.what(), "native exception");
  } catch (...) {
    return fail_exception(
        context, CARRIER_LACAM_INTERNAL_ERROR,
        "unknown native exception",
        "unknown native exception");
  }
}

bool has_completed_result(const CarrierLacamContext& context)
{
  return context.has_result;
}

int map_solve_status(DDSolveStatus status)
{
  switch (status) {
    case DDSolveStatus::SOLVED:
      return CARRIER_LACAM_OK;
    case DDSolveStatus::EXHAUSTED:
      return CARRIER_LACAM_EXHAUSTED;
    case DDSolveStatus::TIMEOUT:
      return CARRIER_LACAM_TIMEOUT;
    case DDSolveStatus::INVALID:
      return CARRIER_LACAM_INVALID_STATE;
  }
  return CARRIER_LACAM_INTERNAL_ERROR;
}

const char* solve_status_message(int status)
{
  switch (status) {
    case CARRIER_LACAM_OK:
      return "";
    case CARRIER_LACAM_INVALID_ARGUMENT:
      return "invalid Carrier-LaCAM argument";
    case CARRIER_LACAM_INVALID_STATE:
      return "Carrier-LaCAM rejected the physical state";
    case CARRIER_LACAM_EXHAUSTED:
      return "Carrier-LaCAM exhausted the search";
    case CARRIER_LACAM_TIMEOUT:
      return "Carrier-LaCAM timed out";
    case CARRIER_LACAM_CANCELLED:
      return "Carrier-LaCAM was cancelled";
    case CARRIER_LACAM_INTERNAL_ERROR:
      return "Carrier-LaCAM internal error";
  }
  return "unknown Carrier-LaCAM status";
}

bool action_index(
    CarrierLacamContext& context, int timestep, int robot,
    const Op** action)
{
  if (!has_completed_result(context) ||
      context.result_status != CARRIER_LACAM_OK) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no solved Carrier-LaCAM plan is available");
    return false;
  }
  if (timestep < 0 ||
      timestep >= static_cast<int>(context.plan.size())) {
    fail(
        context, CARRIER_LACAM_INVALID_ARGUMENT,
        "timestep is outside the solved plan");
    return false;
  }
  if (robot < 0 ||
      robot >= static_cast<int>(context.plan[timestep].size())) {
    fail(
        context, CARRIER_LACAM_INVALID_ARGUMENT,
        "robot is outside the solved plan");
    return false;
  }
  *action = &context.plan[timestep][robot];
  return true;
}

int64_t integer_metric(
    void* handle, long DDStats::*member)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return static_cast<int64_t>(
      context.stats.*member);
}

double duration_metric(
    void* handle, double DDStats::*member)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.*member;
}

}  // namespace

extern "C" {

int carrier_lacam_abi_version(void)
{
  return 1;
}

void* carrier_lacam_create(int seed)
{
  try {
    return new CarrierLacamContext(seed);
  } catch (...) {
    return nullptr;
  }
}

void carrier_lacam_destroy(void* handle)
{
  try {
    delete context_of(handle);
  } catch (...) {
  }
}

int carrier_lacam_reset(void* handle)
{
  return guarded_status(
      handle, [](CarrierLacamContext& context) -> int {
        context.clear_problem();
        context.clear_error();
        return CARRIER_LACAM_OK;
      });
}

int carrier_lacam_set_grid(
    void* handle, int height, int width,
    const uint8_t* wall_mask, int wall_mask_count,
    const uint8_t* storage_mask, int storage_mask_count)
{
  return guarded_status(
      handle, [&](CarrierLacamContext& context) -> int {
        if (height <= 0 || width <= 0)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "grid dimensions must be positive");
        const int64_t cells =
            static_cast<int64_t>(height) * width;
        if (cells > std::numeric_limits<int>::max())
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "grid cell count exceeds the C ABI limit");
        if (wall_mask_count != cells ||
            storage_mask_count != cells)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "grid mask length does not match height * width");

        auto walls = copy_array(
            wall_mask, wall_mask_count, "wall mask");
        auto storage = copy_array(
            storage_mask, storage_mask_count,
            "storage mask");
        for (int cell = 0; cell < wall_mask_count; ++cell) {
          if (walls[cell] > 1 || storage[cell] > 1)
            return fail(
                context, CARRIER_LACAM_INVALID_ARGUMENT,
                "grid masks must contain only 0 or 1");
          if (walls[cell] && storage[cell])
            return fail(
                context, CARRIER_LACAM_INVALID_ARGUMENT,
                "storage cell overlaps a wall");
        }

        DDGrid grid;
        grid.height = height;
        grid.width = width;
        grid.wall = std::move(walls);
        grid.reset_rectangular_adjacency();
        context.grid = std::move(grid);
        context.storage_mask = std::move(storage);
        context.instance.reset();
        context.state.reset();
        context.session.reset();
        context.clear_result();
        context.clear_error();
        return CARRIER_LACAM_OK;
      });
}

int carrier_lacam_set_undirected_adjacency(
    void* handle,
    const int* offsets, int offset_count,
    const int* destinations, int destination_count)
{
  return guarded_status(
      handle, [&](CarrierLacamContext& context) -> int {
        if (!context.grid.has_value())
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "set_grid must succeed before adjacency");
        const int cell_count = context.grid->size();
        if (offset_count != cell_count + 1)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "adjacency offsets must have cell_count + 1 entries");
        auto copied_offsets = copy_array(
            offsets, offset_count, "adjacency offsets");
        auto copied_destinations = copy_array(
            destinations, destination_count,
            "adjacency destinations");
        if (copied_offsets.front() != 0 ||
            copied_offsets.back() != destination_count)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "adjacency offsets do not span destinations");
        for (int cell = 0; cell < cell_count; ++cell)
          if (copied_offsets[cell] < 0 ||
              copied_offsets[cell] > copied_offsets[cell + 1] ||
              copied_offsets[cell + 1] > destination_count)
            return fail(
                context, CARRIER_LACAM_INVALID_ARGUMENT,
                "adjacency offsets are not monotonic");

        std::vector<std::vector<int>> adjacency(cell_count);
        for (int cell = 0; cell < cell_count; ++cell)
          adjacency[cell].assign(
              copied_destinations.begin() + copied_offsets[cell],
              copied_destinations.begin() +
                  copied_offsets[cell + 1]);

        DDGrid candidate = *context.grid;
        candidate.set_undirected_adjacency(adjacency);
        context.grid = std::move(candidate);
        context.instance.reset();
        context.state.reset();
        context.session.reset();
        context.clear_result();
        context.clear_error();
        return CARRIER_LACAM_OK;
      });
}

int carrier_lacam_set_entities(
    void* handle,
    int robot_count, const int* robot_cells,
    int shelf_count, const int* shelf_cells,
    int target_count, const int* target_shelf_indices,
    const int* goal_offsets, int goal_offset_count,
    const int* goal_cells, int goal_cell_count)
{
  return guarded_status(
      handle, [&](CarrierLacamContext& context) -> int {
        if (!context.grid.has_value())
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "set_grid must succeed before set_entities");
        if (robot_count <= 0 || shelf_count <= 0 ||
            target_count <= 0)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "Carrier sessions require robots, shelves, and targets");
        if (target_count > shelf_count)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "target count exceeds shelf count");
        if (target_count == std::numeric_limits<int>::max() ||
            goal_offset_count != target_count + 1)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "goal offset count must equal target count + 1");

        auto robots = copy_array(
            robot_cells, robot_count, "robot cells");
        auto shelves = copy_array(
            shelf_cells, shelf_count, "shelf cells");
        const auto target_indices = copy_array(
            target_shelf_indices, target_count,
            "target shelf indices");
        const auto offsets = copy_array(
            goal_offsets, goal_offset_count,
            "goal offsets");
        const auto goals = copy_array(
            goal_cells, goal_cell_count, "goal cells");

        if (offsets.front() != 0 ||
            offsets.back() != goal_cell_count)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "goal offsets do not cover the goal cell array");
        for (int target = 0; target < target_count; ++target) {
          if (offsets[target] < 0 ||
              offsets[target] >= offsets[target + 1] ||
              offsets[target + 1] > goal_cell_count)
            return fail(
                context, CARRIER_LACAM_INVALID_ARGUMENT,
                "goal offsets must be monotone with non-empty sets");
        }

        std::unordered_set<int> unique_targets;
        DDInstance instance;
        instance.grid = *context.grid;
        instance.shelf_storage = context.storage_mask;
        instance.robots = std::move(robots);
        instance.shelves = std::move(shelves);
        instance.target_starts.reserve(target_count);
        instance.target_goal_sets.reserve(target_count);
        for (int target = 0; target < target_count; ++target) {
          const int shelf = target_indices[target];
          if (shelf < 0 || shelf >= shelf_count)
            return fail(
                context, CARRIER_LACAM_INVALID_ARGUMENT,
                "target shelf index is out of range");
          if (!unique_targets.insert(shelf).second)
            return fail(
                context, CARRIER_LACAM_INVALID_ARGUMENT,
                "target shelf indices contain a duplicate");
          instance.target_starts.push_back(
              instance.shelves[shelf]);
          instance.target_goal_sets.emplace_back(
              goals.begin() + offsets[target],
              goals.begin() + offsets[target + 1]);
        }
        instance.name = "carrier_lacam_c_api";
        instance.finalize();

        context.instance = std::move(instance);
        context.state.reset();
        context.session.reset();
        context.clear_result();
        context.clear_error();
        return CARRIER_LACAM_OK;
      });
}

int carrier_lacam_set_state(
    void* handle,
    const int* robot_cells, int robot_cell_count,
    const int* target_cells, int target_cell_count,
    const int* anonymous_cells, int anonymous_cell_count,
    const int* kappa, int kappa_count)
{
  return guarded_status(
      handle, [&](CarrierLacamContext& context) -> int {
        if (!context.instance.has_value())
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "set_entities must succeed before set_state");
        if (robot_cell_count !=
                static_cast<int>(
                    context.instance->n_robots()) ||
            target_cell_count !=
                static_cast<int>(
                    context.instance->n_targets()) ||
            kappa_count != robot_cell_count)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "state vector sizes do not match the entities");

        PhysConfig state;
        state.robots = copy_array(
            robot_cells, robot_cell_count,
            "state robot cells");
        state.target_pos = copy_array(
            target_cells, target_cell_count,
            "state target cells");
        state.anon_occ = copy_array(
            anonymous_cells, anonymous_cell_count,
            "state anonymous cells");
        state.kappa = copy_array(
            kappa, kappa_count, "state kappa");
        if (!validate_phys_config_root(
                 *context.instance, state)
                 .valid())
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "invalid Carrier physical state");

        context.state = std::move(state);
        context.session = std::make_unique<DDPlanningSession>(
            *context.instance, *context.state, context.seed);
        context.clear_result();
        context.clear_error();
        return CARRIER_LACAM_OK;
      });
}

int carrier_lacam_solve(void* handle, int timeout_ms)
{
  return guarded_status(
      handle, [&](CarrierLacamContext& context) -> int {
        if (!context.instance.has_value() ||
            !context.state.has_value() ||
            context.session == nullptr)
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "set_grid, set_entities, and set_state are required");
        if (timeout_ms < 0)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "timeout must be non-negative");

        context.clear_result();
        const auto result = context.session->solve(
            static_cast<double>(timeout_ms) / 1000.0,
            &context.stats);
        context.plan = result.plan;
        context.result_status = map_solve_status(result.status);
        context.has_result = true;
        const char* message =
            solve_status_message(context.result_status);
        if (!context.set_error(message, message)) {
          context.result_status =
              CARRIER_LACAM_INTERNAL_ERROR;
          return CARRIER_LACAM_INTERNAL_ERROR;
        }
        return context.result_status;
      });
}

int carrier_lacam_commit_prefix(
    void* handle, int executed_steps,
    const int* observed_robot_cells,
    int observed_robot_cell_count,
    const int* observed_target_cells,
    int observed_target_cell_count,
    const int* observed_anonymous_cells,
    int observed_anonymous_cell_count,
    const int* observed_kappa,
    int observed_kappa_count)
{
  return guarded_status(
      handle, [&](CarrierLacamContext& context) -> int {
        if (context.session == nullptr ||
            !context.instance.has_value() ||
            !context.state.has_value() ||
            !context.has_result ||
            context.result_status != CARRIER_LACAM_OK)
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "a solved Carrier-LaCAM plan is required before commit");
        if (executed_steps <= 0 ||
            executed_steps >
                static_cast<int>(context.plan.size()))
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "executed prefix length is outside the solved plan");
        if (observed_robot_cell_count !=
                static_cast<int>(
                    context.instance->n_robots()) ||
            observed_target_cell_count !=
                static_cast<int>(
                    context.instance->n_targets()) ||
            observed_kappa_count !=
                observed_robot_cell_count)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "observed state vector sizes do not match the entities");

        PhysConfig observed;
        observed.robots = copy_array(
            observed_robot_cells,
            observed_robot_cell_count,
            "observed robot cells");
        observed.target_pos = copy_array(
            observed_target_cells,
            observed_target_cell_count,
            "observed target cells");
        observed.anon_occ = copy_array(
            observed_anonymous_cells,
            observed_anonymous_cell_count,
            "observed anonymous cells");
        observed.kappa = copy_array(
            observed_kappa,
            observed_kappa_count,
            "observed kappa");
        if (!validate_phys_config_root(
                 *context.instance, observed)
                 .valid())
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "invalid observed Carrier physical state");

        const auto status = context.session->commit_prefix(
            static_cast<size_t>(executed_steps), observed);
        if (status == DDCommitStatus::INVALID_PREFIX)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "executed prefix is invalid");
        if (status == DDCommitStatus::NO_SOLVED_PLAN)
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "no solved Carrier-LaCAM plan is available");
        if (status == DDCommitStatus::STATE_MISMATCH)
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "observed state does not match the solved plan prefix");

        context.state = std::move(observed);
        context.clear_result();
        context.clear_error();
        return CARRIER_LACAM_OK;
      });
}

int carrier_lacam_rebase_state(
    void* handle,
    const int* observed_robot_cells,
    int observed_robot_cell_count,
    const int* observed_target_cells,
    int observed_target_cell_count,
    const int* observed_anonymous_cells,
    int observed_anonymous_cell_count,
    const int* observed_kappa,
    int observed_kappa_count)
{
  return guarded_status(
      handle, [&](CarrierLacamContext& context) -> int {
        if (context.session == nullptr ||
            !context.instance.has_value() ||
            !context.state.has_value())
          return fail(
              context, CARRIER_LACAM_INVALID_STATE,
              "set_grid, set_entities, and set_state are required");
        if (observed_robot_cell_count !=
                static_cast<int>(
                    context.instance->n_robots()) ||
            observed_target_cell_count !=
                static_cast<int>(
                    context.instance->n_targets()) ||
            observed_kappa_count !=
                observed_robot_cell_count)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "observed state vector sizes do not match the entities");

        PhysConfig observed;
        observed.robots = copy_array(
            observed_robot_cells,
            observed_robot_cell_count,
            "observed robot cells");
        observed.target_pos = copy_array(
            observed_target_cells,
            observed_target_cell_count,
            "observed target cells");
        observed.anon_occ = copy_array(
            observed_anonymous_cells,
            observed_anonymous_cell_count,
            "observed anonymous cells");
        observed.kappa = copy_array(
            observed_kappa,
            observed_kappa_count,
            "observed kappa");
        if (!validate_phys_config_root(
                 *context.instance, observed)
                 .valid())
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "invalid observed Carrier physical state");

        if (context.session->rebase(observed) !=
            DDRebaseStatus::OK)
          return fail(
              context, CARRIER_LACAM_INVALID_ARGUMENT,
              "invalid observed Carrier physical state");

        context.state = std::move(observed);
        context.clear_result();
        context.clear_error();
        return CARRIER_LACAM_OK;
      });
}

int carrier_lacam_get_status(void* handle)
{
  if (handle == nullptr) return CARRIER_LACAM_INVALID_ARGUMENT;
  const auto& context = *context_of(handle);
  return has_completed_result(context)
             ? context.result_status
             : CARRIER_LACAM_INVALID_STATE;
}

int carrier_lacam_get_timestep_count(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return static_cast<int>(context.plan.size());
}

int carrier_lacam_get_robot_count(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context) ||
      !context.instance.has_value()) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return static_cast<int>(context.instance->n_robots());
}

int carrier_lacam_get_action_kind(
    void* handle, int timestep, int robot)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  const Op* action = nullptr;
  if (!action_index(
          context, timestep, robot, &action))
    return -1;
  return static_cast<int>(action->kind);
}

int carrier_lacam_get_action_destination(
    void* handle, int timestep, int robot)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  const Op* action = nullptr;
  if (!action_index(
          context, timestep, robot, &action))
    return -1;
  return action->kind == Op::MOVE ? action->to : -1;
}

double carrier_lacam_get_first_solution_ms(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.first_solution_ms;
}

double carrier_lacam_get_deliverable_ms(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.deliverable_ms;
}

int64_t carrier_lacam_get_makespan(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context) ||
      context.result_status != CARRIER_LACAM_OK) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no solved Carrier-LaCAM plan is available");
    return -1;
  }
  return static_cast<int64_t>(context.plan.size());
}

int64_t carrier_lacam_get_work_scaled(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context) ||
      context.result_status != CARRIER_LACAM_OK) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no solved Carrier-LaCAM plan is available");
    return -1;
  }
  return context.stats.best_work_scaled;
}

int64_t carrier_lacam_get_pair_cache_hits(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.pair_cache_hits;
}

int64_t carrier_lacam_get_root_pair_cache_misses(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.root_pair_cache_misses;
}

int64_t carrier_lacam_get_changed_pair_edges(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.root_pair_edges_evaluated;
}

int64_t carrier_lacam_get_total_pair_edges(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.root_pair_edges_total;
}

int64_t carrier_lacam_get_reused_pair_edges(void* handle)
{
  if (handle == nullptr) return -1;
  auto& context = *context_of(handle);
  if (!has_completed_result(context)) {
    fail(
        context, CARRIER_LACAM_INVALID_STATE,
        "no Carrier-LaCAM solve result is available");
    return -1;
  }
  return context.stats.root_pair_edges_reused;
}

int64_t carrier_lacam_get_rho_incremental_full_solves(void* handle)
{
  return integer_metric(
      handle, &DDStats::rho_incremental_full_solves);
}

int64_t carrier_lacam_get_rho_incremental_repairs(void* handle)
{
  return integer_metric(
      handle, &DDStats::rho_incremental_repairs);
}

int64_t carrier_lacam_get_rho_incremental_zero_row_reuses(
    void* handle)
{
  return integer_metric(
      handle, &DDStats::rho_incremental_zero_row_reuses);
}

int64_t carrier_lacam_get_rho_incremental_changed_rows(void* handle)
{
  return integer_metric(
      handle, &DDStats::rho_incremental_changed_rows_total);
}

double carrier_lacam_get_rho_bottleneck_ms(void* handle)
{
  return duration_metric(
      handle, &DDStats::rho_bottleneck_time_ms);
}

double carrier_lacam_get_rho_secondary_full_ms(void* handle)
{
  return duration_metric(
      handle, &DDStats::rho_secondary_full_time_ms);
}

double carrier_lacam_get_rho_secondary_repair_ms(void* handle)
{
  return duration_metric(
      handle, &DDStats::rho_secondary_repair_time_ms);
}

double carrier_lacam_get_rho_canonical_ms(void* handle)
{
  return duration_metric(
      handle, &DDStats::rho_canonical_time_ms);
}

const char* carrier_lacam_last_error(void* handle)
{
  static const char invalid_handle[] =
      "invalid Carrier-LaCAM handle";
  if (handle == nullptr) return invalid_handle;
  return context_of(handle)->error_message();
}

}  // extern "C"
