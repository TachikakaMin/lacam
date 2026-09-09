// PROTECTED TEST: Carrier-LaCAM native C ABI contract for Code-Labyrinth,
// design_final.md §28 and the integration plan Phase 2. Written before
// implementation (TDD RED).
#include "../lacam/interface/carrier_lacam_jna.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

struct HandleDeleter {
  void operator()(void* handle) const
  {
    carrier_lacam_destroy(handle);
  }
};

using Handle = std::unique_ptr<void, HandleDeleter>;

Handle make_handle(int seed = 0)
{
  return Handle(carrier_lacam_create(seed));
}

void configure_four_action_case(void* handle)
{
  const uint8_t wall[] = {0, 0, 0, 0};
  const uint8_t storage[] = {1, 1, 1, 1};
  ASSERT_EQ(
      carrier_lacam_set_grid(
          handle, 1, 4, wall, 4, storage, 4),
      CARRIER_LACAM_OK);

  const int robots[] = {0, 3};
  const int shelves[] = {1};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {2};
  ASSERT_EQ(
      carrier_lacam_set_entities(
          handle, 2, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1),
      CARRIER_LACAM_OK);

  const int target_cells[] = {1};
  const int kappa[] = {-1, -1};
  ASSERT_EQ(
      carrier_lacam_set_state(
          handle, robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2),
      CARRIER_LACAM_OK);
}

std::vector<int> read_action_kinds(void* handle)
{
  std::vector<int> kinds;
  const int timesteps =
      carrier_lacam_get_timestep_count(handle);
  const int robots = carrier_lacam_get_robot_count(handle);
  for (int timestep = 0; timestep < timesteps; ++timestep)
    for (int robot = 0; robot < robots; ++robot)
      kinds.push_back(
          carrier_lacam_get_action_kind(
              handle, timestep, robot));
  return kinds;
}

}  // namespace

TEST(carrier_lacam_c_api, version_and_null_handle_contract)
{
  EXPECT_EQ(carrier_lacam_abi_version(), 1);
  carrier_lacam_destroy(nullptr);
  EXPECT_EQ(
      carrier_lacam_reset(nullptr),
      CARRIER_LACAM_INVALID_ARGUMENT);
  EXPECT_EQ(
      carrier_lacam_get_status(nullptr),
      CARRIER_LACAM_INVALID_ARGUMENT);
  EXPECT_EQ(carrier_lacam_get_timestep_count(nullptr), -1);
  ASSERT_NE(carrier_lacam_last_error(nullptr), nullptr);
  EXPECT_FALSE(
      std::string(carrier_lacam_last_error(nullptr)).empty());
}

TEST(carrier_lacam_c_api,
     bulk_inputs_are_copied_and_zero_tick_success_is_explicit)
{
  Handle handle = make_handle();
  ASSERT_NE(handle, nullptr);

  uint8_t wall[] = {0, 0, 0, 0};
  uint8_t storage[] = {1, 1, 1, 1};
  ASSERT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 4, wall, 4, storage, 4),
      CARRIER_LACAM_OK);
  wall[1] = 1;
  storage[1] = 0;

  int robots[] = {0, 3};
  int shelves[] = {1};
  int target_shelves[] = {0};
  int goal_offsets[] = {0, 2};
  int goals[] = {1, 2};
  ASSERT_EQ(
      carrier_lacam_set_entities(
          handle.get(), 2, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 2),
      CARRIER_LACAM_OK);
  robots[0] = 99;
  shelves[0] = 99;
  target_shelves[0] = 99;
  goal_offsets[1] = 0;
  goals[0] = 99;

  int state_robots[] = {0, 3};
  int target_cells[] = {1};
  int kappa[] = {-1, -1};
  ASSERT_EQ(
      carrier_lacam_set_state(
          handle.get(), state_robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2),
      CARRIER_LACAM_OK);
  state_robots[0] = 99;
  target_cells[0] = 99;
  kappa[0] = 99;

  EXPECT_EQ(
      carrier_lacam_solve(handle.get(), 1000),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_OK);
  EXPECT_EQ(carrier_lacam_get_timestep_count(handle.get()), 0);
  EXPECT_EQ(carrier_lacam_get_robot_count(handle.get()), 2);
  EXPECT_EQ(carrier_lacam_get_makespan(handle.get()), 0);
  EXPECT_EQ(carrier_lacam_get_work_scaled(handle.get()), 0);
  EXPECT_GE(
      carrier_lacam_get_first_solution_ms(handle.get()), 0);
  EXPECT_GE(
      carrier_lacam_get_deliverable_ms(handle.get()),
      carrier_lacam_get_first_solution_ms(handle.get()));
  EXPECT_STREQ(carrier_lacam_last_error(handle.get()), "");
}

TEST(carrier_lacam_c_api,
     joint_matrix_preserves_wait_move_lift_and_drop)
{
  Handle handle = make_handle();
  ASSERT_NE(handle, nullptr);
  configure_four_action_case(handle.get());

  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  ASSERT_EQ(carrier_lacam_get_timestep_count(handle.get()), 4);
  ASSERT_EQ(carrier_lacam_get_robot_count(handle.get()), 2);

  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 0, 0),
      CARRIER_LACAM_ACTION_MOVE);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(
          handle.get(), 0, 0),
      1);
  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 0, 1),
      CARRIER_LACAM_ACTION_WAIT);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(
          handle.get(), 0, 1),
      -1);

  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 1, 0),
      CARRIER_LACAM_ACTION_LIFT);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(
          handle.get(), 1, 0),
      -1);
  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 2, 0),
      CARRIER_LACAM_ACTION_MOVE);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(
          handle.get(), 2, 0),
      2);
  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), 3, 0),
      CARRIER_LACAM_ACTION_DROP);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(
          handle.get(), 3, 0),
      -1);
  for (int timestep = 1; timestep < 4; ++timestep)
    EXPECT_EQ(
        carrier_lacam_get_action_kind(
            handle.get(), timestep, 1),
        CARRIER_LACAM_ACTION_WAIT);

  EXPECT_EQ(carrier_lacam_get_makespan(handle.get()), 4);
  EXPECT_GE(carrier_lacam_get_work_scaled(handle.get()), 0);
  EXPECT_GE(
      carrier_lacam_get_first_solution_ms(handle.get()), 0);
  EXPECT_GE(
      carrier_lacam_get_deliverable_ms(handle.get()),
      carrier_lacam_get_first_solution_ms(handle.get()));

  EXPECT_EQ(
      carrier_lacam_get_action_kind(handle.get(), -1, 0),
      -1);
  EXPECT_EQ(
      carrier_lacam_get_action_destination(
          handle.get(), 4, 0),
      -1);
  EXPECT_FALSE(
      std::string(carrier_lacam_last_error(handle.get())).empty());
  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_OK);
}

TEST(carrier_lacam_c_api,
     invalid_input_returns_codes_without_crossing_c_boundary)
{
  Handle handle = make_handle();
  ASSERT_NE(handle, nullptr);

  EXPECT_EQ(
      carrier_lacam_solve(handle.get(), 100),
      CARRIER_LACAM_INVALID_STATE);
  EXPECT_FALSE(
      std::string(carrier_lacam_last_error(handle.get())).empty());

  uint8_t wall[] = {0, 0, 0, 0};
  const uint8_t storage[] = {1, 1, 1, 1};
  EXPECT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 4, wall, 3, storage, 4),
      CARRIER_LACAM_INVALID_ARGUMENT);
  wall[2] = 2;
  EXPECT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 4, wall, 4, storage, 4),
      CARRIER_LACAM_INVALID_ARGUMENT);
  wall[2] = 0;
  ASSERT_EQ(
      carrier_lacam_set_grid(
          handle.get(), 1, 4, wall, 4, storage, 4),
      CARRIER_LACAM_OK);

  const int robots[] = {0, 3};
  const int shelves[] = {1};
  const int target_shelves[] = {0};
  const int goal_offsets[] = {0, 1};
  const int goals[] = {2};
  EXPECT_EQ(
      carrier_lacam_set_entities(
          handle.get(), 2, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 1, goals, 1),
      CARRIER_LACAM_INVALID_ARGUMENT);
  ASSERT_EQ(
      carrier_lacam_set_entities(
          handle.get(), 2, robots, 1, shelves, 1,
          target_shelves, goal_offsets, 2, goals, 1),
      CARRIER_LACAM_OK);

  const int target_cells[] = {1};
  const int invalid_kappa[] = {99, -1};
  EXPECT_EQ(
      carrier_lacam_set_state(
          handle.get(), robots, 2, target_cells, 1,
          nullptr, 0, invalid_kappa, 2),
      CARRIER_LACAM_INVALID_ARGUMENT);
  EXPECT_EQ(
      carrier_lacam_solve(handle.get(), 100),
      CARRIER_LACAM_INVALID_STATE);

  const int kappa[] = {-1, -1};
  EXPECT_EQ(
      carrier_lacam_set_state(
          handle.get(), robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_INVALID_STATE);
}

TEST(carrier_lacam_c_api,
     timeout_repeat_solve_and_reset_replace_old_results)
{
  Handle handle = make_handle();
  ASSERT_NE(handle, nullptr);
  configure_four_action_case(handle.get());

  EXPECT_EQ(
      carrier_lacam_solve(handle.get(), 0),
      CARRIER_LACAM_TIMEOUT);
  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_TIMEOUT);
  EXPECT_EQ(carrier_lacam_get_timestep_count(handle.get()), 0);

  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  const auto first = read_action_kinds(handle.get());
  ASSERT_EQ(first.size(), 8u);
  ASSERT_EQ(
      carrier_lacam_solve(handle.get(), 2000),
      CARRIER_LACAM_OK);
  EXPECT_EQ(read_action_kinds(handle.get()), first);

  const int robots[] = {0, 3};
  const int target_cells[] = {1};
  const int kappa[] = {-1, -1};
  ASSERT_EQ(
      carrier_lacam_set_state(
          handle.get(), robots, 2, target_cells, 1,
          nullptr, 0, kappa, 2),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_INVALID_STATE);
  EXPECT_EQ(carrier_lacam_get_timestep_count(handle.get()), -1);
  EXPECT_EQ(carrier_lacam_get_first_solution_ms(handle.get()), -1);
  EXPECT_EQ(carrier_lacam_get_deliverable_ms(handle.get()), -1);

  ASSERT_EQ(
      carrier_lacam_reset(handle.get()),
      CARRIER_LACAM_OK);
  EXPECT_EQ(
      carrier_lacam_get_status(handle.get()),
      CARRIER_LACAM_INVALID_STATE);
  EXPECT_EQ(carrier_lacam_get_robot_count(handle.get()), -1);
  EXPECT_EQ(
      carrier_lacam_solve(handle.get(), 100),
      CARRIER_LACAM_INVALID_STATE);
}
