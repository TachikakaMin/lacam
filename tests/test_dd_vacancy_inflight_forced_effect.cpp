// PROTECTED in-flight custody compilation regression.
// Written before implementation on 2026-09-07 and intentionally observed RED.
#include "../lacam/src/carrier_guidance.hpp"

#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

Op move(const DDInstance& ins, int r, int c)
{
  return Op::make_move(ins.grid.idx(r, c));
}

}  // namespace

TEST(dd_vacancy_inflight_forced_effect,
     corridor_custody_remains_a_shared_task_in_the_next_epoch)
{
  const auto ins = load_dd_instance(
      std::string(DD_TEST_DIR) +
      "/../benchmark/viz_web/warehouse_block_suite/instances/"
      "warehouse_blocks_h20w20_b3_a1_d50_r8_t12_seed0.yaml");
  const std::vector<std::vector<Op>> prefix{
      {
          move(ins, 0, 5), move(ins, 0, 9),
          move(ins, 4, 1), move(ins, 4, 7),
          move(ins, 4, 15), move(ins, 12, 3),
          move(ins, 16, 3), move(ins, 16, 13),
      },
      {
          move(ins, 1, 5), move(ins, 0, 10),
          move(ins, 4, 2), move(ins, 4, 6),
          move(ins, 4, 14), move(ins, 13, 3),
          move(ins, 16, 2), move(ins, 16, 14),
      },
      {
          move(ins, 2, 5), move(ins, 0, 11),
          move(ins, 4, 3), move(ins, 5, 6),
          move(ins, 4, 13), move(ins, 14, 3),
          move(ins, 17, 2), move(ins, 16, 15),
      },
      {
          move(ins, 3, 5), move(ins, 1, 11),
          move(ins, 5, 3), move(ins, 6, 6),
          move(ins, 5, 13), move(ins, 15, 3),
          Op::make_lift(), move(ins, 16, 16),
      },
      {
          move(ins, 3, 4), move(ins, 2, 11),
          move(ins, 6, 3), move(ins, 7, 6),
          move(ins, 6, 13), move(ins, 15, 4),
          move(ins, 16, 2), move(ins, 16, 17),
      },
      {
          move(ins, 3, 3), move(ins, 3, 11),
          move(ins, 7, 3), move(ins, 8, 6),
          Op::make_lift(), move(ins, 15, 5),
          move(ins, 16, 3), move(ins, 17, 17),
      },
      {
          move(ins, 4, 3), move(ins, 4, 11),
          move(ins, 8, 3), move(ins, 9, 6),
          move(ins, 6, 12), move(ins, 16, 5),
          move(ins, 16, 4), move(ins, 18, 17),
      },
  };

  auto physical = initial_phys_config(ins);
  auto guidance =
      dd_task_br_guidance_probe(ins, physical);
  for (const auto& ops : prefix) {
    const auto next = apply_ops(ins, physical, ops);
    ASSERT_TRUE(next.has_value());
    auto next_guidance =
        dd_task_br_guidance_probe(
            ins, *next, &physical, &guidance, &ops);
    physical = *next;
    guidance = std::move(next_guidance);
  }

  ASSERT_NE(guidance.upper_epoch, nullptr);
  const ShelfSelector target0{
      ShelfSelector::Kind::TARGET, 0};
  const TransferKey expected{
      target0, ins.grid.idx(16, 4), ins.grid.idx(17, 5)};
  const auto& tasks =
      guidance.upper_epoch->task_graph.tasks;
  const auto found = std::find_if(
      tasks.begin(), tasks.end(),
      [&](const ShelfTask& task) {
        return carrier_detail::transfer_key(task) == expected;
      });

  ASSERT_NE(found, tasks.end())
      << "an active custody transfer must be seeded into the new epoch "
         "even when its current source is a non-storage corridor cell";
  EXPECT_TRUE(std::binary_search(
      found->roots.begin(), found->roots.end(),
      RootDemand{0, ins.grid.idx(17, 5)}));
  EXPECT_TRUE(std::binary_search(
      found->roots.begin(), found->roots.end(),
      RootDemand{5, ins.grid.idx(15, 6)}));
}
