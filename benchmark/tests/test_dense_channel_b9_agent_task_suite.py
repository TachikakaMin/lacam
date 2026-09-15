"""Protected contracts for the 40x40 b9 agent-by-task expansion."""

import hashlib
import json
import unittest
from pathlib import Path

from generate_dense_channel_agent_task_suite import (
    AGENT_COUNTS,
    BLOCK_SIZE,
    HEIGHT,
    INSTANCE_SEED,
    SHELVES_PER_BLOCK,
    TARGET_COUNTS,
    WIDTH,
    build_case_specs,
)
from generate_dense_channel_report import render_dashboard


BENCH = Path(__file__).resolve().parent.parent


class DenseChannelB9AgentTaskSuiteTest(unittest.TestCase):
    def test_case_specs_are_the_frozen_8_by_3_matrix(self):
        specs = build_case_specs()
        self.assertEqual((HEIGHT, WIDTH), (40, 40))
        self.assertEqual((BLOCK_SIZE, SHELVES_PER_BLOCK), (9, 76))
        self.assertEqual(INSTANCE_SEED, 1)
        self.assertEqual(AGENT_COUNTS, (4, 8, 12, 16, 24, 32, 48, 64))
        self.assertEqual(TARGET_COUNTS, (24, 48, 96))
        self.assertEqual(len(specs), 24)
        self.assertEqual(
            {(row["robots"], row["targets"]) for row in specs},
            {
                (robots, targets)
                for robots in AGENT_COUNTS
                for targets in TARGET_COUNTS
            },
        )
        for row in specs:
            self.assertEqual(row["block_size"], 9)
            self.assertEqual(row["shelves_per_block"], 76)
            self.assertEqual(row["seed"], 1)
            self.assertEqual(row["agent_level"], "r{}".format(row["robots"]))
            self.assertEqual(row["task_level"], "t{}".format(row["targets"]))

    def test_suite_config_freezes_protocol_and_independent_family(self):
        config = json.loads(
            (
                BENCH / "dense_channel_b9_agent_task_benchmark_v1.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(config["name"], "dense_channel_b9_agent_task_v1_24")
        self.assertEqual(
            config["protocol"],
            {
                "methods": ["carrier"],
                "timeout_sec": 10,
                "jobs": 14,
                "solver_seed": 0,
                "objective_weights": [1, 1, 1, 1],
                "following": "allowed",
            },
        )
        self.assertEqual(len(config["groups"]), 1)
        group = config["groups"][0]
        self.assertEqual(group["expected_cases"], 24)
        self.assertEqual(group["family"], "dense_channel_b9_agent_task_v1")
        self.assertEqual(
            group["root"],
            "viz_web/dense_channel_b9_agent_task_suite_v1_20260914",
        )
        self.assertEqual(group["pattern"], "instances/*.yaml")

    def test_historical_quick_and_full_manifests_remain_byte_frozen(self):
        expected = {
            "release_benchmark.json":
                "a881292163ff2fcd2797cc83fddd8f9ebd12def79619586b940976bce07111c6",
            "full_benchmark.json":
                "1998283745b17b636c8d363ada77cc083d73183f22aaec7b3d49e49b8fe2b1d6",
        }
        for name, digest in expected.items():
            self.assertEqual(
                hashlib.sha256((BENCH / name).read_bytes()).hexdigest(),
                digest,
            )

    def test_report_lead_and_result_matrix_are_data_driven(self):
        def record(robots, targets, success, makespan):
            return {
                "block_size": 9,
                "success": success,
                "actual_density": 76 / 81,
                "instance": "b9_r{}_t{}".format(robots, targets),
                "density_level": "76/81",
                "start_edge": 0,
                "start_inner": targets,
                "start_deep": targets,
                "goal_edge": targets,
                "goal_inner": 0,
                "first_ms": 100.0 if success else None,
                "return_ms": 200.0 if success else None,
                "first_makespan": makespan,
                "final_makespan": makespan,
                "weighted_soc": 50 if success else None,
                "exit_reason": "SEARCH_EXHAUSTED" if success else "",
                "animation": (
                    "cases/b9_r{}_t{}.html".format(robots, targets)
                    if success else None
                ),
                "preview": "preview.html",
                "yaml": "case.yaml",
                "certificate": "case.plan",
                "target_profile": "interior_to_block_edge_set",
                "robots": robots,
                "targets": targets,
                "goals_per_target": 32,
            }

        records = [
            record(4, 24, True, 10),
            record(4, 96, False, None),
            record(64, 24, True, 8),
            record(64, 96, True, 20),
        ]
        page = render_dashboard(
            {
                "records": records,
                "summary": {
                    "total": 4,
                    "solved": 3,
                    "inner_starts": 240,
                    "deep_starts": 240,
                    "edge_goals": 240,
                    "inner_goals": 0,
                    "target_slots": 240,
                    "min_density": 76 / 81,
                    "max_density": 76 / 81,
                    "min_goals_per_target": 32,
                    "max_goals_per_target": 32,
                    "agent_counts": [4, 64],
                    "target_counts": [24, 96],
                    "wall_time_sec": 1,
                    "timeout_sec": 10,
                    "jobs": 14,
                },
                "timing": {
                    "provenance": {},
                    "suite": {"name": "dense_channel_b9_agent_task_v1_24"},
                },
            }
        )
        self.assertIn("机器人数量：4、64", page)
        self.assertIn("目标任务数：24、96", page)
        self.assertIn("共 4 个配置", page)
        self.assertNotIn("8/12、16/24、32/48 三档递增", page)
        self.assertIn('data-agent="4" data-targets="24"', page)
        self.assertIn('data-agent="4" data-targets="96"', page)
        self.assertIn('data-agent="64" data-targets="96"', page)
        self.assertIn("✓ 10 拍", page)
        self.assertIn("10 秒内未解", page)


if __name__ == "__main__":
    unittest.main()
