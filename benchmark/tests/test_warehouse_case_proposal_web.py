"""Witness-certified warehouse testcase proposal webpage contracts."""

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

from ddbench.instance import load_instance
from ddbench.validator import (
    apply_joint_action,
    initial_state,
    plan_cost,
    validate_plan,
)
from generate_web_viz import parse_plan
from generate_warehouse_case_proposal_web import (
    generate_site,
    proposal_cases,
)

REPO = Path(__file__).resolve().parents[2]
PLANNER_BIN = REPO / "build" / "dd_benchmark"
GENERATOR = REPO / "benchmark" / "generate_warehouse_case_proposal_web.py"


def embedded_samples(page):
    prefix = "const SAMPLES = "
    payload = page.split(prefix, 1)[1].split(";\nconst key", 1)[0]
    return json.loads(payload)


def relation_for(record, start, goal):
    period = record["block_size"] + record["aisle_width"]
    block_cols = record["width"] // period
    source = (start[0] // period) * block_cols + start[1] // period
    destination = (
        (goal[0] // period) * block_cols + goal[1] // period
    )
    sr, sc = divmod(source, block_cols)
    dr, dc = divmod(destination, block_cols)
    distance = abs(sr - dr) + abs(sc - dc)
    if distance == 0:
        return "intra"
    if distance == 1:
        return "adjacent"
    return "remote"


class WarehouseCaseProposalWebTest(unittest.TestCase):
    ORIGINAL_WITNESS_SHA256 = {
        "warehouse_cert_h8w8_b3_a1_s4of9_r2_t4_mixed_seed0":
            "984cd7a26f7b67b1929d900fa9a3581d2fa01bdddfbb9dcf3d521cdd5e012a74",
        "warehouse_cert_h10w10_b4_a1_s12of16_r4_t4_core_seed0":
            "76f64e7f0974aee2d6957fe158b83c442bbfabf16d7b45b647dd3415db095734",
        "warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0":
            "eeaf369e54a348976f4cea3bca749d44c3adf6fe78f2cdc33dfbfbc9f83542dd",
        "warehouse_cert_h8w8_b3_a1_s4of9_r3_t3_pool4_seed0":
            "ba755932e123c2fbbd345919982ba460232b30b5ba59c992614de7a92046ebab",
    }
    ORIGINAL_YAML_SHA256 = {
        "warehouse_cert_h8w8_b3_a1_s4of9_r2_t4_mixed_seed0":
            "140d01e42acc8ba21f3820b734b790f32d581c2f470c15db88137e93487974c8",
        "warehouse_cert_h10w10_b4_a1_s12of16_r4_t4_core_seed0":
            "f1698381f14cfd46bc15bc64d30f921a2887e6115b71a1357b8f721822eaeec0",
        "warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0":
            "880fd60031622c545698928e5ee94768a961ac798607058176f405b62f45d0f7",
        "warehouse_cert_h8w8_b3_a1_s4of9_r3_t3_pool4_seed0":
            "fcfaa1b626e1270bf0ff701746165f0a566e6b0818dea1de10b911401010322b",
    }
    PLANNER_EXPECTED = {
        "warehouse_cert_h8w8_b3_a1_s4of9_r2_t4_mixed_seed0":
            (18, 34.0, 2,
             "f811358103e97beb4c3d7738ee08fda9bcbb078b672f827e7e951afb961855b5"),
        "warehouse_cert_h10w10_b4_a1_s12of16_r4_t4_core_seed0":
            (12, 36.0, 4,
             "e37df74d67ac0d66390c2fe4d05cf0a0c3b506539a1784d8de7864fac4b2a773"),
        "warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0":
            (31, 93.0, 6,
             "8a103b1a80ad24ab5889d1c158c5983009d7719663c28869414b5634e7e52c4d"),
        "warehouse_cert_h8w8_b3_a1_s4of9_r3_t3_pool4_seed0":
            (8, 23.0, 3,
             "0565ea061693c19c06ae4eafbd851bd43aa9b825bba4753bf77881407612a6c9"),
    }

    @unittest.skipUnless(
        PLANNER_BIN.is_file(), "dd_benchmark not built"
    )
    def test_published_planner_runs_match_the_current_binary(self):
        published = (
            REPO
            / "benchmark"
            / "viz_web"
            / "warehouse_case_proposal"
            / "samples.json"
        )
        records = json.loads(published.read_text(encoding="utf-8"))
        current_sha = hashlib.sha256(
            PLANNER_BIN.read_bytes()
        ).hexdigest()
        self.assertEqual(
            {
                record["planner"]["binary_sha256"]
                for record in records
            },
            {current_sha},
        )

    def test_proposal_covers_the_requested_axes(self):
        rows = proposal_cases()
        self.assertEqual(len(rows), 432)
        self.assertEqual(len({row["id"] for row in rows}), 432)
        self.assertEqual(
            {
                (8, 8),
                (12, 12),
                (12, 20),
                (20, 20),
                (24, 24),
                (30, 30),
            },
            {(row["height"], row["width"]) for row in rows},
        )
        self.assertEqual(
            {3, 4, 5, 9},
            {row["block_size"] for row in rows},
        )
        self.assertEqual(
            {"local", "mixed", "cross_heavy"},
            {row["profile"] for row in rows},
        )
        self.assertEqual(
            {"singleton", "shared_pool"},
            {row["goal_mode"] for row in rows},
        )
        self.assertTrue(
            all(
                sum(
                    candidate["map_family"] == family
                    for candidate in rows
                )
                == 72
                for family in {row["map_family"] for row in rows}
            )
        )

    def test_generated_samples_have_valid_witnesses_and_working_links(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "warehouse_case_proposal"
            summary = generate_site(output)

            self.assertEqual(summary["proposal_count"], 432)
            self.assertEqual(summary["factorial_case_count"], 432)
            self.assertEqual(summary["sample_count"], 4)
            self.assertEqual(
                {record["name"] for record in summary["samples"]},
                {
                    "warehouse_cert_h8w8_b3_a1_s4of9_r2_t4_mixed_seed0",
                    "warehouse_cert_h10w10_b4_a1_s12of16_r4_t4_core_seed0",
                    "warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0",
                    "warehouse_cert_h8w8_b3_a1_s4of9_r3_t3_pool4_seed0",
                },
            )

            seen_kinds = set()
            for record in summary["samples"]:
                yaml_path = output / record["yaml"]
                plan_path = output / record["witness"]
                animation_path = output / record["animation"]
                self.assertTrue(yaml_path.is_file())
                self.assertTrue(plan_path.is_file())
                self.assertTrue(animation_path.is_file())

                ins = load_instance(yaml_path)
                plan = parse_plan(plan_path)
                ok, errors, final = validate_plan(ins, plan)
                self.assertTrue(ok, errors)
                self.assertTrue(set(ins.shelves) <= ins.storage_cells)
                self.assertTrue(
                    all(
                        set(target.eligible_goals()).isdisjoint(ins.shelves)
                        for target in ins.targets
                    )
                )

                raw = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
                layout = raw["warehouse_layout"]
                block_size = int(layout["block_size"])
                expected = int(layout["shelves_per_block"])
                counts = []
                for r0, c0 in layout["block_origins"]:
                    block = {
                        (r, c)
                        for r in range(r0, r0 + block_size)
                        for c in range(c0, c0 + block_size)
                    }
                    counts.append(len(block & set(ins.shelves)))
                self.assertEqual(set(counts), {expected})

                profile = raw["task_profile"]
                self.assertEqual(sum(profile["relation_counts"].values()), len(ins.targets))
                seen_kinds.update(
                    kind
                    for kind, count in profile["relation_counts"].items()
                    if count
                )
                animation = animation_path.read_text(encoding="utf-8")
                self.assertIn("requestAnimationFrame", animation)
                self.assertIn("p * p * (3 - 2 * p)", animation)

                state = initial_state(ins)
                drop_cells = []
                action_counts = [
                    {"move": 0, "lift": 0, "drop": 0, "wait": 0}
                    for _ in ins.robots
                ]
                lifted_targets = [[] for _ in ins.robots]
                dropped_targets = [[] for _ in ins.robots]
                for joint in plan:
                    for robot, action in enumerate(joint):
                        action_counts[robot][action[0]] += 1
                        if action[0] == "lift":
                            target_at = {
                                cell: target_id
                                for target_id, cell in state.target_pos
                                if target_id not in state.carried_shelves()
                            }
                            lifted_targets[robot].append(
                                target_at[state.robots[robot]]
                            )
                        if action[0] == "drop":
                            drop_cells.append(state.robots[robot])
                            dropped_targets[robot].append(
                                state.kappa[robot]
                            )
                            self.assertTrue(
                                ins.can_store_shelf(state.robots[robot])
                            )
                    state = apply_joint_action(ins, state, joint)
                self.assertTrue(drop_cells)

                if record["name"] in self.ORIGINAL_WITNESS_SHA256:
                    self.assertEqual(
                        record["witness_sha256"],
                        self.ORIGINAL_WITNESS_SHA256[record["name"]],
                    )

                if record["goal_mode"] == "shared_pool_slack":
                    pool = set(ins.goal_pool)
                    assignments = {
                        cell for _, cell in final.target_pos
                    }
                    self.assertEqual(len(pool), 4)
                    self.assertEqual(len(assignments), 3)
                    self.assertLessEqual(assignments, pool)
                    self.assertEqual(len(pool - assignments), 1)
                    self.assertTrue(pool <= ins.storage_cells)
                    self.assertTrue(
                        all(
                            set(target.eligible_goals()) == pool
                            and len(target.eligible_goals()) == 4
                            for target in ins.targets
                        )
                    )
                    self.assertEqual(record["goals_per_target"], 4)
                    self.assertEqual(record["goal_pool_size"], 4)
                    self.assertEqual(
                        set(map(tuple, record["witness_assignments"])),
                        assignments,
                    )
                    self.assertEqual(
                        set(map(tuple, record["eligible_goal_union"])),
                        pool,
                    )
                    self.assertEqual(
                        raw["goal_pool"],
                        [list(cell) for cell in ins.goal_pool],
                    )
                    self.assertTrue(
                        all(
                            target["goals"] == "pool"
                            and "goal" not in target
                            for target in raw["targets"]
                        )
                    )
                    self.assertEqual(len(plan), 9)
                    self.assertEqual(record["active_robot_count"], 3)
                    self.assertEqual(
                        record["active_robot_indices"], [0, 1, 2]
                    )
                    self.assertEqual(
                        record["robot_activity"], action_counts
                    )
                    self.assertEqual(
                        record["robot_target_ids"], lifted_targets
                    )
                    for robot in range(3):
                        self.assertEqual(
                            action_counts[robot]["lift"], 1
                        )
                        self.assertEqual(
                            action_counts[robot]["drop"], 1
                        )
                        self.assertEqual(
                            lifted_targets[robot],
                            dropped_targets[robot],
                        )
                        self.assertEqual(len(lifted_targets[robot]), 1)
                    self.assertEqual(
                        {
                            target_id
                            for per_robot in lifted_targets
                            for target_id in per_robot
                        },
                        {"b0", "b1", "b2"},
                    )

            self.assertTrue({"intra", "adjacent", "remote"} <= seen_kinds)

            page = (output / "index.html").read_text(encoding="utf-8")
            self.assertIn('lang="zh-CN"', page)
            self.assertIn("432 个随机配对 testcase", page)
            self.assertIn("理论可解", page)
            self.assertIn('id="axisFilter"', page)
            self.assertIn('id="profileFilter"', page)
            self.assertIn('id="sampleFrame"', page)
            self.assertIn("active agents", page)
            self.assertIn("Planner 实跑未生成", page)
            self.assertNotIn("播放认证 witness", page)
            self.assertNotIn("Witness plan", page)
            self.assertNotIn("Witness 动画", page)
            self.assertNotIn("Witness assignment:", page)
            self.assertNotIn("播放 Planner 实跑", page)
            self.assertNotIn("Planner 动画", page)
            self.assertIn(
                'id="openAnimation" href="" target="_blank" hidden',
                page,
            )
            self.assertNotRegex(
                page,
                r'<iframe id="sampleFrame"[^>]+\ssrc=',
            )
            page_samples = embedded_samples(page)
            self.assertEqual(len(page_samples), 4)
            for record in summary["samples"]:
                self.assertIn(record["yaml"], page)
                self.assertNotIn(record["witness"], page)
                self.assertNotIn(record["animation"], page)
            for sample in page_samples:
                self.assertIsNone(sample["planner"])
                self.assertNotIn("witness", sample)
                self.assertNotIn("animation", sample)
                self.assertNotIn("witness_assignments", sample)
                self.assertNotIn("active_robot_count", sample)
                self.assertNotIn("robot_activity", sample)
                self.assertNotIn("robot_target_ids", sample)
                self.assertNotIn("tasks", sample)

    @unittest.skipUnless(
        PLANNER_BIN.is_file(), "dd_benchmark not built"
    )
    def test_planner_runs_are_separate_validated_visualization_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "warehouse_case_proposal"
            summary = generate_site(output, planner_binary=PLANNER_BIN)

            self.assertEqual(summary["planner_run_count"], 4)
            binary_sha = hashlib.sha256(
                PLANNER_BIN.read_bytes()
            ).hexdigest()
            for record in summary["samples"]:
                name = record["name"]
                makespan, soc, active, plan_sha = (
                    self.PLANNER_EXPECTED[name]
                )
                planner = record["planner"]
                self.assertIsNotNone(planner)
                self.assertEqual(planner["seed"], 0)
                self.assertEqual(planner["time_limit_sec"], 10)
                self.assertEqual(planner["mode"], "lacam")
                self.assertEqual(planner["binary_sha256"], binary_sha)
                self.assertEqual(planner["plan_sha256"], plan_sha)
                self.assertNotIn("runtime", planner)

                plan_path = output / planner["plan"]
                animation_path = output / planner["animation"]
                self.assertTrue(plan_path.is_file())
                self.assertTrue(animation_path.is_file())
                self.assertEqual(
                    hashlib.sha256(plan_path.read_bytes()).hexdigest(),
                    plan_sha,
                )

                ins = load_instance(output / record["yaml"])
                plan = parse_plan(plan_path)
                ok, errors, final = validate_plan(ins, plan)
                self.assertTrue(ok, errors)
                metrics = plan_cost(ins, plan)
                self.assertEqual(
                    metrics["executed_makespan"], makespan
                )
                self.assertEqual(metrics["weighted_soc"], soc)
                self.assertEqual(planner["metrics"], metrics)
                self.assertEqual(
                    planner["active_robot_count"], active
                )
                self.assertEqual(
                    planner["active_robot_indices"],
                    [
                        robot
                        for robot, target_ids in enumerate(
                            planner["robot_target_ids"]
                        )
                        if target_ids
                    ],
                )
                self.assertEqual(
                    sorted(
                        target_id
                        for target_ids in planner["robot_target_ids"]
                        for target_id in target_ids
                    ),
                    sorted(target.id for target in ins.targets),
                )
                self.assertEqual(
                    planner["assignments"],
                    {
                        target_id: list(cell)
                        for target_id, cell in final.target_pos
                    },
                )
                self.assertEqual(
                    {
                        task["id"]: task["assignment"]
                        for task in planner["tasks"]
                    },
                    planner["assignments"],
                )
                expected_relations = {
                    "intra": 0,
                    "adjacent": 0,
                    "remote": 0,
                }
                expected_core_goals = 0
                for task in planner["tasks"]:
                    expected_relation = relation_for(
                        record,
                        task["start"],
                        task["assignment"],
                    )
                    self.assertEqual(
                        task["relation"], expected_relation
                    )
                    expected_relations[expected_relation] += 1
                    expected_core_goals += task["goal_depth"] > 0
                self.assertEqual(
                    planner["relations"], expected_relations
                )
                self.assertEqual(
                    planner["core_goals"], expected_core_goals
                )
                eligible_union = {
                    tuple(cell)
                    for cell in record["eligible_goal_union"]
                }
                assigned_cells = {
                    tuple(cell)
                    for cell in planner["assignments"].values()
                }
                self.assertEqual(
                    set(map(tuple, planner["spare_goals"])),
                    eligible_union - assigned_cells,
                )
                self.assertEqual(
                    len(assigned_cells), len(ins.targets)
                )
                for target in ins.targets:
                    goals = set(target.eligible_goals())
                    assignment = tuple(
                        planner["assignments"][target.id]
                    )
                    self.assertIn(assignment, goals)
                    if len(goals) == 1:
                        self.assertEqual(
                            assignment, next(iter(goals))
                        )
                self.assertEqual(
                    hashlib.sha256(
                        (output / record["yaml"]).read_bytes()
                    ).hexdigest(),
                    self.ORIGINAL_YAML_SHA256[name],
                )
                self.assertEqual(
                    hashlib.sha256(
                        (output / record["witness"]).read_bytes()
                    ).hexdigest(),
                    self.ORIGINAL_WITNESS_SHA256[name],
                )

                if record["goal_mode"] == "shared_pool_slack":
                    pool = set(ins.goal_pool)
                    self.assertEqual(assigned_cells <= pool, True)
                    self.assertEqual(len(assigned_cells), 3)
                    witness_by_id = {
                        task["id"]: task["witness_assignment"]
                        for task in record["tasks"]
                    }
                    self.assertNotEqual(
                        planner["assignments"], witness_by_id
                    )

            page = (output / "index.html").read_text(encoding="utf-8")
            self.assertIn("openSample(0,false)", page)
            self.assertIn("播放 Planner 实跑", page)
            self.assertIn("Planner assignment", page)
            self.assertNotIn("播放认证 witness", page)
            self.assertNotIn("Witness plan", page)
            self.assertNotIn("Witness 动画", page)
            self.assertNotIn("Witness assignment:", page)
            self.assertNotIn("t.witness_assignment", page)
            self.assertNotIn("s.witness_assignments", page)
            self.assertIn("s.planner.tasks", page)
            self.assertIn("s.planner.relations", page)
            self.assertIn("s.planner.core_goals", page)
            self.assertIn("s.planner.spare_goals", page)
            page_samples = {
                sample["name"]: sample
                for sample in embedded_samples(page)
            }
            for record in summary["samples"]:
                self.assertIn(record["planner"]["plan"], page)
                self.assertIn(record["planner"]["animation"], page)
                self.assertNotIn(record["witness"], page)
                self.assertNotIn(record["animation"], page)
                display = page_samples[record["name"]]
                self.assertNotIn("witness", display)
                self.assertNotIn("animation", display)
                self.assertNotIn("witness_assignments", display)
                self.assertNotIn("tasks", display)
                self.assertEqual(
                    display["planner"]["assignments"],
                    record["planner"]["assignments"],
                )
                self.assertEqual(
                    display["planner"]["tasks"],
                    record["planner"]["tasks"],
                )
                self.assertEqual(
                    display["planner"]["relations"],
                    record["planner"]["relations"],
                )

    def test_cli_requires_planner_binary_unless_explicitly_disabled(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "missing-dd-benchmark"
            failed = subprocess.run(
                [
                    sys.executable,
                    str(GENERATOR),
                    "--output-dir",
                    str(Path(tmp) / "failed"),
                    "--planner-binary",
                    str(missing),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn("planner binary", failed.stderr)

            output = Path(tmp) / "witness-only"
            passed = subprocess.run(
                [
                    sys.executable,
                    str(GENERATOR),
                    "--output-dir",
                    str(output),
                    "--no-planner-runs",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertEqual(passed.returncode, 0, passed.stderr)
            self.assertTrue((output / "index.html").is_file())
            self.assertIn(
                "planner_runs=0", passed.stdout
            )


if __name__ == "__main__":
    unittest.main()
