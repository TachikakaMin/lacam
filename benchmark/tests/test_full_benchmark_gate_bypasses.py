"""Copied manifests and renamed full-only cases remain review-gated."""

import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

from run_benchmark import semantic_case_fingerprint


BENCH = Path(__file__).resolve().parent.parent
REPO = BENCH.parent
FULL_CASES = (
    BENCH
    / "viz_web"
    / "warehouse_case_proposal"
    / "factorial_suite"
    / "instances"
)


class TestFullBenchmarkGateBypasses(unittest.TestCase):
    def _run(self, args):
        return subprocess.run(
            [sys.executable, str(BENCH / "run_benchmark.py")] + args,
            cwd=str(REPO),
            capture_output=True,
            text=True,
            timeout=30,
        )

    def test_instances_path_with_renamed_full_case_requires_review(self):
        source = next(FULL_CASES.glob("*.yaml"))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            family = root / "copied"
            family.mkdir()
            shutil.copyfile(source, family / "renamed.yaml")
            proc = self._run(
                [
                    "--instances",
                    str(root),
                    "--out-dir",
                    str(root / "results"),
                    "--methods",
                    "carrier",
                ]
            )
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("independent review approval", proc.stderr)

    def test_comments_name_and_yaml_reserialization_cannot_bypass(self):
        source = next(FULL_CASES.glob("*.yaml"))
        original = source.read_text(encoding="utf-8")
        data = yaml.safe_load(original)
        without_default_flags = dict(data)
        without_default_flags.pop("flags", None)
        variants = {
            "commented.yaml": "# harmless comment\n" + original,
            "renamed.yaml": yaml.safe_dump(
                {**data, "name": "different-display-name"},
                sort_keys=True,
            ),
            "reserialized.yaml": yaml.safe_dump(data, sort_keys=True),
            "default-flags-omitted.yaml": yaml.safe_dump(
                without_default_flags,
                sort_keys=True,
            ),
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            family = root / "copied"
            family.mkdir()
            for filename, text in variants.items():
                path = family / filename
                path.write_text(text, encoding="utf-8")
                proc = self._run(
                    [
                        "--instances",
                        str(root),
                        "--out-dir",
                        str(root / ("results-" + filename)),
                        "--methods",
                        "carrier",
                    ]
                )
                self.assertNotEqual(proc.returncode, 0, filename)
                self.assertIn(
                    "independent review approval", proc.stderr, filename
                )

    def test_omitted_default_false_flags_preserve_fingerprint(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        data.pop("flags", None)
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "default-flags-omitted.yaml"
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_expanded_shared_goal_pool_preserves_fingerprint(self):
        source = next(
            path
            for path in FULL_CASES.glob("*.yaml")
            if yaml.safe_load(path.read_text(encoding="utf-8")).get(
                "goal_pool"
            )
        )
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        pool = data.pop("goal_pool")
        for target in data["targets"]:
            if target.get("goals") == "pool":
                target["goals"] = pool
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "expanded-goal-pool.yaml"
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_carrier_ignores_added_anon_goal_hint(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        target_starts = {
            tuple(target["start"]) for target in data["targets"]
        }
        anonymous_start = next(
            shelf
            for shelf in data["shelves"]
            if tuple(shelf) not in target_starts
        )
        data["anon_goals"] = [[anonymous_start, anonymous_start]]
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "added-anon-goal-hint.yaml"
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_carrier_ignores_target_display_ids(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        for index, target in enumerate(data["targets"]):
            target["id"] = "renamed-{}".format(index)
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "renamed-target-ids.yaml"
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_target_sequence_remains_part_of_carrier_input(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        data["targets"] = list(reversed(data["targets"]))
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "reordered-targets.yaml"
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertNotEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_numeric_string_coordinates_preserve_fingerprint(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))

        def stringify(cell):
            return [str(value) for value in cell]

        data["robots"] = [stringify(cell) for cell in data["robots"]]
        data["shelves"] = [stringify(cell) for cell in data["shelves"]]
        if data.get("goal_pool"):
            data["goal_pool"] = [
                stringify(cell) for cell in data["goal_pool"]
            ]
        for target in data["targets"]:
            target["start"] = stringify(target["start"])
            if "goal" in target:
                target["goal"] = stringify(target["goal"])
            if isinstance(target.get("goals"), list):
                target["goals"] = [
                    stringify(cell) for cell in target["goals"]
                ]
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "numeric-string-coordinates.yaml"
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_yaml_cpp_octal_string_coordinate_preserves_fingerprint(self):
        source = next(
            path
            for path in FULL_CASES.glob("*.yaml")
            if " 10" in path.read_text(encoding="utf-8")
        )
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        changed_one = False

        def octalize(cell):
            nonlocal changed_one
            result = list(cell)
            if not changed_one:
                for index, value in enumerate(result):
                    if value == 10:
                        result[index] = "012"
                        changed_one = True
                        break
            return result

        data["robots"] = [octalize(cell) for cell in data["robots"]]
        data["shelves"] = [octalize(cell) for cell in data["shelves"]]
        for target in data["targets"]:
            target["start"] = octalize(target["start"])
            if "goal" in target:
                target["goal"] = octalize(target["goal"])
        self.assertTrue(changed_one)
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "yaml-cpp-octal-coordinate.yaml"
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_duplicate_yaml_mapping_keys_are_rejected(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        canonical_robots = yaml.safe_dump(
            {"robots": data["robots"]},
            sort_keys=False,
        )
        changed = dict(data)
        changed["robots"] = list(reversed(data["robots"]))
        ambiguous = canonical_robots + yaml.safe_dump(
            changed,
            sort_keys=False,
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "duplicate-robots-key.yaml"
            path.write_text(ambiguous, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate YAML key"):
                semantic_case_fingerprint(path)

    def test_out_of_bounds_linear_alias_coordinates_are_rejected(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        width = len(data["map"].splitlines()[0])
        row, col = data["robots"][0]
        data["robots"][0] = [row - 1, col + width]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "linear-cell-alias.yaml"
            path.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "coordinate outside grid"):
                semantic_case_fingerprint(path)

    def test_index_keyed_mapping_coordinates_preserve_fingerprint(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))

        def as_mapping(cell):
            return {0: cell[0], 1: cell[1]}

        data["robots"] = [as_mapping(cell) for cell in data["robots"]]
        data["shelves"] = [as_mapping(cell) for cell in data["shelves"]]
        if data.get("goal_pool"):
            data["goal_pool"] = [
                as_mapping(cell) for cell in data["goal_pool"]
            ]
        for target in data["targets"]:
            target["start"] = as_mapping(target["start"])
            if "goal" in target:
                target["goal"] = as_mapping(target["goal"])
            if isinstance(target.get("goals"), list):
                target["goals"] = [
                    as_mapping(cell) for cell in target["goals"]
                ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "mapping-coordinates.yaml"
            path.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(path),
            )

    def test_yaml_cpp_false_flag_spellings_preserve_fingerprint(self):
        source = next(FULL_CASES.glob("*.yaml"))
        original = semantic_case_fingerprint(source)
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for spelling in ("false", "off", "no"):
                variant = dict(data)
                variant["flags"] = {
                    key: spelling for key in data["flags"]
                }
                path = root / ("flags-" + spelling + ".yaml")
                path.write_text(
                    yaml.safe_dump(variant, sort_keys=True),
                    encoding="utf-8",
                )
                self.assertEqual(
                    original,
                    semantic_case_fingerprint(path),
                    spelling,
                )

    def test_numeric_zero_flag_is_rejected_like_yaml_cpp(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        data["flags"] = {"remove_on_complete": 0}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "numeric-zero-flag.yaml"
            path.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                ValueError,
                "unsupported non-default flag",
            ):
                semantic_case_fingerprint(path)

    def test_real_planning_semantic_change_changes_fingerprint(self):
        source = next(FULL_CASES.glob("*.yaml"))
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
        occupied = {tuple(cell) for cell in data["robots"][1:]}
        storage_rows = [
            row for row in data["storage_map"].splitlines() if row
        ]
        replacement = next(
            (r, c)
            for r, row in enumerate(storage_rows)
            for c, char in enumerate(row)
            if char == "." and (r, c) not in occupied
            and [r, c] != data["robots"][0]
        )
        with tempfile.TemporaryDirectory() as tmp:
            changed = Path(tmp) / "changed.yaml"
            data["robots"][0] = list(replacement)
            changed.write_text(
                yaml.safe_dump(data, sort_keys=True),
                encoding="utf-8",
            )
            self.assertNotEqual(
                semantic_case_fingerprint(source),
                semantic_case_fingerprint(changed),
            )

    def test_copied_suite_manifest_requires_review(self):
        source = next(FULL_CASES.glob("*.yaml"))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case = root / "copied.yaml"
            shutil.copyfile(source, case)
            suite = root / "copied_full.json"
            suite.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "name": "renamed_full_probe",
                        "protocol": {
                            "methods": ["carrier"],
                            "timeout_sec": 10,
                            "jobs": 14,
                            "solver_seed": 0,
                            "objective_weights": [1, 1, 1, 1],
                            "following": "allowed",
                        },
                        "groups": [
                            {
                                "name": "copied",
                                "root": ".",
                                "pattern": "copied.yaml",
                                "family": "copied",
                                "expected_cases": 1,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            proc = self._run(
                [
                    "--suite-config",
                    str(suite),
                    "--out-dir",
                    str(root / "results"),
                ]
            )
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("independent review approval", proc.stderr)


if __name__ == "__main__":
    unittest.main()
