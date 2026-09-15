"""Protected contracts for the dense b9 carrier-vs-carrier_brd page."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
SCRIPT = BENCH / "generate_dense_channel_b9_comparison_report.py"
SPEC = importlib.util.spec_from_file_location("dense_b9_comparison", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DenseChannelB9ComparisonTest(unittest.TestCase):
    def test_baseline_suite_uses_the_identical_instances_and_protocol(self):
        carrier = json.loads(
            (BENCH / "dense_channel_b9_agent_task_benchmark_v1.json")
            .read_text(encoding="utf-8")
        )
        baseline = json.loads(
            (BENCH / "dense_channel_b9_agent_task_brd_benchmark_v1.json")
            .read_text(encoding="utf-8")
        )
        self.assertEqual(baseline["name"], "dense_channel_b9_agent_task_brd_v1_24")
        self.assertEqual(baseline["protocol"]["methods"], ["carrier_brd"])
        for key in (
            "timeout_sec",
            "jobs",
            "solver_seed",
            "objective_weights",
            "following",
        ):
            self.assertEqual(baseline["protocol"][key], carrier["protocol"][key])
        self.assertEqual(baseline["groups"], carrier["groups"])

    def test_protocol_validator_rejects_any_cross_run_mismatch(self):
        base = {
            "n_tasks": 24,
            "jobs": 14,
            "timeout_per_run_sec": 10,
            "solver_seed": 0,
            "objective_weights": {
                "alpha": 1.0,
                "beta": 1.0,
                "gamma": 1.0,
                "delta": 1.0,
            },
            "following": "allowed",
        }
        MODULE.validate_matching_protocol(base, dict(base))
        for key, changed in (
            ("n_tasks", 23),
            ("jobs", 13),
            ("timeout_per_run_sec", 9),
            ("solver_seed", 1),
            ("following", "forbidden"),
        ):
            other = dict(base)
            other[key] = changed
            with self.assertRaisesRegex(ValueError, key):
                MODULE.validate_matching_protocol(base, other)
        other = dict(base)
        other["objective_weights"] = dict(base["objective_weights"])
        other["objective_weights"]["alpha"] = 2.0
        with self.assertRaisesRegex(ValueError, "objective_weights"):
            MODULE.validate_matching_protocol(base, other)

    def test_dense_specialization_keeps_original_ui_but_fixes_labels(self):
        zh = """<title>carrier vs baseline(carrier_brd) — full 509 首解时间与解质量</title>
<a href="index_en.html">English version</a>
<h3>首解时间（ms，log-log）</h3>
carrier 首解时间 (ms) baseline 首解时间 (ms)
<div>对角线下方 = carrier 数值更小。</div>"""
        en = """<title>Carrier vs baseline (carrier_brd) — full-509 first-solution time and solution quality</title>
<a href="index.html">中文版</a>
First-solution time (ms, log-log)
Carrier first-solution time (ms) Baseline first-solution time (ms)
<div>Below the diagonal = a lower Carrier value.</div>"""
        specialized_zh = MODULE.specialize_page(zh, english=False)
        specialized_en = MODULE.specialize_page(en, english=True)
        self.assertIn("dense b9 Agent×Task 24例", specialized_zh)
        self.assertIn("我们首解 / baseline final", specialized_zh)
        self.assertIn("baseline final 时间 (ms)", specialized_zh)
        self.assertIn("carrier 单方法矩阵", specialized_zh)
        self.assertNotIn("full 509", specialized_zh)
        self.assertIn("dense-b9 Agent×Task 24 cases", specialized_en)
        self.assertIn("our first solution / baseline final", specialized_en)
        self.assertIn("Baseline final time (ms)", specialized_en)
        self.assertIn("Carrier-only matrix", specialized_en)
        self.assertNotIn("full-509", specialized_en)

    def test_comparison_uses_the_new_family_instance_root(self):
        self.assertEqual(MODULE.INSTANCE_FAMILY, "dense_channel_b9_agent_task_v1")
        self.assertEqual(
            MODULE.INSTANCE_ROOT,
            BENCH
            / "viz_web"
            / "dense_channel_b9_agent_task_suite_v1_20260914"
            / "instances",
        )
    def test_protocol_validator_rejects_shared_frozen_protocol_deviation(self):
        frozen = {
            "n_tasks": 24,
            "jobs": 14,
            "timeout_per_run_sec": 10,
            "solver_seed": 0,
            "objective_weights": {
                "alpha": 1.0,
                "beta": 1.0,
                "gamma": 1.0,
                "delta": 1.0,
            },
            "following": "allowed",
        }
        for key, changed in (
            ("n_tasks", 23),
            ("jobs", 1),
            ("timeout_per_run_sec", 99),
            ("solver_seed", 7),
            ("following", "forbidden"),
        ):
            bad = dict(frozen)
            bad[key] = changed
            with self.assertRaisesRegex(ValueError, key):
                MODULE.validate_matching_protocol(bad, dict(bad))
        bad = dict(frozen)
        bad["objective_weights"] = dict(frozen["objective_weights"])
        bad["objective_weights"]["alpha"] = 2.0
        with self.assertRaisesRegex(ValueError, "objective_weights"):
            MODULE.validate_matching_protocol(bad, dict(bad))

    def test_rows_must_exactly_match_manifest_family_and_methods(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = root / "manifest.json"
            carrier = root / "carrier.csv"
            baseline = root / "baseline.csv"
            manifest.write_text(
                json.dumps([{"id": "a"}, {"id": "b"}]), encoding="utf-8"
            )
            header = "instance,family,method,success,first_solution_ms,deliverable_ms\n"
            carrier.write_text(
                header
                + "a,dense_channel_b9_agent_task_v1,carrier,0,,\n"
                + "b,dense_channel_b9_agent_task_v1,carrier,0,,\n",
                encoding="utf-8",
            )
            baseline.write_text(
                header
                + "a,dense_channel_b9_agent_task_v1,carrier_brd,0,,\n"
                + "b,dense_channel_b9_agent_task_v1,carrier_brd,0,,\n",
                encoding="utf-8",
            )
            MODULE.validate_exact_rows(carrier, baseline, manifest)
            carrier.write_text(
                header
                + "a,dense_channel_b9_agent_task_v1,carrier,0,,\n"
                + "a,dense_channel_b9_agent_task_v1,carrier,0,,\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate|manifest"):
                MODULE.validate_exact_rows(carrier, baseline, manifest)
            carrier.write_text(
                header
                + "a,dense_channel_b9_agent_task_v1,carrier_brd,0,,\n"
                + "b,dense_channel_b9_agent_task_v1,carrier,0,,\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "method"):
                MODULE.validate_exact_rows(carrier, baseline, manifest)
            carrier.write_text(
                header
                + "a,wrong_family,carrier,0,,\n"
                + "b,dense_channel_b9_agent_task_v1,carrier,0,,\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "family"):
                MODULE.validate_exact_rows(carrier, baseline, manifest)

    def test_successful_baseline_requires_nonempty_equal_final_times(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = Path(tmp) / "rows.csv"
            header = "instance,method,success,first_solution_ms,deliverable_ms\n"
            rows.write_text(
                header + "a,carrier_brd,1,12.5,12.5\n", encoding="utf-8"
            )
            MODULE.validate_baseline_final_times(rows)
            rows.write_text(
                header + "a,carrier_brd,1,,\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "non-empty"):
                MODULE.validate_baseline_final_times(rows)
            rows.write_text(
                header + "a,carrier,1,12.5,12.5\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "method"):
                MODULE.validate_baseline_final_times(rows)

    def test_ratio_and_per_side_time_labels_are_not_generic_first_solution(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / "index.html").write_text(
                "对角线下方 = carrier 数值更小。", encoding="utf-8"
            )
            (out / "index_en.html").write_text(
                "Below the diagonal = a lower Carrier value.", encoding="utf-8"
            )
            MODULE.update_dense_comparison_semantics(out, 0.5)
            zh = (out / "index.html").read_text(encoding="utf-8")
            en = (out / "index_en.html").read_text(encoding="utf-8")
            self.assertIn(
                "时间几何比（carrier 首个全局 goal / baseline final）= 0.500",
                zh,
            )
            self.assertIn(
                "Time geometric ratio (Carrier first global goal / "
                "baseline final) = 0.500",
                en,
            )
            self.assertNotIn("首解时间几何比", zh)
            self.assertNotIn("First-solution geometric ratio", en)

        zh_source = """<title>carrier vs baseline(carrier_brd) — full 509 首解时间与解质量</title>
<a href="index_en.html">English version</a>
<h3>首解时间（ms，log-log）</h3>
carrier 首解时间 (ms) baseline 首解时间 (ms)
`carrier：${f(r.c)}<br>baseline：${f(r.b)}<br>`
function sideInfoText(s) { return s.solved ? `解出：首解 ${s.first} ms ｜ makespan ${s.mk} ｜ soc ${s.soc}` : `10 s 内未解出`; }
sideInfoText(r.c)); sideInfoText(r.b));
<div>对角线下方 = carrier 数值更小。</div>"""
        zh = MODULE.specialize_page(zh_source, english=False)
        self.assertIn("carrier（首个全局 goal）", zh)
        self.assertIn("baseline（final）", zh)
        self.assertIn('sideInfoText(r.c, "carrier")', zh)
        self.assertIn('sideInfoText(r.b, "baseline")', zh)


if __name__ == "__main__":
    unittest.main()
