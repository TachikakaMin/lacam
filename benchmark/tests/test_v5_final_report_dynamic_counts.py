import unittest

from generate_v5_final_report import _generate_html


def _metric(better, equal, worse, old, new, ratio):
    return {
        "better": better,
        "equal": equal,
        "worse": worse,
        "base_sum": old,
        "new_sum": new,
        "geometric_ratio": ratio,
    }


class TestV5FinalReportDynamicCounts(unittest.TestCase):
    def test_explanatory_copy_uses_supplied_suite_sizes(self):
        summary = {
            "total": 3,
            "solved": 2,
            "failed": 1,
            "success_rate": 2 / 3,
            "makespan_sum": 15,
            "work_sum": 28,
            "runtime_sum": 11.5,
        }
        comparison = {
            "common": 2,
            "success_sets_equal": True,
            "makespan": _metric(1, 1, 0, 17, 15, 0.9),
            "work": _metric(0, 1, 1, 25, 28, 1.1),
            "lexicographic": {
                "better": 1,
                "equal": 1,
                "worse": 0,
            },
            "plan_hash_changes": 1,
        }
        axis_row = {
            "value": "g1",
            "total": 1,
            "solved": 1,
            "makespan_mean": 5,
            "work_mean": 8,
        }
        data = {
            "full": summary,
            "quick": dict(summary, total=2, solved=1, failed=1),
            "factorial": dict(
                summary,
                total=1,
                solved=1,
                failed=0,
                success_rate=1,
            ),
            "quick_comparison": comparison,
            "historical_comparison": comparison,
            "ablations": [
                {
                    "label": label,
                    "summary": summary,
                    "comparison": comparison,
                }
                for label in ("E1 off", "E2 off", "E4 off")
            ],
            "axes": {
                "map_family": [axis_row],
                "density_level": [
                    dict(axis_row, value="low")
                ],
                "agent_level": [
                    dict(axis_row, value="baseline")
                ],
                "task_profile": [
                    dict(axis_row, value="local")
                ],
                "goal_mode": [
                    dict(axis_row, value="singleton")
                ],
            },
            "failures": [
                {
                    "family": "quick",
                    "status": "timeout",
                    "count": 1,
                }
            ],
            "timing": {
                "wall_time_sec": 12.3,
                "solver_time_sum_sec": 21.5,
                "jobs": 2,
                "timeout_sec": 10,
                "binary_sha256": "a" * 64,
                "suite_sha256": "b" * 64,
                "execution_snapshot": "sealed_linux_memfd",
                "historical_wall_time_sec": 4.2,
                "wall_ratio": 2.9,
            },
            "testcase_c": {
                "id": "testcase-c",
                "makespan": 31,
                "work": 93,
                "binary_sha256": "a" * 64,
                "robots": 2,
                "targets": 3,
                "loaded_moves": 7,
                "free_moves": 11,
                "lift_drop": 4,
                "anon_moves": 2,
                "reversals": 3,
            },
        }
        data["ablations"][-1]["comparison"] = {
            **comparison,
            "work": _metric(0, 0, 3, 25, 31, 1.2),
            "lexicographic": {
                "better": 0,
                "equal": 0,
                "worse": 3,
            },
        }

        page = _generate_html(data, cpp_tests=4, python_tests=5)

        for text in (
            "quick 2",
            "正式 full 3",
            "1 个受保护随机配对案例",
            "全部 3 行",
            "2 个真实方案",
            "3 份 YAML",
            "1 个 factorial case",
            "2 台机器人要搬 3 个目标货架",
            "7 次 loaded move",
            "11 次 free move",
            "4 次 Lift/Drop",
            "2 anonymous move",
            "3 reversal",
            "失去 3 行严格 incumbent 改进",
        ):
            self.assertIn(text, page)
        for stale in (
            "quick 77",
            "正式 full 509",
            "432 个受保护",
            "全部 509 行",
            "479 个真实方案",
            "509 份 YAML",
            "432 个 factorial case",
            "失去 17 行严格 incumbent 改进",
        ):
            self.assertNotIn(stale, page)


if __name__ == "__main__":
    unittest.main()
