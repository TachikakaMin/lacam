import unittest

from generate_v5_final_report import TESTCASE_C_ID, _testcase_c


class TestV5FinalReportSamplesSchema(unittest.TestCase):
    def test_production_samples_name_field_identifies_testcase_c(self):
        record = _testcase_c(
            [
                {
                    "name": TESTCASE_C_ID,
                    "planner": {
                        "binary_sha256": "a" * 64,
                        "metrics": {
                            "executed_makespan": 31,
                            "weighted_soc": 93,
                            "loaded_moves": 49,
                            "free_moves": 32,
                            "lift_drop": 12,
                            "anon_moves": 0,
                            "reversals": 0,
                        },
                    },
                    "robots": 6,
                    "targets": 6,
                }
            ]
        )
        self.assertEqual(record["id"], TESTCASE_C_ID)
        self.assertEqual(record["makespan"], 31)
        self.assertEqual(record["work"], 93)
        self.assertEqual(record["robots"], 6)
        self.assertEqual(record["targets"], 6)
        self.assertEqual(record["loaded_moves"], 49)
        self.assertEqual(record["free_moves"], 32)
        self.assertEqual(record["lift_drop"], 12)
        self.assertEqual(record["anon_moves"], 0)
        self.assertEqual(record["reversals"], 0)


if __name__ == "__main__":
    unittest.main()
