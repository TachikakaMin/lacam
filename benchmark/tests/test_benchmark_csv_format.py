"""PROTECTED regression for repository-safe benchmark CSV line endings."""

import tempfile
import unittest
from pathlib import Path

from run_benchmark import FIELDS, write_rows


class TestBenchmarkCsvFormat(unittest.TestCase):
    def test_rows_csv_uses_lf_without_carriage_returns(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rows.csv"
            write_rows(path, [{field: "" for field in FIELDS}])
            data = path.read_bytes()
            self.assertTrue(data.endswith(b"\n"))
            self.assertNotIn(b"\r\n", data)


if __name__ == "__main__":
    unittest.main()
