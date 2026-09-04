"""PROTECTED regression for trailing whitespace in benchmark CSV fields."""

import tempfile
import unittest
from pathlib import Path

from run_benchmark import FIELDS, write_rows


class TestBenchmarkCsvWhitespace(unittest.TestCase):
    def test_rows_csv_strips_field_trailing_whitespace(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rows.csv"
            row = {field: "" for field in FIELDS}
            row["raw"] = "diagnostic with trailing space \n"
            write_rows(path, [row])
            for line in path.read_text().splitlines():
                self.assertEqual(line, line.rstrip())
            self.assertIn("diagnostic with trailing space", path.read_text())


if __name__ == "__main__":
    unittest.main()
