#!/usr/bin/env python3
"""Verify that the 48 PDF steps consume exactly the page 50-51 BOM."""
import json
import sys
from collections import Counter
from pathlib import Path

BASE = Path(__file__).resolve().parent


def key(item):
    return item["part"], item["color"]


def main():
    bom = json.loads((BASE / "brougham_exact_bom.json").read_text())
    steps = json.loads((BASE / "brougham_exact_steps.json").read_text())
    expected = Counter({key(p): p["count"] for p in bom["parts"]})
    actual = Counter()
    for step in steps["steps"]:
        repeat = step.get("repeat", 1)
        for part in step["add"]:
            actual[key(part)] += repeat * part["count"]

    errors = 0
    for part in sorted(set(expected) | set(actual)):
        if expected[part] != actual[part]:
            errors += 1
            print(f"ERROR {part}: BOM={expected[part]} steps={actual[part]}")
    print(f"BOM parts={sum(expected.values())}, step parts={sum(actual.values())}, "
          f"unique color/part pairs={len(expected)}")
    print("RESULT:", "FAIL" if errors else "PASS")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
