#!/usr/bin/env python3
"""Create ITA-CBS-compatible copies of symbotic TAPF fixtures."""

from __future__ import annotations

import argparse
import shutil
from collections import Counter
from pathlib import Path


DEFAULT_FIXTURE_DIRS = (
    "Paper_symbotic_39_37_gp_5",
    "paper_symbotic_39_37_ratio_000",
    "paper_symbotic_39_37_ratio_030",
    "paper_symbotic_39_37_ratio_060",
    "paper_symbotic_39_37_ratio_100",
)


def parse_header_value(lines: list[str], key: str) -> int:
    prefix = f"{key} "
    for line in lines[:4]:
        if line.startswith(prefix):
            return int(line[len(prefix) :])
    raise ValueError(f"missing MovingAI map header value: {key}")


def normalize_map_text(text: str, source: Path) -> tuple[str, Counter[str]]:
    lines = text.splitlines()
    if len(lines) < 4 or lines[3] != "map":
        raise ValueError(f"{source} is not a four-line-header MovingAI map")

    height = parse_header_value(lines, "height")
    width = parse_header_value(lines, "width")
    body = lines[4:]
    if len(body) != height:
        raise ValueError(f"{source} has {len(body)} rows, expected {height}")

    counts: Counter[str] = Counter()
    normalized: list[str] = []
    for row_idx, line in enumerate(body):
        if len(line) != width:
            raise ValueError(
                f"{source} row {row_idx} has width {len(line)}, expected {width}"
            )
        counts.update(line)
        normalized.append("".join(c if c in {"@", "T"} else "." for c in line))

    return "\n".join([*lines[:4], *normalized]) + "\n", counts


def copy_fixture_dir(source: Path, destination: Path, clean: bool) -> dict[str, object]:
    if not source.is_dir():
        raise FileNotFoundError(source)
    if clean and destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True, exist_ok=True)

    yaml_count = 0
    for fixture in source.glob("*.yaml"):
        shutil.copy2(fixture, destination / fixture.name)
        yaml_count += 1

    map_count = 0
    map_chars: Counter[str] = Counter()
    for map_file in source.glob("*.map"):
        normalized_text, counts = normalize_map_text(
            map_file.read_text(encoding="utf-8"),
            map_file,
        )
        (destination / map_file.name).write_text(normalized_text, encoding="utf-8")
        map_chars.update(counts)
        map_count += 1

    return {
        "source": str(source),
        "destination": str(destination),
        "yaml_count": yaml_count,
        "map_count": map_count,
        "map_chars": dict(sorted(map_chars.items())),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path("/media/project0/yimin/lacam_tapf_itacbs_data"),
    )
    parser.add_argument(
        "--out-root",
        type=Path,
        default=Path("build/fixtures/symbotic_itacbs_normalized"),
    )
    parser.add_argument(
        "--fixture-dir",
        action="append",
        default=[],
        help="Fixture directory name under --source-root. Defaults to all symbotic exp1/exp2 dirs.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove each destination directory before copying.",
    )
    args = parser.parse_args()

    fixture_dirs = args.fixture_dir or list(DEFAULT_FIXTURE_DIRS)
    for fixture_dir in fixture_dirs:
        info = copy_fixture_dir(
            args.source_root / fixture_dir,
            args.out_root / fixture_dir,
            args.clean,
        )
        print(
            f"{info['destination']}: yaml={info['yaml_count']} "
            f"maps={info['map_count']} chars={info['map_chars']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
