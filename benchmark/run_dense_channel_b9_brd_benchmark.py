#!/usr/bin/env python3
"""Run the frozen dense-b9 carrier_brd suite from a reviewed binary snapshot."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


BENCH = Path(__file__).resolve().parent
REPO = BENCH.parent
RUNNER = BENCH / "run_benchmark.py"
SUITE_CONFIG = BENCH / "dense_channel_b9_agent_task_brd_benchmark_v1.json"
DEFAULT_BINARY = REPO / "build-release" / "dd_benchmark"
DEFAULT_OUTPUT = BENCH / "results_dense_channel_b9_agent_task_brd_v1_20260914"
FROZEN_BINARY_SHA256 = (
    "d7d76608ba49716f6d07e1b841b8aa729ea73666e6611c68885ee5b949dbe49f"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        while True:
            chunk = source.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def validate_reviewed_binary(path: Path) -> None:
    path = Path(path)
    if not path.is_file() or not os.access(str(path), os.X_OK):
        raise ValueError("reviewed benchmark binary is missing or not executable")
    actual = sha256_file(path)
    if actual != FROZEN_BINARY_SHA256:
        raise ValueError(
            "benchmark binary SHA-256 mismatch: {} != {}".format(
                actual, FROZEN_BINARY_SHA256
            )
        )


def build_runner_command(snapshot: Path, output: Path):
    return [
        sys.executable,
        str(RUNNER),
        "--suite-config",
        str(SUITE_CONFIG),
        "--carrier-bin",
        str(snapshot),
        "--out-dir",
        str(output),
    ]


def run_benchmark(binary: Path, output: Path) -> None:
    binary = Path(binary)
    output = Path(output)
    validate_reviewed_binary(binary)
    if output.exists():
        raise ValueError("output directory already exists: {}".format(output))

    with tempfile.TemporaryDirectory(prefix="dense-b9-brd-binary-") as tmp:
        snapshot = Path(tmp) / "dd_benchmark"
        shutil.copyfile(str(binary), str(snapshot))
        os.chmod(str(snapshot), 0o500)
        validate_reviewed_binary(snapshot)
        subprocess.run(
            build_runner_command(snapshot, output),
            cwd=str(REPO),
            check=True,
        )

    timing_path = output / "timing.json"
    if not timing_path.is_file():
        raise RuntimeError("runner did not publish timing.json")
    timing = json.loads(timing_path.read_text(encoding="utf-8"))
    actual = timing.get("provenance", {}).get("binary_sha256")
    if actual != FROZEN_BINARY_SHA256:
        raise RuntimeError("published benchmark used the wrong binary SHA-256")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--carrier-bin", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    run_benchmark(args.carrier_bin, args.out_dir)


if __name__ == "__main__":
    main()
