"""CORAL grader for the lacam_agent lifelong throughput task."""

from __future__ import annotations

import csv
import json
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

from coral.grader import TaskGrader
from coral.types import ScoreBundle

SCENARIO = {
    "map": "tests/assets/symbotic_star.map",
    "num_agents": 100,
    "horizon": 400,
    "time_limit_sec": 1.0,
    "goal_set_size": 3,
    "outbound_prob": 0.5,
    "release_interval": 10,
    "debug": 0,
    "planner_anytime": 1,
    "multi_carry_capacity": 2,
    "force_full_assignment": 0,
    "pickup_service_duration": 4,
    "delivery_service_duration": 4,
}

ALGORITHM_DEFAULT_SENTINEL = -1
HIDDEN_REAL_SEEDS = [1538, 2703, 4832, 4863, 6031, 6240, 6265, 7915, 7933, 9850]
DEFAULT_TUNE_SEEDS = list(range(10))


class Grader(TaskGrader):
    """Build and score `build/lifelong_benchmark` on the fixed scenario."""

    def describe_tune(self) -> str:
        tune_seeds = self.args.get("tune_seeds", DEFAULT_TUNE_SEEDS)
        return (
            f"Tune mode runs public diagnostic seeds {tune_seeds} instead of "
            "the hidden real-mode seed set. It uses the same fixed horizon=400 scenario and "
            "same benchmark binary, so it is useful for screening hypotheses, "
            "but the official leaderboard score is the normal eval over all "
            "real-mode seeds."
        )

    def evaluate(self) -> ScoreBundle:
        repo = Path(self.codebase_path)
        log_dir = self.eval_logs_dir
        log_dir.mkdir(parents=True, exist_ok=True)

        required = [
            repo / "CMakeLists.txt",
            repo / "tools" / "lifelong_benchmark.cpp",
            repo / SCENARIO["map"],
        ]
        missing = [str(path.relative_to(repo)) for path in required if not path.exists()]
        if missing:
            return self.fail(
                "Missing lacam_agent files: " + ", ".join(missing),
                feedback="Run CORAL with workspace.repo_path pointing at the lacam_agent root.",
            )

        submodule_result = _ensure_submodules(repo, log_dir, self.timeout)
        if submodule_result is not None:
            return submodule_result

        build_result = _build_benchmark(repo, log_dir, self.timeout)
        if build_result is not None:
            return build_result

        seeds = _evaluation_seeds(self.tune, self.args)
        reveal_seeds = self.tune

        binary = _build_dir(repo) / "lifelong_benchmark"
        map_path = repo / SCENARIO["map"]
        if not map_path.exists():
            return self.fail(
                "Missing fixed benchmark map.",
                feedback=f"Expected map at {map_path.relative_to(repo)}.",
            )

        rows: list[dict[str, str]] = []
        failures: list[str] = []
        workers = max(1, min(len(seeds), os.cpu_count() or 1, 16))
        private_runner_root: Path | None = None
        work_log_dir = log_dir
        if not reveal_seeds:
            private_runner_root = Path(tempfile.mkdtemp(prefix="lacam_real_eval_"))
            work_log_dir = private_runner_root
        try:
            cmd = _runner_command(
                repo=repo,
                binary=binary,
                log_dir=work_log_dir,
                seeds=seeds,
                workers=workers,
            )
            result = subprocess.run(
                cmd,
                cwd=repo,
                capture_output=True,
                text=True,
                timeout=self.timeout,
                env=_clean_env(),
            )
            (log_dir / "runner.stdout.txt").write_text(
                _redact_seed_values(result.stdout, seeds, reveal_seeds)
            )
            (log_dir / "runner.stderr.txt").write_text(
                _redact_seed_values(result.stderr, seeds, reveal_seeds)
            )
            (log_dir / "commands.json").write_text(
                json.dumps([_public_command(cmd, reveal_seeds)], indent=2)
            )
            if result.returncode != 0:
                detail = (result.stderr or result.stdout).strip()[-2000:]
                failures.append(
                    f"parallel benchmark runner exited {result.returncode}: "
                    f"{_redact_seed_values(detail, seeds, reveal_seeds)}"
                )
            else:
                runner_csv = work_log_dir / "runner" / "all_results.csv"
                if runner_csv.exists():
                    with runner_csv.open(newline="") as handle:
                        rows = list(csv.DictReader(handle))
                    rows.sort(key=lambda row: int(row.get("seed", row.get("seed_requested", "0"))))
                    if reveal_seeds:
                        _write_rows_csv(log_dir / "rows.csv", rows)
                        _mirror_seed_logs(log_dir, rows)
        finally:
            if private_runner_root is not None:
                shutil.rmtree(private_runner_root, ignore_errors=True)

        if failures:
            return self.fail(
                "Benchmark failed before producing valid score.",
                feedback=_feedback(log_dir, seeds, rows, failures, reveal_seeds=reveal_seeds),
            )
        if len(rows) != len(seeds):
            return self.fail(
                f"Expected {len(seeds)} CSV rows, found {len(rows)}.",
                feedback=_feedback(
                    log_dir,
                    seeds,
                    rows,
                    ["CSV row count mismatch"],
                    reveal_seeds=reveal_seeds,
                ),
            )

        invalid = [row for row in rows if row.get("valid") not in {"1", "true", "True"}]
        if invalid:
            if reveal_seeds:
                errors = [
                    f"seed {row.get('seed')}: {row.get('error', '').strip()}"
                    for row in invalid
                ]
            else:
                errors = ["one or more hidden held-out cases were invalid"]
            return self.fail(
                "One or more benchmark rows were invalid.",
                feedback=_feedback(log_dir, seeds, rows, errors, reveal_seeds=reveal_seeds),
            )

        summary = _summarize(
            rows,
            seeds=seeds,
            mode="tune" if self.tune else "real",
            reveal_seeds=reveal_seeds,
        )
        (log_dir / "summary.json").write_text(json.dumps(summary, indent=2))

        score = float(summary["mean_throughput"])
        seed_label = f"seeds {seeds}" if reveal_seeds else f"{len(seeds)} hidden held-out seeds"
        explanation = (
            f"mean throughput={score:.6f} over {seed_label} | "
            f"mean completed={summary['mean_completed_tasks']:.3f} | "
            f"mean alternating throughput={summary['mean_alternating_throughput']:.6f} | "
            f"mean planner runtime={summary['mean_total_planner_runtime']:.3f} ms"
        )
        return self.score(
            score,
            explanation,
            feedback=_feedback(log_dir, seeds, rows, [], reveal_seeds=reveal_seeds),
            metadata=summary,
        )


def _build_benchmark(repo: Path, log_dir: Path, timeout: int | None) -> ScoreBundle | None:
    build_dir = _build_dir(repo)
    configure = subprocess.run(
        ["cmake", "-B", str(build_dir), "-DCMAKE_BUILD_TYPE=Release"],
        cwd=repo,
        capture_output=True,
        text=True,
        timeout=timeout,
        env=_clean_env(),
    )
    (log_dir / "cmake_configure.stdout.txt").write_text(configure.stdout)
    (log_dir / "cmake_configure.stderr.txt").write_text(configure.stderr)
    if configure.returncode != 0:
        return _failed_bundle(
            "CMake configure failed.",
            log_dir,
            (configure.stderr or configure.stdout).strip()[-2000:],
        )

    jobs = str(min(os.cpu_count() or 2, 16))
    build = subprocess.run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--target",
            "lifelong_benchmark",
            "-j",
            jobs,
        ],
        cwd=repo,
        capture_output=True,
        text=True,
        timeout=timeout,
        env=_clean_env(),
    )
    (log_dir / "cmake_build.stdout.txt").write_text(build.stdout)
    (log_dir / "cmake_build.stderr.txt").write_text(build.stderr)
    if build.returncode != 0:
        return _failed_bundle(
            "lifelong_benchmark build failed.",
            log_dir,
            (build.stderr or build.stdout).strip()[-2000:],
        )
    return None


def _ensure_submodules(repo: Path, log_dir: Path, timeout: int | None) -> ScoreBundle | None:
    required = [
        repo / "third_party" / "argparse" / "CMakeLists.txt",
        repo / "third_party" / "googletest" / "CMakeLists.txt",
    ]
    if all(path.exists() for path in required):
        return None

    result = subprocess.run(
        ["git", "submodule", "update", "--init", "third_party/argparse", "third_party/googletest"],
        cwd=repo,
        capture_output=True,
        text=True,
        timeout=timeout,
        env=_clean_env(),
    )
    (log_dir / "git_submodule.stdout.txt").write_text(result.stdout)
    (log_dir / "git_submodule.stderr.txt").write_text(result.stderr)
    if result.returncode != 0:
        return _failed_bundle(
            "Required submodule initialization failed.",
            log_dir,
            (result.stderr or result.stdout).strip()[-2000:],
        )
    missing = [str(path.relative_to(repo)) for path in required if not path.exists()]
    if missing:
        return _failed_bundle(
            "Required submodules are still missing after initialization.",
            log_dir,
            ", ".join(missing),
        )
    return None


def _build_dir(repo: Path) -> Path:
    return repo / "build" / "coral_eval"


def _evaluation_seeds(tune: bool, args: dict[str, Any]) -> list[int]:
    if tune:
        seeds = args.get("tune_seeds", DEFAULT_TUNE_SEEDS)
        return [int(seed) for seed in seeds]
    return HIDDEN_REAL_SEEDS[:]


def _failed_bundle(explanation: str, log_dir: Path, detail: str) -> ScoreBundle:
    from coral.types import Score, ScoreBundle

    return ScoreBundle(
        scores={"eval": Score(value=None, name="eval", explanation=explanation)},
        aggregated=None,
        feedback=f"{explanation}\n\n{detail}\n\nEval logs: {_log_hint(log_dir)}",
    )


def _runner_command(
    *,
    repo: Path,
    binary: Path,
    log_dir: Path,
    seeds: list[int],
    workers: int,
) -> list[str]:
    runner = repo / "tools" / "run_symbotic_requested_grid.py"
    return [
        sys.executable,
        str(runner),
        "--binary",
        str(binary),
        "--out-dir",
        str(log_dir / "runner"),
        "--maps",
        "symbotic_star",
        "--ks",
        str(SCENARIO["multi_carry_capacity"]),
        "--slots",
        str(ALGORITHM_DEFAULT_SENTINEL),
        "--agent-counts",
        str(SCENARIO["num_agents"]),
        "--dists",
        "50_50",
        "--durations",
        str(SCENARIO["pickup_service_duration"]),
        "--cost-modes",
        str(ALGORITHM_DEFAULT_SENTINEL),
        "--horizon",
        str(SCENARIO["horizon"]),
        "--seeds",
        ",".join(str(seed) for seed in seeds),
        "--time-limit-sec",
        str(SCENARIO["time_limit_sec"]),
        "--goal-set-size",
        str(SCENARIO["goal_set_size"]),
        "--release-interval",
        str(SCENARIO["release_interval"]),
        "--service-commit-agents",
        str(ALGORITHM_DEFAULT_SENTINEL),
        "--timeout-sec",
        str(max(1, 0 if workers <= 0 else 1800)),
        "--workers",
        str(workers),
        "--force",
    ]


def _public_command(cmd: list[str], reveal_seeds: bool) -> list[str]:
    if reveal_seeds:
        return cmd
    redacted = list(cmd)
    for flag in ("--seeds", "--out-dir"):
        if flag in redacted:
            value_index = redacted.index(flag) + 1
            if value_index < len(redacted):
                redacted[value_index] = "<hidden-real-eval>" if flag == "--out-dir" else "<hidden-real-seeds>"
    return redacted


def _redact_seed_values(text: str, seeds: list[int], reveal_seeds: bool) -> str:
    if reveal_seeds:
        return text
    redacted = text
    for seed in seeds:
        redacted = redacted.replace(str(seed), "<hidden-seed>")
    return redacted


def _write_rows_csv(path: Path, rows: list[dict[str, str]]) -> None:
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _mirror_seed_logs(log_dir: Path, rows: list[dict[str, str]]) -> None:
    for row in rows:
        seed = row.get("seed_requested") or row.get("seed")
        if seed is None:
            continue
        run_label = (
            f"map-{row.get('map', 'symbotic_star')}__k-{row.get('k', '2')}"
            f"__slot-{row.get('slot', '5')}__agents-{row.get('agents', '100')}"
            f"__dist-{row.get('dist_label', '50_50')}"
            f"__dur-{row.get('duration', '4')}__cost-{row.get('cost_mode', '0')}"
            f"__seed-{seed}"
        )
        run_dir = log_dir / "runner" / "runs" / run_label
        for stream in ("stdout", "stderr"):
            source = run_dir / f"{stream}.txt"
            if source.exists():
                (log_dir / f"seed_{seed}.{stream}.txt").write_text(source.read_text())


def _benchmark_command(
    binary: Path,
    map_path: Path,
    seed: int,
    rows_csv: Path,
    cache_path: Path,
) -> list[Any]:
    return [
        binary,
        map_path,
        SCENARIO["num_agents"],
        SCENARIO["horizon"],
        seed,
        rows_csv,
        cache_path,
        SCENARIO["time_limit_sec"],
        SCENARIO["goal_set_size"],
        SCENARIO["outbound_prob"],
        SCENARIO["release_interval"],
        SCENARIO["debug"],
        "",
        SCENARIO["planner_anytime"],
        SCENARIO["multi_carry_capacity"],
        SCENARIO["force_full_assignment"],
        ALGORITHM_DEFAULT_SENTINEL,
        ALGORITHM_DEFAULT_SENTINEL,
        SCENARIO["pickup_service_duration"],
        SCENARIO["delivery_service_duration"],
        ALGORITHM_DEFAULT_SENTINEL,
    ]


def _summarize(
    rows: list[dict[str, str]],
    *,
    seeds: list[int],
    mode: str,
    reveal_seeds: bool,
) -> dict[str, Any]:
    def values(name: str) -> list[float]:
        return [float(row[name]) for row in rows]

    def mean(name: str) -> float:
        return statistics.fmean(values(name))

    summary: dict[str, Any] = {
        "mode": mode,
        "seed_count": len(seeds),
        "scenario": SCENARIO,
        "mean_throughput": mean("throughput"),
        "mean_completed_tasks": mean("completed_tasks"),
        "mean_alternating_throughput": mean("alternating_throughput"),
        "mean_total_planner_runtime": mean("total_planner_runtime"),
        "mean_total_assignment_runtime": mean("total_assignment_runtime"),
        "mean_average_delivery_time": mean("average_delivery_time"),
        "mean_pickup_while_loaded_count": mean("pickup_while_loaded_count"),
        "min_throughput": min(values("throughput")),
        "max_throughput": max(values("throughput")),
    }
    if reveal_seeds:
        summary["seeds"] = seeds
        summary["rows"] = rows
    else:
        summary["hidden_seed_count"] = len(seeds)
    return summary


def _feedback(
    log_dir: Path,
    seeds: list[int],
    rows: list[dict[str, str]],
    failures: list[str],
    *,
    reveal_seeds: bool,
) -> str:
    lines = [
        f"Eval logs: {_log_hint(log_dir)}",
    ]
    if reveal_seeds:
        lines.extend(
            [
                f"Seeds: {seeds}",
                "",
                "Per-seed results:",
            ]
        )
        if rows:
            lines.append(
                "seed | valid | throughput | completed | alternating | planner_ms | assign_ms"
            )
            for row in rows:
                lines.append(
                    " | ".join(
                        [
                            row.get("seed", "?"),
                            row.get("valid", "?"),
                            row.get("throughput", "?"),
                            row.get("completed_tasks", "?"),
                            row.get("alternating_throughput", "?"),
                            row.get("total_planner_runtime", "?"),
                            row.get("total_assignment_runtime", "?"),
                        ]
                    )
                )
        else:
            lines.append("(no CSV rows parsed)")
    else:
        lines.extend(
            [
                f"Hidden held-out seed set: {len(seeds)} cases",
                "Per-case real-mode rows and seed logs are withheld to avoid seed leakage.",
            ]
        )
    if failures:
        lines.extend(["", "Failures:"])
        lines.extend(f"- {failure}" for failure in failures)
    lines.extend(
        [
            "",
            "Keep one-off diagnostics under hl_agent/runs/ in your worktree.",
            "Do not change the fixed scenario or tune seed set to improve this score.",
        ]
    )
    return "\n".join(lines)


def _log_hint(log_dir: Path) -> str:
    parts = log_dir.parts
    try:
        idx = parts.index("eval_logs")
    except ValueError:
        return str(log_dir)
    return str(Path(*parts[idx:]))


def _clean_env() -> dict[str, str]:
    env = os.environ.copy()
    env.pop("VIRTUAL_ENV", None)
    env.setdefault("LC_ALL", "C")
    return env
