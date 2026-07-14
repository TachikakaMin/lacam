#!/usr/bin/env python3
"""Run a CPU-aware, exact-instance comparison with the IJCAI-26 MAWPF code."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import statistics
import time


# All 12 Figure-3 map scenarios.  One representative paper density is selected
# per map and five seeds are used by default, as requested for the cross-check.
SCENARIOS = {
    "brc202d": 100,
    "empty-32-32": 50,
    "maze-32-32-2": 25,
    "maze-32-32-4": 25,
    "ost003d": 100,
    "random-32-32-20": 50,
    "random-64-64-20": 50,
    "room-32-32-4": 25,
    "room-64-64-16": 50,
    "room-64-64-8": 50,
    "warehouse-20-40-10-2-1": 100,
    "warehouse-20-40-10-2-2": 100,
}


def run(command: list[str], timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        output += f"\nexternal_timeout_sec={timeout}\n"
        return subprocess.CompletedProcess(command, 124, output)


def parse_key_values(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def system_workers(reserve_cpus: int, requested: int | None) -> tuple[int, dict[str, float]]:
    cpus = os.cpu_count() or 1
    load1, load5, load15 = os.getloadavg()
    # Existing runnable work gets first claim; our pool fills the remainder
    # while preserving the explicit reserve.
    safe = max(1, cpus - reserve_cpus - math.ceil(load1))
    mem_available_kib = 0
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            mem_available_kib = int(line.split()[1])
            break
    mem_available_gib = mem_available_kib / 1024 / 1024
    # The reference implementation materializes its path cache in memory; on
    # large game maps a process can use several GiB.  Keep 8 GiB uncommitted
    # and budget 5 GiB per worker to avoid swapping/OOM during mixed-map runs.
    # The largest reference cache in this suite was observed at about 4.1 GiB.
    memory_safe = max(1, int(max(0.0, mem_available_gib - 8.0) / 5.0))
    safe = min(safe, memory_safe)
    workers = min(safe, requested) if requested else safe
    return max(1, workers), {
        "logical_cpus": cpus,
        "load1_before": load1,
        "load5_before": load5,
        "load15_before": load15,
        "reserved_cpus": reserve_cpus,
        "mem_available_gib_before": mem_available_gib,
        "memory_worker_limit": memory_safe,
    }


def build(repo: Path, reference: Path) -> tuple[Path, Path, Path, Path, Path]:
    ours = run(["cmake", "-S", str(repo), "-B", str(repo / "build")])
    if ours.returncode:
        raise RuntimeError(ours.stdout)
    ours = run(["cmake", "--build", str(repo / "build"), "-j", "8", "--target",
                "tapf_benchmark", "map_distance_precompute",
                "motion_path_precompute"])
    if ours.returncode:
        raise RuntimeError(ours.stdout)
    theirs = run(["cmake", "-S", str(reference), "-B", str(reference / "build")])
    if theirs.returncode:
        raise RuntimeError(theirs.stdout)
    theirs = run(["cmake", "--build", str(reference / "build"), "-j", "8", "--target", "main"])
    if theirs.returncode:
        raise RuntimeError(theirs.stdout)
    dumper = repo / "build" / "mawpf_instance_dump"
    compile_dump = run([
        os.environ.get("CXX", "c++"), "-std=c++17", "-O2",
        "-I", str(reference / "algorithms" / "include"),
        str(repo / "tools" / "mawpf_instance_dump.cpp"),
        str(reference / "build" / "algorithms" / "libalgorithms.a"),
        "-o", str(dumper),
    ])
    if compile_dump.returncode:
        raise RuntimeError(compile_dump.stdout)
    return (repo / "build" / "tapf_benchmark", reference / "build" / "main",
            dumper, repo / "build" / "map_distance_precompute",
            repo / "build" / "motion_path_precompute")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=Path("/tmp/mawpf-reference"))
    parser.add_argument("--output", type=Path, default=Path("experiments/mawpf_paper_comparison"))
    parser.add_argument("--cases-per-scenario", type=int, default=5)
    parser.add_argument("--time-limit", type=float, default=1.0)
    parser.add_argument("--process-timeout", type=float, default=120.0,
                        help="outer wall-clock guard for cache loading plus solving")
    parser.add_argument("--reserve-cpus", type=int, default=10)
    parser.add_argument("--workers", type=int)
    parser.add_argument("--horizon", type=int, default=6)
    parser.add_argument("--max-speed", type=int, default=2)
    parser.add_argument("--rotation-steps", type=int, default=2)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--scenario", action="append", choices=sorted(SCENARIOS),
                        help="limit to selected scenario(s); default covers all 12")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    reference = args.reference.resolve()
    output = (repo / args.output).resolve() if not args.output.is_absolute() else args.output
    cases_dir, raw_dir = output / "cases", output / "raw"
    cache_dir = output / "map_cache"
    motion_cache_dir = output / "motion_path_cache"
    cases_dir.mkdir(parents=True, exist_ok=True)
    raw_dir.mkdir(parents=True, exist_ok=True)
    cache_dir.mkdir(parents=True, exist_ok=True)
    motion_cache_dir.mkdir(parents=True, exist_ok=True)
    if args.skip_build:
        ours, theirs = repo / "build/tapf_benchmark", reference / "build/main"
        dumper = repo / "build/mawpf_instance_dump"
        cache_builder = repo / "build/map_distance_precompute"
        motion_cache_builder = repo / "build/motion_path_precompute"
    else:
        ours, theirs, dumper, cache_builder, motion_cache_builder = build(
            repo, reference)

    cases: list[dict[str, object]] = []
    # Generate with the reference constructor before parallel solve execution;
    # this also serializes per-map path-cache creation and avoids cache races.
    selected = args.scenario or list(SCENARIOS)
    workers, host = system_workers(args.reserve_cpus, args.workers)
    precompute_workers = max(
        1, int(host["logical_cpus"]) - args.reserve_cpus -
        math.ceil(host["load1_before"]))
    host["precompute_workers"] = precompute_workers
    cache_paths: dict[str, Path] = {}
    motion_cache_paths: dict[str, Path] = {}
    cache_preprocess_started = time.time()
    for map_name in selected:
        map_path = reference / "maps" / f"{map_name}.map"
        cache_path = cache_dir / f"{map_name}.bin"
        prepared = run([str(cache_builder), str(map_path), str(cache_path),
                        str(precompute_workers)], timeout=3600)
        if prepared.returncode:
            raise RuntimeError(f"map distance preprocessing failed: {map_name}\n{prepared.stdout}")
        cache_paths[map_name] = cache_path
        motion_cache_path = motion_cache_dir / (
            f"{map_name}-v{args.max_speed}-r{args.rotation_steps}-l{args.horizon}.bin")
        motion_prepared = run([
            str(motion_cache_builder), str(map_path), str(motion_cache_path),
            str(args.max_speed), str(args.rotation_steps), str(args.horizon),
            "all", "1,1,1,1,0,0,0", str(precompute_workers),
        ], timeout=3600)
        if motion_prepared.returncode:
            raise RuntimeError(
                f"motion path preprocessing failed: {map_name}\n{motion_prepared.stdout}")
        motion_cache_paths[map_name] = motion_cache_path
    host["map_cache_files"] = len(cache_paths)
    host["map_cache_bytes"] = sum(path.stat().st_size for path in cache_paths.values())
    host["map_cache_prepare_elapsed_sec"] = time.time() - cache_preprocess_started
    host["motion_path_cache_files"] = len(motion_cache_paths)
    host["motion_path_cache_bytes"] = sum(
        path.stat().st_size for path in motion_cache_paths.values())
    for map_name in selected:
        agents = SCENARIOS[map_name]
        map_path = reference / "maps" / f"{map_name}.map"
        for seed in range(args.cases_per_scenario):
            case_id = f"{map_name}-n{agents}-s{seed}"
            yaml = cases_dir / f"{case_id}.yaml"
            if not yaml.exists() or yaml.stat().st_size < 100:
                generated = run([
                    str(dumper), str(map_path), str(agents), str(seed),
                    str(args.horizon), str(args.max_speed), str(args.rotation_steps),
                    str(yaml),
                ], timeout=600)
                if generated.returncode:
                    raise RuntimeError(f"instance generation failed: {case_id}\n{generated.stdout}")
            cases.append({"id": case_id, "map": map_name, "map_path": map_path,
                          "agents": agents, "seed": seed, "yaml": yaml})

    reference_commit = run(["git", "-C", str(reference), "rev-parse", "HEAD"])
    host["reference_commit"] = reference_commit.stdout.strip()
    host["workers"] = workers
    host["cases"] = len(cases)
    (output / "host.json").write_text(json.dumps(host, indent=2) + "\n")

    def execute(case: dict[str, object]) -> dict[str, object]:
        case_id = str(case["id"])
        author_log = raw_dir / f"{case_id}.mawpf.txt"
        author = run([
            str(theirs), "-m", str(case["map_path"]), "-N", str(case["agents"]),
            "--algo", "lacam", "-s", str(case["seed"]), "-t", str(args.time_limit),
            "-v", "0", "--path_length", str(args.horizon), "--max_speed", str(args.max_speed),
            "--rotation_steps", str(args.rotation_steps), "-o", str(author_log), "-l",
        # The solver enforces the requested one-second algorithm budget itself.
        # This outer guard additionally allows time to map its on-disk cache.
        ], timeout=max(args.process_timeout, args.time_limit))
        author_kv = parse_key_values(author_log.read_text() if author_log.exists() else author.stdout)
        ours_run = run([
            str(ours), str(case["yaml"]), "", str(args.time_limit), "", "0", "0",
            str(case["seed"]), "dfs", "1.5", "h", "1", str(args.max_speed),
            str(args.rotation_steps), str(args.horizon), "all", "1,1,1,1,0,0,0", "1",
            str(cache_paths[str(case["map"])]),
            str(motion_cache_paths[str(case["map"])]),
        ], timeout=max(args.process_timeout, args.time_limit))
        (raw_dir / f"{case_id}.tapf.txt").write_text(ours_run.stdout)
        ours_kv = parse_key_values(ours_run.stdout)
        return {
            "case": case_id, "scenario": case["map"], "agents": case["agents"],
            "seed": case["seed"], "time_limit_sec": args.time_limit,
            "mawpf_solved": int(author_kv.get("solved", "0") == "1"),
            "mawpf_soc": int(float(author_kv.get("soc", "0"))),
            "mawpf_runtime_ms": float(author_kv.get("comp_time", "nan")),
            "tapf_solved": int(ours_kv.get("solved", "0") == "1"),
            "tapf_valid": int(ours_kv.get("valid_solution", "0") == "1"),
            "tapf_soc": int(float(ours_kv.get("soc", "0"))),
            "tapf_runtime_ms": float(ours_kv.get("runtime_ms", "nan")),
            "tapf_map_distance_load_ms": float(ours_kv.get("map_distance_load_ms", "nan")),
            "tapf_motion_graph_preprocess_ms": float(
                ours_kv.get("motion_graph_preprocess_ms", "nan")),
            "tapf_motion_path_load_ms": float(
                ours_kv.get("motion_path_load_ms", "nan")),
            "tapf_assignment_time_ms": float(
                ours_kv.get("assignment_time_ms", "nan")),
            "tapf_assignment_calls": int(ours_kv.get("assignment_calls", "0")),
            "tapf_hl_nodes": int(ours_kv.get("hl_nodes_created", "0")),
            "mawpf_exit": author.returncode, "tapf_exit": ours_run.returncode,
        }

    started = time.time()
    rows: list[dict[str, object]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(execute, case) for case in cases]
        for future in concurrent.futures.as_completed(futures):
            rows.append(future.result())
    rows.sort(key=lambda row: str(row["case"]))
    with (output / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    by_scenario: dict[str, dict[str, object]] = {}
    for scenario in selected:
        group = [row for row in rows if row["scenario"] == scenario]
        by_scenario[scenario] = {
            "cases": len(group),
            "mawpf_solved": sum(int(row["mawpf_solved"]) for row in group),
            "tapf_solved": sum(int(row["tapf_solved"]) for row in group),
            "tapf_valid": sum(int(row["tapf_valid"]) for row in group),
            "mawpf_external_timeouts": sum(int(row["mawpf_exit"]) == 124 for row in group),
            "tapf_external_timeouts": sum(int(row["tapf_exit"]) == 124 for row in group),
            "ta_invariant": all(int(row["tapf_hl_nodes"]) > 0 and
                                int(row["tapf_assignment_calls"]) == int(row["tapf_hl_nodes"])
                                for row in group),
        }
    summary = {
        "comparison_design": {
            "instances": "identical starts/headings and goal-state pool from the reference generator",
            "mawpf_mode": "reference labeled goals",
            "tapf_mode": "dynamic assignment over the same goal-state pool",
            "density_selection": "lowest published agent count per Figure-3 scenario",
            "distance_preprocessing": "parallel all-pairs grid cache; goal rows loaded before solver deadline",
            "motion_preprocessing": "parallel fixed-horizon motion candidates cached per map and loaded before solver deadline",
        },
        "paper_parameters": {"max_speed": args.max_speed,
                             "rotation_steps": args.rotation_steps,
                             "lookahead_horizon": args.horizon},
        "time_limit_sec": args.time_limit,
        "process_timeout_sec": args.process_timeout,
        "cases_per_scenario": args.cases_per_scenario,
        "scenario_count": len(selected),
        "scenario_agents": {scenario: SCENARIOS[scenario] for scenario in selected},
        "total_cases": len(rows),
        "elapsed_sec": time.time() - started,
        "host": host,
        "totals": {
            "mawpf_solved": sum(int(row["mawpf_solved"]) for row in rows),
            "tapf_solved": sum(int(row["tapf_solved"]) for row in rows),
            "tapf_valid": sum(int(row["tapf_valid"]) for row in rows),
            "mawpf_external_timeouts": sum(int(row["mawpf_exit"]) == 124 for row in rows),
            "tapf_external_timeouts": sum(int(row["tapf_exit"]) == 124 for row in rows),
            "ta_invariant": all(int(row["tapf_hl_nodes"]) > 0 and
                                int(row["tapf_assignment_calls"]) == int(row["tapf_hl_nodes"])
                                for row in rows),
        },
        "performance_on_jointly_solved": {
            "cases": len([row for row in rows if row["mawpf_solved"] and row["tapf_solved"]]),
            "mawpf_runtime_median_ms": statistics.median(
                float(row["mawpf_runtime_ms"]) for row in rows
                if row["mawpf_solved"] and row["tapf_solved"]),
            "tapf_runtime_median_ms": statistics.median(
                float(row["tapf_runtime_ms"]) for row in rows
                if row["mawpf_solved"] and row["tapf_solved"]),
            "tapf_over_mawpf_paired_median": statistics.median(
                float(row["tapf_runtime_ms"]) / float(row["mawpf_runtime_ms"])
                for row in rows if row["mawpf_solved"] and row["tapf_solved"]),
            "tapf_assignment_median_ms": statistics.median(
                float(row["tapf_assignment_time_ms"]) for row in rows),
            "tapf_map_row_load_median_ms": statistics.median(
                float(row["tapf_map_distance_load_ms"]) for row in rows),
            "tapf_motion_graph_preprocess_median_ms": statistics.median(
                float(row["tapf_motion_graph_preprocess_ms"]) for row in rows),
            "tapf_motion_path_load_median_ms": statistics.median(
                float(row["tapf_motion_path_load_ms"]) for row in rows),
        },
        "by_scenario": by_scenario,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
