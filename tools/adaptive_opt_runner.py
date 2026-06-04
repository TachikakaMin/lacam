#!/usr/bin/env python3
"""Adaptively run the full opt experiment while reserving CPU threads."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


RUN_CMD_BASE = [
    sys.executable,
    "-u",
    "tools/run_full_three_method_experiment.py",
    "--all-itacbs-data",
    "--skip-ir-suite",
    "--methods",
    "opt_dbs_hungarian",
    "--time-limit",
    "10",
    "--ir-refinement-time-limit",
    "5",
    "--timeout",
    "30",
    "--resume",
]


def log(path: Path, message: str) -> None:
    line = f"[adaptive] {time.strftime('%Y-%m-%dT%H:%M:%S%z')} {message}\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(line)
    print(line, end="", flush=True)


def read_pid(path: Path) -> int | None:
    try:
        return int(path.read_text(encoding="utf-8").strip().split()[0])
    except (FileNotFoundError, IndexError, ValueError):
        return None


def pid_alive(pid: int | None) -> bool:
    if pid is None:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def count_jsonl(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8") as f:
        return sum(1 for line in f if line.strip())


def process_cpu_snapshot(exclude_pgids: set[int]) -> dict[int, int]:
    clk = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    snap: dict[int, int] = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            pgid = os.getpgid(pid)
            if pgid in exclude_pgids:
                continue
            stat = (entry / "stat").read_text(encoding="utf-8", errors="ignore")
            close = stat.rfind(")")
            fields = stat[close + 2 :].split()
            # utime/stime are fields 14/15 in procfs, index 11/12 after removing pid/comm.
            ticks = int(fields[11]) + int(fields[12])
            snap[pid] = ticks
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, IndexError):
            continue
    # Convert back to raw ticks; callers divide by clk.
    snap[-1] = clk
    return snap


def measure_other_cpu_cores(exclude_pgids: set[int], sample_seconds: float) -> float:
    a = process_cpu_snapshot(exclude_pgids)
    clk = a.pop(-1)
    time.sleep(sample_seconds)
    b = process_cpu_snapshot(exclude_pgids)
    b.pop(-1, None)
    total_ticks = 0
    for pid, end_ticks in b.items():
        start_ticks = a.get(pid)
        if start_ticks is not None and end_ticks >= start_ticks:
            total_ticks += end_ticks - start_ticks
    return total_ticks / clk / max(sample_seconds, 0.001)


def read_cpu_totals() -> tuple[int, int]:
    values = list(map(int, Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]))
    idle = values[3] + values[4]
    total = sum(values)
    return idle, total


def measure_idle_threads(logical_threads: int, sample_seconds: float) -> float:
    idle_a, total_a = read_cpu_totals()
    time.sleep(sample_seconds)
    idle_b, total_b = read_cpu_totals()
    total_delta = max(1, total_b - total_a)
    return logical_threads * (idle_b - idle_a) / total_delta


def process_group_of(pid: int | None) -> int | None:
    if pid is None:
        return None
    try:
        return os.getpgid(pid)
    except ProcessLookupError:
        return None


def terminate_process_group(pid: int | None, log_path: Path, label: str) -> None:
    if not pid_alive(pid):
        return
    pgid = process_group_of(pid)
    if pgid is None:
        return
    log(log_path, f"terminating {label} pid={pid} pgid={pgid}")
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.time() + 20
    while time.time() < deadline:
        if not pid_alive(pid):
            return
        time.sleep(0.5)
    log(log_path, f"killing {label} pid={pid} pgid={pgid}")
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def start_runner(args: argparse.Namespace, jobs: int, log_path: Path) -> int:
    run_log = args.run_dir / "run.log"
    cmd = [
        *RUN_CMD_BASE,
        "--jobs",
        str(jobs),
        "--out-dir",
        str(args.run_dir),
    ]
    with run_log.open("a", encoding="utf-8") as f:
        f.write(f"\n[adaptive_restart] {time.strftime('%Y-%m-%dT%H:%M:%S%z')} jobs={jobs}\n")
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    (args.run_dir / "run.pid").write_text(f"{proc.pid}\n", encoding="utf-8")
    log(log_path, f"started runner pid={proc.pid} jobs={jobs}")
    return proc.pid


def start_watcher(args: argparse.Namespace, log_path: Path) -> int:
    finalize_log = args.run_dir / "finalize.log"
    cmd = [
        sys.executable,
        "-u",
        "tools/finalize_2307_figures_with_opt.py",
        "--poll-seconds",
        str(args.finalize_poll_seconds),
    ]
    with finalize_log.open("a", encoding="utf-8") as f:
        f.write(f"\n[adaptive_watcher_restart] {time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    (args.run_dir / "finalize.pid").write_text(f"{proc.pid}\n", encoding="utf-8")
    log(log_path, f"started watcher pid={proc.pid}")
    return proc.pid


def current_jobs_from_cmd(pid: int | None) -> int | None:
    if not pid_alive(pid):
        return None
    try:
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().decode("utf-8", errors="ignore")
    except FileNotFoundError:
        return None
    parts = [p for p in cmdline.split("\0") if p]
    for i, part in enumerate(parts):
        if part == "--jobs" and i + 1 < len(parts):
            try:
                return int(parts[i + 1])
            except ValueError:
                return None
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=Path("build/results/full_2307_exp1_exp2_opt_dbs_hungarian_10s_split5"),
    )
    parser.add_argument("--reserve-threads", type=int, default=10)
    parser.add_argument("--poll-seconds", type=float, default=120.0)
    parser.add_argument("--sample-seconds", type=float, default=5.0)
    parser.add_argument("--min-rows", type=int, default=9360)
    parser.add_argument("--min-jobs", type=int, default=1)
    parser.add_argument("--max-jobs", type=int, default=0)
    parser.add_argument("--adjust-threshold", type=int, default=8)
    parser.add_argument("--idle-tolerance", type=float, default=2.0)
    parser.add_argument("--idle-correction-gain", type=float, default=1.5)
    parser.add_argument("--finalize-poll-seconds", type=float, default=120.0)
    args = parser.parse_args()

    args.run_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.run_dir / "adaptive_controller.log"
    status_path = args.run_dir / "adaptive_status.json"
    logical_threads = os.cpu_count() or 1
    max_jobs = args.max_jobs if args.max_jobs > 0 else logical_threads
    current_jobs: int | None = None
    last_restart = 0.0

    log(
        log_path,
        f"starting controller logical_threads={logical_threads} reserve={args.reserve_threads}",
    )

    while True:
        rows = count_jsonl(args.run_dir / "rows.jsonl")
        run_pid = read_pid(args.run_dir / "run.pid")
        watcher_pid = read_pid(args.run_dir / "finalize.pid")
        run_alive = pid_alive(run_pid)
        watcher_alive = pid_alive(watcher_pid)
        run_pgid = process_group_of(run_pid) if run_alive else None
        controller_pgid = os.getpgid(os.getpid())
        exclude_pgids = {controller_pgid}
        if run_pgid is not None:
            exclude_pgids.add(run_pgid)

        other_cores = measure_other_cpu_cores(exclude_pgids, args.sample_seconds)
        idle_threads = measure_idle_threads(logical_threads, min(2.0, args.sample_seconds))
        desired_by_other = math.floor(logical_threads - args.reserve_threads - other_cores)
        desired = max(args.min_jobs, min(max_jobs, desired_by_other))
        current_jobs = current_jobs_from_cmd(run_pid) if run_alive else None
        if run_alive and current_jobs is not None and idle_threads < args.reserve_threads - args.idle_tolerance:
            deficit = args.reserve_threads - idle_threads
            correction = max(1, math.ceil(deficit * args.idle_correction_gain))
            desired = min(desired, max(args.min_jobs, current_jobs - correction))
        should_restart = False
        reason = ""

        if rows >= args.min_rows:
            log(log_path, f"rows={rows} reached min_rows={args.min_rows}; controller exiting")
            break
        if not run_alive:
            should_restart = True
            reason = "runner_not_alive"
        elif current_jobs is None:
            should_restart = True
            reason = "unknown_current_jobs"
        elif abs(desired - current_jobs) >= args.adjust_threshold and time.time() - last_restart > args.poll_seconds:
            should_restart = True
            reason = f"adjust_jobs current={current_jobs} desired={desired}"

        status = {
            "rows_jsonl": rows,
            "logical_threads": logical_threads,
            "reserve_threads": args.reserve_threads,
            "other_cpu_cores_recent": round(other_cores, 3),
            "idle_threads_recent": round(idle_threads, 3),
            "desired_jobs_by_other_cpu": desired_by_other,
            "desired_jobs": desired,
            "current_jobs": current_jobs,
            "run_pid": run_pid,
            "run_alive": run_alive,
            "watcher_pid": watcher_pid,
            "watcher_alive": watcher_alive,
            "timestamp": time.time(),
        }
        status_path.write_text(json.dumps(status, sort_keys=True) + "\n", encoding="utf-8")
        log(log_path, "status " + json.dumps(status, sort_keys=True))

        if should_restart:
            log(log_path, f"restart reason={reason}")
            terminate_process_group(run_pid, log_path, "runner")
            terminate_process_group(watcher_pid, log_path, "watcher")
            run_pid = start_runner(args, desired, log_path)
            watcher_pid = start_watcher(args, log_path)
            last_restart = time.time()
        elif not watcher_alive:
            watcher_pid = start_watcher(args, log_path)

        time.sleep(args.poll_seconds)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
