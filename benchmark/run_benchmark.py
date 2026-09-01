#!/usr/bin/env python3
"""Benchmark runner for baseline methods (design.md 8.1/8.3).

Methods:
  b4          — single-robot sequential simulation (Theorem 1); in-process.
  crest_base  — CREST executor with all constraint-release OFF
                (= MAPF-DECOMP-style decomposed execution).
  crest_full  — CREST with --STR --DW --GTR on.
  natcbs      — MAWR NAT-CBS (makespan-optimal), small instances only.

Unified metrics per row (rows.csv):
  instance, family, method, success, executed_makespan, weighted_soc,
  loaded_moves, free_moves, lift_drop, runtime_sec, status, raw

External binaries run through the ddtool micromamba env.  Converted inputs and
raw outputs are kept under <out>/work/ for auditability.
"""

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.b4_baseline import B4Failure, solve_b4
from ddbench.converters import ConversionError, to_crest, to_mawr
from ddbench.instance import load_instance
from ddbench.validator import plan_cost, validate_plan

REPO = Path(__file__).resolve().parent.parent
CREST_BIN = REPO / "baselines/CREST/build/CREST"
MAWR_BIN = REPO / "baselines/wh-rearrangement/build/MAWR"
CARRIER_BIN = REPO / "build/dd_benchmark"
MICROMAMBA = Path.home() / ".local/bin/micromamba"

FIELDS = [
    "instance", "family", "method", "success", "executed_makespan",
    "weighted_soc", "loaded_moves", "free_moves", "lift_drop",
    "shelf_switches", "robot_utilization", "first_solution_ms",
    "reversals", "assignment_restarts", "assignment_second_solved",
    "assignment_improvements", "assignment_second_solution_ms",
    "assignment_first_soc", "assignment_second_soc",
    "assignment_first_makespan", "assignment_second_makespan",
    "runtime_sec", "status", "raw",
]


def run_external(cmd, timeout):
    # run binaries directly with the conda env's libs: the micromamba wrapper
    # serializes on a lock under heavy parallelism and skews timing.
    env = dict(
        os.environ,
        LD_LIBRARY_PATH=str(Path.home() / "micromamba/envs/ddtool/lib"),
    )
    full = [str(c) for c in cmd]
    t0 = time.time()
    proc = subprocess.Popen(
        full, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        env=env, start_new_session=True,
    )
    try:
        out, err = proc.communicate(timeout=timeout)
        return proc.returncode, out, err, time.time() - t0, "ok"
    except subprocess.TimeoutExpired:
        import signal
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        proc.communicate()
        return -1, "", "", timeout, "timeout"


def _blank_extra():
    return dict(shelf_switches="", robot_utilization="", reversals="",
                first_solution_ms="")


def row_b4(ins, name, family, timeout, weights=(1.0, 1.0, 1.0, 1.0)):
    t0 = time.time()
    try:
        plan = solve_b4(ins)
    except B4Failure as e:
        return dict(instance=name, family=family, method="b4", success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(time.time() - t0, 3),
                    status="failed", raw=str(e))
    runtime = time.time() - t0
    ok, errs, _ = validate_plan(ins, plan)
    if not ok:
        return dict(instance=name, family=family, method="b4", success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(runtime, 3),
                    status="invalid_plan", raw=";".join(errs)[:200])
    c = plan_cost(ins, plan, *weights)
    return dict(instance=name, family=family, method="b4", success=1,
                executed_makespan=c["executed_makespan"],
                weighted_soc=c["weighted_soc"], loaded_moves=c["loaded_moves"],
                free_moves=c["free_moves"], lift_drop=c["lift_drop"],
                shelf_switches=c["shelf_switches"],
                reversals=c["reversals"],
                robot_utilization=round(c["robot_utilization"], 4),
                first_solution_ms="",
                runtime_sec=round(runtime, 3), status="ok", raw="")


CREST_RE = re.compile(
    r"sum_of_cost : (\S+) makespan : (\S+) agent_travel_time : (\S+) "
    r"pickup_count : (\S+) rearrange_shelf_num : (\S+) "
    r"pickup_dropoff_overhead : (\S+)"
)


def row_crest(ins, name, family, work, timeout, full_release, subopt=1.6):
    method = "crest_full" if full_release else "crest_base"
    mp = work / f"{name}.crest.map"
    sp = work / f"{name}.crest.scen"
    try:
        to_crest(ins, mp, sp)
    except ConversionError as e:
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=0,
                    status="conversion_error", raw=str(e))
    flag = "true" if full_release else "false"
    rc, out, err, rt, status = run_external(
        [CREST_BIN, f"--suboptimality={subopt}", "-m", mp, "-a", sp,
         "-k", str(len(ins.robots)), f"--STR={flag}", f"--DW={flag}",
         f"--GTR={flag}", "--overhead=false", "-t", str(timeout)],
        timeout + 30,
    )
    (work / f"{name}.{method}.out").write_text(out + "\n--stderr--\n" + err)
    m = CREST_RE.search(out)
    if status != "ok" or rc != 0 or not m:
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                    status=status if status != "ok" else f"rc={rc}",
                    raw=(out + err)[-200:].replace("\n", " "))
    soc, mk, travel, pickups, _, overhead = m.groups()
    # CREST loaded_moves = soc - travel - overhead (agent timesteps split)
    return dict(instance=name, family=family, method=method, success=1,
                executed_makespan=int(mk),
                weighted_soc=int(soc),
                loaded_moves=int(soc) - int(travel) - int(overhead),
                free_moves=int(travel), lift_drop=int(overhead),
                **_blank_extra(), runtime_sec=round(rt, 3), status="ok",
                raw=m.group(0)[:200])


def row_natcbs(ins, name, family, work, timeout):
    mp = work / f"{name}.mawr.map"
    sp = work / f"{name}.mawr.scen"
    to_mawr(ins, mp, sp)
    out_csv = work / f"{name}.mawr.csv"
    if out_csv.exists():
        out_csv.unlink()
    out_csv.touch()
    rc, out, err, rt, status = run_external(
        [MAWR_BIN, "-m", mp, "-s", sp, "-a", "NATCBS",
         "-t", str(int(timeout)), "-o", out_csv],
        timeout + 30,
    )
    (work / f"{name}.natcbs.out").write_text(out + "\n--stderr--\n" + err)
    mk = ""
    st = status
    if status == "ok":
        lines = out_csv.read_text().strip().splitlines()
        if len(lines) >= 2:
            parts = lines[-1].split(";")
            val = parts[2]
            if val == "TLR":
                st = "timeout"
            elif val in ("NF", "IVP"):
                st = "failed"
            else:
                mk = int(val)
                st = "ok"
        else:
            st = f"rc={rc}"
    success = 1 if mk != "" else 0
    return dict(instance=name, family=family, method="natcbs", success=success,
                executed_makespan=mk, weighted_soc="", loaded_moves="",
                free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                status=st, raw=(out + err)[-150:].replace("\n", " "))


def parse_carrier_plan(path):
    path = Path(path)
    if not path.is_file():
        raise ValueError("solver reported success without a plan file")
    plan = []
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        joint = []
        for token in line.split(";"):
            parts = token.split()
            if parts == ["w"]:
                joint.append(("wait",))
            elif len(parts) == 3 and parts[0] == "m":
                try:
                    cell = (int(parts[1]), int(parts[2]))
                except ValueError as exc:
                    raise ValueError(
                        f"line {line_no}: invalid move token {token!r}"
                    ) from exc
                joint.append(("move", cell))
            elif parts == ["l"]:
                joint.append(("lift",))
            elif parts == ["d"]:
                joint.append(("drop",))
            else:
                raise ValueError(
                    f"line {line_no}: invalid action token {token!r}"
                )
        plan.append(joint)
    if not plan:
        raise ValueError("solver reported success with an empty plan")
    return plan


def row_carrier(ins, path, name, family, work, timeout, mode="lacam",
                env=None, weights=(1.0, 1.0, 1.0, 1.0)):
    """Carrier-LaCAM (C++): plan re-validated by the authoritative Python
    two-deck validator; unified metrics via plan_cost (same as b4)."""
    from ddbench.validator import apply_joint_action, initial_state, is_goal

    method = {"lacam": "carrier", "b0": "carrier_b0",
              "b1": "carrier_b1"}[mode]
    plan_out = work / f"{name}.{method}.plan"
    if plan_out.exists():
        plan_out.unlink()
    t0 = time.time()
    env = dict(os.environ) if env is None else dict(env)
    for key, value in zip(
        ("DD_ALPHA", "DD_BETA", "DD_GAMMA", "DD_DELTA"), weights
    ):
        env[key] = str(value)
    try:
        p = subprocess.run(
            [str(CARRIER_BIN), str(path), str(timeout), str(plan_out), "0",
             mode],
            capture_output=True, text=True, timeout=timeout + 30, env=env,
        )
        status = "ok"
    except subprocess.TimeoutExpired:
        p = None
        status = "timeout"
    rt = time.time() - t0
    metrics = {}
    if p is not None:
        for line in p.stdout.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                metrics[k.strip()] = v.strip()
    if (status != "ok" or p.returncode != 0 or
            metrics.get("solved") != "1"):
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                    status="timeout" if status != "ok" or
                    metrics.get("timed_out") == "1" else "failed",
                    raw=(p.stdout + p.stderr)[-150:].replace("\n", " ")
                    if p else "")
    # authoritative re-validation
    try:
        plan = parse_carrier_plan(plan_out)
        s = initial_state(ins)
        for joint in plan:
            s = apply_joint_action(ins, s, joint)
        if not is_goal(ins, s):
            raise ValueError("final state is not a goal")
    except Exception as e:  # noqa: BLE001
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                    status="invalid_plan", raw=str(e)[:200])
    c = plan_cost(ins, plan, *weights)
    return dict(instance=name, family=family, method=method, success=1,
                executed_makespan=c["executed_makespan"],
                weighted_soc=c["weighted_soc"], loaded_moves=c["loaded_moves"],
                free_moves=c["free_moves"], lift_drop=c["lift_drop"],
                shelf_switches=c["shelf_switches"],
                reversals=c["reversals"],
                robot_utilization=round(c["robot_utilization"], 4),
                first_solution_ms=metrics.get("first_solution_ms", ""),
                assignment_restarts=metrics.get("assignment_restarts", ""),
                assignment_second_solved=metrics.get(
                    "assignment_second_solved", ""
                ),
                assignment_improvements=metrics.get(
                    "assignment_improvements", ""
                ),
                assignment_second_solution_ms=metrics.get(
                    "assignment_second_solution_ms", ""
                ),
                assignment_first_soc=metrics.get(
                    "assignment_first_soc", ""
                ),
                assignment_second_soc=metrics.get(
                    "assignment_second_soc", ""
                ),
                assignment_first_makespan=metrics.get(
                    "assignment_first_makespan", ""
                ),
                assignment_second_makespan=metrics.get(
                    "assignment_second_makespan", ""
                ),
                runtime_sec=round(rt, 3), status="ok", raw="")


def run_one(task):
    """Top-level worker (picklable): task = (path, family, method, work,
    timeout, natcbs_max_cells, subopt, weights)."""
    path, family, method, work, timeout, natcbs_max, subopt, weights = task
    ins = load_instance(path)
    name = Path(path).stem
    work = Path(work)
    if method == "b4":
        return row_b4(ins, name, family, timeout, weights)
    if method == "carrier":
        return row_carrier(
            ins, path, name, family, work, timeout, "lacam",
            weights=weights,
        )
    if method == "carrier_b0":
        return row_carrier(
            ins, path, name, family, work, timeout, "b0",
            weights=weights,
        )
    if method == "carrier_b1":
        return row_carrier(
            ins, path, name, family, work, timeout, "b1",
            weights=weights,
        )
    if method == "crest_base":
        return row_crest(ins, name, family, work, timeout, False, subopt)
    if method == "crest_full":
        return row_crest(ins, name, family, work, timeout, True, subopt)
    if method == "natcbs":
        if ins.height * ins.width > natcbs_max:
            return dict(instance=name, family=family, method="natcbs",
                        success=0, executed_makespan="", weighted_soc="",
                        loaded_moves="", free_moves="", lift_drop="",
                        **_blank_extra(), runtime_sec=0, status="skipped_too_large", raw="")
        return row_natcbs(ins, name, family, work, timeout)
    raise SystemExit(f"unknown method {method}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instances", default="instances")
    ap.add_argument("--out-dir", default="results")
    ap.add_argument("--methods", nargs="+",
                    default=["b4", "crest_base", "crest_full", "natcbs"])
    ap.add_argument(
        "--timeout", type=float, default=10,
        help="per-run solver deadline in seconds (fixed protocol: 10)",
    )
    ap.add_argument("--natcbs-max-cells", type=int, default=150,
                    help="skip natcbs on instances larger than this")
    ap.add_argument("--jobs", type=int, default=1,
                    help="parallel worker processes")
    ap.add_argument("--suboptimality", type=float, default=1.6,
                    help="CREST ECBS suboptimality bound")
    ap.add_argument(
        "--weights", type=float, nargs=4,
        metavar=("ALPHA", "BETA", "GAMMA", "DELTA"),
        default=(1.0, 1.0, 1.0, 1.0),
        help="carrier/B4 objective weights; defaults to the unit main table",
    )
    args = ap.parse_args()
    weights = tuple(args.weights)
    if (weights != (1.0, 1.0, 1.0, 1.0) and
            any(m in {"crest_base", "crest_full", "natcbs"}
                for m in args.methods)):
        ap.error("non-unit --weights cannot be mixed with native-objective "
                 "external methods")

    out = Path(args.out_dir)
    work = out / "work"
    work.mkdir(parents=True, exist_ok=True)

    files = sorted(Path(args.instances).glob("*/*.yaml"))
    if not files:
        print("no instances found", file=sys.stderr)
        sys.exit(1)
    tasks = [
        (str(f), f.parent.name, m, str(work), args.timeout,
         args.natcbs_max_cells, args.suboptimality, weights)
        for f in files
        for m in args.methods
    ]

    t_start = time.time()
    rows = []
    if args.jobs <= 1:
        for t in tasks:
            r = run_one(t)
            rows.append(r)
            print(f"[{len(rows)}/{len(tasks)}] {r['instance']} [{r['method']}]"
                  f" -> {r['status']} mk={r['executed_makespan']}"
                  f" t={r['runtime_sec']}s", flush=True)
    else:
        from concurrent.futures import ProcessPoolExecutor, as_completed
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            futs = {ex.submit(run_one, t): t for t in tasks}
            for fut in as_completed(futs):
                r = fut.result()
                rows.append(r)
                print(f"[{len(rows)}/{len(tasks)}] {r['instance']}"
                      f" [{r['method']}] -> {r['status']}"
                      f" mk={r['executed_makespan']}"
                      f" t={r['runtime_sec']}s", flush=True)
    wall = time.time() - t_start

    rows.sort(key=lambda r: (r["instance"], r["method"]))
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "rows.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    # summary + timing
    from collections import defaultdict
    agg = defaultdict(lambda: [0, 0, 0.0])
    for r in rows:
        agg[r["method"]][0] += r["success"] if isinstance(r["success"], int) else int(r["success"])
        agg[r["method"]][1] += 1
        agg[r["method"]][2] += float(r["runtime_sec"] or 0)
    print("\n=== success rate (method: solved/total, total solver time) ===",
          flush=True)
    summary = {}
    for m, (s, n, tt) in sorted(agg.items()):
        print(f"{m}: {s}/{n}  solver_time_sum={tt:.1f}s", flush=True)
        summary[m] = {"solved": s, "total": n, "solver_time_sum_sec": round(tt, 1)}
    timing = {
        "wall_time_sec": round(wall, 1),
        "jobs": args.jobs,
        "n_tasks": len(tasks),
        "timeout_per_run_sec": args.timeout,
        "solver_seed": 0,
        "objective_weights": {
            "alpha": weights[0],
            "beta": weights[1],
            "gamma": weights[2],
            "delta": weights[3],
        },
        "following": "allowed",
        "methods": summary,
    }
    (out / "timing.json").write_text(json.dumps(timing, indent=2))
    print(f"wall_time={wall:.1f}s jobs={args.jobs}", flush=True)
    print(f"rows written to {out / 'rows.csv'}", flush=True)


if __name__ == "__main__":
    main()
