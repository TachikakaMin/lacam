#!/usr/bin/env python3
"""Run the PROTECTED dev cases (dev_cases.txt) through dd_benchmark at the
10 s budget and re-validate plans with the authoritative Python validator."""

import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.instance import load_instance
from ddbench.validator import apply_joint_action, initial_state, is_goal
from tests.test_carrier_integration import parse_plan

BENCH = Path(__file__).resolve().parent
BIN = BENCH.parent / "build/dd_benchmark"
TIME_LIMIT = 10


def main():
    cases = [
        line.strip()
        for line in (BENCH / "dev_cases.txt").read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]
    outdir = BENCH / "results_dev"
    outdir.mkdir(exist_ok=True)
    solved = 0
    t0 = time.time()
    for case in cases:
        plan_out = outdir / (Path(case).stem + ".plan")
        t1 = time.time()
        p = subprocess.run(
            [str(BIN), str(BENCH / case), str(TIME_LIMIT), str(plan_out), "0"],
            capture_output=True, text=True, timeout=TIME_LIMIT + 30,
        )
        wall = time.time() - t1
        m = dict(
            l.split("=", 1) for l in p.stdout.splitlines() if "=" in l
        )
        status = "solved" if m.get("solved") == "1" else "FAILED"
        extra = ""
        if m.get("solved") == "1":
            ins = load_instance(BENCH / case)
            s = initial_state(ins)
            try:
                for joint in parse_plan(plan_out):
                    s = apply_joint_action(ins, s, joint)
                ok = is_goal(ins, s)
            except Exception as e:  # noqa: BLE001
                ok = False
                extra = f" VALIDATOR_ERROR: {e}"
            if not ok:
                status = "INVALID_PLAN"
            else:
                solved += 1
                extra = (f" mk={m['makespan']} soc={m['weighted_soc']}"
                         f" nodes={m['hl_nodes']}")
        print(f"{Path(case).stem:44s} {status:12s} t={wall:5.2f}s{extra}",
              flush=True)
    print(f"\nsolved {solved}/{len(cases)}  total_wall={time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
