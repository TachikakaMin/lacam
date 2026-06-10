#!/usr/bin/env python3
"""Run lifelong TAPF benchmark batches."""

import argparse
import subprocess
from pathlib import Path
from typing import List


def parse_int_list(value: str) -> List[int]:
    return [int(item) for item in value.split(",") if item]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="./build/lifelong_benchmark")
    parser.add_argument("--map", default="./tests/assets/symbotic.map")
    parser.add_argument("--output", default="./build/lifelong_results.csv")
    parser.add_argument("--cache", default="./build/lifelong_symbotic.distcache")
    parser.add_argument("--agent-counts", default="10,20,30,40,50")
    parser.add_argument("--seeds", default="0,1,2,3,4")
    parser.add_argument("--horizon", type=int, default=1000)
    parser.add_argument("--time-limit-sec", type=float, default=2.0)
    parser.add_argument("--goal-set-size", type=int, default=5)
    parser.add_argument("--outbound-prob", type=float, default=0.5)
    parser.add_argument("--release-interval", type=int, default=10)
    parser.add_argument("--planner-anytime", action="store_true")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    binary = Path(args.binary)
    if not binary.exists():
        raise FileNotFoundError(f"benchmark binary not found: {binary}")

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.cache).parent.mkdir(parents=True, exist_ok=True)

    for num_agents in parse_int_list(args.agent_counts):
        for seed in parse_int_list(args.seeds):
            cmd = [
                str(binary),
                args.map,
                str(num_agents),
                str(args.horizon),
                str(seed),
                args.output,
                args.cache,
                str(args.time_limit_sec),
                str(args.goal_set_size),
                str(args.outbound_prob),
                str(args.release_interval),
                "1" if args.debug else "0",
                "",
                "1" if args.planner_anytime else "0",
            ]
            print("running", " ".join(cmd), flush=True)
            subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
