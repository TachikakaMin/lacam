#!/usr/bin/env python3
"""AutoStore-style gantry pyramid small experiment (top robots, hoist down).

Model:
  - Robots live on a flat top 2D grid forever (AutoStore gantry).
  - Each (r, c) column is a vertical stack; a brick DROPped on a column cell
    sinks onto the stack below (height += 1) and leaves the 2D robot plane.
  - Grounded connectivity is trivially satisfied: every brick rests on the
    stack of the same column down to the ground (pyramid needs no scaffold).
  - Per round, every unfinished column requests exactly one brick of the
    color required at its current height; brick starts at a depot cell.
  - Each round is one plain Carrier-LaCAM instance solved by dd_benchmark;
    no C++ change, third dimension is handled by this orchestrator.

Validation: final per-column color stacks must equal the target template.
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BINARY = REPO / "build-release" / "dd_benchmark"
WORK = Path(__file__).resolve().parent / "work"
OUT_JSON = Path(__file__).resolve().parent / "gantry_pyramid_plan.json"

ROWS, COLS = 9, 16
PYR_R0, PYR_C0, PYR_BASE, PYR_LEVELS = 2, 2, 5, 3
DEPOT = [(r, c) for r in range(2, 7) for c in range(10, 15)]  # 25 cells
ROBOT_STARTS = [(1, 8), (2, 8), (3, 8), (4, 8), (5, 8), (6, 8)]
COLOR_BY_LEVEL = {1: "blue", 2: "yellow", 3: "red"}
ROUND_TIME_LIMIT = 10.0
SEED = 0


def target_height(r: int, c: int) -> int:
    dr, dc = r - PYR_R0, c - PYR_C0
    if not (0 <= dr < PYR_BASE and 0 <= dc < PYR_BASE):
        return 0
    return min(min(dr, PYR_BASE - 1 - dr), min(dc, PYR_BASE - 1 - dc)) + 1


def template() -> dict:
    return {(r, c): target_height(r, c)
            for r in range(ROWS) for c in range(COLS) if target_height(r, c)}


def write_instance(path: Path, robots, targets) -> None:
    lines = [f"name: gantry_round", "map: |"]
    lines += ["  " + "." * COLS for _ in range(ROWS)]
    lines.append("robots:")
    lines += [f"  - [{r}, {c}]" for r, c in robots]
    lines.append("shelves:")
    lines += [f"  - [{sr}, {sc}]" for (sr, sc), _ in targets]
    lines.append("targets:")
    for i, ((sr, sc), (gr, gc)) in enumerate(targets):
        lines += [f"  - id: b{i}", f"    start: [{sr}, {sc}]",
                  f"    goal: [{gr}, {gc}]"]
    lines.append("flags: {}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def replay_robot_paths(plan_path: Path, robots):
    pos = list(robots)
    paths = [ [p] for p in pos ]
    for line in plan_path.read_text().splitlines():
        toks = line.strip().split(";")
        if len(toks) != len(pos):
            continue
        for i, tok in enumerate(toks):
            parts = tok.split()
            if parts and parts[0] == "m":
                pos[i] = (int(parts[1]), int(parts[2]))
            paths[i].append(pos[i])
    return pos, paths


def main() -> int:
    t0 = time.time()
    WORK.mkdir(parents=True, exist_ok=True)
    tmpl = template()
    heights = {cell: 0 for cell in tmpl}
    stacks = {cell: [] for cell in tmpl}
    robots = list(ROBOT_STARTS)
    rounds_log = []

    rnd = 0
    while any(heights[c] < tmpl[c] for c in tmpl):
        rnd += 1
        active = sorted(c for c in tmpl if heights[c] < tmpl[c])
        assert len(active) <= len(DEPOT), "not enough depot cells"
        targets = [(DEPOT[i], cell) for i, cell in enumerate(active)]
        colors = {cell: COLOR_BY_LEVEL[heights[cell] + 1] for cell in active}

        yaml_path = WORK / f"round_{rnd}.yaml"
        plan_path = WORK / f"round_{rnd}.plan"
        write_instance(yaml_path, robots, targets)
        res = subprocess.run(
            [str(BINARY), str(yaml_path), str(ROUND_TIME_LIMIT),
             str(plan_path), str(SEED), "lacam"],
            capture_output=True, text=True, timeout=ROUND_TIME_LIMIT + 15)
        metrics = dict(kv.split("=") for kv in res.stdout.split()
                       if "=" in kv)
        if metrics.get("solved") != "1":
            print(f"round {rnd}: NOT SOLVED\n{res.stdout}\n{res.stderr}")
            return 1
        robots, paths = replay_robot_paths(plan_path, robots)
        for cell in active:
            stacks[cell].append(colors[cell])
            heights[cell] += 1
        rounds_log.append({
            "round": rnd, "bricks": len(active),
            "makespan": int(metrics["makespan"]),
            "runtime_ms": float(metrics["runtime_ms"]),
            "robot_paths": [[list(p) for p in path] for path in paths],
            "placements": [{"cell": list(cell), "z": heights[cell],
                            "color": colors[cell]} for cell in active],
        })
        print(f"round {rnd}: bricks={len(active)} "
              f"makespan={metrics['makespan']} "
              f"runtime_ms={metrics['runtime_ms']}")

    # authoritative final check: stacks == colored template
    ok = True
    for cell, h in tmpl.items():
        want = [COLOR_BY_LEVEL[z] for z in range(1, h + 1)]
        if stacks[cell] != want:
            ok = False
            print(f"MISMATCH at {cell}: {stacks[cell]} != {want}")
    total = time.time() - t0
    OUT_JSON.write_text(json.dumps({
        "grid": [ROWS, COLS], "template": [
            {"cell": list(c), "height": h} for c, h in sorted(tmpl.items())],
        "rounds": rounds_log, "wall_sec": total, "valid": ok,
    }, indent=1), encoding="utf-8")
    print(f"pyramid complete: rounds={rnd} "
          f"total_bricks={sum(tmpl.values())} wall={total:.2f}s "
          f"valid={'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
