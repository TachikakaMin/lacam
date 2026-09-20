#!/usr/bin/env python3
"""Replay work/round_*.plan into per-frame animation data (plan_data.js).

Frame format: {"R": [[r,c] x robots], "B": [[state,r,c,h] x bricks]}
brick state: 0 not spawned yet, 1 grounded at depot, 2 carried, 3 stacked.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORK = HERE / "work"
ROWS, COLS = 9, 16
ROBOT_STARTS = [(1, 8), (2, 8), (3, 8), (4, 8), (5, 8), (6, 8)]
COLOR_BY_ROUND = {1: "#3b7dd8", 2: "#e8c531", 3: "#d84a3b"}


def parse_yaml(path: Path):
    """Minimal parse of our generated instance yaml: target starts/goals."""
    starts, goals = [], []
    mode = None
    for line in path.read_text().splitlines():
        if line.startswith("targets:"):
            mode = "t"
            continue
        if mode == "t":
            m = re.search(r"start: \[(\d+), (\d+)\]", line)
            if m:
                starts.append((int(m.group(1)), int(m.group(2))))
            m = re.search(r"goal: \[(\d+), (\d+)\]", line)
            if m:
                goals.append((int(m.group(1)), int(m.group(2))))
    return starts, goals


def main() -> int:
    rounds = sorted(WORK.glob("round_*.yaml"),
                    key=lambda p: int(p.stem.split("_")[1]))
    robots = list(ROBOT_STARTS)
    stacks = {}  # (r,c) -> height
    bricks = []  # global: {"color":..}; runtime state kept in arrays below
    b_state = []  # per global brick: [state, r, c, h]
    frames = []

    def emit():
        frames.append({"R": [list(p) for p in robots],
                       "B": [list(s) for s in b_state]})

    emit()
    for ypath in rounds:
        rnd = int(ypath.stem.split("_")[1])
        starts, _goals = parse_yaml(ypath)
        base = len(bricks)
        for s in starts:
            bricks.append({"color": COLOR_BY_ROUND[rnd]})
            b_state.append([1, s[0], s[1], 0])
        kappa = [-1] * len(robots)
        plan = (WORK / f"round_{rnd}.plan").read_text().splitlines()
        for line in plan:
            toks = line.strip().split(";")
            if len(toks) != len(robots):
                continue
            for i, tok in enumerate(toks):
                parts = tok.split()
                if not parts:
                    continue
                if parts[0] == "m":
                    robots[i] = (int(parts[1]), int(parts[2]))
                    if kappa[i] >= 0:
                        b_state[kappa[i]][1:3] = list(robots[i])
                elif parts[0] == "l":
                    for b in range(base, len(bricks)):
                        if b_state[b][0] == 1 and \
                           tuple(b_state[b][1:3]) == robots[i]:
                            kappa[i] = b
                            b_state[b][0] = 2
                            break
                elif parts[0] == "d" and kappa[i] >= 0:
                    cell = robots[i]
                    h = stacks.get(cell, 0)
                    stacks[cell] = h + 1
                    b_state[kappa[i]] = [3, cell[0], cell[1], h]
                    kappa[i] = -1
            emit()

    data = {"rows": ROWS, "cols": COLS,
            "bricks": bricks, "frames": frames,
            "robots": len(robots)}
    out = HERE / "plan_data.js"
    out.write_text("window.GANTRY_DATA = " + json.dumps(data) + ";\n",
                   encoding="utf-8")
    print(f"frames={len(frames)} bricks={len(bricks)} -> {out} "
          f"({out.stat().st_size//1024} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
