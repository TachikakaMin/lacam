#!/usr/bin/env python3
"""Export gantry rounds into house-build's shared app.js PLAN format.

Replays work/round_*.plan on one global timeline and writes plan.js:
  PLAN = {rows, cols, T, depot, pyramid, agents:[{name,path,carry}],
          terrain:[{t,r,c,h,agent}], picks, colors3, meta:{gantry}}
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORK = HERE / "work"
ROWS, COLS = 9, 16
PYR_R0, PYR_C0, PYR_BASE, PYR_LEVELS = 2, 2, 5, 3
DEPOT = [(r, c) for r in range(2, 7) for c in range(10, 15)]
ROBOT_STARTS = [(1, 8), (2, 8), (3, 8), (4, 8), (5, 8), (6, 8)]
HEX_BY_LEVEL = {1: "#3b7dd8", 2: "#e8c531", 3: "#d84a3b"}
RAIL_LEVEL = 5  # rail plane, in block-height units


def parse_starts(path: Path):
    starts = []
    for line in path.read_text().splitlines():
        m = re.search(r"start: \[(\d+), (\d+)\]", line)
        if m:
            starts.append((int(m.group(1)), int(m.group(2))))
    return starts


def main() -> int:
    rounds = sorted(WORK.glob("round_*.yaml"),
                    key=lambda p: int(p.stem.split("_")[1]))
    robots = list(ROBOT_STARTS)
    kappa = [False] * len(robots)
    paths = [[list(p)] for p in robots]
    carry = [[0] for _ in robots]
    stacks = {}
    terrain, picks = [], []
    t = 0

    for ypath in rounds:
        rnd = int(ypath.stem.split("_")[1])
        grounded = set(parse_starts(ypath))  # depot bricks of this round
        plan = (WORK / f"round_{rnd}.plan").read_text().splitlines()
        for line in plan:
            toks = line.strip().split(";")
            if len(toks) != len(robots):
                continue
            t += 1
            for i, tok in enumerate(toks):
                parts = tok.split()
                if parts and parts[0] == "m":
                    robots[i] = (int(parts[1]), int(parts[2]))
                elif parts and parts[0] == "l" and robots[i] in grounded:
                    grounded.discard(robots[i])
                    kappa[i] = True
                    picks.append({"t": t, "agent": i,
                                  "r": robots[i][0], "c": robots[i][1]})
                elif parts and parts[0] == "d" and kappa[i]:
                    kappa[i] = False
                    cell = robots[i]
                    h = stacks.get(cell, 0) + 1
                    stacks[cell] = h
                    terrain.append({"t": t, "r": cell[0], "c": cell[1],
                                    "h": h, "agent": i})
                paths[i].append(list(robots[i]))
                carry[i].append(1 if kappa[i] else 0)

    colors3 = {}
    for (r, c), h in stacks.items():
        for z in range(1, h + 1):
            colors3[f"{r},{c},{z}"] = HEX_BY_LEVEL[z]

    plan = {
        "rows": ROWS, "cols": COLS, "T": t,
        "depot": [list(d) for d in DEPOT],
        "pyramid": {"r0": PYR_R0, "c0": PYR_C0,
                    "base": PYR_BASE, "levels": PYR_LEVELS},
        "agents": [{"name": f"a{i}", "path": paths[i], "carry": carry[i]}
                   for i in range(len(robots))],
        "terrain": terrain, "picks": picks, "colors3": colors3,
        "meta": {"gantry": RAIL_LEVEL,
                 "statsLine": "AutoStore 顶部吊运 · 5×5 三层金字塔 · 35 块砖"},
    }
    out = HERE / "plan.js"
    out.write_text("const PLAN = " + json.dumps(plan) + ";\n",
                   encoding="utf-8")
    print(f"T={t} places={len(terrain)} picks={len(picks)} -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
