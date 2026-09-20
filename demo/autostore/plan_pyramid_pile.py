#!/usr/bin/env python3
"""Gantry pyramid from a random pile (AutoStore digging semantics).

All 35 colored bricks start randomly stacked in a source area, capped at
MAX_H; the robot rail plane sits at MAX_H + 2. Robots hoist only the TOP
brick of a column. Per round:
  - deliver accessible bricks whose color is needed at the build frontier
  - dig: if a needed color is buried, relocate the covering top brick
    to a buffer cell (it stays a normal brick and is used later)
Each round is one plain Carrier-LaCAM 2D instance (build-release/dd_benchmark).

The replay is faithful to the solver plan: a DROP lands the brick at the
robot's actual cell (mid-route parking included), and it may be re-lifted;
at round end every task brick must rest at its goal.

Outputs plan.js in house-build's shared app.js PLAN format.
"""
from __future__ import annotations

import json
import random
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BINARY = REPO / "build-release" / "dd_benchmark"
HERE = Path(__file__).resolve().parent
WORK = HERE / "work_pile"

ROWS, COLS = 9, 16
PYR_R0, PYR_C0, PYR_BASE, PYR_LEVELS = 2, 2, 5, 3
MAX_H = 3                       # tallest allowed stack anywhere
RAIL = MAX_H + 2                # robot rail plane height
SRC = [(r, c) for r in range(2, 6) for c in range(10, 13)]   # 4x3, near-full
BUF = [(r, 8) for r in range(0, 9)] + [(0, c) for c in range(2, 7)]
ROBOT_STARTS = [(0, 9), (1, 9), (3, 9), (5, 9), (7, 9), (8, 9)]
HEX = {"blue": "#3b7dd8", "yellow": "#e8c531", "red": "#d84a3b"}
COLOR_BY_LEVEL = {1: "blue", 2: "yellow", 3: "red"}
MAX_TASKS = 64          # gantry: no congestion reason to cap a round
ROUND_TIME_LIMIT = 3.0
SEED = 7


def target_height(r, c):
    dr, dc = r - PYR_R0, c - PYR_C0
    if not (0 <= dr < PYR_BASE and 0 <= dc < PYR_BASE):
        return 0
    return min(min(dr, PYR_BASE - 1 - dr), min(dc, PYR_BASE - 1 - dc)) + 1


def write_instance(path, robots, tasks, world):
    starts = {s for s, _, _ in tasks}
    lines = ["name: pile_round", "map: |"]
    lines += ["  " + "." * COLS for _ in range(ROWS)]
    # storage mask: parking is only physically meaningful at pile columns,
    # buffer columns, and THIS round's goal cells (warehouse corridors ↔
    # open floor: carry-through only, no mid-route parking)
    lines.append("storage_map: |")
    goal_cells = {g for _, g, _ in tasks}
    parkable = set(SRC) | set(BUF)
    for r in range(ROWS):
        row = ""
        for c in range(COLS):
            base = len(world[(r, c)]) - (1 if (r, c) in starts else 0)
            ok = ((r, c) in goal_cells or
                  ((r, c) in parkable and base < MAX_H))
            row += "S" if ok else "."
        lines.append("  " + row)
    lines.append("robots:")
    lines += [f"  - [{r}, {c}]" for r, c in robots]
    lines.append("shelves:")
    lines += [f"  - [{s[0]}, {s[1]}]" for s, _, _ in tasks]
    lines.append("targets:")
    for i, (s, g, _) in enumerate(tasks):
        lines += [f"  - id: b{i}", f"    start: [{s[0]}, {s[1]}]",
                  f"    goal: [{g[0]}, {g[1]}]"]
    lines.append("flags: {gantry: true}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    t0 = time.time()
    WORK.mkdir(parents=True, exist_ok=True)
    rng = random.Random(SEED)
    tmpl = {(r, c): target_height(r, c) for r in range(ROWS)
            for c in range(COLS) if target_height(r, c)}
    total = sum(tmpl.values())

    # world: every cell -> bottom-to-top list of brick colors
    world = defaultdict(list)
    bricks = [c for cell, h in tmpl.items()
              for c in (COLOR_BY_LEVEL[z] for z in range(1, h + 1))]
    rng.shuffle(bricks)
    for b in bricks:
        open_cols = [c for c in SRC if len(world[c]) < MAX_H]
        world[open_cols[rng.randrange(len(open_cols))]].append(b)
    robots = list(ROBOT_STARTS)

    # ---- visualization log (house-build PLAN format) ----
    paths = [[list(p)] for p in robots]
    carry = [[0] for _ in robots]
    terrain, picks = [], []
    for cell in SRC:
        for z, col in enumerate(world[cell], 1):
            terrain.append({"t": 0, "r": cell[0], "c": cell[1],
                            "h": z, "color": HEX[col]})
    t_global = 0

    def storage_cells():
        return [c for c in list(SRC) + list(BUF)]

    rnd = 0
    n_dig = 0
    while any(len(world[c]) < tmpl[c] for c in tmpl):
        rnd += 1
        frontier = {cell: COLOR_BY_LEVEL[len(world[cell]) + 1]
                    for cell in tmpl if len(world[cell]) < tmpl[cell]}
        need = sorted(frontier.items())
        tops = {c: world[c][-1] for c in storage_cells() if world[c]}

        tasks = []          # (start_cell, goal_cell, color)
        used_src, used_goal = set(), set()
        for gcell, col in need:      # deliveries
            cand = [s for s, tc in tops.items()
                    if tc == col and s not in used_src]
            if not cand:
                continue
            s = min(cand, key=lambda s: abs(s[0]-gcell[0])+abs(s[1]-gcell[1]))
            tasks.append((s, gcell, col))
            used_src.add(s)
            used_goal.add(gcell)
            if len(tasks) >= MAX_TASKS:
                break
        if len(tasks) < MAX_TASKS:   # digging
            unmet = [col for gcell, col in need if gcell not in used_goal]
            for col in unmet:
                dig = [s for s in storage_cells()
                       if s not in used_src and len(world[s]) >= 2
                       and col in world[s][:-1]]
                if not dig:
                    continue
                s = min(dig, key=lambda s: len(world[s]) - 1 - max(
                    i for i, b in enumerate(world[s][:-1]) if b == col))
                free = [b for b in BUF if not world[b]
                        and b not in used_goal and b not in used_src]
                if not free:
                    break
                g = min(free, key=lambda b: abs(b[0]-s[0])+abs(b[1]-s[1]))
                tasks.append((s, g, world[s][-1]))
                used_src.add(s)
                used_goal.add(g)
                n_dig += 1
                if len(tasks) >= MAX_TASKS:
                    break
        if not tasks:
            print(f"round {rnd}: deadlock, no feasible task")
            return 1

        ypath, ppath = WORK / f"round_{rnd}.yaml", WORK / f"round_{rnd}.plan"
        write_instance(ypath, robots, tasks, world)
        res = subprocess.run(
            [str(BINARY), str(ypath), str(ROUND_TIME_LIMIT), str(ppath),
             "0", "lacam"],
            capture_output=True, text=True, timeout=ROUND_TIME_LIMIT + 15)
        metrics = dict(kv.split("=") for kv in res.stdout.split() if "=" in kv)
        if metrics.get("solved") != "1":
            print(f"round {rnd}: NOT SOLVED\n{res.stdout}\n{res.stderr}")
            return 1

        # ---- faithful replay: drops land at the robot's actual cell ----
        live = {bid: {"cell": s, "color": col, "goal": g}
                for bid, (s, g, col) in enumerate(tasks)}
        loc = defaultdict(list)          # cell -> stack of live brick ids
        for bid, b in live.items():
            loc[b["cell"]].append(bid)
        bound = [None] * len(robots)
        for line in ppath.read_text().splitlines():
            toks = line.strip().split(";")
            if len(toks) != len(robots):
                continue
            t_global += 1
            for i, tok in enumerate(toks):
                parts = tok.split()
                if parts and parts[0] == "m":
                    robots[i] = (int(parts[1]), int(parts[2]))
                elif parts and parts[0] == "l" and loc[robots[i]]:
                    bid = loc[robots[i]].pop()
                    bound[i] = bid
                    stack = world[robots[i]]
                    assert stack and stack[-1] == live[bid]["color"]
                    stack.pop()
                    picks.append({"t": t_global, "agent": i,
                                  "r": robots[i][0], "c": robots[i][1]})
                    terrain.append({"t": t_global, "r": robots[i][0],
                                    "c": robots[i][1], "h": len(stack),
                                    "agent": i})
                elif parts and parts[0] == "d" and bound[i] is not None:
                    bid = bound[i]
                    bound[i] = None
                    cell = robots[i]
                    world[cell].append(live[bid]["color"])
                    assert len(world[cell]) <= MAX_H, f"over MAX_H at {cell}"
                    live[bid]["cell"] = cell
                    loc[cell].append(bid)
                    terrain.append({"t": t_global, "r": cell[0], "c": cell[1],
                                    "h": len(world[cell]), "agent": i,
                                    "color": HEX[live[bid]["color"]]})
                paths[i].append(list(robots[i]))
                carry[i].append(1 if bound[i] is not None else 0)
        assert all(b is None for b in bound)
        for bid, b in live.items():
            assert b["cell"] == b["goal"], f"brick {bid} ended off-goal: {b}"
        done = sum(len(world[c]) for c in tmpl)
        print(f"round {rnd}: tasks={len(tasks)} "
              f"(deliver {sum(1 for _, g, _ in tasks if g in tmpl)}, "
              f"dig {sum(1 for _, g, _ in tasks if g not in tmpl)}) "
              f"built {done}/{total} makespan={metrics['makespan']} "
              f"runtime_ms={metrics['runtime_ms']}")

    ok = all(world[c] == [COLOR_BY_LEVEL[z] for z in range(1, tmpl[c] + 1)]
             for c in tmpl)
    leftovers = sum(len(world[c]) for c in storage_cells())
    ok = ok and leftovers == 0
    plan = {
        "rows": ROWS, "cols": COLS, "T": t_global,
        "depot": [list(d) for d in SRC],
        "pyramid": {"r0": PYR_R0, "c0": PYR_C0,
                    "base": PYR_BASE, "levels": PYR_LEVELS},
        "agents": [{"name": f"a{i}", "path": paths[i], "carry": carry[i]}
                   for i in range(len(robots))],
        "terrain": terrain, "picks": picks,
        "meta": {"gantry": RAIL, "statsLine":
                 f"随机砖堆 → 彩色金字塔 · {total} 块 · {rnd} 轮 · 挖掘 {n_dig} 次"},
    }
    (HERE / "plan.js").write_text("const PLAN = " + json.dumps(plan) + ";\n",
                                  encoding="utf-8")
    print(f"done: rounds={rnd} T={t_global} wall={time.time()-t0:.2f}s "
          f"valid={'PASS' if ok else 'FAIL'} leftovers={leftovers}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
