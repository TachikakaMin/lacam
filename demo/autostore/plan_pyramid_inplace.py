#!/usr/bin/env python3
"""Event-driven gantry pyramid: replan globally after EVERY completed drop.

Loop:
  - tasks = committed bricks of carrying robots (kept binding, cold-started
    via the instance `carrying` field) + fresh frontier deliveries + digs
  - solve one Carrier-LaCAM gantry instance
  - execute the plan only up to (and including) the first step with a DROP,
    apply real state changes (stacks, robot cells, lifts in transit)
  - replan; robots already holding a brick keep it and join path planning

Outputs plan.js in house-build's shared app.js PLAN format.
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BINARY = REPO / "build-release" / "dd_benchmark"
HERE = Path(__file__).resolve().parent
WORK = HERE / "work_inplace"

ROWS, COLS = 9, 5
PYR_R0, PYR_C0, PYR_BASE, PYR_LEVELS = 2, 0, 5, 3
MAX_H = 3
RAIL = MAX_H + 2
# in-place rebuild: the random pile sits ON the pyramid footprint itself
SRC = [(r, c) for r in range(PYR_R0, PYR_R0 + PYR_BASE)
       for c in range(PYR_C0, PYR_C0 + PYR_BASE)]
# width == pyramid base: buffers only above and below the site
BUF = [(r, c) for r in (0, 1, 7, 8) for c in range(5)]
ROBOT_STARTS = [(0, 0), (0, 2), (0, 4), (8, 0), (8, 2), (8, 4)]
HEX = {"blue": "#3b7dd8", "yellow": "#e8c531", "red": "#d84a3b"}
COLOR_BY_LEVEL = {1: "blue", 2: "yellow", 3: "red"}
TIME_LIMIT = 2.5
SEED = 2


def target_height(r, c):
    dr, dc = r - PYR_R0, c - PYR_C0
    if not (0 <= dr < PYR_BASE and 0 <= dc < PYR_BASE):
        return 0
    return min(min(dr, PYR_BASE - 1 - dr), min(dc, PYR_BASE - 1 - dc)) + 1


def write_instance(path, robots, tasks, carrying, world):
    """tasks: list of (start_cell, goal_cell, color); carrying: robot->task
    index for tasks whose brick is already held (start == robot cell)."""
    starts = {tasks[t][0] for t in range(len(tasks))
              if t not in carrying.values()}
    lines = ["name: inplace_round", "map: |"]
    lines += ["  " + "." * COLS for _ in range(ROWS)]
    lines.append("storage_map: |")
    goal_cells = {g for _, g, _ in tasks}
    for r in range(ROWS):
        row = ""
        for c in range(COLS):
            base = len(world[(r, c)]) - (1 if (r, c) in starts else 0)
            # gantry: no mid-route parking at all; a brick may only rest
            # at a task start (it is grounded there) or a task goal
            ok = (((r, c) in goal_cells or (r, c) in starts)
                  and base < MAX_H)
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
    if carrying:
        lines.append("carrying:")
        lines += [f"  - [{rob}, {t}]" for rob, t in sorted(carrying.items())]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    t0 = time.time()
    WORK.mkdir(parents=True, exist_ok=True)
    import random
    rng = random.Random(SEED)
    tmpl = {(r, c): target_height(r, c) for r in range(ROWS)
            for c in range(COLS) if target_height(r, c)}
    total = sum(tmpl.values())

    world = defaultdict(list)
    bricks = [c for cell, h in tmpl.items()
              for c in (COLOR_BY_LEVEL[z] for z in range(1, h + 1))]
    rng.shuffle(bricks)
    for b in bricks:
        cols_open = [c for c in SRC if len(world[c]) < MAX_H]
        world[cols_open[rng.randrange(len(cols_open))]].append(b)
    robots = list(ROBOT_STARTS)
    held = {}   # robot -> {"color", "goal"} for bricks in hand

    paths = [[list(p)] for p in robots]
    carry = [[0] for _ in robots]
    terrain, picks = [], []
    for cell in SRC:
        for z, col in enumerate(world[cell], 1):
            terrain.append({"t": 0, "r": cell[0], "c": cell[1],
                            "h": z, "color": HEX[col]})
    t_global = 0
    replans = n_dig = 0
    sticky = {}

    def storage_cells():
        return list(SRC) + list(BUF)

    def template_done():
        return all(world[c] == [COLOR_BY_LEVEL[z]
                                for z in range(1, tmpl[c] + 1)]
                   for c in tmpl)
    while not template_done() or held:
        replans += 1
        # ---- build task set ----
        tasks, carrying = [], {}
        used_goal = set()
        def goal_still_valid(g, color):
            if g in used_goal:
                return False
            if g in tmpl:
                stack = world[g]
                want = [COLOR_BY_LEVEL[z] for z in range(1, tmpl[g] + 1)]
                return (stack == want[:len(stack)] and
                        len(stack) < tmpl[g] and
                        want[len(stack)] == color)
            return not world[g]          # buffer goal: still empty
        for rob, info in sorted(held.items()):     # committed bricks first
            g, color = info["goal"], info["color"]
            if not goal_still_valid(g, color):     # commitment expired
                fresh = [cell for cell in sorted(tmpl)
                         if goal_still_valid(cell, color)]
                if fresh:
                    g = min(fresh, key=lambda c:
                            abs(c[0]-robots[rob][0])+abs(c[1]-robots[rob][1]))
                else:
                    empty = [b for b in BUF if not world[b]
                             and b not in used_goal]
                    g = min(empty, key=lambda c:
                            abs(c[0]-robots[rob][0])+abs(c[1]-robots[rob][1]))
            carrying[rob] = len(tasks)
            tasks.append((robots[rob], g, color))
            used_goal.add(g)
        frontier, mismatch = {}, []
        for cell in sorted(tmpl):
            stack = world[cell]
            want = [COLOR_BY_LEVEL[z] for z in range(1, tmpl[cell] + 1)]
            if stack == want[:len(stack)]:
                if len(stack) < tmpl[cell] and cell not in used_goal:
                    frontier[cell] = want[len(stack)]
            elif cell not in used_goal:
                mismatch.append(cell)   # mis-parked brick: rework needed
        def is_installed(c):
            # a correct template prefix is finished structure, NOT stock
            if c not in tmpl:
                return False
            want = [COLOR_BY_LEVEL[z] for z in range(1, tmpl[c] + 1)]
            return world[c] == want[:len(world[c])]
        tops = {c: world[c][-1] for c in storage_cells()
                if world[c] and not is_installed(c)}
        used_src = set()
        for gcell, col in sorted(frontier.items()):
            cand = [s for s, tc in tops.items()
                    if tc == col and s not in used_src]
            if not cand:
                continue
            prev = sticky.get(gcell)
            if prev in cand:
                s = prev
            else:
                s = min(cand,
                        key=lambda s: abs(s[0]-gcell[0])+abs(s[1]-gcell[1]))
            sticky[gcell] = s
            tasks.append((s, gcell, col))
            used_src.add(s)
            used_goal.add(gcell)
        for cell in mismatch:           # rework: dig the wrong brick out
            if cell in used_src:
                continue
            free = [b for b in BUF if not world[b]
                    and b not in used_goal and b not in used_src]
            if not free:
                break
            g = min(free, key=lambda b: abs(b[0]-cell[0])+abs(b[1]-cell[1]))
            tasks.append((cell, g, world[cell][-1]))
            used_src.add(cell)
            used_goal.add(g)
            n_dig += 1
        unmet = [col for gcell, col in sorted(frontier.items())
                 if gcell not in used_goal]
        for col in unmet:                          # digging
            def diggable(s):
                if s in used_src or len(world[s]) < 2:
                    return False
                if col not in world[s][:-1]:
                    return False
                if s in tmpl:   # never unbuild a correct template prefix
                    want = [COLOR_BY_LEVEL[z] for z in range(1, tmpl[s] + 1)]
                    return world[s] != want[:len(world[s])]
                return True
            dig = [s for s in storage_cells() if diggable(s)]
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
        if not tasks:
            print("deadlock: no feasible task")
            print("frontier:", dict(sorted(frontier.items())))
            print("held:", held)
            piles_state = {c: world[c] for c in storage_cells() if world[c]}
            print("piles/buffers:", piles_state)
            return 1

        ypath = WORK / f"plan_{replans}.yaml"
        ppath = WORK / f"plan_{replans}.plan"
        write_instance(ypath, robots, tasks, carrying, world)
        metrics = {}
        for tl in (TIME_LIMIT, 10.0):      # rescue retry on hard states
            res = subprocess.run(
                [str(BINARY), str(ypath), str(tl), str(ppath),
                 "0", "lacam"],
                capture_output=True, text=True, timeout=tl + 15)
            metrics = dict(kv.split("=") for kv in res.stdout.split()
                           if "=" in kv)
            if metrics.get("solved") == "1":
                break
        if metrics.get("solved") != "1":
            print(f"replan {replans}: NOT SOLVED\n{res.stdout[-400:]}")
            return 1

        # ---- execute up to (and including) the first step with a drop ----
        bound = {rob: t for rob, t in carrying.items()}   # robot -> task idx
        loc = defaultdict(list)
        for t, (s, g, col) in enumerate(tasks):
            if t not in bound.values():
                loc[s].append(t)
        dropped = False
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
                    tid = loc[robots[i]].pop()
                    bound[i] = tid
                    stack = world[robots[i]]
                    assert stack and stack[-1] == tasks[tid][2]
                    stack.pop()
                    picks.append({"t": t_global, "agent": i,
                                  "r": robots[i][0], "c": robots[i][1]})
                    terrain.append({"t": t_global, "r": robots[i][0],
                                    "c": robots[i][1], "h": len(stack),
                                    "agent": i})
                elif parts and parts[0] == "d" and i in bound:
                    tid = bound.pop(i)
                    cell = robots[i]
                    world[cell].append(tasks[tid][2])
                    assert len(world[cell]) <= MAX_H
                    terrain.append({"t": t_global, "r": cell[0], "c": cell[1],
                                    "h": len(world[cell]), "agent": i,
                                    "color": HEX[tasks[tid][2]]})
                    # replan only on a COMPLETION (drop at the task goal);
                    # a mid-route parking drop must not truncate, or a
                    # park-at-start plan prefix livelocks the loop
                    if cell == tasks[tid][1]:
                        dropped = True
                paths[i].append(list(robots[i]))
                carry[i].append(1 if i in bound else 0)
            if dropped:
                break   # world changed: replan everything
        held = {rob: {"color": tasks[tid][2], "goal": tasks[tid][1]}
                for rob, tid in bound.items()}
        if replans % 10 == 0 or not dropped:
            built = sum(len(world[c]) for c in tmpl)
            print(f"replan {replans}: built={built}/{total} tasks={len(tasks)} "
                  f"held={len(held)} dropped={dropped} t={t_global}", flush=True)

    ok = all(world[c] == [COLOR_BY_LEVEL[z] for z in range(1, tmpl[c] + 1)]
             for c in tmpl)
    # in-place rebuild: template columns ARE the (former) pile area, so
    # leftovers = bricks outside the finished template (buffers + surplus)
    leftovers = sum(len(world[c]) for c in BUF)
    leftovers += sum(max(0, len(world[c]) - tmpl.get(c, 0))
                     for c in SRC)
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
                 f"窄带就地重建(宽度=塔底) · {total} 块 · {replans} 次规划 · 挖掘 {n_dig} 次"},
    }
    (HERE / "plan.js").write_text("const PLAN = " + json.dumps(plan) + ";\n",
                                  encoding="utf-8")
    print(f"done: replans={replans} T={t_global} wall={time.time()-t0:.2f}s "
          f"valid={'PASS' if ok else 'FAIL'} leftovers={leftovers}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
