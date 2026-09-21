#!/usr/bin/env python3
"""Red-brick retrieval demo: a box FULL of gray bricks, one red brick at
the very bottom of the center column.  Goal: dig it out and place it on
the TOP layer (it ends as the globally highest brick).

Reuses the gantry event loop of plan_dense.py: robots run on the C++
gantry Carrier-LaCAM (2D), hoists are uninterruptible timed services
(2 ticks per depth unit), replan after every hoist start/completion.
No template here -- every task is a 'park' move; the last park drops the
red brick onto the tallest column.

Usage: plan_red.py [--seed=N]
"""
from __future__ import annotations

import json
import os
import random
import subprocess
import sys
import time
from collections import defaultdict, deque
from pathlib import Path

REPO = Path("/local/home/yimint/research/dd-lacam-next")
BINARY = REPO / "build-release" / "dd_benchmark"
HERE = REPO / "demo" / "autostore"

NR = NC = 5
FILL_H = 5                      # box filled solid to this height
MAX_H = FILL_H + 2              # temporary headroom above the fill
RAIL = MAX_H + 2
RED_CELL = (2, 2)               # red brick buried at z=0 of this column
GRAY, RED = "gray", "red"
HEX = {GRAY: "#9aa4b2", RED: "#d84a3b"}
ROBOT_STARTS = [(0, 0), (0, 4), (4, 0), (4, 4)]
CELLS = [(r, c) for r in range(NR) for c in range(NC)]
SEED = int([a for a in sys.argv if a.startswith("--seed=")][0][7:]) \
    if any(a.startswith("--seed=") for a in sys.argv) else 7
WORK = HERE / "work_red"
OUT = HERE / "plan_red.js"
TIME_LIMIT = 0.25


def nbrs(cell):
    r, c = cell
    return [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= r + dr < NR and 0 <= c + dc < NC]


class World:
    def __init__(self):
        self.vox = defaultdict(dict)   # cell -> {z: color}

    def top_z(self, cell):
        return max(self.vox[cell]) if self.vox[cell] else -1

    def place(self, cell, z, color):
        assert z not in self.vox[cell] and z < MAX_H
        self.vox[cell][z] = color

    def remove_top(self, cell):
        z = self.top_z(cell)
        return z, self.vox[cell].pop(z)

    def find_red(self):
        for cell in CELLS:
            for z, col in self.vox[cell].items():
                if col == RED:
                    return cell, z
        return None, None


def main() -> int:
    t0 = time.time()
    work = WORK / f"s{SEED}"
    work.mkdir(parents=True, exist_ok=True)
    world = World()
    for cell in CELLS:                       # box filled solid
        for z in range(FILL_H):
            world.vox[cell][z] = RED if cell == RED_CELL and z == 0 else GRAY
    robots = list(ROBOT_STARTS)
    held = {}      # robot -> {"color", "goal": ("park", cell)}
    serving = {}   # robot -> {"kind", "cell", "remain", "task", ...}

    def hoist_ticks(z_stop):
        return max(1, 2 * (RAIL - 1 - z_stop))

    paths = [[list(p)] for p in robots]
    carry = [[0] for _ in robots]
    terrain, picks, hoists = [], [], []
    bricks_out = []
    stack_ids = defaultdict(dict)
    hand = {}
    def new_brick(color, r, c, z):
        bid = len(bricks_out)
        bricks_out.append({"color": HEX[color],
                           "segs": [["rest", 0, None, r, c, z]]})
        return bid
    def seg_close(bid, t):
        segs = bricks_out[bid]["segs"]
        if segs and segs[-1][2] is None:
            segs[-1][2] = t
    def seg_add(bid, seg):
        bricks_out[bid]["segs"].append(seg)
    for cell in CELLS:
        for z in sorted(world.vox[cell]):
            terrain.append({"t": 0, "r": cell[0], "c": cell[1],
                            "h": z + 1, "color": HEX[world.vox[cell][z]]})
            stack_ids[cell][z] = new_brick(world.vox[cell][z],
                                           cell[0], cell[1], z)
    t_global = 0
    replans = n_dig = 0

    def global_top():
        return max(world.top_z(c) for c in CELLS)

    def done():
        if held or serving:
            return False
        cell, z = world.find_red()
        return cell is not None and z == world.top_z(cell) and \
            z >= FILL_H and z == global_top()

    def park_spot(avoid, planned_lift, planned_drop, near):
        """lowest column first (spread the dig), then nearest"""
        cand = []
        for c in CELLS:
            if c in avoid or c in planned_lift or c in planned_drop:
                continue
            if world.top_z(c) + 1 >= MAX_H:
                continue
            cand.append((world.top_z(c),
                         abs(c[0] - near[0]) + abs(c[1] - near[1]), c))
        return min(cand)[2] if cand else None

    def red_goal(avoid, planned_lift, planned_drop):
        """tallest column with headroom: red ends the global top"""
        cand = []
        for c in CELLS:
            if c in avoid or c in planned_lift or c in planned_drop:
                continue
            if world.top_z(c) + 1 >= MAX_H:
                continue
            cand.append((-world.top_z(c), c))
        return min(cand)[1] if cand else None

    while not done() or serving:
        replans += 1
        if replans > 500:
            print("livelock: too many replans")
            return 1
        # ---- task compilation (trivial here: dig the red column) ----
        planned_lift = {sv["cell"] for sv in serving.values()}
        planned_drop = set(planned_lift)
        tasks = []                    # (start, goal, color, kind, z)
        carrying = {}
        for rob, info in sorted(held.items()):   # carried bricks first
            color = info["color"]
            if color == RED:
                g = red_goal({robots[rob]}, planned_lift, planned_drop)
            else:
                g = info["goal"][1] if info["goal"][1] not in planned_drop \
                    and world.top_z(info["goal"][1]) + 1 < MAX_H \
                    else park_spot({robots[rob]}, planned_lift,
                                   planned_drop, robots[rob])
            assert g is not None, "no drop slot left"
            carrying[rob] = len(tasks)
            planned_drop.add(g)
            tasks.append((robots[rob], g, color, "park", -1))
            held[rob]["goal"] = ("park", g)
        red_cell, red_z = world.find_red()
        if red_cell is not None and red_cell not in planned_lift:
            top = world.top_z(red_cell)
            if red_z == top:              # red is exposed: lift it out
                g = red_goal({red_cell}, planned_lift, planned_drop)
                if g is not None and not done():
                    planned_lift.add(red_cell)
                    planned_drop.add(g)
                    tasks.append((red_cell, g, RED, "park", -1))
            else:                         # dig: park the covering brick
                g = park_spot({red_cell}, planned_lift, planned_drop,
                              red_cell)
                if g is not None:
                    planned_lift.add(red_cell)
                    planned_drop.add(g)
                    tasks.append((red_cell, g,
                                  world.vox[red_cell][top], "park", -1))
        n_dig += sum(1 for t in tasks if t[2] != RED)
        if not tasks and not serving:
            if done():
                break
            print("deadlock: no feasible task")
            return 1

        # ---- serving cells are walls: connectivity filter ----
        tasks0, carrying0 = list(tasks), dict(carrying)
        while True:
            serving_cells0 = {sv["cell"] for sv in serving.values()}
            active = [i for i in range(len(robots)) if i not in serving]
            if not serving_cells0 or not tasks0:
                tasks, carrying = list(tasks0), dict(carrying0)
                break
            comp = {}
            for cid, i in enumerate(active):
                if robots[i] in comp or robots[i] in serving_cells0:
                    continue
                q = deque([robots[i]])
                comp[robots[i]] = cid
                while q:
                    cur = q.popleft()
                    for nb in nbrs(cur):
                        if nb not in comp and nb not in serving_cells0:
                            comp[nb] = cid
                            q.append(nb)
            keep2, carrying2 = [], {}
            grew = False
            for tid, t in enumerate(tasks0):
                rob_of = next((r for r, x in carrying0.items()
                               if x == tid), None)
                if rob_of is not None and rob_of in serving:
                    continue
                if rob_of is not None:
                    ok_t = t[1] in comp and \
                        comp[t[1]] == comp.get(robots[rob_of])
                else:
                    ok_t = t[0] in comp and t[1] in comp and \
                        comp[t[0]] == comp[t[1]]
                if ok_t:
                    if rob_of is not None:
                        carrying2[rob_of] = len(keep2)
                    keep2.append(t)
                elif rob_of is not None:
                    serving[rob_of] = {"kind": "freeze",
                                       "cell": robots[rob_of],
                                       "remain": 1, "task": t}
                    grew = True
            tasks, carrying = keep2, carrying2
            if not grew:
                break
        active = [i for i in range(len(robots)) if i not in serving]

        # ---- write instance, solve ----
        plan_lines = []
        if active and tasks:
            starts = {tasks[t][0] for t in range(len(tasks))
                      if t not in carrying.values()}
            goal_cells = {t[1] for t in tasks}
            serving_cells = {sv["cell"] for sv in serving.values()}
            lines = ["name: red_round", "map: |"]
            for r in range(NR):
                lines.append("  " + "".join(
                    "@" if (r, c) in serving_cells else "."
                    for c in range(NC)))
            lines.append("storage_map: |")
            for r in range(NR):
                row = ""
                for c in range(NC):
                    base = world.top_z((r, c)) + 1 - \
                        (1 if (r, c) in starts else 0)
                    ok = (((r, c) in goal_cells or (r, c) in starts)
                          and base < MAX_H and (r, c) not in serving_cells)
                    row += "S" if ok else "."
                lines.append("  " + row)
            lines.append("robots:")
            lines += [f"  - [{robots[i][0]}, {robots[i][1]}]" for i in active]
            lines.append("shelves:")
            lines += [f"  - [{t[0][0]}, {t[0][1]}]" for t in tasks]
            lines.append("targets:")
            for i, t in enumerate(tasks):
                lines += [f"  - id: b{i}",
                          f"    start: [{t[0][0]}, {t[0][1]}]",
                          f"    goal: [{t[1][0]}, {t[1][1]}]"]
            lines.append("flags: {gantry: true}")
            act_index = {g: k for k, g in enumerate(active)}
            if carrying:
                lines.append("carrying:")
                lines += [f"  - [{act_index[rob]}, {t}]"
                          for rob, t in sorted(carrying.items())]
            ypath = work / f"plan_{replans}.yaml"
            ppath = work / f"plan_{replans}.plan"
            ypath.write_text("\n".join(lines) + "\n", encoding="utf-8")
            metrics = {}
            for tl, sd in ((TIME_LIMIT, 0), (1.0, 0), (10.0, 0),
                           (10.0, 1), (10.0, 2)):
                res = subprocess.run(
                    [str(BINARY), str(ypath), str(tl), str(ppath),
                     str(sd), "lacam"],
                    capture_output=True, text=True, timeout=tl + 15,
                    env={**os.environ, "DD_FIRST_SOLUTION_ONLY": "1"})
                metrics = dict(kv.split("=") for kv in res.stdout.split()
                               if "=" in kv)
                if metrics.get("solved") == "1":
                    break
            if metrics.get("solved") != "1" and serving:
                for rob, tid in list(carrying.items()):
                    serving[rob] = {"kind": "freeze",
                                    "cell": robots[rob],
                                    "remain": 1, "task": tasks[tid]}
                tasks, carrying = [], {}
                metrics = {"solved": "1"}
            if metrics.get("solved") != "1":
                print(f"replan {replans}: NOT SOLVED")
                return 1
            plan_lines = ppath.read_text().splitlines() \
                if ppath.exists() and tasks else []

        # ---- replay until the next hoist start/completion event ----
        bound = dict(carrying)
        loc = defaultdict(list)
        for t in range(len(tasks)):
            if t not in bound.values():
                loc[tasks[t][0]].append(t)
        completed = False
        li = 0
        while not completed:
            toks = None
            if li < len(plan_lines):
                cand = plan_lines[li].strip().split(";")
                li += 1
                if len(cand) != len(active):
                    continue
                toks = cand
            elif not serving:
                break
            t_global += 1
            if toks is not None:
                for k, i in enumerate(active):
                    parts = toks[k].split()
                    if parts and parts[0] == "m":
                        robots[i] = (int(parts[1]), int(parts[2]))
                    elif parts and parts[0] == "l" and loc[robots[i]]:
                        tid = loc[robots[i]].pop()
                        cell = robots[i]
                        serving[i] = {
                            "kind": "lift", "cell": cell,
                            "remain": hoist_ticks(world.top_z(cell)),
                            "t0": t_global, "z": world.top_z(cell),
                            "task": tasks[tid]}
                        completed = True
                    elif parts and parts[0] == "d" and i in bound:
                        tid = bound.pop(i)
                        cell = robots[i]
                        z_stop = world.top_z(cell) + 1
                        serving[i] = {
                            "kind": "drop", "cell": cell,
                            "remain": hoist_ticks(z_stop),
                            "t0": t_global, "z": z_stop,
                            "task": tasks[tid]}
                        completed = True
            for rob in sorted(serving):
                sv = serving[rob]
                sv["remain"] -= 1
                if sv["remain"] > 0:
                    continue
                s_, g, color, kind, z = sv["task"]
                cell = sv["cell"]
                if sv["kind"] == "freeze":
                    held[rob] = {"color": color, "goal": ("park", g)}
                    bound[rob] = -1
                    del serving[rob]
                    completed = True
                    continue
                if sv["kind"] == "lift":
                    zz, col2 = world.remove_top(cell)
                    assert col2 == color
                    bid = stack_ids[cell].pop(zz)
                    hand[rob] = bid
                    seg_close(bid, sv["t0"])
                    seg_add(bid, ["hoist", sv["t0"], t_global,
                                  cell[0], cell[1], zz, "lift"])
                    seg_add(bid, ["ride", t_global, None, rob])
                    picks.append({"t": t_global, "agent": rob,
                                  "r": cell[0], "c": cell[1]})
                    terrain.append({"t": t_global, "r": cell[0],
                                    "c": cell[1],
                                    "h": world.top_z(cell) + 1,
                                    "agent": rob})
                    bound[rob] = tasks.index(sv["task"]) \
                        if sv["task"] in tasks else -1
                    held[rob] = {"color": color, "goal": ("park", g)}
                else:
                    put_z = world.top_z(cell) + 1
                    world.place(cell, put_z, color)
                    bid = hand.pop(rob)
                    stack_ids[cell][put_z] = bid
                    seg_close(bid, sv["t0"])
                    seg_add(bid, ["hoist", sv["t0"], t_global,
                                  cell[0], cell[1], put_z, "drop"])
                    seg_add(bid, ["rest", t_global, None,
                                  cell[0], cell[1], put_z])
                    terrain.append({"t": t_global, "r": cell[0],
                                    "c": cell[1], "h": put_z + 1,
                                    "agent": rob, "color": HEX[color]})
                    held.pop(rob, None)
                if sv["kind"] in ("lift", "drop"):
                    hoists.append({"agent": rob, "kind": sv["kind"],
                                   "r": cell[0], "c": cell[1],
                                   "z": sv.get("z", 0),
                                   "color": HEX[sv["task"][2]],
                                   "t0": sv.get("t0", t_global),
                                   "t1": t_global})
                del serving[rob]
                completed = True
            for i in range(len(robots)):
                paths[i].append(list(robots[i]))
                in_air = i in bound or (
                    i in serving and serving[i]["kind"] == "drop")
                carry[i].append(1 if in_air else 0)
        new_held = {}
        for rob, tid in bound.items():
            if tid >= 0:
                s_, g, color, kind, z = tasks[tid]
                new_held[rob] = {"color": color, "goal": ("park", g)}
            elif rob in held:
                new_held[rob] = held[rob]
        held = new_held

    red_cell, red_z = world.find_red()
    ok = done()
    total = sum(len(v) for v in world.vox.values())
    plan = {
        "rows": NR, "cols": NC, "T": t_global,
        "depot": [list(RED_CELL)],
        "agents": [{"name": f"a{i}", "path": paths[i], "carry": carry[i]}
                   for i in range(len(robots))],
        "terrain": terrain, "picks": picks, "hoists": hoists,
        "bricks": bricks_out,
        "meta": {"gantry": RAIL, "statsLine":
                 f"满箱红砖检索 · {total} 块(红砖埋于底部) · "
                 f"{replans} 次规划 · 挖掘 {n_dig} 次 · "
                 f"红砖最终 z={red_z}"},
    }
    OUT.write_text("const PLAN = " + json.dumps(plan) + ";\n",
                   encoding="utf-8")
    print(f"done: replans={replans} T={t_global} red={red_cell},z={red_z} "
          f"bricks={total} wall={time.time()-t0:.2f}s "
          f"valid={'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
