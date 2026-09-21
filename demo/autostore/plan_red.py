#!/usr/bin/env python3
"""Bottom-layer-to-top: the WHOLE bottom layer is red.

5x5 box, every column: red at z=0, gray at z=1..3 (100 bricks).  One
layer of headroom exists above the fill (MAX_H = FILL_H + 1).  Goal:
every column ends with its red brick on TOP (gray z=0..2, red z=3) --
the red bottom layer migrates to the top layer.

Solved column by column in snake order: dig the focus column's grays
into neighbors' headroom, hold the red airborne, refill the bottom with
gray donors (over-full columns first -- that is how temporarily buried
bricks come back -- then unprocessed columns' tops), cap with red.

Reuses the gantry event loop: hoists are timed uninterruptible
services, robots in service are walls, replan at every hoist event.
Drops are accepted only at the task goal (the C++ carrier layer may
legally drop at any storage cell; premature drops are intercepted and
re-planned).

Usage: plan_red.py [--seed=N]
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from collections import defaultdict, deque
from pathlib import Path

REPO = Path("/local/home/yimint/research/dd-lacam-next")
BINARY = REPO / "build-release" / "dd_benchmark"
HERE = REPO / "demo" / "autostore"

MINI = "--mini" in sys.argv
NR = NC = 3 if MINI else 5
FILL_H = 3 if MINI else 4       # fill: red z0 + gray above
MAX_H = FILL_H + 1              # exactly ONE headroom layer
RAIL = MAX_H + 2
GRAY, RED = "gray", "red"
HEX = {GRAY: "#9aa4b2", RED: "#d84a3b"}
if MINI:
    ROBOT_STARTS = [(0, 0), (0, 2), (2, 0), (2, 2)]
    N_FOCI = 2
else:
    ROBOT_STARTS = [(0, 0), (0, 4), (4, 0), (4, 4),
                    (0, 2), (4, 2), (2, 0), (2, 4)]
    N_FOCI = 4                  # columns processed in parallel
CELLS = [(r, c) for r in range(NR) for c in range(NC)]
SEED = int([a for a in sys.argv if a.startswith("--seed=")][0][7:]) \
    if any(a.startswith("--seed=") for a in sys.argv) else 7
WORK = HERE / ("work_red_mini" if MINI else "work_red")
OUT = Path("/tmp/plan_red_mini.js") if MINI else HERE / "plan_red.js"
TIME_LIMIT = 0.25

SNAKE = []
for r in range(NR):
    cols = range(NC) if r % 2 == 0 else range(NC - 1, -1, -1)
    SNAKE += [(r, c) for c in cols]


def nbrs(cell):
    r, c = cell
    return [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= r + dr < NR and 0 <= c + dc < NC]


def dist(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class World:
    def __init__(self):
        self.vox = defaultdict(dict)   # cell -> {z: color}

    def top_z(self, cell):
        return max(self.vox[cell]) if self.vox[cell] else -1

    def top_color(self, cell):
        z = self.top_z(cell)
        return self.vox[cell][z] if z >= 0 else None

    def place(self, cell, z, color):
        assert z not in self.vox[cell] and z < MAX_H
        self.vox[cell][z] = color

    def remove_top(self, cell):
        z = self.top_z(cell)
        return z, self.vox[cell].pop(z)


def main() -> int:
    t0 = time.time()
    work = WORK / f"s{SEED}"
    work.mkdir(parents=True, exist_ok=True)
    world = World()
    for cell in CELLS:
        world.vox[cell][0] = RED
        for z in range(1, FILL_H):
            world.vox[cell][z] = GRAY
    robots = list(ROBOT_STARTS)
    held = {}      # robot -> {"color", "goal", "kind"}
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
    replans = n_hoist = 0

    def column_done(cell):
        return len(world.vox[cell]) == FILL_H and \
            world.vox[cell].get(FILL_H - 1) == RED

    def column_buffer(cell):
        """structurally done, plus temporary parked grays on top: not a
        focus -- refill donors drain it (over-full columns rank first)"""
        return len(world.vox[cell]) > FILL_H and \
            world.vox[cell].get(FILL_H - 1) == RED

    def foci():
        out = [cell for cell in SNAKE
               if not column_done(cell) and not column_buffer(cell)]
        return out[:N_FOCI]

    def done():
        return not held and not serving and not foci()

    while not done() or serving:
        replans += 1
        if replans > 3000:
            print("livelock: too many replans")
            return 1
        FOCI = foci()
        fset = set(FOCI)
        planned = {sv["cell"] for sv in serving.values()}
        tasks = []                    # (start, goal, color, kind, z)
        carrying = {}
        def goals_now():
            return {t[1] for t in tasks}
        def starts_now():
            bound_tids = set(carrying.values())
            return {t[0] for i, t in enumerate(tasks)
                    if i not in bound_tids}
        red_home = {h["home"]: rob for rob, h in held.items()
                    if h["color"] == RED}

        def park_target(near, avoid):
            cand = []
            for cl in CELLS:
                if cl in avoid or cl in planned or cl in goals_now() \
                        or cl in fset:
                    continue
                if world.top_z(cl) + 1 >= MAX_H:
                    continue
                rank = 1 if column_done(cl) else 0   # avoid burying reds
                cand.append((rank, len(world.vox[cl]), dist(cl, near), cl))
            return min(cand)[3] if cand else None

        def pick_donor(target, avoid):
            cand = []
            for cl in CELLS:
                if cl in avoid or cl in planned or cl in goals_now() \
                        or cl in starts_now():
                    continue
                if world.top_color(cl) != GRAY:
                    continue
                over = len(world.vox[cl]) > FILL_H
                cand.append((0 if over else 1, dist(cl, target), cl))
            return min(cand)[2] if cand else None

        # ---- bound tasks: robots already loaded ----
        unpinned = []
        for rob, h in sorted(held.items()):
            if h["color"] == RED:
                continue
            g, kind = h["goal"], h["kind"]
            # preferred: refill a focus column whose red is airborne
            tc = h.get("home") if kind == "refill" or g in fset else None
            if tc is None or tc not in fset or tc not in red_home or \
               tc in planned or tc in goals_now() or \
               world.top_z(tc) + 1 > FILL_H - 2:
                tc = next((f for f in FOCI
                           if f in red_home and f not in planned
                           and f not in goals_now()
                           and world.top_z(f) + 1 <= FILL_H - 2), None) \
                    if kind == "refill" or g in fset else None
            if tc is not None:
                carrying[rob] = len(tasks)
                tasks.append((robots[rob], tc, GRAY, "refill", -1))
                held[rob] = {"color": GRAY, "goal": tc,
                             "kind": "refill", "home": tc}
                continue
            if kind == "park" and g not in planned and \
               g not in goals_now() and g not in fset and \
               world.top_z(g) + 1 < MAX_H:
                carrying[rob] = len(tasks)
                tasks.append((robots[rob], g, GRAY, "park", -1))
                continue
            g2 = park_target(robots[rob], set())
            if g2 is not None:
                carrying[rob] = len(tasks)
                tasks.append((robots[rob], g2, GRAY, "park", -1))
                held[rob] = {"color": GRAY, "goal": g2, "kind": "park"}
            else:
                unpinned.append(rob)
        # red caps: bound drops, accepted only when the last slot is open
        for tc, rob in sorted(red_home.items()):
            if tc in planned or tc in goals_now():
                unpinned.append(rob)
            elif world.top_z(tc) == FILL_H - 2:
                carrying[rob] = len(tasks)
                tasks.append((robots[rob], tc, RED, "redcap", -1))
            else:
                unpinned.append(rob)

        # ---- unbound tasks: one per focus column ----
        unbound = []
        def goals_all():
            return goals_now() | {t[1] for t in unbound}
        def starts_all():
            return starts_now() | {t[0] for t in unbound}
        n_free = len([i for i in range(len(robots))
                      if i not in held and i not in serving])
        max_unbound = max(1, n_free - 1)     # never saturate all robots
        for tc in FOCI:
            if len(unbound) >= max_unbound:
                break
            if tc in planned or tc in goals_all() or tc in starts_all():
                continue
            if tc in red_home:               # refill from a donor
                if world.top_z(tc) + 1 <= FILL_H - 2:
                    donor = pick_donor(tc, {tc} | starts_all()
                                       | goals_all())
                    if donor is not None:
                        unbound.append((donor, tc, GRAY, "refill", -1))
            elif world.top_color(tc) == GRAY:    # dig
                g = park_target(tc, {tc} | goals_all() | starts_all())
                if g is not None:
                    unbound.append((tc, g, GRAY, "park", -1))
            elif world.top_color(tc) == RED:     # extract the red
                g = park_target(tc, {tc} | goals_all() | starts_all())
                if g is not None:            # nominal goal only: the
                    unbound.append((tc, g, RED, "redlift", -1))
        # shield loaded-but-idle robots: pin each to a HOLD task whose
        # goal is any unused cell (never the robot's own cell: start==goal
        # is trivially completed by the solver).  The drop is intercepted
        # python-side; hold goals are storage-exempted when writing the
        # instance.
        hold_goals = set()
        for rob in unpinned:
            used = goals_all() | starts_all() | planned | hold_goals | \
                {t[0] for t in tasks} | {t[1] for t in tasks}
            g = next((cl for cl in sorted(
                CELLS, key=lambda x: dist(x, robots[rob]))
                if cl != robots[rob] and cl not in used), None)
            if g is not None:
                hold_goals.add(g)
                carrying[rob] = len(tasks)
                tasks.append((robots[rob], g, held[rob]["color"],
                              "hold", -1))
            else:
                serving[rob] = {"kind": "freeze", "cell": robots[rob],
                                "remain": 1,
                                "task": (robots[rob], held[rob]["goal"],
                                         held[rob]["color"],
                                         held[rob]["kind"], -1)}
        tasks += unbound

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
        if not tasks and not serving:
            if done():
                break
            print("deadlock: no feasible task")
            print("  foci:", FOCI, "held:", held)
            for cl in CELLS:
                if not column_done(cl):
                    print("  col", cl, {z: world.vox[cl][z][0]
                                        for z in sorted(world.vox[cl])})
            return 1

        # ---- write instance, solve (degrade ladder over task subsets:
        #      pathological combinations time out even though each part
        #      is solvable alone) ----
        plan_lines = []
        if active and tasks:
            def write_and_solve(tsk, carr, tag):
                starts = {tsk[t][0] for t in range(len(tsk))
                          if t not in carr.values()}
                goal_cells = {t[1] for t in tsk}
                hold_cells = {t[1] for t in tsk if t[3] == "hold"}
                serving_cells = {sv["cell"] for sv in serving.values()}
                lines = ["name: red_round", "map: |"]
                for r in range(NR):
                    lines.append("  " + "".join(
                        "@" if (r, c2) in serving_cells else "."
                        for c2 in range(NC)))
                lines.append("storage_map: |")
                for r in range(NR):
                    row = ""
                    for c2 in range(NC):
                        base = world.top_z((r, c2)) + 1 - \
                            (1 if (r, c2) in starts else 0)
                        ok = (((r, c2) in goal_cells or (r, c2) in starts)
                              and (base < MAX_H or (r, c2) in hold_cells)
                              and (r, c2) not in serving_cells)
                        row += "S" if ok else "."
                    lines.append("  " + row)
                lines.append("robots:")
                lines += [f"  - [{robots[i][0]}, {robots[i][1]}]"
                          for i in active]
                lines.append("shelves:")
                lines += [f"  - [{t[0][0]}, {t[0][1]}]" for t in tsk]
                lines.append("targets:")
                for i, t in enumerate(tsk):
                    lines += [f"  - id: b{i}",
                              f"    start: [{t[0][0]}, {t[0][1]}]",
                              f"    goal: [{t[1][0]}, {t[1][1]}]"]
                lines.append("flags: {gantry: true}")
                act_index = {g: k for k, g in enumerate(active)}
                if carr:
                    lines.append("carrying:")
                    lines += [f"  - [{act_index[rob]}, {t}]"
                              for rob, t in sorted(carr.items())]
                ypath = work / f"plan_{replans}{tag}.yaml"
                ppath = work / f"plan_{replans}{tag}.plan"
                ypath.write_text("\n".join(lines) + "\n", encoding="utf-8")
                for tl in (TIME_LIMIT, 2.0):
                    res = subprocess.run(
                        [str(BINARY), str(ypath), str(tl), str(ppath),
                         "0", "lacam"],
                        capture_output=True, text=True, timeout=tl + 15,
                        env={**os.environ, "DD_FIRST_SOLUTION_ONLY": "1"})
                    m = dict(kv.split("=") for kv in res.stdout.split()
                             if "=" in kv)
                    if m.get("solved") == "1":
                        return ppath.read_text().splitlines()
                return None
            bound_ids = sorted(carrying.values())
            subsets = [("", list(range(len(tasks))))]
            if bound_ids and len(bound_ids) < len(tasks):
                subsets.append(("b", bound_ids))
                subsets.append(("u", [t for t in range(len(tasks))
                                      if t not in bound_ids]))
            # finest level: each bound task alone (others frozen 1 tick)
            for j, tid in enumerate(bound_ids):
                if tasks[tid][3] != "hold":
                    subsets.append((f"s{j}", [tid]))
            solved_plan = None
            for tag, keep in subsets:
                remap = {o: n for n, o in enumerate(keep)}
                tsk = [tasks[t] for t in keep]
                carr = {r: remap[t] for r, t in carrying.items()
                        if t in remap}
                extra_frozen = []
                if tag.startswith("s"):
                    # everyone else with a brick freezes for this segment
                    for r2, t2 in carrying.items():
                        if t2 not in remap and r2 not in serving:
                            serving[r2] = {"kind": "freeze",
                                           "cell": robots[r2], "remain": 1,
                                           "task": tasks[t2]}
                            extra_frozen.append(r2)
                    active = [i for i in range(len(robots))
                              if i not in serving]
                solved_plan = write_and_solve(tsk, carr, tag)
                if solved_plan is not None:
                    tasks, carrying = tsk, carr
                    plan_lines = solved_plan
                    break
                for r2 in extra_frozen:
                    del serving[r2]
                active = [i for i in range(len(robots))
                          if i not in serving]
            if solved_plan is None:
                # last resort: freeze every carrier a tick, walls fall
                for rob, tid in list(carrying.items()):
                    serving[rob] = {"kind": "freeze",
                                    "cell": robots[rob],
                                    "remain": 1, "task": tasks[tid]}
                tasks, carrying, plan_lines = [], {}, []
                if not serving:
                    print(f"replan {replans}: NOT SOLVED")
                    return 1

        # ---- replay until the next hoist start/completion event ----
        bound = dict(carrying)
        loc = defaultdict(list)
        for t in range(len(tasks)):
            if t not in bound.values():
                loc[tasks[t][0]].append(t)
        completed = False
        li = 0
        ticks_this_seg = 0
        while not completed:
            toks = None
            if li < len(plan_lines):
                if ticks_this_seg >= 60 and not serving:
                    break            # wandering plan: cut and replan
                cand = plan_lines[li].strip().split(";")
                li += 1
                if len(cand) != len(active):
                    continue
                toks = cand
            elif not serving:
                break
            t_global += 1
            ticks_this_seg += 1
            if toks is not None:
                for k, i in enumerate(active):
                    parts = toks[k].split()
                    if parts and parts[0] == "m":
                        robots[i] = (int(parts[1]), int(parts[2]))
                    elif parts and parts[0] == "l" and loc[robots[i]] \
                            and i not in held:
                        tid = loc[robots[i]].pop()
                        cell = robots[i]
                        serving[i] = {
                            "kind": "lift", "cell": cell,
                            "remain": hoist_ticks(world.top_z(cell)),
                            "t0": t_global, "z": world.top_z(cell),
                            "task": tasks[tid]}
                        completed = True
                    elif parts and parts[0] == "d" and i in bound:
                        t_ = tasks[bound[i]]
                        if t_[3] == "hold" or robots[i] != t_[1]:
                            continue         # premature/fake: keep holding
                        if t_[3] == "redcap" and \
                           world.top_z(t_[1]) != FILL_H - 2:
                            continue
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
                    prev = held.get(rob) or {}
                    held[rob] = {"color": color, "goal": g, "kind": kind,
                                 "home": prev.get("home",
                                                  g if kind == "redcap"
                                                  else None)}
                    del serving[rob]
                    completed = True
                    continue
                if sv["kind"] == "lift":
                    zz, col2 = world.remove_top(cell)
                    assert col2 == color, (cell, zz, col2, color)
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
                    held[rob] = {"color": color, "goal": g, "kind": kind,
                                 "home": cell if kind == "redlift" else g}
                    n_hoist += 1
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
                carry[i].append(1 if i in hand else 0)
        if replans % 50 == 0:
            nd = sum(1 for cl in CELLS if column_done(cl))
            print(f"replan {replans}: columns {nd}/25 t={t_global}",
                  flush=True)

    ok = done()
    for cell in CELLS:
        ok = ok and len(world.vox[cell]) == FILL_H and \
            world.vox[cell].get(FILL_H - 1) == RED and \
            all(world.vox[cell].get(z) == GRAY for z in range(FILL_H - 1))
    total = sum(len(v) for v in world.vox.values())
    plan = {
        "rows": NR, "cols": NC, "T": t_global,
        "depot": [],
        "agents": [{"name": f"a{i}", "path": paths[i], "carry": carry[i]}
                   for i in range(len(robots))],
        "terrain": terrain, "picks": picks, "hoists": hoists,
        "bricks": bricks_out,
        "meta": {"gantry": RAIL, "statsLine":
                 f"底层红砖翻到顶层 · {total} 块(25 红) · "
                 f"{replans} 次规划 · {n_hoist} 次吊起"},
    }
    OUT.write_text("const PLAN = " + json.dumps(plan) + ";\n",
                   encoding="utf-8")
    print(f"done: replans={replans} T={t_global} hoists={n_hoist} "
          f"bricks={total} wall={time.time()-t0:.2f}s "
          f"valid={'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
