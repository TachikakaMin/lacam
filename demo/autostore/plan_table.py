#!/usr/bin/env python3
"""Build a TABLE in place: legs + a hovering tabletop (face connectivity).

The board IS the table footprint. Bricks start randomly piled on it.
World = voxels (cell -> {z: color}); a tabletop brick hovers over an empty
column, supported by lateral face connection to installed neighbors.

Physics enforced by the orchestrator (mini oracle, design doc §1/§3/§5):
  place(cell, z): channel above clear; template voxels below installed;
      NON-template slots below empty (anti-entombment); support = ground,
      installed voxel below, or installed lateral neighbor at z.
  remove(cell, z): top of column only; remaining voxels must all reach the
      ground through face adjacency (grounded-connectivity BFS).
Robots run on the C++ gantry Carrier-LaCAM (2D); replan after each
completed placement; carried bricks cold-start via `carrying`.

Usage: plan_table.py [--small]   (small = 3x3 stool, <1 min)
"""
from __future__ import annotations

import json
import random
import os
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

REPO = Path("/local/home/yimint/research/dd-lacam-next")
BINARY = REPO / "build-release" / "dd_benchmark"
HERE = REPO / "demo" / "autostore"

SMALL = "--small" in sys.argv
M = 0                           # board == final product footprint, exactly
if SMALL:
    F = 3                       # stool: legs z0 at corners, top slab z1
    LEG_H, TOP_Z = 1, 1
    WORK = HERE / "work_stool"
    OUT = Path("/tmp/plan_stool.js")
else:
    F = 5                       # table: legs z0-1 at corners, top slab z2
    LEG_H, TOP_Z = 2, 2
    WORK = HERE / "work_table"
    OUT = HERE / "plan.js"
MAX_H = TOP_Z + 2               # bricks may be parked ON the finished top
N = F + 2 * M                   # board side
RAIL = MAX_H + 2
FOOT = [(r, c) for r in range(M, M + F) for c in range(M, M + F)]
LEGS = [(M, M), (M, M + F - 1), (M + F - 1, M), (M + F - 1, M + F - 1)]
ROBOT_STARTS = ([(0, 1), (1, 0), (2, 1)] if SMALL else
                [(0, 1), (0, 3), (2, 0), (2, 4), (4, 1), (4, 3)])
LEG, TOP = "leg", "top"
HEX = {LEG: "#8b5a2b", TOP: "#e8a13c"}
TIME_LIMIT = 0.25   # event segments are short: first solution is enough
# commitment stickiness across replans: measured helpful on the table
# scenario, neutral-to-harmful on horse/dense (self-made cycles) -- so it
# is a per-scenario setting, overridable via DD_DEMO_STICKY
STICKY = os.environ.get("DD_DEMO_STICKY", "1") == "1"
SEED = int([a for a in sys.argv if a.startswith("--seed=")][0][7:]) \
    if any(a.startswith("--seed=") for a in sys.argv) else 7

CELLS = [(r, c) for r in range(N) for c in range(N)]
# template: footprint cell -> {z: color}; margin cells have no template
TMPL = {}
for cell in CELLS:
    col = {}
    if cell in FOOT:
        if cell in LEGS:
            for z in range(LEG_H):
                col[z] = LEG
        col[TOP_Z] = TOP
    TMPL[cell] = col
TOTAL = sum(len(v) for v in TMPL.values())


def nbrs(cell):
    r, c = cell
    return [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= r + dr < N and 0 <= c + dc < N]


class World:
    def __init__(self):
        self.vox = defaultdict(dict)   # cell -> {z: color}
        self.installed = set()         # {(cell, z)} template-correct placed

    def top_z(self, cell):
        return max(self.vox[cell]) if self.vox[cell] else -1

    def grounded_ok(self, removed=None):
        """all voxels reach ground via face adjacency (optionally with one
        voxel hypothetically removed)"""
        voxset = {(cell, z) for cell in self.vox for z in self.vox[cell]}
        if removed is not None:
            voxset.discard(removed)
        frontier = [v for v in voxset if v[1] == 0]
        seen = set(frontier)
        while frontier:
            cell, z = frontier.pop()
            adj = [(cell, z + 1), (cell, z - 1)] + \
                  [(n, z) for n in nbrs(cell)]
            for v in adj:
                if v in voxset and v not in seen:
                    seen.add(v)
                    frontier.append(v)
        return seen == voxset

    def placeable(self, cell, z):
        """completion placement legality for template voxel (cell, z)"""
        if (cell, z) in self.installed or z in self.vox[cell]:
            return False
        if self.top_z(cell) >= z:
            return False                       # channel / slot blocked
        for z2 in range(z):                    # below: template installed,
            if z2 in TMPL[cell]:               # non-template slots EMPTY
                if (cell, z2) not in self.installed:
                    return False
            elif z2 in self.vox[cell]:
                return False                   # anti-entombment
        if z == 0 or (cell, z - 1) in self.installed:
            return True
        return any((n, z) in self.installed for n in nbrs(cell))

    def removable(self, cell):
        """the column's top voxel, if it is stray (not installed), its
        channel is clear by definition, and removal keeps all grounded"""
        z = self.top_z(cell)
        if z < 0 or (cell, z) in self.installed:
            return None
        if not self.grounded_ok(removed=(cell, z)):
            return None
        return z

    def place(self, cell, z, color, completion):
        assert z not in self.vox[cell]
        self.vox[cell][z] = color
        if completion:
            self.installed.add((cell, z))
        assert self.grounded_ok()

    def remove_top(self, cell):
        z = self.top_z(cell)
        color = self.vox[cell].pop(z)
        return z, color


def main() -> int:
    t0 = time.time()
    global WORK
    WORK = WORK / f"s{SEED}"      # parallel runs need isolated workdirs
    WORK.mkdir(parents=True, exist_ok=True)
    rng = random.Random(SEED)
    world = World()
    bricks = [color for col in TMPL.values() for color in col.values()]
    rng.shuffle(bricks)
    for b in bricks:                    # random solid piles on the board
        open_cols = [c for c in FOOT if world.top_z(c) + 1 < MAX_H]
        cell = open_cols[rng.randrange(len(open_cols))]
        world.vox[cell][world.top_z(cell) + 1] = b
    # bricks that landed template-correct from z=0 upward count as installed
    for cell in CELLS:
        for z in sorted(TMPL[cell]):
            if z != 0 and (cell, z - 1) not in world.installed and \
               any(z2 not in TMPL[cell] and z2 in world.vox[cell]
                   for z2 in range(z)):
                break
            if world.vox[cell].get(z) == TMPL[cell][z] and \
               all(TMPL[cell].get(z2) == world.vox[cell].get(z2)
                   for z2 in range(z)):
                world.installed.add((cell, z))
            else:
                break
    robots = list(ROBOT_STARTS)
    held = {}      # robot -> {"color", "goal": (cell, z) or ("park", cell)}
    # serving state (agent_fable-style): a robot running a hoist is
    # committed -- it leaves the planning instance, its cell becomes a
    # wall, and interrupting is impossible by construction
    serving = {}   # robot -> {"kind": lift|drop, "cell", "remain", "task"}

    def hoist_ticks(z_stop):
        """rope travel: down from the rail to z_stop, then back up"""
        return max(1, 2 * (RAIL - 1 - z_stop))

    paths = [[list(p)] for p in robots]
    carry = [[0] for _ in robots]
    terrain, picks, hoists = [], [], []
    # persistent per-brick tracks: each physical brick is ONE object with a
    # timeline of segments -> the viewer never needs mesh handoffs
    bricks_out = []          # bid -> {"color", "segs": [...]}
    stack_ids = defaultdict(dict)   # cell -> {z: bid}
    hand = {}                # robot -> bid
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
    prev_tasks = []      # grounded commitments carried across segments

    def done():
        return (len(world.installed) == TOTAL and not held and
                sum(len(v) for v in world.vox.values()) == TOTAL)

    def pick_park(near, exclude):
        """prefer parking AHEAD of the build wavefront; behind the wave the
        finished tabletop may serve as temporary surface (last resort)"""
        cur = wave_cur()
        cur_i = OIDX[cur] if cur is not None else -1
        cand = []
        for c in CELLS:
            if c in exclude or c == cur:
                continue
            if world.top_z(c) + 1 >= MAX_H:
                continue
            if c not in OIDX:
                rank = 0                     # margin buffer ring
            elif OIDX[c] > cur_i + 1:
                rank = 1                     # ahead of the wave
            elif OIDX[c] == cur_i + 1:
                rank = 2                     # immediately next: avoid
            else:
                rank = 3                     # on finished structure
            cand.append((rank, abs(c[0]-near[0]) + abs(c[1]-near[1]),
                         OIDX.get(c, -1), c))
        return min(cand)[3] if cand else None

    SNAKE = []
    for r in range(M, M + F):
        cols = range(M, M + F) if (r - M) % 2 == 0 \
            else range(M + F - 1, M - 1, -1)
        SNAKE += [(r, c) for c in cols]
    if SNAKE[0] not in LEGS:            # start at a leg column
        SNAKE = SNAKE[::-1]
    OIDX = {cell: i for i, cell in enumerate(SNAKE)}

    def column_done(cell):
        return all((cell, z) in world.installed for z in TMPL[cell]) and \
            all(z in TMPL[cell] for z in world.vox[cell])

    def wave_cur():
        for cell in SNAKE:
            if not column_done(cell):
                return cell
        return None

    def reconcile():
        """adopt strays that happen to sit exactly at their template slot
        (color correct, prefix below correct) as installed structure"""
        for cell in CELLS:
            for z in sorted(TMPL[cell]):
                if (cell, z) in world.installed:
                    continue
                prefix_ok = all((cell, z2) in world.installed
                                for z2 in TMPL[cell] if z2 < z) and \
                    all(z2 in TMPL[cell] or z2 not in world.vox[cell]
                        for z2 in range(z))
                if prefix_ok and world.vox[cell].get(z) == TMPL[cell][z]:
                    world.installed.add((cell, z))
                else:
                    break

    while not done() or serving:
        replans += 1
        reconcile()
        if replans > 1500:
            print("livelock: too many replans")
            return 1
        # ================= mini Task-BR compiler (design doc §6) ======
        # One shared transaction over a simulated world copy.  Each root
        # goal (template voxel, build order) is compiled by recursively
        # ensuring its preconditions: column cleared -> park tasks,
        # support present -> build the supporting voxel first (OR-choice
        # over lateral neighbors, with rollback), source available ->
        # uncover buried bricks.  Held bricks are roots with a fixed
        # source.  Only tasks executable NOW are handed to the robots.
        why = []             # provenance, parallel to tasks
        sim = {c: dict(world.vox[c]) for c in CELLS}
        sim_inst = set(world.installed)
        planned_lift = set()          # cells losing their top this round
        planned_drop = set()          # cells receiving a drop this round
        for sv in serving.values():   # hoists in progress own their cell
            planned_lift.add(sv["cell"])
            planned_drop.add(sv["cell"])
        tasks = []                    # (start, goal, color, kind, z)
        carrying = {}

        def snapshot():
            return ({c: dict(sim[c]) for c in CELLS}, set(sim_inst),
                    set(planned_lift), set(planned_drop), len(tasks))

        def restore(sn):
            nonlocal tasks
            sv, si, pl, pd = sn[0], sn[1], sn[2], sn[3]
            for c in CELLS:
                sim[c] = dict(sv[c])
            sim_inst.clear(); sim_inst.update(si)
            planned_lift.clear(); planned_lift.update(pl)
            planned_drop.clear(); planned_drop.update(pd)
            del tasks[sn[4]:]

        def sim_top(c):
            return max(sim[c]) if sim[c] else -1

        def park_spot(near, avoid):
            """deterministic: finished-surface slots (landing above the
            template top) first, then late snake order; independent of
            robot positions so replans keep choosing the same spot"""
            cand = []
            for c in CELLS:
                if c in avoid or c in planned_drop or c in planned_lift:
                    continue
                if sim_top(c) + 1 >= MAX_H:
                    continue
                z_land = sim_top(c) + 1
                if z_land in TMPL[c] and (c, z_land) not in sim_inst:
                    rank = 2          # would squat a needed template slot
                elif all((c, zz) in sim_inst for zz in TMPL[c]):
                    rank = 0          # on top of finished structure: safe
                else:
                    rank = 1
                # spread parking: prefer LOW columns first (rank), then
                # late snake order; deterministic, robot-independent
                cand.append((rank, sim_top(c) + 1, -OIDX[c], c))
            return min(cand)[3] if cand else None

        def sim_park_top(cell, reason="?"):
            """emit a park task for cell's top stray (in sim)"""
            z = sim_top(cell)
            if z < 0 or (cell, z) in sim_inst or cell in planned_lift:
                return False
            p = park_spot(cell, {cell})
            if p is None:
                return False
            color = sim[cell].pop(z)
            sim[p][sim_top(p) + 1] = color
            planned_lift.add(cell)
            planned_drop.add(p)
            tasks.append((cell, p, color, "park", -1))
            why.append(reason)
            return True

        def sim_placeable(cell, z):
            if (cell, z) in sim_inst or z in sim[cell]:
                return False
            if sim_top(cell) >= z:
                return False
            for z2 in range(z):
                if z2 in TMPL[cell]:
                    if (cell, z2) not in sim_inst:
                        return False
                elif z2 in sim[cell]:
                    return False
            if z == 0 or (cell, z - 1) in sim_inst:
                return True
            return any((n, z) in sim_inst for n in nbrs(cell))

        def sim_find_source(color, avoid):
            """top-accessible stray of this color; None if all buried"""
            best = None
            for c in CELLS:
                if c in avoid or c in planned_lift:
                    continue
                z = sim_top(c)
                if z >= 0 and (c, z) not in sim_inst and \
                   sim[c][z] == color:
                    d = OIDX[c]
                    if best is None or d < best[0]:
                        best = (d, c)
            return best[1] if best else None

        def sim_uncover(color, avoid, budget, reason="?"):
            """park covering strays until a brick of `color` surfaces"""
            for c in CELLS:
                if c in avoid or c in planned_lift:
                    continue
                buried = [z for z in sim[c]
                          if (c, z) not in sim_inst and sim[c][z] == color
                          and z != sim_top(c)]
                if not buried:
                    continue
                depth = sim_top(c) - max(buried)
                if depth > budget:
                    continue
                ok = True
                for _ in range(depth):
                    if not sim_park_top(c, "uncover:" + reason):
                        ok = False
                        break
                    planned_lift.discard(c)   # multi-lift same col allowed
                if ok:
                    return c
            return None

        def try_build(cell, z, depth, visiting, root="?"):
            """ensure template voxel (cell,z) is installed in sim"""
            if (cell, z) in sim_inst:
                return True
            if depth <= 0 or (cell, z) in visiting:
                return False
            if cell in planned_drop or cell in planned_lift:
                return False           # column owned by an in-flight hoist
            visiting = visiting | {(cell, z)}
            # 1) clear the column: strays anywhere in it must leave
            guard = 0
            while any(z2 not in TMPL[cell] and z2 in sim[cell]
                      for z2 in range(MAX_H)) or \
                  any(z2 in sim[cell] and (cell, z2) not in sim_inst
                      for z2 in sim[cell]):
                planned_lift.discard(cell)
                if not sim_park_top(cell, f"clear{cell}<-{root}") or \
                   guard > MAX_H:
                    return False
                guard += 1
            # 2) template prefix below
            for z2 in sorted(TMPL[cell]):
                if z2 >= z:
                    break
                if not try_build(cell, z2, depth - 1, visiting,
                                 f"prefix<-{root}"):
                    return False
            # 3) support: ground/below/lateral (OR-choice with rollback)
            if not (z == 0 or (cell, z - 1) in sim_inst):
                laterals = [n for n in nbrs(cell) if TMPL[n].get(z)]
                laterals.sort(key=lambda n: OIDX[n])
                ok = False
                for n in laterals:
                    if (n, z) in sim_inst:
                        ok = True
                        break
                    sn = snapshot()
                    if try_build(n, z, depth - 1, visiting,
                                 f"support({cell},{z})<-{root}"):
                        ok = True
                        break
                    restore(sn)
                if not ok:
                    return False
            # 4) source brick
            color = TMPL[cell][z]
            src = sim_find_source(color, {cell})
            if src is None:
                if sim_uncover(color, {cell}, budget=2,
                               reason=f"src({cell},{z})<-{root}") is None:
                    return False
                src = sim_find_source(color, {cell})
                if src is None:
                    return False
            zs = sim_top(src)
            sim[src].pop(zs)
            sim[cell][z] = color
            sim_inst.add((cell, z))
            planned_lift.add(src)
            planned_drop.add(cell)
            tasks.append((src, cell, color, "build", z))
            why.append(f"build<-{root}")
            return True

        # ---- roots: held bricks first (fixed source in hand) ----
        for rob, info in sorted(held.items()):
            color = info["color"]
            goal = None
            prev_goal = info.get("goal")
            cells_order = sorted(FOOT, key=lambda c: OIDX[c])
            if prev_goal and prev_goal[0] != "park" and prev_goal[0] in TMPL:
                cells_order = [prev_goal[0]] + \
                    [c for c in cells_order if c != prev_goal[0]]
            for gcell in cells_order:
                for gz in sorted(TMPL[gcell]):
                    if TMPL[gcell][gz] != color or \
                       (gcell, gz) in sim_inst or gcell in planned_drop:
                        continue
                    sn = snapshot()
                    # compile prerequisites but NOT the source (in hand)
                    ok = True
                    guard = 0
                    while any(z2 in sim[gcell] and
                              (gcell, z2) not in sim_inst
                              for z2 in sim[gcell]):
                        if not sim_park_top(gcell) or guard > MAX_H:
                            ok = False
                            break
                        planned_lift.discard(gcell)
                        guard += 1
                    if ok:
                        for z2 in sorted(TMPL[gcell]):
                            if z2 >= gz:
                                break
                            ok = ok and try_build(gcell, z2, 6, set())
                    if ok and not (gz == 0 or (gcell, gz-1) in sim_inst):
                        ok = any((n, gz) in sim_inst for n in nbrs(gcell)) \
                            or any(try_build(n, gz, 6, set())
                                   for n in sorted(
                                       [n for n in nbrs(gcell)
                                        if TMPL[n].get(gz)],
                                       key=lambda n: OIDX[n]))
                    if ok and sim_placeable(gcell, gz) and \
                       world.placeable(gcell, gz):
                        goal = (gcell, gz)     # deliverable RIGHT NOW
                        break
                    restore(sn)
                if goal:
                    break
            if goal is None and prev_goal and prev_goal[0] != "park":
                pg, pz = prev_goal
                if STICKY and info.get("waits", 0) < 6 and \
                   TMPL[pg].get(pz) == color and \
                   (pg, pz) not in world.installed and \
                   pz not in world.vox[pg]:
                    info["waits"] = info.get("waits", 0) + 1
                    _w = info["waits"]
                    # the goal slot still exists; support/clearing is just
                    # in flight -- WAIT (freeze) instead of parking the
                    # brick and re-lifting it next segment
                    serving[rob] = {"kind": "freeze",
                                    "cell": robots[rob], "remain": 2,
                                    "waits": _w,
                                    "task": (robots[rob], pg, color,
                                             "build", pz)}
                    continue
            carrying[rob] = len(tasks)
            info["waits"] = 0
            if goal:
                gcell, gz = goal
                kept = (prev_goal == goal)
                sim[gcell][gz] = color
                sim_inst.add((gcell, gz))
                planned_drop.add(gcell)
                tasks.append((robots[rob], gcell, color, "build", gz))
                why.append("held:goal-kept" if kept else "held:retarget")
                held[rob]["goal"] = goal
            else:
                p = park_spot(robots[rob], {robots[rob]})
                assert p is not None, "no parking slot left"
                sim[p][sim_top(p) + 1] = color
                planned_drop.add(p)
                tasks.append((robots[rob], p, color, "park", -1))
                why.append("held:park-fallback")
                held[rob]["goal"] = ("park", p)

        # ---- sticky commitments from the previous segment ----
        # escape valve: if progress stalls (many replans), periodically
        # drop the stickiness for one segment to break self-made cycles
        if not STICKY or (replans > 400 and replans % 7 == 0):
            prev_tasks = []
        for pt in prev_tasks:
            s_, g_, color, kind, z = pt
            if s_ in planned_lift or g_ in planned_drop or s_ == g_:
                continue
            zt = world.top_z(s_)
            if zt < 0 or world.vox[s_].get(zt) != color or \
               (s_, zt) in world.installed or \
               world.removable(s_) is None:
                continue
            if kind == "build":
                if TMPL[g_].get(z) != color or not world.placeable(g_, z):
                    continue
            else:
                if world.top_z(g_) + 1 >= MAX_H:
                    continue
            zz = sim_top(s_)
            if zz < 0 or sim[s_].get(zz) != color:
                continue
            sim[s_].pop(zz)
            if kind == "build":
                if not sim_placeable(g_, z):
                    sim[s_][zz] = color
                    continue
                sim[g_][z] = color
                sim_inst.add((g_, z))
            else:
                sim[g_][sim_top(g_) + 1] = color
            planned_lift.add(s_)
            planned_drop.add(g_)
            tasks.append((s_, g_, color, kind, z))
            why.append("sticky")

        # ---- roots: template voxels in build order ----
        for gcell in sorted(FOOT, key=lambda c: OIDX[c]):
            if len(tasks) >= len(robots) + 2:
                break
            for gz in sorted(TMPL[gcell]):
                if (gcell, gz) not in sim_inst:
                    try_build(gcell, gz, 6, set(),
                              root=f"root({gcell[0]},{gcell[1]})z{gz}")
                    break

        # ---- hand robots only the tasks executable RIGHT NOW ----
        ready = []
        used_src_cells, used_goal_cells = set(), set()
        for tid, (s_, g_, color, kind, z) in enumerate(tasks):
            if tid in carrying.values():        # reserve carried goals 1st
                used_goal_cells.add(g_)
                used_src_cells.add(s_)
        for tid, (s_, g_, color, kind, z) in enumerate(tasks):
            if tid in carrying.values():
                ready.append(tid)               # held bricks always ready
                continue
            if s_ in used_src_cells or g_ in used_goal_cells or \
               s_ in used_goal_cells or g_ in used_src_cells:
                continue
            zt = world.top_z(s_)
            if zt < 0 or world.vox[s_][zt] != color or \
               world.removable(s_) is None:
                continue
            if kind == "build" and not world.placeable(g_, z):
                continue
            if kind == "park" and world.top_z(g_) + 1 >= MAX_H:
                continue
            ready.append(tid)
            used_src_cells.add(s_)
            used_goal_cells.add(g_)
            if len(ready) >= len(robots):
                break
        keep = sorted(set(ready) | set(carrying.values()))
        seg_log = {"replan": replans, "t": t_global,
                   "generated": len(tasks),
                   "tasks": [{"s": list(tasks[t][0]), "g": list(tasks[t][1]),
                              "color": tasks[t][2], "kind": tasks[t][3],
                              "z": tasks[t][4],
                              "why": why[t] if t < len(why) else "?",
                              "dispatched": t in keep}
                             for t in range(len(tasks))]}
        remap = {old: new for new, old in enumerate(keep)}
        tasks = [tasks[t] for t in keep]
        why = [seg_log["tasks"][t]["why"] for t in keep]
        carrying = {rob: remap[t] for rob, t in carrying.items()}
        n_dig += sum(1 for t in tasks if t[3] == "park")
        # ==============================================================
        if not tasks and serving:
            # nothing plannable while hoists run: advance their clocks
            tasks = []
        elif not tasks:
            print("deadlock: no feasible task")
            return 1

        # ---- solve ----
        active = [i for i in range(len(robots)) if i not in serving]
        # serving cells are walls this segment: drop tasks whose start or
        # goal is disconnected from (or owned by) a hoist wall.  Freezing a
        # sealed-off carrier adds a NEW wall, so iterate to a fixed point.
        from collections import deque
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
                    cur_ = q.popleft()
                    for nb in nbrs(cur_):
                        if nb not in comp and nb not in serving_cells0:
                            comp[nb] = cid
                            q.append(nb)
            keep2, carrying2 = [], {}
            grew = False
            for tid, t in enumerate(tasks0):
                rob_of = next((r for r, x in carrying0.items()
                               if x == tid), None)
                if rob_of is not None and rob_of in serving:
                    continue                     # already frozen
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
        starts = {tasks[t][0] for t in range(len(tasks))
                  if t not in carrying.values()}
        goal_cells = {t[1] for t in tasks}
        serving_cells = {sv["cell"] for sv in serving.values()}
        lines = ["name: table_round", "map: |"]
        for r in range(N):
            lines.append("  " + "".join(
                "@" if (r, c) in serving_cells else "."
                for c in range(N)))
        lines.append("storage_map: |")
        for r in range(N):
            row = ""
            for c in range(N):
                base = world.top_z((r, c)) + 1 - (1 if (r, c) in starts else 0)
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
            lines += [f"  - id: b{i}", f"    start: [{t[0][0]}, {t[0][1]}]",
                      f"    goal: [{t[1][0]}, {t[1][1]}]"]
        lines.append("flags: {gantry: true}")
        act_index = {g: k for k, g in enumerate(active)}
        if carrying:
            lines.append("carrying:")
            lines += [f"  - [{act_index[rob]}, {t}]"
                      for rob, t in sorted(carrying.items())]
        plan_lines = []
        if active and tasks:
            ypath = WORK / f"plan_{replans}.yaml"
            ppath = WORK / f"plan_{replans}.plan"
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
            if metrics.get("solved") != "1" and \
               any(t not in carrying.values()
                   for t in range(len(tasks))):
                # degrade: keep only carried commitments and retry once
                keepc = sorted(carrying.values())
                remapc = {o: n for n, o in enumerate(keepc)}
                tasks = [tasks[t] for t in keepc]
                carrying = {r: remapc[t] for r, t in carrying.items()}
                lines2 = []
                skip = False
                for ln in lines:
                    if ln.startswith("shelves:") or ln.startswith("targets:"):
                        skip = True
                        if ln.startswith("shelves:"):
                            lines2.append("shelves:")
                            lines2 += [f"  - [{t[0][0]}, {t[0][1]}]"
                                       for t in tasks]
                        else:
                            lines2.append("targets:")
                            for i2, t in enumerate(tasks):
                                lines2 += [f"  - id: b{i2}",
                                           f"    start: [{t[0][0]}, {t[0][1]}]",
                                           f"    goal: [{t[1][0]}, {t[1][1]}]"]
                        continue
                    if skip and (ln.startswith("flags:") or
                                 ln.startswith("carrying:")):
                        skip = False
                    if not skip:
                        lines2.append(ln)
                ypath.write_text("\n".join(lines2) + "\n", encoding="utf-8")
                res = subprocess.run(
                    [str(BINARY), str(ypath), "10.0", str(ppath),
                     "0", "lacam"],
                    capture_output=True, text=True, timeout=25)
                metrics = dict(kv.split("=") for kv in res.stdout.split()
                               if "=" in kv)
            if metrics.get("solved") != "1" and serving:
                # segment unsolvable while hoist walls stand (e.g. robots
                # boxed into a dead pocket): freeze all carriers one tick
                # and let the walls fall before retrying
                for rob, tid in list(carrying.items()):
                    serving[rob] = {"kind": "freeze",
                                    "cell": robots[rob],
                                    "remain": 1, "task": tasks[tid]}
                tasks, carrying, plan_lines = [], {}, []
                metrics = {"solved": "1"}
            if metrics.get("solved") != "1":
                print(f"replan {replans}: NOT SOLVED")
                return 1
            plan_lines = ppath.read_text().splitlines() \
                if ppath.exists() and (tasks or not serving) else plan_lines

        # ---- replay; hoist starts/completions and placements are the
        #      truncation events (serving robots tick in parallel) ----
        bound = dict(carrying)
        lifted_from = {}
        loc = defaultdict(list)
        for t in range(len(tasks)):
            if t not in bound.values():
                loc[tasks[t][0]].append(t)
        completed = False
        consumed = set()
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
                break                    # plan exhausted, nothing in flight
            t_global += 1
            if toks is not None:
                for k, i in enumerate(active):
                    parts = toks[k].split()
                    if parts and parts[0] == "m":
                        robots[i] = (int(parts[1]), int(parts[2]))
                    elif parts and parts[0] == "l" and loc[robots[i]]:
                        tid = loc[robots[i]].pop()
                        consumed.add(tid)
                        cell = robots[i]
                        serving[i] = {
                            "kind": "lift", "cell": cell,
                            "remain": hoist_ticks(world.top_z(cell)),
                            "t0": t_global, "z": world.top_z(cell),
                            "task": tasks[tid]}
                        lifted_from[i] = cell
                        completed = True     # hoist START truncates
                    elif parts and parts[0] == "d" and i in bound:
                        tid = bound.pop(i)
                        cell = robots[i]
                        z_stop = tasks[tid][4] \
                            if tasks[tid][3] == "build" and \
                            cell == tasks[tid][1] \
                            else world.top_z(cell) + 1
                        serving[i] = {
                            "kind": "drop", "cell": cell,
                            "remain": hoist_ticks(z_stop),
                            "t0": t_global, "z": z_stop,
                            "task": tasks[tid]}
                        completed = True     # hoist START truncates
            for rob in sorted(serving):
                sv = serving[rob]
                sv["remain"] -= 1
                if sv["remain"] > 0:
                    continue
                s_, g, color, kind, z = sv["task"]
                cell = sv["cell"]
                if sv["kind"] == "freeze":   # sealed-off carrier resumes
                    held[rob] = {"color": color,
                                 "goal": ("park", g) if kind == "park"
                                 else (g, z),
                                 "waits": sv.get("waits", 0)}
                    bound[rob] = -1          # keep it through held rebuild
                    del serving[rob]
                    completed = True
                    continue
                if sv["kind"] == "lift":     # rope back up: brick in hand
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
                    held[rob] = {"color": color,
                                 "goal": ("park", g) if kind == "park"
                                 else (g, z)}
                else:                        # drop done: brick placed
                    if kind == "build" and cell == g and \
                       world.placeable(g, z):
                        world.place(g, z, color, completion=True)
                        put_z = z
                    else:
                        put_z = world.top_z(cell) + 1
                        assert put_z < MAX_H, "park overflow"
                        world.place(cell, put_z, color, completion=False)
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
                completed = True             # hoist COMPLETION truncates
            for i in range(len(robots)):
                paths[i].append(list(robots[i]))
                in_air = i in bound or (
                    i in serving and serving[i]["kind"] == "drop")
                carry[i].append(1 if in_air else 0)
        for k, t in enumerate(sorted(keep)):
            seg_log["tasks"] and None
        started = {t for t in consumed}
        seg_log["started"] = sorted(started)
        seg_log["events_t"] = t_global
        import json as _json
        with open(WORK / "reasons.jsonl", "a") as _f:
            _f.write(_json.dumps(seg_log) + "\n")
        prev_tasks = [tasks[t] for t in range(len(tasks))
                      if t not in carrying.values() and t not in consumed]
        new_held = {}
        for rob, tid in bound.items():
            if tid >= 0:
                s_, g, color, kind, z = tasks[tid]
                new_held[rob] = {"color": color,
                                 "goal": ("park", g) if kind == "park"
                                 else (g, z)}
            elif rob in held:
                new_held[rob] = held[rob]    # lift finished mid-segment
        held = new_held
        if replans % 10 == 0:
            print(f"replan {replans}: installed={len(world.installed)}"
                  f"/{TOTAL} t={t_global}", flush=True)

    ok = done() and world.grounded_ok()
    for cell in CELLS:                       # exact template equality
        ok = ok and world.vox[cell] == TMPL[cell]
    plan = {
        "rows": N, "cols": N, "T": t_global,
        "depot": [list(d) for d in LEGS],
        "agents": [{"name": f"a{i}", "path": paths[i], "carry": carry[i]}
                   for i in range(len(robots))],
        "terrain": terrain, "picks": picks, "hoists": hoists,
        "bricks": bricks_out,
        "meta": {"gantry": RAIL, "statsLine":
                 f"整板搭桌子(悬空桌面·面连接) · {TOTAL} 块 · "
                 f"{replans} 次规划 · 挖掘 {n_dig} 次"},
    }
    OUT.write_text("const PLAN = " + json.dumps(plan) + ";\n",
                   encoding="utf-8")
    print(f"done: replans={replans} T={t_global} wall={time.time()-t0:.2f}s "
          f"valid={'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
