#!/usr/bin/env python3
"""Offline planner for the 3D pyramid-construction demo.

Per-agent state -> goal:
  carrying + reserved placement -> a stand cell next to the placement
  not carrying (reserved or preloading, within quota) -> a depot cell (pick)
  otherwise -> hold (stay put; step off unplaced target cells)

One global LaCAM-TAPF solve per round for all agents; picks happen on depot
arrival without interrupting execution; as soon as some agent completes a
placement, execution is truncated, terrain updated, and everything replans
globally.

Upper layer (construction order): levels strictly one after another; inside a
level, a cell is placeable once all its strictly-deeper 4-neighbors within the
level footprint are placed (dependency-graph frontier, inside-out). This both
maximizes parallelism (center + ring corners are free immediately) and
guarantees a valid stand cell: any shallower neighbor still unplaced sits at
height L-1, and the terrain stays a <=1-step staircase globally.

Rules enforced (validated every round and by validate_plan.py):
  - move only between 4-neighbors with |dh| <= 1 (or wait)
  - pick at a depot cell (infinite supply)
  - place onto a 4-neighbor cell whose current height equals the agent's
    floor height, and only if no agent stands on it
Exports demo/pyramid/plan.js (JS global) + plan.json for the three.js page.
"""
from __future__ import annotations

import json
import subprocess
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
from tapf_schedule_io import load_schedule  # noqa: E402

Coord = Tuple[int, int]

# ---------------------------------------------------------------- config
ROWS, COLS = 14, 22
PYR_R0, PYR_C0, PYR_BASE, PYR_LEVELS = 3, 4, 7, 4
DEPOT = [(r, 20) for r in range(3, 11)]
AGENT_STARTS = [(3, 18), (4, 18), (5, 18), (6, 18), (7, 18), (8, 18)]
BINARY = REPO / "build" / "tapf_benchmark"
WORK = Path(__file__).resolve().parent / "work"
OUT_JSON = Path(__file__).resolve().parent / "plan.json"
OUT_JS = Path(__file__).resolve().parent / "plan.js"
TIME_LIMIT = 2.0
MAX_ROUNDS = 3000
MAX_STEPS = 30000
INF = 10 ** 9
CLIMB_COST = 2  # heuristic cost of a +-1 height move (solver + goal choice)
ACTION_STEPS = 1  # extra timesteps a pick/place action occupies

NBRS = ((-1, 0), (1, 0), (0, -1), (0, 1))


# ---------------------------------------------------------------- level geometry
def level_footprint(lvl: int) -> Dict[Coord, int]:
    """cells of level lvl -> border distance (depth) within the footprint."""
    inset = lvl - 1
    r0, c0 = PYR_R0 + inset, PYR_C0 + inset
    side = PYR_BASE - 2 * inset
    out: Dict[Coord, int] = {}
    if side <= 0:
        return out
    for r in range(r0, r0 + side):
        for c in range(c0, c0 + side):
            out[(r, c)] = min(r - r0, c - c0, r0 + side - 1 - r, c0 + side - 1 - c)
    return out


@dataclass
class Placement:
    cell: Coord
    level: int  # resulting height after placement


@dataclass
class Agent:
    name: str
    pos: Coord
    carrying: bool = False
    placement: Optional[Placement] = None
    goal: Optional[Coord] = None
    kind: str = "hold"  # depot | stand | hold | action
    action: Optional[str] = None  # pending "pick"/"place", occupies a timestep
    path_hist: List[Coord] = field(default_factory=list)
    carry_hist: List[int] = field(default_factory=list)


# ---------------------------------------------------------------- helpers
def bfs_dist(src: Coord, heights: List[List[int]]) -> List[List[int]]:
    """Dijkstra with |dh|=1 edges costing CLIMB_COST (>=2 cliffs impassable)."""
    import heapq
    dist = [[INF] * COLS for _ in range(ROWS)]
    dist[src[0]][src[1]] = 0
    pq = [(0, src)]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if d > dist[r][c]:
            continue
        for dr, dc in NBRS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < ROWS and 0 <= nc < COLS):
                continue
            dh = abs(heights[nr][nc] - heights[r][c])
            if dh > 1:
                continue
            nd = d + (CLIMB_COST if dh == 1 else 1)
            if nd < dist[nr][nc]:
                dist[nr][nc] = nd
                heapq.heappush(pq, (nd, (nr, nc)))
    return dist


def write_map(path: Path) -> None:
    lines = ["type octile", f"height {ROWS}", f"width {COLS}", "map"]
    lines += ["." * COLS for _ in range(ROWS)]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def expand_schedule(entries: Sequence[Dict[str, int]], fallback: Coord, horizon: int) -> List[Coord]:
    path = [fallback] * (horizon + 1)
    cur = fallback
    nodes = sorted(((e["t"], (e["x"], e["y"])) for e in entries), key=lambda x: x[0])
    idx = 0
    for t in range(horizon + 1):
        while idx < len(nodes) and nodes[idx][0] == t:
            cur = nodes[idx][1]
            idx += 1
        path[t] = cur
    return path


# ---------------------------------------------------------------- main loop
def main() -> int:
    WORK.mkdir(parents=True, exist_ok=True)
    heights = [[0] * COLS for _ in range(ROWS)]
    footprints = {lvl: level_footprint(lvl) for lvl in range(1, PYR_LEVELS + 1)}
    footprints = {lvl: fp for lvl, fp in footprints.items() if fp}
    total_blocks = sum(len(fp) for fp in footprints.values())

    cur_level = 1
    pending: List[Placement] = [Placement(cell=c, level=1) for c in footprints[1]]
    reserved: List[Placement] = []

    agents = [Agent(name=f"a{i}", pos=p, path_hist=[p], carry_hist=[0]) for i, p in enumerate(AGENT_STARTS)]
    terrain_events: List[Dict[str, int]] = []
    pick_events: List[Dict[str, int]] = []
    t_global = 0
    placed_count = 0
    round_no = 0
    stall_rounds = 0

    def advance_level() -> None:
        nonlocal cur_level, pending
        while not pending and not reserved and cur_level < max(footprints):
            cur_level += 1
            pending = [Placement(cell=c, level=cur_level) for c in footprints[cur_level]]

    def ready(p: Placement) -> bool:
        """support done + all strictly deeper in-level neighbors placed."""
        r, c = p.cell
        if heights[r][c] != p.level - 1:
            return False
        fp = footprints[p.level]
        d = fp[p.cell]
        for dr, dc in NBRS:
            n = (r + dr, c + dc)
            if n in fp and fp[n] > d and heights[n[0]][n[1]] < p.level:
                return False
        return True

    def unplaced_targets() -> set:
        return {p.cell for p in pending} | {p.cell for p in reserved}

    def stand_cells(pl: Placement, taken: set) -> List[Coord]:
        want_h = pl.level - 1
        tset = unplaced_targets()
        out = []
        for dr, dc in NBRS:
            n = (pl.cell[0] + dr, pl.cell[1] + dc)
            if not (0 <= n[0] < ROWS and 0 <= n[1] < COLS):
                continue
            if n in taken or heights[n[0]][n[1]] != want_h:
                continue
            out.append(n)
        # prefer stand cells that are not themselves unplaced targets
        out.sort(key=lambda n: n in tset)
        return out

    def complete_place(a: Agent, positions: List[Coord]) -> bool:
        """Finish a pending place action; False if the target is blocked."""
        assert a.placement is not None and a.carrying
        tc = a.placement.cell
        if tc in positions:
            return False
        want_h = a.placement.level - 1
        assert heights[a.pos[0]][a.pos[1]] == want_h == heights[tc[0]][tc[1]], "place precondition violated"
        heights[tc[0]][tc[1]] += 1
        terrain_events.append({"t": t_global, "r": tc[0], "c": tc[1], "h": heights[tc[0]][tc[1]],
                               "agent": int(a.name[1:])})
        reserved.remove(a.placement)
        a.placement = None
        a.carrying = False
        a.carry_hist[-1] = 0
        return True

    while placed_count < total_blocks and round_no < MAX_ROUNDS and t_global < MAX_STEPS:
        round_no += 1
        advance_level()

        # --- stall breaker: crossed stand/target waits -> drop reservations
        if stall_rounds >= 3:
            for a in agents:
                if a.placement is not None and a.carrying and a.action is None:
                    reserved.remove(a.placement)
                    pending.append(a.placement)
                    a.placement = None
            stall_rounds = 0

        # --- reserve ready placements (carrying agents first, nearest first)
        frontier = [p for p in pending if ready(p)]
        for a in sorted(agents, key=lambda x: not x.carrying):
            if a.placement is not None or not frontier:
                continue
            dist = bfs_dist(a.pos, heights)
            best = min(frontier, key=lambda p: min(
                (dist[p.cell[0] + dr][p.cell[1] + dc]
                 for dr, dc in NBRS
                 if 0 <= p.cell[0] + dr < ROWS and 0 <= p.cell[1] + dc < COLS), default=INF))
            frontier.remove(best)
            pending.remove(best)
            reserved.append(best)
            a.placement = best

        # --- rebalance: a carrying agent without a task takes over a
        # reservation held by a non-carrying agent (block is in the wrong hands)
        for a in agents:
            if not a.carrying or a.placement is not None:
                continue
            donor = next((b for b in agents
                          if not b.carrying and b.placement is not None and b.action is None), None)
            if donor is not None:
                a.placement = donor.placement
                donor.placement = None

        # --- state-based goals, distinct across agents
        carrying_now = sum(1 for a in agents if a.carrying or a.action == "pick")
        pick_quota = (total_blocks - placed_count) - carrying_now
        taken: set = set()
        # pass 1: agents mid-action claim their own cell first
        for a in agents:
            if a.action is not None:
                a.goal = a.pos
                a.kind = "action"
                taken.add(a.goal)
        for a in agents:
            if a.action is not None:
                continue
            a.goal = None
            dist = bfs_dist(a.pos, heights)
            if a.carrying and a.placement is not None:
                cands = [c for c in stand_cells(a.placement, taken) if dist[c[0]][c[1]] < INF]
                if cands:
                    a.goal = min(cands, key=lambda c: (c in unplaced_targets(), dist[c[0]][c[1]]))
                    a.kind = "stand"
            elif not a.carrying and pick_quota > 0:
                cands = [d for d in DEPOT if d not in taken and dist[d[0]][d[1]] < INF]
                if cands:
                    a.goal = min(cands, key=lambda c: dist[c[0]][c[1]])
                    a.kind = "depot"
                    pick_quota -= 1
            if a.goal is None:  # hold; step off unplaced target cells
                bad = unplaced_targets()
                if a.pos not in bad and a.pos not in taken:
                    a.goal = a.pos
                else:
                    free = [(dist[r][c], (r, c)) for r in range(ROWS) for c in range(COLS)
                            if dist[r][c] < INF and (r, c) not in taken and (r, c) not in bad]
                    if not free:
                        raise RuntimeError(f"no hold cell for {a.name} at round {round_no}")
                    a.goal = min(free)[1]
                a.kind = "hold"
            taken.add(a.goal)

        # --- zero-time: agents already at goal begin their pick/place action
        zero = False
        for a in agents:
            if a.pos != a.goal or a.action is not None:
                continue
            if a.kind == "depot" and not a.carrying:
                a.action = "pick"
                zero = True
            elif a.kind == "stand" and a.carrying and a.placement is not None:
                a.action = "place"
                zero = True
        if zero:
            stall_rounds = 0
            continue  # replan: action agents will hold one step

        # --- one global TAPF solve on the plain grid (terrain is a staircase)
        rd = WORK / f"round_{round_no:04d}"
        rd.mkdir(parents=True, exist_ok=True)
        map_path = rd / "round.map"
        write_map(map_path)
        agents_yaml = [{"name": a.name, "start": [a.pos[0], a.pos[1]],
                        "potentialGoals": [[a.goal[0], a.goal[1]]]} for a in agents]
        in_yaml = rd / "in.yaml"
        in_yaml.write_text(yaml.safe_dump({
            "map": str(map_path.resolve()),
            "heights": [list(row) for row in heights],
            "climbCost": CLIMB_COST,
            "agents": agents_yaml,
        }, sort_keys=False), encoding="utf-8")
        out_yaml = rd / "out.yaml"
        cmd = [str(BINARY), str(in_yaml), "", str(TIME_LIMIT), str(out_yaml), "0", "0", "0"]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=TIME_LIMIT + 10)
        stdout = proc.stdout or ""
        if proc.returncode != 0 or "solved=1" not in stdout:
            raise RuntimeError(f"solver failed round {round_no}:\n{stdout}\n{proc.stderr}")

        data = load_schedule(out_yaml)
        makespan = max(int(data["statistics"]["makespan"]), 1)
        sched = data["schedule"]
        paths = [expand_schedule(sched.get(f"agent{i}", []), agents[i].pos, makespan)
                 for i in range(len(agents))]

        # --- validate solver paths against demo rules
        for i, p in enumerate(paths):
            assert p[0] == agents[i].pos, f"round {round_no}: bad start {i}"
            for t in range(1, len(p)):
                (r0, c0), (r1, c1) = p[t - 1], p[t]
                assert abs(r1 - r0) + abs(c1 - c0) <= 1, f"round {round_no}: non-unit move"
                assert abs(heights[r1][c1] - heights[r0][c0]) <= 1, f"round {round_no}: height jump"
        for t in range(makespan + 1):
            seen = set()
            for p in paths:
                pos = p[min(t, len(p) - 1)]
                assert pos not in seen, f"round {round_no}: vertex collision t={t}"
                seen.add(pos)
        for t in range(makespan):
            for i in range(len(paths)):
                for j in range(i + 1, len(paths)):
                    if paths[i][t] == paths[j][t + 1] and paths[i][t + 1] == paths[j][t]:
                        raise AssertionError(f"round {round_no}: edge collision t={t}")

        # --- execute; actions complete after one held step; new arrivals
        # start actions; any completed pick/place truncates -> global replan
        moved = False
        event_this_round = False
        started_actions = {a.name for a in agents if a.action is not None}
        for t in range(1, makespan + 1):
            for i, a in enumerate(agents):
                if a.pos != paths[i][t]:
                    moved = True
                a.pos = paths[i][t]
                a.path_hist.append(a.pos)
                a.carry_hist.append(1 if a.carrying else 0)
            t_global += 1
            positions = [a.pos for a in agents]
            # complete actions that were pending when the round started
            for a in agents:
                if a.action is None or a.name not in started_actions:
                    continue
                stayed = a.path_hist[-1] == a.path_hist[-2]
                if not stayed:  # displaced mid-action: cancel, retry later
                    a.action = None
                    continue
                if a.action == "pick":
                    a.carrying = True
                    a.carry_hist[-1] = 1
                    pick_events.append({"t": t_global, "agent": int(a.name[1:]),
                                        "r": a.pos[0], "c": a.pos[1]})
                    event_this_round = True
                elif a.action == "place":
                    if complete_place(a, positions):
                        placed_count += 1
                        event_this_round = True
                a.action = None
            # arrivals begin new actions (they take effect next round)
            for a in agents:
                if a.action is not None or a.pos != a.goal:
                    continue
                if a.kind == "depot" and not a.carrying:
                    a.action = "pick"
                    event_this_round = True
                elif a.kind == "stand" and a.carrying and a.placement is not None:
                    a.action = "place"
                    event_this_round = True
            if event_this_round:
                break  # global replan

        stall_rounds = 0 if (moved or event_this_round) else stall_rounds + 1
        print(f"round {round_no:4d}  t={t_global:5d}  placed {placed_count}/{total_blocks}", flush=True)

    if placed_count < total_blocks:
        raise RuntimeError(f"incomplete: {placed_count}/{total_blocks} after {round_no} rounds")

    plan = {
        "rows": ROWS,
        "cols": COLS,
        "T": t_global,
        "depot": [list(d) for d in DEPOT],
        "pyramid": {"r0": PYR_R0, "c0": PYR_C0, "base": PYR_BASE, "levels": PYR_LEVELS},
        "agents": [{"name": a.name, "path": [list(p) for p in a.path_hist],
                    "carry": a.carry_hist} for a in agents],
        "terrain": terrain_events,
        "picks": pick_events,
    }
    OUT_JSON.write_text(json.dumps(plan), encoding="utf-8")
    OUT_JS.write_text("const PLAN = " + json.dumps(plan) + ";\n", encoding="utf-8")
    print(f"done: {placed_count} blocks, {round_no} rounds, T={t_global}")
    print(f"wrote {OUT_JSON} and {OUT_JS}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
